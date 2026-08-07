/*
// Copyright (c) 2017 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
*/

#include "HwmonTempSensor.hpp"

#include <unistd.h>
#include <fcntl.h>
#include <cerrno>

#include <boost/asio/read_until.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <charconv>
#include <iostream>
#include <istream>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

// Temperatures are read in milli degrees Celsius, we need degrees Celsius.
// Pressures are read in kilopascal, we need Pascals.  On D-Bus for Open BMC
// we use the International System of Units without prefixes.
// Links to the kernel documentation:
// https://www.kernel.org/doc/Documentation/hwmon/sysfs-interface
// https://www.kernel.org/doc/Documentation/ABI/testing/sysfs-bus-iio
// For IIO RAW sensors we get a raw_value, an offset, and scale to compute
// the value = (raw_value + offset) * scale

HwmonTempSensor::HwmonTempSensor(
    const std::string& path, const std::string& objectType,
    sdbusplus::asio::object_server& objectServer,
    std::shared_ptr<sdbusplus::asio::connection>& conn,
    boost::asio::io_context& io, const std::string& sensorName,
    std::vector<thresholds::Threshold>&& thresholdsIn,
    const struct SensorParams& thisSensorParameters, const float pollRate,
    const std::string& sensorConfiguration, const PowerState powerState,
    const std::shared_ptr<I2CDevice>& i2cDevice) :
    Sensor(boost::replace_all_copy(sensorName, " ", "_"),
           std::move(thresholdsIn), sensorConfiguration, objectType, false,
           false, thisSensorParameters.maxValue, thisSensorParameters.minValue,
           conn, powerState),
    i2cDevice(i2cDevice), objServer(objectServer),
#ifndef __ZEPHYR__
    inputDev(io),
#endif
    waitTimer(io), path(path), offsetValue(thisSensorParameters.offsetValue),
    scaleValue(thisSensorParameters.scaleValue),
    sensorPollMs(static_cast<unsigned int>(pollRate * 1000))
{
#ifdef __ZEPHYR__
    /* On Zephyr we open the hwmon sysfs node as a bare, non-blocking fd and
     * drive reads from the waitTimer. We never register it with boost::asio's
     * select_reactor (that leaks a descriptor_state per poll cycle and exhausts
     * the malloc arena). O_NONBLOCK guarantees read() returns immediately even
     * if the underlying I2C transaction is not yet ready, so the sensor read
     * can never block the single io_context thread and starve the broker /
     * other sensors. */
    fd = open(path.c_str(), O_RDONLY | O_NONBLOCK);
#else
    int fd = open(path.c_str(), O_RDONLY);
    if (fd >= 0)
    {
        inputDev.assign(fd);
    }
#endif
    sensorInterface = objectServer.add_interface(
        "/xyz/openbmc_project/sensors/" + thisSensorParameters.typeName + "/" +
            name,
        "xyz.openbmc_project.Sensor.Value");

    for (const auto& threshold : thresholds)
    {
        std::string interface = thresholds::getInterface(threshold.level);
        thresholdInterfaces[static_cast<size_t>(threshold.level)] =
            objectServer.add_interface("/xyz/openbmc_project/sensors/" +
                                           thisSensorParameters.typeName + "/" +
                                           name,
                                       interface);
    }
    association = objectServer.add_interface("/xyz/openbmc_project/sensors/" +
                                                 thisSensorParameters.typeName +
                                                 "/" + name,
                                             association::interface);
    setInitialProperties(thisSensorParameters.units);
}

bool HwmonTempSensor::isActive()
{
#ifdef __ZEPHYR__
    return fd >= 0;
#else
    return inputDev.is_open();
#endif
}

void HwmonTempSensor::activate(const std::string& newPath,
                               const std::shared_ptr<I2CDevice>& newI2CDevice)
{
    path = newPath;
    i2cDevice = newI2CDevice;
#ifdef __ZEPHYR__
    /* setupRead() reopens the fd every poll; only (re)open here so isActive()
     * reflects a live fd. Non-blocking so a slow I2C transaction can never
     * block this thread. */
    if (fd >= 0)
    {
        close(fd);
    }
    fd = open(path.c_str(), O_RDONLY | O_NONBLOCK);
#else
    int fd = open(path.c_str(), O_RDONLY);
    if (fd >= 0)
    {
        inputDev.assign(fd);
    }
#endif
    markAvailable(true);
    setupRead();
}

void HwmonTempSensor::deactivate()
{
    markAvailable(false);
    // close the input dev to cancel async operations
#ifdef __ZEPHYR__
    if (fd >= 0)
    {
        close(fd);
        fd = -1;
    }
#else
    inputDev.close();
#endif
    waitTimer.cancel();
    i2cDevice = nullptr;
    path = "";
}

HwmonTempSensor::~HwmonTempSensor()
{
    deactivate();

    for (const auto& iface : thresholdInterfaces)
    {
        objServer.remove_interface(iface);
    }
    objServer.remove_interface(sensorInterface);
    objServer.remove_interface(association);
}

void HwmonTempSensor::setupRead(void)
{
    if (!readingStateGood())
    {
        markAvailable(false);
        updateValue(std::numeric_limits<double>::quiet_NaN());
        restartRead();
        return;
    }

#ifdef __ZEPHYR__
    /* On Zephyr we drive reads with a timer + a plain read(fd). We never
     * register the fd with boost::asio's select_reactor (stream_descriptor)
     * because that leaks a descriptor_state on every poll cycle and exhausts
     * the 5 MB newlib malloc arena.
     * hwmon_i2c 的 sysfs 属性是 "一次 open 只产出一次数据" 语义 (见
     * chan_attr_open/chan_attr_read: open 时 ch->ppos=0, 首读后 ch->ppos>0,
     * 后续 read 直接返回 0/EOF; 且 lseek 改的是 VFS fpos, 不影响 ch->ppos)。
     * 因此必须每次轮询 close+open, 不能复用 fd + lseek。
     * 读取使用 O_NONBLOCK：若 I2C 事务尚未就绪, read() 立即返回 -1/EAGAIN,
     * 此时不阻塞本线程, 而是直接 restartRead 下一轮再试, 从而确保 io_context
     * 线程永远不会被某个慢/无响应的传感器读卡住, 避免饿死 broker 与其他
     * 传感器 (之前的累积性 stuck 正是源于此)。 */
    if (fd >= 0)
    {
        close(fd);
        fd = -1;
    }
    fd = open(path.c_str(), O_RDONLY | O_NONBLOCK);
    if (fd < 0)
    {
        std::cerr << name << " unable to open fd!\n";
        restartRead();
        return;
    }

    ssize_t bytesRead = read(fd, readBuf.data(), readBuf.size());
    if (bytesRead < 0)
    {
        if (errno == EAGAIN || errno == EWOULDBLOCK)
        {
            /* Transaction not ready yet; do not block, just retry next poll. */
            restartRead();
            return;
        }
        bytesRead = 0;
    }
    handleResponse(boost::system::error_code(), static_cast<size_t>(bytesRead));
#else
    std::weak_ptr<HwmonTempSensor> weakRef = weak_from_this();
    inputDev.async_read_some(
        boost::asio::buffer(readBuf),
        [weakRef](const boost::system::error_code& ec, std::size_t bytesRead) {
        std::shared_ptr<HwmonTempSensor> self = weakRef.lock();
        if (self)
        {
            self->handleResponse(ec, bytesRead);
        }
        });
#endif
}

void HwmonTempSensor::restartRead()
{
    std::weak_ptr<HwmonTempSensor> weakRef = weak_from_this();
    waitTimer.expires_after(std::chrono::milliseconds(sensorPollMs));
    waitTimer.async_wait([weakRef](const boost::system::error_code& ec) {
        if (ec == boost::asio::error::operation_aborted)
        {
            return; // we're being canceled
        }
        std::shared_ptr<HwmonTempSensor> self = weakRef.lock();
        if (!self)
        {
            return;
        }
        self->setupRead();
    });
}

void HwmonTempSensor::handleResponse(const boost::system::error_code& err,
                                     size_t bytesRead)
{
    if ((err == boost::system::errc::bad_file_descriptor) ||
        (err == boost::asio::error::misc_errors::not_found))
    {
        std::cerr << "Hwmon temp sensor " << name << " removed " << path
                  << "\n";
        return; // we're being destroyed
    }

    if (!err)
    {
        const char* bufEnd = readBuf.data() + bytesRead;
        int nvalue = 0;
        std::from_chars_result ret =
            std::from_chars(readBuf.data(), bufEnd, nvalue);
        if (ret.ec != std::errc())
        {
            incrementError();
        }
        else
        {
            updateValue((nvalue + offsetValue) * scaleValue);
        }
    }
    else
    {
        incrementError();
    }

#ifdef __ZEPHYR__
    /* Zephyr 路径下 setupRead() 每次轮询都会 close+open 重新打开 fd, 因此
     * 此处无需 lseek 重置偏移 (且 lseek 只改 VFS fpos, 不影响驱动 ch->ppos)。 */
#endif

    restartRead();
}

void HwmonTempSensor::checkThresholds(void)
{
    thresholds::checkThresholds(this);
}
