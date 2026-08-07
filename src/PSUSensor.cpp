/*
// Copyright (c) 2019 Intel Corporation
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

#include "PSUSensor.hpp"

#include <unistd.h>
#ifdef __ZEPHYR__
#include <fcntl.h>
#endif

#ifndef __ZEPHYR__
#include <boost/asio/random_access_file.hpp>
#endif
#include <boost/asio/read_until.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <iostream>
#include <istream>
#include <limits>
#include <memory>
#include <string>
#include <system_error>
#include <vector>
#include <cerrno>

static constexpr const char* sensorPathPrefix = "/xyz/openbmc_project/sensors/";

static constexpr bool debug = false;

PSUSensor::PSUSensor(const std::string& path, const std::string& objectType,
                     sdbusplus::asio::object_server& objectServer,
                     std::shared_ptr<sdbusplus::asio::connection>& conn,
                     boost::asio::io_context& io, const std::string& sensorName,
                     std::vector<thresholds::Threshold>&& thresholdsIn,
                     const std::string& sensorConfiguration,
                     const PowerState& powerState,
                     const std::string& sensorUnits, unsigned int factor,
                     double max, double min, double offset,
                     const std::string& label, size_t tSize, double pollRate) :
    Sensor(escapeName(sensorName), std::move(thresholdsIn), sensorConfiguration,
           objectType, false, false, max, min, conn, powerState),
    objServer(objectServer),
#ifndef __ZEPHYR__
    inputDev(io, path, boost::asio::random_access_file::read_only),
#endif
    waitTimer(io), path(path), sensorFactor(factor), sensorOffset(offset),
    thresholdTimer(io)
{
#ifdef __ZEPHYR__
    /* fd 留作 -1，由 setupRead() 在每次轮询时 open（hwmon_i2c 的 sysfs 属性
     * 一次 open 只产出一次数据，故不能预开复用）。 */
    fd = -1;
#endif
    buffer = std::make_shared<std::array<char, 128>>();
    std::string unitPath = sensor_paths::getPathForUnits(sensorUnits);
    if constexpr (debug)
    {
        std::cerr << "Constructed sensor: path " << path << " type "
                  << objectType << " config " << sensorConfiguration
                  << " typename " << unitPath << " factor " << factor << " min "
                  << min << " max " << max << " offset " << offset << " name \""
                  << sensorName << "\"\n";
    }
    if (pollRate > 0.0)
    {
        sensorPollMs = static_cast<unsigned int>(pollRate * 1000);
    }

    std::string dbusPath = sensorPathPrefix + unitPath + "/" + name;

    sensorInterface = objectServer.add_interface(
        dbusPath, "xyz.openbmc_project.Sensor.Value");

    for (const auto& threshold : thresholds)
    {
        std::string interface = thresholds::getInterface(threshold.level);
        thresholdInterfaces[static_cast<size_t>(threshold.level)] =
            objectServer.add_interface(dbusPath, interface);
    }

    // This should be called before initializing association.
    // createInventoryAssoc() does add more associations before doing
    // register and initialize "Associations" property.
    if (label.empty() || tSize == thresholds.size())
    {
        setInitialProperties(sensorUnits);
    }
    else
    {
        setInitialProperties(sensorUnits, label, tSize);
    }

    association = objectServer.add_interface(dbusPath, association::interface);

    createInventoryAssoc(conn, association, configurationPath);
}

PSUSensor::~PSUSensor()
{
    waitTimer.cancel();
#ifdef __ZEPHYR__
    if (fd >= 0)
    {
        close(fd);
        fd = -1;
    }
#else
    inputDev.close();
#endif
    objServer.remove_interface(sensorInterface);
    for (const auto& iface : thresholdInterfaces)
    {
        objServer.remove_interface(iface);
    }
    objServer.remove_interface(association);
}

void PSUSensor::setupRead(void)
{
    if (!readingStateGood())
    {
        markAvailable(false);
        updateValue(std::numeric_limits<double>::quiet_NaN());
        restartRead();
        return;
    }

    if (buffer == nullptr)
    {
        std::cerr << "Buffer was invalid?";
        return;
    }

#ifdef __ZEPHYR__
    /* hwmon_i2c 的 sysfs 属性是 "一次 open 只产出一次数据" 语义：open() 时
     * 驱动将内部 ppos 清零，第一次 read() 产出数据后 ppos>0，后续 read()
     * （即便 lseek 也无法重置 ppos）会直接返回 0(EOF)。因此必须每次轮询
     * 重新 open，不能复用持久 fd / lseek，否则除首读外全部读到空 -> parse
     * 失败。
     * 读取使用 O_NONBLOCK：若 I2C 事务尚未就绪, read() 立即返回 -1/EAGAIN,
     * 此时不阻塞本线程, 而是直接 restartRead 下一轮再试, 确保 io_context
     * 线程永不被某个慢/无响应的传感器读卡住 (避免饿死 broker 与其他传感器,
     * 这是之前累积性 stuck 的根因之一)。 */
    if (fd >= 0)
    {
        close(fd);
        fd = -1;
    }
    fd = open(path.c_str(), O_RDONLY | O_NONBLOCK);
    if (fd < 0)
    {
        std::cerr << "PSU sensor " << name << " unable to open " << path
                  << "\n";
        restartRead();
        return;
    }

    ssize_t bytesRead = read(fd, buffer->data(), buffer->size() - 1);
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
    std::weak_ptr<PSUSensor> weak = weak_from_this();
    // Note, we are building a asio buffer that is one char smaller than
    // the actual data structure, so that we can always append the null
    // terminator.  This can go away once std::from_chars<double> is available
    // in the standard
    inputDev.async_read_some_at(
        0, boost::asio::buffer(buffer->data(), buffer->size() - 1),
        [weak, buffer{buffer}](const boost::system::error_code& ec,
                               size_t bytesRead) {
        std::shared_ptr<PSUSensor> self = weak.lock();
        if (!self)
        {
            return;
        }

        self->handleResponse(ec, bytesRead);
        });
#endif
}

void PSUSensor::restartRead(void)
{
    std::weak_ptr<PSUSensor> weakRef = weak_from_this();
    waitTimer.expires_after(std::chrono::milliseconds(sensorPollMs));
    waitTimer.async_wait([weakRef](const boost::system::error_code& ec) {
        if (ec == boost::asio::error::operation_aborted)
        {
            std::cerr << "Failed to reschedule\n";
            return;
        }
        std::shared_ptr<PSUSensor> self = weakRef.lock();
        if (self)
        {
            self->setupRead();
        }
    });
}

// Create a buffer expected to be able to hold more characters than will be
// present in the input file.
void PSUSensor::handleResponse(const boost::system::error_code& err,
                               size_t bytesRead)
{
    if (err == boost::asio::error::operation_aborted)
    {
        std::cerr << "Read aborted\n";
        return;
    }
    if ((err == boost::system::errc::bad_file_descriptor) ||
        (err == boost::asio::error::misc_errors::not_found))
    {
        std::cerr << "Bad file descriptor for " << path << "\n";
        return;
    }
    // null terminate the string so we don't walk off the end
    std::array<char, 128>& bufferRef = *buffer;
    bufferRef[bytesRead] = '\0';

    try
    {
        rawValue = std::stod(bufferRef.data());
        updateValue((rawValue / sensorFactor) + sensorOffset);
    }
    catch (const std::invalid_argument&)
    {
        std::cerr << "Could not parse  input from " << path << "\n";
        incrementError();
    }

    /* setupRead() closes+reopens the fd each poll (hwmon_i2c "one read per
     * open" semantics), so no persistent fd state to rewind here. */
    restartRead();
}

void PSUSensor::checkThresholds(void)
{
    if (!readingStateGood())
    {
        return;
    }

    thresholds::checkThresholdsPowerDelay(weak_from_this(), thresholdTimer);
}
