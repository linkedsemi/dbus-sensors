/*
// Copyright (c) 2018 Intel Corporation
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

#include "ChassisIntrusionSensor.hpp"

#include <fcntl.h>
#include <sys/ioctl.h>
#include <systemd/sd-journal.h>
#include <unistd.h>

#include <boost/asio/io_context.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <cerrno>
#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <utility>

extern "C"
{
#include <i2c/smbus.h>
#include <linux/i2c-dev.h>
}

static constexpr bool debug = false;

static constexpr unsigned int intrusionSensorPollSec = 1;

// SMLink Status Register
const static constexpr size_t pchStatusRegIntrusion = 0x04;

// Status bit field masks
const static constexpr size_t pchRegMaskIntrusion = 0x01;

void ChassisIntrusionSensor::updateValue(const std::string& newValue,
                                         uint8_t chMask)
{
    // Take no action if value already equal
    // Same semantics as Sensor::updateValue(const double&)
    if ((newValue == mValue) && (mValue == "Normal"))
    {
        return;
    }

    // indicate that it is internal set call
    mInternalSet = true;
    mIface->set_property("Status", newValue);
    mInternalSet = false;

    mValue = newValue;

    // std::cout << "mValue: " << mValue << std::endl;
    // std::cout << "mOldValue: " << mOldValue << std::endl;
    // std::cout << "mLastChMask: 0x" << std::hex << static_cast<unsigned int>(mLastChMask) << std::endl;
    // std::cout << "chMask: 0x" << std::hex << static_cast<unsigned int>(chMask) << std::endl;
    // std::cout << "chMask^mLastChMask: 0x" << std::hex << static_cast<unsigned int>(chMask ^ mLastChMask) << std::endl;

    if (mOldValue == "Normal" && mValue != "Normal")
    {
        char msg[96];
        snprintf(msg, sizeof(msg),
                 "Chassis intrusion assert event (channel mask: 0x%02x)",
                 static_cast<unsigned int>(chMask));
        sd_journal_send("MESSAGE=%s", msg, "PRIORITY=%i", LOG_INFO,
                        "REDFISH_MESSAGE_ID=%s",
                        "OpenBMC.0.1.ChassisIntrusionDetected", NULL);
        mOldValue = mValue;
        mLastChMask = chMask;
    }
    else if (mOldValue != "Normal" && mValue == "Normal")
    {
        sd_journal_send("MESSAGE=%s", "Chassis intrusion de-assert event",
                        "PRIORITY=%i", LOG_INFO, "REDFISH_MESSAGE_ID=%s",
                        "OpenBMC.0.1.ChassisIntrusionReset", NULL);
        mOldValue = mValue;
        mLastChMask = chMask;
    }
    else if (mOldValue != "Normal" && mValue != "Normal" &&
             (chMask ^ mLastChMask) != 0)
    {
        char msg[96];
        snprintf(msg, sizeof(msg),
                 "Chassis intrusion additional channel assert (channel mask: "
                 "0x%02x, new: 0x%02x)",
                 static_cast<unsigned int>(chMask),
                 static_cast<unsigned int>(chMask & ~mLastChMask));
        sd_journal_send("MESSAGE=%s", msg, "PRIORITY=%i", LOG_INFO,
                        "REDFISH_MESSAGE_ID=%s",
                        "OpenBMC.0.1.ChassisIntrusionDetected", NULL);
        mLastChMask |= chMask;
        std::cout << "Chassis intrusion additional channel assert (channel mask: "
                  << "0x" << std::hex << static_cast<unsigned int>(chMask)
                  << ")\n";
    }
}

int ChassisIntrusionSensor::i2cReadFromPch(int busId, int slaveAddr)
{
    std::string i2cBus = "/dev/i2c-" + std::to_string(busId);

    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-vararg)
    int fd = open(i2cBus.c_str(), O_RDWR | O_CLOEXEC);
    if (fd < 0)
    {
        std::cerr << "unable to open i2c device \n";
        return -1;
    }

    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-vararg)
    if (ioctl(fd, I2C_SLAVE_FORCE, slaveAddr) < 0)
    {
        std::cerr << "unable to set device address\n";
        close(fd);
        return -1;
    }

    unsigned long funcs = 0;

    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-vararg)
    if (ioctl(fd, I2C_FUNCS, &funcs) < 0)
    {
        std::cerr << "not support I2C_FUNCS \n";
        close(fd);
        return -1;
    }

    if ((funcs & I2C_FUNC_SMBUS_READ_BYTE_DATA) == 0U)
    {
        std::cerr << "not support I2C_FUNC_SMBUS_READ_BYTE_DATA \n";
        close(fd);
        return -1;
    }

    int32_t statusMask = pchRegMaskIntrusion;
    int32_t statusReg = pchStatusRegIntrusion;

    int32_t statusValue = i2c_smbus_read_byte_data(fd, statusReg);
    if (debug)
    {
        std::cout << "\nRead bus " << busId << " addr " << slaveAddr
                  << ", value = " << statusValue << "\n";
    }

    close(fd);

    if (statusValue < 0)
    {
        std::cerr << "i2c_smbus_read_byte_data failed \n";
        return -1;
    }

    // Get status value with mask
    int newValue = statusValue & statusMask;

    if (debug)
    {
        std::cout << "statusValue is " << statusValue << "\n";
        std::cout << "Intrusion sensor value is " << newValue << "\n";
    }

    return newValue;
}

void ChassisIntrusionSensor::pollSensorStatusByPch()
{
    // setting a new experation implicitly cancels any pending async wait
    mPollTimer.expires_after(std::chrono::seconds(intrusionSensorPollSec));

    mPollTimer.async_wait([&](const boost::system::error_code& ec) {
        // case of timer expired
        if (!ec)
        {
            int statusValue = i2cReadFromPch(mBusId, mSlaveAddr);
            std::string newValue =
                statusValue != 0 ? "HardwareIntrusion" : "Normal";

            if (newValue != "unknown" && mValue != newValue)
            {
                std::cout << "update value from " << mValue << " to "
                          << newValue << "\n";
                updateValue(newValue);
            }

            // trigger next polling
            pollSensorStatusByPch();
        }
        // case of being canceled
        else if (ec == boost::asio::error::operation_aborted)
        {
            std::cerr << "Timer of intrusion sensor is cancelled. Return \n";
            return;
        }
    });
}

int ChassisIntrusionSensor::hwmonRead()
{
    int fd = open(mHwmonPath.c_str(), O_RDONLY | O_CLOEXEC);
    if (fd < 0)
    {
        std::cerr << "ChassisIntrusionSensor unable to open " << mHwmonPath
                  << "\n";
        return -1;
    }

    char buf[32];
    ssize_t n = read(fd, buf, sizeof(buf) - 1);
    close(fd);
    if (n <= 0)
    {
        std::cerr << "ChassisIntrusionSensor failed to read " << mHwmonPath
                  << "\n";
        return -1;
    }
    buf[n] = '\0';

    int value = 0;
    try
    {
        value = std::stoi(buf);
    }
    catch (const std::exception&)
    {
        std::cerr << "ChassisIntrusionSensor invalid value in " << mHwmonPath
                  << "\n";
        return -1;
    }

    if (debug)
    {
        std::cout << "hwmon " << mHwmonPath << " value = " << value << "\n";
    }
    return value;
}

void ChassisIntrusionSensor::pollSensorStatusByHwmon()
{
    mPollTimer.expires_after(std::chrono::seconds(intrusionSensorPollSec));

    mPollTimer.async_wait([this](const boost::system::error_code& ec) {
        if (!ec)
        {
            int statusValue = hwmonRead();
            if (statusValue >= 0)
            {
                std::string newValue =
                    statusValue != 0 ? "HardwareIntrusion" : "Normal";

                if (newValue != "unknown" /*&& mValue != newValue*/)
                {
                    // std::cout << "update value from " << mValue << " to "
                    //           << newValue << " (channel mask: 0x"
                    //           << std::hex << (statusValue & 0xff) << std::dec
                    //           << ")\n";
                    updateValue(newValue, static_cast<uint8_t>(statusValue));
                }
            }

            pollSensorStatusByHwmon();
        }
        else if (ec == boost::asio::error::operation_aborted)
        {
            std::cerr << "Timer of intrusion sensor is cancelled. Return \n";
            return;
        }
    });
}

int ChassisIntrusionSensor::hwmonRearm()
{
    if (mHwmonPath.empty())
    {
        std::cerr << "ChassisIntrusionSensor hwmon path empty, cannot rearm\n";
        return -1;
    }

    std::string rearmPath = mHwmonPath;
    size_t slash = rearmPath.find_last_of('/');
    if (slash != std::string::npos)
    {
        rearmPath.replace(slash + 1, std::string::npos, "rearm");
    }
    else
    {
        rearmPath = "rearm";
    }

    int fd = open(rearmPath.c_str(), O_WRONLY | O_CLOEXEC);
    if (fd < 0)
    {
        std::cerr << "ChassisIntrusionSensor unable to open " << rearmPath
                  << "\n";
        return -1;
    }

    ssize_t n = write(fd, "1\n", 2);
    close(fd);
    if (n < 0)
    {
        std::cerr << "ChassisIntrusionSensor failed to rearm " << rearmPath
                  << "\n";
        return -1;
    }

    std::cout << "ChassisIntrusionSensor rearmed via " << rearmPath << "\n";
    return 0;
}

void ChassisIntrusionSensor::readGpio()
{
    mGpioLine.event_read();
    auto value = mGpioLine.get_value();

    // set string defined in chassis redfish schema
    std::string newValue = value != 0 ? "HardwareIntrusion" : "Normal";

    if (debug)
    {
        std::cout << "\nGPIO value is " << value << "\n";
        std::cout << "Intrusion sensor value is " << newValue << "\n";
    }

    if (newValue != "unknown" && mValue != newValue)
    {
        std::cout << "update value from " << mValue << " to " << newValue
                  << "\n";
        updateValue(newValue);
    }
}

void ChassisIntrusionSensor::pollSensorStatusByGpio(void)
{
    mGpioFd.async_wait(boost::asio::posix::stream_descriptor::wait_read,
                       [this](const boost::system::error_code& ec) {
        if (ec == boost::system::errc::bad_file_descriptor)
        {
            return; // we're being destroyed
        }
        if (ec)
        {
            std::cerr << "Error on GPIO based intrusion sensor wait event\n";
        }
        else
        {
            readGpio();
        }
        pollSensorStatusByGpio();
    });
}

void ChassisIntrusionSensor::initGpioDeviceFile()
{
    mGpioLine = gpiod::find_line(mPinName);
    if (!mGpioLine)
    {
        std::cerr << "ChassisIntrusionSensor error finding gpio pin name: "
                  << mPinName << "\n";
        return;
    }

    try
    {

        mGpioLine.request(
            {"ChassisIntrusionSensor", gpiod::line_request::EVENT_BOTH_EDGES,
             mGpioInverted ? gpiod::line_request::FLAG_ACTIVE_LOW : 0});

        // set string defined in chassis redfish schema
        auto value = mGpioLine.get_value();
        std::string newValue = value != 0 ? "HardwareIntrusion" : "Normal";
        updateValue(newValue);

        auto gpioLineFd = mGpioLine.event_get_fd();
        if (gpioLineFd < 0)
        {
            std::cerr << "ChassisIntrusionSensor failed to get " << mPinName
                      << " fd\n";
            return;
        }

        mGpioFd.assign(gpioLineFd);
    }
    catch (const std::system_error&)
    {
        std::cerr << "ChassisInrtusionSensor error requesting gpio pin name: "
                  << mPinName << "\n";
        return;
    }
}

int ChassisIntrusionSensor::setSensorValue(const std::string& req,
                                           std::string& propertyValue)
{
    if (!mInternalSet)
    {
        /* For hwmon intrusion sensors, setting Status to Normal means the
         * caller wants to clear the sticky hardware latch (rearm). Perform
         * the real rearm and let subsequent polls update the property. */
        if (mType == IntrusionSensorType::hwmon && req == "Normal")
        {
            if (hwmonRearm() == 0)
            {
                propertyValue = req;
                mOverridenState = false;
            }
            else
            {
                propertyValue = mValue;
            }
        }
        else
        {
            propertyValue = req;
            mOverridenState = true;
        }
    }
    else if (!mOverridenState)
    {
        propertyValue = req;
    }
    return 1;
}

void ChassisIntrusionSensor::start(IntrusionSensorType type, int busId,
                                   int slaveAddr, bool gpioInverted,
                                   const std::string& hwmonPath)
{
    if (debug)
    {
        std::cerr << "enter ChassisIntrusionSensor::start, type = " << type
                  << "\n";
        if (type == IntrusionSensorType::pch)
        {
            std::cerr << "busId = " << busId << ", slaveAddr = " << slaveAddr
                      << "\n";
        }
        else if (type == IntrusionSensorType::gpio)
        {
            std::cerr << "gpio pinName = " << mPinName
                      << ", gpioInverted = " << gpioInverted << "\n";
        }
        else if (type == IntrusionSensorType::hwmon)
        {
            std::cerr << "hwmon path = " << hwmonPath << "\n";
        }
    }

    if ((type == IntrusionSensorType::pch && busId == mBusId &&
         slaveAddr == mSlaveAddr) ||
        (type == IntrusionSensorType::gpio && gpioInverted == mGpioInverted &&
         mInitialized) ||
        (type == IntrusionSensorType::hwmon && hwmonPath == mHwmonPath &&
         mInitialized))
    {
        return;
    }

    mType = type;
    mBusId = busId;
    mSlaveAddr = slaveAddr;
    mGpioInverted = gpioInverted;
    mHwmonPath = hwmonPath;

    bool valid = false;
    if (mType == IntrusionSensorType::pch)
    {
        valid = (mBusId > 0 && mSlaveAddr > 0);
    }
    else if (mType == IntrusionSensorType::gpio)
    {
        valid = true;
    }
    else if (mType == IntrusionSensorType::hwmon)
    {
        valid = !mHwmonPath.empty();
    }

    if (valid)
    {
        std::cerr << "[ChassisIntrusionSensor] starting valid config, type="
                  << mType << "\n";
        // initialize first if not initialized before
        if (!mInitialized)
        {
            mIface->register_property(
                "Status", mValue,
                [&](const std::string& req, std::string& propertyValue) {
                return setSensorValue(req, propertyValue);
                });
            mIface->initialize();

            if (mType == IntrusionSensorType::gpio)
            {
                initGpioDeviceFile();
            }

            mInitialized = true;
        }

        // start polling value
        if (mType == IntrusionSensorType::pch)
        {
            pollSensorStatusByPch();
        }
        else if (mType == IntrusionSensorType::gpio && mGpioLine)
        {
            std::cerr << "Start polling intrusion sensors\n";
            pollSensorStatusByGpio();
        }
        else if (mType == IntrusionSensorType::hwmon)
        {
            pollSensorStatusByHwmon();
        }
    }

    // invalid para, release resource
    else
    {
        if (mInitialized)
        {
            if (mType == IntrusionSensorType::pch ||
                mType == IntrusionSensorType::hwmon)
            {
                mPollTimer.cancel();
            }
            else if (mType == IntrusionSensorType::gpio)
            {
                mGpioFd.close();
                if (mGpioLine)
                {
                    mGpioLine.release();
                }
            }
            mInitialized = false;
        }
    }
}

ChassisIntrusionSensor::ChassisIntrusionSensor(
    boost::asio::io_context& io,
    std::shared_ptr<sdbusplus::asio::dbus_interface> iface) :
    mIface(std::move(iface)),
    mValue("unknown"), mOldValue("unknown"), mPollTimer(io), mGpioFd(io)
{}

ChassisIntrusionSensor::~ChassisIntrusionSensor()
{
    if (mType == IntrusionSensorType::pch ||
        mType == IntrusionSensorType::hwmon)
    {
        mPollTimer.cancel();
    }
    else if (mType == IntrusionSensorType::gpio)
    {
        mGpioFd.close();
        if (mGpioLine)
        {
            mGpioLine.release();
        }
    }
}
