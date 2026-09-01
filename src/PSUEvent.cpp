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

#include "PSUEvent.hpp"

#include "SensorPaths.hpp"

#ifdef __ZEPHYR__
#include <cerrno>
#include <fcntl.h>
#include <unistd.h>
#endif
#include <boost/asio/io_context.hpp>
#include <boost/asio/read_until.hpp>
#include <boost/container/flat_map.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <variant>
#include <vector>

PSUCombineEvent::PSUCombineEvent(
    sdbusplus::asio::object_server& objectServer,
    std::shared_ptr<sdbusplus::asio::connection>& conn,
    boost::asio::io_context& io, const std::string& psuName,
    const PowerState& powerState,
    boost::container::flat_map<std::string, std::vector<std::string>>&
        eventPathList,
    boost::container::flat_map<
        std::string,
        boost::container::flat_map<std::string, std::vector<std::string>>>&
        groupEventPathList,
    const std::string& combineEventName, double pollRate) :
    objServer(objectServer)
{
    std::string psuNameEscaped = sensor_paths::escapePathForDbus(psuName);
    eventInterface = objServer.add_interface(
        "/xyz/openbmc_project/State/Decorator/" + psuNameEscaped + "_" +
            combineEventName,
        "xyz.openbmc_project.State.Decorator.OperationalStatus");
    eventInterface->register_property("functional", true);

    if (!eventInterface->initialize())
    {
        std::cerr << "error initializing event interface\n";
    }

    std::shared_ptr<std::set<std::string>> combineEvent =
        std::make_shared<std::set<std::string>>();
    for (const auto& [eventName, paths] : eventPathList)
    {
        std::shared_ptr<std::set<std::string>> assert =
            std::make_shared<std::set<std::string>>();
        std::shared_ptr<bool> state = std::make_shared<bool>(false);

        std::string eventPSUName = eventName + psuName;
        for (const auto& path : paths)
        {
            auto p = std::make_shared<PSUSubEvent>(
                eventInterface, path, conn, io, powerState, eventName,
                eventName, assert, combineEvent, state, psuName, pollRate);
            p->setupRead();

            events[eventPSUName].emplace_back(p);
            asserts.emplace_back(assert);
            states.emplace_back(state);
        }
    }

    for (const auto& [eventName, groupEvents] : groupEventPathList)
    {
        for (const auto& [groupEventName, paths] : groupEvents)
        {
            std::shared_ptr<std::set<std::string>> assert =
                std::make_shared<std::set<std::string>>();
            std::shared_ptr<bool> state = std::make_shared<bool>(false);

            std::string eventPSUName = groupEventName + psuName;
            for (const auto& path : paths)
            {
                auto p = std::make_shared<PSUSubEvent>(
                    eventInterface, path, conn, io, powerState, groupEventName,
                    eventName, assert, combineEvent, state, psuName, pollRate);
                p->setupRead();
                events[eventPSUName].emplace_back(p);

                asserts.emplace_back(assert);
                states.emplace_back(state);
            }
        }
    }
}

PSUCombineEvent::~PSUCombineEvent()
{
    // Clear unique_ptr first
    for (auto& [psuName, subEvents] : events)
    {
        for (auto& subEventPtr : subEvents)
        {
            subEventPtr.reset();
        }
    }
    events.clear();
    objServer.remove_interface(eventInterface);
}

static boost::container::flat_map<std::string,
                                  std::pair<std::string, std::string>>
    logID = {
        {"PredictiveFailure",
         {"OpenBMC.0.1.PowerSupplyFailurePredicted",
          "OpenBMC.0.1.PowerSupplyPredictedFailureRecovered"}},
        {"Failure",
         {"OpenBMC.0.1.PowerSupplyFailed", "OpenBMC.0.1.PowerSupplyRecovered"}},
        {"ACLost",
         {"OpenBMC.0.1.PowerSupplyPowerLost",
          "OpenBMC.0.1.PowerSupplyPowerRestored"}},
        {"FanFault",
         {"OpenBMC.0.1.PowerSupplyFanFailed",
          "OpenBMC.0.1.PowerSupplyFanRecovered"}},
        {"ConfigureError",
         {"OpenBMC.0.1.PowerSupplyConfigurationError",
          "OpenBMC.0.1.PowerSupplyConfigurationErrorRecovered"}}};

PSUSubEvent::PSUSubEvent(
    std::shared_ptr<sdbusplus::asio::dbus_interface> eventInterface,
    const std::string& path, std::shared_ptr<sdbusplus::asio::connection>& conn,
    boost::asio::io_context& io, const PowerState& powerState,
    const std::string& groupEventName, const std::string& eventName,
    std::shared_ptr<std::set<std::string>> asserts,
    std::shared_ptr<std::set<std::string>> combineEvent,
    std::shared_ptr<bool> state, const std::string& psuName, double pollRate) :
    eventInterface(std::move(eventInterface)),
    asserts(std::move(asserts)), combineEvent(std::move(combineEvent)),
    assertState(std::move(state)), path(path), eventName(eventName),
    readState(powerState), waitTimer(io),

#ifdef __ZEPHYR__
    psuName(psuName), groupEventName(groupEventName), systemBus(conn)
#else
    inputDev(io, path, boost::asio::random_access_file::read_only),
    psuName(psuName), groupEventName(groupEventName), systemBus(conn)
#endif
{
    buffer = std::make_shared<std::array<char, 128>>();
    if (pollRate > 0.0)
    {
        eventPollMs = static_cast<unsigned int>(pollRate * 1000);
    }

    auto found = logID.find(eventName);
    if (found == logID.end())
    {
        assertMessage.clear();
        deassertMessage.clear();
    }
    else
    {
        assertMessage = found->second.first;
        deassertMessage = found->second.second;
    }

    auto fanPos = path.find("fan");
    if (fanPos != std::string::npos)
    {
        fanName = path.substr(fanPos);
        auto fanNamePos = fanName.find('_');
        if (fanNamePos != std::string::npos)
        {
            fanName = fanName.substr(0, fanNamePos);
        }
    }
}

PSUSubEvent::~PSUSubEvent()
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
}

void PSUSubEvent::setupRead(void)
{
    if (!readingStateGood(readState))
    {
        // Deassert the event
        updateValue(0);
        restartRead();
        return;
    }
    if (!buffer)
    {
        std::cerr << "Buffer was invalid?";
        return;
    }

#ifdef __ZEPHYR__
    /* On Zephyr, sysfs/hwmon attrs are "one read per open" (ppos then EOF);
     * reopen every poll so every read gets a fresh value, and use O_NONBLOCK
     * so a slow/unresponsive I2C transaction can never block this io_context
     * thread. Never register the fd with boost::asio's select_reactor: doing
     * so leaks a reactor descriptor_state per poll cycle. */
    if (fd >= 0)
    {
        close(fd);
        fd = -1;
    }
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-vararg)
    fd = open(path.c_str(), O_RDONLY | O_NONBLOCK);
    if (fd < 0)
    {
        std::cerr << eventName << " unable to open event fd!\n";
        restartRead();
        return;
    }
    ssize_t rdLen = read(fd, buffer->data(), buffer->size() - 1);
    if (rdLen < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
    {
        /* I2C transaction not ready yet; retry next poll. Do not tear down
         * the fd; setupRead() reopens it on the next poll anyway. */
        restartRead();
        return;
    }
    /* On read failure (non-EAGAIN) report 0 bytes: handleResponse() then
     * counts it as an error via errCount++ and keeps the poll loop alive
     * instead of bailing out on bad_file_descriptor/not_found. */
    handleResponse(boost::system::error_code(),
                   rdLen > 0 ? static_cast<size_t>(rdLen) : 0);
#else
    std::weak_ptr<PSUSubEvent> weakRef = weak_from_this();
    inputDev.async_read_some_at(
        0, boost::asio::buffer(buffer->data(), buffer->size() - 1),
        [weakRef, buffer{buffer}](const boost::system::error_code& ec,
                                  std::size_t bytesTransferred) {
        std::shared_ptr<PSUSubEvent> self = weakRef.lock();
        if (self)
        {
            self->handleResponse(ec, bytesTransferred);
        }
        });
#endif
}

void PSUSubEvent::restartRead()
{
    std::weak_ptr<PSUSubEvent> weakRef = weak_from_this();
    waitTimer.expires_after(std::chrono::milliseconds(eventPollMs));
    waitTimer.async_wait([weakRef](const boost::system::error_code& ec) {
        if (ec == boost::asio::error::operation_aborted)
        {
            return;
        }
        std::shared_ptr<PSUSubEvent> self = weakRef.lock();
        if (self)
        {
            self->setupRead();
        }
    });
}

void PSUSubEvent::handleResponse(const boost::system::error_code& err,
                                 size_t bytesTransferred)
{
    if (err == boost::asio::error::operation_aborted)
    {
        return;
    }

    if ((err == boost::system::errc::bad_file_descriptor) ||
        (err == boost::asio::error::misc_errors::not_found))
    {
        return;
    }
    if (!buffer)
    {
        std::cerr << "Buffer was invalid?";
        return;
    }
    // null terminate the string so we don't walk off the end
    std::array<char, 128>& bufferRef = *buffer;
    bufferRef[bytesTransferred] = '\0';

    if (!err)
    {
        std::string response;
        try
        {
            int nvalue = std::stoi(bufferRef.data());
            updateValue(nvalue);
            errCount = 0;
        }
        catch (const std::invalid_argument&)
        {
            errCount++;
        }
    }
    else
    {
        errCount++;
    }
    if (errCount >= warnAfterErrorCount)
    {
        if (errCount == warnAfterErrorCount)
        {
            std::cerr << "Failure to read event at " << path << "\n";
        }
        updateValue(0);
        errCount++;
    }
    /* On Zephyr the fd is reopened by setupRead() every poll, so no
     * close/reopen is needed here. */
    restartRead();
}

// Any of the sub events of one event is asserted, then the event will be
// asserted. Only if none of the sub events are asserted, the event will be
// deasserted.
void PSUSubEvent::updateValue(const int& newValue)
{
    // Take no action if value already equal
    // Same semantics as Sensor::updateValue(const double&)
    if (newValue == value)
    {
        return;
    }

    if (newValue == 0)
    {
        // log deassert only after all asserts are gone
        if (!(*asserts).empty())
        {
            auto found = (*asserts).find(path);
            if (found == (*asserts).end())
            {
                return;
            }
            (*asserts).erase(path);

            return;
        }
        if (*assertState)
        {
            *assertState = false;
            auto foundCombine = (*combineEvent).find(groupEventName);
            if (foundCombine == (*combineEvent).end())
            {
                return;
            }
            (*combineEvent).erase(groupEventName);
            if (!deassertMessage.empty())
            {
                // Fan Failed has two args
                if (deassertMessage == "OpenBMC.0.1.PowerSupplyFanRecovered")
                {
                    lg2::info("{EVENT} deassert", "EVENT", eventName,
                              "REDFISH_MESSAGE_ID", deassertMessage,
                              "REDFISH_MESSAGE_ARGS",
                              (psuName + ',' + fanName));
                }
                else
                {
                    lg2::info("{EVENT} deassert", "EVENT", eventName,
                              "REDFISH_MESSAGE_ID", deassertMessage,
                              "REDFISH_MESSAGE_ARGS", psuName);
                }
            }

            if ((*combineEvent).empty())
            {
                eventInterface->set_property("functional", true);
            }
        }
    }
    else
    {
        std::cerr << "PSUSubEvent asserted by " << path << "\n";

        if ((!*assertState) && ((*asserts).empty()))
        {
            *assertState = true;
            if (!assertMessage.empty())
            {
                // Fan Failed has two args
                if (assertMessage == "OpenBMC.0.1.PowerSupplyFanFailed")
                {
                    lg2::warning("{EVENT} assert", "EVENT", eventName,
                                 "REDFISH_MESSAGE_ID", assertMessage,
                                 "REDFISH_MESSAGE_ARGS",
                                 (psuName + ',' + fanName));
                }
                else
                {
                    lg2::warning("{EVENT} assert", "EVENT", eventName,
                                 "REDFISH_MESSAGE_ID", assertMessage,
                                 "REDFISH_MESSAGE_ARGS", psuName);
                }
            }
            if ((*combineEvent).empty())
            {
                eventInterface->set_property("functional", false);
            }
            (*combineEvent).emplace(groupEventName);
        }
        (*asserts).emplace(path);
    }
    value = newValue;
}
