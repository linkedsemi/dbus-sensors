#include "ExternalSensor.hpp"
#include "Utils.hpp"
#include "VariantVisitors.hpp"

#include <boost/algorithm/string/replace.hpp>
#include <boost/container/flat_map.hpp>
#include <boost/container/flat_set.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/bus/match.hpp>

#include <array>
#include <filesystem>
#include <fstream>
#include <functional>
#include <memory>
#include <regex>
#include <stdexcept>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#ifdef __ZEPHYR__
#include <iostream>

#include <dbus_broker.h>
#include <zephyr/kernel.h>
#include <printk_thread.h>

extern struct k_sem external_sensor_ready_sem;

/* Enable a built-in self-test that emulates an external writer (see below).
 * Set to 0 to disable. */
#define EXTERNAL_SENSOR_SELFTEST 0
#endif

// Copied from HwmonTempSensor and inspired by
// https://gerrit.openbmc-project.xyz/c/openbmc/dbus-sensors/+/35476

// The ExternalSensor is a sensor whose value is intended to be writable
// by something external to the BMC, so that the host (or something else)
// can write to it, perhaps by using an IPMI or Redfish connection.

// Unlike most other sensors, an external sensor does not correspond
// to a hwmon file or any other kernel/hardware interface,
// so, after initialization, this module does not have much to do,
// but it handles reinitialization and thresholds, similar to the others.
// The main work of this module is to provide backing storage for a
// sensor that exists only virtually, and to provide an optional
// timeout service for detecting loss of timely updates.

// As there is no corresponding driver or hardware to support,
// all configuration of this sensor comes from the JSON parameters:
// MinValue, MaxValue, Timeout, PowerState, Units, Name

// The purpose of "Units" is to specify the physical characteristic
// the external sensor is measuring, because with an external sensor
// there is no other way to tell, and it will be used for the object path
// here: /xyz/openbmc_project/sensors/<Units>/<Name>

// For more information, see external-sensor.md design document:
// https://gerrit.openbmc-project.xyz/c/openbmc/docs/+/41452
// https://github.com/openbmc/docs/tree/master/designs/

static constexpr bool debug = false;

static const char* sensorType = "ExternalSensor";

void updateReaper(boost::container::flat_map<
                      std::string, std::shared_ptr<ExternalSensor>>& sensors,
                  boost::asio::steady_timer& timer,
                  const std::chrono::steady_clock::time_point& now)
{
    // First pass, reap all stale sensors
    for (const auto& [name, sensor] : sensors)
    {
        if (!sensor)
        {
            continue;
        }

        if (!sensor->isAliveAndPerishable())
        {
            continue;
        }

        if (!sensor->isAliveAndFresh(now))
        {
            // Mark sensor as dead, no longer alive
            sensor->writeInvalidate();
        }
    }

    std::chrono::steady_clock::duration nextCheck;
    bool needCheck = false;

    // Second pass, determine timer interval to next check
    for (const auto& [name, sensor] : sensors)
    {
        if (!sensor)
        {
            continue;
        }

        if (!sensor->isAliveAndPerishable())
        {
            continue;
        }

        auto expiration = sensor->ageRemaining(now);

        if (needCheck)
        {
            nextCheck = std::min(nextCheck, expiration);
        }
        else
        {
            // Initialization
            nextCheck = expiration;
            needCheck = true;
        }
    }

    if (!needCheck)
    {
        if constexpr (debug)
        {
            std::cerr << "Next ExternalSensor timer idle\n";
        }

        return;
    }

    timer.expires_at(now + nextCheck);

    timer.async_wait([&sensors, &timer](const boost::system::error_code& err) {
        if (err != boost::system::errc::success)
        {
            // Cancellation is normal, as timer is dynamically rescheduled
            if (err != boost::asio::error::operation_aborted)
            {
                std::cerr << "ExternalSensor timer scheduling problem: "
                          << err.message() << "\n";
            }
            return;
        }

        updateReaper(sensors, timer, std::chrono::steady_clock::now());
    });

    if constexpr (debug)
    {
        std::cerr << "Next ExternalSensor timer "
                  << std::chrono::duration_cast<std::chrono::microseconds>(
                         nextCheck)
                         .count()
                  << " us\n";
    }
}

void createSensors(
    sdbusplus::asio::object_server& objectServer,
    boost::container::flat_map<std::string, std::shared_ptr<ExternalSensor>>&
        sensors,
    std::shared_ptr<sdbusplus::asio::connection>& dbusConnection,
    const std::shared_ptr<boost::container::flat_set<std::string>>&
        sensorsChanged,
    boost::asio::steady_timer& reaperTimer)
{
    if constexpr (debug)
    {
        std::cerr << "ExternalSensor considering creating sensors\n";
    }

    auto getter = std::make_shared<GetSensorConfiguration>(
        dbusConnection,
        [&objectServer, &sensors, &dbusConnection, sensorsChanged,
         &reaperTimer](const ManagedObjectType& sensorConfigurations) {
        bool firstScan = (sensorsChanged == nullptr);

        for (const std::pair<sdbusplus::message::object_path, SensorData>&
                 sensor : sensorConfigurations)
        {
            const std::string& interfacePath = sensor.first.str;
            const SensorData& sensorData = sensor.second;

            auto sensorBase = sensorData.find(configInterfaceName(sensorType));
            if (sensorBase == sensorData.end())
            {
                std::cerr << "Base configuration not found for "
                          << interfacePath << "\n";
                continue;
            }

            const SensorBaseConfiguration& baseConfiguration = *sensorBase;
            const SensorBaseConfigMap& baseConfigMap = baseConfiguration.second;

            // MinValue and MinValue are mandatory numeric parameters
            auto minFound = baseConfigMap.find("MinValue");
            if (minFound == baseConfigMap.end())
            {
                std::cerr << "MinValue parameter not found for "
                          << interfacePath << "\n";
                continue;
            }
            double minValue =
                std::visit(VariantToDoubleVisitor(), minFound->second);
            if (!std::isfinite(minValue))
            {
                std::cerr << "MinValue parameter not parsed for "
                          << interfacePath << "\n";
                continue;
            }

            auto maxFound = baseConfigMap.find("MaxValue");
            if (maxFound == baseConfigMap.end())
            {
                std::cerr << "MaxValue parameter not found for "
                          << interfacePath << "\n";
                continue;
            }
            double maxValue =
                std::visit(VariantToDoubleVisitor(), maxFound->second);
            if (!std::isfinite(maxValue))
            {
                std::cerr << "MaxValue parameter not parsed for "
                          << interfacePath << "\n";
                continue;
            }

            double timeoutSecs = 0.0;

            // Timeout is an optional numeric parameter
            auto timeoutFound = baseConfigMap.find("Timeout");
            if (timeoutFound != baseConfigMap.end())
            {
                timeoutSecs =
                    std::visit(VariantToDoubleVisitor(), timeoutFound->second);
            }
            if (!std::isfinite(timeoutSecs) || (timeoutSecs < 0.0))
            {
                std::cerr << "Timeout parameter not parsed for "
                          << interfacePath << "\n";
                continue;
            }

            std::string sensorName;
            std::string sensorUnits;

            // Name and Units are mandatory string parameters
            auto nameFound = baseConfigMap.find("Name");
            if (nameFound == baseConfigMap.end())
            {
                std::cerr << "Name parameter not found for " << interfacePath
                          << "\n";
                continue;
            }
            sensorName =
                std::visit(VariantToStringVisitor(), nameFound->second);
            if (sensorName.empty())
            {
                std::cerr << "Name parameter not parsed for " << interfacePath
                          << "\n";
                continue;
            }

            auto unitsFound = baseConfigMap.find("Units");
            if (unitsFound == baseConfigMap.end())
            {
                std::cerr << "Units parameter not found for " << interfacePath
                          << "\n";
                continue;
            }
            sensorUnits =
                std::visit(VariantToStringVisitor(), unitsFound->second);
            if (sensorUnits.empty())
            {
                std::cerr << "Units parameter not parsed for " << interfacePath
                          << "\n";
                continue;
            }

            // on rescans, only update sensors we were signaled by
            auto findSensor = sensors.find(sensorName);
            if (!firstScan && (findSensor != sensors.end()))
            {
                std::string suffixName = "/";
                suffixName += findSensor->second->name;
                bool found = false;
                for (auto it = sensorsChanged->begin();
                     it != sensorsChanged->end(); it++)
                {
                    std::string suffixIt = "/";
                    suffixIt += *it;
                    if (suffixIt.ends_with(suffixName))
                    {
                        sensorsChanged->erase(it);
                        findSensor->second = nullptr;
                        found = true;
                        if constexpr (debug)
                        {
                            std::cerr << "ExternalSensor " << sensorName
                                      << " change found\n";
                        }
                        break;
                    }
                }
                if (!found)
                {
                    continue;
                }
            }

            std::vector<thresholds::Threshold> sensorThresholds;
            if (!parseThresholdsFromConfig(sensorData, sensorThresholds))
            {
                std::cerr << "error populating thresholds for " << sensorName
                          << "\n";
            }

            PowerState readState = getPowerState(baseConfigMap);

            auto& sensorEntry = sensors[sensorName];
            sensorEntry = nullptr;

            sensorEntry = std::make_shared<ExternalSensor>(
                sensorType, objectServer, dbusConnection, sensorName,
                sensorUnits, std::move(sensorThresholds), interfacePath,
                maxValue, minValue, timeoutSecs, readState);
            sensorEntry->initWriteHook(
                [&sensors, &reaperTimer](
                    const std::chrono::steady_clock::time_point& now) {
                updateReaper(sensors, reaperTimer, now);
            });

            if constexpr (debug)
            {
                std::cerr << "ExternalSensor " << sensorName << " created\n";
            }
        }
        });

    getter->getConfiguration(std::vector<std::string>{sensorType});
}

#ifdef __ZEPHYR__

#if EXTERNAL_SENSOR_SELFTEST
/*
 * Self-test emulating an "external writer".
 *
 * It drives the sensor purely through the dbus broker (just like a real
 * external data source would), so the full external-set path is validated
 * end-to-end without any hardware:
 *
 *   Properties.Set("Value")
 *     -> Sensor::setSensorValue          (sensor.hpp:224)
 *     -> externalSetHook
 *     -> ExternalSensor::externalSetTrigger (ExternalSensor.cpp:185)
 *     -> writeBegin()  (records timestamp, marks alive)
 *
 * It also toggles the State.Decorator.Availability property midway to
 * exercise that path (Available=false drives Value -> NaN).
 */
static void externalSensorSelfTest(
    boost::asio::io_context& io,
    sdbusplus::asio::object_server& objectServer,
    std::shared_ptr<sdbusplus::asio::connection>& bus,
    boost::container::flat_map<std::string, std::shared_ptr<ExternalSensor>>&
        sensors)
{
    auto timer = std::make_shared<boost::asio::steady_timer>(io);
    auto waitedOnce = std::make_shared<bool>(false);
    auto remaining = std::make_shared<int>(5);
    auto step = std::make_shared<int>(0);
    auto tick = std::make_shared<std::function<void()>>();

    // Open a SEPARATE broker connection for the write exercise. Calling
    // "xyz.openbmc_project.ExternalSensor" (a name THIS connection owns)
    // through the broker is a self-call that hangs the single-threaded
    // dbroker and freezes every other broker-dependent thread. A distinct
    // source connection avoids the self-call while still exercising the
    // D-Bus write path end-to-end.
    sd_bus* testBus = nullptr;
    int testRc = connect_to_dbroker(&testBus);
    std::shared_ptr<sdbusplus::asio::connection> testConn;
    if (testRc == 0 && testBus != nullptr)
    {
        testConn = std::make_shared<sdbusplus::asio::connection>(io, testBus);
    }
    else
    {
        printk_thread("self-test: cannot open separate test bus (%d), "
                      "skipping broker write exercise", testRc);
    }

    // Phase A: wait a single time for entity-manager to publish the
    // ExternalSensor config (its 5s debounce + ~1s processing).
    // If sensors still empty after that, fallback creates its own sensor.

    *tick = [timer, waitedOnce, remaining, step, &bus, &objectServer,
             &sensors, tick, testConn]() {
        // Phase A: single delay for entity-manager.
        if (sensors.empty() && !*waitedOnce)
        {
            *waitedOnce = true;
            printk_thread("self-test: waiting 6s for ExternalSensor config");
            timer->expires_after(std::chrono::seconds(6));
            timer->async_wait([timer, tick](boost::system::error_code) {
                (*tick)();
            });
            return;
        }

        // Fallback: if still nothing, create a sensor directly so the write
        // path is still validated (entity-manager may be slow or absent).
        if (sensors.empty())
        {
            printk_thread("self-test: no config after 6s, creating fallback "
                          "sensor 'SelfTestExternalSensor'");
            std::vector<thresholds::Threshold> noThresholds;
            auto& entry = sensors["SelfTestExternalSensor"];
            entry = std::make_shared<ExternalSensor>(
                "ExternalSensor", objectServer, bus, "SelfTestExternalSensor",
                "DegreesC", std::move(noThresholds),
                "/xyz/openbmc_project/inventory/system/board/"
                "SelfTestExternalSensor",
                125.0, -40.0, 0.0, PowerState::always);
            entry->initWriteHook(
                [](const std::chrono::steady_clock::time_point&) {});
        }

        if (*remaining <= 0)
        {
            printk_thread("self-test: finished 5 external writes");
            return;
        }

        double value = 20.0 + (*step) * 5.0;  // 20, 25, 30, 35, 40

        for (auto& kv : sensors)
        {
            const std::string& name = kv.first;
            std::shared_ptr<ExternalSensor>& sensor = kv.second;
            std::string path = sensor->sensorInterface->get_object_path();

            if (!testConn)
            {
                printk_thread("self-test: no test bus, skipping write of %s",
                              name.c_str());
                continue;
            }

            testConn->async_method_call(
                [name, value](boost::system::error_code ec) {
                    if (ec)
                    {
                        printk_thread("self-test: Set Value %s=%.1f FAIL %d",
                                      name.c_str(), value, ec.value());
                    }
                    else
                    {
                        printk_thread("self-test: Set Value %s=%.1f OK",
                                      name.c_str(), value);
                    }
                },
                "xyz.openbmc_project.ExternalSensor", path,
                "org.freedesktop.DBus.Properties", "Set",
                "xyz.openbmc_project.Sensor.Value", "Value",
                std::variant<double>(value));

            // Midway, toggle the Availability decorator off then on to
            // exercise the State.Decorator.Availability path
            // (Available=false drives Value -> NaN).
            if (*step == 2)
            {
                bus->async_method_call(
                    [](boost::system::error_code) {},
                    "xyz.openbmc_project.ExternalSensor", path,
                    "org.freedesktop.DBus.Properties", "Set",
                    "xyz.openbmc_project.State.Decorator.Availability",
                    "Available", std::variant<bool>(false));
            }
            if (*step == 3)
            {
                bus->async_method_call(
                    [](boost::system::error_code) {},
                    "xyz.openbmc_project.ExternalSensor", path,
                    "org.freedesktop.DBus.Properties", "Set",
                    "xyz.openbmc_project.State.Decorator.Availability",
                    "Available", std::variant<bool>(true));
            }
        }

        --(*remaining);
        ++(*step);

        timer->expires_after(std::chrono::seconds(1));
        timer->async_wait([timer, tick](boost::system::error_code) {
            (*tick)();
        });
    };

    timer->async_wait([timer, tick](boost::system::error_code) {
        (*tick)();
    });

    disconnect_from_dbroker(testBus);
}
#endif /* EXTERNAL_SENSOR_SELFTEST */

int external_sensor_main()
{
    printk_thread(">>> external_sensor_main");
    boost::asio::io_context io;
    sd_bus* bus = nullptr;
    int rc = connect_to_dbroker(&bus);
    if (rc < 0)
    {
        printk_thread("Failed to connect to dbroker: %d", rc);
        return rc;
    }
    auto systemBus = std::make_shared<sdbusplus::asio::connection>(io, bus);

    sdbusplus::asio::object_server objectServer(systemBus, true);
    objectServer.add_manager("/xyz/openbmc_project/sensors");

    systemBus->request_name("xyz.openbmc_project.ExternalSensor");

    boost::container::flat_map<std::string, std::shared_ptr<ExternalSensor>>
        sensors;
    auto sensorsChanged =
        std::make_shared<boost::container::flat_set<std::string>>();
    boost::asio::steady_timer reaperTimer(io);

    boost::asio::post(io, [&objectServer, &sensors, &systemBus,
                            &reaperTimer]() {
        createSensors(objectServer, sensors, systemBus, nullptr, reaperTimer);
    });

    boost::asio::steady_timer filterTimer(io);
    std::function<void(sdbusplus::message_t&)> eventHandler =
        [&objectServer, &sensors, &systemBus, &sensorsChanged, &filterTimer,
         &reaperTimer](sdbusplus::message_t& message) {
        if (message.is_method_error())
        {
            std::cerr << "callback method error\n";
            return;
        }

        const auto* messagePath = message.get_path();
        sensorsChanged->insert(messagePath);
        if constexpr (debug)
        {
            std::cerr << "ExternalSensor change event received: "
                      << messagePath << "\n";
        }

        // this implicitly cancels the timer
        filterTimer.expires_after(std::chrono::seconds(1));

        filterTimer.async_wait(
            [&objectServer, &sensors, &systemBus, &sensorsChanged,
             &reaperTimer](const boost::system::error_code& ec) {
            if (ec != boost::system::errc::success)
            {
                if (ec != boost::asio::error::operation_aborted)
                {
                    std::cerr << "callback error: " << ec.message() << "\n";
                }
                return;
            }

            createSensors(objectServer, sensors, systemBus, sensorsChanged,
                          reaperTimer);
        });
    };

    std::vector<std::unique_ptr<sdbusplus::bus::match_t>> matches =
        setupPropertiesChangedMatches(
            *systemBus, std::to_array<const char*>({sensorType}), eventHandler);

#if EXTERNAL_SENSOR_SELFTEST
    externalSensorSelfTest(io, objectServer, systemBus, sensors);
#endif

    k_sem_give(&external_sensor_ready_sem);

    io.run();
}
#else
int main()
{
    if constexpr (debug)
    {
        std::cerr << "ExternalSensor service starting up\n";
    }

    boost::asio::io_context io;
    auto systemBus = std::make_shared<sdbusplus::asio::connection>(io);
    sdbusplus::asio::object_server objectServer(systemBus, true);

    objectServer.add_manager("/xyz/openbmc_project/sensors");
    systemBus->request_name("xyz.openbmc_project.ExternalSensor");

    boost::container::flat_map<std::string, std::shared_ptr<ExternalSensor>>
        sensors;
    auto sensorsChanged =
        std::make_shared<boost::container::flat_set<std::string>>();
    boost::asio::steady_timer reaperTimer(io);

    boost::asio::post(io,
                      [&objectServer, &sensors, &systemBus, &reaperTimer]() {
        createSensors(objectServer, sensors, systemBus, nullptr, reaperTimer);
    });

    boost::asio::steady_timer filterTimer(io);
    std::function<void(sdbusplus::message_t&)> eventHandler =
        [&objectServer, &sensors, &systemBus, &sensorsChanged, &filterTimer,
         &reaperTimer](sdbusplus::message_t& message) mutable {
        if (message.is_method_error())
        {
            std::cerr << "callback method error\n";
            return;
        }

        const auto* messagePath = message.get_path();
        sensorsChanged->insert(messagePath);
        if constexpr (debug)
        {
            std::cerr << "ExternalSensor change event received: " << messagePath
                      << "\n";
        }

        // this implicitly cancels the timer
        filterTimer.expires_after(std::chrono::seconds(1));

        filterTimer.async_wait(
            [&objectServer, &sensors, &systemBus, &sensorsChanged,
             &reaperTimer](const boost::system::error_code& ec) mutable {
            if (ec != boost::system::errc::success)
            {
                if (ec != boost::asio::error::operation_aborted)
                {
                    std::cerr << "callback error: " << ec.message() << "\n";
                }
                return;
            }

            createSensors(objectServer, sensors, systemBus, sensorsChanged,
                          reaperTimer);
        });
    };

    std::vector<std::unique_ptr<sdbusplus::bus::match_t>> matches =
        setupPropertiesChangedMatches(
            *systemBus, std::to_array<const char*>({sensorType}), eventHandler);

    if constexpr (debug)
    {
        std::cerr << "ExternalSensor service entering main loop\n";
    }

    io.run();
}
#endif /* __ZEPHYR__ */
