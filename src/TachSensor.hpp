#pragma once

#include "Thresholds.hpp"
#include "sensor.hpp"

#ifdef __ZEPHYR__
#include <boost/asio/streambuf.hpp>
#else
#include <boost/asio/random_access_file.hpp>
#endif
#include <boost/container/flat_map.hpp>
#include <boost/container/flat_set.hpp>
#include <gpiod.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

class PresenceSensor
{
  public:
    PresenceSensor(const std::string& gpioName, bool inverted,
                   boost::asio::io_context& io, const std::string& name);
    ~PresenceSensor();

    void monitorPresence(void);
    void read(void);
    bool getValue(void) const;

  private:
    bool status = true;
    gpiod::line gpioLine;
    boost::asio::posix::stream_descriptor gpioFd;
    std::string name;
};

namespace redundancy
{
constexpr const char* full = "Full";
constexpr const char* degraded = "Degraded";
constexpr const char* failed = "Failed";
} // namespace redundancy

class RedundancySensor
{
  public:
    RedundancySensor(size_t count, const std::vector<std::string>& children,
                     sdbusplus::asio::object_server& objectServer,
                     const std::string& sensorConfiguration);
    ~RedundancySensor();

    void update(const std::string& name, bool failed);

  private:
    size_t count;
    std::string state = redundancy::full;
    std::shared_ptr<sdbusplus::asio::dbus_interface> iface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> association;
    sdbusplus::asio::object_server& objectServer;
    boost::container::flat_map<std::string, bool> statuses;
};

class TachSensor :
    public Sensor,
    public std::enable_shared_from_this<TachSensor>
{
  public:
    TachSensor(const std::string& path, const std::string& objectType,
               sdbusplus::asio::object_server& objectServer,
               std::shared_ptr<sdbusplus::asio::connection>& conn,
               std::unique_ptr<PresenceSensor>&& presence,
               std::optional<RedundancySensor>* redundancy,
               boost::asio::io_context& io, const std::string& fanName,
               std::vector<thresholds::Threshold>&& thresholds,
               const std::string& sensorConfiguration,
               const std::pair<double, double>& limits,
               const PowerState& powerState,
               const std::optional<std::string>& led);
    ~TachSensor() override;
    void setupRead();

  private:
    // Ordering is important here; readBuf is first so that it's not destroyed
    // while async operations from other member fields might still be using it.
#ifdef __ZEPHYR__
    boost::asio::streambuf readBuf;
#else
    std::array<char, 128> readBuf{};
#endif
    sdbusplus::asio::object_server& objServer;
    std::optional<RedundancySensor>* redundancy;
    std::unique_ptr<PresenceSensor> presence;
    std::shared_ptr<sdbusplus::asio::dbus_interface> itemIface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> itemAssoc;
#ifdef __ZEPHYR__
    boost::asio::posix::stream_descriptor inputDev;
#else
    boost::asio::random_access_file inputDev;
#endif
    boost::asio::steady_timer waitTimer;
    std::string path;
    std::optional<std::string> led;
    bool ledState = false;

#ifdef __ZEPHYR__
    void handleResponse(const boost::system::error_code& err);
#else
    void handleResponse(const boost::system::error_code& err, size_t bytesRead);
#endif
    void restartRead(size_t pollTime);
    void checkThresholds(void) override;
};

inline void logFanInserted(const std::string& device)
{
    const auto* msg = "OpenBMC.0.1.FanInserted";
    lg2::error("Fan Inserted", "REDFISH_MESSAGE_ID", msg,
               "REDFISH_MESSAGE_ARGS", device);
}

inline void logFanRemoved(const std::string& device)
{
    const auto* msg = "OpenBMC.0.1.FanRemoved";
    lg2::error("Fan Removed", "REDFISH_MESSAGE_ID", msg, "REDFISH_MESSAGE_ARGS",
               device);
}

inline void logFanRedundancyLost(void)
{
    const auto* msg = "OpenBMC.0.1.FanRedundancyLost";
    lg2::error("Fan Inserted", "REDFISH_MESSAGE_ID", msg);
}

inline void logFanRedundancyRestored(void)
{
    const auto* msg = "OpenBMC.0.1.FanRedundancyRegained";
    lg2::error("Fan Removed", "REDFISH_MESSAGE_ID", msg);
}
