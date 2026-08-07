#pragma once

#include "PwmSensor.hpp"
#include "Thresholds.hpp"
#include "sensor.hpp"

#ifndef __ZEPHYR__
#include <boost/asio/random_access_file.hpp>
#endif
#include <sdbusplus/asio/object_server.hpp>

#include <array>
#include <memory>
#include <string>
#include <utility>

class PSUSensor : public Sensor, public std::enable_shared_from_this<PSUSensor>
{
  public:
    PSUSensor(const std::string& path, const std::string& objectType,
              sdbusplus::asio::object_server& objectServer,
              std::shared_ptr<sdbusplus::asio::connection>& conn,
              boost::asio::io_context& io, const std::string& sensorName,
              std::vector<thresholds::Threshold>&& thresholds,
              const std::string& sensorConfiguration,
              const PowerState& powerState, const std::string& sensorUnits,
              unsigned int factor, double max, double min, double offset,
              const std::string& label, size_t tSize, double pollRate);
    ~PSUSensor() override;
    void setupRead(void);

  private:
    // Note, this buffer is a shared_ptr because during a read, its lifetime
    // might have to outlive the PSUSensor class if the object gets destroyed
    // while in the middle of a read operation
    std::shared_ptr<std::array<char, 128>> buffer;
    sdbusplus::asio::object_server& objServer;
#ifdef __ZEPHYR__
    /* Reads are driven by waitTimer + a plain read()/lseek() on a persistent
     * fd. The fd is deliberately NOT registered with boost::asio's
     * select_reactor: on Zephyr select_reactor::register_descriptor() is a
     * no-op that returns 0, so the reactor never tracks per-descriptor state
     * and an fd that never becomes select()-readable leaves its reactor_op
     * queued forever. Re-assigning the fd each poll therefore accumulated one
     * orphaned read op (plus its handler allocation) per poll cycle, which is
     * what leaked ~44 KB / 155 chunks every 30 s out of the 5 MB arena. */
    int fd{-1};
#else
    boost::asio::random_access_file inputDev;
#endif
    boost::asio::steady_timer waitTimer;
    std::string path;
    unsigned int sensorFactor;
    double sensorOffset;
    thresholds::ThresholdTimer thresholdTimer;
    void restartRead();
    void handleResponse(const boost::system::error_code& err, size_t bytesRead);
    void checkThresholds(void) override;
    unsigned int sensorPollMs = defaultSensorPollMs;

    static constexpr size_t warnAfterErrorCount = 10;

  public:
    static constexpr double defaultSensorPoll = 1.0;
    static constexpr unsigned int defaultSensorPollMs =
        static_cast<unsigned int>(defaultSensorPoll * 1000);
};

class PSUProperty
{
  public:
    PSUProperty(std::string name, double max, double min, unsigned int factor,
                double offset) :
        labelTypeName(std::move(name)),
        maxReading(max), minReading(min), sensorScaleFactor(factor),
        sensorOffset(offset)
    {}
    ~PSUProperty() = default;

    std::string labelTypeName;
    double maxReading;
    double minReading;
    unsigned int sensorScaleFactor;
    double sensorOffset;
};
