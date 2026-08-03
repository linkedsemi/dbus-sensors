if(CONFIG_OPENBMC_DBUS_SENSORS)

    #定义配置变量
    set(VALIDATION_UNSECURE_FEATURE 1)
    set(INSECURE_UNRESTRICTED_SENSOR_OVERRIDE 1)

    # 设置配置文件的源文件和目标文件
    set(DBUS_SENSOR_CONFIG_IN ${ZEPHYR_CURRENT_MODULE_DIR}/src/dbus-sensor_config.h.in)
    set(DBUS_SENSOR_CONFIG ${ZEPHYR_CURRENT_MODULE_DIR}//src/dbus-sensor_config.h)

    # 配置文件
    configure_file(${DBUS_SENSOR_CONFIG_IN} ${DBUS_SENSOR_CONFIG} @ONLY)

    zephyr_library_named(dbus-sensors)
    # Common sources, always built (shared by every sensor).
    zephyr_library_sources(
            ${ZEPHYR_CURRENT_MODULE_DIR}/src/DeviceMgmt.cpp
            ${ZEPHYR_CURRENT_MODULE_DIR}/src/FileHandle.cpp
            ${ZEPHYR_CURRENT_MODULE_DIR}/src/SensorPaths.cpp
            ${ZEPHYR_CURRENT_MODULE_DIR}/src/Thresholds.cpp
            ${ZEPHYR_CURRENT_MODULE_DIR}/src/Utils.cpp
    )
    zephyr_library_sources(
            ${ZEPHYR_CURRENT_MODULE_DIR}/src/ADCSensor.cpp
            ${ZEPHYR_CURRENT_MODULE_DIR}/src/ADCSensorMain.cpp
            ${ZEPHYR_CURRENT_MODULE_DIR}/src/ChassisIntrusionSensor.cpp
            # ${ZEPHYR_CURRENT_MODULE_DIR}/src/DeviceMgmt.cpp
            # #${ZEPHYR_CURRENT_MODULE_DIR}/src/ExitAirTempSensor.cpp
            ${ZEPHYR_CURRENT_MODULE_DIR}/src/ExternalSensor.cpp
            ${ZEPHYR_CURRENT_MODULE_DIR}/src/ExternalSensorMain.cpp
            # ${ZEPHYR_CURRENT_MODULE_DIR}/src/FileHandle.cpp
            ${ZEPHYR_CURRENT_MODULE_DIR}/src/FanMain.cpp
            ${ZEPHYR_CURRENT_MODULE_DIR}/src/HwmonTempSensor.cpp
            ${ZEPHYR_CURRENT_MODULE_DIR}/src/HwmonTempMain.cpp
            # ${ZEPHYR_CURRENT_MODULE_DIR}/src/IntelCPUSensor.cpp
            # ${ZEPHYR_CURRENT_MODULE_DIR}/src/IntelCPUSensorMain.cpp
            ${ZEPHYR_CURRENT_MODULE_DIR}/src/IntrusionSensorMain.cpp
            #${ZEPHYR_CURRENT_MODULE_DIR}/src/IpmbSDRSensor.cpp
            #${ZEPHYR_CURRENT_MODULE_DIR}/src/IpmbSensor.cpp
            #${ZEPHYR_CURRENT_MODULE_DIR}/src/MCUTempSensor.cpp
            #${ZEPHYR_CURRENT_MODULE_DIR}/src/NVMeBasicContext.cpp
            #${ZEPHYR_CURRENT_MODULE_DIR}/src/NVMeSensor.cpp
            #${ZEPHYR_CURRENT_MODULE_DIR}/src/NVMeSensorMain.cpp
            ${ZEPHYR_CURRENT_MODULE_DIR}/src/PSUEvent.cpp
            ${ZEPHYR_CURRENT_MODULE_DIR}/src/PSUSensor.cpp
            ${ZEPHYR_CURRENT_MODULE_DIR}/src/PSUSensorMain.cpp
            ${ZEPHYR_CURRENT_MODULE_DIR}/src/PwmSensor.cpp
            # ${ZEPHYR_CURRENT_MODULE_DIR}/src/SensorPaths.cpp
            ${ZEPHYR_CURRENT_MODULE_DIR}/src/TachSensor.cpp
            # ${ZEPHYR_CURRENT_MODULE_DIR}/src/Thresholds.cpp
            # ${ZEPHYR_CURRENT_MODULE_DIR}/src/Utils.cpp
    )
    zephyr_library_include_directories(${ZEPHYR_CURRENT_MODULE_DIR}/)
    zephyr_library_include_directories(${ZEPHYR_CURRENT_MODULE_DIR}/zephyr)
    zephyr_library_include_directories(${ZEPHYR_CURRENT_MODULE_DIR}/include)
    zephyr_library_include_directories(${ZEPHYR_CURRENT_MODULE_DIR}/src)

    zephyr_library_link_libraries(basu)
    zephyr_library_link_libraries(boost)
    zephyr_library_link_libraries(zephyr-osal)
    zephyr_library_link_libraries(sdbusplus)
    zephyr_library_link_libraries(phosphor-logging)
    zephyr_library_link_libraries(libgpiod)
    zephyr_library_link_libraries(i2c-tools)
    # zephyr_library_compile_definitions(_GNU_SOURCE)
endif()
