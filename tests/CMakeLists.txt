cmake_minimum_required(VERSION 3.13)

project(TrackerTests
    VERSION 1.0.0.0
    LANGUAGES CXX C ASM
)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(ROOT_DIR "..")
set(LINUX_DIR "${ROOT_DIR}/ports/linux")
set(NRF_DIR "${ROOT_DIR}/ports/nrf52840")

# === Git metadata for test report header (TrackerTests + TurtleSimulation) ===
# Captured at CMake configure time. Reconfigure (re-run cmake) to refresh after
# committing if you want the report to show the latest SHA.
execute_process(
    COMMAND git rev-parse --short HEAD
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/..
    OUTPUT_VARIABLE GIT_COMMIT_SHORT
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
)
execute_process(
    COMMAND git rev-parse HEAD
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/..
    OUTPUT_VARIABLE GIT_COMMIT_FULL
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
)
execute_process(
    COMMAND git rev-parse --abbrev-ref HEAD
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/..
    OUTPUT_VARIABLE GIT_BRANCH
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
)
execute_process(
    COMMAND git describe --always --dirty --tags
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/..
    OUTPUT_VARIABLE GIT_DESCRIBE
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
)
if(NOT GIT_COMMIT_SHORT)
    set(GIT_COMMIT_SHORT "unknown")
    set(GIT_COMMIT_FULL "unknown")
    set(GIT_BRANCH "unknown")
    set(GIT_DESCRIBE "unknown")
endif()
string(TIMESTAMP BUILD_TIMESTAMP "%Y-%m-%dT%H:%M:%S")
message(STATUS "Test build metadata: branch=${GIT_BRANCH} commit=${GIT_COMMIT_SHORT} describe=${GIT_DESCRIBE}")

# CppUTest
include(FetchContent)
FetchContent_Declare(
    CppUTest
    GIT_REPOSITORY https://github.com/cpputest/cpputest.git
    GIT_TAG        latest-passing-build # or use release tag, eg. v3.8
)
# Set this to ON if you want to have the CppUTests in your project as well.
set(TESTS OFF CACHE BOOL "Switch off CppUTest Test build")
# Disable memory leak detection to prevent CppUTest's #define new from breaking
# standard library headers (placement new in <variant>, <functional>, etc.)
set(CPPUTEST_MEM_LEAK_DETECTION_DISABLED ON CACHE BOOL "Disable CppUTest memory leak detection" FORCE)
FetchContent_MakeAvailable(CppUTest)
target_compile_definitions(CppUTest PUBLIC CPPUTEST_MEM_LEAK_DETECTION_DISABLED)
target_compile_definitions(CppUTestExt PUBLIC CPPUTEST_MEM_LEAK_DETECTION_DISABLED)

find_package(Threads)
add_executable(${PROJECT_NAME})

# Symlink the test data directory into the build dir so tests can read "data/..."
# relative to their CWD. Remove any pre-existing data link/dir FIRST: file(CREATE_LINK)
# does NOT overwrite an existing link, so a stale one from an older config — notably
# the former RELATIVE "data" target, which resolved circularly to build/data itself and
# exposed 0 files — would persist and make every file-backed test silently read empty
# data (e.g. PASPW/AOP tests failing with MISSING_ARGUMENT). REMOVE_RECURSE on a symlink
# deletes only the link, never the source directory it points to.
file(REMOVE_RECURSE "${CMAKE_CURRENT_BINARY_DIR}/data")
file(CREATE_LINK "${CMAKE_CURRENT_SOURCE_DIR}/data" "${CMAKE_CURRENT_BINARY_DIR}/data" SYMBOLIC)

set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 20)
set_property(TARGET ${PROJECT_NAME} PROPERTY CMAKE_CXX_STANDARD_REQUIRED ON)

target_compile_definitions(${PROJECT_NAME} PRIVATE
    LFS_NO_DEBUG
    DEBUG_ENABLE
    CPPUTEST
    # Enable all sensors for testing
    ENABLE_PH_SENSOR=1
    ENABLE_SEA_TEMP_SENSOR=1
    ENABLE_PRESSURE_SENSOR=1
    ENABLE_ALS_SENSOR=1
    ENABLE_CDT_SENSOR=1
    ENABLE_AXL_SENSOR=1
    ENABLE_THERMISTOR_SENSOR=1
    ENABLE_MORTALITY_SENSOR=1
    ENABLE_SWS_LOG=1
    ENABLE_SWS_ANALOG=1
    LORA_RAK3172=1
    LORA_DCS_ENABLE=1           # Default ON: RAK3172 enforces EU868 duty-cycle
    LORA_POWER_OFF_UNDERWATER=0 # Default OFF: keep module in Stop2 standby during dives
)

target_compile_options(${PROJECT_NAME} PRIVATE
    -g3

    # Warnings
    -Wall
    -Wextra
    -Werror
    -Wduplicated-cond
    -Wduplicated-branches
    -Wlogical-op
    -Wnull-dereference
    -Wshadow
    -Wpointer-arith
    -Wdangling-else
    -Wrestrict
    -Wdouble-promotion
    -Wvla
    -Wswitch-enum
    -Wswitch-default
    -Winvalid-pch
    -Wodr

    # C Only Warnings
    $<$<COMPILE_LANGUAGE:C>:-Wjump-misses-init>

    # Disabled Warnings
    -Wno-expansion-to-defined # The NRFX library requires this
    -Wno-maybe-uninitialized
    -Wno-unused-function
    -Wno-unused-parameter  # Required for GCC 13+ with ETL/inplace_function templates
    -Wno-format
    
    # Compile prepass library for embedded systems
    -DPREPAS_EMBEDDED
    
    # For enabling debug output
    -DDEBUG_ENABLE
    -DDEBUG_LEVEL=4

    # Device model name
    -DDEVICE_MODEL_NAME=std::string\("LinkIt V4"\)

    # For setting firmware app version
    -DFW_APP_VERSION_STR=std::string\("V0.1"\)
    -DFW_APP_VERSION_STR_C="V0.1"

    # Git metadata used by the HTML test report header (TrackerTests main.cpp).
    "-DTEST_BUILD_GIT_SHORT=\"${GIT_COMMIT_SHORT}\""
    "-DTEST_BUILD_GIT_FULL=\"${GIT_COMMIT_FULL}\""
    "-DTEST_BUILD_GIT_BRANCH=\"${GIT_BRANCH}\""
    "-DTEST_BUILD_GIT_DESCRIBE=\"${GIT_DESCRIBE}\""
    "-DTEST_BUILD_TIMESTAMP=\"${BUILD_TIMESTAMP}\""

	# Packet protocol option for including CRC8
    -DARGOS_USE_CRC8
    
    # Workaround allcast encoding bug in CLS messages
    #-DWORKAROUND_ALLCAST_CONSTELLATION_STATUS_ENCODING_BUG
 
    # Increase the inplace_function max sizes for a 64 bit system
    -DINPLACE_FUNCTION_SIZE_SCHEDULER=1024
    -DINPLACE_FUNCTION_SIZE_TIMER=1536
    
    # Always output coloured text from GCC
    -fdiagnostics-color=always
)

# Test related includes
# NOTE: mocks and fakes must come BEFORE NRF_DIR paths to ensure test fakes
# take priority over hardware-specific fakes (e.g., fake_battery_mon.hpp)
target_include_directories(${PROJECT_NAME} PRIVATE
    "mocks"
    "fakes"
    "${ROOT_DIR}/libraries/etl-20.27.3/include"
    "${ROOT_DIR}/libraries/inplace_function"
    "${ROOT_DIR}/libraries/littlefs"
    "${ROOT_DIR}/libraries/prepass/previpass-PREPASS_V4.0"
    "${ROOT_DIR}/libraries/prepass/previpass_compat_layer"
    "${ROOT_DIR}/core/hardware"
    "${ROOT_DIR}/core/sm"
    "${ROOT_DIR}/core/ble_services"
    "${ROOT_DIR}/core/configuration"
    "${ROOT_DIR}/core/logging"
    "${ROOT_DIR}/core/filesystem"
    "${ROOT_DIR}/core/scheduling"
    "${ROOT_DIR}/core/services"
    "${ROOT_DIR}/ports/linux"
    "${ROOT_DIR}/core/protocol"
    "${ROOT_DIR}/core/util"
    "${NRF_DIR}/core/hardware"
    "${NRF_DIR}/core/hardware/m10qasync"
    "${NRF_DIR}/core/hardware/lora_rak3172"
    "${NRF_DIR}/core/hardware/ad5933"
    "${NRF_DIR}/core/hardware/ms58xx"
    "${NRF_DIR}/core/hardware/pressure_sensor"
    "${NRF_DIR}/core/hardware/cdt"
    "${NRF_DIR}/core/memmang"
)

target_sources(${PROJECT_NAME} PRIVATE
    "${ROOT_DIR}/libraries/littlefs/lfs_util.c"
    "${ROOT_DIR}/libraries/littlefs/lfs.c"
    "${ROOT_DIR}/libraries/prepass/previpass-PREPASS_V4.0/prepas.c"
    "${ROOT_DIR}/libraries/prepass/previpass-PREPASS_V4.0/utils.c"
    "${ROOT_DIR}/libraries/prepass/previpass_compat_layer/previpass.c"
    "${ROOT_DIR}/core/util/haversine.cpp"

    "${ROOT_DIR}/core/services/argos_rx_service.cpp"
    "${ROOT_DIR}/core/services/argos_tx_service.cpp"
    "${ROOT_DIR}/core/services/argos_packet_builder.cpp"
    "${ROOT_DIR}/core/services/lora_packet_builder.cpp"
    "${ROOT_DIR}/core/services/lora_tx_service.cpp"
    "${ROOT_DIR}/core/scheduling/lora_tx_scheduler.cpp"
    "${ROOT_DIR}/core/services/depth_pile.cpp"
    "${ROOT_DIR}/core/services/rate_limiter.cpp"
    "${ROOT_DIR}/core/services/hauled_mode_service.cpp"
    "${ROOT_DIR}/core/services/moored_mode_service.cpp"
    "${ROOT_DIR}/core/scheduling/argos_tx_scheduler.cpp"
    "${ROOT_DIR}/core/services/gps_service.cpp"
    "${ROOT_DIR}/core/services/service.cpp"
    "${ROOT_DIR}/core/services/service_cooldown.cpp"
    "${ROOT_DIR}/core/services/mortality_service.cpp"
    "${ROOT_DIR}/core/filesystem/ota_flash_file_updater.cpp"
    "${ROOT_DIR}/core/services/uwdetector_service.cpp"
    "${ROOT_DIR}/core/services/sws_analog_service.cpp"
    "${ROOT_DIR}/core/services/sws_analog_calibration.cpp"
    "${ROOT_DIR}/core/services/sws_analog_detection.cpp"
    "${ROOT_DIR}/core/hardware/reed.cpp"
    "${ROOT_DIR}/core/hardware/sensor.cpp"
    "${ROOT_DIR}/core/protocol/dte_commands.cpp"
    "${ROOT_DIR}/core/protocol/dte_params.cpp"
    "${ROOT_DIR}/core/protocol/dte_handler.cpp"
    "${ROOT_DIR}/core/configuration/calibration.cpp"
    "${ROOT_DIR}/core/sm/buzzm.cpp"
    "${ROOT_DIR}/core/sm/gentracker.cpp"
    "${ROOT_DIR}/core/sm/gentracker_config_state.cpp"
    "${ROOT_DIR}/core/sm/ledsm.cpp"
    "${ROOT_DIR}/core/logging/logger.cpp"
    "${LINUX_DIR}/interrupt_lock.cpp"
    "${NRF_DIR}/core/hardware/nrf_switch.cpp"
    # nrf_battery_mon.cpp excluded: depends on nrf_delay.h (Nordic SDK, not available in test builds)
    "${NRF_DIR}/core/hardware/m10qasync/m10qasync.cpp"
    "${NRF_DIR}/core/hardware/m10qasync/m10qasync_config.cpp"
    "${NRF_DIR}/core/hardware/m10qasync/ubx_comms.cpp"
    "${NRF_DIR}/core/hardware/nrf_uart_async.cpp"
    "${NRF_DIR}/core/hardware/lora_rak3172/lora_rak3172.cpp"
    "${NRF_DIR}/core/hardware/lora_rak3172/lora_rak3172_comm.cpp"
    "${NRF_DIR}/core/hardware/cdt/cdt.cpp"
    "src/timer_test.cpp"
    "src/filesystem_test.cpp"
    "src/gps_test.cpp"
    "src/gps_shortsurface_test.cpp"
    "src/scheduler_test.cpp"
    "src/sm_test.cpp"
    "src/encoder_test.cpp"
	"src/decoder_test.cpp"
	"src/config_store_test.cpp"
    "src/dte_handler_test.cpp"
    "src/bch_test.cpp"
    "src/crc8_test.cpp"
    "src/crc16_test.cpp"
    "src/crc32_test.cpp"
    "src/previpass_test.cpp"
    "src/sws_analog_test.cpp"
    "src/logger_test.cpp"
    "src/nrf_switch_test.cpp"
    # "src/nrf_battery_test.cpp"  # Excluded: depends on NrfBatteryMonitor (needs Nordic SDK nrf_delay.h)
    "src/ph_sensor_test.cpp"
    "src/als_sensor_test.cpp"
    "src/sea_temp_sensor_test.cpp"
    "src/cdt_sensor_test.cpp"
    "src/cdt_mock_test.cpp"
    "src/axl_sensor_test.cpp"
    "src/pressure_sensor_test.cpp"
    "src/calibration_test.cpp"
    "src/argos_tx_test.cpp"
    "src/argos_rx_test.cpp"
    "src/kim2_modulation_test.cpp"
    "src/lora_packet_builder_test.cpp"
    "src/packet_field_encoding_test.cpp"
    "src/exponential_backoff_test.cpp"
    "src/dive_mode_test.cpp"
    "src/m8_test.cpp"
    "src/binascii_test.cpp"
    "src/bitpack_test.cpp"
    "src/timeutils_test.cpp"
    "src/haversine_test.cpp"
    "src/moored_mode_test.cpp"
    "src/thermistor_test.cpp"
    "src/main.cpp"
    "fakes/memmang.cpp"
    "fakes/nrfx_rtc.c"
    "fakes/nrf_libuarte_async.cpp"
    "fakes/nrf_uarte.c"
    "mocks/mock_gpio.cpp"
    "mocks/mock_pmu.cpp"
    "mocks/mock_dfu.cpp"
)

# Vendor code: the host build is stricter than the firmware one, and these
# three files are imported as-is. Same waivers as ports/nrf52840/CMakeLists.txt
# plus the host-only warnings.
set_source_files_properties(${ROOT_DIR}/libraries/prepass/previpass-PREPASS_V4.0/prepas.c PROPERTIES COMPILE_FLAGS "-Wno-unused-parameter -Wno-switch-enum -Wno-switch -Wno-double-promotion -Wno-conversion -Wno-sign-conversion -Wno-float-conversion -Wno-cast-align -Wno-missing-field-initializers")
set_source_files_properties(${ROOT_DIR}/libraries/prepass/previpass-PREPASS_V4.0/utils.c PROPERTIES COMPILE_FLAGS "-Wno-unused-parameter -Wno-switch-enum -Wno-switch -Wno-double-promotion -Wno-conversion -Wno-sign-conversion -Wno-float-conversion -Wno-cast-align -Wno-missing-field-initializers")
set_source_files_properties(${ROOT_DIR}/libraries/prepass/previpass_compat_layer/previpass.c PROPERTIES COMPILE_FLAGS "-Wno-unused-parameter -Wno-switch-enum -Wno-switch -Wno-double-promotion -Wno-conversion -Wno-sign-conversion -Wno-float-conversion -Wno-cast-align -Wno-missing-field-initializers")

target_link_libraries(${PROJECT_NAME} PRIVATE CppUTest CppUTestExt ${CMAKE_THREAD_LIBS_INIT})

# ============================================================================
# TURTLE SIMULATION - Executable séparé avec rapport HTML
# ============================================================================
set(TURTLE_SIM_NAME "TurtleSimulation")
add_executable(${TURTLE_SIM_NAME})

set_property(TARGET ${TURTLE_SIM_NAME} PROPERTY CXX_STANDARD 20)
set_property(TARGET ${TURTLE_SIM_NAME} PROPERTY CMAKE_CXX_STANDARD_REQUIRED ON)

target_compile_definitions(${TURTLE_SIM_NAME} PRIVATE
    LFS_NO_DEBUG
    DEBUG_ENABLE
    CPPUTEST
    # Mirror TrackerTests: enables every sensor + selects the LoRa branch
    # in dte_handler.hpp (otherwise it falls through to kim2.hpp which
    # requires the kim2 include path that this target does not provide).
    ENABLE_PH_SENSOR=1
    ENABLE_SEA_TEMP_SENSOR=1
    ENABLE_PRESSURE_SENSOR=1
    ENABLE_ALS_SENSOR=1
    ENABLE_CDT_SENSOR=1
    ENABLE_AXL_SENSOR=1
    ENABLE_THERMISTOR_SENSOR=1
    ENABLE_MORTALITY_SENSOR=1
    ENABLE_SWS_LOG=1
    ENABLE_SWS_ANALOG=1
    LORA_RAK3172=1
    LORA_DCS_ENABLE=1
    LORA_POWER_OFF_UNDERWATER=0
)

target_compile_options(${TURTLE_SIM_NAME} PRIVATE
    -g3
    -Wall
    -Wextra
    -Werror
    -Wduplicated-cond
    -Wduplicated-branches
    -Wlogical-op
    -Wnull-dereference
    -Wshadow
    -Wpointer-arith
    -Wdangling-else
    -Wrestrict
    -Wdouble-promotion
    -Wvla
    -Wswitch-enum
    -Wswitch-default
    -Winvalid-pch
    -Wodr
    $<$<COMPILE_LANGUAGE:C>:-Wjump-misses-init>
    -Wno-expansion-to-defined
    -Wno-maybe-uninitialized
    -Wno-unused-function
    -Wno-format
    -DPREPAS_EMBEDDED
    -DDEBUG_ENABLE
    -DDEBUG_LEVEL=4
    -DDEVICE_MODEL_NAME=std::string\("SURFACEBOX"\)
    -DFW_APP_VERSION_STR=std::string\("V0.1"\)
    -DFW_APP_VERSION_STR_C="V0.1"
    "-DTEST_BUILD_GIT_SHORT=\"${GIT_COMMIT_SHORT}\""
    "-DTEST_BUILD_GIT_FULL=\"${GIT_COMMIT_FULL}\""
    "-DTEST_BUILD_GIT_BRANCH=\"${GIT_BRANCH}\""
    "-DTEST_BUILD_GIT_DESCRIBE=\"${GIT_DESCRIBE}\""
    "-DTEST_BUILD_TIMESTAMP=\"${BUILD_TIMESTAMP}\""
    -DARGOS_USE_CRC8
    -DINPLACE_FUNCTION_SIZE_SCHEDULER=1024
    -DINPLACE_FUNCTION_SIZE_TIMER=1536
    -fdiagnostics-color=always
)

target_include_directories(${TURTLE_SIM_NAME} PRIVATE
    "mocks"
    "fakes"
    "${ROOT_DIR}/libraries/etl-20.27.3/include"
    "${ROOT_DIR}/libraries/inplace_function"
    "${ROOT_DIR}/libraries/littlefs"
    "${ROOT_DIR}/libraries/prepass/previpass-PREPASS_V4.0"
    "${ROOT_DIR}/libraries/prepass/previpass_compat_layer"
    "${ROOT_DIR}/core/hardware"
    "${ROOT_DIR}/core/sm"
    "${ROOT_DIR}/core/ble_services"
    "${ROOT_DIR}/core/configuration"
    "${ROOT_DIR}/core/logging"
    "${ROOT_DIR}/core/filesystem"
    "${ROOT_DIR}/core/scheduling"
    "${ROOT_DIR}/core/services"
    "${ROOT_DIR}/ports/linux"
    "${ROOT_DIR}/core/protocol"
    "${ROOT_DIR}/core/util"
    "${NRF_DIR}/core/hardware"
    "${NRF_DIR}/core/hardware/m10qasync"
    "${NRF_DIR}/core/hardware/lora_rak3172"
    "${NRF_DIR}/core/hardware/ad5933"
    "${NRF_DIR}/core/hardware/ms58xx"
    "${NRF_DIR}/core/hardware/pressure_sensor"
    "${NRF_DIR}/core/hardware/cdt"
    "${NRF_DIR}/core/memmang"
)

target_sources(${TURTLE_SIM_NAME} PRIVATE
    "${ROOT_DIR}/libraries/littlefs/lfs_util.c"
    "${ROOT_DIR}/libraries/littlefs/lfs.c"
    "${ROOT_DIR}/libraries/prepass/previpass-PREPASS_V4.0/prepas.c"
    "${ROOT_DIR}/libraries/prepass/previpass-PREPASS_V4.0/utils.c"
    "${ROOT_DIR}/libraries/prepass/previpass_compat_layer/previpass.c"
    "${ROOT_DIR}/core/util/haversine.cpp"

    "${ROOT_DIR}/core/services/argos_rx_service.cpp"
    "${ROOT_DIR}/core/services/argos_tx_service.cpp"
    "${ROOT_DIR}/core/services/argos_packet_builder.cpp"
    "${ROOT_DIR}/core/services/depth_pile.cpp"
    "${ROOT_DIR}/core/services/rate_limiter.cpp"
    "${ROOT_DIR}/core/services/hauled_mode_service.cpp"
    "${ROOT_DIR}/core/services/moored_mode_service.cpp"
    "${ROOT_DIR}/core/scheduling/argos_tx_scheduler.cpp"
    "${ROOT_DIR}/core/services/gps_service.cpp"
    "${ROOT_DIR}/core/services/service.cpp"
    "${ROOT_DIR}/core/services/service_cooldown.cpp"
    "${ROOT_DIR}/core/filesystem/ota_flash_file_updater.cpp"
    "${ROOT_DIR}/core/services/uwdetector_service.cpp"
    "${ROOT_DIR}/core/services/sws_analog_service.cpp"
    "${ROOT_DIR}/core/services/sws_analog_calibration.cpp"
    "${ROOT_DIR}/core/services/sws_analog_detection.cpp"
    "${ROOT_DIR}/core/hardware/reed.cpp"
    "${ROOT_DIR}/core/hardware/sensor.cpp"
    "${ROOT_DIR}/core/protocol/dte_commands.cpp"
    "${ROOT_DIR}/core/protocol/dte_params.cpp"
    "${ROOT_DIR}/core/protocol/dte_handler.cpp"
    "${ROOT_DIR}/core/configuration/calibration.cpp"
    "${ROOT_DIR}/core/sm/buzzm.cpp"
    "${ROOT_DIR}/core/sm/gentracker.cpp"
    "${ROOT_DIR}/core/sm/gentracker_config_state.cpp"
    "${ROOT_DIR}/core/sm/ledsm.cpp"
    "${ROOT_DIR}/core/logging/logger.cpp"
    "${LINUX_DIR}/interrupt_lock.cpp"
    "${NRF_DIR}/core/hardware/nrf_switch.cpp"
    # nrf_battery_mon.cpp excluded: depends on nrf_delay.h (Nordic SDK, not available in test builds)
    "${NRF_DIR}/core/hardware/m10qasync/m10qasync.cpp"
    "${NRF_DIR}/core/hardware/m10qasync/m10qasync_config.cpp"
    "${NRF_DIR}/core/hardware/m10qasync/ubx_comms.cpp"
    "${NRF_DIR}/core/hardware/nrf_uart_async.cpp"
    "${NRF_DIR}/core/hardware/lora_rak3172/lora_rak3172.cpp"
    "${NRF_DIR}/core/hardware/lora_rak3172/lora_rak3172_comm.cpp"
    "${NRF_DIR}/core/hardware/cdt/cdt.cpp"
    "src/turtle_simulation_test.cpp"
    "src/turtle_simulation_1year.cpp"
    "src/turtle_simulation_shortsurface.cpp"
    "src/turtle_main.cpp"
    "fakes/memmang.cpp"
    "fakes/fake_bsp.cpp"
    "fakes/nrfx_rtc.c"
    "fakes/nrf_libuarte_async.cpp"
    "fakes/nrf_uarte.c"
    "mocks/mock_gpio.cpp"
    "mocks/mock_pmu.cpp"
    "mocks/mock_dfu.cpp"
)

set_source_files_properties(${ROOT_DIR}/libraries/prepass/previpass-PREPASS_V4.0/prepas.c ${ROOT_DIR}/libraries/prepass/previpass-PREPASS_V4.0/utils.c ${ROOT_DIR}/libraries/prepass/previpass_compat_layer/previpass.c
    PROPERTIES COMPILE_FLAGS "-Wno-unused-parameter -Wno-switch-enum -Wno-switch -Wno-double-promotion -Wno-conversion -Wno-sign-conversion -Wno-float-conversion -Wno-cast-align -Wno-missing-field-initializers")

target_link_libraries(${TURTLE_SIM_NAME} PRIVATE CppUTest CppUTestExt ${CMAKE_THREAD_LIBS_INIT})

# ============================================================================
# RSPB SIMULATION - Long-term bird mortality mission (external-wakeup profile)
# ============================================================================
# Standalone executable, mirrors the TurtleSimulation target but:
#   * defines EXTERNAL_WAKEUP (RSPB / TPL5111 board profile) so the
#     MortalityService duty-cycle adaptation + BOOT_COUNTER_MODULO param exist,
#   * compiles core/services/mortality_service.cpp (not linked by the turtle sim),
#   * drives src/rspb_simulation.cpp via src/rspb_main.cpp.
set(RSPB_SIM_NAME "RSPBSimulation")
add_executable(${RSPB_SIM_NAME})

set_property(TARGET ${RSPB_SIM_NAME} PROPERTY CXX_STANDARD 20)
set_property(TARGET ${RSPB_SIM_NAME} PROPERTY CMAKE_CXX_STANDARD_REQUIRED ON)

target_compile_definitions(${RSPB_SIM_NAME} PRIVATE
    LFS_NO_DEBUG
    DEBUG_ENABLE
    CPPUTEST
    EXTERNAL_WAKEUP
    ENABLE_PH_SENSOR=1
    ENABLE_SEA_TEMP_SENSOR=1
    ENABLE_PRESSURE_SENSOR=1
    ENABLE_ALS_SENSOR=1
    ENABLE_CDT_SENSOR=1
    ENABLE_AXL_SENSOR=1
    ENABLE_THERMISTOR_SENSOR=1
    ENABLE_MORTALITY_SENSOR=1
    ENABLE_SWS_LOG=1
    ENABLE_SWS_ANALOG=1
    LORA_RAK3172=1
    LORA_DCS_ENABLE=1
    LORA_POWER_OFF_UNDERWATER=0
)

target_compile_options(${RSPB_SIM_NAME} PRIVATE
    -g3
    -Wall
    -Wextra
    -Werror
    -Wduplicated-cond
    -Wduplicated-branches
    -Wlogical-op
    -Wnull-dereference
    -Wshadow
    -Wpointer-arith
    -Wdangling-else
    -Wrestrict
    -Wdouble-promotion
    -Wvla
    -Wswitch-enum
    -Wswitch-default
    -Winvalid-pch
    -Wodr
    $<$<COMPILE_LANGUAGE:C>:-Wjump-misses-init>
    -Wno-expansion-to-defined
    -Wno-maybe-uninitialized
    -Wno-unused-function
    -Wno-format
    -DPREPAS_EMBEDDED
    -DDEBUG_ENABLE
    -DDEBUG_LEVEL=4
    -DDEVICE_MODEL_NAME=std::string\("RSPB"\)
    -DFW_APP_VERSION_STR=std::string\("V0.1"\)
    -DFW_APP_VERSION_STR_C="V0.1"
    "-DTEST_BUILD_GIT_SHORT=\"${GIT_COMMIT_SHORT}\""
    "-DTEST_BUILD_GIT_FULL=\"${GIT_COMMIT_FULL}\""
    "-DTEST_BUILD_GIT_BRANCH=\"${GIT_BRANCH}\""
    "-DTEST_BUILD_GIT_DESCRIBE=\"${GIT_DESCRIBE}\""
    "-DTEST_BUILD_TIMESTAMP=\"${BUILD_TIMESTAMP}\""
    -DARGOS_USE_CRC8
    -DINPLACE_FUNCTION_SIZE_SCHEDULER=1024
    -DINPLACE_FUNCTION_SIZE_TIMER=1536
    -fdiagnostics-color=always
)

target_include_directories(${RSPB_SIM_NAME} PRIVATE
    "mocks"
    "fakes"
    "${ROOT_DIR}/libraries/etl-20.27.3/include"
    "${ROOT_DIR}/libraries/inplace_function"
    "${ROOT_DIR}/libraries/littlefs"
    "${ROOT_DIR}/libraries/prepass/previpass-PREPASS_V4.0"
    "${ROOT_DIR}/libraries/prepass/previpass_compat_layer"
    "${ROOT_DIR}/core/hardware"
    "${ROOT_DIR}/core/sm"
    "${ROOT_DIR}/core/ble_services"
    "${ROOT_DIR}/core/configuration"
    "${ROOT_DIR}/core/logging"
    "${ROOT_DIR}/core/filesystem"
    "${ROOT_DIR}/core/scheduling"
    "${ROOT_DIR}/core/services"
    "${ROOT_DIR}/ports/linux"
    "${ROOT_DIR}/core/protocol"
    "${ROOT_DIR}/core/util"
    "${NRF_DIR}/core/hardware"
    "${NRF_DIR}/core/hardware/m10qasync"
    "${NRF_DIR}/core/hardware/lora_rak3172"
    "${NRF_DIR}/core/hardware/ad5933"
    "${NRF_DIR}/core/hardware/ms58xx"
    "${NRF_DIR}/core/hardware/pressure_sensor"
    "${NRF_DIR}/core/hardware/cdt"
    "${NRF_DIR}/core/memmang"
)

target_sources(${RSPB_SIM_NAME} PRIVATE
    "${ROOT_DIR}/libraries/littlefs/lfs_util.c"
    "${ROOT_DIR}/libraries/littlefs/lfs.c"
    "${ROOT_DIR}/libraries/prepass/previpass-PREPASS_V4.0/prepas.c"
    "${ROOT_DIR}/libraries/prepass/previpass-PREPASS_V4.0/utils.c"
    "${ROOT_DIR}/libraries/prepass/previpass_compat_layer/previpass.c"
    "${ROOT_DIR}/core/util/haversine.cpp"

    "${ROOT_DIR}/core/services/argos_rx_service.cpp"
    "${ROOT_DIR}/core/services/argos_tx_service.cpp"
    "${ROOT_DIR}/core/services/argos_packet_builder.cpp"
    "${ROOT_DIR}/core/services/depth_pile.cpp"
    "${ROOT_DIR}/core/services/rate_limiter.cpp"
    "${ROOT_DIR}/core/services/hauled_mode_service.cpp"
    "${ROOT_DIR}/core/services/moored_mode_service.cpp"
    "${ROOT_DIR}/core/services/mortality_service.cpp"
    "${ROOT_DIR}/core/scheduling/argos_tx_scheduler.cpp"
    "${ROOT_DIR}/core/services/gps_service.cpp"
    "${ROOT_DIR}/core/services/service.cpp"
    "${ROOT_DIR}/core/services/service_cooldown.cpp"
    "${ROOT_DIR}/core/filesystem/ota_flash_file_updater.cpp"
    "${ROOT_DIR}/core/services/uwdetector_service.cpp"
    "${ROOT_DIR}/core/services/sws_analog_service.cpp"
    "${ROOT_DIR}/core/services/sws_analog_calibration.cpp"
    "${ROOT_DIR}/core/services/sws_analog_detection.cpp"
    "${ROOT_DIR}/core/hardware/reed.cpp"
    "${ROOT_DIR}/core/hardware/sensor.cpp"
    "${ROOT_DIR}/core/protocol/dte_commands.cpp"
    "${ROOT_DIR}/core/protocol/dte_params.cpp"
    "${ROOT_DIR}/core/protocol/dte_handler.cpp"
    "${ROOT_DIR}/core/configuration/calibration.cpp"
    "${ROOT_DIR}/core/sm/buzzm.cpp"
    "${ROOT_DIR}/core/sm/gentracker.cpp"
    "${ROOT_DIR}/core/sm/gentracker_config_state.cpp"
    "${ROOT_DIR}/core/sm/ledsm.cpp"
    "${ROOT_DIR}/core/logging/logger.cpp"
    "${LINUX_DIR}/interrupt_lock.cpp"
    "${NRF_DIR}/core/hardware/nrf_switch.cpp"
    "${NRF_DIR}/core/hardware/m10qasync/m10qasync.cpp"
    "${NRF_DIR}/core/hardware/m10qasync/m10qasync_config.cpp"
    "${NRF_DIR}/core/hardware/m10qasync/ubx_comms.cpp"
    "${NRF_DIR}/core/hardware/nrf_uart_async.cpp"
    "${NRF_DIR}/core/hardware/lora_rak3172/lora_rak3172.cpp"
    "${NRF_DIR}/core/hardware/lora_rak3172/lora_rak3172_comm.cpp"
    "${NRF_DIR}/core/hardware/cdt/cdt.cpp"
    "src/rspb_simulation.cpp"
    "src/rspb_main.cpp"
    "fakes/memmang.cpp"
    "fakes/fake_bsp.cpp"
    "fakes/nrfx_rtc.c"
    "fakes/nrf_libuarte_async.cpp"
    "fakes/nrf_uarte.c"
    "mocks/mock_gpio.cpp"
    "mocks/mock_pmu.cpp"
    "mocks/mock_dfu.cpp"
)

set_source_files_properties(${ROOT_DIR}/libraries/prepass/previpass-PREPASS_V4.0/prepas.c ${ROOT_DIR}/libraries/prepass/previpass-PREPASS_V4.0/utils.c ${ROOT_DIR}/libraries/prepass/previpass_compat_layer/previpass.c
    PROPERTIES COMPILE_FLAGS "-Wno-unused-parameter -Wno-switch-enum -Wno-switch -Wno-double-promotion -Wno-conversion -Wno-sign-conversion -Wno-float-conversion -Wno-cast-align -Wno-missing-field-initializers")

target_link_libraries(${RSPB_SIM_NAME} PRIVATE CppUTest CppUTestExt ${CMAKE_THREAD_LIBS_INIT})
