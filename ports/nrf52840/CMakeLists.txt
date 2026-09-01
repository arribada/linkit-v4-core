cmake_minimum_required(VERSION 3.13)

project(LinkIt
    VERSION 0.1
    LANGUAGES CXX C ASM
)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Generate compile_commands.json for IDE IntelliSense (clangd, VSCode C/C++)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(SDK_ROOT      drivers/nRF5_SDK_17.0.2)
set(PROJ_DIR      .)
set(CORE_DIR      ../../core)
set(LIB_DIR       ../../libraries)

# ─────────────────────────────────────────────────────────────────────────────
# Boolean build flags — normalisation stricte, une seule fois, pour toutes.
#
# Ces flags finissent dans le préprocesseur C sous la forme `#if FLAG`. Le
# préprocesseur remplace un identifiant inconnu par 0 : `-DFLAG=ON` compile donc
# SILENCIEUSEMENT le code gardé en dehors du binaire, pendant que CMake affiche
# fièrement "FLAG=ON". C'est exactement comme ça que le fast-path BBR du GNSS
# s'est retrouvé désactivé sur les builds SMD et LoRa (constaté 2026-08).
# Le `if()` de CMake a le piège symétrique : `if(FLAG EQUAL 1)` est faux pour "ON".
#
# Règle : on accepte toutes les orthographes qu'un script de build peut employer,
# on normalise en 1/0, et on ÉCHOUE brutalement sur ce qu'on ne comprend pas
# plutôt que de deviner.
function(normalise_bool_flag _name)
    if(NOT DEFINED ${_name})
        return()   # jamais posé — certains flags ne sont transmis que s'ils le sont
    endif()
    set(_val "${${_name}}")
    if(_val STREQUAL "")
        return()   # posé vide = équivalent à non posé (cf. RSPB_VSENSORS_FLOOR)
    endif()
    string(TOUPPER "${_val}" _up)
    if(_up MATCHES "^(1|ON|TRUE|YES|Y)$")
        set(${_name} 1 PARENT_SCOPE)
    elseif(_up MATCHES "^(0|OFF|FALSE|NO|N)$" OR _up MATCHES "NOTFOUND$")
        set(${_name} 0 PARENT_SCOPE)
    else()
        message(FATAL_ERROR
            "Build flag ${_name}='${_val}' n'est pas un booléen. Utiliser "
            "1/0, ON/OFF, TRUE/FALSE ou YES/NO — jamais un mot libre : il "
            "arriverait tel quel au préprocesseur C et y vaudrait 0.")
    endif()
endfunction()

# Liste unique des flags booléens du projet. Tout flag consommé par `#if` ou par
# un `if(... EQUAL 1)` DOIT figurer ici. Les flags numériques (DEBUG_LEVEL,
# WAKEUP_PERIOD, LORA_TX_ERROR_SUSPEND_S) et les chaînes (BOARD,
# BATTERY_CHEMISTRY, BATTERY_MONITOR_TYPE, EXT_GPIO5_DEVICE) n'y sont pas.
set(BOOL_BUILD_FLAGS
    ENABLE_CAM_SENSOR ENABLE_BUZZER ENABLE_PH_SENSOR ENABLE_SEA_TEMP_SENSOR
    ENABLE_PRESSURE_SENSOR ENABLE_ALS_SENSOR ENABLE_CDT_SENSOR ENABLE_AXL_SENSOR
    ENABLE_THERMISTOR_SENSOR ENABLE_MORTALITY_SENSOR ENABLE_SWS_LOG ENABLE_SWS_ANALOG
    GNSS_HAS_BACKUP_BATTERY GPS_FAKE_POSITION STC3117_MIXED_MODE
    ARGOS_SMD LORA_RAK3172 LORA_DCS_ENABLE LORA_POWER_OFF_UNDERWATER
    SMDSAT_USE_SAFE_TIMINGS SMDSAT_AUTOFALLBACK
    BENCH_TEST BENCH_FORCE_LPM_NONE SMD_FLASH_HOLD SMD_UART KIM2_BLIND_WKUP_RELEASE
    CAM_ENABLE BUZZER_ENABLE
    METRIC_LATENCY_LOG_ENABLE VALIDATION_LOG_ENABLE RSPB_VSENSORS_FLOOR)

# Idempotent : appelé une première fois dès que les valeurs utilisateur sont
# connues, puis une seconde fois juste avant target_compile_definitions pour
# rattraper les flags posés entre-temps par la section spécifique à la carte.
macro(normalise_bool_flags)
    foreach(_bf ${BOOL_BUILD_FLAGS})
        normalise_bool_flag(${_bf})
    endforeach()
endmacro()

set(BOARD "LINKIT" CACHE STRING "Board to compile for")
option(ARGOS_SMD "Use SMD satellite module" OFF)
option(LORA_RAK3172 "Use RAK3172-SiP LoRa module instead of satellite" OFF)
# Bench-test harness: '%'-prefixed USB-CDC console (%CFG/%OP/%GPS/...) + runtime
# GPS fix injection for autonomous hardware-in-the-loop validation. Debug builds
# only; entirely compiled out of production firmware. See core/interface/bench_console.*
option(BENCH_TEST "Enable bench-test serial console + GPS injection hook" OFF)
# Bench diagnostic (RSPB/SMD): force the SMD LPM bitmap to NONE (always-on) so a
# satellite-TX SPI failure can be attributed to STOP-mode wake vs the SPI itself.
option(BENCH_FORCE_LPM_NONE "Bench: force SMD LPM=NONE (isolate SPI from STOP-mode wake)" OFF)
# KIM2 + profil MAC BLIND: une fois AT+TX acquitté, le module cadence lui-même
# ses retransmissions et n'attend plus aucune commande AT jusqu'au +TX= final.
# Maintenir SAT_EXTWAKEUP haute le garde alors éveillé pour rien — mesuré au banc,
# 122,6 s d'éveil pour trois émissions espacées de 60 s. Ce flag relâche la pin
# pendant la salve. Défaut OFF : comportement inchangé.
option(KIM2_BLIND_WKUP_RELEASE "KIM2: release SAT_EXTWAKEUP during a module-paced BLIND burst" OFF)

# SMD timing safety knobs.
#
#   SMDSAT_USE_SAFE_TIMINGS  — force the v4.1.4 conservative SAFE baseline at
#                              compile time. Disables the autofallback path
#                              since there is no FAST mode to fall back from.
#                              ON/OFF or 1/0 accepted, default OFF (FAST).
#
#   SMDSAT_AUTOFALLBACK      — enable runtime auto-fallback to SAFE timings
#                              after SMD_MAX_CONSECUTIVE_ERRORS SPI errors,
#                              with exponential-window retest of FAST.
#                              Only meaningful when SMDSAT_USE_SAFE_TIMINGS=0.
#                              ON/OFF or 1/0 accepted, default ON for SMD
#                              builds (best balance of latency + robustness).
if(NOT DEFINED SMDSAT_USE_SAFE_TIMINGS)
    set(SMDSAT_USE_SAFE_TIMINGS OFF)
endif()
if(NOT DEFINED SMDSAT_AUTOFALLBACK)
    set(SMDSAT_AUTOFALLBACK ON)
endif()
# Normalize ON/OFF/TRUE/FALSE/1/0 to a plain integer for the C preprocessor.
if(SMDSAT_USE_SAFE_TIMINGS)
    set(SMDSAT_USE_SAFE_TIMINGS_VAL 1)
else()
    set(SMDSAT_USE_SAFE_TIMINGS_VAL 0)
endif()
if(SMDSAT_AUTOFALLBACK AND NOT SMDSAT_USE_SAFE_TIMINGS_VAL)
    set(SMDSAT_AUTOFALLBACK_VAL 1)
else()
    set(SMDSAT_AUTOFALLBACK_VAL 0)
endif()
if(SMDSAT_USE_SAFE_TIMINGS_VAL AND SMDSAT_AUTOFALLBACK)
    message(STATUS "SMDSAT_USE_SAFE_TIMINGS=ON forces SAFE statically — disabling SMDSAT_AUTOFALLBACK (nothing to fall back from)")
endif()
message(STATUS "SMD timing profile: USE_SAFE_TIMINGS=${SMDSAT_USE_SAFE_TIMINGS_VAL} AUTOFALLBACK=${SMDSAT_AUTOFALLBACK_VAL}")

# LoRa-specific tuning flags (only meaningful when LORA_RAK3172=ON)
#
# LORA_DCS_ENABLE: Duty Cycle Setting — when ON, the RAK3172 enforces EU868
#   legal airtime limits (1 %/0.1 %/10 % per sub-band). Default ON.
#   OFF is for RF bench testing only — **illegal in EU production deployment**.
option(LORA_DCS_ENABLE "Enforce LoRaWAN duty-cycle on RAK3172 (ETSI EN 300 220)" ON)

# LORA_TX_ERROR_SUSPEND_S: how long LoRaTxService stops transmitting after
# DEVICE_ERROR_MAX_CONSECUTIVE consecutive device errors, before letting one
# probe TX through. 0 disables the suspension entirely, leaving only the capped
# exponential backoff (max DEVICE_ERROR_BACKOFF_MAX_MS = 10 min between
# attempts) — that is the bench/field-test setting, where waiting an hour to
# retry makes iteration impossible. Production keeps 3600.
if(NOT DEFINED LORA_TX_ERROR_SUSPEND_S)
    set(LORA_TX_ERROR_SUSPEND_S 3600)
endif()
message(STATUS "LoRa TX error suspend: ${LORA_TX_ERROR_SUSPEND_S} s (0 = no suspension, backoff only)")
#
# LORA_POWER_OFF_UNDERWATER: when ON, the LoRa service cuts the module's power
#   rail (SAT_PWR_EN → 0 V) on every dive event, going from ~1.7 µA (Stop2
#   standby) down to 0 µA. On the next surfacing the full configure path runs,
#   adding ~2 s to time-to-first-TX. Default OFF — standby is fast-wake (~10 ms)
#   which matters for short surfacings (sea turtle, diving bird).
option(LORA_POWER_OFF_UNDERWATER "Cut LoRa module power during underwater periods" OFF)

# Configure GPIO5 device (default: CAM)
if(NOT DEFINED EXT_GPIO5_DEVICE)
    set(EXT_GPIO5_DEVICE "CAM")
endif()

# Optional sensor enables (default: all disabled except AXL)
# These can be overridden via cmake -DENABLE_PH_SENSOR=1 etc.
if(NOT DEFINED ENABLE_PH_SENSOR)
    set(ENABLE_PH_SENSOR 0)
endif()
if(NOT DEFINED ENABLE_SEA_TEMP_SENSOR)
    set(ENABLE_SEA_TEMP_SENSOR 0)
endif()
if(NOT DEFINED ENABLE_PRESSURE_SENSOR)
    set(ENABLE_PRESSURE_SENSOR 0)
endif()
if(NOT DEFINED ENABLE_ALS_SENSOR)
    set(ENABLE_ALS_SENSOR 0)
endif()
if(NOT DEFINED ENABLE_CDT_SENSOR)
    set(ENABLE_CDT_SENSOR 0)
endif()
if(NOT DEFINED ENABLE_AXL_SENSOR)
    set(ENABLE_AXL_SENSOR 1)
endif()
if(NOT DEFINED ENABLE_THERMISTOR_SENSOR)
    set(ENABLE_THERMISTOR_SENSOR 0)
endif()
if(NOT DEFINED ENABLE_MORTALITY_SENSOR)
    set(ENABLE_MORTALITY_SENSOR 0)
endif()
if(NOT DEFINED ENABLE_SWS_LOG)
    set(ENABLE_SWS_LOG 0)
endif()
if(NOT DEFINED GNSS_HAS_BACKUP_BATTERY)
    set(GNSS_HAS_BACKUP_BATTERY 0)
endif()

# Passage 1 : tout ce que l'utilisateur a pu passer en -D est maintenant connu.
# Doit précéder les gardes `if(A AND NOT B)` et les tests `if(X EQUAL 1)`, qui
# donnent tous un résultat faux sur une valeur "ON".
normalise_bool_flags()

# Guard: THERMISTOR and SEA_TEMP share the same depth pile slot in Argos TX.
# Enabling both simultaneously would cause data corruption (last writer wins).
if(ENABLE_THERMISTOR_SENSOR AND ENABLE_SEA_TEMP_SENSOR)
    message(FATAL_ERROR "ENABLE_THERMISTOR_SENSOR and ENABLE_SEA_TEMP_SENSOR cannot both be enabled. "
        "They share the same Argos TX depth pile slot (m_sea_temp_depth_pile).")
endif()

# Guard: Mortality detection requires AXL, thermistor, and GNSS sensors.
# Without these, the algorithm runs with partial data only (biased toward ALIVE).
if(ENABLE_MORTALITY_SENSOR AND NOT ENABLE_AXL_SENSOR)
    message(WARNING "ENABLE_MORTALITY_SENSOR=ON but ENABLE_AXL_SENSOR=OFF. "
        "Mortality detection requires accelerometer data for activity monitoring.")
endif()
if(ENABLE_MORTALITY_SENSOR AND NOT ENABLE_THERMISTOR_SENSOR)
    message(WARNING "ENABLE_MORTALITY_SENSOR=ON but ENABLE_THERMISTOR_SENSOR=OFF. "
        "Mortality detection requires thermistor data for body temperature monitoring.")
endif()

# Battery monitor type: ANALOG, FAKE (always 4.1V), STC3117 (I2C fuel gauge)
# Default: STC3117 for RSPB board, ANALOG for others
if(NOT DEFINED BATTERY_MONITOR_TYPE)
    if(${BOARD} MATCHES RSPB)
        set(BATTERY_MONITOR_TYPE "STC3117")
    else()
        set(BATTERY_MONITOR_TYPE "ANALOG")
    endif()
endif()

# Validate battery monitor type
if(NOT ${BATTERY_MONITOR_TYPE} MATCHES "ANALOG|FAKE|STC3117")
    message(FATAL_ERROR "BATTERY_MONITOR_TYPE must be one of {ANALOG, FAKE, STC3117}")
endif()

# Battery chemistry (ANALOG monitor only): selects discharge LUT + voltage range.
# Allowed: BATT_CHEM_S18650_2600, BATT_CHEM_CGR18650_2250, BATT_CHEM_NCR18650_3100_3400,
#          BATT_CHEM_LS17500_2P (2x Saft LS17500 Li-SOCl2 in parallel).
#          BATT_CHEM_ALKALINE_3S2P (3x AA Alkaline in series, 2x in parallel).
if(NOT DEFINED BATTERY_CHEMISTRY)
    set(BATTERY_CHEMISTRY "BATT_CHEM_NCR18650_3100_3400")
endif()
if(NOT ${BATTERY_CHEMISTRY} MATCHES "^BATT_CHEM_(S18650_2600|CGR18650_2250|NCR18650_3100_3400|LS17500_2P|ALKALINE_3S2P)$")
    message(FATAL_ERROR "BATTERY_CHEMISTRY must be one of {BATT_CHEM_S18650_2600, BATT_CHEM_CGR18650_2250, BATT_CHEM_NCR18650_3100_3400, BATT_CHEM_LS17500_2P, BATT_CHEM_ALKALINE_3S2P}")
endif()

# STC3117 fuel-gauge SOC strategy (STC3117 monitor only — i.e. RSPB by default):
#   STC3117_MIXED_MODE=0 -> VLTG_MODE  : voltage-only (OCV table), lowest power (~3uA standby) [default]
#   STC3117_MIXED_MODE=1 -> MIXED_MODE : coulomb counter + OCV; gauge runs continuously on vBAT (~70uA),
#                                        robust to solar charge/discharge swings (recommended for solar).
if(NOT DEFINED STC3117_MIXED_MODE)
    set(STC3117_MIXED_MODE 0)
endif()
if(NOT "${STC3117_MIXED_MODE}" MATCHES "^[01]$")
    message(FATAL_ERROR "STC3117_MIXED_MODE must be 0 (voltage-only) or 1 (mixed coulomb+OCV)")
endif()

message(STATUS "Battery monitor type: ${BATTERY_MONITOR_TYPE}")
message(STATUS "STC3117 mixed mode: ${STC3117_MIXED_MODE}")
message(STATUS "Battery chemistry: ${BATTERY_CHEMISTRY}")
message(STATUS "Sensors: AXL=${ENABLE_AXL_SENSOR} PRESSURE=${ENABLE_PRESSURE_SENSOR} THERMISTOR=${ENABLE_THERMISTOR_SENSOR} SEA_TEMP=${ENABLE_SEA_TEMP_SENSOR} ALS=${ENABLE_ALS_SENSOR} CDT=${ENABLE_CDT_SENSOR} PH=${ENABLE_PH_SENSOR} MORTALITY=${ENABLE_MORTALITY_SENSOR}")
message(STATUS "GNSS: BBR=${GNSS_HAS_BACKUP_BATTERY} SWS_LOG=${ENABLE_SWS_LOG}")


# Fake GPS position for indoor testing (simulates fix at Saint-Paul, Reunion)
if(NOT DEFINED GPS_FAKE_POSITION)
    set(GPS_FAKE_POSITION 0)
endif()

if(${BOARD} MATCHES RSPB)
    set(TARGET_NAME ${PROJECT_NAME}_RSPB_board)
    set(DEVICE_MODEL "RSPB")
    set(BOARD_DIR "${PROJ_DIR}/bsp/rspbtracker_v1.0")
    set(BOOTLOADER "bootloader/secure_bootloader/rspbtracker_v1.0/armgcc/_build/rspb_bootloader_v1_linkit_merged.hex")
    # RSPB defaults (user can override via -D)
    if(NOT DEFINED ENABLE_THERMISTOR_SENSOR)
        set(ENABLE_THERMISTOR_SENSOR 1)
    endif()
    if(NOT DEFINED ENABLE_MORTALITY_SENSOR)
        set(ENABLE_MORTALITY_SENSOR 1)
    endif()
    # Hardware constraint: RSPB has no SWS ADC channel
    set(ENABLE_SWS_ANALOG 0)
    # TPL5111 external wakeup timer support (hardware, not overridable)
    set(EXTERNAL_WAKEUP 1)
    # TPL5111 configured for 1H45 = 6300 seconds wakeup period (physical config with resistor)
    if(NOT DEFINED WAKEUP_PERIOD)
        set(WAKEUP_PERIOD 6300)
    endif()
    if(ARGOS_SMD)
        set(ARGOS_SMD_ENABLED 1)
    else()
        set(ARGOS_SMD_ENABLED 0)
    endif()
    set(LORA_RAK3172_ENABLED 0)
else()
    set(TARGET_NAME ${PROJECT_NAME}_board)
    set(DEVICE_MODEL "LinkIt V4")
    set(BOARD_DIR "${PROJ_DIR}/bsp/linkitv4_v1.0")
    if(ARGOS_SMD)
        set(ARGOS_SMD_ENABLED 1)
    else()
        set(ARGOS_SMD_ENABLED 0)
    endif()
    if(LORA_RAK3172)
        set(LORA_RAK3172_ENABLED 1)
    else()
        set(LORA_RAK3172_ENABLED 0)
    endif()
    if(LORA_DCS_ENABLE)
        set(LORA_DCS_ENABLE_VAL 1)
    else()
        set(LORA_DCS_ENABLE_VAL 0)
    endif()
    if(LORA_POWER_OFF_UNDERWATER)
        set(LORA_POWER_OFF_UNDERWATER_VAL 1)
    else()
        set(LORA_POWER_OFF_UNDERWATER_VAL 0)
    endif()
    set(BOOTLOADER "bootloader/secure_bootloader/linkitv4_v1.0/armgcc/_build/cls_bootloader_v1_linkit_merged.hex")
    if(NOT DEFINED ENABLE_SWS_ANALOG)
        set(ENABLE_SWS_ANALOG 1)
    endif()
endif()

if(NOT DEFINED DEBUG_LEVEL)
    set(DEBUG_LEVEL 3)
endif()


execute_process(
        COMMAND git describe --tags --dirty --long
        WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
        OUTPUT_VARIABLE FW_APP_VERSION
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
        )

# Fallback when no tag is reachable from HEAD (`git describe` fails silently
# above and returns empty). Synthesize a git-describe-style version using
# v0.0.0 as a virtual tag, total commit count, and short SHA.
if(NOT FW_APP_VERSION)
    execute_process(
        COMMAND git rev-parse --short HEAD
        WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
        OUTPUT_VARIABLE GIT_SHA
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
        )
    execute_process(
        COMMAND git rev-list --count HEAD
        WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
        OUTPUT_VARIABLE GIT_COMMITS
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
        )
    execute_process(
        COMMAND git status --porcelain
        WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
        OUTPUT_VARIABLE GIT_DIRTY
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
        )
    set(FW_APP_VERSION "v0.0.0-${GIT_COMMITS}-g${GIT_SHA}")
    if(GIT_DIRTY)
        set(FW_APP_VERSION "${FW_APP_VERSION}-dirty")
    endif()
endif()
message(STATUS "FW_APP_VERSION: ${FW_APP_VERSION}")

add_executable(${TARGET_NAME})
set(CMAKE_EXECUTABLE_SUFFIX ".elf")

target_include_directories(${TARGET_NAME} SYSTEM PRIVATE
    ${SDK_ROOT}/components/ble/ble_services/ble_ancs_c
    ${SDK_ROOT}/components/ble/ble_services/ble_ias_c
    ${SDK_ROOT}/components/ble/ble_services/ble_gls
    ${SDK_ROOT}/components/libraries/bootloader/ble_dfu
    ${SDK_ROOT}/components/ble/ble_advertising
    ${SDK_ROOT}/components/ble/ble_services/ble_bas_c
    ${SDK_ROOT}/components/ble/ble_services/ble_hrs_c
    ${SDK_ROOT}/components/ble/ble_dtm
    ${SDK_ROOT}/components/ble/ble_services/ble_rscs_c
    ${SDK_ROOT}/components/ble/common
    ${SDK_ROOT}/components/ble/ble_services/ble_lls
    ${SDK_ROOT}/components/ble/ble_services/ble_bas
    ${SDK_ROOT}/components/ble/ble_services/ble_ans_c
    ${SDK_ROOT}/components/ble/ble_services/ble_nus_c
    ${SDK_ROOT}/components/ble/ble_services/ble_ias
    ${SDK_ROOT}/components/nfc/ndef/conn_hand_parser/ble_oob_advdata_parser
    ${SDK_ROOT}/components/ble/ble_services/ble_dfu
    ${SDK_ROOT}/components/ble/ble_services/ble_lbs
    ${SDK_ROOT}/components/ble/ble_services/ble_hts
    ${SDK_ROOT}/components/ble/ble_services/ble_cts_c
    ${SDK_ROOT}/components/ble/ble_services/ble_cscs
    ${SDK_ROOT}/components/ble/ble_link_ctx_manager
    ${SDK_ROOT}/components/ble/ble_services/ble_nus
    ${SDK_ROOT}/components/ble/ble_services/ble_hids
    ${SDK_ROOT}/components/nfc/ndef/connection_handover/ble_oob_advdata
    ${SDK_ROOT}/components/nfc/ndef/connection_handover/ble_pair_msg
    ${SDK_ROOT}/components/ble/peer_manager
    ${SDK_ROOT}/components/ble/ble_services/ble_tps
    ${SDK_ROOT}/components/ble/ble_services/ble_dis
    ${SDK_ROOT}/components/ble/nrf_ble_gatt
    ${SDK_ROOT}/components/ble/nrf_ble_qwr
    ${SDK_ROOT}/components/ble/ble_services/ble_lbs_c
    ${SDK_ROOT}/components/nfc/ndef/connection_handover/ble_pair_lib
    ${SDK_ROOT}/components/ble/ble_racp
    ${SDK_ROOT}/components/ble/ble_services/ble_hrs
    ${SDK_ROOT}/components/ble/ble_services/ble_rscs
    ${SDK_ROOT}/components/libraries/timer
    ${SDK_ROOT}/components/libraries/sortlist
    ${SDK_ROOT}/components/libraries/atomic
    ${SDK_ROOT}/components/libraries/pwr_mgmt
    ${SDK_ROOT}/components/libraries/atomic_flags
    ${SDK_ROOT}/components/libraries/atomic_fifo
    ${SDK_ROOT}/components/libraries/strerror
    ${SDK_ROOT}/components/libraries/mutex
    ${SDK_ROOT}/external/fprintf
    ${SDK_ROOT}/external/utf_converter
    ${SDK_ROOT}/components/libraries/memobj
    ${SDK_ROOT}/components/libraries/balloc
    ${SDK_ROOT}/components/libraries/ringbuf
    ${SDK_ROOT}/components/toolchain/cmsis/include
    ${SDK_ROOT}/components/libraries/delay
    ${SDK_ROOT}/components/libraries/util
    ${SDK_ROOT}/components/libraries/log
    ${SDK_ROOT}/components/libraries/log/src
    ${SDK_ROOT}/components/libraries/experimental_section_vars
    ${SDK_ROOT}/components/libraries/crc32
    ${SDK_ROOT}/components/libraries/crc16
	${SDK_ROOT}/components/libraries/queue
    ${SDK_ROOT}/components/libraries/libuarte
    ${SDK_ROOT}/components/softdevice/common
    ${SDK_ROOT}/components/softdevice/s140/headers
    ${SDK_ROOT}/components/softdevice/s140/headers/nrf52
    ${SDK_ROOT}/modules/nrfx
    ${SDK_ROOT}/modules/nrfx/hal
    ${SDK_ROOT}/modules/nrfx/mdk
    ${SDK_ROOT}/modules/nrfx/drivers/include
    ${SDK_ROOT}/integration/nrfx
    ${SDK_ROOT}/integration/nrfx/legacy
    ${SDK_ROOT}/components/libraries/usbd
    ${SDK_ROOT}/components/libraries/usbd/class/cdc
    ${SDK_ROOT}/components/libraries/usbd/class/cdc/acm
    ${PROJ_DIR}
    ${PROJ_DIR}/drivers/nrf_log_redirect
    ${BOARD_DIR}
    ${LIB_DIR}/etl-20.27.3/include
    ${LIB_DIR}/inplace_function
    ${LIB_DIR}/littlefs
    ${LIB_DIR}/CmBacktrace/cm_backtrace
    ${LIB_DIR}/prepass/prepass_package_code-PREPAS_V3.4
    ${CORE_DIR}/ble_services
    ${CORE_DIR}/hardware
    ${CORE_DIR}/services
    ${CORE_DIR}/configuration
    ${CORE_DIR}/filesystem
    ${CORE_DIR}/interface
    ${CORE_DIR}/protocol
    ${CORE_DIR}/util
    ${CORE_DIR}/logging
    ${CORE_DIR}/sm
    ${CORE_DIR}/scheduling
    ${PROJ_DIR}/core/hardware
    ${PROJ_DIR}/core/interface
    ${PROJ_DIR}/core/memmang
    ${PROJ_DIR}/core/filesystem
    ${PROJ_DIR}/core/scheduling
    ${PROJ_DIR}/core/hardware/kim2
    $<$<OR:$<BOOL:${ENABLE_PRESSURE_SENSOR}>,$<BOOL:${ENABLE_CDT_SENSOR}>>:${CMAKE_CURRENT_SOURCE_DIR}/core/hardware/pressure_sensor>
    $<$<OR:$<BOOL:${ENABLE_PRESSURE_SENSOR}>,$<BOOL:${ENABLE_CDT_SENSOR}>>:${CMAKE_CURRENT_SOURCE_DIR}/core/hardware/ms58xx>
    $<$<OR:$<BOOL:${ENABLE_PRESSURE_SENSOR}>,$<BOOL:${ENABLE_CDT_SENSOR}>>:${CMAKE_CURRENT_SOURCE_DIR}/core/hardware/bar100>
    $<$<BOOL:${ENABLE_CDT_SENSOR}>:${CMAKE_CURRENT_SOURCE_DIR}/core/hardware/ad5933>
    $<$<BOOL:${ENABLE_ALS_SENSOR}>:${CMAKE_CURRENT_SOURCE_DIR}/core/hardware/ltr_303>
    $<$<BOOL:${ENABLE_PH_SENSOR}>:${CMAKE_CURRENT_SOURCE_DIR}/core/hardware/oem_ph>
    $<$<BOOL:${ENABLE_SEA_TEMP_SENSOR}>:${CMAKE_CURRENT_SOURCE_DIR}/core/hardware/oem_rtd>
    $<$<BOOL:${ENABLE_SEA_TEMP_SENSOR}>:${CMAKE_CURRENT_SOURCE_DIR}/core/hardware/ezo_rtd>
    $<$<BOOL:${ENABLE_CDT_SENSOR}>:${CMAKE_CURRENT_SOURCE_DIR}/core/hardware/cdt>
    $<$<BOOL:${ENABLE_AXL_SENSOR}>:${CMAKE_CURRENT_SOURCE_DIR}/core/hardware/bma400>
    $<$<BOOL:${ENABLE_SEA_TEMP_SENSOR}>:${CMAKE_CURRENT_SOURCE_DIR}/core/hardware/tsys01>
    ${PROJ_DIR}/core/hardware/run_cam
    $<$<BOOL:${ENABLE_PRESSURE_SENSOR}>:${CMAKE_CURRENT_SOURCE_DIR}/core/hardware/lps28dfw>
    $<$<STREQUAL:${BATTERY_MONITOR_TYPE},STC3117>:${CMAKE_CURRENT_SOURCE_DIR}/core/hardware/stc3117>
)

target_sources(${TARGET_NAME} PRIVATE
    ${LIB_DIR}/littlefs/lfs_util.c
    ${LIB_DIR}/littlefs/lfs.c
    ${LIB_DIR}/prepass/prepass_package_code-PREPAS_V3.4/previpass.c
    ${LIB_DIR}/prepass/prepass_package_code-PREPAS_V3.4/previpass_util.c
    ${LIB_DIR}/CmBacktrace/cm_backtrace/cm_backtrace.c
    ${SDK_ROOT}/components/ble/common/ble_advdata.c
    ${SDK_ROOT}/components/ble/ble_advertising/ble_advertising.c
    ${SDK_ROOT}/components/ble/common/ble_conn_state.c
    ${SDK_ROOT}/components/ble/ble_link_ctx_manager/ble_link_ctx_manager.c
    ${SDK_ROOT}/components/ble/common/ble_srv_common.c
    ${SDK_ROOT}/components/ble/nrf_ble_gatt/nrf_ble_gatt.c
    ${SDK_ROOT}/components/ble/nrf_ble_qwr/nrf_ble_qwr.c
    ${SDK_ROOT}/components/ble/ble_services/ble_nus/ble_nus.c
    ${SDK_ROOT}/components/libraries/memobj/nrf_memobj.c
    ${SDK_ROOT}/components/libraries/balloc/nrf_balloc.c
    ${SDK_ROOT}/components/libraries/log/src/nrf_log_frontend.c
    ${SDK_ROOT}/components/libraries/log/src/nrf_log_str_formatter.c
    ${SDK_ROOT}/components/libraries/ringbuf/nrf_ringbuf.c
    ${SDK_ROOT}/components/libraries/log/src/nrf_log_backend_serial.c
    ${SDK_ROOT}/external/fprintf/nrf_fprintf.c
    ${SDK_ROOT}/external/fprintf/nrf_fprintf_format.c
    ${SDK_ROOT}/components/libraries/experimental_section_vars/nrf_section_iter.c
    ${SDK_ROOT}/components/libraries/util/app_util_platform.c
    ${SDK_ROOT}/components/libraries/atomic_flags/nrf_atflags.c
    ${SDK_ROOT}/components/libraries/atomic/nrf_atomic.c
    ${SDK_ROOT}/components/libraries/atomic_fifo/nrf_atfifo.c
    ${SDK_ROOT}/components/libraries/util/app_error.c
    ${SDK_ROOT}/components/libraries/util/app_error_handler_gcc.c
    ${SDK_ROOT}/components/libraries/util/app_error_weak.c
    ${SDK_ROOT}/components/libraries/strerror/nrf_strerror.c
    ${SDK_ROOT}/components/libraries/timer/drv_rtc.c
    ${SDK_ROOT}/components/libraries/pwr_mgmt/nrf_pwr_mgmt.c
    ${SDK_ROOT}/components/libraries/sortlist/nrf_sortlist.c
    ${SDK_ROOT}/components/softdevice/common/nrf_sdh.c
    ${SDK_ROOT}/components/softdevice/common/nrf_sdh_ble.c
    ${SDK_ROOT}/components/softdevice/common/nrf_sdh_soc.c
    ${SDK_ROOT}/components/libraries/crc32/crc32.c
    ${SDK_ROOT}/components/libraries/crc16/crc16.c
	${SDK_ROOT}/components/libraries/queue/nrf_queue.c
    ${SDK_ROOT}/components/libraries/libuarte/nrf_libuarte_async.c
    ${SDK_ROOT}/components/libraries/libuarte/nrf_libuarte_drv.c
    ${SDK_ROOT}/components/libraries/usbd/app_usbd.c
    ${SDK_ROOT}/components/libraries/usbd/app_usbd_core.c
    ${SDK_ROOT}/components/libraries/usbd/app_usbd_string_desc.c
    ${SDK_ROOT}/components/libraries/usbd/app_usbd_serial_num.c
    ${SDK_ROOT}/components/libraries/usbd/class/cdc/acm/app_usbd_cdc_acm.c
    ${SDK_ROOT}/integration/nrfx/legacy/nrf_drv_clock.c
    ${SDK_ROOT}/integration/nrfx/legacy/nrf_drv_power.c
    ${SDK_ROOT}/modules/nrfx/mdk/system_nrf52840.c
    ${SDK_ROOT}/modules/nrfx/drivers/src/nrfx_uarte.c
    ${SDK_ROOT}/modules/nrfx/drivers/src/nrfx_qspi.c
    ${SDK_ROOT}/modules/nrfx/drivers/src/nrfx_spim.c
    ${SDK_ROOT}/modules/nrfx/drivers/src/nrfx_twim.c
    ${SDK_ROOT}/modules/nrfx/drivers/src/nrfx_rtc.c
    ${SDK_ROOT}/modules/nrfx/drivers/src/nrfx_wdt.c
	${SDK_ROOT}/modules/nrfx/drivers/src/nrfx_ppi.c
	${SDK_ROOT}/modules/nrfx/drivers/src/nrfx_timer.c
    ${SDK_ROOT}/modules/nrfx/drivers/src/nrfx_gpiote.c
    ${SDK_ROOT}/modules/nrfx/drivers/src/nrfx_saadc.c
    ${SDK_ROOT}/modules/nrfx/drivers/src/nrfx_usbd.c
    ${SDK_ROOT}/modules/nrfx/drivers/src/nrfx_clock.c
    ${SDK_ROOT}/modules/nrfx/drivers/src/nrfx_power.c
    ${SDK_ROOT}/modules/nrfx/soc/nrfx_atomic.c
    ${SDK_ROOT}/modules/nrfx/hal/nrf_nvmc.c
    ${CORE_DIR}/hardware/sensor.cpp
    ${CORE_DIR}/hardware/reed.cpp
    ${CORE_DIR}/sm/gentracker.cpp
    ${CORE_DIR}/sm/ledsm.cpp
    ${CORE_DIR}/sm/buzzm.cpp
    ${CORE_DIR}/services/service.cpp
    ${CORE_DIR}/services/gps_service.cpp
    ${CORE_DIR}/services/argos_tx_service.cpp
    ${CORE_DIR}/services/argos_packet_builder.cpp
    ${CORE_DIR}/services/depth_pile.cpp
    ${CORE_DIR}/services/rate_limiter.cpp
    ${CORE_DIR}/services/hauled_mode_service.cpp
    ${CORE_DIR}/services/moored_mode_service.cpp
    ${CORE_DIR}/services/argos_rx_service.cpp
    ${CORE_DIR}/scheduling/argos_tx_scheduler.cpp
    ${CORE_DIR}/services/uwdetector_service.cpp
    $<$<BOOL:${ENABLE_SWS_ANALOG}>:${CORE_DIR}/services/sws_analog_service.cpp>
    $<$<BOOL:${ENABLE_SWS_ANALOG}>:${CORE_DIR}/services/sws_analog_calibration.cpp>
    $<$<BOOL:${ENABLE_SWS_ANALOG}>:${CORE_DIR}/services/sws_analog_detection.cpp>
    ${CORE_DIR}/logging/logger.cpp
    ${CORE_DIR}/util/haversine.cpp
    ${CORE_DIR}/filesystem/ota_flash_file_updater.cpp
    ${CORE_DIR}/protocol/dte_commands.cpp
    ${CORE_DIR}/protocol/dte_params.cpp
    ${CORE_DIR}/protocol/dte_handler.cpp
    ${CORE_DIR}/configuration/calibration.cpp
    ${BOARD_DIR}/bsp.cpp
    ${PROJ_DIR}/core/filesystem/is25_flash.cpp
    $<$<OR:$<BOOL:${ENABLE_PRESSURE_SENSOR}>,$<BOOL:${ENABLE_CDT_SENSOR}>>:${CMAKE_CURRENT_SOURCE_DIR}/core/hardware/ms58xx/ms58xx.cpp>
    $<$<OR:$<BOOL:${ENABLE_PRESSURE_SENSOR}>,$<BOOL:${ENABLE_CDT_SENSOR}>>:${CMAKE_CURRENT_SOURCE_DIR}/core/hardware/bar100/bar100.cpp>
    $<$<BOOL:${ENABLE_PRESSURE_SENSOR}>:${CMAKE_CURRENT_SOURCE_DIR}/core/hardware/lps28dfw/lps28dfw.cpp>
    $<$<BOOL:${ENABLE_PRESSURE_SENSOR}>:${CMAKE_CURRENT_SOURCE_DIR}/core/hardware/lps28dfw/lps28dfw_reg.c>
    $<$<BOOL:${ENABLE_CDT_SENSOR}>:${CMAKE_CURRENT_SOURCE_DIR}/core/hardware/ad5933/ad5933.cpp>
    $<$<BOOL:${ENABLE_AXL_SENSOR}>:${CMAKE_CURRENT_SOURCE_DIR}/core/hardware/bma400/bma400.cpp>
    $<$<BOOL:${ENABLE_AXL_SENSOR}>:${CMAKE_CURRENT_SOURCE_DIR}/core/hardware/bma400/bma400.c>
    $<$<BOOL:${ENABLE_CDT_SENSOR}>:${CMAKE_CURRENT_SOURCE_DIR}/core/hardware/cdt/cdt.cpp>
    $<$<BOOL:${ENABLE_ALS_SENSOR}>:${CMAKE_CURRENT_SOURCE_DIR}/core/hardware/ltr_303/ltr_303.cpp>
    $<$<BOOL:${ENABLE_PH_SENSOR}>:${CMAKE_CURRENT_SOURCE_DIR}/core/hardware/oem_ph/oem_ph.cpp>
    $<$<BOOL:${ENABLE_SEA_TEMP_SENSOR}>:${CMAKE_CURRENT_SOURCE_DIR}/core/hardware/oem_rtd/oem_rtd.cpp>
    $<$<BOOL:${ENABLE_SEA_TEMP_SENSOR}>:${CMAKE_CURRENT_SOURCE_DIR}/core/hardware/ezo_rtd/ezo_rtd.cpp>
    $<$<BOOL:${ENABLE_SEA_TEMP_SENSOR}>:${CMAKE_CURRENT_SOURCE_DIR}/core/hardware/tsys01/TSYS01.cpp>
    $<$<AND:$<NOT:$<BOOL:${ARGOS_SMD_ENABLED}>>,$<NOT:$<BOOL:${LORA_RAK3172_ENABLED}>>>:${CMAKE_CURRENT_SOURCE_DIR}/core/hardware/kim2/kim2.cpp>
    $<$<AND:$<NOT:$<BOOL:${ARGOS_SMD_ENABLED}>>,$<NOT:$<BOOL:${LORA_RAK3172_ENABLED}>>>:${CMAKE_CURRENT_SOURCE_DIR}/core/hardware/kim2/kim2_comm.cpp>
    ${PROJ_DIR}/core/hardware/nrf_timer.cpp
    ${PROJ_DIR}/core/hardware/nrf_rtc.cpp
    ${PROJ_DIR}/core/hardware/nrf_spim.cpp
    ${PROJ_DIR}/core/hardware/nrf_uart_async.cpp
    ${PROJ_DIR}/core/hardware/nrf_gpio.cpp
    ${PROJ_DIR}/core/hardware/nrf_pmu.cpp
    ${PROJ_DIR}/core/hardware/nrf_switch.cpp
    ${PROJ_DIR}/core/hardware/nrf_irq.cpp
    $<$<STREQUAL:${BATTERY_MONITOR_TYPE},ANALOG>:${CMAKE_CURRENT_SOURCE_DIR}/core/hardware/nrf_battery_mon.cpp>
    $<$<STREQUAL:${BATTERY_MONITOR_TYPE},FAKE>:${CMAKE_CURRENT_SOURCE_DIR}/core/hardware/fake_battery_mon.cpp>
    $<$<STREQUAL:${BATTERY_MONITOR_TYPE},STC3117>:${CMAKE_CURRENT_SOURCE_DIR}/core/hardware/stc3117/stc3117_gasgauge.cpp>
    $<$<STREQUAL:${BATTERY_MONITOR_TYPE},STC3117>:${CMAKE_CURRENT_SOURCE_DIR}/core/hardware/stc3117/stc311x_gasgauge.c>
    $<$<STREQUAL:${BATTERY_MONITOR_TYPE},STC3117>:${CMAKE_CURRENT_SOURCE_DIR}/core/hardware/stc3117/Generic_I2C.c>
    ${PROJ_DIR}/core/hardware/nrf_i2c.cpp
    ${PROJ_DIR}/core/hardware/nrf_usb.cpp
    $<$<BOOL:${BENCH_TEST}>:${PROJ_DIR}/core/interface/bench_console.cpp>
    $<$<STREQUAL:${BOARD},RSPB>:${CMAKE_CURRENT_SOURCE_DIR}/core/hardware/nrf_debug_uart.cpp>
    ${PROJ_DIR}/core/interface/ble_interface.cpp
    ${PROJ_DIR}/core/interface/ble_stm_ota.c
    ${PROJ_DIR}/core/memmang/heap_4.c
    ${PROJ_DIR}/core/memmang/heap_mem.c
    ${PROJ_DIR}/core/memmang/bindings.c
    ${PROJ_DIR}/core/memmang/bindings.cpp
    ${PROJ_DIR}/core/memmang/memmang.cpp
    ${PROJ_DIR}/core/scheduling/interrupt_lock.cpp
    ${PROJ_DIR}/drivers/nrf_log_redirect/nrf_log_redirect.c
    ${PROJ_DIR}/gcc_startup_nrf52840.S
    ${PROJ_DIR}/main.cpp
)

# Configure CAM and BUZZER sensors (mutually exclusive on GPIO5)
# Default to both disabled
if(NOT DEFINED CAM_ENABLE)
    set(CAM_ENABLE 0)
endif()
if(NOT DEFINED BUZZER_ENABLE)
    set(BUZZER_ENABLE 0)
endif()

# Check that both CAM and BUZZER are not enabled simultaneously
if(CAM_ENABLE AND BUZZER_ENABLE)
    message(FATAL_ERROR "CAM_ENABLE and BUZZER_ENABLE cannot both be set to 1 (they share GPIO5)")
endif()

if(CAM_ENABLE)
    message(STATUS "Configuring for CAM sensor on GPIO_EXT_GPIO4/5")
    set(ENABLE_CAM_SENSOR 1)
    set(ENABLE_BUZZER 0)

    # Add CAM-specific sources
    target_sources(${TARGET_NAME} PRIVATE
        ${PROJ_DIR}/core/hardware/run_cam/runcam.cpp
        ${CORE_DIR}/services/cam_service.cpp
    )
elseif(BUZZER_ENABLE)
    message(STATUS "Configuring for BUZZER on GPIO_EXT_GPIO5")
    set(ENABLE_CAM_SENSOR 0)
    set(ENABLE_BUZZER 1)
else()
    # Neither enabled
    message(STATUS "GPIO_EXT_GPIO4/5 not assigned (neither CAM nor BUZZER)")
    set(ENABLE_CAM_SENSOR 0)
    set(ENABLE_BUZZER 0)
endif()

# Always use M10Q GPS module
target_sources(${TARGET_NAME} PRIVATE
    ${PROJ_DIR}/core/hardware/m10qasync/m10qasync.cpp
    ${PROJ_DIR}/core/hardware/m10qasync/ubx_comms.cpp
)

target_include_directories(${TARGET_NAME} SYSTEM PRIVATE
    ${PROJ_DIR}/core/hardware/m10qasync
)

# SMD satellite module (conditional)
# Use -DSMD_UART=ON to select AT/UART transport instead of SPI (default)
if(ARGOS_SMD)
    target_sources(${TARGET_NAME} PRIVATE
        ${PROJ_DIR}/core/hardware/smd_sat/smd_sat.cpp
    )
    if(SMD_UART)
        message(STATUS "SMD transport: AT/UART")
        target_sources(${TARGET_NAME} PRIVATE
            ${PROJ_DIR}/core/hardware/smd_sat/smd_sat_cmd_at.cpp
        )
        target_compile_definitions(${TARGET_NAME} PRIVATE SMD_UART=1)
    else()
        message(STATUS "SMD transport: SPI (default)")
        target_sources(${TARGET_NAME} PRIVATE
            ${PROJ_DIR}/core/hardware/smd_sat/smd_sat_cmd_spi.cpp
        )
        target_compile_definitions(${TARGET_NAME} PRIVATE SMD_SPI=1)
    endif()
    target_include_directories(${TARGET_NAME} SYSTEM PRIVATE
        ${PROJ_DIR}/core/hardware/smd_sat
    )
endif()

# LoRa RAK3172-SiP module (conditional)
if(LORA_RAK3172)
    target_sources(${TARGET_NAME} PRIVATE
        ${PROJ_DIR}/core/hardware/lora_rak3172/lora_rak3172.cpp
        ${PROJ_DIR}/core/hardware/lora_rak3172/lora_rak3172_comm.cpp
        ${CORE_DIR}/services/lora_tx_service.cpp
        ${CORE_DIR}/services/lora_packet_builder.cpp
        ${CORE_DIR}/scheduling/lora_tx_scheduler.cpp
    )
    target_include_directories(${TARGET_NAME} SYSTEM PRIVATE
        ${PROJ_DIR}/core/hardware/lora_rak3172
    )
endif()

# Thermistor NTC sensor (conditional)
if(ENABLE_THERMISTOR_SENSOR EQUAL 1)
    target_sources(${TARGET_NAME} PRIVATE
        ${PROJ_DIR}/core/hardware/thermistor/thermistor.cpp
    )
    target_include_directories(${TARGET_NAME} SYSTEM PRIVATE
        ${PROJ_DIR}/core/hardware/thermistor
    )
endif()

# Mortality detection service (conditional)
if(ENABLE_MORTALITY_SENSOR EQUAL 1)
    target_sources(${TARGET_NAME} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../../core/services/mortality_service.cpp
    )
endif()


# Passage 2 : rattrape les flags posés par la section spécifique à la carte
# (ENABLE_CAM_SENSOR / ENABLE_BUZZER via EXT_GPIO5_DEVICE, ENABLE_SWS_ANALOG
# forcé à 0 sur RSPB, ...). Idempotent sur ceux déjà normalisés au passage 1.
normalise_bool_flags()

target_compile_definitions(${TARGET_NAME} PRIVATE
    Release=0
    Debug=1
    $<$<BOOL:${GPS_FAKE_POSITION}>:GPS_FAKE_POSITION>
    ENABLE_CAM_SENSOR=${ENABLE_CAM_SENSOR}
    ENABLE_BUZZER=${ENABLE_BUZZER}
    ENABLE_PH_SENSOR=${ENABLE_PH_SENSOR}
    ENABLE_SEA_TEMP_SENSOR=${ENABLE_SEA_TEMP_SENSOR}
    ENABLE_PRESSURE_SENSOR=${ENABLE_PRESSURE_SENSOR}
    ENABLE_ALS_SENSOR=${ENABLE_ALS_SENSOR}
    ENABLE_CDT_SENSOR=${ENABLE_CDT_SENSOR}
    ENABLE_AXL_SENSOR=${ENABLE_AXL_SENSOR}
    ENABLE_THERMISTOR_SENSOR=${ENABLE_THERMISTOR_SENSOR}
    ENABLE_MORTALITY_SENSOR=${ENABLE_MORTALITY_SENSOR}
    ENABLE_SWS_LOG=${ENABLE_SWS_LOG}
    ENABLE_SWS_ANALOG=${ENABLE_SWS_ANALOG}
    GNSS_HAS_BACKUP_BATTERY=${GNSS_HAS_BACKUP_BATTERY}
    STC3117_MIXED_MODE=${STC3117_MIXED_MODE}
    ARGOS_SMD=${ARGOS_SMD_ENABLED}
    SMDSAT_USE_SAFE_TIMINGS=${SMDSAT_USE_SAFE_TIMINGS_VAL}
    SMDSAT_AUTOFALLBACK_ENABLED=${SMDSAT_AUTOFALLBACK_VAL}
    # TEST: power SMD at boot and park the nRF so the STM32WL can be flashed
    # without the board resetting. Enable with -DSMD_FLASH_HOLD=ON. See main.cpp.
    $<$<BOOL:${SMD_FLASH_HOLD}>:SMD_FLASH_HOLD=1>
    # KIM2 + BLIND: relâcher SAT_EXTWAKEUP pendant la salve pilotée par le module,
    # pour qu'il descende en basse consommation entre ses retransmissions au lieu
    # de rester éveillé toute la séquence. Défaut OFF = pin maintenue haute, le
    # comportement d'avant. Voir KIM2Device::release_wkup_for_burst().
    $<$<BOOL:${KIM2_BLIND_WKUP_RELEASE}>:KIM2_BLIND_WKUP_RELEASE=1>
    # RSPB: keep VSENSORS always on (I2C pull-ups live on it). 1=floor (default),
    # 0=allow cut (test). Only forwarded when set (build_rspb.sh); other boards
    # fall back to the #ifndef default in nrf_gpio.cpp.
    $<$<NOT:$<STREQUAL:${RSPB_VSENSORS_FLOOR},>>:RSPB_VSENSORS_FLOOR=${RSPB_VSENSORS_FLOOR}>
    # TPL5111 external-wakeup period default (s) — must match the board timing
    # resistor. Set via build_rspb.sh WAKEUP_PERIOD; else falls back to 6300U in
    # config_store.hpp. Only forwarded when set (RSPB); other boards use the fallback.
    $<$<NOT:$<STREQUAL:${WAKEUP_PERIOD_DEFAULT},>>:WAKEUP_PERIOD_DEFAULT=${WAKEUP_PERIOD_DEFAULT}>
    $<$<BOOL:${LORA_RAK3172_ENABLED}>:LORA_RAK3172=1>
    $<$<BOOL:${LORA_RAK3172_ENABLED}>:LORA_DCS_ENABLE=${LORA_DCS_ENABLE_VAL}>
    $<$<BOOL:${LORA_RAK3172_ENABLED}>:LORA_TX_ERROR_SUSPEND_S=${LORA_TX_ERROR_SUSPEND_S}>
    $<$<BOOL:${LORA_RAK3172_ENABLED}>:LORA_POWER_OFF_UNDERWATER=${LORA_POWER_OFF_UNDERWATER_VAL}>
    # Battery monitor type defines
    $<$<STREQUAL:${BATTERY_MONITOR_TYPE},ANALOG>:BATTERY_MONITOR_ANALOG>
    $<$<STREQUAL:${BATTERY_MONITOR_TYPE},FAKE>:BATTERY_MONITOR_FAKE>
    $<$<STREQUAL:${BATTERY_MONITOR_TYPE},STC3117>:BATTERY_MONITOR_STC3117>
    # Battery chemistry (ANALOG only — selects discharge LUT)
    $<$<STREQUAL:${BATTERY_MONITOR_TYPE},ANALOG>:BATTERY_CHEMISTRY=${BATTERY_CHEMISTRY}>
    # USB DTE interface (parallel with BLE)
    USB_DTE_ENABLED
    # Bench-test harness (autonomous HIL validation) — off in production
    $<$<BOOL:${BENCH_TEST}>:BENCH_TEST=1>
    $<$<BOOL:${BENCH_FORCE_LPM_NONE}>:BENCH_FORCE_LPM_NONE=1>
    BUILD_TYPE=${CMAKE_BUILD_TYPE}
    #SCHEDULER_DEBUG
    DEBUG_ENABLE=1
    DEBUG_LEVEL=${DEBUG_LEVEL}
    DEBUG_USING_RTC
    $<$<STREQUAL:${CMAKE_BUILD_TYPE},Debug>:DEBUG_NO_WATCHDOG>
    $<$<STREQUAL:${BOARD},RSPB>:BOARD_RSPB_V1>
    $<$<STREQUAL:${BOARD},RSPB>:BOARD_RSPB>
    $<$<STREQUAL:${BOARD},RSPB>:EXTERNAL_WAKEUP>
    $<$<NOT:$<STREQUAL:${BOARD},RSPB>>:BOARD_LINKIT_V4>
    BOARD_${BOARD}
    CONFIG_GPIO_AS_PINRESET
    CONFIG_NFCT_PINS_AS_GPIOS
    FLOAT_ABI_HARD
    NRF52840_XXAA
    NRF_SD_BLE_API_VERSION=7
    S140
    SOFTDEVICE_PRESENT
    __HEAP_SIZE=0 # Use configTOTAL_HEAP_SIZE instead
    __STACK_SIZE=32768
    configTOTAL_HEAP_SIZE=65536
    DEVICE_MODEL_NAME=std::string\("${DEVICE_MODEL}"\)
    #LFS_YES_TRACE
    #BATTERY_NOT_FITTED
    #DUMMY_PRESSURE_SENSOR
    #DEBUG_AD5933
    DEBUG_TO_SYSTEMLOG
    ETL_LOG_ERRORS
    ARGOS_USE_CRC8
    PREPASS_EMBEDDED
    FW_APP_VERSION_STR=std::string\("${FW_APP_VERSION}"\)
    FW_APP_VERSION_STR_C="${FW_APP_VERSION}"
    #WORKAROUND_ALLCAST_CONSTELLATION_STATUS_ENCODING_BUG
    _XOPEN_SOURCE=700 # Enables strptime() usage
    __STARTUP_FILL_STACK
    # Optional log channels forwarded from build scripts (--metrics / --validation).
    # Each header re-#defines to 0 if unset, so passing 0 here is also fine.
    $<$<BOOL:${METRIC_LATENCY_LOG_ENABLE}>:METRIC_LATENCY_LOG_ENABLE=${METRIC_LATENCY_LOG_ENABLE}>
    $<$<BOOL:${VALIDATION_LOG_ENABLE}>:VALIDATION_LOG_ENABLE=${VALIDATION_LOG_ENABLE}>
)

target_compile_options(${TARGET_NAME} PRIVATE
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
    -Wno-unused-parameter
    -Wno-expansion-to-defined # The NRFX library requires this
    -Wno-psabi
    -Wno-unused-function

    # Build flags
    -mcpu=cortex-m4
    -mthumb
    -mabi=aapcs
    -mfloat-abi=hard
    -mfpu=fpv4-sp-d16
    -ffunction-sections
    -fdata-sections
    -fno-strict-aliasing
    -fno-builtin
    -fshort-enums

    # Always output coloured text from GCC
    -fdiagnostics-color=always

    # Debugging stack corruption
    -fstack-protector-strong

    # Optimise for size always
    -Os
    -g3
)

set_source_files_properties(${LIB_DIR}/prepass/prepass_package_code-PREPAS_V3.4/previpass.c PROPERTIES COMPILE_FLAGS "-Wno-unused-parameter -Wno-switch-enum -Wno-switch")

set_source_files_properties(${LIB_DIR}/CmBacktrace/cm_backtrace/cm_backtrace.c PROPERTIES COMPILE_FLAGS "-Wno-unused-const-variable -Wno-unused-variable")

target_link_options(${TARGET_NAME} PRIVATE
    -L${CMAKE_SOURCE_DIR}/${SDK_ROOT}/modules/nrfx/mdk
    -T${PROJECT_SOURCE_DIR}/gcc_nrf52840.ld
    -mcpu=cortex-m4
    -mthumb
    -mabi=aapcs
    -mfloat-abi=hard
    -mfpu=fpv4-sp-d16
    -Wl,--gc-sections
    #--specs=nano.specs # We use exceptions so we can not use this due to nano-specs being built with exceptions disabled
    -u_printf_float
    -u_scanf_float
    -Xlinker -Map=${TARGET_NAME}.map

    -Wl,--wrap=__stack_chk_fail    
)

target_link_libraries(${TARGET_NAME}
    m
    c
    nosys
)

function(create_bin executable)
    add_custom_command(
        TARGET ${executable}
        POST_BUILD
        COMMENT "Generating ${executable}.bin"
        COMMAND ${CMAKE_OBJCOPY} -O binary ${CMAKE_CURRENT_BINARY_DIR}/${executable}.elf ${CMAKE_CURRENT_BINARY_DIR}/${executable}.bin
    )
endfunction(create_bin)

function(create_hex executable)
    add_custom_command(
        TARGET ${executable}
        POST_BUILD
        COMMENT "Generating ${executable}.hex"
        COMMAND ${CMAKE_OBJCOPY} -O ihex ${CMAKE_CURRENT_BINARY_DIR}/${executable}.elf ${CMAKE_CURRENT_BINARY_DIR}/${executable}.hex
    )
endfunction(create_hex)

function(create_dfu executable)
    add_custom_command(
        TARGET ${executable}
        POST_BUILD
        COMMENT "Generating ${executable}_dfu.zip"
        COMMAND nrfutil pkg generate --hw-version 52 --sd-req 0x0100 --application-version 0 --application ${CMAKE_CURRENT_BINARY_DIR}/${executable}.hex --key-file ${PROJECT_SOURCE_DIR}/${PROJ_DIR}/nrfutil_pkg_key.pem ${CMAKE_CURRENT_BINARY_DIR}/${executable}_dfu.zip
    )
endfunction(create_dfu)

function(create_wrapped_bin executable)
    add_custom_command(
        TARGET ${executable}
        POST_BUILD
        COMMENT "Generating ${executable}.img"
        COMMAND printf "%08x" `stat -c %s ${executable}.bin` | xxd -r -p > ${executable}.img && crc32 ${executable}.bin | xxd -r -p >> ${executable}.img && cat ${executable}.bin >> ${executable}.img
    )
endfunction(create_wrapped_bin)

create_bin(${TARGET_NAME})
create_wrapped_bin(${TARGET_NAME})
create_hex(${TARGET_NAME})
create_dfu(${TARGET_NAME})

add_custom_target(flash_softdev
                  COMMENT "Flashing the softdevice using JLink..."
                  COMMAND nrfjprog -f nrf52 --eraseall
                  COMMAND nrfjprog -f nrf52 --program ${PROJECT_SOURCE_DIR}/${SDK_ROOT}/components/softdevice/s140/hex/s140_nrf52_7.2.0_softdevice.hex --sectoranduicrerase
                  COMMAND nrfjprog -f nrf52 --reset
)

add_custom_target(flash
                  COMMENT "Flashing the softdevice & firmware using JLink..."
                  COMMAND nrfjprog -f nrf52 --eraseall
                  COMMAND nrfjprog -f nrf52 --program ${CMAKE_CURRENT_BINARY_DIR}/${TARGET_NAME}_merged.hex --sectorerase
                  COMMAND nrfjprog -f nrf52 --reset
)

add_custom_target(dfu_app
                  COMMENT "Sending application image over DFU (reset device with DEBUG pin jumpered!)..."
                  COMMAND nrfutil dfu usb-serial -p /dev/ttyACM0 -pkg ${TARGET_NAME}_dfu.zip
)

add_custom_target(merged_hex
                  COMMENT "Create combined bootloader, settings and applications file"
                  COMMAND nrfutil settings generate --family NRF52840 --application ${CMAKE_CURRENT_BINARY_DIR}/${TARGET_NAME}.hex --application-version 0 --bootloader-version 1 --bl-settings-version 2 --app-boot-validation VALIDATE_ECDSA_P256_SHA256 --sd-boot-validation VALIDATE_ECDSA_P256_SHA256 --softdevice ${PROJECT_SOURCE_DIR}/${SDK_ROOT}/components/softdevice/s140/hex/s140_nrf52_7.2.0_softdevice.hex --key-file ${PROJECT_SOURCE_DIR}/${PROJ_DIR}/nrfutil_pkg_key.pem ${TARGET_NAME}_settings.hex
                  COMMAND mergehex -m ${BOOTLOADER} ${CMAKE_CURRENT_BINARY_DIR}/${TARGET_NAME}.hex -o m1.hex
                  COMMAND mergehex -m m1.hex ${CMAKE_CURRENT_BINARY_DIR}/${TARGET_NAME}_settings.hex -o ${CMAKE_CURRENT_BINARY_DIR}/${TARGET_NAME}_merged.hex
                  COMMAND rm -f m1.hex
)

add_dependencies(flash ${TARGET_NAME})
add_dependencies(flash_softdev ${TARGET_NAME})
add_dependencies(dfu_app ${TARGET_NAME})
