#!/bin/bash

# =============================================================================
# RSPB Core Build Script
# =============================================================================
# This script builds the RSPB Core firmware with SMD satellite module support.
#
# IMPORTANT: Update build_config.sh with your toolchain paths
# =============================================================================

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# Load build configuration
if [ -f "$PROJECT_ROOT/build_config.sh" ]; then
    source "$PROJECT_ROOT/build_config.sh"
else
    echo "ERROR: build_config.sh not found!"
    echo "Please create build_config.sh in the project root."
    echo "You can copy build_config.sh.example as a template."
    exit 1
fi

# Verify required tools are available
echo "Checking build tools..."

if ! command -v arm-none-eabi-gcc &> /dev/null; then
    echo "ERROR: arm-none-eabi-gcc not found in PATH!"
    echo ""
    echo "Please update build_config.sh with the correct path:"
    echo "  ARM_TOOLCHAIN_PATH=\"/path/to/your/gcc-arm-none-eabi/bin\""
    echo ""
    echo "Current configuration:"
    echo "  ARM_TOOLCHAIN_PATH=\"$ARM_TOOLCHAIN_PATH\""
    echo ""
    exit 1
fi

if ! command -v nrfutil &> /dev/null; then
    echo "ERROR: nrfutil not found in PATH!"
    echo ""
    echo "Please update build_config.sh with the correct path:"
    echo "  NRFUTIL_PATH=\"/path/to/your/nrfutil/bin\""
    echo ""
    echo "Current configuration:"
    echo "  NRFUTIL_PATH=\"$NRFUTIL_PATH\""
    echo ""
    exit 1
fi

if ! command -v mergehex &> /dev/null; then
    echo "WARNING: mergehex not found in PATH!"
    echo "The build may fail during hex file merging."
    echo "Install mergehex: pip install intelhex"
    echo ""
fi

if ! command -v make &> /dev/null; then
    echo "ERROR: make not found in PATH!"
    echo "Please install build-essential: sudo apt-get install build-essential"
    exit 1
fi

echo "✓ arm-none-eabi-gcc found: $(which arm-none-eabi-gcc)"
echo "✓ nrfutil found: $(which nrfutil)"
if command -v mergehex &> /dev/null; then
    echo "✓ mergehex found: $(which mergehex)"
fi
echo ""

# Parse arguments
CLEAN=false
RECOVER=false
BUILD_TYPE=Release
METRICS=OFF
VALIDATION=OFF
BENCH=OFF
for arg in "$@"; do
    case $arg in
        --clean) CLEAN=true ;;
        --recover) RECOVER=true ;;
        --debug) BUILD_TYPE=Debug ;;
        --release) BUILD_TYPE=Release ;;
        --metrics) METRICS=ON ;;
        --no-metrics) METRICS=OFF ;;
        --validation) VALIDATION=ON ;;
        --no-validation) VALIDATION=OFF ;;
        # Bench harness: auto-inject a fix each (reset-simulated) TPL duty-cycle
        # + powerdown->soft-reset so the board stays reachable over the debug UART.
        # Implies --debug (logs on UART). See ports/nrf52840/core/interface/bench_console.*
        --bench) BENCH=ON; BUILD_TYPE=Debug ;;
    esac
done

if [ "$RECOVER" = true ]; then
    echo "NOTE: --recover flag set — recover command will be shown in flash commands below"
    echo ""
fi

# Build-type banner. RSPB uses UART (not USB_CDC) for its debug channel.
if [ "$BUILD_TYPE" = "Debug" ]; then
    printf '\033[1;33m'
    echo "╔══════════════════════════════════════════════════════════════════╗"
    echo "║                BUILD TYPE: DEBUG  (g_debug_mode=UART)            ║"
    echo "║     UART debug logs ENABLED · DEBUG_NO_WATCHDOG ENABLED          ║"
    echo "║   NOT FOR DEPLOYMENT — for bench validation / VAL_LOG campaigns  ║"
    echo "╚══════════════════════════════════════════════════════════════════╝"
    printf '\033[0m'
else
    printf '\033[1;32m'
    echo "╔══════════════════════════════════════════════════════════════════╗"
    echo "║              BUILD TYPE: RELEASE  (g_debug_mode=NONE)            ║"
    echo "║   UART debug silent (DEPLOY) · WDT active · TPL5111 wakeup       ║"
    echo "║   To enable logs: ./build_rspb.sh --debug                        ║"
    echo "╚══════════════════════════════════════════════════════════════════╝"
    printf '\033[0m'
fi
printf '\033[1;36m   Optional log flags:  METRIC_LATENCY=%s   VALIDATION=%s\033[0m\n' "$METRICS" "$VALIDATION"
echo ""

cd "$PROJECT_ROOT"
BUILD_DIR="ports/nrf52840/build/RSPB"
if [ "$CLEAN" = true ]; then
    echo "Cleaning build directory..."
    rm -rf "$BUILD_DIR"
fi
mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"
git show-ref --tags -d | grep ^`git rev-parse HEAD` | sed -e "s,.* refs/tags/,," -e "s/\\^{}//" > TAG_NAME
if [ -z "$(cat TAG_NAME)" ]; then
    git describe --dirty > TAG_NAME
fi

# RSPB build configuration
# - ARGOS_SMD=ON: Use SMD satellite module (default for RSPB)
# - ENABLE_PRESSURE_SENSOR=ON: Enable LPS28DFW pressure sensor
# - ENABLE_AXL_SENSOR=ON: Enable BMA400 accelerometer
# - ENABLE_THERMISTOR_SENSOR=ON: Enable thermistor temperature sensor
# - ENABLE_MORTALITY_SENSOR=ON: Enable mortality detection (requires AXL + THERMISTOR)
# - GNSS_HAS_BACKUP_BATTERY=1: M10Q has a backup battery (BBR) for warm/hot starts
# - SMDSAT_USE_SAFE_TIMINGS=ON: force the conservative v4.1.4 SAFE SPI timing baseline
# - SMDSAT_AUTOFALLBACK=ON: runtime auto-fallback to SAFE timings (only if USE_SAFE_TIMINGS=OFF)
ARGOS_SMD=${ARGOS_SMD:-ON}
ENABLE_PRESSURE_SENSOR=${ENABLE_PRESSURE_SENSOR:-1}
ENABLE_AXL_SENSOR=${ENABLE_AXL_SENSOR:-1}
ENABLE_THERMISTOR_SENSOR=${ENABLE_THERMISTOR_SENSOR:-1}
ENABLE_MORTALITY_SENSOR=${ENABLE_MORTALITY_SENSOR:-1}
# Numeric 0/1: consumed via `#if GNSS_HAS_BACKUP_BATTERY` in C (passing ON/OFF would evaluate to 0).
GNSS_HAS_BACKUP_BATTERY=${GNSS_HAS_BACKUP_BATTERY:-1}
# STC3117 fuel-gauge SOC strategy: 0=voltage-only (low power, default), 1=mixed (coulomb+OCV, continuous)
STC3117_MIXED_MODE=${STC3117_MIXED_MODE:-0}

# SMD satellite SPI timing profile (ON/OFF: evaluated as CMake booleans).
SMDSAT_USE_SAFE_TIMINGS=${SMDSAT_USE_SAFE_TIMINGS:-OFF}
SMDSAT_AUTOFALLBACK=${SMDSAT_AUTOFALLBACK:-ON}

# TEST ONLY: power the SMD at boot and park the nRF (no Argos TX / no I2C recovery)
# so the STM32WL can be flashed without the board resetting. Default OFF — set
# SMD_FLASH_HOLD=ON for a one-off flashing build, then rebuild OFF for normal use.
SMD_FLASH_HOLD=${SMD_FLASH_HOLD:-OFF}

# RSPB I2C pull-ups are on VSENSORS: 1 = never cut VSENSORS (floor, default/safe);
# 0 = allow it to be cut (test whether the pull-ups really depend on it).
RSPB_VSENSORS_FLOOR=${RSPB_VSENSORS_FLOOR:-1}

# TPL5111 external-wakeup period in seconds — MUST match the board's TPL5111 timing
# resistor. Sets the compiled DEFAULT (config_store.hpp); still runtime-editable via
# DTE param WAKEUP_PERIOD (PWP04).
WAKEUP_PERIOD=${WAKEUP_PERIOD:-3600}


echo "Building RSPB with configuration:"
echo "  ARGOS_SMD=${ARGOS_SMD}"
echo "  ENABLE_PRESSURE_SENSOR=${ENABLE_PRESSURE_SENSOR}"
echo "  ENABLE_AXL_SENSOR=${ENABLE_AXL_SENSOR}"
echo "  ENABLE_THERMISTOR_SENSOR=${ENABLE_THERMISTOR_SENSOR}"
echo "  ENABLE_MORTALITY_SENSOR=${ENABLE_MORTALITY_SENSOR}"
echo "  STC3117_MIXED_MODE=${STC3117_MIXED_MODE}"
echo "  GNSS_HAS_BACKUP_BATTERY=${GNSS_HAS_BACKUP_BATTERY}"
echo "  SMDSAT_USE_SAFE_TIMINGS=${SMDSAT_USE_SAFE_TIMINGS}"
echo "  SMDSAT_AUTOFALLBACK=${SMDSAT_AUTOFALLBACK}"
echo "  SMD_FLASH_HOLD=${SMD_FLASH_HOLD}"
echo "  RSPB_VSENSORS_FLOOR=${RSPB_VSENSORS_FLOOR}"
echo "  WAKEUP_PERIOD=${WAKEUP_PERIOD}s"
echo ""
if [ "${SMD_FLASH_HOLD}" = "ON" ]; then
    echo "  *** WARNING: SMD_FLASH_HOLD build — nRF parks at boot, SMD held powered."
    echo "  ***          For flashing the STM32WL ONLY. Not a normal firmware. ***"
    echo ""
fi

cmake -DCMAKE_TOOLCHAIN_FILE=../../toolchain_arm_gcc_nrf52.cmake \
      -DDEBUG_LEVEL=3 \
      -DBOARD=RSPB \
      -DCMAKE_BUILD_TYPE=${BUILD_TYPE} \
      -DARGOS_SMD=${ARGOS_SMD} \
      -DENABLE_PRESSURE_SENSOR=${ENABLE_PRESSURE_SENSOR} \
      -DENABLE_AXL_SENSOR=${ENABLE_AXL_SENSOR} \
      -DENABLE_THERMISTOR_SENSOR=${ENABLE_THERMISTOR_SENSOR} \
      -DENABLE_MORTALITY_SENSOR=${ENABLE_MORTALITY_SENSOR} \
      -DSTC3117_MIXED_MODE=${STC3117_MIXED_MODE} \
      -DGNSS_HAS_BACKUP_BATTERY=${GNSS_HAS_BACKUP_BATTERY} \
      -DSMDSAT_USE_SAFE_TIMINGS=${SMDSAT_USE_SAFE_TIMINGS} \
      -DSMDSAT_AUTOFALLBACK=${SMDSAT_AUTOFALLBACK} \
      -DSMD_FLASH_HOLD=${SMD_FLASH_HOLD} \
      -DRSPB_VSENSORS_FLOOR=${RSPB_VSENSORS_FLOOR} \
      -DWAKEUP_PERIOD_DEFAULT=${WAKEUP_PERIOD} \
      -DMETRIC_LATENCY_LOG_ENABLE=$([ "$METRICS" = "ON" ] && echo 1 || echo 0) \
      -DVALIDATION_LOG_ENABLE=$([ "$VALIDATION" = "ON" ] && echo 1 || echo 0) \
      -DBENCH_TEST=${BENCH} \
      ../..

make -j 20

# Check if build succeeded (check elf, not hex - hex might not exist if build was up-to-date)
if [ ! -f "LinkIt_RSPB_board.elf" ]; then
    echo "ERROR: Build failed - LinkIt_RSPB_board.elf not found"
    exit 1
fi

# Generate hex file if it doesn't exist (POST_BUILD doesn't run if target is up-to-date)
if [ ! -f "LinkIt_RSPB_board.hex" ]; then
    echo "Generating hex file..."
    arm-none-eabi-objcopy -O ihex LinkIt_RSPB_board.elf LinkIt_RSPB_board.hex
fi

echo ""
echo "✓ Build succeeded!"
echo ""

# Bootloader path
BOOTLOADER_HEX="../../bootloader/secure_bootloader/rspbtracker_v1.0/armgcc/_build/rspb_bootloader_v1_linkit_merged.hex"
SOFTDEVICE_HEX="../../drivers/nRF5_SDK_17.0.2/components/softdevice/s140/hex/s140_nrf52_7.2.0_softdevice.hex"
KEY_FILE="../../nrfutil_pkg_key.pem"

# Check if bootloader and required files exist for merged hex generation
CAN_MERGE=true

if [ ! -f "$BOOTLOADER_HEX" ]; then
    echo "WARNING: Bootloader not found at: $BOOTLOADER_HEX"
    echo "         Skipping merged hex generation."
    echo "         To build bootloader, run: scripts/build_rspb_bootloader.sh"
    CAN_MERGE=false
fi

if [ ! -f "$KEY_FILE" ]; then
    echo "WARNING: nrfutil key file not found at: $KEY_FILE"
    echo "         Skipping merged hex generation."
    CAN_MERGE=false
fi

if [ "$CAN_MERGE" = true ]; then
    echo "Generating settings and merged hex files..."

    # Generate settings
    nrfutil settings generate --family NRF52840 \
        --application LinkIt_RSPB_board.hex \
        --application-version 0 \
        --bootloader-version 1 \
        --bl-settings-version 2 \
        --app-boot-validation VALIDATE_ECDSA_P256_SHA256 \
        --sd-boot-validation VALIDATE_ECDSA_P256_SHA256 \
        --softdevice "$SOFTDEVICE_HEX" \
        --key-file "$KEY_FILE" \
        settings.hex

    if [ $? -eq 0 ]; then
        mergehex -m "$BOOTLOADER_HEX" LinkIt_RSPB_board.hex -o m1.hex
        mergehex -m m1.hex settings.hex -o LinkIt_RSPB_board_merged.hex
        mergehex -m LinkIt_RSPB_board.hex settings.hex -o LinkIt_RSPB_board_app_settings.hex
        rm -f m1.hex settings.hex
        echo "✓ Merged hex generated"
    else
        echo "WARNING: nrfutil settings generation failed"
        CAN_MERGE=false
    fi
fi

# Rename output files with version tag
rm -f LinkIt_RSPB_board-* LinkIt_RSPB_board_dfu-* LinkIt_RSPB_board_merged-*

# Application files (always available after successful build)
cp LinkIt_RSPB_board.elf LinkIt_RSPB_board-`cat TAG_NAME`.elf
mv LinkIt_RSPB_board.hex LinkIt_RSPB_board-`cat TAG_NAME`.hex

# Optional files (may not exist)
if [ -f "LinkIt_RSPB_board_dfu.zip" ]; then
    mv LinkIt_RSPB_board_dfu.zip LinkIt_RSPB_board_dfu-`cat TAG_NAME`.zip
    arm-none-eabi-objcopy -I ihex -O binary LinkIt_RSPB_board-`cat TAG_NAME`.hex LinkIt_RSPB_board_dfu-`cat TAG_NAME`.bin
fi
if [ -f "LinkIt_RSPB_board.img" ]; then
    mv LinkIt_RSPB_board.img LinkIt_RSPB_board-`cat TAG_NAME`.img
fi
if [ -f "LinkIt_RSPB_board_merged.hex" ]; then
    mv LinkIt_RSPB_board_merged.hex LinkIt_RSPB_board_merged-`cat TAG_NAME`.hex
fi
if [ -f "LinkIt_RSPB_board_app_settings.hex" ]; then
    mv LinkIt_RSPB_board_app_settings.hex LinkIt_RSPB_board_app_settings-`cat TAG_NAME`.hex
fi

echo ""
echo "Build complete!"
echo "Output files in: ports/nrf52840/build/RSPB/"
echo ""
echo "Files generated:"
ls -la LinkIt_RSPB_board-* 2>/dev/null || true
ls -la LinkIt_RSPB_board_dfu-* 2>/dev/null || true
ls -la LinkIt_RSPB_board_merged-* 2>/dev/null || true
ls -la LinkIt_RSPB_board_app_settings-* 2>/dev/null || true

if [ "$CAN_MERGE" = false ]; then
    echo ""
    echo "NOTE: Merged hex was not generated (see warnings above)"
fi

# Show flash command
TAG=$(cat TAG_NAME)
BUILD_DIR="ports/nrf52840/build/RSPB"
echo ""
echo "Flash commands:"
echo "  NOTE: If flashing fails with 'Access protection enabled', the device has"
echo "  readback protection (APPROTECT) active. Run 'nrfjprog --recover' first."
echo "  This erases ALL flash (firmware + config) and disables protection."
echo ""
if [ "$RECOVER" = true ]; then
    echo "  Recover (erase ALL flash + disable APPROTECT — run BEFORE flashing):"
    echo "    nrfjprog --recover -f nrf52"
    echo ""
fi
if [ "$CAN_MERGE" = true ] && [ -f "LinkIt_RSPB_board_merged-${TAG}.hex" ]; then
    echo "  Full (app + bootloader + softdevice):"
    echo "    nrfjprog --program ${BUILD_DIR}/LinkIt_RSPB_board_merged-${TAG}.hex --chiperase --verify --reset"
fi
if [ -f "LinkIt_RSPB_board_app_settings-${TAG}.hex" ]; then
    echo "  App only (bootloader + softdevice must already be flashed):"
    echo "    nrfjprog --program ${BUILD_DIR}/LinkIt_RSPB_board_app_settings-${TAG}.hex --sectorerase --verify --reset"
fi
