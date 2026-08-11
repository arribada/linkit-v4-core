#!/bin/bash

# =============================================================================
# LinkIt V4 SMD Build Script
# =============================================================================
# This script builds the LinkIt V4 firmware with SMD satellite module support.
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
    esac
done

if [ "$RECOVER" = true ]; then
    echo "NOTE: --recover flag set — recover command will be shown in flash commands below"
    echo ""
fi

# Build-type banner (see build_linkitv4_kim.sh for rationale).
if [ "$BUILD_TYPE" = "Debug" ]; then
    printf '\033[1;33m'
    echo "╔══════════════════════════════════════════════════════════════════╗"
    echo "║              BUILD TYPE: DEBUG  (g_debug_mode=USB_CDC)           ║"
    echo "║   USB CDC logs ENABLED · DEBUG_NO_WATCHDOG ENABLED · DTE active  ║"
    echo "║   NOT FOR DEPLOYMENT — for bench validation / VAL_LOG campaigns  ║"
    echo "╚══════════════════════════════════════════════════════════════════╝"
    printf '\033[0m'
else
    printf '\033[1;32m'
    echo "╔══════════════════════════════════════════════════════════════════╗"
    echo "║             BUILD TYPE: RELEASE  (g_debug_mode=NONE)             ║"
    echo "║   USB CDC silent (DEPLOY) · WDT active · DTE still enumerable    ║"
    echo "║   To enable logs: ./build_linkitv4_smd.sh --debug                ║"
    echo "╚══════════════════════════════════════════════════════════════════╝"
    printf '\033[0m'
fi
printf '\033[1;36m   Optional log flags:  METRIC_LATENCY=%s   VALIDATION=%s\033[0m\n' "$METRICS" "$VALIDATION"
echo ""

cd "$PROJECT_ROOT"
BUILD_DIR="ports/nrf52840/build/LINKIT_SMD"
if [ "$CLEAN" = true ]; then
    echo "Cleaning build directory..."
    rm -rf "$BUILD_DIR"
fi
mkdir -p "$BUILD_DIR"

# Pull remote tags so `git describe` can resolve a proper version tag.
# Quiet + no-op if offline (graceful fallback to synthetic version below).
git fetch --tags --quiet 2>/dev/null || echo "WARNING: git fetch --tags failed (offline?) — will use cached tags"

cd "$BUILD_DIR"
git show-ref --tags -d | grep ^`git rev-parse HEAD` | sed -e "s,.* refs/tags/,," -e "s/\\^{}//" > TAG_NAME
if [ -z "$(cat TAG_NAME)" ]; then
    git describe --dirty 2>/dev/null > TAG_NAME
fi
if [ -z "$(cat TAG_NAME)" ]; then
    # No tag reachable from HEAD — synthesize git-describe-style version:
    # <tag>-<commits_since_tag>-g<sha>[-dirty]
    SHA="$(git rev-parse --short HEAD)"
    COMMITS="$(git rev-list --count HEAD)"
    SYNTH="v0.0.0-${COMMITS}-g${SHA}"
    if [ -n "$(git status --porcelain)" ]; then
        SYNTH="${SYNTH}-dirty"
    fi
    echo "$SYNTH" > TAG_NAME
fi
echo "Build tag: $(cat TAG_NAME)"

# LinkIt V4 SMD build configuration
# - ARGOS_SMD=ON: Use SMD satellite module
# - ENABLE_AXL_SENSOR=ON: Enable BMA400 accelerometer
# - BATTERY_MONITOR_TYPE=ANALOG: Use nRF SAADC for battery reading
# - BATTERY_CHEMISTRY=BATT_CHEM_LS17500_2P: Discharge LUT for 2x Saft LS17500 LiSOCl2 in parallel @ 3.6V
#   Override with BATTERY_CHEMISTRY=BATT_CHEM_{S18650_2600|CGR18650_2250|NCR18650_3100_3400|LS17500_2P}
ARGOS_SMD=${ARGOS_SMD:-ON}
ENABLE_AXL_SENSOR=${ENABLE_AXL_SENSOR:-OFF}
ENABLE_SWS_LOG=${ENABLE_SWS_LOG:-OFF}
# GNSS_HAS_BACKUP_BATTERY: set ON only when V_BCKP on the M10Q is actually
# backed (coin cell / supercap). It enables the BBR fast-path reconfigure, which
# on a board with no backup supply fails its baud-sync probe every session and
# falls back to the full configure walk — pure waste.
# Default is OFF, and that is deliberate: until the CMake normalisation fix this
# script passed the literal `ON` into `#if GNSS_HAS_BACKUP_BATTERY`, which the
# preprocessor evaluated as 0. Every SMD build ever shipped therefore had the
# fast path disabled. Defaulting to OFF keeps that behaviour byte-for-byte
# instead of silently enabling it on hardware that may not have the coin cell.
GNSS_HAS_BACKUP_BATTERY=${GNSS_HAS_BACKUP_BATTERY:-OFF}
BATTERY_CHEMISTRY=${BATTERY_CHEMISTRY:-BATT_CHEM_LS17500_2P}
# SMD timing safety knobs.
#
# SMDSAT_USE_SAFE_TIMINGS — Force the v4.1.4 conservative SAFE baseline at
#   compile time. ON/OFF or 1/0 accepted. Default OFF: build uses the FAST
#   aggressive 2026-05 reductions for first-TX latency.
#
# SMDSAT_AUTOFALLBACK — Enable the runtime auto-fallback to SAFE after
#   repeated SPI errors (5 consecutive). Persists across reboot via
#   ParamID::SMD_DEGRADED_MODE; retests FAST after 20 successful TX +
#   exponential trust window (1h → 2h → ... → 24h cap). Only meaningful
#   when SMDSAT_USE_SAFE_TIMINGS=OFF. Default ON: FAST latency with
#   self-healing safety net — the recommended production profile.
#   Set to OFF for pure-FAST bench builds (no runtime branch, no persistence).
SMDSAT_USE_SAFE_TIMINGS=${SMDSAT_USE_SAFE_TIMINGS:-OFF}
SMDSAT_AUTOFALLBACK=${SMDSAT_AUTOFALLBACK:-ON}

echo "Building LinkIt V4 SMD with configuration:"
echo "  ARGOS_SMD=${ARGOS_SMD}"
echo "  ENABLE_AXL_SENSOR=${ENABLE_AXL_SENSOR}"
echo "  ENABLE_SWS_LOG=${ENABLE_SWS_LOG}"
echo "  GNSS_HAS_BACKUP_BATTERY=${GNSS_HAS_BACKUP_BATTERY}"
echo "  BATTERY_CHEMISTRY=${BATTERY_CHEMISTRY}"
echo "  SMDSAT_USE_SAFE_TIMINGS=${SMDSAT_USE_SAFE_TIMINGS}"
echo "  SMDSAT_AUTOFALLBACK=${SMDSAT_AUTOFALLBACK}"
echo ""

cmake -DCMAKE_TOOLCHAIN_FILE=../../toolchain_arm_gcc_nrf52.cmake \
      -DDEBUG_LEVEL=3 \
      -DBOARD=LINKIT \
      -DCMAKE_BUILD_TYPE=${BUILD_TYPE} \
      -DARGOS_SMD=${ARGOS_SMD} \
      -DENABLE_AXL_SENSOR=${ENABLE_AXL_SENSOR} \
      -DENABLE_SWS_LOG=${ENABLE_SWS_LOG} \
      -DGNSS_HAS_BACKUP_BATTERY=${GNSS_HAS_BACKUP_BATTERY} \
      -DBATTERY_CHEMISTRY=${BATTERY_CHEMISTRY} \
      -DSMDSAT_USE_SAFE_TIMINGS=${SMDSAT_USE_SAFE_TIMINGS} \
      -DSMDSAT_AUTOFALLBACK=${SMDSAT_AUTOFALLBACK} \
      -DMETRIC_LATENCY_LOG_ENABLE=$([ "$METRICS" = "ON" ] && echo 1 || echo 0) \
      -DVALIDATION_LOG_ENABLE=$([ "$VALIDATION" = "ON" ] && echo 1 || echo 0) \
      ../..

make -j 20

TARGET_NAME="LinkIt_board"

# Check if build succeeded
if [ ! -f "${TARGET_NAME}.elf" ]; then
    echo "ERROR: Build failed - ${TARGET_NAME}.elf not found"
    exit 1
fi

# Generate hex file if it doesn't exist (POST_BUILD doesn't run if target is up-to-date)
if [ ! -f "${TARGET_NAME}.hex" ]; then
    echo "Generating hex file..."
    arm-none-eabi-objcopy -O ihex ${TARGET_NAME}.elf ${TARGET_NAME}.hex
fi

echo ""
echo "✓ Build succeeded!"
echo ""

# Bootloader path
BOOTLOADER_HEX="../../bootloader/secure_bootloader/linkitv4_v1.0/armgcc/_build/cls_bootloader_v1_linkit_merged.hex"
SOFTDEVICE_HEX="../../drivers/nRF5_SDK_17.0.2/components/softdevice/s140/hex/s140_nrf52_7.2.0_softdevice.hex"
KEY_FILE="../../nrfutil_pkg_key.pem"

# Check if bootloader and required files exist for merged hex generation
CAN_MERGE=true

if [ ! -f "$BOOTLOADER_HEX" ]; then
    echo "WARNING: Bootloader not found at: $BOOTLOADER_HEX"
    echo "         Skipping merged hex generation."
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
        --application ${TARGET_NAME}.hex \
        --application-version 0 \
        --bootloader-version 1 \
        --bl-settings-version 2 \
        --app-boot-validation VALIDATE_ECDSA_P256_SHA256 \
        --sd-boot-validation VALIDATE_ECDSA_P256_SHA256 \
        --softdevice "$SOFTDEVICE_HEX" \
        --key-file "$KEY_FILE" \
        settings.hex

    if [ $? -eq 0 ]; then
        mergehex -m "$BOOTLOADER_HEX" ${TARGET_NAME}.hex -o m1.hex
        mergehex -m m1.hex settings.hex -o ${TARGET_NAME}_merged.hex
        mergehex -m ${TARGET_NAME}.hex settings.hex -o ${TARGET_NAME}_app_settings.hex
        rm -f m1.hex settings.hex
        echo "✓ Merged hex generated"
    else
        echo "WARNING: nrfutil settings generation failed"
        CAN_MERGE=false
    fi
fi

# Rename output files with version tag
rm -f ${TARGET_NAME}-* ${TARGET_NAME}_dfu-* ${TARGET_NAME}_merged-*

# Application files (always available after successful build)
cp ${TARGET_NAME}.elf ${TARGET_NAME}-`cat TAG_NAME`.elf
mv ${TARGET_NAME}.hex ${TARGET_NAME}-`cat TAG_NAME`.hex

# Optional files (may not exist)
if [ -f "${TARGET_NAME}_dfu.zip" ]; then
    mv ${TARGET_NAME}_dfu.zip ${TARGET_NAME}_dfu-`cat TAG_NAME`.zip
    arm-none-eabi-objcopy -I ihex -O binary ${TARGET_NAME}-`cat TAG_NAME`.hex ${TARGET_NAME}_dfu-`cat TAG_NAME`.bin
fi
if [ -f "${TARGET_NAME}.img" ]; then
    mv ${TARGET_NAME}.img ${TARGET_NAME}-`cat TAG_NAME`.img
fi
if [ -f "${TARGET_NAME}_merged.hex" ]; then
    mv ${TARGET_NAME}_merged.hex ${TARGET_NAME}_merged-`cat TAG_NAME`.hex
fi
if [ -f "${TARGET_NAME}_app_settings.hex" ]; then
    mv ${TARGET_NAME}_app_settings.hex ${TARGET_NAME}_app_settings-`cat TAG_NAME`.hex
fi

echo ""
echo "Build complete!"
echo "Output files in: ports/nrf52840/build/LINKIT_SMD/"
echo ""
echo "Files generated:"
ls -la ${TARGET_NAME}-* 2>/dev/null || true
ls -la ${TARGET_NAME}_dfu-* 2>/dev/null || true
ls -la ${TARGET_NAME}_merged-* 2>/dev/null || true
ls -la ${TARGET_NAME}_app_settings-* 2>/dev/null || true

if [ "$CAN_MERGE" = false ]; then
    echo ""
    echo "NOTE: Merged hex was not generated (see warnings above)"
fi

# Show flash command
TAG=$(cat TAG_NAME)
BUILD_DIR="ports/nrf52840/build/LINKIT_SMD"
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
if [ "$CAN_MERGE" = true ] && [ -f "${TARGET_NAME}_merged-${TAG}.hex" ]; then
    echo "  Full (app + bootloader + softdevice):"
    echo "    nrfjprog --program ${BUILD_DIR}/${TARGET_NAME}_merged-${TAG}.hex --chiperase --verify --reset"
fi
if [ -f "${TARGET_NAME}_app_settings-${TAG}.hex" ]; then
    echo "  App only (bootloader + softdevice must already be flashed):"
    echo "    nrfjprog --program ${BUILD_DIR}/${TARGET_NAME}_app_settings-${TAG}.hex --sectorerase --verify --reset"
fi
