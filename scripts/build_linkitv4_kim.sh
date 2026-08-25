#!/bin/bash

# =============================================================================
# LinkIt Core Build Script
# =============================================================================
# This script builds the LinkIt Core firmware.
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
WKUP_RELEASE=OFF
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
        # Bench-test harness: '%' USB-CDC console (%CFG/%OP/%GPS) + GPS injection
        # for autonomous HIL validation. Implies --debug (needs logs + no WDT).
        --bench) BENCH=ON; BUILD_TYPE=Debug ;;
        # KIM2 + BLIND : relâcher SAT_EXTWAKEUP pendant la salve pilotée par le
        # module, pour qu'il descende en basse consommation entre ses
        # retransmissions. Défaut OFF = pin maintenue haute (comportement d'avant).
        --wkup-release) WKUP_RELEASE=ON ;;
    esac
done

if [ "$RECOVER" = true ]; then
    echo "NOTE: --recover flag set — recover command will be shown in flash commands below"
    echo ""
fi

# Build-type banner: highly visible because the silent default is Release
# (NDEBUG → g_debug_mode=NONE → no USB CDC logs in the field), and operators
# often confuse the absence of logs with a broken device. See pre-deploy
# protocol + main.cpp:185-191.
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
    echo "║   To enable logs: ./build_linkitv4_kim.sh --debug                ║"
    echo "╚══════════════════════════════════════════════════════════════════╝"
    printf '\033[0m'
fi
printf '\033[1;36m   Optional log flags:  METRIC_LATENCY=%s   VALIDATION=%s\033[0m\n' "$METRICS" "$VALIDATION"
if [ "$BENCH" = "ON" ]; then
    printf '\033[1;35m   BENCH_TEST=ON  →  %% USB-CDC console (%%CFG/%%OP/%%GPS) + GPS injection active\033[0m\n'
fi
echo ""

cd "$PROJECT_ROOT"
BUILD_DIR="ports/nrf52840/build/LINKIT"
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
# Default to both disabled (0)
CAM_ENABLE=${CAM_ENABLE:-1}
BUZZER_ENABLE=${BUZZER_ENABLE:-0}

# Battery configuration (params previously absent from this KIM script vs the SMD one).
# LinkIt uses the ANALOG/SAADC battery monitor, so BATTERY_CHEMISTRY selects the
# discharge LUT. Classic Li-ion 18650 (3.2–4.2 V). Override with one of:
#   BATT_CHEM_S18650_2600 | BATT_CHEM_CGR18650_2250 |
#   BATT_CHEM_NCR18650_3100_3400 (default) | BATT_CHEM_LS17500_2P (LiSOCl2 primary).
BATTERY_CHEMISTRY=${BATTERY_CHEMISTRY:-BATT_CHEM_NCR18650_3100_3400}
# GNSS backup (BBR) battery present? Consumed as `#if GNSS_HAS_BACKUP_BATTERY` in C,
# so it MUST be numeric 0/1 — ON/OFF would BOTH evaluate to 0. 0 = no backup battery.
GNSS_HAS_BACKUP_BATTERY=${GNSS_HAS_BACKUP_BATTERY:-0}
# SWS analog underwater-detector service. LinkIt has the SWS ADC channel, so the
# service is compiled in by default (CMake default = 1 on LINKIT). MUST be numeric
# 0/1 (consumed as `#if ENABLE_SWS_ANALOG` in C — ON/OFF would both evaluate to 0).
# Runtime detection is still gated by UNDERWATER_EN (UNP01); this flag only controls
# whether the service is compiled/instantiated at all.
ENABLE_SWS_ANALOG=${ENABLE_SWS_ANALOG:-1}

echo "  CAM_ENABLE=${CAM_ENABLE}  BUZZER_ENABLE=${BUZZER_ENABLE}"
echo "  BATTERY_CHEMISTRY=${BATTERY_CHEMISTRY}"
echo "  GNSS_HAS_BACKUP_BATTERY=${GNSS_HAS_BACKUP_BATTERY}"
echo "  ENABLE_SWS_ANALOG=${ENABLE_SWS_ANALOG}"
echo "  METRIC_LATENCY=${METRICS}  VALIDATION=${VALIDATION}"

cmake -DCMAKE_TOOLCHAIN_FILE=../../toolchain_arm_gcc_nrf52.cmake -DDEBUG_LEVEL=3 -DBOARD=LINKIT -DCMAKE_BUILD_TYPE=${BUILD_TYPE} -DCAM_ENABLE=${CAM_ENABLE} -DBUZZER_ENABLE=${BUZZER_ENABLE} -DGNSS_HAS_BACKUP_BATTERY=${GNSS_HAS_BACKUP_BATTERY} -DBATTERY_CHEMISTRY=${BATTERY_CHEMISTRY} -DENABLE_SWS_ANALOG=${ENABLE_SWS_ANALOG} -DMETRIC_LATENCY_LOG_ENABLE=$([ "$METRICS" = "ON" ] && echo 1 || echo 0) -DVALIDATION_LOG_ENABLE=$([ "$VALIDATION" = "ON" ] && echo 1 || echo 0) -DBENCH_TEST=${BENCH} -DKIM2_BLIND_WKUP_RELEASE=${WKUP_RELEASE} ../..
make -j 20
# GARDE-FOU (2026-08): sans ce controle, un echec de compilation laissait le
# script continuer, re-fusionner le HEX de la build PRECEDENTE et afficher des
# commandes de flash parfaitement valides pointant sur un binaire perime. C'est
# exactement ce qui s'est produit au banc (firmware sans console %BENCH flashe
# sans que rien ne le signale).
if [ $? -ne 0 ]; then
    echo ""
    echo "  *** COMPILATION ECHOUEE — aucun binaire produit, rien a flasher ***"
    echo ""
    exit 1
fi
nrfutil settings generate --family NRF52840 --application LinkIt_board.hex --application-version 0 --bootloader-version 1 --bl-settings-version 2 --app-boot-validation VALIDATE_ECDSA_P256_SHA256 --sd-boot-validation VALIDATE_ECDSA_P256_SHA256 --softdevice ../../drivers/nRF5_SDK_17.0.2/components/softdevice/s140/hex/s140_nrf52_7.2.0_softdevice.hex --key-file ../../nrfutil_pkg_key.pem settings.hex
mergehex -m ../../bootloader/secure_bootloader/linkitv4_v1.0/armgcc/_build/cls_bootloader_v1_linkit_merged.hex LinkIt_board.hex -o m1.hex
mergehex -m m1.hex settings.hex -o LinkIt_board_merged.hex
mergehex -m LinkIt_board.hex settings.hex -o LinkIt_board_app_settings.hex
rm -f m1.hex settings.hex
rm -f LinkIt_board-* LinkIt_board_dfu-* LinkIt_board_merged-*
mv LinkIt_board_dfu.zip LinkIt_board_dfu-`cat TAG_NAME`.zip
arm-none-eabi-objcopy -I ihex -O binary LinkIt_board-`cat TAG_NAME`.hex LinkIt_board_dfu-`cat TAG_NAME`.bin
cp LinkIt_board.elf LinkIt_board-`cat TAG_NAME`.elf
mv LinkIt_board.hex LinkIt_board-`cat TAG_NAME`.hex
mv LinkIt_board.img LinkIt_board-`cat TAG_NAME`.img
mv LinkIt_board_merged.hex LinkIt_board_merged-`cat TAG_NAME`.hex
if [ -f "LinkIt_board_app_settings.hex" ]; then
    mv LinkIt_board_app_settings.hex LinkIt_board_app_settings-`cat TAG_NAME`.hex
fi

# Show flash command
TAG=$(cat TAG_NAME)
BUILD_DIR="ports/nrf52840/build/LINKIT"
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
if [ -f "LinkIt_board_merged-${TAG}.hex" ]; then
    echo "  Full (app + bootloader + softdevice):"
    echo "    nrfjprog --program ${BUILD_DIR}/LinkIt_board_merged-${TAG}.hex --chiperase --verify --reset"
fi
if [ -f "LinkIt_board_app_settings-${TAG}.hex" ]; then
    echo "  App only (bootloader + softdevice must already be flashed):"
    echo "    nrfjprog --program ${BUILD_DIR}/LinkIt_board_app_settings-${TAG}.hex --sectorerase --verify --reset"
fi