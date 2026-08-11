#!/bin/bash

# =============================================================================
# Rewild Cyprus — LinkIt V4 LoRa boat tracker build
# =============================================================================
# Thin wrapper over build_linkitv4_lora.sh. It does NOT duplicate the build
# logic: it pins the deployment-specific options and delegates, so a future
# change to the shared LoRa script is inherited here automatically — except for
# the values pinned below, which is the whole point of having this file.
#
# Exactly ONE firmware option genuinely differs from the standard LoRa build:
#
#   BATTERY_CHEMISTRY — Cyprus runs solar + 1S LiPo, whose discharge curve spans
#   3200-4200 mV. The LoRa/SMD default (BATT_CHEM_LS17500_2P) is the Li-SOCl2
#   LUT for the turtle trackers, 2700-3700 mV. Using it on a LiPo saturates the
#   gauge at 100 % on a charged pack and puts the LB_THRESHOLD / LB_CRITICAL
#   trip points on the wrong part of the curve. See core/hardware/nrf_battery_mon.cpp.
#
# The other three are already the shared defaults. They are restated explicitly
# because they are load-bearing for this deployment and must not change silently
# if someone edits the defaults in build_linkitv4_lora.sh:
#
#   DISABLE_LORA_DCS=OFF     -> LORA_DCS_ENABLE=ON. ETSI EN 300 220 duty-cycle
#                               enforcement. Cyprus is in the EU; shipping with
#                               DCS off is not legal.
#   GNSS_HAS_BACKUP_BATTERY  -> OFF. The Cyprus boards have no V_BCKP coin cell,
#                               so the M10Q BBR fast-path reconfigure would fail
#                               its baud probe every session and fall back to a
#                               full configure anyway. Set ON only on a variant
#                               that actually fits the backup supply.
#   ENABLE_AXL_SENSOR=OFF    -> Step 1 and step 2 do not use the BMA400, and
#                               leaving it out keeps the largest sensor packet at
#                               14 B so the driver's DR auto-bump never overrides
#                               LORA_DR. Set ON for the step-3 movement work.
#
# Runtime configuration (ARGOS_MODE, ARP11, GNP52, LoRaWAN credentials, DR, ...)
# is NOT set here — it is provisioned device-side over DTE. See the deployment
# plan, part E.
#
# Usage: same flags as build_linkitv4_lora.sh, e.g.
#   ./scripts/build_linkitv4_lora_cyprus.sh --clean --debug
# --debug is what enables the USB CDC console; without it the bench is silent.
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "═══════════════════════════════════════════════════════════════════"
echo "  Rewild Cyprus — boat tracker (LinkIt V4 + RAK3172, EU868)"
echo "═══════════════════════════════════════════════════════════════════"
echo ""

# Own build directory: sharing LINKIT_LORA's CMake cache would mean whichever
# script ran last decides what a bare `make` produces.
export LORA_BUILD_SUBDIR=LINKIT_CYPRUS

export BATTERY_CHEMISTRY=BATT_CHEM_NCR18650_3100_3400
export DISABLE_LORA_DCS=OFF
export GNSS_HAS_BACKUP_BATTERY=OFF
export ENABLE_AXL_SENSOR=OFF

exec "$SCRIPT_DIR/build_linkitv4_lora.sh" "$@"
