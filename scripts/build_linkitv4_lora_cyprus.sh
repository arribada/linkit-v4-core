#!/bin/bash

# =============================================================================
# Rewild Cyprus — LinkIt V4 LoRa boat tracker build
# =============================================================================
# Thin wrapper over build_linkitv4_lora.sh. It does NOT duplicate the build
# logic: it pins the deployment-specific options and delegates, so a future
# change to the shared LoRa script is inherited here automatically — except for
# the values pinned below, which is the whole point of having this file.
#
# SIX firmware options differ from the standard LoRa build. Every one of them
# is load-bearing; none may change silently if someone edits the defaults in
# build_linkitv4_lora.sh. Each line below states the shared default it overrides.
#
#   BATTERY_CHEMISTRY  (shared default: BATT_CHEM_LS17500_2P)
#   Cyprus runs solar + 1S LiPo, whose discharge curve spans 3200-4200 mV. The
#   shared default is the Li-SOCl2 LUT for the turtle trackers, 2700-3700 mV.
#   Using it on a LiPo saturates the gauge at 100 % on a charged pack and puts
#   the LB_THRESHOLD / LB_CRITICAL trip points on the wrong part of the curve.
#   See core/hardware/nrf_battery_mon.cpp.
#
#   LORA_DCS_ENABLE=ON  (shared default: OFF)
#   ETSI EN 300 220 duty-cycle enforcement (AT+DCS=1). Cyprus is in the EU;
#   shipping with DCS off is not legal. The shared script keeps OFF only to stay
#   byte-identical for the existing LoRa turtles, and points here for the
#   deployments that must opt in.
#
#   GNSS_HAS_BACKUP_BATTERY=OFF  (shared default: ON)
#   The Cyprus boards have no V_BCKP coin cell, so the M10Q BBR fast-path
#   reconfigure would fail its baud probe every session and fall back to a full
#   configure anyway. Set ON only on a variant that actually fits the backup
#   supply.
#
#   ENABLE_AXL_SENSOR=ON  (shared default: OFF)
#   Required by moored-vs-underway mode. The BMA400 is the cheap sentinel
#   between two GNSS points: ~3.5 uA with a hardware GEN1 wake-on-motion
#   interrupt, feeding MooredModeService — whose motion-driven exit kicks an
#   immediate GNSS acquisition itself, so GNP26 stays off. This flag is NOT
#   cosmetic — it compiles in the accelerometer branch of the peer-event
#   funnel in ServiceManager::notify_peer_event, the wake-up consumer in
#   GPSService::service_is_triggered_on_event, and the AXP* parameters
#   themselves (is_implemented). With it OFF, moored mode still works but has no
#   sentinel: the only way out is the next scheduled fix, up to MOORED_DLOC away.
#
#   Checked, since the step-1/2 comment used to warn about it: enabling the AXL
#   takes the largest sensor packet from 106 bits (14 B) to 173 bits (22 B),
#   still far below the 51 B threshold at which lora_rak3172.cpp bumps the data
#   rate. LORA_DR is NOT overridden. Keep AXP05 (AXL_SENSOR_ENABLE_TX_MODE) at
#   OFF: the accelerometer is a sentinel here, not a payload.
#
#   LORA_MOTION_EXT=ON  (shared default: OFF)
#   Appends the 4-byte MOTION debug block to every frame but CloudLocate:
#   moored/underway, AXL hold-off, accelerometer wake-ups since the last
#   transmitted frame, minutes since the last wake-up and since the last
#   accelerometer-driven MOORED exit. Debug telemetry for tuning moored mode:
#   it reads no sensor and changes no decision. GPS_MULTI x3 goes from 30 to
#   34 B (SF9 airtime 287.7 -> 308.2 ms); GPS frames are sized against the DR
#   limit minus the block, so they never outgrow it. The ChirpStack codec must
#   know the block: layout in LoRaPacketBuilder::append_motion_ext and wiki
#   page 12.
#
#   LORA_LINKCHECK=ON  (shared default: OFF)
#   Every uplink carries a LinkCheckReq, and a depth-pile position is spent
#   only once a gateway heard a frame carrying it. Out of coverage nothing is
#   spent; the pile keeps the newest ARGOS_DEPTH_PILE positions (24 max: 2 h
#   under way at 5 min, a day moored) and sends them as soon as a frame is
#   heard again. Cost: one FOpts byte and one downlink per uplink (gateway RX1
#   1 %, or RX2 10 % if ChirpStack answers there). Device side: ARP16=12 and
#   LBP08=12 (DTE depth-pile code 12 = 24 positions), ARP19=3, LBP11=3.
#   Image without it, same options otherwise:
#     LORA_LINKCHECK=OFF ./scripts/build_linkitv4_lora_cyprus.sh --clean
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
export LORA_DCS_ENABLE=ON
export GNSS_HAS_BACKUP_BATTERY=OFF
export ENABLE_AXL_SENSOR=ON
export LORA_MOTION_EXT=ON
export LORA_LINKCHECK=${LORA_LINKCHECK:-ON}

# LORA_TX_ERROR_SUSPEND_S is deliberately NOT pinned here: the default artifact
# of this script must be the deployment-safe one (shared default 3600 s — after
# three consecutive device errors, TX suspends for an hour, then one probe).
# Gateway down on that regime costs ~one join window per hour; without the
# suspension it is one attempt per capped backoff (10 min), six times the
# radio-on time, forever. The boat has no surface events to clear the counter,
# so the probe is its only way back — which the suspension provides.
#
# Commissioning, where an hour between retries makes iteration impossible:
#   ./scripts/build_linkitv4_lora_cyprus.sh --clean --debug --no-tx-suspend
# (--no-tx-suspend sets LORA_TX_ERROR_SUSPEND_S=0; the capped backoff still
# bounds retries to one per 10 min.)

exec "$SCRIPT_DIR/build_linkitv4_lora.sh" "$@"
