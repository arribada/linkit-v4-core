#!/usr/bin/env bash
#
# flash.sh — flash the LinkIt V4 KIM bench firmware over SWD (J-Link / nrfjprog).
#
#   ./flash.sh            app-only, fast (sectorerase; keeps bootloader+SoftDevice)
#   ./flash.sh --full     full merged image (chiperase app+BL+SD) — use for 1st flash
#   ./flash.sh --recover  unlock APPROTECT (erases ALL flash) then full flash
#
# Runs from WSL against a J-Link attached via usbipd (see wsl_usb.sh). Assumes the
# firmware was built with:  ./scripts/build_linkitv4_kim.sh --bench
#
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
BD="$REPO/ports/nrf52840/build/LINKIT"

MODE="app"
for a in "$@"; do
    case "$a" in
        --full) MODE="full" ;;
        --recover) MODE="recover" ;;
    esac
done

latest() { ls -t "$BD"/$1 2>/dev/null | head -1; }

MERGED="$(latest 'LinkIt_board_merged-*.hex')"
APPSET="$(latest 'LinkIt_board_app_settings-*.hex')"

if ! command -v nrfjprog >/dev/null; then
    echo "nrfjprog not found on PATH." >&2; exit 1
fi

# Confirm a probe is visible before doing anything destructive.
if ! nrfjprog --ids >/dev/null 2>&1 || [ -z "$(nrfjprog --ids 2>/dev/null)" ]; then
    echo "No J-Link / debug probe visible to nrfjprog." >&2
    echo "Under WSL2, attach it first:  tests/bench/wsl_usb.sh attach <busid>" >&2
    exit 2
fi

do_recover() {
    echo ">> nrfjprog --recover (erases ALL flash, clears APPROTECT)"
    nrfjprog --recover -f nrf52
}

flash_full() {
    [ -n "$MERGED" ] || { echo "No merged hex — run build_linkitv4_kim.sh --bench" >&2; exit 3; }
    echo ">> Full flash: $(basename "$MERGED")"
    nrfjprog --program "$MERGED" --chiperase --verify --reset
}

flash_app() {
    [ -n "$APPSET" ] || { echo "No app_settings hex — run build_linkitv4_kim.sh --bench" >&2; exit 3; }
    echo ">> App flash: $(basename "$APPSET")"
    nrfjprog --program "$APPSET" --sectorerase --verify --reset
}

case "$MODE" in
    recover) do_recover; flash_full ;;
    full)    flash_full ;;
    app)
        if ! flash_app; then
            echo "App flash failed — retrying full (chiperase)…" >&2
            if ! flash_full; then
                echo "Full flash failed — attempting recover…" >&2
                do_recover; flash_full
            fi
        fi
        ;;
esac

echo ">> Done. Board reset. Bench console live on USB-CDC."
