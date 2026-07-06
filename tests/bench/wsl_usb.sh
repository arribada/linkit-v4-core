#!/usr/bin/env bash
#
# wsl_usb.sh — bridge the board's USB-CDC and the J-Link SWD probe into WSL2.
#
# WSL2 has no native USB; devices must be attached from Windows via usbipd-win.
#   https://github.com/dorssel/usbipd-win  (install once on Windows:  winget install usbipd)
#
#   ./wsl_usb.sh list                 show all USB devices + busids + state
#   ./wsl_usb.sh attach <busid>       attach one device to WSL (e.g. 2-4)
#   ./wsl_usb.sh auto                 attach anything that looks like a J-Link or nRF CDC
#
# First-time attach of a given device needs a one-off "bind" with admin rights on
# Windows:  (elevated PowerShell)  usbipd bind --busid <busid>
# This script prints that hint if attach fails.
#
set -euo pipefail

USBIPD="$(command -v usbipd.exe || echo '/mnt/c/Program Files/usbipd-win/usbipd.exe')"
if [ ! -x "$USBIPD" ] && ! command -v usbipd.exe >/dev/null; then
    echo "usbipd.exe not found. Install usbipd-win on Windows (winget install usbipd)." >&2
    exit 1
fi

cmd="${1:-list}"

case "$cmd" in
    list)
        "$USBIPD" list
        ;;
    attach)
        busid="${2:?usage: wsl_usb.sh attach <busid>}"
        echo ">> attaching busid $busid to WSL…"
        if ! "$USBIPD" attach --wsl --busid "$busid"; then
            echo "" >&2
            echo "attach failed. If it says 'not shared', bind it once from an" >&2
            echo "ELEVATED PowerShell on Windows:" >&2
            echo "    usbipd bind --busid $busid" >&2
            echo "then re-run this attach." >&2
            exit 2
        fi
        sleep 1
        echo ">> now visible in WSL as:"; ls -1 /dev/ttyACM* 2>/dev/null || true
        ;;
    keep)
        busid="${2:?usage: wsl_usb.sh keep <busid>}"
        echo ">> auto-attach $busid to WSL (survives flash resets; Ctrl-C to stop)…"
        # --auto-attach re-attaches automatically whenever the device re-enumerates
        # (every flash resets the nRF USB). Device must already be 'Shared' (bound).
        exec "$USBIPD" attach --wsl --busid "$busid" --auto-attach
        ;;
    auto)
        echo ">> scanning for J-Link / nRF CDC devices…"
        # Match SEGGER J-Link and Nordic/CDC entries in the usbipd list output.
        "$USBIPD" list | grep -iE "j-link|jlink|segger|nordic|cdc|nrf|linkit" || {
            echo "nothing obvious found — run './wsl_usb.sh list' and attach by busid"; exit 0; }
        echo ""
        echo "Attach each relevant busid above with:  ./wsl_usb.sh attach <busid>"
        ;;
    *)
        echo "usage: wsl_usb.sh {list|attach <busid>|auto}" >&2
        exit 1
        ;;
esac
