#!/usr/bin/env python3
"""
Test CloudLocate — decode Argos CloudLocate packets and query u-blox CloudLocate API.

Usage:
  # Decode an Argos packet from logs (the hex after "data=")
  python3 test_cloudlocate.py --decode E9B90A1CF32B78F1CA036B1C8CD296FF4D1

  # Decode and send to CloudLocate API for positioning
  python3 test_cloudlocate.py --decode E9B90A1CF32B78F1CA036B1C8CD296FF4D1 --locate

  # Send a raw MEAS20 blob directly (hex, 20 bytes = 40 hex chars)
  python3 test_cloudlocate.py --raw-blob <40_hex_chars> --format meas20 --locate

  # Use a custom token (a default is inlined below)
  python3 test_cloudlocate.py --decode <hex> --locate --token "QkVCQzNBMEUyODozRjkwMzMzNg=="
"""

import argparse
import base64
import json
import sys

try:
    import requests
    HAS_REQUESTS = True
except ImportError:
    HAS_REQUESTS = False

# Default token (previously quoted from a template under template_conf/)
DEFAULT_TOKEN_B64 = "QkVCQzNBMEUyODozRjkwMzMzNg=="

# Packet constants (from argos_packet_builder.hpp)
HEADER_BITS = 3
FORMAT_BITS = 2
BATTERY_BITS = 7
LOW_BATT_BITS = 1
CLOUDLOCATE_HEADER = 0b111  # Type 7

REF_BATT_MV = 2700
MV_PER_UNIT = 20


def hex_to_bits(hex_str: str) -> str:
    """Convert hex string to binary string."""
    # Pad hex to even length
    if len(hex_str) % 2:
        hex_str = hex_str + "0"
    # Validate hex characters
    try:
        raw = bytes.fromhex(hex_str)
    except ValueError as e:
        print(f"ERROR: invalid hex string: {e}")
        sys.exit(1)
    return "".join(f"{b:08b}" for b in raw)


def extract_bits(bits: str, offset: int, length: int) -> int:
    """Extract integer value from bit string at given offset."""
    return int(bits[offset:offset + length], 2)


def decode_argos_cloudlocate_packet(hex_data: str) -> dict:
    """
    Decode an Argos CloudLocate packet (from firmware logs).

    Bit layout (MSB-first):
      [0:3]   Header      — 3 bits, must be 0b111 (type 7)
      [3:5]   Format ID   — 2 bits, 0=MEASC12 (12B), 1=MEAS20 (20B)
      [5:5+N] Blob        — N*8 bits, raw GNSS measurement
      [...]   Battery     — 7 bits, (val * 20 + 2700) mV
      [...]   Low battery — 1 bit
      [...]   Padding     — zero-fill to byte boundary
    """
    bits = hex_to_bits(hex_data)
    pos = 0

    # Header
    header = extract_bits(bits, pos, HEADER_BITS)
    pos += HEADER_BITS
    if header != CLOUDLOCATE_HEADER:
        print(f"WARNING: header={header:#05b}, expected {CLOUDLOCATE_HEADER:#05b} (type 7)")
        print(f"         This may not be a CloudLocate packet (could be type {header})")

    # Format ID
    format_id = extract_bits(bits, pos, FORMAT_BITS)
    pos += FORMAT_BITS
    format_name = {0: "MEASC12", 1: "MEAS20", 2: "MEAS50"}.get(format_id, f"UNKNOWN({format_id})")
    blob_size = {0: 12, 1: 20, 2: 50}.get(format_id, 0)

    if blob_size == 0:
        print(f"ERROR: unknown format_id={format_id}")
        return {}

    # Check we have enough bits for the full packet
    expected_bits = HEADER_BITS + FORMAT_BITS + blob_size * 8 + BATTERY_BITS + LOW_BATT_BITS
    if len(bits) < expected_bits:
        print(f"WARNING: packet too short ({len(bits)} bits, need {expected_bits})")
        print(f"         Expected {expected_bits // 8 + (1 if expected_bits % 8 else 0)} bytes = "
              f"{(expected_bits + 7) // 4} hex chars, got {len(bits) // 4} hex chars")
        print(f"         Log output may be truncated — copy the full 'data=' hex value")
        # Try to extract what we can
        available_blob_bytes = min(blob_size, (len(bits) - pos) // 8)
        print(f"         Extracting {available_blob_bytes}/{blob_size} blob bytes available")
        blob_size = available_blob_bytes

    # Blob
    blob_bytes = []
    for i in range(blob_size):
        b = extract_bits(bits, pos, 8)
        blob_bytes.append(b)
        pos += 8
    blob_hex = bytes(blob_bytes).hex().upper()

    # Battery (if enough bits remain)
    batt_raw = 0
    batt_mv = 0
    batt_v = 0.0
    low_batt = 0
    if pos + BATTERY_BITS + LOW_BATT_BITS <= len(bits):
        batt_raw = extract_bits(bits, pos, BATTERY_BITS)
        pos += BATTERY_BITS
        batt_mv = batt_raw * MV_PER_UNIT + REF_BATT_MV
        batt_v = batt_mv / 1000.0
        low_batt = extract_bits(bits, pos, LOW_BATT_BITS)
        pos += LOW_BATT_BITS
    else:
        print("WARNING: not enough bits for battery/low_batt fields (truncated packet)")

    result = {
        "header": header,
        "format_id": format_id,
        "format_name": format_name,
        "blob_size": blob_size,
        "blob_hex": blob_hex,
        "battery_raw": batt_raw,
        "battery_mv": batt_mv,
        "battery_v": batt_v,
        "low_battery": bool(low_batt),
        "total_bits_used": pos,
        "total_bits_packet": len(bits),
        "padding_bits": len(bits) - pos,
    }

    return result


def print_decoded(info: dict):
    """Pretty-print decoded packet info."""
    print("=" * 60)
    print("  Argos CloudLocate Packet Decoded")
    print("=" * 60)
    print(f"  Header:       {info['header']:#05b} (type {info['header']})")
    print(f"  Format:       {info['format_name']} ({info['blob_size']} bytes)")
    print(f"  Blob (hex):   {info['blob_hex']}")
    print(f"  Battery:      {info['battery_v']:.2f} V ({info['battery_mv']} mV)")
    print(f"  Low battery:  {info['low_battery']}")
    print(f"  Bits used:    {info['total_bits_used']} / {info['total_bits_packet']} ({info['padding_bits']} padding)")
    print("=" * 60)


def query_cloudlocate(blob_hex: str, format_name: str, token_b64: str) -> dict:
    """
    Send raw GNSS measurement to u-blox CloudLocate API.

    The token is base64-encoded "UID:TOKEN" string used for AssistNow/CloudLocate auth.
    """
    if not HAS_REQUESTS:
        print("ERROR: 'requests' module not installed. Run: pip install requests")
        sys.exit(1)

    # Decode token from base64
    token_decoded = base64.b64decode(token_b64).decode("ascii")
    print(f"  Token (decoded): {token_decoded}")

    # CloudLocate API endpoint
    # See: https://developer.thingstream.io/guides/location-services/cloudlocate-getting-started
    url = "https://api.services.u-blox.com/cloudlocate/v1/request"

    headers = {
        "Content-Type": "application/json",
    }

    # The blob must be base64-encoded for the API
    blob_bytes = bytes.fromhex(blob_hex)
    blob_b64 = base64.b64encode(blob_bytes).decode("ascii")

    # Build request body
    payload = {
        "token": token_decoded,
        "body": {
            "meas": [{
                "data": blob_b64,
                "format": format_name.lower(),
            }]
        }
    }

    print(f"\n  Sending to CloudLocate API...")
    print(f"  URL:    {url}")
    print(f"  Format: {format_name}")
    print(f"  Blob:   {blob_hex}")
    print(f"  Blob (base64): {blob_b64}")

    try:
        resp = requests.post(url, headers=headers, json=payload, timeout=30)
        print(f"  Status: {resp.status_code}")

        if resp.status_code == 200:
            result = resp.json()
            print(f"\n  CloudLocate Result:")
            print(f"  {json.dumps(result, indent=4)}")

            if "lat" in result and "lon" in result:
                lat = result["lat"]
                lon = result["lon"]
                acc = result.get("accuracy", "?")
                print(f"\n  Position: lat={lat:.6f} lon={lon:.6f} accuracy={acc}m")
                print(f"  Google Maps: https://maps.google.com/?q={lat},{lon}")
            return result
        else:
            print(f"  Error response: {resp.text}")
            return {"error": resp.text, "status": resp.status_code}

    except requests.exceptions.RequestException as e:
        print(f"  Request failed: {e}")
        return {"error": str(e)}


def main():
    parser = argparse.ArgumentParser(
        description="Decode and test LinkIt V4 CloudLocate Argos packets",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument("--decode", metavar="HEX",
                        help="Decode an Argos CloudLocate packet (hex from firmware logs)")
    parser.add_argument("--raw-blob", metavar="HEX",
                        help="Raw GNSS measurement blob (hex, without Argos packet framing)")
    parser.add_argument("--format", choices=["measc12", "meas20", "meas50"], default="meas20",
                        help="Format when using --raw-blob (default: meas20)")
    parser.add_argument("--locate", action="store_true",
                        help="Send blob to u-blox CloudLocate API for positioning")
    parser.add_argument("--token", default=DEFAULT_TOKEN_B64,
                        help=f"CloudLocate token (base64). Default: {DEFAULT_TOKEN_B64}")

    args = parser.parse_args()

    if not args.decode and not args.raw_blob:
        parser.print_help()
        print("\nExample with log data:")
        print("  python3 test_cloudlocate.py --decode E9B90A1CF32B78F1CA036B1C8CD296FF4D1")
        sys.exit(1)

    blob_hex = None
    format_name = None

    if args.decode:
        info = decode_argos_cloudlocate_packet(args.decode)
        if not info:
            sys.exit(1)
        print_decoded(info)
        blob_hex = info["blob_hex"]
        format_name = info["format_name"]

    if args.raw_blob:
        blob_hex = args.raw_blob.upper()
        format_name = args.format.upper()
        expected_len = {"MEASC12": 24, "MEAS20": 40, "MEAS50": 100}
        exp = expected_len.get(format_name, 0)
        if len(blob_hex) != exp:
            print(f"WARNING: blob length {len(blob_hex)} hex chars, expected {exp} for {format_name}")
        print(f"  Raw blob: {blob_hex} ({format_name})")

    if args.locate:
        if not blob_hex:
            print("ERROR: no blob to send (use --decode or --raw-blob)")
            sys.exit(1)
        print()
        query_cloudlocate(blob_hex, format_name, args.token)


if __name__ == "__main__":
    main()
