#!/usr/bin/env python3
"""magnet_trace.py <secondes> — trace horodatée d'une manipulation aimant.

Combine deux sources : la présence du CDC (AWAKE/ASLEEP, indicateur binaire du
System OFF) et les lignes série pertinentes (gestes reed, transitions d'état,
powerdown). Se reconnecte tout seul quand la carte se rendort puis repart.
"""
import os, re, sys, time
from datetime import datetime
try:
    import serial
except ImportError:
    sys.exit("pyserial manquant")

P = '/dev/ttyACM0'
DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 120
PAT = re.compile(r"ReedSwitch|Reed |confirmation|OffState|powerdown|PreOperational|"
                 r"entry: |exit: |storage|BatteryCritical|reset cause|RESETREAS", re.I)

def ts(): return datetime.now().strftime("%H:%M:%S.%f")[:-3]
def log(m): print(f"[{ts()}] {m}", flush=True)

s = None
last_present = os.path.exists(P)
log(f"état initial : {'AWAKE' if last_present else 'ASLEEP'} — trace {DUR:.0f}s, vas-y")
t0 = time.time()
while time.time() - t0 < DUR:
    present = os.path.exists(P)
    if present != last_present:
        log(f"*** {'AWAKE (USB énuméré)' if present else 'ASLEEP (USB tombé)'}  t+{time.time()-t0:6.1f}s")
        last_present = present
        if not present and s:
            try: s.close()
            except Exception: pass
            s = None
    if present and s is None:
        try:
            s = serial.Serial(P, 115200, timeout=0.05)
        except Exception:
            s = None
    if s:
        try:
            while s.in_waiting:
                line = s.readline().decode(errors="replace").strip()
                if line and PAT.search(line):
                    log("  | " + line[:150])
        except Exception:
            try: s.close()
            except Exception: pass
            s = None
    time.sleep(0.1)
log(f"fin de trace — état final {'AWAKE' if os.path.exists(P) else 'ASLEEP'}")
