#!/usr/bin/env python3
"""
kim_bench.py — Autonomous hardware-in-the-loop bench driver for LinkIt V4 KIM.

Requires firmware built with the bench harness:

    ./scripts/build_linkitv4_kim.sh --bench      # implies --debug (USB-CDC logs on)

That firmware exposes a '%'-prefixed USB-CDC console (no magnet, no antenna):

    %PING                                  handshake  -> "%BENCH OK state=<S>"
    %STATE                                 -> "%STATE <S>"
    %CFG                                   enter ConfigurationState (bypass reed)
    %OP                                    leave config -> Operational
    %GPS <lat> <lon> [hAcc_mm] [numSV]     inject a synthetic 3D fix (no antenna)

plus the standard CLS DTE protocol ($CMD#LEN;payload\\r) while in config mode, so
every parameter can be read/written and every diagnostic command run over the same
link. Debug logs stream on the same CDC port; this driver demuxes replies (leading
'%' or '$O;'/'$N;') from log lines and asserts on both.

Usage:
    ./kim_bench.py --detect              find the board, handshake, print state
    ./kim_bench.py --monitor             stream timestamped logs (Ctrl-C to stop)
    ./kim_bench.py --shell               interactive: type % or $ commands
    ./kim_bench.py --run                 run the full validation suite, report PASS/FAIL
    ./kim_bench.py --gps -21.0097 55.2707   one-shot: go operational + inject a fix
    ./kim_bench.py --port /dev/ttyACM0   override auto-detection (Windows: --port COM5)
    ./kim_bench.py --run --allow-rf      do NOT force ARGOS_MODE=0 before the fix injection

Runs natively on Windows (python kim_bench.py --run) and on Linux/WSL: the board
is found by its USB VID:PID 1915:520F (env BENCH_PORT overrides), and every
text file is written in UTF-8 (bench_platform.py).

NO REAL RF by default: --run writes ARGOS_MODE=0 before leaving configuration,
so the injected fix cannot unlock a transmission. A KIM2 that answers with an
empty master RCONF transmits on its built-in LDK RCONF and its factory ID --
"no credentials" does not mean "no RF". Any AT+TX / TX START / TX EMITTED OK
seen in the session FAILS the run and %AMODE 0 is sent at once.

Every session is transcript-logged to tests/bench/logs/<timestamp>.log.
"""

import argparse
import datetime
import os
import re
import sys
import threading
import time

try:
    import serial  # pyserial
except ImportError:
    sys.exit("pyserial not installed. Run: pip install pyserial")

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import bench_platform as P  # noqa: E402

REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
DTE_PARAMS = os.path.join(REPO, "core", "protocol", "dte_params.cpp")
LOGDIR = os.path.join(HERE, "logs")

# --- CLS param map, parsed straight from the firmware source so it can never
# --- drift from the build under test: { "FRIENDLY_NAME", "KEY", ... }
_PARAM_RE = re.compile(r'\{\s*"([A-Z0-9_]+)"\s*,\s*"([A-Z0-9]+)"')

# Some parameters are not written as a raw value but as an INDEX into a code
# table: DLOC_ARG_NOM=4 means "1 hour", not "4 seconds". Writing seconds gets
# the WHOLE PARMW frame rejected -- the board names the offending key and
# applies the other ones, so the case's precondition silently never lands and
# the verdict that follows is meaningless. Measured on 2026-08-27 with
# DLOC_ARG_NOM=600. Parse the permitted set from the firmware source too, so
# this check cannot drift from the build under test.
_CODED_RE = re.compile(
    r'\{\s*"([A-Z0-9_]+)"\s*,\s*"([A-Z0-9]+)"\s*,\s*BaseEncoding::'
    r'(AQPERIOD|DEPTHPILE|ARGOSPOWER|ARGOSMODE|GNSSFIXMODE|GNSSDYNMODEL)\s*,'
    r'[^{}]*\{([^}]*)\}')


def load_coded_params(path=DTE_PARAMS):
    """{ NAME: (encoding, {permitted codes}) } for code-table parameters."""
    out = {}
    try:
        src = P.read_text(path)
    except OSError:
        return out
    for m in _CODED_RE.finditer(src):
        nom, _cle, enc, valeurs = m.groups()
        codes = {int(x) for x in re.findall(r'(\d+)U?', valeurs)}
        if codes:
            out[nom] = (enc, codes)
    return out


def load_param_map(path=DTE_PARAMS):
    name2key, key2name = {}, {}
    try:
        with open(path, encoding="utf-8", errors="replace") as f:
            for line in f:
                m = _PARAM_RE.search(line)
                if m:
                    name, key = m.group(1), m.group(2)
                    name2key[name] = key
                    key2name[key] = name
    except OSError as e:
        print(f"[warn] could not read {path}: {e} (friendly param names unavailable)")
    return name2key, key2name


# Firmware lines that mean a frame is going on air, or has gone. All ASCII on
# purpose (the firmware's em dashes are not): kim2.cpp:1394/1397 (AT+TX about
# to be sent -- the module answered its init, so it WILL transmit),
# kim2.cpp:1481, argos_tx_service.cpp:3030/3041. A mute KIM2 never reaches
# state_transmit_enter (its init ping fails first), so on this bench any of
# them means real RF.
RF_SIGNATURES = (
    "KIM2Device::state_transmit_enter: TX START",
    "KIM2Device::state_transmit_enter: AT+TX=",
    "KIM2Device::state_transmit: TX EMITTED OK",
    "ArgosTxService: TX START",
    "ArgosTxService: TX SUCCESS",
)


class RFGuard:
    """Watches every line; on the first RF signature it records the line and
    sends %AMODE 0 at once (RAM write of ARGOS_MODE, no config round trip) so
    no second frame follows. Callers decide what to fail."""

    def __init__(self, bench):
        self.b = bench
        self.seen = None
        bench.line_hooks.append(self)

    def __call__(self, line):
        if self.seen is None and any(sig in line for sig in RF_SIGNATURES):
            self.seen = line
            try:
                self.b._send("%AMODE 0\r\n")
            except Exception:
                pass

    def reset(self):
        self.seen = None


class Bench:
    def __init__(self, port=None, baud=115200, quiet=False):
        self.port = port
        self.baud = baud
        self.quiet = quiet
        self.ser = None
        self.history = []            # list of (monotonic_ts, line)
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._reader_thread = None
        self._buf = b""
        # Called from the reader thread with every line (RF guard, counters).
        # A hook that raises is ignored: it must never kill the reader.
        self.line_hooks = []
        self._wlock = threading.Lock()
        self.name2key, self.key2name = load_param_map()
        self._coded = load_coded_params()
        os.makedirs(LOGDIR, exist_ok=True)
        stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.logpath = os.path.join(LOGDIR, f"{stamp}.log")
        self._logf = P.utf8_open(self.logpath, "a", buffering=1)

    # ---- connection -------------------------------------------------------
    def open(self):
        if self.port is None:
            self.port = os.environ.get("BENCH_PORT") or self._autodetect()
            if self.port is None:
                raise RuntimeError(
                    "No bench board found (USB 1915:520F). Is it plugged in (under WSL: "
                    "attached with wsl_usb.sh; on Windows: nothing else holding the COM "
                    "port)? Is the firmware built with --bench?")
        # write_timeout n est pas un detail: sur un CDC a moitie mort — le
        # noeud /dev/ttyACM existe, mais l endpoint ne draine plus — un write()
        # sans borne BLOQUE POUR TOUJOURS. Mesure du 2026-08-27: une campagne
        # est restee pendue dans _autodetect, sans une ligne de sortie, jusqu a
        # ce qu on aille lire sa pile. C est le pire mode de defaillance pour un
        # essai autonome: il ne rate pas, il s arrete indefiniment. Avec la
        # borne, l ecriture leve et connect() peut reparer le lien.
        self.ser = serial.Serial(self.port, self.baud, timeout=0.1, write_timeout=5)
        self._start_reader()
        return self

    def _candidate_ports(self):
        # VID:PID first (Windows and Linux), then the Linux glob newest first:
        # sorted by name, ttyACM10 came before ttyACM2.
        return P.find_ports()

    def _autodetect(self):
        for p in self._candidate_ports():
            try:
                s = serial.Serial(p, self.baud, timeout=0.1, write_timeout=5)
            except OSError:
                continue
            try:
                s.reset_input_buffer()
                # Une seule tentative dans une fenetre de 1,5 s declarait la carte
                # absente alors qu elle repondait: mesure du 2026-08-30, ou une
                # sonde manuelle a 3 s obtenait "%BENCH OK state=OPERATIONAL"
                # quelques secondes apres que l autodetection eut abandonne. Juste
                # apres un reset ou une reprise du lien usbipd, le premier %PING se
                # perd souvent. Trois tentatives de 3 s coutent au pire 9 s sur un
                # port muet, et evitent d abandonner sur une carte vivante.
                acc = b""
                for _essai in range(3):
                    s.write(b"%PING\r\n")
                    s.flush()
                    deadline = time.time() + 3.0
                    while time.time() < deadline:
                        acc += s.read(256)
                        if b"%BENCH" in acc:
                            print(f"[detect] bench board on {p}")
                            s.close()
                            return p
            except (serial.SerialException, OSError):
                # Port present mais moribond: on passe au candidat suivant au
                # lieu d avorter tout le balayage. Sans write_timeout ce chemin
                # n existait pas — l ecriture pendait a la place.
                pass
            finally:
                s.close()
        return None

    # ---- reader thread ----------------------------------------------------
    def _start_reader(self):
        self._reader_thread = threading.Thread(target=self._reader, daemon=True)
        self._reader_thread.start()

    def _reader(self):
        while not self._stop.is_set():
            try:
                chunk = self.ser.read(512)
            except (OSError, serial.SerialException, AttributeError, TypeError):
                # TypeError/AttributeError, not just SerialException: when
                # close() runs while this thread sits inside read(), pyserial
                # has already set self.fd to None and os.read(None, ...) raises
                # TypeError. Left uncaught it dumps a traceback over the
                # campaign log and hides the real event -- a dead USB link.
                break
            if not chunk:
                continue
            self._buf += chunk
            # Frame on either \r or \n so DTE ('\r'-only) and console ('\r\n')
            # lines both split cleanly.
            while True:
                idx = min([i for i in (self._buf.find(b"\r"), self._buf.find(b"\n")) if i >= 0], default=-1)
                if idx < 0:
                    break
                raw, self._buf = self._buf[:idx], self._buf[idx + 1:]
                if raw == b"":
                    continue
                line = raw.decode("utf-8", "replace")
                ts = time.monotonic()
                with self._lock:
                    self.history.append((ts, line))
                # Nothing below may raise out of the reader: a dead reader thread
                # is a "mute board" for every following wait.
                try:
                    self._logf.write(f"{datetime.datetime.now().strftime('%H:%M:%S.%f')[:-3]}  {line}\n")
                    if not self.quiet:
                        print(f"  | {line}")
                except Exception:
                    pass
                for hook in list(self.line_hooks):
                    try:
                        hook(line)
                    except Exception:
                        pass

    def close(self):
        self._stop.set()
        if self.ser:
            try:
                self.ser.close()
            except OSError:
                pass
        self._logf.close()

    # ---- low-level send ---------------------------------------------------
    def _send(self, s):
        try:
            self._logf.write(f"{datetime.datetime.now().strftime('%H:%M:%S.%f')[:-3]}  >> {s!r}\n")
        except Exception:
            pass
        if not self.quiet:
            print(f">> {s!r}")
        # Serialised: the RF guard may send %AMODE 0 from the reader thread.
        with self._wlock:
            self.ser.write(s.encode())
            self.ser.flush()

    # ---- expect -----------------------------------------------------------
    def expect(self, pattern, timeout=5.0, from_idx=None):
        """Wait until a history line matches `pattern` (regex). Returns the
        re.Match or None on timeout. Non-consuming (scans history forward)."""
        rx = re.compile(pattern)
        if from_idx is None:
            with self._lock:
                from_idx = len(self.history)
        deadline = time.time() + timeout
        i = from_idx
        while time.time() < deadline:
            with self._lock:
                snapshot = self.history[i:]
                i = len(self.history)
            for _, line in snapshot:
                m = rx.search(line)
                if m:
                    return m
            time.sleep(0.02)
        return None

    def mark(self):
        with self._lock:
            return len(self.history)

    # ---- bench '%' commands ----------------------------------------------
    def ping(self, timeout=3.0):
        mk = self.mark()
        self._send("%PING\r\n")
        return self.expect(r"%BENCH OK state=(\S+)", timeout, from_idx=mk)

    def get_state(self, timeout=3.0):
        mk = self.mark()
        self._send("%STATE\r\n")
        m = self.expect(r"%STATE (\S+)", timeout, from_idx=mk)
        return m.group(1) if m else None

    # FSM entry-log for each state — a definitive passive signal that survives the
    # config↔operational transition window where %STATE polling can briefly stall.
    _ENTRY_LOG = {
        "OPERATIONAL": "entry: OperationalState",
        "CONFIG": "entry: ConfigurationState",
        "PREOP": "entry: PreOperationalState",
        "OFF": "entry: OffState",
    }

    def wait_state(self, want, timeout=20.0):
        mk = self.mark()
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self.get_state() == want:
                return True
            # Also accept the passive FSM entry log (robust during transitions).
            if want in self._ENTRY_LOG and \
               self.expect(re.escape(self._ENTRY_LOG[want]), 0.1, from_idx=mk):
                return True
            time.sleep(0.4)
        return False

    def enter_config(self, timeout=12.0):
        mk = self.mark()
        self._send("%CFG\r\n")
        self.expect(r"%CFG OK", 3.0, from_idx=mk)
        return self.wait_state("CONFIG", timeout)

    def exit_config(self, timeout=12.0):
        mk = self.mark()
        self._send("%OP\r\n")
        self.expect(r"%OP OK", 3.0, from_idx=mk)
        return self.wait_state("OPERATIONAL", timeout)

    def settle_console(self, timeout=20.0):
        """Wait until the bench console answers a %PING within one second.

        Drains any backlog of late replies left by a busy main loop (service
        start-up after %OP). Without this the next command reads a stale answer
        and every subsequent check is measuring the wrong thing.
        """
        deadline = time.time() + timeout
        while time.time() < deadline:
            mk = self.mark()
            self._send("%PING\r\n")
            if self.expect(r"%BENCH OK", 1.0, from_idx=mk):
                return True
        return False

    def inject_gps(self, lat, lon, hacc_mm=0, numsv=0, timeout=5.0):
        mk = self.mark()
        args = f"{lat} {lon}"
        if hacc_mm:
            args += f" {hacc_mm}"
            if numsv:
                args += f" {numsv}"
        self._send(f"%GPS {args}\r\n")
        return self.expect(r"%GPS OK", timeout, from_idx=mk)

    # ---- DTE '$...#' commands --------------------------------------------
    def dte(self, cmd, payload="", timeout=6.0):
        """Send a raw DTE command and return the response line (or None).
        Frames as $CMD#<len3hex>;<payload>\\r."""
        mk = self.mark()
        frame = f"${cmd}#{len(payload):03X};{payload}\r"
        self._send(frame)
        # Response: $O;CMD#...  or  $N;CMD#...
        return self.expect(rf"\$([ON]);{cmd}#", timeout, from_idx=mk)

    def _key(self, name_or_key):
        return self.name2key.get(name_or_key, name_or_key)

    def read_params(self, names, timeout=6.0):
        """PARMR one or more params (friendly names or raw keys). Returns the
        raw response line, and a dict of key->value parsed from it."""
        keys = [self._key(n) for n in names]
        payload = ",".join(keys)
        mk = self.mark()
        self._send(f"$PARMR#{len(payload):03X};{payload}\r")
        m = self.expect(r"\$([ON]);PARMR#[0-9A-Fa-f]{3};(.*)$", timeout, from_idx=mk)
        if not m:
            return None, {}
        ok = m.group(1) == "O"
        body = m.group(2).rstrip("\r")
        parsed = {}
        if ok:
            for tok in body.split(","):
                if "=" in tok:
                    k, v = tok.split("=", 1)
                    parsed[k] = v
        return m.group(0), parsed

    def check_coded(self, kv, strict=True):
        """Raise before sending if a code-table parameter got a raw value.

        Catching this here rather than in the reply saves a whole case: the
        board's partial rejection still applies the other keys, so the run
        continues on a half-established precondition.

        Silent when strict=False, and that is not a loophole -- it is the
        distinction that matters. strict=True means "I intend every one of these
        to be applied, catch my mistake"; strict=False means "I am deliberately
        probing what the board refuses". A bounds case writes ARGOS_MODE=6 ON
        PURPOSE and needs the board's own rejection, not the host's. Without
        this, PAR-02 went red on a guard that was doing its job.
        """
        if not self._coded or not strict:
            return
        for nom, val in kv.items():
            info = self._coded.get(nom)
            if not info or not isinstance(val, int):
                continue
            enc, codes = info
            if val not in codes:
                raise ValueError(
                    f"{nom} is a {enc} code table, not a raw value: {val} is not "
                    f"one of {sorted(codes)}")

    def write_params(self, kv, timeout=6.0, strict=True):
        """PARMW a dict of {name_or_key: value}. Returns the response match.

        A partial rejection answers `$N;PARMW#<len>;<key1>,<key2>` and the board
        still applies every OTHER key. That is correct firmware behaviour, but
        for a TEST it is a trap: a mistyped parameter name is passed through
        verbatim by _key(), lands in the rejected list, and the precondition it
        was supposed to establish is silently never applied — the case then goes
        green while testing something else entirely. This is not hypothetical:
        `ARGOS_NTRY_PER_MESSAGE` (the real name is NTRY_PER_MESSAGE / ARP19) sat
        in four campaign cases doing exactly that.

        strict=True raises on any rejected key. Pass strict=False only when a
        rejection is what the case is deliberately provoking.
        """
        self.check_coded(kv, strict)
        toks = [f"{self._key(k)}={v}" for k, v in kv.items()]
        payload = ",".join(toks)
        mk = self.mark()
        self._send(f"$PARMW#{len(payload):03X};{payload}\r")
        m = self.expect(r"\$([ON]);PARMW#([0-9A-Fa-f]{3});?(.*)$", timeout, from_idx=mk)
        if strict and m and m.group(1) == 'N':
            refus = m.group(3).rstrip('\r').strip()
            raise RuntimeError(
                f"PARMW rejected: {refus or '(no key named)'} "
                f"— requested {sorted(kv)}; the other keys WERE applied")
        return m


# =====================================================================
#  Validation suite
# =====================================================================
class Suite:
    def __init__(self, b: Bench, allow_rf=False):
        self.b = b
        self.allow_rf = allow_rf
        self.guard = RFGuard(b)
        self.results = []  # (name, ok, detail)

    def check(self, name, ok, detail=""):
        self.results.append((name, bool(ok), detail))
        tag = P.colour("PASS", "1;32") if ok else P.colour("FAIL", "1;31")
        print(f"[{tag}] {name}" + (f"  — {detail}" if detail else ""))
        return ok

    def run(self):
        b = self.b
        print("\n=== LinkIt V4 KIM — autonomous bench validation ===\n")

        # 1. Handshake — confirms bench firmware + serial link.
        m = b.ping()
        self.check("handshake (%PING)", m is not None,
                   f"state={m.group(1)}" if m else "no %BENCH reply")

        # 2. Reed bypass — enter config mode without a magnet.
        ok = b.enter_config()
        self.check("enter config (reed bypass %CFG)", ok, f"state={b.get_state()}")

        # 3. DTE reachable in config — status read round-trips.
        r = b.dte("STATR")
        self.check("DTE STATR responds", r is not None and r.group(1) == "O",
                   r.group(0) if r else "no response")

        # 4. Read device identity (proves PARMR + param-map decode).
        line, parsed = b.read_params(["ARGOS_HEXID", "ARGOS_DECID", "DEVICE_MODEL"])
        self.check("PARMR device identity", bool(parsed),
                   ", ".join(f"{k}={v}" for k, v in parsed.items()) or "no params")

        # 5. Param write/read round-trip (proves PARMW).
        target = "120"
        w = b.write_params({"GNSS_ACQ_TIMEOUT": target})
        _, rp = b.read_params(["GNSS_ACQ_TIMEOUT"])
        got = rp.get(b._key("GNSS_ACQ_TIMEOUT"))
        self.check("PARMW/PARMR round-trip (GNSS_ACQ_TIMEOUT=120)",
                   w is not None and w.group(1) == "O" and got == target,
                   f"wrote {target}, read {got}")

        # 6. SATVF answers. This proves the DTE path to the radio, NOT that
        # credentials are valid: $N is the expected answer on a mute KIM2 or an
        # unprovisioned master, and the check says which one it got rather than
        # pretending to verify credentials.
        r = b.dte("SATVF", timeout=15.0)
        if r is not None:
            sens = ("verified" if r.group(1) == "O"
                    else "NOT verified (expected: mute KIM2 or no master RCONF)")
        self.check("SATVF answers (radio path)", r is not None,
                   f"{r.group(0)} -> {sens}" if r else "no response (check KIM2 UART)")

        # 6b. No real RF from here on: the fix injected below unlocks Argos TX
        # in every mode, and a KIM2 that answers transmits on its built-in LDK
        # RCONF with its factory ID even without credentials.
        if not self.allow_rf:
            try:
                b.write_params({"ARGOS_MODE": 0, "CERT_TX_ENABLE": 0})
                _, am = b.read_params(["ARGOS_MODE"])
                self.check("ARGOS_MODE=0 forced (no RF)", am.get(b._key("ARGOS_MODE")) == "0",
                           f"ARP01={am.get(b._key('ARGOS_MODE'))}")
            except Exception as e:
                self.check("ARGOS_MODE=0 forced (no RF)", False, f"{type(e).__name__}: {e}")

        # 7. Back to operational.
        #
        # Leaving config starts every service, and the KIM2 bring-up alone is
        # several seconds of UART round-trips (RCONF read + validate). The bench
        # console is polled from the same main loop, so its replies queue up
        # behind that: a %STATE sent immediately after %OP routinely answers
        # 2-5 s late. That is not a fault -- it is what a board doing real work
        # looks like -- but a caller that does not wait for it desynchronises
        # every following command, because the late reply is then read as the
        # answer to the NEXT one. That is exactly what used to make check 8 fail
        # while its own log line proved the injection had happened.
        ok = b.exit_config()
        b.settle_console(20.0)
        self.check("exit config (%OP)", ok, f"state={b.get_state()}")

        # 8. Synthetic GPS fix injection -> post-fix pipeline.
        # 20 s, not 5: the console can still be behind the service start-up here.
        mk = b.mark()
        m = b.inject_gps(-21.0097, 55.2707, timeout=20.0)
        logline = b.expect(r"GPSService::bench_inject_fix: lat=" r"-21", 20.0, from_idx=mk)
        self.check("GPS injection (%GPS)", m is not None and logline is not None,
                   "injected -21.0097,55.2707")
        # Assert on an INFO/WARN log (task_process_gnss_data itself is TRACE-level,
        # filtered at DEBUG_LEVEL=3).
        #
        # Les deux premiers motifs sont CIRCONSTANCIELS, et c est ce qui a fait
        # rougir ce cas le 2026-08-31 sur une carte parfaitement saine:
        # "retry_counter: reset ntry=N->0" n est emis que si des tentatives se
        # sont accumulees (ntry>0), et "service GNSS completed" disparait sous
        # GNSS_SESSION_SINGLE_FIX, qui ne replanifie pas apres le premier fix.
        # La preuve DIRECTE que le fix a traverse le pipeline est son entree
        # dans la pile de profondeur — elle ne depend d aucun reglage.
        # (The former alternatives "DepthPileManager: GPS FIX stored" and
        # "service GNSS completed" exist nowhere in the firmware.)
        pipeline = b.expect(r"(first valid GPS fix this session"
                            r"|retry_counter: reset ntry.*PVT fix)",
                            8.0, from_idx=mk)
        self.check("fix accepted into pipeline (PVT)", pipeline is not None,
                   pipeline.group(0).split(']')[-1].strip() if pipeline else "no PVT-accept log")

        # 9. Argos TX service started (service_init runs even with ARGOS_MODE=0)
        # and what the KIM2 said at its boot resync. Scan the whole session.
        alive = b.expect(r"ArgosTxService::service_init: Argos ID=(\d+)", 1.0, from_idx=0)
        mute = b.expect(r"KIM2Device::resync_rconf_cache: ping failed", 1.0, from_idx=0)
        empty = b.expect(r"(KIM2Device::resync_rconf_cache: master RCONF empty"
                         r"|KIM2Device::resync_rconf_cache: no RCONF configured)", 1.0, from_idx=0)
        etat = ("KIM2 MUTE (resync ping failed)" if mute else
                "KIM2 answers, master RCONF empty -> built-in LDK" if empty else "KIM2 answers")
        self.check("Argos TX service started", alive is not None,
                   f"ID={alive.group(1)} -- {etat}" if alive else "no ArgosTxService init log seen")

        # 10. No real RF anywhere in the session (guard armed since the start).
        time.sleep(5.0)
        if self.allow_rf:
            self.check("RF guard (--allow-rf: informative)", True,
                       f"RF seen: {self.guard.seen}" if self.guard.seen else "no RF seen")
        else:
            self.check("no real RF in the session", self.guard.seen is None,
                       f"RF SEEN, %AMODE 0 sent: {self.guard.seen}" if self.guard.seen
                       else "no AT+TX / TX START / TX EMITTED OK")

        # summary
        npass = sum(1 for _, ok, _ in self.results if ok)
        print(f"\n=== {npass}/{len(self.results)} checks passed ===")
        print(f"Transcript: {b.logpath}")
        return all(ok for _, ok, _ in self.results)


# =====================================================================
def main():
    ap = argparse.ArgumentParser(description="LinkIt V4 KIM bench driver")
    ap.add_argument("--port", help="serial port (default: auto-detect)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--detect", action="store_true", help="find board + handshake")
    ap.add_argument("--monitor", action="store_true", help="stream logs")
    ap.add_argument("--shell", action="store_true", help="interactive command shell")
    ap.add_argument("--run", action="store_true", help="run validation suite")
    ap.add_argument("--gps", nargs=2, metavar=("LAT", "LON"),
                    help="go operational + inject a fix, then stream logs")
    ap.add_argument("--allow-rf", action="store_true",
                    help="--run: do not force ARGOS_MODE=0 (a live KIM2 WILL transmit)")
    args = ap.parse_args()
    P.setup_stdio()

    b = Bench(port=args.port, baud=args.baud,
              quiet=not (args.monitor or args.shell or args.gps))
    b.open()
    try:
        if args.detect or not any([args.monitor, args.shell, args.run, args.gps]):
            m = b.ping()
            if m:
                print(f"Board OK — state={m.group(1)}  port={b.port}")
                print(f"Params known: {len(b.name2key)}  transcript={b.logpath}")
            else:
                print("No %BENCH handshake — is this a --bench firmware?")
                return 1
        if args.monitor:
            print("Streaming logs (Ctrl-C to stop)…")
            while True:
                time.sleep(1)
        if args.gps:
            b.exit_config()
            b.inject_gps(float(args.gps[0]), float(args.gps[1]))
            time.sleep(20)
        if args.shell:
            print("Shell — type a line (%CFG, %STATE, $STATR#000;, ...), 'q' to quit")
            while True:
                try:
                    line = input()
                except EOFError:
                    break
                if line.strip() in ("q", "quit", "exit"):
                    break
                if line and not line.endswith("\r"):
                    line += "\r\n" if line.startswith("%") else "\r"
                b._send(line)
                time.sleep(0.3)
        if args.run:
            ok = Suite(b, allow_rf=args.allow_rf).run()
            return 0 if ok else 1
    except KeyboardInterrupt:
        pass
    finally:
        b.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
