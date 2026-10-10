#!/usr/bin/env python3
"""bench_platform.py — the few things that differ between a Linux/WSL bench and
a native Windows bench, gathered in one place.

The bench scripts used to assume Linux everywhere: `fcntl` for the campaign
lock, `/tmp` for outputs, `/dev/ttyACM*` for the board, `usbipd.exe` called
from WSL to repair a dropped link, and a `nrfjprog` that inherits DISPLAY. A
native Windows machine (Python + pyserial + nrfjprog.exe + J-Link) has none of
these, and every one of them failed differently: an ImportError at load time,
a "no board found" while the board was on COM5, a usbip repair that could only
fail and then reset the board for nothing.

What lives here:
  - find_ports()      the tag's CDC port, by USB VID:PID (1915:520F), on both OSes
  - nrfjprog()        one guarded call: no stdin, no inherited pipe, hard timeout,
                      the whole process tree killed on expiry, return code kept
  - recover()/flash_merged()  the CLAUDE.md flashing sequence, with its checks
  - BenchLock         an exclusive, non-blocking lock that also works on Windows
  - utf8_open()/setup_stdio()/colour()  text that cannot kill a reader thread

Nothing here touches a board on import.
"""
import glob
import os
import re
import shutil
import signal
import subprocess
import sys
import tempfile
import time

IS_WINDOWS = os.name == "nt"

# USB identity of the LinkIt application CDC (ports/nrf52840/bsp/linkitv4_v1.0/
# sdk_config.h). The VID alone is NOT enough: the DFU bootloader enumerates as
# 1915:521F, and the J-Link (VID 1366) exposes its own virtual COM port that
# opens fine and never says anything -- a reader bound to it reads nothing for
# hours without ever erroring.
NORDIC_VID = 0x1915
APP_PID = 0x520F
DFU_PID = 0x521F

# NRF_USBD->USBPULLUP: writing 0 then 1 over SWD is a software unplug/replug of
# the tag's USB device. Used to recover a CDC endpoint that stopped draining.
USBPULLUP_ADDR = "0x40027504"


def is_wsl():
    if IS_WINDOWS:
        return False
    try:
        with open("/proc/version", encoding="utf-8", errors="replace") as f:
            return "microsoft" in f.read().lower()
    except OSError:
        return False


def temp_dir():
    """C:\\Users\\<u>\\AppData\\Local\\Temp on Windows, /tmp on Linux."""
    return tempfile.gettempdir()


# ---------------------------------------------------------------- text / console
def utf8_open(path, mode="a", **kw):
    """open() that cannot raise UnicodeEncodeError/DecodeError.

    On Windows the default encoding is cp1252. The firmware logs characters
    cp1252 cannot write (arrows in hauled_mode_service.cpp, gentracker.cpp), and
    line noise decodes to U+FFFD: either one used to kill the serial reader
    thread silently, after which every wait timed out and the campaign blamed
    a "mute board" and reset it.
    """
    if "b" in mode:
        return open(path, mode, **kw)
    kw.setdefault("encoding", "utf-8")
    kw.setdefault("errors", "replace")
    return open(path, mode, **kw)


def read_text(path):
    """Whole UTF-8 source file, never a decode error (sws_analog_constants.hpp
    holds box-drawing characters that cp1252 cannot decode)."""
    with open(path, encoding="utf-8", errors="replace") as f:
        return f.read()


def setup_stdio():
    """Make print() safe when stdout is a pipe captured by an agent (cp1252)."""
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(encoding="utf-8", errors="replace")
        except (AttributeError, ValueError):
            pass
    if IS_WINDOWS:
        # Enables VT processing in the classic console so colour escapes render.
        try:
            os.system("")
        except Exception:
            pass


def colour(text, code):
    """ANSI colour only on an interactive terminal: an agent capturing output
    gets plain PASS/FAIL, not raw escape bytes."""
    try:
        tty = sys.stdout.isatty()
    except Exception:
        tty = False
    return f"\033[{code}m{text}\033[0m" if tty else text


# ---------------------------------------------------------------- serial ports
def find_ports(include_linux_glob=True):
    """Candidate CDC ports of the tag, most likely first.

    1) Portable: serial.tools.list_ports filtered on 1915:520F (a port whose PID
       the driver does not report is accepted on the VID alone, the DFU PID and
       the J-Link VID are always refused).
    2) Linux only: /dev/ttyACM* newest first, for drivers that hide the IDs --
       but ONLY nodes whose IDs are unknown: a node list_ports identifies with
       another VID/PID (the J-Link VCOM, VID 1366, the DFU bootloader) is never
       a candidate, else a reader bound to it during a re-enumeration reads
       silence for hours.
    Never probes every COM port on Windows: opening a Bluetooth serial port can
    block for many seconds.
    """
    out = []
    identified = set()          # nodes list_ports reports WITH a VID
    try:
        from serial.tools import list_ports
        for p in list_ports.comports():
            if p.vid is not None:
                identified.add(p.device)
                try:
                    identified.add(os.path.realpath(p.device))
                except Exception:
                    pass
            if p.vid != NORDIC_VID:
                continue
            if p.pid is not None and p.pid != APP_PID:
                continue
            out.append(p.device)
    except Exception:
        pass
    if include_linux_glob and not IS_WINDOWS:
        def mtime(p):
            try:
                return os.stat(p).st_mtime
            except OSError:
                return 0
        for c in sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"), key=mtime, reverse=True):
            if c in out or c in identified or os.path.realpath(c) in identified:
                continue
            out.append(c)
    return out


def wait_for_port(timeout=10.0):
    """Poll until the tag's CDC enumerates. Returns the port or None."""
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        ports = find_ports()
        if ports:
            return ports[0]
        time.sleep(0.5)
    return None


# ---------------------------------------------------------------- nrfjprog
class NrfjprogResult:
    def __init__(self, args, rc, out, timed_out, seconds):
        self.args = args
        self.returncode = rc
        self.stdout = out          # str: stdout and stderr merged
        self.stderr = ""
        self.timed_out = timed_out
        self.seconds = seconds

    @property
    def ok(self):
        return self.returncode == 0 and not self.timed_out

    def __repr__(self):
        st = "TIMEOUT" if self.timed_out else f"rc={self.returncode}"
        return f"<nrfjprog {' '.join(self.args)} {st} {self.seconds:.0f}s>"


def nrfjprog_exe():
    return os.environ.get("NRFJPROG") or shutil.which("nrfjprog") or "nrfjprog"


def _kill_tree(proc):
    if IS_WINDOWS:
        subprocess.run(["taskkill", "/F", "/T", "/PID", str(proc.pid)],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        # The J-Link DLL spawns a GUI server that keeps the probe (rc -102 on
        # every following call) when its parent is killed.
        for image in ("JLinkGUIServer.exe", "JLinkGUIServerExe.exe"):
            subprocess.run(["taskkill", "/F", "/IM", image],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    else:
        try:
            os.killpg(proc.pid, signal.SIGKILL)
        except Exception:
            try:
                proc.kill()
            except Exception:
                pass
    try:
        proc.wait(timeout=10)
    except Exception:
        pass


def nrfjprog(args, timeout=180, snr=None, family=True, log_dir=None):
    """Run nrfjprog once and ALWAYS come back.

    - stdin from DEVNULL and, on Linux, no DISPLAY: the J-Link GUI server waits
      forever on a dialog nobody can see (four --recover hung on 2026-08-29).
    - stdout/stderr to a temp FILE, not a pipe: on Windows a child that inherits
      the pipe (JLinkGUIServer) keeps it open after the kill, and the
      post-timeout communicate() of subprocess.run() then waits for ever.
    - on timeout the whole process tree is killed and the result says so; the
      caller decides, nothing here retries. On Ctrl-C (or any exception) the
      tree is killed too, then the exception propagates.
    - `-f nrf52` and `--snr` (env NRFJPROG_SNR) so no family or probe-selection
      prompt can appear. `--log` (when log_dir is given) leaves a trace of a hang.
    """
    exe = nrfjprog_exe()
    cmd = [exe] + list(args)
    if family and "-f" not in args and "--family" not in args and "--ids" not in args \
            and "--version" not in args:
        cmd += ["-f", "nrf52"]
    snr = snr or os.environ.get("NRFJPROG_SNR")
    if snr and "--snr" not in args and "--ids" not in args:
        cmd += ["--snr", str(snr)]
    if log_dir:
        cmd += ["--log", os.path.join(log_dir, "nrfjprog.log")]
    env = {k: v for k, v in os.environ.items() if k not in ("DISPLAY", "WAYLAND_DISPLAY")}
    kw = {}
    if IS_WINDOWS:
        kw["creationflags"] = getattr(subprocess, "CREATE_NO_WINDOW", 0) | \
            getattr(subprocess, "CREATE_NEW_PROCESS_GROUP", 0)
    else:
        kw["start_new_session"] = True
    t0 = time.monotonic()
    with tempfile.TemporaryFile() as out, open(os.devnull, "rb") as devnull:
        try:
            proc = subprocess.Popen(cmd, stdin=devnull, stdout=out, stderr=subprocess.STDOUT, env=env, **kw)
        except OSError as e:
            return NrfjprogResult(args, 127, f"cannot start {exe}: {e}", False, 0.0)
        timed_out = False
        try:
            proc.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            timed_out = True
            _kill_tree(proc)
        except BaseException:
            # Ctrl-C (KeyboardInterrupt) or any other exception: the child runs
            # in its own session / process group, so the terminal's Ctrl-C never
            # reaches it (CREATE_NEW_PROCESS_GROUP disables CTRL+C for the group
            # on Windows). Without this the orphan keeps the J-Link and the next
            # flash fails with rc -102 or hangs.
            _kill_tree(proc)
            try:
                print(f"nrfjprog {' '.join(args)} interrupted -- process tree killed, probe released",
                      file=sys.stderr, flush=True)
            except Exception:
                pass
            raise
        out.seek(0)
        text = out.read().decode("utf-8", "replace")
    rc = proc.returncode if proc.returncode is not None else -1
    return NrfjprogResult(args, rc, text, timed_out, time.monotonic() - t0)


def run_text(cmd, timeout=30):
    """Run a console command and return its stdout as text, '' on any failure.

    Captured as bytes and decoded here: with PYTHONUTF8=1, text=True decodes as
    UTF-8, but Windows console tools (tasklist, usbipd, powershell) write in the
    OEM code page; a French tasklist puts 0xFF (no-break space) in "19 772 Ko",
    the reader thread dies and .stdout comes back as None."""
    try:
        raw = subprocess.run(cmd, capture_output=True, timeout=timeout).stdout or b""
    except Exception:
        return ""
    for enc in (("oem", "mbcs") if IS_WINDOWS else ()) + ("utf-8",):
        try:
            return raw.decode(enc, errors="replace")
        except LookupError:
            continue
    return raw.decode("latin-1", errors="replace")


def probe_holders():
    """Processes known to hold the J-Link (Windows only). Reported, never killed
    here: nRF Connect for Desktop belongs to the operator."""
    if not IS_WINDOWS:
        return []
    out = run_text(["tasklist", "/FO", "CSV", "/NH"], timeout=20)
    found = []
    for line in out.splitlines():
        name = line.split(",")[0].strip('"')
        if re.match(r"(?i)(nrfutil-device|nrfconnect|nRF Connect|JLinkGUIServer|JLink\.exe|JFlash)", name):
            found.append(name)
    return sorted(set(found))


RECOVER_TIMEOUT_S = 120


def recover(log_dir=None):
    """`nrfjprog --recover`, ONE attempt, hard limit 120 s.

    A recover that hangs or fails is reported, never retried in a loop: the
    usual causes (probe held by nRF Connect / nrfutil-device, a J-Link firmware
    update prompt, the board unplugged) are not cured by repetition.
    WIPES the configuration store: every parameter a case relies on must be
    re-applied afterwards.
    """
    return nrfjprog(["--recover"], timeout=RECOVER_TIMEOUT_S, log_dir=log_dir)


def find_merged_hex(root=None):
    """Newest ports/nrf52840/build*/LINKIT/LinkIt_board_merged-*.hex."""
    root = root or os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
    cands = glob.glob(os.path.join(root, "ports", "nrf52840", "build*", "LINKIT", "LinkIt_board_merged-*.hex"))
    cands.sort(key=lambda p: os.path.getmtime(p), reverse=True)
    return cands[0] if cands else None


def flash_merged(hexpath, log_dir=None, say=print):
    """CLAUDE.md sequence: --recover, then --program merged --chiperase --verify
    --reset, and BOTH 'Verify successful' and 'Run.' must be seen. Returns
    (ok: bool, report: str). Never retries."""
    if not hexpath or not os.path.exists(hexpath):
        return False, f"merged hex not found: {hexpath}"
    if "_merged-" not in os.path.basename(hexpath):
        return False, f"refusing {os.path.basename(hexpath)}: only the MERGED hex is flashed (CLAUDE.md)"
    say(f">> nrfjprog --recover (limit {RECOVER_TIMEOUT_S} s)")
    r = recover(log_dir=log_dir)
    if not r.ok:
        holders = probe_holders()
        why = "TIMEOUT after %ds" % RECOVER_TIMEOUT_S if r.timed_out else f"rc={r.returncode}"
        return False, (f"--recover failed ({why}). Not retried.\n{r.stdout.strip()[-600:]}"
                       + (f"\nprocesses holding the probe: {', '.join(holders)}" if holders else ""))
    say(f">> nrfjprog --program {os.path.basename(hexpath)} --chiperase --verify --reset")
    p = nrfjprog(["--program", hexpath, "--chiperase", "--verify", "--reset"], timeout=240, log_dir=log_dir)
    txt = p.stdout
    verified = "Verify successful" in txt or "Verified OK" in txt
    ran = re.search(r"(?m)^\s*Run\.\s*$", txt) is not None or "Run." in txt
    if p.ok and verified and ran:
        return True, ("flashed and verified ('Verify successful' + 'Run.'). The configuration store was "
                      "WIPED by --recover: re-apply every parameter a test depends on.")
    return False, (f"--program NOT confirmed (rc={p.returncode}, timeout={p.timed_out}, "
                   f"verify={'yes' if verified else 'NO'}, run={'yes' if ran else 'NO'}) -- the chip may still "
                   f"run the OLD image.\n{txt.strip()[-800:]}")


def soft_replug():
    """Software USB unplug/replug through SWD (USBPULLUP 0 -> 1). Platform
    neutral; needs the J-Link. Returns True if nrfjprog accepted all writes."""
    ok = True
    for v in ("0", "1"):
        ok &= nrfjprog(["--memwr", USBPULLUP_ADDR, "--val", v], timeout=60).ok
        ok &= nrfjprog(["--run"], timeout=60).ok
        if v == "0":
            time.sleep(4)
    return ok


# ---------------------------------------------------------------- lock
class BenchBusy(RuntimeError):
    pass


class BenchLock:
    """Exclusive non-blocking lock on a file in the temp dir.

    Two campaigns on one bench do not just share a port: each one resets the
    board under the other, which looks exactly like a firmware fault. The OS
    releases the lock when the process dies, on both platforms. Opened in 'a+'
    (not 'w') so a second process cannot truncate the holder's PID.
    """

    def __init__(self, name="dte_campaign.lock"):
        self.path = os.path.join(temp_dir(), name)
        self.f = open(self.path, "a+")
        try:
            if IS_WINDOWS:
                import msvcrt
                self.f.seek(0)
                msvcrt.locking(self.f.fileno(), msvcrt.LK_NBLCK, 1)
            else:
                import fcntl
                fcntl.flock(self.f, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError:
            self.f.close()
            raise BenchBusy(f"another campaign holds the bench (lock {self.path})")
        if not IS_WINDOWS:
            self.f.seek(0)
            self.f.truncate()
            self.f.write(str(os.getpid()))
            self.f.flush()

    def release(self):
        try:
            if IS_WINDOWS:
                import msvcrt
                self.f.seek(0)
                msvcrt.locking(self.f.fileno(), msvcrt.LK_UNLCK, 1)
            self.f.close()
        except Exception:
            pass
