#!/usr/bin/env python3
"""Bench checks for the Argos TX device-error backoff and suspension.

These are the on-board counterpart to the host tests in
tests/src/argos_tx_test.cpp. The host suite proves the state machine; only a
real board proves that the timings survive a main loop that is also driving a
KIM2, a GNSS receiver and an LFS commit on every warning line.

WHAT TO FLASH FIRST
-------------------
Build the bench firmware with a short suspension, or the probe check has to
wait an hour:

    ARGOS_TX_ERROR_SUSPEND_S=120 ./scripts/build_linkitv4_kim.sh --bench

Then flash as usual (tests/bench/flash.sh) and run:

    python3 tests/bench/argos_backoff_bench.py --run

Run tests/bench/kim_bench.py --run first. It is the regression baseline (10/10);
this script assumes the board is already known good.

FORCING THE FAILURE
-------------------
The interesting half only runs if Argos TX actually attempts and FAILS. On a
board with valid KIM2 credentials transmissions succeed and there is no error
ladder to watch -- that is a good outcome, and those checks report SKIP rather
than FAIL. To exercise the ladder deliberately, run against a board whose KIM2
credentials are absent or whose satellite module is disconnected: the TX is
then attempted and refused, which is exactly the fault this machinery exists
for.
"""

import argparse
import os
import re
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from kim_bench import Bench  # noqa: E402


# --- log lines this suite keys on -----------------------------------------
# All of them are WARN/ERROR/INFO, so they survive DEBUG_LEVEL=3. The em dash
# in the firmware strings is matched loosely on purpose: it travels over USB
# CDC as UTF-8 and a decode slip must not turn a real failure into a pass.
RX_STRIKE = r"KineisEventDeviceError \(consecutive=(\d+)/(\d+)\)"
RX_BACKOFF = r"ArgosTxService: backoff (\d+) ms before next TX attempt"
RX_SUSPEND = r"ArgosTxService: (\d+) consecutive device errors .* suspending TX for (\d+) s"
RX_HELD = r"service_initiate: skipping TX .* suspended for another (\d+) s"
RX_PROBE = r"service_initiate: suspension elapsed .* probing with one TX"
RX_CLEAR_GPS = r"clearing (\d+)-error suspension on new GPS session"
RX_CLEAR_SURF = r"clearing (\d+)-error suspension on surface event"


def count_since(b, pattern, from_idx):
    """How many history lines since `from_idx` match `pattern`."""
    rx = re.compile(pattern)
    with b._lock:
        lines = list(b.history[from_idx:])
    return sum(1 for _, line in lines if rx.search(line))


def sched(b, service="ARGOSTX", timeout=5.0):
    """(ms, reason) when a run is due, else (-1, reason).

    A third shape exists since the ScheduleDecision migration: hold<n>s means
    the service is waiting on something named but will re-interrogate itself in
    n seconds. It has no run deadline, so it reports -1 like none does -- the
    reason string, and the hold delay now carried in it, tell the two apart.
    """
    mk = b.mark()
    b._send("%SCHED\r\n")
    m = b.expect(r"%SCHED .*" + service + r"=(\d+)ms\(([^)]*)\)", timeout, from_idx=mk)
    if m:
        return int(m.group(1)), m.group(2)
    m = b.expect(r"%SCHED .*" + service + r"=hold(\d+)s\(([^)]*)\)", 0.5, from_idx=mk)
    if m:
        return -1, "hold %ss: %s" % (m.group(1), m.group(2))
    m = b.expect(r"%SCHED .*" + service + r"=none\(([^)]*)\)", 0.5, from_idx=mk)
    if m:
        return -1, m.group(1)
    return None, None


def schedq(b, timeout=5.0):
    """(immediate, deferred) scheduler queue occupancy, or (None, None)."""
    mk = b.mark()
    b._send("%SCHEDQ\r\n")
    m = b.expect(r"%SCHEDQ imm=(\d+)/\d+\(hw=\d+\) deferred=(\d+)/", timeout, from_idx=mk)
    return (int(m.group(1)), int(m.group(2))) if m else (None, None)


def dive(b, timeout=5.0):
    mk = b.mark()
    b._send("%DIVE\r\n")
    return b.expect(r"%DIVE OK", timeout, from_idx=mk) is not None


def surface(b, timeout=5.0):
    mk = b.mark()
    b._send("%SURFACE\r\n")
    return b.expect(r"%SURFACE OK", timeout, from_idx=mk) is not None


class BackoffSuite:
    def __init__(self, b, suspend_s):
        self.b = b
        self.suspend_s = suspend_s
        self.results = []

    def check(self, name, ok, detail=""):
        self.results.append((name, bool(ok), detail))
        tag = "\033[1;32mPASS\033[0m" if ok else "\033[1;31mFAIL\033[0m"
        print(f"[{tag}] {name}" + (f"  - {detail}" if detail else ""))
        return ok

    def skip(self, name, why):
        self.results.append((name, True, f"SKIPPED: {why}"))
        print(f"[\033[1;33mSKIP\033[0m] {name}  - {why}")

    def run(self):
        b = self.b
        print("\n=== Argos TX backoff / suspension - bench validation ===")
        print(f"    firmware built with ARGOS_TX_ERROR_SUSPEND_S={self.suspend_s} s\n")

        # 1. The board is alive and running the bench console.
        m = b.ping()
        if not self.check("handshake (%PING)", m is not None,
                          f"state={m.group(1)}" if m else "no %BENCH reply"):
            return False

        # 2. Operational, console drained. Everything below reads schedules, so
        #    a console still answering the previous command would measure the
        #    wrong service.
        if b.get_state() != "OPERATIONAL":
            b.exit_config()
        b.settle_console(20.0)
        self.check("operational, console settled", b.get_state() == "OPERATIONAL",
                   f"state={b.get_state()}")

        # 3. Argos TX is scheduled at all. This is the baseline every later
        #    check is a delta against, and it is also the check that would
        #    catch the suspension firing spuriously at boot.
        ms, why = sched(b)
        self.check("%SCHED reports ARGOSTX", ms is not None,
                   f"ARGOSTX={ms}ms({why})" if ms is not None else "no ARGOSTX field")
        baseline_ms = ms

        # 4. No suspension in a healthy session. A board that has just booted
        #    must not be holding TX; if it is, everything after this is noise.
        held_at_start = count_since(b, RX_SUSPEND, 0)
        self.check("no suspension armed at boot", held_at_start == 0,
                   f"{held_at_start} suspension log(s) before any test action")

        # 5. Lift the RTC gate so TX is actually attempted rather than held.
        mk = b.mark()
        b.inject_gps(-21.0097, 55.2707, timeout=20.0)
        b.expect(r"bench_inject_fix: lat=-21", 20.0, from_idx=mk)
        self.check("GPS injected (lifts the first-fix TX gate)", True,
                   "-21.0097,55.2707")

        # 6. Watch for the error ladder. Three failed transmissions take at
        #    least 60 + 120 s of backoff, so give it four minutes.
        print("\n    watching for a device-error ladder (up to 4 min)...")
        ladder_mk = b.mark()
        first = b.expect(RX_STRIKE, 240.0, from_idx=ladder_mk)

        if first is None:
            why = "no device error in 4 min - TX is succeeding or still gated"
            for name in ("backoff ladder is 60 s then 120 s",
                         "suspension arms on the third strike",
                         "suspension does not spin",
                         "exactly one probe when the deadline passes",
                         "GPS session clears the suspension",
                         "surface event clears the suspension"):
                self.skip(name, why)
            return self.summary()

        # 6a. The ladder itself: 60 s, then 120 s. These are the numbers the
        #     host test pins; here they must also survive a real main loop.
        b1 = b.expect(RX_BACKOFF, 30.0, from_idx=ladder_mk)
        b2 = None
        if b1 and b1.group(1) == "60000":
            after1 = b.mark()
            b2 = b.expect(RX_BACKOFF, 200.0, from_idx=after1)
        got = f"{b1.group(1) if b1 else '?'} then {b2.group(1) if b2 else '?'}"
        self.check("backoff ladder is 60 s then 120 s",
                   b1 is not None and b1.group(1) == "60000"
                   and b2 is not None and b2.group(1) == "120000",
                   f"backoff {got} ms")

        # 6b. Third strike arms the deadline, and the deadline is the one the
        #     firmware was built with.
        susp = b.expect(RX_SUSPEND, 240.0, from_idx=ladder_mk)
        self.check("suspension arms on the third strike",
                   susp is not None and int(susp.group(1)) == 3
                   and int(susp.group(2)) == self.suspend_s,
                   f"{susp.group(1)} strikes, {susp.group(2)} s"
                   if susp else "no suspension log")
        if susp is None:
            for name in ("suspension does not spin",
                         "exactly one probe when the deadline passes",
                         "GPS session clears the suspension",
                         "surface event clears the suspension"):
                self.skip(name, "suspension never armed")
            return self.summary()

        # 7. THE REGRESSION THIS SCRIPT EXISTS FOR. Before the probe deadline
        #    landed, completing without a reschedule left the safety-net
        #    timeout armed; it fired, rescheduled at zero delay, hit the guard
        #    in service_initiate and rearmed itself. The board woke, logged and
        #    skipped once per service_next_timeout (~30 s), forever, writing an
        #    LFS commit every pass. Watch a window shorter than the deadline
        #    and count how many times it wakes.
        window = max(30, min(self.suspend_s - 20, 90))
        print(f"\n    watching {window} s of suspension for wake-up spam...")
        spin_mk = b.mark()
        time.sleep(window)
        held = count_since(b, RX_HELD, spin_mk)
        probes = count_since(b, RX_PROBE, spin_mk)
        # One or two holds are fine -- a schedule already in flight when the
        # suspension armed still lands on the guard. The old spin produced one
        # every ~30 s, so anything at that cadence is the bug coming back.
        budget = max(2, window // 60)
        self.check("suspension does not spin", held <= budget and probes == 0,
                   f"{held} hold(s) in {window} s (budget {budget}), {probes} probe(s)")

        ms, why = sched(b)
        self.check("service parked, not rescheduling at zero delay",
                   ms is None or ms == -1 or ms > 30000,
                   f"ARGOSTX={ms}ms({why})")

        # 8. The deadline passes and exactly one dispatch is let through.
        print(f"\n    waiting out the rest of the {self.suspend_s} s deadline...")
        probe_mk = b.mark()
        probe = b.expect(RX_PROBE, self.suspend_s + 90.0, from_idx=probe_mk)
        self.check("exactly one probe when the deadline passes",
                   probe is not None and count_since(b, RX_PROBE, probe_mk) == 1,
                   f"{count_since(b, RX_PROBE, probe_mk)} probe log(s)")

        # 9. A GPS session clears the suspension without waiting for the next
        #    deadline. This is the 2026-06-30 field case: a land tracker
        #    recovering from a transient module fault without a power cycle.
        gps_mk = b.mark()
        b.inject_gps(-21.0097, 55.2707, timeout=20.0)
        cleared = b.expect(RX_CLEAR_GPS, 30.0, from_idx=gps_mk)
        self.check("GPS session clears the suspension", cleared is not None,
                   f"cleared {cleared.group(1)} strikes" if cleared else "no clearing log")
        if cleared:
            ms, why = sched(b)
            # Clearing the counter alone used to leave the earliest-TX floor an
            # hour out. The reschedule that comes with it is what makes the
            # recovery prompt, and this is where that would show.
            self.check("recovery is prompt, not parked on the old floor",
                       ms is not None and ms != -1 and ms < self.suspend_s * 1000,
                       f"ARGOSTX={ms}ms({why})")

        # 10. And a surface event does the same, through the base class rather
        #     than through the GNSS branch.
        surf_mk = b.mark()
        if dive(b) and surface(b):
            cl = b.expect(RX_CLEAR_SURF, 30.0, from_idx=surf_mk)
            if cl is None and count_since(b, RX_STRIKE, surf_mk) == 0:
                self.skip("surface event clears the suspension",
                          "no strikes outstanding at surfacing - nothing to clear")
            else:
                self.check("surface event clears the suspension", cl is not None,
                           f"cleared {cl.group(1)} strikes" if cl else "no clearing log")
        else:
            self.skip("surface event clears the suspension", "%DIVE/%SURFACE not accepted")

        # 11. Nothing leaked into the scheduler queue along the way.
        imm, deferred = schedq(b)
        self.check("scheduler queue not leaking", imm is not None and deferred is not None,
                   f"imm={imm} deferred={deferred} (baseline ARGOSTX={baseline_ms}ms)")

        return self.summary()

    def summary(self):
        npass = sum(1 for _, ok, _ in self.results if ok)
        print(f"\n=== {npass}/{len(self.results)} checks passed ===")
        print(f"Transcript: {self.b.logpath}")
        return all(ok for _, ok, _ in self.results)


def main():
    ap = argparse.ArgumentParser(description="Argos TX backoff bench checks")
    ap.add_argument("--port", default=None, help="serial port (autodetected if omitted)")
    ap.add_argument("--run", action="store_true", help="run the suite")
    ap.add_argument("--suspend-s", type=int, default=120,
                    help="ARGOS_TX_ERROR_SUSPEND_S the firmware was built with (default 120)")
    ap.add_argument("--quiet", action="store_true", help="do not echo the serial stream")
    args = ap.parse_args()

    b = Bench(port=args.port, quiet=args.quiet)
    try:
        b.open()
        if not args.run:
            print("Nothing to do (pass --run).")
            return 0
        return 0 if BackoffSuite(b, args.suspend_s).run() else 1
    except KeyboardInterrupt:
        return 130
    finally:
        b.close()


if __name__ == "__main__":
    sys.exit(main())
