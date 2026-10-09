#!/usr/bin/env python3
"""DX1 v3 - reference encoder + reference behaviour tracker (source of truth for test vectors).

Frame (MSB-first):
  byte0      0x00 marker (frozen)
  byte1      BATT7 | LB (frozen, packet_field_encoding.hpp)
  byte2      VER(2)=01 | EXT(1)=0 | DRY(3) | DEND(2)
  byte3      CRC8(bytes 0..2)              (views >= 32 bits only)
  bits 32-111 rich block                   (LDK view only)
  bits 112-127 reserved 0x00 (Phase 2)
Views: VLDA4 = 3 B / 24, LDA2 = 4 B / 32, LDK = 16 B / 128.
"""
import json
import sys

# ----------------------------------------------------------------- codecs


def crc8(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            if crc & 0x8000:
                crc ^= 0x8380
            crc = (crc << 1) & 0xFFFF
    return (crc >> 8) & 0xFF


def batt(mv):
    return min(127, max(mv - 2700, 0) // 20)


def em(v, E, M):
    """Floor quasi-log. Code 0 is reserved for 'unknown'; caller passes v >= 1."""
    assert v >= 1
    base = 1 << M
    if v < base:
        return v
    k = v.bit_length() - 1
    e = k - M + 1
    m = (v >> (k - M)) - base
    if e > (1 << E) - 1:
        return (1 << (E + M)) - 1
    return (e << M) | m


def em_dec(c, E, M):
    if c == 0:
        return None
    base = 1 << M
    e, m = c >> M, c & (base - 1)
    return m if e == 0 else (base + m) << (e - 1)


def q4(n):
    if n <= 7:
        return n
    for code, lo in ((15, 4096), (14, 1024), (13, 256), (12, 128), (11, 64), (10, 32), (9, 16), (8, 8)):
        if n >= lo:
            return code


OBS_MIN_S = 43200
BOUNDS = [20, 40, 80, 160, 320, 640]  # per-mille lower bounds of classes 2..7


def pct_code(dry_s, obs_s):
    if obs_s < OBS_MIN_S:
        return 0
    p = (200 * dry_s + obs_s) // (2 * obs_s)
    return 1 + min(p, 100)


def dry_class(dry_s, obs_s):
    if obs_s < OBS_MIN_S:
        return 0
    pm = (1000 * dry_s) // obs_s
    c = 1
    for i, lo in enumerate(BOUNDS):
        if pm >= lo:
            c = i + 2
    return c


class Bits:
    def __init__(self, nbytes):
        self.b = bytearray(nbytes)
        self.pos = 0

    def put(self, v, w):
        assert 0 <= v < (1 << w), (v, w)
        for i in range(w - 1, -1, -1):
            if (v >> i) & 1:
                self.b[self.pos // 8] |= 0x80 >> (self.pos % 8)
            self.pos += 1


RICH_LAYOUT = [  # (name, width) bits 32..111
    ("latch", 2), ("blind", 1), ("partial", 1), ("surf_seq", 9), ("rst", 3),
    ("pct", 7), ("n_surf", 7), ("s_last", 7), ("d_prev", 7), ("d_max", 7),
    ("delay", 6), ("contrast", 6), ("dto", 4), ("fsc", 4), ("src", 4),
    ("method", 3), ("dfloor", 1), ("adcf", 1),
]
assert sum(w for _, w in RICH_LAYOUT) == 80


def image(mv, lb, dry, dend, rich=None):
    """Full 16-byte DX1 image. rich = dict of already-encoded codes, or None (all zero, LATCH=0)."""
    b = Bits(16)
    b.put(0, 8)
    b.put(batt(mv), 7)
    b.put(1 if lb else 0, 1)
    b.put(1, 2)
    b.put(0, 1)
    b.put(dry, 3)
    b.put(dend, 2)
    b.put(crc8(bytes(b.b[:3])), 8)
    if rich:
        for name, w in RICH_LAYOUT:
            b.put(rich.get(name, 0), w)
    return bytes(b.b)


def view(img, mode):
    n = {"VLDA4": 3, "LDA2": 4, "LDK": 16}[mode]
    return img[:n], n * 8


def legacy(mv, lb):
    return bytes([0, (batt(mv) << 1) | (1 if lb else 0), 0])


def health_codes(delay_us=0, contrast_x10=0, calibrated=True, threshold_air=1, dto=0, fsc=0, src=0,
                 delay_min=0, delay_max=0, adcf=False):
    return {
        "delay": 0 if not delay_us else em(max(1, (delay_us + 25) // 50), 3, 3),
        "contrast": 0 if (not calibrated or not contrast_x10 or not threshold_air) else em(max(1, contrast_x10), 3, 3),
        "dto": q4(dto), "fsc": q4(fsc), "src": q4(src),
        "dfloor": 1 if (delay_us > 0 and delay_us == delay_min and delay_min < delay_max) else 0,
        "adcf": 1 if adcf else 0,
    }


# ----------------------------------------------------------------- tracker

BLOCK_MS = 21_600_000
UNKNOWN, UW, SURF = 0, 1, 2


class Tracker:
    """Reference model of DiveBehaviourTracker (v3 semantics). Times in ms since tracker reset."""

    def __init__(self, reset_ms=0):
        self.reset_ms = reset_ms
        self.state = UNKNOWN
        self.blind = False
        self.blind_start = 0
        self.acct_from = reset_ms
        self.ring = {}  # block id -> dict(dry, obs, n, dmax, blind)
        self.seq = 0
        # current dive
        self.has_snap = False
        self.dive_start = 0
        self.dive_start_real = False
        self.dive_blinded = False
        self.dto0 = self.fsc0 = 0
        # surface edge
        self.t_s = 0
        self.level_s = 0
        self.dto_s = self.fsc_s = 0
        self.adcf_s = False
        self.surface_known = False
        # latch
        self.latched = False
        self.l = {}

    # ---- block helpers
    def bid(self, t):
        return (t - self.reset_ms) // BLOCK_MS

    def blk(self, i):
        if i not in self.ring:
            self.ring[i] = dict(dry=0, obs=0, n=0, dmax=0, blind=False)
            for k in [k for k in self.ring if k < i - 4]:
                del self.ring[k]
        return self.ring[i]

    def account(self, t0, t1, dry):
        """Add [t0,t1) to obs (and dry). Integer ms, split across blocks, credited in whole seconds per block."""
        t = t0
        while t < t1:
            i = self.bid(t)
            end = min(t1, self.reset_ms + (i + 1) * BLOCK_MS)
            s = (end - t) // 1000
            b = self.blk(i)
            b["obs"] += s
            if dry:
                b["dry"] += s
            t = end

    def mark_blind(self, t0, t1):
        for i in range(self.bid(t0), self.bid(max(t0, t1)) + 1):
            self.blk(i)["blind"] = True

    # ---- inputs
    def on_blind_start(self, now):
        if self.blind:
            return
        self._close_open_episode(now)
        self.blind = True
        self.blind_start = now
        self.dive_blinded = True
        self.mark_blind(now, now)

    def on_blind_end(self, now):
        if not self.blind:
            return
        self.mark_blind(self.blind_start, now)
        self.blind = False
        self.state = UNKNOWN
        self.acct_from = now

    def on_test_event(self, now):
        """UW event while SWS test mode runs: ignored, state lost, window flagged."""
        self._close_open_episode(now)
        self.state = UNKNOWN
        self.dive_blinded = True
        self.mark_blind(now, now)
        self.acct_from = now

    def _surface_forced(self):
        return (self.has_snap and self.fsc_s > self.fsc0) or self.adcf_s

    def _close_open_episode(self, now):
        if self.state == UW:
            self.account(self.acct_from, now, False)
        elif self.state == SURF:
            if self.acct_from < self.t_s:
                self.account(self.acct_from, self.t_s, False)
            if not self._surface_forced():
                self.account(max(self.acct_from, self.t_s), now, True)
        self.acct_from = now

    def on_uw_event(self, uw, now, dto=0, fsc=0, adcf=False, level=0):
        if self.blind:
            return
        if self.state == UNKNOWN:
            self.state = UW if uw else SURF
            self.acct_from = now
            if uw:
                self.has_snap = True
                self.dive_start = now
                self.dive_start_real = False
                self.dive_blinded = False
                self.dto0, self.fsc0 = dto, fsc
            else:
                # surface entered without an edge: nothing about it is real
                self.t_s = now
                self.level_s = 0
                self.dto_s, self.fsc_s, self.adcf_s = dto, fsc, adcf
                self.surface_known = False
            return
        if (self.state == UW) == uw:
            return  # re-emission
        if not uw:  # UW -> SURF : store only
            self.state = SURF
            self.t_s = now
            self.level_s = level
            self.dto_s, self.fsc_s, self.adcf_s = dto, fsc, adcf
            self.surface_known = True
            return
        # SURF -> UW : dive edge, all the work
        forced = self._surface_forced()
        surface_real = self.surface_known and not forced
        # accounting: [acct_from, t_s) wet, [t_s, now) dry if real else unobserved
        if self.acct_from < self.t_s:
            self.account(self.acct_from, self.t_s, False)
        if surface_real:
            self.account(max(self.acct_from, self.t_s), now, True)
        # closed dive
        dive_known = self.surface_known and self.has_snap and self.dive_start_real and not self.dive_blinded
        dive_s = (self.t_s - self.dive_start) // 1000 if dive_known else None
        if dive_known:
            b = self.blk(self.bid(self.t_s))
            b["dmax"] = max(b["dmax"], max(1, dive_s))
        s_last = None
        if surface_real:
            self.seq = self.seq % 511 + 1
            self.blk(self.bid(now))["n"] += 1
            s_last = max(1, (now - self.t_s) // 1000)
        if not self.surface_known:
            method = 0
        elif forced:
            method = 6
        else:
            method = self.level_s if 1 <= self.level_s <= 5 else 6
        # window
        cur = self.bid(now)
        comp = [self.ring[i] for i in range(cur - 4, cur) if i >= 0 and i in self.ring]
        partial = cur < 4
        dry_s = sum(b["dry"] for b in comp)
        obs_s = sum(b["obs"] for b in comp)
        blind = self.blind or any(self.ring[i]["blind"] for i in range(cur - 4, cur + 1) if i in self.ring)
        n = sum(b["n"] for b in comp)
        dmax = max([b["dmax"] for b in comp] + [0])
        self.l = {
            "latch": 1, "blind": 1 if blind else 0, "partial": 1 if partial else 0, "surf_seq": self.seq,
            "pct": pct_code(dry_s, obs_s), "dry": dry_class(dry_s, obs_s),
            "n_surf": 0 if partial else em(n + 1, 4, 3),
            "s_last": 0 if s_last is None else em(s_last, 4, 3),
            "d_prev": 0 if dive_s is None else em(max(1, dive_s), 4, 3),
            "d_max": 0 if (partial or dmax == 0) else em(dmax, 4, 3),
            "method": method,
        }
        self.latched = True
        # new dive
        self.state = UW
        self.acct_from = now
        self.has_snap = True
        self.dive_start = now
        self.dive_start_real = surface_real
        self.dive_blinded = False
        self.dto0, self.fsc0 = dto, fsc

    # ---- outputs
    def dend(self, dto_now=0, fsc_now=0, adcf_now=False):
        if self.state == UNKNOWN or not self.has_snap:
            return 0
        if self.state == UW:
            dd, fd, af = dto_now - self.dto0, fsc_now - self.fsc0, adcf_now
        else:
            if not self.surface_known:
                return 0
            dd, fd, af = self.dto_s - self.dto0, self.fsc_s - self.fsc0, self.adcf_s
        if dd < 0 or fd < 0:
            return 0
        if fd > 0 or af:
            return 3
        if dd > 0:
            return 2
        return 1

    def latch(self, now):
        if self.latched:
            return dict(self.l)
        cur = self.bid(now)
        return {"latch": 2, "blind": 1 if (self.blind or any(b["blind"] for b in self.ring.values())) else 0,
                "partial": 1 if cur < 4 else 0, "surf_seq": self.seq, "pct": 0, "dry": 0, "n_surf": 0,
                "s_last": 0, "d_prev": 0, "d_max": 0, "method": 0}


def build(tr, now, mv, lb, health, rst, counters, mode):
    lt = tr.latch(now)
    dend = tr.dend(*counters)
    rich = dict(lt)
    rich.update(health)
    rich["rst"] = rst
    img = image(mv, lb, lt["dry"], dend, rich)
    return view(img, mode)


def H(b):
    return b.hex().upper()


# ----------------------------------------------------------------- vectors

def vectors():
    V = {"codecs": {}, "frames": [], "scenarios": []}
    V["codecs"]["crc8_123456789"] = crc8(b"123456789")
    V["codecs"]["e4m3"] = [[v, em(v, 4, 3), em_dec(em(v, 4, 3), 4, 3)] for v in
                           (1, 7, 8, 15, 16, 60, 90, 1200, 3480, 21600, 245760, 300000)]
    V["codecs"]["e3m3"] = [[v, em(v, 3, 3), em_dec(em(v, 3, 3), 3, 3)] for v in (4, 20, 200, 600, 960, 2000)]
    V["codecs"]["q4"] = [[n, q4(n)] for n in (0, 7, 8, 15, 16, 255, 256, 1023, 1024, 4095, 4096, 65535)]
    V["codecs"]["pct"] = [[d, o, pct_code(d, o)] for d, o in
                          ((5789, 86400), (2600, 86400), (86400, 86400), (0, 86400), (100, 43199), (21600, 43200))]
    V["codecs"]["dry_class"] = [[d, o, dry_class(d, o)] for d, o in
                                ((0, 86400), (1727, 86400), (1728, 86400), (3456, 86400), (5789, 86400),
                                 (13824, 86400), (27648, 86400), (55296, 86400), (86400, 86400), (100, 43199))]
    V["codecs"]["legacy"] = [[3700, 0, H(legacy(3700, 0))], [2800, 1, H(legacy(2800, 1))]]

    # pure frame vectors (field codes in, bytes out)
    for name, mv, lb, dry, dend in (("compact_3700_d3_e1", 3700, 0, 3, 1), ("compact_unknown", 3700, 0, 0, 0),
                                    ("compact_3600_d2_e3", 3600, 0, 2, 3), ("compact_2800lb_d7_e0", 2800, 1, 7, 0)):
        img = image(mv, lb, dry, dend)
        V["frames"].append({"name": name, "mv": mv, "lb": lb, "dry": dry, "dend": dend, "rich": None,
                            **{m: H(view(img, m)[0]) for m in ("VLDA4", "LDA2", "LDK")}})
    rich_e = {"latch": 1, "blind": 0, "partial": 0, "surf_seq": 59, "rst": 0, "pct": pct_code(5789, 86400),
              "n_surf": em(59 + 1, 4, 3), "s_last": em(90, 4, 3), "d_prev": em(1200, 4, 3), "d_max": em(3480, 4, 3),
              **health_codes(1000, 150, dto=0, fsc=0, src=0, delay_min=200, delay_max=10000), "method": 2}
    img = image(3700, 0, dry_class(5789, 86400), 1, rich_e)
    V["frames"].append({"name": "rich_nominal", "mv": 3700, "lb": 0, "dry": dry_class(5789, 86400), "dend": 1,
                        "rich": rich_e, **{m: H(view(img, m)[0]) for m in ("VLDA4", "LDA2", "LDK")}})
    rich_g = {"latch": 1, "blind": 0, "partial": 0, "surf_seq": 300, "rst": 2, "pct": pct_code(2600, 86400),
              "n_surf": em(20 + 1, 4, 3), "s_last": em(75, 4, 3), "d_prev": em(21600, 4, 3), "d_max": em(21600, 4, 3),
              **health_codes(1000, 120, dto=6, fsc=2, src=1, delay_min=200, delay_max=10000), "method": 6}
    img = image(3600, 0, dry_class(2600, 86400), 3, rich_g)
    V["frames"].append({"name": "rich_forced", "mv": 3600, "lb": 0, "dry": dry_class(2600, 86400), "dend": 3,
                        "rich": rich_g, **{m: H(view(img, m)[0]) for m in ("VLDA4", "LDA2", "LDK")}})
    rich_h = {"latch": 1, "blind": 0, "partial": 0, "surf_seq": 511, "rst": 7, "pct": pct_code(86400, 86400),
              "n_surf": em(0 + 1, 4, 3), "s_last": 0, "d_prev": 0, "d_max": 0,
              **health_codes(200, 32, dto=0, fsc=0, src=9, delay_min=200, delay_max=10000, adcf=True), "method": 1}
    img = image(2800, 1, 7, 3, rich_h)
    V["frames"].append({"name": "rich_surface_lock_adcf", "mv": 2800, "lb": 1, "dry": 7, "dend": 3,
                        "rich": rich_h, **{m: H(view(img, m)[0]) for m in ("VLDA4", "LDA2", "LDK")}})

    # scenario vectors: drive the reference tracker, record LDK image + LDA2 view at checkpoints
    hc = health_codes(1000, 150, delay_min=200, delay_max=10000)
    S = 1000

    def sc_cycles():
        out = []
        tr = Tracker(0)
        t = 0
        tr.on_uw_event(True, t)  # boot re-emission underwater: not an edge
        out.append(("boot_uw", t, H(build(tr, t, 3700, 0, hc, 0, (0, 0, False), "LDK")[0])))
        dives = 0
        while t < 30 * 3600 * S:
            t += 1200 * S
            tr.on_uw_event(False, t, level=2)
            if dives < 2:
                out.append((f"surface_{dives+1}", t, H(build(tr, t, 3700, 0, hc, 0, (0, 0, False), "LDK")[0])))
            t += 90 * S
            tr.on_uw_event(True, t)
            dives += 1
            if dives <= 2:
                out.append((f"dive_edge_{dives}", t, H(build(tr, t, 3700, 0, hc, 0, (0, 0, False), "LDK")[0])))
        out.append(("after_30h", t, H(build(tr, t, 3700, 0, hc, 0, (0, 0, False), "LDK")[0])))
        out.append(("after_30h_lda2", t, H(build(tr, t, 3700, 0, hc, 0, (0, 0, False), "LDA2")[0])))
        return out

    def sc_forced():
        out = []
        tr = Tracker(0)
        t = 0
        tr.on_uw_event(False, t)                       # boot at surface: no edge
        t += 60 * S
        tr.on_uw_event(True, t, dto=0, fsc=0)          # real dive edge? surface not known -> dive start not real
        t += 1000 * S
        tr.on_uw_event(False, t, dto=0, fsc=0, level=1)  # real surface edge
        out.append(("surf_normal_dend", t, tr.dend(0, 0, False)))
        t += 120 * S
        tr.on_uw_event(True, t, dto=0, fsc=0)          # dive edge: S_LAST 120, D_PREV unknown (start not real)
        out.append(("dive_after_normal", t, H(build(tr, t, 3700, 0, hc, 0, (0, 0, False), "LDK")[0])))
        out.append(("uw_inprogress_dend_after_timeout", t + 7200 * S, tr.dend(1, 0, False)))
        t += 21600 * S
        tr.on_uw_event(False, t, dto=3, fsc=1, level=0)  # forced surface (escalation)
        out.append(("forced_surface_dend", t, tr.dend(3, 1, False)))
        out.append(("forced_surface_ping2_lda2", t, H(build(tr, t, 3600, 0, hc, 0, (3, 1, False), "LDA2")[0])))
        t += 30 * S
        tr.on_uw_event(True, t, dto=3, fsc=1)          # SWS re-detects water: closes a forced episode
        out.append(("dive_after_forced", t, H(build(tr, t, 3600, 0, hc, 0, (3, 1, False), "LDK")[0])))
        t += 600 * S
        tr.on_uw_event(False, t, dto=3, fsc=1, level=3)
        t += 60 * S
        tr.on_uw_event(True, t, dto=3, fsc=1)          # D_PREV unknown (start followed a forced episode)
        out.append(("dive_after_post_forced", t, H(build(tr, t, 3600, 0, hc, 0, (3, 1, False), "LDK")[0])))
        return out

    def sc_blind():
        out = []
        tr = Tracker(0)
        t = 0
        tr.on_uw_event(True, t)
        t += 1200 * S; tr.on_uw_event(False, t, level=2)
        t += 90 * S; tr.on_uw_event(True, t)           # first real dive edge (dive start real from here)
        t += 1200 * S; tr.on_uw_event(False, t, level=2)
        t += 90 * S; tr.on_uw_event(True, t)
        out.append(("before_blind", t, H(build(tr, t, 3700, 0, hc, 0, (0, 0, False), "LDK")[0])))
        tr.on_blind_start(t + 10 * S)
        tr.on_uw_event(False, t + 500 * S, level=2)    # ignored while blind
        tr.on_blind_end(t + 2710 * S)
        t += 2710 * S
        tr.on_uw_event(True, t)                        # re-emission after blind: state set, not an edge
        t += 1200 * S; tr.on_uw_event(False, t, level=4)
        t += 90 * S; tr.on_uw_event(True, t)           # D_PREV unknown (dive start not real), S_LAST 90, BLIND 1
        out.append(("after_blind", t, H(build(tr, t, 3700, 0, hc, 0, (0, 0, False), "LDK")[0])))
        tr.on_blind_end(t + 5)                         # no-op without start
        return out

    for name, fn in (("cycles_20min_90s", sc_cycles), ("forced_surface", sc_forced), ("cooldown_blind", sc_blind)):
        V["scenarios"].append({"name": name, "checkpoints": [{"label": a, "t_ms": b, "value": c} for a, b, c in fn()]})
    return V


if __name__ == "__main__":
    V = vectors()
    if len(sys.argv) > 1:
        with open(sys.argv[1], "w") as f:
            json.dump(V, f, indent=1)
    print(json.dumps(V, indent=1))
