#!/usr/bin/env python3
"""
SWS Analog Detection Optimizer
Simulates the multi-level detection algorithm with realistic ADC scenarios
based on real field test data and tests parameter combinations to find optimal values.

Usage: python3 scripts/sws_optimizer.py
"""

import itertools
import random
from dataclasses import dataclass
from typing import List, Tuple

# ─── Simulation Parameters ───

@dataclass
class Params:
    # Threshold
    threshold_ratio_pct: int = 35
    hysteresis_pct: int = 6
    alpha_pct: int = 19
    min_dry_samples: int = 1

    # Multi-level detection
    L1_drop_pct: int = 15
    L2_drop_pct: int = 5
    L2_min_consecutive: int = 2
    L3_min_consecutive: int = 3
    L3_drop_pct: int = 3
    L4_drop_pct: int = 15
    L5_drop_pct: int = 15
    L5_min_time: int = 10

    # Guards
    proximity_guard_pct: int = 95
    override_min_time: int = 2
    surface_lockout: int = 10
    min_surface_time: int = 10
    max_dive_time: int = 7200


# ─── ADC Scenario Generator ───

@dataclass
class Scenario:
    name: str
    # Either a raw ADC sequence with expected state, or auto-generated
    raw_sequence: List[Tuple[int, bool]] = None  # (adc, expected_underwater)
    # For auto-generated scenarios
    adc_air: int = 0
    adc_water: int = 0
    noise_air: int = 0
    noise_water: int = 0
    transitions: List[Tuple[str, int]] = None
    transition_samples: int = 3
    biofouling_drift_per_sample: float = 0.0
    weight: float = 1.0  # Score weight for this scenario


def generate_adc_sequence(sc: Scenario, sample_interval: float = 1.0) -> List[Tuple[int, bool]]:
    """Generate (adc_value, expected_underwater) pairs for a scenario."""
    if sc.raw_sequence:
        return sc.raw_sequence

    sequence = []
    rng = random.Random(42)

    for state, duration in (sc.transitions or []):
        n_samples = int(duration / sample_interval)
        is_uw = (state == "water")

        for i in range(n_samples):
            if state == "water":
                base = sc.adc_water
                noise = rng.randint(-sc.noise_water, sc.noise_water)
                drift = int(base * sc.biofouling_drift_per_sample * i)
                val = base + noise - drift
            elif state == "air":
                base = sc.adc_air
                noise = rng.randint(-sc.noise_air, sc.noise_air)
                val = base + noise
            elif state == "transition_out":
                # Water -> Air: exponential decay (electrode drying)
                progress = min(1.0, i / max(1, sc.transition_samples))
                # Drying is not linear - fast initial drop then slow tail
                exp_progress = 1 - (1 - progress) ** 2
                base = int(sc.adc_water * (1 - exp_progress) + sc.adc_air * exp_progress)
                noise = rng.randint(-sc.noise_water // 2, sc.noise_water // 2)
                val = base + noise
                is_uw = (i < 1)  # Only first sample considered UW during transition
            elif state == "transition_in":
                # Air -> Water: fast (immersion is nearly instant for electrodes)
                progress = min(1.0, i / max(1, sc.transition_samples))
                exp_progress = progress ** 0.5  # Fast rise
                base = int(sc.adc_air * (1 - exp_progress) + sc.adc_water * exp_progress)
                noise = rng.randint(-sc.noise_water // 2, sc.noise_water // 2)
                val = base + noise
                is_uw = (i >= 1)  # After first sample, should be UW
            else:
                val = sc.adc_air
                is_uw = False

            sequence.append((max(0, min(16383, val)), is_uw))

    return sequence


# ─── Detector Simulation (mirrors sws_analog_service.cpp) ───

class SWSDetectorSim:
    def __init__(self, params: Params):
        self.p = params
        self.reset()

    def reset(self):
        self.air = 0
        self.water = 0
        self.threshold = 0
        self.hysteresis = 0
        self.calibrated = False

        self.current_state = False
        self.time_in_state = 0
        self.consecutive_samples = 0

        self.adc_history = [0, 0]
        self.adc_idx = 0

        self.prev_raw = 0
        self.drop_reference = 0
        self.consecutive_raw_drops = 0

        self.trend_buffer = [0, 0, 0]
        self.trend_idx = 0
        self.trend_count = 0
        self.prev_ma3 = 0
        self.ma3_trend_start = 0
        self.ma3_trend_count = 0

        self.peak_adc = 0
        self.min_adc = 0xFFFF

        self.lockout_remaining = 0
        self.first_sample_done = False

        # Stats
        self.total_samples = 0
        self.correct = 0
        self.false_positives = 0
        self.false_negatives = 0
        self.detection_latencies = []
        self._last_expected = None
        self._transition_counter = 0
        self._waiting_for_detection = False
        self.level_triggers = {1: 0, 2: 0, 3: 0, 4: 0, 5: 0}
        self.false_surface_triggers = 0  # L-override while actually underwater

    def calibrate_initial(self, first_value: int):
        if first_value > 2500:
            self.water = first_value
            self.air = first_value // 3
        else:
            self.air = first_value
            self.water = first_value * 3
            if self.water > 8000:
                self.water = 8000
        self._update_threshold()
        self.calibrated = True

    def _update_threshold(self):
        if self.water <= self.air:
            return
        contrast = self.water / max(1, self.air) if self.air > 0 else 50.0
        if contrast >= 8.0:
            ratio = self.p.threshold_ratio_pct / 100.0
        elif contrast >= 4.0:
            ratio = 0.65
        else:
            ratio = 0.80
        rng = self.water - self.air
        self.threshold = int(self.air + rng * ratio)
        self.hysteresis = max(10, int(self.threshold * self.p.hysteresis_pct / 100.0))

    def _calibrate_water(self, value: int):
        alpha = self.p.alpha_pct / 100.0
        threshold_margin = self.threshold + self.hysteresis
        min_expected = int(self.water * 0.85)

        # Only apply air ratio guard when air is reasonable (<1000)
        air_ratio_ok = (self.air >= 1000) or (value >= self.air * 3)

        ok = (value > threshold_margin and
              value >= 2000 and
              air_ratio_ok and
              (value >= min_expected or value > self.water))
        if ok:
            self.water = int(alpha * value + (1 - alpha) * self.water)
            self._update_threshold()

    def _add_to_history(self, value: int) -> int:
        self.adc_history[self.adc_idx] = value
        self.adc_idx = (self.adc_idx + 1) % 2
        vals = [v for v in self.adc_history if v > 0]
        return sum(vals) // len(vals) if vals else value

    def process_sample(self, raw_value: int, expected_uw: bool):
        self.total_samples += 1

        if not self.calibrated:
            self.calibrate_initial(raw_value)

        # First-sample coherence check
        if not self.first_sample_done:
            self.first_sample_done = True
            incoherent = False
            if (raw_value > self.threshold + self.hysteresis * 3 and
                    raw_value > self.water * 1.3):
                incoherent = True
            elif raw_value < self.air * 0.5 and self.air > 2000:
                incoherent = True
            if incoherent:
                self.calibrate_initial(raw_value)

        prev_raw = self.prev_raw
        self.prev_raw = raw_value
        filtered = self._add_to_history(raw_value)

        self.time_in_state += 1

        # ── Multi-level surface detection ──
        surface_level = 0

        # Use max(water, peak) as reference to handle EMA drift
        proximity_ref = max(self.water, self.peak_adc) if self.peak_adc > 0 else self.water
        proximity_ok = (proximity_ref == 0 or
                        filtered < int(proximity_ref * self.p.proximity_guard_pct / 100.0))

        # L1 & L2
        if (self.current_state and prev_raw > 0 and
                self.time_in_state >= self.p.override_min_time and proximity_ok):
            if raw_value < prev_raw:
                if self.consecutive_raw_drops == 0:
                    self.drop_reference = prev_raw
                self.consecutive_raw_drops += 1
                if self.drop_reference > 0:
                    cumul_pct = (self.drop_reference - raw_value) * 100 // self.drop_reference
                    if cumul_pct >= self.p.L1_drop_pct:
                        surface_level = 1
                    elif (self.consecutive_raw_drops >= self.p.L2_min_consecutive and
                          cumul_pct >= self.p.L2_drop_pct):
                        surface_level = 2
            else:
                self.consecutive_raw_drops = 0
        elif not self.current_state:
            self.consecutive_raw_drops = 0

        # L3: MA3 trend
        self.trend_buffer[self.trend_idx] = filtered
        self.trend_idx = (self.trend_idx + 1) % 3
        if self.trend_count < 3:
            self.trend_count += 1

        current_ma3 = filtered
        if self.trend_count >= 3:
            current_ma3 = sum(self.trend_buffer) // 3

        if (surface_level == 0 and self.current_state and self.prev_ma3 > 0 and
                self.trend_count >= 3 and
                self.time_in_state >= self.p.override_min_time and proximity_ok):
            if current_ma3 < self.prev_ma3:
                if self.ma3_trend_count == 0:
                    self.ma3_trend_start = self.prev_ma3
                self.ma3_trend_count += 1
            else:
                if self.ma3_trend_count > 0:
                    self.ma3_trend_count -= 1

            if (self.ma3_trend_count >= self.p.L3_min_consecutive and
                    self.ma3_trend_start > 0):
                ma3_drop = (self.ma3_trend_start - current_ma3) * 100 // self.ma3_trend_start
                if ma3_drop >= self.p.L3_drop_pct:
                    surface_level = 3
        elif not self.current_state:
            self.ma3_trend_count = 0
            self.ma3_trend_start = 0

        self.prev_ma3 = current_ma3

        # L4 & L5
        if self.current_state:
            if filtered > self.peak_adc:
                self.peak_adc = filtered
            if filtered < self.min_adc:
                self.min_adc = filtered

        if (surface_level == 0 and self.current_state and
                self.time_in_state >= self.p.override_min_time and proximity_ok):
            if self.water > 0:
                water_thresh = int(self.water * (1 - self.p.L4_drop_pct / 100.0))
                if filtered < water_thresh:
                    surface_level = 4
            if (surface_level == 0 and self.peak_adc > 0 and
                    self.time_in_state > self.p.L5_min_time):
                drop = (self.peak_adc - filtered) * 100 // max(1, self.peak_adc)
                if drop >= self.p.L5_drop_pct:
                    surface_level = 5

        # ── State determination ──
        new_state = self.current_state
        threshold_high = self.threshold + self.hysteresis
        threshold_low = self.threshold - self.hysteresis

        if filtered > threshold_high:
            new_state = True
            self.consecutive_samples = 0
            if self.lockout_remaining == 0:
                self._calibrate_water(filtered)
        elif filtered < threshold_low:
            if self.current_state:
                self.consecutive_samples += 1
                if self.consecutive_samples >= self.p.min_dry_samples:
                    new_state = False
            else:
                new_state = False
                self.consecutive_samples = 0
        else:
            self.consecutive_samples = 0

        # L-override
        if surface_level > 0 and self.current_state and new_state:
            new_state = False
            self.consecutive_samples = 0
            self.level_triggers[surface_level] += 1

            # Track false L-overrides (triggered while actually underwater)
            if expected_uw:
                self.false_surface_triggers += 1

            max_air = int(self.water * 0.80)
            if filtered > self.air * 2 and filtered < max_air:
                self.air = filtered
                self._update_threshold()

            if self.p.min_surface_time > 0:
                self.lockout_remaining = self.p.surface_lockout

        # Max dive timeout (no air recalibration - just force surface)
        if (self.current_state and self.p.max_dive_time > 0 and
                self.time_in_state >= self.p.max_dive_time):
            new_state = False
            self.lockout_remaining = self.p.surface_lockout
            self.consecutive_samples = 0

        # Lockout
        if self.lockout_remaining > 0:
            self.lockout_remaining -= 1
            if new_state:
                new_state = False

        # State change
        if new_state != self.current_state:
            self.time_in_state = 0
            self.consecutive_samples = 0
            self.min_adc = 0xFFFF
            self.peak_adc = 0
            self.consecutive_raw_drops = 0
            self.drop_reference = 0
            self.trend_count = 0
            self.trend_idx = 0
            self.ma3_trend_count = 0
            self.ma3_trend_start = 0
            self.prev_ma3 = 0

        # Transition detection latency
        if self._last_expected is not None and expected_uw != self._last_expected:
            self._waiting_for_detection = True
            self._transition_counter = 0
        if self._waiting_for_detection:
            self._transition_counter += 1
            if new_state == expected_uw:
                self.detection_latencies.append(self._transition_counter)
                self._waiting_for_detection = False
        self._last_expected = expected_uw

        # Accuracy
        if new_state == expected_uw:
            self.correct += 1
        elif expected_uw:
            self.false_negatives += 1
        else:
            self.false_positives += 1

        self.current_state = new_state


# ─── Test Scenarios Based on Real Field Data ───

def build_scenarios() -> List[Scenario]:
    scenarios = []

    # ═══════════════════════════════════════════════════════
    # REAL FIELD DATA SCENARIOS (from user's actual tests)
    # ═══════════════════════════════════════════════════════

    # Scenario 1: Real test - Gain 1/6, cable connected
    # Measured: air=50-60, water=3140-3180
    scenarios.append(Scenario(
        name="[REAL] Gain 1/6, clean electrode",
        adc_air=55, adc_water=3160, noise_air=15, noise_water=30,
        transitions=[
            ("air", 30),                       # 30s in air (calibration)
            ("transition_in", 3), ("water", 120),  # Dive 2min
            ("transition_out", 4), ("air", 45),    # Surface 45s
            ("transition_in", 3), ("water", 90),   # Dive 1.5min
            ("transition_out", 4), ("air", 60),    # Surface 1min
            ("transition_in", 3), ("water", 60),   # Dive 1min
            ("transition_out", 4), ("air", 120),   # Surface 2min
        ],
        transition_samples=3,
        weight=3.0,  # High weight - this is the primary use case
    ))

    # Scenario 2: Real test - Boot in water (gain 1/6)
    # User's actual issue: device starts underwater
    scenarios.append(Scenario(
        name="[REAL] Boot in water, gain 1/6",
        adc_air=55, adc_water=3160, noise_air=15, noise_water=30,
        transitions=[
            ("water", 90),                     # Already underwater at boot
            ("transition_out", 4), ("air", 60),
            ("transition_in", 3), ("water", 180),
            ("transition_out", 4), ("air", 90),
        ],
        transition_samples=3,
        weight=2.0,
    ))

    # Scenario 3: Real test - Underwater drift (caused L3 false positive)
    # Measured: water drifts from 3160 to ~3046 over 5min (3.6% drift)
    scenarios.append(Scenario(
        name="[REAL] UW drift (anti false L3)",
        adc_air=55, adc_water=3160, noise_air=15, noise_water=25,
        transitions=[
            ("air", 20),
            ("transition_in", 3), ("water", 300),  # 5min dive with drift
            ("transition_out", 5), ("air", 60),
        ],
        transition_samples=3,
        biofouling_drift_per_sample=0.00012,  # ~3.6% over 300 samples
        weight=3.0,  # Critical: must NOT false trigger
    ))

    # Scenario 3b: REAL TEST 07/03 - Heavy drift + wet electrode
    # Measured: air=0, water starts 3130 drifts to 2830 in 2min (10% drift)
    # Out of water: electrode stays wet, reads ~2650-2700 (not 0!)
    # Must detect surface in <2s despite only 5-7% drop
    scenarios.append(Scenario(
        name="[REAL] Heavy drift + wet electrode",
        raw_sequence=_generate_drift_wet_scenario(),
        weight=4.0,  # Highest weight - this is the actual failing test case
    ))

    # ═══════════════════════════════════════════════════════
    # BIOFOULING SCENARIOS (simulated from field experience)
    # ═══════════════════════════════════════════════════════

    # Scenario 4: Light biofouling (1 week deployed)
    # Air baseline rises due to surface contamination
    scenarios.append(Scenario(
        name="[BIO] Light biofouling (1 week)",
        adc_air=150, adc_water=3000, noise_air=30, noise_water=50,
        transitions=[
            ("air", 25),
            ("transition_in", 4), ("water", 120),
            ("transition_out", 5), ("air", 50),
            ("transition_in", 4), ("water", 90),
            ("transition_out", 5), ("air", 60),
        ],
        transition_samples=4,
        weight=2.0,
    ))

    # Scenario 5: Moderate biofouling (2-3 weeks)
    # Reduced contrast, slower transitions
    scenarios.append(Scenario(
        name="[BIO] Moderate biofouling (2-3 weeks)",
        adc_air=400, adc_water=2600, noise_air=50, noise_water=80,
        transitions=[
            ("air", 30),
            ("transition_in", 6), ("water", 180),
            ("transition_out", 8), ("air", 60),
            ("transition_in", 6), ("water", 120),
            ("transition_out", 8), ("air", 90),
        ],
        transition_samples=6,
        biofouling_drift_per_sample=0.0001,
        weight=2.0,
    ))

    # Scenario 6: Heavy biofouling (1+ month)
    # Low contrast, very slow transitions
    scenarios.append(Scenario(
        name="[BIO] Heavy biofouling (1+ month)",
        adc_air=800, adc_water=2000, noise_air=60, noise_water=100,
        transitions=[
            ("air", 30),
            ("transition_in", 10), ("water", 300),
            ("transition_out", 12), ("air", 90),
            ("transition_in", 10), ("water", 200),
            ("transition_out", 12), ("air", 60),
        ],
        transition_samples=10,
        biofouling_drift_per_sample=0.0002,
        weight=1.5,
    ))

    # Scenario 7: Extreme biofouling (algae/barnacles)
    # Very low contrast, contrast < 2x
    scenarios.append(Scenario(
        name="[BIO] Extreme biofouling (algae)",
        adc_air=1200, adc_water=1800, noise_air=80, noise_water=100,
        transitions=[
            ("air", 30),
            ("transition_in", 12), ("water", 300),
            ("transition_out", 15), ("air", 120),
            ("transition_in", 12), ("water", 200),
            ("transition_out", 15), ("air", 90),
        ],
        transition_samples=12,
        biofouling_drift_per_sample=0.0003,
        weight=1.0,
    ))

    # ═══════════════════════════════════════════════════════
    # SALINITY SCENARIOS
    # ═══════════════════════════════════════════════════════

    # Scenario 8: Low salinity (brackish/estuary)
    scenarios.append(Scenario(
        name="[SAL] Low salinity (brackish)",
        adc_air=55, adc_water=1800, noise_air=15, noise_water=60,
        transitions=[
            ("air", 25),
            ("transition_in", 4), ("water", 120),
            ("transition_out", 5), ("air", 50),
            ("transition_in", 4), ("water", 90),
            ("transition_out", 5), ("air", 60),
        ],
        transition_samples=4,
        weight=1.5,
    ))

    # Scenario 9: Very high salinity (Dead Sea / tropical lagoon)
    scenarios.append(Scenario(
        name="[SAL] High salinity (tropical)",
        adc_air=40, adc_water=4200, noise_air=10, noise_water=50,
        transitions=[
            ("air", 25),
            ("transition_in", 2), ("water", 120),
            ("transition_out", 3), ("air", 50),
            ("transition_in", 2), ("water", 90),
            ("transition_out", 3), ("air", 60),
        ],
        transition_samples=2,
        weight=1.0,
    ))

    # ═══════════════════════════════════════════════════════
    # BEHAVIORAL SCENARIOS (turtle behavior)
    # ═══════════════════════════════════════════════════════

    # Scenario 10: Rapid surfacing (turtle breathing)
    # Short surface time (10-20s), must detect quickly
    scenarios.append(Scenario(
        name="[BEHAV] Rapid surfacing (breathing)",
        adc_air=55, adc_water=3160, noise_air=15, noise_water=30,
        transitions=[
            ("air", 15),
            ("transition_in", 2), ("water", 45),
            ("transition_out", 3), ("air", 12),   # 12s surface
            ("transition_in", 2), ("water", 60),
            ("transition_out", 3), ("air", 15),   # 15s surface
            ("transition_in", 2), ("water", 90),
            ("transition_out", 3), ("air", 10),   # 10s surface
            ("transition_in", 2), ("water", 120),
            ("transition_out", 3), ("air", 20),
        ],
        transition_samples=2,
        weight=2.5,  # High weight - real turtle behavior
    ))

    # Scenario 11: Long dive (deep dive turtle)
    scenarios.append(Scenario(
        name="[BEHAV] Long dive (60min)",
        adc_air=55, adc_water=3160, noise_air=15, noise_water=40,
        transitions=[
            ("air", 30),
            ("transition_in", 3), ("water", 3600),  # 60min dive
            ("transition_out", 5), ("air", 120),
        ],
        transition_samples=3,
        biofouling_drift_per_sample=0.00003,  # Slight drift over 1h
        weight=1.5,
    ))

    # Scenario 12: Splash zone (wave action at surface)
    # Alternating wet/dry from waves - should NOT trigger false underwater
    scenarios.append(Scenario(
        name="[BEHAV] Splash zone (anti false UW)",
        adc_air=55, adc_water=3160, noise_air=15, noise_water=30,
        raw_sequence=_generate_splash_sequence(55, 3160, 15, 30),
        weight=2.0,
    ))

    return scenarios


def _generate_drift_wet_scenario() -> List[Tuple[int, bool]]:
    """Replicate exact test data from 07/03: heavy underwater drift + wet electrode.
    ADC starts at 3130, drifts to ~2830 in 120s underwater.
    Out of water: reads ~2650-2700 (wet electrode), NOT zero.
    """
    rng = random.Random(77)
    seq = []

    # Phase 1: Air (5s) - clean electrode reads 0
    for _ in range(5):
        seq.append((rng.randint(0, 20), False))

    # Phase 2: Transition into water (2s)
    seq.append((1500 + rng.randint(-50, 50), True))
    seq.append((3100 + rng.randint(-30, 30), True))

    # Phase 3: Underwater 120s - drifts from 3130 to 2830 (10% drift)
    for i in range(120):
        base = 3130 - int(300 * i / 120)  # Linear drift 3130 → 2830
        noise = rng.randint(-20, 20)
        seq.append((base + noise, True))

    # Phase 4: Out of water (30s) - wet electrode reads ~2650-2700
    # Slow drop from 2830 to 2650 (first 3s) then stable at 2650
    for i in range(30):
        if i < 3:
            base = 2830 - int(180 * i / 3)  # Drop to 2650 in 3s
        else:
            base = 2650
        noise = rng.randint(-30, 30)
        seq.append((base + noise, False))

    # Phase 5: Back in water (60s) - jumps to ~3120, drifts to 2900
    seq.append((2900 + rng.randint(-30, 30), True))
    seq.append((3100 + rng.randint(-30, 30), True))
    for i in range(58):
        base = 3120 - int(220 * i / 58)  # Drift 3120 → 2900
        noise = rng.randint(-20, 20)
        seq.append((base + noise, True))

    # Phase 6: Out of water again (20s) - wet electrode ~2600-2650
    for i in range(20):
        if i < 2:
            base = 2900 - int(300 * i / 2)
        else:
            base = 2600
        noise = rng.randint(-30, 30)
        seq.append((base + noise, False))

    return seq


def _generate_splash_sequence(air: int, water: int, noise_air: int, noise_water: int
                               ) -> List[Tuple[int, bool]]:
    """Generate splash zone: rapid alternating readings, should stay as surface."""
    rng = random.Random(123)
    seq = []

    # 20s in air (calibration)
    for _ in range(20):
        seq.append((air + rng.randint(-noise_air, noise_air), False))

    # 60s of splash zone: 70% air readings, 30% brief water-like spikes
    for i in range(60):
        if rng.random() < 0.30:
            # Wave splash: high reading for 1 sample
            val = water - rng.randint(0, water // 3)  # Partial immersion
            seq.append((val, False))  # Expected: NOT underwater (just a splash)
        else:
            seq.append((air + rng.randint(-noise_air, noise_air * 3), False))

    # 30s clean air
    for _ in range(30):
        seq.append((air + rng.randint(-noise_air, noise_air), False))

    return seq


# ─── Optimizer ───

@dataclass
class Score:
    accuracy: float = 0.0
    weighted_accuracy: float = 0.0
    avg_latency: float = 0.0
    false_positives: int = 0
    false_negatives: int = 0
    false_l_overrides: int = 0
    total_score: float = 0.0


def evaluate_params(params: Params, scenarios: List[Scenario], verbose: bool = False) -> Score:
    total_correct = 0
    total_samples = 0
    total_fp = 0
    total_fn = 0
    total_false_l = 0
    all_latencies = []
    weighted_acc_sum = 0.0
    weight_sum = 0.0
    level_triggers = {1: 0, 2: 0, 3: 0, 4: 0, 5: 0}

    for sc in scenarios:
        sequence = generate_adc_sequence(sc)
        det = SWSDetectorSim(params)

        for adc, expected_uw in sequence:
            det.process_sample(adc, expected_uw)

        total_correct += det.correct
        total_samples += det.total_samples
        total_fp += det.false_positives
        total_fn += det.false_negatives
        total_false_l += det.false_surface_triggers
        all_latencies.extend(det.detection_latencies)
        for k in level_triggers:
            level_triggers[k] += det.level_triggers[k]

        sc_acc = det.correct / det.total_samples * 100 if det.total_samples > 0 else 0
        weighted_acc_sum += sc_acc * sc.weight
        weight_sum += sc.weight

        if verbose:
            avg_lat = (sum(det.detection_latencies) / len(det.detection_latencies)
                       if det.detection_latencies else 0)
            false_l_str = f" FALSE_L={det.false_surface_triggers}" if det.false_surface_triggers else ""
            print(f"  {sc.name:42s} | acc={sc_acc:5.1f}% | FP={det.false_positives:3d} "
                  f"FN={det.false_negatives:3d} | lat={avg_lat:4.1f}s | "
                  f"L1={det.level_triggers[1]} L2={det.level_triggers[2]} "
                  f"L3={det.level_triggers[3]} L4={det.level_triggers[4]} "
                  f"L5={det.level_triggers[5]}{false_l_str}")

    accuracy = total_correct / total_samples * 100 if total_samples > 0 else 0
    weighted_acc = weighted_acc_sum / weight_sum if weight_sum > 0 else 0
    avg_latency = sum(all_latencies) / len(all_latencies) if all_latencies else 999

    # Composite score:
    # - Weighted accuracy is primary (accounts for scenario importance)
    # - False L-overrides are VERY bad (stuck in wrong state for lockout duration)
    # - Regular FP/FN are bad but recoverable
    # - Low latency is nice but secondary
    score = (weighted_acc * 10
             - avg_latency * 1.5
             - total_false_l * 20    # Heavy penalty for false L-overrides
             - total_fp * 0.3
             - total_fn * 0.1)

    if verbose:
        print(f"\n  TOTAL: accuracy={accuracy:.1f}% (weighted={weighted_acc:.1f}%) | "
              f"latency={avg_latency:.1f}s | FP={total_fp} FN={total_fn} "
              f"FALSE_L={total_false_l}")
        print(f"  Levels: L1={level_triggers[1]} L2={level_triggers[2]} "
              f"L3={level_triggers[3]} L4={level_triggers[4]} L5={level_triggers[5]}")

    return Score(
        accuracy=accuracy,
        weighted_accuracy=weighted_acc,
        avg_latency=avg_latency,
        false_positives=total_fp,
        false_negatives=total_fn,
        false_l_overrides=total_false_l,
        total_score=score,
    )


def optimize():
    scenarios = build_scenarios()

    print("=" * 75)
    print("  SWS ANALOG DETECTOR - PARAMETER OPTIMIZER")
    print("  Based on real field test data + biofouling/salinity simulations")
    print("=" * 75)

    # Evaluate current parameters
    print("\n--- CURRENT Parameters ---")
    current = Params()
    print(f"  threshold_ratio={current.threshold_ratio_pct}% hysteresis={current.hysteresis_pct}% "
          f"alpha={current.alpha_pct}% min_dry={current.min_dry_samples}")
    print(f"  L1={current.L1_drop_pct}% L2={current.L2_drop_pct}% "
          f"L3={current.L3_drop_pct}%/{current.L3_min_consecutive}consec "
          f"L4={current.L4_drop_pct}% proximity={current.proximity_guard_pct}%")
    print()
    current_score = evaluate_params(current, scenarios, verbose=True)
    print(f"\n  SCORE: {current_score.total_score:.1f}")

    # Parameter grid
    param_grid = {
        'threshold_ratio_pct': [25, 30, 35, 40, 45, 50],
        'hysteresis_pct':      [6, 8, 10, 12, 14, 18],
        'alpha_pct':           [10, 15, 19, 25, 30],
        'min_dry_samples':     [1, 2, 3],
        'L1_drop_pct':         [8, 10, 12, 15, 20, 25],
        'L2_drop_pct':         [3, 5, 8, 10],
        'L3_drop_pct':         [2, 3, 5, 8],
        'L3_min_consecutive':  [2, 3, 4, 5],
        'L4_drop_pct':         [10, 12, 15, 20],
        'L5_drop_pct':         [10, 15, 20],
        'proximity_guard_pct': [80, 85, 90, 95],
        'surface_lockout':     [10, 15, 20, 30, 45],
        'override_min_time':   [1, 2, 3, 5],
    }

    # Phase 1: Greedy optimization
    print("\n\n--- Phase 1: Greedy Single-Parameter Sweep ---")
    best_params = Params()
    best_score = current_score
    improved = True

    iteration = 0
    while improved and iteration < 3:
        improved = False
        iteration += 1
        print(f"\n  Iteration {iteration}:")

        for param_name, values in param_grid.items():
            best_val = getattr(best_params, param_name)
            best_param_score = best_score

            for val in values:
                test_params = Params(**{k: getattr(best_params, k) for k in vars(best_params)})
                setattr(test_params, param_name, val)
                score = evaluate_params(test_params, scenarios)

                if score.total_score > best_param_score.total_score:
                    best_param_score = score
                    best_val = val

            if best_val != getattr(best_params, param_name):
                old_val = getattr(best_params, param_name)
                print(f"    {param_name}: {old_val} -> {best_val} "
                      f"(+{best_param_score.total_score - best_score.total_score:.1f})")
                setattr(best_params, param_name, best_val)
                best_score = best_param_score
                improved = True
            # else: unchanged, don't print (reduce noise)

    # Phase 2: Cross-parameter optimization
    print("\n--- Phase 2: Cross-Parameter Pairs ---")
    critical_pairs = [
        ('threshold_ratio_pct', 'hysteresis_pct'),
        ('L1_drop_pct', 'L2_drop_pct'),
        ('L3_drop_pct', 'L3_min_consecutive'),
        ('proximity_guard_pct', 'L4_drop_pct'),
        ('min_dry_samples', 'surface_lockout'),
        ('alpha_pct', 'threshold_ratio_pct'),
        ('override_min_time', 'proximity_guard_pct'),
    ]

    for p1_name, p2_name in critical_pairs:
        v1_list = param_grid[p1_name]
        v2_list = param_grid[p2_name]
        best_pair_score = best_score
        best_v1 = getattr(best_params, p1_name)
        best_v2 = getattr(best_params, p2_name)

        for v1, v2 in itertools.product(v1_list, v2_list):
            test_params = Params(**{k: getattr(best_params, k) for k in vars(best_params)})
            setattr(test_params, p1_name, v1)
            setattr(test_params, p2_name, v2)
            score = evaluate_params(test_params, scenarios)

            if score.total_score > best_pair_score.total_score:
                best_pair_score = score
                best_v1 = v1
                best_v2 = v2

        if best_pair_score.total_score > best_score.total_score:
            setattr(best_params, p1_name, best_v1)
            setattr(best_params, p2_name, best_v2)
            print(f"  {p1_name}={best_v1}, {p2_name}={best_v2} "
                  f"(+{best_pair_score.total_score - best_score.total_score:.1f})")
            best_score = best_pair_score
        else:
            print(f"  {p1_name}/{p2_name}: no improvement")

    # Final evaluation
    print("\n\n" + "=" * 75)
    print("  OPTIMIZED PARAMETERS")
    print("=" * 75)

    fields = [
        ('threshold_ratio_pct', 'Threshold ratio', '%'),
        ('hysteresis_pct', 'Hysteresis', '%'),
        ('alpha_pct', 'EMA alpha (water)', '%'),
        ('min_dry_samples', 'Min dry samples', ''),
        ('L1_drop_pct', 'L1 single drop', '%'),
        ('L2_drop_pct', 'L2 cumul drop', '%'),
        ('L3_drop_pct', 'L3 MA3 drop', '%'),
        ('L3_min_consecutive', 'L3 min consecutive', ''),
        ('L4_drop_pct', 'L4 relative drop', '%'),
        ('L5_drop_pct', 'L5 cumul peak drop', '%'),
        ('proximity_guard_pct', 'Proximity guard', '%'),
        ('surface_lockout', 'Surface lockout', 's'),
        ('override_min_time', 'Override min time', 's'),
    ]

    cur = Params()
    for attr, label, unit in fields:
        old = getattr(cur, attr)
        new = getattr(best_params, attr)
        changed = " <-- CHANGED" if old != new else ""
        print(f"  {label:25s} = {new:4d}{unit}  (was {old}{unit}){changed}")

    print(f"\n--- Optimized Results ---")
    opt_score = evaluate_params(best_params, scenarios, verbose=True)
    delta = opt_score.total_score - current_score.total_score
    print(f"\n  SCORE: {opt_score.total_score:.1f} (was {current_score.total_score:.1f}, "
          f"delta={delta:+.1f})")

    # Generate C defines
    print("\n\n--- Copy-Paste C Defines for sws_analog_service.cpp ---\n")
    print(f"#define DEFAULT_THRESHOLD_RATIO_PERCENT {best_params.threshold_ratio_pct}")
    print(f"#define DEFAULT_HYSTERESIS_PERCENT {best_params.hysteresis_pct}")
    print(f"#define DEFAULT_ALPHA_PERCENT {best_params.alpha_pct}")
    print(f"#define DEFAULT_MIN_DRY_SAMPLES {best_params.min_dry_samples}")
    print(f"#define L1_DROP_PERCENT {best_params.L1_drop_pct}")
    print(f"#define L2_DROP_PERCENT {best_params.L2_drop_pct}")
    print(f"#define L2_MIN_CONSECUTIVE {best_params.L2_min_consecutive}")
    print(f"#define L3_MIN_CONSECUTIVE {best_params.L3_min_consecutive}")
    print(f"#define L3_DROP_PERCENT {best_params.L3_drop_pct}")
    print(f"#define L4_DROP_PERCENT {best_params.L4_drop_pct}")
    print(f"#define L5_DROP_PERCENT {best_params.L5_drop_pct}")
    print(f"#define L5_MIN_TIME_SEC {best_params.L5_min_time}")
    print(f"#define OVERRIDE_MIN_TIME_SEC {best_params.override_min_time}")
    print(f"#define SURFACE_LOCKOUT_DURATION_SEC {best_params.surface_lockout}")
    print(f"#define PROXIMITY_GUARD_PERCENT {best_params.proximity_guard_pct}")

    return best_params


if __name__ == "__main__":
    optimize()
