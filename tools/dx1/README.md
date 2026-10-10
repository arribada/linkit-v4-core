# DX1 v3 — LinkIt turtle Doppler extension (SWS health + turtle behaviour)

Source of truth for the bytes: `dx1_ref.py` (this folder) and the vectors it emits, `dx1_vectors_v3.json`
(`python3 dx1_ref.py dx1_vectors_v3.json` regenerates them; `dx1_expected.py --ts OUT.ts` emits the platform
decoder fixture). Firmware (`tests/src/dx1_doppler_test.cpp`), platform decoder and wiki must all reproduce
those vectors byte for byte. Firmware: `core/services/dive_behaviour_tracker.hpp`, `argos_packet_builder.*`,
`argos_tx_service.cpp`.

## 0. Design constraints
- No adaptive modulation in the design: the Doppler goes on whatever modulation the module actually frames with
  (master RCONF when adaptive is off). All four turtle templates ship `ARGOS_ADAPTIVE_MODULATION = 0`
  (decision 2026-10), so the Doppler view follows the master: LDA2 master → 4 B compact view (DRY/DEND + CRC),
  LDK master → 16 B view with the rich block. The 3 B VLDA4 view only arises from a VLDA4 master.
- Reuse the existing Doppler message (byte0 = 0x00). No new message type.
- The payload must be fully built BEFORE surfacing. The surfacing path may only do RAM reads + integer ops
  (< ~100 µs): no ADC, no flash, no DEBUG line, no allocation.
- Release gate: new boolean param `ARGOS_DOPPLER_EXT_EN`, DTE key `ARP47`, default **false**. When false
  (or UNDERWATER_EN false, or the build lacks the SWS guard) the Doppler is the legacy 3-byte frame,
  bit-exact (006400 / 24 bits), on every modulation, exactly as today.

## 1. Frame layout (MSB-first)
```
byte0      0x00                                  frozen marker
byte1      BATT7 | LB                            frozen (packet_field_encoding.hpp)
byte2      VER(2)=01 | EXT(1)=0 | DRY(3) | DEND(2)
byte3      CRC8::checksum(packet, 24)            (views >= 32 bits; crc8.hpp; NOT apply_lda2_crc8)
bits 32-111  rich block (LDK view only)
bits 112-127 0x0000 reserved (Phase 2: accel activity / temperature)
```
Rich block (absolute bit offsets):
| bits | field | encoding |
|---|---|---|
| 32-33 | LATCH | 0 = rich absent (padded LDA2 view); 1 = behaviour latched at a dive edge; 2 = no dive edge since tracker reset (behaviour fields all 0); 3 = reserved |
| 34 | BLIND | 1 if a blind interval (cooldown, SWS test mode) overlaps the window (completed blocks + current) or is in progress |
| 35 | PARTIAL | 1 if fewer than 4 complete 6-h blocks since tracker reset |
| 36-44 | SURF_SEQ | 9 b; 0 = none; counted real surfacings since tracker reset, 1..511, wraps 511→1 |
| 45-47 | RST | 3 b; warm boots since last power-on that reached Operational (noinit+CRC), saturating 7 |
| 48-54 | PCT_DRY | 0 = unknown (Σobs < 43200 s); else 1 + floor((200·dry + obs)/(2·obs)), 1..101 |
| 55-61 | N_SURF_24H | E4M3(n+1); 0 = unknown (PARTIAL) |
| 62-68 | S_LAST | E4M3 seconds of the surface episode closed by the latching dive edge; 0 = unknown |
| 69-75 | D_PREV | E4M3 seconds of the dive closed by that surface episode; 0 = unknown |
| 76-82 | D_MAX_24H | E4M3 seconds; 0 = unknown (PARTIAL) or no known dive |
| 83-88 | SAMPLE_DELAY | E3M3(max(1,(delay_us+25)/50)); 0 = sample_delay_us == 0 |
| 89-94 | CONTRAST | E3M3(max(1,contrast_x10)); 0 = !is_calibrated or contrast_x10==0 or threshold_air==0 |
| 95-98 | DTO | Q4(dive_timeout_count) |
| 99-102 | FSC | Q4(force_surface_count) |
| 103-106 | SRC | Q4(stuck_recovery_count) |
| 107-109 | SURF_METHOD | of the surface episode closed at the latch: 1..5 = L1..L5; 6 = other / forced / ADC fallback; 0 = unknown |
| 110 | DFLOOR | sample_delay_us > 0 && == delay_min && delay_min < delay_max |
| 111 | ADCF | adc fault active (m_consecutive_invalid_adc >= MAX_CONSECUTIVE_INVALID_ADC) |

Codecs (all integer, floor), exactly as `dx1_ref.py`: `em(v,E,M)`, `q4`, `pct_code`, `dry_class`
(classes 1..7 at per-mille bounds 20/40/80/160/320/640, 0 = unknown, **no hysteresis**), `batt`.

Views (one 16-byte image, per-modulation prefix): VLDA4 3 B/24 bits, LDA2 4 B/32, LDK 16 B/128.
The view is chosen from the modulation the driver will ACTUALLY frame with = `m_kineis.get_current_modulation()`
read right before `m_kineis.send()` (after any ensure_modulation). `doppler_view(img, mode, in_bits)` returns
`in_bits` unchanged for a non-DX1 packet; DX1-ness is an explicit flag stored with the prepared packet and passed
as the 4th argument `is_dx1` (never inferred from length alone).

## 2. Field timing (per-ping contract)
- Battery and health fields (SAMPLE_DELAY, CONTRAST, DTO, FSC, SRC, DFLOOR, ADCF, RST): evaluated when the image
  is built (dive / hourly underwater refresh for ping 1, TX time for pings 2+ and heartbeats).
- Behaviour fields (LATCH, BLIND, PARTIAL, SURF_SEQ, PCT, N, S_LAST, D_PREV, D_MAX, METHOD) and DRY: latched at the
  last dive edge; identical in every frame until the next dive edge. **There is no re-latch** (dropped vs v2).
- DEND is a health field computed from the tracker at build time (see §3) AND refreshed on the prewarm send:
  the prewarm path patches the 2 DEND bits of byte 2 and recomputes byte 3 (if the view is ≥ 4 B) from tracker
  RAM. That is the only surfacing-path work DX1 adds. So ping 1 already says how the dive that just ended ended.
- Ping 1 = prewarm image (≤ 1 h old) with DEND refreshed; if the prewarm is rejected (stale/empty), ping 1 is built
  on the standard path at TX time like pings 2+.

## 3. DiveBehaviourTracker semantics (reference: class `Tracker` in dx1_ref.py — mirror it exactly)
Clock: `PMU::get_timestamp_ms()` (monotonic, restarts at boot). Times relative to the tracker reset time.
State ∈ {UNKNOWN, UW, SURF}, plus `blind`.
- Input: UW_SENSOR SERVICE_LOG_UPDATED(bool) in `ServiceManager::notify_peer_event` (service.cpp ~147), placed before
  the RTC-gated hauled call. Counters/flags captured from SWSAnalogService static RAM (Diagnostics dto/fsc, adc fault,
  Status.surface_level) at the event.
- If `SWSAnalogService::is_test_running()`: `on_test_event(now)` (close open episode, state UNKNOWN, window BLIND), return.
- If blind: ignore.
- UNKNOWN → set state without an edge; if UW take the dive snapshot (dive_start=now, start NOT real, dto0/fsc0);
  if SURF the surface is "not known". `acct_from = now`.
- Same state as tracked: re-emission, ignore.
- UW→SURF (surface edge): STORE ONLY t_s, surface_level, dto_s, fsc_s, adcf_s, surface_known = true. O(1).
- SURF→UW (dive edge), all the work:
  - forced = (has_snapshot && fsc_s > fsc0) || adcf_s; surface_real = surface_known && !forced.
  - Account [acct_from, t_s) wet; [t_s, now) dry if surface_real, else unobserved. Split across 6-h blocks;
    each piece credited as `(piece_end_ms - piece_start_ms) / 1000` whole seconds to its block (obs, and dry).
  - Closed dive known iff surface_known && has_snapshot && dive_start_real && !dive_blinded. If known,
    block(t_s).dmax = max(dmax, max(1, dive_s)).
  - If surface_real: seq = seq % 511 + 1; block(now).n++; s_last = max(1,(now - t_s)/1000).
  - METHOD = 0 if !surface_known; 6 if forced; level if 1..5 else 6.
  - Window = completed blocks cur-4..cur-1 (cur = block(now)); PARTIAL = cur < 4; PCT/DRY from Σdry/Σobs over them;
    N = Σn (0 if PARTIAL); D_MAX = max dmax (0 if PARTIAL or none); BLIND = blind || any blind flag in cur-4..cur.
  - Latch these with LATCH = 1. Then new dive: snapshot dto0/fsc0, dive_start = now, dive_start_real = surface_real.
- on_blind_start(now) (service_cooldown.cpp, RTC-set path, interval > 0, where the SWS gate starts refusing):
  close the open episode up to now (wet if UW; dry if SURF and not forced), blind = true, dive_blinded = true,
  flag block. on_blind_end(now) (exit_cooldown_sleep): flag blocks [blind_start, now], blind = false,
  state UNKNOWN, acct_from = now; no-op if not blind.
- Ring: 5 slots {id, dry_s, obs_s, n, dmax_s, blind}, keyed by block id; older ids evicted.
- DEND(now counters): 0 if UNKNOWN or no snapshot; UW: deltas vs snapshot using current counters/flag;
  SURF: 0 if !surface_known, else deltas of the surface-edge values. Negative delta → 0. fsc delta > 0 or adcf → 3;
  dto delta > 0 → 2; else 1.
- latch() before any dive edge: LATCH = 2, PARTIAL from block(now), BLIND from ring, SURF_SEQ, everything else 0.
- reset(): at boot (static init) and in ServiceManager::startall (next to HauledModeService::restore_state()).
  The first reset() after a boot also does the RST bookkeeping: noinit {magic, count, crc}; valid → count =
  min(count+1, 7); invalid → count = 0. Storage with `section(".noinit")` must be DEFINED in a .cpp that every
  target already compiles (e.g. service.cpp) — not as an inline variable in the header.
- Guard: everything under `#if ENABLE_SWS_ANALOG && !defined(BOARD_RSPB) && !defined(EXTERNAL_WAKEUP)`
  (verify the RSPB host simulation target either compiles it out or keeps legacy output; it must stay legacy).

## 4. ArgosTx integration
- `doppler_ext()` returns nullptr unless guard && UNDERWATER_EN && ARGOS_DOPPLER_EXT_EN.
- prepare_doppler_packet (dive + hourly refresh): build the DX1 image (16 B) when the ext is on; store image + flag.
- Underwater refresh block (argos_tx_service.cpp ~1224): never runs on UW_SENSOR events EXCEPT
  SERVICE_INACTIVE when `SWSAnalogService::get_status().is_underwater` is true (status is pushed before
  service_complete). In particular never on UW_SENSOR SERVICE_ACTIVE, nor on SERVICE_LOG_UPDATED(false).
  Non-UW events keep today's behaviour. (This also removes today's battery-change rebuild on the surface event.)
- Prewarm send (~2379-2404): move the prepared image, refresh DEND + CRC (RAM only), apply the view for the real
  modulation, send. No new log line (extend the existing DEBUG_INFO args only if needed).
- Standard path (~2702-2748): build with ext, view from the real modulation.
- `prepared_doppler_inputs_changed()` stays battery-only.
- RSPB branch untouched. dte_handler SATDP / build_doppler_packet default argument keeps legacy.

## 5. Param
`ARGOS_DOPPLER_EXT_EN`, "ARP47", BOOLEAN, default false, appended at the END of all three tables
(base_types.hpp enum before __PARAM_SIZE → 267; dte_params.cpp last entry; config_store.hpp default_params last
entry). NO config version bump (read the comment above the constant). Run `python3 scripts/check_param_tables.py`.
Update the DTE golden string in tests/src/dte_handler_test.cpp if PARMR output changes. Check
`tools/check_template_conf.py` / pylinkit map: add ARP47 to the map if the tool requires every firmware param;
do NOT enable it in any template.

## 6. Platform decoder contract (arribada-web-platform packages/core/src/argosDecoder.ts)
- New terminal branch for byte0 == 0x00 at bitLength ∈ {24, 32, 128}, placed before the zero-tail heuristic and the
  GNSS_SHORT branch; only for LinkIt devices when a deviceType is supplied (undefined → apply).
- 24 bits (VLDA4): VER0 → legacy battery (byte2 must be 0 else corrupt); VER1 → battery + DRY/DEND (EXT ignored);
  VER2-3 → battery only.
- 32 bits (LDA2): VER0 → legacy iff byte2==0 && byte3==0, else corrupt; VER≥1 → byte3 must equal argosCrc8(bytes,0,3),
  pass → fields, fail → corrupt.
- 128 bits: modulation 'LDA2' → bytes 0-3 only (compact). Else (LDK/unknown): VER0 → legacy iff bytes 2-15 all zero else
  corrupt; VER≥1 → CRC check; LATCH≠0 → rich; LATCH==0 → compact only.
- corrupt = DOPPLER message with crcValid=false and NO battery / payload keys (absent, not null); extractSensorData → [].
- Hex length ≠ bitLength hint: a VER0 frame whose bytes 2.. are all zero still decodes as legacy; anything else is corrupt.
- Platform standalone-SMD layouts also emit byte0 = 0x00 (DOPPLER counter 0, UW_DOPPLER SWS state 0): the caller MUST pass
  deviceType so the branch only claims LinkIt frames.
- Safety net: GNSS_SHORT must never accept day 0.
- Never fall through to another family for byte0 == 0x00 at those lengths.
- Expose dx1Version, crcValid, dryClass, diveEnd, rich{...} with unknown → null and E4M3/E3M3/Q4 decoded as lower bounds.
- Regression: legacy 006400/24, 00640000/32, 006400+13×00/128 decode exactly as today; other families unchanged.

## 7. Known residuals (documented, not fixed)
- S_LAST/PCT read ~1-2 surface sampling periods high; D_PREV/D_MAX saturate near the SWS escalation (~6 h).
- Behaviour fields are stale during a surface lock (no dive edge); SURF_SEQ frozen across frames reveals it.
- RST does not count reboots that never reach Operational.
- Turtle templates use adaptive=0 (2026-10): the view is set by the master RCONF. An LDA2 master gives the 4 B
  compact view only; the rich block needs an LDK master. That choice is a provisioning decision per unit, and it
  also decides whether the depth pile and CloudLocate MEAS20 work (both need LDA2, see template_conf/README.md).
