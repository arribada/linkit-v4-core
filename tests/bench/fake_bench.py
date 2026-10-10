#!/usr/bin/env python3
"""fake_bench.py — OFFLINE check of the bench harness against a simulated board.

No serial port, no nrfjprog, no board: a small single-threaded simulator answers
the % console and the DTE frames, and emits the firmware log lines the cases
wait for, on a virtual clock (a 6-minute case runs in seconds). It exists to
check SEQUENCING and PARSING of the cases (does each one reach a verdict, does
it parse what the firmware prints, does it restore what it changed, does the RF
guard fire and skip the satellite part) -- NOT the firmware: the simulator's
behaviour is a model written from the sources, and a PASS here proves nothing
about the board.

    python tests/bench/fake_bench.py                 edge-2026-10 + kim_bench --run, mute KIM2
    python tests/bench/fake_bench.py --radio vivant  same with a KIM2 that answers (RF guard)
    python tests/bench/fake_bench.py --ciel          also the sky cases (simulated fixes)
    python tests/bench/fake_bench.py --radio vivant --maitresse --cases EDG-D4,EDG-D5
                                                     live KIM2 re-reading a provisioned LDK master at boot

Outputs (JSONL, logs) go to a temp directory, never into tests/bench/logs.
"""
import argparse
import heapq
import itertools
import os
import re
import sys
import tempfile
import time as _real_time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)


# --------------------------------------------------------------- virtual clock
class FakeClock:
    def __init__(self):
        self.t = _real_time.time()
        self.q = []
        self.seq = itertools.count()

    def time(self):
        return self.t

    def monotonic(self):
        return self.t

    def sleep(self, dt):
        self.advance(max(0.0, float(dt)))

    def at(self, delay, fn):
        heapq.heappush(self.q, (self.t + delay, next(self.seq), fn))

    def advance(self, dt):
        end = self.t + dt
        while self.q and self.q[0][0] <= end:
            when, _, fn = heapq.heappop(self.q)
            self.t = max(self.t, when)
            fn()
        self.t = end

    def cancel_all(self, tag):
        self.q = [e for e in self.q if getattr(e[2], 'tag', None) != tag]
        heapq.heapify(self.q)

    # passthrough
    strftime = staticmethod(_real_time.strftime)
    localtime = staticmethod(_real_time.localtime)
    gmtime = staticmethod(_real_time.gmtime)


CLOCK = FakeClock()
T0 = CLOCK.t


def tagged(tag, fn):
    def wrapper():
        fn()
    wrapper.tag = tag
    return wrapper


# --------------------------------------------------------------- board model
class Board:
    """Minimal model of the KIM bench firmware, written from the sources."""

    MOD_NAME = {0: 'LDK', 1: 'LDA2', 2: 'VLDA4'}          # KineisModulation
    NOT_IMPLEMENTED = {'STP04', 'PHP04', 'LTP04', 'PRP04', 'THP06', 'PWP05', 'LBP14'}

    # A 32-hex master RCONF for --maitresse (content irrelevant: the model only
    # needs a non-empty IDP14 whose read-back decodes to LDK).
    MAITRESSE_LDK = '0123456789ABCDEF0123456789ABCDEF'

    def __init__(self, radio='muet', ciel=False, maitresse=False):
        import kim_bench
        import dte_campaign as C
        self.name2key, self.key2name = kim_bench.load_param_map()
        self.readonly = C._cles_non_inscriptibles() | {'ART12', 'ART13', 'ART14', 'ART15'}
        self.radio, self.ciel = radio, ciel
        self.p = {k: '0' for k in self.key2name}
        self.p.update({'ARP01': '2', 'UNP26': '1', 'GNP40': '7', 'GNP05': '120', 'GNP01': '1',
                       'IDP14': '', 'SMP01': '0', 'ART12': '0', 'ART13': '3', 'ART14': '0',
                       'ART15': '0', 'GNP52': '4294967295', 'GNP43': '10', 'GNP42': '10',
                       'IDP12': '294910', 'IDT06': '47FFE', 'IDT02': 'LINKIT', 'IDT03': 'v4.2.0',
                       'ART02': '3', 'LBP02': '10', 'LBP12': '0', 'UNP20': '0', 'UNP30': '3',
                       'ARP43': '3', 'GNP46': '0', 'GNP45': '0', 'UNP04': '10.000000'})
        if maitresse:
            self.p['IDP14'] = self.MAITRESSE_LDK
        self.bench = None
        self.boot()

    # ---- plumbing
    def boot(self):
        self.state = 'OPERATIONAL'
        self.t_boot = CLOCK.time()
        self.underwater = None
        self.rtc = None
        self.bbr = False
        self.gnss_active = False
        self.test_mode = False
        self.errors = 0
        self.hold_until = 0
        self.suspended = False
        self.prepared = None
        self.burst = False
        self.burst_count = 0
        self.cool_armed = False
        self.cool_until = 0
        self.last_fix = None
        self.pmreq = dict(seq=0, giveup=0)

    def emit(self, line, delay=0.0):
        if delay:
            CLOCK.at(delay, lambda: self.emit(line))
            return
        if self.bench is not None:
            self.bench._emit(line)

    def log(self, lvl, msg, delay=0.0):
        stamp = _real_time.strftime('%d/%m/%Y %H:%M:%S', _real_time.gmtime(CLOCK.time()))
        self.emit(f'{stamp} [{lvl}]\t{msg}', delay)

    def uptime_ms(self):
        return int((CLOCK.time() - self.t_boot) * 1000)

    def i(self, k):
        try:
            return int(float(self.p.get(k, '0') or 0))
        except ValueError:
            return 0

    # ---- console
    def handle(self, s):
        for raw in re.split(r'[\r\n]+', s):
            if not raw:
                continue
            if raw.startswith('%'):
                self.console(raw.strip())
            elif raw.startswith('$'):
                self.dte(raw)

    def reply(self, text):
        self.emit(text, 0.05)

    def console(self, line):
        cmd = line.split(' ')[0]
        if cmd == '%PING':
            self.reply(f'%BENCH OK state={self.state}')
        elif cmd == '%STATE':
            self.reply(f'%STATE {self.state}')
        elif cmd == '%CFG':
            already = self.state == 'CONFIG'
            self.reply('%CFG OK already-config' if already else '%CFG OK entering-config')
            if not already:
                self.state = 'CONFIG'
                self.stop_services()
                self.log('INFO', 'entry: ConfigurationState', 0.3)
        elif cmd == '%OP':
            if self.state != 'CONFIG':
                self.reply('%OP OK not-in-config')
            else:
                self.reply('%OP OK leaving-config')
                self.state = 'OPERATIONAL'
                self.log('INFO', 'entry: OperationalState', 0.3)
                self.start_services()
        elif cmd == '%AMODE':
            m = re.match(r'%AMODE (\d+)', line)
            if m:
                self.p['ARP01'] = m.group(1)
            self.reply(f'%AMODE mode={self.p["ARP01"]} burst_max=30 s')
        elif cmd in ('%DIVE', '%SURFACE'):
            wet = cmd == '%DIVE'
            self.reply(f'{"%DIVE OK underwater" if wet else "%SURFACE OK surfaced"} uptime={self.uptime_ms()}')
            if self.state == 'OPERATIONAL':
                CLOCK.at(0.2, lambda: self.uw_event(wet, source='bench'))
        elif cmd == '%ARGOSCFG':
            axl = (self.i('GNP01') and self.i('AXP01') and self.i('AXP05')) << 12
            self.reply(f'%ARGOSCFG mode={self.p["ARP01"]} sensor_tx=0x{axl:08X} depth=1 ntry=0 tr_nom=60 '
                       f'duty=0xFFFFFF lb=0 prepass=0')
        elif cmd == '%LB':
            lb, crit = self.i('LBP02'), self.i('LBP12')
            ok = 1 if (crit == 0 or crit < lb) else 0
            self.reply(f'%LB coherent={ok} lb={lb} crit={crit}')
        elif cmd == '%I2C':
            self.reply('%I2C bus=ONBOARD found=2 addrs=14 70 ')
        elif cmd == '%PMREQ':
            self.reply(f'%PMREQ flags=6 settle=0 seq={self.pmreq["seq"]} probes={self.pmreq["seq"]} '
                       f'first_ok={self.pmreq["seq"]} giveup={self.pmreq["giveup"]}')
        elif cmd == '%BOOT':
            self.reply(f'%BOOT cause=SOFTRESET crash=none uptime_ms={self.uptime_ms()}')
        elif cmd == '%GPS':
            if self.state != 'OPERATIONAL':
                self.reply('%GPS ERR not-operational (use %OP first)')
                return
            f = line.split()
            self.reply(f'%GPS OK injected lat={f[1]} lon={f[2]}')
            self.log('WARN', f'GPSService::bench_inject_fix: lat={float(f[1]):f} lon={float(f[2]):f} '
                     'hAcc=5000mm numSV=9', 0.1)
            self.log('INFO', 'ArgosTxService: first valid GPS fix this session — TX unlocked', 0.3)
        elif cmd == '%GPSCL':
            if self.state != 'OPERATIONAL':
                self.reply('%GPSCL ERR not-operational (use %OP first)')
                return
            self.reply('%GPSCL OK cloudlocate injected')
            self.log('WARN', 'GPSService::bench_inject_cloudlocate: MEASC12 raw measurement injected', 0.1)
            if self.i('GNP46') == 0:
                self.log('INFO', 'GPSService::task_process_cloudlocate_data: stored MEASC12 (12 bytes)', 0.3)
            else:
                f = self.i('GNP46')
                self.log('WARN', f'GPSService::task_process_cloudlocate_data: configured format {f} UNAVAILABLE '
                         '(have measc12=1 meas20=0) — no CloudLocate TX', 0.3)
                self.log('INFO', f'[VAL-GNSS] cloudlocate_format_unavailable want={f} measc12=1 meas20=0', 0.35)
        else:
            self.reply(f'%ERR unknown-cmd {cmd}')

    # ---- DTE
    def dte(self, raw):
        m = re.match(r'\$([A-Z]+)#([0-9A-Fa-f]{3});(.*)$', raw)
        if not m:
            return
        cmd, lg, payload = m.group(1), m.group(2), m.group(3)
        if self.state != 'CONFIG':
            return
        if int(lg, 16) != len(payload):
            self.reply(f'$N;{cmd}#001;4')
            return
        fn = getattr(self, f'dte_{cmd}', None)
        if fn is None:
            self.reply(f'$N;{cmd}#001;6')
            return
        fn(payload)

    def ok(self, cmd, body=''):
        self.reply(f'$O;{cmd}#{len(body):03X};{body}')

    def check_value(self, k, v):
        """Firmware decode + validate, modelled on dte_protocol.hpp."""
        name = self.key2name.get(k, '')
        if k == 'AXP05':
            return v if v in ('0', '1', '2', '3') else None
        if k in ('IDP14', 'IDP11', 'ARP51', 'ARP52', 'ARP53'):
            return v
        if k == 'UNP04':
            try:
                return f'{float(v):f}'
            except ValueError:
                return None
        m = re.match(r'\s*([+-]?\d+)', v)            # sscanf "%u": a prefix is enough
        if not m:
            return None
        n = int(m.group(1)) % (1 << 32)
        bounds = {'UNP26': (0, 2), 'GNP05': (10, 600), 'GNP09': (10, 600), 'ARP01': (0, 5),
                  'LBP02': (0, 100), 'LBP12': (0, 100), 'GNP45': (0, 2), 'GNP46': (0, 2),
                  'GNP43': (0, 90), 'GNP42': (0, 50), 'UNP30': (0, 3), 'GNP40': (1, 63)}
        if k in bounds and not bounds[k][0] <= n <= bounds[k][1]:
            return None
        if name.endswith('_EN') or k in ('ARP54', 'AXP01', 'GNP30', 'GNP51', 'GNP53', 'CTP01', 'UNP01'):
            return str(1 if n else 0)
        return str(n)

    def dte_PARMW(self, payload):
        rejected = []
        for tok in payload.split(','):
            if '=' not in tok:
                rejected.append(tok)
                continue
            k, v = tok.split('=', 1)
            if k not in self.p or k in self.NOT_IMPLEMENTED:
                rejected.append(k)
                continue
            if k in self.readonly:
                rejected.append(k)
                continue
            val = self.check_value(k, v)
            if val is None:
                rejected.append(k)
                continue
            if k in ('IDP14', 'ARP54') and self.p.get(k) != val:
                self.p['ART13'] = '3'
                self.p['ART14'] = '0'
            self.p[k] = val
        self.refresh_art12()
        if rejected:
            body = ','.join(rejected)
            self.reply(f'$N;PARMW#{len(body):03X};{body}')
        else:
            self.ok('PARMW')

    def dte_PARMR(self, payload):
        keys = [k for k in payload.split(',') if k] or \
            [k for k in self.p if len(k) == 5 and k[2] == 'P' and k not in self.NOT_IMPLEMENTED]
        if any(k not in self.p for k in keys):
            self.reply('$N;PARMR#001;6')
            return
        self.ok('PARMR', ','.join(f'{k}={self.p[k]}' for k in keys))

    def dte_PARML(self, payload):
        # KIM build: no SMD-only (ARP60 SMD_LPM_MODE) nor LoRa-only (LRPxx) keys.
        keys = [k for k in self.p if len(k) == 5 and k[2] == 'P' and k not in self.NOT_IMPLEMENTED
                and k != 'ARP60' and not k.startswith('LR')]
        self.ok('PARML', ','.join(keys))

    def dte_STATR(self, payload):
        self.refresh_art12()
        if self.rtc:
            self.p['SYT01'] = str(int(self.rtc + CLOCK.time()))
        keys = [k for k in payload.split(',') if k] or [k for k in self.p if len(k) == 5 and k[2] == 'T']
        if any(k not in self.p for k in keys):
            self.reply('$N;STATR#001;6')
            return
        self.ok('STATR', ','.join(f'{k}={self.p[k]}' for k in keys))

    def refresh_art12(self):
        import dte_campaign as C
        a13 = self.i('ART13')
        self.p['ART12'] = str(C._art12_modele(self.i('GNP01'), self.i('GNP45'), self.i('GNP46'),
                                              self.i('ARP54'), self.p['IDP14'] == '', a13))

    def dte_SMDDFU(self, payload):
        # The DTE answers only in CONFIG, where the KIM2 is ALWAYS powered off:
        # KIM2Device::get_firmware_version() returns "" whatever the module's
        # health (kim2.cpp:1209-1213), so live and mute read the same here.
        self.ok('SMDDFU', '0,1,0,0,Failed to read version')

    def dte_GNSSI(self, payload):
        self.ok('GNSSI', 'ROM SPG 5.10,00190000,M10')

    def dte_SATVF(self, payload):
        self.reply('$N;SATVF#001;3')

    def dte_SWSTST(self, payload):
        on = payload.strip() == '1'
        if on and not self.test_mode:
            self.test_mode = True
            self.log('INFO', 'SWSAnalog: Test mode started', 0.1)
            self.sws_push()
        elif not on and self.test_mode:
            self.test_mode = False
            self.log('INFO', 'SWSAnalog: Test mode stopped', 0.1)
        self.ok('SWSTST', '1' if self.test_mode else '0')

    def sws_push(self):
        if not self.test_mode:
            return
        self.reply('$O;SWSST#030;50,150,90,5,48,49,1,0,12,0,30,52,500')
        CLOCK.at(2.0, self.sws_push)

    def dte_RTCW(self, payload):
        self.rtc = int(payload) - CLOCK.time()
        self.ok('RTCW')

    def dte_RSTBW(self, payload):
        self.ok('RSTBW')

        def reboot():
            CLOCK.cancel_all('svc')
            self.boot()
            self.bench = None       # the CDC drops: the old link is dead
            self.start_services()
        CLOCK.at(1.0, reboot)

    # ---- services
    def stop_services(self):
        CLOCK.cancel_all('svc')
        self.bbr = False                       # %CFG cuts the GNSS rail
        self.gnss_active = False
        self.burst = False
        self.cool_armed = False

    def start_services(self):
        # Firmware order: service_init -> enforce_modulation_state -> resync
        # (ping), THEN "Argos ID=" (argos_tx_service.cpp:151 then :226).
        if self.i('ARP54') == 0:
            if self.radio == 'muet':
                self.log('WARN', 'KIM2Device::send_AT: timeout (cmd=0)', 0.3)
                self.log('WARN', 'KIM2Device::resync_rconf_cache: ping failed — module not responding', 0.4)
            elif self.p['IDP14']:
                # Genuine read-back of the master: cache_master_readback().
                import dte_campaign as C
                self.p['ART13'] = '1'                                   # LDK (SMP01 order)
                self.p['ART14'] = str(C._empreinte_rconf(self.p['IDP14']))
                self.refresh_art12()
        self.log('INFO', 'ArgosTxService::service_init: Argos ID=294910', 0.5)
        if self.i('UNP01'):
            CLOCK.at(5.0, tagged('svc', lambda: self.sws_first()))
        if self.i('AXP01'):
            CLOCK.at(30.0, tagged('svc', lambda: self.log(
                'INFO', 'BMA400: raw(12|-30|1010) g(0.01|-0.03|0.99) cal(0.01|-0.03|0.99)')))
        if self.test_mode:
            self.sws_push()

    def sws_first(self):
        self.log('INFO', 'UWDetectorService: state changed: state=0')
        self.uw_event(False, source='sws')

    def in_cooldown(self):
        return self.cool_until and CLOCK.time() < self.cool_until

    def uw_event(self, wet, source):
        if self.state != 'OPERATIONAL':
            return
        if wet:
            self.underwater = True
            self.burst = False
            if self.gnss_active:
                self.gnss_end(cancel=True)
            if self.i('ARP01') == 5:
                self.prepared = 128 if self.dx1() and self.mod() == 0 else (32 if self.dx1() and self.mod() == 1
                                                                            else 24)
            if self.cool_armed and self.rtc:
                self.cool_armed = False
                self.cool_until = CLOCK.time() + self.i('UNP20')
                self.log('INFO', f'ServiceManager: cycle complete at {int(self.rtc + CLOCK.time())}, '
                         'cooldown started', 0.1)
                self.log('INFO', f'[VAL-COOLDOWN] enter t={int(self.rtc + CLOCK.time())} '
                         f'interval_s={self.i("UNP20")}', 0.12)
                CLOCK.at(self.i('UNP20'), tagged('svc', self.cool_exit))
            return
        if self.in_cooldown():
            self.log('INFO', 'Service::notify_underwater_state: service GPS skipped (cooldown active)', 0.1)
            return
        self.underwater = False
        if self.errors:
            self.log('INFO', f'ArgosTxService: clearing {self.errors}-error suspension on surface event '
                     '— fresh session', 0.05)
            self.errors, self.suspended, self.hold_until = 0, False, 0
        if self.i('ARP01') == 5 and self.i('UNP30') == 0 and not self.in_cooldown():
            self.cool_armed = True
            self.log('INFO', 'ArgosTxService: cooldown armed (AT_SURFACE)', 0.06)
        if self.i('GNP01'):
            self.gnss_start()
        if self.i('ARP01') == 5:
            self.burst = True
            self.burst_count = 0
            CLOCK.at(1.0, tagged('svc', self.tx_attempt))

    def cool_exit(self):
        self.log('INFO', f'ServiceManager: exiting cooldown sleep (RTC={int(self.rtc + CLOCK.time())}) '
                 '— restarting SWS')
        CLOCK.at(3.0, tagged('svc', self.sws_first))

    def dx1(self):
        return self.i('UNP26') == 2 and self.i('UNP01') == 1

    def mod(self):
        return 1      # mute KIM2 with empty master: cache LDA2 (as on the October bench)

    def tx_attempt(self):
        if not self.burst or self.underwater or self.state != 'OPERATIONAL' or self.i('ARP01') == 0:
            return
        if self.suspended:
            return
        self.burst_count += 1
        if self.burst_count > max(1, self.i('ARP43')):
            return
        kind = 'doppler'
        if self.burst_count == 1 and self.prepared:
            self.log('INFO', f'ArgosTxService::process_doppler_burst: PREWARM mode={self.MOD_NAME[self.mod()]} '
                     f'sz={self.prepared} age=20000 ms')
            self.prepared = None
            kind = 'doppler-prewarm'
        self.log('INFO', 'KIM2Device::state_power_on', 0.2)
        if self.radio == 'vivant':
            self.log('INFO', 'KIM2Device::state_transmit_enter: TX START — RCONF=[x] mode=1, payload=3 bytes',
                     0.6)
            self.log('INFO', 'KIM2Device::state_transmit_enter: AT+TX=006400)', 0.62)
            self.log('INFO', 'KIM2Device::state_transmit: TX EMITTED OK — Argos message sent (mode=1, RCONF=[x])',
                     1.5)
            CLOCK.at(30.0, tagged('svc', self.tx_attempt))
            return
        self.log('WARN', 'KIM2Device::send_AT: timeout (cmd=0)', 1.2)
        self.log('ERROR', 'KIM2Device::state_error', 1.3)
        self.errors += 1
        n = self.errors
        self.log('WARN', f'ArgosTxService::react: KineisEventDeviceError (consecutive={n}/3)', 1.35)
        self.log('WARN', f'ArgosTxService: TX FAILED — type={kind} mode={self.mod()} (consecutive={n}/3, '
                 'will backoff/retry)', 1.4)
        if n >= 3:
            self.suspended = True
            self.log('ERROR', 'ArgosTxService: 3 consecutive device errors — suspending TX for 120 s, then one '
                     'probe', 1.5)
            return
        back = 60000 * (2 ** (n - 1))
        self.log('WARN', f'ArgosTxService: backoff {back} ms before next TX attempt', 1.5)
        CLOCK.at(1.5 + back / 1000.0, tagged('svc', self.tx_attempt))

    # ---- GNSS
    def gnss_start(self):
        self.gnss_active = True
        warm = self.bbr and self.i('GNP52') != 0
        if warm:
            self.log('INFO', '[VAL-GNSS] extint_pulse_wake from_state=7', 0.3)
            self.log('INFO', '[VAL-GNSS] extint_wake_presync baud=460800', 0.4)
            fresh = self.last_fix and CLOCK.time() - self.last_fix < 7200
            self.log('INFO', '[VAL-GNSS] state configure->%s users=1 t=0'
                     % ('startreceive' if fresh else 'sendofflinedatabase'), 0.6)
        else:
            self.log('INFO', '[VAL-GNSS] boot_sync_ok baud=9600 step=0 bbr=0', 0.8)
            self.log('INFO', '[VAL-GNSS] state configure->sendofflinedatabase users=1 t=0', 1.0)
            self.log('INFO', '[VAL-GNSS] ano_sent file=4096B msgs=30 bytes=3000 t=900', 1.6)
            self.log('INFO', 'M10QAsyncReceiver::state_senddatabase: injecting 2048 bytes of assistance', 1.8)
        cls = 'S1_hot bbr=1 n_eph=8' if warm else 'S0_cold bbr=0 n_eph=0'
        self.log('INFO', f'[VAL-GNSS] start_class={cls} n_aop=0 n_ano=0 n_alm=6 n_good=8 numSvs=10', 4.0)
        if self.ciel and self.i('GNP43') < 90:
            ttff = 8.0 if warm else 35.0
            CLOCK.at(ttff, tagged('svc', self.gnss_fix))

    def gnss_fix(self):
        if not self.gnss_active:
            return
        self.log('INFO', 'M10QAsyncReceiver: GNSS time persisted (1791590400), measured drift 1 ppm')
        self.last_fix = CLOCK.time()
        if self.i('GNP45') == 2 and self.i('GNP46') == 1:
            self.log('INFO', '[VAL-GNSS] cloudlocate_ready measc12=0 meas20=1 meas50=0', 0.1)
            self.log('INFO', 'GPSService::task_process_cloudlocate_data: stored MEAS20 (20 bytes)', 0.2)
            if self.mod() == 0:
                self.log('INFO', 'ArgosTxService::process_doppler_burst: CloudLocate 192 bits does not fit '
                         'master mod LDK (ARGOS_AD_MOD=0, LDA2 master required) — sending position/Doppler '
                         'instead', 0.3)
        self.gnss_end(cancel=False)

    def gnss_end(self, cancel):
        self.gnss_active = False
        g = self.i('GNP52')
        if g == 0:
            self.log('INFO', '[VAL-GNSS] dispatch=immediate_off GNP52=0', 0.1)
            self.bbr = False
        else:
            self.log('INFO', f'[VAL-GNSS] dispatch=deep_idle_armed_via_poweroff_chain GNP52={g}', 0.1)
            self.log('INFO', '[VAL-GNSS] dispatch=never_off GNP52=sentinel gate_reopen_s=86400', 0.12)
            self.log('INFO', '[VAL-GNSS] enterbackup_enter rail_warm=1', 0.5)
            self.log('INFO', '[VAL-GNSS] pmreq_verify_ok', 1.0)
            self.pmreq['seq'] += 1
            self.bbr = True


# --------------------------------------------------------------- fake Bench
def make_fake_bench(board):
    import kim_bench

    class _Ser:
        write_timeout = 5

    class FakeBench(kim_bench.Bench):
        def open(self):
            self.port = 'FAKE0'
            self.ser = _Ser()
            board.bench = self
            return self

        def _emit(self, line):
            with self._lock:
                self.history.append((CLOCK.monotonic(), line))
            try:
                self._logf.write(f'{CLOCK.monotonic() - T0:9.2f}  {line}\n')
            except Exception:
                pass
            for hook in list(self.line_hooks):
                try:
                    hook(line)
                except Exception:
                    pass

        def _send(self, s):
            try:
                self._logf.write(f'{CLOCK.monotonic() - T0:9.2f}  >> {s!r}\n')
            except Exception:
                pass
            if board.bench is not self:
                raise OSError('link down (simulated re-enumeration)')
            board.handle(s)

        def close(self):
            if board.bench is self:
                board.bench = None
            try:
                self._logf.close()
            except Exception:
                pass

    return FakeBench


def install(out_dir):
    import bench_platform as P
    import kim_bench
    import dte_campaign as C
    for mod in (kim_bench, C):
        mod.time = CLOCK
    kim_bench.LOGDIR = out_dir
    P.find_ports = lambda include_linux_glob=True: ['FAKE0']
    P.wait_for_port = lambda timeout=10.0: 'FAKE0'

    class _R:
        ok, returncode, stdout, timed_out = True, 0, 'Verify successful\nRun.\n', False
    P.nrfjprog = lambda *a, **k: _R()
    C.Runner.relink = lambda self: True
    return C, kim_bench


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--radio', choices=('muet', 'vivant'), default='muet')
    ap.add_argument('--ciel', action='store_true')
    ap.add_argument('--maitresse', action='store_true', help='IDP14 provisioned (LDK master)')
    ap.add_argument('--cases', default='', help='restrict the edge suite to these ids')
    ap.add_argument('--no-smoke', action='store_true', help='skip the kim_bench Suite')
    ap.add_argument('--no-fake-rf', action='store_true',
                    help='with --radio vivant: no synthetic emitter first, so the liveness probe itself is exercised')
    a = ap.parse_args()
    out = tempfile.mkdtemp(prefix='fake_bench_')
    os.environ['DTE_CAMPAIGN_OUT'] = out
    if a.ciel:
        os.environ['DTE_CIEL'] = '1'
    C, kim_bench = install(out)
    # dte_campaign read DTE_CAMPAIGN_OUT at import: point its outputs here too.
    C.OUT, C.RESULTS, C.LOG = out, os.path.join(out, 'campaign_results.jsonl'), os.path.join(out, 'campaign.log')
    board = Board(radio=a.radio, ciel=a.ciel, maitresse=a.maitresse)
    FakeBench = make_fake_bench(board)
    C.Bench = FakeBench

    # 1) kim_bench --run smoke suite
    if not a.no_smoke:
        b = FakeBench(port='FAKE0', quiet=True).open()
        ok = kim_bench.Suite(b).run()
        b.close()
        print(f'kim_bench Suite on the simulator: {"all PASS" if ok else "FAIL(s)"}')

    # 2) the edge suite, plus a synthetic emitter to exercise the RF guard
    cases = list(C.CASES_V33)
    if a.cases:
        keep = set(a.cases.split(','))
        cases = [c for c in cases if c['id'] in keep]
    if a.radio == 'vivant' and not a.no_fake_rf:
        def c_emetteur(r, case):
            r.b.enter_config()
            r.b.write_params({'ARGOS_MODE': 5, 'UNDERWATER_EN': 1})
            r.b.exit_config()
            r.b._send('%SURFACE\r\n')
            C._patiente(r, 30)
            r.record(case, 'PASS', 'emission attendue (synthetique)')
        cases.insert(0, dict(id='FAKE-RF', risque='MAJEUR', titre='synthetique: un cas qui fait emettre',
                             satellite=True, fn=c_emetteur))
    t0 = CLOCK.time()
    runner = C.Runner()
    rc = runner.run(cases)
    print(f'virtual time: {(CLOCK.time() - t0) / 60:.1f} min; results: {C.RESULTS}')
    verdicts = {}
    for cid, v, _ in runner.bilan:
        verdicts[cid] = v
    print(' '.join(f'{k}={v}' for k, v in verdicts.items()))
    return rc


if __name__ == '__main__':
    sys.exit(main())
