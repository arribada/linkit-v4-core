#!/usr/bin/env python3
"""
storage_bench.py — campagne extinction magnet / stockage / réveil (LinkIt V4 KIM).

Principe : en System OFF le périphérique USB du nRF52840 disparaît du bus. La
présence de /dev/ttyACM* (ou du VID Nordic dans lsusb) est donc un indicateur
binaire et instrument-free de « la carte est réveillée » — suffisant pour prouver
un réveil parasite sans PPK2. Le PPK2, quand il est là, ne sert qu'à chiffrer le
courant ; il ne change aucun verdict PASS/FAIL de T2, T3, T8 ou T9.

Les étapes qui demandent un aimant sont explicitement guidées : le script annonce
le geste, compte le temps, et enregistre ce que la carte fait ensuite.

    ./storage_bench.py --watch [secondes]      trace AWAKE/ASLEEP horodatée
    ./storage_bench.py --rst                   %RST  (cause + RESETREAS + GPREGRET)
    ./storage_bench.py --sense                 %SENSE (sources de réveil armées)
    ./storage_bench.py --store                 %STORE puis vérifie l'endormissement
    ./storage_bench.py --t2                    T2 — réveil parasite (tap d'aimant)
    ./storage_bench.py --t3 12                 T3 — aimant maintenu N s après extinction
    ./storage_bench.py --t8                    T8 — réveil délibéré (maintien 4 s)
"""

import argparse, glob, subprocess, sys, time
from datetime import datetime
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import bench_platform as P  # noqa: E402

NORDIC_VID = "1915:"
POLL_S     = 0.25

try:
    import serial
except ImportError:
    serial = None


def ts() -> str:
    return datetime.now().strftime("%H:%M:%S.%f")[:-3]


def log(msg: str) -> None:
    print(f"[{ts()}] {msg}", flush=True)


def port_path():
    """Port CDC de la carte (VID:PID 1915:520F, Windows ou Linux), ou None si elle
    n'est pas enumeree -- en System OFF le peripherique USB du nRF disparait."""
    ports = P.find_ports()
    return ports[0] if ports else None


def usb_present() -> bool:
    """Vrai tant que le nRF est enumere. Sous Linux on garde en plus lsusb, parce
    que sous WSL le noeud peut survivre quelques centaines de ms a la disparition."""
    if port_path():
        return True
    if P.IS_WINDOWS:
        return False
    try:
        out = subprocess.run(["lsusb"], capture_output=True, text=True, timeout=5).stdout
        return NORDIC_VID in out
    except Exception:
        return False


def wait_state(target_awake: bool, timeout_s: float) -> float | None:
    """Attend AWAKE (True) ou ASLEEP (False). Retourne le délai, ou None si timeout."""
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        if usb_present() == target_awake:
            return time.time() - t0
        time.sleep(POLL_S)
    return None


def send(cmd: str, wait_reply_s: float = 2.0) -> list[str]:
    """Envoie une commande '%' et renvoie les lignes de réponse."""
    if serial is None:
        sys.exit("pyserial manquant : pip install pyserial")
    p = port_path()
    if not p:
        log("ERREUR : carte non énumérée (endormie ?) — impossible d'envoyer " + cmd)
        return []
    out = []
    try:
        s = serial.Serial(p, 115200, timeout=0.3)
    except serial.SerialException as e:
        log(f"  port {p} inaccessible ({e})")
        return []
    with s:
        s.reset_input_buffer()
        s.write((cmd + "\r\n").encode())
        t0 = time.time()
        while time.time() - t0 < wait_reply_s:
            try:
                raw = s.readline()
            except serial.SerialException:
                # %STORE : la carte part en System OFF et le port disparait sous
                # nos pieds (Windows : ClearCommError, PermissionError 13).
                log("  (port disparu pendant la lecture -- attendu apres %STORE)")
                break
            line = raw.decode(errors="replace").strip()
            if not line:
                continue
            if line.startswith("%"):
                out.append(line)
                log("  << " + line)
    if not out:
        log(f"  (aucune réponse à {cmd} — firmware sans le hook, ou build sans BENCH_TEST)")
    return out


def cmd_watch(seconds: float) -> None:
    log(f"Trace d'état sur {seconds:.0f} s — Ctrl-C pour arrêter")
    last = usb_present()
    log(f"état initial : {'AWAKE (USB énuméré)' if last else 'ASLEEP (pas d USB)'}")
    t0 = time.time()
    transitions = 0
    while time.time() - t0 < seconds:
        now = usb_present()
        if now != last:
            transitions += 1
            log(f"*** TRANSITION → {'AWAKE' if now else 'ASLEEP'} (t+{time.time()-t0:6.1f} s)")
            last = now
        time.sleep(POLL_S)
    log(f"fin de trace — {transitions} transition(s), état final "
        f"{'AWAKE' if last else 'ASLEEP'}")


def enter_storage_via_console() -> bool:
    log("Entrée en stockage par %STORE (sans aimant)")
    send("%STORE", 1.5)
    d = wait_state(False, 20)
    if d is None:
        log("ÉCHEC : la carte est toujours énumérée 20 s après %STORE.")
        log("        → build sans BENCH_ALLOW_POWERDOWN (powerdown supprimé), ou")
        log("          powerdown() n'a pas abouti. Voir §0.1 du protocole.")
        return False
    log(f"OK : plus d'USB après {d:.1f} s → la carte est en System OFF")
    return True


def prompt(msg: str) -> None:
    print(f"\n>>> {msg}", flush=True)
    # STORAGE_PROMPT_S=<n> : pas de clavier (agent distant) -> le geste est
    # annonce, puis on laisse n secondes a l'operateur au lieu d'attendre Entree.
    wait = os.environ.get("STORAGE_PROMPT_S")
    if wait:
        log(f"    (geste attendu dans les {wait} s)")
        time.sleep(float(wait))
    else:
        input("    [Entrée quand c'est fait] ")


def cmd_t2() -> None:
    log("=== T2 — réveil parasite (LE test qui prouve le défaut) ===")
    if not usb_present():
        log("Carte déjà endormie — on part de là.")
    else:
        if not enter_storage_via_console():
            prompt("Fais l'extinction 2-temps à l'aimant, PUIS retire l'aimant tout de suite")
            if wait_state(False, 30) is None:
                log("ÉCHEC : la carte ne s'endort pas. Abandon de T2.")
                return
    time.sleep(3)
    prompt("Approche l'aimant ~200 ms puis RETIRE-LE complètement. Ne touche plus à rien")
    log("Observation 120 s…")
    d = wait_state(True, 20)
    if d is None:
        log("RÉSULTAT : aucun réveil détecté → le tap bref ne réveille pas.")
        log("VERDICT T2 : PASS (comportement corrigé)")
        return
    log(f"La carte s'est réveillée après {d:.1f} s. On regarde si elle se rendort…")
    d2 = wait_state(False, 100)
    if d2 is None:
        log("RÉSULTAT : réveillée et TOUJOURS ACTIVE après 100 s.")
        log("VERDICT T2 : FAIL — c'est le défaut D1 : un aimant parasite déclenche")
        log("             une session complète et la carte ne retourne jamais dormir.")
    else:
        log(f"RÉSULTAT : réveillée puis rendormie après {d2:.1f} s.")
        log("VERDICT T2 : PASS si ce délai est court (re-dodo qualifié) — noter la valeur.")


def cmd_t3(hold_s: float) -> None:
    log(f"=== T3 — aimant maintenu {hold_s:.0f} s après la confirmation d'extinction ===")
    if not usb_present():
        log("Carte endormie : réveille-la d'abord (aimant maintenu ~4 s).")
        return
    prompt(f"Fais le geste d'extinction 2-temps, puis GARDE l'aimant en place {hold_s:.0f} s")
    log(f"Observation pendant {hold_s + 40:.0f} s…")
    t0 = time.time()
    slept = False
    while time.time() - t0 < hold_s + 40:
        awake = usb_present()
        if not awake and not slept:
            slept = True
            log(f"*** ASLEEP à t+{time.time()-t0:.1f} s")
        elif awake and slept:
            log(f"*** RÉVEIL à t+{time.time()-t0:.1f} s — la carte est repartie")
            log("VERDICT T3 : FAIL — l'extinction s'est transformée en allumage (D2)")
            return
        time.sleep(POLL_S)
    if slept:
        log("VERDICT T3 : PASS — la carte est restée endormie")
    else:
        log("VERDICT T3 : FAIL — elle ne s'est jamais endormie (D2)")


def cmd_t8() -> None:
    log("=== T8 — réveil délibéré (non-régression R1) ===")
    if usb_present():
        if not enter_storage_via_console():
            return
        time.sleep(2)
    prompt("Applique l'aimant et MAINTIENS-LE au moins 5 s (bootloader ~0,5 s + 3 s de confirmation)")
    d = wait_state(True, 30)
    if d is None:
        log("VERDICT T8 : FAIL — la carte ne se réveille pas au magnet. CRITIQUE (R1).")
        return
    log(f"Réveil détecté après {d:.1f} s")
    time.sleep(6)
    if usb_present():
        log("VERDICT T8 : PASS — réveillée et stable")
        send("%RST")
    else:
        log("VERDICT T8 : FAIL — réveillée puis rendormie : le maintien n'a pas été pris.")


def cmd_soak(seconds: float) -> int:
    """Stockage sans aimant : verifie les sources de reveil, entre en System OFF,
    puis surveille pendant `seconds` qu'AUCUN reveil ne se produit. 0 = PASS."""
    log(f"=== SOAK — aucun reveil inopine pendant {seconds:.0f} s ===")
    if not usb_present():
        log("Carte deja endormie : reveille-la (aimant ~4 s) puis relance.")
        return 2
    lines = send("%SENSE", 3.0)
    autres = [l for l in lines if l.startswith("%SENSE pin=")]
    log(f"sources de reveil armees : {autres or 'aucune ligne pin='}")
    send("%RST", 2.0)
    if not enter_storage_via_console():
        return 1
    t0 = time.time()
    while time.time() - t0 < seconds:
        if usb_present():
            log(f"VERDICT SOAK : FAIL — REVEIL INOPINE a t+{time.time()-t0:.1f} s")
            time.sleep(8)
            send("%RST", 2.0)
            return 1
        time.sleep(POLL_S)
    log(f"VERDICT SOAK : PASS — aucune re-enumeration USB en {seconds:.0f} s")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--watch", nargs="?", const=60, type=float)
    ap.add_argument("--rst", action="store_true")
    ap.add_argument("--sense", action="store_true")
    ap.add_argument("--store", action="store_true")
    ap.add_argument("--t2", action="store_true")
    ap.add_argument("--t3", nargs="?", const=12, type=float)
    ap.add_argument("--t8", action="store_true")
    ap.add_argument("--soak", nargs="?", const=3600, type=float,
                    help="%%STORE puis surveillance sans aimant (defaut 3600 s)")
    a = ap.parse_args()

    if a.watch is not None: cmd_watch(a.watch)
    elif a.rst:             send("%RST")
    elif a.sense:           send("%SENSE", 3.0)
    elif a.store:           enter_storage_via_console()
    elif a.t2:              cmd_t2()
    elif a.t3 is not None:  cmd_t3(a.t3)
    elif a.t8:              cmd_t8()
    elif a.soak is not None: return cmd_soak(a.soak)
    else:                   ap.print_help()
    return 0


if __name__ == "__main__":
    sys.exit(main())
