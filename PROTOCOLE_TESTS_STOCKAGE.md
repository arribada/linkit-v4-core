# Protocole de validation — extinction magnet / stockage / réveil
**Cible : LinkIt V4 KIM · instrument : Power Profiler Kit II · branche `fix/storage-wake-qualification`**

---

## 0. Préparation — à lire en entier avant de brancher quoi que ce soit

### 0.1 Le piège qui invalide tout
`BENCH_TEST` **supprime purement et simplement** `PMU::powerdown()` (`nrf_pmu.cpp`, bloc
`#if defined(BENCH_TEST) && !defined(BENCH_ALLOW_POWERDOWN)`) : la carte ne dort jamais, exprès,
pour rester joignable en USB. Toute mesure de stockage faite avec un build `--bench` classique
mesure une carte **allumée**.

- Tests de comportement + mesures : `./scripts/build_linkitv4_kim.sh --bench-storage`
- Mesures de référence « telles que livrées » : `./scripts/build_linkitv4_kim.sh --release`

`--bench-storage` implique `Debug`, donc `DEBUG_NO_WATCHDOG` : **pas de watchdog**. Conséquence à
connaître — si `SYSTEMOFF` est refusé (interface debug latchée), la boucle failsafe `for(;;) __WFI()`
est infinie et seul un reset sonde/coupure d'alim en sort. En release le WDT 15 min casserait la boucle.

### 0.2 POR obligatoire après chaque flash SWD
Le bit DIF (interface debug) reste latché jusqu'à un POR ou un pin reset ; le soft reset de
`powerdown()` ne l'efface pas. Une carte flashée puis endormie **sans coupure d'alimentation** ne fait
pas un vrai System OFF.

> **Après chaque `nrfjprog --program` : débrancher l'alimentation/batterie 5 s, rebrancher, et
> seulement ensuite commencer la séquence.** Le test T6 sert justement à quantifier ce qui se passe
> si on l'oublie.

### 0.3 Montage PPK2
- PPK2 en **source meter**, 3,6 V (ou ampèremètre en série sur la pile — dans ce cas, attention :
  couper la pile = POR, ce qui détruit l'état à mesurer).
- Échantillonnage 100 kHz, log continu — les réveils parasites durent quelques centaines de ms et
  se ratent en lecture instantanée.
- Repères attendus : stockage ≈ **8-20 µA** · idle System ON ≈ centaines de µA · session GNSS ≈ **25-35 mA**.

### 0.4 Nouvelles commandes console (build `--bench-storage`)
| Commande | Effet |
|---|---|
| `%RST` | cause décodée + `RESETREAS` brut + `GPREGRET` brut |
| `%REED` | état live du pin reed |
| `%SENSE` | liste **tout** pin avec un champ SENSE armé (= source de réveil System OFF) |
| `%STORE` | déclenche une vraie entrée en stockage sans aimant (l'USB tombe — c'est voulu) |

---

## 1. Séquence de référence — firmware **actuel** (`main` / `fix-gnss`, sans mes correctifs)

But : chiffrer le défaut avant de le corriger. À faire en premier, sinon on n'a pas de point de comparaison.

### T1 — Courant de stockage nominal
1. Build `--release`, flasher, **POR**.
2. Geste 2-temps d'extinction, **retirer l'aimant en moins de 3 s**.
3. Attendre 30 s, relever le courant moyen sur 60 s.

| Résultat | Interprétation |
|---|---|
| < 20 µA | conforme (référence f92592cc : 8,5 µA) |
| 100 µA – 2 mA | System OFF refusé → DIF latché (0.2 non respecté) ou rails non parkés |
| > 5 mA | la carte n'a jamais dormi → vérifier que le build n'est pas `--bench` |

### T2 — Réveil parasite ⚠️ **le test qui prouve le défaut**
1. Sur la carte de T1, en stockage. Log PPK2 en marche.
2. Approcher un aimant **200-300 ms** puis le retirer complètement. Ne plus rien toucher.
3. Observer 10 minutes.

- **Attendu firmware actuel** : boot complet, montée à quelques mA, session GNSS, **et le courant ne
  redescend jamais au niveau stockage**. C'est la cause n°1 des devices retrouvés à plat.
- **Attendu firmware corrigé** : brève impulsion (bootloader ≈ 200-500 ms) puis **retour < 20 µA**.

### T3 — L'extinction qui rallume
Trois essais, carte réveillée entre chaque :

| Essai | Aimant maintenu après confirmation | Attendu actuel | Attendu corrigé |
|---|---|---|---|
| a | 5 s | reste éteint | reste éteint |
| b | 8 s | reste éteint | reste éteint |
| c | **12 s** | **double bip + boot Operational** | clignotement **rouge** = « retire l'aimant », puis stockage dès le retrait |

### T4 — VBUS
Brancher l'USB sur une carte en stockage. Noter si elle boote et ce que `%RST` affiche ensuite.

### T5 — `RESETREAS` brut ⚠️ **le test qui décide de la suite**
Avec le build `--bench-storage`, relever `%RST` après chacun de ces trois boots :

| Boot | `RESETREAS` attendu | Ce que ça décide |
|---|---|---|
| réveil aimant depuis stockage | bit 16 (`0x00010000`) présent **ou absent** | **absent** ⇒ le bootloader mange le bit ⇒ le marqueur GPREGRET est la seule solution pour le parc |
| insertion batterie | `0x00000000` | confirme « batterie = état normal » |
| après un reset WDT | bit 1 (`DOG`) | contrôle de cohérence |

### T6 — DIF latché (coût d'oublier le POR)
Flasher en SWD, **sans** couper l'alim faire le geste d'extinction, mesurer 5 min.
Comparer à T1. Un écart d'un facteur 10 à 100 valide la règle « POR après SWD » comme consigne de production.

### T7 — Inventaire des sources de réveil
`%SENSE` juste avant `%STORE`. **Seul le reed doit apparaître.** Tout autre pin (notamment **P0.11**,
armé en SENSE_LOW par le bouton DFU du bootloader) est un vecteur de réveil parasite à désarmer.

---

## 2. Séquence de validation — firmware **corrigé** (`fix/storage-wake-qualification`)

Rejouer T1, T2, T3, T4, T7 à l'identique. Critères de passage :

| # | Critère de succès | Critère d'échec |
|---|---|---|
| T1 | < 20 µA, **3 flashs verts** visibles à l'entrée en stockage | pas de flash = `powerdown()` n'a pas suivi le chemin attendu |
| T2 | pic < 1 s puis retour < 20 µA, **aucune session GNSS** | tout état stable au-dessus de 100 µA |
| T3c | clignotement rouge tant que l'aimant est là, stockage dans les 500 ms après retrait | boot en Operational |
| T4 | (comportement à documenter, non corrigé à ce stade) | — |
| T7 | reed seul dans la liste | tout autre pin armé |

### T8 — Réveil délibéré (non-régression, **le test qui compte pour R1**)
Sur une carte en stockage corrigé : appliquer l'aimant et **le maintenir**.
- Attendu : boot, LED blanche, buzzer, puis double bip après 3 s → carte en marche.
- **La règle d'usage change et doit être documentée : pour réveiller un tag stocké, il faut
  appliquer l'aimant ET LE MAINTENIR ~4 s** (≈0,5 s de bootloader + 3 s de confirmation).
  Un tap bref ne réveille plus — c'est l'objet même du correctif.

### T9 — Aimant maintenu long / reed collé
Aimant laissé en place 2 min sur une carte en stockage.
Attendu : la carte boote (le maintien vaut confirmation) et reste stable — **pas** de cycle
boot/sleep répété. Puis, aimant retiré, le geste d'extinction doit fonctionner normalement.

### T10 — Stockage long (à lancer en dernier, laisser tourner)
24 h en stockage, log PPK2 continu.
- Charge cumulée attendue ≈ 20 µA × 24 h ≈ **0,5 mAh**.
- Tout pic isolé dans le log = réveil spontané : relever l'horodatage et `%RST` au retour.

### T11 — Hall à 2,3 V (résiduel matériel)
`storage_off_check()` descend VSYS à 2,3 V **avant** d'armer le SENSE du reed.
Vérifier que le réveil aimant fonctionne encore à ~0 °C (frigo 30 min) et à ~40 °C.
Un échec ici = le capteur ne tient pas 2,3 V → il faudra supprimer la descente de rail
(≈ 8 µA de gain, sans commune mesure avec le risque).

---

## 3. Ce qu'il faut remonter

Pour chaque test : n° · build utilisé · courant moyen et pics · sortie `%RST` / `%SENSE` · verdict.
Les captures PPK2 de T2 et T10 sont les deux pièces à conserver — ce sont elles qui prouvent
que le réveil parasite est mort.
