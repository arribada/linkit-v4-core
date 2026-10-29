# Campagne de test finale avant merge — branche `v4.2.1-optim`

**Objet :** la liste de tout ce qu'il faut vérifier, sans régression, avant de fusionner
cette branche dans `main`. Le document grandit au fil des correctifs de l'audit des coûts
cachés du 27 octobre 2026, traités un par un. Chaque correctif y ajoute sa section, avec
ce qui a déjà été vérifié et ce qui reste à faire.

**Ce document ne remplace pas [`PLAN_VALIDATION_v4.2.1.md`](PLAN_VALIDATION_v4.2.1.md).**
Celui-ci reste la référence pour la validation générale de la branche : préconditions
de banc (§0), régression des commits d'octobre (partie A), validation de déploiement
(partie B). Ici ne figure que ce qui s'y ajoute.

**Règles de la campagne :**
- Chaque correctif est validé par les tests sur PC **et** sur une vraie carte.
- Les résultats sont rapportés bruts, échecs compris. Une case n'est cochée que si le
  test a été fait et a passé, avec la preuve indiquée.
- **Aucune émission radio** sans accord explicite, antenne ou charge, et alimentation
  d'au moins 1 A. Un KIM2 qui répond émet même sans identifiants. `$SMDTST` émet.

---

## 0. Préconditions

Celles de [`PLAN_VALIDATION_v4.2.1.md` §0](PLAN_VALIDATION_v4.2.1.md) s'appliquent :
variante de firmware conforme à la carte, pont USB, séquence de flash
`--recover` + hex fusionné, et `Verify successful` + `Run.` obligatoires.

Avec ces corrections et compléments, établis au banc le 27 octobre 2026 :

| Point | Ce qui est vrai |
|---|---|
| Magasin de configuration | **Il survit au `--recover`.** Vérifié deux fois le 27/10 : la configuration de test était intacte après chaque flash. La mention contraire de `PLAN_VALIDATION` §0.4 et du `CLAUDE.md` était fausse ; corrigée dans les deux le 28/10. |
| APPROTECT | Se réarme à **chaque reset** : le SDK 17.0.2 ne fait pas la séquence `APPROTECT.DISABLE`. Après un reset, le J-Link ne peut plus lire ni écrire la mémoire sans `--recover`, qui efface la puce. Une mesure qui dépend du J-Link ne survit donc à aucun redémarrage. |
| Mesures de courant | **Le PPK doit être la seule alimentation.** USB branché, le PPK lit de 0 à environ la moitié du courant réel, selon le moment. Une mesure d'écart (A/B) reste visible mais sous-estimée ; une valeur absolue est fausse. |
| Carte du banc LinkIt SMD | **Défectueuse** : elle consomme plus qu'une carte normale (environ 19 à 24 mA en veille). Ses valeurs absolues ne valent rien ; seuls les écarts mesurés sur elle comptent. Les valeurs absolues se mesurent sur une carte saine (une LinkIt KIM est disponible). |
| GNSS sans position | La relance après démarrage à froid (`GNP23`, 60 s par défaut) relance une session chaque minute tant qu'aucune position n'a été obtenue, quelle que soit la période `ARP11`. |
| Campagne DTE sans émission | **`--suite sans-radio`**, jamais `interieur`, qui contient les cas satellite (`ARG-`, `SURF-`, `MOD-`…). Le 29/10, ARG-03 a fait émettre le SMD de la carte du banc, port RF ouvert. **Une carte sans `IDP13`/`IDP14` n'est pas muette** : le module garde ses propres identifiants, et le 1er ping d'émersion en SURFACING_BURST ne vérifie pas ceux de la carte (laissé tel quel par l'utilisateur : ce ping ne doit pas être retardé). Le garde du harnais n'agit qu'après la 1re émission. |

**Pilotage du PPK2 depuis WSL** : par le Python Windows, avec la bibliothèque `ppk2-api`
installée dans `C:\Python314` et le script
`C:\Users\fourn\AppData\Local\Temp\claude_ppk\ppk_log.py`. La commande
`ppk_log.py COM6 3600 <durée_s> <fichier.csv> [cycle]` alimente la carte à 3,6 V et écrit
la moyenne du courant toutes les 200 ms. Il faut d'abord libérer COM6 dans Power Profiler.
**Le PPK coupe l'alimentation de la carte à la fin de chaque enregistrement**, malgré
ce qu'annonce le commentaire du script. USB débranché, la carte s'éteint donc à la fin
de la mesure (constaté le 27/10 : 0 V vu par le J-Link). Pour flasher ou garder la carte
en marche, rebrancher l'USB ou lancer un enregistrement qui couvre l'opération.

**Configuration de test du 27/10 à remettre après la campagne.** La carte du banc a été
réglée pour atteindre vite la veille GNSS. Valeurs d'origine à reposer :

| Clé | Test | Origine |
|---|---|---|
| GNP05 | 30 | 120 |
| GNP09 | 30 | 240 |
| GNP23 | 3600 | 60 |
| ARP11 | 10 | 13 |
| UNP01 | 0 | 1 |
| LDP01 | 0 | 1 |

GNP01, GNP52, ARP01, CAP01 et GNP25 n'ont pas changé (1, 4294967295, 0, 0, 1).

---

## 1. Suites automatiques — sur l'arbre final, juste avant le merge

| # | Commande | Attendu |
|---|---|---|
| 1.1 | `cd tests && ./build/TrackerTests -p -xg SWS` | tout passe (1120/1120 au 29/10, après stockage C5 de 2.15 ; aussi en un seul processus, sans `-p`) — ✅ 29/10 sur l'arbre final `2d8194fa`, avec et sans `-p` |
| 1.2 | `cd tests && ./build/TrackerTestsCyprus -p -xg SWS` | tout passe (1138/1138 au 29/10, après stockage C5 de 2.15 ; aussi en un seul processus) — ✅ 29/10 sur `2d8194fa` |
| 1.3 | `cd tests && ./build/TrackerTests -p -g SWSAnalog`, puis `-g SWSAnalogFlash`, puis `-sg SWSLongRun -sn H45_FixedDelayClean_ControlDetectsSurfacings` | tout passe |
| 1.4 | `cd tests && ./build/TurtleSimulation -v` | 11/11, dont les tests budget `BatteryAndBusyWaitWithinBudget` et `JournalAtWarnHoldsOnlyFaults` |
| 1.5 | `cd tests && ./build/RSPBSimulation -v` | 15/15 au 29/10, après P6 de 2.15 |
| 1.6 | `python3 scripts/check_param_tables.py` | OK |
| 1.7 | `python3 tools/check_cached_modulation_writes.py`, `scripts/check_doc_links.py`, `tools/check_battery_guard_callers.py` | OK |
| 1.8 | `python3 tools/check_gpio_output_pulls.py` (nouveau) | OK |
| 1.9 | clang-format sur les fichiers modifiés (commande du `CLAUDE.md`, section CI) | aucune violation — ✅ 29/10 : 8 fichiers de la branche refusés, reformatés (`2d8194fa`), puis les 192 conformes avec le filtre exact du job `format` de `tests.yml` |
| 1.10 | Matrice firmware : KIM, SMD SPI, SMD UART, LoRa, SMD SPI + caméra UART, RSPB (tension et mixte), le woodbuoy de CLS (`scripts/build_woodbuoy.sh`), et les builds de banc, caméra UART comprise | compile, 0 avertissement, et une marge RAM positive : `arm-none-eabi-size -A`, `.data + .bss + .noinit` plus la pile sous 250 240 o (2.14) — ✅ 29/10 sur `2d8194fa` : 15 variantes (dont RSPB mixte et woodbuoy), 0 avertissement, marge la plus faible 2964 o (SMD SPI + caméra UART, banc) |
| 1.11 | `python3 tools/check_log_lines.py` (nouveau) | OK : aucune ligne de journal ne dépasse 118 octets |
| 1.12 | `python3 tools/check_bench_log_patterns.py` | « toutes presentes » : chaque texte attendu par le banc existe dans le firmware |

**Le test budget de 1.4 est un cliquet.** Il plafonne les mesures batterie (49 par jour
simulé), l'attente active (1 s par jour) et les lignes écrites dans le journal flash
(1380 par jour depuis le point 2, 3100 avant). Un correctif qui supprime des lignes de
journal doit **baisser** ce plafond (`SYSLOG_LINES_RATCHET_PER_DAY`,
`tests/src/turtle_mission_sim.cpp`), jamais le remonter sans justification. Le test
imprime les 40 lignes les plus fréquentes : c'est par là qu'on cherche ce qui a grossi.

---

## 2. Correctifs de cette série — vérifications par correctif

### 2.1 Batterie : une mesure au plus toutes les 30 minutes

**Ce qui a changé.** Avant, chaque lecture de la configuration refaisait une mesure ADC,
précédée d'une attente active de 500 ms sur KIM et 100 ms sur SMD : 4023 mesures par
jour simulé, 13 avant la première trame d'une émersion. Maintenant,
`BatteryMonitor::update()` réutilise une mesure de moins de 30 min. Une mesure est prise
quand même dans quatre cas : `SENSR`, un `STATR` qui lit la batterie (mesure de moins de
10 s), un changement des seuils `LBP02` ou `LBP12`, et 5 min après une mesure rejetée.

| Vérification | Comment | Fait |
|---|---|---|
| Tests PC : règle de mesure | groupe `BatterySampling` (8 tests) | ✅ 27/10 |
| Tests PC : budget sur un jour simulé | `TurtleMission.BatteryAndBusyWaitWithinBudget` : 42 mesures par jour, au plus 1 avant la 1re trame | ✅ 27/10 |
| Compilation RSPB, jauge STC3117 | matrice : mode tension et mode mixte | ✅ 27/10 |
| Carte : valeurs batterie plausibles | `STATR` → `BATT_SOC`, `BATT_VOLTAGE` cohérents avec la tension réelle | ✅ 29/10 : `POT06` 4,276 V et `POT03` 100 % sur l'USB, `SENSR` 4192 mV (84 mV d'écart) |
| Carte : `STATR` répété | deux `STATR` à moins de 10 s → même valeur ; à plus de 10 s → nouvelle mesure | ✅ 29/10, mesuré au temps de réponse (une mesure coûte 100 ms d'attente active sur cette carte) : à +2 s 102/122/101 ms, réutilisée ; à +14 s 202/202/222 ms, nouvelle mesure ; référence 102 ms |
| Carte : `SENSR` | une nouvelle mesure à chaque appel | ✅ 29/10 : deux `SENSR` de suite, 404 ms chacun, deux mesures (4222 puis 4202 mV) |
| Carte : changement de seuil | `PARMW LBP02` → le drapeau batterie faible suit au refresh suivant, sans attendre 30 min | ✅ 29/10 en partie : après `LBP02` changé, le `STATR` suivant mesure (202 ms, une mesure venait d'être prise) ; même valeur reposée, il réutilise (102 ms). Le drapeau lui-même ne bascule pas sur l'USB (SOC 100 %) : à voir sur une alimentation réglable |
| Carte : 1re mesure au démarrage | pas de 0 V dans le premier paquet après une mise sous tension | ✅ 29/10 : la 1re entrée GNSS après le flash porte 4,251 V (le reset vide la RAM comme une mise sous tension ; sans émission, l'entrée GNSS tient lieu de paquet) |
| RSPB : jauge | `SENSR` renvoie des valeurs vivantes, émission et GNSS normaux sur un cycle TPL | ⬜ |

### 2.2 Point 1 : plus de tirage interne sur les sorties pilotées

**Ce qui a changé.** Plusieurs sorties pilotées en push-pull avaient leur tirage interne
actif : `GPS_PWR_EN` (P1.06), `SAT_EN` (P1.15), `SAT_WKUP` (P0.30) et `POWER_CONTROL`
(P0.04) sur LinkIt, `GPS_PWR_EN` et `SAT_EN` sur RSPB. Elles sont passées en
`NRF_GPIO_PIN_NOPULL` (`ports/nrf52840/bsp/*/bsp.cpp`). Un tirage vers la masse sur une
sortie à l'état haut conduit environ 0,25 mA en permanence ; or `GPS_PWR_EN` reste haute
pendant toute la veille GNSS. Le nouveau contrôle `tools/check_gpio_output_pulls.py`,
ajouté en CI, refuse désormais une telle configuration.

| Vérification | Comment | Fait |
|---|---|---|
| Le défaut existe (avant correctif) | `%PULLAB 30 14` sur la carte du banc, PPK2 : écart tirage − sans tirage **117 ± 13 µA** sur 7 cycles, USB branché (partage de l'alimentation, valeur sous-estimée) | ✅ 27/10 |
| Contrôle CI | échoue sur l'ancien code (6 broches signalées), passe sur le nouveau | ✅ 27/10 |
| Compilation | matrice firmware : 7 variantes, RSPB comprise, 0 avertissement | ✅ 27/10 |
| Carte : rail GNSS | 3 allumages du rail → le M10Q répond à chaque fois (`boot sync at 9600 baud`) ; 4 passages en veille vérifiés (`PMREQ-backup verified`) ; coupure en fin de veille (`poweroff_from_deep_idle: cutting rail`) puis rallumage → le M10Q répond | ✅ 27/10 |
| Carte : module satellite | `PWRON 3` et `PWRON 4` acceptés (`$O`), mais l'effet n'est pas mesurable USB branché : le PPK n'y voit qu'une part du courant, qui varie. **À refaire** USB débranché, ou par une session SMD complète en firmware SPI | ⚠️ partiel |
| Carte : mesure après correctif | `%PULLAB 30 10` sur le firmware corrigé (`bsp_pull=none`) : l'état par défaut est au niveau bas, et remettre le tirage fait remonter le courant dans les 9 comparaisons (+88 ± 31 µA, USB branché). Avant correctif : +142 ± 50 µA sur 13 comparaisons. Les amplitudes diffèrent parce que l'USB ne fournit pas la même part d'un enregistrement à l'autre | ✅ 27/10 (écart relatif) |
| Carte du banc, USB débranché | `%PULLAB 30 32` lancé USB branché, puis USB débranché à +21 s : le PPK, seule alimentation, voit toute la consommation. Remettre le tirage coûte **+237 µA, écart-type 2,6 µA, 29 comparaisons sur 29 positives** (233 à 243 µA). C'est la valeur attendue (environ 3,1 V / 13 kΩ). Avec le rail GNSS allumé en permanence (never-poweroff), l'économie est d'environ **5,7 mAh par jour** | ✅ 27/10 |
| Carte saine, valeur absolue | courant de veille GNSS absolu sur une carte non défectueuse, pour le bilan énergétique (l'écart, lui, est établi ci-dessus) | ⬜ |
| Carte KIM2 | réveil du module par `SAT_WKUP` (mode BLIND) et émission normale | ⬜ |
| RSPB | GNSS et module SMD alimentés normalement sur un cycle TPL5111 | ⬜ |
| Carte SMD en firmware SPI | session SMD complète (`$SMDDFU#001;5`), sans émission | ⬜ |

Observé pendant la validation, **préexistant** : `M10QAsyncReceiver: fast path failed
(attempt 1/3) — retry` au réveil depuis la veille, suivi d'une reprise réussie. Le même
message figure dans l'enregistrement fait avant le correctif. À surveiller, mais ce
n'est pas une régression de ce point.

### 2.3 Pilotage UART de la RunCam Split 4 (`CAM_CONTROL=UART`)

Validé sur la caméra du banc les 26 et 27/10 :
- départ et arrêt vérifiés sur la carte SD, avec un rattrapage ;
- horloge réglée en UTC ;
- résolution par `CAP07`, avec redémarrage de la caméra ;
- commande `CAMTST` ;
- état `CAT00` à `CAT06` ;
- firmware non-banc en fonctionnement normal avec le GNSS.

| Reste à vérifier | Fait |
|---|---|
| Build `CAM_CONTROL=BUTTON` sur carte (pilote inchangé mais jamais lancé pendant ces travaux) | ⬜ |
| `CAP06=AUTO` quand la caméra démarre seule (testé sur PC seulement) | ⬜ |
| Alimentation réelle de la caméra par `CAM_PWR_EN` (la caméra du banc est alimentée à part) | ⬜ |
| Avec le module SMD en firmware SPI : `CAMTST`, `$SMDDFU#001;5`, puis `%CAM SNIFF` après une extinction du SMD (le SMD en firmware UART masquait la caméra) | ⬜ |

### 2.4 Point 2 : le journal flash, une ligne par étape

**Ce qui a changé.** En Release, chaque ligne INFO, WARN ou ERROR est une écriture
LittleFS de 70 à 480 ms, dans un journal de 8192 entrées. Les services en écrivaient
2811 par jour tortue simulé, et les pilotes en ajoutaient sur carte : environ 4 par
émission KIM2, 6 à 10 par session M10Q. L'historique tenait 1,5 à 4 jours.
- Nouveau niveau `DEBUG_DETAIL` (`core/logging/debug.hpp`) : affiché sur la console de
  banc sous l'étiquette `[DETAIL]`, jamais écrit en flash. Les sous-étapes y passent,
  **texte inchangé**, pour que les scripts du banc les lisent toujours sur la console.
- Le journal garde une ligne par étape : démarrage (version et validité de la
  configuration, cause du reset, état des journaux en une ligne), entrée dans chaque
  état, plongée et remontée avec la durée de l'état précédent, début de session GNSS
  avec les champs de diagnostic, fix ou absence de fix, rail GNSS, prochaine session,
  chaque émission (`TX SUCCESS`, désormais aussi sur SMD hors build de validation), geste
  aimant confirmé. WARN et ERROR inchangés, sauf deux WARN qui décrivaient un
  fonctionnement normal (« NO_FIX cached », « no satellite detected »), devenus des
  détails : la ligne de fin de session les dit déjà.
- **Nouvelle ligne de fix** (`task_process_gnss_data: fix in N s, hAcc=M m, numSV=K`) :
  le journal n'en avait aucune, et le compteur de fixes de `log_harvest.py` restait à 0.
- 53 messages tronqués à 118 octets raccourcis, l'explication passée en commentaire ;
  nouveau contrôle CI `tools/check_log_lines.py`. `tools/check_bench_log_patterns.py`
  ajouté en CI.
- Corrigé au passage : « cycle complete …, cooldown started » écrit en flash à chaque
  remontée alors qu'aucun cooldown n'était configuré.
- **Nouveau paramètre `DBP02` `SYSTEM_LOG_LEVEL`** (0 OFF, 1 ERROR, 2 WARN, 3 INFO ;
  3 par défaut, le comportement d'avant). Ajouté en fin de table, sans changer le code de
  version. Il ne filtre que le journal flash : ni la console, ni les autres journaux, ni
  le comportement. Il s'applique à l'entrée en PreOperational : l'enregistrement du
  démarrage (version, cause du reset, trace de plantage) est donc toujours écrit, et une
  valeur posée par DTE prend effet à la sortie de configuration. Une ligne filtrée ne
  coûte rien (ni mise en forme, ni place dans la fenêtre de rétention, ni flash).
  Pour une tortue qu'on ne relira pas : `DBP02=2`.
- Trois lignes passées de WARN à INFO ou DETAIL parce qu'elles décrivaient un
  fonctionnement normal, et qu'à WARN elles auraient été tout le journal : « no
  satellite detected » et « NO_FIX cached » (la fin de session le dit), la session GNSS
  coupée par une plongée (seul le dépassement du délai de sécurité reste un WARN), et
  « sensor timeout » quand seul le bit GNSS manque (le gestionnaire n'y remplit rien).
- Noté ici, **corrigé le 28/10 (2.12)** : ce même gestionnaire ne complétait qu'**un**
  capteur manquant par délai (chaîne de `else if`). Avec deux capteurs absents, le second
  n'était jamais rempli et la pile n'était pas mise à jour. Sans effet tant qu'aucun
  capteur hors GNSS n'est activé dans l'émission, ce qui est le cas des modèles tortue.

| Vérification | Comment | Fait |
|---|---|---|
| Tests PC : un DETAIL n'atteint jamais le flash | `DebugLogger.DetailReachesOnlyTheConsoleTaggedDetail`, `DetailIsNeverReplayedByTheHoldWindow` | ✅ 27/10 |
| Tests PC : budget | `BatteryAndBusyWaitWithinBudget` : **1252 lignes par jour** (14,9 par remontée, contre 2811 et 33,5), cliquet abaissé à 1380. La simulation compte désormais par le journal lui-même (`DEBUG_TO_SYSTEMLOG`), niveau et fenêtre de rétention compris | ✅ 27/10 |
| Tests PC : niveau WARN | `JournalAtWarnHoldsOnlyFaults` : à `DBP02=2`, la même journée écrit **3 lignes** (avertissements de configuration du démarrage), plafond 10 ; les 691 émissions partent toujours | ✅ 27/10 |
| Tests PC : paramètre | `DebugLogger.SystemLogLevel*` (3 tests : filtre du flash seul, fenêtre de rétention, OFF), `Sm.PreOperationalAppliesSystemLogLevel`, `DTEHandler.PARMW_DBP02_SystemLogLevel` (défaut 3, 0..3 accepté, 4 refusé) | ✅ 27/10 |
| Tests PC : suites | `-p -xg SWS` 1039/1039, Cyprus 1057/1057, SWSAnalog 62, SWSAnalogFlash 27, TurtleSimulation 11/11, RSPBSimulation 10/10 | ✅ 27/10 |
| Contrôles | `check_log_lines` : 1477 lignes, toutes ≤ 118 octets ; `check_bench_log_patterns` : 123 chaînes, toutes présentes ; clang-format : aucune violation nouvelle | ✅ 27/10 |
| Compilation | matrice 7 variantes sur l'arbre final (avec `DBP02`), plus un build `DEBUG_LEVEL=2` : 0 avertissement ; build de banc, 0 avertissement | ✅ 27/10 |
| Carte : console | cycle complet (démarrage, OP, session GNSS, veille, coupure, rallumage, entrée et sortie de configuration) : les sous-étapes apparaissent en `[DETAIL]`, les étapes en `[INFO]` | ✅ 27/10, build de banc SMD UART : 13 lignes `[INFO]` et 54 `[DETAIL]` sur le cycle |
| Carte : journal flash | `DUMPD` du `system.log` après ce cycle : seules les lignes d'étape, compter les entrées ajoutées par session GNSS (5 attendues : `M10Q on`, `boot sync`, fin, rail, prochaine session) | ✅ 27/10 : les 5, plus « auto-poweroff timer fired » et sa replanification quand le rail est coupé. Vus à cette occasion : l'ERROR « error opening MGA ANO file » à **chaque** session, corrigé (désormais une fois par démarrage, ensuite `[DETAIL]`) ; le WARN de trame UBX à chaque allumage du M10Q, passé en `[DETAIL]` puis **remis en WARN le 28/10** : sa pause d'écriture flash espace les erreurs de la synchro du 1er allumage, sans elle la synchro échoue (2.12). L'ERROR `UBXCommsEventError: type=0c` reste lui aussi, pour la même raison |
| Carte : harnais | une vague courte de `gnss_bench.py` ou `dte_campaign.py --suite sans-radio` : aucun cas cassé par un texte | ✅ 29/10, `--suite sans-radio` complète sur `2d8194fa` : 80 ok, 5 sautés à bon droit (pas de pont KIM2 sur SMD, électrode à mouiller, RCONF maîtresse non provisionnée, 2 cas `--validation`), 3 échecs et 3 erreurs tous pendant des chutes du lien USB ; les 6 rejoués : 6 ok. Les deux cas `--validation` (EDG-G2, EDG-G3) joués ensuite sur une image `--validation` : ok. Aucun cas cassé par un texte, aucune émission |
| Carte : émission | une émission avec antenne ou charge (accord requis) : une ligne `TX SUCCESS` en flash, les lignes KIM2/SMD en `[DETAIL]` | ⬜ |
| Carte : `DBP02=2` | `PARMW DBP02=2`, sortie de configuration, même cycle : console inchangée, `DUMPD` ne contient que l'enregistrement de démarrage et les WARN/ERROR ; retour à `DBP02=3` ensuite | ✅ 27/10 : 6 entrées sur le cycle — le geste aimant et l'entrée en PreOperational (écrits avant que le niveau s'applique), puis par session le WARN de trame UBX (gardé, voir 2.12) et l'ERROR ANO (corrigé depuis, ligne ci-dessus). Console inchangée (15 `[INFO]`, 94 `[DETAIL]`). Retour à `DBP02=3` fait |
| Carte : mise à jour | un firmware antérieur flashé puis celui-ci, magasin conservé : `DBP02` lu à 3, le reste de la configuration intact | ✅ 29/10 : la release v4.2.0 (`ccf65628`) flashée, cinq valeurs posées (ARP05, GNP23, LBP02, MRP06 et PPP12, dernier emplacement de sa table), puis `2d8194fa` : les cinq intactes, `DBP02` à 3, les 147 clés de la v4.2.0 relues identiques |

### 2.5 Point 3 : l'attente du capteur d'eau salée — décision : réglages inchangés

**Constat.** Chaque lecture SWS (une par seconde sous l'eau) attend en attente active
que le condensateur se charge à travers l'eau. L'attente part de `UNP08` (1000 µs) et
s'ajuste entre `UNP09` (200 µs) et `UNP10` (10 000 µs). D'après le code et la
simulation, elle monte jusqu'au plafond sur des électrodes propres : environ 750 s de
processeur bloqué et 0,7 mAh par jour. Le terrain (WWF-YanaRiba, 3 à 4 mois sur 7
balises) a fonctionné avec des bornes de 200 à 1000 µs posées par la configuration ;
les modèles tortue actuels n'en posent aucune.

**Décision de l'utilisateur (27/10).** Valeurs par défaut et modèles inchangés : ce sont
des paramètres, modifiables par déploiement (`PARMW UNP10=1000` pour revenir aux bornes du
terrain). Seul le commentaire de `adjust_sample_delay()`, qui décrivait l'inverse du code,
est corrigé. Aucun changement de comportement, donc rien à vérifier sur carte pour ce
point.

### 2.6 Point 4 : la chauffe du TCXO du SMD, écrite seulement quand il le faut

**Constat.** Le module SMD garde la chauffe en RAM (`argos-smd-at-kineis-firmware`,
`mcu_misc.c` : 2000 ms à chaque démarrage) et l'attend, bloqué, avant **chaque** émission.
Le pilote la réécrivait aussi à la fin de la première émission de chaque mise sous
tension (50 ms d'attente active, jusqu'à trois essais, puis une commande), juste avant de
couper le rail : environ 0,3 s perdue par émission. Le KIM2 n'est pas concerné (il gère sa
chauffe lui-même).

**Ce qui a changé** (règle demandée par l'utilisateur) : la chauffe n'est écrite que si le
module a redémarré — mise sous tension, reset, sortie de veille profonde, erreur — ou si
la valeur voulue change. Le pilote retient ce que le module détient
(`smd_tcxo_register.hpp`) ; l'écriture de fin d'émission est supprimée. La première
émission d'une remontée part toujours à 0 s (sauf puce sous 5 °C), les suivantes à ARP35.

| Vérification | Comment | Fait |
|---|---|---|
| Tests PC : la règle | `SmdTcxoRegister` (4 tests : inconnu au départ, valeur détenue, oubli au redémarrage, remontée complète) | ✅ 27/10 |
| Compilation | matrice 8 builds, bancs SMD UART et RSPB : 0 avertissement | ✅ 27/10 |
| Carte SMD UART : remontée simulée **avec émission** (accord, antenne ou charge, alimentation ≥ 1 A) | `%DIVE` puis `%SURFACE`, 3 émissions : 1re `SmdSat: TCXO=0 ms`, suivantes `TCXO=2000 ms` à chaque mise sous tension, plus aucune écriture après l'émission, `TX SUCCESS` à chaque fois | ⬜ |
| Carte SMD SPI (firmware SPI du module) | même essai | ⬜ |

### 2.7 Point 5 : la remontée de VSYS au réveil, seulement si VSYS était bas

**Constat.** À chaque réveil de la boucle principale, `restore_power_rails()` remettait
VSYS à 3,3 V et attendait 2 ms, même quand `reduce_power_rails()` ne l'avait pas abaissé :
c'est le cas tant que le rail GNSS est allumé (veille profonde, never-poweroff), que la LED
est allumée ou que la flash travaille. Une fois par seconde sous l'eau, environ 170 s
d'attente active par jour.

**Ce qui a changé.** VSYS n'est remonté (avec ses 2 ms) que s'il est bas, d'après le verrou
de sortie de la broche. Le seuil de brown-out (POF), abaissé à 2,0 V pour le rail de veille,
est remonté à 2,7 V seulement s'il avait été abaissé — y compris quand
`ensure_vsys_nominal()` avait déjà remonté le rail. Comportement électrique identique.
Commande de banc `%RAILS` (BENCH_TEST) : restaurations « settled » et « skipped ».

| Vérification | Comment | Fait |
|---|---|---|
| Compilation | matrice 8 builds : 0 avertissement | ✅ 27/10 |
| Carte : rail GNSS allumé | `%RAILS` deux fois à quelques minutes : « skipped » augmente, « settled » ne bouge pas | ⚠️ **non mesurable sur le build de banc** : `%RAILS` reste à `settled=0 skipped=0` rail allumé comme coupé, car la console de banc scrute l'USB toutes les 50 ms et la boucle ne s'endort jamais assez pour abaisser les rails. Le chemin n'existe que hors banc |
| Carte : rail GNSS coupé | `GNP52=30`, attendre la coupure : « settled » augmente, le M10Q répond au rallumage, aucun POFWARN | ✅ 27/10 sur le **build de debug** (sans console de banc, donc sans `%RAILS`) : `GNP52=30`, `GNP23=90`, 5 min de console, 4 sessions GNSS avec coupure puis rallumage du rail, le M10Q répond à chaque fois (`boot sync`), aucun POF, aucun WARN ni ERROR. Les compteurs eux-mêmes ne sont pas observés |

**Au passage :** le build de banc RSPB ne compilait plus depuis fin août (`EXT_I2C_BUS` et
la sonde UART du port satellite, propres au LinkIt). Les deux sont protégés ; il compile,
0 avertissement.

### 2.8 Point 6 : `config.dat` — sauvegarde toutes les 30 min gardée, relecture par clé

**Constat.** Une sauvegarde de la configuration réécrit tout `config.dat` (25 à 30 Ko) :
≈ 403 ms en médiane sur la carte du banc, ≈ 900 ms au pire. La sauvegarde toutes les
30 min date du 8 avril 2026 (6e5af3e9) : elle a remplacé une sauvegarde après **chaque**
émission et chaque réception, les compteurs restant en RAM entre deux. Depuis mai elle
écrit aussi `LAST_KNOWN_RTC`, seule source de l'heure après un reset sur LinkIt (sans
elle, une balise scellée sans fix repart en 1970). Environ 20 s de flash par jour, de
l'ordre de 0,1 mAh/j (estimé, non mesuré au PPK).

**Décision de l'utilisateur (27/10).** Rythme de 30 min inchangé.

**Défaut trouvé en mesurant, corrigé.** La relecture prenait les enregistrements dans
l'ordre, un par paramètre, et abandonnait le paramètre dès que la clé ne correspondait
pas, en consommant quand même l'enregistrement. Un enregistrement en trop ou en moins
décalait donc tous les suivants : remis à leur défaut, puis sauvegardés ainsi, sans
trace dans le journal (le boot n'a pas encore de journal à ce moment). Cas réels :
- passage caméra → sans caméra sur la carte du banc : 71 paramètres perdus à partir du
  slot 172, dont IDP13, IDP14 et `LAST_KNOWN_RTC` (horloge revenue en 1970) ;
- mise à jour v4.1.9 → v4.1.10 : 17 paramètres (slots 226 à 242 ; l'incident ARP36/37
  décrit au-dessus de la constante de version) ;
- builds de dev LinkIt du 3 juillet au 29 août 2026, mis à jour en v4.2.x : 60 à 76
  paramètres à partir du slot 176, dont toutes les clés LoRaWAN sur un build LoRa.

La mise à jour v4.2.0 → v4.2.1 n'était pas touchée (simulée sur 5 variantes).

**Ce qui a changé** (`config_store_fs.hpp`). Chaque enregistrement va au paramètre qui
porte sa clé. Un enregistrement d'un paramètre absent du build est ignoré. Un paramètre
sans enregistrement prend son défaut, comme avant. Un fichier au format du build, ce
qui est le cas de tous les boots sauf le premier après un changement, se lit avec les
mêmes lectures qu'avant. Inchangés : le format, l'écriture, et le chemin « changement
de version » (tout à défaut sauf identifiants et credentials). Le commentaire de la
constante de version est complété : un slot retiré ou déplacé ne demande plus de bump.
Deux limites y sont notées : un retour à un firmware antérieur relit toujours par
position, et un slot qui change de sens demande une nouvelle clé.

| Vérification | Comment | Fait |
|---|---|---|
| Tests PC : le défaut | `ConfigStore.RecordsOfSlotsThisBuildLacksShiftNothing` (PWP05/LBP14 en trop), `MissingRecordsResetOnlyTheirOwnSlots` (fichier sans caméra lu avec caméra), `ARecordWithAKeyThisBuildDoesNotKnowIsSkipped` : **échouent sur l'ancien code** (IDP14 vide, `LAST_KNOWN_RTC` à 0), passent avec le correctif | ✅ 27/10 |
| Tests PC : ordre, pas de réécriture | `RecordsOutOfOrderAreAllPlaced` ; `AFileInThisBuildsLayoutIsNotRewrittenAtBoot` (un fichier au format du build n'est pas réécrit au boot) | ✅ 27/10 |
| Vitesse | `init()` sur un fichier au format du build, ancien et nouveau binaire lancés en alternance, 18 mesures chacun (PC) : médiane 211,8 µs avant, 211,6 µs après | ✅ 27/10 |
| Suites et contrôles | `-p -xg SWS` 1048/1048, Cyprus 1066/1066, SWSAnalog 62/62, SWSAnalogFlash 27/27, TurtleSimulation 11/11, RSPBSimulation 10/10 ; `check_log_lines`, `check_bench_log_patterns`, `check_param_tables`, clang-format : OK | ✅ 27/10 |
| Compilation | matrice 8 variantes : OK, +330 à +420 octets de flash | ✅ 27/10 |
| Carte : changement de variante | build de banc SMD UART sans caméra → avec caméra → sans caméra, `PARMR` complet à chaque étape, repères posés après le slot 172 (GNP52, DBP02, RLP02, HMP01). Les 161 clés communes restent identiques à chaque passage, sauf PWP06 qui avance avec l'heure (+15 s au total). Les 6 clés caméra apparaissent à leur défaut, puis disparaissent. Aucune émission (ARP01=0, IDP13/IDP14 vides) | ✅ 27/10 |
| Carte : mise à jour depuis la v4.2.0 | flasher la release v4.2.0, poser des valeurs, puis ce firmware : configuration intacte | ✅ 29/10 : voir 2.4, mise à jour (147 clés identiques, LinkIt SMD) |
| RSPB, KIM2 | même essai sur ces cartes (code commun, sans partie propre à la carte) | ⬜ |

### 2.9 Point 7 : plus d'écriture flash entre l'émersion et le premier ping, DBD à la plongée

**Constat.** À chaque vraie remontée, deux écritures LittleFS passaient **avant** que
l'émersion soit même publiée :
1. `SWS.CAL`, réécrit dans l'échantillon qui détecte la surface (au plus une fois par
   minute, donc à presque chaque vraie remontée) ;
2. la ligne de journal `UWDetectorService: state changed`.

Elles passaient avant l'ouverture de la fenêtre de rétention d'ArgosTxService, qui ne
s'ouvre qu'au moment où la diffusion de l'événement atteint ce service. Or cette fenêtre
a été créée précisément parce que ces écritures coûtaient 0,6 à 0,9 s sur un trajet
surface → antenne de 1,7 s. Le banc ne les voyait pas : `%SURFACE` et `%DIVE` publient
l'événement eux-mêmes et sautent le détecteur.

Par ailleurs, la DBD (`gnss_dbd.dat`) était écrite 15 s après chaque fix, en pleine
rafale d'émersion. Son écriture bloque l'ordonnanceur : 0,5 à 5,5 s mesurés avant le
report de 15 s, jamais remesurés depuis.

**Décisions de l'utilisateur (28/10) :**
- accélérer la détection de surface et les opérations qui suivent, sans rien casser ;
- DBD : sur LinkIt, écriture à la plongée suivante avec un délai maximal ; sur RSPB,
  délai maximal ou extinction de la carte ;
- pas de test de trempage pour l'instant.

**Ce qui a changé :**
- **`SWS.CAL`** (`sws_analog_*.cpp`). La transition ne fait qu'armer la sauvegarde.
  Celle-ci est écrite dans un échantillon ultérieur, **sous l'eau**, qui ne change pas
  l'état, au moins 30 s après le dernier changement. L'anti-rebond de 60 s est gardé.
  L'arrêt du service écrit une sauvegarde encore armée.
  - Une sauvegarde armée en surface attend la plongée suivante. Un reset logiciel ne perd
    rien (`m_calib` est en `.noinit`). Une coupure d'alimentation ne perd que ce que la
    surface a appris depuis la dernière plongée.
- **Ligne « state changed »** (`uwdetector_service.cpp`). Elle est écrite **après** la
  diffusion de l'événement, avec le même texte et sans changement sur la console.
  - À l'émersion en SURFACING_BURST, elle tombe dans la fenêtre de rétention et part en
    flash une fois le ping parti.
  - À la plongée, ArgosTxService réagit avant que la flash soit touchée.
- **DBD** (`dbd_save_schedule.hpp`, pilote M10Q, GPSService) :
  - jamais moins de 15 s après le fix (comme avant) ;
  - au plus tard 5 min après le **premier** fix non sauvegardé : un fix suivant ne
    repousse pas l'échéance ;
  - écrite 30 s après une plongée si la balise est toujours sous l'eau ;
  - écrite comme avant à l'extinction, en OFF, en configuration, avant un reset par chien
    de garde et avant la réutilisation du tampon.

  Sur RSPB, qui ne plonge pas, cela donne : l'échéance, ou l'extinction en fin de cycle
  TPL, après les émissions.
- **Banc (`BENCH_TEST` seulement)** :
  - `%SWSRAW <0..16383>|OFF` injecte la lecture ADC du capteur SWS et fait passer de
    vraies transitions par tout le chemin de détection ;
  - la ligne console `[DETAIL] UWDetectorService: bench publish state=<s> after <ms> ms`
    mesure le délai entre le début de l'échantillon et la publication de l'événement.

| Vérification | Comment | Fait |
|---|---|---|
| Tests PC : émersion | `SWSAnalogFlash.CalibrationSaveWaitsForAQuietUnderwaterSample`, `ArmedCalibrationSaveIsCommittedOnStop`, `StateChangeLineFollowsTheBroadcast` : **échouent tous les trois sur l'ancien code** (SWS.CAL écrit dans l'échantillon d'émersion à t = 91 s ; ligne déjà écrite au moment de la diffusion), passent avec le correctif | ✅ 28/10 |
| Tests PC : DBD | `DbdSaveSchedule` (4 tests : échéance au délai maximal, un fix suivant ne la repousse pas, jamais avant 15 s, nouvelle fenêtre après écriture) ; `GPSService.ADiveLetsTheDriverPersistAfter30s`, `AShortDiveAsksTheDriverNothing` | ✅ 28/10 |
| Suites et contrôles | `-p -xg SWS` 1054/1054, Cyprus 1072/1072, SWSAnalog 65/65, SWSAnalogFlash 30/30, TurtleSimulation 11/11, RSPBSimulation 10/10 ; `check_log_lines`, `check_bench_log_patterns`, `check_param_tables` OK ; clang-format : aucune violation nouvelle | ✅ 28/10 |
| Compilation | matrice 8 variantes : toutes compilent, 0 avertissement ; +550 octets de flash environ (+950 sur le build de banc, outils `%SWSRAW` compris) | ✅ 28/10 |
| Carte : délai avant publication | build de banc SMD UART avant / après correctif, ARP01=0 (aucune émission), UNP01=1 le temps de la mesure puis remis à 0, 5 cycles `%SWSRAW` 151 / 51 (calibration air 52, eau 150, seuil 91 ; plongée 75 s) : délai du début de l'échantillon à la publication. **Avant : plongée 165-192 ms, émersion 185-785 ms (médiane 675 ms ; ≈ 700 ms quand SWS.CAL est écrit dans l'échantillon, 3 cycles sur 5, ≈ 190 ms sinon), ligne « state changed » écrite avant la publication. Après : 1 ms dans les deux sens, 5 cycles sur 5, ligne écrite après la publication.** Une ligne de journal coûte ≈ 180 ms sur cette carte, une écriture de SWS.CAL ≈ 500 ms | ✅ 28/10 |
| Carte : trempage réel | électrodes dans l'eau salée puis à l'air, même relevé | ⬜ (impossible pour l'instant, dit l'utilisateur) |
| Carte : DBD, **vue du ciel** | protocole ci-dessous (a à d) | ⬜ |
| RSPB : DBD | un cycle TPL avec fix : la ligne `DBD written` apparaît avant l'extinction de fin de cycle, après les émissions | ⬜ |

**Protocole DBD (campagne finale, vue du ciel).** L'écriture se lit sur la console de banc :
`[DETAIL] M10QAsyncReceiver: DBD written, <n> bytes, <m> s after its session (t=…)`, où
`<m>` est compté depuis la dernière session qui a rapporté une DBD. Ce n'est pas une ligne
de journal flash. Préconditions :
- build de banc ;
- antenne GNSS en vue du ciel ;
- ARP01=0 (aucune émission) et GNP01=1 ;
- GNP52=4294967295 (valeur actuelle du banc : rail GNSS jamais coupé, BBR gardée, donc pas
  d'écriture au démarrage de la session suivante).

| Cas | Réglage | Attendu |
|---|---|---|
| a) fix puis plongée | UNP01=1, ARP11=10 (24 h, aucune autre session) ; 1 min après la ligne `fix in N s`, `%SWSRAW 151` | `DBD written` 30 s après `bench publish state=1`, jamais avant ; puis `%SWSRAW OFF` |
| b) fix sans plongée | UNP01=0, ARP11=10 | `DBD written` à m ≈ 300, soit 5 min après la session |
| c) deux fix en moins de 5 min | UNP01=0, ARP11=12 (2 min) | une seule écriture, **5 min après la première session** (heure console), pas 5 min après la dernière |
| d) fix puis configuration | `%CFG` avant les 5 min | `DBD written` à l'entrée en configuration ; après une coupure d'alimentation, la 1re session lit `load_dbd_from_flash: <n> bytes, age …` |

Remettre ensuite ARP11, UNP01 et GNP52 aux valeurs de la section 0.

**Incident de banc du 28/10.** Le premier `nrfjprog --recover` de la mesure avant / après
n'a pas abouti dans son délai de 120 s. Ensuite :
- tout accès mémoire SWD se figeait (`--recover`, `--program`, `--memrd`) ;
- la sonde répondait, la tension cible était de 3,28 V et le port AHB était visible ;
- seul `--pinreset` passait.

Une coupure d'alimentation de la carte l'a débloquée (APPROTECT réarmé, puis séquence de
flash normale). Piège de mesure au passage : `UNP01` (UNDERWATER_EN) valait 0 sur la carte
du banc ; sans lui, le service SWS s'initialise mais n'échantillonne jamais. La carte
tourne ensuite sur le build de banc corrigé, avec UNP01=0 et ARP01=0 remis.

### 2.10 Point 8 : la configuration reconstruite à chaque appel — décision : rien à changer

**Constat.** Sur une journée simulée de tortue, `get_argos_configuration()` est appelée
7017 fois (84 par remontée) et `get_gnss_configuration()` 524 fois. Chaque appel fait
environ 45 lectures de paramètres en RAM et 4 copies de chaînes sur le tas (charge utile
de certification, 3 RCONF) : de l'ordre de 0,5 s de processeur par jour (estimation),
quelque 28 000 allocations aussitôt libérées.

La fonction a des effets de bord : évaluation des modes « hauled » et « moored », et
journal des changements de mode. La mettre en cache changerait le moment où ils se
produisent.

Le prépass : les modèles `drifter` et `fix_beacon` sont en PASS_PREDICTION, mode qui saute
le bloc de prépass « orthogonal ». Leur recherche de passage est la planification elle-même,
une fois par émission prévue (de l'ordre de 10 à 20 s de processeur par jour, avec le
chiffre de l'audit, non remesuré).

**Décision de l'utilisateur (28/10).** Rien à changer, aucun gain qui le justifie.

### 2.11 Point 9 : le reed et l'accéléromètre en entrée GPIOTE « haute précision » — décision : pas de changement

**Constat.** Sur les deux cartes, le reed (`GPIO_REED_SW`, TOGGLE) et l'interruption de
l'accéléromètre (`GPIO_ACC_INT1` / `GPIO_INT1_AG`, LOTOHI) sont des entrées GPIOTE en mode
IN (`hi_accuracy = true`). Selon les sources Nordic, ce mode coûte en veille, tant qu'un canal
est ouvert, de l'ordre de 20 µA (erratum) à 250 µA (horloge rapide maintenue). Le mode PORT
(SENSE) coûte environ 1 µA. Le chiffre réel de notre carte reste à mesurer.

**Deux pièges avant tout passage en mode PORT :**
- `NRFX_GPIOTE_CONFIG_NUM_OF_LOW_POWER_EVENTS` (et son pendant `GPIOTE_...`) vaut **1**
  dans les deux `sdk_config.h`. Avec deux broches en PORT, la seconde initialisation échoue :
  le reed ou l'accéléromètre devient inactif, une brique pour une balise scellée. Il faut
  le relever à 2 au moins.
- Le mode PORT repose sur le signal DETECT, commun à toutes les broches : une broche non
  gérée qui garderait une SENSE armée (le bootloader en arme une sur P0.11, son bouton de
  DFU) peut bloquer les événements suivants. P0.11 est déclarée en sortie par l'application
  (GPIO_DEBUG, TX de l'UART de debug) : à vérifier sur carte (`PIN_CNF`).

Le pilote nrfx du SDK 17.0.2 traite le mode PORT avec le registre LATCH du nRF52840 : un
front bref ou arrivé avant le traitement n'est pas perdu.

**Relevé sur la carte du banc (28/10, build de banc, USB, sans aimant)** :
- `%SENSE` : aucune broche armée en SENSE en fonctionnement (la SENSE de P0.11 laissée par
  le bootloader est bien effacée par l'application) ;
- `%GPIOTE` : canal 0 = P1.02 (accéléromètre) alloué mais désactivé ; canal 1 = P1.03
  (reed) alternativement activé et désactivé ;
- **l'interruption du reed se déclenche 3,64 fois par seconde sans aimant**, en
  Operational comme en configuration. La broche relit 0 à chaque fin d'anti-rebond, donc
  aucun faux geste. Le délai entre la réactivation et le front suivant vaut 26 ms en
  moyenne (0 à 538 ms, 248 cycles) : ce sont des fronts brefs sur la ligne, pas un
  artefact de réactivation.

Conséquences sur cette carte :
- l'anti-rebond (250 ms) garde le canal du reed désactivé environ 92 % du temps. Le coût
  statique du mode IN y est donc presque absent, et une mesure A/B au PPK n'y aurait pas
  de sens ;
- le processeur se réveille environ 7 fois par seconde pour rien.

La cause est inconnue : étage Q4 du reed (oscillation connue quand VBAT descend sous
VSYS − |V_GS(th)|, voir la mémoire du 01/09 ; mais VBAT lit 4,28 V ici), défaut de cette
carte, ou alimentation par l'USB.

**Décision de l'utilisateur (28/10).** Pas de changement : le reed ne sert qu'avant et
après une mission. Nuance consignée : c'est son armement qui coûte, pas son usage. Il
écoute toute la mission, et UW_DIVE_MODE_ENABLE, qui le mettrait en pause sous l'eau, est
désactivé par défaut et dans tous les modèles.

Un commentaire explique comment changer plus tard sans piège :
- au-dessus de `GPIO_REED_SW` dans les deux `bsp.cpp` ;
- au-dessus des deux plafonds `*_NUM_OF_LOW_POWER_EVENTS` des deux `sdk_config.h` ;
- dans `nrf_switch.cpp`, pour les fronts parasites mesurés.

Les essais ci-dessous restent utiles avant tout changement futur, mais ne bloquent pas le
merge.

| Vérification | Comment | Fait |
|---|---|---|
| Fronts du reed sur une carte saine | même build de banc, `%GPIOTE` : `reed_edges` doit rester à 0 sans aimant. Sur une LinkIt KIM : antenne ou charge, car un KIM2 qui répond émet | ⬜ |
| Mesure du coût | sur une carte saine sans fronts parasites : `%GPIOTEAB <période> <bascules>` (build de banc) alterne tous les canaux GPIOTE IN entre événement et désactivé ; PPK seule alimentation, USB débranché ; écart moyen entre fenêtres | ⬜ |
| Ligne du reed à l'oscilloscope | P1.03 sur la carte du banc, sans aimant : forme et fréquence des fronts | ⬜ |

### 2.12 Petits points du 28 octobre

| Point | Ce qui a changé | Vérifié |
|---|---|---|
| `Is25Flash` | `FlashInterface(block_size, blocks, page_size)` recevait le nombre de blocs et la taille de bloc inversés. Sans effet aujourd'hui, les deux valent 4096 ; il en aurait avec une autre puce. | ✅ 28/10, compile sur les 8 variantes |
| Pile de profondeur | le délai de remplissage complète **chaque** capteur manquant, plus seulement le premier (2.4) | ✅ 28/10, `DepthPileManagerTimeoutFillsEveryMissingSensor` : échoue sur l'ancien code, passe |
| `--recover` | `CLAUDE.md` et `PLAN_VALIDATION` §0.4 disaient qu'il efface la configuration : corrigés (section 0) | ✅ 28/10 |
| Lignes UBX du 1er allumage | essai de passer l'ERROR `type=0c` en `[DETAIL]` : **annulé**, et le WARN de trame du point 2 remis. Les deux sont identiques à HEAD, avec un commentaire qui interdit de les rétrograder sans pause explicite (ci-dessous) | ✅ 28/10, carte |
| Cliquet de complexité | le commit était refusé : 17 fonctions plus complexes, et `bench_cam` (41) au-dessus du plafond de 25. Décision de l'utilisateur : découper les six gros écarts, épingler les douze petits. Découpées sans changement de comportement : la console de banc (sondes de l'audit dans leur propre groupe, une fonction par commande ; `bench_cam` par sous-commande), `deserialize_config` (15 → 7) et `UWDetectorService::service_initiate` (16 → 15). Épinglées à la main, chacune au prix d'un ajout du lot : `GenTracker::react`, `handle_dte_message`, `process_received_data`, `process_usb_data`, `PMU::powerdown`, `reduce_power_rails`, `init_sensors`, `GPSService::notify_peer_event`, `detector_state`, `ConsoleLog::write`, `log_level_str`, `DepthPileManager::notify_peer_event` (+1 à +3). Puis `--tighten` | ✅ 28/10 : suites PC ; 12 variantes, dont les builds de banc caméra et RSPB, 0 avertissement ; carte : chaque commande de banc déplacée répond, relecture complète de la configuration identique à celle d'avant le flash (161 clés), une plongée puis une émersion par `%SWSRAW` (publication en 1 ms, ligne « state changed » avec la durée de la plongée), `%DIVE` adopté sans seconde diffusion. `bench_cam` n'est que compilé : la carte du banc n'a pas de caméra en UART |

**La synchro du 1er allumage du M10Q dépend des écritures du journal.** Après un
démarrage de la carte, le pilote sonde 460800, 38400, 921600, puis 9600 bauds. Sans pile
de sauvegarde, le module répond à 9600, son débit d'usine. Aux autres débits, l'UART
relève des erreurs : `0x0c` (trame et break) ou `0x04` (trame). Pendant le démarrage, le
pilote en tolère dix par débit et relance la réception après chacune. **Le seuil compte
les erreurs, il ne mesure pas le temps.** Ce qui les espace, c'est la ligne de journal de
chaque erreur : environ 180 ms d'écriture flash, exécutée avant la relance.

| Firmware (carte du banc, redémarrages par la broche reset) | Synchro au 1er allumage |
|---|---|
| HEAD `1ac5d894`, build de banc | 7 sur 7 |
| WARN de trame en `[DETAIL]` (état du point 2 au 27/10) | 4 sur 5. L'échec : des rafales de dix erreurs en moins d'une milliseconde, les quatre débits épuisés en ~1,5 s, « failed to sync comms » |
| WARN de trame et ERROR `0x0c` en `[DETAIL]` (petit point essayé) | 1 sur 5 |
| Les deux lignes comme sur HEAD (arbre final) | 17 sur 19 (5 sur 7, puis 12 sur 12) |

L'écart restant avec HEAD n'est pas significatif sur ces effectifs. La comparaison est de
plus biaisée en faveur de HEAD : sur cette carte, le reed rebondit en continu (2.11), et
HEAD écrit chaque rebond en INFO, 36 à 58 lignes de flash par capture de 70 s. Ce sont
autant de pauses au hasard, qu'une carte saine n'a pas ; l'arbre final affiche ces
rebonds en `[DETAIL]`.

- **Proposé, non traité** (point à part, à mesurer sur carte saine) : rendre cette
  tolérance indépendante du journal, par une pause explicite avant `restart_rx()` ou un
  seuil en temps. Les 4 à 9 lignes UBX de chaque 1er allumage pourraient alors quitter
  le journal.
- Sur RSPB, chaque cycle TPL redémarre la carte : chaque session commence par cette
  synchro. À vérifier sur un cycle TPL.

| Vérification | Comment | Fait |
|---|---|---|
| Synchro sur carte saine | build de banc, ≥ 10 redémarrages par la broche reset, compter « boot sync at » contre « failed to sync comms » à la 1re session ; HEAD puis arbre final sur la même carte | ⬜ |
| Synchro au réveil TPL | carte RSPB, plusieurs cycles TPL : la 1re session de chaque cycle se synchronise | ⬜ |

### 2.13 Récapitulatif : ce qui reste à faire, par condition

Toutes les cases ⬜ de la section 2, regroupées par ce qu'il faut réunir pour les faire.
Chaque ligne renvoie à sa section, qui donne le protocole et le résultat attendu.

| Condition | Essais |
|---|---|
| **Vue du ciel** (vrai fix GNSS) | 2.9 DBD, cas a à d |
| **Émission radio** (accord explicite, antenne ou charge 50 Ω, alimentation ≥ 1 A) | 2.4 une émission (`TX SUCCESS` en flash, lignes pilote en `[DETAIL]`) ; 2.6 trois émissions SMD UART (chauffe TCXO 0 puis 2000 ms) |
| **Module SMD en firmware SPI** | 2.2 session SMD complète sans émission ; 2.3 caméra avec le SMD en SPI ; 2.6 trois émissions SMD SPI (accord RF) |
| **Eau salée** | 2.9 trempage réel des électrodes (même relevé que `%SWSRAW`) |
| **Carte KIM2** | 2.2 réveil par `SAT_WKUP` en mode BLIND et émission (accord RF) ; 2.8 changement de variante |
| **Carte RSPB** | 2.1 jauge sur un cycle TPL ; 2.2 GNSS et SMD sur un cycle TPL ; 2.8 changement de variante ; 2.9 DBD à l'extinction de fin de cycle ; 2.12 synchro GNSS au réveil TPL ; 2.15 P1, P3 (coupures), P4 (`FAULT_INJECT` 1 à 4, PPK pour la coupure), P5 (échecs d'émission, reset de récupération), P6 (jauge), P7 (mortalité), P8 (extinction au PPK, lignes du journal) |
| **Carte du banc, builds de validation** | 2.15 P4 : `FAULT_INJECT=2`, `=3`, `=4` sur la LinkIt SMD, puis reflash du build normal ✅ faits le 29/10 (FI=2, 3 et 4 en Release, lus par SWD) |
| **Carte saine, PPK seule alimentation** | 2.2 courant absolu de veille GNSS |
| **Carte saine** (reed sans rebonds) | 2.11 fronts du reed et mesure du coût ; 2.12 synchro du 1er allumage, HEAD puis arbre final |
| **Oscilloscope** | 2.11 ligne du reed (P1.03) sur la carte du banc, sans aimant |
| **Ancien firmware** | 2.4 mise à jour depuis un firmware sans DBP02 (lu à 3) ; 2.8 mise à jour depuis la release v4.2.0 (configuration intacte) |
| **Caméra** | 2.3 build `CAM_CONTROL=BUTTON`, `CAP06=AUTO`, alimentation par `CAM_PWR_EN` |
| **Sans condition** (carte du banc) | 2.1 valeurs batterie, `STATR` répété, `SENSR`, seuil `LBP02`, 1re mesure au démarrage ; 2.4 une vague courte de `gnss_bench.py` ou `dte_campaign.py --suite sans-radio` ✅ faits le 29/10 (le drapeau batterie faible demande une alimentation réglable) |
| **Carte KIM2** (suite) | 2.14 identité lue au démarrage, puis 1re session ; 2.15 P1 (compteur d'échecs après 60 s) |
| **Matériel CLS** (woodbuoy) | 2.14 trame woodbuoy reçue et décodée, baromètre et alimentation, buzzer |

### 2.14 Fusion de `dev/cls_102026` (CLS, 28/10)

**Ce que c'est.** 12 commits de Susanna Polino (CLS) du 9/09 au 21/10, partis du
1/09 (`ccf65628`), pendant que cette branche en faisait 206. Ils servent surtout un
produit CLS, le **woodbuoy** : une bouée KIM2 avec buzzer, piles alcalines,
température de mer et un baromètre Vaisala PTB110 lu par un ADS1115, sans capteur
d'eau salée. Fusion faite le 28/10, chaque commit relu contre l'état de cette branche.

| Commit CLS | Décision | Pourquoi |
|---|---|---|
| `4f157cf7` script KIM | pris | Caméra à 0 par défaut : déjà fait ici. Mise en forme de la commande cmake : prise, avec nos options (`DEBUG_LEVEL`, `BENCH_ALLOW_POWERDOWN`). **`ENABLE_SWS_ANALOG` à 0 par défaut : pris** (décision de l'utilisateur, 28/10 : la tortue n'est pas le seul produit KIM2). Une tortue KIM se construit avec `ENABLE_SWS_ANALOG=1 ./scripts/build_linkitv4_kim.sh` ; plan de validation, harnais de banc KIM et README mis à jour, la matrice CI passe déjà la valeur |
| `d8348a47`, `4484d227`, `650170e2` script, trame et baromètre woodbuoy | pris, corrigés | Voir ci-dessous. Les paramètres BAP01-BAP05 passent de 266-270 à **283-287** : ces emplacements étaient pris ici par UNP26..DBP02. La relecture de `config.dat` par clé (2.8) les replace sur une balise CLS déjà configurée |
| `832b9a0f` compilation de la température de mer | pris | |
| `4579d432` température de mer × 1000 | déjà fait | Même formule que `0b7ac2ee` (10/09), qui garde en plus la division par 10 côté LoRa |
| `c01c1ccf` lignes de journal en TRACE | **refusé, remplacé** | Même but que le point 2 (2.4), mais TRACE est retiré du binaire au niveau 3 : ces lignes disparaissaient aussi de la console de banc, et `M10Q on`, `boot sync at`, `next GNSS acquisition`, `TX SUCCESS` sont des lignes d'étape que le banc et `log_harvest.py` lisent. Les lignes qu'ils étaient seuls à toucher passent en DETAIL |
| `2b9279c4` cause de reset « pseudo power-on » | déjà fait | Couvert ici, avec des marqueurs explicites dans GPREGRET (0x80, 0x81) |
| `98fafb92` buzzer et geste aimant | pris | Carillon au démarrage, phases sonores du geste d'extinction, coupure de sécurité à 30 s. Builds `BUZZER_ENABLE` seulement |
| `4caeb8e4` TX du KIM2 à l'extinction | **refusé, remplacé** | Même fuite que `34ebb4a1` (19/09, PPK : 70 → 27 µA). Leur `park_tx_for_power_off()` doublait le nôtre sous le même nom (haute impédance, instance UART 1 en dur) : un seul mécanisme gardé. Pris : le désabonnement du module éteint |
| `f0136b9c` identité du KIM2 au démarrage | pris, corrigé | ID, ADDR et version lus à la construction (≈ 550 ms, **aucune émission**). `sync_power_up()` et `sync_power_down()` factorisent quatre copies ; la coupure passe par `cut_module_rail()`, TX tirée bas d'abord, sans quoi la fuite de 43 µA revenait |
| `c93ce7fd` batterie lue seulement à la demande | **remplacé, garantie gardée** | Leur version ne mesurait plus dans les accesseurs de configuration ni dans l'entrée GPS d'un fix (elle reprend la mesure du début de session). Restait une mesure à chaque début de session GNSS, entrée sans fix, dégradée ou CloudLocate, émission Doppler, et contrôle critique en mode batterie basse. Ici, la règle de 2.1 (mesure réutilisée au plus 30 min) partout, plus l'option `BATTERY_SAMPLE_EACH_GNSS_SESSION=1`, posée par `build_woodbuoy.sh` : le service GNSS mesure alors aux mêmes points que leur firmware, vérifié dans le binaire woodbuoy (quatre appels à la mesure fraîche, le fix à la mesure réutilisable). Pas sur la tortue : 84 sessions par jour simulé feraient plus que doubler ses mesures. Différences restantes (émission Doppler et contrôle critique sur une mesure de moins de 30 min, verdict critique confirmé de `c3ba3cac`) : `NOTES_FUSION_CLS_102026.md` §2.8 |

**Corrigé dans leur code en fusionnant** (rien de cela ne compile hors woodbuoy) :
- trame woodbuoy : le champ baromètre déréférençait un pointeur nul dans une trame
  « température seule ». Leur binaire, compilé ici et désassemblé, lit alors 8 octets à
  l'adresse 0 (MBR), un double dénormalisé converti en 0 : les bits transmis sont les
  mêmes, la correction retire le comportement indéfini ;
- encodage `(hPa − 800) × 100` borné aux 15 bits. Hors plage, il était indéfini sous
  800 hPa, et `PACK_BITS` repliait en silence une pression plausible et fausse
  au-delà de 1127 hPa ;
- ADS1115 : lecture en entier signé, sinon un capteur éteint lisait 1652 hPa ;
- export du journal BARO : `%d` sur un `double` ;
- pile et cache du baromètre sous `#if ENABLE_BARO_SENSOR`, comme les autres capteurs ;
- CMake refuse le baromètre hors LinkIt et sur un SMD SPI (P0.06 est aussi le MOSI du
  module) ;
- paramètres de capteur non utilisés selon le format marqués `[[maybe_unused]]` : les
  tests PC ne compilaient plus.

**La RAM manquait, et pas à cause de CLS seulement.** Avec la section `.noinit`, la marge
RAM d'avant la fusion était de 1212 o sur la tortue KIM, 920 o sur son banc, 1024 o sur le
banc SMD UART, 376 o sur la caméra UART et **52 o** sur son banc. La fusion ajoute
environ 1,1 Ko à chaque build, et quatre variantes ne se liaient plus. La cause est la
table des paramètres : chaque paramètre coûte environ 216 o de RAM sur **tous** les builds,
implémenté ou non (`param_map` garde des chaînes, environ 134 o par entrée, soit 38 Ko au
total, plus deux emplacements de 40 o dans le magasin). Correctif : la pile réservée passe
de 32 à **28 Ko** (`__STACK_SIZE`). Le pic mesuré par le firmware lui-même (pile remplie au
démarrage, ligne « STACK: N max » de MemoryMonitorService) est de 4,2 à 4,5 Ko sur les
journaux du banc : 28 Ko en laissent six fois plus. Le tas (64 Ko, au plus bas 13 Ko
libres) n'est pas touché. **Proposé, non traité** : sortir `param_map` de la RAM, ce qui
libérerait environ 38 Ko. Nouvelle commande de banc `%MEM` : pic de pile et minimum de
tas libre depuis le démarrage.

**À transmettre à CLS, non changé faute de matériel :**
- **Modulation.** Sans effet pour CLS tant qu'ils restent sans modulation adaptative
  avec un maître LDK, ce qui doit être leur cas puisque leurs trames partent : même trame
  LDK, même RCONF (`load_rconf_for_mode` identique), compteur de messages sans effet sur
  KIM2. La trame woodbuoy est forcée en LDK (128 bits). Avec
  `ARGOS_ADAPTIVE_MODULATION=0` et un RCONF maître d'une autre modulation, leur branche
  **abandonnait l'émission** à chaque fois (DeviceError, puis report) : la trame ne
  partait jamais. Ici, la garde de modulation la ré-encapsule dans la modulation du
  maître et elle part, mais en LDA2 sur un maître LDA2. Pour recevoir la trame telle
  qu'elle est conçue, il faut un maître LDK, ou la modulation adaptative.
- **ADS1115 en conversion continue** (`0xE2`, MODE = 0, alors que le commentaire dit
  « single »). `BARO_PWR_EN` ne coupe que le PTB110 : l'ADS1115 est configuré une fois au
  démarrage, quand `BARO_PWR_EN` est à 0, et le pilote I²C lève une exception sur un NACK.
  Il est donc sur une alimentation permanente, et convertit en permanence : environ 150 µA,
  3,6 mAh par jour. En conversion unique, déclenchée à chaque lecture, environ 0,5 µA.
- **Attente active de 2 s** à chaque mise sous tension du baromètre : l'ordonnanceur
  est bloqué pendant ce temps. **Non changé** : la fin de session GNSS lit aussi le
  capteur (`sensor_handler(false, true)`), et c'est ce blocage qui garantit que cette
  lecture vient après le préchauffage. En non bloquant, une session GNSS finie en moins
  de 2 s lirait un capteur pas encore stabilisé.
- **Mode périodique (BAP02 > 0)** : l'alimentation du PTB110 n'est jamais recoupée après
  une mesure périodique. Corrigé d'abord, puis **remis comme chez CLS** : couper aurait
  ajouté 2 s d'attente active à chaque mesure périodique et changé le préchauffage, un
  changement de comportement non vérifiable sans leur matériel.
- **Retour à un firmware CLS** après celui-ci : leur relecture de `config.dat` est
  encore positionnelle. Elle attend BAP01 à l'emplacement 266 et y trouve UNP26 : elle
  remet donc BAP01 à BAP05 à leur défaut (baromètre désactivé), les emplacements 0 à
  265 restent intacts. Fusionner cette branche chez eux corrige ce point.
- Le segment sol doit décoder les en-têtes `001` et `101` en LDK (16 octets, sans CRC).
  L'en-tête `001` est aussi celui de la trame capteur générique LDA2 (24 octets).

| Vérification | Comment | Fait |
|---|---|---|
| Tests PC | `-p -xg SWS` 1055/1055, Cyprus 1073/1073, SWSAnalog 65, SWSAnalogFlash 30, TurtleSimulation 11/11, RSPBSimulation 10/10 : mêmes effectifs qu'avant la fusion | ✅ 28/10 |
| Contrôles | tables de paramètres (288 emplacements), lignes de journal, textes du banc, sorties sans tirage, garde batterie, cliquet de complexité, liens de doc ; clang-format sur les lignes fusionnées | ✅ 28/10 |
| Compilation | 14 variantes : les 8 de la matrice, la caméra UART et son banc, les bancs SMD UART et RSPB, le woodbuoy et son banc | ✅ 28/10 : toutes compilent, 0 avertissement, marge RAM de 3036 à 13616 o avec la pile à 28 Ko. Avec 32 Ko, quatre ne se liaient plus (ci-dessus) |
| Gardes CMake | baromètre sur SMD SPI, puis sur RSPB : refusés, avec le message | ✅ 28/10 |
| Carte du banc (SMD UART) | la fusion touche du code commun : relecture complète de la configuration identique, commandes de banc, plongée et remontée par `%SWSRAW` | ✅ 28/10 : 34 contrôles sur 34, configuration identique (161 clés, PWP06 à part), publication de l'émersion en 1 ms, 1re session GNSS synchronisée, aucune émission |
| Pile à 28 Ko | build de banc, après une session complète (DTE, GNSS, plongée et remontée, commandes de banc) : `%MEM`, `stack_max` loin sous `stack_size` | ✅ 28/10 : `stack_max=4220` sur 28672 ; même contrôle carte que ci-dessus, 36 sur 36, rejoué sur l'arbre final (option batterie étendue aux trois entrées GNSS) avec le même résultat. Au passage, `heap_min_free=9968` sur 65536 après des lectures complètes des paramètres : le tas est la ressource la plus serrée, d'où le choix de ne pas y toucher |
| Pile à 28 Ko, longue durée | sur carte saine et sur le terrain, les lignes « STACK: N max » du journal (toutes les 12 h) restent loin sous 28 Ko | ⬜ |
| Carte KIM2 | au démarrage, une seule mise sous tension du module, `ARGOS_DECID` / `ARGOS_HEXID` lus et sauvés une fois, aucune émission ; puis une session normale | ⬜ |
| Woodbuoy (CLS) | la liste de `NOTES_FUSION_CLS_102026.md` §6 : mise à jour depuis leur firmware (BAP intacts), trame `001`/`101` décodée, mesure batterie à chaque session GNSS, alimentation du PTB110 coupée après l'échantillonnage pour l'émission, buzzer, veille du KIM2 | ⬜ CLS |

### 2.15 Audit de juillet : les correctifs de `fix-investigation` enfin portés

**Ce que c'est.** La branche `fix-investigation` (18-19/07) porte trois commits d'audit,
`0899f57b`, `3a562b38` et `e713ca66`, jamais intégrés : seules la mortalité, l'I2C du
RSPB, le correctif M10Q et les tests sont revenus le 21/08, et `a8b96b87` donnait le
reste pour « remplacé », à tort. L'inventaire du 28/10 (cinq vérifications, correctif par
correctif) en a trouvé une trentaine absents ; ils sont portés un par un, P1 à P10, dans
cet ordre. Les mécanismes de récupération (P4) et les décisions de batterie ou de
mortalité (P5 à P8) changent un comportement : chacun est soumis à l'utilisateur avant
d'être écrit, puis validé sur la carte concernée.

**Builds de validation, `FAULT_INJECT`.** Pour voir une échelle de récupération sur une
vraie carte, `FAULT_INJECT=<n>` provoque la panne à chaque démarrage : 1 une faute
processeur pendant l'init, 2 une exception pendant l'init, 3 l'échec de l'init de la flash
IS25, 4 une exception juste après le démarrage de la machine à états. CMake refuse toute
autre valeur et avertit dès qu'elle n'est pas 0. **Firmware de validation : il ne
démarre jamais, reflasher un build normal ensuite.**
- LinkIt SMD UART : `FAULT_INJECT=2 ./scripts/build_linkitv4_smd.sh --bench --uart`, console USB.
- RSPB : `FAULT_INJECT=3 ./scripts/build_rspb.sh --bench`, journaux sur l'UART de débogage
  (921600, émission seule). La faute processeur (1) ne passe par le nouveau compteur
  qu'en build release, sans console : `FAULT_INJECT=1 ./scripts/build_rspb.sh`, et
  l'observation se fait au PPK.

| Correctif | Carte | Comment | Attendu | Fait |
|---|---|---|---|---|
| P1 `21a0e18b` : le compteur d'échecs de démarrage n'est effacé qu'après 60 s en fonctionnement | PC | `Sm.CrashInTheFirstMinuteOfOperationalBacksOff`, `Sm.FaultBeforeTheMinuteKeepsTheBootFailCount` | passent, échouent sans le correctif | ✅ 28/10 |
| | LinkIt SMD (banc) | après un flash, sans entrer en configuration : `%BOOTFAIL` dans la 1re minute, puis jusqu'au 0 | 1, puis 0 vers 60 s, et `BootFail: clearing` à la console | ✅ 28/10 : 1 à 3 s, 0 à 65 s, ligne vue ; 36 sur 36 |
| | KIM2, RSPB | même relevé | idem | ⬜ |
| P2 `5d6fcac6` : un morceau de journal illisible ne laisse plus de pointeur vers un handle libéré | PC | `Logger.UnreadableChunkLeavesNoDanglingReadHandle` | passe ; sans le correctif, le processus meurt (signal 11) | ✅ 28/10 |
| | LinkIt SMD (banc) | régression : contrôle carte complet | 36 sur 36, `DUMPD` complet | ✅ 29/10 sur `2d8194fa` : 36 sur 36 (pic de pile 4684 o sur 28 672), `DUMPD` 100 paquets sur 100 |
| P3 `c66d73a4` : une sauvegarde qui échoue n'empêche plus la coupure | PC | `BatteryCriticalGuard.a_failed_save_before_the_cut_still_powers_down` | passe ; sans le correctif, l'exception sort et `powerdown()` n'est pas appelé | ✅ 28/10 |
| | RSPB | build release normal : un réveil qui n'est pas son tour, un réveil complet jusqu'au budget d'émission, puis la coupure (PPK) | coupure comme avant ; journal sans `config save failed` | ⬜ |
| P4 : mécanismes de récupération (choix de l'utilisateur : les quatre) | PC | aucun test possible : code cible, fautes réelles | compilation de 10 variantes, dont les builds de validation | voir ci-dessous |
| | LinkIt SMD (banc) | `FAULT_INJECT=2`, puis `=4` | 5 × `init threw (n/5) — reset to retry` ; puis `sleeping until the watchdog`, 60 s de rouge, et un redémarrage par le chien de garde au bout de 15 min | ✅ 29/10, images Release (chien de garde en marche), lues par SWD une fois à 25 s et à 85 s : `s_initfail` 6 puis 6 pour FI=2 comme pour FI=4, magique `1417FA11` : cinq resets, puis l'attente du chien de garde. À 16 min la lecture est refusée (APPROTECT réarmé par le reset du chien de garde, comme pour C2). Le 28/10 : **non concluant** : avant la machine à états, ni la console USB (jamais énumérée, la boucle principale ne tourne pas) ni le journal flash (ouvert dans `BootState`) ne voient ces lignes ; et au PPK, USB branché, la carte du banc ne montre qu'un palier plat (10,5 puis 27,8 mA). À refaire en regardant la LED (5 × 2 s de rouge, 60 s de rouge, noir ~15 min, de nouveau 60 s), ou au PPK seul sur une carte saine. `=4` écrit dans le journal flash pour les deux premiers démarrages (`BootState` a ouvert le journal) |
| | LinkIt SMD (banc) | `FAULT_INJECT=3` | 5 × `IS25 flash init failed (n/5)`, puis `deferring to the watchdog` : inchangé sur LinkIt | ✅ 29/10, image Release (chien de garde en marche), lue par SWD : `s_flashfail` 6 à 25 s puis 6 à 85 s, stable : cinq resets, puis l'attente du chien de garde |
| | RSPB | `FAULT_INJECT=2`, `=3`, `=4` (build de banc) | 5 resets, puis `asking the TPL5111 to cut the power` et la coupure : courant au niveau TPL éteint, UART muette jusqu'à la période suivante, qui recommence | ⬜ |
| | RSPB | `FAULT_INJECT=1` (build release) | au PPK : 4 démarrages brefs, la coupure à la 5e faute, puis le même cycle à la période suivante | ⬜ |
| | LinkIt release | `FAULT_INJECT=1` | inchangé : resets en boucle, comme avant (le compteur n'existe que sur RSPB) | ⬜ |
| | toutes | build normal reflashé après chaque essai | démarrage normal, configuration identique | ✅ 29/10 LinkIt SMD : après les trois images, `%BOOTFAIL` 1 (le compte de FI=4 n'est pas hérité), OPERATIONAL, configuration identique ; RSPB à faire |
| P5 : le reset de récupération garde la sécurité (H6) ; 3 échecs d'émission consécutifs coupent (RSPB, budget réglé, `SHUTDOWN_TIMER=0`) ; modulo < 2 = chaque réveil (C2b) | PC | `ConfigStore.RecoveryResetKeepsTheSafetyParamsAndFactrDoesNot`, `TxFailuresEndTheWake` (3), `RSPBRecovery` (2, dans RSPBSimulation) | passent ; les nouveaux échouent sans leur correctif | ✅ 28/10 |
| | RSPB | `SHUTDOWN_TIMER=0`, `SHUTDOWN_NTIME_SAT=3`, module satellite débranché ou sans réponse : un réveil | `TX failures in a row, no SHUTDOWN_TIMER` au 3e échec, puis la coupure (PPK) au lieu d'une heure de suspension | ⬜ |
| | RSPB | même essai avec `SHUTDOWN_TIMER=600` | inchangé : la suspension, puis la coupure par le minuteur | ⬜ |
| | RSPB | récupération automatique (5 démarrages ratés, `FAULT_INJECT=4`), puis un build normal : `PARMR` | `PWP01`, `PWP03`, `PWP04`, `PWP05`, `LBP01/02/12/14` gardés, les modes revenus aux défauts | ⬜ |
| | LinkIt | `FACTR` | reset complet : `LBP01/02/12` reviennent aussi aux défauts | ✅ 29/10 LinkIt SMD, sans émission : `LBP01/02/12` posés à 1/33/7, `FACTW`, puis 0/10/5, `ARP01` LEGACY, `IDP11` FACTORY, DECID et HEXID gardés ; configuration remise |
| P6 : jauge STC3117 du RSPB. « Critique » exige l'état de charge **et** la tension sous `LBP12` (H3a) ; une erreur I²C au démarrage de la jauge est vue et la jauge arrêtée (compteur, #7) ; borne de l'écriture I²C (#8). Jauge muette = batterie basse (H3b) : refusé, inchangé | PC | `RSPBGauge` (3, dans RSPBSimulation) | passent ; le verdict et le compteur échouent sans leur correctif | ✅ 29/10 |
| | RSPB | réveil normal, jauge saine | `STC3117: V=… SOC=…` comme avant, aucune ligne `not critical`, courant de la jauge en veille inchangé (PPK) | ⬜ |
| | RSPB | `LBP12` réglé au-dessus de l'état de charge lu, `LBP02` au-dessus de `LBP12` : SOC et tension sont alors d'accord | `BatteryCriticalState` et coupure immédiate, comme avant : le veto ne retient pas une vraie batterie critique. Remettre `LBP12` et `LBP02` : aimant au réveil, `BatteryCriticalState` passe alors en configuration | ⬜ |
| P7 : BMA400. Calibration `SCALW` 3 moyennée sur les lectures réussies, **refusée** sous la moitié : l'ancienne reste dans `AXL.CAL` et `SCALW` répond une erreur (M1) ; lecture sans donnée prête = 0/0/0 au lieu de la valeur précédente (L8) ; capteur à 0 g = activité maximale, une ligne de journal par épisode (M3b) | PC | `Bma400Math` (4, TrackerTests et Cyprus) | passent ; trois échouent avec l'ancien calcul (le 4e fige le calcul inchangé) | ✅ 29/10 |
| | LinkIt SMD (banc) | `SCALR` 0,4 à 0,6 (calibration en place, à noter), `SCALR` 0,1 à 0,3 (lecture), puis `SCALW` 0,3 (auto-calibration) carte immobile, et de nouveau `SCALR` | lectures plausibles (\|g\| ≈ 1 au repos), calibration acceptée (`$O`), décalages proches des anciens si la carte n'a pas bougé. Remettre la calibration notée (`SCALW` 0,0 à 0,2) | ✅ 29/10 : calibration en place = défaut (0, 0, 1 g), carte inclinée (\|g\| 0,94). La calibration ne stocke pas un décalage mais le vecteur lu au repos, que la lecture retranche (Z garde 1 g) : l'auto-calibration (`$O`) a stocké (−0,258 ; −0,602 ; 0,682), le vecteur au repos à 0,003 g près, et la lecture suivante a donné (0,006 ; 0,002 ; 0,995 g). « Proches des anciens » ne vaut que si l'ancienne calibration a été faite dans la même position. Défaut remis sur les trois axes et sauvé |
| | RSPB | une session de mortalité complète | activité et score comme avant, aucune ligne `all axes ~0 g` | ⬜ |
| P8 : RSPB. Attente avant `MCU_DONE` ramenée de 1 s à 20 ms (M5). Journal flash (L12) : firmware inchangé, mesurer d'abord. Suspension du SMD gardée à travers les coupures TPL (D1) : **non fait** (décision du 29/10 : P5 borne déjà un réveil à 3 échecs, et un paramètre coûte ~216 o de RAM partout) | RSPB | build release, PPK seule alimentation : un réveil « pas son tour », puis un réveil complet, contre le build d'avant sur le même montage | la coupure arrive toujours, environ 1 s plus tôt dans les deux cas | ⬜ |
| | RSPB | L12 : un réveil complet, puis `DUMPD` | nombre de lignes écrites dans `system.log` par réveil ; si le coût le justifie, `DBP02=2` dans le gabarit RSPB | ⬜ |
| P9 : petits correctifs de juillet, sans effet en fonctionnement normal. DUMPM borné sans débordement 32 bits (M3) ; tampon UBX décalé par `memmove` (L1) ; pile sans division par zéro (L2) ; observateur de coupure d'alimentation sans exception (L3) ; RTCW refuse avant 2000 et après 2100 (L4) ; pseudo-RTC RSPB borné (L6) ; sondage d'émission SMD SPI qui lève → état d'erreur du SMD, plus de reset (6b). **Non portés** : L5 et la borne 2035 de L4 (bombe à retardement), M4 (le reset sur liste de minuteurs pleine est voulu), L11 (garde la 2e tentative de récupération I²C). L10 : à part, ci-dessous | PC | `NrfMemoryAccess` (2), `TplClock.AdvanceStopsAtTheHorizon`, `ArgosTxService.DepthPileZeroDepthOrCapRetrievesSafely`, `DTEHandler.RTCW_REQ_RefusesATimeTheFirmwareWouldNotBelieve` | passent ; ceux de M3, L2 et L4 échouent sans leur correctif | ✅ 29/10 |
| | LinkIt SMD (banc) | `$DUMPM` sur `FFFFFC00`, 0x500 octets ; `$RTCW` à 946684799 puis 4102444801 | `$N;DUMPM#001;7` (refus, pas de faute ni de redémarrage) ; `$N;RTCW#001;7` deux fois, horloge inchangée | ✅ 29/10 : `$N;DUMPM#001;7`, puis `$N;RTCW#001;7` deux fois, horloge inchangée |
| | SMD SPI | 6b : un défaut SPI pendant le sondage d'émission n'est pas provocable sans injection | revue de code ; l'émission elle-même reste couverte par 2.6 | — |
| P9-L10 : un démarrage GNSS refusé par le limiteur laisse la veille profonde en place (avant : plus de coupure du rail jusqu'à la levée du limiteur) | PC | `GPSShortSurface.RateLimitedStartLeavesTheDeepIdleWindowArmed` | passe ; échoue sans le correctif (le rail reste allumé après la fenêtre de 600 s) | ✅ 29/10 |
| | LinkIt SMD (banc) | `RLP01=1`, `RLP03=1`, `GNP52=600`, une session, une émission… | sans émission au banc, le limiteur ne se remplit pas : non jouable ici, à faire pendant la campagne RF | ⬜ |
| Argos C+D (de `robust/argos`) : la position de prépasse semée au démarrage n'est plus effacée par la remise à zéro du planificateur (C) ; un magasin de passes illisible (après une écriture d'AOP ratée) se lit « pas d'AOP » au lieu d'empêcher le service Argos de démarrer, et la réception descendante ne planifie rien au lieu de lever (D) | PC | `ArgosTxService.PrepassStartsFromThePersistedFixAfterAReset`, `ArgosTxService.AnUnreadablePassPredictStoreReadsAsNoAopRecord`, `ArgosRxService.AnUnreadablePassPredictStoreSchedulesNoReception` | passent ; les trois échouent sur HEAD | ✅ 29/10 |
| | LinkIt (prépasse) | `PPP07=1` avec un AOP à jour et une position connue, redémarrage (`RSTBW`), puis `STATR` | `PPT0x` donne la prochaine passe dès le démarrage (avant : rien jusqu'au premier fix) ; aucune émission nécessaire pour le lire | ⬜ |
| Stockage C6+C7 (de `robust/storage`) : l'alimentation de la flash est rendue sur tous les chemins d'erreur, VSYS peut redescendre à 2,3 V après une écriture ratée (C6) ; une erreur de lecture de `pass_predict.dat` garde l'AOP téléversé et charge le jeu intégré en RAM seulement, et un nombre d'enregistrements au-delà de 40 est refusé (C7) | PC | groupe `ConfigStoreFlashFaults` (5), flash à pannes injectées | passent ; les cinq échouent sur HEAD | ✅ 29/10 |
| | LinkIt SMD (banc) | régression : contrôle carte complet après flash, `PASPW` d'un AOP puis `PARMR` de `PPT01` | 36 sur 36 ; l'AOP relu après redémarrage | ✅ 29/10 en partie : 36 sur 36 ; l'AOP de la carte (jeu intégré du 07/10/2021) relu identique après reflash et redémarrage. Un `PASPW` ne passe pas par l'USB (la capture contient des LF, la trame est coupée : AOP-01) : le téléversement se vérifie par BLE |
| Argos A (de `robust/argos`) : sur SMD en AT, une plongée entre l'AT+TX et son `+OK` (chauffe TCXO comprise) rend le crédit de la pile de profondeur ; TxStarted, levé dès l'écriture de l'AT+TX, le comptait émis. KIM2, LoRa et SMD SPI inchangés | PC | groupe `SmdSatCmdAtRfStart` (10, octets du module injectés), `ArgosTxService.CreditIsGivenBackWhenTheBackendSaysRfNeverStarted`, `…CreditStaysSpentOnceTheBackendSaysRfStarted`, `…CreditIsGivenBackWhenTheTxDiesBeforeTxStarted` | passent ; dix mutations détectées | ✅ 29/10 |
| | LinkIt SMD (banc) | régression sans émission : contrôle carte complet après flash, puis `SMDDFU 5` (lecture de version, chemin `+OK` remanié) | 36 sur 36 ; `v1.0.0` relu ; image `v4.2.0-239-gbddb2e15` | ✅ 29/10 |
| | LinkIt SMD, **émission autorisée** (antenne ou charge, alimentation ≥ 1 A) | `ARGOS_TCXO_WARMUP_TIME` ≥ 2 s, plongée par `%SWSRAW` pendant la chauffe d'un AT+TX qui n'est pas la 1re de l'émersion (celle-là part sans chauffe) | « TX ended before reaching the air — 1 depth-pile credit(s) given back », la même position réémise à l'émersion suivante | ⚠️ 29/10 non concluant : la TX est partie 0,6 s après `%GPS`, avant la plongée ; puis **reset à la mise sous tension au démarrage de la RF** (chauffe 10 s, GNSS en marche), 4 `TXSTART` et 4 resets après la chute du lien USB, jusqu'à l'ErrorState. Cause matérielle à établir avant tout nouvel essai ; refaire avec `%AMODE` (RAM) et la plongée sur un crochet de ligne |
| HAULED (de `robust/hauled`) : la sécheresse se compte depuis l'émersion, une plongée y met fin ; une balise jamais mouillée reste AT_SEA (la synchro RTC ne fabrique plus de plongée) ; après un reset, rien ne s'engage avant le 1er rapport du détecteur ; `.noinit` à étiquette de disposition. Lève le plancher HMP01 ≥ 12 h du plan B4 | PC | 12 tests repris + `HauledRestoreRejectsAFutureDrySpell` ; cas existants passés par une émersion | passent ; onze mutations détectées | ✅ 29/10 |
| | LinkIt SMD (banc), sans émission | la séquence de HAUL-02 à la main, Argos coupé **dans les deux profils** : `ARP01=0`, `GNP01=0`, et `HMP10=0`, `HMP12=0`, `HMP13=2`, sans quoi l'engagement substitue `HMP10` (LEGACY par défaut) et la balise émet. `HMP00=1`, `HMP01=1`, puis `%HAULED RESET`, `%DIVE`, `%SURFACE`, `RTCW` +2 h, `%HAULED` ; enfin `%HAULED RESET` et la configuration de référence remise. La campagne classe tous les cas `HAUL-`/`HAULED-` comme émetteurs (HAUL-02 laisse `HMP10` à LEGACY, HAULED-01 émet toutes les 60 s) : pas de `--cases` sans accord | `AT_SEA → HAULED (dry for 7213 s, threshold 1 h)` après le saut de 2 h ; HAULED après une plongée (1/2), AT_SEA à la seconde ; 12 sur 12, configuration remise (0 écart) | ✅ 29/10 |
| | LinkIt SMD, **émission autorisée** | `dte_campaign --cases HAULED-01,HAUL-02,HAUL-03` | les trois passent | ✅ 29/10 : 3 ok ; aucune émission (IDP13/IDP14 vides, `can_transmit()` retient) ; 4 clés remises |
| Stockage C2 (de `robust/storage`) : le compteur d'échecs du stockage n'est plus remis à zéro dès que l'IS25 répond, mais après un `configuration_store->init()` réussi ; un magasin qui refuse de s'initialiser atteint l'escalade (sommeil sous chien de garde, coupure TPL sur RSPB) au lieu de redémarrer sans fin | LinkIt SMD (banc), sans émission | `FAULT_INJECT=5` (init du magasin échouée à dessein), compteur `s_flashfail` lu par SWD après 25 s de marche libre, une seule lecture puis `--run` (`--memrd` fige le cœur) | témoin à l'ancienne place : 1, boucle sans fin ; correctif : 6, puis 9 après 70 s en build de banc (reset lent sans chien de garde) ; Release : 6, chien de garde en marche ; relance à 15 min déduite (APPROTECT réarmé), pas lue | ✅ 29/10 |
| | LinkIt SMD (banc) | image normale, contrôle carte complet | 35 sur 36 : seul `PWP07` (LAST_TIME_SYNC) diffère, posé par la remise à l'heure après l'essai radio ; configuration = référence | ✅ 29/10 |
| Stockage C1 (de `robust/storage`) : seul un `config.dat` **absent** repart des défauts ; toute autre erreur de lecture n'écrit rien, relit 3 fois, puis lève (C2 fait ses 5 resets, le démarrage suivant relit le fichier intact). Budget épuisé : dernier recours — `config.dat` gardé en `config.bad` s'il reste ≥ 25 % libre (sinon supprimé), défauts d'usine avec `TR_NOM` 1200 s, un fix GNSS par heure, batterie faible activée, et l'identité reprise au module radio (SMD, LoRa ; KIM2 émet sur la sienne) ; décisions de l'utilisateur du 29/10 | PC | groupe `ConfigStoreFlashFaults` (+9 : illisible laissé tel quel, panne passagère, erreur au milieu du fichier, dernier recours à un coup, gardé / supprimé faute de place / supprimé faute de mesure, identité Argos et LoRa adoptées) | passent ; dix mutations détectées | ✅ 29/10 |
| | LinkIt SMD (banc), sans émission | `FAULT_INJECT=6` (lecture de `config.dat` refusée à l'init de main.cpp, Argos coupé dans les quatre profils), `%AMODE 0` et garde RF ; puis image normale, configuration remise, contrôle carte | 6e démarrage : profil de survie, DECID, HEXID `6B972D`, clé et radioconf repris du SMD ; journal : « Configuration started over … kept as config.bad », « Radio identity taken back … » ; aucune émission ; ensuite 36 sur 36, configuration = référence | ✅ 29/10 |
| Stockage C5 (de `robust/storage`) : la récupération automatique après des démarrages ratés ne formate plus ; `config.dat` réécrit en place (protégés et sécurité gardés), calibrations réécrites, les autres fichiers supprimés un à un (journaux, AOP, base GNSS, images OTA ; `config.bad` et `radio_ids.todo` gardés). FACTR garde son formatage | PC | `ACutAnywhereInTheRecoveryKeepsTheIdentity` (coupure après chacune des 140 écritures : l'identité relue à chaque fois ; l'ancien formatage la perd dès la 1re), `TheRecoveryKeepsTheCalibrationAndDropsTheState`, `FactrStillFormats` | passent ; trois mutations détectées | ✅ 29/10 |
| | LinkIt SMD (banc), sans émission | `%RECOVERY` (banc) : la récupération sur demande, fichiers listés avant et après ; calibration marquée sur les trois axes | 17 fichiers avant, 4 après (`AXL.CAL`, `SWS.CAL`, `config.bad`, `config.dat`) ; après le redémarrage DECID et sécurité gardés, le reste d'usine, calibration conservée ; puis configuration remise, 36 sur 36 | ✅ 29/10 |
| Banc, LED-03 | carte saine | rejouer `dte_campaign --cases LED-03` | rouge pour une session sans position ; sur la carte du banc le reed rebondit sans aimant (110 fronts en 30 s), d'où le blanc « aimant présent » | ⬜ |

---

## 3. Critères de merge

**Bloquant :**
- une suite de la section 1 en échec ;
- une case « carte » de la section 2 en échec ;
- une émission radio non voulue pendant la campagne.

**À arbitrer, pas bloquant :** une case ⬜ qui ne peut pas être faite faute de matériel
(carte RSPB, carte KIM2, firmware SPI du SMD). Elle doit alors être notée « non vérifié »
dans la description du merge, avec sa raison.

**En fin de campagne :** remettre la configuration d'origine de la carte du banc
(section 0), et dire dans le rapport ce qui a été réellement mesuré.
