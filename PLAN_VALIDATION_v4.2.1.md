# Plan de validation carte — branche `v4.2.1-optim`

**Pour une session dédiée au banc, sous WSL + usbipd.**
Objectif : décider si cette branche peut partir sur des tortues.

Ce plan couvre deux choses distinctes. La **partie A** valide les 35 commits ajoutés
depuis l'audit du 12 octobre : c'est une recherche de régression, et rien n'y a été
éprouvé sur matériel. La **partie B** valide la branche pour un déploiement, ce qui
est plus large et inclut des chemins antérieurs à ces commits.

Lire la section 0 en entier avant de brancher quoi que ce soit : elle contient quatre
pièges qui invalident silencieusement une campagne.

---

## 0. Préconditions

### 0.1 La variante doit correspondre à la carte

Le banc du 14 octobre a conclu « le KIM2 ne répond à aucune commande, absent ou en
panne ». C'était faux : la carte est une **SMD**, et le firmware avait été compilé
avec `build_linkitv4_kim.sh`, qui n'active pas `ARGOS_SMD` et parle des commandes AT
KIM2 à un STM32WL qui ne les comprend pas.

| Carte | Script |
|---|---|
| LinkIt V4 + SMD (STM32WL) | `./scripts/build_linkitv4_smd.sh` |
| LinkIt V4 + KIM2 | `./scripts/build_linkitv4_kim.sh` |
| LinkIt V4 + RAK3172 LoRa | `./scripts/build_linkitv4_lora.sh` |
| Cyprus (bateau) | `./scripts/build_linkitv4_lora_cyprus.sh` |
| RSPB | `./scripts/build_rspb.sh` |

**Identifier la carte avant de compiler.** Un module satellite muet est le symptôme
d'une variante fausse avant d'être celui d'une panne.

### 0.2 Le dépôt doit être à jour

```bash
git fetch && git log --oneline -1 origin/v4.2.1-optim
```

Doit afficher `845c764f` ou plus récent. Un arbre en retard teste du code absent.

### 0.3 Le pont USB

```bash
./tests/bench/wsl_usb.sh list              # busids + état
./tests/bench/wsl_usb.sh attach <busid>    # une fois par périphérique
```

Deux périphériques sont nécessaires : la sonde **J-Link** (VID `1366:0101`) pour le
flash, et le **CDC Nordic** (VID `1915:520f`) pour la console. Le premier attachement
d'un périphérique donné demande un `bind` administrateur côté Windows.

Si les nœuds apparaissent en `root:root 0600` et sont inouvrables, `udevd` ne tourne
pas : `sudo /lib/systemd/systemd-udevd --daemon`, puis **ré-attacher** pour que les
règles s'appliquent.

Le lien usbip tombe régulièrement, y compris sur la sonde SWD (erreur -102 « Cannot
connect to J-Link »). Les harnais se reconnectent seuls ; un reflash, non.

### 0.4 Les quatre pièges qui invalident une campagne

**APPROTECT se ré-arme, et le flash échoue en silence.** Le pied de page de
`nrfjprog` n'est pas une erreur visible dans un `tail`, et on croit tester du code
corrigé qui n'est pas sur la puce.

```bash
nrfjprog --recover -f nrf52
nrfjprog --program ports/nrf52840/build/<DIR>/LinkIt_board_merged-<version>.hex \
         --chiperase --verify --reset
```

Exiger de voir `Verify successful` **et** `Run.` dans la sortie. Le hex **fusionné**,
jamais `app_settings` seul. Vérifier une chaîne dans l'ELF ne prouve rien sur ce qui
tourne.

**`--recover` efface le magasin de configuration.** Tout paramètre posé pour un essai
est à réécrire après chaque flash. Et un `--chiperase` n'efface **pas** le magasin,
qui vit sur la QSPI IS25 externe : les nouveaux défauts firmware ne s'appliquent donc
qu'à une configuration **créée à neuf**, pas par un simple reflash.

**`BENCH_TEST` n'est pas neutre.** `PMU::powerdown()` retourne immédiatement, et sur
RSPB le saut modulo du TPL5111 est désactivé. Toute mesure de stockage faite sur un
build `--bench` ordinaire mesure une carte **allumée**. Utiliser `--bench-storage`
quand le sommeil fait partie du test.

**`DEBUG_LEVEL=3`, défaut des builds de banc, compile `DEBUG_TRACE` hors du binaire.**
Un test qui guette une ligne TRACE ne la verra jamais. `DEBUG_INFO`, `DEBUG_WARN` et
`DEBUG_ERROR` passent. Et un `DEBUG_WARN` part aussi dans le journal flash, à ~170 ms
la ligne : une boucle qui en émet des dizaines coûte des secondes.

### 0.5 État laissé par la campagne du 14 octobre

Trois paramètres ont été mis à zéro pour empêcher une émission involontaire, et y
sont restés : **`LBP04` 4→0, `ZOP11` 2→0, `HMP10` 2→0**. À reposer avant tout essai
satellite. Les laisser à zéro pour tout le reste.

### 0.6 Les tests hôte d'abord

```bash
cd tests/build && cmake -GNinja .. && ninja TrackerTests TurtleSimulation RSPBSimulation
cd .. && ./build/TrackerTests -xg SWSLongRun
```

Attendu : **948/948**. Lancer depuis `tests/`, pas depuis la racine : cinq tests PASPW
lisent `tests/data/*.json` en chemin relatif et échouent sinon — faux échecs qui
envoient chercher un défaut inexistant.

Garder le `-xg SWSLongRun` : sans lui la suite dure **53 minutes** au lieu de 24
secondes, parce qu'elle simule quinze déploiements de 75 jours.

---

## Partie A — Régression des 35 commits

Rien de ce qui suit n'a été éprouvé sur matériel. Par ordre d'exposition : le premier
groupe n'a **aucune** couverture hôte.

### A1. Balayage de la console `%` — priorité absolue

```bash
DEBUG_LEVEL=3 ./scripts/build_linkitv4_<variante>.sh --bench
# flasher, puis :
python3 tests/bench/console_sweep.py
```

`handle_line` est passé d'une chaîne de 35 comparaisons `cmd ==` à cinq fonctions par
famille, dans un fichier qui n'existe que sous `BENCH_TEST` — donc zéro test hôte. Le
script extrait les 32 commandes de la source du firmware, ce qui lui interdit de
dériver, et vérifie que chacune est **routée** : la réponse n'est pas `unknown-cmd`.
Il ne vérifie pas le comportement, seulement l'aiguillage, qui est exactement ce
qu'un refactoring de dispatch peut casser.

Attendu : **31 routées**, et `%AXLFAIL` annoncée « pas dans cette variante » sauf sur
un build où `LORA_MOTION_EXT` est actif. `%STORE` est exclue, elle couperait le lien
USB.

Le script pose le garde RF `%AMODE 0` avant tout le reste, parce qu'un KIM2 qui
répond **émet** même sans credentials chargés.

### A2. Aller-retour DTE sur les décodeurs devenus des tables

`AQPERIOD`, `ARGOS_MODE`, `DEPTH_PILE`, `GNSS_DYN_MODEL`. Pour chacun : écrire chaque
code valide, relire, comparer ; puis écrire un code hors table et vérifier le refus
par `DTE_PROTOCOL_VALUE_OUT_OF_RANGE` **sans que la valeur change**.

`tests/bench/dte_campaign.py` couvre déjà une large campagne DTE sans émission radio.

Deux pièges, sinon on conclut à tort à un bug : `DEPTH_PILE` a une table où le code
n'est **pas** la valeur (9→12, 10→16, 11→20, 12→24), et `GNSS_DYN_MODEL` n'a pas de
code 1.

**Ne pas chercher `ARGOS_POWER`** : c'est `_RESERVED_ARGOS_POWER`, clef vide,
`is_implemented=false`, « Obsolete: RADIOCONF controls power ».

Rappel sur le protocole : le champ longueur est **hexadécimal**, encodé `"%03X"`.
Décimal et hexadécimal coïncident jusqu'à 9 caractères de charge utile et divergent
dès 10. Et le code d'erreur **4 est `DATA_LENGTH_MISMATCH`**, pas `VALUE_OUT_OF_RANGE`
qui vaut 7.

### A3. Session GNSS

```bash
python3 tests/bench/gnss_bench.py      # campagne intérieure, aucun fix nécessaire
python3 tests/bench/gps_validate.py    # classifieur start_class + TTFF, ciel réel
```

`getParameterSize` remplace 111 comparaisons par une table de 113 pointeurs vers les
déclarations, et rend décodables trois clefs CloudLocate qui ne l'étaient pas.

**Portée réelle, à ne pas surévaluer** : la seule consommatrice d'une valeur décodée
est un `DEBUG_TRACE`, et `m_op_state` passe à `SUCCESS` que la boucle sorte tôt ou
non. Le défaut coûtait trois lignes de diagnostic, compilées hors binaire sous
`DEBUG_LEVEL 4` de surcroît. Ce qu'il faut vérifier est donc simplement qu'une
session se déroule normalement.

### A4. Les quatre niveaux de journalisation compilent et démarrent

```bash
for L in 1 2 3 4; do
  DEBUG_LEVEL=$L ./scripts/build_linkitv4_<variante>.sh --release
done
```

`DEBUG_LEVEL=2` **ne compilait pas** avant ces commits : dix erreurs `-Werror`, sur
des variables devenues inutilisées quand les macros `DEBUG_*` s'effaçaient, plus des
`if (cond) ;` vides refusés par `-Werror=empty-body`. C'est le réglage qu'un
déploiement demande, et il était cassé.

Flasher le niveau 2 et vérifier le démarrage. Attention : un build **release** a une
console muette par conception (`NDEBUG` force `g_debug_mode = NONE`, et rien ne le
réattache — l'override DTE ne concerne que `BLE_NUS`). Le filtrage WARN/ERROR se
vérifie donc sur un build `--debug` au même niveau, ou dans le journal flash par
`tests/bench/log_harvest.py`.

### A5. Chemins de brique — demande une panne provoquée

B1 et B2 ne se déclenchent que sur un vrai défaut flash, et c'est pour ça qu'ils
n'ont jamais été éprouvés.

- **B1** — une écriture de configuration qui échoue sortait de `main()` par
  `std::terminate` : aucun log, aucune LED, et le watchdog rebootait toutes les 15
  minutes dans le même `terminate`. Elle passe maintenant par `storage_unusable()` :
  quelques reboots annoncés avec clignotement rouge, puis report au watchdog.
- **B2** — un seul `mount()` raté déclenchait `format()`, qui efface la configuration
  **et les credentials**. Trois tentatives non destructives précèdent désormais le
  format.

Provocation possible : rendre la QSPI illisible (brouiller le `MOSI` au repos, ou
poser une image flash corrompue), puis observer le clignotement rouge et le compte de
reboots au lieu d'une carte muette. À défaut, consigner que ce n'est pas éprouvé.

### A6. Journal CSV des capteurs scalaires

Vider le journal (`tests/bench/log_harvest.py`) et vérifier les colonnes :
`log_datetime,temp` / `,pH` / `,sea_temp` / `,lumens`, date en `dd/mm/yyyy HH:MM:SS`,
séparateur virgule, **CRLF en fin de ligne**. Les quatre formateurs partagent
désormais un seul écrivain ; la thermistance écrivait dans un tampon de 128 octets là
où les trois autres avaient 512, ce qui tronquait la ligne sur valeur aberrante.

---

## Partie B — Validation de déploiement

Au-delà des 35 commits. Ce qui décide si la branche peut partir en mer.

### B1. Séquence extinction / stockage / réveil

`PROTOCOLE_TESTS_STOCKAGE.md` à la racine porte la procédure complète, avec montage
PPK2, repères de courant attendus (stockage **8-20 µA**, idle System ON en centaines
de µA, session GNSS **25-35 mA**) et les cas T1 à T6.

`python3 tests/bench/storage_bench.py` pilote la campagne. Build
`--bench-storage`, sans quoi `powerdown()` ne s'exécute pas.

Le point à vérifier en priorité : **après extinction, un maintien d'aimant de 3 s
rallume la balise**, et un effleurement la laisse endormie. Et depuis la correction du
défaut D2, l'extinction attend le **retrait** de l'aimant — clignotement magenta lent
jusqu'à ce qu'il parte, borné à 60 s.

### B2. Démarrage GNSS à chaud

`python3 tests/bench/gps_validate.py` classe le type de départ et mesure le TTFF.

Références mesurées le 9 octobre : `S0_cold` TTFF 33,8 s · `S4_stale` 10,3 s ·
`S1_hot` **4,8 s**. Une régression ici coûte directement de l'autonomie, puisque
l'acquisition tire ~25 mA.

Vérifier aussi la courbe de vieillissement : un écart de 30 min donne un TTFF de
~7,7 s, mais **4 h donne 31,6 s** — le réveil reste rapide à tous les écarts, c'est
l'acquisition qui s'effondre une fois l'éphéméride périmée.

### B3. Émission satellite réelle

Demande des credentials chargés et une autorisation d'émettre.

- `tests/bench/argos_validate.py` — modes LEGACY et BLIND sur carte vivante
- `tests/bench/sat_dx1_smoke.py` — Doppler de surface, trames DX1, fix GNSS
- `tests/bench/turtle_mission_rf.py` — mission de bout en bout, **émission réelle**

**Garde RF** : un module qui répond émet, même sans credentials. Poser `%AMODE 0`
avant tout ce qui n'est pas un essai d'émission délibéré.

### B4. Freins d'émission — à activer, puis à vérifier

Les deux freins décidés le 12 octobre **ne sont pas dans les templates** :
`RATE_LIMIT_EN = 0` dans les quatre, `HAULED_DETECT_EN = 0`.

⚠️ **L'ordre compte, et un seul des deux est de la configuration.** Le frein
d'échouage ne peut pas être activé tel quel : son entrée mesure
`now − last_uw_event_rtc`, c'est-à-dire le temps depuis le dernier *événement de
plongée*, et non le temps passé à sec ([`hauled_mode_service.cpp:196`]). Pendant une
plongée longue aucun événement n'arrive, le compteur croît, et le mode échoué
s'engage **sous l'eau**. Une tortue plonge plus d'une heure : la balise basculerait en
LEGACY 30 min pour émettre dans l'eau.

Donc : corriger l'entrée **avant** d'activer `HAULED_DETECT_EN`. Le limiteur de débit,
lui, est purement configuration.

### B5. Batterie et seuils critiques

Vérifier que le verdict critique demande des lectures basses étalées sur une heure, et
non trois échantillons d'une même salve.

⚠️ **La jauge du banc est alimentée par USB, sans cellule** : la lecture VBAT flotte
entre 3,4 et 3,9 V, n'est pas représentative, et n'a aucun lien avec VSYS. Elle peut
parquer la carte en `OffState` ; neutraliser par `LBP12=0` au banc.

### B6. Endurance

`python3 tests/bench/endurance.py` — essai long, indépendant du lien USB, pour les
fuites qui n'apparaissent qu'après des heures.

---

## Ce qui ne peut pas être validé au banc

À consigner comme non couvert, plutôt que supposé bon.

- **La fuite EXTINT.** Estimée 75-375 µA, jamais mesurée. Demande un PPK2 sur V_IO
  pendant un deep-idle complet, EXTINT bas puis haut, en comptant les réveils
  parasites dans la même session. Le remède évident est de la même famille que le
  changement qui a causé une régression mesurée à **2 mA**, soit 5 à 27 fois pire.
- **`GNP52` sentinelle contre fini** dans les templates. Demande une mesure de courant
  sur cellule réelle ; la jauge du banc ne peut pas y répondre.
- **Le comportement SWS en mer.** Une fragilité est documentée dans
  `tests/src/sws_longrun_sim_test.cpp` : si le délai adaptatif atteint son plancher,
  rien ne le fait remonter. La télémétrie dit que ce n'est **pas** une panne —
  `surface_dry_pct` lit 3,7 à 16,1 % sur dix balises et quatre mois. Pour la dater :
  vider le journal SWS d'une balise récupérée et lire la colonne `sample_delay_us`.
  Si elle affiche 200, la fragilité est réelle en mer.

---

## Critères de décision

**Bloquant si l'un échoue :**

- A1 — une commande de console non routée
- A2 — un paramètre qui ne se relit pas à l'identique, ou un code invalide accepté
- A4 — un niveau de journalisation qui ne compile pas ou ne démarre pas
- B1 — l'extinction ne tient pas, ou un maintien de 3 s ne rallume pas
- B4 — `HAULED_DETECT_EN` activé sans le correctif d'entrée

**À arbitrer, pas bloquant :** A5 et A6 non éprouvés, B3 sans credentials, B6 écourté,
et les trois points non validables ci-dessus.

**Rapporter les résultats bruts, y compris les échecs, et ne rien corriger sans le
dire.** Un rapport déjà « réparé » sur une base inconnue vaut moins qu'un échec
documenté.
