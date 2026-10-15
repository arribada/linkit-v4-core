# Évaluation multi-agents — firmware LinkIt V4 Core (2026-10-15)

*7 dimensions, 48 constats soumis, 3 vérificateurs adverses chacun. 5 constats ont survécu à la majorité.*

---

## 1. VERDICT

Le code est tenu là où il compte : les quatre chemins de brique de l'audit du 12 octobre sont fermés et relus un par un, aucun des cinq constats survivants n'est une panne, et un seul est une fragilité. Ce qui l'empêche d'aller plus haut n'est pas la survie mais la forme — une fonction de 768 lignes à CCN 150 arbitre seule le mouillé/sec (`core/services/sws_analog_detection.cpp:177`), le chemin d'émersion est écrit deux fois et a déjà divergé jusqu'au terrain (`core/services/lora_tx_service.cpp:136-137`), et rien n'empêche mécaniquement ces deux dettes de croître (`.github/workflows/tests.yml:113`). Et la mesure elle-même plafonne : aucune couverture de lignes dans le dépôt, aucune mesure de courant, une variante compilée sur huit — le code paraît meilleur que ce qu'on peut prouver, et cette asymétrie est le vrai risque avant un déploiement scellé.

---

## 2. TABLEAU DES NOTES

| Dimension | /20 | Ce qui plafonne | Base 12/10 | Écart |
|---|---|---|---|---|
| Robustesse (survie du scellé) | **16** | Le durcissement `PROTECTED_PARAMS` (13 entrées, `config_store_fs.hpp:433-454`) n'est câblé que sur `factory_reset`, pas sur le chemin de désaccord de version (`config_store_fs.hpp:273`) — prouvé au banc hôte | 13 | +3, réel sur les 4 chemins de brique ; mais une partie du périmètre est passée dans « fiabilité », donc pas purement du progrès |
| Fiabilité (erreurs, reprise, FSM) | **15** | Barrière d'exception asymétrique : `Service::run_scheduled_task` l'a, les trois répartiteurs de pilote (`smd_sat.cpp:346`, `kim2.cpp:956`, `m10qasync.cpp:780`) non | — | Dimension nouvelle, ne se compare pas |
| Lisibilité | **12** | `detector_state` 768 lignes / CCN 150 / 81 % de son fichier, plus 9 renvois de ligne périmés sur 10 | 10 (agrégée) | Non comparable directement : la base fusionnait lisibilité et réutilisation |
| Réutilisation et couplage | **12** | Argos et LoRa sont la même machine à états d'émersion écrite deux fois (`lora_tx_service.hpp:120`), 17 membres sur 29 de noms identiques | 10 (agrégée) | Idem — indicatif seulement |
| Qualité (tests, outillage) | **13** | Trois garde-fous passent à vide : aucun job de CI ne compile le firmware (`build.yml:17`, `workflow_dispatch` seul), `TrackerTestsCyprus` n'est ni compilé ni lancé, `RSPBSimulation` est compilé puis jeté (`tests.yml:59`) | 13 | Stable. La discipline s'est améliorée, les garde-fous pas |
| Complexité | **11** | Pire fonction toujours à CCN 150, 17 sections sur 19 à sortir, et aucune garde de complexité nulle part | 8 | +3, réel : cinq réductions chiffrées et testées (`getParameterSize` 111→3, `handle_line` 118→12, `detector_state` 177→150) |
| Énergie | **13** | L'énergie RF par émission est absente de l'arbre (ni datasheet KIM2, ni SMD/STM32WL, ni courant TCXO), donc le poste dominant — 73-78 % du budget — n'est pas chiffrable | 11 (« power ») | +2, mais les deux mesures ne portent pas sur la même base : le budget GNSS écrit dans le template et la fenêtre `hold()` sont postérieurs |

**Réserve sur la note énergie** : sa justification s'appuyait en partie sur « la phase 2 de SURFACING_BURST n'a aucun plafond » (`argos_tx_service.cpp:775`). Ce constat a été réfuté 3/3 — la phase est bornée par `depth_pile × ntry` et le code le dit explicitement (`argos_tx_service.cpp:1121-1123`). Le 13 est donc plutôt conservateur sur cet axe ; son vrai plafond est l'absence de données RF.

---

## 3. CE QUI COMPTE VRAIMENT

### Aucune panne

**C'est le résultat principal.** Aucun constat de classe « panne » n'a survécu à la vérification, et les huit candidats qui le prétendaient ont été réfutés unanimement. Les quatre chemins de brique de l'audit du 12 octobre sont vérifiés fermés, lus un par un :

- init du magasin de config sous try/catch triple avec escalade bornée — `ports/nrf52840/main.cpp:955-963`
- format LFS seulement après 3 remontages non destructifs — `main.cpp:919-931`
- `ErrorState` ne va plus jamais en `OffState` sur LinkIt, mais enchaîne 5s→1h puis un reset — `core/sm/gentracker.cpp:202`, `:921-941`
- `startall` met chaque service derrière sa propre barrière — `core/services/service.cpp:117-144`

S'y ajoute un cinquième verrou non prévu par la base : les quatre sites `PMU::powerdown()` des services sont soit un geste opérateur, soit RSPB-seulement, soit derrière un verdict qui exige 3 relevés batterie espacés de ≥30 min (`nrf_battery_mon.hpp:46-47`).

### Fragilité — 1 constat

**L'OTA MCU accepte n'importe quel binaire dont le CRC32 déclaré correspond** — `core/filesystem/ota_flash_file_updater.cpp:94`
L'en-tête écrit en tête de zone fait 8 octets, `[taille:4][crc32:4]` et rien d'autre (`FLASH_HEADER_SIZE`, ligne 28) ; la seule validation de taille est `length == 0 || length & 3 || length > zone - 8` (ligne 67), donc le plancher est 4 octets, et le CRC attendu vient du client BLE, pas du binaire. Le bootloader ne complète rien — il ne compare que le CRC calculé à celui de cet en-tête (`ports/nrf52840/bootloader/secure_bootloader/dfu_ext_flash.c:141-144`), sans signature ni identifiant produit, `NRF_DFU_HW_VERSION` vaut 52 (la famille de puce) et les cinq scripts de build passent `--application-version 0` : les six arbres de build produisent des images indistinguables à tous les niveaux de cette chaîne. Deux gardes bon marché manquent au même endroit : un identifiant de variante dans le bloc de 8 octets (il reste de la place) et une taille plancher plausible.

### Dettes — 4 constats

**`detector_state` : 768 lignes, CCN 150, 19 sections numérotées à la main** — `core/services/sws_analog_detection.cpp:177`
C'est la fonction qui décide mouillé/sec, donc la porte de toute émission, du cycle GNSS et du budget d'énergie — et la numérotation manuelle des sections *est* la structure, sans aucune vérification par le compilateur. L'équipe y travaille et le prouve (`de05f12a`, `a8d5b0df` : CCN 177 → 161 → 150, 62 tests au vert à chaque pas), mais 17 sections restent, et le risque réel de l'extraction n'est pas les 13 locales — le compilateur les attrape — c'est l'ordre des 26 champs mutés, dont 5 sont lus dans une section située *avant* celle qui les écrit (`m_time_in_current_state`, `m_observed_peak_adc`, `m_surface_lockout_remaining`, `m_peak_adc_since_underwater`, `m_prev_ma3`).

**Argos et LoRa sont la même machine à états d'émersion écrite deux fois** — `core/services/lora_tx_service.hpp:120`
17 des 29 membres `m_*` portent le même nom des deux côtés, 20 des 32 fonctions aussi, et 18 commentaires renvoient explicitement à `ArgosTxService` (« mirrors ArgosTxService::m_cooldown_armed »). La divergence s'est déjà matérialisée sur le terrain et le code le dit : `lora_tx_service.cpp:136-137`, « ce chemin avait été oublié lors du durcissement d'ArgosTxService : une balise LoRa s'éteignait encore sur un transitoire » — un `powerdown()` sur un relevé batterie isolé, appareil scellé, sans downlink.

**Aucune garde de complexité, alors que le dépôt sait se poser des gardes** — `.github/workflows/tests.yml:113`
`grep -rniE 'lizard|cyclomatic|cppcheck|clang-tidy|complexity'` sur `.github/`, `scripts/`, `tools/` et tous les CMake ne renvoie rien, alors que six vérificateurs maison tournent déjà, trois en CI. Et poser la garde naïve serait pire que rien : mesuré sur cette branche, `bench_console.cpp` passe de 4 à 8 avertissements pendant que sa pire fonction tombe de CCN 118 à 12 — la seule garde utile est un cliquet sur le maximum par fonction, ou sur la somme de CCN par fichier.

**Le basculement du rail VSYS est à somme nulle à la cadence qui a servi à régler son seuil** — `ports/nrf52840/main.cpp:1596`
`restore_power_rails()` dépense 2 ms d'attente active (`nrf_pmu.cpp:768`) ; le commentaire crédite le mécanisme de « ~8 µA saved » et borne son coût à « 0,4 % time overhead », mais 0,4 % de 3,3 mA font ~13 µA — plus que le gain revendiqué. À la cadence immergée (1,0 s, `config_store.hpp:260`, qu'aucun des huit templates ne surcharge) c'est l'équilibre, et le seuil a précisément été abaissé de 5000 à 250 ms en prenant le cas immergé comme justification. Le même bloc décrit deux fois le rail comme « 3.3V↔1.8V » alors que le code commute à 2,3 V (`nrf_pmu.cpp:702`), ce qui empêche de recalculer.

---

## 4. LES FORCES

**Aucune extinction volontaire n'est plus à un relevé près.** `CriticalBandConfirm::SAMPLES=3` relevés sous le seuil espacés de `MIN_SPACING_MS=30 min` (`nrf_battery_mon.hpp:46-47`), soit ≥1 h, avec remise à zéro sur un seul relevé sain et un horodatage volontairement **non persisté** pour qu'une boucle de reboot ne puisse jamais confirmer ; plus une relecture à 2 min qui ramène en Operational (`gentracker.cpp:889-898`).

**La politique de brique est bornée *et* testée.** Cinq attentes `{5, 30, 120, 600, 3600}` s (`gentracker.cpp:202`), un seul `factory_reset` qui exige à la fois ≥5 échecs ET ≥30 min déjà passées à échouer (`gentracker.cpp:204-205`, `:301-307`), 13 entrées conservées (`config_store_fs.hpp:433-454`), et cinq tests dédiés (`tests/src/sm_test.cpp:310, 338, 374, 389, 427`) dont un qui exige `expectNoCall("powerdown")` sur 13 cycles d'échec consécutifs. Groupes Sm 28/28, ConfigStore 66/66.

**`storage_unusable()` ferme un piège définitif.** `main.cpp:433-460` remplace un `fault_blink_loop` infini qui *nourrissait* le chien de garde par 5 reboots annoncés puis une boucle WFI qui cesse délibérément de le nourrir, pour que le WDT 15 min reprenne la carte indéfiniment ; le compteur vit en `.noinit` sous magie `0x15250FA1` et est remis à zéro dès que la flash répond (`main.cpp:723-727`), donc une panne transitoire dans six mois retrouve tout son budget.

**La séquence d'extinction pilote l'ordre des fronts I2C.** TWIM désactivé au niveau registre, puis SCL tiré bas **avant** SDA, les deux laissés bas (`nrf_pmu.cpp:411-443`) — ce qui empêche le STC3117, alimenté sur VBAT pendant que tout le reste s'éteint, de rester armé en START toute la période off du TPL5111, blocage que seul un retrait de pile défaisait. Complété par une reprise de bus à 3 niveaux bornée en marche (`nrf_i2c.cpp:92-255`) qui refuse explicitement de couper VSENSORS parce que cela tue les pull-ups.

**Un backoff qui sature au lieu de déborder, avec le défaut chiffré dans son en-tête.** `core/util/exponential_backoff.hpp` remplace `base << (attempt-1)` par une boucle plafonnée et documente exactement ce qu'il supprime : avec la base de 60 s des deux services TX (60000 = 2⁵ × 1875) l'ancienne écriture renvoyait **exactement 0 à partir de la 28ᵉ tentative** — retry immédiat après vingt-huit échecs — et débordait la largeur du type à la 33ᵉ. Une définition, 3 tests, deux appelants qui portaient tous les deux la mauvaise écriture.

**Discipline de compilation vérifiable.** `-Wall -Wextra -Werror` plus 14 warnings supplémentaires (`ports/nrf52840/CMakeLists.txt:931-948`) pour seulement 4 suppressions ; build ARM réel lancé pendant l'audit, exit 0, zéro occurrence de `warning:` ; et les trois seules dérogations par fichier visent `libraries/` — aucun fichier de `core/` ou `ports/nrf52840/core/` n'a de waiver. `clang-format` 23.1.0 (la version épinglée en CI) : 219/227 fichiers du périmètre conformes, 8 non conformes nommables. La mise en garde du CLAUDE.md (« beaucoup de fichiers déjà non conformes sur main ») n'est plus vraie de ce périmètre.

**Six vérificateurs de cohérence qui comptent quelque chose de réel**, tous lancés à exit 0 : `check_param_tables.py` (271 slots, 3 tables alignées par index ET par nom), `check_template_conf.py` (255 paramètres, 8 fichiers — et il déclare honnêtement 8 lignes non vérifiables), `check_bench_log_patterns.py` (123 chaînes de log confrontées aux sources firmware), `check_english_logs.py`, `check_cached_modulation_writes.py`, `check_doc_links.py`. Trois tournent en CI (`tests.yml:112-130`) et le hook ne peut pas dériver : `.git/hooks/pre-commit` est une amorce de 353 octets qui `exec` la version du dépôt.

**Neuf `message(FATAL_ERROR)` interdisent les combinaisons dangereuses**, dont `ports/nrf52840/CMakeLists.txt:240` qui refuse `ENABLE_THERMISTOR_SENSOR` et `ENABLE_SEA_TEMP_SENSOR` ensemble parce qu'ils écrivent le **même** `m_sea_temp_cache.port[0]` avec deux encodages incompatibles (×1000+126 contre ×100+40), et `:718` qui refuse CAM et BUZZER qui partagent GPIO5. Plus 38 `static_assert` dans le périmètre.

**Le point le plus dangereux du produit est le mieux documenté, et la documentation est exacte.** `config_store.hpp:159-195` est un journal de décision en place de 37 lignes qui explique chaque génération du code de version, pourquoi `0x1F` a été bumpé et pourquoi `0x20` est conservé cinq fois de suite, avec la conséquence nommée (« would leave every provisioned LoRa unit unable to rejoin its network, with nothing in the logs to say why »). Sa dernière affirmation chiffrée — `__PARAM_SIZE 267 -> 271` — a été vérifiée contre `base_types.hpp:503` : exacte. Un `static_assert` à `:524` transforme un oubli en échec de compilation.

**Le dépôt a déjà trouvé et corrigé le défaut de mock que l'audit cherchait.** `tests/mocks/mock_pmu.cpp:60-62` : « It used to hardcode POWER_ON — which is exactly the value that hid the storage-wake bug, since a System OFF wake also decoded as POWER_ON » ; `reset_cause()` est désormais paramétrable. Même réflexe pour `SWSLongRun` (`tests.yml:207-211`, qui constate qu'il « n'atteignait aucun job de CI ») et pour le lien `data` relatif qui exposait 0 fichier (`tests/CMakeLists.txt:72-79`).

**Hygiène de fuite consciente des errata.** Les deux utilisateurs du SAADC font `uninit` + `nrf_peripheral_power_reset` sur **chaque** sortie, chemins d'erreur compris, en citant l'erratum 241 et ses 400 µA (`sws_analog_service.cpp:533`, `:539`, `nrf_battery_mon.cpp:256`), avec init/deinit **par échantillon** ; le QSPI sort par deep-power-down plus les deux écritures de l'erratum 122 avant `uninit`, puis flotte ses six broches (`is25_flash.cpp:508-556`).

---

## 5. CE QUE LA VÉRIFICATION A ÉCARTÉ

43 constats écartés, dont **36 à l'unanimité** des trois vérificateurs. Quatre biais se répètent, et le premier est de loin le plus coûteux.

### Biais 1 — prendre le coût de diagnostic pour un coût de fonctionnement

Le cas le plus net : **la même ligne a été remontée cinq fois, par cinq agents de cinq dimensions différentes** — `core/services/hauled_mode_service.cpp:196` — et les quinze vérificateurs ont tous réfuté. Chaque fois, la conséquence annoncée était « la balise bascule en LEGACY 30 min pour émettre sous l'eau ». Les deux moitiés sont fausses, et je les ai revérifiées :

- `ArgosTxService` et `LoRaTxService` ne surchargent pas `service_is_usable_underwater()` (défaut `false`, `core/services/service.hpp:203` — aucune surcharge dans leurs en-têtes), donc `notify_underwater_state(true)` les déprogramme et les annule (`service.cpp:421-429`) et `reschedule()` court-circuite tant que `m_is_underwater`. **Aucune émission sous l'eau n'est possible**, quel que soit le mode substitué.
- `HAULED_TR_NOM` vaut **7200 s (2 h)**, pas 1800 (table `config_store.hpp`, index [236]). Le 1800 vient d'un fixture de test : `tests/src/config_store_test.cpp:1630`.

Le fait sous-jacent est réel : la ligne 196 compare `now - last_uw_event_rtc`, estampé seulement à l'entrée en plongée (`:101` jette l'émersion), donc la grandeur inclut la durée de la plongée en cours — et trois textes l'appellent « dry ». Mais au seuil livré (24 h, index [233]) l'engagement sous l'eau est **inatteignable** : la SWS force une émersion après 3 × `UW_MAX_DIVE_TIME` = 3 × 7200 s (`sws_analog_constants.hpp:115`, `config_store.hpp:375`, `sws_analog_detection.cpp:787-795`), puis la re-plongée ré-estampe — plafond ~6 h contre un seuil de 24 h. Le coût réel d'un faux engagement est une perte de positions aux 2-3 émersions suivantes (GNSS éteint par `HAULED_GNSS_STRAT=REUSE_LAST`), auto-résorbée.

> **Correction directe de mon brief précédent.** J'avais écrit « le code d'abord, la config ensuite » et donné `hauled_mode_service.cpp:196` comme bloquant de code avant activation. C'est faux à la valeur livrée. La contrainte est une contrainte de **configuration** : `HAULED_IDLE_THRESHOLD_H` doit rester nettement au-dessus de 3 × `UW_MAX_DIVE_TIME`, soit ≥ 12 h ; et la valeur de banc 1 h (minimum DTE, `dte_params.cpp:423`) est un piège d'instrumentation, pas un défaut de code. `HAULED_DETECT_EN` peut être activé tel quel sous cette contrainte.

Autres instances du même biais : `ble_interface.cpp:522` et `smd_sat.cpp:346`, où `std::terminate` ne mène pas à `fault_blink_loop` (réservée aux handlers de faute, et seulement hors `NDEBUG`) mais à `abort()` qui se bloque dans `fputc` jusqu'au WDT 15 min — ce que le dépôt documente lui-même deux fois (`service.cpp:766-771`, `main.cpp:1692-1698`). Reboot et reprise de mission, pas brique. Et le `throw SPI_COMMS_ERROR` invoqué est du **code mort** : `SPI_USE_IRQ` n'est défini nulle part et sa branche ne compilerait pas (`event_handler` contre `spim_event_handler`, `nrf_spim.cpp:13` vs `:55`).

### Biais 2 — mesurer sur le binaire hôte et conclure sur l'ARM

- `config_store.hpp:723` : « 55 KB de flash » venait de `tests/build/TrackerTests`, dont `CMAKE_BUILD_TYPE` est **vide** — donc -O0, x86. Mesure au `nm` sur les ELF réels : 3 364 o (LINKIT), 5 080 o (RSPB), soit 0,5 % du `.text`. Facteur 11 à 16.
- `gentracker.cpp:390` : « 231 paramètres / 30,7 ko » est le build hôte, qui allume les neuf `ENABLE_*_SENSOR` ; le build tortue livré en a 178-181, soit ~24 ko. Et la durée « ~0,5 s » n'était pas mesurée, alors que le dépôt chiffre lui-même ~100 ms (`nrf_pmu.cpp:42`).
- `gps_service.cpp:1838` : « ~36 s » venait de multiplier 12 sondages par un budget *de palier* de 3 s. Chaque sondage est `initiate_timeout(500)` (`m10qasync_config.cpp:54`) : 6 s, et le code le dit (`m10qasync.cpp:73-74`).

**Règle** : tout chiffre de taille ou de durée doit nommer le binaire d'où il vient.

### Biais 3 — lire une métrique comme un jugement, et un produit cartésien comme un espace d'états

- `argos_tx_service.cpp:1338` : « 16 booléens = 65 536 états nominaux ». Au décompte, 10 sont hors machine (verrous de journalisation une-fois, descripteurs de charge utile pré-construite, comptabilité en vol, portes par boot) ; 5-6 sont réellement couplés et décodés dans **une** cascade ordonnée, chaque transition écrivant le jeu complet en bloc.
- `dive_behaviour_tracker.hpp:352` : « la fonction la plus dense du périmètre » — elle est 12ᵉ sur 119.
- `config_store.hpp:723` : la « chaîne de 18 branches » compile à 8 comparaisons sur la variante tortue, GCC insérant même une sortie de plage anticipée. lizard compte les `#if` comme points de décision — 211 des 3 132 CCN d'avertissement, et 10 avertissements sur 114 disparaissent une fois le préprocesseur retiré.
- Même effet, documenté dans les mesures elles-mêmes : le compteur d'avertissements punit le bon travail. C'est exactement pourquoi le constat survivant sur la garde de complexité demande un cliquet sur le maximum par fonction.

**Observation méta** : le même fait survit ou tombe selon la conséquence qu'on y attache. `detector_state` (`sws_analog_detection.cpp:177`) a survécu sous l'angle lisibilité et a été réfuté sous l'angle complexité — parce que la seconde version illustrait son risque par un couplage (`current_ma3` consommé 600 lignes plus loin) dont l'unique consommateur était un argument de `DEBUG_TRACE`, compilé hors du binaire à `DEBUG_LEVEL=3`. Un exemple rattrapé par le compilateur, donc le mode de défaillance le moins cher qui existe.

### Biais 4 — déclarer absent un garde-fou qui existe ailleurs, ou absente une décision qui est écrite

C'est la cause de la majorité des réfutations unanimes.

- `pmu.hpp:31` (contrat de `powerdown()` faux en build de banc) : **six** garde-fous indépendants, dont trois durs — `%STORE` refuse de s'exécuter sans `-DBENCH_ALLOW_POWERDOWN` (`bench_console.cpp:976-980`), `tests/bench/storage_bench.py:137` détecte la carte encore énumérée et nomme le flag, et le protocole de mesure mappe « > 5 mA » sur « la carte n'a jamais dormi ».
- `gentracker.cpp:491` (« formate au premier échec ») : la garde `!is_mounted()` a été ajoutée exprès en 2022 (`d9305776`, CG-410), le corps est mort sur cible parce que `init_storage()` est `[[noreturn]]` en cas d'échec (`main.cpp:432`) et `GenTracker::start()` est huit lignes plus loin, et le comportement est épinglé par quatre tests (`sm_test.cpp:166-212`).
- `gps_service.cpp:388` (« les dix renvois de ligne pointent tous à côté ») : un des dix est juste (`argos_tx_service.cpp:2892`), et huit des neuf faux portent le nom du symbole dans la même phrase. La population **décroît** (13 → 9 sur les derniers mois).
- `argos_tx_service.cpp:2598`, `:775`, `m10qasync_config.cpp:573`, `moored_mode_service.cpp:62` : dans chaque cas, une table unique ou un test nommé existait — `ModulationGuard::max_payload_bits` partagée par les gardes du service, le contrôle de configuration et le cadrage des pilotes, avec `frame_for_modulation` qui **refuse** au lieu de tronquer ; `DepthPileNtrySweepRotation` ; et `sws_analog_constants.hpp:171-190` qui documente la règle `offsetof`/`sizeof` avec son anti-exemple et le commit qui l'a corrigée.

### Quatre règles pour le prochain audit

1. **Suivre la valeur jusqu'à son consommateur avant d'écrire une conséquence.** Les cinq pires réfutations tombent parce que le consommateur était une ligne de journal, une LED, un masque d'avertissement consultatif ou du code mort.
2. **Nommer le binaire derrière chaque chiffre** de taille, de durée ou de courant.
3. **Distinguer ce qui est dans le binaire livré.** `bench_console.cpp` (8 fonctions, 174-205 de CCN) est sous `#ifdef BENCH_TEST` et dit lui-même « zero footprint in production builds » ; les branches `#if` des autres variantes ne sont pas compilées. Un grep ne le dit pas.
4. **Chercher la décision avant de supposer l'oubli.** Le dépôt documente ses arbitrages en place : `config_store.hpp:159-195`, `nrf_pmu.cpp:40-57`, `m10qasync_config.cpp:201-213`, `tools/dx1/README.md`, `sws_analog_constants.hpp:171-190` — et les épingle par des tests nommés.

---

## 6. CE QUI RESTE HORS DE PORTÉE

**Aucune carte n'a été flashée dans aucune session.** Aucun des cinq constats survivants n'est reproduit sur matériel ; tous sont établis par lecture de code, mesure statique ou test hôte.

### Il faut une mesure au PPK pour trancher

- **Le constat survivant E lui-même** (`main.cpp:1596`) : les ~8 µA revendiqués contre les ~13 µA du cycle de 2 ms sont de l'arithmétique sur des constantes de datasheet, pas une mesure.
- La fuite EXTINT-bas (75-375 µA), le courant réel en System OFF, et le portail d'allumage par aimant.
- GPIOTE configuré en mode IN event au lieu de l'événement PORT basse consommation (`ports/nrf52840/bsp/linkitv4_v1.0/bsp.cpp:42` et `:54`, `hi_accuracy = true`) : surcoût **permanent** s'il est réel, non chiffrable depuis l'arbre, et le BSP est hors périmètre de jugement.
- **Surtout** : l'énergie RF par émission est absente de l'arbre — ni datasheet KIM2, ni SMD/STM32WL, ni courant TCXO, ni courant d'attente module. Le poste dominant (73-78 % du budget) ne peut pas être converti en fraction de pack. C'est le plus gros trou de l'évaluation.

### Il faut une panne provoquée pour trancher

- **Le constat survivant A** : un vrai `apply` OTA, et le cas de l'image d'une autre variante poussée sur la carte (`ota_flash_file_updater.cpp:94`). Le bootloader n'est pas dans le périmètre jugé ; `dfu_ext_flash.c` n'a été lu que pour confirmer la conséquence bout-en-bout.
- La brique du rail IS25 à 2,3 V.
- Les deux constats en contexte d'interruption (fenêtres de quelques instructions) : il faudrait un défaut SPI/flash provoqué ou un build instrumenté.
- Le blocage I2C (SCL tenu bas par le STM32WL).

### Il faut un déploiement ou une balise récupérée

- Le comportement SWS en mer.
- L'efficacité réelle du gating prepass : le mécanisme existe et est orthogonal au mode (`argos_tx_service.cpp:444-466`), mais `SAT_PREPASS_EN = 0` dans les quatre templates tortue — zéro donnée de déploiement. Compté comme force de conception, pas comme économie acquise.
- `GNP52`.

### Ce que l'outillage interdit aujourd'hui

- **Aucune couverture de lignes ni de branches.** `grep -rn 'coverage\|gcov\|lcov\|-fprofile'` sur `tests/CMakeLists.txt`, `.github/workflows/` et `scripts/run_unit_tests.sh` ne rend que le mot dans un commentaire. Le « 100 % de `core/` » signifie « les 34 `.cpp` sont dans une cible hôte », pas « exercés » — un module compilé et instancié peut avoir 20 % de ses branches couvertes sans que rien ne le dise. Vaut aussi pour les 150 branches de `detector_state` et les 62 tests qui l'entourent.
- **Une variante compilée sur huit.** Le « zéro avertissement » ne vaut que pour LINKIT/KIM ; SMD_SPI, SMD_UART, LORA, Cyprus et les quatre RSPB peuvent porter des avertissements ou ne pas compiler. Le constat sur `build.yml:17` a été écarté 2/3, mais **son fait n'a pas été contesté** : `workflow_dispatch` est le seul déclencheur, et aucun job n'a jamais compilé le firmware.
- **`SWSLongRun` (53 min, 15 scénarios de 75 jours simulés) n'a été lancé par aucun agent.**
- lizard a tourné sur l'arbre complet, donc toutes les branches `#if` sont comptées simultanément : le CCN d'un binaire réellement construit est plus bas, de façon différente pour chacune des six variantes. Aucune mesure par variante.
- Pas de seconde métrique (complexité cognitive) pour arbitrer « complexe pour une raison » contre « complexe par accident » : le partage est une lecture étayée par des décomptes, pas un chiffre indépendant.
- L'accord commentaire/code n'est exhaustif que sur deux classes (renvois de ligne, défauts chiffrés) ; les affirmations de prose — « never », « always », « the only writer is », « cannot happen » — n'ont été contrôlées que sur une douzaine au hasard. Il en reste des centaines.

### Non audité, faute de temps

`core/sm/ledsm.cpp` (26 ko), le reste de `dte_handler.cpp`, `core/ble_services/`, `core/util/`, le chemin de streaming DFU SMD (`smd_sat.cpp firmware_update`, CCN 90 et 78), `cam_service`, `mortality_service`, `dive_mode_service`, les chemins propres à Cyprus, l'intérieur du détecteur SWS, les 85 blocs de duplication intra-fichiers, les 13 marqueurs TODO/FIXME/HACK, la qualité interne des harnais Python de banc (`tests/bench/` — hors périmètre de production, mais toute la validation matérielle en dépend), et la qualité des assertions au-delà du dénombrement et d'un sondage ciblé.

### Quatre questions laissées ouvertes

- L'allocateur FreeRTOS `heap_4.c` et son shim de région critique (`heap_mem.c`, `static uint8_t g_nested` remis à 0 à chaque entrée, donc non réentrant) : savoir si un `malloc` en ISR peut s'imbriquer dans un `malloc` du contexte principal dépend des priorités que `sd_nvic_critical_region_enter` masque, comportement SoftDevice que cet arbre ne permet pas de vérifier. **Aucun constat n'a été émis.**
- Le WDT nRF52840 continue-t-il réellement de tourner en System OFF ? L'ordre d'appel de `storage_off_check()` l'affirme en commentaire ; l'ordre est conservateur dans les deux cas, il a été laissé tel quel.
- Les 8 boucles `while (true)` du pilote `m10qasync` n'ont pas été prouvées sortantes une par une : le cadre arme bien des délais bornés autour (`Service::run_scheduled_task`, `on_service_timeout`, `m_task_timeout` armé en un seul endroit), mais pas chaque état interne.
- Le correctif de null-check du handler POF nrfx (`nrf_drv_power.c`) n'a pas pu être vérifié : `ports/nrf52840/drivers/` est hors périmètre. Le HardFault au débranchement USB reste donc non vérifié — sans conséquence sur une balise scellée, où l'USB n'est pas atteignable.