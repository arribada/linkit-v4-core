# Modèles par projet

Quel modèle de ce dossier sert quel projet, et en quoi il diffère de ce qui
tourne réellement. Établi le 29/10/2026 à partir de l'export de la plateforme
(`config_templates`, `device_configs`, en lecture seule, secrets masqués) :
c'est la base de la revue des configurations, pas une décision.

## Projet → modèle

| Projet | Balises | Build | Modèle | D'où viennent ses valeurs |
|---|---|---|---|---|
| Cyprus-Boats | 10 LinkIt LoRa, bateaux au mouillage | `build_linkitv4_lora_cyprus.sh` | `cyprus_boat.cfg` (campagne de seuils : `cyprus_boat_campaign.cfg`) | Conçu sur la campagne de La Réunion et au banc (09/2026). La flotte tourne sur le modèle de plateforme Cyprus-test-2, fait après : [15 écarts](#cyprus-boats) |
| WWF-YanaRiba, Argos | 8 LinkIt SMD, tortues olivâtres | `build_linkitv4_smd.sh` | `turtle_argos_surface.cfg` | Conçu en 10/2026 (remontées courtes, CloudLocate MEAS20). Chaque balise de la flotte a été réglée à la main en v4.1.8 : [écarts](#wwf-yanariba-argos) |
| WWF-YanaRiba, LoRa | 1 LinkIt LoRa (Caipirina) | `build_linkitv4_lora.sh` | `turtle_lora.cfg` (nouveau) | La configuration de Caipirina, deux lignes corrigées ; délai SWS aux défauts, comme les autres modèles tortue |
| RSPB Mortality | cartes RSPB, oiseaux | `build_rspb.sh` | `rspb_mortality.cfg` (nouveau) | Le modèle de plateforme RSPB Mortality V2, aucune valeur changée |
| ARRIBADA-CapVerde | 10 SMD autonomes, Doppler | firmware SMD autonome | aucun ici | Pas ce firmware : les modèles « Standalone » de la plateforme décrivent le module SMD seul |

Sans projet en production : `turtle_gps.cfg`, `turtle_cloudlocate.cfg`,
`turtle_doppler_only.cfg`, `drifter.cfg`, `fix_beacon.cfg`.

Les deux nouveaux modèles reprennent les valeurs de production ; une valeur n'y
est changée que pour un défaut établi ou une décision déjà prise, et la ligne le
dit (`CORRIGE`, ou le commentaire du délai SWS). Une recommandation qui n'est ni
l'un ni l'autre est marquée `A REVOIR`, et la valeur reste celle de production.

## Ce que l'export a montré

1. **Le Config Tool n'écrit pas la langue de PyLinkit.** Il écrit des codes fil
   (`ARGOS_MODE = 2`), des secondes (`GNSS_DELTATIME_ACQ = 300`), `true` /
   `false`, le nom du firmware pour certains paramètres, et trois noms que ni
   PyLinkit ni le firmware ne connaissent :

   | Config Tool | PyLinkit | Firmware (clé) |
   |---|---|---|
   | `LB_TRESHOLD` | `LB_THRESHOLD` | `LB_THRESHOLD` (LBP02) |
   | `UW_PIN_SAMPLE_DELAY` | `UW_PIN_SAMPLE_DELAY_US` | `UW_PIN_SAMPLE_DELAY_US` (UNP08) |
   | `GNSS_ASSISTNOW_ORBIT_MAX_ERR` | `GNSS_ORBIT_MAX_ERROR` | `GNSS_ORBMAXERR` (GNP41) |

   Un fichier de la plateforme ne se pousse donc pas tel quel avec
   `pylinkit_cli config push` : PyLinkit ignorerait ces trois lignes sans le
   dire, et les codes seraient refusés. Les modèles de ce dossier sont traduits.
2. **`ARGOS_DEPTH_PILE = 12` est ambigu.** 12 positions si le Config Tool garde
   la profondeur, 24 s'il garde le code fil (le code 12 vaut 24). À relire sur
   une balise de Chypre : `PARMR ARP16`.
3. **Cyprus-test-2 a un seuil accéléromètre de 0,94 g**, qu'aucune balise
   n'utilise : 7 sont à 0,094 g, Cyprus 05 à 0,4 g. Le modèle de la plateforme
   est à corriger.
4. **Les modèles SMD autonomes de la plateforme portent en clair la clé Argos
   et l'identité de la carte** qui a servi à les créer (masquées dans l'export).
5. **Un masque horaire vide ne rend pas la balise muette** hors `DUTY_CYCLE` :
   aucun autre mode ne le lit. Toute la flotte YanaRiba a émis avec un masque à
   0. Le vérificateur affirmait l'inverse ; corrigé, comme le README et un
   commentaire de `cyprus_boat.cfg`.
6. **BeiDou partout.** `GNSS_CONSTELLATION_MASK = 15` sur Chypre, sur 7 balises
   YanaRiba sur 8 et sur RSPB, avec AssistNow activé. Sur le M10, BeiDou B1C
   désactive AssistNow : le premier réglage annule le second. Les modèles
   tortue sont à 7 ; à revoir pour Chypre et RSPB.
7. **YanaRiba : le format CloudLocate suit la RCONF maîtresse.** MEAS20 sur les
   4 balises à maîtresse LDA2 (Bachata, Caprisun, Ovni, Pili), MEASC12 sur les
   4 à maîtresse LDK (Gabrielle, Lucienne, Tu'uga, wwf-turtle-smd-5) : MEAS20
   (192 bits) ne tient pas dans une trame LDK. `turtle_argos_surface.cfg` pose
   MEAS20, il demande donc une maîtresse LDA2 (README, « The depth pile only pays
   off on LDA2 »).
8. **YanaRiba : le limiteur d'émission était actif** (`RATE_LIMIT_EN = 1`, 10 à
   20 émissions par heure selon la balise). Les modèles l'ont coupé : aucun frein
   n'est activé par défaut, c'est un choix par projet.

## Cyprus-Boats

Flotte = les 8 balises écrites depuis Cyprus-test-2 (Cyprus 01-03, 05, 07-10).
Exclues : Cyprus 04 (ancien modèle Cyprus-test, pas réécrite depuis le 15/09) et
Cyprus 06 (firmware « dirty », accéléromètre coupé, jamais vue sur la
plateforme).

| Paramètre | Flotte | `cyprus_boat.cfg` |
|---|---|---|
| `GNSS_ACQ_TIMEOUT` | 180 | 90 |
| `GNSS_MIN_NUM_FIXES` | 5 | 10 |
| `MOORED_AXL_HOLDOFF_S` | 900 | 1800 |
| `MOORED_DLOC` | 30 min | 60 min |
| `MOORED_TR_NOM` | 1900 | 3600 |
| `AXL_SENSOR_PERIODIC` | 600 | 0 |
| `AXL_SENSOR_WAKEUP_THRESH` | 0,094 ×7, 0,4 ×1 | 0,094 |
| `AXL_SENSOR_WAKEUP_SAMPLES` | 5 ×5, 2 ×3 | 5 |
| `LORA_DR` | 2 ×3, 0 ×3, 1 ×2 | 3 |
| `ARGOS_DEPTH_PILE` | 12 (ou 24, voir 2.) | 3 |
| `LB_THRESHOLD` | 5 | 10 |
| `LB_CRITICAL_THRESH` | 1 | 5 |
| `LB_GNSS_ACQ_TIMEOUT` | 240 | 120 |
| `LB_GNSS_HACCFILT_THR` | 100 | 300 |
| `LB_GNSS_HDOPFILT_THR` | 5 | 10 |

`cyprus_boat.cfg` justifie la plupart de ses valeurs (10 fixes contre le
multipath, maintien de 1800 s contre les fausses sorties répétées) ; la flotte
ne dit pas pourquoi elle s'en écarte. `LORA_DR` dépend de la distance à la
passerelle : c'est un réglage par balise.

## WWF-YanaRiba, Argos

Flotte = les 8 balises Argos, chacune réglée à la main. Quand elles diffèrent,
la colonne donne chaque valeur et son nombre de balises. Tu'uga a le GNSS coupé.

| Paramètre | Flotte | `turtle_argos_surface.cfg` |
|---|---|---|
| `ARGOS_TX_REPETITION` | 30 ×5, 60 ×3 | 60 |
| `ARGOS_NTRY_PER_MESSAGE` | 3 ×7, 2 ×1 | 2 |
| `ARGOS_DEPTH_PILE` | 1 | 4 |
| `ARGOS_TX_JITTER_EN` | 0 | 1 |
| `ARGOS_TCXO_WARMUP_TIME` | 0 | 2 |
| `SURFACING_BURST_STEP` | 5 ×5, 10 ×3 | 10 |
| `SURFACING_BURST_MAX_MSG` | 5 ×5, 3 ×3 | 6 |
| `RATE_LIMIT_EN` | 1 | 0 |
| `GNSS_ENABLE` | 1 ×7, 0 ×1 | 1 |
| `GNSS_ACQ_TIMEOUT` | 60 ×4, 30 ×3, 120 ×1 | 60 |
| `GNSS_COLD_ACQ_TIMEOUT` | 120 ×5, 60 ×3 | 90 |
| `GNSS_NTRY` | 3 ×6, 1 ×2 | 2 |
| `GNSS_FIX_MODE` | 2D ×7, AUTO ×1 | AUTO |
| `GNSS_HDOPFILT_ENABLE` | 0 ×6, 1 ×2 | 0 |
| `GNSS_HACCFILT_ENABLE` | 0 ×6, 1 ×2 | 0 |
| `GNSS_CONSTELLATION_MASK` | 15 ×7, 1 ×1 | 7 |
| `GNSS_MIN_ELEVATION` | 10 | 5 |
| `GNSS_SESSION_SINGLE_FIX` | 0 | 1 |
| `GNSS_ANO_STALE_DAYS` | 35 ×6, 33 ×1, 5 ×1 | 5 |
| `GNSS_ORBIT_MAX_ERROR` | 500 ×7, 300 ×1 | 300 |
| `GNSS_REUSE_FIX_MAX_AGE_S` | 86400 | 300 |
| `GNSS_DEEP_IDLE_AFTER_OFF_S` | 600 ×3, 300, 500, 601, 900, 0 | 1800 |
| `GNSS_FASTLOC_MODE` | 2 ×7, 0 ×1 | 2 |
| `GNSS_CLOUDLOCATE_FORMAT` | MEAS20 ×4, MEASC12 ×4 (voir 7.) | MEAS20 |
| `GNSS_CLOUDLOCATE_ALWAYS` | 1 ×7, 0 ×1 | 1 |
| `UW_DRY_TIME_BEFORE_TX` | 0 | 5 |
| `MIN_SURFACE_CYCLE_INTERVAL` | 120 ×3, 60 ×2, 30 ×2, 0 ×1 | 0 |
| `COOLDOWN_TRIGGER_MODE` | 0 | 3 |
| `LB_EN` | 0 | 1 |
| `LB_THRESHOLD` | 10 | 15 |
| `LB_ARGOS_MODE` | LEGACY | DOPPLER |
| `LB_ARGOS_TX_REPETITION` | 240 | 600 |
| `LB_GNSS_EN` | 1 | 0 |
| `LED_MODE` | 24HRS | OFF |
| `ARGOS_RX_EN` | 1 | 0 |

La flotte porte aussi les valeurs SWS du terrain (`UW_PIN_SAMPLE_DELAY_US` 500,
`SWS_DELAY_MIN` 200, `SWS_DELAY_MAX` 1000) que le modèle laisse aux défauts du
firmware (1000, 200, 10000), conformément à la décision du 27/10/2026.

## À trancher en revue

- **Chypre** : la référence est-elle la flotte (Cyprus-test-2) ou
  `cyprus_boat.cfg` ? Puis corriger le seuil de Cyprus-test-2 sur la plateforme,
  et relire `ARP16` sur une balise.
- **YanaRiba** : redéployer sur `turtle_argos_surface.cfg` suppose des
  maîtresses LDA2 partout (ou MEASC12 sur les LDK) ; limiteur et basse batterie
  à décider.
- **Tortue LoRa** : `LB_EN = 0` et des délais GNSS de 30 s, froid compris.
- **RSPB** : la constellation (6.).
- **Plateforme** : les trois noms du Config Tool, et les secrets des modèles SMD
  autonomes.
