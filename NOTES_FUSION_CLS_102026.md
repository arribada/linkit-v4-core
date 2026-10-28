# Fusion de `dev/cls_102026` dans `v4.2.1-optim` — ce qui change pour le woodbuoy

**Pour :** l'équipe firmware de CLS (Susanna Polino), afin de vérifier sur un woodbuoy ce que
la fusion a modifié dans votre code.
**Branche :** `v4.2.1-optim` d'arribada/linkit-v4-core. Fusion `8863fe3a` du 28/10/2026,
complétée par le commit qui la suit sur la même branche. Les 12 commits de
`dev/cls_102026` y sont présents tels quels dans l'historique ; seules les résolutions de
fusion et les corrections listées ci-dessous s'y ajoutent.

**En une phrase :** en fonctionnement normal, la trame woodbuoy (en `001` comme en `101`), la
mesure de pression et le pilotage du baromètre sont **identiques** à votre firmware. Ce qui
change touche les cas d'erreur, la batterie, les lignes de journal et le reste de la branche,
que la fusion vous apporte.

---

## 1. Vos 12 commits : ce qui a été pris, adapté ou remplacé

| Commit | Sujet | Dans la fusion |
|---|---|---|
| `4f157cf7` | script KIM | **Pris.** `ENABLE_SWS_ANALOG=0` et `CAM_ENABLE=0` par défaut dans `build_linkitv4_kim.sh` ; une tortue KIM se construit désormais avec `ENABLE_SWS_ANALOG=1`. La commande cmake garde aussi nos options (`DEBUG_LEVEL`, `BENCH_ALLOW_POWERDOWN`) |
| `d8348a47` | script woodbuoy | **Pris**, plus une option batterie (§2.8) |
| `832b9a0f` | compilation SEA_TEMP | **Pris** |
| `4484d227` | trame capteur woodbuoy | **Pris**, une correction (§2.1) |
| `4579d432` | température de mer × 1000 | **Déjà présent** de notre côté (`0b7ac2ee`, 10/09) : même formule, avec en plus la division par 10 dans la trame LoRa |
| `650170e2` | baromètre PTB110 / ADS1115 | **Pris**, corrections §2.2 à §2.5 ; paramètres déplacés (§2.6) |
| `c01c1ccf` | lignes de journal en TRACE | **Remplacé** par notre niveau `DEBUG_DETAIL` (§2.9) |
| `2b9279c4` | cause de reset « pseudo power-on » | **Déjà corrigé** de notre côté, avec des marqueurs GPREGRET explicites (0x80 demande de stockage, 0x81 en stockage) ; même décodage (PSEUDO avant SREQ) |
| `98fafb92` | buzzer et geste aimant | **Pris** tel quel |
| `4caeb8e4` | TX du KIM2 à l'extinction | **Remplacé** par notre correctif de la même fuite (§2.7) ; le désabonnement du module éteint est pris |
| `f0136b9c` | identité du KIM2 au démarrage | **Pris**, une correction (§2.7) |
| `c93ce7fd` | mesures de batterie | **Remplacé**, votre garantie conservée par une option (§2.8) |

---

## 2. Ce qui a été modifié dans votre code

Chaque point dit ce qui change **en fonctionnement normal** et ce qui change **dans les cas
d'erreur**.

### 2.1 Trame woodbuoy « température seule » (en-tête `001`)

`ArgosPacketBuilder::build_sensor_packet()` lisait `baro_sensor->port[0]` même quand
`baro_sensor` était nul. C'est le cas de chaque trame `001` : baromètre désactivé (BAP01 à 0,
la valeur par défaut) ou pas encore mesuré. C'est un comportement indéfini en C++. Votre
branche, compilée ici avec les options de `build_woodbuoy.sh` (arm-none-eabi-gcc 10.2.1,
10-2020-q4), le traduit par une lecture de 8 octets à l'adresse 0 (`ldrd r0, r1, [r6]` avec
r6 = 0). C'est la table de vecteurs du MBR, lisible sans faute matérielle. Ces 8 octets
(0x20000400, 0x00000A81 dans la S140 7.2.0) forment un double dénormalisé, 5,7·10⁻³¹¹, que
`__aeabi_d2uiz` convertit en 0.
- **Maintenant :** ces 15 bits sont mis à 0 explicitement, sans lecture à l'adresse 0.
- **Trame transmise : identique**, en `001` comme en `101`. Vos trames `001` portaient déjà 0.
- **Vérifier :** sur une trame `001` reçue, les bits 99 à 113 valent 0.

### 2.2 Encodage de la pression : borné aux 15 bits

`(hPa − 800) × 100` est désormais borné à 0..32767 (`baro_encoded()`, `depth_pile.cpp`).
- **De 800 à 1127,67 hPa** : **valeur identique bit pour bit** (même calcul en double, même
  troncature). Cela couvre toute la plage du PTB110 : 800 à 1060 hPa pour 0 à 2,5 V.
- **Sous 800 hPa** : 0, comme avant. Sur le nRF, la conversion d'un double négatif en entier
  non signé (`__aeabi_d2uiz`) donnait déjà 0.
- **Au-delà de 1127,67 hPa** (plus de 3,15 V en entrée, hors de la plage du PTB110, donc
  capteur en défaut) : 32767. Avant, `pack_bits` gardait les 15 bits de poids faible et la
  valeur repartait de 800 hPa : la lecture −1 du §2.3, soit 1652 hPa, partait comme
  996,59 hPa, une pression plausible.

### 2.3 ADS1115 : lecture en entier signé

`sample_adc()` renvoie un `int16_t`, qui est le format du composant (complément à deux).
- **Entre 0 et 4,096 V** : identique.
- **Juste sous 0 V**, capteur éteint ou bruit : avant, −1 était lu 65535, soit 8,19 V et
  1652 hPa, transmis comme 996,59 hPa (§2.2). Maintenant c'est une tension négative, encodée 0.

### 2.4 Export CSV du journal BARO

`BAROLogFormatter` utilisait `%d` sur un `double` : la colonne contenait le mot de poids
faible du double lu comme un entier, un nombre sans rapport avec la pression. C'est
maintenant `%.2f`. L'enregistrement binaire en flash ne change pas.

### 2.5 Garde-fous de compilation (aucun effet sur un build woodbuoy)

- La pile et le cache du baromètre (`DepthPileManager`), ainsi que le bloc baromètre de
  `get_argos_configuration()`, sont sous `#if ENABLE_BARO_SENSOR`, comme les autres capteurs.
- CMake refuse `ENABLE_BARO_SENSOR` hors LinkIt, et sur un SMD en SPI : P0.06, qui porte
  `BARO_PWR_EN`, y est le MOSI du module.
- `[[maybe_unused]]` sur les paramètres de `build_sensor_packet()` que chaque format
  n'utilise pas.

### 2.6 Paramètres BAP01–BAP05 : emplacements 266–270 → 283–287

Les emplacements 266 à 282 étaient pris de notre côté (UNP26 à DBP02).
- **Mise à jour vers ce firmware** : `config.dat` est maintenant relu **par clé**. Les valeurs
  BAP d'une bouée configurée par votre firmware sont retrouvées à leur nouvel emplacement.
  Le code de version de configuration est le même des deux côtés, donc pas d'effacement.
- ⚠️ **Retour arrière vers votre firmware** : votre relecture est encore **positionnelle**.
  Elle attend la clé de BAP01 à l'emplacement 266 et y trouve UNP26 ; sur un `config.dat`
  écrit par ce firmware, elle remet donc BAP01 à BAP05 à leur valeur par défaut, baromètre
  désactivé. Les emplacements 0 à 265 sont relus normalement. Intégrer cette branche chez
  vous supprime le problème.
- Les clés et encodages sont inchangés ; il reste à les ajouter dans PyLinkit.

### 2.7 KIM2 : coupure du module

- `sync_power_up()`, `sync_power_down()` et `identify()` sont gardés tels quels.
  `sync_power_down()` coupe maintenant le rail par `cut_module_rail()`, qui met la TX au
  **niveau bas** avant la coupure.
- Votre `4caeb8e4` traitait la même fuite (environ 43 µA) en passant la TX en haute impédance
  après `deinit()`. Les deux méthodes l'évitent. La nôtre est mesurée au PPK sur LinkIt KIM
  (70 → 27 µA, mode BLIND compris), et chacune des six coupures du rail y passe.
- Votre `park_tx_for_power_off()` portait le même nom que notre fonction existante, avec un
  comportement différent et l'instance UART 1 en dur : un seul mécanisme a été gardé.
- Le `catch` du constructeur (sonde d'identité qui lève une exception) coupe aussi par
  `cut_module_rail()`.
- **Vérifier :** le courant de veille du module éteint, qui doit rester au niveau de votre
  firmware.

### 2.8 Batterie

Votre `c93ce7fd` supprimait la mesure dans les accesseurs de configuration (sauf la lecture
de POT03 et POT06), ainsi que celle de l'entrée GPS d'un fix, qui reprend la mesure du début
de session. Restaient, avec une mesure à chaque appel : le début de session GNSS, les entrées
GPS sans fix, dégradée et CloudLocate, l'émission Doppler, et le contrôle de batterie critique
que fait la planification Argos en mode batterie basse. Notre branche avait traité le même
problème autrement : une mesure est réutilisée tant qu'elle a moins de 30 min. STATR (mesure de
moins de 10 s), SENSR, un changement de seuil LBP02 ou LBP12, et un échec de mesure forcent
une nouvelle mesure.
- **Pour le woodbuoy** : `build_woodbuoy.sh` pose `BATTERY_SAMPLE_EACH_GNSS_SESSION=1`. Le
  service GNSS mesure alors **aux mêmes points que votre firmware** : une mesure fraîche au
  début de chaque session et dans chaque entrée sans fix, dégradée ou CloudLocate ; l'entrée
  d'un fix reprend la mesure du début de session. La tension écrite dans la trame vient de
  cette entrée : elle est donc prise au même moment que chez vous. Vérifié dans le binaire
  woodbuoy : ces quatre points appellent la mesure fraîche, le fix la mesure réutilisable.
- **Ce qui reste différent :**
  - en mode batterie basse, le contrôle critique de la planification réutilise une mesure de
    moins de 30 min au lieu d'en prendre une, et il exige un verdict confirmé (point suivant) ;
  - une émission Doppler (mode `DOPPLER` ou `SURFACING_BURST`) réutilise la dernière mesure ;
  - les accesseurs de configuration mesurent quand la dernière mesure a plus de 30 min. Si
    les sessions GNSS sont espacées de plus de 30 min, cela ajoute au plus une mesure par
    demi-heure, et l'état de la batterie reste à jour entre deux sessions.

  La trame woodbuoy elle-même (`process_sensor_burst`) ne prend de mesure ni chez vous ni
  ici : sa tension vient de l'entrée GPS. Une mesure coûte une attente active de 500 ms sur
  KIM (`BAT_ADC_SETTLE_MS`).
- **Nouveau pour vous (`c3ba3cac`)** : le verdict « batterie critique », qui finit en extinction,
  exige trois mesures basses espacées d'au moins 30 min, sans mesure saine entre elles. Votre
  firmware le rendait sur trois mesures consécutives, qui pouvaient toutes tomber dans une
  même rafale d'émission, sous charge, sur des piles alcalines. Et le contrôle de la
  planification Argos en mode batterie basse, qui chez vous éteignait la bouée sur une seule
  mesure sous le seuil, exige maintenant ce verdict.

### 2.9 Lignes de journal

Vos passages en `DEBUG_TRACE` ont été remplacés par notre niveau `DEBUG_DETAIL`. Les deux
gardent ces lignes hors du journal flash, mais `DETAIL` reste visible sur la console de banc,
alors que TRACE est retiré du binaire au niveau 3.

Quatre lignes d'étape restent écrites en INFO dans `system.log` : `M10Q on`,
`boot sync at`, `next GNSS acquisition` et `TX SUCCESS`. Elles servent au banc et à
`log_harvest.py`.

Pour ne garder en flash que les WARN et les ERROR, le nouveau paramètre **DBP02**
(`SYSTEM_LOG_LEVEL`) à 2. Il couvre le besoin de `c01c1ccf` : une journée tortue simulée
écrit alors 3 lignes.

---

## 3. Ce qui ne change pas — vérifié

- **Modulation.** Sans modulation adaptative (`ARP54=0`) et avec un maître LDK (`IDP14`), la
  trame woodbuoy part en LDK exactement comme chez vous. Le choix du RCONF
  (`load_rconf_for_mode`) est identique, et le compteur de messages ajouté de notre côté est
  sans effet sur un KIM2. La seule différence concerne un maître d'une autre modulation. La
  trame woodbuoy est demandée en LDK en dur (`process_sensor_burst`) ; avec un maître LDA2, le
  KIM2 de votre firmware abandonnait donc chaque émission, celui-ci recadre les mêmes 16
  octets en LDA2 et les émet. Avec un maître VLDA4 (3 octets), les deux refusent la trame.
- **Préchauffage de 2 s du PTB110** : inchangé. Le rendre non bloquant (lecture planifiée 2 s
  plus tard) a été écarté. La fin de session GNSS appelle aussi `sensor_handler()`, qui lit le
  capteur sur-le-champ. Aujourd'hui, le blocage garantit que cette lecture vient après le
  préchauffage ; en non bloquant, une session GNSS finie en moins de 2 s ferait lire un
  capteur pas encore stabilisé.
- **Mode périodique du baromètre (BAP02 > 0)** : inchangé. L'alimentation reste allumée après
  la première mesure périodique, comme dans votre firmware (voir §4).
- **ADS1115** : configuration inchangée (`0xE2 0xE3`).

---

## 4. Recommandations, non appliquées faute de matériel

1. **ADS1115 en conversion continue.** `MSB_CONFIG = 0xE2` met MODE à 0, alors que le
   commentaire dit « single conversion ». Le composant est configuré une seule fois au
   démarrage, alors que `BARO_PWR_EN` est à 0. Il est donc sur une alimentation permanente
   (sinon la configuration échouerait sur un NACK et le baromètre serait « not detected »). Il
   convertit en permanence : environ 150 µA (valeur typique de la fiche TI), soit 3,6 mAh par jour.
   En conversion unique (MODE à 1, OS à 1 écrit avant chaque lecture, environ 1,2 ms à
   860 SPS), il tomberait à environ 0,5 µA entre deux mesures. À confirmer sur votre schéma.
2. **Mode périodique** : l'alimentation du PTB110 n'est jamais recoupée après une mesure
   périodique (`sensor_service_terminate()` n'est appelé qu'en fin d'échantillonnage pour
   l'émission et en cas d'erreur). Si vous utilisez BAP02, le capteur reste alimenté en
   permanence.
3. **Attente active de 2 s** à chaque mise sous tension du PTB110 : l'ordonnanceur est bloqué
   pendant ce temps, y compris pendant une session GNSS.
4. **En-tête `101` sur 16 octets** : même en-tête et même taille que la trame RSPB Short. Rien
   dans la trame ne les distingue ; le décodeur doit choisir selon le produit de la balise
   (wiki, page 11 §F.16). L'en-tête `001` sur 16 octets se distingue de la trame capteur
   générique (24 octets) par sa taille.

---

## 5. Ce que la branche vous apporte au-delà de vos commits

Liste courte. Le détail est dans `CAMPAGNE_TEST_AVANT_MERGE.md` et dans les messages de
commit.
- **Pile réservée de 28 Ko au lieu de 32.** La RAM ne suffisait plus après la fusion : chaque
  paramètre coûte environ 216 o de RAM sur tous les builds. Pic de pile mesuré par le firmware
  (« STACK: N max ») : 4,2 à 4,5 Ko. Analyse statique : environ 6,4 Ko pour la prédiction de
  passage (`prepas`, 5,6 Ko à elle seule), environ 5 Ko pour la réception Argos, 2 à 3 Ko pour
  l'OTA par Bluetooth. Marge RAM du woodbuoy : 4780 o. Commande de banc `%MEM` pour lire pile
  et tas.
- **Relecture de `config.dat` par clé** (voir §2.6).
- **Pilote KIM2 et UART** : un envoi attend la fin du DMA précédent au lieu d'être refusé
  (`54332af5`, un AT+TX se perdait à l'émersion), verrou « busy » de l'UART corrigé
  (`9d2f339d`, `ed8ac932`), barrière d'exception autour des pilotes (`a8edf75d`, `cb0640a1`),
  modulation vérifiée après reset (`c5de1408`), fuite de la TX (`34ebb4a1`).
- **Journal** : niveau `DEBUG_DETAIL`, une ligne par étape, paramètre DBP02.
- **Capteurs** (`4a6a87fb`) : une lecture de réveil, à la fin d'une session GNSS par exemple,
  ne termine plus un service de capteur qui n'avait pas commencé. Avant, chacune écrivait un
  WARN dans le journal en flash et pouvait libérer la retenue de journal d'un autre service.
- **Batterie** : réutilisation 30 min, verdict critique étalé, garde de plausibilité de la
  mesure (`39259277`).
- **OTA** : zone réservée de 1 Mo en flash externe. L'image woodbuoy fait 650 Ko, la zone
  applicative interne 889 Ko. Les structures `.noinit` qui survivent au reset sont toutes
  validées par un CRC ou un magique avant d'être relues. Entre votre firmware et celui-ci, la
  section `.noinit` a changé d'adresse (0x20034AC0 → 0x200378B8, sans recouvrement) : après la
  mise à jour, chaque structure trouve à sa place un reste de votre pile, le rejette et repart
  de zéro, comme après une mise sous tension.

---

## 6. À vérifier sur un woodbuoy

1. Construire avec `./scripts/build_woodbuoy.sh`, puis vérifier dans la sortie
   `BATTERY_SAMPLE_EACH_GNSS_SESSION=1` et `ENABLE_BARO_SENSOR=1`.
2. Mettre à jour par OTA une bouée configurée par votre firmware, puis `PARMR` complet :
   BAP01–BAP05 et tous les autres paramètres doivent être intacts.
3. Au démarrage :
   - le baromètre est détecté (pas de « EXT BAROMETER: not detected ») ;
   - le KIM2 est alimenté une seule fois, ID et ADDR sont lus, et rien n'est émis.
4. Session GNSS puis émission : trame `101` décodée, pression et température plausibles, et
   tension batterie mesurée à cette session.
5. Avant la première mesure du baromètre : la trame `001` est décodée sans erreur, bits 99 à
   113 à 0 (§2.1).
6. Alimentation du PTB110 coupée après l'échantillonnage pour l'émission (BAP03 différent
   d'OFF), courant de veille du KIM2 éteint au niveau de votre firmware.
7. Buzzer : carillon au démarrage par l'aimant, phases du geste d'extinction.
8. Après quelques jours : la ligne « STACK: N max » du journal (toutes les 12 h, DBP02 à 3,
   la valeur par défaut) reste loin sous 28 Ko.
