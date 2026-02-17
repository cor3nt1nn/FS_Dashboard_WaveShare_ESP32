# Formula Student Dashboard - ESP32-S3

Tableau de bord temps réel pour la voiture de Sigma Racing pour la Formula Student 2026. Le système affiche la télémétrie du véhicule, calcule la consommation énergétique et estime l'autonomie restante pour optimiser la stratégie de course.

## Vue d'ensemble

Ce projet implémente un système de télémétrie embarqué pour la voiture de Sigma Racing pour la Formula Student 2026. Il utilise un ESP32-S3 couplé à un écran RGB 800x480 pour afficher en temps réel les données critiques du véhicule et assister le pilote dans la gestion énergétique.

### Caractéristiques principales

- Communication CAN bus pour la réception des données véhicule
- Calculs temps réel de consommation et d'autonomie
- Affichage graphique haute résolution (800x480)
- Enregistrement de toutes les trames CAN sur carte SD
- Architecture multi-thread pour performances optimales
- Gestion thread-safe des données partagées

## Architecture matérielle

### Plateforme

- **Microcontrôleur** : ESP32-S3 DevKitC-1
- **Fréquence CPU** : 240 MHz (dual-core)
- **Mémoire** : PSRAM activée
- **Écran** : 800x480 RGB parallèle (interface 16-bit RGB565)
- **Stockage** : Carte SD via interface SD_MMC
- **Bus CAN** : Contrôleur TWAI intégré à 500 kbit/s

### Configuration des broches

#### Interface écran RGB (16 données + 4 contrôle)

- **Données RGB** : GPIO 14, 38, 18, 17, 10, 39, 0, 45, 48, 47, 21, 1, 2, 42, 41, 40
- **Contrôle** : HSYNC (GPIO 46), VSYNC (GPIO 3), DE (GPIO 5), PCLK (GPIO 7)
- **Rétroéclairage** : GPIO 2

#### Bus CAN

- **TX** : GPIO 20
- **RX** : GPIO 19

#### Carte SD (SD_MMC)

- **CLK** : GPIO 12
- **CMD** : GPIO 11
- **DATA0** : GPIO 13

## Architecture logicielle

Le système repose sur une architecture multi-thread utilisant FreeRTOS pour paralléliser les opérations critiques et maintenir une interface utilisateur réactive.

### Modules (threads RTOS)

#### 1. Module CAN (canTask)

- **Priorité** : 2 (haute)
- **Core** : 1
- **Stack** : 4096 octets
- **Rôle** : Réception des trames CAN et mise à jour de l'état véhicule

Ce module gère la réception des messages CAN via le contrôleur TWAI et met à jour la structure `CarState` de manière thread-safe. Il tourne en boucle avec un délai minimal (1 ms) pour garantir une latence minimale dans la réception des données.

#### 2. Module Calcul (calculationTask)

- **Priorité** : 1 (moyenne)
- **Core** : 1
- **Stack** : 4096 octets
- **Période** : 100 ms
- **Rôle** : Calculs énergétiques et d'autonomie

Ce module effectue tous les calculs dérivés qui ne sont pas directement fournis par le bus CAN. Il s'exécute à intervalles réguliers de 100 ms pour calculer la distance parcourue, l'énergie consommée/régénérée, la consommation moyenne et l'autonomie restante.

#### 3. Module Interface (uiTask)

- **Priorité** : 1 (moyenne)
- **Core** : 0
- **Stack** : 8192 octets
- **Période** : 100 ms
- **Rôle** : Mise à jour de l'affichage graphique

Ce module rafraîchit l'interface graphique LVGL en lisant les données de `CarState` et en mettant à jour les widgets visuels. Il inclut le gestionnaire de timers LVGL (`lv_timer_handler`) nécessaire au fonctionnement de la bibliothèque graphique.

#### 4. Module Logger (loggerTask)

- **Priorité** : 0 (basse)
- **Core** : 0
- **Stack** : 4096 octets
- **Rôle** : Enregistrement des trames CAN sur carte SD

Ce module gère l'enregistrement asynchrone de toutes les trames CAN reçues sur carte SD au format CSV. Il utilise une queue de 500 entrées pour tamponner les messages et éviter la perte de données lors des écritures SD.

### Synchronisation et thread-safety

Le système utilise plusieurs mécanismes pour garantir la cohérence des données entre threads :

- **Variables atomiques** : Pour les données scalaires simples (vitesse, accélérateur, frein)
- **Mutex FreeRTOS** : Pour les structures complexes (données batterie, températures, énergie)
- **Timeout** : Tous les verrous ont un timeout de 10 ms pour éviter les blocages
- **Queue FreeRTOS** : Pour la communication asynchrone vers le logger (500 entrées)

## Données véhicule

### Identifiants CAN étendus (29-bit)

Toutes les trames utilisent des identifiants étendus au format :

| ID CAN     | Description           | Données        | Format        |
| ---------- | --------------------- | -------------- | ------------- |
| 0x18010001 | Position accélérateur | Uint16 (‰)     | Little-endian |
| 0x18010002 | Position frein        | Uint16 (‰)     | Little-endian |
| 0x18010003 | Vitesse véhicule      | Uint16 (dkm/h) | Little-endian |
| 0x18020001 | Tension batterie      | Uint16 (cV)    | Little-endian |
| 0x18020002 | Courant batterie      | Int16 (cA)     | Little-endian |
| 0x18030001 | État de charge (SOC)  | Uint16 (‰)     | Little-endian |
| 0x18030002 | Température batterie  | Int16 (d°C)    | Little-endian |
| 0x18030003 | Température moteur    | Uint16 (d°C)   | Little-endian |
| 0x18030004 | Température onduleur  | Uint16 (d°C)   | Little-endian |

### Unités et facteurs d'échelle

Le système utilise des entiers pour tous les calculs afin d'éviter les opérations flottantes coûteuses. Les conversions se font au moment de l'affichage.

#### Unités brutes (stockage interne)

- **Vitesse** : déci-kilomètres par heure (dkm/h) → 1 dkm/h = 0.1 km/h
- **Position accélérateur/frein** : pour-mille (‰) → 1 ‰ = 0.1%
- **Tension** : centi-volts (cV) → 1 cV = 0.01 V
- **Courant** : centi-ampères (cA) → 1 cA = 0.01 A
- **SOC** : pour-mille (‰) → 1 ‰ = 0.1%
- **Températures** : déci-degrés Celsius (d°C) → 1 d°C = 0.1 °C
- **Distance** : mètres (m)
- **Énergie** : milliwatt-heures (mWh) → 1 mWh = 0.001 Wh
- **Puissance** : milliwatts (mW) → 1 mW = 0.001 W
- **Consommation** : centi-watt-heures par kilomètre (cWh/km) → 1 cWh/km = 0.01 Wh/km
- **Delta** : centimètres (cm)

#### Unités affichage

Les valeurs sont converties en unités standard pour l'affichage :

- Vitesse : km/h
- Position accélérateur/frein : %
- Tension : V
- Courant : A
- SOC : %
- Températures : °C
- Distance : km
- Consommation : Wh/km
- Delta : m

### Structure CarState

La structure `CarState` centralise toutes les données véhicule et calculs dérivés. Elle est organisée en sous-structures protégées par mutex :

#### BatteryElectrical

- `voltage_cV` : Tension en cV
- `amperage_cA` : Courant en cA (positif = consommation, négatif = régénération)
- `powerOutput_mW` : Puissance instantanée en mW

#### BatteryState

- `soc_permil` : État de charge en ‰
- `temperature_dC` : Température en d°C
- `energyAvailable_mWh` : Énergie disponible en mWh

#### TemperatureData

- `engine_dC` : Température moteur en d°C
- `inverter_dC` : Température onduleur en d°C

#### EnergyData

- `tripDistance_m` : Distance parcourue depuis reset en m
- `energyConsumed_mWh` : Énergie totale consommée en mWh
- `energyRegenerated_mWh` : Énergie totale régénérée en mWh
- `energyConsumption_cWhKm` : Consommation moyenne en cWh/km
- `rangeRemaining_m` : Autonomie restante en m
- `energyDelta_cm` : Delta course en cm

## Calculs et algorithmes

### Calcul de puissance instantanée

La puissance est calculée lors de la réception du courant batterie :

```
P (mW) = V (cV) × I (cA) × 0.1
```

Avec :

- V : tension batterie en centi-volts
- I : courant batterie en centi-ampères (positif = décharge, négatif = charge)
- Facteur 0.1 : conversion (cV × cA = 0.0001 W = 0.1 mW)

### Calcul d'énergie disponible

L'énergie disponible est calculée lors de la réception du SOC :

```
E_disponible (mWh) = Capacité_batterie (mWh) × SOC (‰) × 0.001
```

Avec :

- Capacité batterie par défaut : 11 300 000 mWh (11.3 kWh)
- SOC en pour-mille

### Calcul de distance parcourue

Exécuté toutes les 100 ms dans `updateEnergyAndDelta()` :

```
d (m) = v (dkm/h) × Δt (ms) / 36000
```

Avec :

- v : vitesse instantanée en déci-kilomètres par heure
- Δt : intervalle de temps depuis dernier calcul en millisecondes
- Facteur 36000 : conversion (0.1 km/h × 0.001 h = m)

La distance n'est comptabilisée que si la vitesse dépasse 10 dkm/h (1 km/h) pour éviter les dérives à l'arrêt.

### Calcul d'énergie consommée/régénérée

Intégration de la puissance sur le temps :

```
ΔE (mWh) = P (mW) × Δt (ms) / 3 600 000
```

Avec :

- P : puissance instantanée en milliwatts
- Δt : intervalle de temps en millisecondes
- Facteur 3 600 000 : conversion (mW × ms → mWh)

Si ΔE > 0 : ajout à `energyConsumed_mWh`
Si ΔE < 0 : ajout de |ΔE| à `energyRegenerated_mWh`

### Calcul de consommation moyenne

Le système utilise un filtre de lissage exponentiel pour stabiliser la mesure de consommation.

#### Consommation instantanée

```
C_inst (cWh/km) = (E_nette (mWh) × 100) / d (m)
```

Avec :

- E_nette = E_consommée - E_régénérée
- d : distance totale parcourue en mètres

La consommation n'est calculée qu'après 800 m parcourus (constante `MIN_DISTANCE_M`). Avant ce seuil, une valeur par défaut de 410 Wh/km (41000 cWh/km) est utilisée.

#### Filtre de lissage exponentiel

Pour éviter les variations brusques, la consommation affichée est filtrée :

```
C_filtrée[n] = (α × C_inst[n] + (1-α) × C_filtrée[n-1])
```

Avec :

- α = 0.15 (15% de poids pour la nouvelle valeur)
- 1-α = 0.85 (85% de poids pour la valeur précédente)

Implémentation en arithmétique entière fixed-point (shift de 10 bits = division par 1024) :

```
α_fixed = 154    (154/1024 ≈ 0.15)
inv_α_fixed = 870    (870/1024 ≈ 0.85)

C_filtrée[n] = (α_fixed × C_inst[n] + inv_α_fixed × C_filtrée[n-1]) >> 10
```

Ce filtre offre un bon compromis entre réactivité et stabilité de l'affichage.

### Calcul d'autonomie restante

```
Autonomie (m) = (E_disponible (mWh) × 100) / C_filtrée (cWh/km)
```

Avec protection contre division par zéro : si C_filtrée < 10 cWh/km (0.1 Wh/km), alors Autonomie = 999 999 m.

### Calcul du delta course

Le delta représente la différence entre l'autonomie restante et la distance restante à parcourir. C'est l'indicateur clé pour la stratégie de course.

```
Delta (m) = Autonomie_restante (m) - Distance_restante_course (m)
```

Avec :

- Distance_restante = Distance_totale_course - Distance_parcourue
- Distance totale par défaut : 20 000 m (20 km)
- Clamping : -99 999 m ≤ Delta ≤ 99 999 m

**Interprétation** :

- Delta > 0 : Le véhicule peut parcourir plus que la distance restante (marge de sécurité)
- Delta = 0 : L'autonomie correspond exactement à la distance restante
- Delta < 0 : Le véhicule risque de ne pas finir la course (économie nécessaire)

## Optimisations de performance

Le système embarqué doit maintenir une interface fluide à 10 FPS tout en gérant la communication CAN et l'enregistrement SD. Plusieurs optimisations ont été mises en place :

### 1. Arithmétique entière

Tous les calculs utilisent des entiers 32 ou 64 bits au lieu de flottants. Les conversions en float ne se font qu'au moment de l'affichage. Cela réduit considérablement le temps de calcul sur l'ESP32.

### 2. Architecture multi-core

Le système utilise les deux cœurs de l'ESP32-S3 :

- **Core 1** : Traitement CAN et calculs (tâches temps réel)
- **Core 0** : Interface graphique et logging (tâches moins critiques)

### 3. Mise à jour conditionnelle de l'UI

Chaque élément graphique possède :

- Un intervalle de rafraîchissement minimum (ex: 200 ms pour la tension)
- Un seuil de variation minimum (ex: 1 V pour la tension)

L'élément n'est redessiné que si :

- L'intervalle minimum est écoulé ET
- La valeur a varié au-delà du seuil

Cela réduit drastiquement les appels LVGL coûteux.

### 4. Double buffering LVGL

Un buffer de dessin de 800×120 pixels est utilisé pour le rendu par bandes, optimisant l'utilisation de la mémoire tout en maintenant des performances acceptables.

### 5. DMA pour l'affichage

L'utilisation de `pushImageDMA()` permet le transfert des données d'affichage en arrière-plan via DMA, libérant le CPU pour d'autres tâches.

### 6. Logging asynchrone

Le logger utilise une queue FreeRTOS de 500 entrées. Les trames CAN sont ajoutées à la queue sans attente (timeout=0) et écrites par lots sur la SD lors de cycles dédiés. Cela évite de bloquer la réception CAN pendant les écritures SD lentes.

### 7. Flush et rotation intelligents

Les données SD sont flushées toutes les 1.5 secondes seulement (constante `FLUSH_INTERVAL_MS`) et les fichiers tournent toutes les 5 minutes (constante `FILE_ROTATION_MS`). Les écritures sont groupées par lots de 50 entrées maximum (`MAX_BATCH_SIZE`).

### 8. Configuration compilateur

Le fichier `platformio.ini` active plusieurs optimisations :

- `-O2` : Optimisation niveau 2
- `-mfix-esp32-psram-cache-issue` : Correction bug cache PSRAM
- `board_build.f_cpu = 240000000L` : CPU à fréquence maximale
- `CORE_DEBUG_LEVEL=0` : Désactivation des logs debug

## Logger CAN

Le module `CANLogger` enregistre toutes les trames CAN reçues sur carte SD au format CSV pour analyse post-course.

### Format de fichier

Les fichiers sont nommés selon le timestamp de création : `/canlog_<timestamp>.csv`

Format CSV :

```
Timestamp,ID,Ext,DLC,Data
1234,0x18010001,1,2,64,00
1235,0x18020001,1,2,E8,03
```

Colonnes :

- **Timestamp** : Temps en ms depuis démarrage
- **ID** : Identifiant CAN en hexadécimal
- **Ext** : 1 si trame étendue (29-bit), 0 si standard (11-bit)
- **DLC** : Nombre d'octets de données (0-8)
- **Data** : Octets de données en hexadécimal séparés par virgules

### Gestion de la queue

- **Capacité** : 500 trames
- **Comportement** : Non-bloquant, les trames sont perdues si la queue est pleine
- **Statistiques** : Compteurs de trames enregistrées et perdues

### Rotation de fichiers

Un nouveau fichier est créé toutes les 5 minutes pour :

- Limiter la taille des fichiers
- Faciliter la récupération en cas de crash
- Permettre l'analyse partielle pendant la course

### Performance

Le système peut gérer jusqu'à environ 1000 messages/seconde sans perte significative grâce à :

- La queue tampon de 500 entrées
- L'écriture par lots (50 entrées max par cycle)
- La priorité basse du thread logger (n'impacte pas les tâches critiques)

## Interface utilisateur

L'interface est conçue avec LVGL 8.3.6 et optimisée pour la lisibilité en conditions de course.

### Éléments principaux

#### Vitesse

- Police large, affichage central
- Mise à jour : 500 ms ou 1 km/h de variation

#### Accélérateur et frein

- Barres graphiques verticales
- Mise à jour : 100 ms ou 1% de variation

#### Batterie

- Barre horizontale avec pourcentage
- Couleurs : Rouge (≤15%), Blanc (16-74%), Vert (≥75%)
- Mise à jour : 2 s ou 1% de variation

#### Tension et courant

- Barres avec valeurs numériques
- Courant bidirectionnel : consommation (positif) et régénération (négatif)
- Mise à jour : 200 ms ou 1 unité de variation

#### Températures

- Batterie : Arc avec valeur numérique (rouge si >70°C)
- Moteur : Indicateur avec code couleur (vert <90°C, jaune <110°C, rouge ≥110°C)
- Onduleur : Indicateur avec code couleur (vert <90°C, jaune <110°C, rouge ≥110°C)
- Mise à jour : 2-3 s ou 1°C de variation

#### Delta

- Arc bicolore avec valeur numérique
- Couleurs : Vert (positif), Rouge (négatif), Blanc (nul)
- Plage : -20 km à +20 km
- Mise à jour : 2 s ou 0.1 km de variation

#### Consommation

- Valeur numérique en Wh/km
- Mise à jour : 2 s ou 0.1 Wh/km de variation

#### Distance parcourue

- Compteur kilométrique avec décimale
- Mise à jour : 1 s ou 0.095 km de variation

## Configuration et personnalisation

### Paramètres de course

Dans le constructeur de `CANSocket` :

```cpp
CANSocket::CANSocket(gpio_num_t tx, gpio_num_t rx, twai_timing_config_t timing)
    : _tx(tx), _rx(rx), _timing(timing), _state(20.0f, 10.2f) {}
```

Arguments de `_state` :

- Premier paramètre : Distance totale de course en km (défaut : 20 km)
- Second paramètre : Capacité batterie en kWh (défaut : 10.2 kWh)

### Constantes énergétiques

Dans `main.cpp` :

```cpp
constexpr int32_t ALPHA_FIXED = 154;           // Poids filtre (15%)
constexpr int32_t MIN_DISTANCE_M = 800;        // Distance min pour calcul conso
constexpr int32_t DEFAULT_CONSUMPTION = 41000; // Conso par défaut (410 Wh/km)
constexpr int32_t SPEED_THRESHOLD = 10;        // Seuil vitesse (1 km/h)
```

### Configuration CAN

Par défaut :

- Vitesse : 500 kbit/s
- Mode : Normal
- Filtre : Accepte tous les messages

Modifiable dans le constructeur de `CANSocket`.

## Compilation et déploiement

### Prérequis

- PlatformIO Core ou PlatformIO IDE
- Câble USB pour programmation ESP32-S3

### Commandes

```bash
# Compilation
pio run

# Upload sur la carte
pio run --target upload

# Monitoring série
pio device monitor
```

### Dépendances

Automatiquement gérées par PlatformIO :

- LVGL 8.3.6 : Bibliothèque graphique
- LovyanGFX 1.1.12 : Driver d'affichage optimisé
- ESP32_IO_Expander : Extension GPIO
- esp-lib-utils 0.2.0 : Utilitaires ESP32

## Licence et crédits

**Auteur** : BESQUEUT Corentin pour l'équipe Sigma Racing  
**Version** : 1.0  
**Date** : 31 janvier 2026

Développé pour la voiture de Sigma Racing pour la Formula Student 2026.
