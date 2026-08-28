# Documentation du Module Teleplot

## Présentation
Le module `Teleplot` fournit une bibliothèque légère, modulaire et optimisée pour envoyer des données de télémétrie en temps réel vers l'outil de visualisation [Teleplot](https://teleplot.fr/) (extension VS Code ou navigateur).

---

## 1. Fichiers du Module
- `lib/Teleplot/teleplot.h` : Déclarations des fonctions de télémétrie
- `lib/Teleplot/teleplot.cpp` : Implémentation série optimisée
- `lib/Teleplot/teleplot_usage.md` : Guide d'utilisation et exemples

---

## 2. Inclusion dans votre code
```cpp
#include "teleplot.h"
```

---

## 3. Guide des Fonctions

### A. Valeurs Simples avec Unités Optionnelles
Envoie une grandeur scalaire avec un timestamp et une unité physique affichée sur l'axe Teleplot.
```cpp
uint32_t now = millis();

// Sans unité : >speed:123456:150.5
teleplot_print("speed", 150.5f, now);

// Avec unité : >temperature:123456:25.6§°C
teleplot_print("temperature", 25.6f, now, "°C");
```

---

### B. Groupes de Courbes (Superposition sur un même graphique)
Permet de regrouper plusieurs courbes sur la même fenêtre graphique en utilisant la syntaxe `groupe/nom`.
```cpp
uint32_t now = millis();

// Graphique "Speed" avec 2 courbes :
teleplot_print_group("Speed", "Belt", belt_speed, now, "mm/s");
teleplot_print_group("Speed", "Steps", steps_speed, now, "mm/s");

// Graphique "Lift_Height" comparant Mesure et Consigne :
teleplot_print_group("Lift_Height", "Measured", lift_height, now, "mm");
teleplot_print_group("Lift_Height", "Setpoint", setpoint, now, "mm");
```
*Format généré :* `>Speed/Belt:123456:150.2§mm/s`

---

### C. Télémétrie d'États Textuels (`|t`)
Affiche des états machine, modes de fonctionnement ou alarmes sous forme de texte ou de ruban d'états dans Teleplot.
```cpp
uint32_t now = millis();

// Affichage d'état simple : >Status/Mode:123456:AUTO|t
teleplot_print_text("Mode", "AUTO", now, "Status");

// Changement d'état de l'encodeur : >Status/Encoder:123456:BELT|t
teleplot_print_text("Encoder", coils[0] ? "STEPS" : "BELT", now, "Status");
```

---

### D. Trajectoires et Graphiques 2D XY (`|xy`)
Permet de tracer des courbes 2D X/Y (par exemple : profil hauteur vs angle, ou trajectoire).
```cpp
uint32_t now = millis();

// Format généré : >Kinematics/Profile:123456:45.3:320.5|xy§mm
teleplot_print_xy("Profile", angle_deg, height_mm, now, "Kinematics", "mm");
```

---

### E. Tableaux de Données Instantanées (Même Timestamp)
Envoie plusieurs valeurs issues de capteurs synchrones en une seule ligne.
```cpp
uint32_t now = millis();
float currents[] = {4.05f, 12.30f, 19.85f};

// Format généré : >Sensors/DACs:123456:4.05;123456:12.30;123456:19.85§mA
teleplot_print_array("DACs", currents, 3, now, "Sensors", "mA");
```

---

### F. Streaming Haute Vitesse par Paquets (Batch 200 Hz)
Idéal pour capturer **100% des points haute fréquence (200 Hz)** à débit réduit (**115 200 bauds**) en éliminant la répétition des en-têtes.
```cpp
#define BATCH_SIZE 10
uint32_t timestamps[BATCH_SIZE];
float raw_speeds[BATCH_SIZE];

// Remplissage du buffer sur 10 ticks de 5ms...
// Émission du paquet de 10 points :
// Format : >Speed/Belt_Raw:1000:150.2;1005:151.0;...;1045:150.8§mm/s
teleplot_print_batch("Speed/Belt_Raw", raw_speeds, timestamps, BATCH_SIZE, "mm/s");
```

---

## 4. Tableau Récapitulatif des Formats

| Type de Donnée | Fonction C++ | Format Série Émis |
| :--- | :--- | :--- |
| **Scalaire simple** | `teleplot_print("var", val, t, "u")` | `>var:t:val§u` |
| **Groupe de courbes** | `teleplot_print_group("G", "var", val, t, "u")` | `>G/var:t:val§u` |
| **Texte / État** | `teleplot_print_text("var", "texte", t, "G")` | `>G/var:t:texte\|t` |
| **Graphique 2D XY** | `teleplot_print_xy("var", x, y, t, "G", "u")` | `>G/var:t:x:y\|xy§u` |
| **Tableau instantané** | `teleplot_print_array("var", arr, N, t, "G", "u")` | `>G/var:t:a;t:b;t:c§u` |
| **Paquet Batch 200Hz** | `teleplot_print_batch("G/var", arr, t_arr, N, "u")` | `>G/var:t0:v0;t1:v1;...§u` |
