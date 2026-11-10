# Guide Complet de Simulation PyBullet : MentorPi (ROS2) & Hexapode 18-DOF

Ce projet fournit un **Centre de Simulation Robotique 3D** complet, léger et 100 % natif sous **Windows** (PyBullet + Tkinter), regroupant deux robots complets avec leurs interfaces de commande et leurs capteurs embarqués (**LiDAR 360°** et **Caméra 3D RGB-D**) :

1. **🤖 Hiwonder MentorPi (Raspberry Pi 5 / ROS2)** :
   - Modèle CAO **SolidCraft** 50 corps 100 % manifold (`closed: true`).
   - Bascule mécanique en temps réel entre **Châssis T1 (Chenilles Tank)** et **Châssis M1 (4 Roues Mecanum Omnidirectionnelles)**.
   - **Tourelle LiDAR 360° ToF (STL27L / LD19)** rotative avec écran **Radar Polaire 2D**, faisceaux laser 3D et **Exploration Autonome d'Évitement d'Obstacles**.
   - **Tourelle Caméra 3D Depth Pan-Tilt (2-DOF)** avec retour vidéo en direct (**Flux RGB Couleur** et **Carte de Profondeur 3D Thermique**).
2. **🕷️ Robot Hexapode 18-DOF (Dynamixel AX-18 / ABS)** :
   - Moteur de **Cinématique Inverse (IK) 3-DOF** analytique en temps réel sur 6 pattes.
   - Locomotion tripode omnidirectionnelle, contrôle d'assiette 6-DOF, **Saut Dynamique Explosif** et contrôle direct des 18 servomoteurs.
   - Équipé d'un onglet **📡 LiDAR 360° & Caméra 3D** avec radar polaire, caméra frontale RGB-D et marche autonome avec évitement d'obstacles.

---

## 1. Démarrage Rapide

### Option A : En 1 clic via le Menu de Sélection (Recommandé)
Double-cliquez sur :
👉 [**`run_pybullet.bat`**](run_pybullet.bat)

Le **Centre de Simulation Robotique** ([`sim_launcher.py`](sim_launcher.py)) s'ouvre et vous permet de choisir :
- **🚜 Lancer MentorPi — Mode T1 (Chenilles Tank)**
- **🛞 Lancer MentorPi — Mode M1 (Roues Mecanum)**
- **🕷️ Lancer le Robot Hexapode 18-DOF**

### Option B : En ligne de commande directe
```powershell
# Ouvrir le menu de choix graphique
py -3.12 sim_launcher.py

# Ou lancer directement un robot spécifique :
py -3.12 sim_launcher.py mentorpi_t1
py -3.12 sim_launcher.py mentorpi_m1
py -3.12 sim_launcher.py hexapod
```

> **Astuce** : Dans chaque simulation, les boutons situés en haut à droite de la télécommande (**`🔀 Menu Robots`**, **`🕷️ Basculer vers Hexapode`** ou **`🤖 Basculer vers MentorPi`**) permettent de changer de robot instantanément.

---

## 2. Poste de Commande du Hiwonder MentorPi ([`mentorpi_gui.py`](mentorpi_gui.py))

### Onglet 1 : « 🚗 Pilotage & Châssis (T1/M1) »
- **Configuration Mécanique du Châssis (Temps Réel)** :
  - **🚜 MentorPi T1 (Chenilles Tout-Terrain)** : Affiche les chenilles et barbotins, pilotage différentiel skid-steer ($V_x, \omega_z$).
  - **🛞 MentorPi M1 (4 Roues Mecanum)** : Affiche les 4 roues Mecanum, débloque le déplacement holonome 3-DOF incluant le pas de crabe latéral ($V_x, V_y, \omega_z$).
- **Pavé Directionnel & Curseurs de Vitesse** :
  - 8 directions, rotations sur place, arrêt d'urgence (`Espace`) et multiplicateur Turbo.
- **Tourelle Caméra 3D Pan-Tilt (2 Servos)** :
  - Curseurs **Pan / Lacet (`-90°` à `+90°`)** et **Tilt / Tangage (`-45°` à `+60°`)**.
  - Mode **🔄 Balayage Panoramique Auto (`Scan_Demo`)**.
- **Navigation Autonome & Arène 3D** :
  - Case **🤖 Activer l'Exploration Autonome avec Évitement d'Obstacles LiDAR**.
  - Case **🧱 Afficher l'Arène d'Obstacles 3D** (piliers et murs colorés autour du robot).

### Onglet 2 : « 📡 LiDAR 360° & Caméra 3D »
- **Radar Polaire 360° (STL27L / LD19)** :
  - Balayage temps réel sur 72 rayons (360°), affichage des échos colorés selon la distance (rouge < 0.55 m, orange < 1.0 m, vert > 1.0 m).
  - Télémétrie numérique par secteur (*Avant, Gauche, Droite, Arrière, Distance minimale*).
  - Option d'affichage des faisceaux laser 3D directement dans la scène PyBullet.
- **Caméra 3D Depth Embarquée** :
  - Retour vidéo en direct depuis la tête optique `Camera_Depth_3D_Head`.
  - Bascule instantanée entre **📷 Flux RGB Couleur** et **🌈 Profondeur 3D (Depth Thermique)** avec télémètre de distance au centre du réticule.

### Raccourcis Clavier MentorPi (Fenêtre 3D)
- <kbd>↑</kbd> / <kbd>↓</kbd> : Avancer / Reculer
- <kbd>←</kbd> / <kbd>→</kbd> : Pivoter à gauche / droite
- <kbd>A</kbd> / <kbd>E</kbd> : Déplacement latéral en crabe (Mode M1 Mecanum)
- <kbd>M</kbd> : Basculer entre Châssis T1 (Chenilles) et M1 (Mecanum)
- <kbd>J</kbd> / <kbd>L</kbd> et <kbd>I</kbd> / <kbd>K</kbd> : Orienter la caméra Pan / Tilt
- <kbd>Espace</kbd> : STOP d'urgence | <kbd>R</kbd> : Réinitialiser au centre

---

## 3. Poste de Commande du Robot Hexapode 18-DOF ([`hexapod_gui.py`](hexapod_gui.py))

1. **Onglet « 🚶 Marche & Virages »** : Pad directionnel 8 directions, virages fluides, pas de crabe latéral, curseurs $V_x, V_y, \omega_z$ et réglage de la foulée (fréquence, hauteur, longueur).
2. **Onglet « 📡 LiDAR & Caméra 3D »** :
   - Radar LiDAR 360° embarqué sur le dôme supérieur + rayons laser 3D.
   - **🤖 Marche Autonome Évitement LiDAR** : l'hexapode marche en tripode et évite seul les obstacles.
   - **📷 Caméra Frontale 3D (RGB & Profondeur Thermique)** orientable en Pan/Tilt.
3. **Onglet « 📐 Assiette (IK) »** : Contrôle temps réel de la hauteur $Z$, du Roulis, du Tangage et du Lacet pieds au sol.
4. **Onglet « ⚙️ 18 Servos »** : Pilotage individuel des 18 servomoteurs Dynamixel AX-18 (Coxa, Femur, Tibia).
5. **Onglet « ✨ Démos & Presets »** : Postures *Debout*, *Accroupi*, *Repos*, **🚀 Saut Dynamique (`J`)**, Banc d'essai en l'air et réglage de la gravité / couple AX-18.

---

## 4. Architecture des Fichiers

- [`sim_launcher.py`](sim_launcher.py) : Menu graphique principal de sélection de robot et bascule inter-simulations.
- [`sensor_widgets.py`](sensor_widgets.py) : Module partagé de capteurs (`LidarScanner`, `OnboardCameraRenderer`, `LidarRadarWidget`, `CameraFeedWidget`, `ObstacleArenaManager`).
- [`mentorpi_sim.py`](mentorpi_sim.py) & [`mentorpi_gui.py`](mentorpi_gui.py) : Simulateur PyBullet et poste de commande Tkinter du Hiwonder MentorPi.
- [`mentorpi_solidcraft/mentorpi_pybullet.urdf`](mentorpi_solidcraft/mentorpi_pybullet.urdf) : Modèle URDF multi-matériaux du MentorPi exporté depuis SolidCraft (`mentorpi.sc`).
- [`hexapod_sim.py`](hexapod_sim.py), [`hexapod_kinematics.py`](hexapod_kinematics.py) & [`hexapod_gui.py`](hexapod_gui.py) : Simulateur PyBullet, cinématique inverse 3-DOF et télécommande de l'Hexapode 18-DOF.
- [`run_pybullet.bat`](run_pybullet.bat) : Lanceur Windows en 1 clic.
