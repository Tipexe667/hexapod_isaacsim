# Guide d'utilisation ROS 2 & Gazebo Sim (Hexapode)

Ce package a été adapté pour **ROS 2 Humble** et **Gazebo Sim** (Harmonic / Fortress via `ros_gz` et `gz_ros2_control`).

---

## 1. Dépendances requises

Installez les dépendances ROS 2 nécessaires sur votre machine Ubuntu / WSL :

```bash
sudo apt update
sudo apt install -y \
  ros-humble-ros-gz \
  ros-humble-ros-gz-sim \
  ros-humble-ros-gz-bridge \
  ros-humble-gz-ros2-control \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-joint-state-publisher-gui \
  ros-humble-robot-state-publisher \
  ros-humble-rviz2
```

---

## 2. Compilation du workspace

Placez ce package dans le dossier `src/` de votre workspace ROS 2 (ex: `~/ros2_ws/src/`) :

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select hexapod_isaacsim
source install/setup.bash
```

---

## 3. Lancement

### A. Simulation complète dans Gazebo + RViz2 + ros2_control

```bash
ros2 launch hexapod_isaacsim gazebo.launch.py
```

*Arguments disponibles :*
- `rviz:=false` : désactive l'ouverture de RViz2 (simulation Gazebo seule).
- `z:=0.2` : ajuste la hauteur d'apparition du robot au-dessus du sol.

### B. Inspection visuelle seule dans RViz2 (avec sliders)

Pour vérifier les articulations sans lancer Gazebo :

```bash
ros2 launch hexapod_isaacsim display.launch.py
```

---

## 4. Contrôle du robot

### A. Vérifier les contrôleurs actifs

```bash
ros2 control list_controllers
```

Vous devriez voir :
- `joint_state_broadcaster` [active]
- `joint_trajectory_controller` [active]
- `forward_position_controller` [inactive]

### B. Obtenir les états articulaires et capteurs

- États des articulations :
  ```bash
  ros2 topic echo /joint_states
  ```
- Données de l'IMU :
  ```bash
  ros2 topic echo /imu
  ```

### C. Basculer sur le contrôleur de commande directe (Forward Command Controller)

Si vous implémentez un moteur de cinématique inverse en temps réel :

```bash
ros2 control switch_controllers --activate forward_position_controller --deactivate joint_trajectory_controller
```

Puis envoyez vos consignes sur le topic :
```bash
ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```

---

## 5. Fichiers clés

- [`urdf/hexapod_gazebo.urdf`](urdf/hexapod_gazebo.urdf) : Description du robot avec balises Gazebo (frictions, IMU, plugin `gz_ros2_control`) sans altérer la cinématique ni les maillages d'origine.
- [`config/ros2_controllers.yaml`](config/ros2_controllers.yaml) : Configuration des contrôleurs `ros2_control` pour les 18 servomoteurs.
- [`launch/gazebo.launch.py`](launch/gazebo.launch.py) : Script de lancement principal Gazebo + ponts ROS 2.
- [`launch/display.launch.py`](launch/display.launch.py) : Script de visualisation rapide dans RViz2.
- [`rviz/hexapod.rviz`](rviz/hexapod.rviz) : Configuration RViz préconfigurée (RobotModel, Grid).
