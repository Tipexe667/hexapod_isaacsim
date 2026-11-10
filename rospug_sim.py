"""
Simulateur PyBullet 3D temps réel pour le robot chien quadrupède Hiwonder ROSPug (12-DOF / Jetson Nano).
Fonctionnalités :
- Modèle CAO SolidCraft 62 corps (Châssis, Jetson Nano, LiDAR ToF, Caméra HD, 4 Pattes Bioniques à bielle parallèle)
- Cinématique Quadrupède 12-DOF : Allures Trot, Marche (Walk) et Amble (Pace)
- Postures Bioniques : Debout (Stand), Accroupi (Crouch), Assis (Sit), Salut (Play Bow) & Assiette IMU (Roulis/Tangage)
- LiDAR 360° ToF rotatif avec radar polaire 2D, faisceaux 3D et évitement d'obstacles autonome
- Caméra Frontale Grand-Angle HD 8MP avec flux vidéo temps réel (RGB & Profondeur Thermique 3D)
"""
import os
import sys
import time
import math
import pybullet as p
import pybullet_data

from rospug_gui import RosPugControlGUI
from sensor_widgets import ObstacleArenaManager, LidarScanner, OnboardCameraRenderer


GAIT_PHASES = {
    "TROT": {"FL": 0.0, "FR": math.pi, "RL": math.pi, "RR": 0.0},
    "WALK": {"FL": 0.0, "FR": math.pi, "RL": 0.5 * math.pi, "RR": 1.5 * math.pi},
    "PACE": {"FL": 0.0, "FR": math.pi, "RL": 0.0, "RR": math.pi},
}


def compute_rospug_joint_targets(state, gait_phase):
    """
    Calcule les consignes angulaires (rad) des 12 servomoteurs des 4 pattes (FL, FR, RL, RR)
    selon la posture, l'assiette (hauteur, roulis, tangage) et les vitesses (vx, vy, wz).
    """
    posture = state["posture"]
    gait = state["gait_mode"]
    vx = state["vx"]
    vy = state["vy"]
    wz = state["wz"]
    step_amp = state["step_amp"]
    body_z = state["body_z_rad"]
    roll = state["roll_rad"]
    pitch = state["pitch_rad"]

    phases = GAIT_PHASES.get(gait, GAIT_PHASES["TROT"])
    motion_mag = min(1.2, math.hypot(vx, vy) + 0.6 * abs(wz))
    is_moving = motion_mag > 0.03

    targets = {}
    for leg in ("FL", "FR", "RL", "RR"):
        is_front = leg.startswith("F")
        is_left = leg.endswith("L")
        s_front = 1.0 if is_front else -1.0
        s_left = 1.0 if is_left else -1.0

        # 1. Posture de base
        hip_angle = 0.0
        sh_angle = -body_z * 0.6
        kn_angle = body_z * 1.1

        if posture == "CROUCH":
            sh_angle -= 0.38
            kn_angle += 0.72
        elif posture == "SIT":
            if is_front:
                sh_angle += 0.18
                kn_angle -= 0.25
            else:
                sh_angle -= 0.58
                kn_angle += 0.95
        elif posture == "BOW":
            if is_front:
                sh_angle -= 0.55
                kn_angle += 0.92
            else:
                sh_angle += 0.16
                kn_angle -= 0.22

        # 2. Compensation d'assiette IMU (Roulis / Tangage)
        sh_angle += s_front * pitch * 0.55 + s_left * roll * 0.45
        kn_angle -= s_front * pitch * 0.85 + s_left * roll * 0.75
        hip_angle += roll * 0.35

        # 3. Cycle de marche quadrupède (Swing / Stance)
        if is_moving:
            phi = gait_phase + phases[leg]
            c_phi = math.cos(phi)
            lift = max(0.0, math.sin(phi))

            # Direction longitudinale + différentiel de lacet
            leg_vx = vx - s_left * 0.35 * wz
            stride = step_amp * max(-1.0, min(1.0, leg_vx))

            sh_angle += stride * c_phi - 0.32 * step_amp * lift * motion_mag
            kn_angle += 0.78 * step_amp * lift * motion_mag - 0.25 * stride * c_phi

            # Abduction latérale (Pas de crabe Vy + Rotation Wz)
            lat_cmd = vy + s_front * 0.35 * wz
            hip_angle += 0.28 * max(-1.0, min(1.0, lat_cmd)) * c_phi

        targets[f"Joint_Hip_{leg}"] = max(-0.60, min(0.60, hip_angle))
        targets[f"Joint_Shoulder_{leg}"] = max(-1.00, min(1.00, sh_angle))
        targets[f"Joint_Knee_{leg}"] = max(-1.10, min(1.10, kn_angle))

    return targets


def run_rospug_simulation():
    """
    Lance la simulation PyBullet + IHM Tkinter du robot chien Hiwonder ROSPug.
    Retourne :
      - "MENU" pour revenir au menu principal
      - "MENTORPI" pour basculer vers le MentorPi
      - "HEXAPOD" pour basculer vers l'Hexapode
      - None si fermeture normale
    """
    print("=================================================================")
    print(" 🐕 Démarrage de la simulation PyBullet - Hiwonder ROSPug 12-DOF")
    print("=================================================================")

    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.setRealTimeSimulation(0)

    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)
    p.resetDebugVisualizerCamera(
        cameraDistance=0.54,
        cameraYaw=42,
        cameraPitch=-24,
        cameraTargetPosition=[0.0, 0.0, 0.11]
    )

    plane_id = p.loadURDF("plane.urdf")
    p.changeDynamics(plane_id, -1, lateralFriction=1.1, spinningFriction=0.05, rollingFriction=0.01)

    urdf_path = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "rospug_solidcraft",
        "rospug_pybullet.urdf"
    )
    if not os.path.exists(urdf_path):
        print(f"[ERREUR] Fichier URDF introuvable : {urdf_path}")
        p.disconnect()
        return None

    start_pos = [0.0, 0.0, 0.004]
    start_orn = p.getQuaternionFromEuler([0.0, 0.0, 0.0])
    robot_id = p.loadURDF(urdf_path, start_pos, start_orn, useFixedBase=False)
    init_base_pos, init_base_orn = p.getBasePositionAndOrientation(robot_id)

    joint_name_to_id = {}
    link_name_to_id = {}
    num_joints = p.getNumJoints(robot_id)
    for i in range(num_joints):
        j_info = p.getJointInfo(robot_id, i)
        j_name = j_info[1].decode("utf-8")
        l_name = j_info[12].decode("utf-8")
        joint_name_to_id[j_name] = i
        link_name_to_id[l_name] = i
        if "Calf_Module" in l_name:
            p.changeDynamics(robot_id, i, lateralFriction=1.2)

    print(f"[INFO] Hiwonder ROSPug chargé avec {num_joints} liaisons URDF (12-DOF pattes + LiDAR).")

    arena_mgr = ObstacleArenaManager()
    arena_mgr.spawn_arena()

    lidar_scanner = LidarScanner(num_rays=72, max_range=2.20, min_range=0.09)
    cam_renderer = OnboardCameraRenderer(width=160, height=120, fov=70.0, near=0.05, far=3.50)

    switch_target = {"value": None}

    def on_gui_command(cmd):
        if cmd == "RESET_SIM":
            p.resetBasePositionAndOrientation(robot_id, init_base_pos, init_base_orn)
            p.resetBaseVelocity(robot_id, [0, 0, 0], [0, 0, 0])
            gui.cmd_stop()
        elif cmd == "TOGGLE_ARENA":
            arena_mgr.toggle_arena(gui.arena_var.get())
        elif cmd == "SWITCH_MENU":
            switch_target["value"] = "MENU"
        elif cmd == "SWITCH_MENTORPI":
            switch_target["value"] = "MENTORPI"
        elif cmd == "SWITCH_HEXAPOD":
            switch_target["value"] = "HEXAPOD"

    gui = RosPugControlGUI(on_command_change=on_gui_command)

    print("\n--- COMMANDES CLAVIER (Fenêtre 3D PyBullet) ---")
    print("  Flèches Haut/Bas        : Avancer / Reculer (Trot / Walk)")
    print("  Flèches Gauche/Droite   : Pivoter sur place (Lacet)")
    print("  Touches A / E           : Pas de crabe latéral Gauche / Droite")
    print("  Touches 1 / 2 / 3 / 4   : Postures (1=Debout, 2=Accroupi, 3=Assis, 4=Salut)")
    print("  Barre Espace            : STOP d'urgence")
    print("  Touche R                : Réinitialiser la position")

    dt = 1.0 / 240.0
    sim_time = 0.0
    gait_phase = 0.0
    step_counter = 0

    try:
        while p.isConnected():
            if switch_target["value"] is not None:
                break

            try:
                gui.root.update_idletasks()
                gui.root.update()
            except Exception:
                break

            state = gui.get_state()

            # Raccourcis clavier PyBullet
            keys = p.getKeyboardEvents()
            if ord(" ") in keys and keys[ord(" ")] & p.KEY_WAS_TRIGGERED:
                gui.cmd_stop()
                state = gui.get_state()
            if ord("r") in keys and keys[ord("r")] & p.KEY_WAS_TRIGGERED:
                on_gui_command("RESET_SIM")
                state = gui.get_state()
            if ord("1") in keys and keys[ord("1")] & p.KEY_WAS_TRIGGERED:
                gui.set_posture("STAND")
                state = gui.get_state()
            if ord("2") in keys and keys[ord("2")] & p.KEY_WAS_TRIGGERED:
                gui.set_posture("CROUCH")
                state = gui.get_state()
            if ord("3") in keys and keys[ord("3")] & p.KEY_WAS_TRIGGERED:
                gui.set_posture("SIT")
                state = gui.get_state()
            if ord("4") in keys and keys[ord("4")] & p.KEY_WAS_TRIGGERED:
                gui.set_posture("BOW")
                state = gui.get_state()

            if p.B3G_UP_ARROW in keys and (keys[p.B3G_UP_ARROW] & p.KEY_IS_DOWN):
                gui.set_vel(0.75, 0.0, 0.0)
            elif p.B3G_DOWN_ARROW in keys and (keys[p.B3G_DOWN_ARROW] & p.KEY_IS_DOWN):
                gui.set_vel(-0.75, 0.0, 0.0)
            elif p.B3G_LEFT_ARROW in keys and (keys[p.B3G_LEFT_ARROW] & p.KEY_IS_DOWN):
                gui.set_vel(0.0, 0.0, 0.90)
            elif p.B3G_RIGHT_ARROW in keys and (keys[p.B3G_RIGHT_ARROW] & p.KEY_IS_DOWN):
                gui.set_vel(0.0, 0.0, -0.90)
            elif ord("a") in keys and (keys[ord("a")] & p.KEY_IS_DOWN):
                gui.set_vel(0.0, 0.70, 0.0)
            elif ord("e") in keys and (keys[ord("e")] & p.KEY_IS_DOWN):
                gui.set_vel(0.0, -0.70, 0.0)

            base_pos, base_orn = p.getBasePositionAndOrientation(robot_id)
            base_euler = p.getEulerFromQuaternion(base_orn)
            base_yaw = base_euler[2]

            # 1. Rotation continue de la tourelle LiDAR ToF
            if "Joint_LiDAR_Spin" in joint_name_to_id:
                p.setJointMotorControl2(
                    robot_id,
                    joint_name_to_id["Joint_LiDAR_Spin"],
                    controlMode=p.VELOCITY_CONTROL,
                    targetVelocity=state["lidar_speed"],
                    force=0.5
                )

            # 2. Balayage LiDAR 360° (30 Hz)
            if step_counter % 8 == 0:
                if "LiDAR_Rotor_Turret" in link_name_to_id:
                    l_state = p.getLinkState(robot_id, link_name_to_id["LiDAR_Rotor_Turret"])
                    lidar_pos = [l_state[0][0], l_state[0][1], l_state[0][2] + 0.010]
                else:
                    lidar_pos = [base_pos[0] + 0.038, base_pos[1], base_pos[2] + 0.185]

                telemetry = lidar_scanner.scan(
                    sensor_pos=lidar_pos,
                    base_yaw=base_yaw,
                    draw_3d_rays=state["lidar_rays_3d"]
                )
                gui.radar_widget.update_radar(lidar_scanner, telemetry)

            # 3. Rendu Caméra Frontale Grand-Angle HD (20 FPS)
            if step_counter % 12 == 0 and state["cam_enable"]:
                cam_yaw = base_yaw
                cam_pitch = base_euler[1] - 0.05
                eye_pos = [
                    base_pos[0] + 0.145 * math.cos(base_yaw),
                    base_pos[1] + 0.145 * math.sin(base_yaw),
                    base_pos[2]
                ]
                ppm_bytes, center_depth = cam_renderer.render_ppm(
                    cam_pos=eye_pos,
                    cam_yaw=cam_yaw,
                    cam_pitch=cam_pitch,
                    mode=state["cam_mode"]
                )
                gui.camera_widget.update_image(ppm_bytes, center_depth)

            # 4. Consignes de vitesse (Manuel ou Évitement Autonome LiDAR)
            if state["auto_avoid"]:
                vx_cmd, vy_cmd, wz_cmd = lidar_scanner.compute_avoidance_vel(
                    forward_speed=0.42 * state["freq_scale"],
                    turn_speed=0.85 * state["freq_scale"],
                    safe_dist=0.58
                )
                state["vx"] = vx_cmd / 0.42
                state["vy"] = vy_cmd / 0.35
                state["wz"] = wz_cmd / 1.15
                state["posture"] = "STAND"
            else:
                vx_cmd = state["vx"] * 0.42
                vy_cmd = state["vy"] * 0.35
                wz_cmd = state["wz"] * 1.15

            # 5. Avancement de phase de marche & Servos 12-DOF
            gait_phase += 2.0 * math.pi * 2.4 * state["freq_scale"] * dt
            joint_targets = compute_rospug_joint_targets(state, gait_phase)

            for jname, target_rad in joint_targets.items():
                if jname in joint_name_to_id:
                    p.setJointMotorControl2(
                        robot_id,
                        joint_name_to_id[jname],
                        controlMode=p.POSITION_CONTROL,
                        targetPosition=target_rad,
                        force=12.0,
                        maxVelocity=14.0
                    )

            # 6. Stabilisation & Propulsion fluide du corps dans le repère monde
            lin_vel_world, _ = p.getBaseVelocity(robot_id)
            cy = math.cos(base_yaw)
            sy = math.sin(base_yaw)
            world_vx = vx_cmd * cy - vy_cmd * sy
            world_vy = vx_cmd * sy + vy_cmd * cy

            p.resetBaseVelocity(
                robot_id,
                linearVelocity=[world_vx, world_vy, lin_vel_world[2]],
                angularVelocity=[0.0, 0.0, wz_cmd]
            )

            p.stepSimulation()
            sim_time += dt
            step_counter += 1
            time.sleep(dt)

    except KeyboardInterrupt:
        print("\n[INFO] Arrêt demandé par l'utilisateur.")
    finally:
        try:
            gui.root.destroy()
        except Exception:
            pass
        if p.isConnected():
            p.disconnect()

    return switch_target["value"]


if __name__ == "__main__":
    res = run_rospug_simulation()
    if res == "MENU":
        import sim_launcher
        sim_launcher.main()
    elif res == "MENTORPI":
        import mentorpi_sim
        mentorpi_sim.run_mentorpi_simulation()
    elif res == "HEXAPOD":
        import hexapod_sim
        hexapod_sim.main()
