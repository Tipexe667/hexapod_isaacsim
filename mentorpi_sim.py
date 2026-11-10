"""
Simulateur PyBullet 3D temps réel pour le robot Hiwonder MentorPi (ROS2 / Raspberry Pi 5).
Fonctionnalités :
- Modèle CAO SolidCraft 100% manifold (Châssis T1 Chenilles & M1 Roues Mecanum)
- Bascule dynamique en direct entre Châssis T1 (Skid-Steer) et Châssis M1 (Omnidirectionnel Mecanum)
- LiDAR 360° ToF (STL27L/LD19) rotatif avec radar polaire 2D, faisceaux 3D et évitement d'obstacles autonome
- Caméra 3D Depth sur tourelle Pan-Tilt 2-DOF avec flux vidéo temps réel (RGB & Profondeur Thermique)
"""
import os
import sys
import time
import math
import numpy as np
import pybullet as p
import pybullet_data

from mentorpi_gui import MentorPiControlGUI
from sensor_widgets import ObstacleArenaManager, LidarScanner, OnboardCameraRenderer


def apply_chassis_visibility(robot_id, link_name_to_idx, mode="T1"):
    """
    Bascule dynamiquement la visibilité entre :
    - Mode 'T1' : Modules de chenilles gauche/droite + 4 barbotins orange visibles, roues Mecanum masquées
    - Mode 'M1' : 4 roues Mecanum bleues visibles, modules de chenilles + barbotins masqués
    """
    is_t1 = (mode.upper() == "T1")
    track_links = ["Track_Module_Left", "Track_Module_Right"]
    sprocket_links = ["Track_Sprocket_FL", "Track_Sprocket_RL", "Track_Sprocket_FR", "Track_Sprocket_RR"]
    wheel_links = ["Mecanum_Wheel_FL", "Mecanum_Wheel_RL", "Mecanum_Wheel_FR", "Mecanum_Wheel_RR"]

    for lname in track_links:
        if lname in link_name_to_idx:
            lidx = link_name_to_idx[lname]
            # Visual shape 0 : gomme chenille, Visual shape 1 : galets acier
            p.changeVisualShape(
                robot_id, lidx, shapeIndex=0,
                rgbaColor=[0.110, 0.120, 0.135, 1.0 if is_t1 else 0.0]
            )
            try:
                p.changeVisualShape(
                    robot_id, lidx, shapeIndex=1,
                    rgbaColor=[0.722, 0.753, 0.784, 1.0 if is_t1 else 0.0]
                )
            except Exception:
                pass

    for lname in sprocket_links:
        if lname in link_name_to_idx:
            lidx = link_name_to_idx[lname]
            p.changeVisualShape(
                robot_id, lidx, shapeIndex=0,
                rgbaColor=[1.0, 0.42, 0.0, 1.0 if is_t1 else 0.0]
            )

    for lname in wheel_links:
        if lname in link_name_to_idx:
            lidx = link_name_to_idx[lname]
            p.changeVisualShape(
                robot_id, lidx, shapeIndex=0,
                rgbaColor=[0.227, 0.525, 1.0, 1.0 if not is_t1 else 0.0]
            )


def run_mentorpi_simulation(initial_chassis="T1"):
    """
    Lance la simulation PyBullet + IHM Tkinter du Hiwonder MentorPi.
    Retourne :
      - "MENU" si l'utilisateur demande à revenir au menu de sélection
      - "HEXAPOD" si l'utilisateur demande à basculer vers le robot Hexapode
      - None si l'utilisateur ferme la fenêtre normalement
    """
    print("=================================================================")
    print(" 🤖 Démarrage de la simulation PyBullet - Hiwonder MentorPi")
    print("=================================================================")

    physics_client = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.setRealTimeSimulation(0)

    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)
    p.resetDebugVisualizerCamera(
        cameraDistance=0.52,
        cameraYaw=42,
        cameraPitch=-26,
        cameraTargetPosition=[0.0, 0.0, 0.08]
    )

    plane_id = p.loadURDF("plane.urdf")
    p.changeDynamics(plane_id, -1, lateralFriction=1.0, spinningFriction=0.05, rollingFriction=0.01)

    urdf_path = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "mentorpi_solidcraft",
        "mentorpi_pybullet.urdf"
    )
    if not os.path.exists(urdf_path):
        print(f"[ERREUR] Fichier URDF introuvable : {urdf_path}")
        p.disconnect()
        return None

    start_pos = [0.0, 0.0, 0.005]
    start_orn = p.getQuaternionFromEuler([0.0, 0.0, 0.0])
    robot_id = p.loadURDF(urdf_path, start_pos, start_orn, useFixedBase=False)

    # Indexation des joints et links par nom
    joint_name_to_id = {}
    link_name_to_id = {}
    num_joints = p.getNumJoints(robot_id)
    for i in range(num_joints):
        j_info = p.getJointInfo(robot_id, i)
        j_name = j_info[1].decode("utf-8")
        l_name = j_info[12].decode("utf-8")
        joint_name_to_id[j_name] = i
        link_name_to_id[l_name] = i

    print(f"[INFO] MentorPi chargé avec {num_joints} liaisons URDF.")

    # Gestionnaire d'arène d'obstacles, LiDAR 360° et Caméra 3D embarquée
    arena_mgr = ObstacleArenaManager()
    arena_mgr.spawn_arena()

    lidar_scanner = LidarScanner(num_rays=72, max_range=2.20, min_range=0.09)
    cam_renderer = OnboardCameraRenderer(width=160, height=120, fov=65.0, near=0.05, far=3.50)

    switch_target = {"value": None}
    last_chassis_mode = {"value": initial_chassis.upper()}

    def on_gui_command(cmd):
        if cmd == "RESET_SIM":
            p.resetBasePositionAndOrientation(robot_id, start_pos, start_orn)
            p.resetBaseVelocity(robot_id, [0, 0, 0], [0, 0, 0])
            gui.cmd_stop()
            gui.center_camera()
        elif cmd == "CHASSIS_CHANGE":
            new_mode = gui.chassis_mode_var.get()
            apply_chassis_visibility(robot_id, link_name_to_id, new_mode)
            last_chassis_mode["value"] = new_mode
        elif cmd == "TOGGLE_ARENA":
            arena_mgr.toggle_arena(gui.arena_var.get())
        elif cmd == "SWITCH_MENU":
            switch_target["value"] = "MENU"
        elif cmd == "SWITCH_ROSPUG":
            switch_target["value"] = "ROSPUG"
        elif cmd == "SWITCH_HEXAPOD":
            switch_target["value"] = "HEXAPOD"

    gui = MentorPiControlGUI(on_command_change=on_gui_command, initial_chassis=initial_chassis)
    apply_chassis_visibility(robot_id, link_name_to_id, initial_chassis)

    print("\n--- COMMANDES CLAVIER (Fenêtre 3D PyBullet) ---")
    print("  Flèches Haut/Bas ou Z/S : Avancer / Reculer")
    print("  Flèches Gauche/Droite   : Tourner sur place (Lacet)")
    print("  Touches A / E           : Déplacement latéral en crabe (Mode M1 Mecanum)")
    print("  Touches I/K et J/L      : Orienter la caméra Pan-Tilt 3D")
    print("  Touche M                : Basculer entre Châssis T1 (Chenilles) et M1 (Mecanum)")
    print("  Barre Espace            : STOP d'urgence")
    print("  Touche R                : Réinitialiser la position")

    dt = 1.0 / 240.0
    sim_time = 0.0
    step_counter = 0

    # Paramètres géométriques cinématique inverse Mecanum / Chenilles
    wheel_radius = 0.0325
    lx_plus_ly = 0.065 + 0.092  # Demi-empattement + demi-voie (m)

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

            # Vérifie le changement de châssis T1/M1
            if state["chassis_mode"] != last_chassis_mode["value"]:
                apply_chassis_visibility(robot_id, link_name_to_id, state["chassis_mode"])
                last_chassis_mode["value"] = state["chassis_mode"]

            # Lecture des raccourcis clavier PyBullet
            keys = p.getKeyboardEvents()
            if ord(" ") in keys and keys[ord(" ")] & p.KEY_WAS_TRIGGERED:
                gui.cmd_stop()
                state = gui.get_state()
            if ord("r") in keys and keys[ord("r")] & p.KEY_WAS_TRIGGERED:
                on_gui_command("RESET_SIM")
                state = gui.get_state()
            if ord("m") in keys and keys[ord("m")] & p.KEY_WAS_TRIGGERED:
                next_mode = "M1" if gui.chassis_mode_var.get() == "T1" else "T1"
                gui.chassis_mode_var.set(next_mode)
                gui._on_chassis_change()
                state = gui.get_state()

            # Pilotage clavier direct
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

            # Orientation clavier de la tourelle caméra (I/K = Tilt, J/L = Pan)
            if ord("j") in keys and (keys[ord("j")] & p.KEY_IS_DOWN):
                gui.cam_pan_deg_var.set(min(90.0, gui.cam_pan_deg_var.get() + 1.0))
            if ord("l") in keys and (keys[ord("l")] & p.KEY_IS_DOWN):
                gui.cam_pan_deg_var.set(max(-90.0, gui.cam_pan_deg_var.get() - 1.0))
            if ord("i") in keys and (keys[ord("i")] & p.KEY_IS_DOWN):
                gui.cam_tilt_deg_var.set(min(60.0, gui.cam_tilt_deg_var.get() + 1.0))
            if ord("k") in keys and (keys[ord("k")] & p.KEY_IS_DOWN):
                gui.cam_tilt_deg_var.set(max(-45.0, gui.cam_tilt_deg_var.get() - 1.0))

            # Pose actuelle du robot
            base_pos, base_orn = p.getBasePositionAndOrientation(robot_id)
            base_euler = p.getEulerFromQuaternion(base_orn)
            base_yaw = base_euler[2]

            # 1. Rotation continue de la tourelle LiDAR STL27L
            if "Joint_LiDAR_Spin" in joint_name_to_id:
                p.setJointMotorControl2(
                    robot_id,
                    joint_name_to_id["Joint_LiDAR_Spin"],
                    controlMode=p.VELOCITY_CONTROL,
                    targetVelocity=state["lidar_speed"],
                    force=0.5
                )

            # 2. Contrôle des servos Pan-Tilt de la caméra 3D Depth
            if state["cam_autoscan"]:
                cam_pan_target = 0.85 * math.sin(1.6 * sim_time)
                cam_tilt_target = 0.25 * math.sin(0.9 * sim_time)
                gui.cam_pan_deg_var.set(math.degrees(cam_pan_target))
                gui.cam_tilt_deg_var.set(math.degrees(cam_tilt_target))
            else:
                cam_pan_target = state["cam_pan_rad"]
                cam_tilt_target = state["cam_tilt_rad"]

            if "Joint_Camera_Pan" in joint_name_to_id:
                p.setJointMotorControl2(
                    robot_id,
                    joint_name_to_id["Joint_Camera_Pan"],
                    controlMode=p.POSITION_CONTROL,
                    targetPosition=cam_pan_target,
                    force=2.0,
                    maxVelocity=6.0
                )
            if "Joint_Camera_Tilt" in joint_name_to_id:
                p.setJointMotorControl2(
                    robot_id,
                    joint_name_to_id["Joint_Camera_Tilt"],
                    controlMode=p.POSITION_CONTROL,
                    targetPosition=cam_tilt_target,
                    force=2.0,
                    maxVelocity=6.0
                )

            # 3. Balayage LiDAR 360° (tous les 8 pas = 30 Hz)
            if step_counter % 8 == 0:
                if "LiDAR_Rotor_Turret" in link_name_to_id:
                    l_state = p.getLinkState(robot_id, link_name_to_id["LiDAR_Rotor_Turret"])
                    lidar_pos = [l_state[0][0], l_state[0][1], l_state[0][2] + 0.008]
                else:
                    lidar_pos = [base_pos[0], base_pos[1], base_pos[2] + 0.11]

                telemetry = lidar_scanner.scan(
                    sensor_pos=lidar_pos,
                    base_yaw=base_yaw,
                    draw_3d_rays=state["lidar_rays_3d"]
                )
                gui.radar_widget.update_radar(lidar_scanner, telemetry)

            # 4. Rendu Caméra 3D Embarquée (tous les 12 pas = 20 FPS)
            if step_counter % 12 == 0 and state["cam_enable"]:
                if "Camera_Depth_3D_Head" in link_name_to_id:
                    c_state = p.getLinkState(robot_id, link_name_to_id["Camera_Depth_3D_Head"])
                    c_pos = [c_state[0][0], c_state[0][1], c_state[0][2] + 0.015]
                else:
                    c_pos = [base_pos[0] + 0.08, base_pos[1], base_pos[2] + 0.14]
                cam_yaw = base_yaw + cam_pan_target
                cam_pitch = cam_tilt_target

                # Avance légèrement le point d'oeil devant la vitre optique de la caméra
                eye_pos = [
                    c_pos[0] + 0.032 * math.cos(cam_yaw) * math.cos(cam_pitch),
                    c_pos[1] + 0.032 * math.sin(cam_yaw) * math.cos(cam_pitch),
                    c_pos[2] + 0.032 * math.sin(cam_pitch)
                ]
                ppm_bytes, center_depth = cam_renderer.render_ppm(
                    cam_pos=eye_pos,
                    cam_yaw=cam_yaw,
                    cam_pitch=cam_pitch,
                    mode=state["cam_mode"]
                )
                gui.camera_widget.update_image(ppm_bytes, center_depth)

            # 5. Calcul des consignes de déplacement (Manuel ou Évitement Autonome LiDAR)
            if state["auto_avoid"]:
                vx_cmd, vy_cmd, wz_cmd = lidar_scanner.compute_avoidance_vel(
                    forward_speed=0.45 * gui.speed_scale_var.get(),
                    turn_speed=0.85 * gui.speed_scale_var.get(),
                    safe_dist=0.58
                )
            else:
                vx_cmd = state["vx"] * 0.55
                vy_cmd = state["vy"] * 0.45
                wz_cmd = state["wz"] * 1.35

            # Cinématique inverse des 4 roues Mecanum M1 et des 4 barbotins de chenilles T1
            w_fl = (vx_cmd - vy_cmd - lx_plus_ly * wz_cmd) / wheel_radius
            w_fr = (vx_cmd + vy_cmd + lx_plus_ly * wz_cmd) / wheel_radius
            w_rl = (vx_cmd + vy_cmd - lx_plus_ly * wz_cmd) / wheel_radius
            w_rr = (vx_cmd - vy_cmd + lx_plus_ly * wz_cmd) / wheel_radius

            for jname, w_target in [
                ("Joint_Mecanum_FL", w_fl),
                ("Joint_Mecanum_FR", w_fr),
                ("Joint_Mecanum_RL", w_rl),
                ("Joint_Mecanum_RR", w_rr),
                ("Joint_Sprocket_FL", w_fl),
                ("Joint_Sprocket_FR", w_fr),
                ("Joint_Sprocket_RL", w_rl),
                ("Joint_Sprocket_RR", w_rr),
            ]:
                if jname in joint_name_to_id:
                    p.setJointMotorControl2(
                        robot_id,
                        joint_name_to_id[jname],
                        controlMode=p.VELOCITY_CONTROL,
                        targetVelocity=w_target,
                        force=1.5
                    )

            # Propulsion planaire fluide dans le repère monde tout en conservant la gravité et les collisions
            lin_vel_world, ang_vel_world = p.getBaseVelocity(robot_id)
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
    chassis_arg = "T1"
    if len(sys.argv) > 1 and sys.argv[1].upper() in ("T1", "M1"):
        chassis_arg = sys.argv[1].upper()
    res = run_mentorpi_simulation(initial_chassis=chassis_arg)
    if res == "MENU":
        import sim_launcher
        sim_launcher.main()
    elif res == "HEXAPOD":
        import hexapod_sim
        hexapod_sim.main()
