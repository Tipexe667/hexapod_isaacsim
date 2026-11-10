#!/usr/bin/env python3
"""
Simulation Hexapode sous PyBullet (Windows Natif).
==================================================
Physique réaliste ABS & Dynamixel AX-18 :
- Solveur haute précision sans rebond artificiel ni secousse au repos
- Machine à états de postures (STAND, REST, CROUCH, WALK, MANUAL)
- Réinitialisation complète et synchronisée (simulation + curseurs GUI)
- Cinématique omnidirectionnelle C1-smooth et asservissement critique AX-18
"""

import os
import sys
import time
import math
import pybullet as p
import pybullet_data

from hexapod_kinematics import HexapodKinematics
from hexapod_gui import HexapodControlPanel
from sensor_widgets import ObstacleArenaManager, LidarScanner, OnboardCameraRenderer


class HexapodSimulation:
    def __init__(self, urdf_path="urdf/hexapod_pybullet.urdf", gui=True, use_panel=True):
        self.urdf_path = os.path.abspath(urdf_path)
        if not os.path.exists(self.urdf_path):
            raise FileNotFoundError(f"Fichier URDF introuvable : {self.urdf_path}")

        self.gui = gui
        self.use_panel = use_panel

        if self.gui:
            self.physics_client = p.connect(p.GUI)
            p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        else:
            self.physics_client = p.connect(p.DIRECT)

        # Moteur physique Bullet calibré
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.current_gravity = -9.81
        p.setGravity(0, 0, self.current_gravity)
        self.time_step = 1.0 / 240.0
        p.setTimeStep(self.time_step)
        p.setRealTimeSimulation(0)

        # Solveur de contraintes optimisé pour la stabilité et zéro rebond
        p.setPhysicsEngineParameter(
            numSolverIterations=150,
            numSubSteps=2,
            enableConeFriction=1,
            contactERP=0.15,
            frictionERP=0.15,
            deterministicOverlappingPairs=1
        )

        # Sol avec dissipation plastique
        self.plane_id = p.loadURDF("plane.urdf")
        p.changeDynamics(
            self.plane_id, -1,
            lateralFriction=1.5,
            restitution=0.0,
            contactStiffness=30000.0,
            contactDamping=1500.0
        )

        # Hauteur d'apparition d'équilibre nominale (zéro pénétration, zéro choc)
        self.spawn_pos = [0, 0, 0.1878]
        self.spawn_orn = p.getQuaternionFromEuler([0, 0, 0])
        print(f"Chargement du robot : {self.urdf_path}")
        self.robot_id = p.loadURDF(
            self.urdf_path,
            self.spawn_pos,
            self.spawn_orn,
            flags=p.URDF_USE_INERTIA_FROM_FILE
        )

        # Châssis en ABS : amortissement des vibrations structurelles
        p.changeDynamics(
            self.robot_id, -1,
            linearDamping=0.04,
            angularDamping=0.04
        )

        # Caractéristiques physiques réelles Dynamixel AX-18 (à 12.0 V)
        # - Couple max : 1.8 N.m (18.3 kgf.cm)
        # - Vitesse max : 9.8 rad/s (97 RPM)
        # - kp=0.20, kd=1.00 : asservissement critique stable (zéro oscillation au repos)
        self.ax18_stall_torque = 1.8
        self.ax18_max_velocity = 9.8
        self.ax18_kp = 0.20
        self.ax18_kd = 1.00

        # Caméra 3D avec point cible stabilisé
        self.cam_target = [0.0, 0.0, 0.1878]
        if self.gui:
            p.resetDebugVisualizerCamera(
                cameraDistance=0.60,
                cameraYaw=45,
                cameraPitch=-25,
                cameraTargetPosition=self.cam_target
            )

        # Indexation des articulations et des liens
        self.num_joints = p.getNumJoints(self.robot_id)
        self.joint_name_to_id = {}
        self.link_name_to_id = {}

        for i in range(self.num_joints):
            info = p.getJointInfo(self.robot_id, i)
            joint_name = info[1].decode("utf-8")
            link_name = info[12].decode("utf-8")
            self.joint_name_to_id[joint_name] = i
            self.link_name_to_id[link_name] = i

        self.leg_joints = {}
        for leg_idx in range(1, 7):
            self.leg_joints[leg_idx] = {
                "coxa": self.joint_name_to_id.get(f"coxa_joint_leg{leg_idx}"),
                "femur": self.joint_name_to_id.get(f"femur_joint_leg{leg_idx}"),
                "tibia": self.joint_name_to_id.get(f"tibia_joint_leg{leg_idx}"),
                "foot": self.link_name_to_id.get(f"foot_leg{leg_idx}")
            }

        # Propriétés de contact des pieds (caoutchouc noir) et friction servos AX-18
        for i in range(self.num_joints):
            info = p.getJointInfo(self.robot_id, i)
            j_type = info[2]
            link_name = info[12].decode("utf-8")
            if "foot" in link_name:
                p.changeDynamics(
                    self.robot_id, i,
                    lateralFriction=2.0,
                    spinningFriction=0.04,
                    rollingFriction=0.005,
                    restitution=0.0,
                    contactStiffness=25000.0,
                    contactDamping=1800.0
                )
            elif j_type == p.JOINT_REVOLUTE:
                p.changeDynamics(
                    self.robot_id, i,
                    jointDamping=0.12,
                    restitution=0.0
                )

        # Moteur cinématique analytique
        self.kinematics = HexapodKinematics()

        # État et mode de fonctionnement
        self.mode = "STAND"  # "STAND", "WALK", "REST", "CROUCH", "MANUAL", "JUMP"
        self.jump_phase = "IDLE"  # "IDLE", "COMPRESS", "PUSHOFF", "FLIGHT", "LANDING"
        self.jump_timer = 0

        # Variables filtrées pour transitions continues
        self.cur_vx = 0.0
        self.cur_vy = 0.0
        self.cur_wz = 0.0
        self.cur_body_z = 0.0
        self.cur_roll = 0.0
        self.cur_pitch = 0.0
        self.cur_yaw = 0.0

        # Initialisation stable au sol dans la posture debout
        init_targets = self.kinematics.compute_all_legs(dt=0.0)
        for leg_idx in range(1, 7):
            c_j = self.leg_joints[leg_idx]["coxa"]
            f_j = self.leg_joints[leg_idx]["femur"]
            t_j = self.leg_joints[leg_idx]["tibia"]

            tc = init_targets[leg_idx]["coxa"]
            tf = init_targets[leg_idx]["femur"]
            tt = init_targets[leg_idx]["tibia"]

            p.resetJointState(self.robot_id, c_j, tc)
            p.resetJointState(self.robot_id, f_j, tf)
            p.resetJointState(self.robot_id, t_j, tt)

            p.setJointMotorControl2(self.robot_id, c_j, p.POSITION_CONTROL, targetPosition=tc,
                                    force=self.ax18_stall_torque, maxVelocity=self.ax18_max_velocity,
                                    positionGain=self.ax18_kp, velocityGain=self.ax18_kd)
            p.setJointMotorControl2(self.robot_id, f_j, p.POSITION_CONTROL, targetPosition=tf,
                                    force=self.ax18_stall_torque, maxVelocity=self.ax18_max_velocity,
                                    positionGain=self.ax18_kp, velocityGain=self.ax18_kd)
            p.setJointMotorControl2(self.robot_id, t_j, p.POSITION_CONTROL, targetPosition=tt,
                                    force=self.ax18_stall_torque, maxVelocity=self.ax18_max_velocity,
                                    positionGain=self.ax18_kp, velocityGain=self.ax18_kd)

        # Télécommande GUI
        self.panel = None
        if self.use_panel:
            self.panel = HexapodControlPanel(on_command_change=self.on_gui_command)

        # Capteurs embarqués (LiDAR 360° & Caméra 3D) et Arène d'obstacles
        self.arena_mgr = ObstacleArenaManager()
        self.arena_mgr.spawn_arena()
        self.lidar_scanner = LidarScanner(num_rays=72, max_range=2.20, min_range=0.14)
        self.cam_renderer = OnboardCameraRenderer(width=160, height=120, fov=65.0, near=0.05, far=3.50)
        self.step_counter = 0
        self.switch_target = None

        # Pas de stabilisation initial
        for _ in range(40):
            p.stepSimulation()

    def on_gui_command(self, cmd_type):
        """Gestion des commandes spéciales provenant de la GUI."""
        if cmd_type == "RESET_SIM":
            self.reset_simulation()
        elif cmd_type == "POSTURE_STAND":
            self.mode = "STAND"
            if self.panel:
                self.panel.cmd_stop()
                self.panel.reset_pose()
        elif cmd_type == "POSTURE_REST":
            self.mode = "REST"
            if self.panel:
                self.panel.cmd_stop()
        elif cmd_type == "POSTURE_CROUCH":
            self.mode = "CROUCH"
            if self.panel:
                self.panel.cmd_stop()
        elif cmd_type == "TRIGGER_JUMP":
            self.trigger_jump()
        elif cmd_type == "TOGGLE_ARENA":
            if self.panel:
                self.arena_mgr.toggle_arena(self.panel.arena_var.get())
        elif cmd_type == "SWITCH_MENU":
            self.switch_target = "MENU"
        elif cmd_type == "SWITCH_MENTORPI":
            self.switch_target = "MENTORPI"

    def trigger_jump(self):
        """Déclenche la séquence de saut dynamique (compression, impulsion, vol, réception)."""
        if self.jump_phase == "IDLE":
            self.jump_phase = "COMPRESS"
            self.jump_timer = 45  # ~0.18s de flexion préparatoire
            self.mode = "JUMP"
            print("[ACTION] Declenchement du saut dynamique !")

    def reset_simulation(self):
        """Réinitialise complètement la position du robot, les vitesses et les curseurs."""
        self.mode = "STAND"
        self.jump_phase = "IDLE"
        self.jump_timer = 0
        p.resetBasePositionAndOrientation(self.robot_id, self.spawn_pos, self.spawn_orn)
        p.resetBaseVelocity(self.robot_id, [0, 0, 0], [0, 0, 0])

        self.cur_vx = 0.0
        self.cur_vy = 0.0
        self.cur_wz = 0.0
        self.cur_body_z = 0.0
        self.cur_roll = 0.0
        self.cur_pitch = 0.0
        self.cur_yaw = 0.0
        self.cam_target = [0.0, 0.0, 0.1878]
        self.kinematics.gait_phase = 0.0

        init_targets = self.kinematics.compute_all_legs(dt=0.0)
        for leg_idx in range(1, 7):
            c_j = self.leg_joints[leg_idx]["coxa"]
            f_j = self.leg_joints[leg_idx]["femur"]
            t_j = self.leg_joints[leg_idx]["tibia"]

            tc = init_targets[leg_idx]["coxa"]
            tf = init_targets[leg_idx]["femur"]
            tt = init_targets[leg_idx]["tibia"]

            p.resetJointState(self.robot_id, c_j, tc, targetVelocity=0.0)
            p.resetJointState(self.robot_id, f_j, tf, targetVelocity=0.0)
            p.resetJointState(self.robot_id, t_j, tt, targetVelocity=0.0)

            p.setJointMotorControl2(self.robot_id, c_j, p.POSITION_CONTROL, targetPosition=tc,
                                    force=self.ax18_stall_torque, maxVelocity=self.ax18_max_velocity,
                                    positionGain=self.ax18_kp, velocityGain=self.ax18_kd)
            p.setJointMotorControl2(self.robot_id, f_j, p.POSITION_CONTROL, targetPosition=tf,
                                    force=self.ax18_stall_torque, maxVelocity=self.ax18_max_velocity,
                                    positionGain=self.ax18_kp, velocityGain=self.ax18_kd)
            p.setJointMotorControl2(self.robot_id, t_j, p.POSITION_CONTROL, targetPosition=tt,
                                    force=self.ax18_stall_torque, maxVelocity=self.ax18_max_velocity,
                                    positionGain=self.ax18_kp, velocityGain=self.ax18_kd)

        if self.panel:
            self.panel.reset_all_controls()

    def handle_keyboard(self):
        """Lit les touches du clavier dans la vue 3D PyBullet."""
        if not self.gui:
            return 0.0, 0.0, 0.0

        keys = p.getKeyboardEvents()
        vx, vy, wz = 0.0, 0.0, 0.0

        if p.B3G_UP_ARROW in keys and keys[p.B3G_UP_ARROW] & p.KEY_IS_DOWN:
            vx += 1.0
        if p.B3G_DOWN_ARROW in keys and keys[p.B3G_DOWN_ARROW] & p.KEY_IS_DOWN:
            vx -= 1.0
        if p.B3G_LEFT_ARROW in keys and keys[p.B3G_LEFT_ARROW] & p.KEY_IS_DOWN:
            wz -= 1.0
        if p.B3G_RIGHT_ARROW in keys and keys[p.B3G_RIGHT_ARROW] & p.KEY_IS_DOWN:
            wz += 1.0

        if ord('q') in keys and keys[ord('q')] & p.KEY_IS_DOWN:
            vy += 1.0
        if ord('d') in keys and keys[ord('d')] & p.KEY_IS_DOWN:
            vy -= 1.0

        if ord('r') in keys and keys[ord('r')] & p.KEY_WAS_TRIGGERED:
            self.reset_simulation()

        if (ord('j') in keys and keys[ord('j')] & p.KEY_WAS_TRIGGERED) or \
           (ord('J') in keys and keys[ord('J')] & p.KEY_WAS_TRIGGERED):
            self.trigger_jump()

        if ord(' ') in keys and keys[ord(' ')] & p.KEY_WAS_TRIGGERED:
            self.mode = "STAND"
            if self.panel:
                self.panel.cmd_stop()

        return vx, vy, wz

    def update(self):
        """Boucle principale de simulation."""
        if self.switch_target is not None or not p.isConnected():
            return False

        is_walking = False
        state = None
        if self.panel and not self.panel.is_closed:
            self.panel.update()
            state = self.panel.get_state()
        elif self.panel and self.panel.is_closed:
            return False

        # Mise à jour de la gravité et couple AX-18
        if state:
            new_g = state.get("gravity", -9.81)
            if abs(new_g - self.current_gravity) > 0.05:
                self.current_gravity = new_g
                p.setGravity(0, 0, self.current_gravity)
            self.ax18_stall_torque = state.get("ax18_torque", 1.8)

        # Clavier
        kb_vx, kb_vy, kb_wz = self.handle_keyboard()

        # Mode Banc d'essai (châssis suspendu)
        if state and state.get("bench_mode", False):
            p.resetBasePositionAndOrientation(self.robot_id, [0, 0, 0.22], [0, 0, 0, 1])
            p.resetBaseVelocity(self.robot_id, [0, 0, 0], [0, 0, 0])

        # Capteurs embarqués : LiDAR 360° (30 Hz) & Caméra 3D (20 Hz)
        base_pos, base_orn = p.getBasePositionAndOrientation(self.robot_id)
        base_euler = p.getEulerFromQuaternion(base_orn)
        base_yaw = base_euler[2]

        if self.panel and state:
            if self.step_counter % 8 == 0:
                lidar_pos = [base_pos[0], base_pos[1], base_pos[2] + 0.065]
                tel = self.lidar_scanner.scan(
                    sensor_pos=lidar_pos,
                    base_yaw=base_yaw,
                    draw_3d_rays=state.get("lidar_rays_3d", True)
                )
                self.panel.radar_widget.update_radar(self.lidar_scanner, tel)

            if self.step_counter % 12 == 0 and state.get("cam_enable", True):
                cam_yaw = base_yaw + state.get("cam_pan_rad", 0.0)
                cam_pitch = base_euler[1] + state.get("cam_tilt_rad", -0.08)
                eye_pos = [
                    base_pos[0] + 0.11 * math.cos(base_yaw),
                    base_pos[1] + 0.11 * math.sin(base_yaw),
                    base_pos[2] + 0.035
                ]
                ppm_bytes, center_depth = self.cam_renderer.render_ppm(
                    cam_pos=eye_pos,
                    cam_yaw=cam_yaw,
                    cam_pitch=cam_pitch,
                    mode=state.get("cam_mode", "RGB")
                )
                self.panel.camera_widget.update_image(ppm_bytes, center_depth)

        self.step_counter += 1

        # Lecture des commandes utilisateur (ou Marche Autonome LiDAR)
        if state and state.get("auto_avoid", False) and abs(kb_vx) < 0.01 and abs(kb_vy) < 0.01 and abs(kb_wz) < 0.01:
            av_vx, av_vy, av_wz = self.lidar_scanner.compute_avoidance_vel(
                forward_speed=0.75, turn_speed=0.80, safe_dist=0.58
            )
            target_vx, target_vy, target_wz = av_vx, av_vy, av_wz
        else:
            target_vx = kb_vx if abs(kb_vx) > 0.01 else (state["vx"] if state else 0.0)
            target_vy = kb_vy if abs(kb_vy) > 0.01 else (state["vy"] if state else 0.0)
            target_wz = kb_wz if abs(kb_wz) > 0.01 else (state["wz"] if state else 0.0)

        # Détection d'une intention de marche
        is_walking_cmd = (abs(target_vx) > 0.02 or abs(target_vy) > 0.02 or abs(target_wz) > 0.02)

        # Si l'utilisateur commande un déplacement alors qu'on était en repos/accroupi, revenir debout et marcher
        if is_walking_cmd and self.mode in ["REST", "CROUCH"]:
            self.mode = "WALK"
            if self.panel:
                self.panel.body_z_var.set(0.0)

        # Gestion des modes
        if self.jump_phase != "IDLE":
            self.mode = "JUMP"
            pos, _ = p.getBasePositionAndOrientation(self.robot_id)
            vel, _ = p.getBaseVelocity(self.robot_id)
            contacts = p.getContactPoints(self.robot_id)
            n_contacts = len(contacts)

            # Machine à états pour le saut dynamique
            if self.jump_phase == "COMPRESS":
                self.jump_timer -= 1
                # Flexion préparatoire des 6 pattes
                targets = self.kinematics.compute_all_legs(dt=0.0, body_z=-0.055)
                f_val, v_val, kp, kd = 2.0, 9.0, 0.35, 1.0
                if self.jump_timer <= 0:
                    self.jump_phase = "PUSHOFF"
                    self.jump_timer = 20  # ~0.08s de poussée explosive vers le sol
            elif self.jump_phase == "PUSHOFF":
                self.jump_timer -= 1
                # Détente explosive des 6 pattes avec propulsion verticale (+ avant si en marche)
                targets = self.kinematics.compute_all_legs(dt=0.0, vx=self.cur_vx * 0.5, body_z=0.030)
                f_val, v_val, kp, kd = 2.4, 14.0, 0.65, 0.50
                if self.jump_timer <= 0:
                    self.jump_phase = "FLIGHT"
                    self.jump_timer = 15  # Délai initial de dégagement aérien
            elif self.jump_phase == "FLIGHT":
                self.jump_timer -= 1
                # Phase aérienne : repli léger des pattes prêt pour le contact
                targets = self.kinematics.compute_all_legs(dt=0.0, body_z=-0.005)
                f_val, v_val, kp, kd = 1.8, 9.8, 0.25, 1.0
                # Détection de l'atterrissage : le robot redescend (vz < 0) et s'approche du sol ou touche
                if self.jump_timer <= 0 and vel[2] < 0.0 and (pos[2] <= 0.195 or n_contacts >= 3):
                    self.jump_phase = "LANDING"
                    self.jump_timer = 50  # ~0.20s d'amortissement de l'impact
            elif self.jump_phase == "LANDING":
                self.jump_timer -= 1
                # Amortissement et stabilisation debout
                targets = self.kinematics.compute_all_legs(dt=0.0, body_z=0.0)
                f_val, v_val, kp, kd = self.ax18_stall_torque, self.ax18_max_velocity, 0.25, 1.2
                if self.jump_timer <= 0:
                    self.jump_phase = "IDLE"
                    self.mode = "STAND"
                    if self.panel:
                        self.panel.body_z_var.set(0.0)

            for leg_idx in range(1, 7):
                c_j = self.leg_joints[leg_idx]["coxa"]
                f_j = self.leg_joints[leg_idx]["femur"]
                t_j = self.leg_joints[leg_idx]["tibia"]
                p.setJointMotorControl2(self.robot_id, c_j, p.POSITION_CONTROL, targetPosition=targets[leg_idx]["coxa"],
                                        force=f_val, maxVelocity=v_val, positionGain=kp, velocityGain=kd)
                p.setJointMotorControl2(self.robot_id, f_j, p.POSITION_CONTROL, targetPosition=targets[leg_idx]["femur"],
                                        force=f_val, maxVelocity=v_val, positionGain=kp, velocityGain=kd)
                p.setJointMotorControl2(self.robot_id, t_j, p.POSITION_CONTROL, targetPosition=targets[leg_idx]["tibia"],
                                        force=f_val, maxVelocity=v_val, positionGain=kp, velocityGain=kd)

        elif state and state.get("manual_servos", False):
            self.mode = "MANUAL"
            servos = state["servos"]
            for leg_idx in range(1, 7):
                c_val = servos.get(f"L{leg_idx}_coxa", 0.0)
                f_val = servos.get(f"L{leg_idx}_femur", 0.4503)
                t_val = servos.get(f"L{leg_idx}_tibia", 0.8493)

                c_j = self.leg_joints[leg_idx]["coxa"]
                f_j = self.leg_joints[leg_idx]["femur"]
                t_j = self.leg_joints[leg_idx]["tibia"]

                p.setJointMotorControl2(self.robot_id, c_j, p.POSITION_CONTROL, targetPosition=c_val,
                                        force=self.ax18_stall_torque, maxVelocity=self.ax18_max_velocity,
                                        positionGain=self.ax18_kp, velocityGain=self.ax18_kd)
                p.setJointMotorControl2(self.robot_id, f_j, p.POSITION_CONTROL, targetPosition=f_val,
                                        force=self.ax18_stall_torque, maxVelocity=self.ax18_max_velocity,
                                        positionGain=self.ax18_kp, velocityGain=self.ax18_kd)
                p.setJointMotorControl2(self.robot_id, t_j, p.POSITION_CONTROL, targetPosition=t_val,
                                        force=self.ax18_stall_torque, maxVelocity=self.ax18_max_velocity,
                                        positionGain=self.ax18_kp, velocityGain=self.ax18_kd)

        else:
            # Modes STAND, CROUCH, REST et WALK (résolus continûment par l'IK analytique 3-DOF)
            if state:
                self.kinematics.step_frequency = state["freq"]
                self.kinematics.step_height = state["step_h"]
                self.kinematics.step_length_scale = state["step_l"]
                target_bz = state["body_z"]
                target_roll = state["roll"]
                target_pitch = state["pitch"]
                target_yaw = state["yaw"]
            else:
                if self.mode == "REST":
                    target_bz = -0.08
                elif self.mode == "CROUCH":
                    target_bz = -0.04
                else:
                    target_bz = 0.0
                target_roll, target_pitch, target_yaw = 0.0, 0.0, 0.0

            # Lissage progressif des consignes (évite tout choc mécanique sur les AX-18)
            alpha = 0.08
            self.cur_vx += (target_vx - self.cur_vx) * alpha
            self.cur_vy += (target_vy - self.cur_vy) * alpha
            self.cur_wz += (target_wz - self.cur_wz) * alpha
            self.cur_body_z += (target_bz - self.cur_body_z) * alpha
            self.cur_roll += (target_roll - self.cur_roll) * alpha
            self.cur_pitch += (target_pitch - self.cur_pitch) * alpha
            self.cur_yaw += (target_yaw - self.cur_yaw) * alpha

            # Deadbands strictes à l'arrêt pour bloquer tout micro-frétillement
            if abs(self.cur_vx) < 0.003: self.cur_vx = 0.0
            if abs(self.cur_vy) < 0.003: self.cur_vy = 0.0
            if abs(self.cur_wz) < 0.003: self.cur_wz = 0.0
            if abs(self.cur_roll) < 0.03: self.cur_roll = 0.0
            if abs(self.cur_pitch) < 0.03: self.cur_pitch = 0.0
            if abs(self.cur_yaw) < 0.03: self.cur_yaw = 0.0
            if abs(self.cur_body_z) < 0.0005: self.cur_body_z = 0.0

            is_walking = (abs(self.cur_vx) > 0.005 or abs(self.cur_vy) > 0.005 or abs(self.cur_wz) > 0.005)
            if is_walking:
                self.mode = "WALK"
            elif self.mode not in ["REST", "CROUCH"]:
                self.mode = "STAND"

            # Calcul cinématique analytique exact (erreur < 0.001 mm)
            joint_targets = self.kinematics.compute_all_legs(
                dt=self.time_step,
                vx=self.cur_vx, vy=self.cur_vy, wz=self.cur_wz,
                body_z=self.cur_body_z,
                roll_deg=self.cur_roll,
                pitch_deg=self.cur_pitch,
                yaw_deg=self.cur_yaw,
                is_walking=is_walking
            )

            # Application aux 18 servomoteurs AX-18
            for leg_idx in range(1, 7):
                c_j = self.leg_joints[leg_idx]["coxa"]
                f_j = self.leg_joints[leg_idx]["femur"]
                t_j = self.leg_joints[leg_idx]["tibia"]

                tc = joint_targets[leg_idx]["coxa"]
                tf = joint_targets[leg_idx]["femur"]
                tt = joint_targets[leg_idx]["tibia"]

                p.setJointMotorControl2(self.robot_id, c_j, p.POSITION_CONTROL, targetPosition=tc,
                                        force=self.ax18_stall_torque, maxVelocity=self.ax18_max_velocity,
                                        positionGain=self.ax18_kp, velocityGain=self.ax18_kd)
                p.setJointMotorControl2(self.robot_id, f_j, p.POSITION_CONTROL, targetPosition=tf,
                                        force=self.ax18_stall_torque, maxVelocity=self.ax18_max_velocity,
                                        positionGain=self.ax18_kp, velocityGain=self.ax18_kd)
                p.setJointMotorControl2(self.robot_id, t_j, p.POSITION_CONTROL, targetPosition=tt,
                                        force=self.ax18_stall_torque, maxVelocity=self.ax18_max_velocity,
                                        positionGain=self.ax18_kp, velocityGain=self.ax18_kd)

        # Pas physique Bullet
        p.stepSimulation()

        # Suivi automatique caméra fluide uniquement pendant la marche active
        if self.gui and is_walking:
            pos, _ = p.getBasePositionAndOrientation(self.robot_id)
            dx = pos[0] - self.cam_target[0]
            dy = pos[1] - self.cam_target[1]
            if (dx * dx + dy * dy) > 0.0004:  # > 2cm de déplacement
                self.cam_target[0] += dx * 0.05
                self.cam_target[1] += dy * 0.05
                self.cam_target[2] = pos[2]
                cam_info = p.getDebugVisualizerCamera()
                p.resetDebugVisualizerCamera(
                    cameraDistance=cam_info[10],
                    cameraYaw=cam_info[8],
                    cameraPitch=cam_info[9],
                    cameraTargetPosition=self.cam_target
                )

            # Affichage points de contact
            if state and state.get("contacts", False):
                contacts = p.getContactPoints(self.robot_id, self.plane_id)
                for cp in contacts:
                    cpos = cp[5]
                    p.addUserDebugLine(cpos, [cpos[0], cpos[1], cpos[2] + 0.035], [0, 1, 0], lineWidth=2, lifeTime=0.04)

        return True


def main():
    print("=" * 65)
    print("  SIMULATION HEXAPODE PYBULLET + TÉLÉCOMMANDE GRAPHIQUE")
    print("=" * 65)
    print("Dynamique ABS & Dynamixel AX-18 activée :")
    print("  - Postures : STAND, REST, CROUCH persistantes")
    print("  - Zéro frétillement au repos (solveur critique)")
    print("  - Reset complet (touche R ou bouton Reset)")
    print("=" * 65)

    sim = HexapodSimulation(gui=True, use_panel=True)

    try:
        while True:
            t0 = time.time()
            running = sim.update()
            if not running:
                break
            elapsed = time.time() - t0
            sleep_time = sim.time_step - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
    except KeyboardInterrupt:
        print("\nArrêt de la simulation.")
    finally:
        try:
            if sim.panel and not sim.panel.is_closed:
                sim.panel.root.destroy()
        except Exception:
            pass
        if p.isConnected():
            p.disconnect()

    return sim.switch_target


if __name__ == "__main__":
    res = main()
    if res == "MENU":
        import sim_launcher
        sim_launcher.main()
    elif res == "MENTORPI":
        import mentorpi_sim
        mentorpi_sim.run_mentorpi_simulation("T1")
