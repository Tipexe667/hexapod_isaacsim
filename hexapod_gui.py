#!/usr/bin/env python3
"""
Panneau de contrôle graphique (Télécommande GUI) pour robot Hexapode.
=====================================================================
Fenêtre séparée développée en Tkinter (natif Windows, zéro dépendance supplémentaire).

Contient :
- Pad directionnel et boutons de virages (Avancer + Rotation, Pas de crabe latéral)
- Sliders continus de vitesse (Vx, Vy, Wz combinables)
- Contrôle de posture du châssis en temps réel (Hauteur, Roulis, Tangage, Lacet)
- Contrôle direct des 18 servomoteurs
- Boutons d'actions rapides (Banc d'essai, Debout, Repos, Reset)
"""

import math
import tkinter as tk
from tkinter import ttk
from sensor_widgets import LidarRadarWidget, CameraFeedWidget


class HexapodControlPanel:
    def __init__(self, on_command_change=None):
        self.root = tk.Tk()
        self.root.title("🕷️ Télécommande Hexapode - Panneau de Contrôle (18-DOF + LiDAR + Caméra 3D)")
        self.root.geometry("580x820")
        self.root.minsize(520, 680)

        self.on_command_change = on_command_change
        self.is_closed = False
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        # Variables d'état
        self.vx_var = tk.DoubleVar(value=0.0)
        self.vy_var = tk.DoubleVar(value=0.0)
        self.wz_var = tk.DoubleVar(value=0.0)

        self.speed_freq_var = tk.DoubleVar(value=1.5)
        self.step_height_var = tk.DoubleVar(value=0.022)
        self.step_length_var = tk.DoubleVar(value=0.024)

        self.body_z_var = tk.DoubleVar(value=0.0)
        self.roll_var = tk.DoubleVar(value=0.0)
        self.pitch_var = tk.DoubleVar(value=0.0)
        self.yaw_var = tk.DoubleVar(value=0.0)

        self.bench_mode_var = tk.BooleanVar(value=False)
        self.manual_servo_mode_var = tk.BooleanVar(value=False)
        self.contacts_var = tk.BooleanVar(value=False)

        # Paramètres physiques (ABS & AX-18)
        self.gravity_var = tk.DoubleVar(value=-9.81)
        self.ax18_torque_var = tk.DoubleVar(value=1.8)

        # Variables Capteurs Embarqués (LiDAR 360° & Caméra 3D)
        self.lidar_rays_3d_var = tk.BooleanVar(value=True)
        self.auto_avoid_var = tk.BooleanVar(value=False)
        self.arena_var = tk.BooleanVar(value=True)
        self.cam_enable_var = tk.BooleanVar(value=True)
        self.cam_pan_deg_var = tk.DoubleVar(value=0.0)
        self.cam_tilt_deg_var = tk.DoubleVar(value=-5.0)

        # 18 variables pour les servos manuels
        self.servo_vars = {}
        for leg_id in range(1, 7):
            self.servo_vars[f"L{leg_id}_coxa"] = tk.DoubleVar(value=0.0)
            self.servo_vars[f"L{leg_id}_femur"] = tk.DoubleVar(value=0.45)
            self.servo_vars[f"L{leg_id}_tibia"] = tk.DoubleVar(value=0.85)

        self._build_ui()
        self.root.bind("<j>", lambda e: self.cmd_jump())
        self.root.bind("<J>", lambda e: self.cmd_jump())
        self.root.bind("<space>", lambda e: self.cmd_jump())

    def _on_close(self):
        self.is_closed = True
        self.root.destroy()

    def update(self):
        """Met à jour les événements de l'interface non-bloquante."""
        if not self.is_closed:
            self.root.update_idletasks()
            self.root.update()

    def _build_ui(self):
        style = ttk.Style()
        style.theme_use("clam")

        # Bandeau de navigation : changement de robot / retour au menu
        nav_header = tk.Frame(self.root, bg="#1e272e", padx=8, pady=5)
        nav_header.pack(fill="x", side="top")

        tk.Label(
            nav_header, text="🕷️ HEXAPODE 18-DOF (AX-18)",
            bg="#1e272e", fg="#ff9f43", font=("Arial", 10, "bold")
        ).pack(side="left")

        tk.Button(
            nav_header, text="🔀 Menu Robots", bg="#0984e3", fg="white",
            font=("Arial", 9, "bold"), padx=8, pady=2,
            command=lambda: self._trigger_cmd("SWITCH_MENU")
        ).pack(side="right", padx=3)

        tk.Button(
            nav_header, text="🤖 MentorPi", bg="#00b894", fg="white",
            font=("Arial", 9, "bold"), padx=6, pady=2,
            command=lambda: self._trigger_cmd("SWITCH_MENTORPI")
        ).pack(side="right", padx=2)

        tk.Button(
            nav_header, text="🐕 ROSPug", bg="#ff6b1a", fg="white",
            font=("Arial", 9, "bold"), padx=6, pady=2,
            command=lambda: self._trigger_cmd("SWITCH_ROSPUG")
        ).pack(side="right", padx=2)

        # Barre supérieure : boutons d'urgence et statuts
        top_frame = ttk.Frame(self.root, padding=6)
        top_frame.pack(fill="x", side="top")

        btn_stop = tk.Button(
            top_frame, text="🛑 ARRÊT / STABILISER", bg="#e74c3c", fg="white",
            font=("Arial", 10, "bold"), relief="raised", padx=10, pady=4,
            command=self.cmd_stop
        )
        btn_stop.pack(side="left", padx=4)

        btn_reset = tk.Button(
            top_frame, text="🔄 Reset (R)", bg="#7f8c8d", fg="white",
            font=("Arial", 9, "bold"), padx=8, pady=4,
            command=self.cmd_reset
        )
        btn_reset.pack(side="left", padx=4)

        btn_jump = tk.Button(
            top_frame, text="🚀 SAUTER (J)", bg="#8e44ad", fg="white",
            font=("Arial", 9, "bold"), padx=10, pady=4,
            command=self.cmd_jump
        )
        btn_jump.pack(side="left", padx=6)

        chk_bench = ttk.Checkbutton(
            top_frame, text="🛠 Banc d'essai (En l'air)",
            variable=self.bench_mode_var,
            command=self._notify_change
        )
        chk_bench.pack(side="right", padx=6)

        # Onglets principaux
        notebook = ttk.Notebook(self.root)
        notebook.pack(fill="both", expand=True, padx=6, pady=4)

        tab_move = ttk.Frame(notebook, padding=8)
        tab_sensors = ttk.Frame(notebook, padding=8)
        tab_pose = ttk.Frame(notebook, padding=8)
        tab_servos = ttk.Frame(notebook, padding=8)
        tab_actions = ttk.Frame(notebook, padding=8)

        notebook.add(tab_move, text="🚶 Marche & Virages")
        notebook.add(tab_sensors, text="📡 LiDAR & Caméra 3D")
        notebook.add(tab_pose, text="📐 Assiette (IK)")
        notebook.add(tab_servos, text="⚙️ 18 Servos")
        notebook.add(tab_actions, text="✨ Démos & Presets")

        self._build_move_tab(tab_move)
        self._build_sensors_tab(tab_sensors)
        self._build_pose_tab(tab_pose)
        self._build_servos_tab(tab_servos)
        self._build_actions_tab(tab_actions)

    def _build_move_tab(self, parent):
        # 1. Grille de boutons directionnels (D-PAD)
        pad_frame = ttk.LabelFrame(parent, text="Direction Rapide (Boutons)", padding=6)
        pad_frame.pack(fill="x", pady=4)

        btn_grid = [
            [("↖ Av-G", lambda: self.set_vel(0.7, 0.7, 0.0)),
             ("⬆ AVANCER", lambda: self.set_vel(1.0, 0.0, 0.0)),
             ("↗ Av-D", lambda: self.set_vel(0.7, -0.7, 0.0))],

            [("⬅ LATÉRAL G", lambda: self.set_vel(0.0, 1.0, 0.0)),
             ("⏹ STOP", self.cmd_stop),
             ("➡ LATÉRAL D", lambda: self.set_vel(0.0, -1.0, 0.0))],

            [("↙ Rec-G", lambda: self.set_vel(-0.7, 0.7, 0.0)),
             ("⬇ RECULER", lambda: self.set_vel(-1.0, 0.0, 0.0)),
             ("↘ Rec-D", lambda: self.set_vel(-0.7, -0.7, 0.0))],
        ]

        for r, row in enumerate(btn_grid):
            for c, (text, cmd) in enumerate(row):
                bg_col = "#e74c3c" if "STOP" in text else ("#3498db" if "AVANCER" in text else "#ecf0f1")
                fg_col = "white" if ("STOP" in text or "AVANCER" in text) else "black"
                btn = tk.Button(
                    pad_frame, text=text, width=12, height=1, bg=bg_col, fg=fg_col,
                    font=("Arial", 9, "bold" if "STOP" in text or "AVANCER" in text else "normal"),
                    command=cmd
                )
                btn.grid(row=r, column=c, padx=3, pady=2, sticky="nsew")

        pad_frame.columnconfigure((0, 1, 2), weight=1)

        # Boutons Virages et Rotations
        turn_frame = ttk.LabelFrame(parent, text="Virages & Rotations Combinés", padding=6)
        turn_frame.pack(fill="x", pady=4)

        turns = [
            ("↺ Pivoter Gauche", lambda: self.set_vel(0.0, 0.0, -1.0)),
            ("↰ Virage Av-G", lambda: self.set_vel(0.8, 0.0, -0.6)),
            ("↱ Virage Av-D", lambda: self.set_vel(0.8, 0.0, 0.6)),
            ("↻ Pivoter Droite", lambda: self.set_vel(0.0, 0.0, 1.0)),
        ]
        for c, (text, cmd) in enumerate(turns):
            btn = tk.Button(turn_frame, text=text, bg="#e8f8f5", command=cmd, font=("Arial", 9))
            btn.grid(row=0, column=c, padx=2, pady=2, sticky="nsew")
        turn_frame.columnconfigure((0, 1, 2, 3), weight=1)

        # 2. Sliders continus (Joystick virtuel)
        sliders_frame = ttk.LabelFrame(parent, text="Contrôle Continu des Vitesses (Sliders)", padding=6)
        sliders_frame.pack(fill="x", pady=6)

        self._add_slider_row(sliders_frame, "Avancer / Reculer (Vx) :", self.vx_var, -1.0, 1.0, 0.0)
        self._add_slider_row(sliders_frame, "Pas Latéral G/D (Vy) :", self.vy_var, -1.0, 1.0, 0.0)
        self._add_slider_row(sliders_frame, "Rotation G/D (Wz) :", self.wz_var, -1.0, 1.0, 0.0)

        hint = ttk.Label(
            sliders_frame,
            text="💡 Exemple : Avancer (Vx > 0) + Rotation (Wz > 0) = Virage fluide à droite !",
            foreground="#2980b9", font=("Arial", 8, "italic")
        )
        hint.pack(pady=4)

        # 3. Réglages de la foulée
        gait_frame = ttk.LabelFrame(parent, text="Paramètres de Foulée", padding=6)
        gait_frame.pack(fill="x", pady=4)
        self._add_slider_row(gait_frame, "Fréquence (Hz) :", self.speed_freq_var, 0.6, 2.5, 1.5, resolution=0.1)
        self._add_slider_row(gait_frame, "Levée de patte (cm) :", self.step_height_var, 0.010, 0.035, 0.022, scale_mult=100)
        self._add_slider_row(gait_frame, "Longueur de pas (cm) :", self.step_length_var, 0.012, 0.035, 0.024, scale_mult=100)

    def _build_pose_tab(self, parent):
        info = ttk.Label(
            parent,
            text="Modifiez l'assiette et la hauteur du robot en temps réel.\nLes pieds restent en contact stable au sol grâce à l'IK.",
            font=("Arial", 9, "italic"), foreground="#555"
        )
        info.pack(pady=4)

        frame = ttk.LabelFrame(parent, text="Orientation du Châssis (Degrés)", padding=8)
        frame.pack(fill="x", pady=6)

        self._add_slider_row(frame, "Hauteur Châssis Z (cm) :", self.body_z_var, -0.08, 0.03, 0.0, scale_mult=100)
        self._add_slider_row(frame, "Roulis / Roll (°) :", self.roll_var, -10.0, 10.0, 0.0)
        self._add_slider_row(frame, "Tangage / Pitch (°) :", self.pitch_var, -12.0, 12.0, 0.0)
        self._add_slider_row(frame, "Lacet / Yaw (°) :", self.yaw_var, -15.0, 15.0, 0.0)

        btn_level = tk.Button(
            frame, text="📐 Remettre à l'horizontale (Assiette plate)",
            bg="#2ecc71", fg="white", font=("Arial", 9, "bold"), pady=4,
            command=self.reset_pose
        )
        btn_level.pack(fill="x", pady=6)

    def _build_servos_tab(self, parent):
        top = ttk.Frame(parent)
        top.pack(fill="x", pady=2)

        chk_manual = ttk.Checkbutton(
            top, text="Activer le contrôle manuel direct des 18 servos",
            variable=self.manual_servo_mode_var,
            command=self._notify_change
        )
        chk_manual.pack(side="left")

        btn_reset_s = ttk.Button(top, text="Reset Posture Debout", command=self.reset_servos_to_stand)
        btn_reset_s.pack(side="right")

        canvas = tk.Canvas(parent, highlightthickness=0)
        scrollbar = ttk.Scrollbar(parent, orient="vertical", command=canvas.yview)
        scroll_frame = ttk.Frame(canvas)

        scroll_frame.bind(
            "<Configure>", lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )
        canvas.create_window((0, 0), window=scroll_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)

        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

        leg_labels = {
            1: "P1 (Arrière Gauche)",
            2: "P2 (Milieu Gauche)",
            3: "P3 (Avant Gauche)",
            4: "P4 (Avant Droit)",
            5: "P5 (Milieu Droit)",
            6: "P6 (Arrière Droit)"
        }

        for leg_id in range(1, 7):
            lframe = ttk.LabelFrame(scroll_frame, text=leg_labels[leg_id], padding=4)
            lframe.pack(fill="x", padx=4, pady=3)

            self._add_mini_slider(lframe, "Coxa :", self.servo_vars[f"L{leg_id}_coxa"], -1.5, 1.5)
            self._add_mini_slider(lframe, "Femur :", self.servo_vars[f"L{leg_id}_femur"], -1.0, 1.5)
            self._add_mini_slider(lframe, "Tibia :", self.servo_vars[f"L{leg_id}_tibia"], -0.5, 2.0)

    def _build_actions_tab(self, parent):
        frame = ttk.LabelFrame(parent, text="Postures Préconfigurées", padding=10)
        frame.pack(fill="x", pady=6)

        presets = [
            ("🦿 Posture Debout (Stand)", self.cmd_stand),
            ("🦀 Posture Accroupie (Crouch)", self.cmd_crouch),
            ("🛌 Posture Repos au sol (Rest)", self.cmd_rest),
            ("🚀 Saut Dynamique (Jump)", self.cmd_jump),
        ]
        for text, cmd in presets:
            btn = tk.Button(frame, text=text, font=("Arial", 10), pady=4, command=cmd)
            btn.pack(fill="x", pady=3)

        frame_phys = ttk.LabelFrame(parent, text="🔬 Physique & Matériaux (ABS & Dynamixel AX-18)", padding=8)
        frame_phys.pack(fill="x", pady=6)

        info_lbl = ttk.Label(
            frame_phys,
            text="• Structure plastique : ABS (amortissement interne inélastique)\n• Servomoteurs : Dynamixel AX-18 (1.8 N.m, 97 RPM, Réducteur 254:1)",
            font=("Arial", 8, "italic"), foreground="#2c3e50"
        )
        info_lbl.pack(anchor="w", pady=2)

        self._add_slider_row(frame_phys, "Gravité (m/s²) :", self.gravity_var, -15.0, -5.0, -9.81)
        self._add_slider_row(frame_phys, "Couple AX-18 (N.m) :", self.ax18_torque_var, 0.8, 2.5, 1.8)

        frame_visu = ttk.LabelFrame(parent, text="Outils & Affichage", padding=10)
        frame_visu.pack(fill="x", pady=6)

        chk_contacts = ttk.Checkbutton(
            frame_visu, text="👁 Afficher les points de contact au sol",
            variable=self.contacts_var,
            command=self._notify_change
        )
        chk_contacts.pack(anchor="w", pady=4)

        btn_reset_sim = tk.Button(
            frame_visu, text="🔄 Réinitialiser le robot au centre (R)",
            bg="#f39c12", fg="white", font=("Arial", 9, "bold"), pady=4,
            command=self.cmd_reset
        )
        btn_reset_sim.pack(fill="x", pady=4)

    def _build_sensors_tab(self, parent):
        """Construit l'onglet LiDAR 360° & Caméra 3D embarquée sur l'Hexapode."""
        frame_lidar = ttk.LabelFrame(parent, text="📡 Radar LiDAR 360° Embarqué (Détection d'Obstacles)", padding=6)
        frame_lidar.pack(fill="x", pady=4)

        self.radar_widget = LidarRadarWidget(frame_lidar, size=195)
        self.radar_widget.pack(fill="x")

        ctrl_row = ttk.Frame(frame_lidar)
        ctrl_row.pack(fill="x", pady=3)

        ttk.Checkbutton(
            ctrl_row, text="✨ Rayons laser 3D",
            variable=self.lidar_rays_3d_var, command=self._notify_change
        ).pack(side="left", padx=4)

        ttk.Checkbutton(
            ctrl_row, text="🤖 Marche Autonome Évitement LiDAR",
            variable=self.auto_avoid_var, command=self._notify_change
        ).pack(side="left", padx=6)

        ttk.Checkbutton(
            ctrl_row, text="🧱 Arène Obstacles",
            variable=self.arena_var, command=lambda: self._trigger_cmd("TOGGLE_ARENA")
        ).pack(side="right", padx=4)

        frame_cam = ttk.LabelFrame(parent, text="📷 Caméra Frontale 3D (RGB & Profondeur Thermique)", padding=6)
        frame_cam.pack(fill="both", expand=True, pady=4)

        self.camera_widget = CameraFeedWidget(frame_cam, width=160, height=120)
        self.camera_widget.pack(fill="x")

        self._add_slider_row(frame_cam, "Orientation Cam Pan (°) :", self.cam_pan_deg_var, -75.0, 75.0, 0.0, resolution=1.0)
        self._add_slider_row(frame_cam, "Inclinaison Cam Tilt (°) :", self.cam_tilt_deg_var, -35.0, 35.0, -5.0, resolution=1.0)

        cam_bottom = ttk.Frame(frame_cam)
        cam_bottom.pack(fill="x", pady=2)
        ttk.Checkbutton(
            cam_bottom, text="Activer le flux caméra embarqué",
            variable=self.cam_enable_var
        ).pack(side="left", padx=4)
        ttk.Button(
            cam_bottom, text="🎯 Centrer Vue",
            command=lambda: [self.cam_pan_deg_var.set(0.0), self.cam_tilt_deg_var.set(-5.0)]
        ).pack(side="right", padx=4)

    def _trigger_cmd(self, cmd_name):
        if self.on_command_change:
            self.on_command_change(cmd_name)

    def _add_slider_row(self, parent, label_text, var, from_, to, default, resolution=0.05, scale_mult=1.0):
        row = ttk.Frame(parent)
        row.pack(fill="x", pady=2)

        lbl = ttk.Label(row, text=label_text, width=22)
        lbl.pack(side="left")

        val_lbl = ttk.Label(row, text=f"{default * scale_mult:.2f}", width=6)

        # Mise à jour automatique de l'étiquette même si modifiée par code/preset
        var.trace_add("write", lambda *args: val_lbl.config(text=f"{var.get() * scale_mult:.2f}"))

        def on_slide(val):
            self._notify_change()

        slider = ttk.Scale(
            row, from_=from_, to=to, variable=var,
            command=on_slide
        )
        slider.pack(side="left", fill="x", expand=True, padx=4)
        val_lbl.pack(side="left", padx=2)

        btn_zero = ttk.Button(
            row, text="0", width=3,
            command=lambda: [var.set(default), self._notify_change()]
        )
        btn_zero.pack(side="right")

    def _add_mini_slider(self, parent, label_text, var, from_, to):
        row = ttk.Frame(parent)
        row.pack(fill="x", pady=1)
        lbl = ttk.Label(row, text=label_text, width=8)
        lbl.pack(side="left")

        val_lbl = ttk.Label(row, text=f"{var.get():.2f}", width=5)
        var.trace_add("write", lambda *args: val_lbl.config(text=f"{var.get():.2f}"))

        def on_slide(val):
            self._notify_change()

        slider = ttk.Scale(row, from_=from_, to=to, variable=var, command=on_slide)
        slider.pack(side="left", fill="x", expand=True, padx=2)
        val_lbl.pack(side="right")

    def set_vel(self, vx, vy, wz):
        """Définit simultanément les consignes de vitesse."""
        self.manual_servo_mode_var.set(False)
        self.auto_avoid_var.set(False)
        self.vx_var.set(vx)
        self.vy_var.set(vy)
        self.wz_var.set(wz)
        self._notify_change()

    def cmd_stop(self):
        """Arrête toute vitesse et stabilise debout."""
        self.auto_avoid_var.set(False)
        self.vx_var.set(0.0)
        self.vy_var.set(0.0)
        self.wz_var.set(0.0)
        self.manual_servo_mode_var.set(False)
        self._notify_change()

    def cmd_reset(self):
        if self.on_command_change:
            self.on_command_change("RESET_SIM")

    def cmd_stand(self):
        """Posture debout nominale (0.0 cm)."""
        self.cmd_stop()
        self.reset_pose()
        self.body_z_var.set(0.0)
        if self.on_command_change:
            self.on_command_change("POSTURE_STAND")

    def cmd_rest(self):
        """Posture repos à plat au sol (-8.0 cm)."""
        self.cmd_stop()
        self.reset_pose()
        self.body_z_var.set(-0.08)
        if self.on_command_change:
            self.on_command_change("POSTURE_REST")

    def cmd_crouch(self):
        """Posture accroupie basse (-4.0 cm)."""
        self.cmd_stop()
        self.reset_pose()
        self.body_z_var.set(-0.04)
        if self.on_command_change:
            self.on_command_change("POSTURE_CROUCH")

    def cmd_jump(self):
        """Déclenche la séquence de saut dynamique."""
        if self.on_command_change:
            self.on_command_change("TRIGGER_JUMP")

    def reset_pose(self):
        self.body_z_var.set(0.0)
        self.roll_var.set(0.0)
        self.pitch_var.set(0.0)
        self.yaw_var.set(0.0)
        self._notify_change()

    def reset_servos_to_stand(self):
        for leg_id in range(1, 7):
            self.servo_vars[f"L{leg_id}_coxa"].set(0.0)
            self.servo_vars[f"L{leg_id}_femur"].set(0.4503)
            self.servo_vars[f"L{leg_id}_tibia"].set(0.8493)
        self._notify_change()

    def reset_all_controls(self):
        """Réinitialise l'ensemble des curseurs GUI et modes à zéro."""
        self.vx_var.set(0.0)
        self.vy_var.set(0.0)
        self.wz_var.set(0.0)
        self.body_z_var.set(0.0)
        self.roll_var.set(0.0)
        self.pitch_var.set(0.0)
        self.yaw_var.set(0.0)
        self.bench_mode_var.set(False)
        self.manual_servo_mode_var.set(False)
        self.reset_servos_to_stand()
        self._notify_change()

    def _notify_change(self, *args):
        if self.on_command_change:
            self.on_command_change("UPDATE")

    def get_state(self):
        """Retourne l'ensemble de l'état actuel sous forme de dictionnaire."""
        state = {
            "vx": self.vx_var.get(),
            "vy": self.vy_var.get(),
            "wz": self.wz_var.get(),
            "freq": self.speed_freq_var.get(),
            "step_h": self.step_height_var.get(),
            "step_l": self.step_length_var.get(),
            "body_z": self.body_z_var.get(),
            "roll": self.roll_var.get(),
            "pitch": self.pitch_var.get(),
            "yaw": self.yaw_var.get(),
            "bench_mode": self.bench_mode_var.get(),
            "manual_servos": self.manual_servo_mode_var.get(),
            "contacts": self.contacts_var.get(),
            "gravity": self.gravity_var.get(),
            "ax18_torque": self.ax18_torque_var.get(),
            "lidar_rays_3d": self.lidar_rays_3d_var.get(),
            "auto_avoid": self.auto_avoid_var.get(),
            "arena": self.arena_var.get(),
            "cam_enable": self.cam_enable_var.get(),
            "cam_mode": self.camera_widget.mode_var.get(),
            "cam_pan_rad": math.radians(self.cam_pan_deg_var.get()),
            "cam_tilt_rad": math.radians(self.cam_tilt_deg_var.get()),
            "servos": {k: v.get() for k, v in self.servo_vars.items()}
        }
        return state
