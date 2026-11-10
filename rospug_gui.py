"""
Interface Graphique (Tkinter) de contrôle pour le robot chien quadrupède Hiwonder ROSPug (12-DOF / Jetson Nano).
Inclut :
- Sélection d'allure quadrupède (Trot, Marche 4-temps, Amble/Pace) & Postures bioniques (Debout, Assis, Salut/Bow, Accroupi)
- Contrôle d'assiette du corps (Hauteur Z, Roulis, Tangage) & Vitesse omnidirectionnelle (Vx, Vy, Wz)
- Radar polaire 360° du LiDAR ToF + Mode d'évitement autonome d'obstacles
- Retour vidéo en direct (RGB & Carte de Profondeur 3D) de la caméra grand-angle HD 8MP
"""
import math
import tkinter as tk
from tkinter import ttk
from sensor_widgets import LidarRadarWidget, CameraFeedWidget


class RosPugControlGUI:
    def __init__(self, on_command_change=None):
        self.root = tk.Tk()
        self.root.title("🐕 Poste de Commande - Hiwonder ROSPug (Quadrupède 12-DOF + LiDAR + Caméra HD)")
        self.root.geometry("580x920")
        self.root.resizable(True, True)

        self.on_command_change = on_command_change

        # Variables de locomotion et d'allure
        self.gait_mode_var = tk.StringVar(value="TROT")
        self.posture_var = tk.StringVar(value="STAND")
        self.vx_var = tk.DoubleVar(value=0.0)
        self.vy_var = tk.DoubleVar(value=0.0)
        self.wz_var = tk.DoubleVar(value=0.0)
        self.speed_scale_var = tk.DoubleVar(value=1.0)
        self.step_height_var = tk.DoubleVar(value=0.35)

        # Variables d'assiette du châssis (IMU / Self-Balancing)
        self.body_z_deg_var = tk.DoubleVar(value=0.0)
        self.roll_deg_var = tk.DoubleVar(value=0.0)
        self.pitch_deg_var = tk.DoubleVar(value=0.0)

        # Variables LiDAR & Caméra HD
        self.lidar_rpm_var = tk.DoubleVar(value=12.0)
        self.lidar_rays_3d_var = tk.BooleanVar(value=True)
        self.auto_avoid_var = tk.BooleanVar(value=False)
        self.arena_var = tk.BooleanVar(value=True)
        self.cam_enable_var = tk.BooleanVar(value=True)

        self._build_ui()

    def _build_ui(self):
        header = tk.Frame(self.root, bg="#1e272e", padx=10, pady=6)
        header.pack(fill="x")

        tk.Label(
            header, text="🐕 HIWONDER ROSPUG (12-DOF • JETSON NANO)",
            bg="#1e272e", fg="#ff6b1a", font=("Arial", 11, "bold")
        ).pack(side="left")

        tk.Button(
            header, text="🔀 Menu Robots",
            bg="#0984e3", fg="white", font=("Arial", 9, "bold"),
            padx=8, pady=2, command=lambda: self._trigger_cmd("SWITCH_MENU")
        ).pack(side="right", padx=3)

        tk.Button(
            header, text="🕷️ Hexapode",
            bg="#6c5ce7", fg="white", font=("Arial", 9, "bold"),
            padx=6, pady=2, command=lambda: self._trigger_cmd("SWITCH_HEXAPOD")
        ).pack(side="right", padx=2)

        tk.Button(
            header, text="🚜 MentorPi",
            bg="#00a8ff", fg="white", font=("Arial", 9, "bold"),
            padx=6, pady=2, command=lambda: self._trigger_cmd("SWITCH_MENTORPI")
        ).pack(side="right", padx=2)

        notebook = ttk.Notebook(self.root)
        notebook.pack(fill="both", expand=True, padx=8, pady=6)

        tab_drive = ttk.Frame(notebook, padding=8)
        notebook.add(tab_drive, text="🐾 Locomotion 12-DOF & Postures")
        self._build_drive_tab(tab_drive)

        tab_sensors = ttk.Frame(notebook, padding=8)
        notebook.add(tab_sensors, text="📡 LiDAR ToF 360° & Caméra HD")
        self._build_sensors_tab(tab_sensors)

    def _build_drive_tab(self, parent):
        # 1. Allure Quadrupède & Postures Bioniques
        frame_gait = ttk.LabelFrame(parent, text="🐕 Allure Cinématique & Postures Bioniques", padding=8)
        frame_gait.pack(fill="x", pady=4)

        gait_row = ttk.Frame(frame_gait)
        gait_row.pack(fill="x", pady=2)
        ttk.Label(gait_row, text="Allure (Gait) :", font=("Arial", 9, "bold")).pack(side="left", padx=(0, 8))
        for code, label in [
            ("TROT", "⚡ Trot Dynamique"),
            ("WALK", "🐾 Marche (Walk)"),
            ("PACE", "🌊 Amble (Pace)"),
        ]:
            ttk.Radiobutton(
                gait_row, text=label, variable=self.gait_mode_var, value=code,
                command=self._notify_change
            ).pack(side="left", padx=6)

        pose_row = ttk.Frame(frame_gait)
        pose_row.pack(fill="x", pady=(6, 2))
        for code, label, color in [
            ("STAND", "🐕 Debout (Stand)", "#2ecc71"),
            ("CROUCH", "🐾 Accroupi (Crouch)", "#e67e22"),
            ("SIT", "🐶 Assis (Sit)", "#3498db"),
            ("BOW", "🙇 Salut (Play Bow)", "#9b59b6"),
        ]:
            tk.Button(
                pose_row, text=label, bg=color, fg="white",
                font=("Arial", 9, "bold"), padx=6, pady=4,
                command=lambda c=code: self.set_posture(c)
            ).pack(side="left", expand=True, fill="x", padx=2)

        # 2. Pavé Directionnel Omnidirectionnel
        frame_pad = ttk.LabelFrame(parent, text="🕹️ Pavé Directionnel Omnidirectionnel (Marche & Crabe)", padding=8)
        frame_pad.pack(fill="x", pady=5)

        pad_grid = ttk.Frame(frame_pad)
        pad_grid.pack()

        btn_cfg = {"width": 13, "padding": 4}
        ttk.Button(pad_grid, text="↖ Diag Av-G", command=lambda: self.set_vel(0.6, 0.5, 0.0), **btn_cfg).grid(row=0, column=0, padx=3, pady=3)
        ttk.Button(pad_grid, text="⬆️ Avancer (Z/↑)", command=lambda: self.set_vel(0.8, 0.0, 0.0), **btn_cfg).grid(row=0, column=1, padx=3, pady=3)
        ttk.Button(pad_grid, text="↗ Diag Av-D", command=lambda: self.set_vel(0.6, -0.5, 0.0), **btn_cfg).grid(row=0, column=2, padx=3, pady=3)

        ttk.Button(pad_grid, text="⬅️ Pas Crabe G (A)", command=lambda: self.set_vel(0.0, 0.7, 0.0), **btn_cfg).grid(row=1, column=0, padx=3, pady=3)
        tk.Button(
            pad_grid, text="🛑 STOP (Espace)", bg="#d63031", fg="white",
            font=("Arial", 9, "bold"), width=14, pady=5, command=self.cmd_stop
        ).grid(row=1, column=1, padx=3, pady=3)
        ttk.Button(pad_grid, text="➡️ Pas Crabe D (E)", command=lambda: self.set_vel(0.0, -0.7, 0.0), **btn_cfg).grid(row=1, column=2, padx=3, pady=3)

        ttk.Button(pad_grid, text="🔄 Pivot G (Q/←)", command=lambda: self.set_vel(0.0, 0.0, 0.8), **btn_cfg).grid(row=2, column=0, padx=3, pady=3)
        ttk.Button(pad_grid, text="⬇️ Reculer (S/↓)", command=lambda: self.set_vel(-0.8, 0.0, 0.0), **btn_cfg).grid(row=2, column=1, padx=3, pady=3)
        ttk.Button(pad_grid, text="🔃 Pivot D (D/→)", command=lambda: self.set_vel(0.0, 0.0, -0.8), **btn_cfg).grid(row=2, column=2, padx=3, pady=3)

        # 3. Curseurs de vitesse continus
        frame_vel = ttk.LabelFrame(parent, text="🎚️ Consignes de Vitesse & Amplitude de Foulée", padding=8)
        frame_vel.pack(fill="x", pady=5)

        self._add_slider_row(frame_vel, "Vitesse Avant Vx (m/s) :", self.vx_var, -1.0, 1.0, 0.0)
        self._add_slider_row(frame_vel, "Pas Latéral Vy (m/s) :", self.vy_var, -1.0, 1.0, 0.0)
        self._add_slider_row(frame_vel, "Rotation Lacet Wz (rad/s) :", self.wz_var, -1.5, 1.5, 0.0)
        self._add_slider_row(frame_vel, "Amplitude Foulée (rad) :", self.step_height_var, 0.15, 0.65, 0.35)
        self._add_slider_row(frame_vel, "Fréquence / Cadence :", self.speed_scale_var, 0.4, 2.0, 1.0)

        # 4. Contrôle d'Assiette IMU / Posture 6-DOF
        frame_pose = ttk.LabelFrame(parent, text="⚖️ Contrôle d'Assiette & Équilibre IMU (Roulis / Tangage / Hauteur)", padding=8)
        frame_pose.pack(fill="x", pady=5)

        self._add_slider_row(frame_pose, "Hauteur Châssis (°) :", self.body_z_deg_var, -30.0, 25.0, 0.0)
        self._add_slider_row(frame_pose, "Roulis / Roll (°) :", self.roll_deg_var, -25.0, 25.0, 0.0)
        self._add_slider_row(frame_pose, "Tangage / Pitch (°) :", self.pitch_deg_var, -25.0, 25.0, 0.0)

        # 5. Modes intelligents & Arène
        frame_modes = ttk.LabelFrame(parent, text="🧭 Navigation Autonome ROS & Environnement", padding=8)
        frame_modes.pack(fill="x", pady=5)

        ttk.Checkbutton(
            frame_modes,
            text="🤖 Activer l'Exploration Autonome avec Évitement d'Obstacles LiDAR ToF",
            variable=self.auto_avoid_var, command=self._notify_change
        ).pack(anchor="w", pady=2)

        ttk.Checkbutton(
            frame_modes,
            text="🧱 Afficher l'Arène d'Obstacles 3D (Piliers & Murs de test)",
            variable=self.arena_var, command=lambda: self._trigger_cmd("TOGGLE_ARENA")
        ).pack(anchor="w", pady=2)

        tk.Button(
            frame_modes, text="🔄 Réinitialiser le Robot au Centre (R)",
            bg="#f39c12", fg="white", font=("Arial", 9, "bold"), pady=4,
            command=lambda: self._trigger_cmd("RESET_SIM")
        ).pack(fill="x", pady=4)

    def _build_sensors_tab(self, parent):
        frame_lidar = ttk.LabelFrame(parent, text="📡 LiDAR ToF 360° - Radar SLAM Temps Réel", padding=6)
        frame_lidar.pack(fill="x", pady=4)

        self.radar_widget = LidarRadarWidget(frame_lidar, size=205)
        self.radar_widget.pack(fill="x")

        lidar_ctrl = ttk.Frame(frame_lidar)
        lidar_ctrl.pack(fill="x", pady=3)
        ttk.Checkbutton(
            lidar_ctrl, text="✨ Afficher les faisceaux laser 3D dans PyBullet",
            variable=self.lidar_rays_3d_var, command=self._notify_change
        ).pack(side="left", padx=4)
        ttk.Checkbutton(
            lidar_ctrl, text="🤖 Évitement Autonome LiDAR",
            variable=self.auto_avoid_var, command=self._notify_change
        ).pack(side="right", padx=4)

        self._add_slider_row(frame_lidar, "Vitesse Tourelle LiDAR :", self.lidar_rpm_var, 0.0, 25.0, 12.0)

        frame_cam = ttk.LabelFrame(parent, text="📷 Caméra Frontale Grand-Angle HD 8MP (RGB + Profondeur 3D)", padding=6)
        frame_cam.pack(fill="both", expand=True, pady=4)

        self.camera_widget = CameraFeedWidget(frame_cam, width=160, height=120)
        self.camera_widget.pack(fill="x")

        cam_ctrl = ttk.Frame(frame_cam)
        cam_ctrl.pack(fill="x", pady=2)
        ttk.Checkbutton(
            cam_ctrl, text="Activer le rendu vidéo embarqué",
            variable=self.cam_enable_var
        ).pack(side="left", padx=4)
        ttk.Button(
            cam_ctrl, text="🧱 Obstacles On/Off",
            command=self._toggle_arena_btn
        ).pack(side="right", padx=4)

    def _add_slider_row(self, parent, label_text, var, from_, to, default):
        row = ttk.Frame(parent)
        row.pack(fill="x", pady=2)

        lbl = ttk.Label(row, text=label_text, width=23)
        lbl.pack(side="left")

        val_lbl = ttk.Label(row, text=f"{default:.2f}", width=6)
        var.trace_add("write", lambda *args: val_lbl.config(text=f"{var.get():.2f}"))

        slider = ttk.Scale(
            row, from_=from_, to=to, variable=var,
            command=lambda _v: self._notify_change()
        )
        slider.pack(side="left", fill="x", expand=True, padx=4)
        val_lbl.pack(side="left", padx=2)

        ttk.Button(
            row, text="0", width=3,
            command=lambda: [var.set(default), self._notify_change()]
        ).pack(side="right")

    def _toggle_arena_btn(self):
        self.arena_var.set(not self.arena_var.get())
        self._trigger_cmd("TOGGLE_ARENA")

    def set_posture(self, pose_code):
        self.posture_var.set(pose_code)
        if pose_code in ("SIT", "BOW"):
            self.vx_var.set(0.0)
            self.vy_var.set(0.0)
            self.wz_var.set(0.0)
        self._notify_change()

    def set_vel(self, vx, vy, wz):
        self.auto_avoid_var.set(False)
        if self.posture_var.get() in ("SIT", "BOW"):
            self.posture_var.set("STAND")
        self.vx_var.set(vx)
        self.vy_var.set(vy)
        self.wz_var.set(wz)
        self._notify_change()

    def cmd_stop(self):
        self.auto_avoid_var.set(False)
        self.vx_var.set(0.0)
        self.vy_var.set(0.0)
        self.wz_var.set(0.0)
        self.posture_var.set("STAND")
        self._notify_change()

    def _trigger_cmd(self, cmd_name):
        if self.on_command_change:
            self.on_command_change(cmd_name)

    def _notify_change(self, *args):
        if self.on_command_change:
            self.on_command_change("UPDATE")

    def get_state(self):
        return {
            "gait_mode": self.gait_mode_var.get(),
            "posture": self.posture_var.get(),
            "vx": self.vx_var.get() * self.speed_scale_var.get(),
            "vy": self.vy_var.get() * self.speed_scale_var.get(),
            "wz": self.wz_var.get() * self.speed_scale_var.get(),
            "step_amp": self.step_height_var.get(),
            "freq_scale": self.speed_scale_var.get(),
            "body_z_rad": math.radians(self.body_z_deg_var.get()),
            "roll_rad": math.radians(self.roll_deg_var.get()),
            "pitch_rad": math.radians(self.pitch_deg_var.get()),
            "lidar_speed": self.lidar_rpm_var.get(),
            "lidar_rays_3d": self.lidar_rays_3d_var.get(),
            "auto_avoid": self.auto_avoid_var.get(),
            "cam_enable": self.cam_enable_var.get(),
            "cam_mode": self.camera_widget.mode_var.get(),
            "arena": self.arena_var.get(),
        }
