"""
Interface Graphique (Tkinter) de contrôle pour le robot Hiwonder MentorPi (ROS2 / RPi5).
Inclut :
- Bascule temps réel Châssis T1 (Chenilles Tank) <-> Châssis M1 (4 Roues Mecanum Omnidirectionnelles)
- Radar polaire 360° du LiDAR STL27L/LD19 + Mode d'évitement autonome d'obstacles
- Retour vidéo en direct (RGB & Carte de Profondeur 3D) de la caméra Pan-Tilt 2-DOF
"""
import math
import tkinter as tk
from tkinter import ttk
from sensor_widgets import LidarRadarWidget, CameraFeedWidget


class MentorPiControlGUI:
    def __init__(self, on_command_change=None, initial_chassis="T1"):
        self.root = tk.Tk()
        self.root.title("🤖 Poste de Commande - Hiwonder MentorPi (RPi5 + LiDAR + Caméra 3D)")
        self.root.geometry("560x900")
        self.root.resizable(True, True)

        self.on_command_change = on_command_change

        # Variables de pilotage châssis
        self.chassis_mode_var = tk.StringVar(value=initial_chassis.upper())
        self.vx_var = tk.DoubleVar(value=0.0)
        self.vy_var = tk.DoubleVar(value=0.0)
        self.wz_var = tk.DoubleVar(value=0.0)
        self.speed_scale_var = tk.DoubleVar(value=1.0)

        # Variables Caméra Pan-Tilt 2-DOF (en degrés dans l'interface)
        self.cam_pan_deg_var = tk.DoubleVar(value=0.0)
        self.cam_tilt_deg_var = tk.DoubleVar(value=0.0)
        self.cam_autoscan_var = tk.BooleanVar(value=False)
        self.cam_enable_var = tk.BooleanVar(value=True)

        # Variables LiDAR & Autonomie
        self.lidar_rpm_var = tk.DoubleVar(value=12.0)  # rad/s
        self.lidar_rays_3d_var = tk.BooleanVar(value=True)
        self.auto_avoid_var = tk.BooleanVar(value=False)
        self.arena_var = tk.BooleanVar(value=True)

        self._build_ui()

    def _build_ui(self):
        # Barre supérieure d'en-tête + bouton retour au Menu Principal
        header = tk.Frame(self.root, bg="#1e272e", padx=10, pady=6)
        header.pack(fill="x")

        lbl_title = tk.Label(
            header, text="🤖 HIWONDER MENTORPI (RPi5)",
            bg="#1e272e", fg="#00d2d3", font=("Arial", 11, "bold")
        )
        lbl_title.pack(side="left")

        btn_menu = tk.Button(
            header, text="🔀 Menu Robots",
            bg="#0984e3", fg="white", font=("Arial", 9, "bold"),
            padx=8, pady=2, command=self.cmd_switch_menu
        )
        btn_menu.pack(side="right", padx=3)

        btn_hexapod = tk.Button(
            header, text="🕷️ Basculer vers Hexapode",
            bg="#6c5ce7", fg="white", font=("Arial", 9, "bold"),
            padx=8, pady=2, command=self.cmd_switch_hexapod
        )
        btn_hexapod.pack(side="right", padx=3)

        notebook = ttk.Notebook(self.root)
        notebook.pack(fill="both", expand=True, padx=8, pady=6)

        # Onglet 1 : Pilotage, Châssis T1/M1 & Caméra Pan-Tilt
        tab_drive = ttk.Frame(notebook, padding=8)
        notebook.add(tab_drive, text="🚗 Pilotage & Châssis (T1/M1)")
        self._build_drive_tab(tab_drive)

        # Onglet 2 : Capteurs (LiDAR 360° & Caméra 3D RGB-D)
        tab_sensors = ttk.Frame(notebook, padding=8)
        notebook.add(tab_sensors, text="📡 LiDAR 360° & Caméra 3D")
        self._build_sensors_tab(tab_sensors)

    def _build_drive_tab(self, parent):
        # 1. Choix de la configuration du châssis (T1 Chenilles vs M1 Mecanum)
        frame_chassis = ttk.LabelFrame(parent, text="⚙️ Configuration Mécanique du Châssis", padding=8)
        frame_chassis.pack(fill="x", pady=4)

        rb_t1 = ttk.Radiobutton(
            frame_chassis,
            text="🚜 MentorPi T1 : Châssis Chenilles Tout-Terrain (Tank Skid-Steer)",
            variable=self.chassis_mode_var, value="T1",
            command=self._on_chassis_change
        )
        rb_t1.pack(anchor="w", pady=2)

        rb_m1 = ttk.Radiobutton(
            frame_chassis,
            text="🛞 MentorPi M1 : 4 Roues Mecanum Omnidirectionnelles (Holonome X/Y/Yaw)",
            variable=self.chassis_mode_var, value="M1",
            command=self._on_chassis_change
        )
        rb_m1.pack(anchor="w", pady=2)

        self.lbl_chassis_hint = ttk.Label(
            frame_chassis,
            text="Mode actif : T1 Chenilles (Différentiel avant/arrière + rotation sur place)",
            font=("Arial", 8, "italic"), foreground="#0984e3"
        )
        self.lbl_chassis_hint.pack(anchor="w", pady=(3, 0))

        # 2. Pavé Directionnel (8 directions + Rotations + Stop)
        frame_pad = ttk.LabelFrame(parent, text="🕹️ Pavé Directionnel & Mouvements Rapides", padding=8)
        frame_pad.pack(fill="x", pady=5)

        pad_grid = ttk.Frame(frame_pad)
        pad_grid.pack()

        btn_cfg = {"width": 13, "padding": 4}
        ttk.Button(pad_grid, text="↖ Diag Av-G", command=lambda: self.set_vel(0.6, 0.5, 0.0), **btn_cfg).grid(row=0, column=0, padx=3, pady=3)
        ttk.Button(pad_grid, text="⬆️ Avancer (Z/↑)", command=lambda: self.set_vel(0.8, 0.0, 0.0), **btn_cfg).grid(row=0, column=1, padx=3, pady=3)
        ttk.Button(pad_grid, text="↗ Diag Av-D", command=lambda: self.set_vel(0.6, -0.5, 0.0), **btn_cfg).grid(row=0, column=2, padx=3, pady=3)

        ttk.Button(pad_grid, text="⬅️ Crabe G (A)", command=lambda: self.set_vel(0.0, 0.7, 0.0), **btn_cfg).grid(row=1, column=0, padx=3, pady=3)
        btn_stop = tk.Button(
            pad_grid, text="🛑 STOP (Espace)", bg="#d63031", fg="white",
            font=("Arial", 9, "bold"), width=14, pady=5, command=self.cmd_stop
        )
        btn_stop.grid(row=1, column=1, padx=3, pady=3)
        ttk.Button(pad_grid, text="➡️ Crabe D (E)", command=lambda: self.set_vel(0.0, -0.7, 0.0), **btn_cfg).grid(row=1, column=2, padx=3, pady=3)

        ttk.Button(pad_grid, text="🔄 Pivot G (Q/←)", command=lambda: self.set_vel(0.0, 0.0, 0.8), **btn_cfg).grid(row=2, column=0, padx=3, pady=3)
        ttk.Button(pad_grid, text="⬇️ Reculer (S/↓)", command=lambda: self.set_vel(-0.8, 0.0, 0.0), **btn_cfg).grid(row=2, column=1, padx=3, pady=3)
        ttk.Button(pad_grid, text="🔃 Pivot D (D/→)", command=lambda: self.set_vel(0.0, 0.0, -0.8), **btn_cfg).grid(row=2, column=2, padx=3, pady=3)

        # 3. Curseurs de vitesse continus
        frame_vel = ttk.LabelFrame(parent, text="🎚️ Consignes de Vitesse Continues", padding=8)
        frame_vel.pack(fill="x", pady=5)

        self._add_slider_row(frame_vel, "Vitesse Avant Vx (m/s) :", self.vx_var, -1.0, 1.0, 0.0)
        self._add_slider_row(frame_vel, "Vitesse Latérale Vy (M1) :", self.vy_var, -1.0, 1.0, 0.0)
        self._add_slider_row(frame_vel, "Rotation Lacet Wz (rad/s) :", self.wz_var, -1.5, 1.5, 0.0)
        self._add_slider_row(frame_vel, "Multiplicateur Turbo :", self.speed_scale_var, 0.3, 2.0, 1.0)

        # 4. Contrôle Tourelle Caméra Pan-Tilt 2-DOF
        frame_pt = ttk.LabelFrame(parent, text="🎥 Tourelle Caméra 3D Pan-Tilt (2 Servos)", padding=8)
        frame_pt.pack(fill="x", pady=5)

        self._add_slider_row(frame_pt, "Pan / Lacet Caméra (°) :", self.cam_pan_deg_var, -90.0, 90.0, 0.0, resolution=1.0)
        self._add_slider_row(frame_pt, "Tilt / Tangage Caméra (°) :", self.cam_tilt_deg_var, -45.0, 60.0, 0.0, resolution=1.0)

        pt_btns = ttk.Frame(frame_pt)
        pt_btns.pack(fill="x", pady=3)
        ttk.Checkbutton(
            pt_btns, text="🔄 Balayage Panoramique Auto (Scan_Demo)",
            variable=self.cam_autoscan_var, command=self._notify_change
        ).pack(side="left", padx=4)
        ttk.Button(
            pt_btns, text="🎯 Centrer Caméra",
            command=self.center_camera
        ).pack(side="right", padx=4)

        # 5. Modes intelligents & Arène
        frame_modes = ttk.LabelFrame(parent, text="🧭 Navigation Autonome & Environnement de Test", padding=8)
        frame_modes.pack(fill="x", pady=5)

        chk_avoid = ttk.Checkbutton(
            frame_modes,
            text="🤖 Activer l'Exploration Autonome avec Évitement d'Obstacles LiDAR",
            variable=self.auto_avoid_var,
            command=self._notify_change
        )
        chk_avoid.pack(anchor="w", pady=2)

        chk_arena = ttk.Checkbutton(
            frame_modes,
            text="🧱 Afficher l'Arène d'Obstacles 3D (Piliers & Murs de test)",
            variable=self.arena_var,
            command=lambda: self._trigger_cmd("TOGGLE_ARENA")
        )
        chk_arena.pack(anchor="w", pady=2)

        btn_reset = tk.Button(
            frame_modes, text="🔄 Réinitialiser le Robot au Centre (R)",
            bg="#f39c12", fg="white", font=("Arial", 9, "bold"), pady=4,
            command=lambda: self._trigger_cmd("RESET_SIM")
        )
        btn_reset.pack(fill="x", pady=4)

    def _build_sensors_tab(self, parent):
        # Section 1 : Radar LiDAR 360° STL27L
        frame_lidar = ttk.LabelFrame(parent, text="📡 LiDAR 360° ToF (STL27L / LD19) - Radar Temps Réel", padding=6)
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

        # Section 2 : Flux Vidéo Caméra 3D Depth
        frame_cam = ttk.LabelFrame(parent, text="📷 Caméra 3D Depth (RGB + Carte de Profondeur Thermique)", padding=6)
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

    def _add_slider_row(self, parent, label_text, var, from_, to, default, resolution=0.05):
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

        btn_zero = ttk.Button(
            row, text="0", width=3,
            command=lambda: [var.set(default), self._notify_change()]
        )
        btn_zero.pack(side="right")

    def _on_chassis_change(self):
        mode = self.chassis_mode_var.get()
        if mode == "T1":
            self.vy_var.set(0.0)
            self.lbl_chassis_hint.config(
                text="Mode actif : T1 Chenilles (Différentiel avant/arrière + rotation sur place)"
            )
        else:
            self.lbl_chassis_hint.config(
                text="Mode actif : M1 Mecanum (Holonome 3-DOF : Avant/Arrière + Crabe Vy + Rotation)"
            )
        self._trigger_cmd("CHASSIS_CHANGE")

    def _toggle_arena_btn(self):
        self.arena_var.set(not self.arena_var.get())
        self._trigger_cmd("TOGGLE_ARENA")

    def set_vel(self, vx, vy, wz):
        self.auto_avoid_var.set(False)
        if self.chassis_mode_var.get() == "T1":
            vy = 0.0  # Les chenilles ne se déplacent pas en crabe latéral
        self.vx_var.set(vx)
        self.vy_var.set(vy)
        self.wz_var.set(wz)
        self._notify_change()

    def cmd_stop(self):
        self.auto_avoid_var.set(False)
        self.vx_var.set(0.0)
        self.vy_var.set(0.0)
        self.wz_var.set(0.0)
        self._notify_change()

    def center_camera(self):
        self.cam_autoscan_var.set(False)
        self.cam_pan_deg_var.set(0.0)
        self.cam_tilt_deg_var.set(0.0)
        self._notify_change()

    def cmd_switch_menu(self):
        self._trigger_cmd("SWITCH_MENU")

    def cmd_switch_hexapod(self):
        self._trigger_cmd("SWITCH_HEXAPOD")

    def _trigger_cmd(self, cmd_name):
        if self.on_command_change:
            self.on_command_change(cmd_name)

    def _notify_change(self, *args):
        if self.on_command_change:
            self.on_command_change("UPDATE")

    def get_state(self):
        return {
            "chassis_mode": self.chassis_mode_var.get(),
            "vx": self.vx_var.get() * self.speed_scale_var.get(),
            "vy": (0.0 if self.chassis_mode_var.get() == "T1" else self.vy_var.get()) * self.speed_scale_var.get(),
            "wz": self.wz_var.get() * self.speed_scale_var.get(),
            "cam_pan_rad": math.radians(self.cam_pan_deg_var.get()),
            "cam_tilt_rad": math.radians(self.cam_tilt_deg_var.get()),
            "cam_autoscan": self.cam_autoscan_var.get(),
            "cam_enable": self.cam_enable_var.get(),
            "cam_mode": self.camera_widget.mode_var.get(),
            "lidar_speed": self.lidar_rpm_var.get(),
            "lidar_rays_3d": self.lidar_rays_3d_var.get(),
            "auto_avoid": self.auto_avoid_var.get(),
            "arena": self.arena_var.get(),
        }

