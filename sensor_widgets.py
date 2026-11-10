"""
Module partagé de capteurs embarqués (LiDAR 360° STL27L/LD19 & Caméra 3D RGB-D)
et gestionnaire d'arène d'obstacles pour les simulations PyBullet (MentorPi & Hexapode).
"""
import math
import tkinter as tk
from tkinter import ttk
import numpy as np
import pybullet as p


class ObstacleArenaManager:
    """Gère l'apparition et la suppression d'obstacles 3D autour du robot pour tester LiDAR et Caméra."""

    def __init__(self):
        self.obstacle_ids = []
        self.active = False

    def toggle_arena(self, enable=None):
        if enable is None:
            enable = not self.active
        if enable and not self.active:
            self.spawn_arena()
        elif not enable and self.active:
            self.clear_arena()
        return self.active

    def spawn_arena(self):
        self.clear_arena()
        # Définition d'obstacles variés (piliers, murs, caisses colorées) autour de (0, 0)
        specs = [
            # (type, pos_xyz, half_extents_or_radius_height, rgba)
            ("box", [0.85, 0.0, 0.15], [0.08, 0.25, 0.15], [0.90, 0.25, 0.20, 1.0]),      # Mur rouge face avant
            ("cyl", [0.65, 0.55, 0.18], (0.09, 0.36), [0.20, 0.75, 0.35, 1.0]),           # Pilier vert avant-gauche
            ("cyl", [0.60, -0.55, 0.18], (0.09, 0.36), [0.15, 0.55, 0.95, 1.0]),          # Pilier bleu avant-droit
            ("box", [-0.80, 0.35, 0.14], [0.14, 0.14, 0.14], [0.95, 0.65, 0.10, 1.0]),    # Cube orange arrière-gauche
            ("box", [-0.75, -0.45, 0.16], [0.18, 0.10, 0.16], [0.65, 0.30, 0.85, 1.0]),   # Bloc violet arrière-droit
            ("cyl", [0.0, 0.90, 0.20], (0.11, 0.40), [0.10, 0.80, 0.80, 1.0]),            # Pilier cyan gauche
            ("cyl", [0.0, -0.90, 0.20], (0.11, 0.40), [0.95, 0.85, 0.15, 1.0]),           # Pilier jaune droite
            ("box", [1.35, 0.45, 0.22], [0.12, 0.35, 0.22], [0.30, 0.35, 0.42, 1.0]),     # Cloison grise lointaine
        ]
        for kind, pos, dims, rgba in specs:
            if kind == "box":
                col = p.createCollisionShape(p.GEOM_BOX, halfExtents=dims)
                vis = p.createVisualShape(p.GEOM_BOX, halfExtents=dims, rgbaColor=rgba)
            else:
                radius, height = dims
                col = p.createCollisionShape(p.GEOM_CYLINDER, radius=radius, height=height)
                vis = p.createVisualShape(p.GEOM_CYLINDER, radius=radius, length=height, rgbaColor=rgba)
            body_id = p.createMultiBody(
                baseMass=0,  # Statique
                baseCollisionShapeIndex=col,
                baseVisualShapeIndex=vis,
                basePosition=pos
            )
            self.obstacle_ids.append(body_id)
        self.active = True

    def clear_arena(self):
        for bid in self.obstacle_ids:
            try:
                p.removeBody(bid)
            except Exception:
                pass
        self.obstacle_ids.clear()
        self.active = False


class LidarScanner:
    """Simulateur de LiDAR 2D/360° (type STL27L / LD19) par lancer de rayons PyBullet."""

    def __init__(self, num_rays=72, max_range=2.20, min_range=0.08):
        self.num_rays = num_rays
        self.max_range = max_range
        self.min_range = min_range
        self.angles_rad = np.linspace(0.0, 2.0 * math.pi, num_rays, endpoint=False)
        self.distances = np.full(num_rays, max_range, dtype=np.float32)
        self.hit_flags = np.zeros(num_rays, dtype=bool)
        self.debug_line_ids = []
        self.sweep_phase = 0.0

    def clear_debug_lines(self):
        for lid in self.debug_line_ids:
            try:
                p.removeUserDebugItem(lid)
            except Exception:
                pass
        self.debug_line_ids.clear()

    def scan(self, sensor_pos, base_yaw, draw_3d_rays=False):
        """
        Exécute un balayage 360° autour de `sensor_pos` orienté selon `base_yaw`.
        Retourne un dictionnaire de télémétrie (distances par secteur, min_dist, etc.).
        """
        self.sweep_phase = (self.sweep_phase + 0.35) % (2.0 * math.pi)
        ray_froms = []
        ray_tos = []

        sx, sy, sz = sensor_pos
        for ang in self.angles_rad:
            world_ang = base_yaw + ang
            ca = math.cos(world_ang)
            sa = math.sin(world_ang)
            ray_froms.append([sx + self.min_range * ca, sy + self.min_range * sa, sz])
            ray_tos.append([sx + self.max_range * ca, sy + self.max_range * sa, sz])

        results = p.rayTestBatch(ray_froms, ray_tos)
        span = self.max_range - self.min_range
        for i, res in enumerate(results):
            hit_obj = res[0]
            hit_frac = res[2]
            if hit_obj != -1 and hit_frac < 0.999:
                self.distances[i] = self.min_range + hit_frac * span
                self.hit_flags[i] = True
            else:
                self.distances[i] = self.max_range
                self.hit_flags[i] = False

        # Affichage 3D optionnel dans PyBullet (1 rayon sur 3 pour garder 240 Hz fluide)
        if draw_3d_rays:
            step = 3
            needed = list(range(0, self.num_rays, step))
            while len(self.debug_line_ids) < len(needed):
                lid = p.addUserDebugLine([0, 0, 0], [0, 0, 0], [0, 1, 0], lineWidth=1.2)
                self.debug_line_ids.append(lid)
            for idx_slot, ray_idx in enumerate(needed):
                ang = base_yaw + self.angles_rad[ray_idx]
                d = float(self.distances[ray_idx])
                p0 = ray_froms[ray_idx]
                p1 = [sx + d * math.cos(ang), sy + d * math.sin(ang), sz]
                if self.hit_flags[ray_idx]:
                    color = [1.0, 0.25, 0.15] if d < 0.55 else [1.0, 0.75, 0.10]
                else:
                    color = [0.05, 0.75, 0.95]
                p.addUserDebugLine(
                    p0, p1, lineColorRGB=color, lineWidth=1.5,
                    replaceItemUniqueId=self.debug_line_ids[idx_slot]
                )
        elif self.debug_line_ids:
            self.clear_debug_lines()

        return self.get_telemetry()

    def get_telemetry(self):
        """Calcule les distances minimales par secteur (Avant, Gauche, Droite, Arrière)."""
        # Angles en degrés dans [0, 360) : 0 = Avant, 90 = Gauche, 180 = Arrière, 270 = Droite
        degs = np.degrees(self.angles_rad)
        front_mask = (degs <= 35.0) | (degs >= 325.0)
        left_mask = (degs > 35.0) & (degs <= 135.0)
        rear_mask = (degs > 135.0) & (degs < 225.0)
        right_mask = (degs >= 225.0) & (degs < 325.0)

        front_d = float(np.min(self.distances[front_mask]))
        left_d = float(np.min(self.distances[left_mask]))
        rear_d = float(np.min(self.distances[rear_mask]))
        right_d = float(np.min(self.distances[right_mask]))
        min_idx = int(np.argmin(self.distances))
        min_d = float(self.distances[min_idx])
        min_ang = float(degs[min_idx])
        if min_ang > 180.0:
            min_ang -= 360.0

        return {
            "front": front_d,
            "left": left_d,
            "right": right_d,
            "rear": rear_d,
            "min_dist": min_d,
            "min_angle_deg": min_ang,
        }

    def compute_avoidance_vel(self, forward_speed=0.45, turn_speed=0.75, safe_dist=0.58):
        """
        Génère des consignes (vx, vy, wz) d'exploration autonome avec évitement d'obstacles LiDAR.
        """
        tel = self.get_telemetry()
        if tel["front"] < safe_dist:
            # Obstacle devant : ralentit/recule légèrement et tourne vers le côté le plus dégagé
            turn_dir = 1.0 if tel["left"] >= tel["right"] else -1.0
            vx = -0.12 if tel["front"] < (safe_dist * 0.65) else 0.0
            wz = turn_dir * turn_speed
            return vx, 0.0, wz
        else:
            # Légère répulsion latérale si un mur est très proche à gauche ou à droite
            wz = 0.0
            if tel["left"] < 0.40:
                wz -= 0.35
            if tel["right"] < 0.40:
                wz += 0.35
            return forward_speed, 0.0, wz


class OnboardCameraRenderer:
    """Rendu d'image caméra embarquée PyBullet (RGB + Carte de Profondeur 3D) vers PPM Tkinter."""

    def __init__(self, width=160, height=120, fov=65.0, near=0.05, far=3.50):
        self.width = width
        self.height = height
        self.fov = fov
        self.near = near
        self.far = far
        self.proj_matrix = p.computeProjectionMatrixFOV(
            fov=self.fov,
            aspect=float(self.width) / float(self.height),
            nearVal=self.near,
            farVal=self.far
        )
        self.last_center_depth_m = self.far

    def render_ppm(self, cam_pos, cam_yaw, cam_pitch, mode="RGB"):
        """
        Rend une image depuis `cam_pos` orientée selon `(cam_yaw, cam_pitch)`.
        `mode` : "RGB" ou "DEPTH" (carte thermique 3D).
        Retourne `(ppm_bytes, center_depth_m)`.
        """
        cp = math.cos(cam_pitch)
        sp = math.sin(cam_pitch)
        cy = math.cos(cam_yaw)
        sy = math.sin(cam_yaw)

        forward = [cp * cy, cp * sy, sp]
        target = [
            cam_pos[0] + forward[0] * 1.0,
            cam_pos[1] + forward[1] * 1.0,
            cam_pos[2] + forward[2] * 1.0,
        ]
        up = [-sp * cy, -sp * sy, cp]

        view_matrix = p.computeViewMatrix(
            cameraEyePosition=cam_pos,
            cameraTargetPosition=target,
            cameraUpVector=up
        )

        _, _, rgba, depth_buf, _ = p.getCameraImage(
            width=self.width,
            height=self.height,
            viewMatrix=view_matrix,
            projectionMatrix=self.proj_matrix,
            renderer=p.ER_TINY_RENDERER
        )

        # Linéarisation du Z-buffer OpenGL en mètres réels
        depth_arr = np.asarray(depth_buf, dtype=np.float32).reshape((self.height, self.width))
        n = self.near
        f = self.far
        linear_depth = (f * n) / (f - (f - n) * depth_arr + 1e-6)
        cy_px, cx_px = self.height // 2, self.width // 2
        center_patch = linear_depth[cy_px - 3:cy_px + 3, cx_px - 3:cx_px + 3]
        self.last_center_depth_m = float(np.mean(center_patch))

        if mode == "DEPTH":
            # Colorisation thermique (Proche = Rouge/Jaune chaud, Moyen = Vert/Cyan, Loin = Bleu nuit)
            norm = np.clip((linear_depth - 0.15) / 2.20, 0.0, 1.0)
            r = np.clip(1.5 - np.abs(4.0 * norm - 1.0), 0.0, 1.0)
            g = np.clip(1.5 - np.abs(4.0 * norm - 2.0), 0.0, 1.0)
            b = np.clip(1.5 - np.abs(4.0 * norm - 3.0), 0.0, 1.0)
            rgb = (np.stack([r, g, b], axis=-1) * 255.0).astype(np.uint8)
        else:
            rgba_arr = np.asarray(rgba, dtype=np.uint8).reshape((self.height, self.width, 4))
            rgb = rgba_arr[:, :, :3].copy()

        # Réticule central (Crosshair)
        rgb[cy_px, cx_px - 8:cx_px + 9] = [0, 255, 180]
        rgb[cy_px - 8:cy_px + 9, cx_px] = [0, 255, 180]

        header = f"P6\n{self.width} {self.height}\n255\n".encode("ascii")
        return header + rgb.tobytes(), self.last_center_depth_m


class LidarRadarWidget(ttk.Frame):
    """Widget Tkinter affichant un écran radar polaire 360° en temps réel + télémétrie."""

    def __init__(self, parent, size=210):
        super().__init__(parent)
        self.size = size
        self.center = size // 2
        self.radius = (size // 2) - 16

        top_row = ttk.Frame(self)
        top_row.pack(fill="x")

        self.canvas = tk.Canvas(
            top_row, width=size, height=size,
            bg="#0b1622", highlightthickness=1, highlightbackground="#1f3a52"
        )
        self.canvas.pack(side="left", padx=4, pady=2)

        info_col = ttk.Frame(top_row)
        info_col.pack(side="left", fill="both", expand=True, padx=6)

        ttk.Label(info_col, text="📡 Télémétrie LiDAR 360°", font=("Arial", 9, "bold")).pack(anchor="w", pady=(2, 4))
        self.lbl_front = ttk.Label(info_col, text="Avant   : -- m", font=("Consolas", 9))
        self.lbl_front.pack(anchor="w")
        self.lbl_left = ttk.Label(info_col, text="Gauche  : -- m", font=("Consolas", 9))
        self.lbl_left.pack(anchor="w")
        self.lbl_right = ttk.Label(info_col, text="Droite  : -- m", font=("Consolas", 9))
        self.lbl_right.pack(anchor="w")
        self.lbl_rear = ttk.Label(info_col, text="Arrière : -- m", font=("Consolas", 9))
        self.lbl_rear.pack(anchor="w")
        self.lbl_min = ttk.Label(info_col, text="Min     : -- m", font=("Consolas", 9, "bold"), foreground="#0984e3")
        self.lbl_min.pack(anchor="w", pady=(4, 0))

        self._draw_static_grid()

    def _draw_static_grid(self):
        c = self.center
        r = self.radius
        self.canvas.delete("grid")
        for frac in (0.33, 0.66, 1.0):
            rf = r * frac
            self.canvas.create_oval(
                c - rf, c - rf, c + rf, c + rf,
                outline="#1c3b57", width=1, tags="grid"
            )
        self.canvas.create_line(c, c - r, c, c + r, fill="#1c3b57", tags="grid")
        self.canvas.create_line(c - r, c, c + r, c, fill="#1c3b57", tags="grid")
        self.canvas.create_text(c, 8, text="AVANT (0°)", fill="#58a6ff", font=("Arial", 7, "bold"), tags="grid")
        # Triangle représentant le robot au centre
        self.canvas.create_polygon(
            c, c - 7, c - 5, c + 6, c + 5, c + 6,
            fill="#00d2d3", outline="white", tags="grid"
        )

    def update_radar(self, scanner: LidarScanner, telemetry: dict):
        c = self.center
        r = self.radius
        max_r = scanner.max_range

        self.canvas.delete("scan")

        # Ligne de balayage rotative
        sweep_x = c - r * math.sin(scanner.sweep_phase)
        sweep_y = c - r * math.cos(scanner.sweep_phase)
        self.canvas.create_line(c, c, sweep_x, sweep_y, fill="#1dd1a1", width=1, tags="scan")

        # Points d'impact LiDAR (Dans le repère écran : Haut = Avant (+X), Gauche = +Y)
        poly_pts = []
        for ang, dist, hit in zip(scanner.angles_rad, scanner.distances, scanner.hit_flags):
            norm_d = min(1.0, float(dist) / max_r)
            px = c - (norm_d * r) * math.sin(float(ang))
            py = c - (norm_d * r) * math.cos(float(ang))
            poly_pts.extend([px, py])
            if hit:
                col = "#ff4757" if dist < 0.55 else ("#ffa502" if dist < 1.0 else "#2ed573")
                self.canvas.create_oval(px - 2.2, py - 2.2, px + 2.2, py + 2.2, fill=col, outline="", tags="scan")

        if len(poly_pts) >= 6:
            self.canvas.create_polygon(
                *poly_pts, outline="#00a8ff", fill="", width=1, tags="scan"
            )

        self.lbl_front.config(text=f"Avant   : {telemetry['front']:.2f} m")
        self.lbl_left.config(text=f"Gauche  : {telemetry['left']:.2f} m")
        self.lbl_right.config(text=f"Droite  : {telemetry['right']:.2f} m")
        self.lbl_rear.config(text=f"Arrière : {telemetry['rear']:.2f} m")
        min_d = telemetry["min_dist"]
        min_col = "#d63031" if min_d < 0.55 else ("#e17055" if min_d < 0.95 else "#0984e3")
        self.lbl_min.config(
            text=f"Min: {min_d:.2f}m ({telemetry['min_angle_deg']:+.0f}°)",
            foreground=min_col
        )


class CameraFeedWidget(ttk.Frame):
    """Widget Tkinter affichant le flux vidéo en direct (RGB ou Depth 3D) de la caméra embarquée."""

    def __init__(self, parent, width=160, height=120):
        super().__init__(parent)
        self.width = width
        self.height = height
        self._photo = None

        top_bar = ttk.Frame(self)
        top_bar.pack(fill="x", pady=2)

        self.mode_var = tk.StringVar(value="RGB")
        rb_rgb = ttk.Radiobutton(top_bar, text="📷 Flux RGB Couleur", variable=self.mode_var, value="RGB")
        rb_rgb.pack(side="left", padx=4)
        rb_depth = ttk.Radiobutton(top_bar, text="🌈 Profondeur 3D (Depth)", variable=self.mode_var, value="DEPTH")
        rb_depth.pack(side="left", padx=4)

        self.lbl_depth = ttk.Label(
            top_bar, text="Cible : -- m", font=("Consolas", 9, "bold"), foreground="#008080"
        )
        self.lbl_depth.pack(side="right", padx=4)

        self.canvas = tk.Canvas(
            self, width=width * 2, height=height * 2,
            bg="#101418", highlightthickness=1, highlightbackground="#2f3640"
        )
        self.canvas.pack(pady=3)

    def update_image(self, ppm_bytes: bytes, center_depth_m: float):
        try:
            raw_img = tk.PhotoImage(data=ppm_bytes, format="PPM")
            self._photo = raw_img.zoom(2, 2)
            self.canvas.delete("all")
            self.canvas.create_image(0, 0, anchor="nw", image=self._photo)
            mode_txt = "RGB-D (Thermique)" if self.mode_var.get() == "DEPTH" else "Caméra RGB"
            self.canvas.create_text(
                8, 10, anchor="nw",
                text=f"{mode_txt} | Profondeur centre : {center_depth_m:.2f} m",
                fill="#00ffcc", font=("Consolas", 8, "bold")
            )
            self.lbl_depth.config(text=f"Cible : {center_depth_m:.2f} m")
        except Exception:
            pass
