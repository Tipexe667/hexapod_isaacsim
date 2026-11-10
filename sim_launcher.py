#!/usr/bin/env python3
"""
Menu Principal de Sélection de Simulation Robotique (PyBullet + Tkinter).
Permet de choisir et de basculer à tout moment entre :
1. 🤖 Hiwonder MentorPi (Châssis T1 Chenilles ou M1 Roues Mecanum + LiDAR 360° + Caméra 3D Pan-Tilt)
2. 🕷️ Robot Hexapode 18-DOF (Dynamixel AX-18 + Cinématique Inverse + Saut + LiDAR 360° + Caméra 3D)
"""
import os
import sys
import tkinter as tk
from tkinter import ttk


class SimLauncherMenu:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("🚀 Centre de Simulation Robotique 3D - Choix du Robot (PyBullet)")
        self.root.geometry("820x540")
        self.root.configure(bg="#14191f")
        self.root.resizable(False, False)

        self.selected_action = None
        self._build_ui()

    def _build_ui(self):
        # En-tête
        header = tk.Frame(self.root, bg="#0d1117", pady=16)
        header.pack(fill="x")

        tk.Label(
            header,
            text="🤖 CENTRE DE SIMULATION ROBOTIQUE 3D (PYBULLET)",
            bg="#0d1117", fg="#58a6ff", font=("Arial", 16, "bold")
        ).pack()

        tk.Label(
            header,
            text="Sélectionnez le robot à lancer avec son interface de commande dédiée, son LiDAR 360° et sa Caméra 3D embarquée",
            bg="#0d1117", fg="#8b949e", font=("Arial", 10)
        ).pack(pady=(4, 0))

        # Conteneur des 2 cartes robots côte à côte
        cards_frame = tk.Frame(self.root, bg="#14191f", padx=20, pady=18)
        cards_frame.pack(fill="both", expand=True)
        cards_frame.columnconfigure(0, weight=1)
        cards_frame.columnconfigure(1, weight=1)

        # =====================================================================
        # CARTE 1 : HIWONDER MENTORPI (T1 CHENILLES / M1 MECANUM)
        # =====================================================================
        card_mp = tk.Frame(
            cards_frame, bg="#1c232b", bd=1, relief="solid",
            highlightbackground="#00d2d3", highlightthickness=2, padx=16, pady=14
        )
        card_mp.grid(row=0, column=0, sticky="nsew", padx=(0, 10))

        tk.Label(
            card_mp, text="🤖 HIWONDER MENTORPI",
            bg="#1c232b", fg="#00d2d3", font=("Arial", 14, "bold")
        ).pack(anchor="w")

        tk.Label(
            card_mp, text="Robot Mobile ROS2 • Raspberry Pi 5 • SolidCraft CAO",
            bg="#1c232b", fg="#a4b0be", font=("Arial", 9, "italic")
        ).pack(anchor="w", pady=(0, 10))

        mp_features = [
            "✔ Modèle CAO 50 corps 100% manifold (Standoffs, PCB, RPi5)",
            "✔ Bascule temps réel : T1 (Chenilles) ⇄ M1 (4 Roues Mecanum)",
            "✔ LiDAR 360° ToF (STL27L) rotatif + Radar Polaire 2D & Rayons 3D",
            "✔ Tourelle Caméra 3D Depth 2-DOF (Pan/Tilt + Balayage Auto)",
            "✔ Flux Vidéo Direct : Couleur RGB & Profondeur Thermique 3D",
            "✔ Mode d'Exploration Autonome avec Évitement d'Obstacles LiDAR",
        ]
        for feat in mp_features:
            tk.Label(
                card_mp, text=feat, bg="#1c232b", fg="#dfe4ea",
                font=("Arial", 9), justify="left"
            ).pack(anchor="w", pady=2)

        btn_mp_t1 = tk.Button(
            card_mp,
            text="🚜 Lancer MentorPi — Mode T1 (Chenilles Tank)",
            bg="#00a8ff", fg="white", activebackground="#0097e6", activeforeground="white",
            font=("Arial", 10, "bold"), pady=8, cursor="hand2",
            command=lambda: self._choose("MENTORPI_T1")
        )
        btn_mp_t1.pack(fill="x", pady=(16, 6))

        btn_mp_m1 = tk.Button(
            card_mp,
            text="🛞 Lancer MentorPi — Mode M1 (Roues Mecanum)",
            bg="#e67e22", fg="white", activebackground="#d35400", activeforeground="white",
            font=("Arial", 10, "bold"), pady=8, cursor="hand2",
            command=lambda: self._choose("MENTORPI_M1")
        )
        btn_mp_m1.pack(fill="x", pady=4)

        # =====================================================================
        # CARTE 2 : ROBOT HEXAPODE 18-DOF (DYNAMIXEL AX-18)
        # =====================================================================
        card_hex = tk.Frame(
            cards_frame, bg="#1c232b", bd=1, relief="solid",
            highlightbackground="#ff9f43", highlightthickness=2, padx=16, pady=14
        )
        card_hex.grid(row=0, column=1, sticky="nsew", padx=(10, 0))

        tk.Label(
            card_hex, text="🕷️ ROBOT HEXAPODE 18-DOF",
            bg="#1c232b", fg="#ff9f43", font=("Arial", 14, "bold")
        ).pack(anchor="w")

        tk.Label(
            card_hex, text="Châssis ABS • 18 Servos Dynamixel AX-18 • Cinématique IK",
            bg="#1c232b", fg="#a4b0be", font=("Arial", 9, "italic")
        ).pack(anchor="w", pady=(0, 10))

        hex_features = [
            "✔ Cinématique Inverse (IK) 3-DOF exacte sur les 6 pattes",
            "✔ Marche tripode omnidirectionnelle fluide (Vx, Vy, Lacet Wz)",
            "✔ Contrôle d'assiette du corps 6-DOF (Hauteur, Roulis, Tangage)",
            "✔ Saut Dynamique Explosif (Jump) & Postures (Stand, Crouch, Rest)",
            "✔ LiDAR 360° embarqué + Marche Autonome d'Évitement d'Obstacles",
            "✔ Caméra Frontale 3D embarquée (Flux RGB & Profondeur Thermique)",
        ]
        for feat in hex_features:
            tk.Label(
                card_hex, text=feat, bg="#1c232b", fg="#dfe4ea",
                font=("Arial", 9), justify="left"
            ).pack(anchor="w", pady=2)

        btn_hex = tk.Button(
            card_hex,
            text="🕷️ Lancer le Robot Hexapode 18-DOF",
            bg="#8e44ad", fg="white", activebackground="#9b59b6", activeforeground="white",
            font=("Arial", 11, "bold"), pady=12, cursor="hand2",
            command=lambda: self._choose("HEXAPOD")
        )
        btn_hex.pack(fill="x", pady=(22, 6))

        btn_quit = tk.Button(
            card_hex,
            text="❌ Quitter le Lanceur",
            bg="#2f3640", fg="#dcdde1", font=("Arial", 9), pady=6,
            command=lambda: self._choose(None)
        )
        btn_quit.pack(fill="x", pady=4)

        # Pied de page
        footer = tk.Label(
            self.root,
            text="💡 Astuce : Depuis chaque simulation, le bouton « 🔀 Menu Robots » en haut à droite permet de changer de robot à la volée.",
            bg="#14191f", fg="#7f8fa6", font=("Arial", 9)
        )
        footer.pack(pady=(0, 10))

    def _choose(self, action):
        self.selected_action = action
        self.root.destroy()

    def show(self):
        self.root.mainloop()
        return self.selected_action


def main():
    # Support d'arguments en ligne de commande optionnels
    next_action = "MENU"
    if len(sys.argv) > 1:
        arg = sys.argv[1].lower()
        if arg in ("mentorpi", "mentorpi_t1", "t1"):
            next_action = "MENTORPI_T1"
        elif arg in ("mentorpi_m1", "m1"):
            next_action = "MENTORPI_M1"
        elif arg in ("hexapod", "hexapode"):
            next_action = "HEXAPOD"

    while next_action is not None:
        if next_action == "MENU":
            menu = SimLauncherMenu()
            choice = menu.show()
            if not choice:
                break
            next_action = choice

        elif next_action in ("MENTORPI", "MENTORPI_T1", "MENTORPI_M1"):
            import mentorpi_sim
            chassis = "M1" if next_action == "MENTORPI_M1" else "T1"
            res = mentorpi_sim.run_mentorpi_simulation(initial_chassis=chassis)
            if res == "MENU":
                next_action = "MENU"
            elif res == "HEXAPOD":
                next_action = "HEXAPOD"
            else:
                break

        elif next_action == "HEXAPOD":
            import hexapod_sim
            res = hexapod_sim.main()
            if res == "MENU":
                next_action = "MENU"
            elif res == "MENTORPI":
                next_action = "MENTORPI_T1"
            else:
                break
        else:
            break


if __name__ == "__main__":
    main()

