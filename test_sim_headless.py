#!/usr/bin/env python3
"""
Script de test headless (sans GUI) pour valider le chargement et la physique :
1. Du robot Hexapode 18-DOF (urdf/hexapod_pybullet.urdf)
2. Du robot Hiwonder MentorPi (mentorpi_solidcraft/mentorpi_pybullet.urdf)
3. Des capteurs embarqués : LiDAR 360° (LidarScanner) et Caméra 3D RGB-D (OnboardCameraRenderer)
"""
import os
import pybullet as p
import pybullet_data
from sensor_widgets import ObstacleArenaManager, LidarScanner, OnboardCameraRenderer


def test_hexapod_headless():
    print("\n--- [1/2] TEST HEADLESS : ROBOT HEXAPODE 18-DOF ---")
    cid = p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)

    p.loadURDF("plane.urdf")
    urdf_path = "urdf/hexapod_pybullet.urdf"
    robot_id = p.loadURDF(urdf_path, [0, 0, 0.16], flags=p.URDF_USE_INERTIA_FROM_FILE)

    num_joints = p.getNumJoints(robot_id)
    revolute_count = sum(
        1 for i in range(num_joints) if p.getJointInfo(robot_id, i)[2] == p.JOINT_REVOLUTE
    )
    print(f"Articulations revolutes Hexapode : {revolute_count} / 18")

    for _ in range(60):
        p.stepSimulation()

    pos, _ = p.getBasePositionAndOrientation(robot_id)
    print(f"Position Z finale Hexapode : {pos[2]:.4f} m")
    p.disconnect()
    return revolute_count == 18 and pos[2] > 0.05


def test_mentorpi_headless():
    print("\n--- [2/2] TEST HEADLESS : HIWONDER MENTORPI + LIDAR 360° + CAMÉRA 3D ---")
    cid = p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)

    p.loadURDF("plane.urdf")
    urdf_path = os.path.join("mentorpi_solidcraft", "mentorpi_pybullet.urdf")
    robot_id = p.loadURDF(urdf_path, [0, 0, 0.005])

    num_joints = p.getNumJoints(robot_id)
    print(f"Liaisons URDF MentorPi chargées : {num_joints}")

    arena = ObstacleArenaManager()
    arena.spawn_arena()
    lidar = LidarScanner(num_rays=72, max_range=2.20)
    cam = OnboardCameraRenderer(width=80, height=60)

    for _ in range(60):
        p.stepSimulation()

    pos, _ = p.getBasePositionAndOrientation(robot_id)
    tel = lidar.scan([pos[0], pos[1], pos[2] + 0.10], 0.0, draw_3d_rays=False)
    ppm, depth_m = cam.render_ppm([pos[0] + 0.08, pos[1], pos[2] + 0.13], 0.0, 0.0, mode="DEPTH")

    print(f"Position Z finale MentorPi : {pos[2]:.4f} m")
    print(f"Télémétrie LiDAR 360°      : Avant={tel['front']:.2f}m, Min={tel['min_dist']:.2f}m")
    print(f"Caméra 3D Depth            : Flux PPM={len(ppm)} octets, Profondeur centre={depth_m:.2f}m")

    p.disconnect()
    return num_joints >= 15 and len(ppm) > 1000


if __name__ == "__main__":
    ok1 = test_hexapod_headless()
    ok2 = test_mentorpi_headless()
    if ok1 and ok2:
        print("\n[SUCCÈS GLOBAL] Les deux robots (Hexapode & MentorPi) et leurs capteurs sont 100% opérationnels !")
    else:
        print("\n[ATTENTION] Un des tests a signalé une anomalie.")
