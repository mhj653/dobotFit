from __future__ import annotations

import unittest

from core.models import Pose
from simulation.mg400_kinematics import MG400Kinematics
from simulation.robot_model import KinematicMG400Model
from simulation.pybullet_engine import PyBulletEngine


class SimulationModelTests(unittest.TestCase):
    def test_move_builds_path_and_updates_joints(self) -> None:
        model = KinematicMG400Model()
        result = model.move(Pose(320, 60, 210, 90), "MoveJ", 20, 50)
        self.assertTrue(result.success)
        self.assertGreater(len(model.last_path), 2)
        self.assertAlmostEqual(model.pose.x, 320)
        self.assertNotEqual(model.joints.j1, 0.0)

    def test_workspace_limit_rejects_unreachable_target(self) -> None:
        model = KinematicMG400Model()
        result = model.move(Pose(700, 0, 210, 0), "MoveL", 20, 50)
        self.assertFalse(result.success)
        self.assertEqual(result.error_code, "LIMIT_X")

    def test_mg400_inverse_forward_roundtrip(self) -> None:
        kinematics = MG400Kinematics()
        target = Pose(300, 80, 210, 45)
        joints = kinematics.inverse(target)
        self.assertIsNotNone(joints)
        assert joints is not None
        pose = kinematics.forward(joints)
        self.assertAlmostEqual(pose.x, target.x, places=6)
        self.assertAlmostEqual(pose.y, target.y, places=6)
        self.assertAlmostEqual(pose.z, target.z, places=6)
        self.assertAlmostEqual(pose.r, target.r, places=6)

    def test_invalid_mg400_joint_solution_is_rejected(self) -> None:
        model = KinematicMG400Model()
        result = model.move(Pose(-430, 20, 210, 0), "MoveJ", 20, 50)
        self.assertFalse(result.success)
        self.assertEqual(result.error_code, "KINEMATICS")

    def test_pybullet_backend_is_optional(self) -> None:
        engine = PyBulletEngine()
        if engine.available:
            self.assertTrue(engine.connect_direct())
            self.assertTrue(engine.initialize_scene())
            self.assertGreater(engine.object_count(), 0)
            self.assertTrue(engine.uses_urdf_model)
            frame = engine.render_camera(96, 72)
            self.assertIsNotNone(frame)
            assert frame is not None
            self.assertEqual(frame[0:2], (96, 72))
            self.assertEqual(len(frame[2]), 96 * 72 * 4)
            engine.set_camera_view("Top")
            top_frame = engine.render_camera(96, 72)
            self.assertIsNotNone(top_frame)
            assert top_frame is not None
            self.assertEqual(len(top_frame[2]), 96 * 72 * 4)
            engine.disconnect()
        else:
            self.assertFalse(engine.connect_direct())

    def test_model_reports_backend(self) -> None:
        model = KinematicMG400Model()
        snapshot = model.snapshot()
        self.assertIn(snapshot.backend, {"Kinematic", "PyBullet"})
        if snapshot.backend == "PyBullet":
            self.assertGreater(snapshot.physics_objects, 0)
            self.assertGreater(snapshot.render_width, 0)
            self.assertGreater(snapshot.render_height, 0)
            self.assertIsNotNone(snapshot.render_rgba)
