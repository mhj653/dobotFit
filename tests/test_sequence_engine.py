from __future__ import annotations

import sys
import unittest

from PySide6.QtCore import QCoreApplication

from core.device_manager import DeviceManager
from core.project_manager import DEFAULT_POSITIONS
from core.sequence_engine import SequenceEngine
from core.models import SequenceStep
from tests.test_real_mg400_tcp import FakeDobotServer


class SequenceEngineTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.app = QCoreApplication.instance() or QCoreApplication(sys.argv)

    def test_robot_move_and_gripper_close(self) -> None:
        devices = DeviceManager()
        engine = SequenceEngine(devices)
        move = SequenceStep(1, "Robot", "MoveJ", "HOME")
        result = engine.execute_step(move, list(DEFAULT_POSITIONS))
        self.assertTrue(result.success)

        close = SequenceStep(2, "Gripper", "Close")
        result = engine.execute_step(close, list(DEFAULT_POSITIONS))
        self.assertTrue(result.success)
        self.assertTrue(devices.io.read_di(1))

    def test_real_mode_does_not_fallback_to_simulation(self) -> None:
        devices = DeviceManager()
        devices.set_mode("REAL")
        engine = SequenceEngine(devices)
        result = engine.execute_step(SequenceStep(1, "Robot", "MoveJ", "HOME"), list(DEFAULT_POSITIONS))
        self.assertFalse(result.success)
        self.assertEqual(result.error_code, "NOT_CONNECTED")

    def test_jog_and_setdo_steps(self) -> None:
        devices = DeviceManager()
        engine = SequenceEngine(devices)
        jog = engine.execute_step(SequenceStep(1, "Robot", "Jog", "X+10"), list(DEFAULT_POSITIONS))
        self.assertTrue(jog.success)
        self.assertGreater(devices.robot_pose().x, 300.0)
        set_do = engine.execute_step(SequenceStep(2, "I/O", "SetDO", "DO03=ON"), list(DEFAULT_POSITIONS))
        self.assertTrue(set_do.success)
        self.assertTrue(devices.io.do[3])

    def test_disabled_condition_and_step_motion_settings(self) -> None:
        devices = DeviceManager()
        engine = SequenceEngine(devices)
        disabled = SequenceStep(1, "Robot", "MoveJ", "HOME", enabled=False)
        result = engine.execute_step(disabled, list(DEFAULT_POSITIONS))
        self.assertTrue(result.success)
        self.assertEqual(disabled.status, "Skipped")

        conditional = SequenceStep(2, "Robot", "MoveJ", "HOME", condition="part_ok", speed=33, acceleration=44)
        result = engine.execute_step(conditional, list(DEFAULT_POSITIONS))
        self.assertTrue(result.success)
        self.assertEqual(conditional.status, "Done")

    def test_sequence_validation_reports_bad_targets(self) -> None:
        devices = DeviceManager()
        engine = SequenceEngine(devices)
        issues = engine.validate_sequence([SequenceStep(1, "Robot", "MoveJ", "MISSING")], list(DEFAULT_POSITIONS))
        self.assertTrue(issues)

    def test_real_mode_io_and_gripper_use_mg400_tcp(self) -> None:
        dashboard = FakeDobotServer(
            {
                "DO(3,1)": "0,{},DO();",
                "DO(4,0)": "0,{},DO();",
                "DO(5,1)": "0,{},DO();",
                "DI(2)": "0,{1},DI();",
            }
        )
        move = FakeDobotServer({})
        try:
            devices = DeviceManager()
            devices.set_mode("REAL")
            devices.configure_real_robot("127.0.0.1", dashboard.port, move.port, timeout_s=0.5)
            self.assertTrue(devices.connect_all().success)
            self.assertTrue(devices.configure_gripper_io(4, 5, 2).success)
            self.assertTrue(devices.set_do(3, True).success)
            self.assertTrue(devices.gripper_close().success)
            self.assertTrue(devices.last_grip_ok)
            self.assertTrue(devices.io.do[3])
            self.assertTrue(devices.io.do[5])
            self.assertTrue(devices.io.di[2])
            self.assertIn("DO(3,1)", dashboard.commands)
            self.assertIn("DO(4,0)", dashboard.commands)
            self.assertIn("DO(5,1)", dashboard.commands)
            self.assertIn("DI(2)", dashboard.commands)
        finally:
            devices.robot.disconnect()
            dashboard.close()
            move.close()


if __name__ == "__main__":
    unittest.main()
