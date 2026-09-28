from __future__ import annotations

import socket
import threading
import unittest

from core.models import Pose
from drivers.robot.mg400_tcp import MG400ConnectionConfig, RealMG400


class FakeDobotServer:
    def __init__(self, replies: dict[str, str]) -> None:
        self.replies = replies
        self.commands: list[str] = []
        self._stop = threading.Event()
        self._ready = threading.Event()
        self._server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server.bind(("127.0.0.1", 0))
        self.port = self._server.getsockname()[1]
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()
        self._ready.wait(1.0)

    def close(self) -> None:
        self._stop.set()
        try:
            with socket.create_connection(("127.0.0.1", self.port), timeout=0.2):
                pass
        except OSError:
            pass
        self.thread.join(1.0)
        self._server.close()

    def _run(self) -> None:
        self._server.listen(1)
        self._ready.set()
        while not self._stop.is_set():
            try:
                self._server.settimeout(0.2)
                conn, _addr = self._server.accept()
            except OSError:
                continue
            with conn:
                conn.settimeout(0.5)
                while not self._stop.is_set():
                    try:
                        data = conn.recv(4096)
                    except OSError:
                        break
                    if not data:
                        break
                    command = data.decode("utf-8")
                    self.commands.append(command)
                    reply = self.replies.get(command, f"0,{{}},${command};")
                    conn.send(reply.encode("utf-8"))


class RealMG400TcpTests(unittest.TestCase):
    def test_connect_enable_move_and_refresh_pose(self) -> None:
        dashboard = FakeDobotServer(
            {
                "EnableRobot()": "0,{},EnableRobot();",
                "SpeedJ(20)": "0,{},SpeedJ(20);",
                "AccJ(50)": "0,{},AccJ(50);",
                "GetPose()": "0,{300.000,10.000,220.000,90.000},GetPose();",
                "GetAngle()": "0,{1.000,2.000,3.000,4.000},GetAngle();",
            }
        )
        move = FakeDobotServer(
            {
                "MovJ(300.000,10.000,220.000,90.000)": "0,{},MovJ();",
                "Sync()": "0,{},Sync();",
            }
        )
        try:
            robot = RealMG400(MG400ConnectionConfig("127.0.0.1", dashboard.port, move.port, timeout_s=0.5))
            self.assertTrue(robot.connect().success)
            self.assertTrue(robot.enable().success)
            self.assertTrue(robot.move_j(Pose(300, 10, 220, 90), 20, 50).success)
            self.assertEqual(robot.get_pose().x, 300.0)
            self.assertIn("EnableRobot()", dashboard.commands)
            self.assertIn("MovJ(300.000,10.000,220.000,90.000)", move.commands)
            self.assertIn("Sync()", move.commands)
        finally:
            robot.disconnect()
            dashboard.close()
            move.close()

    def test_controller_error_code_is_reported(self) -> None:
        dashboard = FakeDobotServer({"EnableRobot()": "-1,{},EnableRobot();"})
        move = FakeDobotServer({})
        try:
            robot = RealMG400(MG400ConnectionConfig("127.0.0.1", dashboard.port, move.port, timeout_s=0.5))
            self.assertTrue(robot.connect().success)
            result = robot.enable()
            self.assertFalse(result.success)
            self.assertEqual(result.error_code, "DOBOT_-1")
            self.assertIn("execution failed", result.message)
        finally:
            robot.disconnect()
            dashboard.close()
            move.close()

    def test_jog_uses_current_pose_and_step_increment(self) -> None:
        dashboard = FakeDobotServer(
            {
                "GetPose()": "0,{300.000,10.000,220.000,90.000},GetPose();",
                "GetAngle()": "0,{1.000,2.000,3.000,4.000},GetAngle();",
                "SpeedL(10)": "0,{},SpeedL(10);",
                "AccL(30)": "0,{},AccL(30);",
            }
        )
        move = FakeDobotServer(
            {
                "MovL(305.000,10.000,220.000,90.000)": "0,{},MovL();",
                "Sync()": "0,{},Sync();",
            }
        )
        try:
            robot = RealMG400(MG400ConnectionConfig("127.0.0.1", dashboard.port, move.port, timeout_s=0.5))
            self.assertTrue(robot.connect().success)
            self.assertTrue(robot.jog("X", 1, 5).success)
            self.assertIn("GetPose()", dashboard.commands)
            self.assertIn("MovL(305.000,10.000,220.000,90.000)", move.commands)
        finally:
            robot.disconnect()
            dashboard.close()
            move.close()

    def test_refresh_status_decodes_robot_mode_and_error_ids(self) -> None:
        dashboard = FakeDobotServer(
            {
                "RobotMode()": "0,{9},RobotMode();",
                "GetErrorID()": "0,{[[-2],[],[],[],[],[]]},GetErrorID();",
            }
        )
        move = FakeDobotServer({})
        try:
            robot = RealMG400(MG400ConnectionConfig("127.0.0.1", dashboard.port, move.port, timeout_s=0.5))
            self.assertTrue(robot.connect().success)
            result = robot.refresh_status()
            self.assertTrue(result.success)
            self.assertEqual(robot.robot_mode_text, "ERROR")
            self.assertEqual(robot.error_ids, [-2])
            self.assertIn("Alarm IDs: -2", result.message)
        finally:
            robot.disconnect()
            dashboard.close()
            move.close()

    def test_digital_output_and_input(self) -> None:
        dashboard = FakeDobotServer(
            {
                "DO(1,1)": "0,{},DO();",
                "DI(1)": "0,{1},DI();",
            }
        )
        move = FakeDobotServer({})
        try:
            robot = RealMG400(MG400ConnectionConfig("127.0.0.1", dashboard.port, move.port, timeout_s=0.5))
            self.assertTrue(robot.connect().success)
            self.assertTrue(robot.digital_output(1, True).success)
            result, state = robot.read_digital_input(1)
            self.assertTrue(result.success)
            self.assertTrue(state)
            self.assertIn("DO(1,1)", dashboard.commands)
            self.assertIn("DI(1)", dashboard.commands)
        finally:
            robot.disconnect()
            dashboard.close()
            move.close()


if __name__ == "__main__":
    unittest.main()
