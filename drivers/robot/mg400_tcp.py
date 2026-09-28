from __future__ import annotations

import re
import socket
import threading
import time
from dataclasses import dataclass
from datetime import datetime
from typing import Callable

from core.models import JointState, Pose, Result
from drivers.robot.robot_interface import IRobot


@dataclass
class MG400ConnectionConfig:
    ip: str = "192.168.1.6"
    dashboard_port: int = 29999
    move_port: int = 30003
    feedback_port: int = 30004
    timeout_s: float = 1.5


PacketCallback = Callable[[dict], None]


DOBOT_ERROR_CODES = {
    0: "No error",
    -1: "Command received but execution failed",
    -2: "Robot is in alarm status. Clear the alarm before retrying",
    -3: "Emergency stop is active. Release E-stop, clear alarm, then retry",
    -4: "Robot is powered off",
    -5: "Robot is running or paused in script/project mode",
    -6: "Jog axis and motion type do not match",
    -7: "Robot script is paused. Stop the script first",
    -8: "Robot certification expired or unavailable",
    -10000: "Command does not exist",
    -20000: "Incorrect number of command parameters",
    -30001: "First parameter type is incorrect",
    -30002: "Second parameter type is incorrect",
    -30003: "Third parameter type is incorrect",
    -30004: "Fourth parameter type is incorrect",
    -40001: "First parameter is outside the valid range",
    -40002: "Second parameter is outside the valid range",
    -40003: "Third parameter is outside the valid range",
    -40004: "Fourth parameter is outside the valid range",
    -50001: "First optional parameter type is incorrect",
    -50002: "Second optional parameter type is incorrect",
    -60001: "First optional parameter is outside the valid range",
    -60002: "Second optional parameter is outside the valid range",
}


ROBOT_MODES = {
    1: "INIT",
    2: "BRAKE_OPEN",
    3: "POWEROFF",
    4: "DISABLED",
    5: "ENABLE_IDLE",
    6: "BACKDRIVE",
    7: "RUNNING",
    8: "SINGLE_MOVE",
    9: "ERROR",
    10: "PAUSE",
    11: "COLLISION",
}


class DobotTcpChannel:
    def __init__(self, ip: str, port: int, timeout_s: float, packet_callback: PacketCallback | None = None) -> None:
        self.ip = ip
        self.port = port
        self.timeout_s = timeout_s
        self.packet_callback = packet_callback
        self.sock: socket.socket | None = None
        self.lock = threading.Lock()

    @property
    def connected(self) -> bool:
        return self.sock is not None

    def connect(self) -> None:
        self.close()
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(self.timeout_s)
        sock.connect((self.ip, self.port))
        self.sock = sock

    def close(self) -> None:
        if self.sock is not None:
            try:
                self.sock.close()
            finally:
                self.sock = None

    def request(self, command: str) -> str:
        if self.sock is None:
            self._emit_packet("ERR", "Socket is not connected", command, 0.0, True)
            raise ConnectionError(f"Socket is not connected: {self.ip}:{self.port}")
        start = time.perf_counter()
        self._emit_packet("TX", command, command, 0.0, False)
        try:
            with self.lock:
                self.sock.send(command.encode("utf-8"))
                data = self.sock.recv(4096)
            elapsed_ms = (time.perf_counter() - start) * 1000.0
            if not data:
                raise TimeoutError(f"No response for {command}")
            reply = data.decode("utf-8", errors="replace").strip()
            self._emit_packet("RX", reply, command, elapsed_ms, False)
            return reply
        except (OSError, TimeoutError, ConnectionError) as exc:
            elapsed_ms = (time.perf_counter() - start) * 1000.0
            self._emit_packet("ERR", str(exc), command, elapsed_ms, True)
            raise

    def _emit_packet(self, direction: str, message: str, command: str, elapsed_ms: float, error: bool) -> None:
        if self.packet_callback is None:
            return
        self.packet_callback(
            {
                "time": datetime.now().strftime("%H:%M:%S.%f")[:-3],
                "direction": direction,
                "ip": self.ip,
                "port": self.port,
                "command": command,
                "message": message,
                "elapsed_ms": elapsed_ms,
                "error": error,
            }
        )


class RealMG400(IRobot):
    """DOBOT MG400/M1Pro TCP/IP driver.

    Command names and default ports follow DOBOT's MG400/M1Pro 4-axis TCP/IP
    Python SDK. All methods return Result instead of raising into the UI layer.
    """

    def __init__(self, config: MG400ConnectionConfig | None = None, packet_callback: PacketCallback | None = None) -> None:
        self.config = config or MG400ConnectionConfig()
        self.dashboard = DobotTcpChannel(self.config.ip, self.config.dashboard_port, self.config.timeout_s, packet_callback)
        self.move = DobotTcpChannel(self.config.ip, self.config.move_port, self.config.timeout_s, packet_callback)
        self.feedback = DobotTcpChannel(self.config.ip, self.config.feedback_port, self.config.timeout_s, packet_callback)
        self.pose = Pose()
        self.joints = JointState()
        self.enabled = False
        self.last_reply = ""
        self.robot_mode_code: int | None = None
        self.robot_mode_text = "Unknown"
        self.error_ids: list[int] = []

    def connect(self) -> Result:
        try:
            self.dashboard.connect()
            self.move.connect()
        except OSError as exc:
            self.disconnect()
            return Result.fail(f"MG400 connection failed: {exc}", "CONNECTION_FAILED")
        return Result.ok(f"MG400 connected at {self.config.ip}")

    def disconnect(self) -> Result:
        self.dashboard.close()
        self.move.close()
        self.feedback.close()
        self.enabled = False
        return Result.ok("MG400 disconnected")

    def enable(self) -> Result:
        result = self._dashboard("EnableRobot()")
        if result.success:
            self.enabled = True
        return result

    def disable(self) -> Result:
        result = self._dashboard("DisableRobot()")
        if result.success:
            self.enabled = False
        return result

    def clear_error(self) -> Result:
        result = self._dashboard("ClearError()")
        if result.success:
            self.error_ids = []
        return result

    def get_pose(self) -> Pose:
        if not self.dashboard.connected:
            return self.pose
        try:
            reply = self.dashboard.request("GetPose()")
            values = _extract_reply_numbers(reply)
            if len(values) >= 4 and _reply_error_code(reply) == 0:
                self.pose = Pose(values[0], values[1], values[2], values[3])
        except Exception:
            return self.pose
        return self.pose

    def refresh_pose(self) -> Result:
        if not self.dashboard.connected:
            return Result.fail("MG400 is not connected", "NOT_CONNECTED")
        try:
            reply = self.dashboard.request("GetPose()")
        except (OSError, TimeoutError, ConnectionError) as exc:
            return Result.fail(f"GetPose failed: {exc}", "COMMUNICATION_ERROR")
        values = _extract_reply_numbers(reply)
        if len(values) < 4:
            return Result.fail(f"Unexpected GetPose reply: {reply}", "BAD_REPLY")
        self.pose = Pose(values[0], values[1], values[2], values[3])
        self.last_reply = reply
        return _result_from_reply(reply, "GetPose")

    def refresh_joints(self) -> Result:
        if not self.dashboard.connected:
            return Result.fail("MG400 is not connected", "NOT_CONNECTED")
        try:
            reply = self.dashboard.request("GetAngle()")
        except (OSError, TimeoutError, ConnectionError) as exc:
            return Result.fail(f"GetAngle failed: {exc}", "COMMUNICATION_ERROR")
        values = _extract_reply_numbers(reply)
        if len(values) >= 4:
            self.joints = JointState(values[0], values[1], values[2], values[3])
        self.last_reply = reply
        return _result_from_reply(reply, "GetAngle")

    def get_joints(self) -> JointState:
        if not self.dashboard.connected:
            return self.joints
        self.refresh_joints()
        return self.joints

    def move_j(self, pose: Pose, speed: float, acceleration: float) -> Result:
        return self._move("MovJ", pose, speed, acceleration)

    def move_l(self, pose: Pose, speed: float, acceleration: float) -> Result:
        return self._move("MovL", pose, speed, acceleration)

    def jog(self, axis: str, direction: int, step: float) -> Result:
        if not self.move.connected:
            return Result.fail("MG400 move port is not connected", "NOT_CONNECTED")
        axis = axis.upper()
        if axis not in {"X", "Y", "Z", "R"}:
            return Result.fail(f"Unsupported jog axis {axis}", "BAD_AXIS")
        pose_result = self.refresh_pose()
        if not pose_result.success:
            return pose_result
        delta = step * (1 if direction >= 0 else -1)
        target = Pose(self.pose.x, self.pose.y, self.pose.z, self.pose.r)
        if axis == "X":
            target.x += delta
        elif axis == "Y":
            target.y += delta
        elif axis == "Z":
            target.z += delta
        elif axis == "R":
            target.r += delta
        return self.move_l(target, 10.0, 30.0)

    def stop(self) -> Result:
        result = self._dashboard("EmergencyStop()")
        if not result.success:
            reset_result = self._dashboard("ResetRobot()")
            return reset_result if reset_result.success else result
        return result

    def digital_input(self, channel: int) -> Result:
        return self._dashboard(f"DI({channel:d})")

    def read_digital_input(self, channel: int) -> tuple[Result, bool]:
        result = self.digital_input(channel)
        if not result.success:
            return result, False
        values = _extract_reply_numbers(self.last_reply)
        if not values:
            return Result.fail(f"Unexpected DI{channel:02d} reply: {self.last_reply}", "BAD_REPLY"), False
        return result, bool(int(values[0]))

    def digital_output(self, channel: int, state: bool) -> Result:
        return self._dashboard(f"DO({channel:d},{1 if state else 0})")

    def refresh_status(self) -> Result:
        mode_result = self._dashboard("RobotMode()")
        if mode_result.success:
            values = _extract_reply_numbers(self.last_reply)
            if values:
                self.robot_mode_code = int(values[0])
                self.robot_mode_text = ROBOT_MODES.get(self.robot_mode_code, f"UNKNOWN_{self.robot_mode_code}")
        error_result = self._dashboard("GetErrorID()")
        if error_result.success:
            self.error_ids = _extract_error_ids(self.last_reply)
        if not mode_result.success:
            return mode_result
        if not error_result.success:
            return error_result
        error_text = "No controller/servo alarms" if not self.error_ids else f"Alarm IDs: {', '.join(str(item) for item in self.error_ids)}"
        return Result.ok(f"RobotMode={self.robot_mode_code} ({self.robot_mode_text}); {error_text}")

    def _move(self, name: str, pose: Pose, speed: float, acceleration: float) -> Result:
        if not self.move.connected:
            return Result.fail("MG400 move port is not connected", "NOT_CONNECTED")
        speed_key = "SpeedJ" if name == "MovJ" else "SpeedL"
        acc_key = "AccJ" if name == "MovJ" else "AccL"
        for command in [f"{speed_key}({int(speed)})", f"{acc_key}({int(acceleration)})"]:
            result = self._dashboard(command)
            if not result.success:
                return result
        return self._move_command(f"{name}({pose.x:.3f},{pose.y:.3f},{pose.z:.3f},{pose.r:.3f})")

    def _dashboard(self, command: str) -> Result:
        if not self.dashboard.connected:
            return Result.fail("MG400 dashboard port is not connected", "NOT_CONNECTED")
        try:
            reply = self.dashboard.request(command)
        except (OSError, TimeoutError, ConnectionError) as exc:
            return Result.fail(f"{command} failed: {exc}", "COMMUNICATION_ERROR")
        self.last_reply = reply
        return _result_from_reply(reply, command)

    def _move_command(self, command: str) -> Result:
        try:
            reply = self.move.request(command)
        except (OSError, TimeoutError, ConnectionError) as exc:
            return Result.fail(f"{command} failed: {exc}", "COMMUNICATION_ERROR")
        self.last_reply = reply
        result = _result_from_reply(reply, command)
        if result.success and command.startswith(("MovJ", "MovL")):
            sync = self._move_command("Sync()")
            if sync.success:
                self.refresh_pose()
                self.refresh_joints()
            return sync
        return result


def _reply_error_code(reply: str) -> int | None:
    match = re.match(r"\s*(-?\d+)\s*,", reply)
    if not match:
        return None
    return int(match.group(1))


def _result_from_reply(reply: str, command: str) -> Result:
    code = _reply_error_code(reply)
    if code is None:
        return Result.fail(f"{command} returned unparseable reply: {reply}", "BAD_REPLY")
    if code == 0:
        return Result.ok(f"{command} OK")
    detail = DOBOT_ERROR_CODES.get(code, "Unknown Dobot controller error")
    return Result.fail(f"{command} failed with controller code {code}: {detail}. Raw reply: {reply}", f"DOBOT_{code}")


def _extract_reply_numbers(reply: str) -> list[float]:
    match = re.search(r"\{([^}]*)\}", reply)
    if not match:
        return []
    numbers = []
    for token in match.group(1).split(","):
        token = token.strip()
        if not token:
            continue
        try:
            numbers.append(float(token))
        except ValueError:
            continue
    return numbers


def _extract_error_ids(reply: str) -> list[int]:
    match = re.search(r"\{(.+)\}", reply)
    if not match:
        return []
    return [int(value) for value in re.findall(r"-?\d+", match.group(1)) if int(value) != 0]
