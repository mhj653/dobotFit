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
        return self._dashboard("ClearError()")

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

    def digital_output(self, channel: int, state: bool) -> Result:
        return self._dashboard(f"DO({channel:d},{1 if state else 0})")

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
    return Result.fail(f"{command} failed with controller code {code}: {reply}", f"DOBOT_{code}")


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
