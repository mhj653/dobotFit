from __future__ import annotations

from PySide6.QtCore import QObject, Signal

from core.models import JointState, Pose, Result
from core.tool_config import GripperIOConfig
from drivers.camera import RealSenseD405Camera
from drivers.gripper import DobotSoftGripper, SimulationSoftGripper
from drivers.io import SimulationIO
from drivers.plc import MockPLC
from drivers.robot import RealMG400, SimulationMG400
from drivers.robot.mg400_tcp import MG400ConnectionConfig


class DeviceManager(QObject):
    state_changed = Signal()
    log_requested = Signal(str, str, str)
    packet_logged = Signal(object)

    def __init__(self) -> None:
        super().__init__()
        self.mode = "SIMULATION"
        self.real_robot_config = MG400ConnectionConfig()
        self.gripper_io = GripperIOConfig()
        self.robot = SimulationMG400()
        self.gripper = SimulationSoftGripper()
        self.io = SimulationIO()
        self.camera2d = RealSenseD405Camera()
        self.camera3d = self.camera2d
        self.plc = MockPLC()
        self.last_alarm = ""
        self.last_grip_ok = False
        self._apply_gripper_config()
        self.connect_all()

    def real_robot_connected(self) -> bool:
        connected = getattr(self.robot, "connected", None)
        if isinstance(connected, bool):
            return connected
        dashboard = getattr(self.robot, "dashboard", None)
        move = getattr(self.robot, "move", None)
        return bool(dashboard is not None and move is not None and dashboard.connected and move.connected)

    def connect_realsense(self, serial: str = "") -> Result:
        disconnect = getattr(self.camera3d, "disconnect", None)
        if callable(disconnect):
            disconnect()
        camera = RealSenseD405Camera(serial=serial)
        result = camera.connect()
        if result.success:
            self.camera2d = camera
            self.camera3d = camera
        self._log("INFO" if result.success else "ERROR", "Vision", result.message)
        self.state_changed.emit()
        return result

    def set_mode(self, mode: str) -> Result:
        normalized = "REAL" if mode == "REAL" else "SIMULATION"
        if normalized == self.mode:
            if normalized == "REAL" and isinstance(self.robot, RealMG400):
                message = "Real robot mode already selected"
                if self.real_robot_connected():
                    message += "; existing MG400 connection preserved"
                result = Result.ok(message)
            elif normalized == "SIMULATION" and isinstance(self.robot, SimulationMG400):
                result = Result.ok("Simulation mode already selected")
            else:
                result = Result.ok(f"{normalized} mode already selected")
            self._log("INFO", "Device", result.message)
            self.state_changed.emit()
            return result

        disconnect = getattr(self.robot, "disconnect", None)
        if callable(disconnect):
            disconnect()

        self.mode = normalized
        if normalized == "REAL":
            self.robot = RealMG400(self.real_robot_config, self._on_robot_packet)
            self.gripper = DobotSoftGripper(self._write_robot_do, self._read_robot_di)
            self._apply_gripper_config()
            result = Result.ok("Real robot mode selected. Connect explicitly before motion.")
            self.last_alarm = ""
            self._log("WARNING", "Device", result.message)
        else:
            self.robot = SimulationMG400()
            self.gripper = SimulationSoftGripper()
            self._apply_gripper_config()
            self.last_grip_ok = False
            result = self.connect_all()
            self.last_alarm = ""
        self.state_changed.emit()
        return result

    def configure_real_robot(
        self,
        ip: str,
        dashboard_port: int = 29999,
        move_port: int = 30003,
        feedback_port: int = 30004,
        timeout_s: float = 1.5,
    ) -> Result:
        new_config = MG400ConnectionConfig(ip, dashboard_port, move_port, feedback_port, timeout_s)
        same_config = new_config == self.real_robot_config
        self.real_robot_config = new_config
        if self.mode == "REAL":
            if same_config and isinstance(self.robot, RealMG400):
                message = f"Real MG400 connection settings unchanged: {ip}"
                if self.real_robot_connected():
                    message += "; existing connection preserved"
                result = Result.ok(message)
                self._log("INFO", "Device", result.message)
                self.state_changed.emit()
                return result
            disconnect = getattr(self.robot, "disconnect", None)
            if callable(disconnect):
                disconnect()
            self.robot = RealMG400(self.real_robot_config, self._on_robot_packet)
            self.gripper = DobotSoftGripper(self._write_robot_do, self._read_robot_di)
            self._apply_gripper_config()
        result = Result.ok(f"Real MG400 connection settings updated: {ip}")
        self._log("INFO", "Device", result.message)
        self.state_changed.emit()
        return result

    def connect_all(self) -> Result:
        robot_result = self.robot.connect()
        if robot_result.success:
            enable_result = self.robot.enable()
            if not enable_result.success:
                self._log("WARNING", "Robot", enable_result.message)
        else:
            self.last_alarm = robot_result.message
        self._log("INFO" if robot_result.success else "ERROR", "Device", robot_result.message)
        self.state_changed.emit()
        return robot_result

    def robot_pose(self) -> Pose:
        return self.robot.get_pose()

    def robot_joints(self) -> JointState:
        return self.robot.get_joints()

    def refresh_robot_state(self) -> Result:
        results: list[Result] = []
        refresh_pose = getattr(self.robot, "refresh_pose", None)
        refresh_joints = getattr(self.robot, "refresh_joints", None)
        refresh_status = getattr(self.robot, "refresh_status", None)
        if callable(refresh_pose):
            results.append(refresh_pose())
        else:
            self.robot.get_pose()
        if callable(refresh_joints):
            results.append(refresh_joints())
        else:
            self.robot.get_joints()
        if callable(refresh_status):
            results.append(refresh_status())
        failed = next((result for result in results if not result.success), None)
        result = failed or Result.ok("Robot state refreshed")
        self._log("INFO" if result.success else "ERROR", "Robot", result.message)
        if not result.success:
            self.last_alarm = result.message
        elif getattr(self.robot, "error_ids", []):
            self.last_alarm = ", ".join(str(item) for item in self.robot.error_ids)
        else:
            self.last_alarm = ""
        self.state_changed.emit()
        return result

    def move_robot(self, motion: str, pose: Pose, speed: float, acceleration: float) -> Result:
        result = self.robot.move_l(pose, speed, acceleration) if motion == "MoveL" else self.robot.move_j(pose, speed, acceleration)
        self._log("INFO" if result.success else "ERROR", "Robot", result.message)
        if not result.success:
            self.last_alarm = result.message
        self.state_changed.emit()
        return result

    def jog_robot(self, axis: str, direction: int, step: float) -> Result:
        result = self.robot.jog(axis, direction, step)
        self._log("INFO" if result.success else "ERROR", "Robot", result.message)
        if not result.success:
            self.last_alarm = result.message
        self.state_changed.emit()
        return result

    def stop_robot(self) -> Result:
        result = self.robot.stop()
        self._log("WARNING", "Robot", result.message)
        self.state_changed.emit()
        return result

    def enable_robot(self) -> Result:
        result = self.robot.enable()
        self._log("INFO" if result.success else "ERROR", "Robot", result.message)
        self.state_changed.emit()
        return result

    def disable_robot(self) -> Result:
        result = self.robot.disable()
        self._log("INFO", "Robot", result.message)
        self.state_changed.emit()
        return result

    def clear_error(self) -> Result:
        result = self.robot.clear_error()
        if result.success:
            self.last_alarm = ""
        self._log("INFO" if result.success else "ERROR", "Robot", result.message)
        self.state_changed.emit()
        return result

    def set_do(self, channel: int, state: bool) -> Result:
        if self.mode == "REAL":
            result = self._write_robot_do(channel, state)
        else:
            result = self.io.set_do(channel, state)
        self._log("INFO" if result.success else "ERROR", "I/O", result.message)
        self.state_changed.emit()
        return result

    def set_di(self, channel: int, state: bool) -> Result:
        if self.mode == "REAL":
            result, actual = self._read_robot_di(channel)
            if result.success:
                result = Result.ok(f"DI{channel:02d}={'ON' if actual else 'OFF'}")
        else:
            result = self.io.set_di(channel, state)
        self._log("INFO" if result.success else "ERROR", "I/O", result.message)
        self.state_changed.emit()
        return result

    def read_di(self, channel: int) -> bool:
        if self.mode == "REAL":
            self._read_robot_di(channel)
        return self.io.read_di(channel)

    def gripper_open(self) -> Result:
        result = self.gripper.open()
        if result.success and self.mode == "SIMULATION":
            if hasattr(self.robot, "model"):
                self.robot.model.set_gripper(False)
            self.io.set_do(self.gripper_io.open_output, True)
            self.io.set_do(self.gripper_io.close_output, False)
            self.io.set_di(self.gripper_io.sensor_input, False)
            self.last_grip_ok = False
        elif result.success and self.mode == "REAL":
            self.last_grip_ok = False
        self._log("INFO" if result.success else "ERROR", "Gripper", result.message)
        self.state_changed.emit()
        return result

    def gripper_close(self) -> Result:
        result = self.gripper.close()
        if result.success and self.mode == "SIMULATION":
            if hasattr(self.robot, "model"):
                self.robot.model.set_gripper(True)
            self.io.set_do(self.gripper_io.open_output, False)
            self.io.set_do(self.gripper_io.close_output, True)
            self.io.set_di(self.gripper_io.sensor_input, True)
            self.last_grip_ok = True
        elif result.success and self.mode == "REAL":
            self.last_grip_ok = bool(getattr(self.gripper, "last_sensor_state", False))
        self._log("INFO" if result.success else "ERROR", "Gripper", result.message)
        self.state_changed.emit()
        return result

    def configure_gripper_io(self, open_output: int, close_output: int, sensor_input: int) -> Result:
        self.gripper_io = GripperIOConfig(open_output, close_output, sensor_input)
        self._apply_gripper_config()
        self.io.alias = {
            f"DO{self.gripper_io.open_output:02d}": "Gripper_Open",
            f"DO{self.gripper_io.close_output:02d}": "Gripper_Close",
            f"DI{self.gripper_io.sensor_input:02d}": "Grip_OK",
        }
        result = Result.ok(
            f"Gripper I/O set: OPEN {self.gripper_io.open_label}, CLOSE {self.gripper_io.close_label}, SENSOR {self.gripper_io.sensor_label}"
        )
        self._log("INFO", "Gripper", result.message)
        self.state_changed.emit()
        return result

    def load_gripper_config(self, data: dict | None) -> None:
        self.gripper_io = GripperIOConfig.from_dict(data)
        self._apply_gripper_config()

    def gripper_config_dict(self) -> dict:
        return {"soft_gripper": self.gripper_io.to_dict()}

    def _apply_gripper_config(self) -> None:
        configure = getattr(self.gripper, "configure_io", None)
        if callable(configure):
            configure(self.gripper_io.open_output, self.gripper_io.close_output, self.gripper_io.sensor_input)

    def _write_robot_do(self, channel: int, state: bool) -> Result:
        digital_output = getattr(self.robot, "digital_output", None)
        if not callable(digital_output):
            return Result.fail("Current robot does not support digital output", "DO_NOT_SUPPORTED")
        result = digital_output(channel, state)
        if result.success and channel in self.io.do:
            self.io.do[channel] = bool(state)
        if not result.success:
            self.last_alarm = result.message
        return result

    def _read_robot_di(self, channel: int) -> tuple[Result, bool]:
        read_input = getattr(self.robot, "read_digital_input", None)
        if not callable(read_input):
            return Result.fail("Current robot does not support digital input", "DI_NOT_SUPPORTED"), False
        result, state = read_input(channel)
        if result.success and channel in self.io.di:
            self.io.di[channel] = bool(state)
        if not result.success:
            self.last_alarm = result.message
        return result, bool(state)

    def _log(self, level: str, source: str, message: str) -> None:
        self.log_requested.emit(level, source, message)

    def _on_robot_packet(self, packet: dict) -> None:
        self.packet_logged.emit(packet)

    def simulation_snapshot(self, render: bool = True):
        if hasattr(self.robot, "model"):
            return self.robot.model.snapshot(render)
        return None

    def configure_simulation_tool(self, tool) -> Result:
        if not hasattr(self.robot, "model"):
            return Result.fail("Simulation tool settings are available only in simulation mode", "NOT_SIMULATION")
        self.robot.model.set_tool_profile(tool)
        result = Result.ok(f"Simulation tool set: {tool.name}")
        self._log("INFO", "Simulation", result.message)
        self.state_changed.emit()
        return result

    def set_simulation_objects(self, objects) -> Result:
        if not hasattr(self.robot, "model"):
            return Result.fail("Simulation scene settings are available only in simulation mode", "NOT_SIMULATION")
        self.robot.model.set_scene_objects(objects)
        result = Result.ok(f"Simulation scene updated: {len(objects)} object(s)")
        self._log("INFO", "Simulation", result.message)
        self.state_changed.emit()
        return result

    def reset_simulation_objects(self) -> Result:
        if not hasattr(self.robot, "model"):
            return Result.fail("Simulation scene settings are available only in simulation mode", "NOT_SIMULATION")
        self.robot.model.reset_scene_objects()
        result = Result.ok("Simulation scene reset")
        self._log("INFO", "Simulation", result.message)
        self.state_changed.emit()
        return result

    def simulation_warnings_for_pose(self, pose: Pose) -> list[str]:
        if hasattr(self.robot, "model"):
            return self.robot.model.check_path([self.robot.model.pose, pose])
        return []

    def set_simulation_view(self, name: str) -> None:
        if hasattr(self.robot, "model"):
            self.robot.model.set_view(name)
            self.state_changed.emit()
