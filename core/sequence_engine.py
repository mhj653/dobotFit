from __future__ import annotations

import time

from PySide6.QtCore import QObject, Signal

from core.device_manager import DeviceManager
from core.models import Pose, Position, Result, SequenceStep
from drivers.camera.base import detection_to_variables
from core.vision_profiles import (
    calibration_config_from_profile,
    default_calibration_store,
    default_vision_store,
    detection_config_from_profile,
    find_profile,
    normalize_calibration_store,
    normalize_vision_store,
    profile_names,
)


class SequenceEngine(QObject):
    step_finished = Signal(object)
    variables_changed = Signal(dict)

    def __init__(self, device_manager: DeviceManager) -> None:
        super().__init__()
        self.device_manager = device_manager
        self.variables: dict[str, float | bool | int | str] = {
            "vision_x": 320.0,
            "vision_y": 60.0,
            "vision_z": 170.0,
            "vision_r": 90.0,
            "part_ok": True,
            "grip_ok": False,
            "count": 0,
        }
        self.last_step_times: list[tuple[int, str, float]] = []
        self.vision_store = default_vision_store()
        self.calibration_store = default_calibration_store()

    def set_profiles(self, vision: dict | None, calibration: dict | None) -> None:
        self.vision_store = normalize_vision_store(vision)
        self.calibration_store = normalize_calibration_store(calibration)

    def execute_step(self, step: SequenceStep, positions: list[Position]) -> Result:
        start = time.perf_counter()
        should_run, skip_message = self._should_run(step)
        if should_run:
            result = self._execute(step, positions)
            step.status = "Done" if result.success else "Error"
        else:
            result = Result.ok(skip_message)
            step.status = "Skipped"
        step.elapsed_ms = (time.perf_counter() - start) * 1000.0
        self.last_step_times.append((step.no, f"{step.device}.{step.command}", step.elapsed_ms))
        self.step_finished.emit(step)
        self.variables_changed.emit(dict(self.variables))
        return result

    def _execute(self, step: SequenceStep, positions: list[Position]) -> Result:
        device = step.device.lower()
        command = step.command.lower()
        target = step.target.strip()

        if device == "robot" and command in {"movej", "movel"}:
            pose = self._resolve_pose(target, positions)
            if pose is None:
                return Result.fail(f"Unknown robot target: {target}", "BAD_TARGET")
            return self.device_manager.move_robot("MoveL" if command == "movel" else "MoveJ", pose, step.speed, step.acceleration)

        if device == "robot" and command == "jog":
            axis, direction, step_size = self._parse_jog(target)
            return self.device_manager.jog_robot(axis, direction, step_size)

        if device == "gripper" and command == "open":
            return self.device_manager.gripper_open()
        if device == "gripper" and command == "close":
            result = self.device_manager.gripper_close()
            self.variables["grip_ok"] = result.success
            return result

        if device == "wait" and command == "time":
            seconds = max(0.0, min(300.0, self._parse_wait_seconds(target)))
            time.sleep(seconds)
            return Result.ok(f"Wait {seconds:.2f} s complete")

        if device == "wait" and command == "di":
            channel = int(float(target or 1))
            if self.device_manager.read_di(channel):
                return Result.ok(f"DI{channel:02d} is ON")
            return Result.fail(f"DI{channel:02d} is OFF", "WAIT_DI_TIMEOUT")

        if device in {"io", "i/o"} and command == "setdo":
            channel, state = self._parse_do(target)
            return self.device_manager.set_do(channel, state)

        if device == "vision" and command == "detect":
            self._apply_step_vision_profiles(step)
            result = self.device_manager.camera2d.detect()
            self.variables.update(detection_to_variables(result))
            vision_name = step.vision_profile or str(self.vision_store.get("active", ""))
            calibration_name = step.calibration_profile or str(self.calibration_store.get("active", ""))
            return Result.ok(f"Vision detect complete: {target or 'D405_LOCATE'} [{vision_name} / {calibration_name}]")

        if device == "plc" and command == "setoutput":
            name, state = self._parse_plc_output(target)
            return self.device_manager.plc.write_output(name, state)

        if command == "comment":
            return Result.ok("Comment")

        return Result.fail(f"Unsupported step: {step.device} {step.command}", "UNSUPPORTED_STEP")

    def validate_sequence(self, sequence: list[SequenceStep], positions: list[Position]) -> list[str]:
        issues: list[str] = []
        for index, step in enumerate(sequence, start=1):
            prefix = f"Step {index}"
            device = step.device.lower()
            command = step.command.lower()
            if not step.enabled:
                continue
            if step.condition.strip() and self._evaluate_condition(step.condition) is None:
                issues.append(f"{prefix}: unsupported condition '{step.condition}'")
            if device == "robot" and command in {"movej", "movel"}:
                if self._resolve_pose(step.target.strip(), positions) is None:
                    issues.append(f"{prefix}: unknown position '{step.target}'")
                if not (1.0 <= step.speed <= 100.0):
                    issues.append(f"{prefix}: speed must be 1..100")
                if not (1.0 <= step.acceleration <= 100.0):
                    issues.append(f"{prefix}: acceleration must be 1..100")
            elif device == "robot" and command == "jog":
                try:
                    self._parse_jog(step.target)
                except ValueError:
                    issues.append(f"{prefix}: invalid jog target '{step.target}'")
            elif device == "wait" and command == "time":
                try:
                    value = self._parse_wait_seconds(step.target)
                    if value < 0.0 or value > 300.0:
                        issues.append(f"{prefix}: wait time must be 0.00..300.00 s")
                except ValueError:
                    issues.append(f"{prefix}: invalid wait time '{step.target}'")
            elif device == "wait" and command == "di":
                try:
                    channel = int(float(step.target or 1))
                    if channel < 1 or channel > 8:
                        issues.append(f"{prefix}: DI channel must be 1..8")
                except ValueError:
                    issues.append(f"{prefix}: invalid DI channel '{step.target}'")
            elif device in {"io", "i/o"} and command == "setdo":
                try:
                    channel, _ = self._parse_do(step.target)
                    if channel < 1 or channel > 8:
                        issues.append(f"{prefix}: DO channel must be 1..8")
                except ValueError:
                    issues.append(f"{prefix}: invalid DO target '{step.target}'")
            elif device == "gripper" and command in {"open", "close"}:
                pass
            elif device == "vision" and command == "detect":
                if step.vision_profile and step.vision_profile not in profile_names(self.vision_store):
                    issues.append(f"{prefix}: unknown vision profile '{step.vision_profile}'")
                if step.calibration_profile and step.calibration_profile not in profile_names(self.calibration_store):
                    issues.append(f"{prefix}: unknown calibration profile '{step.calibration_profile}'")
            elif device == "plc" and command == "setoutput":
                pass
            elif command == "comment":
                pass
            else:
                issues.append(f"{prefix}: unsupported step {step.device}.{step.command}")
        return issues

    def _should_run(self, step: SequenceStep) -> tuple[bool, str]:
        if not step.enabled:
            return False, "Step disabled"
        condition = step.condition.strip()
        if not condition:
            return True, ""
        evaluated = self._evaluate_condition(condition)
        if evaluated is None:
            return False, f"Unsupported condition: {condition}"
        if not evaluated:
            return False, f"Condition false: {condition}"
        return True, ""

    def _evaluate_condition(self, condition: str) -> bool | None:
        text = condition.strip()
        if not text or text.upper() in {"ALWAYS", "TRUE"}:
            return True
        if text.upper() in {"FALSE", "NEVER"}:
            return False
        lowered = text.lower()
        if lowered.startswith("not "):
            value = self._evaluate_condition(text[4:].strip())
            return None if value is None else not value
        if lowered.startswith("di"):
            normalized = text.upper().replace(" ", "")
            if "=" in normalized:
                channel_text, state_text = normalized.replace("==", "=").split("=", 1)
            else:
                parts = normalized.split()
                if len(parts) != 2:
                    return None
                channel_text, state_text = parts
            try:
                channel = int(channel_text.replace("DI", ""))
            except ValueError:
                return None
            actual = self.device_manager.read_di(channel)
            expected = state_text in {"ON", "1", "TRUE"}
            return actual == expected
        for op in [">=", "<=", "==", "!=", ">", "<", "="]:
            if op in text:
                left, right = text.split(op, 1)
                left_value = self.variables.get(left.strip())
                if left_value is None:
                    return None
                return self._compare(left_value, op, right.strip())
        value = self.variables.get(text)
        if isinstance(value, bool):
            return value
        if isinstance(value, (float, int)):
            return value != 0
        if isinstance(value, str):
            return value.lower() in {"1", "true", "on", "yes", "ok"}
        return None

    def _apply_step_vision_profiles(self, step: SequenceStep) -> None:
        vision_profile = find_profile(self.vision_store, step.vision_profile)
        calibration_profile = find_profile(self.calibration_store, step.calibration_profile)
        configure_detection = getattr(self.device_manager.camera2d, "configure_detection", None)
        if callable(configure_detection):
            configure_detection(detection_config_from_profile(vision_profile))
        configure_calibration = getattr(self.device_manager.camera2d, "configure_calibration", None)
        if callable(configure_calibration):
            configure_calibration(calibration_config_from_profile(calibration_profile))

    def _compare(self, left_value: float | bool | int | str, op: str, right_text: str) -> bool | None:
        if isinstance(left_value, bool):
            right_value: float | bool | str = right_text.lower() in {"1", "true", "on", "yes", "ok"}
        elif isinstance(left_value, (float, int)):
            try:
                right_value = float(right_text)
            except ValueError:
                return None
        else:
            right_value = right_text.strip("'\"")
        if op in {"=", "=="}:
            return left_value == right_value
        if op == "!=":
            return left_value != right_value
        if not isinstance(left_value, (float, int)) or not isinstance(right_value, (float, int)):
            return None
        if op == ">":
            return left_value > right_value
        if op == "<":
            return left_value < right_value
        if op == ">=":
            return left_value >= right_value
        if op == "<=":
            return left_value <= right_value
        return None

    def _resolve_pose(self, target: str, positions: list[Position]) -> Pose | None:
        if target.upper() == "VISION_XYZ":
            return Pose(
                float(self.variables.get("vision_x", 300.0)),
                float(self.variables.get("vision_y", 0.0)),
                float(self.variables.get("vision_z", 180.0)),
                float(self.variables.get("vision_r", 0.0)),
            )
        for position in positions:
            if position.name.upper() == target.upper():
                return position.pose
        return None

    def _parse_plc_output(self, target: str) -> tuple[str, bool]:
        if "=" not in target:
            return target or "COMPLETE", True
        name, state = target.split("=", 1)
        return name.strip(), state.strip().upper() in {"1", "ON", "TRUE"}

    def _parse_do(self, target: str) -> tuple[int, bool]:
        if "=" in target:
            channel_text, state_text = target.split("=", 1)
        else:
            channel_text, state_text = target, "ON"
        channel_text = channel_text.upper().replace("DO", "").strip()
        state = state_text.strip().upper() in {"1", "ON", "TRUE"}
        return int(channel_text or "1"), state

    def _parse_jog(self, target: str) -> tuple[str, int, float]:
        text = (target or "X+10").strip().upper()
        axis = text[0] if text[0] in {"X", "Y", "Z", "R"} else "X"
        direction = -1 if "-" in text else 1
        amount = "".join(ch for ch in text[1:] if ch.isdigit() or ch == ".")
        return axis, direction, float(amount or 10.0)

    def _parse_wait_seconds(self, target: str) -> float:
        value = float(target or 0.0)
        if value > 60.0 and value <= 5000.0:
            return value / 1000.0
        return value
