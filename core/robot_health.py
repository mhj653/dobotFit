from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class RobotHealthStep:
    name: str
    success: bool
    message: str
    required: bool = True

    def line(self) -> str:
        prefix = "OK" if self.success else "NG"
        return f"{prefix} | {self.name}: {self.message}"


@dataclass
class RobotHealthReport:
    mode: str = "SIMULATION"
    controller_mode_code: int | None = None
    controller_mode_text: str = "Unknown"
    checks: list[RobotHealthStep] = field(default_factory=list)

    @property
    def success(self) -> bool:
        return all(step.success for step in self.checks if step.required)

    @property
    def first_failure(self) -> RobotHealthStep | None:
        return next((step for step in self.checks if step.required and not step.success), None)

    def summary(self) -> str:
        if self.mode != "REAL":
            return "MG400 Simulation Connected"
        if not self.checks:
            return "REAL MG400 NOT TESTED"
        if self.success:
            if self.controller_mode_code is not None:
                return f"REAL MG400 READY: {self.controller_mode_text}"
            return "REAL MG400 READY"
        failure = self.first_failure
        return f"REAL MG400 ISSUE: {failure.name if failure else 'Unknown'}"

    def detail_text(self) -> str:
        if not self.checks:
            return "No robot communication test has been run."
        return "\n".join(step.line() for step in self.checks)

    def result_message(self) -> str:
        hint = self.troubleshooting_hint()
        return self.summary() + "\n" + self.detail_text() + (f"\nAction: {hint}" if hint else "")

    def troubleshooting_hint(self) -> str:
        failure = self.first_failure
        if failure is None:
            return ""
        text = f"{failure.name} {failure.message}".lower()
        if "tcp" in text or "socket" in text or "connection" in text or "feedback" in text:
            return "Check robot IP, PC network segment, cable, and ports 29999/30003/30004."
        if "enablerobot" in text or "getpose" in text or "robotmode" in text or "getangle" in text:
            return "Check TCP/IP secondary development mode, clear alarms/E-stop, stop running scripts/projects, then reconnect."
        return "Open Packet Log and check the first NG command raw reply."
