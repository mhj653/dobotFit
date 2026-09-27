from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from app.application import run
from core.device_manager import DeviceManager


if __name__ == "__main__":
    if "--self-check" in sys.argv:
        devices = DeviceManager()
        snapshot = devices.simulation_snapshot()
        backend = snapshot.backend if snapshot is not None else "NONE"
        physics_objects = snapshot.physics_objects if snapshot is not None else 0
        render_width = snapshot.render_width if snapshot is not None else 0
        render_height = snapshot.render_height if snapshot is not None else 0
        render_bytes = len(snapshot.render_rgba or b"") if snapshot is not None else 0
        print(
            "RobotAutomationStudio self-check "
            f"backend={backend} physics_objects={physics_objects} "
            f"render={render_width}x{render_height} bytes={render_bytes}"
        )
        raise SystemExit(0 if backend == "PyBullet" and render_bytes > 0 else 2)
    raise SystemExit(run())
