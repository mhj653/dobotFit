from __future__ import annotations

import os

from core.runtime_paths import configs_dir


def yolo_enabled() -> bool:
    value = os.environ.get("RAS_ENABLE_YOLO", "").strip().lower()
    if value in {"1", "true", "yes", "on"}:
        return True
    if value in {"0", "false", "no", "off"}:
        return False
    return (configs_dir() / "enable_yolo.txt").exists()
