from __future__ import annotations

import sys
from pathlib import Path


def app_root() -> Path:
    if getattr(sys, "frozen", False):
        return Path(sys.executable).resolve().parent
    return Path(__file__).resolve().parents[1]


def models_dir() -> Path:
    path = app_root() / "models"
    path.mkdir(parents=True, exist_ok=True)
    return path


def projects_dir() -> Path:
    path = app_root() / "projects"
    path.mkdir(parents=True, exist_ok=True)
    return path


def configs_dir() -> Path:
    path = app_root() / "configs"
    path.mkdir(parents=True, exist_ok=True)
    return path


def training_tool_candidates() -> list[Path]:
    root = app_root()
    return [
        root.parent / "YoloTrainingUtility" / "YoloSegTrainingUtility.exe",
        root / "YoloTrainingUtility" / "YoloSegTrainingUtility.exe",
        root.parent / "YoloSegTrainingUtility" / "YoloSegTrainingUtility.exe",
        Path(r"D:\Samsung\realsense\dist\YoloSegTrainingUtility\YoloSegTrainingUtility.exe"),
    ]


def find_training_tool() -> Path | None:
    for candidate in training_tool_candidates():
        if candidate.exists():
            return candidate
    return None
