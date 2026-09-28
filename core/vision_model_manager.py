from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

from core.runtime_paths import find_training_tool, models_dir


VALID_MODEL_SUFFIXES = {".pt", ".onnx"}


def import_yolo_model(source_path: str | Path, overwrite: bool = False) -> Path:
    source = Path(source_path).expanduser().resolve()
    if not source.exists():
        raise FileNotFoundError(f"Model file not found: {source}")
    if source.suffix.lower() not in VALID_MODEL_SUFFIXES:
        raise ValueError("YOLO model must be a .pt or .onnx file")
    target_dir = models_dir()
    target_dir.mkdir(parents=True, exist_ok=True)
    target = target_dir / source.name
    if target.exists() and not overwrite:
        raise FileExistsError(f"Model already exists: {target}")
    if source != target:
        shutil.copy2(source, target)
    return target


def launch_training_tool() -> Path:
    executable = find_training_tool()
    if executable is None:
        raise FileNotFoundError("YoloSegTrainingUtility.exe was not found in the release or development paths")
    subprocess.Popen([str(executable)], cwd=str(executable.parent), close_fds=True)
    return executable
