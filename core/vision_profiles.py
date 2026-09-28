from __future__ import annotations

from copy import deepcopy
from typing import Any

from drivers.camera.base import CalibrationConfig, DetectionConfig


DEFAULT_VISION_PROFILE = "Default Blob"
DEFAULT_CALIBRATION_PROFILE = "Default Camera To Robot"


def default_detection_dict() -> dict[str, Any]:
    config = DetectionConfig()
    return {
        "method": config.method,
        "threshold": config.threshold,
        "min_area": config.min_area,
        "max_area": config.max_area,
        "blob_polarity": config.blob_polarity,
        "blur_kernel": config.blur_kernel,
        "open_iterations": config.open_iterations,
        "close_iterations": config.close_iterations,
        "min_circularity": config.min_circularity,
        "yolo_model_path": config.yolo_model_path,
        "yolo_class_filter": config.yolo_class_filter,
        "angle_offset_deg": config.angle_offset_deg,
        "roi": {
            "x_percent": config.roi_x_percent,
            "y_percent": config.roi_y_percent,
            "w_percent": config.roi_w_percent,
            "h_percent": config.roi_h_percent,
        },
    }


def default_calibration_dict() -> dict[str, Any]:
    return {
        "enabled": False,
        "camera_to_robot": CalibrationConfig.identity().camera_to_robot,
        "method": "Point Pair",
        "checkerboard": {"columns": 7, "rows": 6, "square_size_mm": 20.0},
        "pairs": [],
        "robot_poses": [],
    }


def default_vision_store() -> dict[str, Any]:
    return {
        "version": 3,
        "active": DEFAULT_VISION_PROFILE,
        "profiles": [{"name": DEFAULT_VISION_PROFILE, "detection": default_detection_dict()}],
    }


def default_calibration_store() -> dict[str, Any]:
    return {
        "version": 3,
        "active": DEFAULT_CALIBRATION_PROFILE,
        "profiles": [{"name": DEFAULT_CALIBRATION_PROFILE, "calibration": default_calibration_dict()}],
    }


def normalize_vision_store(data: dict[str, Any] | None) -> dict[str, Any]:
    if not isinstance(data, dict) or not data:
        return default_vision_store()
    if "profiles" in data:
        store = deepcopy(data)
    elif "detection" in data:
        store = {"version": 3, "active": DEFAULT_VISION_PROFILE, "profiles": [{"name": DEFAULT_VISION_PROFILE, "detection": data["detection"]}]}
    else:
        store = {"version": 3, "active": DEFAULT_VISION_PROFILE, "profiles": [{"name": DEFAULT_VISION_PROFILE, "detection": default_detection_dict()}]}
    return _normalize_store(store, "detection", DEFAULT_VISION_PROFILE, default_detection_dict())


def normalize_calibration_store(data: dict[str, Any] | None) -> dict[str, Any]:
    if not isinstance(data, dict) or not data:
        return default_calibration_store()
    if "profiles" in data:
        store = deepcopy(data)
    elif "calibration" in data:
        store = {
            "version": 3,
            "active": DEFAULT_CALIBRATION_PROFILE,
            "profiles": [{"name": DEFAULT_CALIBRATION_PROFILE, "calibration": data["calibration"]}],
        }
    elif "robot_base" in data:
        calibration = default_calibration_dict()
        calibration["camera_to_robot"] = data.get("robot_base", calibration["camera_to_robot"])
        store = {
            "version": 3,
            "active": DEFAULT_CALIBRATION_PROFILE,
            "profiles": [{"name": DEFAULT_CALIBRATION_PROFILE, "calibration": calibration}],
        }
    else:
        store = default_calibration_store()
    return _normalize_store(store, "calibration", DEFAULT_CALIBRATION_PROFILE, default_calibration_dict())


def _normalize_store(store: dict[str, Any], payload_key: str, default_name: str, default_payload: dict[str, Any]) -> dict[str, Any]:
    profiles = []
    seen: set[str] = set()
    for index, profile in enumerate(store.get("profiles", [])):
        if not isinstance(profile, dict):
            continue
        name = str(profile.get("name") or f"{default_name} {index + 1}").strip()
        if not name or name in seen:
            continue
        payload = profile.get(payload_key)
        profiles.append({"name": name, payload_key: deepcopy(payload if isinstance(payload, dict) else default_payload)})
        seen.add(name)
    if not profiles:
        profiles = [{"name": default_name, payload_key: deepcopy(default_payload)}]
    active = str(store.get("active") or profiles[0]["name"]).strip()
    if active not in {profile["name"] for profile in profiles}:
        active = profiles[0]["name"]
    return {"version": 3, "active": active, "profiles": profiles}


def profile_names(store: dict[str, Any]) -> list[str]:
    return [str(profile.get("name", "")).strip() for profile in store.get("profiles", []) if str(profile.get("name", "")).strip()]


def find_profile(store: dict[str, Any], name: str | None) -> dict[str, Any] | None:
    wanted = str(name or "").strip()
    profiles = store.get("profiles", [])
    for profile in profiles:
        if str(profile.get("name", "")).strip() == wanted:
            return profile
    active = str(store.get("active", "")).strip()
    for profile in profiles:
        if str(profile.get("name", "")).strip() == active:
            return profile
    return profiles[0] if profiles else None


def replace_profile(store: dict[str, Any], payload_key: str, name: str, payload: dict[str, Any]) -> dict[str, Any]:
    clean_name = name.strip()
    if not clean_name:
        raise ValueError("Profile name is required")
    updated = deepcopy(store)
    profiles = updated.setdefault("profiles", [])
    for profile in profiles:
        if str(profile.get("name", "")).strip() == clean_name:
            profile[payload_key] = deepcopy(payload)
            break
    else:
        profiles.append({"name": clean_name, payload_key: deepcopy(payload)})
    updated["active"] = clean_name
    updated["version"] = 3
    return updated


def detection_config_from_profile(profile: dict[str, Any] | None) -> DetectionConfig:
    data = profile.get("detection", {}) if isinstance(profile, dict) else {}
    roi = data.get("roi", {}) if isinstance(data, dict) else {}
    threshold = float(data.get("threshold", 0.55))
    if threshold > 1.0:
        threshold = threshold / 255.0
    return DetectionConfig(
        method=str(data.get("method", "Blob")),
        roi_x_percent=float(roi.get("x_percent", 35.0)),
        roi_y_percent=float(roi.get("y_percent", 30.0)),
        roi_w_percent=float(roi.get("w_percent", 30.0)),
        roi_h_percent=float(roi.get("h_percent", 40.0)),
        threshold=threshold,
        min_area=float(data.get("min_area", 80.0)),
        max_area=float(data.get("max_area", 20000.0)),
        blob_polarity=str(data.get("blob_polarity", "Auto")),
        blur_kernel=int(float(data.get("blur_kernel", 5))),
        open_iterations=int(float(data.get("open_iterations", 0))),
        close_iterations=int(float(data.get("close_iterations", 0))),
        min_circularity=float(data.get("min_circularity", 0.0)),
        yolo_model_path=str(data.get("yolo_model_path", "")),
        yolo_class_filter=str(data.get("yolo_class_filter", "")),
        angle_offset_deg=float(data.get("angle_offset_deg", 0.0)),
    )


def calibration_config_from_profile(profile: dict[str, Any] | None) -> CalibrationConfig:
    data = profile.get("calibration", {}) if isinstance(profile, dict) else {}
    matrix = data.get("camera_to_robot", CalibrationConfig.identity().camera_to_robot)
    if not isinstance(matrix, list) or len(matrix) != 16:
        matrix = CalibrationConfig.identity().camera_to_robot
    return CalibrationConfig([float(value) for value in matrix], bool(data.get("enabled", False)))
