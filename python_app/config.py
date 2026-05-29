from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any


APP_DIR = Path(__file__).resolve().parent
CONFIG_DIR = APP_DIR / "config"
DATA_DIR = APP_DIR / "data"
GESTURE_DATA_DIR = DATA_DIR / "gestures"
CALIBRATION_PATH = CONFIG_DIR / "calibration.json"


DEFAULT_CALIBRATION: dict[str, Any] = {
    "cam_pan_center": 307,
    "cam_tilt_center": 307,
    "r_pan_center": 307,
    "r_tilt_center": 307,
    "l_pan_center": 307,
    "l_tilt_center": 307,
    "x_gain_ticks": 120,
    "y_gain_ticks": 90,
    "deadband": 4,
    "smoothing_alpha": 0.35,
    "hand_boundary_left": 0.0,
    "hand_boundary_right": 1.0,
    "hand_boundary_top": 0.0,
    "hand_boundary_bottom": 1.0,
    "hand_calibration_points": {},
    "use_dock_geometry": 1,
    "use_dock_servo_pair": 0,
    "hand_servo_deadband": 2.0,
    "hand_servo_smoothing_alpha": 0.62,
    "initial_hand_distance_mm": 700.0,
    "min_valid_tof_mm": 80.0,
    "max_valid_tof_mm": 2000.0,
    "tof_depth_alpha": 0.45,
    "prediction_latency_ms": 50.0,
    "camera_horizontal_fov_deg": 70.0,
    "camera_vertical_fov_deg": 43.0,
    "left_bracket_x_mm": -59.5,
    "left_bracket_y_mm": 0.0,
    "left_bracket_z_mm": -60.0,
    "right_bracket_x_mm": 100.5,
    "right_bracket_y_mm": 0.0,
    "right_bracket_z_mm": -60.0,
    "left_tilt_pivot_offset_x_mm": 0.0,
    "left_tilt_pivot_offset_y_mm": 0.0,
    "left_tilt_pivot_offset_z_mm": 20.0,
    "right_tilt_pivot_offset_x_mm": 0.0,
    "right_tilt_pivot_offset_y_mm": 0.0,
    "right_tilt_pivot_offset_z_mm": 20.0,
    "left_tof_offset_x_mm": 0.0,
    "left_tof_offset_y_mm": 0.0,
    "left_tof_offset_z_mm": 25.0,
    "right_tof_offset_x_mm": 0.0,
    "right_tof_offset_y_mm": 0.0,
    "right_tof_offset_z_mm": 25.0,
    "pan_ticks_per_degree": 2.25,
    "tilt_ticks_per_degree": 2.25,
    "r_pan_sign": -1.0,
    "r_tilt_sign": -1.0,
    "l_pan_sign": -1.0,
    "l_tilt_sign": -1.0,
    "r_pan_angle_offset_deg": 27.17,
    "r_tilt_angle_offset_deg": 0.0,
    "l_pan_angle_offset_deg": -6.93,
    "l_tilt_angle_offset_deg": 0.0,
    "session_calibration": {},
    "ota_wifi_ssid": "",
    "ota_wifi_password": "",
    "ota_server_port": 8765,
}


@dataclass
class AppConfig:
    serial_port: str | None = None
    serial_baud: int = 921600
    camera_index: int = 1
    camera_width: int = 640
    camera_height: int = 480
    horizontal_fov_deg: float = 70.0
    vertical_fov_deg: float = 43.0
    servo_min_tick: int = 0
    servo_max_tick: int = 4095
    calibration_path: Path = CALIBRATION_PATH
    gesture_data_dir: Path = GESTURE_DATA_DIR
    calibration: dict[str, Any] = field(default_factory=lambda: dict(DEFAULT_CALIBRATION))


def ensure_directories() -> None:
    CONFIG_DIR.mkdir(parents=True, exist_ok=True)
    GESTURE_DATA_DIR.mkdir(parents=True, exist_ok=True)


def _merged_calibration(data: dict[str, Any] | None) -> dict[str, Any]:
    merged = dict(DEFAULT_CALIBRATION)
    if data:
        for key, value in data.items():
            if key in merged:
                merged[key] = value
    return merged


def load_calibration(path: Path = CALIBRATION_PATH) -> dict[str, Any]:
    ensure_directories()
    if not path.exists():
        save_calibration(DEFAULT_CALIBRATION, path)
        return dict(DEFAULT_CALIBRATION)

    try:
        with path.open("r", encoding="utf-8") as f:
            data = json.load(f)
    except (OSError, json.JSONDecodeError):
        return dict(DEFAULT_CALIBRATION)
    return _merged_calibration(data if isinstance(data, dict) else None)


def save_calibration(calibration: dict[str, Any], path: Path = CALIBRATION_PATH) -> None:
    ensure_directories()
    merged = _merged_calibration(calibration)
    with path.open("w", encoding="utf-8") as f:
        json.dump(merged, f, indent=2)
        f.write("\n")


def load_app_config() -> AppConfig:
    ensure_directories()
    return AppConfig(calibration=load_calibration())
