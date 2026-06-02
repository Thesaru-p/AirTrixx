from __future__ import annotations

import json
import shutil
from datetime import datetime
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from app_paths import (
    SOURCE_APP_DIR,
    build_app_paths,
    ensure_app_paths,
    migrate_legacy_runtime_files,
)

APP_DIR = SOURCE_APP_DIR
PATHS = build_app_paths()
CONFIG_DIR = PATHS.config_dir
DATA_DIR = PATHS.user_data_dir
LOG_DIR = PATHS.logs_dir
TEMP_DIR = PATHS.temp_dir
EXPORTS_DIR = PATHS.exports_dir
GESTURE_DATA_DIR = PATHS.gesture_data_dir
CALIBRATION_PATH = PATHS.calibration_path
MAPPING_PATH = PATHS.mapping_path
SERVO_DEBUG_LOG_PATH = PATHS.servo_debug_log_path
AUDIO_RECORDING_PATH = PATHS.audio_recording_path


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
    "use_startup_user_distance": 1,
    "startup_distance_live_weight": 0.35,
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
    "cam_pan_sign": -1.0,
    "cam_tilt_sign": -1.0,
    "cam_pan_angle_offset_deg": 0.0,
    "cam_tilt_angle_offset_deg": 0.0,
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
    "deepgram_api_key": "",
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
    user_data_dir: Path = DATA_DIR
    config_dir: Path = CONFIG_DIR
    logs_dir: Path = LOG_DIR
    temp_dir: Path = TEMP_DIR
    exports_dir: Path = EXPORTS_DIR
    calibration_path: Path = CALIBRATION_PATH
    mapping_path: Path = MAPPING_PATH
    gesture_data_dir: Path = GESTURE_DATA_DIR
    servo_debug_log_path: Path = SERVO_DEBUG_LOG_PATH
    audio_recording_path: Path = AUDIO_RECORDING_PATH
    calibration: dict[str, Any] = field(default_factory=lambda: dict(DEFAULT_CALIBRATION))
    startup_warnings: list[str] = field(default_factory=list)


def ensure_directories() -> None:
    ensure_app_paths(PATHS)


def _backup_invalid_file(path: Path) -> Path | None:
    if not path.exists():
        return None
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    backup = path.with_name(f"{path.stem}.invalid_{timestamp}{path.suffix}")
    try:
        shutil.copy2(path, backup)
        return backup
    except OSError:
        return None


def _merged_calibration(data: dict[str, Any] | None) -> dict[str, Any]:
    merged = dict(DEFAULT_CALIBRATION)
    if data:
        for key, value in data.items():
            if key in merged:
                merged[key] = value
    return merged


def load_calibration(path: Path = CALIBRATION_PATH) -> dict[str, Any]:
    calibration, _warnings = load_calibration_with_warnings(path)
    return calibration


def load_calibration_with_warnings(path: Path = CALIBRATION_PATH) -> tuple[dict[str, Any], list[str]]:
    ensure_directories()
    if not path.exists():
        save_calibration(DEFAULT_CALIBRATION, path)
        return dict(DEFAULT_CALIBRATION), []

    try:
        with path.open("r", encoding="utf-8") as f:
            data = json.load(f)
    except json.JSONDecodeError as exc:
        backup = _backup_invalid_file(path)
        message = f"Calibration config was invalid and defaults were loaded: {exc}"
        if backup:
            message += f" Backup saved to {backup}."
        return dict(DEFAULT_CALIBRATION), [message]
    except OSError as exc:
        return dict(DEFAULT_CALIBRATION), [f"Calibration config could not be read and defaults were loaded: {exc}"]
    if not isinstance(data, dict):
        return dict(DEFAULT_CALIBRATION), ["Calibration config was not an object and defaults were loaded."]
    return _merged_calibration(data), []


def save_calibration(calibration: dict[str, Any], path: Path = CALIBRATION_PATH) -> None:
    ensure_directories()
    merged = _merged_calibration(calibration)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        json.dump(merged, f, indent=2)
        f.write("\n")


def load_app_config() -> AppConfig:
    ensure_directories()
    warnings = migrate_legacy_runtime_files(PATHS)
    calibration, calibration_warnings = load_calibration_with_warnings(PATHS.calibration_path)
    startup_warnings = [f"Migrated runtime file: {item}" for item in warnings]
    startup_warnings.extend(calibration_warnings)
    return AppConfig(
        user_data_dir=PATHS.user_data_dir,
        config_dir=PATHS.config_dir,
        logs_dir=PATHS.logs_dir,
        temp_dir=PATHS.temp_dir,
        exports_dir=PATHS.exports_dir,
        calibration_path=PATHS.calibration_path,
        mapping_path=PATHS.mapping_path,
        gesture_data_dir=PATHS.gesture_data_dir,
        servo_debug_log_path=PATHS.servo_debug_log_path,
        audio_recording_path=PATHS.audio_recording_path,
        calibration=calibration,
        startup_warnings=startup_warnings,
    )
