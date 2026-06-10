from __future__ import annotations

import os
import platform
import shutil
import sys
from dataclasses import dataclass
from pathlib import Path


APP_NAME = "AirTrixx"
SOURCE_APP_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = SOURCE_APP_DIR.parent


@dataclass(frozen=True)
class AppPaths:
    user_data_dir: Path
    config_dir: Path
    logs_dir: Path
    temp_dir: Path
    exports_dir: Path
    gesture_data_dir: Path
    keyboard_data_dir: Path
    audio_training_dir: Path
    calibration_path: Path
    mapping_path: Path
    servo_debug_log_path: Path
    audio_recording_path: Path
    keyboard_dataset_path: Path
    keyboard_model_path: Path
    keyboard_words_path: Path


def is_frozen() -> bool:
    return bool(getattr(sys, "frozen", False))


def resource_path(*parts: str) -> Path:
    base = Path(getattr(sys, "_MEIPASS", SOURCE_APP_DIR))
    return base.joinpath(*parts)


def project_resource_path(*parts: str) -> Path:
    if is_frozen():
        return resource_path(*parts)
    return PROJECT_ROOT.joinpath(*parts)


def _home_dir() -> Path:
    return Path.home()


def user_data_root(app_name: str = APP_NAME) -> Path:
    system = platform.system()
    if system == "Windows":
        root = os.environ.get("APPDATA") or os.environ.get("LOCALAPPDATA")
        if root:
            return Path(root) / app_name
        return _home_dir() / "AppData" / "Roaming" / app_name
    if system == "Darwin":
        return _home_dir() / "Library" / "Application Support" / app_name
    root = os.environ.get("XDG_DATA_HOME")
    if root:
        return Path(root) / app_name
    return _home_dir() / ".local" / "share" / app_name


def build_app_paths(app_name: str = APP_NAME) -> AppPaths:
    user_data_dir = user_data_root(app_name)
    config_dir = user_data_dir / "config"
    logs_dir = user_data_dir / "logs"
    temp_dir = user_data_dir / "temp"
    exports_dir = user_data_dir / "exports"
    gesture_data_dir = user_data_dir / "gestures"
    keyboard_data_dir = user_data_dir / "keyboard"
    audio_training_dir = user_data_dir / "audio_training"
    return AppPaths(
        user_data_dir=user_data_dir,
        config_dir=config_dir,
        logs_dir=logs_dir,
        temp_dir=temp_dir,
        exports_dir=exports_dir,
        gesture_data_dir=gesture_data_dir,
        keyboard_data_dir=keyboard_data_dir,
        audio_training_dir=audio_training_dir,
        calibration_path=config_dir / "calibration.json",
        mapping_path=config_dir / "input_mappings.json",
        servo_debug_log_path=logs_dir / "servo_debug.log",
        audio_recording_path=temp_dir / "last_esp32_recording.wav",
        keyboard_dataset_path=keyboard_data_dir / "raw_samples.csv",
        keyboard_model_path=keyboard_data_dir / "word_knn_model.npz",
        keyboard_words_path=keyboard_data_dir / "current_training_words.txt",
    )


def ensure_app_paths(paths: AppPaths) -> None:
    for directory in (
        paths.user_data_dir,
        paths.config_dir,
        paths.logs_dir,
        paths.temp_dir,
        paths.exports_dir,
        paths.gesture_data_dir,
        paths.keyboard_data_dir,
        paths.audio_training_dir,
    ):
        directory.mkdir(parents=True, exist_ok=True)


def migrate_legacy_runtime_files(paths: AppPaths) -> list[str]:
    """Copy source-tree runtime files into the user data directory once."""
    ensure_app_paths(paths)
    migrated: list[str] = []
    legacy_config_dir = SOURCE_APP_DIR / "config"
    legacy_data_dir = SOURCE_APP_DIR / "data"
    candidates = (
        (legacy_config_dir / "calibration.json", paths.calibration_path),
        (legacy_config_dir / "input_mappings.json", paths.mapping_path),
        (legacy_data_dir / "servo_debug.log", paths.servo_debug_log_path),
        (SOURCE_APP_DIR / "last_esp32_recording.wav", paths.audio_recording_path),
    )
    for source, target in candidates:
        if source.exists() and not target.exists():
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source, target)
            migrated.append(f"{source} -> {target}")

    legacy_gestures = legacy_data_dir / "gestures"
    if legacy_gestures.exists():
        copied_count = 0
        for source_file in legacy_gestures.rglob("*"):
            if not source_file.is_file():
                continue
            target = paths.gesture_data_dir / source_file.relative_to(legacy_gestures)
            if target.exists():
                continue
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source_file, target)
            copied_count += 1
        if copied_count:
            migrated.append(f"{legacy_gestures} -> {paths.gesture_data_dir} ({copied_count} gesture samples)")
    return migrated
