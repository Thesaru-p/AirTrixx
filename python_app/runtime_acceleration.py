from __future__ import annotations

import platform
import sys
from pathlib import Path
from typing import Any


WINDOWS_GPU_PREFERENCE_KEY = r"Software\Microsoft\DirectX\UserGpuPreferences"
HIGH_PERFORMANCE_GPU_VALUE = "GpuPreference=2;"


def configure_runtime_acceleration() -> list[str]:
    messages: list[str] = []
    messages.extend(request_windows_high_performance_gpu())
    messages.extend(configure_opencv_acceleration())
    return messages


def request_windows_high_performance_gpu(
    executable_path: str | Path | None = None,
    *,
    frozen_only: bool = True,
) -> list[str]:
    if platform.system() != "Windows":
        return []
    if frozen_only and not bool(getattr(sys, "frozen", False)):
        return []

    try:
        import winreg
    except Exception as exc:  # pragma: no cover - Windows only
        return [f"Could not load the Windows registry API for GPU preference: {exc}"]

    exe_path = Path(executable_path or sys.executable).resolve()
    exe_key_name = str(exe_path)

    try:
        with winreg.CreateKeyEx(
            winreg.HKEY_CURRENT_USER,
            WINDOWS_GPU_PREFERENCE_KEY,
            0,
            winreg.KEY_READ | winreg.KEY_WRITE,
        ) as key:
            try:
                existing_value, _value_type = winreg.QueryValueEx(key, exe_key_name)
            except FileNotFoundError:
                existing_value = ""

            if "GpuPreference=2;" in str(existing_value):
                return []

            winreg.SetValueEx(key, exe_key_name, 0, winreg.REG_SZ, HIGH_PERFORMANCE_GPU_VALUE)
    except OSError as exc:
        return [f"Could not set Windows high-performance GPU preference: {exc}"]

    return [
        "Windows high-performance GPU preference was enabled for AirTrixx.exe. "
        "Restart AirTrixx once for Windows to apply it."
    ]


def configure_opencv_acceleration(cv2_module: Any | None = None) -> list[str]:
    try:
        cv2 = cv2_module
        if cv2 is None:
            import cv2 as imported_cv2

            cv2 = imported_cv2
    except Exception as exc:
        return [f"OpenCV acceleration could not be configured: {exc}"]

    messages: list[str] = []
    try:
        cv2.setUseOptimized(True)
    except Exception:
        pass

    opencl_available = False
    opencl_enabled = False
    try:
        opencl_available = bool(cv2.ocl.haveOpenCL())
        if opencl_available:
            cv2.ocl.setUseOpenCL(True)
        opencl_enabled = bool(cv2.ocl.useOpenCL())
    except Exception:
        opencl_available = False
        opencl_enabled = False

    if opencl_enabled:
        messages.append("OpenCV OpenCL acceleration is enabled.")
    elif opencl_available:
        messages.append("OpenCV OpenCL is available but could not be enabled.")
    else:
        messages.append("OpenCV OpenCL acceleration is not available on this system.")
    return messages
