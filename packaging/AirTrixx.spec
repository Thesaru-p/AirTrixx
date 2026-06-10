# -*- mode: python ; coding: utf-8 -*-

from __future__ import annotations

import sys
from pathlib import Path

from PyInstaller.utils.hooks import collect_data_files, collect_dynamic_libs, collect_submodules


ROOT = Path(SPECPATH).resolve()
PROJECT_ROOT = ROOT.parent
PYTHON_APP = PROJECT_ROOT / "python_app"
ASSET_DIR = ROOT / "assets" / "generated"
ICON_ICO = ASSET_DIR / "AirTrixx.ico"
ICON_ICNS = ASSET_DIR / "AirTrixx.icns"
HAND_LANDMARKER_MODEL = ASSET_DIR / "hand_landmarker.task"


def _collect_data(package: str):
    try:
        return collect_data_files(package)
    except Exception:
        return []


def _collect_binaries(package: str):
    try:
        return collect_dynamic_libs(package)
    except Exception:
        return []


def _collect_submodules(package: str):
    try:
        return collect_submodules(package)
    except Exception:
        return []


datas = []
binaries = []
hiddenimports = [
    "PIL._tkinter_finder",
    "serial.tools.list_ports",
    "mediapipe.tasks.python.core.base_options",
    "mediapipe.tasks.python.vision",
    "mediapipe.tasks.python.vision.hand_landmarker",
]

datas += _collect_data("mediapipe")
binaries += _collect_binaries("mediapipe")
hiddenimports += _collect_submodules("mediapipe.tasks.python.core")
hiddenimports += _collect_submodules("mediapipe.tasks.python.components")
hiddenimports += _collect_submodules("mediapipe.tasks.python.vision")
datas += _collect_data("cv2")
binaries += _collect_binaries("cv2")
hiddenimports += _collect_submodules("pynput")
hiddenimports += _collect_submodules("pynput.keyboard")
hiddenimports += _collect_submodules("pynput.mouse")

for source, target in (
    (PROJECT_ROOT / "docs", "docs"),
    (PROJECT_ROOT / "firmware", "firmware"),
    (PYTHON_APP / "data" / "keyboard", "python_app/data/keyboard"),
):
    if source.exists():
        datas.append((str(source), target))

if HAND_LANDMARKER_MODEL.exists():
    datas.append((str(HAND_LANDMARKER_MODEL), "models"))

icon_file = None
if sys.platform == "darwin" and ICON_ICNS.exists():
    icon_file = str(ICON_ICNS)
elif sys.platform.startswith("win") and ICON_ICO.exists():
    icon_file = str(ICON_ICO)

a = Analysis(
    [str(PYTHON_APP / "main.py")],
    pathex=[str(PYTHON_APP)],
    binaries=binaries,
    datas=datas,
    hiddenimports=hiddenimports,
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=[],
    noarchive=False,
)
pyz = PYZ(a.pure)

exe = EXE(
    pyz,
    a.scripts,
    [],
    exclude_binaries=True,
    name="AirTrixx",
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=False,
    console=False,
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
    icon=icon_file,
)
coll = COLLECT(
    exe,
    a.binaries,
    a.zipfiles,
    a.datas,
    strip=False,
    upx=False,
    upx_exclude=[],
    name="AirTrixx",
)

if sys.platform == "darwin":
    app = BUNDLE(
        coll,
        name="AirTrixx.app",
        icon=str(ICON_ICNS) if ICON_ICNS.exists() else None,
        bundle_identifier="com.knurdz.airtrixx",
        info_plist={
            "CFBundleName": "AirTrixx",
            "CFBundleDisplayName": "AirTrixx",
            "CFBundleShortVersionString": "0.1.0",
            "CFBundleVersion": "0.1.0",
            "NSCameraUsageDescription": "AirTrixx uses the camera for hand and face tracking.",
            "NSMicrophoneUsageDescription": "AirTrixx may receive microphone recordings from the Audio Dock for transcription.",
        },
    )
