from __future__ import annotations

"""
Standalone camera-centering debugger for AirTrixx.

What this script does:
1. Opens the USB camera.
2. Detects the user's face with OpenCV's Haar cascade detector.
3. Tries to move the camera pan/tilt bracket until the face is near the
   top-center of the camera image.
4. Sends the same JSON servo command that main.py sends to the Antenna.

Run from the repo root:
    python python_app/camCenter.py

Useful keys while the preview window is focused:
    q / Esc  quit
    c        toggle automatic centering
    m        toggle mirrored preview
    arrows   manually jog camera pan/tilt
    [ / ]    decrease/increase manual jog step
    r        reset pan/tilt to configured center

If no serial port is set below, the script prints available ports and tries the
first one. For reliable debugging, set SERIAL_PORT to your Antenna COM port.
"""

import json
import time
from pathlib import Path
from typing import Any

import cv2

try:
    import serial
    from serial.tools import list_ports
except Exception as exc:  # pragma: no cover - runtime dependency check
    raise SystemExit("pyserial is missing. Run: pip install -r python_app/requirements.txt") from exc


# ---------------------------------------------------------------------------
# User-editable settings
# ---------------------------------------------------------------------------

# Set this to your Antenna COM port for deterministic behavior, for example:
# SERIAL_PORT = "COM7"
SERIAL_PORT: COM6 | None = None

SERIAL_BAUD = 921600

# In this project, camera index 1 is the USB camera and index 0 is built-in.
CAMERA_INDEX = 1

# Preview mirroring is visual only. The face detector and servo math use the
# real unmirrored camera frame, so the control direction stays predictable.
MIRROR_PREVIEW = True

# Servo center values. These are loaded from config/calibration.json if present.
DEFAULT_CENTER_TICKS = 307

# Where we want the user's head/face to appear in the camera frame.
# X=0.50 means horizontally centered.
# TOP_Y=0.12 means the top of the detected face is near the top of the frame.
TARGET_FACE_X = 0.50
TARGET_FACE_TOP_Y = 0.12

# Error smaller than this is considered centered.
DEADBAND_X = 0.045
DEADBAND_Y = 0.045

# How strongly image error changes servo ticks. Increase if it moves too
# slowly; decrease if it overshoots or oscillates.
PAN_GAIN_TICKS = 42.0
TILT_GAIN_TICKS = 32.0

# Clamp each automatic update so one noisy face detection cannot make a large
# jump.
MAX_AUTO_STEP_TICKS = 10

# Minimum delay between automatic servo commands.
COMMAND_INTERVAL_S = 0.12

# Manual keyboard jog step.
MANUAL_STEP_TICKS = 5

SERVO_MIN_TICK = 0
SERVO_MAX_TICK = 4095


APP_DIR = Path(__file__).resolve().parent
CALIBRATION_PATH = APP_DIR / "config" / "calibration.json"


def load_centers() -> tuple[int, int]:
    """Load camera center pan/tilt from the same calibration file as main.py."""
    try:
        data = json.loads(CALIBRATION_PATH.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        data = {}

    pan = int(data.get("cam_pan_center", DEFAULT_CENTER_TICKS))
    tilt = int(data.get("cam_tilt_center", DEFAULT_CENTER_TICKS))
    return clamp_tick(pan), clamp_tick(tilt)


def clamp_tick(value: int | float) -> int:
    """Keep servo ticks inside the legal PCA9685 pulse range."""
    return max(SERVO_MIN_TICK, min(SERVO_MAX_TICK, int(round(value))))


def bounded_step(value: float) -> int:
    """Convert a float correction into a small nonzero integer servo step."""
    if abs(value) < 0.001:
        return 0
    step = int(round(value))
    if step == 0:
        step = 1 if value > 0 else -1
    return max(-MAX_AUTO_STEP_TICKS, min(MAX_AUTO_STEP_TICKS, step))


def serial_port_score(port: Any) -> tuple[int, str]:
    """Prefer the ESP32 USB serial port over Windows Bluetooth COM ports.

    On this prototype laptop, Windows reports several "Standard Serial over
    Bluetooth" ports before the real USB ESP32 port. If we open one of those
    Bluetooth ports, the script runs but the servo never moves because the
    Antenna never receives the JSON command.
    """
    text = f"{port.description or ''} {port.hwid or ''}".lower()
    if "vid:pid=303a" in text:
        return (0, port.device)
    if "usb" in text and "bluetooth" not in text:
        return (1, port.device)
    if "bluetooth" in text or "bthenum" in text:
        return (9, port.device)
    return (5, port.device)


def list_serial_ports() -> list[Any]:
    ports = sorted(list_ports.comports(), key=serial_port_score)
    if ports:
        print("Available serial ports, preferred first:")
        for port in ports:
            print(f"  {port.device} | {port.description} | {port.hwid}")
    else:
        print("No serial ports found.")
    return ports


def open_serial() -> serial.Serial | None:
    """Open the Antenna serial port.

    The Antenna expects newline-delimited JSON commands. If serial is not
    connected, the preview still works and commands are printed as skipped.
    """
    port = SERIAL_PORT
    if port is None:
        ports = list_serial_ports()
        port = ports[0].device if ports else None
    if port is None:
        return None

    try:
        ser = serial.Serial(port=port, baudrate=SERIAL_BAUD, timeout=0.05, write_timeout=0.1)
    except Exception as exc:
        print(f"Could not open {port}: {exc}")
        return None

    print(f"Opened {port} at {SERIAL_BAUD} baud.")
    return ser


def send_camera_servo_command(ser: serial.Serial | None, pan_tick: int, tilt_tick: int) -> bool:
    """Send a camera-bracket pan/tilt command through the Antenna.

    The Antenna firmware parses these JSON fields and forwards a binary
    ServoCommandPacket to the Cam Dock over ESP-NOW.
    """
    command = {
        "cmd": "servo",
        "target": "camdock",
        "active_pair": "camera",
        "disable_unused": True,
        "servos": {
            "r_pan": 0,
            "r_tilt": 0,
            "l_pan": 0,
            "l_tilt": 0,
            "cam_pan": clamp_tick(pan_tick),
            "cam_tilt": clamp_tick(tilt_tick),
        },
    }

    if ser is None or not ser.is_open:
        return False

    line = json.dumps(command, separators=(",", ":")) + "\n"
    try:
        ser.write(line.encode("utf-8"))
        return True
    except Exception as exc:
        print(f"Serial write failed: {exc}")
        return False


def load_face_detector() -> cv2.CascadeClassifier:
    """Load OpenCV's built-in frontal-face detector."""
    cascade_path = cv2.data.haarcascades + "haarcascade_frontalface_default.xml"
    detector = cv2.CascadeClassifier(cascade_path)
    if detector.empty():
        raise SystemExit(f"Could not load face detector: {cascade_path}")
    return detector


def detect_largest_face(
    detector: cv2.CascadeClassifier,
    frame_bgr: Any,
) -> dict[str, float | bool]:
    """Return normalized face coordinates for the largest detected face."""
    gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
    faces = detector.detectMultiScale(
        gray,
        scaleFactor=1.1,
        minNeighbors=5,
        minSize=(60, 60),
    )
    if len(faces) == 0:
        return {"visible": False}

    x, y, w, h = max(faces, key=lambda face: face[2] * face[3])
    frame_h, frame_w = frame_bgr.shape[:2]
    return {
        "visible": True,
        "x": (x + w / 2) / frame_w,
        "y": (y + h / 2) / frame_h,
        "top_y": y / frame_h,
        "width": w / frame_w,
        "height": h / frame_h,
        "box_x": float(x),
        "box_y": float(y),
        "box_w": float(w),
        "box_h": float(h),
    }


def draw_debug_overlay(
    frame_bgr: Any,
    face: dict[str, float | bool],
    pan_tick: int,
    tilt_tick: int,
    auto_enabled: bool,
    manual_step: int,
    serial_connected: bool,
) -> None:
    """Draw target guides and current state on the preview frame."""
    frame_h, frame_w = frame_bgr.shape[:2]
    target_x_px = int(TARGET_FACE_X * frame_w)
    target_top_y_px = int(TARGET_FACE_TOP_Y * frame_h)

    cv2.line(frame_bgr, (target_x_px, 0), (target_x_px, frame_h), (255, 180, 0), 1)
    cv2.line(frame_bgr, (0, target_top_y_px), (frame_w, target_top_y_px), (255, 180, 0), 1)

    if face.get("visible"):
        x = int(face["box_x"])
        y = int(face["box_y"])
        w = int(face["box_w"])
        h = int(face["box_h"])
        cv2.rectangle(frame_bgr, (x, y), (x + w, y + h), (60, 220, 120), 2)
        cv2.circle(frame_bgr, (int(face["x"] * frame_w), int(face["top_y"] * frame_h)), 5, (60, 220, 120), -1)

    status = "AUTO" if auto_enabled else "MANUAL"
    serial_status = "serial ok" if serial_connected else "serial off"
    lines = [
        f"{status} | {serial_status}",
        f"pan={pan_tick} tilt={tilt_tick} step={manual_step}",
        "target: face top at upper center",
        "q/Esc quit | c auto | m mirror | arrows jog | r reset",
    ]
    for index, text in enumerate(lines):
        y = 24 + index * 24
        cv2.putText(frame_bgr, text, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(frame_bgr, text, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255, 255), 1, cv2.LINE_AA)


def main() -> None:
    pan_tick, tilt_tick = load_centers()
    center_pan, center_tilt = pan_tick, tilt_tick
    manual_step = MANUAL_STEP_TICKS
    auto_enabled = True
    mirror_preview = MIRROR_PREVIEW
    last_command_s = 0.0

    ser = open_serial()
    detector = load_face_detector()

    cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_DSHOW)
    if not cap.isOpened():
        cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        raise SystemExit(f"Could not open camera index {CAMERA_INDEX}.")

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    print("Camera centering debugger started.")
    print("Move the camera bracket until your head is near the top-center guide.")

    while True:
        ok, frame_bgr = cap.read()
        if not ok:
            time.sleep(0.03)
            continue

        face = detect_largest_face(detector, frame_bgr)
        now = time.monotonic()

        if auto_enabled and face.get("visible") and now - last_command_s >= COMMAND_INTERVAL_S:
            # Positive error_x means the face is too far right in the image.
            # This hardware needs camera pan inverted, so a right-side face
            # reduces the pan tick instead of increasing it.
            error_x = float(face["x"]) - TARGET_FACE_X

            # Positive error_y means the face top is too low in the image.
            # Adjust tilt by a small amount. If it moves away, invert tilt_delta.
            error_y = float(face["top_y"]) - TARGET_FACE_TOP_Y

            if abs(error_x) > DEADBAND_X:
                pan_tick = clamp_tick(pan_tick + bounded_step(-error_x * PAN_GAIN_TICKS))
            if abs(error_y) > DEADBAND_Y:
                tilt_tick = clamp_tick(tilt_tick + bounded_step(error_y * TILT_GAIN_TICKS))

            send_camera_servo_command(ser, pan_tick, tilt_tick)
            last_command_s = now

        draw_debug_overlay(
            frame_bgr,
            face,
            pan_tick,
            tilt_tick,
            auto_enabled,
            manual_step,
            bool(ser and ser.is_open),
        )

        preview = cv2.flip(frame_bgr, 1) if mirror_preview else frame_bgr
        cv2.imshow("AirTrixx Camera Center Debug", preview)
        key = cv2.waitKey(1) & 0xFF

        if key in (27, ord("q")):
            break
        if key == ord("c"):
            auto_enabled = not auto_enabled
            print(f"Automatic centering {'enabled' if auto_enabled else 'disabled'}.")
        elif key == ord("m"):
            mirror_preview = not mirror_preview
            print(f"Mirror preview {'enabled' if mirror_preview else 'disabled'}.")
        elif key == ord("r"):
            pan_tick, tilt_tick = center_pan, center_tilt
            send_camera_servo_command(ser, pan_tick, tilt_tick)
            print(f"Reset camera bracket to center pan={pan_tick}, tilt={tilt_tick}.")
        elif key == ord("["):
            manual_step = max(1, manual_step - 1)
        elif key == ord("]"):
            manual_step = min(100, manual_step + 1)

        # OpenCV arrow key codes vary by backend. These values work on the
        # Windows HighGUI backend used by most OpenCV installs.
        elif key == 81:  # Left arrow
            pan_tick = clamp_tick(pan_tick + manual_step)
            send_camera_servo_command(ser, pan_tick, tilt_tick)
        elif key == 83:  # Right arrow
            pan_tick = clamp_tick(pan_tick - manual_step)
            send_camera_servo_command(ser, pan_tick, tilt_tick)
        elif key == 82:  # Up arrow
            tilt_tick = clamp_tick(tilt_tick + manual_step)
            send_camera_servo_command(ser, pan_tick, tilt_tick)
        elif key == 84:  # Down arrow
            tilt_tick = clamp_tick(tilt_tick - manual_step)
            send_camera_servo_command(ser, pan_tick, tilt_tick)

    cap.release()
    if ser and ser.is_open:
        ser.close()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
