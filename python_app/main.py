from __future__ import annotations

import tkinter as tk

from config import load_app_config
from fusion_state import FusionState
from gui import AirTrixxGUI
from mediapipe_tracker import HandTracker
from serial_bridge import SerialBridge
from servo_controller import ServoController


def main() -> None:
    config = load_app_config()
    serial_bridge = SerialBridge(baud_rate=config.serial_baud)
    hand_tracker = HandTracker(
        camera_index=config.camera_index,
        width=config.camera_width,
        height=config.camera_height,
    )
    servo_controller = ServoController(
        serial_bridge,
        config.calibration,
        servo_min_tick=config.servo_min_tick,
        servo_max_tick=config.servo_max_tick,
        camera_width=config.camera_width,
        camera_height=config.camera_height,
        horizontal_fov_deg=config.horizontal_fov_deg,
        vertical_fov_deg=config.vertical_fov_deg,
    )
    fusion_state = FusionState()

    root = tk.Tk()
    app = AirTrixxGUI(root, config, serial_bridge, hand_tracker, servo_controller, fusion_state)
    hand_tracker.start()
    root.mainloop()
    del app


if __name__ == "__main__":
    main()
