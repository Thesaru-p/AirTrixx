from __future__ import annotations

import sys
import unittest
from pathlib import Path


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from config import DEFAULT_CALIBRATION
from servo_controller import ServoController


class FakeSerialBridge:
    def __init__(self) -> None:
        self.commands: list[dict] = []

    def send_command(self, command: dict) -> bool:
        self.commands.append(command)
        return True


class ServoControllerGeometryTests(unittest.TestCase):
    def make_controller(self, **overrides) -> ServoController:
        calibration = dict(DEFAULT_CALIBRATION)
        calibration.update(
            {
                "hand_servo_smoothing_alpha": 1.0,
                "hand_servo_deadband": 0.0,
                "r_pan_center": 307,
                "r_tilt_center": 307,
                "l_pan_center": 307,
                "l_tilt_center": 307,
                "cam_pan_center": 307,
                "cam_tilt_center": 307,
                "pan_ticks_per_degree": 2.25,
                "tilt_ticks_per_degree": 2.25,
                "r_pan_angle_offset_deg": 0.0,
                "r_tilt_angle_offset_deg": 0.0,
                "l_pan_angle_offset_deg": 0.0,
                "l_tilt_angle_offset_deg": 0.0,
                "use_startup_user_distance": 1,
                "startup_distance_live_weight": 0.35,
                "session_calibration": {},
            }
        )
        calibration.update(overrides)
        return ServoController(FakeSerialBridge(), calibration)

    def test_camera_pose_uses_last_commanded_camera_ticks(self) -> None:
        controller = self.make_controller()
        controller.send_camera_with_parallel_tof(284, 296)

        yaw_deg, pitch_deg = controller.current_camera_pose_degrees()

        self.assertAlmostEqual(yaw_deg, (284 - 307) / (-1.0 * 2.25), places=3)
        self.assertAlmostEqual(pitch_deg, (296 - 307) / (-1.0 * 2.25), places=3)

    def test_camera_pose_compensates_center_ray(self) -> None:
        plain = self.make_controller()
        compensated = self.make_controller()
        compensated.send_camera_with_parallel_tof(284, 307)

        _pan, _tilt, plain_details = plain._geometry_ticks_for_hand("right", 0.5, 0.5, 700.0)
        _pan, _tilt, compensated_details = compensated._geometry_ticks_for_hand("right", 0.5, 0.5, 700.0)

        self.assertGreater(compensated_details["ray"][0], plain_details["ray"][0])
        self.assertGreater(compensated_details["yaw_deg"], plain_details["yaw_deg"])

    def test_session_calibration_anchors_neutral_pose_to_center_ticks(self) -> None:
        controller = self.make_controller()
        hands = {"visible": True, "x": 0.62, "y": 0.42, "score": 0.9}
        entry = controller.build_session_calibration_entry("right", hands, 700.0, "tof")
        calibration = dict(controller.calibration)
        calibration["session_calibration"] = {"right": entry}
        controller.update_calibration(calibration)

        pan_tick, tilt_tick, details = controller._geometry_ticks_for_hand("right", 0.62, 0.42, 700.0)

        self.assertEqual(pan_tick, 307)
        self.assertEqual(tilt_tick, 307)
        self.assertAlmostEqual(details["pan_angle_deg"], 0.0, places=3)
        self.assertAlmostEqual(details["tilt_angle_deg"], 0.0, places=3)

    def test_startup_user_distance_is_used_without_live_tof(self) -> None:
        controller = self.make_controller(
            session_calibration={
                "user_distance_mm": 500.0,
                "right": {"tof_mm": 500.0},
                "left": {"tof_mm": 505.0},
            }
        )

        distance_mm = controller._distance_mm_for_side("right", serial_state={})

        self.assertEqual(distance_mm, 500.0)
        self.assertEqual(controller._last_distance_debug["right"]["source"], "startup_user_distance")

    def test_live_tof_is_blended_around_startup_user_distance(self) -> None:
        controller = self.make_controller(
            startup_distance_live_weight=0.25,
            session_calibration={
                "user_distance_mm": 500.0,
                "right": {"tof_mm": 500.0},
                "left": {"tof_mm": 500.0},
            },
        )
        serial_state = {
            "devices": {
                "camdock": {
                    "tof": {
                        "right_mm": 800.0,
                    }
                }
            }
        }

        distance_mm = controller._distance_mm_for_side("right", serial_state)

        self.assertEqual(distance_mm, 575.0)
        self.assertEqual(controller._last_distance_debug["right"]["source"], "startup_user_distance_blend")
        self.assertEqual(controller._last_distance_debug["right"]["startup_user_distance_mm"], 500.0)


if __name__ == "__main__":
    unittest.main()
