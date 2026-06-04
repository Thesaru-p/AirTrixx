from __future__ import annotations

import unittest
import sys
from pathlib import Path


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from fusion_state import FusionState


def serial_with_wrist(*, pitch: float, roll: float) -> dict[str, object]:
    return {
        "devices": {
            "wristband": {
                "pitch": pitch,
                "roll": roll,
                "accel": {},
                "gyro": {},
            },
            "camdock": {"tof": {}},
            "keyboard": {"tof": {}},
            "fans": {},
        }
    }


class FusionStateTests(unittest.TestCase):
    def test_wrist_roll_dominates_pitch_when_roll_delta_is_larger(self) -> None:
        fusion = FusionState()
        fusion.build_input_dict(serial_with_wrist(pitch=0, roll=0), {}, now_s=0.0)
        values = fusion.build_input_dict(serial_with_wrist(pitch=6, roll=35), {}, now_s=0.2)

        self.assertAlmostEqual(values["wrist_roll_delta"], 35)
        self.assertAlmostEqual(values["wrist_pitch_delta"], 6)
        self.assertTrue(values["wrist_roll_dominant"])
        self.assertFalse(values["wrist_pitch_dominant"])
        self.assertEqual(values["wrist_dominant_axis"], "roll")

    def test_wrist_pitch_does_not_dominate_when_roll_is_stronger(self) -> None:
        fusion = FusionState()
        fusion.build_input_dict(serial_with_wrist(pitch=0, roll=0), {}, now_s=0.0)
        values = fusion.build_input_dict(serial_with_wrist(pitch=24, roll=35), {}, now_s=0.2)

        self.assertFalse(values["wrist_pitch_dominant"])
        self.assertFalse(values["wrist_roll_dominant"])
        self.assertEqual(values["wrist_dominant_axis"], "none")

    def test_wrist_roll_delta_handles_angle_wrap(self) -> None:
        fusion = FusionState()
        fusion.build_input_dict(serial_with_wrist(pitch=0, roll=170), {}, now_s=0.0)
        values = fusion.build_input_dict(serial_with_wrist(pitch=2, roll=-160), {}, now_s=0.2)

        self.assertAlmostEqual(values["wrist_roll_delta"], 30)
        self.assertTrue(values["wrist_roll_dominant"])

    def test_trained_roll_right_detects_held_large_roll(self) -> None:
        fusion = FusionState()
        seen = False
        for index, roll in enumerate([0, -20, -60, -95, -120, -130, -145, -155, -152, -150, -148]):
            values = fusion.build_input_dict(serial_with_wrist(pitch=5, roll=roll), {}, now_s=index * 0.2)
            if index <= 7:
                self.assertFalse(values["wrist_roll_right_detected"])
            seen = seen or values["wrist_roll_right_detected"]

        self.assertTrue(seen)
        self.assertFalse(values["wrist_roll_right_then_neutral_detected"])

    def test_trained_roll_right_then_neutral_detects_return(self) -> None:
        fusion = FusionState()
        seen = False
        for index, roll in enumerate([0, -30, -90, -150, -100, -35, -4, -3, -2]):
            values = fusion.build_input_dict(serial_with_wrist(pitch=0, roll=roll), {}, now_s=index * 0.2)
            seen = seen or values["wrist_roll_right_then_neutral_detected"]

        self.assertTrue(seen)
        self.assertFalse(values["wrist_roll_right_detected"])

    def test_trained_roll_right_then_neutral_does_not_fire_held_roll_early(self) -> None:
        fusion = FusionState()
        sequence = [0, 0, 8, 176, 175, 174, 173, 172, 100, -86, -4, -3]
        for index, roll in enumerate(sequence):
            values = fusion.build_input_dict(serial_with_wrist(pitch=0, roll=roll), {}, now_s=index * 0.2)
            if index <= 9:
                self.assertFalse(values["wrist_roll_right_detected"])

        self.assertEqual(values["wrist_motion"], "roll_right_then_neutral")
        self.assertTrue(values["wrist_roll_right_then_neutral_detected"])
        self.assertFalse(values["wrist_roll_right_detected"])

    def test_trained_roll_left_uses_sampled_pitch_pattern(self) -> None:
        fusion = FusionState()
        seen = False
        for index, pitch in enumerate([0, -10, -30, -50, -65, -70, -68, -65, -62]):
            values = fusion.build_input_dict(serial_with_wrist(pitch=pitch, roll=-5), {}, now_s=index * 0.2)
            seen = seen or values["wrist_roll_left_detected"]

        self.assertTrue(seen)
        self.assertFalse(values["wrist_roll_right_detected"])
        self.assertFalse(values["wrist_pitch_down_detected"])

    def test_trained_roll_right_waits_for_velocity_peak_to_fall(self) -> None:
        fusion = FusionState()
        for index, roll in enumerate([0, -10, -25, -48, -78, -116]):
            values = fusion.build_input_dict(serial_with_wrist(pitch=0, roll=roll), {}, now_s=index * 0.2)

        self.assertEqual(values["wrist_roll_velocity_profile"], "rising")
        self.assertEqual(values["wrist_motion"], "none")
        self.assertFalse(values["wrist_roll_right_detected"])

    def test_pitch_up_detects_sustained_pitch_without_roll(self) -> None:
        fusion = FusionState()
        seen = False
        for index, pitch in enumerate([0, 8, 24, 38, 48, 52, 49, 46, 43]):
            values = fusion.build_input_dict(serial_with_wrist(pitch=pitch, roll=2), {}, now_s=index * 0.2)
            if index <= 5:
                self.assertFalse(values["wrist_pitch_up_detected"])
            seen = seen or values["wrist_pitch_up_detected"]

        self.assertTrue(seen)
        self.assertFalse(values["wrist_pitch_down_detected"])
        self.assertEqual(values["wrist_motion"], "none")

    def test_pitch_booleans_stay_off_during_roll_right_wobble(self) -> None:
        fusion = FusionState()
        samples = [
            (0, 0),
            (-10, -2),
            (42, -4),
            (10, -70),
            (25, -120),
            (25, -126),
            (25, -128),
            (25, -129),
            (25, -129),
            (25, -130),
            (25, -130),
            (25, -130),
            (25, -128),
            (25, -126),
        ]
        seen_roll_right = False
        for index, (pitch, roll) in enumerate(samples):
            values = fusion.build_input_dict(serial_with_wrist(pitch=pitch, roll=roll), {}, now_s=index * 0.2)
            self.assertFalse(values["wrist_pitch_up_detected"])
            self.assertFalse(values["wrist_pitch_down_detected"])
            self.assertFalse(values["wrist_roll_left_detected"])
            seen_roll_right = seen_roll_right or values["wrist_roll_right_detected"]

        self.assertTrue(seen_roll_right)
        self.assertTrue(values["wrist_roll_candidate_active"])

    def test_roll_right_fires_once_after_peak_passes(self) -> None:
        fusion = FusionState()
        previous = False
        rising_edges = 0
        for index, roll in enumerate([0, -20, -60, -95, -120, -130, -145, -155, -152, -150, -148, -148, -148]):
            values = fusion.build_input_dict(serial_with_wrist(pitch=5, roll=roll), {}, now_s=index * 0.2)
            current = values["wrist_roll_right_detected"]
            if index <= 7:
                self.assertFalse(current)
            if current and not previous:
                rising_edges += 1
            previous = current

        self.assertEqual(rising_edges, 1)
        self.assertFalse(values["wrist_roll_event_blocked"])

    def test_pitch_peak_below_multiplier_does_not_fire(self) -> None:
        fusion = FusionState()
        for index, pitch in enumerate([0, 8, 24, 31, 35, 34, 33, 20]):
            values = fusion.build_input_dict(serial_with_wrist(pitch=pitch, roll=2), {}, now_s=index * 0.2)

        self.assertFalse(values["wrist_pitch_up_detected"])
        self.assertFalse(values["wrist_pitch_down_detected"])

    def test_roll_right_then_neutral_pulses_once_after_return(self) -> None:
        fusion = FusionState()
        previous = False
        rising_edges = 0
        for index, roll in enumerate([0, -30, -90, -150, -100, -35, -4, -3, -2, -1, 0, 0, 0, 0]):
            values = fusion.build_input_dict(serial_with_wrist(pitch=0, roll=roll), {}, now_s=index * 0.2)
            current = values["wrist_roll_right_then_neutral_detected"]
            if current and not previous:
                rising_edges += 1
            previous = current

        self.assertEqual(rising_edges, 1)
        self.assertFalse(values["wrist_roll_right_detected"])

    def test_roll_right_then_neutral_rearms_after_stable_neutral(self) -> None:
        fusion = FusionState()
        previous = False
        rising_edges = 0
        rolls = [0, -30, -90, -150, -100, -35, -4, -3, -2]
        rolls.extend([0] * 18)
        rolls.extend([0, -30, -90, -150, -100, -35, -4, -3, -2])
        for index, roll in enumerate(rolls):
            values = fusion.build_input_dict(serial_with_wrist(pitch=0, roll=roll), {}, now_s=index * 0.2)
            current = values["wrist_roll_right_then_neutral_detected"]
            if current and not previous:
                rising_edges += 1
            previous = current

        self.assertEqual(rising_edges, 2)


if __name__ == "__main__":
    unittest.main()
