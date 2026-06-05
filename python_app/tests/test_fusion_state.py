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

    def test_charging_dock_input_comes_from_serial_state(self) -> None:
        fusion = FusionState()
        serial_state = serial_with_wrist(pitch=0, roll=0)
        serial_state["devices"]["charging_dock"] = {"input": "charging"}
        values = fusion.build_input_dict(serial_state, {}, now_s=0.0)

        self.assertEqual(values["charging_dock_input"], "charging")

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

    def test_sampled_wrist_roll_left_maps_to_roll_left(self) -> None:
        fusion = FusionState()
        samples = [
            (0.0, -14.52, -173.91),
            (0.0404512882232666, -19.68, -175.36),
            (0.08152532577514648, -32.17, -175.90),
            (0.12275123596191406, -38.05, -177.75),
            (0.16492772102355957, -38.05, -177.75),
            (0.2087712287902832, -58.88, 177.99),
            (0.2494792938232422, -70.44, 164.44),
            (0.29001688957214355, -75.74, 119.26),
            (0.33057165145874023, -73.82, 102.58),
            (0.3726460933685303, -49.83, 53.90),
            (0.4133119583129883, -48.62, 51.01),
            (0.4543015956878662, -9.61, 36.77),
            (0.49708080291748047, -1.10, 35.30),
            (0.5372476577758789, -2.94, 21.64),
            (0.5816268920898438, 5.13, 20.45),
            (0.6292707920074463, 9.77, 11.65),
            (0.6744534969329834, 11.76, 9.70),
            (0.7211785316467285, 17.45, 5.57),
            (0.7612576484680176, 15.27, 4.88),
            (0.8026273250579834, 15.27, 4.88),
            (0.8443906307220459, 18.35, 4.11),
            (0.8864610195159912, 16.76, 5.28),
            (0.9274899959564209, 18.08, 5.47),
            (0.967597484588623, 18.08, 5.47),
            (1.008460521697998, 17.04, 4.34),
            (1.0507111549377441, 18.18, 3.08),
            (1.0936133861541748, 19.10, 3.12),
            (1.1345043182373047, 17.17, 2.43),
            (1.175478219985962, 17.20, 0.54),
            (1.2165377140045166, 15.09, 0.25),
            (1.2577900886535645, 16.55, 0.29),
            (1.2984790802001953, 16.34, -0.09),
            (1.3393166065216064, 15.26, -0.26),
            (1.3794820308685303, 14.82, -1.72),
            (1.4205117225646973, 13.26, -2.26),
            (1.4661006927490234, 12.42, -1.78),
            (1.5122160911560059, 11.63, -2.14),
            (1.5532422065734863, 11.84, -2.51),
            (1.5951666831970215, 10.56, -2.11),
            (1.6362996101379395, 9.20, -3.15),
            (1.677250862121582, 8.51, -3.36),
            (1.7174561023712158, 6.64, -3.06),
            (1.7596232891082764, 6.64, -2.80),
            (1.8007242679595947, 3.94, -3.47),
            (1.8430712223052979, 3.40, -3.14),
            (1.8873775005340576, 1.54, -3.75),
            (1.9278159141540527, 1.35, -3.47),
            (1.9679665565490723, -1.09, -3.16),
        ]
        seen = False
        motion_seen = False
        for sample_s, pitch, roll in samples:
            values = fusion.build_input_dict(serial_with_wrist(pitch=pitch, roll=roll), {}, now_s=sample_s)
            seen = seen or values["wrist_roll_left_detected"]
            motion_seen = motion_seen or values["wrist_motion"] == "roll_left"
            self.assertFalse(values["wrist_roll_right_detected"])
            self.assertFalse(values["wrist_roll_right_then_neutral_detected"])

        self.assertTrue(seen)
        self.assertTrue(motion_seen)

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
