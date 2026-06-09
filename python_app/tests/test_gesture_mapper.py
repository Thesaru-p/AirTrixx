from __future__ import annotations

import json
import sys
import tempfile
import unittest
from pathlib import Path


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from gesture_mapper import (
    RepSeries,
    analyze,
    candidate_to_rule,
    list_gesture_folders,
    load_gesture_dir,
    load_rep,
)


def make_rep(
    *,
    gesture_name: str = "gesture",
    repetition_index: int = 1,
    frames: list[dict[str, object]],
    duration_s: float = 2.0,
) -> RepSeries:
    return RepSeries(
        path=None,
        gesture_name=gesture_name,
        repetition_index=repetition_index,
        field_order=list(frames[0].keys()) if frames else [],
        frames=frames,
        duration_s=duration_s,
    )


def ramp_frames(
    key: str,
    *,
    start: float,
    end: float,
    count: int = 50,
    duration_s: float = 2.0,
) -> list[dict[str, object]]:
    frames: list[dict[str, object]] = []
    for index in range(count):
        fraction = index / max(1, count - 1)
        value = start + (end - start) * fraction
        frames.append({"wrist_pitch": 0.0, "wrist_roll": 0.0, key: value})
    return frames


class GestureMapperTests(unittest.TestCase):
    def test_delta_increase_ranks_first(self) -> None:
        baseline = make_rep(
            gesture_name="baseline",
            frames=[{"wrist_pitch": 1.0 + (index % 3) * 0.2, "wrist_roll": 0.0} for index in range(40)],
        )
        target = make_rep(
            gesture_name="wrist_tilt_up",
            frames=ramp_frames("wrist_pitch", start=1.0, end=45.0),
        )
        result = analyze([baseline], [target])
        self.assertTrue(result.candidates)
        top = result.candidates[0]
        self.assertEqual(top.source, "fused.wrist_pitch")
        self.assertEqual(top.comparator, "delta_increase")
        self.assertGreater(float(top.threshold), 0.0)

    def test_delta_decrease(self) -> None:
        baseline = make_rep(
            gesture_name="baseline",
            frames=[{"wrist_roll": 0.0} for _ in range(40)],
        )
        target = make_rep(
            gesture_name="wrist_rotate_right",
            frames=ramp_frames("wrist_roll", start=0.0, end=-80.0),
        )
        result = analyze([baseline], [target])
        decrease = [item for item in result.candidates if item.comparator == "delta_decrease"]
        self.assertTrue(decrease)
        self.assertEqual(decrease[0].field_key, "wrist_roll")

    def test_flat_signal_excluded(self) -> None:
        baseline = make_rep(
            gesture_name="baseline",
            frames=[{"wrist_pitch": 2.0, "wrist_roll": 1.0} for _ in range(40)],
        )
        roll_ramp = [
            {"wrist_pitch": 2.0, "wrist_roll": 1.0 + (40.0 - 1.0) * (index / 49)}
            for index in range(50)
        ]
        target = make_rep(gesture_name="flat", frames=roll_ramp)
        result = analyze([baseline], [target])
        pitch_candidates = [item for item in result.candidates if item.field_key == "wrist_pitch"]
        self.assertEqual(pitch_candidates, [])

    def test_boolean_detected(self) -> None:
        baseline = make_rep(
            gesture_name="baseline",
            frames=[{"wrist_roll_right_detected": False} for _ in range(40)],
        )
        target = make_rep(
            gesture_name="wrist_roll_right",
            frames=[{"wrist_roll_right_detected": index > 10} for index in range(50)],
        )
        result = analyze([baseline], [target])
        boolean = [item for item in result.candidates if item.signal_kind == "boolean"]
        self.assertTrue(boolean)
        self.assertEqual(boolean[0].comparator, "truthy")

    def test_baseline_drift_tolerated(self) -> None:
        baseline_a = make_rep(
            gesture_name="baseline",
            frames=[{"wrist_pitch": 5.0 + (index % 2) * 0.1} for index in range(20)],
        )
        baseline_b = make_rep(
            gesture_name="baseline",
            repetition_index=2,
            frames=[{"wrist_pitch": 8.0 + (index % 2) * 0.1} for index in range(20)],
        )
        target = make_rep(
            gesture_name="wrist_tilt_up",
            frames=ramp_frames("wrist_pitch", start=7.0, end=50.0),
        )
        result = analyze([baseline_a, baseline_b], [target])
        flat = [item for item in result.candidates if item.field_key == "wrist_roll"]
        self.assertEqual(flat, [])
        self.assertTrue(any(item.field_key == "wrist_pitch" for item in result.candidates))

    def test_active_region_trimming(self) -> None:
        frames: list[dict[str, object]] = []
        for index in range(50):
            value = 80.0 if index < 5 else 1.0
            frames.append({"wrist_pitch": value})
        baseline = make_rep(
            gesture_name="baseline",
            frames=[{"wrist_pitch": 1.0} for _ in range(40)],
        )
        target = make_rep(gesture_name="spike_start", frames=frames)
        result = analyze([baseline], [target])
        pitch = [item for item in result.candidates if item.field_key == "wrist_pitch"]
        self.assertEqual(pitch, [])

    def test_candidate_to_rule(self) -> None:
        from gesture_mapper import TriggerCandidate

        candidate = TriggerCandidate(
            source="fused.wrist_pitch",
            field_key="wrist_pitch",
            comparator="delta_increase",
            threshold=12.5,
            confidence=0.8,
            signal_kind="anchor_delta",
            direction="increase",
        )
        rule = candidate_to_rule(candidate, gesture_name="wrist_tilt_up")
        self.assertEqual(rule.source, "fused.wrist_pitch")
        self.assertEqual(rule.comparator, "delta_increase")
        self.assertEqual(rule.threshold, 12.5)
        self.assertEqual(rule.action.type, "keyboard_tap")

    def test_warnings_empty_folder(self) -> None:
        result = analyze([], [])
        self.assertTrue(result.warnings)

    def test_opposite_direction_not_mixed(self) -> None:
        baseline = make_rep(
            gesture_name="baseline",
            frames=[{"wrist_pitch": 1.0 + (index % 3) * 0.2, "wrist_roll": 0.0} for index in range(40)],
        )
        target = make_rep(
            gesture_name="wrist_tilt_up",
            frames=ramp_frames("wrist_pitch", start=1.0, end=45.0),
        )
        result = analyze([baseline], [target])
        pitch_candidates = [item for item in result.candidates if item.field_key == "wrist_pitch"]
        self.assertTrue(pitch_candidates)
        increase = [item for item in pitch_candidates if item.comparator == "delta_increase"]
        decrease = [item for item in pitch_candidates if item.comparator == "delta_decrease"]
        self.assertTrue(increase)
        if decrease:
            self.assertGreater(increase[0].confidence, decrease[0].confidence)
        else:
            self.assertEqual(decrease, [])
        self.assertIn("pitch2-pitch1", increase[0].rationale)

    def test_signed_window_delta_field(self) -> None:
        baseline = make_rep(
            gesture_name="baseline",
            frames=[{"wrist_pitch_delta": (index % 3) * 0.1} for index in range(40)],
        )
        target = make_rep(
            gesture_name="wrist_tilt_up",
            frames=[{"wrist_pitch_delta": 2.0 + index * 0.8} for index in range(50)],
        )
        result = analyze([baseline], [target])
        delta_candidates = [item for item in result.candidates if item.field_key == "wrist_pitch_delta"]
        parent_candidates = [item for item in result.candidates if item.field_key == "wrist_pitch"]
        chosen = delta_candidates or parent_candidates
        self.assertTrue(chosen)
        top = chosen[0]
        self.assertIn(top.comparator, {"delta_increase", "gt"})
        self.assertGreater(float(top.threshold), 0.0)

    def test_list_and_load_rep_from_json(self) -> None:
        sample_path = (
            Path(__file__).resolve().parents[1]
            / "data"
            / "gestures"
            / "wrist_rotate_left_5"
            / "20260529_164115_rep_1.json"
        )
        if not sample_path.exists():
            self.skipTest("bundled gesture sample not available")
        rep = load_rep(sample_path)
        self.assertEqual(rep.gesture_name, "wrist_rotate_left_5")
        self.assertGreater(len(rep.frames), 0)

        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            gesture_dir = root / "baseline"
            gesture_dir.mkdir()
            payload = {
                "gesture_name": "baseline",
                "repetition_index": 1,
                "field_order": ["wrist_pitch"],
                "samples": [{"t_rel": 0.0, "input_dict": {"wrist_pitch": 1.0}}],
            }
            (gesture_dir / "20260609_120000_rep_1.json").write_text(
                json.dumps(payload),
                encoding="utf-8",
            )
            folders = list_gesture_folders(root)
            self.assertEqual(folders, ["baseline"])
            reps = load_gesture_dir(gesture_dir)
            self.assertEqual(len(reps), 1)
            self.assertEqual(reps[0].gesture_name, "baseline")


if __name__ == "__main__":
    unittest.main()
