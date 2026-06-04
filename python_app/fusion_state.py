from __future__ import annotations

import time
from typing import Any


WRIST_AXIS_WINDOW_S = 0.45
WRIST_AXIS_MIN_DELTA_DEG = 8.0
WRIST_AXIS_DOMINANCE_RATIO = 1.6
WRIST_AXIS_DOMINANCE_MARGIN_DEG = 6.0
WRIST_MOTION_WINDOW_S = 2.50
WRIST_MOTION_BASELINE_S = 0.25
WRIST_MOTION_CURRENT_S = 0.15
WRIST_TRAINED_ROLL_LARGE_DEG = 85.0
WRIST_TRAINED_ROLL_HELD_DEG = 70.0
WRIST_TRAINED_ROLL_RETURN_DEG = 28.0
WRIST_TRAINED_ROLL_CURRENT_S = 0.20
WRIST_TRAINED_ROLL_RIGHT_DECISION_S = 1.15
WRIST_TRAINED_ROLL_RETURN_SETTLE_S = 0.12
WRIST_TRAINED_ROLL_LARGE_SPAN_S = 0.08
WRIST_TRAINED_ROLL_RETURN_NEUTRAL_RATIO = 0.55
WRIST_TRAINED_ROLL_HOLD_RETENTION_RATIO = 0.75
WRIST_TRAINED_ROLL_AXIS_RATIO = 1.45
WRIST_TRAINED_LEFT_PITCH_DEG = 35.0
WRIST_TRAINED_LEFT_ROLL_LIMIT_DEG = 45.0
WRIST_TRAINED_LEFT_AXIS_RATIO = 1.35
WRIST_TRAINED_LEFT_DECISION_S = 0.35
WRIST_SAMPLED_LEFT_ROLL_DEG = 90.0
WRIST_SAMPLED_LEFT_PITCH_DEG = 55.0
WRIST_SAMPLED_LEFT_CURRENT_DEG = 65.0
WRIST_SAMPLED_LEFT_DECISION_S = 1.0
WRIST_SAMPLED_LEFT_AXIS_RATIO = 1.05
WRIST_SAMPLED_LEFT_STABLE_RATIO = 0.65
WRIST_PITCH_GESTURE_DEG = 30.0
WRIST_PITCH_GESTURE_HELD_DEG = 25.0
WRIST_PITCH_DECISION_S = 0.35
WRIST_PITCH_HOLD_RETENTION_RATIO = 0.70
WRIST_ROLL_GUARD_DEG = 45.0
WRIST_ROLL_GUARD_VELOCITY_DPS = 90.0
WRIST_VELOCITY_CURRENT_S = 0.15
WRIST_VELOCITY_MIN_PEAK_DPS = 90.0
WRIST_VELOCITY_DECAY_RATIO = 0.45
WRIST_VELOCITY_PEAK_MAX_FRACTION = 0.75
WRIST_VELOCITY_PEAK_SETTLE_S = 0.08
GESTURE_COOLDOWN_SEC = 0.6
GESTURE_SUSTAINED_MIN_MS = 120
GESTURE_PEAK_MULTIPLIER = 1.3


FIELD_ORDER = [
    "right_hand_x",
    "right_hand_y",
    "right_hand_z_mm",
    "right_hand_gesture",
    "left_hand_x",
    "left_hand_y",
    "left_hand_z_mm",
    "left_hand_gesture",
    "wrist_accel_x",
    "wrist_accel_y",
    "wrist_accel_z",
    "wrist_gyro_x",
    "wrist_gyro_y",
    "wrist_gyro_z",
    "wrist_pitch",
    "wrist_roll",
    "camdock_battery_level",
    "wristband_battery_level",
    "fans_battery_level",
    "keyboard_input",
    "keyboard_sensor_1_mm",
    "keyboard_sensor_2_mm",
    "keyboard_sensor_3_mm",
    "keyboard_sensor_4_mm",
    "charging_dock_input",
    "audiodock_input",
    "fans_input",
    "wrist_pitch_delta",
    "wrist_roll_delta",
    "wrist_pitch_abs_delta",
    "wrist_roll_abs_delta",
    "wrist_pitch_dominant",
    "wrist_roll_dominant",
    "wrist_dominant_axis",
    "wrist_motion_roll_delta",
    "wrist_motion_pitch_delta",
    "wrist_motion_roll_abs_delta",
    "wrist_motion_pitch_abs_delta",
    "wrist_motion",
    "wrist_roll_right_detected",
    "wrist_roll_left_detected",
    "wrist_roll_right_then_neutral_detected",
    "wrist_roll_velocity_dps",
    "wrist_pitch_velocity_dps",
    "wrist_roll_velocity_abs_dps",
    "wrist_pitch_velocity_abs_dps",
    "wrist_roll_velocity_peak_dps",
    "wrist_pitch_velocity_peak_dps",
    "wrist_roll_velocity_peak_ratio",
    "wrist_pitch_velocity_peak_ratio",
    "wrist_roll_velocity_peak_detected",
    "wrist_pitch_velocity_peak_detected",
    "wrist_roll_velocity_profile",
    "wrist_pitch_velocity_profile",
    "wrist_roll_candidate_active",
    "wrist_pitch_candidate_active",
    "wrist_pitch_up_detected",
    "wrist_pitch_down_detected",
    "wrist_roll_event_cooldown_active",
    "wrist_roll_event_blocked",
    "wrist_roll_event_pulse_active",
]


class FusionState:
    def __init__(self) -> None:
        self._wrist_axis_history: list[tuple[float, float, float]] = []
        self._wrist_motion_history: list[tuple[float, float, float]] = []
        self._gesture_peak_states: dict[str, dict[str, Any]] = {}

    @staticmethod
    def _camera_y_up(values: dict[str, Any]) -> float | None:
        if not values.get("visible") or values.get("y") is None:
            return None
        try:
            image_y = float(values["y"])
        except (TypeError, ValueError):
            return None
        image_y = max(0.0, min(1.0, image_y))
        return 1.0 - image_y

    @staticmethod
    def _number(value: Any) -> float | None:
        if isinstance(value, bool) or value is None:
            return None
        try:
            return float(value)
        except (TypeError, ValueError):
            return None

    @staticmethod
    def _angle_delta(current: float, anchor: float) -> float:
        return (current - anchor + 180.0) % 360.0 - 180.0

    @staticmethod
    def _unwrap_angle_samples(samples: list[tuple[float, float]]) -> list[tuple[float, float]]:
        if not samples:
            return []
        unwrapped: list[tuple[float, float]] = []
        previous = samples[0][1]
        offset = 0.0
        for index, (sample_s, value) in enumerate(samples):
            if index:
                step = value - previous
                if step > 180.0:
                    offset -= 360.0
                elif step < -180.0:
                    offset += 360.0
            unwrapped.append((sample_s, value + offset))
            previous = value
        return unwrapped

    @staticmethod
    def _median(values: list[float]) -> float | None:
        if not values:
            return None
        ordered = sorted(values)
        middle = len(ordered) // 2
        if len(ordered) % 2:
            return ordered[middle]
        return (ordered[middle - 1] + ordered[middle]) / 2.0

    @staticmethod
    def _percentile(values: list[float], fraction: float) -> float | None:
        if not values:
            return None
        ordered = sorted(values)
        if len(ordered) == 1:
            return ordered[0]
        position = (len(ordered) - 1) * max(0.0, min(1.0, fraction))
        low = int(position)
        high = min(len(ordered) - 1, low + 1)
        if low == high:
            return ordered[low]
        return ordered[low] + (ordered[high] - ordered[low]) * (position - low)

    @classmethod
    def _velocity_profile(
        cls,
        samples: list[tuple[float, float]],
        now_s: float,
    ) -> dict[str, Any]:
        velocities: list[tuple[float, float]] = []
        for previous, current in zip(samples, samples[1:]):
            dt_s = current[0] - previous[0]
            if dt_s <= 0.0 or dt_s > 0.25:
                continue
            velocities.append((current[0], cls._angle_delta(current[1], previous[1]) / dt_s))
        if not velocities:
            return {
                "velocity_dps": 0.0,
                "velocity_abs_dps": 0.0,
                "velocity_peak_dps": 0.0,
                "velocity_peak_ratio": 1.0,
                "velocity_peak_detected": False,
                "velocity_profile": "flat",
            }

        current_values = [
            velocity
            for sample_s, velocity in velocities
            if sample_s >= now_s - WRIST_VELOCITY_CURRENT_S
        ]
        if not current_values:
            current_values = [velocities[-1][1]]
        current_velocity = cls._median(current_values) or 0.0
        current_abs_velocity = abs(current_velocity)
        peak_index = max(range(len(velocities)), key=lambda index: abs(velocities[index][1]))
        peak_s, peak_velocity = velocities[peak_index]
        peak_abs_velocity = abs(peak_velocity)
        first_s = samples[0][0]
        last_s = samples[-1][0]
        duration_s = max(0.001, last_s - first_s)
        peak_fraction = (peak_s - first_s) / duration_s
        peak_ratio = current_abs_velocity / max(peak_abs_velocity, 1.0)
        peak_detected = (
            peak_abs_velocity >= WRIST_VELOCITY_MIN_PEAK_DPS
            and peak_ratio <= WRIST_VELOCITY_DECAY_RATIO
            and peak_fraction <= WRIST_VELOCITY_PEAK_MAX_FRACTION
            and now_s - peak_s >= WRIST_VELOCITY_PEAK_SETTLE_S
        )
        if peak_detected:
            profile = "peaked"
        elif peak_abs_velocity < WRIST_VELOCITY_MIN_PEAK_DPS:
            profile = "flat"
        else:
            profile = "rising"
        return {
            "velocity_dps": current_velocity,
            "velocity_abs_dps": current_abs_velocity,
            "velocity_peak_dps": peak_abs_velocity,
            "velocity_peak_ratio": peak_ratio,
            "velocity_peak_detected": peak_detected,
            "velocity_profile": profile,
        }

    @staticmethod
    def _dominates(primary_abs: float, secondary_abs: float) -> bool:
        if primary_abs < WRIST_AXIS_MIN_DELTA_DEG:
            return False
        if primary_abs - secondary_abs < WRIST_AXIS_DOMINANCE_MARGIN_DEG:
            return False
        return primary_abs >= secondary_abs * WRIST_AXIS_DOMINANCE_RATIO

    @staticmethod
    def _empty_wrist_features() -> dict[str, Any]:
        return {
            "wrist_pitch_delta": None,
            "wrist_roll_delta": None,
            "wrist_pitch_abs_delta": None,
            "wrist_roll_abs_delta": None,
            "wrist_pitch_dominant": False,
            "wrist_roll_dominant": False,
            "wrist_dominant_axis": "none",
            "wrist_motion_roll_delta": None,
            "wrist_motion_pitch_delta": None,
            "wrist_motion_roll_abs_delta": None,
            "wrist_motion_pitch_abs_delta": None,
            "wrist_motion": "none",
            "wrist_roll_right_detected": False,
            "wrist_roll_left_detected": False,
            "wrist_roll_right_then_neutral_detected": False,
            "wrist_roll_velocity_dps": 0.0,
            "wrist_pitch_velocity_dps": 0.0,
            "wrist_roll_velocity_abs_dps": 0.0,
            "wrist_pitch_velocity_abs_dps": 0.0,
            "wrist_roll_velocity_peak_dps": 0.0,
            "wrist_pitch_velocity_peak_dps": 0.0,
            "wrist_roll_velocity_peak_ratio": 1.0,
            "wrist_pitch_velocity_peak_ratio": 1.0,
            "wrist_roll_velocity_peak_detected": False,
            "wrist_pitch_velocity_peak_detected": False,
            "wrist_roll_velocity_profile": "flat",
            "wrist_pitch_velocity_profile": "flat",
            "wrist_roll_candidate_active": False,
            "wrist_pitch_candidate_active": False,
            "wrist_pitch_up_detected": False,
            "wrist_pitch_down_detected": False,
            "wrist_roll_event_cooldown_active": False,
            "wrist_roll_event_blocked": False,
            "wrist_roll_event_pulse_active": False,
        }

    def _reset_gesture_peak_states(self) -> None:
        self._gesture_peak_states.clear()

    def _peak_trigger(
        self,
        name: str,
        value: float,
        threshold: float,
        condition_ok: bool,
        now_s: float,
    ) -> bool:
        state = self._gesture_peak_states.setdefault(
            name,
            {
                "state": "IDLE",
                "entered_s": 0.0,
                "cooldown_until_s": 0.0,
                "peak": 0.0,
                "last_value": value,
            },
        )
        phase = state["state"]
        last_value = float(state.get("last_value", value))
        decreasing = value < last_value
        state["last_value"] = value

        if phase == "COOLDOWN":
            if now_s < float(state.get("cooldown_until_s", 0.0)):
                return False
            if condition_ok and value >= threshold:
                return False
            phase = "IDLE"
            state["state"] = phase
            state["peak"] = 0.0

        if not condition_ok or value < threshold:
            state["state"] = "IDLE"
            state["entered_s"] = 0.0
            state["peak"] = 0.0
            return False

        if phase == "IDLE":
            state["state"] = "ABOVE_THRESHOLD"
            state["entered_s"] = now_s
            state["peak"] = value
            return False

        if phase == "ABOVE_THRESHOLD":
            state["peak"] = max(float(state.get("peak", 0.0)), value)
            sustained_s = now_s - float(state.get("entered_s", now_s))
            peak_required = threshold * GESTURE_PEAK_MULTIPLIER
            if (
                sustained_s >= GESTURE_SUSTAINED_MIN_MS / 1000.0
                and float(state["peak"]) >= peak_required
                and decreasing
            ):
                state["state"] = "COOLDOWN"
                state["cooldown_until_s"] = now_s + GESTURE_COOLDOWN_SEC
                return True
            return False

        state["state"] = "IDLE"
        state["peak"] = 0.0
        return False

    def _wrist_motion_features(self, now_s: float) -> dict[str, Any]:
        if not self._wrist_motion_history:
            return {
                "wrist_motion_roll_delta": None,
                "wrist_motion_pitch_delta": None,
                "wrist_motion_roll_abs_delta": None,
                "wrist_motion_pitch_abs_delta": None,
                "wrist_motion": "none",
                "wrist_roll_right_detected": False,
                "wrist_roll_left_detected": False,
                "wrist_roll_right_then_neutral_detected": False,
                "wrist_roll_velocity_dps": 0.0,
                "wrist_pitch_velocity_dps": 0.0,
                "wrist_roll_velocity_abs_dps": 0.0,
                "wrist_pitch_velocity_abs_dps": 0.0,
                "wrist_roll_velocity_peak_dps": 0.0,
                "wrist_pitch_velocity_peak_dps": 0.0,
                "wrist_roll_velocity_peak_ratio": 1.0,
                "wrist_pitch_velocity_peak_ratio": 1.0,
                "wrist_roll_velocity_peak_detected": False,
                "wrist_pitch_velocity_peak_detected": False,
                "wrist_roll_velocity_profile": "flat",
                "wrist_pitch_velocity_profile": "flat",
                "wrist_roll_candidate_active": False,
                "wrist_pitch_candidate_active": False,
                "wrist_pitch_up_detected": False,
                "wrist_pitch_down_detected": False,
                "wrist_roll_event_cooldown_active": False,
                "wrist_roll_event_blocked": False,
                "wrist_roll_event_pulse_active": False,
            }

        first_s = self._wrist_motion_history[0][0]
        last_s = self._wrist_motion_history[-1][0]
        baseline_end_s = first_s + min(WRIST_MOTION_BASELINE_S, max(0.05, (last_s - first_s) / 4.0))
        current_start_s = last_s - WRIST_MOTION_CURRENT_S
        baseline = [item for item in self._wrist_motion_history if item[0] <= baseline_end_s]
        current = [item for item in self._wrist_motion_history if item[0] >= current_start_s]
        baseline_pitch = self._median([item[1] for item in baseline])
        baseline_roll = self._median([item[2] for item in baseline])
        current_pitch = self._median([item[1] for item in current])
        current_roll = self._median([item[2] for item in current])
        if baseline_pitch is None or baseline_roll is None or current_pitch is None or current_roll is None:
            return {
                "wrist_motion_roll_delta": None,
                "wrist_motion_pitch_delta": None,
                "wrist_motion_roll_abs_delta": None,
                "wrist_motion_pitch_abs_delta": None,
                "wrist_motion": "none",
                "wrist_roll_right_detected": False,
                "wrist_roll_left_detected": False,
                "wrist_roll_right_then_neutral_detected": False,
                "wrist_roll_velocity_dps": 0.0,
                "wrist_pitch_velocity_dps": 0.0,
                "wrist_roll_velocity_abs_dps": 0.0,
                "wrist_pitch_velocity_abs_dps": 0.0,
                "wrist_roll_velocity_peak_dps": 0.0,
                "wrist_pitch_velocity_peak_dps": 0.0,
                "wrist_roll_velocity_peak_ratio": 1.0,
                "wrist_pitch_velocity_peak_ratio": 1.0,
                "wrist_roll_velocity_peak_detected": False,
                "wrist_pitch_velocity_peak_detected": False,
                "wrist_roll_velocity_profile": "flat",
                "wrist_pitch_velocity_profile": "flat",
                "wrist_roll_candidate_active": False,
                "wrist_pitch_candidate_active": False,
                "wrist_pitch_up_detected": False,
                "wrist_pitch_down_detected": False,
                "wrist_roll_event_cooldown_active": False,
                "wrist_roll_event_blocked": False,
                "wrist_roll_event_pulse_active": False,
            }

        raw_roll_samples = [(item[0], item[2]) for item in self._wrist_motion_history]
        unwrapped_roll_samples = self._unwrap_angle_samples(raw_roll_samples)
        baseline_unwrapped_roll_values = [
            value
            for sample_s, value in unwrapped_roll_samples
            if sample_s <= baseline_end_s
        ]
        current_unwrapped_roll_values = [
            value
            for sample_s, value in unwrapped_roll_samples
            if sample_s >= current_start_s
        ]
        baseline_roll_unwrapped = self._median(baseline_unwrapped_roll_values) or baseline_roll
        current_roll_unwrapped = self._median(current_unwrapped_roll_values) or current_roll
        roll_delta_samples = [
            (item[0], self._angle_delta(item[2], baseline_roll))
            for item in self._wrist_motion_history
        ]
        roll_unwrapped_delta_samples = [
            (sample_s, value - baseline_roll_unwrapped)
            for sample_s, value in unwrapped_roll_samples
        ]
        roll_deltas = [delta for _t, delta in roll_delta_samples]
        roll_unwrapped_deltas = [delta for _t, delta in roll_unwrapped_delta_samples]
        pitch_deltas = [self._angle_delta(item[1], baseline_pitch) for item in self._wrist_motion_history]
        pitch_delta_samples = [
            (item[0], self._angle_delta(item[1], baseline_pitch))
            for item in self._wrist_motion_history
        ]
        roll_low = self._percentile(roll_deltas, 0.05) or 0.0
        roll_high = self._percentile(roll_deltas, 0.95) or 0.0
        pitch_low = self._percentile(pitch_deltas, 0.05) or 0.0
        pitch_high = self._percentile(pitch_deltas, 0.95) or 0.0
        roll_unwrapped_low = self._percentile(roll_unwrapped_deltas, 0.05) or 0.0
        roll_unwrapped_high = self._percentile(roll_unwrapped_deltas, 0.95) or 0.0
        roll_abs_delta = max(abs(roll_low), abs(roll_high))
        roll_abs_delta_unwrapped = max(abs(roll_unwrapped_low), abs(roll_unwrapped_high))
        pitch_abs_delta = max(abs(pitch_low), abs(pitch_high))
        roll_delta = self._angle_delta(current_roll, baseline_roll)
        roll_delta_unwrapped = current_roll_unwrapped - baseline_roll_unwrapped
        pitch_delta = self._angle_delta(current_pitch, baseline_pitch)
        roll_velocity = self._velocity_profile(
            raw_roll_samples,
            now_s,
        )
        roll_unwrapped_velocity = self._velocity_profile(
            unwrapped_roll_samples,
            now_s,
        )
        pitch_velocity = self._velocity_profile(
            [(item[0], item[1]) for item in self._wrist_motion_history],
            now_s,
        )
        large_roll_times = [
            sample_s
            for sample_s, sample_delta in roll_delta_samples
            if abs(sample_delta) >= WRIST_TRAINED_ROLL_LARGE_DEG
        ]
        first_large_roll_s = min(large_roll_times) if large_roll_times else None
        last_large_roll_s = max(large_roll_times) if large_roll_times else None
        large_roll_span_s = (
            last_large_roll_s - first_large_roll_s
            if first_large_roll_s is not None and last_large_roll_s is not None
            else 0.0
        )
        current_roll_deltas = [
            sample_delta
            for sample_s, sample_delta in roll_delta_samples
            if sample_s >= now_s - WRIST_TRAINED_ROLL_CURRENT_S
        ]
        current_roll_abs_deltas = [abs(sample_delta) for sample_delta in current_roll_deltas]
        current_roll_unwrapped_deltas = [
            sample_delta
            for sample_s, sample_delta in roll_unwrapped_delta_samples
            if sample_s >= now_s - WRIST_TRAINED_ROLL_CURRENT_S
        ]
        current_roll_abs_unwrapped_deltas = [abs(sample_delta) for sample_delta in current_roll_unwrapped_deltas]
        roll_neutral_fraction = (
            sum(1 for sample_delta in current_roll_abs_deltas if sample_delta <= WRIST_TRAINED_ROLL_RETURN_DEG)
            / len(current_roll_abs_deltas)
            if current_roll_abs_deltas
            else 0.0
        )
        current_roll_abs_median = self._median(current_roll_abs_deltas) or 0.0
        current_roll_abs_unwrapped_median = self._median(current_roll_abs_unwrapped_deltas) or 0.0
        stable_roll_return = bool(current_roll_abs_deltas) and (
            current_roll_abs_median <= WRIST_TRAINED_ROLL_RETURN_DEG
            and roll_neutral_fraction >= WRIST_TRAINED_ROLL_RETURN_NEUTRAL_RATIO
        )
        stable_roll_hold = bool(current_roll_deltas) and (
            sum(
                1
                for sample_delta in current_roll_deltas
                if abs(sample_delta) >= WRIST_TRAINED_ROLL_HELD_DEG
            )
            / len(current_roll_deltas)
            >= 0.75
        )
        stable_sampled_left_hold = bool(current_roll_abs_unwrapped_deltas) and (
            sum(
                1
                for sample_delta in current_roll_abs_unwrapped_deltas
                if sample_delta >= WRIST_SAMPLED_LEFT_CURRENT_DEG
            )
            / len(current_roll_abs_unwrapped_deltas)
            >= WRIST_SAMPLED_LEFT_STABLE_RATIO
        )
        roll_hold_retention = abs(roll_delta) / max(roll_abs_delta, 1.0)
        roll_axis_clear = roll_abs_delta >= max(
            WRIST_TRAINED_ROLL_LARGE_DEG,
            pitch_abs_delta * WRIST_TRAINED_ROLL_AXIS_RATIO,
        )
        left_axis_clear = pitch_abs_delta >= max(
            WRIST_TRAINED_LEFT_PITCH_DEG,
            roll_abs_delta * WRIST_TRAINED_LEFT_AXIS_RATIO,
        )
        pitch_axis_clear = pitch_abs_delta >= max(
            WRIST_PITCH_GESTURE_DEG,
            roll_abs_delta * WRIST_TRAINED_LEFT_AXIS_RATIO,
        )
        roll_candidate_active = (
            bool(large_roll_times)
            or roll_abs_delta >= WRIST_ROLL_GUARD_DEG
            or (
                abs(roll_delta) >= WRIST_ROLL_GUARD_DEG
                and roll_velocity["velocity_peak_dps"] >= WRIST_ROLL_GUARD_VELOCITY_DPS
            )
        )
        pitch_candidate_active = pitch_axis_clear and abs(pitch_delta) >= WRIST_PITCH_GESTURE_HELD_DEG
        roll_right_decision_ready = (
            first_large_roll_s is not None
            and now_s - first_large_roll_s >= WRIST_TRAINED_ROLL_RIGHT_DECISION_S
        )
        roll_return_ready = (
            last_large_roll_s is not None
            and now_s - last_large_roll_s >= WRIST_TRAINED_ROLL_RETURN_SETTLE_S
            and large_roll_span_s >= WRIST_TRAINED_ROLL_LARGE_SPAN_S
        )
        large_left_pitch_times = [
            sample_s
            for sample_s, sample_delta in pitch_delta_samples
            if sample_delta <= -WRIST_TRAINED_LEFT_PITCH_DEG
        ]
        first_large_left_pitch_s = min(large_left_pitch_times) if large_left_pitch_times else None
        left_decision_ready = (
            first_large_left_pitch_s is not None
            and now_s - first_large_left_pitch_s >= WRIST_TRAINED_LEFT_DECISION_S
        )
        sampled_left_large_roll_times = [
            sample_s
            for sample_s, sample_delta in roll_unwrapped_delta_samples
            if abs(sample_delta) >= WRIST_SAMPLED_LEFT_ROLL_DEG
        ]
        first_sampled_left_large_roll_s = min(sampled_left_large_roll_times) if sampled_left_large_roll_times else None
        sampled_left_decision_ready = (
            first_sampled_left_large_roll_s is not None
            and now_s - first_sampled_left_large_roll_s >= WRIST_SAMPLED_LEFT_DECISION_S
        )
        pitch_up_times = [
            sample_s
            for sample_s, sample_delta in pitch_delta_samples
            if sample_delta >= WRIST_PITCH_GESTURE_DEG
        ]
        pitch_down_times = [
            sample_s
            for sample_s, sample_delta in pitch_delta_samples
            if sample_delta <= -WRIST_PITCH_GESTURE_DEG
        ]
        current_pitch_deltas = [
            sample_delta
            for sample_s, sample_delta in pitch_delta_samples
            if sample_s >= now_s - WRIST_TRAINED_ROLL_CURRENT_S
        ]
        stable_pitch_up = bool(current_pitch_deltas) and (
            sum(1 for sample_delta in current_pitch_deltas if sample_delta >= WRIST_PITCH_GESTURE_HELD_DEG)
            / len(current_pitch_deltas)
            >= 0.75
        )
        stable_pitch_down = bool(current_pitch_deltas) and (
            sum(1 for sample_delta in current_pitch_deltas if sample_delta <= -WRIST_PITCH_GESTURE_HELD_DEG)
            / len(current_pitch_deltas)
            >= 0.75
        )
        pitch_hold_retention = abs(pitch_delta) / max(pitch_abs_delta, 1.0)
        pitch_up_ready = bool(pitch_up_times) and now_s - min(pitch_up_times) >= WRIST_PITCH_DECISION_S
        pitch_down_ready = bool(pitch_down_times) and now_s - min(pitch_down_times) >= WRIST_PITCH_DECISION_S

        sampled_roll_left_early_active = (
            pitch_abs_delta >= WRIST_SAMPLED_LEFT_PITCH_DEG
            and roll_abs_delta_unwrapped >= max(
                WRIST_SAMPLED_LEFT_ROLL_DEG,
                pitch_abs_delta * WRIST_SAMPLED_LEFT_AXIS_RATIO,
            )
            and current_roll_abs_unwrapped_median >= WRIST_SAMPLED_LEFT_CURRENT_DEG
            and stable_sampled_left_hold
        )
        raw_roll_right_then_neutral = (
            roll_axis_clear
            and stable_roll_return
            and roll_return_ready
            and not sampled_roll_left_early_active
            and roll_velocity["velocity_peak_dps"] >= WRIST_VELOCITY_MIN_PEAK_DPS
        )
        sampled_roll_left_candidate_active = (
            not raw_roll_right_then_neutral
            and sampled_roll_left_early_active
            and sampled_left_decision_ready
            and roll_unwrapped_velocity["velocity_peak_dps"] >= WRIST_VELOCITY_MIN_PEAK_DPS
        )
        roll_left_candidate_active = (
            not raw_roll_right_then_neutral
            and left_axis_clear
            and left_decision_ready
            and not roll_candidate_active
            and roll_abs_delta <= WRIST_TRAINED_LEFT_ROLL_LIMIT_DEG
            and pitch_velocity["velocity_peak_dps"] >= WRIST_VELOCITY_MIN_PEAK_DPS
        )
        sampled_roll_left = self._peak_trigger(
            "sampled_roll_left",
            current_roll_abs_unwrapped_median,
            WRIST_SAMPLED_LEFT_CURRENT_DEG,
            sampled_roll_left_candidate_active,
            now_s,
        )
        roll_right = self._peak_trigger(
            "roll_right",
            abs(roll_delta),
            WRIST_TRAINED_ROLL_HELD_DEG,
            not raw_roll_right_then_neutral
            and not sampled_roll_left
            and not sampled_roll_left_candidate_active
            and roll_axis_clear
            and stable_roll_hold
            and roll_hold_retention >= WRIST_TRAINED_ROLL_HOLD_RETENTION_RATIO
            and roll_right_decision_ready
            and roll_velocity["velocity_peak_dps"] >= WRIST_VELOCITY_MIN_PEAK_DPS,
            now_s,
        )
        roll_right_then_neutral = raw_roll_right_then_neutral
        pitch_style_roll_left = self._peak_trigger(
            "roll_left",
            max(0.0, -pitch_delta),
            WRIST_TRAINED_LEFT_PITCH_DEG,
            not roll_right_then_neutral
            and not roll_right
            and not sampled_roll_left
            and not sampled_roll_left_candidate_active
            and roll_left_candidate_active,
            now_s,
        )
        roll_left = sampled_roll_left or pitch_style_roll_left
        pitch_up = self._peak_trigger(
            "pitch_up",
            max(0.0, pitch_delta),
            WRIST_PITCH_GESTURE_DEG,
            not roll_right_then_neutral
            and not roll_right
            and not roll_left
            and not roll_candidate_active
            and not roll_left_candidate_active
            and pitch_axis_clear
            and stable_pitch_up
            and pitch_hold_retention >= WRIST_PITCH_HOLD_RETENTION_RATIO
            and pitch_up_ready,
            now_s,
        )
        pitch_down = self._peak_trigger(
            "pitch_down",
            max(0.0, -pitch_delta),
            WRIST_PITCH_GESTURE_DEG,
            not roll_right_then_neutral
            and not roll_right
            and not roll_left
            and not roll_candidate_active
            and not roll_left_candidate_active
            and pitch_axis_clear
            and stable_pitch_down
            and pitch_hold_retention >= WRIST_PITCH_HOLD_RETENTION_RATIO
            and pitch_down_ready,
            now_s,
        )
        roll_candidate_active = roll_candidate_active or roll_right or roll_right_then_neutral
        pitch_candidate_active = pitch_candidate_active or pitch_up or pitch_down or roll_left
        motion = "none"
        if roll_right_then_neutral:
            motion = "roll_right_then_neutral"
        elif roll_right:
            motion = "roll_right"
        elif roll_left:
            motion = "roll_left"

        return {
            "wrist_motion_roll_delta": roll_delta,
            "wrist_motion_pitch_delta": pitch_delta,
            "wrist_motion_roll_abs_delta": roll_abs_delta,
            "wrist_motion_pitch_abs_delta": pitch_abs_delta,
            "wrist_motion": motion,
            "wrist_roll_right_detected": roll_right,
            "wrist_roll_left_detected": roll_left,
            "wrist_roll_right_then_neutral_detected": roll_right_then_neutral,
            "wrist_roll_velocity_dps": roll_velocity["velocity_dps"],
            "wrist_pitch_velocity_dps": pitch_velocity["velocity_dps"],
            "wrist_roll_velocity_abs_dps": roll_velocity["velocity_abs_dps"],
            "wrist_pitch_velocity_abs_dps": pitch_velocity["velocity_abs_dps"],
            "wrist_roll_velocity_peak_dps": roll_velocity["velocity_peak_dps"],
            "wrist_pitch_velocity_peak_dps": pitch_velocity["velocity_peak_dps"],
            "wrist_roll_velocity_peak_ratio": roll_velocity["velocity_peak_ratio"],
            "wrist_pitch_velocity_peak_ratio": pitch_velocity["velocity_peak_ratio"],
            "wrist_roll_velocity_peak_detected": roll_velocity["velocity_peak_detected"],
            "wrist_pitch_velocity_peak_detected": pitch_velocity["velocity_peak_detected"],
            "wrist_roll_velocity_profile": roll_velocity["velocity_profile"],
            "wrist_pitch_velocity_profile": pitch_velocity["velocity_profile"],
            "wrist_roll_candidate_active": roll_candidate_active,
            "wrist_pitch_candidate_active": pitch_candidate_active,
            "wrist_pitch_up_detected": pitch_up,
            "wrist_pitch_down_detected": pitch_down,
            "wrist_roll_event_cooldown_active": False,
            "wrist_roll_event_blocked": False,
            "wrist_roll_event_pulse_active": False,
        }

    def _wrist_axis_features(self, pitch: Any, roll: Any, now_s: float) -> dict[str, Any]:
        pitch_number = self._number(pitch)
        roll_number = self._number(roll)
        if pitch_number is None or roll_number is None:
            self._wrist_axis_history.clear()
            self._wrist_motion_history.clear()
            self._reset_gesture_peak_states()
            return self._empty_wrist_features()

        self._wrist_axis_history.append((now_s, pitch_number, roll_number))
        cutoff_s = now_s - WRIST_AXIS_WINDOW_S
        self._wrist_axis_history = [item for item in self._wrist_axis_history if item[0] >= cutoff_s]
        anchor = self._wrist_axis_history[0]
        pitch_delta = self._angle_delta(pitch_number, anchor[1])
        roll_delta = self._angle_delta(roll_number, anchor[2])
        pitch_abs_delta = abs(pitch_delta)
        roll_abs_delta = abs(roll_delta)
        pitch_dominant = self._dominates(pitch_abs_delta, roll_abs_delta)
        roll_dominant = self._dominates(roll_abs_delta, pitch_abs_delta)
        dominant_axis = "roll" if roll_dominant else "pitch" if pitch_dominant else "none"
        features = {
            "wrist_pitch_delta": pitch_delta,
            "wrist_roll_delta": roll_delta,
            "wrist_pitch_abs_delta": pitch_abs_delta,
            "wrist_roll_abs_delta": roll_abs_delta,
            "wrist_pitch_dominant": pitch_dominant,
            "wrist_roll_dominant": roll_dominant,
            "wrist_dominant_axis": dominant_axis,
        }
        self._wrist_motion_history.append((now_s, pitch_number, roll_number))
        motion_cutoff_s = now_s - WRIST_MOTION_WINDOW_S
        self._wrist_motion_history = [item for item in self._wrist_motion_history if item[0] >= motion_cutoff_s]
        features.update(self._wrist_motion_features(now_s))
        return features

    def build_input_dict(
        self,
        serial_state: dict[str, Any],
        hand_state: dict[str, dict[str, Any]],
        now_s: float | None = None,
    ) -> dict[str, Any]:
        now_s = time.monotonic() if now_s is None else now_s
        devices = serial_state.get("devices", {}) if isinstance(serial_state, dict) else {}
        wrist = devices.get("wristband", {}) if isinstance(devices, dict) else {}
        camdock = devices.get("camdock", {}) if isinstance(devices, dict) else {}
        keyboard = devices.get("keyboard", {}) if isinstance(devices, dict) else {}
        fans = devices.get("fans", {}) if isinstance(devices, dict) else {}
        tof = camdock.get("tof", {}) if isinstance(camdock, dict) else {}
        keyboard_tof = keyboard.get("tof", {}) if isinstance(keyboard, dict) else {}
        accel = wrist.get("accel", {}) if isinstance(wrist, dict) else {}
        gyro = wrist.get("gyro", {}) if isinstance(wrist, dict) else {}
        right = hand_state.get("right", {}) if isinstance(hand_state, dict) else {}
        left = hand_state.get("left", {}) if isinstance(hand_state, dict) else {}
        wrist_pitch = wrist.get("pitch")
        wrist_roll = wrist.get("roll")

        input_dict = {
            "right_hand_x": right.get("x") if right.get("visible") else None,
            "right_hand_y": self._camera_y_up(right),
            "right_hand_z_mm": tof.get("right_mm"),
            "right_hand_gesture": right.get("gesture") if right.get("visible") else None,
            "left_hand_x": left.get("x") if left.get("visible") else None,
            "left_hand_y": self._camera_y_up(left),
            "left_hand_z_mm": tof.get("left_mm"),
            "left_hand_gesture": left.get("gesture") if left.get("visible") else None,
            "wrist_accel_x": accel.get("x"),
            "wrist_accel_y": accel.get("y"),
            "wrist_accel_z": accel.get("z"),
            "wrist_gyro_x": gyro.get("x"),
            "wrist_gyro_y": gyro.get("y"),
            "wrist_gyro_z": gyro.get("z"),
            "wrist_pitch": wrist_pitch,
            "wrist_roll": wrist_roll,
            "camdock_battery_level": camdock.get("battery_level"),
            "wristband_battery_level": wrist.get("battery_level"),
            "fans_battery_level": fans.get("battery_level") if isinstance(fans, dict) else None,
            "keyboard_input": keyboard.get("input") if isinstance(keyboard, dict) else None,
            "keyboard_sensor_1_mm": keyboard_tof.get("sensor_1_mm") if isinstance(keyboard_tof, dict) else None,
            "keyboard_sensor_2_mm": keyboard_tof.get("sensor_2_mm") if isinstance(keyboard_tof, dict) else None,
            "keyboard_sensor_3_mm": keyboard_tof.get("sensor_3_mm") if isinstance(keyboard_tof, dict) else None,
            "keyboard_sensor_4_mm": keyboard_tof.get("sensor_4_mm") if isinstance(keyboard_tof, dict) else None,
            "charging_dock_input": "TBD",
            "audiodock_input": "TBD",
            "fans_input": fans.get("input") if isinstance(fans, dict) else None,
        }
        input_dict.update(self._wrist_axis_features(wrist_pitch, wrist_roll, now_s))
        return input_dict

    def build_input_array(self, input_dict: dict[str, Any]) -> list[Any]:
        return [input_dict.get(field) for field in FIELD_ORDER]

    def build_snapshot(
        self,
        serial_state: dict[str, Any],
        hand_state: dict[str, dict[str, Any]],
        now_s: float | None = None,
    ) -> dict[str, Any]:
        input_dict = self.build_input_dict(serial_state, hand_state, now_s=now_s)
        return {
            "field_order": FIELD_ORDER,
            "input_dict": input_dict,
            "input_array": self.build_input_array(input_dict),
            "raw_device_state": serial_state,
            "hand_state": hand_state,
        }
