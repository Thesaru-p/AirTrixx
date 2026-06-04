from __future__ import annotations

import math
import time
from typing import Any

from serial_bridge import SerialBridge


SERVO_FIELDS = ("r_pan", "r_tilt", "l_pan", "l_tilt", "cam_pan", "cam_tilt")
BRACKET_SERVO_FIELDS = {
    "right": ("r_pan", "r_tilt"),
    "left": ("l_pan", "l_tilt"),
    "camera": ("cam_pan", "cam_tilt"),
}
HAND_PREFIX = {
    "right": "r",
    "left": "l",
}
MIN_HAND_BOUNDARY_SPAN = 0.05


class ServoController:
    def __init__(
        self,
        serial_bridge: SerialBridge,
        calibration: dict[str, Any],
        servo_min_tick: int = 0,
        servo_max_tick: int = 4095,
        max_command_hz: float = 40.0,
        camera_width: int = 640,
        camera_height: int = 480,
        horizontal_fov_deg: float = 70.0,
        vertical_fov_deg: float = 43.0,
    ) -> None:
        self.serial_bridge = serial_bridge
        self.calibration = dict(calibration)
        self.servo_min_tick = servo_min_tick
        self.servo_max_tick = servo_max_tick
        self.max_command_hz = max_command_hz
        self.camera_width = max(1, int(camera_width))
        self.camera_height = max(1, int(camera_height))
        self.horizontal_fov_deg = float(horizontal_fov_deg)
        self.vertical_fov_deg = float(vertical_fov_deg)
        self._last_send_time = 0.0
        self._smoothed: dict[str, float] = {}
        self._hand_history: dict[str, tuple[float, float, float]] = {}
        self._last_raw_tof_mm: dict[str, float] = {}
        self._last_distance_mm: dict[str, float] = {}
        self._last_distance_debug: dict[str, dict[str, Any]] = {}
        self._debug_sequence = 0
        self.last_debug_snapshot: dict[str, Any] = {}
        self._last_auto_bracket_ticks: dict[str, dict[str, Any]] = {}
        self._last_sent_bracket_ticks: dict[str, dict[str, Any]] = {}

    def update_calibration(self, calibration: dict[str, Any]) -> None:
        self.calibration = dict(calibration)
        self._smoothed.clear()
        self._hand_history.clear()
        self._last_raw_tof_mm.clear()
        self._last_distance_mm.clear()
        self._last_distance_debug.clear()

    def send_for_hands(
        self,
        hands: dict[str, dict[str, Any]],
        serial_state: dict[str, Any] | None = None,
        force: bool = False,
    ) -> bool:
        now = time.monotonic()
        if not force and now - self._last_send_time < 1.0 / self.max_command_hz:
            return False

        servos = {name: 0 for name in SERVO_FIELDS}
        use_geometry = self._use_dock_geometry()
        debug: dict[str, Any] = {
            "seq": self._next_debug_sequence(),
            "mode": "auto_geometry" if use_geometry else "auto_normalized",
            "time_s": time.time(),
            "active_pair": "none",
            "sent": False,
            "hands": {},
            "servos": {},
        }
        if use_geometry:
            active_pair = self._fill_geometry_servo_targets(hands, serial_state, servos, now, debug["hands"])
        else:
            active_pair = self._fill_normalized_servo_targets(hands, servos, now, debug["hands"])

        if active_pair == "none":
            self._smoothed.clear()
            self._hand_history.clear()

        sent = self._send(active_pair, servos, disable_unused=False, coalesce=True)
        debug["active_pair"] = active_pair
        debug["sent"] = sent
        debug["servos"] = {name: int(servos.get(name, 0)) for name in SERVO_FIELDS}
        self.last_debug_snapshot = debug
        if sent:
            self._last_send_time = now
            self._record_bracket_ticks(active_pair, servos, source="auto", auto=True)
        return sent

    def center_camera(self, force: bool = True) -> bool:
        servos = {name: 0 for name in SERVO_FIELDS}
        servos["cam_pan"] = self._clamp_tick(int(self.calibration.get("cam_pan_center", 307)))
        servos["cam_tilt"] = self._clamp_tick(int(self.calibration.get("cam_tilt_center", 307)))
        sent = self._send("camera", servos)
        if sent or force:
            self._last_send_time = time.monotonic()
        if sent:
            self._record_bracket_ticks("camera", servos, source="center", auto=False)
        return sent

    def center_bracket(self, bracket: str) -> bool:
        pan, tilt = self.center_ticks_for_bracket(bracket)
        return self.send_bracket_position(bracket, pan, tilt)

    def center_ticks_for_bracket(self, bracket: str) -> tuple[int, int]:
        if bracket not in BRACKET_SERVO_FIELDS:
            raise ValueError(f"Unknown servo bracket: {bracket}")
        pan_key, tilt_key = self._center_keys_for_bracket(bracket)
        return (
            self._clamp_tick(int(self.calibration.get(pan_key, 307))),
            self._clamp_tick(int(self.calibration.get(tilt_key, 307))),
        )

    def current_camera_ticks(self) -> tuple[int, int]:
        entry = self._last_sent_bracket_ticks.get("camera")
        if entry:
            return self._clamp_tick(int(entry["pan"])), self._clamp_tick(int(entry["tilt"]))
        return self.center_ticks_for_bracket("camera")

    def current_camera_pose_degrees(self) -> tuple[float, float]:
        cam_pan, cam_tilt = self.current_camera_ticks()
        return self._camera_pose_degrees_for_ticks(cam_pan, cam_tilt)

    def parallel_hand_ticks_from_camera(self, cam_pan: int, cam_tilt: int) -> tuple[int, int, int, int]:
        cam_center_pan, cam_center_tilt = self.center_ticks_for_bracket("camera")
        pan_delta = int(cam_pan) - cam_center_pan
        tilt_delta = int(cam_tilt) - cam_center_tilt
        r_center_pan, r_center_tilt = self.center_ticks_for_bracket("right")
        l_center_pan, l_center_tilt = self.center_ticks_for_bracket("left")
        return (
            self._clamp_tick(r_center_pan + pan_delta),
            self._clamp_tick(r_center_tilt + tilt_delta),
            self._clamp_tick(l_center_pan + pan_delta),
            self._clamp_tick(l_center_tilt + tilt_delta),
        )

    def center_all_brackets(self) -> bool:
        cam_pan, cam_tilt = self.center_ticks_for_bracket("camera")
        return self.send_camera_with_parallel_tof(cam_pan, cam_tilt)

    def send_camera_with_parallel_tof(self, cam_pan: int, cam_tilt: int) -> bool:
        """Move camera and both ToF brackets with the same pan/tilt offset from center."""
        cam_pan = self._clamp_tick(cam_pan)
        cam_tilt = self._clamp_tick(cam_tilt)
        r_pan, r_tilt, l_pan, l_tilt = self.parallel_hand_ticks_from_camera(cam_pan, cam_tilt)

        zero = {name: 0 for name in SERVO_FIELDS}
        cam_servos = dict(zero)
        cam_servos["cam_pan"] = cam_pan
        cam_servos["cam_tilt"] = cam_tilt
        right_servos = dict(zero)
        right_servos["r_pan"] = r_pan
        right_servos["r_tilt"] = r_tilt
        left_servos = dict(zero)
        left_servos["l_pan"] = l_pan
        left_servos["l_tilt"] = l_tilt

        all_servos = dict(zero)
        all_servos["cam_pan"] = cam_pan
        all_servos["cam_tilt"] = cam_tilt
        all_servos["r_pan"] = r_pan
        all_servos["r_tilt"] = r_tilt
        all_servos["l_pan"] = l_pan
        all_servos["l_tilt"] = l_tilt

        # Single "dock" command only when Cam Dock firmware supports ACTIVE_PAIR_DOCK.
        if bool(self.calibration.get("use_dock_servo_pair", False)):
            sent = self._send("dock", all_servos, disable_unused=True)
        else:
            # Legacy path: three commands without disabling other brackets between steps.
            sent = self._send("camera", cam_servos, disable_unused=False)
            sent = self._send("right", right_servos, disable_unused=False) and sent
            sent = self._send("left", left_servos, disable_unused=False) and sent

        if sent:
            self._last_send_time = time.monotonic()
            self._record_bracket_ticks("dock", all_servos, source="parallel", auto=False)
        return sent

    def send_bracket_position(self, bracket: str, pan_tick: int, tilt_tick: int) -> bool:
        if bracket not in BRACKET_SERVO_FIELDS:
            raise ValueError(f"Unknown servo bracket: {bracket}")
        pan_field, tilt_field = BRACKET_SERVO_FIELDS[bracket]
        servos = {name: 0 for name in SERVO_FIELDS}
        servos[pan_field] = self._clamp_tick(pan_tick)
        servos[tilt_field] = self._clamp_tick(tilt_tick)
        sent = self._send(bracket, servos)
        if sent:
            self._last_send_time = time.monotonic()
            self._record_bracket_ticks(bracket, servos, source="manual", auto=False)
        return sent

    def last_auto_bracket_ticks(self, bracket: str) -> dict[str, Any] | None:
        entry = self._last_auto_bracket_ticks.get(bracket)
        return dict(entry) if entry else None

    def last_sent_bracket_ticks(self, bracket: str) -> dict[str, Any] | None:
        entry = self._last_sent_bracket_ticks.get(bracket)
        return dict(entry) if entry else None

    def last_debug_for_bracket(self, bracket: str) -> dict[str, Any] | None:
        hands = self.last_debug_snapshot.get("hands", {})
        entry = hands.get(bracket) if isinstance(hands, dict) else None
        return dict(entry) if isinstance(entry, dict) else None

    def save_bracket_center(self, bracket: str, pan_tick: int, tilt_tick: int) -> dict[str, Any]:
        pan_key, tilt_key = self._center_keys_for_bracket(bracket)
        self.calibration[pan_key] = self._clamp_tick(pan_tick)
        self.calibration[tilt_key] = self._clamp_tick(tilt_tick)
        self._smoothed.clear()
        return dict(self.calibration)

    def build_session_calibration_entry(
        self,
        side: str,
        values: dict[str, Any],
        distance_mm: float,
        distance_source: str,
    ) -> dict[str, Any]:
        if side not in HAND_PREFIX:
            raise ValueError(f"Unknown hand side: {side}")
        x = max(0.0, min(1.0, float(values["x"])))
        y = max(0.0, min(1.0, float(values["y"])))
        distance_mm = max(1.0, float(distance_mm))
        solution = self._geometry_solution_for_hand(side, x, y, distance_mm)
        prefix = HAND_PREFIX[side]

        pan_center_key, tilt_center_key = self._center_keys_for_bracket(side)
        pan_center = self._calib_float(pan_center_key, 307.0)
        tilt_center = self._calib_float(tilt_center_key, 307.0)
        pan_ticks_per_deg = self._calib_float("pan_ticks_per_degree", 2.25)
        tilt_ticks_per_deg = self._calib_float("tilt_ticks_per_degree", 2.25)
        pan_sign = self._calib_float(f"{prefix}_pan_sign", -1.0)
        tilt_sign = self._calib_float(f"{prefix}_tilt_sign", -1.0)
        base_pan_offset = self._calib_float(f"{prefix}_pan_angle_offset_deg", 0.0)
        base_tilt_offset = self._calib_float(f"{prefix}_tilt_angle_offset_deg", 0.0)

        anchor_pan_tick = self._clamp_tick(round(pan_center))
        anchor_tilt_tick = self._clamp_tick(round(tilt_center))
        session_pan_offset = self._angle_offset_for_anchor_tick(
            solution["yaw_deg"],
            anchor_pan_tick,
            pan_center,
            pan_sign,
            pan_ticks_per_deg,
            base_pan_offset,
        )
        session_tilt_offset = self._angle_offset_for_anchor_tick(
            solution["pitch_deg"],
            anchor_tilt_tick,
            tilt_center,
            tilt_sign,
            tilt_ticks_per_deg,
            base_tilt_offset,
        )
        camera_pose = solution["camera_pose"]
        point = solution["point_mm"]
        return {
            "x": round(x, 4),
            "y": round(y, 4),
            "tof_mm": round(distance_mm, 1),
            "distance_source": distance_source,
            "score": round(float(values.get("score") or 0.0), 4),
            "neutral_yaw_deg": round(float(solution["yaw_deg"]), 4),
            "neutral_pitch_deg": round(float(solution["pitch_deg"]), 4),
            "pan_angle_offset_deg": round(session_pan_offset, 4),
            "tilt_angle_offset_deg": round(session_tilt_offset, 4),
            "anchor_pan_tick": anchor_pan_tick,
            "anchor_tilt_tick": anchor_tilt_tick,
            "camera_pan_tick": int(camera_pose["pan_tick"]),
            "camera_tilt_tick": int(camera_pose["tilt_tick"]),
            "camera_yaw_deg": round(float(camera_pose["yaw_deg"]), 4),
            "camera_pitch_deg": round(float(camera_pose["pitch_deg"]), 4),
            "point_x_mm": round(float(point[0]), 1),
            "point_y_mm": round(float(point[1]), 1),
            "point_z_mm": round(float(point[2]), 1),
        }

    def disable_all(self) -> bool:
        self._smoothed.clear()
        self._hand_history.clear()
        return self._send("none", {name: 0 for name in SERVO_FIELDS})

    def _fill_geometry_servo_targets(
        self,
        hands: dict[str, dict[str, Any]],
        serial_state: dict[str, Any] | None,
        servos: dict[str, int],
        now: float,
        hand_debug: dict[str, Any],
    ) -> str:
        active_sides: list[str] = []
        for side in ("right", "left"):
            values = hands.get(side, {}) if isinstance(hands, dict) else {}
            if not self._hand_visible(values):
                self._hand_history.pop(side, None)
                continue

            x, y = self._predicted_hand_position(side, float(values["x"]), float(values["y"]), now)
            distance_mm = self._distance_mm_for_side(side, serial_state)
            pan_tick, tilt_tick, details = self._geometry_ticks_for_hand(side, x, y, distance_mm)
            pan_field, tilt_field = BRACKET_SERVO_FIELDS[side]
            servos[pan_field] = pan_tick
            servos[tilt_field] = tilt_tick
            details.update(
                {
                    "raw_image_x": float(values["x"]),
                    "raw_image_y": float(values["y"]),
                    "predicted_image_x": x,
                    "predicted_image_y": y,
                    "predicted_y_up": 1.0 - y,
                    "score": float(values.get("score") or 0.0),
                    "gesture": values.get("gesture"),
                    "distance_debug": dict(self._last_distance_debug.get(side, {})),
                }
            )
            hand_debug[side] = details
            active_sides.append(side)

        return self._active_pair_from_sides(active_sides)

    def _fill_normalized_servo_targets(
        self,
        hands: dict[str, dict[str, Any]],
        servos: dict[str, int],
        now: float,
        hand_debug: dict[str, Any],
    ) -> str:
        active_sides: list[str] = []
        for side in ("right", "left"):
            values = hands.get(side, {}) if isinstance(hands, dict) else {}
            if not self._hand_visible(values):
                self._hand_history.pop(side, None)
                continue
            x, y = self._predicted_hand_position(side, float(values["x"]), float(values["y"]), now)
            x = self._hand_coordinate_from_boundaries(x, "x")
            y = self._hand_coordinate_from_boundaries(y, "y")
            y = 1.0 - y
            prefix = HAND_PREFIX[side]
            pan_field, tilt_field = BRACKET_SERVO_FIELDS[side]
            servos[pan_field] = self._pulse_from_normalized(pan_field, f"{prefix}_pan_center", x, "x")
            servos[tilt_field] = self._pulse_from_normalized(tilt_field, f"{prefix}_tilt_center", y, "y")
            hand_debug[side] = {
                "raw_image_x": float(values["x"]),
                "raw_image_y": float(values["y"]),
                "bounded_x": x,
                "bounded_y_up": y,
                "score": float(values.get("score") or 0.0),
                "gesture": values.get("gesture"),
                "pan_tick": int(servos[pan_field]),
                "tilt_tick": int(servos[tilt_field]),
            }
            active_sides.append(side)

        return self._active_pair_from_sides(active_sides)

    def _geometry_ticks_for_hand(self, side: str, x: float, y: float, distance_mm: float) -> tuple[int, int, dict[str, Any]]:
        prefix = HAND_PREFIX[side]
        solution = self._geometry_solution_for_hand(side, x, y, distance_mm)
        camera_ray = solution["camera_ray"]
        ray = solution["ray"]
        point = solution["point_mm"]
        yaw_deg = solution["yaw_deg"]
        pitch_deg = solution["pitch_deg"]
        camera_pose = solution["camera_pose"]

        pan_center_key, tilt_center_key = self._center_keys_for_bracket(side)
        pan_center = self._calib_float(pan_center_key, 307.0)
        tilt_center = self._calib_float(tilt_center_key, 307.0)
        pan_ticks_per_deg = self._calib_float("pan_ticks_per_degree", 2.25)
        tilt_ticks_per_deg = self._calib_float("tilt_ticks_per_degree", 2.25)
        pan_sign = self._calib_float(f"{prefix}_pan_sign", -1.0)
        tilt_sign = self._calib_float(f"{prefix}_tilt_sign", -1.0)
        pan_offset_deg = self._calib_float(f"{prefix}_pan_angle_offset_deg", 0.0)
        tilt_offset_deg = self._calib_float(f"{prefix}_tilt_angle_offset_deg", 0.0)
        session_pan_offset_deg = self._session_angle_offset(side, "pan_angle_offset_deg")
        session_tilt_offset_deg = self._session_angle_offset(side, "tilt_angle_offset_deg")

        pan_angle_deg = yaw_deg + pan_offset_deg + session_pan_offset_deg
        tilt_angle_deg = pitch_deg + tilt_offset_deg + session_tilt_offset_deg
        pan_target = pan_center + pan_sign * pan_angle_deg * pan_ticks_per_deg
        tilt_target = tilt_center + tilt_sign * tilt_angle_deg * tilt_ticks_per_deg
        pan_tick = self._smooth_and_clamp_hand(f"{prefix}_pan", pan_target)
        tilt_tick = self._smooth_and_clamp_hand(f"{prefix}_tilt", tilt_target)
        return pan_tick, tilt_tick, {
            "distance_mm": distance_mm,
            "camera_ray": camera_ray,
            "ray": ray,
            "point_mm": point,
            "yaw_deg": yaw_deg,
            "pitch_deg": pitch_deg,
            "camera_pose": camera_pose,
            "pan_center": pan_center,
            "tilt_center": tilt_center,
            "pan_sign": pan_sign,
            "tilt_sign": tilt_sign,
            "base_pan_offset_deg": pan_offset_deg,
            "base_tilt_offset_deg": tilt_offset_deg,
            "session_pan_offset_deg": session_pan_offset_deg,
            "session_tilt_offset_deg": session_tilt_offset_deg,
            "pan_angle_deg": pan_angle_deg,
            "tilt_angle_deg": tilt_angle_deg,
            "pan_target": pan_target,
            "tilt_target": tilt_target,
            "pan_tick": pan_tick,
            "tilt_tick": tilt_tick,
        }

    def _camera_ray(self, x: float, y: float) -> tuple[float, float, float]:
        x = max(0.0, min(1.0, x))
        y = max(0.0, min(1.0, y))
        h_fov = max(1.0, min(179.0, self._calib_float("camera_horizontal_fov_deg", self.horizontal_fov_deg)))
        v_fov = max(1.0, min(179.0, self._calib_float("camera_vertical_fov_deg", self.vertical_fov_deg)))
        fx = self.camera_width / (2.0 * math.tan(math.radians(h_fov) / 2.0))
        fy = self.camera_height / (2.0 * math.tan(math.radians(v_fov) / 2.0))
        rx = ((x * self.camera_width) - (self.camera_width / 2.0)) / fx
        # MediaPipe image coordinates have y increasing downward. Dock geometry
        # uses +Y upward, so invert the image y component for the physical ray.
        ry = ((self.camera_height / 2.0) - (y * self.camera_height)) / fy
        rz = 1.0
        length = math.sqrt(rx * rx + ry * ry + rz * rz)
        return rx / length, ry / length, rz / length

    def _geometry_solution_for_hand(
        self,
        side: str,
        x: float,
        y: float,
        distance_mm: float,
    ) -> dict[str, Any]:
        camera_ray = self._camera_ray(x, y)
        camera_pose = self._camera_pose_debug()
        pitched_ray = self._pitch_rotate_vector(camera_ray, math.radians(camera_pose["pitch_deg"]))
        ray = self._normalize_vector(
            self._yaw_local_to_global(pitched_ray, math.radians(camera_pose["yaw_deg"]))
        )
        point = self._point_for_tof_distance(side, ray, distance_mm)
        yaw_deg, pitch_deg = self._angles_for_point(side, point)
        return {
            "camera_ray": camera_ray,
            "ray": ray,
            "point_mm": point,
            "yaw_deg": yaw_deg,
            "pitch_deg": pitch_deg,
            "camera_pose": camera_pose,
        }

    def _camera_pose_debug(self) -> dict[str, Any]:
        cam_pan, cam_tilt = self.current_camera_ticks()
        yaw_deg, pitch_deg = self._camera_pose_degrees_for_ticks(cam_pan, cam_tilt)
        return {
            "pan_tick": cam_pan,
            "tilt_tick": cam_tilt,
            "yaw_deg": yaw_deg,
            "pitch_deg": pitch_deg,
        }

    def _camera_pose_degrees_for_ticks(self, cam_pan: int, cam_tilt: int) -> tuple[float, float]:
        yaw_deg = self._angle_from_servo_ticks(
            cam_pan,
            "cam_pan_center",
            "pan_ticks_per_degree",
            "cam_pan_sign",
            "cam_pan_angle_offset_deg",
            -1.0,
        )
        pitch_deg = self._angle_from_servo_ticks(
            cam_tilt,
            "cam_tilt_center",
            "tilt_ticks_per_degree",
            "cam_tilt_sign",
            "cam_tilt_angle_offset_deg",
            -1.0,
        )
        return yaw_deg, pitch_deg

    def _angle_from_servo_ticks(
        self,
        tick: int,
        center_key: str,
        ticks_per_degree_key: str,
        sign_key: str,
        offset_key: str,
        default_sign: float,
    ) -> float:
        center = self._calib_float(center_key, 307.0)
        ticks_per_degree = self._calib_float(ticks_per_degree_key, 2.25)
        sign = self._calib_float(sign_key, default_sign)
        offset = self._calib_float(offset_key, 0.0)
        denominator = sign * ticks_per_degree
        if abs(denominator) < 0.0001:
            return 0.0
        return ((float(tick) - center) / denominator) - offset

    @staticmethod
    def _angle_offset_for_anchor_tick(
        measured_angle_deg: float,
        anchor_tick: int,
        center_tick: float,
        sign: float,
        ticks_per_degree: float,
        base_offset_deg: float,
    ) -> float:
        denominator = sign * ticks_per_degree
        if abs(denominator) < 0.0001:
            return 0.0
        anchored_angle_deg = (float(anchor_tick) - center_tick) / denominator
        return anchored_angle_deg - measured_angle_deg - base_offset_deg

    def _session_angle_offset(self, side: str, key: str) -> float:
        session = self.calibration.get("session_calibration", {})
        side_data = session.get(side, {}) if isinstance(session, dict) else {}
        if not isinstance(side_data, dict):
            return 0.0
        try:
            return float(side_data.get(key, 0.0))
        except (TypeError, ValueError):
            return 0.0

    @staticmethod
    def _normalize_vector(vector: tuple[float, float, float]) -> tuple[float, float, float]:
        length = math.sqrt(vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2])
        if length <= 0.0001:
            return 0.0, 0.0, 1.0
        return vector[0] / length, vector[1] / length, vector[2] / length

    def _bracket_vector(self, side: str) -> tuple[float, float, float]:
        return (
            self._calib_float(f"{side}_bracket_x_mm", -40.0 if side == "left" else 40.0),
            self._calib_float(f"{side}_bracket_y_mm", 0.0),
            self._calib_float(f"{side}_bracket_z_mm", 0.0),
        )

    def _tilt_pivot_offset_vector(self, side: str) -> tuple[float, float, float]:
        return (
            self._calib_float(f"{side}_tilt_pivot_offset_x_mm", 0.0),
            self._calib_float(f"{side}_tilt_pivot_offset_y_mm", 45.0),
            self._calib_float(f"{side}_tilt_pivot_offset_z_mm", 5.0),
        )

    def _tof_offset_vector(self, side: str) -> tuple[float, float, float]:
        return (
            self._calib_float(f"{side}_tof_offset_x_mm", 0.0),
            self._calib_float(f"{side}_tof_offset_y_mm", 0.0),
            self._calib_float(f"{side}_tof_offset_z_mm", 0.0),
        )

    def _point_for_tof_distance(
        self,
        side: str,
        ray: tuple[float, float, float],
        distance_mm: float,
    ) -> tuple[float, float, float]:
        point = (ray[0] * distance_mm, ray[1] * distance_mm, ray[2] * distance_mm)
        for _ in range(4):
            yaw_deg, pitch_deg = self._angles_for_point(side, point)
            sensor = self._tof_sensor_vector(side, math.radians(yaw_deg), math.radians(pitch_deg))
            point = self._point_on_ray_at_sensor_distance(ray, sensor, distance_mm)
        return point

    def _angles_for_point(self, side: str, point: tuple[float, float, float]) -> tuple[float, float]:
        pan_pivot = self._bracket_vector(side)
        target_from_pan = (
            point[0] - pan_pivot[0],
            point[1] - pan_pivot[1],
            point[2] - pan_pivot[2],
        )
        yaw_rad = math.atan2(target_from_pan[0], target_from_pan[2])
        target_in_pan_frame = self._yaw_global_to_local(target_from_pan, yaw_rad)
        tilt_pivot = self._tilt_pivot_offset_vector(side)
        target_from_tilt = (
            target_in_pan_frame[0] - tilt_pivot[0],
            target_in_pan_frame[1] - tilt_pivot[1],
            target_in_pan_frame[2] - tilt_pivot[2],
        )
        pitch_rad = math.atan2(
            target_from_tilt[1],
            math.hypot(target_from_tilt[0], target_from_tilt[2]),
        )
        return math.degrees(yaw_rad), math.degrees(pitch_rad)

    def _tof_sensor_vector(self, side: str, yaw_rad: float, pitch_rad: float) -> tuple[float, float, float]:
        pan_pivot = self._bracket_vector(side)
        tilt_pivot = self._tilt_pivot_offset_vector(side)
        tof_offset = self._pitch_rotate_vector(self._tof_offset_vector(side), pitch_rad)
        sensor_in_pan_frame = (
            tilt_pivot[0] + tof_offset[0],
            tilt_pivot[1] + tof_offset[1],
            tilt_pivot[2] + tof_offset[2],
        )
        sensor_global_offset = self._yaw_local_to_global(sensor_in_pan_frame, yaw_rad)
        return (
            pan_pivot[0] + sensor_global_offset[0],
            pan_pivot[1] + sensor_global_offset[1],
            pan_pivot[2] + sensor_global_offset[2],
        )

    def _point_on_ray_at_sensor_distance(
        self,
        ray: tuple[float, float, float],
        sensor: tuple[float, float, float],
        distance_mm: float,
    ) -> tuple[float, float, float]:
        distance_mm = max(1.0, float(distance_mm))
        dot = ray[0] * sensor[0] + ray[1] * sensor[1] + ray[2] * sensor[2]
        sensor_norm_sq = sensor[0] * sensor[0] + sensor[1] * sensor[1] + sensor[2] * sensor[2]
        discriminant = dot * dot - sensor_norm_sq + distance_mm * distance_mm
        if discriminant < 0.0:
            s = max(distance_mm, dot)
        else:
            root = math.sqrt(discriminant)
            s = dot + root
            if s <= 0.0:
                s = dot - root
            if s <= 0.0:
                s = distance_mm
        return ray[0] * s, ray[1] * s, ray[2] * s

    @staticmethod
    def _yaw_global_to_local(vector: tuple[float, float, float], yaw_rad: float) -> tuple[float, float, float]:
        cos_y = math.cos(yaw_rad)
        sin_y = math.sin(yaw_rad)
        x, y, z = vector
        return cos_y * x - sin_y * z, y, sin_y * x + cos_y * z

    @staticmethod
    def _yaw_local_to_global(vector: tuple[float, float, float], yaw_rad: float) -> tuple[float, float, float]:
        cos_y = math.cos(yaw_rad)
        sin_y = math.sin(yaw_rad)
        x, y, z = vector
        return cos_y * x + sin_y * z, y, -sin_y * x + cos_y * z

    @staticmethod
    def _pitch_rotate_vector(vector: tuple[float, float, float], pitch_rad: float) -> tuple[float, float, float]:
        cos_p = math.cos(pitch_rad)
        sin_p = math.sin(pitch_rad)
        x, y, z = vector
        return x, cos_p * y + sin_p * z, -sin_p * y + cos_p * z

    def _distance_mm_for_side(self, side: str, serial_state: dict[str, Any] | None) -> float:
        startup_distance = self._startup_user_distance_mm()
        raw_tof_mm = self._tof_mm_from_serial(side, serial_state)
        if raw_tof_mm is not None:
            alpha = max(0.0, min(1.0, self._calib_float("tof_depth_alpha", 0.45)))
            previous_raw = self._last_raw_tof_mm.get(side)
            smoothed_tof_mm = raw_tof_mm if previous_raw is None else previous_raw + alpha * (raw_tof_mm - previous_raw)
            self._last_raw_tof_mm[side] = smoothed_tof_mm
            if startup_distance is not None and self._use_startup_user_distance():
                live_weight = max(0.0, min(1.0, self._calib_float("startup_distance_live_weight", 0.35)))
                distance_mm = startup_distance + live_weight * (smoothed_tof_mm - startup_distance)
                source = "startup_user_distance_blend"
            else:
                live_weight = 1.0
                distance_mm = smoothed_tof_mm
                source = "tof"
            self._last_distance_mm[side] = distance_mm
            self._last_distance_debug[side] = {
                "source": source,
                "raw_tof_mm": raw_tof_mm,
                "smoothed_tof_mm": smoothed_tof_mm,
                "startup_user_distance_mm": startup_distance,
                "startup_distance_live_weight": live_weight,
                "effective_distance_mm": distance_mm,
            }
            return distance_mm

        if side in self._last_distance_mm:
            distance_mm = self._last_distance_mm[side]
            self._last_distance_debug[side] = {
                "source": "last_effective_distance",
                "raw_tof_mm": None,
                "smoothed_tof_mm": None,
                "startup_user_distance_mm": startup_distance,
                "startup_distance_live_weight": None,
                "effective_distance_mm": distance_mm,
            }
            return self._last_distance_mm[side]

        if startup_distance is not None and self._use_startup_user_distance():
            self._last_distance_mm[side] = startup_distance
            self._last_distance_debug[side] = {
                "source": "startup_user_distance",
                "raw_tof_mm": None,
                "smoothed_tof_mm": None,
                "startup_user_distance_mm": startup_distance,
                "startup_distance_live_weight": 0.0,
                "effective_distance_mm": startup_distance,
            }
            return startup_distance

        session_distance = self._session_calibration_distance(side)
        if session_distance is not None:
            self._last_distance_mm[side] = session_distance
            self._last_distance_debug[side] = {
                "source": "session_hand_distance",
                "raw_tof_mm": None,
                "smoothed_tof_mm": None,
                "startup_user_distance_mm": startup_distance,
                "startup_distance_live_weight": None,
                "effective_distance_mm": session_distance,
            }
            return session_distance

        distance_mm = self._calib_float("initial_hand_distance_mm", 700.0)
        self._last_distance_debug[side] = {
            "source": "initial_hand_distance",
            "raw_tof_mm": None,
            "smoothed_tof_mm": None,
            "startup_user_distance_mm": startup_distance,
            "startup_distance_live_weight": None,
            "effective_distance_mm": distance_mm,
        }
        return distance_mm

    def _tof_mm_from_serial(self, side: str, serial_state: dict[str, Any] | None) -> float | None:
        if not isinstance(serial_state, dict):
            return None
        devices = serial_state.get("devices", {})
        camdock = devices.get("camdock", {}) if isinstance(devices, dict) else {}
        tof = camdock.get("tof", {}) if isinstance(camdock, dict) else {}
        key = f"{side}_mm"
        return self._valid_tof_mm(tof.get(key) if isinstance(tof, dict) else None)

    def _session_calibration_distance(self, side: str) -> float | None:
        session = self.calibration.get("session_calibration", {})
        side_data = session.get(side, {}) if isinstance(session, dict) else {}
        if not isinstance(side_data, dict):
            return None
        return self._valid_tof_mm(side_data.get("tof_mm"))

    def _startup_user_distance_mm(self) -> float | None:
        session = self.calibration.get("session_calibration", {})
        if isinstance(session, dict):
            distance = self._valid_tof_mm(session.get("user_distance_mm"))
            if distance is not None:
                return distance

            side_distances = []
            for side in ("right", "left"):
                side_data = session.get(side, {})
                if not isinstance(side_data, dict):
                    continue
                side_distance = self._valid_tof_mm(side_data.get("tof_mm"))
                if side_distance is not None:
                    side_distances.append(side_distance)
            if side_distances:
                return sum(side_distances) / len(side_distances)
        return None

    def _use_startup_user_distance(self) -> bool:
        try:
            return int(float(self.calibration.get("use_startup_user_distance", 1))) != 0
        except (TypeError, ValueError):
            return True

    def _valid_tof_mm(self, value: Any) -> float | None:
        try:
            distance_mm = float(value)
        except (TypeError, ValueError):
            return None
        min_mm = self._calib_float("min_valid_tof_mm", 80.0)
        max_mm = self._calib_float("max_valid_tof_mm", 2000.0)
        if min_mm <= distance_mm <= max_mm:
            return distance_mm
        return None

    def _predicted_hand_position(self, side: str, x: float, y: float, now: float) -> tuple[float, float]:
        x = max(0.0, min(1.0, x))
        y = max(0.0, min(1.0, y))
        latency_s = max(0.0, self._calib_float("prediction_latency_ms", 50.0)) / 1000.0
        previous = self._hand_history.get(side)
        self._hand_history[side] = (now, x, y)
        if previous is None or latency_s <= 0.0:
            return x, y

        previous_time, previous_x, previous_y = previous
        dt = now - previous_time
        if dt <= 0.001 or dt > 0.5:
            return x, y

        predicted_x = x + ((x - previous_x) / dt) * latency_s
        predicted_y = y + ((y - previous_y) / dt) * latency_s
        return max(0.0, min(1.0, predicted_x)), max(0.0, min(1.0, predicted_y))

    def _send(
        self,
        active_pair: str,
        servos: dict[str, int],
        *,
        disable_unused: bool = True,
        coalesce: bool = False,
    ) -> bool:
        servo_values = {name: int(servos.get(name, 0)) for name in SERVO_FIELDS}
        command: dict[str, Any] = {
            "cmd": "servo",
            "target": "camdock",
            "active_pair": active_pair,
            "disable_unused": disable_unused,
            "servos": servo_values,
        }
        command.update(servo_values)
        if coalesce:
            try:
                return self.serial_bridge.send_command(command, coalesce_key="servo_auto")
            except TypeError:
                return self.serial_bridge.send_command(command)
        return self.serial_bridge.send_command(command)

    def _next_debug_sequence(self) -> int:
        self._debug_sequence += 1
        return self._debug_sequence

    def _record_bracket_ticks(
        self,
        active_pair: str,
        servos: dict[str, int],
        source: str,
        auto: bool,
    ) -> None:
        for bracket in self._brackets_for_active_pair(active_pair):
            pan_field, tilt_field = BRACKET_SERVO_FIELDS[bracket]
            entry = {
                "pan": int(servos.get(pan_field, 0)),
                "tilt": int(servos.get(tilt_field, 0)),
                "source": source,
                "debug_seq": self.last_debug_snapshot.get("seq"),
                "time_s": time.time(),
            }
            self._last_sent_bracket_ticks[bracket] = entry
            if auto:
                self._last_auto_bracket_ticks[bracket] = entry

    def _pulse_from_normalized(self, servo_name: str, center_key: str, value: float, axis: str) -> int:
        center = float(self.calibration.get(center_key, 307))
        gain_key = "x_gain_ticks" if axis == "x" else "y_gain_ticks"
        gain = float(self.calibration.get(gain_key, 100))
        value = max(0.0, min(1.0, value))
        offset = (value - 0.5) * 2.0 * gain
        if axis == "x":
            offset = -offset
        target = center + offset
        return self._smooth_and_clamp(servo_name, target)

    def _smooth_and_clamp_hand(self, servo_name: str, target: float) -> int:
        deadband = self._calib_float("hand_servo_deadband", 2.0)
        alpha = self._calib_float("hand_servo_smoothing_alpha", 0.62)
        return self._smooth_and_clamp(
            servo_name,
            target,
            deadband=deadband,
            alpha=alpha,
        )

    def _smooth_and_clamp(
        self,
        servo_name: str,
        target: float,
        *,
        deadband: float | None = None,
        alpha: float | None = None,
    ) -> int:
        if deadband is None:
            deadband = float(self.calibration.get("deadband", 4))
        if alpha is None:
            alpha = float(self.calibration.get("smoothing_alpha", 0.35))
        alpha = max(0.0, min(1.0, alpha))
        previous = self._smoothed.get(servo_name)
        if previous is None:
            smoothed = target
        elif abs(target - previous) <= deadband:
            smoothed = previous
        else:
            smoothed = previous + alpha * (target - previous)
        self._smoothed[servo_name] = smoothed
        return self._clamp_tick(round(smoothed))

    def _hand_coordinate_from_boundaries(self, value: float, axis: str) -> float:
        value = max(0.0, min(1.0, float(value)))
        if axis == "x":
            low_key, high_key = "hand_boundary_left", "hand_boundary_right"
        else:
            low_key, high_key = "hand_boundary_top", "hand_boundary_bottom"

        try:
            low = float(self.calibration.get(low_key, 0.0))
            high = float(self.calibration.get(high_key, 1.0))
        except (TypeError, ValueError):
            return value

        low, high = sorted((max(0.0, min(1.0, low)), max(0.0, min(1.0, high))))
        span = high - low
        if span < MIN_HAND_BOUNDARY_SPAN:
            return value
        return max(0.0, min(1.0, (value - low) / span))

    def _use_dock_geometry(self) -> bool:
        try:
            return int(float(self.calibration.get("use_dock_geometry", 1))) != 0
        except (TypeError, ValueError):
            return True

    def _calib_float(self, key: str, default: float) -> float:
        try:
            return float(self.calibration.get(key, default))
        except (TypeError, ValueError):
            return default

    def _clamp_tick(self, value: int) -> int:
        active_min_tick = max(1, self.servo_min_tick)
        return max(active_min_tick, min(self.servo_max_tick, int(value)))

    @staticmethod
    def _active_pair_from_sides(active_sides: list[str]) -> str:
        if "right" in active_sides and "left" in active_sides:
            return "hands"
        if "right" in active_sides:
            return "right"
        if "left" in active_sides:
            return "left"
        return "none"

    @staticmethod
    def _brackets_for_active_pair(active_pair: str) -> tuple[str, ...]:
        if active_pair == "hands":
            return "right", "left"
        if active_pair == "dock":
            return ("camera", "right", "left")
        if active_pair in BRACKET_SERVO_FIELDS:
            return (active_pair,)
        return ()

    @staticmethod
    def _hand_visible(values: dict[str, Any]) -> bool:
        return bool(values.get("visible") and values.get("x") is not None and values.get("y") is not None)

    @staticmethod
    def _center_keys_for_bracket(bracket: str) -> tuple[str, str]:
        if bracket == "right":
            return "r_pan_center", "r_tilt_center"
        if bracket == "left":
            return "l_pan_center", "l_tilt_center"
        if bracket == "camera":
            return "cam_pan_center", "cam_tilt_center"
        raise ValueError(f"Unknown servo bracket: {bracket}")
