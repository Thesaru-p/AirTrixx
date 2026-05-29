from __future__ import annotations

import importlib
import threading
import time
from typing import Any, Callable

import numpy as np

try:
    import cv2
except Exception:  # pragma: no cover
    cv2 = None

try:
    import mediapipe as mp
except Exception:  # pragma: no cover
    mp = None


LogCallback = Callable[[str], None]

FACE_SWITCH_AREA_RATIO = 1.25
FACE_LOCK_MAX_CENTER_DISTANCE = 0.28
FACE_SMOOTHING_ALPHA = 0.35


def _empty_hand_state() -> dict[str, dict[str, Any]]:
    return {
        "right": {"visible": False, "x": None, "y": None, "score": 0.0, "gesture": "none"},
        "left": {"visible": False, "x": None, "y": None, "score": 0.0, "gesture": "none"},
    }


def _empty_face_state() -> dict[str, Any]:
    return {
        "visible": False,
        "x": None,
        "y": None,
        "top_y": None,
        "width": None,
        "height": None,
        "score": 0.0,
    }


def _load_mediapipe_solution_modules() -> tuple[Any, Any, Any]:
    """Load the classic MediaPipe Hands solution across package layouts."""
    candidates = [
        (
            "mediapipe.solutions.hands",
            "mediapipe.solutions.drawing_utils",
            "mediapipe.solutions.drawing_styles",
        ),
        (
            "mediapipe.python.solutions.hands",
            "mediapipe.python.solutions.drawing_utils",
            "mediapipe.python.solutions.drawing_styles",
        ),
    ]
    last_error: Exception | None = None
    for hands_name, drawing_name, styles_name in candidates:
        try:
            return (
                importlib.import_module(hands_name),
                importlib.import_module(drawing_name),
                importlib.import_module(styles_name),
            )
        except Exception as exc:
            last_error = exc
    if last_error:
        raise last_error
    raise ImportError("MediaPipe Hands solution modules were not found.")


def _finger_is_extended(landmarks: Any, tip_index: int, pip_index: int) -> bool:
    return landmarks[tip_index].y < landmarks[pip_index].y - 0.025


def classify_hand_gesture(landmarks: Any) -> str:
    index_up = _finger_is_extended(landmarks, 8, 6)
    middle_up = _finger_is_extended(landmarks, 12, 10)
    ring_up = _finger_is_extended(landmarks, 16, 14)
    pinky_up = _finger_is_extended(landmarks, 20, 18)
    extended_count = sum([index_up, middle_up, ring_up, pinky_up])

    if extended_count >= 4:
        return "open_palm"
    if index_up and not middle_up and not ring_up and not pinky_up:
        return "index_finger_up"
    if extended_count == 0:
        return "closed_fist"
    return "unknown"


class HandTracker:
    def __init__(
        self,
        camera_index: int = 0,
        width: int = 640,
        height: int = 480,
        on_log: LogCallback | None = None,
    ) -> None:
        self.camera_index = camera_index
        self.width = width
        self.height = height
        self.on_log = on_log
        self.mirror_preview = True
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()
        self._latest_hands = _empty_hand_state()
        self._latest_face = _empty_face_state()
        self._selected_face: dict[str, Any] | None = None
        self._latest_frame_rgb: np.ndarray | None = None

    def _log(self, message: str) -> None:
        if self.on_log:
            self.on_log(message)

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        if cv2 is None or mp is None:
            self._log("OpenCV or MediaPipe is missing. Run pip install -r requirements.txt.")
            return
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        thread = self._thread
        if thread and thread.is_alive() and thread is not threading.current_thread():
            thread.join(timeout=1.0)

    def configure(self, camera_index: int | None = None, mirror_preview: bool | None = None) -> None:
        restart = camera_index is not None and camera_index != self.camera_index
        if mirror_preview is not None:
            self.mirror_preview = bool(mirror_preview)
        if camera_index is not None:
            self.camera_index = int(camera_index)
        if restart:
            was_running = bool(self._thread and self._thread.is_alive())
            self.stop()
            with self._lock:
                self._latest_hands = _empty_hand_state()
                self._latest_face = _empty_face_state()
                self._selected_face = None
                self._latest_frame_rgb = None
            if was_running:
                self._stop_event.clear()
                self.start()

    def get_latest_hands(self) -> dict[str, dict[str, Any]]:
        with self._lock:
            return {
                side: dict(values)
                for side, values in self._latest_hands.items()
            }

    def get_latest_frame_rgb(self) -> np.ndarray | None:
        with self._lock:
            if self._latest_frame_rgb is None:
                return None
            return self._latest_frame_rgb.copy()

    def get_latest_face(self) -> dict[str, Any]:
        with self._lock:
            return dict(self._latest_face)

    def _load_face_cascade(self) -> Any:
        if cv2 is None:
            return None
        try:
            cascade_path = cv2.data.haarcascades + "haarcascade_frontalface_default.xml"
        except Exception:
            return None
        cascade = cv2.CascadeClassifier(cascade_path)
        if cascade.empty():
            return None
        return cascade

    def _select_front_face(
        self,
        faces: Any,
        frame_w: int,
        frame_h: int,
    ) -> tuple[dict[str, Any] | None, tuple[int, int, int, int] | None]:
        candidates: list[tuple[float, dict[str, Any], tuple[int, int, int, int]]] = []
        for face in faces:
            x, y, w, h = (int(value) for value in face)
            area = float(w * h)
            state = {
                "visible": True,
                "x": float((x + w / 2) / frame_w),
                "y": float((y + h / 2) / frame_h),
                "top_y": float(y / frame_h),
                "width": float(w / frame_w),
                "height": float(h / frame_h),
                "score": area,
                "faces_seen": int(len(faces)),
            }
            candidates.append((area, state, (x, y, w, h)))

        if not candidates:
            self._selected_face = None
            return None, None

        largest_area, largest_state, largest_rect = max(candidates, key=lambda item: item[0])
        selected_area = largest_area
        selected_state = largest_state
        selected_rect = largest_rect
        selected_is_previous = False

        previous = self._selected_face
        if previous and previous.get("visible"):
            previous_x = previous.get("x")
            previous_y = previous.get("y")
            if previous_x is not None and previous_y is not None:
                previous_candidate = min(
                    candidates,
                    key=lambda item: self._face_center_distance(item[1], previous),
                )
                previous_distance = self._face_center_distance(previous_candidate[1], previous)
                previous_area = previous_candidate[0]
                if (
                    previous_distance <= FACE_LOCK_MAX_CENTER_DISTANCE
                    and largest_area <= previous_area * FACE_SWITCH_AREA_RATIO
                ):
                    selected_area, selected_state, selected_rect = previous_candidate
                    selected_is_previous = True

        if previous and selected_is_previous:
            selected_state = self._smooth_face_state(previous, selected_state)
            selected_state["score"] = selected_area
            selected_state["faces_seen"] = int(len(faces))

        self._selected_face = dict(selected_state)
        return selected_state, selected_rect

    @staticmethod
    def _face_center_distance(face: dict[str, Any], previous: dict[str, Any]) -> float:
        try:
            dx = float(face.get("x")) - float(previous.get("x"))
            dy = float(face.get("y")) - float(previous.get("y"))
        except (TypeError, ValueError):
            return 999.0
        return float((dx * dx + dy * dy) ** 0.5)

    @staticmethod
    def _smooth_face_state(previous: dict[str, Any], selected: dict[str, Any]) -> dict[str, Any]:
        smoothed = dict(selected)
        alpha = FACE_SMOOTHING_ALPHA
        for key in ("x", "y", "top_y", "width", "height"):
            previous_value = previous.get(key)
            selected_value = selected.get(key)
            if previous_value is None or selected_value is None:
                continue
            try:
                smoothed[key] = float(previous_value) + alpha * (
                    float(selected_value) - float(previous_value)
                )
            except (TypeError, ValueError):
                pass
        return smoothed

    def _open_camera_capture(self, preferred_index: int) -> tuple[Any | None, int | None]:
        # Try the selected camera first, then a few common fallback indices.
        trial_indices: list[int] = [preferred_index]
        for fallback_index in (0, 1, 2, 3):
            if fallback_index not in trial_indices:
                trial_indices.append(fallback_index)

        for camera_index in trial_indices:
            cap = cv2.VideoCapture(camera_index, cv2.CAP_DSHOW)
            if not cap.isOpened():
                cap.release()
                cap = cv2.VideoCapture(camera_index)
            if cap.isOpened():
                return cap, camera_index
            cap.release()
        return None, None

    def _run(self) -> None:
        requested_camera_index = self.camera_index
        try:
            mp_hands, mp_drawing, mp_styles = _load_mediapipe_solution_modules()
        except Exception as exc:
            version = getattr(mp, "__version__", "unknown") if mp else "not installed"
            location = getattr(mp, "__file__", "unknown") if mp else "unknown"
            self._log(
                "MediaPipe Hands could not be loaded. "
                f"Installed mediapipe version={version}, path={location}. "
                f"Error: {type(exc).__name__}: {exc}"
            )
            return

        cap, active_camera_index = self._open_camera_capture(requested_camera_index)
        if cap is None or active_camera_index is None:
            self._log(f"Could not open camera index {requested_camera_index} or fallback indices 0-3.")
            return

        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        face_cascade = self._load_face_cascade()

        with mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=2,
            model_complexity=0,
            min_detection_confidence=0.55,
            min_tracking_confidence=0.45,
        ) as hands:
            self._log(
                f"MediaPipe hand tracker started on camera index {active_camera_index} "
                f"(requested {requested_camera_index})."
            )
            while not self._stop_event.is_set():
                ok, frame_bgr = cap.read()
                if not ok:
                    time.sleep(0.03)
                    continue

                frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
                results = hands.process(frame_rgb)
                state = _empty_hand_state()
                face_state = _empty_face_state()

                if results.multi_hand_landmarks and results.multi_handedness:
                    for landmarks, handedness in zip(
                        results.multi_hand_landmarks,
                        results.multi_handedness,
                    ):
                        classification = handedness.classification[0]
                        side = classification.label.lower()
                        # MediaPipe Hands reports handedness for mirrored selfie
                        # input. This app tracks an unmirrored camera frame, so
                        # swap labels before servo control consumes them.
                        if side == "right":
                            side = "left"
                        elif side == "left":
                            side = "right"
                        if side not in state:
                            continue
                        landmark_list = landmarks.landmark
                        xs = [lm.x for lm in landmark_list]
                        ys = [lm.y for lm in landmark_list]
                        score = float(classification.score)
                        if score >= state[side]["score"]:
                            state[side] = {
                                "visible": True,
                                "x": float(sum(xs) / len(xs)),
                                "y": float(sum(ys) / len(ys)),
                                "score": score,
                                "gesture": classify_hand_gesture(landmark_list),
                            }
                        mp_drawing.draw_landmarks(
                            frame_bgr,
                            landmarks,
                            mp_hands.HAND_CONNECTIONS,
                            mp_styles.get_default_hand_landmarks_style(),
                            mp_styles.get_default_hand_connections_style(),
                        )

                if face_cascade is not None:
                    gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
                    faces = face_cascade.detectMultiScale(
                        gray,
                        scaleFactor=1.1,
                        minNeighbors=5,
                        minSize=(60, 60),
                    )
                    if len(faces) > 0:
                        frame_h, frame_w = frame_bgr.shape[:2]
                        selected_face, selected_rect = self._select_front_face(faces, frame_w, frame_h)
                        if selected_face is not None and selected_rect is not None:
                            face_state = selected_face
                            sx, sy, sw, sh = selected_rect
                            for x, y, w, h in faces:
                                color = (100, 100, 100)
                                thickness = 1
                                if (
                                    int(x) == sx
                                    and int(y) == sy
                                    and int(w) == sw
                                    and int(h) == sh
                                ):
                                    color = (60, 220, 120)
                                    thickness = 2
                                cv2.rectangle(
                                    frame_bgr,
                                    (int(x), int(y)),
                                    (int(x + w), int(y + h)),
                                    color,
                                    thickness,
                                )
                    else:
                        self._selected_face = None

                annotated_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
                if self.mirror_preview:
                    annotated_rgb = np.ascontiguousarray(np.flip(annotated_rgb, axis=1))
                with self._lock:
                    self._latest_hands = state
                    self._latest_face = face_state
                    self._latest_frame_rgb = annotated_rgb

        cap.release()
        self._log("MediaPipe hand tracker stopped.")
