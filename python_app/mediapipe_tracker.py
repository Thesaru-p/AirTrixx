from __future__ import annotations

import importlib
import threading
import time
from pathlib import Path
from typing import Any, Callable

import numpy as np

from app_paths import project_resource_path

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
HAND_LANDMARKER_MODEL_CANDIDATES = (
    project_resource_path("models", "hand_landmarker.task"),
    project_resource_path("packaging", "assets", "generated", "hand_landmarker.task"),
)
MIN_VISIBLE_FRAME_MEAN = 2.0
MIN_VISIBLE_FRAME_STD = 4.0
CAMERA_VALIDATION_FRAMES = 30
CAMERA_MIN_VISIBLE_VALIDATION_FRAMES = 5
CAMERA_CAPTURE_FPS = 30
CAMERA_BUFFER_SIZE = 1
HAND_CONNECTIONS = (
    (0, 1),
    (1, 2),
    (2, 3),
    (3, 4),
    (0, 5),
    (5, 6),
    (6, 7),
    (7, 8),
    (5, 9),
    (9, 10),
    (10, 11),
    (11, 12),
    (9, 13),
    (13, 14),
    (14, 15),
    (15, 16),
    (13, 17),
    (0, 17),
    (17, 18),
    (18, 19),
    (19, 20),
)


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


def _load_mediapipe_tasks_modules() -> tuple[Any, Any]:
    base_options_module = importlib.import_module("mediapipe.tasks.python")
    vision_module = importlib.import_module("mediapipe.tasks.python.vision")
    return base_options_module.BaseOptions, vision_module


def _find_hand_landmarker_model() -> Path | None:
    for candidate in HAND_LANDMARKER_MODEL_CANDIDATES:
        if candidate.exists():
            return candidate
    return None


def _draw_task_landmarks(frame_bgr: Any, landmarks: list[Any]) -> None:
    if cv2 is None or not landmarks:
        return
    frame_h, frame_w = frame_bgr.shape[:2]
    points: list[tuple[int, int]] = []
    for landmark in landmarks:
        x = int(max(0.0, min(1.0, float(landmark.x))) * frame_w)
        y = int(max(0.0, min(1.0, float(landmark.y))) * frame_h)
        points.append((x, y))
    for start, end in HAND_CONNECTIONS:
        if start < len(points) and end < len(points):
            cv2.line(frame_bgr, points[start], points[end], (64, 220, 160), 2)
    for point in points:
        cv2.circle(frame_bgr, point, 3, (255, 255, 255), -1)
        cv2.circle(frame_bgr, point, 4, (15, 118, 110), 1)


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
        tracking_frame_skip: int = 1,
        face_detection_enabled: bool = True,
        on_log: LogCallback | None = None,
    ) -> None:
        self.camera_index = camera_index
        self.width = max(1, int(width))
        self.height = max(1, int(height))
        self.tracking_frame_skip = max(0, int(tracking_frame_skip))
        self.face_detection_enabled = bool(face_detection_enabled)
        self.on_log = on_log
        self.mirror_preview = True
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()
        self._latest_hands = _empty_hand_state()
        self._latest_face = _empty_face_state()
        self._selected_face: dict[str, Any] | None = None
        self._latest_frame_rgb: np.ndarray | None = None
        self._running_requested = False
        self._restart_generation = 0

    def _log(self, message: str) -> None:
        if self.on_log:
            self.on_log(message)

    def start(self) -> None:
        self._running_requested = True
        if self._thread and self._thread.is_alive():
            return
        if cv2 is None or mp is None:
            self._log("OpenCV or MediaPipe is missing. Run pip install -r requirements.txt.")
            return
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running_requested = False
        self._stop_event.set()
        thread = self._thread
        if thread and thread.is_alive() and thread is not threading.current_thread():
            thread.join(timeout=1.0)

    def configure(
        self,
        camera_index: int | None = None,
        mirror_preview: bool | None = None,
        width: int | None = None,
        height: int | None = None,
        tracking_frame_skip: int | None = None,
        face_detection_enabled: bool | None = None,
    ) -> None:
        new_width = self.width if width is None else max(1, int(width))
        new_height = self.height if height is None else max(1, int(height))
        restart = bool(
            (camera_index is not None and camera_index != self.camera_index)
            or new_width != self.width
            or new_height != self.height
        )
        if mirror_preview is not None:
            self.mirror_preview = bool(mirror_preview)
        if camera_index is not None:
            self.camera_index = int(camera_index)
        self.width = new_width
        self.height = new_height
        if tracking_frame_skip is not None:
            self.tracking_frame_skip = max(0, min(8, int(tracking_frame_skip)))
        if face_detection_enabled is not None:
            self.face_detection_enabled = bool(face_detection_enabled)
        if restart:
            was_running = self._running_requested or bool(self._thread and self._thread.is_alive())
            self._restart_generation += 1
            restart_generation = self._restart_generation
            previous_thread = self._thread
            self._stop_event.set()
            with self._lock:
                self._latest_hands = _empty_hand_state()
                self._latest_face = _empty_face_state()
                self._selected_face = None
                self._latest_frame_rgb = None
            if was_running:
                threading.Thread(
                    target=self._restart_when_stopped,
                    args=(previous_thread, restart_generation),
                    daemon=True,
                ).start()

    def _restart_when_stopped(
        self,
        previous_thread: threading.Thread | None,
        restart_generation: int,
    ) -> None:
        if previous_thread and previous_thread.is_alive() and previous_thread is not threading.current_thread():
            previous_thread.join()
        if not self._running_requested:
            return
        if restart_generation != self._restart_generation:
            return
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

    def has_latest_frame(self) -> bool:
        with self._lock:
            return self._latest_frame_rgb is not None

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
        trial_indices: list[int] = []
        if preferred_index >= 0:
            trial_indices.append(preferred_index)
        for fallback_index in (0, 1, 2, 3):
            if fallback_index not in trial_indices:
                trial_indices.append(fallback_index)

        for camera_index in trial_indices:
            cap = cv2.VideoCapture(camera_index, cv2.CAP_DSHOW)
            if not cap.isOpened():
                cap.release()
                cap = cv2.VideoCapture(camera_index)
            if cap.isOpened():
                try:
                    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
                    cap.set(cv2.CAP_PROP_FPS, CAMERA_CAPTURE_FPS)
                    cap.set(cv2.CAP_PROP_BUFFERSIZE, CAMERA_BUFFER_SIZE)
                except Exception:
                    pass
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
                if self._capture_has_visible_frame(cap):
                    return cap, camera_index
                self._log(f"Camera index {camera_index} opened but produced only black frames; trying fallback.")
                cap.release()
                continue
            cap.release()
        return None, None

    @staticmethod
    def _capture_has_visible_frame(cap: Any) -> bool:
        visible_frames = 0
        for _ in range(CAMERA_VALIDATION_FRAMES):
            ok, frame = cap.read()
            if not ok or frame is None:
                time.sleep(0.03)
                continue
            frame_mean = float(np.mean(frame))
            frame_std = float(np.std(frame))
            if frame_mean >= MIN_VISIBLE_FRAME_MEAN or frame_std >= MIN_VISIBLE_FRAME_STD:
                visible_frames += 1
                if visible_frames >= CAMERA_MIN_VISIBLE_VALIDATION_FRAMES:
                    return True
            time.sleep(0.03)
        return False

    def _run(self) -> None:
        requested_camera_index = self.camera_index
        mode = "legacy"
        legacy_error: Exception | None = None
        mp_hands = None
        mp_drawing = None
        mp_styles = None
        BaseOptions = None
        vision = None
        model_path: Path | None = None
        try:
            mp_hands, mp_drawing, mp_styles = _load_mediapipe_solution_modules()
        except Exception as exc:
            legacy_error = exc
            try:
                BaseOptions, vision = _load_mediapipe_tasks_modules()
                model_path = _find_hand_landmarker_model()
                mode = "tasks"
            except Exception as task_exc:
                version = getattr(mp, "__version__", "unknown") if mp else "not installed"
                location = getattr(mp, "__file__", "unknown") if mp else "unknown"
                self._log(
                    "MediaPipe Hands could not be loaded. "
                    f"Installed mediapipe version={version}, path={location}. "
                    f"Legacy error: {type(exc).__name__}: {exc}. "
                    f"Tasks error: {type(task_exc).__name__}: {task_exc}. "
                    "Camera preview will continue without hand tracking."
                )
                mode = "preview"
            if mode == "tasks" and model_path is None:
                self._log(
                    "MediaPipe Tasks is installed, but hand_landmarker.task is missing. "
                    "Run packaging/download_models.py before packaging or source runs. "
                    "Camera preview will continue without hand tracking."
                )
                mode = "preview"

        cap, active_camera_index = self._open_camera_capture(requested_camera_index)
        if cap is None or active_camera_index is None:
            self._log(f"Could not open camera index {requested_camera_index} or fallback indices 0-3.")
            return

        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        face_cascade = self._load_face_cascade()

        def process_legacy(frame_rgb: Any, frame_bgr: Any, hands: Any) -> dict[str, dict[str, Any]]:
            state = _empty_hand_state()
            results = hands.process(frame_rgb)
            if results.multi_hand_landmarks and results.multi_handedness:
                for landmarks, handedness in zip(
                    results.multi_hand_landmarks,
                    results.multi_handedness,
                ):
                    classification = handedness.classification[0]
                    side = classification.label.lower()
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
            return state

        def process_tasks(frame_rgb: Any, frame_bgr: Any, landmarker: Any) -> dict[str, dict[str, Any]]:
            state = _empty_hand_state()
            mp_image = mp.Image(
                image_format=mp.ImageFormat.SRGB,
                data=np.ascontiguousarray(frame_rgb),
            )
            results = landmarker.detect(mp_image)
            if results.hand_landmarks and results.handedness:
                for landmarks, handedness in zip(results.hand_landmarks, results.handedness):
                    if not handedness:
                        continue
                    classification = handedness[0]
                    side = str(classification.category_name).lower()
                    if side == "right":
                        side = "left"
                    elif side == "left":
                        side = "right"
                    if side not in state:
                        continue
                    xs = [lm.x for lm in landmarks]
                    ys = [lm.y for lm in landmarks]
                    score = float(classification.score)
                    if score >= state[side]["score"]:
                        state[side] = {
                            "visible": True,
                            "x": float(sum(xs) / len(xs)),
                            "y": float(sum(ys) / len(ys)),
                            "score": score,
                            "gesture": classify_hand_gesture(landmarks),
                        }
                    _draw_task_landmarks(frame_bgr, landmarks)
            return state

        def run_loop(detector: Any, detector_label: str) -> None:
            self._log(
                f"{detector_label} hand tracker started on camera index {active_camera_index} "
                f"(requested {requested_camera_index}, {self.width}x{self.height}, "
                f"tracking frame skip {self.tracking_frame_skip})."
            )
            frame_index = 0
            last_hand_state = _empty_hand_state()
            while not self._stop_event.is_set():
                ok, frame_bgr = cap.read()
                if not ok:
                    time.sleep(0.03)
                    continue

                frame_skip = max(0, int(self.tracking_frame_skip))
                process_hands = frame_index % (frame_skip + 1) == 0
                frame_index += 1
                if process_hands and mode == "legacy":
                    frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
                    state = process_legacy(frame_rgb, frame_bgr, detector)
                    last_hand_state = state
                elif process_hands and mode == "tasks":
                    frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
                    state = process_tasks(frame_rgb, frame_bgr, detector)
                    last_hand_state = state
                else:
                    state = {side: dict(values) for side, values in last_hand_state.items()}
                face_state = _empty_face_state()

                if face_cascade is not None and self.face_detection_enabled:
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
                elif not self.face_detection_enabled:
                    self._selected_face = None

                annotated_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
                if self.mirror_preview:
                    annotated_rgb = np.ascontiguousarray(np.flip(annotated_rgb, axis=1))
                with self._lock:
                    self._latest_hands = state
                    self._latest_face = face_state
                    self._latest_frame_rgb = annotated_rgb

        try:
            if mode == "legacy":
                with mp_hands.Hands(
                    static_image_mode=False,
                    max_num_hands=2,
                    model_complexity=0,
                    min_detection_confidence=0.55,
                    min_tracking_confidence=0.45,
                ) as hands:
                    run_loop(hands, "MediaPipe Solutions")
            elif mode == "tasks":
                if legacy_error:
                    self._log(f"MediaPipe Solutions unavailable; using MediaPipe Tasks ({type(legacy_error).__name__}).")
                options = vision.HandLandmarkerOptions(
                    base_options=BaseOptions(model_asset_path=str(model_path)),
                    running_mode=vision.RunningMode.IMAGE,
                    num_hands=2,
                    min_hand_detection_confidence=0.55,
                    min_hand_presence_confidence=0.45,
                    min_tracking_confidence=0.45,
                )
                with vision.HandLandmarker.create_from_options(options) as landmarker:
                    run_loop(landmarker, "MediaPipe Tasks")
            else:
                run_loop(None, "Preview-only")
        finally:
            cap.release()
        self._log("MediaPipe hand tracker stopped.")
