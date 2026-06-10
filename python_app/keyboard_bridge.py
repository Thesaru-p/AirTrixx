from __future__ import annotations

import copy
import queue
import re
import shutil
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable

try:
    import serial
    from serial.tools import list_ports
except Exception:  # pragma: no cover - handled at runtime for missing dependency
    serial = None
    list_ports = None

from app_paths import project_resource_path
from keyboard_model import (
    EXPECTED_BOARD_COLUMNS,
    MAX_DETECT_MM,
    MM_COLUMNS,
    RAW_COLUMNS,
    RawFingerDetector,
    append_sample_rows,
    extract_feature,
    format_max_detect_mm,
    load_model,
    parse_board_row,
    parse_detect_limits,
    predict_knn,
    remove_sample_rows,
    sample_frame_counts,
    train_model,
    write_collection_metadata,
)


LogCallback = Callable[[str], None]
StatusCallback = Callable[[str], None]
PredictionCallback = Callable[[str, float, list[tuple[str, float]]], None]
TrainingCallback = Callable[[dict[str, Any]], None]

COMMAND_WORDS = ("space", "return", "backspace", "capslock")
WORD_PATTERN = re.compile(r"[a-z]+")


@dataclass
class _LiveCapture:
    detector: RawFingerDetector
    rows: list[dict[str, str]] = field(default_factory=list)
    pre_roll: deque[dict[str, str]] = field(default_factory=lambda: deque(maxlen=15))
    started: bool = False
    idle_started_s: float | None = None


@dataclass
class _TrainingCapture:
    sample_id: str
    word: str
    repetition: int
    detector: RawFingerDetector
    wait_started_s: float
    rows: list[dict[str, str | int]] = field(default_factory=list)
    gesture_started_s: float | None = None
    idle_started_s: float | None = None


@dataclass
class _TrainingSession:
    words: list[str]
    repetitions: int
    queue: list[tuple[str, int]]
    index: int = 0
    completed: int = 0

    @property
    def total(self) -> int:
        return len(self.queue)

    def next_item(self) -> tuple[str, int] | None:
        if self.index >= len(self.queue):
            return None
        item = self.queue[self.index]
        self.index += 1
        return item


class KeyboardBridge:
    def __init__(
        self,
        *,
        dataset_path: Path,
        model_path: Path,
        words_path: Path,
        baud_rate: int = 115200,
        on_log: LogCallback | None = None,
        on_status: StatusCallback | None = None,
        on_prediction: PredictionCallback | None = None,
        on_training: TrainingCallback | None = None,
    ) -> None:
        self.dataset_path = dataset_path
        self.model_path = model_path
        self.words_path = words_path
        self.baud_rate = baud_rate
        self.on_log = on_log
        self.on_status = on_status
        self.on_prediction = on_prediction
        self.on_training = on_training

        self._serial = None
        self._serial_lock = threading.RLock()
        self._latest_lock = threading.Lock()
        self._reader_thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._current_port: str | None = None
        self._write_queue: queue.Queue[str] = queue.Queue(maxsize=128)
        self._writer_thread = threading.Thread(target=self._write_loop, daemon=True)
        self._writer_thread.start()

        self.status = "Waiting for Antenna"
        self.live_prediction_enabled = True
        self.logging_enabled = False
        self.latest_prediction = ""
        self.latest_confidence = 0.0
        self.latest_ranking: list[tuple[str, float]] = []
        self.prediction_sequence = 0
        self.last_prediction_s = 0.0
        self.detect_limits: list[float] | None = None
        self.latest_distances_mm: dict[str, int | None] = {f"sensor_{index}_mm": None for index in range(1, 5)}
        self.latest_valid: dict[str, bool] = {f"sensor_{index}": False for index in range(1, 5)}
        self.model: dict[str, Any] | None = None
        self.model_error = ""
        self.model_words: list[str] = []
        self._live_capture: _LiveCapture | None = None
        self._training_session: _TrainingSession | None = None
        self._training_capture: _TrainingCapture | None = None
        self._training_status = "Idle"
        self._training_lock = threading.Lock()
        self._train_thread: threading.Thread | None = None
        self._last_training_prompt = ""
        self._antenna_last_sequence: int | None = None
        self._antenna_last_frame_s = 0.0
        self._antenna_frame_counter = 0
        self._antenna_logged = False

        self._ensure_starter_model()
        self.reload_model()

    @staticmethod
    def available_ports() -> list[dict[str, str]]:
        if list_ports is None:
            return []
        return [
            {
                "device": port.device,
                "description": port.description or "",
                "hwid": port.hwid or "",
            }
            for port in list_ports.comports()
        ]

    @property
    def is_connected(self) -> bool:
        with self._serial_lock:
            return bool(self._serial and self._serial.is_open)

    @property
    def current_port(self) -> str | None:
        return self._current_port

    @property
    def antenna_active(self) -> bool:
        return self._antenna_stream_active()

    @property
    def source_ready(self) -> bool:
        return self.is_connected or self._antenna_stream_active()

    @property
    def training_status(self) -> str:
        return self._training_status

    def connect(self, port: str | None = None) -> bool:
        if serial is None:
            self._log("pyserial is not installed. Run pip install -r requirements.txt.")
            return False
        if self.is_connected:
            return True

        candidates = [port] if port else [p["device"] for p in self.available_ports()]
        candidates = [candidate for candidate in candidates if candidate]
        if not candidates:
            self._log("No keyboard COM ports found.")
            return False

        for candidate in candidates:
            try:
                connection = serial.Serial(candidate, self.baud_rate, timeout=0.1, write_timeout=0.5)
            except Exception as exc:
                self._log(f"Failed to open keyboard {candidate}: {exc}")
                continue
            with self._serial_lock:
                self._serial = connection
            self._current_port = candidate
            self._stop_event.clear()
            self._reader_thread = threading.Thread(target=self._read_loop, daemon=True)
            self._reader_thread.start()
            self._set_status("Connected")
            self._log(f"Connected to keyboard on {candidate} at {self.baud_rate} baud.")
            self.send_command("help")
            self.send_command("limits")
            self.send_command("label live")
            self.send_command("ai on")
            return True

        self._set_status("Disconnected")
        return False

    def disconnect(self) -> None:
        self._stop_event.set()
        self.cancel_training()
        self.send_command("ai off")
        with self._serial_lock:
            connection = self._serial
            self._serial = None
        if connection:
            try:
                connection.close()
            except Exception:
                pass
        if self._reader_thread and self._reader_thread.is_alive() and self._reader_thread is not threading.current_thread():
            self._reader_thread.join(timeout=0.5)
        self._current_port = None
        self._live_capture = None
        self._set_status("Antenna ESP-NOW" if self._antenna_stream_active() else "Waiting for Antenna")
        self._log("Keyboard USB disconnected; waiting for Antenna ESP-NOW data.")

    def reset_for_calibration(self) -> bool:
        with self._serial_lock:
            connection = self._serial
        if not connection or not connection.is_open:
            self._log("Keyboard is not connected.")
            return False
        try:
            self._log("Resetting keyboard for cover and idle calibration.")
            connection.dtr = False
            connection.rts = True
            time.sleep(0.15)
            connection.rts = False
            time.sleep(0.2)
            return True
        except Exception as exc:
            self._log(f"Keyboard reset failed: {exc}")
            return False

    def send_command(self, command: str) -> bool:
        command = str(command).strip()
        if not command:
            return False
        with self._serial_lock:
            if not self._serial or not self._serial.is_open:
                return False
        try:
            self._write_queue.put_nowait(command + "\n")
            return True
        except queue.Full:
            self._log("Keyboard write queue is full; dropping command.")
            return False

    def set_live_prediction_enabled(self, enabled: bool) -> None:
        self.live_prediction_enabled = bool(enabled)
        self._live_capture = None
        if enabled and self.is_connected:
            self.send_command("label live")
            self.send_command("ai on")

    def reload_model(self) -> bool:
        if not self.model_path.exists():
            self.model = None
            self.model_words = []
            self.model_error = f"Missing model: {self.model_path}"
            return False
        try:
            self.model = load_model(self.model_path)
        except Exception as exc:
            self.model = None
            self.model_words = []
            self.model_error = str(exc)
            self._log(f"Could not load keyboard model: {exc}")
            return False
        self.model_words = sorted({str(word) for word in self.model["words"]})
        self.model_error = ""
        self._log(f"Loaded keyboard model with words: {', '.join(self.model_words)}")
        return True

    def start_training(
        self,
        words: list[str],
        *,
        repetitions: int,
        include_command_words: bool = True,
        reset_dataset: bool = False,
    ) -> bool:
        cleaned = self._clean_words(words, include_command_words=include_command_words)
        if not cleaned:
            self._log("Add at least one keyboard word before training.")
            return False
        if repetitions < 1:
            self._log("Keyboard training needs at least one sample per word.")
            return False
        if not self.source_ready:
            self._log("Connect the Antenna and wait for keyboard ESP-NOW data before training.")
            return False

        with self._training_lock:
            if reset_dataset:
                for path in (self.dataset_path, self.dataset_path.with_suffix(self.dataset_path.suffix + ".meta.json")):
                    try:
                        path.unlink(missing_ok=True)
                    except OSError as exc:
                        self._log(f"Could not remove {path}: {exc}")
            self.words_path.parent.mkdir(parents=True, exist_ok=True)
            self.words_path.write_text("\n".join(cleaned) + "\n", encoding="utf-8")
            queue_items = [(word, repetition) for word in cleaned for repetition in range(1, repetitions + 1)]
            self._training_session = _TrainingSession(cleaned, repetitions, queue_items)
            self._training_capture = None
            self._training_status = f"Ready: {len(cleaned)} words x {repetitions} samples"
        self._emit_training()
        self._log(f"Keyboard training plan: {len(cleaned)} word(s), {repetitions} sample(s) each.")
        return True

    def arm_next_training_sample(self) -> bool:
        if not self.source_ready:
            self._log("Connect the Antenna and wait for keyboard ESP-NOW data before recording samples.")
            return False
        with self._training_lock:
            if self._training_capture is not None:
                self._log("Keyboard sample is already armed.")
                return False
            if self._training_session is None:
                self._log("Start a keyboard training plan first.")
                return False
            item = self._training_session.next_item()
            if item is None:
                self._training_status = "All samples collected. Train the model next."
                self._emit_training()
                return False
            word, repetition = item
            sample_id = f"{word}__{repetition:02d}"
            self._training_capture = _TrainingCapture(
                sample_id=sample_id,
                word=word,
                repetition=repetition,
                detector=RawFingerDetector(max_detect_mm=self._active_detect_limits()),
                wait_started_s=time.monotonic(),
            )
            self._last_training_prompt = f"Swipe '{word}' ({repetition}/{self._training_session.repetitions})"
            self._training_status = self._last_training_prompt
        if self.is_connected:
            self.send_command(f"label {sample_id}")
            self.send_command("ai on")
        self._emit_training()
        self._log(f"Armed keyboard sample {sample_id}.")
        return True

    def cancel_training(self) -> None:
        with self._training_lock:
            self._training_capture = None
            self._training_session = None
            self._training_status = "Idle"
        self._emit_training()

    def train_model_async(self) -> bool:
        if self._train_thread and self._train_thread.is_alive():
            self._log("Keyboard model training is already running.")
            return False
        if not self.dataset_path.exists():
            self._log("No keyboard dataset yet. Record samples first.")
            return False
        self._train_thread = threading.Thread(target=self._train_model_worker, daemon=True)
        self._train_thread.start()
        return True

    def snapshot(self, *, input_max_age_s: float = 1.2) -> dict[str, Any]:
        with self._latest_lock:
            latest_prediction = self.latest_prediction
            latest_confidence = self.latest_confidence
            latest_ranking = list(self.latest_ranking)
            prediction_sequence = self.prediction_sequence
            last_prediction_s = self.last_prediction_s
            distances = dict(self.latest_distances_mm)
            valid = dict(self.latest_valid)
            detect_limits = list(self.detect_limits) if self.detect_limits else None
        now_s = time.monotonic()
        input_value = latest_prediction if latest_prediction and now_s - last_prediction_s <= input_max_age_s else None
        antenna_active = self._antenna_stream_active(now_s=now_s)
        source = "antenna_espnow" if antenna_active else "usb" if self.is_connected else "none"
        top_matches = [
            {"word": word, "confidence": confidence}
            for word, confidence in latest_ranking[:3]
        ]
        return {
            "status": self.status,
            "app_connected": antenna_active or self.is_connected,
            "antenna_active": antenna_active,
            "source": source,
            "port": self.current_port,
            "input": input_value,
            "predicted_word": latest_prediction or None,
            "prediction_confidence": latest_confidence,
            "prediction_sequence": prediction_sequence,
            "prediction_age_s": max(0.0, now_s - last_prediction_s) if last_prediction_s else None,
            "top_matches": top_matches,
            "model_loaded": self.model is not None,
            "model_error": self.model_error,
            "model_words": list(self.model_words),
            "training_status": self.training_status,
            "detect_limits_mm": detect_limits,
            "tof": distances,
            "valid": valid,
        }

    def ingest_antenna_device(self, device: dict[str, Any] | None, *, now_s: float | None = None) -> bool:
        now_s = time.monotonic() if now_s is None else now_s
        if not isinstance(device, dict):
            self.tick(now_s=now_s)
            return False

        status = str(device.get("status") or "").lower()
        if status in {"", "not_connected", "disconnected", "tbd", "off"}:
            self.tick(now_s=now_s)
            return False

        sequence = self._optional_int(device.get("sequence"))
        if sequence is not None and sequence == self._antenna_last_sequence:
            self.tick(now_s=now_s)
            return False

        row = self._row_from_antenna_device(device, sequence=sequence, now_s=now_s)
        if row is None:
            self.tick(now_s=now_s)
            return False

        self._antenna_last_sequence = sequence
        self._antenna_last_frame_s = now_s
        if not self._antenna_logged:
            self._antenna_logged = True
            self._log("Receiving keyboard ToF over Antenna ESP-NOW.")
        if self.status in {"Waiting for Antenna", "Disconnected", "Ready"}:
            self._set_status("Antenna ESP-NOW")

        self._update_latest_distances(row)
        self._handle_board_row(row)
        return True

    def tick(self, *, now_s: float | None = None) -> None:
        now_s = time.monotonic() if now_s is None else now_s
        self._check_training_timeout(now_s=now_s)
        if self.is_connected:
            return
        if not self._antenna_stream_active(now_s=now_s) and self.status not in {"Waiting for Antenna", "Reading swipe"}:
            self._set_status("Waiting for Antenna")

    def _write_loop(self) -> None:
        while True:
            try:
                line = self._write_queue.get()
            except Exception:
                continue
            with self._serial_lock:
                connection = self._serial
            if not connection or not connection.is_open:
                continue
            try:
                connection.write(line.encode("ascii", errors="ignore"))
                connection.flush()
            except Exception as exc:
                self._log(f"Keyboard write failed: {exc}")

    def _read_loop(self) -> None:
        while not self._stop_event.is_set():
            with self._serial_lock:
                connection = self._serial
            if not connection or not connection.is_open:
                time.sleep(0.1)
                continue
            try:
                raw = connection.readline()
            except Exception as exc:
                self._log(f"Keyboard read failed: {exc}")
                self._set_status("Disconnected")
                break
            if not raw:
                self._check_training_timeout()
                continue
            line = raw.decode("utf-8", errors="replace").strip()
            if not line:
                continue
            self._handle_line(line)
        self._set_status("Disconnected")

    def _handle_line(self, line: str) -> None:
        detect_limits = parse_detect_limits(line)
        if detect_limits is not None:
            with self._latest_lock:
                self.detect_limits = detect_limits
            write_collection_metadata(self.dataset_path, detect_limits)
            self._log(f"Keyboard detect limits: {format_max_detect_mm(detect_limits)}")
            return

        row = parse_board_row(line)
        if row is not None:
            self._update_latest_distances(row)
            self._handle_board_row(row)
            return

        if "Calibration complete" in line:
            self._set_status("Ready")
        if line.startswith(("AI RAW STREAM", "LOGGING ", "LABEL SET", "Commands:", "COVER ", "AUTOMATIC ", "CH")):
            self._log(f"[Keyboard] {line}")
        elif line.startswith(("RAW SWIPE:", "WORD:", "SPACE", "RETURN", "BACKSPACE", "CAPS")):
            self._log(f"[Keyboard] {line}")
        elif line.startswith("FATAL ERROR"):
            self._set_status("Firmware error")
            self._log(f"[Keyboard] {line}")

    def _handle_board_row(self, row: dict[str, str]) -> None:
        with self._training_lock:
            training_active = self._training_capture is not None
        if training_active:
            self._update_training_capture(row)
            return
        if self.live_prediction_enabled and self.model is not None:
            self._update_live_capture(row)

    def _update_latest_distances(self, row: dict[str, str]) -> None:
        distances: dict[str, int | None] = {}
        valid: dict[str, bool] = {}
        for index, column in enumerate(MM_COLUMNS, start=1):
            try:
                value = int(float(row[column]))
            except (KeyError, TypeError, ValueError):
                value = -1
            distances[f"sensor_{index}_mm"] = value if value >= 0 else None
            valid[f"sensor_{index}"] = value >= 0
        with self._latest_lock:
            self.latest_distances_mm = distances
            self.latest_valid = valid

    def _update_live_capture(self, row: dict[str, str]) -> None:
        if self._live_capture is None:
            self._live_capture = _LiveCapture(RawFingerDetector(max_detect_mm=self._active_detect_limits()))
        capture = self._live_capture
        active = capture.detector.update(row)
        now_s = time.monotonic()
        if active:
            if not capture.started:
                capture.started = True
                capture.rows.extend(capture.pre_roll)
                self._set_status("Reading swipe")
            capture.idle_started_s = None
        elif capture.started:
            if capture.idle_started_s is None:
                capture.idle_started_s = now_s
            elif now_s - capture.idle_started_s >= 1.0:
                rows = capture.rows
                self._live_capture = None
                self._predict_rows(rows)
                return

        if capture.started:
            capture.rows.append(dict(row))
        else:
            capture.pre_roll.append(dict(row))

    def _predict_rows(self, rows: list[dict[str, str]]) -> None:
        if self.model is None:
            return
        try:
            feature = extract_feature(
                rows,
                points=self.model["points"],
                representation=self.model["representation"],
                max_detect_mm=self._active_detect_limits(),
            )
            standardized = (feature - self.model["mean"]) / self.model["std"]
            predicted, confidence, ranked = predict_knn(
                self.model["features"],
                self.model["words"],
                standardized,
                self.model["k"],
                self.model["metric"],
            )
        except Exception as exc:
            self._log(f"Keyboard prediction failed: {exc}")
            self._set_status("Ready")
            return
        with self._latest_lock:
            self.latest_prediction = predicted
            self.latest_confidence = confidence
            self.latest_ranking = ranked
            self.prediction_sequence += 1
            self.last_prediction_s = time.monotonic()
        ranking_text = ", ".join(f"{word} {score * 100:.1f}%" for word, score in ranked[:3])
        self._log(f"Keyboard prediction: {predicted} ({confidence * 100:.1f}%). {ranking_text}")
        self._set_status("Ready")
        if self.on_prediction:
            self.on_prediction(predicted, confidence, copy.deepcopy(ranked))

    def _update_training_capture(self, row: dict[str, str]) -> None:
        with self._training_lock:
            capture = self._training_capture
        if capture is None:
            return

        now_s = time.monotonic()
        board_row = dict(row)
        capture.rows.append(
            {
                "sample_id": capture.sample_id,
                "word": capture.word,
                "repetition": capture.repetition,
                "host_time_utc": datetime.now(timezone.utc).isoformat(),
                **board_row,
            }
        )
        active = capture.detector.update(board_row)
        if active:
            if capture.gesture_started_s is None:
                capture.gesture_started_s = now_s
                self._training_status = f"Recording {capture.sample_id}; remove your hand to finish."
                self._emit_training()
            capture.idle_started_s = None
        elif capture.gesture_started_s is not None:
            if capture.idle_started_s is None:
                capture.idle_started_s = now_s
            elif now_s - capture.idle_started_s >= 1.0:
                self._finish_training_capture("complete")
        elif now_s - capture.wait_started_s >= 30.0:
            self._finish_training_capture("start_timeout")

    def _check_training_timeout(self, *, now_s: float | None = None) -> None:
        with self._training_lock:
            capture = self._training_capture
        if capture is None:
            return
        now_s = time.monotonic() if now_s is None else now_s
        if capture.gesture_started_s is None and now_s - capture.wait_started_s >= 30.0:
            self._finish_training_capture("start_timeout")

    def _finish_training_capture(self, status: str) -> None:
        with self._training_lock:
            capture = self._training_capture
            self._training_capture = None
            session = self._training_session
        if capture is None:
            return

        if status == "complete" and capture.rows:
            counts = sample_frame_counts(self.dataset_path)
            if capture.sample_id in counts:
                remove_sample_rows(self.dataset_path, capture.sample_id)
            append_sample_rows(self.dataset_path, capture.rows)
            with self._training_lock:
                if self._training_session:
                    self._training_session.completed += 1
                    completed = self._training_session.completed
                    total = self._training_session.total
                else:
                    completed = 1
                    total = 1
                self._training_status = f"Saved {capture.sample_id} ({completed}/{total})."
            self._log(f"Saved {len(capture.rows)} keyboard frames for {capture.sample_id}.")
        elif status == "start_timeout":
            with self._training_lock:
                self._training_status = f"No finger detected for {capture.sample_id}; arm it again."
                if self._training_session:
                    self._training_session.index = max(0, self._training_session.index - 1)
            self._log(f"Keyboard sample {capture.sample_id} timed out before a swipe.")
        else:
            with self._training_lock:
                self._training_status = f"Sample {capture.sample_id} was not saved."

        if session and session.index >= session.total and status == "complete":
            self._training_status = "All samples collected. Train the model next."
        if self.is_connected:
            self.send_command("label live")
        self._emit_training()

    def _train_model_worker(self) -> None:
        self._training_status = "Training model..."
        self._emit_training()
        try:
            result = train_model(
                self.dataset_path,
                self.model_path,
                fallback_max_detect_mm=MAX_DETECT_MM,
                log=lambda message: self._log(f"[Keyboard Train] {message}"),
            )
        except Exception as exc:
            self._training_status = f"Training failed: {exc}"
            self._log(f"Keyboard model training failed: {exc}")
            self._emit_training()
            return
        self.reload_model()
        self._training_status = (
            f"Model trained: {len(result['labels'])} word(s), accuracy {result['accuracy'] * 100:.1f}%."
        )
        self._emit_training()

    def _emit_training(self) -> None:
        if not self.on_training:
            return
        with self._training_lock:
            session = self._training_session
            payload = {
                "status": self._training_status,
                "prompt": self._last_training_prompt,
                "completed": session.completed if session else 0,
                "total": session.total if session else 0,
                "active": self._training_capture is not None,
                "dataset_path": str(self.dataset_path),
                "model_path": str(self.model_path),
            }
        self.on_training(payload)

    def _active_detect_limits(self) -> list[float]:
        if self.detect_limits:
            return list(self.detect_limits)
        if self.model and self.model.get("max_detect_mm"):
            return [float(value) for value in self.model["max_detect_mm"]]
        return [MAX_DETECT_MM] * len(RAW_COLUMNS)

    def _antenna_stream_active(self, *, max_age_s: float = 1.5, now_s: float | None = None) -> bool:
        if not self._antenna_last_frame_s:
            return False
        now_s = time.monotonic() if now_s is None else now_s
        return now_s - self._antenna_last_frame_s <= max_age_s

    def _row_from_antenna_device(
        self,
        device: dict[str, Any],
        *,
        sequence: int | None,
        now_s: float,
    ) -> dict[str, str] | None:
        tof = device.get("tof") if isinstance(device.get("tof"), dict) else {}
        valid_map = device.get("valid") if isinstance(device.get("valid"), dict) else {}
        if not isinstance(tof, dict):
            return None

        values: list[int] = []
        saw_sensor_field = False
        for index in range(1, 5):
            sensor_key = f"sensor_{index}"
            value = tof.get(f"{sensor_key}_mm")
            if value is not None:
                saw_sensor_field = True
            valid = bool(valid_map.get(sensor_key, value is not None)) if isinstance(valid_map, dict) else value is not None
            if not valid or value is None:
                values.append(-1)
                continue
            try:
                distance = int(round(float(value)))
            except (TypeError, ValueError):
                values.append(-1)
                continue
            values.append(distance if distance >= 0 else -1)

        if not saw_sensor_field:
            return None

        self._antenna_frame_counter += 1
        frame = sequence if sequence is not None else self._antenna_frame_counter
        packet_ms = self._optional_int(device.get("t_ms"))
        timestamp_ms = packet_ms if packet_ms is not None else int(now_s * 1000.0)
        with self._training_lock:
            label = self._training_capture.sample_id if self._training_capture else "antenna"

        channel_values = {
            "ch1": values[0],
            "ch2": values[1],
            "ch0": values[2],
            "ch3": values[3],
        }
        row = {
            "frame": str(frame),
            "ms": str(timestamp_ms),
            "label": label,
        }
        for channel in ("ch1", "ch2", "ch0", "ch3"):
            row[f"{channel}_raw"] = str(channel_values[channel])
        for channel in ("ch1", "ch2", "ch0", "ch3"):
            row[f"{channel}_mm"] = str(channel_values[channel])
        return row

    @staticmethod
    def _optional_int(value: Any) -> int | None:
        if isinstance(value, bool) or value is None:
            return None
        try:
            return int(float(value))
        except (TypeError, ValueError):
            return None

    def _ensure_starter_model(self) -> None:
        if self.model_path.exists():
            return
        starter = project_resource_path("python_app", "data", "keyboard", "word_knn_model.npz")
        if not starter.exists():
            return
        try:
            self.model_path.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(starter, self.model_path)
        except OSError as exc:
            self._log(f"Could not install starter keyboard model: {exc}")

    def _clean_words(self, words: list[str], *, include_command_words: bool) -> list[str]:
        cleaned: list[str] = []
        seen: set[str] = set()
        for raw_word in words:
            word = str(raw_word).strip().lower()
            if not word or word in seen:
                continue
            if not WORD_PATTERN.fullmatch(word):
                self._log(f"Ignoring invalid keyboard word: {word!r}")
                continue
            cleaned.append(word)
            seen.add(word)
        if include_command_words:
            for command in COMMAND_WORDS:
                if command not in seen:
                    cleaned.append(command)
                    seen.add(command)
        return cleaned

    def _set_status(self, status: str) -> None:
        if self.status == status:
            return
        self.status = status
        if self.on_status:
            self.on_status(status)

    def _log(self, message: str) -> None:
        if self.on_log:
            self.on_log(f"[Keyboard] {message}")
