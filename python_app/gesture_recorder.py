from __future__ import annotations

import json
import re
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Any, Callable

from fusion_state import FIELD_ORDER


SnapshotProvider = Callable[[], dict[str, Any]]
StatusCallback = Callable[[str], None]


class GestureRecorder:
    def __init__(
        self,
        gesture_root: Path,
        snapshot_provider: SnapshotProvider,
        on_status: StatusCallback | None = None,
        sample_hz: float = 50.0,
    ) -> None:
        self.gesture_root = Path(gesture_root)
        self.snapshot_provider = snapshot_provider
        self.on_status = on_status
        self.sample_hz = sample_hz
        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._progress = 0.0

    @property
    def is_recording(self) -> bool:
        return bool(self._thread and self._thread.is_alive())

    @property
    def progress(self) -> float:
        return self._progress

    def start(self, gesture_name: str, repetitions: int, duration_s: float, countdown_s: int = 3) -> bool:
        if self.is_recording:
            self._status("Recording is already running.")
            return False

        clean_name = self._sanitize_name(gesture_name)
        if not clean_name:
            self._status("Enter a gesture name before recording.")
            return False

        repetitions = max(1, int(repetitions))
        duration_s = max(0.2, float(duration_s))
        countdown_s = max(0, int(countdown_s))

        self._stop_event.clear()
        self._progress = 0.0
        self._thread = threading.Thread(
            target=self._record_worker,
            args=(clean_name, repetitions, duration_s, countdown_s),
            daemon=True,
        )
        self._thread.start()
        return True

    def stop(self) -> None:
        self._stop_event.set()

    def _record_worker(
        self,
        gesture_name: str,
        repetitions: int,
        duration_s: float,
        countdown_s: int,
    ) -> None:
        gesture_dir = self.gesture_root / gesture_name
        gesture_dir.mkdir(parents=True, exist_ok=True)

        session_start = time.time()
        total_duration = repetitions * (countdown_s + duration_s)

        def update_progress() -> None:
            elapsed = time.time() - session_start
            self._progress = min(1.0, elapsed / max(0.001, total_duration))

        for rep in range(1, repetitions + 1):
            if self._stop_event.is_set():
                break

            for remaining in range(countdown_s, 0, -1):
                update_progress()
                self._status(f"{gesture_name} rep {rep}/{repetitions}: starting in {remaining}...")
                if self._sleep_interruptible(1.0):
                    return

            self._status(f"Recording {gesture_name} rep {rep}/{repetitions}.")
            samples: list[dict[str, Any]] = []
            start = time.time()
            end = start + duration_s
            interval = 1.0 / self.sample_hz

            while time.time() < end and not self._stop_event.is_set():
                now = time.time()
                update_progress()
                snapshot = self.snapshot_provider()
                samples.append(
                    {
                        "t_epoch": now,
                        "t_rel": now - start,
                        "input_array": snapshot.get("input_array", []),
                        "input_dict": snapshot.get("input_dict", {}),
                    }
                )
                sleep_for = max(0.0, interval - (time.time() - now))
                if self._sleep_interruptible(sleep_for):
                    return

            finished = time.time()
            sample_rate = len(samples) / max(0.001, finished - start)
            final_snapshot = self.snapshot_provider()
            payload = {
                "gesture_name": gesture_name,
                "repetition_index": rep,
                "start_time": datetime.fromtimestamp(start).isoformat(timespec="milliseconds"),
                "sample_rate_estimate": sample_rate,
                "field_order": FIELD_ORDER,
                "samples": samples,
                "raw_device_state_at_end": final_snapshot.get("raw_device_state", {}),
                "hand_state_at_end": final_snapshot.get("hand_state", {}),
            }

            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            output_path = gesture_dir / f"{timestamp}_rep_{rep}.json"
            with output_path.open("w", encoding="utf-8") as f:
                json.dump(payload, f, indent=2)
                f.write("\n")
            self._status(f"Saved {output_path}")

        self._progress = 1.0
        self._status("Gesture recording finished.")
        self._progress = 0.0

    def _sleep_interruptible(self, seconds: float) -> bool:
        return self._stop_event.wait(max(0.0, seconds))

    def _status(self, message: str) -> None:
        if self.on_status:
            self.on_status(message)

    @staticmethod
    def _sanitize_name(name: str) -> str:
        cleaned = re.sub(r"[^A-Za-z0-9_-]+", "_", name.strip())
        return cleaned.strip("_")

