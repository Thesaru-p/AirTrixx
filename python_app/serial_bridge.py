from __future__ import annotations

import copy
import json
import queue
import threading
import time
from typing import Any, Callable

try:
    import serial
    from serial.tools import list_ports
except Exception:  # pragma: no cover - handled at runtime for missing dependency
    serial = None
    list_ports = None


LogCallback = Callable[[str], None]


class SerialBridge:
    def __init__(
        self,
        baud_rate: int = 921600,
        on_log: LogCallback | None = None,
        on_state: Callable[[dict[str, Any]], None] | None = None,
    ) -> None:
        self.baud_rate = baud_rate
        self.on_log = on_log
        self.on_state = on_state
        self._serial = None
        self._serial_lock = threading.RLock()
        self._latest_lock = threading.Lock()
        self._latest_state: dict[str, Any] = {}
        self._reader_thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._manual_disconnect = False
        self._current_port: str | None = None
        self.audio_dock_bridge = None
        self._write_queue: queue.Queue[tuple[str, str | None]] = queue.Queue(maxsize=512)
        self._coalesced_writes: dict[str, str] = {}
        self._queued_coalesce_keys: set[str] = set()
        self._coalesced_lock = threading.Lock()
        self._writer_thread = threading.Thread(target=self._write_loop, daemon=True)
        self._writer_thread.start()

    @staticmethod
    def available_ports() -> list[dict[str, str]]:
        if list_ports is None:
            return []
        ports = []
        for port in list_ports.comports():
            ports.append(
                {
                    "device": port.device,
                    "description": port.description or "",
                    "hwid": port.hwid or "",
                }
            )
        return ports

    def _log(self, message: str) -> None:
        if self.on_log:
            self.on_log(message)

    def connect(self, port: str | None = None) -> bool:
        if serial is None:
            self._log("pyserial is not installed. Run pip install -r requirements.txt.")
            return False

        if self.is_connected:
            return True

        self._manual_disconnect = False
        candidates = [port] if port else [p["device"] for p in self.available_ports()]
        candidates = [p for p in candidates if p]
        if not candidates:
            self._log("No COM ports found.")
            return False

        for candidate in candidates:
            if self._open_port(candidate):
                self._current_port = candidate
                self._start_reader()
                self._log(f"Connected to {candidate} at {self.baud_rate} baud.")
                return True

        self._log("Could not connect to any candidate COM port.")
        return False

    def disconnect(self) -> None:
        self._manual_disconnect = True
        self._stop_event.set()
        self._close_serial()
        self._clear_write_queue()
        thread = self._reader_thread
        if thread and thread.is_alive() and thread is not threading.current_thread():
            thread.join(timeout=0.5)
        with self._latest_lock:
            self._latest_state = {}
        self._current_port = None
        self._log("Serial disconnected.")

    @property
    def is_connected(self) -> bool:
        with self._serial_lock:
            return bool(self._serial and self._serial.is_open)

    @property
    def current_port(self) -> str | None:
        return self._current_port

    def get_latest_state(self) -> dict[str, Any]:
        with self._latest_lock:
            return copy.deepcopy(self._latest_state)

    def send_command(self, command: dict[str, Any], coalesce_key: str | None = None) -> bool:
        line = json.dumps(command, separators=(",", ":")) + "\n"
        with self._serial_lock:
            if not self._serial or not self._serial.is_open:
                return False
        try:
            if coalesce_key:
                should_enqueue = False
                with self._coalesced_lock:
                    self._coalesced_writes[coalesce_key] = line
                    if coalesce_key not in self._queued_coalesce_keys:
                        self._queued_coalesce_keys.add(coalesce_key)
                        should_enqueue = True
                if should_enqueue:
                    self._write_queue.put_nowait(("coalesced", coalesce_key))
            else:
                self._write_queue.put_nowait(("line", line))
            return True
        except queue.Full:
            if coalesce_key:
                with self._coalesced_lock:
                    self._queued_coalesce_keys.discard(coalesce_key)
            self._log("Serial write queue is full; dropping command.")
            return False

    def _write_loop(self) -> None:
        while True:
            try:
                kind, payload = self._write_queue.get()
            except Exception:
                continue
            line: str | None
            if kind == "coalesced":
                if payload is None:
                    continue
                with self._coalesced_lock:
                    line = self._coalesced_writes.pop(payload, None)
                    self._queued_coalesce_keys.discard(payload)
                if line is None:
                    continue
            else:
                line = payload
            if line:
                self._write_line(line)

    def _write_line(self, line: str) -> None:
        with self._serial_lock:
            ser = self._serial
            if not ser or not ser.is_open:
                return
            try:
                ser.write(line.encode("utf-8"))
            except Exception as exc:
                self._log(f"Serial write failed: {exc}")
                self._close_serial()

    def _clear_write_queue(self) -> None:
        with self._coalesced_lock:
            self._coalesced_writes.clear()
            self._queued_coalesce_keys.clear()
        try:
            while True:
                self._write_queue.get_nowait()
        except queue.Empty:
            pass

    def _open_port(self, port: str) -> bool:
        try:
            ser = serial.Serial(
                port=port,
                baudrate=self.baud_rate,
                timeout=0.1,
                write_timeout=0.1,
            )
            with self._serial_lock:
                self._serial = ser
            return True
        except Exception as exc:
            self._log(f"Failed to open {port}: {exc}")
            return False

    def _close_serial(self) -> None:
        with self._serial_lock:
            ser = self._serial
            self._serial = None
        if ser:
            try:
                ser.close()
            except Exception:
                pass

    def _start_reader(self) -> None:
        self._stop_event.clear()
        if self._reader_thread and self._reader_thread.is_alive():
            return
        self._reader_thread = threading.Thread(target=self._read_loop, daemon=True)
        self._reader_thread.start()

    def _read_loop(self) -> None:
        while not self._stop_event.is_set():
            with self._serial_lock:
                ser = self._serial

            if ser is None or not ser.is_open:
                if not self._manual_disconnect and self._current_port:
                    time.sleep(0.5)
                    self._open_port(self._current_port)
                else:
                    time.sleep(0.1)
                continue

            try:
                raw = ser.readline()
            except Exception as exc:
                self._log(f"Serial read failed: {exc}")
                self._close_serial()
                continue

            if not raw:
                continue

            try:
                line = raw.decode("utf-8", errors="replace").strip()
            except Exception:
                continue
            if line.startswith("ANTENNA_"):
                self._log(f"[Antenna] {line}")
                continue

            looks_like_direct_audio_dock = (
                line.startswith("CLAP_SCORES") or
                line.startswith("Triggered!") or
                line.startswith("RECORD_STREAM_") or
                line.startswith("Audio RAM ") or
                line.startswith("AUDIO_DOCK_MAC:")
            )
            if looks_like_direct_audio_dock:
                self._log(
                    "Audio Dock debug output detected on the active serial port. "
                    "Connect the app to the Antenna ESP32-S3 COM port; the Audio Dock should talk wirelessly."
                )
                continue

            is_audiodock = (
                "AUDIODOCK_" in line or 
                "UDIODOCK_" in line or 
                "DOCK_AUDIO" in line or 
                "_AUDIO:" in line or
                (self.audio_dock_bridge and self.audio_dock_bridge.expected_audio_size is not None and len(line) > 100)
            )

            if is_audiodock:
                first_idx = len(line)
                for p in ["AUDIODOCK_", "UDIODOCK_", "DOCK_AUDIO", "_AUDIO:"]:
                    idx = line.find(p)
                    if 0 <= idx < first_idx:
                        first_idx = idx
                
                if first_idx < len(line) and first_idx > 0:
                    leading = line[:first_idx].strip()
                    if leading:
                        try:
                            state = json.loads(leading)
                            if isinstance(state, dict):
                                with self._latest_lock:
                                    self._latest_state = state
                                if self.on_state:
                                    self.on_state(copy.deepcopy(state))
                        except Exception:
                            pass
                
                if self.audio_dock_bridge:
                    self.audio_dock_bridge.handle_antenna_line(line)
                continue

            try:
                state = json.loads(line)
            except json.JSONDecodeError:
                self._log(f"Ignored malformed serial line: {line[:120]}")
                continue

            if not isinstance(state, dict):
                continue

            with self._latest_lock:
                self._latest_state = state
            if self.on_state:
                self.on_state(copy.deepcopy(state))
