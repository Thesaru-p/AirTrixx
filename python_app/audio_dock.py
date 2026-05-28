from __future__ import annotations

import json
import os
import re
import sys
import time
import threading
import urllib.error
import urllib.parse
import urllib.request
from pathlib import Path
from typing import Any, Callable

try:
    import serial
    from serial.tools import list_ports
except Exception:
    serial = None
    list_ports = None

DEFAULT_BAUD = 115200
DEFAULT_MODEL = "nova-3"
DEFAULT_LANGUAGE = "en-IN"


class AudioDockBridge:
    def __init__(
        self,
        on_log: Callable[[str], None] | None = None,
        on_status: Callable[[str], None] | None = None,
        on_transcript: Callable[[str, str], None] | None = None,
    ) -> None:
        self.on_log = on_log
        self.on_status = on_status
        self.on_transcript = on_transcript  # Callback takes (clap_type, text)
        self._serial = None
        self._serial_lock = threading.RLock()
        self._thread = None
        self._stop_event = threading.Event()
        self.is_connected = False
        self.current_port = None
        self.latest_transcript = ""
        self.last_trigger = "-"
        self.status = "Disconnected"

    def _log(self, message: str) -> None:
        if self.on_log:
            self.on_log(f"[Audio Dock] {message}")

    def _set_status(self, status: str) -> None:
        self.status = status
        if self.on_status:
            self.on_status(status)

    def load_deepgram_key(self) -> str:
        env_key = os.environ.get("DEEPGRAM_API_KEY", "").strip()
        if env_key:
            return env_key
        raise RuntimeError("DEEPGRAM_API_KEY environment variable not found.")

    def connect(self, port: str) -> bool:
        if serial is None:
            self._log("pyserial is not installed.")
            return False

        if self.is_connected:
            return True

        try:
            self.load_deepgram_key()
        except Exception as exc:
            self._log(f"Error: {exc}")
            return False

        try:
            ser = serial.Serial(
                port=port,
                baudrate=DEFAULT_BAUD,
                timeout=1,
                write_timeout=10,
            )
            self._serial = ser
            self.current_port = port
            self.is_connected = True
            self._stop_event.clear()
            self._set_status("Connected")
            self._log(f"Connected to {port} successfully.")

            self._thread = threading.Thread(target=self._run_loop, daemon=True)
            self._thread.start()
            return True
        except Exception as exc:
            self._log(f"Connection failed to {port}: {exc}")
            self._set_status("Error")
            return False

    def disconnect(self) -> None:
        self._stop_event.set()
        self.is_connected = False
        self.current_port = None
        self._set_status("Disconnected")
        
        with self._serial_lock:
            if self._serial:
                try:
                    self._serial.close()
                except Exception:
                    pass
                self._serial = None
        self._log("Disconnected.")

    def _read_line(self) -> str | None:
        with self._serial_lock:
            if not self._serial:
                return None
            try:
                raw = self._serial.readline()
            except Exception:
                return None
        if not raw:
            return None
        return raw.decode("utf-8", errors="replace").strip()

    def _write_line(self, data: bytes) -> bool:
        with self._serial_lock:
            if not self._serial:
                return False
            try:
                self._serial.write(data)
                self._serial.flush()
                return True
            except Exception as exc:
                self._log(f"Write failed: {exc}")
                return False

    def _wait_for_ready(self, timeout_s: float) -> bool:
        self._log("Waiting for ESP32 READY...")
        deadline = time.monotonic() + timeout_s

        while time.monotonic() < deadline and not self._stop_event.is_set():
            line = self._read_line()
            if not line:
                continue
            self._log(f"ESP32: {line}")
            if "READY" in line:
                return True

        self._log("ESP32 READY not seen. Proceeding anyway.")
        return False

    def _run_loop(self) -> None:
        try:
            self._serial.reset_input_buffer()
            time.sleep(1.0)
            self._wait_for_ready(10.0)

            while not self._stop_event.is_set():
                self._set_status("Waiting for Clap")
                self._log("Sending RECORD command to start clap listener...")
                if not self._write_line(b"RECORD\n"):
                    self._log("Failed to send command. Reconnecting...")
                    break

                audio_size = None
                self._log("Waiting for clap detection and audio stream...")
                
                while audio_size is None and not self._stop_event.is_set():
                    line = self._read_line()
                    if not line:
                        continue
                    self._log(f"ESP32: {line}")
                    
                    if "Triggered!" in line:
                        self._set_status("Clap Detected")
                        # Parse type: e.g. "Triggered! Detected: Double clap with score 0.96"
                        match = re.search(r'Detected:\s*([A-Za-z\s]+)\s*with', line)
                        if match:
                          self.last_trigger = match.group(1).strip()
                        else:
                          self.last_trigger = "Clap"
                          
                    elif "RECORDING_START" in line:
                        self._set_status("Recording")
                        
                    elif line.startswith("AUDIO_BEGIN "):
                        try:
                            audio_size = int(line.split()[1])
                        except Exception:
                            pass

                if self._stop_event.is_set() or audio_size is None:
                    break

                self._set_status("Receiving Audio")
                self._log(f"Receiving {audio_size} bytes of WAV audio...")
                
                chunks = []
                received = 0
                deadline = time.monotonic() + max(30.0, audio_size / 5000.0)

                while received < audio_size and not self._stop_event.is_set():
                    if time.monotonic() > deadline:
                        self._log("Timed out receiving WAV data.")
                        break

                    with self._serial_lock:
                        if not self._serial:
                            break
                        try:
                            chunk = self._serial.read(min(4096, audio_size - received))
                        except Exception:
                            chunk = None

                    if not chunk:
                        time.sleep(0.01)
                        continue

                    chunks.append(chunk)
                    received += len(chunk)

                if self._stop_event.is_set() or received < audio_size:
                    break

                audio = b"".join(chunks)
                self._log("WAV audio successfully received.")
                
                # Save to disk as history
                wav_path = Path(__file__).parent / "last_esp32_recording.wav"
                try:
                    wav_path.write_bytes(audio)
                except Exception:
                    pass

                # Stream audio end
                while not self._stop_event.is_set():
                    line = self._read_line()
                    if not line:
                        continue
                    self._log(f"ESP32: {line}")
                    if line == "AUDIO_END":
                        break

                self._set_status("Transcribing")
                self._log("Uploading audio to Deepgram API...")
                
                try:
                    api_key = self.load_deepgram_key()
                    transcript = self._transcribe(audio, api_key)
                    self._log(f"Transcript: \"{transcript}\"")
                except Exception as exc:
                    self._log(f"Transcribe error: {exc}")
                    transcript = "[Transcription Error]"

                self._set_status("Sending Transcript")
                clean = transcript.replace("\r", " ").replace("TRANSCRIPT_BEGIN", "").replace("TRANSCRIPT_END", "")
                
                self._write_line(b"TRANSCRIPT_BEGIN\n")
                if clean:
                    for l in clean.splitlines():
                        self._write_line(l.encode("utf-8", errors="replace") + b"\n")
                self._write_line(b"TRANSCRIPT_END\n")

                self.latest_transcript = transcript
                if self.on_transcript:
                    self.on_transcript(self.last_trigger, transcript)

                # Wait for bridge done
                self._log("Waiting for ESP32 acknowledgement...")
                done_deadline = time.monotonic() + 5.0
                while time.monotonic() < done_deadline and not self._stop_event.is_set():
                    line = self._read_line()
                    if not line:
                        continue
                    self._log(f"ESP32: {line}")
                    if "BRIDGE_DONE" in line:
                        break

                self._log("Bridge transaction finished. Looping...")
                time.sleep(1.0)

        except Exception as exc:
            self._log(f"Bridge loop error: {exc}")
        finally:
            self.disconnect()

    def _transcribe(self, audio: bytes, api_key: str) -> str:
        query = {
            "model": DEFAULT_MODEL,
            "smart_format": "true",
            "language": DEFAULT_LANGUAGE,
        }
        url = "https://api.deepgram.com/v1/listen?" + urllib.parse.urlencode(query)
        request = urllib.request.Request(
            url,
            data=audio,
            headers={
                "Authorization": f"Token {api_key}",
                "Content-Type": "audio/wav",
            },
            method="POST",
        )

        with urllib.request.urlopen(request, timeout=30) as response:
            body = response.read().decode("utf-8", errors="replace")

        data = json.loads(body)
        try:
            transcript = data["results"]["channels"][0]["alternatives"][0].get("transcript", "")
            return transcript.strip()
        except Exception:
            raise RuntimeError(f"Deepgram returned invalid JSON: {body}")
