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
from datetime import datetime
from pathlib import Path
from typing import Any, Callable

try:
    import serial
    from serial.tools import list_ports
except Exception:
    serial = None
    list_ports = None

DEFAULT_MODEL = "nova-3"
DEFAULT_LANGUAGE = "en-IN"
TRAINING_LABELS = ("double_clap", "single_clap", "noise", "false_trigger", "tap_noise", "speech")


class AudioDockBridge:
    def __init__(
        self,
        on_log: Callable[[str], None] | None = None,
        on_status: Callable[[str], None] | None = None,
        on_transcript: Callable[[str, str], None] | None = None,
        on_training: Callable[[str], None] | None = None,
        on_training_sample: Callable[[str, Path], None] | None = None,
        serial_bridge: Any | None = None,
        deepgram_api_key: str = "",
        audio_recording_path: Path | None = None,
        training_data_dir: Path | None = None,
    ) -> None:
        self.on_log = on_log
        self.on_status = on_status
        self.on_transcript = on_transcript  # Callback takes (clap_type, text)
        self.on_training = on_training
        self.on_training_sample = on_training_sample
        self.serial_bridge = serial_bridge
        self.is_connected = False
        self.latest_transcript = ""
        self.last_trigger = "-"
        self.status = "Disconnected"
        self.expected_audio_size = None
        self.audio_buffer = bytearray()
        self.deepgram_api_key = deepgram_api_key.strip()
        self.audio_recording_path = audio_recording_path
        self.training_data_dir = training_data_dir
        self.training_label = TRAINING_LABELS[0]
        self.training_capture_remaining = 0
        self.training_saved_count = 0
        self.last_audio_data: bytes | None = None
        self.last_audio_path: Path | None = None
        self.last_training_sample_path: Path | None = None
        self._missing_key_warning_logged = False

    def _log(self, message: str) -> None:
        if self.on_log:
            self.on_log(f"[Audio Dock] {message}")

    def _set_status(self, status: str) -> None:
        self.status = status
        if self.on_status:
            self.on_status(status)

    def _set_training_status(self, status: str) -> None:
        if self.on_training:
            self.on_training(status)

    @staticmethod
    def _normalize_training_label(label: str) -> str:
        normalized = re.sub(r"[^a-zA-Z0-9_-]+", "_", label.strip().lower()).strip("_")
        return normalized or TRAINING_LABELS[0]

    def load_deepgram_key(self) -> str:
        if self.deepgram_api_key:
            return self.deepgram_api_key
        env_key = os.environ.get("DEEPGRAM_API_KEY", "").strip()
        if env_key:
            return env_key
        raise RuntimeError("Deepgram API key is not set. Add it in Settings or set DEEPGRAM_API_KEY.")

    def set_deepgram_key(self, api_key: str) -> None:
        self.deepgram_api_key = api_key.strip()
        if self.deepgram_api_key:
            self._missing_key_warning_logged = False

    def arm_training_capture(self, label: str, count: int = 1) -> None:
        self.training_label = self._normalize_training_label(label)
        self.training_capture_remaining = max(1, int(count))
        self._set_training_status(f"Armed: {self.training_label} x{self.training_capture_remaining}")
        self._log(f"Training capture armed for {self.training_label} ({self.training_capture_remaining} clip(s)).")

    def cancel_training_capture(self) -> None:
        self.training_capture_remaining = 0
        self._set_training_status("Capture stopped")
        self._log("Training capture stopped.")

    def save_last_training_sample(self, label: str | None = None) -> Path | None:
        if self.last_audio_data is None:
            self._set_training_status("No clip available")
            self._log("No Audio Dock clip is available to save yet.")
            return None
        path = self._save_training_sample(self.last_audio_data, label or self.training_label, self.last_trigger)
        if path and self.on_training_sample:
            self.on_training_sample(path.parent.name, path)
        return path

    def _save_training_sample(self, audio_data: bytes, label: str, trigger: str) -> Path | None:
        if not self.training_data_dir:
            self._set_training_status("Training folder unavailable")
            return None

        normalized_label = self._normalize_training_label(label)
        label_dir = self.training_data_dir / normalized_label
        label_dir.mkdir(parents=True, exist_ok=True)
        trigger_slug = self._normalize_training_label(trigger if trigger and trigger != "-" else "unknown")
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        path = label_dir / f"{timestamp}_{trigger_slug}.wav"
        try:
            path.write_bytes(audio_data)
        except OSError as exc:
            self._set_training_status("Save failed")
            self._log(f"Failed to save training sample: {exc}")
            return None

        self.training_saved_count += 1
        self.last_training_sample_path = path
        status = f"Saved {normalized_label}: {path.name}"
        self._set_training_status(status)
        self._log(f"Saved training sample: {path}")
        return path

    def connect(self, port: str | None = None) -> bool:
        if not self.serial_bridge:
            self._log("Error: Serial bridge reference not set.")
            return False

        if not self.serial_bridge.is_connected:
            self._log("Error: Antenna is not connected. Please connect the Antenna first on the Live Data / Fused Input page.")
            self._set_status("Error")
            return False

        if self.is_connected:
            return True

        self.is_connected = True
        self._set_status("Waiting for Clap")
        self._log("Connected wirelessly via Antenna ESP-NOW bridge.")
        try:
            self.load_deepgram_key()
        except Exception as exc:
            if not self._missing_key_warning_logged:
                self._missing_key_warning_logged = True
                self._log(f"Warning: {exc} Audio will be received, but transcription needs a key.")
        self._log("Waiting for clap detection and audio stream...")
        return True

    def disconnect(self) -> None:
        self.is_connected = False
        self.expected_audio_size = None
        self.audio_buffer = bytearray()
        self._set_status("Disconnected")
        self._log("Disconnected.")

    def send_control(self, control: str, **params: Any) -> bool:
        if not self.serial_bridge or not self.serial_bridge.is_connected:
            self._log("Error: Antenna is not connected.")
            return False

        normalized = control.strip().lower()
        aliases = {
            "ledtest": "led_test",
            "led": "led_test",
            "speakertest": "speaker_test",
            "speaker": "speaker_test",
            "spktest": "speaker_test",
            "trainingrecord": "training_record",
            "record_sample": "training_record",
            "sample": "training_record",
        }
        normalized = aliases.get(normalized, normalized)
        if normalized not in {"led_test", "speaker_test", "training_record"}:
            self._log(f"Unsupported control: {control}")
            return False

        if normalized == "training_record" and not self.is_connected:
            self._log("Error: Audio Dock is not connected in the dashboard. Click Connect before Start Capture.")
            return False

        command = {
            "cmd": "audiodock",
            "control": normalized,
        }
        if normalized == "training_record" and params.get("count") is not None:
            try:
                command["count"] = max(1, min(100, int(params["count"])))
            except (TypeError, ValueError):
                command["count"] = 1
        if self.serial_bridge.send_command(command):
            labels = {
                "led_test": "LED ring test",
                "speaker_test": "speaker test",
                "training_record": "training record",
            }
            label = labels[normalized]
            self._log(f"Sent wireless {label} command.")
            return True

        self._log("Failed to send Audio Dock control command to Antenna.")
        return False

    def handle_antenna_line(self, line: str) -> None:
        if not self.is_connected:
            return

        if "TRIGGER:" in line:
            # If we are already receiving audio, ignore duplicate or delayed triggers
            if self.expected_audio_size is not None:
                return
            try:
                trigger_str = line.split("TRIGGER:", 1)[1]
                parts = trigger_str.split(",")
                clap_type = int(parts[0].strip())
                audio_size = int(parts[1].strip())
                
                self.expected_audio_size = audio_size
                self.audio_buffer = bytearray()
                if clap_type == 2:
                    self.last_trigger = "Double clap"
                elif clap_type == 1:
                    self.last_trigger = "Single clap"
                elif clap_type == 0:
                    self.last_trigger = "Training sample"
                else:
                    self.last_trigger = f"Trigger {clap_type}"
                self._set_status("Receiving Audio")
                self._log(f"Audio trigger: {self.last_trigger}. Expecting {audio_size} bytes of WAV audio.")
            except Exception as exc:
                self._log(f"Failed to parse trigger: {exc}")

        elif self.expected_audio_size is not None and (
            "AUDIO:" in line or 
            "AUDIO_CHUNK" in line or 
            len(line) > 100
        ):
            try:
                normalized = line
                for prefix in ["AUDIODOCK_AUDIO:", "UDIODOCK_AUDIO:", "DIODOCK_AUDIO:", "IODOCK_AUDIO:", "AUDIO:"]:
                    normalized = normalized.replace(prefix, "|")
                
                if "|" not in normalized:
                    chunks = [normalized]
                else:
                    chunks = normalized.split("|")

                for chunk in chunks:
                    chunk = chunk.strip()
                    if not chunk:
                        continue
                    
                    # Extract the contiguous trailing hex digits (skipping any prefix)
                    match = re.search(r'[0-9a-fA-F]+$', chunk)
                    if not match:
                        continue
                    
                    hex_data = match.group(0)
                    
                    # Verify by length to ignore any small segments or JSON frames
                    if len(hex_data) < 40:
                        continue
                    
                    if len(hex_data) % 2 != 0:
                        hex_data = hex_data[:-1]  # Ensure even length for fromhex()
                    
                    chunk_bytes = bytes.fromhex(hex_data)
                    self.audio_buffer.extend(chunk_bytes)
                
                # Periodically log progress so user has visual feedback
                total_len = len(self.audio_buffer)
                if total_len > 0 and ((total_len // 200) % 40 == 0 or total_len >= self.expected_audio_size):
                    self._log(f"Receiving audio... {total_len}/{self.expected_audio_size} bytes.")

                if total_len >= self.expected_audio_size:
                    expected = self.expected_audio_size
                    self.expected_audio_size = None  # Prevent duplicate triggers
                    
                    self._log("WAV audio successfully received wireless.")
                    # Crop buffer to exactly expected size to guarantee perfect WAV structure
                    audio_data = bytes(self.audio_buffer[:expected])
                    
                    wav_path = self.audio_recording_path or (Path.cwd() / "last_esp32_recording.wav")
                    try:
                        wav_path.parent.mkdir(parents=True, exist_ok=True)
                        wav_path.write_bytes(audio_data)
                        self.last_audio_path = wav_path
                    except Exception:
                        pass

                    self.last_audio_data = audio_data
                    training_capture_active = self.training_capture_remaining > 0
                    if training_capture_active:
                        saved_path = self._save_training_sample(audio_data, self.training_label, self.last_trigger)
                        if saved_path:
                            self.training_capture_remaining -= 1
                            if self.training_capture_remaining > 0:
                                self._set_training_status(
                                    f"Armed: {self.training_label} x{self.training_capture_remaining}"
                                )
                            else:
                                self._set_training_status(f"Capture complete: {self.training_label}")
                            if self.on_training_sample:
                                self.on_training_sample(saved_path.parent.name, saved_path)
                    
                    self._set_status("Saving Sample" if training_capture_active else "Transcribing")
                    threading.Thread(
                        target=self._transcribe_and_send,
                        args=(audio_data,),
                        kwargs={"skip_transcription": training_capture_active},
                        daemon=True,
                    ).start()
            except Exception as exc:
                self._log(f"Failed to parse audio chunk: {exc}")

    def _transcribe_and_send(self, audio_data: bytes, *, skip_transcription: bool = False) -> None:
        if skip_transcription:
            self._log("Training sample saved; skipping transcription.")
            transcript = "[Training Sample Saved]"
            self.latest_transcript = transcript
            if self.on_transcript:
                self.on_transcript(self.last_trigger, transcript)
            if self.is_connected:
                self._set_status("Waiting for Clap")
            return
        else:
            try:
                self._log("Uploading audio to Deepgram API...")
                api_key = self.load_deepgram_key()
                transcript = self._transcribe(audio_data, api_key)
                self._log(f"Transcript: \"{transcript}\"")
            except Exception as exc:
                self._log(f"Transcribe error: {exc}")
                transcript = "[Transcription Error]"

        self._set_status("Sending Transcript")
        self.latest_transcript = transcript

        # Send to Antenna which will forward to Audio Dock over ESP-NOW
        cmd = {
            "cmd": "audiodock",
            "transcript": transcript
        }
        if self.serial_bridge.send_command(cmd):
            self._log("Transcript command successfully sent to Antenna.")
        else:
            self._log("Failed to send transcript command to Antenna.")

        if self.on_transcript:
            self.on_transcript(self.last_trigger, transcript)

        # Show transcript for 2 seconds, then return to waiting state
        time.sleep(2.0)
        if self.is_connected:
            self.expected_audio_size = None
            self.audio_buffer = bytearray()
            self._set_status("Waiting for Clap")
            self._log("Waiting for next clap detection...")

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

        try:
            with urllib.request.urlopen(request, timeout=30) as response:
                body = response.read().decode("utf-8", errors="replace")
        except urllib.error.HTTPError as exc:
            details = exc.read().decode("utf-8", errors="replace")
            raise RuntimeError(f"Deepgram HTTP {exc.code}: {details}") from exc

        data = json.loads(body)
        try:
            transcript = data["results"]["channels"][0]["alternatives"][0].get("transcript", "")
            return transcript.strip()
        except Exception:
            raise RuntimeError(f"Deepgram returned invalid JSON: {body}")
