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

DEFAULT_MODEL = "nova-3"
DEFAULT_LANGUAGE = "en-IN"


class AudioDockBridge:
    def __init__(
        self,
        on_log: Callable[[str], None] | None = None,
        on_status: Callable[[str], None] | None = None,
        on_transcript: Callable[[str, str], None] | None = None,
        serial_bridge: Any | None = None,
    ) -> None:
        self.on_log = on_log
        self.on_status = on_status
        self.on_transcript = on_transcript  # Callback takes (clap_type, text)
        self.serial_bridge = serial_bridge
        self.is_connected = False
        self.latest_transcript = ""
        self.last_trigger = "-"
        self.status = "Disconnected"
        self.expected_audio_size = None
        self.audio_buffer = bytearray()

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

    def connect(self, port: str | None = None) -> bool:
        try:
            self.load_deepgram_key()
        except Exception as exc:
            self._log(f"Error: {exc}")
            return False

        if not self.serial_bridge:
            self._log("Error: Serial bridge reference not set.")
            return False

        if not self.serial_bridge.is_connected:
            self._log("Error: Antenna is not connected. Please connect the Antenna first on the Live Data / Fused Input page.")
            self._set_status("Error")
            return False

        self.is_connected = True
        self._set_status("Waiting for Clap")
        self._log("Connected wirelessly via Antenna ESP-NOW bridge.")
        self._log("Waiting for clap detection and audio stream...")
        return True

    def disconnect(self) -> None:
        self.is_connected = False
        self.expected_audio_size = None
        self.audio_buffer = bytearray()
        self._set_status("Disconnected")
        self._log("Disconnected.")

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
                self.last_trigger = "Double clap" if clap_type == 2 else "Single clap"
                self._set_status("Receiving Audio")
                self._log(f"Clap detected: {self.last_trigger}. Expecting {audio_size} bytes of WAV audio.")
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
                    
                    wav_path = Path(__file__).parent / "last_esp32_recording.wav"
                    try:
                        wav_path.write_bytes(audio_data)
                    except Exception:
                        pass
                    
                    self._set_status("Transcribing")
                    threading.Thread(target=self._transcribe_and_send, args=(audio_data,), daemon=True).start()
            except Exception as exc:
                self._log(f"Failed to parse audio chunk: {exc}")

    def _transcribe_and_send(self, audio_data: bytes) -> None:
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
