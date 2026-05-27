from __future__ import annotations

import json
import queue
import time
import tkinter as tk
from tkinter import ttk
from typing import Any

from config import load_app_config
from serial_bridge import SerialBridge


DEVICE_LABELS = {
    "antenna": "Antenna USB",
    "wristband": "Wristband ESP-NOW",
    "camdock": "Cam Dock ESP-NOW",
    "keyboard": "Keyboard",
    "charging_dock": "Charging Dock TBD",
    "audiodock": "Audio Dock TBD",
    "fans": "Fans TBD",
}


class ConnectionStatusApp:
    def __init__(self, root: tk.Tk, serial_bridge: SerialBridge) -> None:
        self.root = root
        self.serial_bridge = serial_bridge
        self.log_queue: queue.Queue[str] = queue.Queue()
        self.latest_state: dict[str, Any] = {}
        self.last_state_time: float | None = None
        self.device_rows: dict[str, dict[str, ttk.Label]] = {}

        self.serial_bridge.on_log = self.log
        self.serial_bridge.on_state = self.on_state

        self.root.title("AirTrixx Connection Status")
        self.root.geometry("900x620")
        self.root.protocol("WM_DELETE_WINDOW", self.close)

        self._build_ui()
        self.refresh_ports()
        self._tick()

    def _build_ui(self) -> None:
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(2, weight=1)

        top = ttk.Frame(self.root, padding=10)
        top.grid(row=0, column=0, sticky="ew")
        top.columnconfigure(1, weight=1)

        ttk.Label(top, text="COM Port").grid(row=0, column=0, sticky="w")
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(top, textvariable=self.port_var, width=44)
        self.port_combo.grid(row=0, column=1, sticky="ew", padx=(8, 8))
        ttk.Button(top, text="Refresh", command=self.refresh_ports).grid(row=0, column=2, padx=(0, 8))
        self.connect_button = ttk.Button(top, text="Connect", command=self.toggle_serial)
        self.connect_button.grid(row=0, column=3)

        summary = ttk.LabelFrame(self.root, text="Link Summary", padding=10)
        summary.grid(row=1, column=0, sticky="ew", padx=10)
        summary.columnconfigure(1, weight=1)
        summary.columnconfigure(3, weight=1)

        self.usb_status_var = tk.StringVar(value="USB: disconnected")
        self.stream_status_var = tk.StringVar(value="JSON stream: no data")
        self.last_packet_var = tk.StringVar(value="Last packet: never")
        self.sequence_var = tk.StringVar(value="Antenna sequence: unknown")

        ttk.Label(summary, textvariable=self.usb_status_var).grid(row=0, column=0, sticky="w", padx=(0, 24))
        ttk.Label(summary, textvariable=self.stream_status_var).grid(row=0, column=1, sticky="w")
        ttk.Label(summary, textvariable=self.last_packet_var).grid(row=1, column=0, sticky="w", padx=(0, 24))
        ttk.Label(summary, textvariable=self.sequence_var).grid(row=1, column=1, sticky="w")

        body = ttk.PanedWindow(self.root, orient="horizontal")
        body.grid(row=2, column=0, sticky="nsew", padx=10, pady=10)

        status_frame = ttk.LabelFrame(body, text="Devices", padding=8)
        raw_frame = ttk.LabelFrame(body, text="Latest Antenna JSON", padding=8)
        body.add(status_frame, weight=1)
        body.add(raw_frame, weight=2)

        headers = ["Device", "Status", "Age", "Battery", "Sequence", "Details"]
        for col, header in enumerate(headers):
            ttk.Label(status_frame, text=header).grid(row=0, column=col, sticky="w", padx=4, pady=(0, 4))
        for col in range(len(headers)):
            status_frame.columnconfigure(col, weight=1 if col in (0, 5) else 0)

        for row, device in enumerate(DEVICE_LABELS, start=1):
            labels = {
                "device": ttk.Label(status_frame, text=DEVICE_LABELS[device]),
                "status": ttk.Label(status_frame, text="unknown"),
                "age": ttk.Label(status_frame, text="-"),
                "battery": ttk.Label(status_frame, text="-"),
                "sequence": ttk.Label(status_frame, text="-"),
                "details": ttk.Label(status_frame, text="-"),
            }
            for col, key in enumerate(["device", "status", "age", "battery", "sequence", "details"]):
                labels[key].grid(row=row, column=col, sticky="w", padx=4, pady=3)
            self.device_rows[device] = labels

        raw_frame.rowconfigure(0, weight=1)
        raw_frame.columnconfigure(0, weight=1)
        self.raw_text = tk.Text(raw_frame, wrap="none", height=18)
        self.raw_text.grid(row=0, column=0, sticky="nsew")

        log_frame = ttk.LabelFrame(self.root, text="Log", padding=8)
        log_frame.grid(row=3, column=0, sticky="ew", padx=10, pady=(0, 10))
        log_frame.columnconfigure(0, weight=1)
        self.log_text = tk.Text(log_frame, height=6, wrap="word")
        self.log_text.grid(row=0, column=0, sticky="ew")

    def refresh_ports(self) -> None:
        ports = self.serial_bridge.available_ports()
        values = [f'{p["device"]} - {p["description"]}' for p in ports]
        self.port_combo["values"] = values
        if values and not self.port_var.get():
            self.port_var.set(values[0])
        self.log(f"Found {len(values)} COM port(s).")

    def toggle_serial(self) -> None:
        if self.serial_bridge.is_connected:
            self.serial_bridge.disconnect()
            return
        selected = self.port_var.get().split(" - ", 1)[0].strip() or None
        self.serial_bridge.connect(selected)

    def on_state(self, state: dict[str, Any]) -> None:
        self.latest_state = state
        self.last_state_time = time.monotonic()

    def log(self, message: str) -> None:
        timestamp = time.strftime("%H:%M:%S")
        self.log_queue.put(f"[{timestamp}] {message}")

    def close(self) -> None:
        self.serial_bridge.disconnect()
        self.root.destroy()

    def _tick(self) -> None:
        self._drain_log_queue()
        self._update_summary()
        self._update_devices()
        self._update_raw_json()
        self.connect_button.configure(text="Disconnect" if self.serial_bridge.is_connected else "Connect")
        self.root.after(250, self._tick)

    def _update_summary(self) -> None:
        connected = self.serial_bridge.is_connected
        port = self.serial_bridge.current_port or "none"
        self.usb_status_var.set(f"USB: {'connected' if connected else 'disconnected'} ({port})")

        if self.last_state_time is None:
            self.stream_status_var.set("JSON stream: no data")
            self.last_packet_var.set("Last packet: never")
        else:
            age = time.monotonic() - self.last_state_time
            stream = "ok" if age < 1.0 else "stale"
            self.stream_status_var.set(f"JSON stream: {stream}")
            self.last_packet_var.set(f"Last packet: {age:.1f}s ago")

        sequence = self.latest_state.get("sequence")
        self.sequence_var.set(f"Antenna sequence: {sequence if sequence is not None else 'unknown'}")

    def _update_devices(self) -> None:
        devices = self.latest_state.get("devices", {}) if isinstance(self.latest_state, dict) else {}
        now_ms = self.latest_state.get("t_ms")

        self._set_row(
            "antenna",
            "ok" if self.serial_bridge.is_connected else "disconnected",
            self._stream_age_text(),
            "-",
            self.latest_state.get("sequence", "-"),
            "USB serial JSON stream",
        )

        for device in DEVICE_LABELS:
            if device == "antenna":
                continue
            payload = devices.get(device, {}) if isinstance(devices, dict) else {}
            if not isinstance(payload, dict):
                self._set_row(device, "unknown", "-", "-", "-", "-")
                continue

            status = str(payload.get("status", "unknown"))
            battery = self._format_battery(payload)
            sequence = payload.get("sequence", "-")
            device_t_ms = payload.get("t_ms")
            age_text = self._device_age_text(now_ms, device_t_ms)
            details = self._device_details(device, payload)
            self._set_row(device, status, age_text, str(battery), sequence, details)

    def _set_row(
        self,
        device: str,
        status: str,
        age: str,
        battery: str,
        sequence: Any,
        details: str,
    ) -> None:
        row = self.device_rows[device]
        row["status"].configure(text=status)
        row["age"].configure(text=age)
        row["battery"].configure(text=battery)
        row["sequence"].configure(text=str(sequence))
        row["details"].configure(text=details)

    def _device_age_text(self, now_ms: Any, device_t_ms: Any) -> str:
        if not isinstance(now_ms, int) or not isinstance(device_t_ms, int):
            return "-"
        age_ms = max(0, now_ms - device_t_ms)
        return f"{age_ms} ms"

    def _stream_age_text(self) -> str:
        if self.last_state_time is None:
            return "-"
        return f"{time.monotonic() - self.last_state_time:.1f}s"

    def _device_details(self, device: str, payload: dict[str, Any]) -> str:
        if device == "camdock":
            tof = payload.get("tof", {})
            if isinstance(tof, dict):
                return (
                    f"L {tof.get('left_mm', '-')} mm, "
                    f"R {tof.get('right_mm', '-')} mm, "
                    f"target {payload.get('active_target', '-')}"
                )
        if device == "wristband":
            return f"pitch {payload.get('pitch', '-')}, roll {payload.get('roll', '-')}"
        return str(payload.get("input", "-"))

    def _format_battery(self, payload: dict[str, Any]) -> str:
        battery = payload.get("battery")
        if isinstance(battery, dict):
            status = battery.get("status")
            percent = battery.get("percent")
            voltage = battery.get("voltage_v")
            if isinstance(percent, (int, float)) and isinstance(voltage, (int, float)):
                suffix = "" if status == "ok" else f" {status}"
                return f"{percent:.0f}% ({voltage:.2f}V){suffix}"
            if status:
                return str(status)

        level = payload.get("battery_level")
        voltage = payload.get("battery_voltage")
        if isinstance(level, (int, float)) and isinstance(voltage, (int, float)):
            return f"{level:.0f}% ({voltage:.2f}V)"
        if level is not None:
            return str(level)
        return "-"

    def _update_raw_json(self) -> None:
        text = json.dumps(self.latest_state, indent=2) if self.latest_state else "{}"
        self.raw_text.delete("1.0", "end")
        self.raw_text.insert("1.0", text)

    def _drain_log_queue(self) -> None:
        while True:
            try:
                message = self.log_queue.get_nowait()
            except queue.Empty:
                break
            self.log_text.insert("end", message + "\n")
            self.log_text.see("end")


def main() -> None:
    config = load_app_config()
    bridge = SerialBridge(baud_rate=config.serial_baud)
    root = tk.Tk()
    ConnectionStatusApp(root, bridge)
    root.mainloop()


if __name__ == "__main__":
    main()
