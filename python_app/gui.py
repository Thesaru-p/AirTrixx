from __future__ import annotations

import hashlib
import http.server
import json
import queue
import shutil
import socket
import subprocess
import threading
import time
import tkinter as tk
from tkinter import ttk
from pathlib import Path
from typing import Any

from PIL import Image, ImageDraw, ImageTk

from config import APP_DIR, AppConfig, load_calibration, save_calibration
from fusion_state import FIELD_ORDER, FusionState
from gesture_recorder import GestureRecorder
from mediapipe_tracker import HandTracker
from serial_bridge import SerialBridge
from servo_controller import ServoController


SERVO_BRACKETS = {
    "right": "Right Bracket",
    "left": "Left Bracket",
    "camera": "Camera Bracket",
}

CAMERA_SOURCE_INDICES = {
    "Auto (first available)": -1,
    "USB camera": 1,
    "Built-in camera": 0,
}

CAMERA_CENTER_TARGET_X = 0.50
CAMERA_CENTER_TARGET_FACE_TOP_Y = 0.12
CAMERA_CENTER_DEADBAND_X = 0.045
CAMERA_CENTER_DEADBAND_Y = 0.045
CAMERA_CENTER_TIMEOUT_S = 30.0
CAMERA_CENTER_SETTLED_S = 0.8
CAMERA_CENTER_COMMAND_INTERVAL_S = 0.12
CAMERA_CENTER_MAX_STEP_TICKS = 10
CAMERA_SEARCH_COMMAND_INTERVAL_S = 0.65
CAMERA_SEARCH_PAN_RANGE_TICKS = 75
CAMERA_SEARCH_PAN_STEP_TICKS = 25
CAMERA_SEARCH_UP_DEG = 40.0
CAMERA_SEARCH_DOWN_DEG = 25.0
CAMERA_SEARCH_TILT_STEP_DEG = 12.0
SERIAL_AUTOCONNECT_DELAY_MS = 250
SERIAL_AUTOCONNECT_RETRY_MS = 1000
LIVE_DATA_HISTORY_ROWS = 10
KEYBOARD_DISTANCE_ROWS = 30
KEYBOARD_DISTANCE_BAND_MM = 10
SERVO_DEBUG_INTERVAL_S = 0.2
SERVO_DEBUG_LOG_LIMIT = 600
SERVO_DEBUG_LOG_PATH = APP_DIR / "data" / "servo_debug.log"
WRISTBAND_FIRMWARE_DIR = APP_DIR.parent / "firmware" / "wristband_esp32c3"
WRISTBAND_FIRMWARE_BIN = WRISTBAND_FIRMWARE_DIR / ".pio" / "build" / "esp32c3_supermini" / "firmware.bin"
FANS_FIRMWARE_DIR = APP_DIR.parent / "firmware" / "fan_controller_esp32c3"
FANS_FIRMWARE_BIN = FANS_FIRMWARE_DIR / ".pio" / "build" / "esp32c3_supermini" / "firmware.bin"
CAMDOCK_FIRMWARE_DIR = APP_DIR.parent / "firmware" / "camdock_esp32s3"
CAMDOCK_FIRMWARE_BIN = CAMDOCK_FIRMWARE_DIR / ".pio" / "build" / "esp32s3_camdock" / "firmware.bin"
HAND_CALIBRATION_POINTS = [
    ("top_left", "top left", 0.15, 0.18),
    ("top_right", "top right", 0.85, 0.18),
    ("bottom_right", "bottom right", 0.85, 0.82),
    ("bottom_left", "bottom left", 0.15, 0.82),
]
HAND_CALIBRATION_MIN_SPAN = 0.08
HAND_CALIBRATION_TARGET_RADIUS = 0.095
SESSION_CALIBRATION_SETTLED_S = 0.9
SESSION_CALIBRATION_MIN_SCORE = 0.45
CALIBRATION_ENTRY_KEYS = [
    "cam_pan_center",
    "cam_tilt_center",
    "r_pan_center",
    "r_tilt_center",
    "l_pan_center",
    "l_tilt_center",
    "x_gain_ticks",
    "y_gain_ticks",
    "deadband",
    "smoothing_alpha",
    "hand_boundary_left",
    "hand_boundary_right",
    "hand_boundary_top",
    "hand_boundary_bottom",
    "use_dock_geometry",
    "initial_hand_distance_mm",
    "min_valid_tof_mm",
    "max_valid_tof_mm",
    "tof_depth_alpha",
    "prediction_latency_ms",
    "camera_horizontal_fov_deg",
    "camera_vertical_fov_deg",
    "left_bracket_x_mm",
    "left_bracket_y_mm",
    "left_bracket_z_mm",
    "right_bracket_x_mm",
    "right_bracket_y_mm",
    "right_bracket_z_mm",
    "left_tilt_pivot_offset_x_mm",
    "left_tilt_pivot_offset_y_mm",
    "left_tilt_pivot_offset_z_mm",
    "right_tilt_pivot_offset_x_mm",
    "right_tilt_pivot_offset_y_mm",
    "right_tilt_pivot_offset_z_mm",
    "left_tof_offset_x_mm",
    "left_tof_offset_y_mm",
    "left_tof_offset_z_mm",
    "right_tof_offset_x_mm",
    "right_tof_offset_y_mm",
    "right_tof_offset_z_mm",
    "pan_ticks_per_degree",
    "tilt_ticks_per_degree",
    "r_pan_sign",
    "r_tilt_sign",
    "l_pan_sign",
    "l_tilt_sign",
    "r_pan_angle_offset_deg",
    "r_tilt_angle_offset_deg",
    "l_pan_angle_offset_deg",
    "l_tilt_angle_offset_deg",
]
FLOAT_CALIBRATION_KEYS = {
    "smoothing_alpha",
    "hand_boundary_left",
    "hand_boundary_right",
    "hand_boundary_top",
    "hand_boundary_bottom",
    "initial_hand_distance_mm",
    "min_valid_tof_mm",
    "max_valid_tof_mm",
    "tof_depth_alpha",
    "prediction_latency_ms",
    "camera_horizontal_fov_deg",
    "camera_vertical_fov_deg",
    "left_bracket_x_mm",
    "left_bracket_y_mm",
    "left_bracket_z_mm",
    "right_bracket_x_mm",
    "right_bracket_y_mm",
    "right_bracket_z_mm",
    "left_tilt_pivot_offset_x_mm",
    "left_tilt_pivot_offset_y_mm",
    "left_tilt_pivot_offset_z_mm",
    "right_tilt_pivot_offset_x_mm",
    "right_tilt_pivot_offset_y_mm",
    "right_tilt_pivot_offset_z_mm",
    "left_tof_offset_x_mm",
    "left_tof_offset_y_mm",
    "left_tof_offset_z_mm",
    "right_tof_offset_x_mm",
    "right_tof_offset_y_mm",
    "right_tof_offset_z_mm",
    "pan_ticks_per_degree",
    "tilt_ticks_per_degree",
    "r_pan_sign",
    "r_tilt_sign",
    "l_pan_sign",
    "l_tilt_sign",
    "r_pan_angle_offset_deg",
    "r_tilt_angle_offset_deg",
    "l_pan_angle_offset_deg",
    "l_tilt_angle_offset_deg",
}


class AirTrixxGUI:
    def __init__(
        self,
        root: tk.Tk,
        config: AppConfig,
        serial_bridge: SerialBridge,
        hand_tracker: HandTracker,
        servo_controller: ServoController,
        fusion_state: FusionState,
    ) -> None:
        self.root = root
        self.config = config
        self.serial_bridge = serial_bridge
        self.hand_tracker = hand_tracker
        self.servo_controller = servo_controller
        self.fusion_state = fusion_state
        self.log_queue: queue.Queue[str] = queue.Queue()
        self._photo: ImageTk.PhotoImage | None = None
        self._last_text_update_s = 0.0
        self._latest_snapshot: dict[str, Any] = {}
        self.centering_bracket: str | None = None
        self.centering_positions: dict[str, dict[str, int]] = {}
        self.bracket_buttons: dict[str, ttk.Button] = {}
        self.data_columns: list[str] = []
        self.data_history: list[list[tuple[str, str, str]]] = []
        self.servo_debug_lines: list[str] = []
        self.nav_buttons: dict[str, ttk.Button] = {}
        self.pages: dict[str, ttk.Frame] = {}
        self.keyboard_cells: list[list[tk.Label]] = []
        self.keyboard_status_var = tk.StringVar(value="Keyboard: waiting for ToF data.")
        self.active_page = "Dashboard"
        self.camera_popup: tk.Toplevel | None = None
        self.camera_popup_label: ttk.Label | None = None
        self._popup_photo: ImageTk.PhotoImage | None = None
        self._last_servo_debug_sequence = 0
        self._last_servo_debug_log_s = 0.0
        self.ota_server: http.server.ThreadingHTTPServer | None = None
        self.ota_server_thread: threading.Thread | None = None
        self.ota_in_progress = False
        self.fans_requested_on = False
        self.serial_autoconnect_enabled = True
        self.camera_centering_active = True
        self.camera_centering_started_s: float | None = None
        self.camera_centering_settled_s: float | None = None
        self.camera_centering_last_send_s = 0.0
        self.camera_centering_position: dict[str, int] = {}
        self.camera_search_anchor: dict[str, int] = {}
        self.camera_search_index = 0
        self.camera_search_last_send_s = 0.0
        self.camera_search_found_once = False
        self.startup_hand_calibration_pending = True
        self.hand_calibration_active = False
        self.hand_calibration_index = 0
        self.hand_calibration_points: dict[str, dict[str, Any]] = {}
        self.hand_calibration_fist_armed = False
        self.hand_calibration_fist_side: str | None = None
        self.hand_calibration_target_inside = False
        self.hand_calibration_seen_since_s: float | None = None
        self.hand_calibration_auto_capture = True

        self.serial_bridge.on_log = self.log
        self.hand_tracker.on_log = self.log
        self.recorder = GestureRecorder(
            self.config.gesture_data_dir,
            self._snapshot_provider,
            on_status=self.log,
        )

        self.root.title("AirTrixx Prototype Console")
        self.root.geometry("1280x860")
        self.root.minsize(1120, 760)
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.root.after(0, lambda: self.root.state("zoomed"))

        self._configure_styles()
        self._build_ui()
        self.refresh_ports()
        self.root.after(SERIAL_AUTOCONNECT_DELAY_MS, self.auto_connect_serial)
        self._tick()

    def log(self, message: str) -> None:
        self.log_queue.put(message)

    def _configure_styles(self) -> None:
        style = ttk.Style(self.root)
        if "clam" in style.theme_names():
            style.theme_use("clam")

        self.root.configure(bg="#f5f7fb")
        style.configure(".", font=("Segoe UI", 9), background="#f5f7fb", foreground="#1f2937")
        style.configure("TFrame", background="#f5f7fb")
        style.configure("Panel.TFrame", background="#ffffff")
        style.configure("Header.TLabel", font=("Segoe UI Semibold", 18), background="#f5f7fb", foreground="#111827")
        style.configure("Subtle.TLabel", background="#f5f7fb", foreground="#6b7280")
        style.configure("Value.TLabel", font=("Segoe UI Semibold", 10), background="#ffffff", foreground="#111827")
        style.configure("TLabelframe", background="#ffffff", bordercolor="#d7dce5", relief="solid")
        style.configure("TLabelframe.Label", font=("Segoe UI Semibold", 10), background="#f5f7fb", foreground="#374151")
        style.configure("TButton", padding=(10, 6))
        style.configure("Nav.TButton", padding=(14, 10), anchor="w", background="#f8fafc", foreground="#1f2937")
        style.configure("NavActive.TButton", padding=(14, 10), anchor="w", background="#dbeafe", foreground="#1d4ed8")
        style.configure("Accent.TButton", padding=(10, 7), background="#2563eb", foreground="#ffffff")
        style.map("Accent.TButton", background=[("active", "#1d4ed8"), ("disabled", "#9ca3af")])
        style.configure("Secondary.TButton", padding=(10, 6), background="#eef2ff", foreground="#1f2937")
        style.configure("TEntry", padding=(6, 4), fieldbackground="#ffffff")
        style.configure("TCombobox", padding=(6, 4), fieldbackground="#ffffff")
        style.configure("Treeview", rowheight=24, background="#ffffff", fieldbackground="#ffffff", foreground="#1f2937")
        style.configure("Treeview.Heading", font=("Segoe UI Semibold", 9), background="#eef2f7", foreground="#374151")
        style.configure("TNotebook", background="#f5f7fb", borderwidth=0)
        style.configure("TNotebook.Tab", padding=(12, 6), font=("Segoe UI", 9))

    @staticmethod
    def _style_text_widget(widget: tk.Text, *, dark: bool = False) -> None:
        if dark:
            widget.configure(
                bg="#111827",
                fg="#e5e7eb",
                insertbackground="#e5e7eb",
                selectbackground="#374151",
            )
        else:
            widget.configure(
                bg="#ffffff",
                fg="#1f2937",
                insertbackground="#1f2937",
                selectbackground="#dbeafe",
            )
        widget.configure(
            relief="flat",
            borderwidth=1,
            padx=8,
            pady=6,
            font=("Consolas", 9),
        )

    def _build_ui(self) -> None:
        self._scroll_targets: dict[str, Any] = {}
        self.root.columnconfigure(0, weight=0)
        self.root.columnconfigure(1, weight=1)
        self.root.rowconfigure(0, weight=1)

        sidebar = ttk.Frame(self.root, padding=(14, 14), style="Panel.TFrame")
        sidebar.grid(row=0, column=0, sticky="ns")
        sidebar.columnconfigure(0, weight=1)

        ttk.Label(sidebar, text="AirTrixx", style="Header.TLabel").grid(row=0, column=0, sticky="w")
        ttk.Label(sidebar, text="Prototype Console", style="Subtle.TLabel").grid(row=1, column=0, sticky="w")

        serial_box = ttk.LabelFrame(sidebar, text="Antenna Serial", padding=8)
        serial_box.grid(row=2, column=0, sticky="ew", pady=(14, 12))
        serial_box.columnconfigure(0, weight=1)
        serial_box.columnconfigure(1, weight=1)
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(serial_box, textvariable=self.port_var, width=26)
        self.port_combo.grid(row=0, column=0, columnspan=2, sticky="ew", pady=(0, 8))
        ttk.Button(serial_box, text="Refresh", command=self.refresh_ports, style="Secondary.TButton").grid(
            row=1, column=0, sticky="ew", padx=(0, 4)
        )
        self.connect_button = ttk.Button(serial_box, text="Connect", command=self.toggle_serial, style="Accent.TButton")
        self.connect_button.grid(row=1, column=1, sticky="ew", padx=(4, 0))

        nav_frame = ttk.Frame(sidebar, style="Panel.TFrame")
        nav_frame.grid(row=3, column=0, sticky="ew")
        nav_items = ("Dashboard", "Camera", "Keyboard", "Live Data", "Servo Control", "Firmware", "Calibration", "Logs / Debug")
        for row, name in enumerate(nav_items):
            button = ttk.Button(nav_frame, text=name, style="Nav.TButton", command=lambda page=name: self.show_page(page))
            button.grid(row=row, column=0, sticky="ew", pady=(0, 4))
            self.nav_buttons[name] = button
        nav_frame.columnconfigure(0, weight=1)

        content = ttk.Frame(self.root, padding=(14, 14))
        content.grid(row=0, column=1, sticky="nsew")
        content.rowconfigure(0, weight=1)
        content.columnconfigure(0, weight=1)

        for name in nav_items:
            page = ttk.Frame(content)
            page.grid(row=0, column=0, sticky="nsew")
            page.rowconfigure(1, weight=1)
            page.columnconfigure(0, weight=1)
            self.pages[name] = page

        self._build_dashboard_page(self.pages["Dashboard"])
        self._build_camera_page(self.pages["Camera"])
        self._build_keyboard_page(self.pages["Keyboard"])
        self._build_live_data_page(self.pages["Live Data"])
        self._build_servo_page(self.pages["Servo Control"])
        self._build_firmware_page(self.pages["Firmware"])
        self._build_calibration_page(self.pages["Calibration"])
        self._build_logs_page(self.pages["Logs / Debug"])

        self.root.bind_all("<MouseWheel>", self._on_mousewheel, add="+")
        self.root.bind_all("<Button-4>", self._on_mousewheel, add="+")
        self.root.bind_all("<Button-5>", self._on_mousewheel, add="+")
        self.root.bind_all("<Up>", self._handle_centering_arrow)
        self.root.bind_all("<Down>", self._handle_centering_arrow)
        self.root.bind_all("<Left>", self._handle_centering_arrow)
        self.root.bind_all("<Right>", self._handle_centering_arrow)
        self.show_page(self.active_page)

    def _build_page_header(self, parent: ttk.Frame, title: str, subtitle: str) -> ttk.Frame:
        header = ttk.Frame(parent)
        header.grid(row=0, column=0, sticky="ew", pady=(0, 12))
        ttk.Label(header, text=title, style="Header.TLabel").grid(row=0, column=0, sticky="w")
        ttk.Label(header, text=subtitle, style="Subtle.TLabel").grid(row=1, column=0, sticky="w", pady=(2, 0))
        return header

    def _scrollable_body(self, parent: ttk.Frame) -> ttk.Frame:
        canvas = tk.Canvas(parent, highlightthickness=0, bg="#f5f7fb")
        canvas.grid(row=1, column=0, sticky="nsew")
        scrollbar = ttk.Scrollbar(parent, orient="vertical", command=canvas.yview)
        scrollbar.grid(row=1, column=1, sticky="ns")
        canvas.configure(yscrollcommand=scrollbar.set)
        body = ttk.Frame(canvas, padding=(0, 0, 10, 0))
        window = canvas.create_window((0, 0), window=body, anchor="nw")
        body.bind("<Configure>", lambda _event: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.bind("<Configure>", lambda event: canvas.itemconfigure(window, width=event.width))
        body.columnconfigure(0, weight=1)
        self._register_scroll_target(canvas, canvas)
        self._register_scroll_target(body, canvas)
        return body

    def _build_dashboard_page(self, page: ttk.Frame) -> None:
        self._build_page_header(page, "Dashboard", "Daily controls and current device state.")
        body = self._scrollable_body(page)
        body.columnconfigure(0, weight=1)
        body.columnconfigure(1, weight=1)

        fan_box = ttk.LabelFrame(body, text="Fans", padding=10)
        fan_box.grid(row=0, column=0, sticky="nsew", padx=(0, 8), pady=(0, 10))
        fan_box.columnconfigure(0, weight=1)
        self.fan_status_var = tk.StringVar(value="Fans: waiting for controller.")
        self.fan_button = ttk.Button(fan_box, text="Turn Fans On", command=self.toggle_fans, style="Accent.TButton")
        self.fan_button.grid(row=0, column=0, sticky="ew")
        ttk.Label(fan_box, textvariable=self.fan_status_var, wraplength=420).grid(row=1, column=0, sticky="ew", pady=(8, 0))

        camera_box = ttk.LabelFrame(body, text="Camera", padding=10)
        camera_box.grid(row=0, column=1, sticky="nsew", padx=(8, 0), pady=(0, 10))
        camera_box.columnconfigure(0, weight=1)
        self.camera_centering_status_var = tk.StringVar(value="Camera centering: waiting for USB camera and Antenna link.")
        ttk.Label(camera_box, textvariable=self.camera_centering_status_var, wraplength=420).grid(row=0, column=0, sticky="ew")
        ttk.Button(camera_box, text="Run Camera Centering", command=self.start_camera_centering, style="Accent.TButton").grid(
            row=1, column=0, sticky="ew", pady=(8, 0)
        )

        quick_box = ttk.LabelFrame(body, text="Quick Status", padding=10)
        quick_box.grid(row=1, column=0, columnspan=2, sticky="nsew")
        quick_box.rowconfigure(0, weight=1)
        quick_box.columnconfigure(0, weight=1)
        self.dashboard_text = tk.Text(quick_box, height=12, wrap="word")
        self.dashboard_text.grid(row=0, column=0, sticky="nsew")
        self._style_text_widget(self.dashboard_text)
        self._register_scroll_target(self.dashboard_text, self.dashboard_text)

    def _build_camera_page(self, page: ttk.Frame) -> None:
        self._build_page_header(page, "Camera", "USB feed, face centering, mirroring, and pop-out view.")
        body = self._scrollable_body(page)
        body.columnconfigure(0, weight=1)
        controls = ttk.LabelFrame(body, text="Camera Controls", padding=10)
        controls.grid(row=0, column=0, sticky="ew", pady=(0, 10))
        controls.columnconfigure(1, weight=1)
        self.camera_source_var = tk.StringVar(value=self._camera_source_for_index(self.config.camera_index))
        ttk.Label(controls, text="Camera source").grid(row=0, column=0, sticky="w", padx=(0, 8))
        self.camera_source_combo = ttk.Combobox(
            controls,
            textvariable=self.camera_source_var,
            values=list(CAMERA_SOURCE_INDICES.keys()),
            state="readonly",
            width=18,
        )
        self.camera_source_combo.grid(row=0, column=1, sticky="w")
        self.camera_source_combo.bind("<<ComboboxSelected>>", self.on_camera_source_changed)
        self.camera_mirror_var = tk.BooleanVar(value=self.hand_tracker.mirror_preview)
        self.mirror_button = ttk.Button(
            controls,
            text=f"Mirror: {'On' if self.camera_mirror_var.get() else 'Off'}",
            command=self.toggle_camera_mirror,
        )
        self.mirror_button.grid(row=0, column=2, sticky="e", padx=(8, 0))
        ttk.Button(controls, text="Run Camera Centering", command=self.start_camera_centering).grid(
            row=0, column=3, sticky="e", padx=(8, 0)
        )
        ttk.Button(controls, text="Pop Out Feed", command=self.open_camera_popup).grid(
            row=0, column=4, sticky="e", padx=(8, 0)
        )
        ttk.Label(controls, textvariable=self.camera_centering_status_var).grid(
            row=1, column=0, columnspan=5, sticky="w", pady=(8, 0)
        )

        preview_box = ttk.LabelFrame(body, text="Live Camera Feed", padding=10)
        preview_box.grid(row=1, column=0, sticky="nsew")
        preview_box.columnconfigure(0, weight=1)
        self.preview_label = ttk.Label(preview_box)
        self.preview_label.grid(row=0, column=0, sticky="w")

    def _build_keyboard_page(self, page: ttk.Frame) -> None:
        self._build_page_header(page, "Keyboard", "Three ToF lanes mapped from 0 to 300 mm in 10 mm bands.")
        body = self._scrollable_body(page)
        body.columnconfigure(0, weight=1)

        status_box = ttk.LabelFrame(body, text="Keyboard Status", padding=10)
        status_box.grid(row=0, column=0, sticky="ew", pady=(0, 10))
        status_box.columnconfigure(0, weight=1)
        ttk.Label(status_box, textvariable=self.keyboard_status_var, wraplength=760).grid(row=0, column=0, sticky="ew")

        grid_box = ttk.LabelFrame(body, text="ToF Distance Grid", padding=10)
        grid_box.grid(row=1, column=0, sticky="ew")
        for col in range(5):
            grid_box.columnconfigure(col, weight=1 if col > 0 else 0)

        header_style = {"bg": "#e7edf7", "fg": "#1f2d3d", "font": ("Segoe UI", 10, "bold")}
        cell_style = {"bg": "#f8fafc", "fg": "#1f2d3d", "font": ("Segoe UI", 10), "relief": "solid", "bd": 1}
        tk.Label(grid_box, text="Band", padx=8, pady=8, **header_style).grid(row=0, column=0, sticky="nsew", padx=1, pady=1)
        for col in range(4):
            tk.Label(grid_box, text=f"Sensor {col + 1}", padx=8, pady=8, **header_style).grid(
                row=0, column=col + 1, sticky="nsew", padx=1, pady=1
            )

        self.keyboard_cells = []
        for row in range(KEYBOARD_DISTANCE_ROWS):
            start_mm = row * KEYBOARD_DISTANCE_BAND_MM
            end_mm = start_mm + KEYBOARD_DISTANCE_BAND_MM - 1
            if row == KEYBOARD_DISTANCE_ROWS - 1:
                end_mm = start_mm + KEYBOARD_DISTANCE_BAND_MM
            tk.Label(grid_box, text=f"{start_mm}-{end_mm} mm", padx=8, pady=7, **cell_style).grid(
                row=row + 1, column=0, sticky="nsew", padx=1, pady=1
            )
            cells: list[tk.Label] = []
            for col in range(4):
                label = tk.Label(grid_box, text="", width=14, padx=8, pady=7, **cell_style)
                label.grid(row=row + 1, column=col + 1, sticky="nsew", padx=1, pady=1)
                cells.append(label)
            self.keyboard_cells.append(cells)

    def _build_live_data_page(self, page: ttk.Frame) -> None:
        self._build_page_header(page, "Live Data", "Current and previous input snapshots grouped by device.")
        data_box = ttk.LabelFrame(page, text="Inputs", padding=10)
        data_box.grid(row=1, column=0, sticky="nsew")
        data_box.rowconfigure(1, weight=1)
        data_box.columnconfigure(0, weight=1)
        filter_bar = ttk.Frame(data_box)
        filter_bar.grid(row=0, column=0, columnspan=2, sticky="ew", pady=(0, 8))
        filter_bar.columnconfigure(3, weight=1)
        self.live_data_device_var = tk.StringVar(value="All")
        self.live_data_search_var = tk.StringVar(value="")
        ttk.Label(filter_bar, text="Device").grid(row=0, column=0, sticky="w", padx=(0, 6))
        self.live_data_device_combo = ttk.Combobox(
            filter_bar,
            textvariable=self.live_data_device_var,
            values=("All", "Antenna", "Wristband", "Cam Dock", "Keyboard", "Fans", "Camera", "Calibration", "MediaPipe", "Gesture Recorder", "Fused Input"),
            state="readonly",
            width=18,
        )
        self.live_data_device_combo.grid(row=0, column=1, sticky="w", padx=(0, 14))
        ttk.Label(filter_bar, text="Search").grid(row=0, column=2, sticky="w", padx=(0, 6))
        self.live_data_search_entry = ttk.Entry(filter_bar, textvariable=self.live_data_search_var)
        self.live_data_search_entry.grid(row=0, column=3, sticky="ew")
        self.live_data_device_combo.bind("<<ComboboxSelected>>", lambda _event: self._update_text_views())
        self.live_data_search_var.trace_add("write", lambda *_args: self._update_text_views())

        self.data_tree = ttk.Treeview(data_box, columns=("device", "input"), show="headings", height=18)
        self.data_tree.grid(row=1, column=0, sticky="nsew")
        data_scroll = ttk.Scrollbar(data_box, orient="vertical", command=self.data_tree.yview)
        data_scroll.grid(row=1, column=1, sticky="ns")
        data_xscroll = ttk.Scrollbar(data_box, orient="horizontal", command=self.data_tree.xview)
        data_xscroll.grid(row=2, column=0, sticky="ew")
        self.data_tree.configure(yscrollcommand=data_scroll.set, xscrollcommand=data_xscroll.set)
        self._register_scroll_target(self.data_tree, self.data_tree)

    def _build_servo_page(self, page: ttk.Frame) -> None:
        self._build_page_header(page, "Servo Control", "Manual centering, neutral calibration, and gesture recording.")
        body = self._scrollable_body(page)
        body.columnconfigure(0, weight=1)
        centering_box = ttk.LabelFrame(body, text="Servo Centering", padding=10)
        centering_box.grid(row=0, column=0, sticky="ew", pady=(0, 10))
        centering_box.columnconfigure(0, weight=1)
        centering_box.columnconfigure(1, weight=1)
        centering_box.columnconfigure(2, weight=1)
        for col, (bracket, label) in enumerate(SERVO_BRACKETS.items()):
            button = ttk.Button(centering_box, text=label, command=lambda selected=bracket: self.select_servo_bracket(selected))
            button.grid(row=0, column=col, sticky="ew", padx=(0 if col == 0 else 4, 0))
            self.bracket_buttons[bracket] = button
        self.selected_bracket_var = tk.StringVar(value="Auto hand tracking")
        self.center_pan_var = tk.StringVar(value="-")
        self.center_tilt_var = tk.StringVar(value="-")
        self.center_step_var = tk.StringVar(value="5")
        self.centering_status_var = tk.StringVar(value="Select a bracket to use arrow keys.")
        ttk.Label(centering_box, textvariable=self.selected_bracket_var).grid(row=1, column=0, columnspan=3, sticky="w", pady=(8, 0))
        ttk.Label(centering_box, text="Pan").grid(row=2, column=0, sticky="w", pady=(6, 0))
        ttk.Label(centering_box, textvariable=self.center_pan_var, style="Value.TLabel").grid(row=2, column=1, sticky="w", pady=(6, 0))
        ttk.Label(centering_box, text="Tilt").grid(row=3, column=0, sticky="w")
        ttk.Label(centering_box, textvariable=self.center_tilt_var, style="Value.TLabel").grid(row=3, column=1, sticky="w")
        ttk.Label(centering_box, text="Step").grid(row=4, column=0, sticky="w", pady=(6, 0))
        ttk.Spinbox(centering_box, from_=1, to=100, increment=1, textvariable=self.center_step_var, width=8).grid(
            row=4, column=1, sticky="w", pady=(6, 0)
        )
        ttk.Button(centering_box, text="Save Center Position", command=self.save_center_position).grid(
            row=5, column=0, columnspan=3, sticky="ew", pady=(8, 0)
        )
        ttk.Button(centering_box, text="Return to Auto Tracking", command=self.resume_auto_tracking).grid(
            row=6, column=0, columnspan=3, sticky="ew", pady=(4, 0)
        )
        ttk.Label(centering_box, textvariable=self.centering_status_var, wraplength=760).grid(
            row=7, column=0, columnspan=3, sticky="ew", pady=(6, 0)
        )

        hand_calibration_box = ttk.LabelFrame(body, text="Calibration Phase", padding=10)
        hand_calibration_box.grid(row=1, column=0, sticky="ew", pady=(0, 10))
        hand_calibration_box.columnconfigure(0, weight=1)
        hand_calibration_box.columnconfigure(1, weight=1)
        self.hand_calibration_status_var = tk.StringVar(value="Calibration phase: waiting for camera centering.")
        ttk.Label(hand_calibration_box, textvariable=self.hand_calibration_status_var, wraplength=760).grid(
            row=0, column=0, columnspan=2, sticky="ew"
        )
        ttk.Button(hand_calibration_box, text="Run Calibration", command=self.start_hand_calibration).grid(
            row=1, column=0, sticky="ew", pady=(8, 0), padx=(0, 4)
        )
        self.skip_calibration_button = ttk.Button(hand_calibration_box, text="Skip Calibration", command=self.skip_hand_calibration)
        self.skip_calibration_button.grid(row=1, column=1, sticky="ew", pady=(8, 0))

        record_box = ttk.LabelFrame(body, text="Gesture Recorder", padding=10)
        record_box.grid(row=2, column=0, sticky="ew")
        record_box.columnconfigure(1, weight=1)
        ttk.Label(record_box, text="Gesture name").grid(row=0, column=0, sticky="w", padx=(0, 8))
        self.gesture_name_var = tk.StringVar()
        ttk.Entry(record_box, textvariable=self.gesture_name_var).grid(row=0, column=1, sticky="ew")
        ttk.Label(record_box, text="Repetitions").grid(row=1, column=0, sticky="w", pady=(6, 0), padx=(0, 8))
        self.repetitions_var = tk.StringVar(value="3")
        ttk.Entry(record_box, textvariable=self.repetitions_var, width=10).grid(row=1, column=1, sticky="w", pady=(6, 0))
        ttk.Label(record_box, text="Duration seconds").grid(row=2, column=0, sticky="w", pady=(6, 0), padx=(0, 8))
        self.duration_var = tk.StringVar(value="2.0")
        ttk.Entry(record_box, textvariable=self.duration_var, width=10).grid(row=2, column=1, sticky="w", pady=(6, 0))
        self.record_button = ttk.Button(record_box, text="Record Gesture", command=self.record_gesture, style="Accent.TButton")
        self.record_button.grid(row=3, column=0, columnspan=2, sticky="ew", pady=(8, 0))

    def _build_firmware_page(self, page: ttk.Frame) -> None:
        self._build_page_header(page, "Firmware", "Wireless firmware updates through the antenna bridge.")
        body = self._scrollable_body(page)
        box = ttk.LabelFrame(body, text="Wireless Flash", padding=10)
        box.grid(row=0, column=0, sticky="ew")
        box.columnconfigure(1, weight=1)
        self.ota_ssid_var = tk.StringVar(value=str(self.config.calibration.get("ota_wifi_ssid", "")))
        self.ota_password_var = tk.StringVar(value=str(self.config.calibration.get("ota_wifi_password", "")))
        ttk.Label(box, text="Wi-Fi SSID").grid(row=0, column=0, sticky="w", padx=(0, 8))
        ttk.Entry(box, textvariable=self.ota_ssid_var).grid(row=0, column=1, sticky="ew")
        ttk.Label(box, text="Password").grid(row=1, column=0, sticky="w", padx=(0, 8), pady=(8, 0))
        ttk.Entry(box, textvariable=self.ota_password_var, show="*").grid(row=1, column=1, sticky="ew", pady=(8, 0))
        self.ota_button = ttk.Button(box, text="Wireless Flash Wristband", command=self.start_wristband_wireless_flash, style="Secondary.TButton")
        self.ota_button.grid(row=2, column=0, columnspan=2, sticky="ew", pady=(12, 0))
        self.camdock_ota_button = ttk.Button(box, text="Wireless Flash Cam Dock", command=self.start_camdock_wireless_flash, style="Secondary.TButton")
        self.camdock_ota_button.grid(row=3, column=0, columnspan=2, sticky="ew", pady=(6, 0))
        self.fans_ota_button = ttk.Button(box, text="Wireless Flash Fans", command=self.start_fans_wireless_flash, style="Secondary.TButton")
        self.fans_ota_button.grid(row=4, column=0, columnspan=2, sticky="ew", pady=(6, 0))

    def _build_calibration_page(self, page: ttk.Frame) -> None:
        self._build_page_header(page, "Calibration", "Dock geometry and servo tuning values.")
        page.rowconfigure(1, weight=1)
        page.columnconfigure(0, weight=1)
        canvas = tk.Canvas(page, highlightthickness=0, bg="#ffffff")
        canvas.grid(row=1, column=0, sticky="nsew")
        scrollbar = ttk.Scrollbar(page, orient="vertical", command=canvas.yview)
        scrollbar.grid(row=1, column=1, sticky="ns")
        canvas.configure(yscrollcommand=scrollbar.set)
        inner = ttk.Frame(canvas, padding=10, style="Panel.TFrame")
        window = canvas.create_window((0, 0), window=inner, anchor="nw")
        inner.bind("<Configure>", lambda _event: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.bind("<Configure>", lambda event: canvas.itemconfigure(window, width=event.width))
        inner.columnconfigure(1, weight=1)
        self.calibration_vars: dict[str, tk.StringVar] = {}
        for row, key in enumerate(CALIBRATION_ENTRY_KEYS):
            ttk.Label(inner, text=key).grid(row=row, column=0, sticky="w", pady=3, padx=(0, 12))
            var = tk.StringVar(value=str(self.config.calibration.get(key, "")))
            self.calibration_vars[key] = var
            ttk.Entry(inner, textvariable=var, width=18).grid(row=row, column=1, sticky="ew", pady=3)
        actions = ttk.Frame(page)
        actions.grid(row=2, column=0, columnspan=2, sticky="ew", pady=(10, 0))
        for col in range(3):
            actions.columnconfigure(col, weight=1)
        ttk.Button(actions, text="Send Camera Center", command=self.calibrate_camera_center).grid(row=0, column=0, sticky="ew", padx=(0, 4))
        ttk.Button(actions, text="Save Calibration", command=self.save_centers).grid(row=0, column=1, sticky="ew", padx=4)
        ttk.Button(actions, text="Load Calibration", command=self.load_centers).grid(row=0, column=2, sticky="ew", padx=(4, 0))
        self._register_scroll_target(canvas, canvas)
        self._register_scroll_target(inner, canvas)

    def _build_logs_page(self, page: ttk.Frame) -> None:
        self._build_page_header(page, "Logs / Debug", "Serial log, raw antenna state, fused state, and servo diagnostics.")
        page.rowconfigure(1, weight=1)
        page.columnconfigure(0, weight=1)
        notebook = ttk.Notebook(page)
        notebook.grid(row=1, column=0, sticky="nsew")
        log_frame = ttk.Frame(notebook, padding=6)
        json_frame = ttk.Frame(notebook, padding=6)
        fused_frame = ttk.Frame(notebook, padding=6)
        field_frame = ttk.Frame(notebook, padding=6)
        servo_frame = ttk.Frame(notebook, padding=6)
        for frame in (log_frame, json_frame, fused_frame, field_frame, servo_frame):
            frame.rowconfigure(0, weight=1)
            frame.columnconfigure(0, weight=1)
        self.log_text = tk.Text(log_frame, height=10, wrap="word")
        self.log_text.grid(row=0, column=0, sticky="nsew")
        self.json_text = tk.Text(json_frame, height=10, wrap="none")
        self.json_text.grid(row=0, column=0, sticky="nsew")
        self.fused_text = tk.Text(fused_frame, height=10, wrap="none")
        self.fused_text.grid(row=0, column=0, sticky="nsew")
        self.field_text = tk.Text(field_frame, height=10, wrap="none")
        self.field_text.grid(row=0, column=0, sticky="nsew")
        self.servo_debug_text = tk.Text(servo_frame, height=10, wrap="none")
        self.servo_debug_text.grid(row=1, column=0, sticky="nsew")
        servo_frame.rowconfigure(1, weight=1)
        toolbar = ttk.Frame(servo_frame)
        toolbar.grid(row=0, column=0, sticky="ew", pady=(0, 6))
        ttk.Button(toolbar, text="Log Current Servo Target", command=self._log_current_servo_debug).grid(row=0, column=0, sticky="w")
        ttk.Button(toolbar, text="Clear", command=self._clear_servo_debug).grid(row=0, column=1, sticky="w", padx=(6, 0))
        for widget, dark in (
            (self.log_text, False),
            (self.json_text, True),
            (self.fused_text, True),
            (self.field_text, False),
            (self.servo_debug_text, True),
        ):
            self._style_text_widget(widget, dark=dark)
            self._register_scroll_target(widget, widget)
        notebook.add(log_frame, text="Status Log")
        notebook.add(json_frame, text="Antenna JSON")
        notebook.add(fused_frame, text="Fused Snapshot")
        notebook.add(field_frame, text="Field Order")
        notebook.add(servo_frame, text="Servo Debug")
        self._set_text(self.field_text, "\n".join(f"{i:02d}: {field}" for i, field in enumerate(FIELD_ORDER)))

    def show_page(self, page_name: str) -> None:
        page = self.pages.get(page_name)
        if page is None:
            return
        self.active_page = page_name
        page.tkraise()
        for name, button in self.nav_buttons.items():
            button.configure(style="NavActive.TButton" if name == page_name else "Nav.TButton")

    def _register_scroll_target(self, widget: tk.Widget, target: tk.Widget) -> None:
        self._scroll_targets[str(widget)] = target

    def _on_mousewheel(self, event: tk.Event) -> str | None:
        widget = self.root.winfo_containing(event.x_root, event.y_root)
        while widget is not None:
            target = self._scroll_targets.get(str(widget))
            if target is not None:
                if getattr(event, "num", None) == 4:
                    amount = -3
                elif getattr(event, "num", None) == 5:
                    amount = 3
                else:
                    delta = getattr(event, "delta", 0)
                    amount = -int(delta / 120) if abs(delta) >= 120 else (-1 if delta > 0 else 1)
                target.yview_scroll(amount, "units")
                return "break"
            parent_name = widget.winfo_parent()
            if not parent_name:
                break
            widget = self.root.nametowidget(parent_name)
        return None

    def open_camera_popup(self) -> None:
        if self.camera_popup is not None and self.camera_popup.winfo_exists():
            self.camera_popup.lift()
            return
        popup = tk.Toplevel(self.root)
        popup.title("AirTrixx Camera Feed")
        popup.geometry("960x540")
        popup.minsize(640, 360)
        popup.protocol("WM_DELETE_WINDOW", self._close_camera_popup)
        popup.configure(bg="#111827")
        popup.rowconfigure(0, weight=1)
        popup.columnconfigure(0, weight=1)
        self.camera_popup = popup
        self.camera_popup_label = ttk.Label(popup)
        self.camera_popup_label.grid(row=0, column=0, sticky="nsew")
        self._update_preview()

    def _close_camera_popup(self) -> None:
        if self.camera_popup is not None and self.camera_popup.winfo_exists():
            self.camera_popup.destroy()
        self.camera_popup = None
        self.camera_popup_label = None
        self._popup_photo = None

    def refresh_ports(self) -> None:
        ports = self.serial_bridge.available_ports()
        ports = sorted(ports, key=self._serial_port_sort_key)
        values = [self._serial_port_label(p) for p in ports]
        self.port_combo["values"] = values
        if values and (
            not self.port_var.get()
            or (
                self.serial_autoconnect_enabled
                and not self.serial_bridge.is_connected
                and self._preferred_serial_label(values) != self.port_var.get()
            )
        ):
            preferred = self._preferred_serial_label(values)
            self.port_var.set(preferred or values[0])

    def auto_connect_serial(self) -> None:
        if self.serial_bridge.is_connected or not self.serial_autoconnect_enabled:
            return
        self.refresh_ports()
        selected = self.port_var.get().split(" - ", 1)[0].strip() or None
        if selected and self.serial_bridge.connect(selected):
            self.log(f"Auto-connected Antenna serial on {selected}.")
            return
        self.root.after(SERIAL_AUTOCONNECT_RETRY_MS, self.auto_connect_serial)

    def toggle_serial(self) -> None:
        if self.serial_bridge.is_connected:
            self.serial_autoconnect_enabled = False
            self.serial_bridge.disconnect()
            return
        self.serial_autoconnect_enabled = True
        selected = self.port_var.get().split(" - ", 1)[0].strip() or None
        self.serial_bridge.connect(selected)

    def start_wristband_wireless_flash(self) -> None:
        if self.ota_in_progress:
            self.log("Wristband wireless flash is already running.")
            return
        if not self.serial_bridge.is_connected:
            self.log("Connect to the Antenna serial port before wireless flashing.")
            return

        ssid = self.ota_ssid_var.get().strip()
        password = self.ota_password_var.get()
        if not ssid:
            self.log("Enter the Wi-Fi SSID the wristband should join for OTA.")
            return
        if len(ssid.encode("utf-8")) > 32:
            self.log("Wi-Fi SSID is too long for the OTA packet.")
            return
        if len(password.encode("utf-8")) > 64:
            self.log("Wi-Fi password is too long for the OTA packet.")
            return

        calibration = dict(self.config.calibration)
        calibration["ota_wifi_ssid"] = ssid
        calibration["ota_wifi_password"] = password
        self.config.calibration = calibration
        save_calibration(calibration, self.config.calibration_path)

        self.ota_in_progress = True
        self.ota_button.configure(text="Flashing...", state="disabled")
        self.fans_ota_button.configure(state="disabled")
        self.camdock_ota_button.configure(state="disabled")
        worker = threading.Thread(
            target=self._run_wristband_wireless_flash,
            args=(ssid, password),
            daemon=True,
        )
        worker.start()

    def start_fans_wireless_flash(self) -> None:
        if self.ota_in_progress:
            self.log("Wireless flash is already running.")
            return
        if not self.serial_bridge.is_connected:
            self.log("Connect to the Antenna serial port before wireless flashing.")
            return

        ssid = self.ota_ssid_var.get().strip()
        password = self.ota_password_var.get()
        if not ssid:
            self.log("Enter the Wi-Fi SSID the fan controller should join for OTA.")
            return
        if len(ssid.encode("utf-8")) > 32:
            self.log("Wi-Fi SSID is too long for the OTA packet.")
            return
        if len(password.encode("utf-8")) > 64:
            self.log("Wi-Fi password is too long for the OTA packet.")
            return

        calibration = dict(self.config.calibration)
        calibration["ota_wifi_ssid"] = ssid
        calibration["ota_wifi_password"] = password
        self.config.calibration = calibration
        save_calibration(calibration, self.config.calibration_path)

        self.ota_in_progress = True
        self.fans_ota_button.configure(text="Flashing...", state="disabled")
        self.ota_button.configure(state="disabled")
        self.camdock_ota_button.configure(state="disabled")
        worker = threading.Thread(
            target=self._run_fans_wireless_flash,
            args=(ssid, password),
            daemon=True,
        )
        worker.start()

    def start_camdock_wireless_flash(self) -> None:
        if self.ota_in_progress:
            self.log("Wireless flash is already running.")
            return
        if not self.serial_bridge.is_connected:
            self.log("Connect to the Antenna serial port before wireless flashing.")
            return

        ssid = self.ota_ssid_var.get().strip()
        password = self.ota_password_var.get()
        if not ssid:
            self.log("Enter the Wi-Fi SSID the cam dock should join for OTA.")
            return
        if len(ssid.encode("utf-8")) > 32:
            self.log("Wi-Fi SSID is too long for the OTA packet.")
            return
        if len(password.encode("utf-8")) > 64:
            self.log("Wi-Fi password is too long for the OTA packet.")
            return

        calibration = dict(self.config.calibration)
        calibration["ota_wifi_ssid"] = ssid
        calibration["ota_wifi_password"] = password
        self.config.calibration = calibration
        save_calibration(calibration, self.config.calibration_path)

        self.ota_in_progress = True
        self.camdock_ota_button.configure(text="Flashing...", state="disabled")
        self.ota_button.configure(state="disabled")
        self.fans_ota_button.configure(state="disabled")
        worker = threading.Thread(
            target=self._run_camdock_wireless_flash,
            args=(ssid, password),
            daemon=True,
        )
        worker.start()

    def _run_wristband_wireless_flash(self, ssid: str, password: str) -> None:
        try:
            firmware_bin = self._build_wristband_firmware()
            firmware_md5 = self._file_md5(firmware_bin)
            port = int(self.config.calibration.get("ota_server_port", 8765))
            self._start_ota_server(firmware_bin, firmware_md5, port, "/wristband.bin")
            host_ip = self._local_ip_for_ota()
            url = f"http://{host_ip}:{port}/wristband.bin"
            if len(url.encode("utf-8")) > 96:
                self.log(f"OTA URL is too long for ESP-NOW packet: {url}")
                return

            command = {
                "cmd": "ota",
                "target": "wristband",
                "ssid": ssid,
                "password": password,
                "url": url,
                "md5": firmware_md5,
            }
            if self.serial_bridge.send_command(command):
                self.log(f"Sent wristband OTA command. Serving {url}")
            else:
                self.log("Failed to send OTA command to Antenna.")
        except Exception as exc:
            self.log(str(exc))
        finally:
            self.root.after(0, self._finish_wristband_wireless_flash)

    def _run_fans_wireless_flash(self, ssid: str, password: str) -> None:
        try:
            firmware_bin = self._build_fans_firmware()
            firmware_md5 = self._file_md5(firmware_bin)
            port = int(self.config.calibration.get("ota_server_port", 8765))
            self._start_ota_server(firmware_bin, firmware_md5, port, "/fans.bin")
            host_ip = self._local_ip_for_ota()
            url = f"http://{host_ip}:{port}/fans.bin"
            if len(url.encode("utf-8")) > 96:
                self.log(f"OTA URL is too long for ESP-NOW packet: {url}")
                return

            command = {
                "cmd": "ota",
                "target": "fans",
                "ssid": ssid,
                "password": password,
                "url": url,
                "md5": firmware_md5,
            }
            if self.serial_bridge.send_command(command):
                self.log(f"Sent fan controller OTA command. Serving {url}")
            else:
                self.log("Failed to send fan OTA command to Antenna.")
        except Exception as exc:
            self.log(str(exc))
        finally:
            self.root.after(0, self._finish_fans_wireless_flash)

    def _run_camdock_wireless_flash(self, ssid: str, password: str) -> None:
        try:
            firmware_bin = self._build_camdock_firmware()
            firmware_md5 = self._file_md5(firmware_bin)
            port = int(self.config.calibration.get("ota_server_port", 8765))
            self._start_ota_server(firmware_bin, firmware_md5, port, "/camdock.bin")
            host_ip = self._local_ip_for_ota()
            url = f"http://{host_ip}:{port}/camdock.bin"
            if len(url.encode("utf-8")) > 96:
                self.log(f"OTA URL is too long for ESP-NOW packet: {url}")
                return

            command = {
                "cmd": "ota",
                "target": "camdock",
                "ssid": ssid,
                "password": password,
                "url": url,
                "md5": firmware_md5,
            }
            if self.serial_bridge.send_command(command):
                self.log(f"Sent cam dock OTA command. Serving {url}")
            else:
                self.log("Failed to send cam dock OTA command to Antenna.")
        except Exception as exc:
            self.log(str(exc))
        finally:
            self.root.after(0, self._finish_camdock_wireless_flash)

    def _finish_wristband_wireless_flash(self) -> None:
        self.ota_in_progress = False
        self.ota_button.configure(text="Wireless Flash Wristband", state="normal")
        self.fans_ota_button.configure(state="normal")
        self.camdock_ota_button.configure(state="normal")

    def _finish_fans_wireless_flash(self) -> None:
        self.ota_in_progress = False
        self.fans_ota_button.configure(text="Wireless Flash Fans", state="normal")
        self.ota_button.configure(state="normal")
        self.camdock_ota_button.configure(state="normal")

    def _finish_camdock_wireless_flash(self) -> None:
        self.ota_in_progress = False
        self.camdock_ota_button.configure(text="Wireless Flash Cam Dock", state="normal")
        self.ota_button.configure(state="normal")
        self.fans_ota_button.configure(state="normal")

    def _build_wristband_firmware(self) -> Path:
        pio = shutil.which("pio") or shutil.which("platformio")
        if not pio:
            raise RuntimeError("PlatformIO command not found in PATH.")

        self.log("Building wristband firmware for wireless flash...")
        result = subprocess.run(
            [pio, "run", "-e", "esp32c3_supermini"],
            cwd=WRISTBAND_FIRMWARE_DIR,
            text=True,
            capture_output=True,
            timeout=180,
        )
        if result.returncode != 0:
            tail = (result.stdout + "\n" + result.stderr).strip().splitlines()[-12:]
            raise RuntimeError("Wristband firmware build failed:\n" + "\n".join(tail))
        if not WRISTBAND_FIRMWARE_BIN.exists():
            raise RuntimeError(f"Built firmware not found: {WRISTBAND_FIRMWARE_BIN}")
        self.log("Wristband firmware build complete.")
        return WRISTBAND_FIRMWARE_BIN

    def _build_fans_firmware(self) -> Path:
        pio = shutil.which("pio") or shutil.which("platformio")
        if not pio:
            raise RuntimeError("PlatformIO command not found in PATH.")

        self.log("Building fan controller firmware for wireless flash...")
        result = subprocess.run(
            [pio, "run", "-e", "esp32c3_supermini"],
            cwd=FANS_FIRMWARE_DIR,
            text=True,
            capture_output=True,
            timeout=180,
        )
        if result.returncode != 0:
            tail = (result.stdout + "\n" + result.stderr).strip().splitlines()[-12:]
            raise RuntimeError("Fan controller firmware build failed:\n" + "\n".join(tail))
        if not FANS_FIRMWARE_BIN.exists():
            raise RuntimeError(f"Built firmware not found: {FANS_FIRMWARE_BIN}")
        self.log("Fan controller firmware build complete.")
        return FANS_FIRMWARE_BIN

    def _build_camdock_firmware(self) -> Path:
        pio = shutil.which("pio") or shutil.which("platformio")
        if not pio:
            raise RuntimeError("PlatformIO command not found in PATH.")

        self.log("Building cam dock firmware for wireless flash...")
        result = subprocess.run(
            [pio, "run", "-e", "esp32s3_camdock"],
            cwd=CAMDOCK_FIRMWARE_DIR,
            text=True,
            capture_output=True,
            timeout=180,
        )
        if result.returncode != 0:
            tail = (result.stdout + "\n" + result.stderr).strip().splitlines()[-12:]
            raise RuntimeError("Cam dock firmware build failed:\n" + "\n".join(tail))
        if not CAMDOCK_FIRMWARE_BIN.exists():
            raise RuntimeError(f"Built firmware not found: {CAMDOCK_FIRMWARE_BIN}")
        self.log("Cam dock firmware build complete.")
        return CAMDOCK_FIRMWARE_BIN

    def _start_ota_server(self, firmware_bin: Path, firmware_md5: str, port: int, firmware_path: str) -> None:
        self._stop_ota_server()
        firmware_bytes = firmware_bin.read_bytes()

        class FirmwareHandler(http.server.BaseHTTPRequestHandler):
            def do_GET(handler_self) -> None:  # noqa: N802
                if handler_self.path.split("?", 1)[0] != firmware_path:
                    handler_self.send_error(404)
                    return
                handler_self.send_response(200)
                handler_self.send_header("Content-Type", "application/octet-stream")
                handler_self.send_header("Content-Length", str(len(firmware_bytes)))
                handler_self.send_header("x-MD5", firmware_md5)
                handler_self.end_headers()
                handler_self.wfile.write(firmware_bytes)

            def log_message(handler_self, _format: str, *args: Any) -> None:
                return

        server = http.server.ThreadingHTTPServer(("0.0.0.0", port), FirmwareHandler)
        self.ota_server = server
        self.ota_server_thread = threading.Thread(target=server.serve_forever, daemon=True)
        self.ota_server_thread.start()

    def _stop_ota_server(self) -> None:
        if self.ota_server is None:
            return
        self.ota_server.shutdown()
        self.ota_server.server_close()
        self.ota_server = None
        self.ota_server_thread = None

    @staticmethod
    def _file_md5(path: Path) -> str:
        digest = hashlib.md5()
        with path.open("rb") as f:
            for chunk in iter(lambda: f.read(65536), b""):
                digest.update(chunk)
        return digest.hexdigest()

    @staticmethod
    def _local_ip_for_ota() -> str:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            try:
                sock.connect(("8.8.8.8", 80))
                return sock.getsockname()[0]
            except OSError:
                return socket.gethostbyname(socket.gethostname())

    @staticmethod
    def _serial_port_label(port: dict[str, str]) -> str:
        return f'{port["device"]} - {port["description"]}'

    @staticmethod
    def _serial_port_sort_key(port: dict[str, str]) -> tuple[int, str]:
        text = f'{port.get("description", "")} {port.get("hwid", "")}'.lower()
        if "vid:pid=303a" in text:
            return (0, port.get("device", ""))
        if "usb" in text and "bluetooth" not in text:
            return (1, port.get("device", ""))
        if "bluetooth" in text or "bthenum" in text:
            return (9, port.get("device", ""))
        return (5, port.get("device", ""))

    def _preferred_serial_label(self, labels: list[str]) -> str | None:
        if self.config.serial_port:
            for label in labels:
                if label.startswith(f"{self.config.serial_port} - "):
                    return label
        return labels[0] if labels else None

    def on_camera_source_changed(self, _event: tk.Event | None = None) -> None:
        source = self.camera_source_var.get()
        camera_index = CAMERA_SOURCE_INDICES.get(source, self.config.camera_index)
        self.config.camera_index = camera_index
        self.hand_tracker.configure(camera_index=camera_index)
        self.log(f"Camera source set to {source} (index {camera_index}).")

    def toggle_camera_mirror(self) -> None:
        mirrored = not self.camera_mirror_var.get()
        self.camera_mirror_var.set(mirrored)
        self.hand_tracker.configure(mirror_preview=mirrored)
        self.mirror_button.configure(text=f"Mirror: {'On' if mirrored else 'Off'}")
        self.log(f"Camera mirroring {'enabled' if mirrored else 'disabled'}.")

    def toggle_fans(self) -> None:
        if not self.serial_bridge.is_connected:
            self.log("Connect to the Antenna serial port before controlling fans.")
            return

        fans = self._fan_device_state()
        current_on = fans.get("fan_on")
        if isinstance(current_on, bool):
            desired_on = not current_on
        else:
            desired_on = not self.fans_requested_on

        command = {
            "cmd": "fans",
            "target": "fans",
            "fan_on": desired_on,
        }
        if self.serial_bridge.send_command(command):
            self.fans_requested_on = desired_on
            self.fan_status_var.set(f"Fans: command sent, {'on' if desired_on else 'off'} requested.")
            self.fan_button.configure(text=f"Turn Fans {'Off' if desired_on else 'On'}")
            self.log(f"Sent fan {'on' if desired_on else 'off'} command.")
        else:
            self.log("Failed to send fan command to Antenna.")

    def _fan_device_state(self) -> dict[str, Any]:
        serial_state = self.serial_bridge.get_latest_state()
        devices = serial_state.get("devices", {}) if isinstance(serial_state, dict) else {}
        fans = devices.get("fans", {}) if isinstance(devices, dict) else {}
        return fans if isinstance(fans, dict) else {}

    def _update_fan_controls(self) -> None:
        fans = self._fan_device_state()
        status = fans.get("status", "not_connected")
        fan_on = fans.get("fan_on")
        temps = fans.get("temps", {}) if isinstance(fans.get("temps"), dict) else {}
        temp_1 = self._format_table_value(temps.get("sensor_1_c"))
        temp_2 = self._format_table_value(temps.get("sensor_2_c"))
        battery_level = self._format_table_value(fans.get("battery_level"))
        battery_voltage = self._format_table_value(fans.get("battery_voltage"))

        if isinstance(fan_on, bool):
            self.fans_requested_on = fan_on
            self.fan_button.configure(text=f"Turn Fans {'Off' if fan_on else 'On'}")
            state_text = "on" if fan_on else "off"
        else:
            self.fan_button.configure(text=f"Turn Fans {'Off' if self.fans_requested_on else 'On'}")
            state_text = "unknown"

        self.fan_status_var.set(
            f"Fans: {status}, {state_text}. Temp 1: {temp_1} C, Temp 2: {temp_2} C. "
            f"Battery: {battery_level}% / {battery_voltage}V."
        )

    def start_camera_centering(self) -> None:
        if self._apply_calibration_entries() is None:
            return
        self.centering_bracket = None
        self.hand_calibration_active = False
        self.hand_calibration_fist_armed = False
        self.hand_calibration_fist_side = None
        self.hand_calibration_target_inside = False
        self.hand_calibration_seen_since_s = None
        self.hand_calibration_auto_capture = True
        self.camera_centering_active = True
        self.camera_centering_started_s = None
        self.camera_centering_settled_s = None
        self.camera_centering_last_send_s = 0.0
        self.camera_centering_position = {}
        self.camera_search_anchor = {}
        self.camera_search_index = 0
        self.camera_search_last_send_s = 0.0
        self.camera_search_found_once = False
        self._update_bracket_buttons()
        self.camera_centering_status_var.set("Camera centering: searching for face.")
        self.hand_calibration_status_var.set("Calibration phase: waiting for camera centering.")
        self.serial_autoconnect_enabled = True
        if not self.serial_bridge.is_connected:
            self.auto_connect_serial()
        self.log("Camera centering phase started.")

    def select_servo_bracket(self, bracket: str) -> None:
        if self._apply_calibration_entries() is None:
            return
        self.camera_centering_active = False
        self.hand_calibration_active = False
        self.hand_calibration_fist_armed = False
        self.hand_calibration_target_inside = False
        self.hand_calibration_seen_since_s = None
        stored_position = self.centering_positions.get(bracket, {})
        pan = stored_position.get("pan")
        tilt = stored_position.get("tilt")
        source = "stored_manual"
        if pan is None or tilt is None:
            auto_position = self.servo_controller.last_auto_bracket_ticks(bracket)
            if auto_position:
                pan = auto_position["pan"]
                tilt = auto_position["tilt"]
                source = f"last_auto_seq_{auto_position.get('debug_seq')}"
            else:
                sent_position = self.servo_controller.last_sent_bracket_ticks(bracket)
                if sent_position:
                    pan = sent_position["pan"]
                    tilt = sent_position["tilt"]
                    source = f"last_sent_{sent_position.get('source')}"
                else:
                    pan, tilt = self.servo_controller.center_ticks_for_bracket(bracket)
                    source = "saved_center"
        self.centering_bracket = bracket
        self.centering_positions[bracket] = {"pan": int(pan), "tilt": int(tilt)}
        self.center_pan_var.set(str(pan))
        self.center_tilt_var.set(str(tilt))
        self._update_bracket_buttons()
        self._send_centering_position()
        self._append_servo_debug(
            f"MANUAL_START bracket={bracket} source={source} pan={int(pan)} tilt={int(tilt)}"
        )
        self.root.focus_set()

    def resume_auto_tracking(self) -> None:
        self.centering_bracket = None
        self.selected_bracket_var.set("Auto hand tracking")
        self.center_pan_var.set("-")
        self.center_tilt_var.set("-")
        self.centering_status_var.set("Auto hand tracking active.")
        self._update_bracket_buttons()

    def save_center_position(self) -> None:
        if self.centering_bracket is None:
            self.log("Select a servo bracket before saving a center position.")
            return
        position = self.centering_positions.get(self.centering_bracket)
        if not position:
            self.log("No servo position is selected to save.")
            return

        calibration = self.servo_controller.save_bracket_center(
            self.centering_bracket,
            position["pan"],
            position["tilt"],
        )
        self.config.calibration = calibration
        self._sync_calibration_entries(calibration)
        save_calibration(calibration, self.config.calibration_path)
        label = SERVO_BRACKETS[self.centering_bracket]
        self.centering_status_var.set(f"Saved {label} center: pan {position['pan']}, tilt {position['tilt']}.")
        self.log(f"Saved {label} center to {self.config.calibration_path}.")

    def _handle_centering_arrow(self, event: tk.Event) -> str | None:
        if self.centering_bracket is None:
            return None
        if self._focus_is_text_input():
            return None

        position = self.centering_positions.setdefault(
            self.centering_bracket,
            self._current_center_ticks(self.centering_bracket),
        )
        before = dict(position)
        step = self._centering_step()
        if event.keysym == "Left":
            position["pan"] += self._centering_pan_delta("Left", step)
        elif event.keysym == "Right":
            position["pan"] += self._centering_pan_delta("Right", step)
        elif event.keysym == "Up":
            position["tilt"] -= step
        elif event.keysym == "Down":
            position["tilt"] += step
        else:
            return None

        position["pan"] = self._clamp_servo_tick(position["pan"])
        position["tilt"] = self._clamp_servo_tick(position["tilt"])
        self.center_pan_var.set(str(position["pan"]))
        self.center_tilt_var.set(str(position["tilt"]))
        sent = self._send_centering_position()
        self._log_manual_servo_adjustment(event.keysym, before, dict(position), step, sent)
        return "break"

    def _send_centering_position(self) -> bool:
        if self.centering_bracket is None:
            return False
        position = self.centering_positions[self.centering_bracket]
        label = SERVO_BRACKETS[self.centering_bracket]
        sent = self.servo_controller.send_bracket_position(
            self.centering_bracket,
            position["pan"],
            position["tilt"],
        )
        if sent:
            self.centering_status_var.set(
                f"{label}: pan {position['pan']}, tilt {position['tilt']}."
            )
        else:
            self.centering_status_var.set(
                f"{label}: pan {position['pan']}, tilt {position['tilt']} queued; serial is not connected."
            )
        return sent

    def _log_manual_servo_adjustment(
        self,
        key: str,
        before: dict[str, int],
        after: dict[str, int],
        step: int,
        sent: bool,
    ) -> None:
        if self.centering_bracket is None:
            return
        bracket = self.centering_bracket
        auto = self.servo_controller.last_auto_bracket_ticks(bracket)
        debug = self.servo_controller.last_debug_for_bracket(bracket)
        correction = ""
        if auto:
            correction = (
                f" auto_pan={auto.get('pan')} auto_tilt={auto.get('tilt')}"
                f" correction_pan={after['pan'] - int(auto.get('pan', after['pan']))}"
                f" correction_tilt={after['tilt'] - int(auto.get('tilt', after['tilt']))}"
            )

        hand = ""
        if debug:
            hand = (
                f" hand_img=({self._fmt(debug.get('raw_image_x'))},{self._fmt(debug.get('raw_image_y'))})"
                f" hand_y_up={self._fmt(debug.get('predicted_y_up'))}"
                f" dist_mm={self._fmt(debug.get('distance_mm'), 1)}"
                f" yaw={self._fmt(debug.get('yaw_deg'), 2)}"
                f" pitch={self._fmt(debug.get('pitch_deg'), 2)}"
            )

        self._append_servo_debug(
            "MANUAL_ADJUST"
            f" bracket={bracket} key={key} step={step} sent={self._bool_text(sent)}"
            f" before_pan={before['pan']} before_tilt={before['tilt']}"
            f" after_pan={after['pan']} after_tilt={after['tilt']}"
            f"{correction}{hand}"
        )

    def _update_servo_debug_console(self, force: bool = False) -> None:
        snapshot = self.servo_controller.last_debug_snapshot
        if not snapshot:
            return
        seq = int(snapshot.get("seq") or 0)
        if seq <= 0:
            return
        now = time.monotonic()
        if not force and seq == self._last_servo_debug_sequence:
            return
        if not force and now - self._last_servo_debug_log_s < SERVO_DEBUG_INTERVAL_S:
            return
        self._last_servo_debug_sequence = seq
        self._last_servo_debug_log_s = now
        self._append_servo_debug(self._format_servo_debug_snapshot(snapshot))

    def _log_current_servo_debug(self) -> None:
        self._update_servo_debug_console(force=True)

    def _clear_servo_debug(self) -> None:
        self.servo_debug_lines.clear()
        if hasattr(self, "servo_debug_text"):
            self.servo_debug_text.delete("1.0", "end")
        try:
            SERVO_DEBUG_LOG_PATH.parent.mkdir(parents=True, exist_ok=True)
            SERVO_DEBUG_LOG_PATH.write_text("", encoding="utf-8")
        except OSError:
            pass

    def _format_servo_debug_snapshot(self, snapshot: dict[str, Any]) -> str:
        lines = [
            "AUTO"
            f" seq={snapshot.get('seq')}"
            f" mode={snapshot.get('mode')}"
            f" active={snapshot.get('active_pair')}"
            f" sent={self._bool_text(bool(snapshot.get('sent')))}"
            f" servos={json.dumps(snapshot.get('servos', {}), separators=(',', ':'))}"
        ]
        hands = snapshot.get("hands", {})
        if isinstance(hands, dict):
            for side in ("right", "left"):
                details = hands.get(side)
                if not isinstance(details, dict):
                    continue
                lines.append(
                    "  "
                    f"{side}:"
                    f" img=({self._fmt(details.get('raw_image_x'))},{self._fmt(details.get('raw_image_y'))})"
                    f" pred=({self._fmt(details.get('predicted_image_x'))},{self._fmt(details.get('predicted_image_y'))})"
                    f" y_up={self._fmt(details.get('predicted_y_up'))}"
                    f" score={self._fmt(details.get('score'), 2)}"
                    f" dist_mm={self._fmt(details.get('distance_mm'), 1)}"
                    f" yaw={self._fmt(details.get('yaw_deg'), 2)}"
                    f" pitch={self._fmt(details.get('pitch_deg'), 2)}"
                    f" target=({self._fmt(details.get('pan_target'), 1)},{self._fmt(details.get('tilt_target'), 1)})"
                    f" ticks=({details.get('pan_tick', '-')},{details.get('tilt_tick', '-')})"
                    f" ray={self._fmt_tuple(details.get('ray'))}"
                    f" point_mm={self._fmt_tuple(details.get('point_mm'), 1)}"
                )
        return "\n".join(lines)

    def _append_servo_debug(self, message: str) -> None:
        timestamp = time.strftime("%H:%M:%S")
        entry = f"{timestamp} {message}"
        self.servo_debug_lines.append(entry)
        if len(self.servo_debug_lines) > SERVO_DEBUG_LOG_LIMIT:
            del self.servo_debug_lines[: len(self.servo_debug_lines) - SERVO_DEBUG_LOG_LIMIT]
        if hasattr(self, "servo_debug_text"):
            self.servo_debug_text.insert("end", entry + "\n")
            self.servo_debug_text.see("end")
        try:
            SERVO_DEBUG_LOG_PATH.parent.mkdir(parents=True, exist_ok=True)
            with SERVO_DEBUG_LOG_PATH.open("a", encoding="utf-8") as handle:
                handle.write(entry + "\n")
        except OSError:
            pass

    @staticmethod
    def _fmt(value: Any, digits: int = 3) -> str:
        try:
            number = float(value)
        except (TypeError, ValueError):
            return "-"
        return f"{number:.{digits}f}"

    def _fmt_tuple(self, value: Any, digits: int = 3) -> str:
        if not isinstance(value, (tuple, list)):
            return "-"
        return "(" + ",".join(self._fmt(item, digits) for item in value) + ")"

    @staticmethod
    def _bool_text(value: bool) -> str:
        return "yes" if value else "no"

    def _update_bracket_buttons(self) -> None:
        for bracket, button in self.bracket_buttons.items():
            label = SERVO_BRACKETS[bracket]
            if bracket == self.centering_bracket:
                button.configure(text=f"{label} *")
                self.selected_bracket_var.set(f"Centering: {label}")
            else:
                button.configure(text=label)

    def _centering_step(self) -> int:
        try:
            return max(1, min(100, int(float(self.center_step_var.get()))))
        except ValueError:
            self.center_step_var.set("5")
            return 5

    def _centering_pan_delta(self, direction: str, step: int) -> int:
        if self.centering_bracket == "camera":
            return step if direction == "Left" else -step
        return -step if direction == "Left" else step

    def _clamp_servo_tick(self, value: int) -> int:
        active_min_tick = max(1, self.config.servo_min_tick)
        return max(active_min_tick, min(self.config.servo_max_tick, int(value)))

    def _focus_is_text_input(self) -> bool:
        widget = self.root.focus_get()
        return isinstance(widget, (tk.Entry, ttk.Entry, ttk.Combobox, ttk.Spinbox))

    def _sync_calibration_entries(self, calibration: dict[str, Any]) -> None:
        for key, value in calibration.items():
            if key in self.calibration_vars:
                self.calibration_vars[key].set(str(value))

    def _current_center_ticks(self, bracket: str) -> dict[str, int]:
        pan, tilt = self.servo_controller.center_ticks_for_bracket(bracket)
        return {"pan": pan, "tilt": tilt}

    @staticmethod
    def _camera_source_for_index(camera_index: int) -> str:
        for source, index in CAMERA_SOURCE_INDICES.items():
            if index == camera_index:
                return source
        return "Auto (first available)"

    def calibrate_camera_center(self) -> None:
        if self._apply_calibration_entries() is None:
            return
        if self.servo_controller.center_camera(force=True):
            self.log("Sent camera center pulse command.")
        else:
            self.log("Camera center command not sent; serial is not connected.")

    def save_centers(self) -> None:
        calibration = self._apply_calibration_entries()
        if calibration is None:
            return
        save_calibration(calibration, self.config.calibration_path)
        self.log(f"Saved calibration to {self.config.calibration_path}")

    def load_centers(self) -> None:
        calibration = load_calibration(self.config.calibration_path)
        self.config.calibration = calibration
        self._sync_calibration_entries(calibration)
        self.servo_controller.update_calibration(calibration)
        self.centering_positions.clear()
        self.log(f"Loaded calibration from {self.config.calibration_path}")

    def start_hand_calibration(self, auto: bool = False) -> None:
        if self._apply_calibration_entries() is None:
            return
        self.startup_hand_calibration_pending = False
        self.camera_centering_active = False
        self.centering_bracket = None
        self._update_bracket_buttons()
        self.hand_calibration_active = True
        self.hand_calibration_index = 0
        self.hand_calibration_points = {}
        self.hand_calibration_fist_armed = False
        self.hand_calibration_fist_side = None
        self.hand_calibration_target_inside = False
        self.hand_calibration_seen_since_s = None
        self.hand_calibration_auto_capture = bool(auto)
        self._set_hand_calibration_prompt()
        self.serial_autoconnect_enabled = True
        if not self.serial_bridge.is_connected:
            self.auto_connect_serial()
        prefix = "Startup" if auto else "Manual"
        self.log(f"{prefix} calibration phase started.")

    def capture_hand_calibration_point(self) -> None:
        if not self.hand_calibration_active:
            self.start_hand_calibration()
        if not self.hand_calibration_active:
            return

        hands = self.hand_tracker.get_latest_hands()
        visible_hands = self._visible_session_calibration_hands(hands)
        if visible_hands is None:
            self.hand_calibration_status_var.set(
                "Calibration phase: both hands must be visible before capture."
            )
            self.log("Session calibration not captured because both hands are not visible.")
            return

        self._finish_session_calibration(visible_hands, self.serial_bridge.get_latest_state())

    def skip_hand_calibration(self) -> None:
        self.startup_hand_calibration_pending = False
        self.hand_calibration_active = False
        self.hand_calibration_fist_armed = False
        self.hand_calibration_fist_side = None
        self.hand_calibration_target_inside = False
        self.hand_calibration_seen_since_s = None
        self.hand_calibration_auto_capture = True
        self.hand_calibration_status_var.set(
            "Calibration phase: skipped; using saved dock geometry values."
        )
        self.log("Calibration phase skipped; saved dock geometry values remain active.")

    def _set_hand_calibration_prompt(self) -> None:
        self.hand_calibration_status_var.set(
            "Calibration phase: hold both hands in the normal start pose."
        )

    def _best_visible_hand(self, hands: dict[str, dict[str, Any]] | None = None) -> tuple[str, dict[str, Any]] | None:
        if hands is None:
            hands = self.hand_tracker.get_latest_hands()
        candidates: list[tuple[float, str, dict[str, Any]]] = []
        for side, values in hands.items():
            if not values.get("visible") or values.get("x") is None or values.get("y") is None:
                continue
            candidates.append((float(values.get("score") or 0.0), side, values))
        if not candidates:
            return None
        _score, side, values = max(candidates, key=lambda item: item[0])
        return side, values

    def _update_hand_calibration(
        self,
        hands: dict[str, dict[str, Any]],
        serial_state: dict[str, Any],
    ) -> None:
        if not self.serial_bridge.is_connected:
            self.hand_calibration_seen_since_s = None
            self.hand_calibration_status_var.set("Calibration phase: waiting for Antenna serial link.")
            return

        visible_hands = self._visible_session_calibration_hands(hands)
        if visible_hands is None:
            self.hand_calibration_seen_since_s = None
            self.hand_calibration_status_var.set(
                "Calibration phase: hold both hands in view at the normal start pose."
            )
            return

        self.servo_controller.send_for_hands(visible_hands, serial_state, force=True)

        if not self.hand_calibration_auto_capture:
            self.hand_calibration_seen_since_s = None
            self.hand_calibration_status_var.set(
                "Calibration phase: auto aiming. Select Right/Left Bracket, use arrow keys, then send me the Servo Debug log."
            )
            return

        now = time.monotonic()
        if self.hand_calibration_seen_since_s is None:
            self.hand_calibration_seen_since_s = now
            self.hand_calibration_status_var.set("Calibration phase: aiming brackets; hold still.")
            return

        held_s = now - self.hand_calibration_seen_since_s
        if held_s < SESSION_CALIBRATION_SETTLED_S:
            self.hand_calibration_status_var.set(
                f"Calibration phase: hold still {SESSION_CALIBRATION_SETTLED_S - held_s:.1f}s."
            )
            return

        self._finish_session_calibration(visible_hands, serial_state)

    def _display_hand_position(self, values: dict[str, Any]) -> tuple[float, float]:
        x = float(values["x"])
        y = float(values["y"])
        if self.hand_tracker.mirror_preview:
            x = 1.0 - x
        return max(0.0, min(1.0, x)), max(0.0, min(1.0, y))

    def _visible_session_calibration_hands(
        self,
        hands: dict[str, dict[str, Any]],
    ) -> dict[str, dict[str, Any]] | None:
        visible: dict[str, dict[str, Any]] = {}
        for side in ("right", "left"):
            values = hands.get(side, {}) if isinstance(hands, dict) else {}
            if not values.get("visible") or values.get("x") is None or values.get("y") is None:
                return None
            if float(values.get("score") or 0.0) < SESSION_CALIBRATION_MIN_SCORE:
                return None
            visible[side] = dict(values)
        return visible

    def _finish_session_calibration(
        self,
        hands: dict[str, dict[str, Any]],
        serial_state: dict[str, Any],
    ) -> None:
        calibration = dict(self.config.calibration)
        session: dict[str, dict[str, Any]] = {}
        valid_tof_values: list[float] = []
        fallback_distance = float(calibration.get("initial_hand_distance_mm", 700.0))

        for side in ("right", "left"):
            values = hands[side]
            tof_mm = self._valid_calibration_tof_mm(side, serial_state)
            source = "tof" if tof_mm is not None else "fallback"
            if tof_mm is None:
                tof_mm = fallback_distance
            else:
                valid_tof_values.append(tof_mm)
            session[side] = {
                "x": round(float(values["x"]), 4),
                "y": round(float(values["y"]), 4),
                "tof_mm": round(float(tof_mm), 1),
                "distance_source": source,
                "score": round(float(values.get("score") or 0.0), 4),
            }

        if valid_tof_values:
            calibration["initial_hand_distance_mm"] = round(sum(valid_tof_values) / len(valid_tof_values), 1)
        calibration["session_calibration"] = session
        self.config.calibration = calibration
        self._sync_calibration_entries(calibration)
        self.servo_controller.update_calibration(calibration)
        save_calibration(calibration, self.config.calibration_path)

        self.hand_calibration_active = False
        self.hand_calibration_fist_armed = False
        self.hand_calibration_fist_side = None
        self.hand_calibration_target_inside = False
        self.hand_calibration_seen_since_s = None
        self.hand_calibration_status_var.set(
            "Calibration phase: saved neutral hand pose and starting distance."
        )
        self.log(f"Saved dock geometry session calibration to {self.config.calibration_path}.")

    def _valid_calibration_tof_mm(self, side: str, serial_state: dict[str, Any]) -> float | None:
        devices = serial_state.get("devices", {}) if isinstance(serial_state, dict) else {}
        camdock = devices.get("camdock", {}) if isinstance(devices, dict) else {}
        tof = camdock.get("tof", {}) if isinstance(camdock, dict) else {}
        raw = tof.get(f"{side}_mm") if isinstance(tof, dict) else None
        try:
            value = float(raw)
            min_mm = float(self.config.calibration.get("min_valid_tof_mm", 80.0))
            max_mm = float(self.config.calibration.get("max_valid_tof_mm", 2000.0))
        except (TypeError, ValueError):
            return None
        if min_mm <= value <= max_mm:
            return value
        return None

    def record_gesture(self) -> None:
        try:
            repetitions = int(self.repetitions_var.get())
            duration_s = float(self.duration_var.get())
        except ValueError:
            self.log("Repetitions and duration must be numbers.")
            return
        self.recorder.start(self.gesture_name_var.get(), repetitions, duration_s)

    def close(self) -> None:
        self._close_camera_popup()
        self._stop_ota_server()
        self.recorder.stop()
        self.servo_controller.disable_all()
        self.hand_tracker.stop()
        self.serial_bridge.disconnect()
        self.root.destroy()

    def _tick(self) -> None:
        self._drain_log_queue()

        hands = self.hand_tracker.get_latest_hands()
        serial_state = self.serial_bridge.get_latest_state()
        camera_centering_claimed_servo = False
        if self.centering_bracket is None and self.camera_centering_active:
            camera_centering_claimed_servo = self._update_camera_centering()
        if self.hand_calibration_active:
            self._update_hand_calibration(hands, serial_state)
        if self.centering_bracket is None and not camera_centering_claimed_servo and not self.hand_calibration_active:
            self.servo_controller.send_for_hands(hands, serial_state)
        self._latest_snapshot = self.fusion_state.build_snapshot(serial_state, hands)
        self._update_servo_debug_console()
        self._update_preview()
        if time.monotonic() - self._last_text_update_s >= 0.2:
            self._last_text_update_s = time.monotonic()
            self._update_text_views()

        self.connect_button.configure(text="Disconnect" if self.serial_bridge.is_connected else "Connect")
        self.record_button.configure(text="Recording..." if self.recorder.is_recording else "Record Gesture")
        self._update_fan_controls()
        self.root.after(33, self._tick)

    def _update_camera_centering(self) -> bool:
        if not self.serial_bridge.is_connected:
            self.camera_centering_status_var.set("Camera centering: waiting for Antenna serial link.")
            return True

        now = time.monotonic()
        self._ensure_camera_centering_started(now)

        if (
            self.camera_search_found_once
            and self.camera_centering_started_s is not None
            and now - self.camera_centering_started_s > CAMERA_CENTER_TIMEOUT_S
        ):
            self._complete_camera_centering("timeout")
            return True

        face = self.hand_tracker.get_latest_face()
        if not face.get("visible"):
            self.camera_centering_settled_s = None
            self._sweep_camera_for_face(now)
            return True

        if not self.camera_search_found_once:
            self.camera_search_found_once = True
            self.camera_centering_started_s = now
            self.log("Face detected; switching camera centering from search to tracking.")

        face_x = (
            float(face.get("x"))
            if face.get("x") is not None
            else CAMERA_CENTER_TARGET_X
        )
        face_top_y = (
            float(face.get("top_y"))
            if face.get("top_y") is not None
            else CAMERA_CENTER_TARGET_FACE_TOP_Y
        )
        error_x = face_x - CAMERA_CENTER_TARGET_X
        error_y = face_top_y - CAMERA_CENTER_TARGET_FACE_TOP_Y
        x_ready = abs(error_x) <= CAMERA_CENTER_DEADBAND_X
        y_ready = abs(error_y) <= CAMERA_CENTER_DEADBAND_Y

        if x_ready and y_ready:
            if self.camera_centering_settled_s is None:
                self.camera_centering_settled_s = now
            if now - self.camera_centering_settled_s >= CAMERA_CENTER_SETTLED_S:
                self._complete_camera_centering("centered")
                return False
            self.camera_centering_status_var.set("Camera centering: holding target.")
            return True

        self.camera_centering_settled_s = None
        if now - self.camera_centering_last_send_s < CAMERA_CENTER_COMMAND_INTERVAL_S:
            return True

        x_gain = float(self.config.calibration.get("x_gain_ticks", 120))
        y_gain = float(self.config.calibration.get("y_gain_ticks", 90))
        pan_delta = self._bounded_camera_step(-error_x * x_gain * 0.35)
        tilt_delta = self._bounded_camera_step(error_y * y_gain * 0.35)
        if not x_ready:
            self.camera_centering_position["pan"] = self._clamp_servo_tick(
                self.camera_centering_position["pan"] + pan_delta
            )
        if not y_ready:
            self.camera_centering_position["tilt"] = self._clamp_servo_tick(
                self.camera_centering_position["tilt"] + tilt_delta
            )

        sent = self.servo_controller.send_bracket_position(
            "camera",
            self.camera_centering_position["pan"],
            self.camera_centering_position["tilt"],
        )
        self.camera_centering_last_send_s = now
        if sent:
            self.camera_centering_status_var.set(
                "Camera centering: pan "
                f"{self.camera_centering_position['pan']}, tilt {self.camera_centering_position['tilt']}."
            )
        else:
            self.camera_centering_status_var.set("Camera centering: serial write failed.")
        return True

    def _ensure_camera_centering_started(self, now: float) -> None:
        if self.camera_centering_started_s is not None:
            return
        self.camera_centering_started_s = now
        self.camera_centering_position = self._current_center_ticks("camera")
        self.camera_search_anchor = dict(self.camera_centering_position)
        self.log("Camera centering started.")

    def _sweep_camera_for_face(self, now: float) -> None:
        if not self.camera_search_anchor:
            self.camera_search_anchor = self._current_center_ticks("camera")
            self.camera_centering_position = dict(self.camera_search_anchor)

        if now - self.camera_search_last_send_s < CAMERA_SEARCH_COMMAND_INTERVAL_S:
            return

        offsets = self._camera_search_offsets()
        pan_offset, tilt_offset = offsets[self.camera_search_index % len(offsets)]
        self.camera_search_index += 1
        self.camera_centering_position = {
            "pan": self._clamp_servo_tick(self.camera_search_anchor["pan"] + pan_offset),
            "tilt": self._clamp_servo_tick(self.camera_search_anchor["tilt"] + tilt_offset),
        }
        sent = self.servo_controller.send_bracket_position(
            "camera",
            self.camera_centering_position["pan"],
            self.camera_centering_position["tilt"],
        )
        self.camera_search_last_send_s = now
        self.camera_centering_last_send_s = now
        if sent:
            self.camera_centering_status_var.set(
                "Camera centering: searching for face, pan "
                f"{self.camera_centering_position['pan']}, tilt "
                f"{self.camera_centering_position['tilt']}."
            )
        else:
            self.camera_centering_status_var.set("Camera centering: face search serial write failed.")

    def _camera_search_offsets(self) -> list[tuple[int, int]]:
        negative_offsets = list(
            range(
                -CAMERA_SEARCH_PAN_STEP_TICKS,
                -CAMERA_SEARCH_PAN_RANGE_TICKS - 1,
                -CAMERA_SEARCH_PAN_STEP_TICKS,
            )
        )
        positive_offsets = list(
            range(
                CAMERA_SEARCH_PAN_STEP_TICKS,
                CAMERA_SEARCH_PAN_RANGE_TICKS + 1,
                CAMERA_SEARCH_PAN_STEP_TICKS,
            )
        )
        pan_offsets = (
            [0]
            + negative_offsets
            + list(reversed(negative_offsets[:-1]))
            + [0]
            + positive_offsets
            + list(reversed(positive_offsets[:-1]))
        )

        tilt_ticks_per_deg = abs(float(self.config.calibration.get("tilt_ticks_per_degree", 2.25)))
        up_ticks = max(1, int(round(CAMERA_SEARCH_UP_DEG * tilt_ticks_per_deg)))
        down_ticks = max(1, int(round(CAMERA_SEARCH_DOWN_DEG * tilt_ticks_per_deg)))
        tilt_step_ticks = max(1, int(round(CAMERA_SEARCH_TILT_STEP_DEG * tilt_ticks_per_deg)))
        up_offset = -up_ticks
        center_offset = 0
        down_offset = down_ticks

        offsets: list[tuple[int, int]] = []
        offsets.extend(self._camera_search_tilt_ramp(center_offset, up_offset, tilt_step_ticks))
        offsets.extend((pan_offset, up_offset) for pan_offset in pan_offsets)
        offsets.extend(self._camera_search_tilt_ramp(up_offset, center_offset, tilt_step_ticks))
        offsets.extend((pan_offset, center_offset) for pan_offset in reversed(pan_offsets))
        offsets.extend(self._camera_search_tilt_ramp(center_offset, down_offset, tilt_step_ticks))
        offsets.extend((pan_offset, down_offset) for pan_offset in pan_offsets)
        offsets.extend(self._camera_search_tilt_ramp(down_offset, center_offset, tilt_step_ticks))
        return offsets

    @staticmethod
    def _camera_search_tilt_ramp(start: int, end: int, step: int) -> list[tuple[int, int]]:
        if start == end:
            return []
        direction = 1 if end > start else -1
        offsets: list[tuple[int, int]] = []
        current = start
        while current != end:
            next_value = current + (direction * step)
            if direction > 0:
                next_value = min(next_value, end)
            else:
                next_value = max(next_value, end)
            offsets.append((0, next_value))
            current = next_value
        return offsets

    def _complete_camera_centering(self, reason: str) -> None:
        self.camera_centering_active = False
        self.camera_centering_status_var.set(f"Camera centering: {reason}.")
        self.log(f"Camera centering finished: {reason}.")
        if reason == "centered" and self.startup_hand_calibration_pending:
            self.start_hand_calibration(auto=True)

    @staticmethod
    def _bounded_camera_step(value: float) -> int:
        if value == 0.0:
            return 0
        step = int(round(value))
        if step == 0:
            step = 1 if value > 0 else -1
        return max(-CAMERA_CENTER_MAX_STEP_TICKS, min(CAMERA_CENTER_MAX_STEP_TICKS, step))

    def _update_preview(self) -> None:
        frame = self.hand_tracker.get_latest_frame_rgb()
        if frame is None:
            return
        image = Image.fromarray(frame)
        self._draw_hand_calibration_overlay(image)
        preview_image = image.copy()
        preview_image.thumbnail((960, 540), Image.Resampling.LANCZOS)
        self._photo = ImageTk.PhotoImage(image=preview_image)
        self.preview_label.configure(image=self._photo)
        if self.camera_popup is not None and self.camera_popup.winfo_exists() and self.camera_popup_label is not None:
            popup_image = image.copy()
            width = max(640, self.camera_popup.winfo_width() - 20)
            height = max(360, self.camera_popup.winfo_height() - 20)
            popup_image.thumbnail((width, height), Image.Resampling.LANCZOS)
            self._popup_photo = ImageTk.PhotoImage(image=popup_image)
            self.camera_popup_label.configure(image=self._popup_photo)
        elif self.camera_popup is not None:
            self.camera_popup = None
            self.camera_popup_label = None
            self._popup_photo = None

    def _draw_hand_calibration_overlay(self, image: Image.Image) -> None:
        if not self.hand_calibration_active:
            return
        draw = ImageDraw.Draw(image, "RGBA")
        width, height = image.size
        inset = max(4, min(width, height) // 80)
        draw.rectangle(
            (inset, inset, width - inset, height - inset),
            outline=(70, 190, 255, 190),
            width=max(2, inset // 2),
        )

    def _update_text_views(self) -> None:
        serial_state = self.serial_bridge.get_latest_state()
        input_dict = self._latest_snapshot.get("input_dict", {})
        input_array = self._latest_snapshot.get("input_array", [])
        self._update_data_table(serial_state, input_dict)
        self._update_dashboard_text(serial_state)
        self._update_keyboard_grid(serial_state)
        self._set_text(self.json_text, json.dumps(serial_state, indent=2))
        fused = {
            "field_order": FIELD_ORDER,
            "input_array": input_array,
            "input_dict": input_dict,
        }
        self._set_text(self.fused_text, json.dumps(fused, indent=2))

    def _update_dashboard_text(self, serial_state: dict[str, Any]) -> None:
        devices = serial_state.get("devices", {}) if isinstance(serial_state, dict) else {}
        wrist = devices.get("wristband", {}) if isinstance(devices, dict) else {}
        camdock = devices.get("camdock", {}) if isinstance(devices, dict) else {}
        keyboard = devices.get("keyboard", {}) if isinstance(devices, dict) else {}
        fans = devices.get("fans", {}) if isinstance(devices, dict) else {}
        lines = [
            f"Antenna serial: {'connected' if self.serial_bridge.is_connected else 'disconnected'}",
            f"Wristband: {wrist.get('status', 'not_connected') if isinstance(wrist, dict) else 'not_connected'}",
            f"Cam Dock: {camdock.get('status', 'not_connected') if isinstance(camdock, dict) else 'not_connected'}",
            f"Cam Dock battery: {camdock.get('battery_level', '-')}% / {camdock.get('battery_voltage', '-')}V"
            if isinstance(camdock, dict)
            else "Cam Dock battery: -",
            f"Keyboard: {keyboard.get('status', 'not_connected') if isinstance(keyboard, dict) else 'not_connected'}",
            f"Fans: {fans.get('status', 'not_connected') if isinstance(fans, dict) else 'not_connected'}",
            f"Fan state: {'on' if fans.get('fan_on') else 'off'}" if isinstance(fans, dict) else "Fan state: -",
            f"Fans battery: {fans.get('battery_level', '-')}% / {fans.get('battery_voltage', '-')}V"
            if isinstance(fans, dict)
            else "Fans battery: -",
            self.camera_centering_status_var.get(),
            self.hand_calibration_status_var.get(),
        ]
        if hasattr(self, "dashboard_text"):
            self._set_text(self.dashboard_text, "\n".join(lines))

    def _update_keyboard_grid(self, serial_state: dict[str, Any]) -> None:
        if not self.keyboard_cells:
            return

        devices = serial_state.get("devices", {}) if isinstance(serial_state, dict) else {}
        keyboard = devices.get("keyboard", {}) if isinstance(devices, dict) else {}
        tof = keyboard.get("tof", {}) if isinstance(keyboard, dict) else {}
        valid = keyboard.get("valid", {}) if isinstance(keyboard, dict) else {}
        status = keyboard.get("status", "not_connected") if isinstance(keyboard, dict) else "not_connected"

        inactive_bg = "#f8fafc"
        active_bg = "#22c55e"
        active_fg = "#052e16"
        inactive_fg = "#1f2d3d"
        for row_cells in self.keyboard_cells:
            for cell in row_cells:
                cell.configure(bg=inactive_bg, fg=inactive_fg, text="")

        distance_text: list[str] = []
        for index in range(4):
            sensor_key = f"sensor_{index + 1}"
            distance = tof.get(f"{sensor_key}_mm") if isinstance(tof, dict) else None
            is_valid = bool(valid.get(sensor_key)) if isinstance(valid, dict) else distance is not None
            distance_text.append(f"S{index + 1}: {self._format_table_value(distance)} mm")
            if status != "ok" or not is_valid or not isinstance(distance, (int, float)):
                continue
            if distance < 0 or distance > KEYBOARD_DISTANCE_ROWS * KEYBOARD_DISTANCE_BAND_MM:
                continue
            row = min(KEYBOARD_DISTANCE_ROWS - 1, int(distance // KEYBOARD_DISTANCE_BAND_MM))
            cell = self.keyboard_cells[row][index]
            cell.configure(bg=active_bg, fg=active_fg, text=f"{distance:.0f} mm")

        self.keyboard_status_var.set(f"Keyboard: {status}. " + ", ".join(distance_text))

    def _update_data_table(self, serial_state: dict[str, Any], input_dict: dict[str, Any]) -> None:
        devices = serial_state.get("devices", {}) if isinstance(serial_state, dict) else {}
        wrist = devices.get("wristband", {}) if isinstance(devices, dict) else {}
        camdock = devices.get("camdock", {}) if isinstance(devices, dict) else {}
        keyboard = devices.get("keyboard", {}) if isinstance(devices, dict) else {}
        fans = devices.get("fans", {}) if isinstance(devices, dict) else {}
        wrist_accel = wrist.get("accel", {}) if isinstance(wrist, dict) else {}
        wrist_gyro = wrist.get("gyro", {}) if isinstance(wrist, dict) else {}
        tof = camdock.get("tof", {}) if isinstance(camdock, dict) else {}
        keyboard_tof = keyboard.get("tof", {}) if isinstance(keyboard, dict) else {}
        keyboard_valid = keyboard.get("valid", {}) if isinstance(keyboard, dict) else {}
        fan_temps = fans.get("temps", {}) if isinstance(fans, dict) else {}
        hands = self._latest_snapshot.get("hand_state", {})
        right = hands.get("right", {}) if isinstance(hands, dict) else {}
        left = hands.get("left", {}) if isinstance(hands, dict) else {}
        face = self.hand_tracker.get_latest_face()

        cells: list[tuple[str, str, str]] = []

        def add(group: str, field: str, value: Any) -> None:
            cells.append((group, field, self._format_table_value(value)))

        def image_y_to_up(values: dict[str, Any]) -> float | None:
            if not values.get("visible") or values.get("y") is None:
                return None
            try:
                image_y = float(values["y"])
            except (TypeError, ValueError):
                return None
            return 1.0 - max(0.0, min(1.0, image_y))

        add("Antenna", "USB serial", "connected" if self.serial_bridge.is_connected else "disconnected")
        add("Antenna", "t_ms", serial_state.get("t_ms"))
        add("Antenna", "sequence", serial_state.get("sequence"))
        add("Wristband", "status", wrist.get("status") if isinstance(wrist, dict) else None)
        add("Wristband", "sequence", wrist.get("sequence") if isinstance(wrist, dict) else None)
        add("Wristband", "battery_level", wrist.get("battery_level") if isinstance(wrist, dict) else None)
        add("Wristband", "battery_voltage", wrist.get("battery_voltage") if isinstance(wrist, dict) else None)
        for axis in ("x", "y", "z"):
            add("Wristband", f"accel_{axis}", wrist_accel.get(axis))
            add("Wristband", f"gyro_{axis}", wrist_gyro.get(axis))
        for field in ("pitch", "roll"):
            add("Wristband", field, wrist.get(field) if isinstance(wrist, dict) else None)

        add("Cam Dock", "status", camdock.get("status") if isinstance(camdock, dict) else None)
        add("Cam Dock", "sequence", camdock.get("sequence") if isinstance(camdock, dict) else None)
        add("Cam Dock", "battery_level", camdock.get("battery_level") if isinstance(camdock, dict) else None)
        add("Cam Dock", "battery_voltage", camdock.get("battery_voltage") if isinstance(camdock, dict) else None)
        add("Cam Dock", "active_target", camdock.get("active_target") if isinstance(camdock, dict) else None)
        add("Cam Dock", "tof_left_mm", tof.get("left_mm"))
        add("Cam Dock", "tof_right_mm", tof.get("right_mm"))

        add("Keyboard", "status", keyboard.get("status") if isinstance(keyboard, dict) else None)
        add("Keyboard", "sequence", keyboard.get("sequence") if isinstance(keyboard, dict) else None)
        for sensor_index in range(1, 5):
            add("Keyboard", f"sensor_{sensor_index}_mm", keyboard_tof.get(f"sensor_{sensor_index}_mm") if isinstance(keyboard_tof, dict) else None)
            add("Keyboard", f"sensor_{sensor_index}_valid", keyboard_valid.get(f"sensor_{sensor_index}") if isinstance(keyboard_valid, dict) else None)

        add("Fans", "status", fans.get("status") if isinstance(fans, dict) else None)
        add("Fans", "fan_on", fans.get("fan_on") if isinstance(fans, dict) else None)
        add("Fans", "sequence", fans.get("sequence") if isinstance(fans, dict) else None)
        add("Fans", "battery_level", fans.get("battery_level") if isinstance(fans, dict) else None)
        add("Fans", "battery_voltage", fans.get("battery_voltage") if isinstance(fans, dict) else None)
        add("Fans", "temp_1_c", fan_temps.get("sensor_1_c") if isinstance(fan_temps, dict) else None)
        add("Fans", "temp_2_c", fan_temps.get("sensor_2_c") if isinstance(fan_temps, dict) else None)
        add(
            "Fans",
            "last_command_sequence",
            fans.get("last_command_sequence") if isinstance(fans, dict) else None,
        )

        add("Camera", "source", self.camera_source_var.get())
        add("Camera", "mirror", self.camera_mirror_var.get())
        add("Camera", "centering", self.camera_centering_status_var.get())
        add("Camera", "face_visible", face.get("visible"))
        add("Camera", "face_x", face.get("x"))
        add("Camera", "face_top_y", face.get("top_y"))
        add("Calibration", "phase_active", self.hand_calibration_active)
        add("Calibration", "status", self.hand_calibration_status_var.get())
        add("Calibration", "boundary_left", self.config.calibration.get("hand_boundary_left"))
        add("Calibration", "boundary_right", self.config.calibration.get("hand_boundary_right"))
        add("Calibration", "boundary_top", self.config.calibration.get("hand_boundary_top"))
        add("Calibration", "boundary_bottom", self.config.calibration.get("hand_boundary_bottom"))
        add("Calibration", "dock_geometry", self.config.calibration.get("use_dock_geometry"))
        add("Calibration", "initial_distance_mm", self.config.calibration.get("initial_hand_distance_mm"))
        add("MediaPipe", "right_visible", right.get("visible"))
        add("MediaPipe", "right_gesture", right.get("gesture"))
        add("MediaPipe", "right_x", right.get("x"))
        add("MediaPipe", "right_image_y", right.get("y"))
        add("MediaPipe", "right_y_up", image_y_to_up(right))
        add("MediaPipe", "right_score", right.get("score"))
        add("MediaPipe", "left_visible", left.get("visible"))
        add("MediaPipe", "left_gesture", left.get("gesture"))
        add("MediaPipe", "left_x", left.get("x"))
        add("MediaPipe", "left_image_y", left.get("y"))
        add("MediaPipe", "left_y_up", image_y_to_up(left))
        add("MediaPipe", "left_score", left.get("score"))
        add("Gesture Recorder", "recording", self.recorder.is_recording)
        add("Gesture Recorder", "gesture_name", self.gesture_name_var.get())

        for field in FIELD_ORDER:
            add("Fused Input", field, input_dict.get(field))

        self._remember_live_data(cells)
        self._set_transposed_table(self._filtered_live_data(cells))

    def _remember_live_data(self, cells: list[tuple[str, str, str]]) -> None:
        self.data_history.insert(0, cells)
        del self.data_history[LIVE_DATA_HISTORY_ROWS:]

    def _filtered_live_data(self, cells: list[tuple[str, str, str]]) -> list[tuple[str, str, str]]:
        search = self.live_data_search_var.get().strip().lower() if hasattr(self, "live_data_search_var") else ""
        filtered = []
        for group, field, value in cells:
            if not self._live_data_group_visible(group):
                continue
            haystack = f"{group} {field} {value}".lower()
            if search and search not in haystack:
                continue
            filtered.append((group, field, value))
        return filtered

    def _live_data_group_visible(self, group: str) -> bool:
        selected = self.live_data_device_var.get() if hasattr(self, "live_data_device_var") else "All"
        return selected == "All" or selected == group

    def _set_transposed_table(self, cells: list[tuple[str, str, str]]) -> None:
        columns = ["device", "input", "now"] + [f"prev_{index}" for index in range(1, LIVE_DATA_HISTORY_ROWS)]
        if columns != self.data_columns:
            self.data_columns = columns
            self.data_tree.configure(columns=columns)
            self.data_tree.heading("device", text="Device")
            self.data_tree.heading("input", text="Input")
            self.data_tree.heading("now", text="Current")
            self.data_tree.column("device", width=130, stretch=False)
            self.data_tree.column("input", width=190, stretch=False)
            self.data_tree.column("now", width=140, stretch=False)
            for index, column in enumerate(columns[3:], start=1):
                self.data_tree.heading(column, text=f"Prev {index}")
                self.data_tree.column(column, width=120, stretch=False)
            for item_id in self.data_tree.get_children():
                self.data_tree.delete(item_id)

        for item_id in self.data_tree.get_children():
            self.data_tree.delete(item_id)

        history_maps = []
        for history_cells in self.data_history[:LIVE_DATA_HISTORY_ROWS]:
            history_maps.append({(group, field): value for group, field, value in history_cells})

        for row_index, (group, field, value) in enumerate(cells):
            values = [group, field, value]
            for history_index in range(1, LIVE_DATA_HISTORY_ROWS):
                history = history_maps[history_index] if history_index < len(history_maps) else {}
                values.append(history.get((group, field), "-"))
            self.data_tree.insert("", "end", iid=f"input_{row_index}", values=values)

    @staticmethod
    def _format_table_value(value: Any) -> str:
        if value is None:
            return "-"
        if isinstance(value, bool):
            return "true" if value else "false"
        if isinstance(value, float):
            return f"{value:.3f}"
        if isinstance(value, (dict, list)):
            return json.dumps(value, separators=(",", ":"))
        return str(value)

    def _snapshot_provider(self) -> dict[str, Any]:
        serial_state = self.serial_bridge.get_latest_state()
        hand_state = self.hand_tracker.get_latest_hands()
        return self.fusion_state.build_snapshot(serial_state, hand_state)

    def _apply_calibration_entries(self) -> dict[str, Any] | None:
        calibration: dict[str, Any] = dict(self.config.calibration)
        try:
            for key, var in self.calibration_vars.items():
                raw = var.get().strip()
                if key in FLOAT_CALIBRATION_KEYS:
                    calibration[key] = float(raw)
                else:
                    calibration[key] = int(float(raw))
        except ValueError:
            self.log("Calibration values must be numeric.")
            return None
        self.config.calibration = calibration
        self.servo_controller.update_calibration(calibration)
        return calibration

    def _drain_log_queue(self) -> None:
        while True:
            try:
                message = self.log_queue.get_nowait()
            except queue.Empty:
                break
            self.log_text.insert("end", message + "\n")
            self.log_text.see("end")

    @staticmethod
    def _set_text(widget: tk.Text, text: str) -> None:
        widget.delete("1.0", "end")
        widget.insert("1.0", text)
