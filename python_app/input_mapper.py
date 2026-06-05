from __future__ import annotations

import json
import time
import uuid
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable

from config import MAPPING_PATH
from input_backend import InputBackend, normalize_mouse_button, parse_key_combo


MAPPING_SCHEMA_VERSION = 1
DEFAULT_PROFILE_NAME = "Default"
DEFAULT_MAPPING_PATH = MAPPING_PATH

ACTION_TYPES = {
    "keyboard_tap",
    "keyboard_hold",
    "keyboard_repeat",
    "mouse_click",
    "mouse_hold",
    "mouse_scroll",
    "mouse_move",
    "mouse_absolute",
}

STANDARD_COMPARATORS = {
    "lt",
    "lte",
    "gt",
    "gte",
    "between",
    "outside",
    "eq",
    "neq",
    "present",
    "truthy",
    "falsey",
}
DELTA_COMPARATORS = {
    "delta_decrease",
    "delta_increase",
}
COMPARATORS = STANDARD_COMPARATORS | DELTA_COMPARATORS
GESTURE_COOLDOWN_SEC = 0.6
WRIST_GESTURE_SIGNAL_NAMES = {
    "fused.wrist_roll_right_detected": "wrist_roll_right",
    "fused.wrist_roll_left_detected": "wrist_roll_left",
    "fused.wrist_pitch_up_detected": "wrist_pitch_up",
    "fused.wrist_pitch_down_detected": "wrist_pitch_down",
    "fused.wrist_roll_right_then_neutral_detected": "wrist_roll_right_then_neutral",
}


@dataclass
class SignalValue:
    id: str
    group: str
    label: str
    value: Any

    @property
    def display_value(self) -> str:
        if self.value is None:
            return "-"
        if isinstance(self.value, bool):
            return "true" if self.value else "false"
        if isinstance(self.value, float):
            return f"{self.value:.3f}"
        if isinstance(self.value, (dict, list)):
            return json.dumps(self.value, separators=(",", ":"))
        return str(self.value)


class SignalCatalog:
    GROUP_ORDER = {
        "Keyboard": 0,
        "Wristband": 1,
        "Cam Dock": 2,
        "Hands": 3,
        "Camera": 4,
        "Audio Dock": 5,
        "Fans": 6,
        "Fused Input": 7,
        "Antenna": 8,
    }

    @classmethod
    def flatten(cls, snapshot: dict[str, Any]) -> dict[str, SignalValue]:
        signals: dict[str, SignalValue] = {}

        def add(group: str, signal_id: str, label: str, value: Any) -> None:
            signals[signal_id] = SignalValue(signal_id, group, label, value)

        raw_state = snapshot.get("raw_device_state", {}) if isinstance(snapshot, dict) else {}
        input_dict = snapshot.get("input_dict", {}) if isinstance(snapshot, dict) else {}
        hand_state = snapshot.get("hand_state", {}) if isinstance(snapshot, dict) else {}
        face_state = snapshot.get("face_state", {}) if isinstance(snapshot, dict) else {}
        devices = raw_state.get("devices", {}) if isinstance(raw_state, dict) else {}

        add("Antenna", "antenna.connected", "Antenna connected", bool(raw_state))
        if isinstance(raw_state, dict):
            add("Antenna", "antenna.t_ms", "Antenna t_ms", raw_state.get("t_ms"))
            add("Antenna", "antenna.sequence", "Antenna sequence", raw_state.get("sequence"))

        keyboard = devices.get("keyboard", {}) if isinstance(devices, dict) else {}
        keyboard_tof = keyboard.get("tof", {}) if isinstance(keyboard, dict) else {}
        keyboard_valid = keyboard.get("valid", {}) if isinstance(keyboard, dict) else {}
        add("Keyboard", "keyboard.status", "Status", keyboard.get("status") if isinstance(keyboard, dict) else None)
        add("Keyboard", "keyboard.input", "Input state", keyboard.get("input") if isinstance(keyboard, dict) else None)
        add("Keyboard", "keyboard.sequence", "Sequence", keyboard.get("sequence") if isinstance(keyboard, dict) else None)
        for index in range(1, 5):
            add(
                "Keyboard",
                f"keyboard.sensor_{index}_mm",
                f"Sensor {index} distance mm",
                keyboard_tof.get(f"sensor_{index}_mm") if isinstance(keyboard_tof, dict) else None,
            )
            add(
                "Keyboard",
                f"keyboard.sensor_{index}_valid",
                f"Sensor {index} valid",
                keyboard_valid.get(f"sensor_{index}") if isinstance(keyboard_valid, dict) else None,
            )

        wrist = devices.get("wristband", {}) if isinstance(devices, dict) else {}
        wrist_accel = wrist.get("accel", {}) if isinstance(wrist, dict) else {}
        wrist_gyro = wrist.get("gyro", {}) if isinstance(wrist, dict) else {}
        add("Wristband", "wristband.status", "Status", wrist.get("status") if isinstance(wrist, dict) else None)
        add("Wristband", "wristband.sequence", "Sequence", wrist.get("sequence") if isinstance(wrist, dict) else None)
        add("Wristband", "wristband.battery_level", "Battery level", wrist.get("battery_level") if isinstance(wrist, dict) else None)
        add("Wristband", "wristband.pitch", "Pitch", wrist.get("pitch") if isinstance(wrist, dict) else None)
        add("Wristband", "wristband.roll", "Roll", wrist.get("roll") if isinstance(wrist, dict) else None)
        add("Wristband", "wristband.yaw", "Yaw", wrist.get("yaw") if isinstance(wrist, dict) else None)
        for axis in ("x", "y", "z"):
            add("Wristband", f"wristband.accel_{axis}", f"Accel {axis}", wrist_accel.get(axis))
            add("Wristband", f"wristband.gyro_{axis}", f"Gyro {axis}", wrist_gyro.get(axis))

        camdock = devices.get("camdock", {}) if isinstance(devices, dict) else {}
        camdock_tof = camdock.get("tof", {}) if isinstance(camdock, dict) else {}
        add("Cam Dock", "camdock.status", "Status", camdock.get("status") if isinstance(camdock, dict) else None)
        add("Cam Dock", "camdock.sequence", "Sequence", camdock.get("sequence") if isinstance(camdock, dict) else None)
        add("Cam Dock", "camdock.active_target", "Active target", camdock.get("active_target") if isinstance(camdock, dict) else None)
        add("Cam Dock", "camdock.tof_left_mm", "Left ToF mm", camdock_tof.get("left_mm") if isinstance(camdock_tof, dict) else None)
        add("Cam Dock", "camdock.tof_right_mm", "Right ToF mm", camdock_tof.get("right_mm") if isinstance(camdock_tof, dict) else None)
        add("Cam Dock", "camdock.battery_level", "Battery level", camdock.get("battery_level") if isinstance(camdock, dict) else None)

        if isinstance(hand_state, dict):
            for side in ("right", "left"):
                hand = hand_state.get(side, {})
                if not isinstance(hand, dict):
                    hand = {}
                label = side.title()
                add("Hands", f"hands.{side}.visible", f"{label} visible", hand.get("visible"))
                add("Hands", f"hands.{side}.x", f"{label} x", hand.get("x"))
                add("Hands", f"hands.{side}.y", f"{label} image y", hand.get("y"))
                z_mm = input_dict.get(f"{side}_hand_z_mm") if hand.get("visible") else None
                add("Hands", f"hands.{side}.z_mm", f"{label} z mm", z_mm)
                add("Hands", f"hands.{side}.score", f"{label} score", hand.get("score"))
                add("Hands", f"hands.{side}.gesture", f"{label} gesture", hand.get("gesture"))

        if isinstance(face_state, dict):
            add("Camera", "camera.face_visible", "Face visible", face_state.get("visible"))
            add("Camera", "camera.face_x", "Face x", face_state.get("x"))
            add("Camera", "camera.face_top_y", "Face top y", face_state.get("top_y"))
            add("Camera", "camera.face_y", "Face y", face_state.get("y"))

        audiodock = devices.get("audiodock", {}) if isinstance(devices, dict) else {}
        add("Audio Dock", "audiodock.status", "Status", audiodock.get("status") if isinstance(audiodock, dict) else None)
        add("Audio Dock", "audiodock.clap_detected", "Clap detected", audiodock.get("clap_detected") if isinstance(audiodock, dict) else None)
        add("Audio Dock", "audiodock.clap_type", "Clap type", audiodock.get("clap_type") if isinstance(audiodock, dict) else None)

        fans = devices.get("fans", {}) if isinstance(devices, dict) else {}
        fan_temps = fans.get("temps", {}) if isinstance(fans, dict) else {}
        add("Fans", "fans.status", "Status", fans.get("status") if isinstance(fans, dict) else None)
        add("Fans", "fans.input", "Input state", fans.get("input") if isinstance(fans, dict) else None)
        add("Fans", "fans.fan_on", "Fan on", fans.get("fan_on") if isinstance(fans, dict) else None)
        add("Fans", "fans.temp_1_c", "Temp 1 C", fan_temps.get("sensor_1_c") if isinstance(fan_temps, dict) else None)
        add("Fans", "fans.temp_2_c", "Temp 2 C", fan_temps.get("sensor_2_c") if isinstance(fan_temps, dict) else None)

        if isinstance(input_dict, dict):
            for key, value in input_dict.items():
                add("Fused Input", f"fused.{key}", key, value)

        return signals

    @classmethod
    def rows(cls, snapshot: dict[str, Any]) -> list[SignalValue]:
        return sorted(
            cls.flatten(snapshot).values(),
            key=lambda signal: (cls.GROUP_ORDER.get(signal.group, 99), signal.label.lower(), signal.id),
        )


@dataclass
class MappingAction:
    type: str = "keyboard_tap"
    keys: list[str] = field(default_factory=list)
    button: str = "left"
    clicks: int = 1
    interval_ms: int = 250
    scroll_x: int = 0
    scroll_y: int = 1
    speed_x: float = 0.0
    speed_y: float = 0.0
    absolute_x: float = 0.5
    absolute_y: float = 0.5
    absolute_x_source: str = ""
    absolute_y_source: str = ""
    absolute_x_invert: bool = False
    absolute_y_invert: bool = False
    absolute_deadband: float = 0.0
    absolute_smoothing_alpha: float = 1.0
    continuous: bool = False

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "MappingAction":
        if not isinstance(data, dict):
            raise ValueError("mapping action must be an object")
        action_type = str(data.get("type", "keyboard_tap"))
        if action_type not in ACTION_TYPES:
            raise ValueError(f"unknown mapping action type: {action_type}")
        smoothing_alpha = float(data.get("absolute_smoothing_alpha", 1.0) or 1.0)
        return cls(
            type=action_type,
            keys=parse_key_combo(data.get("keys", [])),
            button=normalize_mouse_button(str(data.get("button", "left"))),
            clicks=max(1, int(float(data.get("clicks", 1) or 1))),
            interval_ms=max(20, int(float(data.get("interval_ms", 250) or 250))),
            scroll_x=int(float(data.get("scroll_x", 0) or 0)),
            scroll_y=int(float(data.get("scroll_y", 1) or 0)),
            speed_x=float(data.get("speed_x", 0.0) or 0.0),
            speed_y=float(data.get("speed_y", 0.0) or 0.0),
            absolute_x=max(0.0, min(1.0, float(data.get("absolute_x", 0.5) or 0.0))),
            absolute_y=max(0.0, min(1.0, float(data.get("absolute_y", 0.5) or 0.0))),
            absolute_x_source=str(data.get("absolute_x_source") or ""),
            absolute_y_source=str(data.get("absolute_y_source") or ""),
            absolute_x_invert=bool(data.get("absolute_x_invert", False)),
            absolute_y_invert=bool(data.get("absolute_y_invert", False)),
            absolute_deadband=max(0.0, min(0.5, float(data.get("absolute_deadband", 0.0) or 0.0))),
            absolute_smoothing_alpha=max(0.01, min(1.0, smoothing_alpha)),
            continuous=bool(data.get("continuous", False)),
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "type": self.type,
            "keys": list(self.keys),
            "button": self.button,
            "clicks": self.clicks,
            "interval_ms": self.interval_ms,
            "scroll_x": self.scroll_x,
            "scroll_y": self.scroll_y,
            "speed_x": self.speed_x,
            "speed_y": self.speed_y,
            "absolute_x": self.absolute_x,
            "absolute_y": self.absolute_y,
            "absolute_x_source": self.absolute_x_source,
            "absolute_y_source": self.absolute_y_source,
            "absolute_x_invert": self.absolute_x_invert,
            "absolute_y_invert": self.absolute_y_invert,
            "absolute_deadband": self.absolute_deadband,
            "absolute_smoothing_alpha": self.absolute_smoothing_alpha,
            "continuous": self.continuous,
        }

    @property
    def mode(self) -> str:
        if self.type.endswith("_hold"):
            return "hold"
        if self.type in {"keyboard_repeat", "mouse_scroll"}:
            return "repeat"
        if self.type == "mouse_move":
            return "continuous"
        return "tap"

    def summary(self) -> str:
        if self.type.startswith("keyboard"):
            combo = "+".join(self.keys) if self.keys else "(no keys)"
            if self.type == "keyboard_repeat":
                return f"repeat {combo}"
            if self.type == "keyboard_hold":
                return f"hold {combo}"
            return f"tap {combo}"
        if self.type == "mouse_click":
            return f"click {self.button}"
        if self.type == "mouse_hold":
            return f"hold mouse {self.button}"
        if self.type == "mouse_scroll":
            return f"scroll x={self.scroll_x} y={self.scroll_y}"
        if self.type == "mouse_move":
            return f"move x={self.speed_x:g}/s y={self.speed_y:g}/s"
        if self.type == "mouse_absolute":
            if self.absolute_x_source or self.absolute_y_source:
                x_source = self.absolute_x_source or f"{self.absolute_x:.2f}"
                y_source = self.absolute_y_source or f"{self.absolute_y:.2f}"
                flags = []
                if self.absolute_x_invert:
                    flags.append("invert x")
                if self.absolute_y_invert:
                    flags.append("invert y")
                if self.absolute_deadband > 0.0:
                    flags.append(f"deadband {self.absolute_deadband:g}")
                suffix = f" ({', '.join(flags)})" if flags else ""
                return f"follow {x_source},{y_source}{suffix}"
            return f"move absolute {self.absolute_x:.2f},{self.absolute_y:.2f}"
        return self.type


@dataclass
class MappingCondition:
    source: str = ""
    comparator: str = "truthy"
    threshold: Any = True
    low: Any = 0.0
    high: Any = 1.0
    hysteresis: float = 0.0
    output_keys: list[str] = field(default_factory=list)

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "MappingCondition":
        if not isinstance(data, dict):
            raise ValueError("mapping condition must be an object")
        comparator = str(data.get("comparator", "truthy"))
        if comparator not in STANDARD_COMPARATORS:
            raise ValueError(f"unknown condition comparator: {comparator}")
        return cls(
            source=str(data.get("source") or ""),
            comparator=comparator,
            threshold=data.get("threshold", True),
            low=data.get("low", 0.0),
            high=data.get("high", 1.0),
            hysteresis=max(0.0, float(data.get("hysteresis", 0.0) or 0.0)),
            output_keys=parse_key_combo(data.get("output_keys", [])),
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "source": self.source,
            "comparator": self.comparator,
            "threshold": self.threshold,
            "low": self.low,
            "high": self.high,
            "hysteresis": self.hysteresis,
            "output_keys": list(self.output_keys),
        }

    def summary(self) -> str:
        if self.comparator in {"present", "truthy", "falsey"}:
            summary = f"{self.source} {self.comparator}"
        elif self.comparator in {"between", "outside"}:
            summary = f"{self.source} {self.comparator} {self.low}..{self.high}"
        else:
            summary = f"{self.source} {self.comparator} {self.threshold}"
        if self.output_keys:
            summary += f" -> hold {'+'.join(self.output_keys)}"
        return summary


@dataclass
class MappingRule:
    id: str = field(default_factory=lambda: uuid.uuid4().hex)
    name: str = "New mapping"
    enabled: bool = True
    source: str = ""
    comparator: str = "lt"
    threshold: Any = 100.0
    low: Any = 0.0
    high: Any = 1.0
    hysteresis: float = 0.0
    debounce_ms: int = 0
    gate_source: str = ""
    gate_comparator: str = "truthy"
    gate_threshold: Any = True
    gate_low: Any = 0.0
    gate_high: Any = 1.0
    conditions: list[MappingCondition] = field(default_factory=list)
    recognition_label: str = ""
    action: MappingAction = field(default_factory=MappingAction)

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "MappingRule":
        if not isinstance(data, dict):
            raise ValueError("mapping rule must be an object")
        comparator = str(data.get("comparator", "lt"))
        if comparator not in COMPARATORS:
            raise ValueError(f"unknown comparator: {comparator}")
        gate_comparator = str(data.get("gate_comparator", "truthy"))
        if gate_comparator not in STANDARD_COMPARATORS:
            raise ValueError(f"unknown gate comparator: {gate_comparator}")
        return cls(
            id=str(data.get("id") or uuid.uuid4().hex),
            name=str(data.get("name") or "Mapping"),
            enabled=bool(data.get("enabled", True)),
            source=str(data.get("source") or ""),
            comparator=comparator,
            threshold=data.get("threshold", 100.0),
            low=data.get("low", 0.0),
            high=data.get("high", 1.0),
            hysteresis=max(0.0, float(data.get("hysteresis", 0.0) or 0.0)),
            debounce_ms=max(0, int(float(data.get("debounce_ms", 0) or 0))),
            gate_source=str(data.get("gate_source") or ""),
            gate_comparator=gate_comparator,
            gate_threshold=data.get("gate_threshold", True),
            gate_low=data.get("gate_low", 0.0),
            gate_high=data.get("gate_high", 1.0),
            conditions=[
                MappingCondition.from_dict(item)
                for item in data.get("conditions", [])
                if isinstance(item, dict)
            ],
            recognition_label=str(data.get("recognition_label") or ""),
            action=MappingAction.from_dict(data.get("action", {})),
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "id": self.id,
            "name": self.name,
            "enabled": self.enabled,
            "source": self.source,
            "comparator": self.comparator,
            "threshold": self.threshold,
            "low": self.low,
            "high": self.high,
            "hysteresis": self.hysteresis,
            "debounce_ms": self.debounce_ms,
            "gate_source": self.gate_source,
            "gate_comparator": self.gate_comparator,
            "gate_threshold": self.gate_threshold,
            "gate_low": self.gate_low,
            "gate_high": self.gate_high,
            "conditions": [condition.to_dict() for condition in self.conditions],
            "recognition_label": self.recognition_label,
            "action": self.action.to_dict(),
        }

    def all_conditions(self) -> list[MappingCondition]:
        conditions = list(self.conditions)
        if self.gate_source:
            conditions.append(
                MappingCondition(
                    source=self.gate_source,
                    comparator=self.gate_comparator,
                    threshold=self.gate_threshold,
                    low=self.gate_low,
                    high=self.gate_high,
                    hysteresis=self.hysteresis,
                )
            )
        return conditions

    def condition_summary(self) -> str:
        if self.comparator in {"delta_decrease", "delta_increase"}:
            direction = "decrease" if self.comparator == "delta_decrease" else "increase"
            summary = f"{direction} by {self.threshold}"
        elif self.comparator in {"present", "truthy", "falsey"}:
            summary = self.comparator
        elif self.comparator in {"between", "outside"}:
            summary = f"{self.comparator} {self.low}..{self.high}"
        else:
            summary = f"{self.comparator} {self.threshold}"
        conditions = [condition.summary() for condition in self.all_conditions() if condition.source]
        if conditions:
            return f"{summary} when {' + '.join(conditions)}"
        return summary


@dataclass
class MappingProfile:
    name: str = DEFAULT_PROFILE_NAME
    mappings: list[MappingRule] = field(default_factory=list)

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "MappingProfile":
        if not isinstance(data, dict):
            raise ValueError("profile must be an object")
        mappings = [MappingRule.from_dict(item) for item in data.get("mappings", [])]
        return cls(name=str(data.get("name") or DEFAULT_PROFILE_NAME), mappings=mappings)

    def to_dict(self) -> dict[str, Any]:
        return {"name": self.name, "mappings": [rule.to_dict() for rule in self.mappings]}


@dataclass
class MappingConfig:
    version: int = MAPPING_SCHEMA_VERSION
    enabled_on_start: bool = False
    active_profile: str = DEFAULT_PROFILE_NAME
    profiles: list[MappingProfile] = field(default_factory=lambda: [MappingProfile()])

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "MappingConfig":
        if not isinstance(data, dict):
            raise ValueError("mapping config must be an object")
        version = int(data.get("version", 0))
        if version != MAPPING_SCHEMA_VERSION:
            raise ValueError(f"unsupported mapping config version: {version}")
        profiles = [MappingProfile.from_dict(item) for item in data.get("profiles", [])]
        if not profiles:
            profiles = [MappingProfile()]
        _upgrade_wrist_pitch_rules(profiles)
        active_profile = str(data.get("active_profile") or profiles[0].name)
        if active_profile not in {profile.name for profile in profiles}:
            active_profile = profiles[0].name
        return cls(
            version=version,
            enabled_on_start=bool(data.get("enabled_on_start", False)),
            active_profile=active_profile,
            profiles=profiles,
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "version": self.version,
            "enabled_on_start": self.enabled_on_start,
            "active_profile": self.active_profile,
            "profiles": [profile.to_dict() for profile in self.profiles],
        }

    def active(self) -> MappingProfile:
        for profile in self.profiles:
            if profile.name == self.active_profile:
                return profile
        self.active_profile = self.profiles[0].name
        return self.profiles[0]

    def profile_names(self) -> list[str]:
        return [profile.name for profile in self.profiles]


def default_mapping_config() -> MappingConfig:
    return MappingConfig()


def _upgrade_wrist_pitch_rules(profiles: list[MappingProfile]) -> None:
    for profile in profiles:
        for rule in profile.mappings:
            if rule.source != "fused.wrist_pitch_delta":
                continue
            if not any(condition.source == "fused.wrist_pitch_dominant" for condition in rule.all_conditions()):
                continue
            try:
                threshold = float(rule.threshold)
            except (TypeError, ValueError):
                continue
            if rule.comparator == "gt" and threshold > 0:
                rule.source = "fused.wrist_pitch_up_detected"
            elif rule.comparator == "lt" and threshold < 0:
                rule.source = "fused.wrist_pitch_down_detected"
            else:
                continue
            rule.comparator = "truthy"
            rule.threshold = True
            rule.low = 0.0
            rule.high = 1.0
            rule.hysteresis = 0.0


def load_mapping_config(path: Path = DEFAULT_MAPPING_PATH) -> tuple[MappingConfig, str | None]:
    if not path.exists():
        return default_mapping_config(), None
    try:
        data = json.loads(path.read_text(encoding="utf-8-sig"))
        return MappingConfig.from_dict(data), None
    except Exception as exc:
        return default_mapping_config(), str(exc)


def save_mapping_config(config: MappingConfig, path: Path = DEFAULT_MAPPING_PATH) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(config.to_dict(), indent=2) + "\n", encoding="utf-8")


@dataclass
class _RuntimeState:
    active: bool = False
    pending_desired: bool | None = None
    pending_since_s: float = 0.0
    enter_blocked: bool = False
    last_fired_s: float | None = None
    last_repeat_s: float = 0.0
    last_process_s: float | None = None
    residual_x: float = 0.0
    residual_y: float = 0.0
    absolute_x: float | None = None
    absolute_y: float | None = None
    status: str = "idle"


class InputMapper:
    def __init__(
        self,
        backend: InputBackend,
        config: MappingConfig | None = None,
        on_log: Callable[[str], None] | None = None,
    ) -> None:
        self.backend = backend
        self.config = config or default_mapping_config()
        self.on_log = on_log
        self.enabled = bool(self.config.enabled_on_start)
        self._states: dict[str, _RuntimeState] = {}
        self._held_keys: dict[str, set[str]] = {}
        self._held_buttons: dict[str, set[str]] = {}
        self._rule_keys: dict[str, set[str]] = {}
        self._rule_buttons: dict[str, set[str]] = {}
        self._delta_anchors: dict[tuple[Any, ...], float] = {}
        self._gesture_last_fired: dict[str, float] = {}
        self._last_recognition_label = ""
        self._last_recognition_s = 0.0
        self._last_status = "armed" if self.enabled else "disabled"

    @property
    def last_status(self) -> str:
        if not self.backend.available:
            return self.backend.error or "input backend unavailable"
        return self._last_status

    def set_enabled(self, enabled: bool) -> None:
        enabled = bool(enabled)
        if self.enabled == enabled:
            return
        self.enabled = enabled
        self._last_status = "armed" if enabled else "disabled"
        if not enabled:
            self.release_all()

    def set_config(self, config: MappingConfig) -> None:
        self.release_all()
        self.config = config
        self.enabled = bool(config.enabled_on_start)
        self._states.clear()
        self._gesture_last_fired.clear()

    def set_active_profile(self, profile_name: str) -> bool:
        if profile_name not in self.config.profile_names():
            return False
        if self.config.active_profile != profile_name:
            self.release_all()
            self.config.active_profile = profile_name
            self._states.clear()
            self._gesture_last_fired.clear()
        return True

    def active_rules(self) -> list[MappingRule]:
        return self.config.active().mappings

    def last_recognition(self, max_age_s: float = 1.2, now_s: float | None = None) -> str:
        now_s = time.monotonic() if now_s is None else now_s
        if not self._last_recognition_label:
            return ""
        if now_s - self._last_recognition_s > max(0.0, max_age_s):
            return ""
        return self._last_recognition_label

    def process(self, snapshot: dict[str, Any], now_s: float | None = None, suppress_output: bool = False) -> None:
        now_s = time.monotonic() if now_s is None else now_s
        if suppress_output:
            if self.has_held_outputs:
                self.release_all()
            self._last_status = "suppressed"
            return
        if not self.enabled:
            if self.has_held_outputs:
                self.release_all()
            self._last_status = "disabled"
            return
        if not self.backend.available:
            if self.has_held_outputs:
                self.release_all()
            self._last_status = self.backend.error or "input backend unavailable"
            return

        signals = SignalCatalog.flatten(snapshot)
        live_rule_ids = set()
        for rule in self.active_rules():
            live_rule_ids.add(rule.id)
            state = self._states.setdefault(rule.id, _RuntimeState())
            if not rule.enabled or not rule.source:
                self._transition(rule, state, False, now_s)
                self.release_rule(rule.id)
                self._reset_delta_anchor(rule)
                state.status = "disabled" if not rule.enabled else "no source"
                continue
            condition_results = self._sync_condition_outputs(rule, signals, state)
            conditions_active, conditions_status = self._conditions_active(rule, signals, state, condition_results)
            if not conditions_active:
                state.pending_desired = None
                self._transition(rule, state, False, now_s)
                self._reset_delta_anchor(rule)
                state.status = conditions_status
                continue
            signal = signals.get(rule.source)
            if signal is None or signal.value is None:
                state.pending_desired = None
                self._transition(rule, state, False, now_s)
                self._reset_delta_anchor(rule)
                state.status = "missing"
                continue

            desired = self._desired_condition(rule, signal.value, state)
            active = self._debounced_active(rule, state, desired, now_s)
            self._transition(rule, state, active, now_s, signal.value, signals)
            state.status = "active" if state.active else "idle"

        for rule_id, state in list(self._states.items()):
            if rule_id not in live_rule_ids:
                self.release_rule(rule_id)
                state.active = False
                state.status = "removed"
        self._last_status = "armed"

    @property
    def has_held_outputs(self) -> bool:
        return bool(self._held_keys or self._held_buttons)

    def release_rule(self, rule_id: str, *, include_modifiers: bool = True) -> None:
        holder_ids = [rule_id]
        if include_modifiers:
            prefix = f"{rule_id}:modifier:"
            holder_ids.extend(
                holder_id
                for holder_id in set(self._rule_keys) | set(self._rule_buttons)
                if holder_id.startswith(prefix)
            )
        for holder_id in holder_ids:
            self._release_holder(holder_id)

    def _release_holder(self, holder_id: str) -> None:
        for token in list(self._rule_keys.get(holder_id, set())):
            holders = self._held_keys.get(token)
            if not holders:
                continue
            holders.discard(holder_id)
            if not holders:
                self.backend.release_key(token)
                self._held_keys.pop(token, None)
        for button in list(self._rule_buttons.get(holder_id, set())):
            holders = self._held_buttons.get(button)
            if not holders:
                continue
            holders.discard(holder_id)
            if not holders:
                self.backend.release_mouse(button)
                self._held_buttons.pop(button, None)
        self._rule_keys.pop(holder_id, None)
        self._rule_buttons.pop(holder_id, None)

    def release_all(self) -> None:
        for token in list(self._held_keys.keys()):
            self.backend.release_key(token)
        for button in list(self._held_buttons.keys()):
            self.backend.release_mouse(button)
        self._held_keys.clear()
        self._held_buttons.clear()
        self._rule_keys.clear()
        self._rule_buttons.clear()
        self._delta_anchors.clear()
        for state in self._states.values():
            state.active = False
            state.pending_desired = None
            state.enter_blocked = False
            state.status = "idle"

    def test_action(self, action: MappingAction) -> None:
        if not self.backend.available:
            self._log(self.backend.error or "Input backend unavailable.")
            return
        if action.type == "keyboard_hold":
            self.backend.tap_keys(action.keys)
            return
        if action.type == "mouse_hold":
            self.backend.click_mouse(action.button, 1)
            return
        self._execute_enter(MappingRule(id="__test__", name="Test action", action=action), action, time.monotonic())

    def state_for_rule(self, rule_id: str) -> _RuntimeState:
        return self._states.setdefault(rule_id, _RuntimeState())

    def _debounced_active(self, rule: MappingRule, state: _RuntimeState, desired: bool, now_s: float) -> bool:
        if desired == state.active:
            state.pending_desired = None
            return state.active
        debounce_s = 0.0 if self._gesture_name_for_rule(rule) else rule.debounce_ms / 1000.0
        if debounce_s <= 0:
            state.pending_desired = None
            return desired
        if state.pending_desired != desired:
            state.pending_desired = desired
            state.pending_since_s = now_s
            return state.active
        if now_s - state.pending_since_s >= debounce_s:
            state.pending_desired = None
            return desired
        return state.active

    def _transition(
        self,
        rule: MappingRule,
        state: _RuntimeState,
        active: bool,
        now_s: float,
        source_value: Any = None,
        signals: dict[str, SignalValue] | None = None,
    ) -> None:
        if active and not state.active:
            gesture_name = self._gesture_name_for_rule(rule)
            state.active = True
            state.last_process_s = now_s
            state.residual_x = 0.0
            state.residual_y = 0.0
            state.absolute_x = None
            state.absolute_y = None
            state.enter_blocked = bool(gesture_name and not self._can_fire_gesture(gesture_name, now_s))
            if state.enter_blocked:
                state.status = "gesture cooldown"
                return
            self._execute_enter(rule, rule.action, now_s)
        elif not active and state.active:
            state.active = False
            state.last_process_s = None
            state.enter_blocked = False
            self.release_rule(rule.id, include_modifiers=False)
            return

        if state.active and not state.enter_blocked:
            self._execute_active(rule.id, rule.action, state, now_s, source_value, signals or {})

    def _gesture_name_for_rule(self, rule: MappingRule) -> str | None:
        gesture_name = WRIST_GESTURE_SIGNAL_NAMES.get(rule.source)
        if gesture_name:
            return gesture_name
        if rule.source == "fused.wrist_motion" and rule.comparator == "eq":
            threshold = str(rule.threshold).strip().lower()
            if threshold and threshold != "none":
                return f"wrist_motion:{threshold}"
        return None

    def _can_fire_gesture(self, name: str, now_s: float) -> bool:
        last = self._gesture_last_fired.get(name)
        if last is None or now_s - last >= GESTURE_COOLDOWN_SEC:
            self._gesture_last_fired[name] = now_s
            return True
        return False

    def _execute_enter(self, rule: MappingRule, action: MappingAction, now_s: float) -> None:
        rule_id = rule.id
        if action.type == "keyboard_tap":
            self.backend.tap_keys(action.keys)
        elif action.type == "keyboard_hold":
            self._hold_keys(rule_id, action.keys)
        elif action.type == "keyboard_repeat":
            self.backend.tap_keys(action.keys)
        elif action.type == "mouse_click":
            self.backend.click_mouse(action.button, action.clicks)
        elif action.type == "mouse_hold":
            self._hold_mouse(rule_id, action.button)
        elif action.type == "mouse_scroll":
            self.backend.scroll(action.scroll_x, action.scroll_y)
        elif action.type == "mouse_absolute" and not action.continuous:
            self._move_absolute(action)
        state = self._states.setdefault(rule_id, _RuntimeState())
        state.last_fired_s = now_s
        state.last_repeat_s = now_s
        if rule.recognition_label:
            self._last_recognition_label = rule.recognition_label
            self._last_recognition_s = now_s

    def _sync_condition_outputs(
        self,
        rule: MappingRule,
        signals: dict[str, SignalValue],
        state: _RuntimeState,
    ) -> list[tuple[MappingCondition, bool, str]]:
        results: list[tuple[MappingCondition, bool, str]] = []
        for index, condition in enumerate(condition for condition in rule.all_conditions() if condition.source):
            active, status = self._condition_active(condition, signals, state)
            holder_id = self._modifier_holder_id(rule.id, index)
            if active and condition.output_keys:
                self._hold_keys(holder_id, condition.output_keys)
            else:
                self.release_rule(holder_id, include_modifiers=False)
            results.append((condition, active, status))
        return results

    @staticmethod
    def _modifier_holder_id(rule_id: str, index: int) -> str:
        return f"{rule_id}:modifier:{index}"

    def _condition_active(
        self,
        condition: MappingCondition,
        signals: dict[str, SignalValue],
        state: _RuntimeState,
    ) -> tuple[bool, str]:
        signal = signals.get(condition.source)
        if signal is None or signal.value is None:
            return False, "condition missing"
        active = _evaluate_standard_condition(
            condition.comparator,
            signal.value,
            condition.threshold,
            condition.low,
            condition.high,
            condition.hysteresis,
            state.active,
        )
        return active, "conditions active" if active else "condition idle"

    def _conditions_active(
        self,
        rule: MappingRule,
        signals: dict[str, SignalValue],
        state: _RuntimeState,
        condition_results: list[tuple[MappingCondition, bool, str]] | None = None,
    ) -> tuple[bool, str]:
        results = condition_results
        if results is None:
            results = [
                (condition, *self._condition_active(condition, signals, state))
                for condition in rule.all_conditions()
                if condition.source
            ]
        if not results:
            return True, ""
        for _condition, active, status in results:
            if not active:
                return False, status
        return True, "conditions active"

    def _desired_condition(self, rule: MappingRule, value: Any, state: _RuntimeState) -> bool:
        if rule.comparator in DELTA_COMPARATORS:
            return self._evaluate_delta_condition(rule, value)
        return evaluate_condition(rule, value, state.active)

    def _evaluate_delta_condition(self, rule: MappingRule, value: Any) -> bool:
        number = _to_number(value)
        threshold = _to_number(rule.threshold)
        if number is None or threshold is None or threshold <= 0.0:
            self._reset_delta_anchor(rule)
            return False
        key = self._delta_anchor_key(rule)
        anchor = self._delta_anchors.get(key)
        if anchor is None:
            self._delta_anchors[key] = number
            return False
        delta = number - anchor
        if rule.comparator == "delta_decrease" and delta <= -threshold:
            self._delta_anchors[key] = number
            return True
        if rule.comparator == "delta_increase" and delta >= threshold:
            self._delta_anchors[key] = number
            return True
        return False

    def _delta_anchor_key(self, rule: MappingRule) -> tuple[Any, ...]:
        conditions = tuple(
            (
                condition.source,
                condition.comparator,
                str(condition.threshold),
                str(condition.low),
                str(condition.high),
            )
            for condition in rule.all_conditions()
            if condition.source
        )
        return (
            rule.source,
            conditions,
        )

    def _reset_delta_anchor(self, rule: MappingRule) -> None:
        if rule.comparator in DELTA_COMPARATORS:
            self._delta_anchors.pop(self._delta_anchor_key(rule), None)

    def _execute_active(
        self,
        rule_id: str,
        action: MappingAction,
        state: _RuntimeState,
        now_s: float,
        source_value: Any,
        signals: dict[str, SignalValue],
    ) -> None:
        interval_s = max(0.02, action.interval_ms / 1000.0)
        if action.type == "keyboard_repeat" and now_s - state.last_repeat_s >= interval_s:
            self.backend.tap_keys(action.keys)
            state.last_fired_s = now_s
            state.last_repeat_s = now_s
        elif action.type == "mouse_scroll" and now_s - state.last_repeat_s >= interval_s:
            self.backend.scroll(action.scroll_x, action.scroll_y)
            state.last_fired_s = now_s
            state.last_repeat_s = now_s
        elif action.type == "mouse_move":
            last = state.last_process_s if state.last_process_s is not None else now_s
            dt = max(0.0, min(0.2, now_s - last))
            state.last_process_s = now_s
            self._move_relative(action, state, dt, now_s)
        elif action.type == "mouse_absolute" and action.continuous:
            if self._move_absolute(action, signals, state):
                state.last_fired_s = now_s

    def _hold_keys(self, rule_id: str, tokens: list[str]) -> None:
        for token in tokens:
            if not token:
                continue
            holders = self._held_keys.setdefault(token, set())
            if not holders:
                self.backend.press_key(token)
            holders.add(rule_id)
            self._rule_keys.setdefault(rule_id, set()).add(token)

    def _hold_mouse(self, rule_id: str, button: str) -> None:
        button = normalize_mouse_button(button)
        holders = self._held_buttons.setdefault(button, set())
        if not holders:
            self.backend.press_mouse(button)
        holders.add(rule_id)
        self._rule_buttons.setdefault(rule_id, set()).add(button)

    def _move_relative(self, action: MappingAction, state: _RuntimeState, dt: float, now_s: float) -> None:
        move_x = (action.speed_x * dt) + state.residual_x
        move_y = (action.speed_y * dt) + state.residual_y
        step_x = int(move_x)
        step_y = int(move_y)
        state.residual_x = move_x - step_x
        state.residual_y = move_y - step_y
        if step_x or step_y:
            self.backend.move(step_x, step_y)
            state.last_fired_s = now_s

    def _move_absolute(
        self,
        action: MappingAction,
        signals: dict[str, SignalValue] | None = None,
        state: _RuntimeState | None = None,
    ) -> bool:
        try:
            import tkinter as tk

            root = tk._default_root
            width = root.winfo_screenwidth() if root is not None else 1920
            height = root.winfo_screenheight() if root is not None else 1080
        except Exception:
            width, height = 1920, 1080
        absolute_x = self._absolute_axis_value(
            action.absolute_x,
            action.absolute_x_source,
            signals,
            invert=action.absolute_x_invert,
        )
        absolute_y = self._absolute_axis_value(
            action.absolute_y,
            action.absolute_y_source,
            signals,
            invert=action.absolute_y_invert,
        )
        if state is not None and action.continuous:
            previous_x = state.absolute_x
            previous_y = state.absolute_y
            if previous_x is not None and previous_y is not None:
                dx = absolute_x - previous_x
                dy = absolute_y - previous_y
                deadband = max(0.0, action.absolute_deadband)
                if abs(dx) < deadband and abs(dy) < deadband:
                    return False
                alpha = max(0.01, min(1.0, action.absolute_smoothing_alpha))
                absolute_x = previous_x + dx * alpha
                absolute_y = previous_y + dy * alpha
            state.absolute_x = absolute_x
            state.absolute_y = absolute_y
        x = int(max(0.0, min(1.0, absolute_x)) * max(1, width - 1))
        y = int(max(0.0, min(1.0, absolute_y)) * max(1, height - 1))
        self.backend.move_absolute(x, y)
        return True

    @staticmethod
    def _absolute_axis_value(
        fallback: float,
        source: str,
        signals: dict[str, SignalValue] | None,
        *,
        invert: bool = False,
    ) -> float:
        value = fallback
        if source and signals:
            signal = signals.get(source)
            number = _to_number(signal.value) if signal is not None else None
            if number is not None:
                value = number
        value = max(0.0, min(1.0, value))
        return 1.0 - value if invert else value

    def _log(self, message: str) -> None:
        if self.on_log:
            self.on_log(message)


def evaluate_condition(rule: MappingRule, value: Any, was_active: bool = False) -> bool:
    if rule.comparator in DELTA_COMPARATORS:
        return False
    return _evaluate_standard_condition(
        rule.comparator,
        value,
        rule.threshold,
        rule.low,
        rule.high,
        rule.hysteresis,
        was_active,
    )


def _evaluate_standard_condition(
    comparator: str,
    value: Any,
    threshold_value: Any,
    low_value: Any,
    high_value: Any,
    hysteresis_value: float,
    was_active: bool = False,
) -> bool:
    hysteresis = max(0.0, float(hysteresis_value or 0.0))
    if comparator == "present":
        return value is not None and value != ""
    if comparator == "truthy":
        return bool(value)
    if comparator == "falsey":
        return not bool(value)
    if comparator in {"eq", "neq"}:
        same = _compare_equal(value, threshold_value)
        return same if comparator == "eq" else not same

    number = _to_number(value)
    if number is None:
        return False
    threshold = _to_number(threshold_value)
    low = _to_number(low_value)
    high = _to_number(high_value)
    if comparator in {"lt", "lte", "gt", "gte"} and threshold is None:
        return False
    if comparator in {"between", "outside"} and (low is None or high is None):
        return False

    if comparator == "lt":
        return number <= threshold + hysteresis if was_active else number < threshold
    if comparator == "lte":
        return number <= threshold + hysteresis if was_active else number <= threshold
    if comparator == "gt":
        return number >= threshold - hysteresis if was_active else number > threshold
    if comparator == "gte":
        return number >= threshold - hysteresis if was_active else number >= threshold
    if comparator == "between":
        lo, hi = sorted((low, high))
        if was_active:
            return lo - hysteresis <= number <= hi + hysteresis
        return lo <= number <= hi
    if comparator == "outside":
        lo, hi = sorted((low, high))
        if was_active:
            return number < lo + hysteresis or number > hi - hysteresis
        return number < lo or number > hi
    return False


def _to_number(value: Any) -> float | None:
    if value is None:
        return None
    if isinstance(value, bool):
        return 1.0 if value else 0.0
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _compare_equal(left: Any, right: Any) -> bool:
    left_number = _to_number(left)
    right_number = _to_number(right)
    if left_number is not None and right_number is not None:
        return left_number == right_number
    return str(left).strip().lower() == str(right).strip().lower()
