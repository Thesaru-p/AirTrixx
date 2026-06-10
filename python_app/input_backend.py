from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Protocol


class InputBackend(Protocol):
    @property
    def available(self) -> bool:
        ...

    @property
    def error(self) -> str | None:
        ...

    def press_key(self, token: str) -> None:
        ...

    def release_key(self, token: str) -> None:
        ...

    def tap_keys(self, tokens: list[str]) -> None:
        ...

    def type_text(self, text: str) -> None:
        ...

    def press_mouse(self, button: str) -> None:
        ...

    def release_mouse(self, button: str) -> None:
        ...

    def click_mouse(self, button: str, clicks: int = 1) -> None:
        ...

    def scroll(self, dx: int = 0, dy: int = 0) -> None:
        ...

    def move(self, dx: int = 0, dy: int = 0) -> None:
        ...

    def move_absolute(self, x: int, y: int) -> None:
        ...


KEY_ALIASES: dict[str, str] = {
    "control": "ctrl",
    "ctrl_l": "ctrl_l",
    "ctrl_r": "ctrl_r",
    "command": "cmd",
    "win": "cmd",
    "windows": "cmd",
    "super": "cmd",
    "option": "alt",
    "escape": "esc",
    "return": "enter",
    "delete": "delete",
    "del": "delete",
    "pgup": "page_up",
    "pageup": "page_up",
    "pgdn": "page_down",
    "pagedown": "page_down",
    "up_arrow": "up",
    "down_arrow": "down",
    "left_arrow": "left",
    "right_arrow": "right",
    "plus": "+",
    "minus": "-",
}


BUTTON_ALIASES: dict[str, str] = {
    "primary": "left",
    "secondary": "right",
    "wheel": "middle",
    "mid": "middle",
}


def normalize_key_token(token: str) -> str:
    cleaned = " ".join(str(token).strip().split())
    if not cleaned:
        return ""
    lowered = cleaned.replace(" ", "_").replace("-", "_").lower()
    return KEY_ALIASES.get(lowered, lowered)


def normalize_mouse_button(button: str) -> str:
    cleaned = str(button).strip().lower().replace(" ", "_").replace("-", "_")
    return BUTTON_ALIASES.get(cleaned, cleaned or "left")


def parse_key_combo(value: str | list[str] | tuple[str, ...]) -> list[str]:
    if isinstance(value, (list, tuple)):
        raw_tokens = [str(item) for item in value]
    else:
        text = str(value).replace("+", ",")
        raw_tokens = text.split(",")
    tokens = [normalize_key_token(item) for item in raw_tokens]
    return [token for token in tokens if token]


@dataclass
class _PynputModules:
    keyboard: Any
    mouse: Any


class PynputInputBackend:
    def __init__(self) -> None:
        self._available = False
        self._error: str | None = None
        self._modules: _PynputModules | None = None
        self._keyboard_controller: Any = None
        self._mouse_controller: Any = None
        try:
            from pynput import keyboard, mouse

            self._modules = _PynputModules(keyboard=keyboard, mouse=mouse)
            self._keyboard_controller = keyboard.Controller()
            self._mouse_controller = mouse.Controller()
            self._available = True
        except Exception as exc:  # pragma: no cover - depends on OS permissions/display
            self._error = f"pynput input backend unavailable: {exc}"

    @property
    def available(self) -> bool:
        return self._available

    @property
    def error(self) -> str | None:
        return self._error

    def press_key(self, token: str) -> None:
        key = self._resolve_key(token)
        if key is not None:
            self._keyboard_controller.press(key)

    def release_key(self, token: str) -> None:
        key = self._resolve_key(token)
        if key is not None:
            self._keyboard_controller.release(key)

    def tap_keys(self, tokens: list[str]) -> None:
        keys = [self._resolve_key(token) for token in tokens]
        keys = [key for key in keys if key is not None]
        for key in keys:
            self._keyboard_controller.press(key)
        for key in reversed(keys):
            self._keyboard_controller.release(key)

    def type_text(self, text: str) -> None:
        self._keyboard_controller.type(str(text))

    def press_mouse(self, button: str) -> None:
        resolved = self._resolve_button(button)
        if resolved is not None:
            self._mouse_controller.press(resolved)

    def release_mouse(self, button: str) -> None:
        resolved = self._resolve_button(button)
        if resolved is not None:
            self._mouse_controller.release(resolved)

    def click_mouse(self, button: str, clicks: int = 1) -> None:
        resolved = self._resolve_button(button)
        if resolved is not None:
            self._mouse_controller.click(resolved, max(1, int(clicks)))

    def scroll(self, dx: int = 0, dy: int = 0) -> None:
        self._mouse_controller.scroll(int(dx), int(dy))

    def move(self, dx: int = 0, dy: int = 0) -> None:
        self._mouse_controller.move(int(dx), int(dy))

    def move_absolute(self, x: int, y: int) -> None:
        self._mouse_controller.position = (int(x), int(y))

    def _resolve_key(self, token: str) -> Any:
        if not self._modules:
            return None
        token = normalize_key_token(token)
        if not token:
            return None
        keyboard = self._modules.keyboard
        if len(token) == 1:
            return keyboard.KeyCode.from_char(token)
        if token.startswith("f") and token[1:].isdigit():
            key = getattr(keyboard.Key, token, None)
            if key is not None:
                return key
        named_key = getattr(keyboard.Key, token, None)
        if named_key is not None:
            return named_key
        try:
            return keyboard.KeyCode.from_char(token)
        except Exception:
            return None

    def _resolve_button(self, button: str) -> Any:
        if not self._modules:
            return None
        name = normalize_mouse_button(button)
        return getattr(self._modules.mouse.Button, name, None)


class FakeInputBackend:
    def __init__(self) -> None:
        self.events: list[tuple[Any, ...]] = []
        self._available = True
        self._error: str | None = None

    @property
    def available(self) -> bool:
        return self._available

    @property
    def error(self) -> str | None:
        return self._error

    def press_key(self, token: str) -> None:
        self.events.append(("key_down", normalize_key_token(token)))

    def release_key(self, token: str) -> None:
        self.events.append(("key_up", normalize_key_token(token)))

    def tap_keys(self, tokens: list[str]) -> None:
        normalized = [normalize_key_token(token) for token in tokens if normalize_key_token(token)]
        self.events.append(("key_tap", tuple(normalized)))

    def type_text(self, text: str) -> None:
        self.events.append(("type_text", str(text)))

    def press_mouse(self, button: str) -> None:
        self.events.append(("mouse_down", normalize_mouse_button(button)))

    def release_mouse(self, button: str) -> None:
        self.events.append(("mouse_up", normalize_mouse_button(button)))

    def click_mouse(self, button: str, clicks: int = 1) -> None:
        self.events.append(("mouse_click", normalize_mouse_button(button), max(1, int(clicks))))

    def scroll(self, dx: int = 0, dy: int = 0) -> None:
        self.events.append(("scroll", int(dx), int(dy)))

    def move(self, dx: int = 0, dy: int = 0) -> None:
        self.events.append(("move", int(dx), int(dy)))

    def move_absolute(self, x: int, y: int) -> None:
        self.events.append(("move_absolute", int(x), int(y)))
