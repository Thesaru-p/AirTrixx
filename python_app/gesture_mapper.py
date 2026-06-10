from __future__ import annotations

import json
import math
import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from fusion_state import GESTURE_SUSTAINED_MIN_MS
from input_mapper import MappingAction, MappingRule


WRIST_GESTURE_DEBOUNCE_MS = GESTURE_SUSTAINED_MIN_MS
BASELINE_NOISE_PERCENTILE = 0.95
ACTIVE_REGION_START_FRAC = 0.15
ACTIVE_REGION_END_FRAC = 0.85
MIN_CONFIDENCE = 0.25
MIN_BASELINE_FRAMES = 20
MIN_KEY_FRAME_FRACTION = 0.10

MIN_STEP_AMPLITUDE_BY_SUFFIX: dict[str, float] = {
    "_mm": 2.0,
    "_deg": 0.3,
    "_dps": 2.0,
}
DEFAULT_MIN_STEP_AMPLITUDE = 0.5

EXCLUDED_KEY_PATTERNS = (
    re.compile(r"_gesture$"),
    re.compile(r"_input$"),
    re.compile(r"battery"),
    re.compile(r"^wrist_motion$"),
    re.compile(r"^wrist_dominant_axis$"),
    re.compile(r"_profile$"),
    re.compile(r"_cooldown_active$"),
    re.compile(r"_blocked$"),
    re.compile(r"_pulse_active$"),
    re.compile(r"_candidate_active$"),
)

REP_FILE_PATTERN = re.compile(r"^.+_rep_\d+\.json$", re.IGNORECASE)


@dataclass
class RepSeries:
    path: Path | None
    gesture_name: str
    repetition_index: int
    field_order: list[str]
    frames: list[dict[str, Any]]
    duration_s: float


@dataclass
class TriggerCandidate:
    source: str
    field_key: str
    comparator: str
    threshold: float | bool
    confidence: float
    signal_kind: str
    direction: str
    baseline_noise: float | None = None
    gesture_amplitude: float | None = None
    rationale: str = ""
    debounce_ms: int = WRIST_GESTURE_DEBOUNCE_MS


@dataclass
class AnalysisResult:
    candidates: list[TriggerCandidate] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)
    baseline_frame_count: int = 0
    target_frame_count: int = 0
    baseline_rep_count: int = 0
    target_rep_count: int = 0


def _median(values: list[float]) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    middle = len(ordered) // 2
    if len(ordered) % 2:
        return ordered[middle]
    return (ordered[middle - 1] + ordered[middle]) / 2.0


def _percentile(values: list[float], fraction: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    position = (len(ordered) - 1) * max(0.0, min(1.0, fraction))
    low = int(position)
    high = min(len(ordered) - 1, low + 1)
    if low == high:
        return ordered[low]
    return ordered[low] + (ordered[high] - ordered[low]) * (position - low)


def _to_number(value: Any) -> float | None:
    if isinstance(value, bool) or value is None:
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(number):
        return None
    return number


def _is_truthy(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)) and not isinstance(value, bool):
        return value != 0
    if isinstance(value, str):
        return value.strip().lower() in {"1", "true", "yes", "on"}
    return False


def _round_threshold(value: float) -> float:
    if value >= 100:
        return round(value, 1)
    if value >= 10:
        return round(value, 2)
    return round(value, 3)


def _min_step_amplitude_for_key(key: str) -> float:
    for suffix, minimum in MIN_STEP_AMPLITUDE_BY_SUFFIX.items():
        if key.endswith(suffix):
            return minimum
    if key.startswith("wrist_gyro") or key.endswith("_dps"):
        return MIN_STEP_AMPLITUDE_BY_SUFFIX["_dps"]
    if key.endswith("_mm") or "z_mm" in key:
        return MIN_STEP_AMPLITUDE_BY_SUFFIX["_mm"]
    if key in {"wrist_pitch", "wrist_roll"}:
        return MIN_STEP_AMPLITUDE_BY_SUFFIX["_deg"]
    return DEFAULT_MIN_STEP_AMPLITUDE


def _is_excluded_key(key: str) -> bool:
    return any(pattern.search(key) for pattern in EXCLUDED_KEY_PATTERNS)


def _classify_key(key: str, sample_values: list[Any]) -> str | None:
    if _is_excluded_key(key):
        return None
    if key.endswith("_detected"):
        return "boolean"
    if key.endswith("_delta") or key.endswith("_abs_delta"):
        return "window_delta"
    if any(_is_truthy(value) or value is False for value in sample_values if isinstance(value, bool)):
        return "boolean"
    if any(_to_number(value) is not None for value in sample_values):
        return "anchor_delta"
    return None


def list_gesture_folders(root: Path) -> list[str]:
    root = Path(root)
    if not root.is_dir():
        return []
    folders: list[str] = []
    for child in sorted(root.iterdir()):
        if not child.is_dir():
            continue
        if any(REP_FILE_PATTERN.match(path.name) for path in child.glob("*.json")):
            folders.append(child.name)
    return folders


def load_rep(path: Path) -> RepSeries:
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError(f"gesture rep must be an object: {path}")
    samples = data.get("samples", [])
    frames: list[dict[str, Any]] = []
    max_t_rel = 0.0
    if isinstance(samples, list):
        for sample in samples:
            if not isinstance(sample, dict):
                continue
            frame = sample.get("input_dict", {})
            if isinstance(frame, dict):
                frames.append(dict(frame))
            try:
                max_t_rel = max(max_t_rel, float(sample.get("t_rel", 0.0) or 0.0))
            except (TypeError, ValueError):
                pass
    duration_s = max(max_t_rel, 0.001)
    field_order = data.get("field_order", [])
    if not isinstance(field_order, list):
        field_order = []
    return RepSeries(
        path=Path(path),
        gesture_name=str(data.get("gesture_name") or Path(path).parent.name),
        repetition_index=int(data.get("repetition_index") or 0),
        field_order=[str(item) for item in field_order],
        frames=frames,
        duration_s=duration_s,
    )


def load_gesture_dir(directory: Path) -> list[RepSeries]:
    directory = Path(directory)
    reps: list[RepSeries] = []
    for path in sorted(directory.glob("*.json")):
        if not REP_FILE_PATTERN.match(path.name):
            continue
        reps.append(load_rep(path))
    return reps


def _iter_frames(reps: list[RepSeries], *, active_region_only: bool) -> list[dict[str, Any]]:
    frames: list[dict[str, Any]] = []
    for rep in reps:
        start_s = rep.duration_s * ACTIVE_REGION_START_FRAC
        end_s = rep.duration_s * ACTIVE_REGION_END_FRAC
        for index, frame in enumerate(rep.frames):
            if not active_region_only:
                frames.append(frame)
                continue
            if not rep.frames:
                continue
            t_rel = rep.duration_s * (index / max(1, len(rep.frames) - 1))
            if start_s <= t_rel <= end_s:
                frames.append(frame)
    return frames


def _collect_keys(baseline_frames: list[dict[str, Any]], target_frames: list[dict[str, Any]]) -> list[str]:
    counts: dict[str, int] = {}
    total = len(baseline_frames) + len(target_frames)
    if total == 0:
        return []
    for frame in baseline_frames + target_frames:
        for key, value in frame.items():
            if value is None:
                continue
            counts[key] = counts.get(key, 0) + 1
    minimum = max(1, int(total * MIN_KEY_FRAME_FRACTION))
    return sorted(key for key, count in counts.items() if count >= minimum)


def _values_for_key(frames: list[dict[str, Any]], key: str) -> list[float]:
    values: list[float] = []
    for frame in frames:
        number = _to_number(frame.get(key))
        if number is not None:
            values.append(number)
    return values


def _truthy_rate(frames: list[dict[str, Any]], key: str) -> float:
    if not frames:
        return 0.0
    true_count = sum(1 for frame in frames if _is_truthy(frame.get(key)))
    return true_count / len(frames)


def _signed_steps(values: list[float]) -> list[float]:
    return [values[index + 1] - values[index] for index in range(len(values) - 1)]


def _parent_field_key(key: str) -> str | None:
    if key.endswith("_abs_delta"):
        return None
    if key.endswith("_delta"):
        return key[: -len("_delta")]
    return None


def _values_for_rep(rep: RepSeries, key: str, *, active_region_only: bool) -> list[float]:
    start_s = rep.duration_s * ACTIVE_REGION_START_FRAC
    end_s = rep.duration_s * ACTIVE_REGION_END_FRAC
    values: list[float] = []
    for index, frame in enumerate(rep.frames):
        if active_region_only:
            t_rel = rep.duration_s * (index / max(1, len(rep.frames) - 1))
            if t_rel < start_s or t_rel > end_s:
                continue
        number = _to_number(frame.get(key))
        if number is not None:
            values.append(number)
    return values


def _steps_for_rep(rep: RepSeries, key: str, *, active_region_only: bool) -> list[float]:
    return _signed_steps(_values_for_rep(rep, key, active_region_only=active_region_only))


def _directional_noise(steps: list[float], direction: str) -> float:
    if direction == "increase":
        directed = [step for step in steps if step > 0]
    else:
        directed = [-step for step in steps if step < 0]
    return _percentile(directed, BASELINE_NOISE_PERCENTILE) or 0.0


def _baseline_directional_noise(
    reps: list[RepSeries],
    key: str,
    direction: str,
) -> float:
    steps: list[float] = []
    for rep in reps:
        steps.extend(_steps_for_rep(rep, key, active_region_only=False))
    return _directional_noise(steps, direction)


def _gesture_directional_amplitude(
    reps: list[RepSeries],
    key: str,
    direction: str,
    *,
    active_region_only: bool,
) -> list[float]:
    amplitudes: list[float] = []
    for rep in reps:
        steps = _steps_for_rep(rep, key, active_region_only=active_region_only)
        if not steps:
            continue
        if direction == "increase":
            positive = [step for step in steps if step > 0]
            if positive:
                amplitudes.append(max(positive))
        else:
            negative = [-step for step in steps if step < 0]
            if negative:
                amplitudes.append(max(negative))
    return amplitudes


def _signed_values_for_rep(rep: RepSeries, key: str, *, active_region_only: bool) -> list[float]:
    return _values_for_rep(rep, key, active_region_only=active_region_only)


def _baseline_signed_value_noise(
    frames: list[dict[str, Any]],
    key: str,
    direction: str,
) -> float:
    values = _values_for_key(frames, key)
    if direction == "increase":
        directed = [value for value in values if value > 0]
    else:
        directed = [-value for value in values if value < 0]
    return _percentile(directed, BASELINE_NOISE_PERCENTILE) or 0.0


def _gesture_signed_value_amplitude(
    reps: list[RepSeries],
    key: str,
    direction: str,
    *,
    active_region_only: bool,
) -> list[float]:
    amplitudes: list[float] = []
    for rep in reps:
        values = _signed_values_for_rep(rep, key, active_region_only=active_region_only)
        if not values:
            continue
        peak = _percentile(values, BASELINE_NOISE_PERCENTILE)
        if peak is None:
            continue
        if direction == "increase" and peak > 0:
            amplitudes.append(peak)
        elif direction == "decrease" and peak < 0:
            amplitudes.append(-peak)
    return amplitudes


def _emit_directional_step_candidate(
    candidates: list[TriggerCandidate],
    *,
    key: str,
    baseline_reps: list[RepSeries],
    target_reps: list[RepSeries],
    direction: str,
    comparator: str,
    signal_kind: str,
) -> None:
    noise = _baseline_directional_noise(baseline_reps, key, direction)
    per_rep_amplitudes = _gesture_directional_amplitude(
        target_reps,
        key,
        direction,
        active_region_only=True,
    )
    gesture_amplitude = _median(per_rep_amplitudes) or 0.0
    separation = gesture_amplitude - noise
    min_amplitude = _min_step_amplitude_for_key(key)
    if separation <= 0 or gesture_amplitude < min_amplitude:
        return
    threshold = _round_threshold((noise + gesture_amplitude) / 2.0)
    confidence = _confidence(separation, gesture_amplitude, noise, per_rep_amplitudes)
    if confidence < MIN_CONFIDENCE:
        return
    sign = "+" if direction == "increase" else "-"
    candidates.append(
        TriggerCandidate(
            source=f"fused.{key}",
            field_key=key,
            comparator=comparator,
            threshold=max(threshold, noise + 1e-3),
            confidence=confidence,
            signal_kind=signal_kind,
            direction=direction,
            baseline_noise=noise,
            gesture_amplitude=gesture_amplitude,
            rationale=(
                f"{key} step {sign}{_round_threshold(gesture_amplitude)} "
                f"(pitch2-pitch1) vs baseline {sign}noise ~{_round_threshold(noise)}"
            ),
        )
    )


def _confidence(
    separation: float,
    gesture_amplitude: float,
    baseline_noise: float,
    per_rep_values: list[float],
) -> float:
    confidence = separation / (gesture_amplitude + baseline_noise + 1e-6)
    if len(per_rep_values) >= 2:
        mean_value = sum(per_rep_values) / len(per_rep_values)
        if mean_value > 0:
            variance = sum((value - mean_value) ** 2 for value in per_rep_values) / len(per_rep_values)
            std_dev = math.sqrt(variance)
            if std_dev / mean_value > 0.5:
                confidence *= 0.7
    return max(0.0, min(1.0, confidence))


def _dedupe_candidates(candidates: list[TriggerCandidate]) -> list[TriggerCandidate]:
    best_by_field_direction: dict[tuple[str, str], TriggerCandidate] = {}
    for candidate in candidates:
        dedupe_key = (candidate.field_key, candidate.direction)
        current = best_by_field_direction.get(dedupe_key)
        if current is None or candidate.confidence > current.confidence:
            best_by_field_direction[dedupe_key] = candidate
    return sorted(best_by_field_direction.values(), key=lambda item: item.confidence, reverse=True)


def analyze(
    baseline_reps: list[RepSeries],
    target_reps: list[RepSeries],
    *,
    max_candidates: int = 8,
) -> AnalysisResult:
    result = AnalysisResult(
        baseline_rep_count=len(baseline_reps),
        target_rep_count=len(target_reps),
    )
    if not baseline_reps:
        result.warnings.append("Baseline folder has no gesture rep files.")
        return result
    if not target_reps:
        result.warnings.append("Gesture folder has no gesture rep files.")
        return result

    baseline_frames = _iter_frames(baseline_reps, active_region_only=False)
    target_frames = _iter_frames(target_reps, active_region_only=True)
    result.baseline_frame_count = len(baseline_frames)
    result.target_frame_count = len(target_frames)

    if result.baseline_frame_count < MIN_BASELINE_FRAMES:
        result.warnings.append(
            f"Baseline has only {result.baseline_frame_count} frames; "
            f"record at least {MIN_BASELINE_FRAMES} for reliable analysis."
        )

    keys = _collect_keys(baseline_frames, target_frames)
    key_set = set(keys)
    candidates: list[TriggerCandidate] = []
    processed_anchor_keys: set[str] = set()

    for key in keys:
        sample_values = [frame.get(key) for frame in baseline_frames + target_frames if key in frame]
        kind = _classify_key(key, sample_values)
        if kind is None:
            continue

        if kind == "boolean":
            baseline_rate = _truthy_rate(baseline_frames, key)
            target_rate = _truthy_rate(target_frames, key)
            separation = target_rate - baseline_rate
            if target_rate < 0.25 or separation < 0.35:
                continue
            candidates.append(
                TriggerCandidate(
                    source=f"fused.{key}",
                    field_key=key,
                    comparator="truthy",
                    threshold=True,
                    confidence=max(0.0, min(1.0, separation)),
                    signal_kind="boolean",
                    direction="true",
                    baseline_noise=baseline_rate,
                    gesture_amplitude=target_rate,
                    rationale=(
                        f"{key} true in {target_rate * 100:.0f}% of gesture frames "
                        f"vs {baseline_rate * 100:.0f}% baseline"
                    ),
                )
            )
            continue

        if kind == "window_delta":
            parent_key = _parent_field_key(key)
            if parent_key is None:
                continue
            if parent_key in key_set:
                continue

            for direction, comparator in (("increase", "gt"), ("decrease", "lt")):
                noise = _baseline_signed_value_noise(baseline_frames, key, direction)
                per_rep_amplitudes = _gesture_signed_value_amplitude(
                    target_reps,
                    key,
                    direction,
                    active_region_only=True,
                )
                gesture_amplitude = _median(per_rep_amplitudes) or 0.0
                separation = gesture_amplitude - noise
                min_amplitude = _min_step_amplitude_for_key(key)
                if separation <= 0 or gesture_amplitude < min_amplitude:
                    continue
                midpoint = _round_threshold((noise + gesture_amplitude) / 2.0)
                threshold = midpoint if direction == "increase" else -midpoint
                confidence = _confidence(separation, gesture_amplitude, noise, per_rep_amplitudes)
                if confidence < MIN_CONFIDENCE:
                    continue
                sign = "+" if direction == "increase" else "-"
                candidates.append(
                    TriggerCandidate(
                        source=f"fused.{key}",
                        field_key=key,
                        comparator=comparator,
                        threshold=threshold if direction == "increase" else min(threshold, -noise - 1e-3),
                        confidence=confidence,
                        signal_kind="signed_window_delta",
                        direction=direction,
                        baseline_noise=noise,
                        gesture_amplitude=gesture_amplitude,
                        rationale=(
                            f"{key} signed delta {sign}{_round_threshold(gesture_amplitude)} "
                            f"vs baseline {sign}noise ~{_round_threshold(noise)}"
                        ),
                    )
                )
            continue

        if key in processed_anchor_keys:
            continue
        for direction, comparator in (("increase", "delta_increase"), ("decrease", "delta_decrease")):
            _emit_directional_step_candidate(
                candidates,
                key=key,
                baseline_reps=baseline_reps,
                target_reps=target_reps,
                direction=direction,
                comparator=comparator,
                signal_kind="anchor_delta",
            )
        processed_anchor_keys.add(key)

    ranked = _dedupe_candidates(candidates)
    result.candidates = ranked[:max_candidates]
    if not result.candidates:
        result.warnings.append("No discriminating signals found; try a longer gesture or cleaner baseline.")
    return result


def candidate_to_rule(candidate: TriggerCandidate, *, gesture_name: str = "") -> MappingRule:
    label = gesture_name or candidate.field_key
    return MappingRule(
        name=f"Auto: {label}",
        source=candidate.source,
        comparator=candidate.comparator,
        threshold=candidate.threshold,
        debounce_ms=candidate.debounce_ms,
        recognition_label=label,
        action=MappingAction(type="keyboard_tap", keys=["space"]),
    )
