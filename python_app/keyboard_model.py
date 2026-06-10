from __future__ import annotations

import csv
import json
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any, Callable

import numpy as np


EXPECTED_BOARD_COLUMNS = [
    "frame",
    "ms",
    "label",
    "ch1_raw",
    "ch2_raw",
    "ch0_raw",
    "ch3_raw",
    "ch1_mm",
    "ch2_mm",
    "ch0_mm",
    "ch3_mm",
]
RAW_CHANNELS = (1, 2, 0, 3)
RAW_COLUMNS = ("ch1_raw", "ch2_raw", "ch0_raw", "ch3_raw")
MM_COLUMNS = ("ch1_mm", "ch2_mm", "ch0_mm", "ch3_mm")
RAW_TRIGGER_MM = 35.0
MAX_DETECT_MM = 210.0
OUTPUT_COLUMNS = [
    "sample_id",
    "word",
    "repetition",
    "host_time_utc",
    *EXPECTED_BOARD_COLUMNS,
]


def normalize_max_detect_mm(max_detect_mm: float | list[float] | tuple[float, ...]) -> np.ndarray:
    if isinstance(max_detect_mm, (int, float)):
        return np.repeat(float(max_detect_mm), len(RAW_COLUMNS))
    values = np.array([float(value) for value in max_detect_mm], dtype=np.float32)
    if values.shape != (len(RAW_COLUMNS),):
        raise ValueError(f"Expected {len(RAW_COLUMNS)} max-detect distances, got {len(values)}")
    return values


def format_max_detect_mm(max_detect_mm: float | list[float] | tuple[float, ...]) -> str:
    values = normalize_max_detect_mm(max_detect_mm)
    return ", ".join(f"CH{channel}={value:.0f}mm" for channel, value in zip(RAW_CHANNELS, values))


def parse_detect_limits(line: str) -> list[float] | None:
    if not line.startswith("DETECT_LIMITS_MM"):
        return None

    limits_by_channel: dict[int, float] = {}
    for part in line.split(",")[1:]:
        name, separator, value = part.strip().partition("=")
        if not separator or not name.lower().startswith("ch"):
            continue
        try:
            limits_by_channel[int(name[2:])] = float(value)
        except ValueError:
            continue

    if all(channel in limits_by_channel for channel in RAW_CHANNELS):
        return [limits_by_channel[channel] for channel in RAW_CHANNELS]
    return None


def metadata_path_for(dataset_path: Path) -> Path:
    return dataset_path.with_suffix(dataset_path.suffix + ".meta.json")


def write_collection_metadata(
    dataset_path: Path,
    max_detect_mm: float | list[float] | tuple[float, ...],
) -> None:
    metadata_path = metadata_path_for(dataset_path)
    metadata_path.parent.mkdir(parents=True, exist_ok=True)
    metadata_path.write_text(
        json.dumps(
            {
                "raw_channels": list(RAW_CHANNELS),
                "max_detect_mm_by_channel": normalize_max_detect_mm(max_detect_mm).tolist(),
            },
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )


def load_max_detect_mm(dataset_path: Path, fallback: float | None = None) -> float | list[float]:
    metadata_path = metadata_path_for(dataset_path)
    if metadata_path.exists():
        metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
        values = metadata.get("max_detect_mm_by_channel")
        if values:
            return [float(value) for value in values]
    return float(fallback if fallback is not None else MAX_DETECT_MM)


def parse_board_row(line: str) -> dict[str, str] | None:
    parts = line.split(",")
    if len(parts) != len(EXPECTED_BOARD_COLUMNS) or not parts[0].isdigit():
        return None
    return dict(zip(EXPECTED_BOARD_COLUMNS, parts))


class RawFingerDetector:
    """Detect one swipe from drops in raw distance readings."""

    def __init__(
        self,
        trigger_mm: float = RAW_TRIGGER_MM,
        max_detect_mm: float | list[float] | tuple[float, ...] = MAX_DETECT_MM,
    ) -> None:
        self.trigger_mm = trigger_mm
        self.max_detect_mm = normalize_max_detect_mm(max_detect_mm)
        self.baseline: list[float | None] = [None] * len(RAW_COLUMNS)

    def update(self, row: dict[str, str]) -> bool:
        readings = [float(row[column]) for column in RAW_COLUMNS]
        for index, reading in enumerate(readings):
            if reading >= 0 and self.baseline[index] is None:
                self.baseline[index] = reading

        active = any(
            reading >= 0
            and reading <= self.max_detect_mm[index]
            and baseline is not None
            and baseline - reading >= self.trigger_mm
            for index, (reading, baseline) in enumerate(zip(readings, self.baseline))
        )

        if not active:
            for index, reading in enumerate(readings):
                if reading < 0:
                    continue
                baseline = self.baseline[index]
                self.baseline[index] = reading if baseline is None else baseline * 0.95 + reading * 0.05
        return active


def sample_frame_counts(path: Path) -> Counter[str]:
    if not path.exists():
        return Counter()
    with path.open("r", newline="", encoding="utf-8") as file:
        return Counter(row["sample_id"] for row in csv.DictReader(file) if row.get("sample_id"))


def append_sample_rows(path: Path, rows: list[dict[str, str | int]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    write_header = not path.exists() or path.stat().st_size == 0
    with path.open("a", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=OUTPUT_COLUMNS)
        if write_header:
            writer.writeheader()
        writer.writerows(rows)


def remove_sample_rows(path: Path, sample_id: str) -> None:
    if not path.exists():
        return

    temp_path = path.with_suffix(path.suffix + ".tmp")
    with path.open("r", newline="", encoding="utf-8") as source:
        reader = csv.DictReader(source)
        with temp_path.open("w", newline="", encoding="utf-8") as destination:
            writer = csv.DictWriter(destination, fieldnames=OUTPUT_COLUMNS)
            writer.writeheader()
            for row in reader:
                if row.get("sample_id") != sample_id:
                    writer.writerow(row)
    temp_path.replace(path)


def load_samples(path: Path) -> list[dict[str, Any]]:
    grouped: defaultdict[str, list[dict[str, str]]] = defaultdict(list)
    with path.open("r", newline="", encoding="utf-8") as file:
        for row in csv.DictReader(file):
            grouped[row["sample_id"]].append(row)

    samples = []
    for sample_id, rows in grouped.items():
        samples.append(
            {
                "sample_id": sample_id,
                "word": rows[0]["word"],
                "repetition": int(rows[0]["repetition"]),
                "rows": rows,
            }
        )
    return sorted(samples, key=lambda sample: sample["sample_id"])


def _resample(values: np.ndarray, points: int) -> np.ndarray:
    if len(values) == 1:
        return np.repeat(values, points, axis=0)
    old_x = np.linspace(0.0, 1.0, len(values))
    new_x = np.linspace(0.0, 1.0, points)
    return np.column_stack([np.interp(new_x, old_x, values[:, channel]) for channel in range(values.shape[1])])


def extract_feature(
    rows: list[dict[str, str]],
    points: int = 48,
    representation: str = "both",
    max_detect_mm: float | list[float] | tuple[float, ...] = MAX_DETECT_MM,
) -> np.ndarray:
    mm = np.array([[float(row[column]) for column in MM_COLUMNS] for row in rows])
    raw = np.array([[float(row[column]) for column in RAW_COLUMNS] for row in rows])
    timestamps = np.array([float(row["ms"]) for row in rows])
    max_detect = normalize_max_detect_mm(max_detect_mm)

    raw_valid = raw >= 0
    raw_baseline = np.array(
        [
            np.percentile(raw[raw_valid[:, channel], channel], 90)
            if raw_valid[:, channel].any()
            else 0.0
            for channel in range(raw.shape[1])
        ]
    )
    detectable_raw = raw_valid & (raw <= max_detect)
    raw_delta_mm = np.where(detectable_raw, np.maximum(raw_baseline - raw, 0.0), 0.0)
    active_channels = raw_delta_mm >= RAW_TRIGGER_MM
    active_rows = active_channels.any(axis=1)
    active_indexes = np.flatnonzero(active_rows)
    if not len(active_indexes):
        raise ValueError("Swipe contains no active sensor readings")

    start, end = active_indexes[0], active_indexes[-1] + 1
    mm_crop = mm[start:end]
    raw_crop = raw[start:end]
    active_crop = active_channels[start:end].astype(float)

    detectable_crop = (raw_crop >= 0) & (raw_crop <= max_detect)
    raw_delta = np.where(detectable_crop, np.clip(raw_baseline - raw_crop, 0.0, 700.0), 0.0) / 700.0
    mm_filled = np.where(mm_crop < 0, 260.0, np.clip(mm_crop, 0.0, 260.0)) / 260.0

    channels = [active_crop]
    if representation in ("mm", "both"):
        channels.append(mm_filled)
    if representation in ("raw", "both"):
        channels.append(raw_delta)
    sequence = np.concatenate(channels, axis=1)
    resampled = _resample(sequence, points).T.reshape(-1)

    duration_seconds = max(0.0, timestamps[end - 1] - timestamps[start]) / 1000.0
    transitions = np.abs(np.diff(active_crop, axis=0)).sum(axis=0) if len(active_crop) > 1 else np.zeros(4)
    summary_parts = [
        np.array([duration_seconds, len(active_crop) / 100.0]),
        active_crop.mean(axis=0),
        transitions / max(1, len(active_crop) - 1),
    ]
    if representation in ("mm", "both"):
        summary_parts.extend([mm_filled.min(axis=0), mm_filled.mean(axis=0)])
    if representation in ("raw", "both"):
        summary_parts.extend([raw_delta.max(axis=0), raw_delta.mean(axis=0)])
    summary = np.concatenate(summary_parts)
    return np.concatenate([resampled, summary]).astype(np.float32)


def build_features(
    samples: list[dict[str, Any]],
    points: int,
    representation: str,
    max_detect_mm: float | list[float] | tuple[float, ...] = MAX_DETECT_MM,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    features = [
        extract_feature(
            sample["rows"],
            points=points,
            representation=representation,
            max_detect_mm=max_detect_mm,
        )
        for sample in samples
    ]
    words = np.array([sample["word"] for sample in samples])
    repetitions = np.array([sample["repetition"] for sample in samples])
    return np.vstack(features), words, repetitions


def standardize_fit(features: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    mean = features.mean(axis=0)
    std = features.std(axis=0)
    std[std < 1e-6] = 1.0
    return (features - mean) / std, mean, std


def predict_knn(
    train_features: np.ndarray,
    train_words: np.ndarray,
    query_features: np.ndarray,
    k: int,
    metric: str = "euclidean",
) -> tuple[str, float, list[tuple[str, float]]]:
    if metric == "euclidean":
        distances = np.sqrt(np.mean((train_features - query_features) ** 2, axis=1))
    elif metric == "manhattan":
        distances = np.mean(np.abs(train_features - query_features), axis=1)
    elif metric == "cosine":
        query_norm = np.linalg.norm(query_features)
        train_norms = np.linalg.norm(train_features, axis=1)
        similarities = (train_features @ query_features) / np.maximum(train_norms * query_norm, 1e-6)
        distances = 1.0 - similarities
    else:
        raise ValueError(f"Unsupported distance metric: {metric}")
    nearest = np.argsort(distances)[:k]

    scores: defaultdict[str, float] = defaultdict(float)
    for index in nearest:
        scores[str(train_words[index])] += 1.0 / (float(distances[index]) + 1e-6)

    predicted = max(scores.items(), key=lambda item: item[1])[0]
    class_scores = {}
    for word in sorted(set(str(word) for word in train_words)):
        class_distance = float(np.min(distances[train_words == word]))
        class_scores[word] = 1.0 / (class_distance + 1e-6)
    total = sum(class_scores.values())
    ranked = sorted(
        ((word, score / total) for word, score in class_scores.items()),
        key=lambda item: item[1],
        reverse=True,
    )
    confidence = dict(ranked)[predicted]
    return predicted, confidence, ranked


def save_model(
    path: Path,
    features: np.ndarray,
    words: np.ndarray,
    mean: np.ndarray,
    std: np.ndarray,
    points: int,
    representation: str,
    k: int,
    metric: str,
    max_detect_mm: float | list[float] | tuple[float, ...] = MAX_DETECT_MM,
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        path,
        features=features,
        words=words,
        mean=mean,
        std=std,
        points=np.array(points),
        representation=np.array(representation),
        k=np.array(k),
        metric=np.array(metric),
        max_detect_mm=normalize_max_detect_mm(max_detect_mm),
    )


def load_model(path: Path) -> dict[str, Any]:
    model = np.load(path)
    return {
        "features": model["features"],
        "words": model["words"],
        "mean": model["mean"],
        "std": model["std"],
        "points": int(model["points"]),
        "representation": str(model["representation"]),
        "k": int(model["k"]),
        "metric": str(model["metric"]) if "metric" in model else "euclidean",
        "max_detect_mm": (
            model["max_detect_mm"].tolist()
            if "max_detect_mm" in model
            else [MAX_DETECT_MM] * len(RAW_COLUMNS)
        ),
    }


def _evaluate(
    features: np.ndarray,
    words: np.ndarray,
    repetitions: np.ndarray,
    k: int,
    metric: str,
) -> tuple[float, list[tuple[str, str]]]:
    predictions: list[tuple[str, str]] = []
    for repetition in sorted(set(repetitions)):
        test_mask = repetitions == repetition
        train_mask = ~test_mask
        if not np.any(train_mask) or not np.any(test_mask):
            continue
        train_standard, mean, std = standardize_fit(features[train_mask])
        test_standard = (features[test_mask] - mean) / std
        for query, actual in zip(test_standard, words[test_mask]):
            predicted, _, _ = predict_knn(train_standard, words[train_mask], query, k, metric)
            predictions.append((str(actual), predicted))

    if not predictions:
        return 0.0, []
    correct = sum(actual == predicted for actual, predicted in predictions)
    return correct / len(predictions), predictions


def _filter_valid_samples(
    samples: list[dict[str, Any]],
    max_detect_mm: float | list[float],
    log: Callable[[str], None] | None,
) -> list[dict[str, Any]]:
    valid_samples: list[dict[str, Any]] = []
    skipped: list[tuple[str, str]] = []
    for sample in samples:
        try:
            extract_feature(sample["rows"], points=48, representation="raw", max_detect_mm=max_detect_mm)
        except ValueError as error:
            skipped.append((sample["sample_id"], str(error)))
            continue
        valid_samples.append(sample)

    if skipped and log:
        log("Skipped invalid keyboard samples:")
        for sample_id, reason in skipped[:12]:
            log(f"  {sample_id}: {reason}")
        if len(skipped) > 12:
            log(f"  ...and {len(skipped) - 12} more")
    return valid_samples


def train_model(
    dataset_path: Path,
    model_path: Path,
    *,
    fallback_max_detect_mm: float | None = None,
    log: Callable[[str], None] | None = None,
) -> dict[str, Any]:
    samples = load_samples(dataset_path)
    if not samples:
        raise ValueError("No keyboard samples found.")
    max_detect_mm = load_max_detect_mm(dataset_path, fallback_max_detect_mm)
    if log:
        log(f"Using detect limits: {format_max_detect_mm(max_detect_mm)}")
    samples = _filter_valid_samples(samples, max_detect_mm, log)
    if len(samples) < 2:
        raise ValueError("Not enough valid samples found.")

    best: dict[str, Any] | None = None
    if log:
        log(f"Loaded {len(samples)} keyboard samples.")
        log("Training keyboard model configurations...")
    for representation in ("active", "raw"):
        for points in (16, 24, 32, 48, 64, 96):
            features, words, repetitions = build_features(samples, points, representation, max_detect_mm=max_detect_mm)
            for metric in ("euclidean", "manhattan", "cosine"):
                for k in (1, 3, 5):
                    accuracy, predictions = _evaluate(features, words, repetitions, k, metric)
                    candidate = {
                        "accuracy": accuracy,
                        "representation": representation,
                        "points": points,
                        "k": k,
                        "metric": metric,
                        "features": features,
                        "words": words,
                        "predictions": predictions,
                    }
                    if best is None or accuracy > best["accuracy"]:
                        best = candidate

    if best is None:
        raise ValueError("Could not train a keyboard model.")

    standardized, mean, std = standardize_fit(best["features"])
    save_model(
        model_path,
        standardized,
        best["words"],
        mean,
        std,
        best["points"],
        best["representation"],
        best["k"],
        best["metric"],
        max_detect_mm=max_detect_mm,
    )
    labels = sorted({str(word) for word in best["words"]})
    if log:
        log(
            "Selected model: "
            f"rep={best['representation']}, points={best['points']}, "
            f"k={best['k']}, metric={best['metric']}, "
            f"accuracy={best['accuracy'] * 100:.1f}%"
        )
        log(f"Saved keyboard model: {model_path}")
    return {
        "accuracy": float(best["accuracy"]),
        "representation": best["representation"],
        "points": int(best["points"]),
        "k": int(best["k"]),
        "metric": best["metric"],
        "labels": labels,
        "model_path": model_path,
    }
