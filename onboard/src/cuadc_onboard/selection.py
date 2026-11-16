from __future__ import annotations

import math
from dataclasses import dataclass
from statistics import median
from typing import Iterable


@dataclass(frozen=True)
class Detection:
    """A detector-owned spatial track; it never contains a scenario well id."""

    track_id: str
    value: int
    confidence: float
    frames: int = 1
    label: str = ""
    north_m: float = 0.0
    east_m: float = 0.0


@dataclass(frozen=True)
class Selection:
    track_id: str | None
    value: int | None
    north_m: float | None
    east_m: float | None
    reason: str
    source: str
    guessed: bool = False


def _combined_confidence(items: Iterable[Detection]) -> float:
    missed = 1.0
    for item in items:
        missed *= 1.0 - max(0.0, min(1.0, item.confidence))
    return 1.0 - missed


def aggregate_detections(
    detections: Iterable[Detection],
    cluster_radius_m: float = 8.0,
    prefer_frame_votes: bool = True,
) -> list[Detection]:
    """Fuse detections by estimated ground position and then by class.

    Track ids are generated from detector-space clusters, never from the
    scenario.  The online engine passes its complete short history on each
    call, so deterministic ordering also keeps ids stable between frames.
    """

    if cluster_radius_m <= 0.0:
        raise ValueError("cluster_radius_m must be positive")
    clusters: list[list[Detection]] = []
    for detection in detections:
        if not (math.isfinite(detection.north_m) and math.isfinite(detection.east_m)):
            continue
        best_index: int | None = None
        best_distance = math.inf
        for index, cluster in enumerate(clusters):
            total_weight = sum(max(0.01, item.confidence) for item in cluster)
            center_north = sum(
                item.north_m * max(0.01, item.confidence) for item in cluster
            ) / total_weight
            center_east = sum(
                item.east_m * max(0.01, item.confidence) for item in cluster
            ) / total_weight
            distance = math.hypot(
                detection.north_m - center_north,
                detection.east_m - center_east,
            )
            if distance <= cluster_radius_m and distance < best_distance:
                best_index = index
                best_distance = distance
        if best_index is None:
            clusters.append([detection])
        else:
            clusters[best_index].append(detection)

    fused: list[Detection] = []
    for index, cluster in enumerate(clusters, start=1):
        hypotheses: dict[int, list[Detection]] = {}
        for item in cluster:
            hypotheses.setdefault(item.value, []).append(item)
        value, winning = max(
            hypotheses.items(),
            key=lambda pair: (
                sum(max(1, item.frames) for item in pair[1]) if prefer_frame_votes else 0,
                (sum(item.confidence*max(1,item.frames) for item in pair[1]) if prefer_frame_votes
                 else max(item.confidence for item in pair[1])),
                -pair[0],
            ),
        )
        total_weight = sum(max(0.01, item.confidence) for item in winning)
        north = sum(
            item.north_m * max(0.01, item.confidence) for item in winning
        ) / total_weight
        east = sum(
            item.east_m * max(0.01, item.confidence) for item in winning
        ) / total_weight
        existing_id = next((item.track_id for item in cluster if item.track_id), "")
        fused.append(
            Detection(
                track_id=existing_id or f"T{index:02d}",
                value=value,
                confidence=_combined_confidence(winning) if prefer_frame_votes else max(item.confidence for item in winning),
                frames=sum(max(1, item.frames) for item in winning) if prefer_frame_votes else 1,
                label=next((item.label for item in winning if item.label), str(value)),
                north_m=north,
                east_m=east,
            )
        )
    return sorted(fused, key=lambda item: item.track_id)


def select_target(task: int, detections: Iterable[Detection]) -> Selection:
    candidates = aggregate_detections(detections)
    if not candidates:
        return Selection(
            None,
            None,
            None,
            None,
            "No model detection; use target-zone center",
            "NO_DETECTION",
            guessed=True,
        )
    incomplete = len(candidates) < 3
    if task == 1:
        # The published field layout contains exactly three occupied wells.
        # A fourth spatial cluster is therefore a detector false positive or a
        # brief projection split; keep the three tracks with the strongest
        # multi-frame support before applying the task's highest-value rule.
        eligible = (
            sorted(
                candidates,
                key=lambda item: (item.frames, item.confidence),
                reverse=True,
            )[:3]
            if len(candidates) >= 3
            else candidates
        )
        selected = max(
            eligible,
            key=lambda item: (
                item.value,
                item.confidence,
                item.frames,
                item.track_id,
            ),
        )
        return Selection(
            selected.track_id,
            selected.value,
            selected.north_m,
            selected.east_m,
            "Task 1 highest recognized value",
            "YOLO",
            guessed=incomplete,
        )
    if task != 2:
        raise ValueError("task must be 1 or 2")
    if len(candidates) >= 3:
        # A fourth spatial cluster is possible after a brief projection outlier;
        # the three strongest stable tracks are the occupied-target candidates.
        stable = sorted(
            candidates,
            key=lambda item: (item.frames, item.confidence),
            reverse=True,
        )[:3]
        target_value = int(median([item.value for item in stable]))
        selected = max(
            (item for item in stable if item.value == target_value),
            key=lambda item: (item.confidence, item.frames, item.track_id),
        )
        return Selection(
            selected.track_id,
            selected.value,
            selected.north_m,
            selected.east_m,
            "Task 2 median of three recognized targets",
            "YOLO",
            guessed=False,
        )
    if len(candidates) == 2:
        selected = min(
            candidates,
            key=lambda item: (
                abs(item.value - 49.5),
                -item.confidence,
                -item.frames,
                item.track_id,
            ),
        )
        reason = "Task 2 two-target guess: value nearest 49.5"
    else:
        selected = candidates[0]
        reason = "Task 2 one-target guess: use the recognized target"
    return Selection(
        selected.track_id,
        selected.value,
        selected.north_m,
        selected.east_m,
        reason,
        "YOLO",
        guessed=True,
    )
