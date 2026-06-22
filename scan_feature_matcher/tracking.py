"""Temporal stabilization for custom scan landmark detections."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import List, Sequence

from .matching import normalize_angle
from .patterns import LineFeatureDetection


@dataclass(frozen=True)
class LineFeatureTrackingConfig:
    enabled: bool = True
    confirmations_required: int = 2
    hold_frames: int = 3
    max_match_distance: float = 0.18
    max_match_yaw: float = 0.35
    max_match_length_delta: float = 0.20
    smoothing_alpha: float = 0.55


@dataclass
class _LineFeatureTrack:
    detection: LineFeatureDetection
    hits: int
    misses: int
    confirmed: bool


class LineFeatureTracker:
    """Debounce and smooth line landmark detections across scan frames."""

    def __init__(self, config: LineFeatureTrackingConfig | None = None) -> None:
        self.config = config or LineFeatureTrackingConfig()
        self._tracks: List[_LineFeatureTrack] = []

    def reset(self) -> None:
        self._tracks = []

    def update(
        self,
        detections: Sequence[LineFeatureDetection],
        config: LineFeatureTrackingConfig | None = None,
    ) -> List[LineFeatureDetection]:
        if config is not None:
            self.config = config
        if not self.config.enabled:
            self.reset()
            return list(detections)

        matched_track_indices = set()
        matched_detection_indices = set()
        existing_track_count = len(self._tracks)

        for detection_index, detection in enumerate(detections):
            track_index = self._best_track_index(detection, matched_track_indices)
            if track_index is None:
                continue
            track = self._tracks[track_index]
            track.detection = _smooth_detection(
                track.detection, detection, self.config.smoothing_alpha
            )
            track.hits += 1
            track.misses = 0
            if track.hits >= self.config.confirmations_required:
                track.confirmed = True
            matched_track_indices.add(track_index)
            matched_detection_indices.add(detection_index)

        next_tracks: List[_LineFeatureTrack] = []
        for track_index, track in enumerate(self._tracks[:existing_track_count]):
            if track_index not in matched_track_indices:
                track.misses += 1
            if track.confirmed:
                if track.misses <= self.config.hold_frames:
                    next_tracks.append(track)
            elif track.misses == 0:
                next_tracks.append(track)
        self._tracks = next_tracks

        for detection_index, detection in enumerate(detections):
            if detection_index in matched_detection_indices:
                continue
            self._tracks.append(
                _LineFeatureTrack(
                    detection=detection,
                    hits=1,
                    misses=0,
                    confirmed=self.config.confirmations_required <= 1,
                )
            )

        return [
            track.detection
            for track in self._tracks
            if track.confirmed and track.misses <= self.config.hold_frames
        ]

    def _best_track_index(
        self, detection: LineFeatureDetection, used_track_indices: set[int]
    ) -> int | None:
        best_index = None
        best_score = float("inf")
        for index, track in enumerate(self._tracks):
            if index in used_track_indices:
                continue
            score = _match_score(track.detection, detection, self.config)
            if score is None:
                continue
            if score < best_score:
                best_index = index
                best_score = score
        return best_index


def _match_score(
    previous: LineFeatureDetection,
    current: LineFeatureDetection,
    config: LineFeatureTrackingConfig,
) -> float | None:
    distance = math.hypot(
        previous.center_x - current.center_x,
        previous.center_y - current.center_y,
    )
    if distance > config.max_match_distance:
        return None

    yaw_delta = abs(normalize_angle(previous.yaw - current.yaw))
    yaw_delta = min(yaw_delta, abs(math.pi - yaw_delta))
    if yaw_delta > config.max_match_yaw:
        return None

    length_delta = abs(previous.length - current.length)
    if length_delta > config.max_match_length_delta:
        return None

    return distance + 0.2 * yaw_delta + 0.5 * length_delta


def _smooth_detection(
    previous: LineFeatureDetection,
    current: LineFeatureDetection,
    alpha: float,
) -> LineFeatureDetection:
    alpha = max(0.0, min(1.0, alpha))
    beta = 1.0 - alpha
    center_x = beta * previous.center_x + alpha * current.center_x
    center_y = beta * previous.center_y + alpha * current.center_y
    yaw = normalize_angle(
        previous.yaw
        + alpha * normalize_angle(current.yaw - previous.yaw)
    )
    length = beta * previous.length + alpha * current.length
    width = beta * previous.width + alpha * current.width
    endpoint_a = (
        center_x - math.cos(yaw) * length * 0.5,
        center_y - math.sin(yaw) * length * 0.5,
    )
    endpoint_b = (
        center_x + math.cos(yaw) * length * 0.5,
        center_y + math.sin(yaw) * length * 0.5,
    )
    return LineFeatureDetection(
        center_x=center_x,
        center_y=center_y,
        yaw=yaw,
        length=length,
        width=width,
        point_count=current.point_count,
        score=current.score,
        scan_indices=current.scan_indices,
        endpoints=(endpoint_a, endpoint_b),
    )
