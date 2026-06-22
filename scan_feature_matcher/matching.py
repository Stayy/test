"""Pure-Python 2D LaserScan feature extraction and matching utilities.

The functions in this module intentionally avoid ROS imports so the scan
matching math can be tested without a running ROS 2 environment.
"""

from __future__ import annotations

from dataclasses import dataclass
from itertools import combinations
import math
import random
from typing import Iterable, List, Optional, Sequence, Tuple


@dataclass(frozen=True)
class ScanPoint:
    index: int
    range: float
    bearing: float
    x: float
    y: float


@dataclass(frozen=True)
class ScanFeature:
    index: int
    kind: str
    x: float
    y: float
    range: float
    bearing: float
    response: float
    descriptor: Tuple[float, ...]


@dataclass(frozen=True)
class FeatureMatch:
    previous: ScanFeature
    current: ScanFeature
    score: float


@dataclass(frozen=True)
class Transform2D:
    x: float
    y: float
    yaw: float
    inlier_count: int = 0
    rmse: float = 0.0


@dataclass(frozen=True)
class Pose2D:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


@dataclass(frozen=True)
class FeatureExtractionConfig:
    corner_window: int = 4
    corner_threshold: float = 0.28
    range_jump_threshold: float = 0.35
    min_feature_separation: int = 5
    max_features: int = 80
    descriptor_offsets: Tuple[int, ...] = (-6, -3, 0, 3, 6)


@dataclass(frozen=True)
class MatchingConfig:
    max_correspondence_distance: float = 0.9
    max_bearing_shift: float = 0.65
    max_descriptor_distance: float = 2.5
    descriptor_weight: float = 0.35
    bearing_weight: float = 0.15
    require_same_kind: bool = True
    min_matches: int = 3
    ransac_inlier_threshold: float = 0.18
    max_ransac_trials: int = 160


def normalize_angle(angle: float) -> float:
    """Normalize an angle to [-pi, pi)."""
    while angle >= math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def is_valid_range(value: float, range_min: float, range_max: float) -> bool:
    if not math.isfinite(value):
        return False
    if range_min > 0.0 and value < range_min:
        return False
    if math.isfinite(range_max) and range_max > 0.0 and value > range_max:
        return False
    return value > 0.0


def build_points(
    ranges: Sequence[float],
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    *,
    laser_x: float = 0.0,
    laser_y: float = 0.0,
    laser_yaw: float = 0.0,
) -> List[ScanPoint]:
    """Convert valid LaserScan ranges to 2D points in the base frame."""
    points: List[ScanPoint] = []
    cos_mount = math.cos(laser_yaw)
    sin_mount = math.sin(laser_yaw)

    for index, distance in enumerate(ranges):
        if not is_valid_range(distance, range_min, range_max):
            continue

        bearing = angle_min + index * angle_increment
        laser_px = distance * math.cos(bearing)
        laser_py = distance * math.sin(bearing)
        base_x = cos_mount * laser_px - sin_mount * laser_py + laser_x
        base_y = sin_mount * laser_px + cos_mount * laser_py + laser_y
        base_bearing = normalize_angle(math.atan2(base_y, base_x))
        base_range = math.hypot(base_x, base_y)
        points.append(
            ScanPoint(
                index=index,
                range=base_range,
                bearing=base_bearing,
                x=base_x,
                y=base_y,
            )
        )

    return points


def extract_features(
    ranges: Sequence[float],
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    config: FeatureExtractionConfig,
    *,
    laser_x: float = 0.0,
    laser_y: float = 0.0,
    laser_yaw: float = 0.0,
) -> List[ScanFeature]:
    """Extract corner and range-discontinuity features from one scan."""
    if not ranges:
        return []

    points = build_points(
        ranges,
        angle_min,
        angle_increment,
        range_min,
        range_max,
        laser_x=laser_x,
        laser_y=laser_y,
        laser_yaw=laser_yaw,
    )
    by_index = {point.index: point for point in points}
    candidates: List[ScanFeature] = []
    window = max(1, int(config.corner_window))

    for point in points:
        left = by_index.get(point.index - window)
        right = by_index.get(point.index + window)
        if left is not None and right is not None:
            response = _corner_response(left, point, right)
            if response >= config.corner_threshold:
                candidates.append(
                    _make_feature(point, "corner", response, ranges, config)
                )

        jump_response = _range_jump_response(
            ranges, point.index, range_min, range_max
        )
        if jump_response >= config.range_jump_threshold:
            candidates.append(
                _make_feature(point, "edge", jump_response, ranges, config)
            )

    return _non_maximum_suppression(candidates, config)


def match_features(
    previous: Sequence[ScanFeature],
    current: Sequence[ScanFeature],
    config: MatchingConfig,
) -> List[FeatureMatch]:
    """Find mutual-nearest feature correspondences between two scans."""
    if not previous or not current:
        return []

    best_previous_for_current = {}
    for current_feature in current:
        best: Optional[FeatureMatch] = None
        for previous_feature in previous:
            score = _match_score(previous_feature, current_feature, config)
            if score is None:
                continue
            candidate = FeatureMatch(previous_feature, current_feature, score)
            if best is None or candidate.score < best.score:
                best = candidate
        if best is not None:
            best_previous_for_current[current_feature.index, current_feature.kind] = best

    best_current_for_previous = {}
    for previous_feature in previous:
        best = None
        for current_feature in current:
            score = _match_score(previous_feature, current_feature, config)
            if score is None:
                continue
            candidate = FeatureMatch(previous_feature, current_feature, score)
            if best is None or candidate.score < best.score:
                best = candidate
        if best is not None:
            best_current_for_previous[
                previous_feature.index, previous_feature.kind
            ] = best

    mutual_matches: List[FeatureMatch] = []
    for current_key, current_best in best_previous_for_current.items():
        previous_key = (
            current_best.previous.index,
            current_best.previous.kind,
        )
        previous_best = best_current_for_previous.get(previous_key)
        if previous_best is None:
            continue
        if (
            previous_best.current.index,
            previous_best.current.kind,
        ) == current_key:
            mutual_matches.append(current_best)

    mutual_matches.sort(key=lambda match: match.score)
    return mutual_matches


def estimate_transform(
    matches: Sequence[FeatureMatch],
    config: MatchingConfig,
) -> Optional[Transform2D]:
    """Estimate the transform that maps current scan points into previous scan frame."""
    if len(matches) < config.min_matches:
        return None

    if len(matches) == 1:
        return None

    candidate_pairs = _ransac_pairs(len(matches), config.max_ransac_trials)
    best_inlier_indices: List[int] = []
    best_rmse = float("inf")

    for first_index, second_index in candidate_pairs:
        seed_matches = [matches[first_index], matches[second_index]]
        transform = _fit_transform(seed_matches)
        if transform is None:
            continue
        inlier_indices, rmse = _score_transform(
            transform, matches, config.ransac_inlier_threshold
        )
        if (
            len(inlier_indices) > len(best_inlier_indices)
            or len(inlier_indices) == len(best_inlier_indices)
            and rmse < best_rmse
        ):
            best_inlier_indices = inlier_indices
            best_rmse = rmse

    if len(best_inlier_indices) < config.min_matches:
        return None

    inlier_matches = [matches[index] for index in best_inlier_indices]
    refined = _fit_transform(inlier_matches)
    if refined is None:
        return None

    _, refined_rmse = _score_transform(
        refined, inlier_matches, config.ransac_inlier_threshold
    )
    return Transform2D(
        x=refined.x,
        y=refined.y,
        yaw=refined.yaw,
        inlier_count=len(inlier_matches),
        rmse=refined_rmse,
    )


def compose_pose(pose: Pose2D, delta: Transform2D) -> Pose2D:
    """Compose a pose with a delta expressed in the previous body frame."""
    cos_yaw = math.cos(pose.yaw)
    sin_yaw = math.sin(pose.yaw)
    return Pose2D(
        x=pose.x + cos_yaw * delta.x - sin_yaw * delta.y,
        y=pose.y + sin_yaw * delta.x + cos_yaw * delta.y,
        yaw=normalize_angle(pose.yaw + delta.yaw),
    )


def transform_point(transform: Transform2D, x: float, y: float) -> Tuple[float, float]:
    cos_yaw = math.cos(transform.yaw)
    sin_yaw = math.sin(transform.yaw)
    return (
        cos_yaw * x - sin_yaw * y + transform.x,
        sin_yaw * x + cos_yaw * y + transform.y,
    )


def _corner_response(left: ScanPoint, center: ScanPoint, right: ScanPoint) -> float:
    left_vector = (left.x - center.x, left.y - center.y)
    right_vector = (right.x - center.x, right.y - center.y)
    left_norm = math.hypot(*left_vector)
    right_norm = math.hypot(*right_vector)
    if left_norm < 1.0e-6 or right_norm < 1.0e-6:
        return 0.0

    dot = (
        left_vector[0] * right_vector[0]
        + left_vector[1] * right_vector[1]
    ) / (left_norm * right_norm)
    dot = max(-1.0, min(1.0, dot))
    angle = math.acos(dot)
    return max(0.0, math.pi - angle)


def _range_jump_response(
    ranges: Sequence[float], index: int, range_min: float, range_max: float
) -> float:
    center = ranges[index]
    responses = []
    for neighbor_index in (index - 1, index + 1):
        if neighbor_index < 0 or neighbor_index >= len(ranges):
            continue
        neighbor = ranges[neighbor_index]
        if not is_valid_range(neighbor, range_min, range_max):
            continue
        responses.append(abs(center - neighbor))
    return max(responses, default=0.0)


def _make_feature(
    point: ScanPoint,
    kind: str,
    response: float,
    ranges: Sequence[float],
    config: FeatureExtractionConfig,
) -> ScanFeature:
    return ScanFeature(
        index=point.index,
        kind=kind,
        x=point.x,
        y=point.y,
        range=point.range,
        bearing=point.bearing,
        response=response,
        descriptor=_descriptor(ranges, point.index, config.descriptor_offsets),
    )


def _descriptor(
    ranges: Sequence[float], index: int, offsets: Iterable[int]
) -> Tuple[float, ...]:
    center = ranges[index]
    if not math.isfinite(center) or center <= 0.0:
        return tuple(0.0 for _ in offsets)

    values = []
    for offset in offsets:
        neighbor_index = index + offset
        if 0 <= neighbor_index < len(ranges):
            neighbor = ranges[neighbor_index]
            if math.isfinite(neighbor) and neighbor > 0.0:
                values.append(max(-2.0, min(2.0, (neighbor - center) / center)))
                continue
        values.append(2.0)
    return tuple(values)


def _non_maximum_suppression(
    candidates: Sequence[ScanFeature], config: FeatureExtractionConfig
) -> List[ScanFeature]:
    by_location = {}
    for candidate in candidates:
        key = (candidate.index, candidate.kind)
        current = by_location.get(key)
        if current is None or candidate.response > current.response:
            by_location[key] = candidate

    selected: List[ScanFeature] = []
    separation = max(1, int(config.min_feature_separation))
    for candidate in sorted(
        by_location.values(), key=lambda feature: feature.response, reverse=True
    ):
        if len(selected) >= config.max_features:
            break
        if any(abs(candidate.index - other.index) < separation for other in selected):
            continue
        selected.append(candidate)

    selected.sort(key=lambda feature: feature.index)
    return selected


def _match_score(
    previous: ScanFeature, current: ScanFeature, config: MatchingConfig
) -> Optional[float]:
    if config.require_same_kind and previous.kind != current.kind:
        return None

    point_distance = math.hypot(previous.x - current.x, previous.y - current.y)
    if point_distance > config.max_correspondence_distance:
        return None

    bearing_delta = abs(normalize_angle(previous.bearing - current.bearing))
    if bearing_delta > config.max_bearing_shift:
        return None

    descriptor_distance = _descriptor_distance(
        previous.descriptor, current.descriptor
    )
    if descriptor_distance > config.max_descriptor_distance:
        return None

    return (
        point_distance
        + config.bearing_weight * bearing_delta
        + config.descriptor_weight * descriptor_distance
    )


def _descriptor_distance(
    previous: Sequence[float], current: Sequence[float]
) -> float:
    if len(previous) != len(current):
        return float("inf")
    if not previous:
        return 0.0
    return sum(abs(a - b) for a, b in zip(previous, current)) / len(previous)


def _ransac_pairs(
    match_count: int, max_trials: int
) -> List[Tuple[int, int]]:
    pairs = list(combinations(range(match_count), 2))
    if len(pairs) <= max_trials:
        return pairs
    sampler = random.Random(17)
    return sampler.sample(pairs, max_trials)


def _fit_transform(matches: Sequence[FeatureMatch]) -> Optional[Transform2D]:
    if len(matches) < 2:
        return None

    source_centroid_x = sum(match.current.x for match in matches) / len(matches)
    source_centroid_y = sum(match.current.y for match in matches) / len(matches)
    target_centroid_x = sum(match.previous.x for match in matches) / len(matches)
    target_centroid_y = sum(match.previous.y for match in matches) / len(matches)

    h_xx = h_xy = h_yx = h_yy = 0.0
    for match in matches:
        source_x = match.current.x - source_centroid_x
        source_y = match.current.y - source_centroid_y
        target_x = match.previous.x - target_centroid_x
        target_y = match.previous.y - target_centroid_y
        h_xx += source_x * target_x
        h_xy += source_x * target_y
        h_yx += source_y * target_x
        h_yy += source_y * target_y

    if abs(h_xx) + abs(h_xy) + abs(h_yx) + abs(h_yy) < 1.0e-9:
        return None

    yaw = math.atan2(h_xy - h_yx, h_xx + h_yy)
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    translation_x = (
        target_centroid_x
        - cos_yaw * source_centroid_x
        + sin_yaw * source_centroid_y
    )
    translation_y = (
        target_centroid_y
        - sin_yaw * source_centroid_x
        - cos_yaw * source_centroid_y
    )
    return Transform2D(x=translation_x, y=translation_y, yaw=normalize_angle(yaw))


def _score_transform(
    transform: Transform2D,
    matches: Sequence[FeatureMatch],
    threshold: float,
) -> Tuple[List[int], float]:
    inlier_indices: List[int] = []
    squared_error = 0.0
    for index, match in enumerate(matches):
        mapped_x, mapped_y = transform_point(
            transform, match.current.x, match.current.y
        )
        residual = math.hypot(
            match.previous.x - mapped_x,
            match.previous.y - mapped_y,
        )
        if residual <= threshold:
            inlier_indices.append(index)
            squared_error += residual * residual

    if not inlier_indices:
        return [], float("inf")
    return inlier_indices, math.sqrt(squared_error / len(inlier_indices))
