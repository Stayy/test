"""Custom landmark pattern detection for LaserScan data."""

from __future__ import annotations

from dataclasses import dataclass
from itertools import combinations
import math
from typing import List, Optional, Sequence, Tuple

from .matching import ScanPoint, build_points, normalize_angle


@dataclass(frozen=True)
class PolePatternConfig:
    pole_cluster_jump_threshold: float = 0.06
    pole_min_points: int = 1
    pole_max_points: int = 12
    pole_min_width: float = 0.0
    pole_max_width: float = 0.08
    pole_max_range: float = 6.0
    fence_pole_count: int = 5
    fence_spacing: float = 0.10
    fence_spacing_tolerance: float = 0.03
    fence_collinearity_tolerance: float = 0.025
    fence_max_pattern_error: float = 0.035
    fence_max_candidate_poles: int = 30
    fence_max_detections: int = 3
    line_cluster_jump_threshold: float = 0.16
    line_min_points: int = 5
    line_min_length: float = 0.25
    line_max_length: float = 0.75
    line_max_width: float = 0.08
    line_max_range: float = 6.0
    line_max_detections: int = 5


@dataclass(frozen=True)
class PoleDetection:
    index: int
    x: float
    y: float
    range: float
    bearing: float
    width: float
    point_count: int
    scan_indices: Tuple[int, ...]


@dataclass(frozen=True)
class FenceDetection:
    poles: Tuple[PoleDetection, ...]
    center_x: float
    center_y: float
    yaw: float
    spacing_error: float
    collinearity_error: float
    score: float
    span: float


@dataclass(frozen=True)
class LineFeatureDetection:
    center_x: float
    center_y: float
    yaw: float
    length: float
    width: float
    point_count: int
    score: float
    scan_indices: Tuple[int, ...]
    endpoints: Tuple[Tuple[float, float], Tuple[float, float]]


def detect_poles_from_scan(
    ranges: Sequence[float],
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    config: PolePatternConfig,
    *,
    laser_x: float = 0.0,
    laser_y: float = 0.0,
    laser_yaw: float = 0.0,
) -> List[PoleDetection]:
    """Detect small isolated scan clusters that can represent vertical poles."""
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
    clusters = _segment_points(points, config.pole_cluster_jump_threshold)
    poles: List[PoleDetection] = []
    for cluster in clusters:
        pole = _cluster_to_pole(cluster, config)
        if pole is not None:
            poles.append(pole)

    poles.sort(key=lambda pole: (pole.range, pole.bearing))
    return poles[: max(0, int(config.fence_max_candidate_poles))]


def detect_fence_patterns(
    poles: Sequence[PoleDetection], config: PolePatternConfig
) -> List[FenceDetection]:
    """Find groups of equally spaced collinear poles."""
    pole_count = max(2, int(config.fence_pole_count))
    if len(poles) < pole_count:
        return []

    candidate_poles = list(poles[: max(pole_count, config.fence_max_candidate_poles)])
    candidates: List[FenceDetection] = []
    for group in combinations(candidate_poles, pole_count):
        candidate = _fit_fence(group, config)
        if candidate is not None:
            candidates.append(candidate)

    selected: List[FenceDetection] = []
    used_index_sets: List[set] = []
    for candidate in sorted(candidates, key=lambda item: item.score):
        candidate_indices = {pole.index for pole in candidate.poles}
        if any(
            len(candidate_indices & used) >= pole_count - 1
            for used in used_index_sets
        ):
            continue
        selected.append(candidate)
        used_index_sets.append(candidate_indices)
        if len(selected) >= config.fence_max_detections:
            break
    return selected


def detect_fences_from_scan(
    ranges: Sequence[float],
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    config: PolePatternConfig,
    *,
    laser_x: float = 0.0,
    laser_y: float = 0.0,
    laser_yaw: float = 0.0,
) -> Tuple[List[PoleDetection], List[FenceDetection]]:
    """Detect pole candidates and custom fence landmarks in one scan."""
    poles = detect_poles_from_scan(
        ranges,
        angle_min,
        angle_increment,
        range_min,
        range_max,
        config,
        laser_x=laser_x,
        laser_y=laser_y,
        laser_yaw=laser_yaw,
    )
    return poles, detect_fence_patterns(poles, config)


def detect_line_features_from_scan(
    ranges: Sequence[float],
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    config: PolePatternConfig,
    *,
    laser_x: float = 0.0,
    laser_y: float = 0.0,
    laser_yaw: float = 0.0,
) -> List[LineFeatureDetection]:
    """Detect short straight scan clusters like the boxed landmarks in RViz."""
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
    clusters = _segment_points_by_distance(
        points, config.line_cluster_jump_threshold
    )
    detections = []
    for cluster in clusters:
        detection = _cluster_to_line_feature(cluster, config)
        if detection is not None:
            detections.append(detection)

    detections.sort(key=lambda item: item.score)
    return detections[: max(0, int(config.line_max_detections))]


def _segment_points(
    points: Sequence[ScanPoint], jump_threshold: float
) -> List[List[ScanPoint]]:
    if not points:
        return []

    clusters: List[List[ScanPoint]] = []
    current = [points[0]]
    for point in points[1:]:
        previous = current[-1]
        point_gap = math.hypot(point.x - previous.x, point.y - previous.y)
        if point.index != previous.index + 1 or point_gap > jump_threshold:
            clusters.append(current)
            current = [point]
        else:
            current.append(point)
    clusters.append(current)
    return clusters


def _segment_points_by_distance(
    points: Sequence[ScanPoint], jump_threshold: float
) -> List[List[ScanPoint]]:
    if not points:
        return []

    clusters: List[List[ScanPoint]] = []
    current = [points[0]]
    for point in points[1:]:
        previous = current[-1]
        point_gap = math.hypot(point.x - previous.x, point.y - previous.y)
        if point_gap > jump_threshold:
            clusters.append(current)
            current = [point]
        else:
            current.append(point)
    clusters.append(current)
    return clusters


def _cluster_to_pole(
    cluster: Sequence[ScanPoint], config: PolePatternConfig
) -> Optional[PoleDetection]:
    point_count = len(cluster)
    if point_count < config.pole_min_points or point_count > config.pole_max_points:
        return None

    width = _cluster_width(cluster)
    if width < config.pole_min_width or width > config.pole_max_width:
        return None

    center_x = sum(point.x for point in cluster) / point_count
    center_y = sum(point.y for point in cluster) / point_count
    center_range = math.hypot(center_x, center_y)
    if config.pole_max_range > 0.0 and center_range > config.pole_max_range:
        return None

    center_index = cluster[point_count // 2].index
    return PoleDetection(
        index=center_index,
        x=center_x,
        y=center_y,
        range=center_range,
        bearing=math.atan2(center_y, center_x),
        width=width,
        point_count=point_count,
        scan_indices=tuple(point.index for point in cluster),
    )


def _cluster_width(cluster: Sequence[ScanPoint]) -> float:
    if len(cluster) <= 1:
        return 0.0
    first = cluster[0]
    last = cluster[-1]
    return math.hypot(last.x - first.x, last.y - first.y)


def _fit_fence(
    poles: Sequence[PoleDetection], config: PolePatternConfig
) -> Optional[FenceDetection]:
    center_x = sum(pole.x for pole in poles) / len(poles)
    center_y = sum(pole.y for pole in poles) / len(poles)
    yaw = _principal_axis_yaw(poles, center_x, center_y)
    axis_x = math.cos(yaw)
    axis_y = math.sin(yaw)

    projected = []
    perpendicular_errors = []
    for pole in poles:
        dx = pole.x - center_x
        dy = pole.y - center_y
        along = dx * axis_x + dy * axis_y
        across = -dx * axis_y + dy * axis_x
        projected.append((along, pole))
        perpendicular_errors.append(abs(across))

    projected.sort(key=lambda item: item[0])
    ordered_poles = tuple(item[1] for item in projected)
    positions = [item[0] for item in projected]
    spacings = [
        positions[index + 1] - positions[index]
        for index in range(len(positions) - 1)
    ]
    spacing_errors = [
        abs(spacing - config.fence_spacing) for spacing in spacings
    ]
    max_spacing_error = max(spacing_errors, default=0.0)
    max_collinearity_error = max(perpendicular_errors, default=0.0)
    if max_spacing_error > config.fence_spacing_tolerance:
        return None
    if max_collinearity_error > config.fence_collinearity_tolerance:
        return None

    spacing_rmse = _rmse(spacing_errors)
    collinearity_rmse = _rmse(perpendicular_errors)
    score = math.hypot(spacing_rmse, collinearity_rmse)
    if score > config.fence_max_pattern_error:
        return None

    first_pole = ordered_poles[0]
    last_pole = ordered_poles[-1]
    yaw = math.atan2(last_pole.y - first_pole.y, last_pole.x - first_pole.x)

    return FenceDetection(
        poles=ordered_poles,
        center_x=center_x,
        center_y=center_y,
        yaw=normalize_angle(yaw),
        spacing_error=spacing_rmse,
        collinearity_error=collinearity_rmse,
        score=score,
        span=positions[-1] - positions[0],
    )


def _cluster_to_line_feature(
    cluster: Sequence[ScanPoint], config: PolePatternConfig
) -> Optional[LineFeatureDetection]:
    point_count = len(cluster)
    if point_count < config.line_min_points:
        return None

    center_x = sum(point.x for point in cluster) / point_count
    center_y = sum(point.y for point in cluster) / point_count
    center_range = math.hypot(center_x, center_y)
    if config.line_max_range > 0.0 and center_range > config.line_max_range:
        return None

    yaw = _principal_axis_yaw_for_points(cluster, center_x, center_y)
    axis_x = math.cos(yaw)
    axis_y = math.sin(yaw)
    projections = []
    lateral_errors = []
    for point in cluster:
        dx = point.x - center_x
        dy = point.y - center_y
        along = dx * axis_x + dy * axis_y
        across = -dx * axis_y + dy * axis_x
        projections.append(along)
        lateral_errors.append(abs(across))

    length = max(projections) - min(projections)
    width = max(lateral_errors, default=0.0) * 2.0
    if length < config.line_min_length or length > config.line_max_length:
        return None
    if width > config.line_max_width:
        return None

    if projections[-1] < projections[0]:
        yaw = normalize_angle(yaw + math.pi)
        axis_x = math.cos(yaw)
        axis_y = math.sin(yaw)

    half_length = length * 0.5
    endpoint_a = (
        center_x - axis_x * half_length,
        center_y - axis_y * half_length,
    )
    endpoint_b = (
        center_x + axis_x * half_length,
        center_y + axis_y * half_length,
    )
    score = _rmse(lateral_errors) + 0.01 / max(point_count, 1)
    return LineFeatureDetection(
        center_x=center_x,
        center_y=center_y,
        yaw=normalize_angle(yaw),
        length=length,
        width=width,
        point_count=point_count,
        score=score,
        scan_indices=tuple(point.index for point in cluster),
        endpoints=(endpoint_a, endpoint_b),
    )


def _principal_axis_yaw(
    poles: Sequence[PoleDetection], center_x: float, center_y: float
) -> float:
    sxx = syy = sxy = 0.0
    for pole in poles:
        dx = pole.x - center_x
        dy = pole.y - center_y
        sxx += dx * dx
        syy += dy * dy
        sxy += dx * dy
    if abs(sxx - syy) + abs(sxy) < 1.0e-12:
        return 0.0
    return 0.5 * math.atan2(2.0 * sxy, sxx - syy)


def _principal_axis_yaw_for_points(
    points: Sequence[ScanPoint], center_x: float, center_y: float
) -> float:
    sxx = syy = sxy = 0.0
    for point in points:
        dx = point.x - center_x
        dy = point.y - center_y
        sxx += dx * dx
        syy += dy * dy
        sxy += dx * dy
    if abs(sxx - syy) + abs(sxy) < 1.0e-12:
        return 0.0
    return 0.5 * math.atan2(2.0 * sxy, sxx - syy)


def _rmse(values: Sequence[float]) -> float:
    if not values:
        return 0.0
    return math.sqrt(sum(value * value for value in values) / len(values))
