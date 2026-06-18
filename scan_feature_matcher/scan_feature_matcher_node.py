"""ROS 2 node for single-line LaserScan feature matching."""

from __future__ import annotations

import math
from typing import List, Optional

import rclpy
from geometry_msgs.msg import Point, Pose, PoseArray, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from .matching import (
    FeatureExtractionConfig,
    FeatureMatch,
    MatchingConfig,
    Pose2D,
    ScanFeature,
    Transform2D,
    compose_pose,
    estimate_transform,
    extract_features,
    match_features,
)
from .patterns import (
    FenceDetection,
    LineFeatureDetection,
    PoleDetection,
    PolePatternConfig,
    detect_fences_from_scan,
    detect_line_features_from_scan,
)


class ScanFeatureMatcherNode(Node):
    """Match geometric scan features between consecutive LaserScan frames."""

    def __init__(self) -> None:
        super().__init__("scan_feature_matcher")

        self._declare_parameters()
        self.scan_topic = self.get_parameter("scan_topic").value
        self.base_frame = self.get_parameter("base_frame").value
        self.odom_frame = self.get_parameter("odom_frame").value
        self.publish_tf = bool(self.get_parameter("publish_tf").value)

        self.previous_features: Optional[List[ScanFeature]] = None
        self.pose = Pose2D()
        self.last_scan_time = None
        self.last_delta: Optional[Transform2D] = None
        self.last_matches: List[FeatureMatch] = []
        self.last_poles: List[PoleDetection] = []
        self.last_fences: List[FenceDetection] = []
        self.last_line_features: List[LineFeatureDetection] = []

        self.odom_pub = self.create_publisher(
            Odometry, "scan_feature_matcher/odom", 10
        )
        self.marker_pub = self.create_publisher(
            MarkerArray, "scan_feature_matcher/markers", 10
        )
        self.fence_pose_pub = self.create_publisher(
            PoseArray, "scan_feature_matcher/fence_poses", 10
        )
        self.line_feature_pose_pub = self.create_publisher(
            PoseArray, "scan_feature_matcher/line_feature_poses", 10
        )
        self.tf_broadcaster = (
            TransformBroadcaster(self) if self.publish_tf else None
        )
        self.scan_sub = self.create_subscription(
            LaserScan, self.scan_topic, self._scan_callback, 10
        )

        self.get_logger().info(
            f"Listening to {self.scan_topic}; publishing odom in "
            f"{self.odom_frame}->{self.base_frame}"
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("visualization_frame", "scan")

        # Fixed laser mounting transform from laser frame to base frame.
        self.declare_parameter("laser_x", 0.0)
        self.declare_parameter("laser_y", 0.0)
        self.declare_parameter("laser_yaw", 0.0)

        self.declare_parameter("corner_window", 4)
        self.declare_parameter("corner_threshold", 0.28)
        self.declare_parameter("range_jump_threshold", 0.35)
        self.declare_parameter("min_feature_separation", 5)
        self.declare_parameter("max_features", 80)

        self.declare_parameter("max_correspondence_distance", 0.9)
        self.declare_parameter("max_bearing_shift", 0.65)
        self.declare_parameter("max_descriptor_distance", 2.5)
        self.declare_parameter("descriptor_weight", 0.35)
        self.declare_parameter("bearing_weight", 0.15)
        self.declare_parameter("min_matches", 3)
        self.declare_parameter("ransac_inlier_threshold", 0.18)
        self.declare_parameter("max_ransac_trials", 160)
        self.declare_parameter("max_motion_translation", 1.2)
        self.declare_parameter("max_motion_rotation", 0.8)

        self.declare_parameter("custom_pattern_enabled", True)
        self.declare_parameter("custom_pattern_frame", "scan")
        self.declare_parameter("pole_cluster_jump_threshold", 0.06)
        self.declare_parameter("pole_min_points", 1)
        self.declare_parameter("pole_max_points", 12)
        self.declare_parameter("pole_min_width", 0.0)
        self.declare_parameter("pole_max_width", 0.08)
        self.declare_parameter("pole_max_range", 6.0)
        self.declare_parameter("fence_pole_count", 5)
        self.declare_parameter("fence_spacing", 0.10)
        self.declare_parameter("fence_spacing_tolerance", 0.03)
        self.declare_parameter("fence_collinearity_tolerance", 0.025)
        self.declare_parameter("fence_max_pattern_error", 0.035)
        self.declare_parameter("fence_max_candidate_poles", 30)
        self.declare_parameter("fence_max_detections", 3)
        self.declare_parameter("line_cluster_jump_threshold", 0.16)
        self.declare_parameter("line_min_points", 5)
        self.declare_parameter("line_min_length", 0.25)
        self.declare_parameter("line_max_length", 0.75)
        self.declare_parameter("line_max_width", 0.08)
        self.declare_parameter("line_max_range", 6.0)
        self.declare_parameter("line_anchor_cluster_jump_threshold", 0.06)
        self.declare_parameter("line_min_anchor_count", 3)
        self.declare_parameter("line_group_max_anchor_gap", 0.18)
        self.declare_parameter("line_max_detections", 5)

    def _scan_callback(self, scan: LaserScan) -> None:
        ranges = list(scan.ranges)
        extraction_config = self._extraction_config()
        matching_config = self._matching_config()

        features = extract_features(
            ranges,
            scan.angle_min,
            scan.angle_increment,
            scan.range_min,
            scan.range_max,
            extraction_config,
            laser_x=float(self.get_parameter("laser_x").value),
            laser_y=float(self.get_parameter("laser_y").value),
            laser_yaw=float(self.get_parameter("laser_yaw").value),
        )
        poles, fences, line_features = self._detect_custom_patterns(scan, ranges)

        matches: List[FeatureMatch] = []
        delta: Optional[Transform2D] = None
        if self.previous_features is not None:
            matches = match_features(
                self.previous_features, features, matching_config
            )
            delta = estimate_transform(matches, matching_config)
            if delta is not None and self._motion_is_plausible(delta):
                self.pose = compose_pose(self.pose, delta)
                self.last_delta = delta
                self.last_matches = matches
                self._publish_odometry(scan, delta)
                self._publish_tf(scan)
                self.get_logger().debug(
                    "scan match accepted: "
                    f"features={len(features)} matches={len(matches)} "
                    f"inliers={delta.inlier_count} rmse={delta.rmse:.3f} "
                    f"dx={delta.x:.3f} dy={delta.y:.3f} yaw={delta.yaw:.3f}"
                )
            else:
                self.get_logger().debug(
                    "scan match rejected: "
                    f"features={len(features)} matches={len(matches)}"
                )

        self._publish_fence_poses(scan, fences)
        self._publish_line_feature_poses(scan, line_features)
        self._publish_markers(
            scan, features, matches, delta, poles, fences, line_features
        )
        self.previous_features = features
        self.last_poles = poles
        self.last_fences = fences
        self.last_line_features = line_features
        self.last_scan_time = scan.header.stamp

    def _extraction_config(self) -> FeatureExtractionConfig:
        return FeatureExtractionConfig(
            corner_window=int(self.get_parameter("corner_window").value),
            corner_threshold=float(self.get_parameter("corner_threshold").value),
            range_jump_threshold=float(
                self.get_parameter("range_jump_threshold").value
            ),
            min_feature_separation=int(
                self.get_parameter("min_feature_separation").value
            ),
            max_features=int(self.get_parameter("max_features").value),
        )

    def _matching_config(self) -> MatchingConfig:
        return MatchingConfig(
            max_correspondence_distance=float(
                self.get_parameter("max_correspondence_distance").value
            ),
            max_bearing_shift=float(self.get_parameter("max_bearing_shift").value),
            max_descriptor_distance=float(
                self.get_parameter("max_descriptor_distance").value
            ),
            descriptor_weight=float(self.get_parameter("descriptor_weight").value),
            bearing_weight=float(self.get_parameter("bearing_weight").value),
            min_matches=int(self.get_parameter("min_matches").value),
            ransac_inlier_threshold=float(
                self.get_parameter("ransac_inlier_threshold").value
            ),
            max_ransac_trials=int(self.get_parameter("max_ransac_trials").value),
        )

    def _pattern_config(self) -> PolePatternConfig:
        return PolePatternConfig(
            pole_cluster_jump_threshold=float(
                self.get_parameter("pole_cluster_jump_threshold").value
            ),
            pole_min_points=int(self.get_parameter("pole_min_points").value),
            pole_max_points=int(self.get_parameter("pole_max_points").value),
            pole_min_width=float(self.get_parameter("pole_min_width").value),
            pole_max_width=float(self.get_parameter("pole_max_width").value),
            pole_max_range=float(self.get_parameter("pole_max_range").value),
            fence_pole_count=int(self.get_parameter("fence_pole_count").value),
            fence_spacing=float(self.get_parameter("fence_spacing").value),
            fence_spacing_tolerance=float(
                self.get_parameter("fence_spacing_tolerance").value
            ),
            fence_collinearity_tolerance=float(
                self.get_parameter("fence_collinearity_tolerance").value
            ),
            fence_max_pattern_error=float(
                self.get_parameter("fence_max_pattern_error").value
            ),
            fence_max_candidate_poles=int(
                self.get_parameter("fence_max_candidate_poles").value
            ),
            fence_max_detections=int(
                self.get_parameter("fence_max_detections").value
            ),
            line_cluster_jump_threshold=float(
                self.get_parameter("line_cluster_jump_threshold").value
            ),
            line_min_points=int(self.get_parameter("line_min_points").value),
            line_min_length=float(self.get_parameter("line_min_length").value),
            line_max_length=float(self.get_parameter("line_max_length").value),
            line_max_width=float(self.get_parameter("line_max_width").value),
            line_max_range=float(self.get_parameter("line_max_range").value),
            line_anchor_cluster_jump_threshold=float(
                self.get_parameter("line_anchor_cluster_jump_threshold").value
            ),
            line_min_anchor_count=int(
                self.get_parameter("line_min_anchor_count").value
            ),
            line_group_max_anchor_gap=float(
                self.get_parameter("line_group_max_anchor_gap").value
            ),
            line_max_detections=int(
                self.get_parameter("line_max_detections").value
            ),
        )

    def _detect_custom_patterns(
        self, scan: LaserScan, ranges: List[float]
    ) -> tuple[List[PoleDetection], List[FenceDetection], List[LineFeatureDetection]]:
        if not bool(self.get_parameter("custom_pattern_enabled").value):
            return [], [], []

        laser_x, laser_y, laser_yaw = self._custom_pattern_mount_transform()
        pattern_config = self._pattern_config()
        poles, fences = detect_fences_from_scan(
            ranges,
            scan.angle_min,
            scan.angle_increment,
            scan.range_min,
            scan.range_max,
            pattern_config,
            laser_x=laser_x,
            laser_y=laser_y,
            laser_yaw=laser_yaw,
        )
        line_features = detect_line_features_from_scan(
            ranges,
            scan.angle_min,
            scan.angle_increment,
            scan.range_min,
            scan.range_max,
            pattern_config,
            laser_x=laser_x,
            laser_y=laser_y,
            laser_yaw=laser_yaw,
        )
        if fences:
            best = fences[0]
            self.get_logger().debug(
                "custom fence detected: "
                f"poles={len(best.poles)} center=({best.center_x:.3f}, "
                f"{best.center_y:.3f}) yaw={best.yaw:.3f} score={best.score:.3f}"
            )
        if line_features:
            best_line = line_features[0]
            self.get_logger().debug(
                "custom line feature detected: "
                f"center=({best_line.center_x:.3f}, {best_line.center_y:.3f}) "
                f"yaw={best_line.yaw:.3f} length={best_line.length:.3f} "
                f"width={best_line.width:.3f} points={best_line.point_count}"
            )
        return poles, fences, line_features

    def _custom_pattern_mount_transform(self) -> tuple[float, float, float]:
        if str(self.get_parameter("custom_pattern_frame").value).lower() == "scan":
            return 0.0, 0.0, 0.0
        return (
            float(self.get_parameter("laser_x").value),
            float(self.get_parameter("laser_y").value),
            float(self.get_parameter("laser_yaw").value),
        )

    def _custom_pattern_frame_id(self, scan: LaserScan) -> str:
        if str(self.get_parameter("custom_pattern_frame").value).lower() == "scan":
            return scan.header.frame_id
        return self.base_frame

    def _marker_frame_id(self, scan: LaserScan) -> str:
        if str(self.get_parameter("visualization_frame").value).lower() == "scan":
            return scan.header.frame_id
        return self.base_frame

    def _motion_is_plausible(self, delta: Transform2D) -> bool:
        max_translation = float(self.get_parameter("max_motion_translation").value)
        max_rotation = float(self.get_parameter("max_motion_rotation").value)
        return (
            math.hypot(delta.x, delta.y) <= max_translation
            and abs(delta.yaw) <= max_rotation
        )

    def _publish_odometry(self, scan: LaserScan, delta: Transform2D) -> None:
        odom = Odometry()
        odom.header.stamp = scan.header.stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.pose.x
        odom.pose.pose.position.y = self.pose.y
        odom.pose.pose.orientation.z = math.sin(self.pose.yaw * 0.5)
        odom.pose.pose.orientation.w = math.cos(self.pose.yaw * 0.5)

        dt = self._scan_period_seconds(scan)
        if dt > 1.0e-6:
            odom.twist.twist.linear.x = delta.x / dt
            odom.twist.twist.linear.y = delta.y / dt
            odom.twist.twist.angular.z = delta.yaw / dt

        odom.pose.covariance[0] = max(delta.rmse * delta.rmse, 1.0e-4)
        odom.pose.covariance[7] = max(delta.rmse * delta.rmse, 1.0e-4)
        odom.pose.covariance[35] = max(delta.rmse * delta.rmse, 1.0e-4)
        self.odom_pub.publish(odom)

    def _publish_tf(self, scan: LaserScan) -> None:
        if self.tf_broadcaster is None:
            return

        transform = TransformStamped()
        transform.header.stamp = scan.header.stamp
        transform.header.frame_id = self.odom_frame
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = self.pose.x
        transform.transform.translation.y = self.pose.y
        transform.transform.rotation.z = math.sin(self.pose.yaw * 0.5)
        transform.transform.rotation.w = math.cos(self.pose.yaw * 0.5)
        self.tf_broadcaster.sendTransform(transform)

    def _publish_fence_poses(
        self, scan: LaserScan, fences: List[FenceDetection]
    ) -> None:
        poses = PoseArray()
        poses.header.stamp = scan.header.stamp
        poses.header.frame_id = self._custom_pattern_frame_id(scan)
        for fence in fences:
            pose = Pose()
            pose.position.x = fence.center_x
            pose.position.y = fence.center_y
            pose.orientation.z = math.sin(fence.yaw * 0.5)
            pose.orientation.w = math.cos(fence.yaw * 0.5)
            poses.poses.append(pose)
        self.fence_pose_pub.publish(poses)

    def _publish_line_feature_poses(
        self, scan: LaserScan, line_features: List[LineFeatureDetection]
    ) -> None:
        poses = PoseArray()
        poses.header.stamp = scan.header.stamp
        poses.header.frame_id = self._custom_pattern_frame_id(scan)
        for line_feature in line_features:
            pose = Pose()
            pose.position.x = line_feature.center_x
            pose.position.y = line_feature.center_y
            pose.orientation.z = math.sin(line_feature.yaw * 0.5)
            pose.orientation.w = math.cos(line_feature.yaw * 0.5)
            poses.poses.append(pose)
        self.line_feature_pose_pub.publish(poses)

    def _publish_markers(
        self,
        scan: LaserScan,
        features: List[ScanFeature],
        matches: List[FeatureMatch],
        delta: Optional[Transform2D],
        poles: List[PoleDetection],
        fences: List[FenceDetection],
        line_features: List[LineFeatureDetection],
    ) -> None:
        marker_array = MarkerArray()
        delete_marker = Marker()
        delete_marker.header.stamp = scan.header.stamp
        delete_marker.header.frame_id = self._marker_frame_id(scan)
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)

        marker_array.markers.append(
            self._feature_marker(scan, features, "corner", 1, (0.1, 1.0, 0.1))
        )
        marker_array.markers.append(
            self._feature_marker(scan, features, "edge", 2, (1.0, 0.7, 0.1))
        )
        marker_array.markers.append(self._match_marker(scan, matches, delta))
        marker_array.markers.append(self._pole_marker(scan, poles))
        marker_array.markers.append(self._fence_marker(scan, fences))
        marker_array.markers.append(self._fence_center_marker(scan, fences))
        marker_array.markers.append(self._line_feature_marker(scan, line_features))
        marker_array.markers.append(
            self._line_feature_center_marker(scan, line_features)
        )
        self.marker_pub.publish(marker_array)

    def _feature_marker(
        self,
        scan: LaserScan,
        features: List[ScanFeature],
        kind: str,
        marker_id: int,
        rgb: tuple,
    ) -> Marker:
        marker = Marker()
        marker.header.stamp = scan.header.stamp
        marker.header.frame_id = self._custom_pattern_frame_id(scan)
        marker.ns = f"scan_{kind}_features"
        marker.id = marker_id
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.08
        marker.scale.y = 0.08
        marker.scale.z = 0.08
        marker.color.r = rgb[0]
        marker.color.g = rgb[1]
        marker.color.b = rgb[2]
        marker.color.a = 0.9
        marker.points = [
            Point(x=feature.x, y=feature.y, z=0.05)
            for feature in features
            if feature.kind == kind
        ]
        return marker

    def _match_marker(
        self,
        scan: LaserScan,
        matches: List[FeatureMatch],
        delta: Optional[Transform2D],
    ) -> Marker:
        marker = Marker()
        marker.header.stamp = scan.header.stamp
        marker.header.frame_id = self._custom_pattern_frame_id(scan)
        marker.ns = "scan_feature_matches"
        marker.id = 3
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.02
        marker.color.r = 0.1
        marker.color.g = 0.4
        marker.color.b = 1.0
        marker.color.a = 0.8

        if delta is None:
            return marker

        for match in matches:
            marker.points.append(
                Point(x=match.current.x, y=match.current.y, z=0.02)
            )
            marker.points.append(
                Point(x=match.previous.x, y=match.previous.y, z=0.02)
            )
        return marker

    def _pole_marker(
        self, scan: LaserScan, poles: List[PoleDetection]
    ) -> Marker:
        marker = Marker()
        marker.header.stamp = scan.header.stamp
        marker.header.frame_id = self._custom_pattern_frame_id(scan)
        marker.ns = "custom_pole_candidates"
        marker.id = 4
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.r = 0.7
        marker.color.g = 0.2
        marker.color.b = 1.0
        marker.color.a = 0.9
        marker.points = [
            Point(x=pole.x, y=pole.y, z=0.08) for pole in poles
        ]
        return marker

    def _fence_marker(
        self, scan: LaserScan, fences: List[FenceDetection]
    ) -> Marker:
        marker = Marker()
        marker.header.stamp = scan.header.stamp
        marker.header.frame_id = self._marker_frame_id(scan)
        marker.ns = "custom_five_pole_fence"
        marker.id = 5
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.035
        marker.color.r = 1.0
        marker.color.g = 0.05
        marker.color.b = 0.05
        marker.color.a = 0.95

        for fence in fences:
            for first, second in zip(fence.poles, fence.poles[1:]):
                marker.points.append(Point(x=first.x, y=first.y, z=0.12))
                marker.points.append(Point(x=second.x, y=second.y, z=0.12))
        return marker

    def _fence_center_marker(
        self, scan: LaserScan, fences: List[FenceDetection]
    ) -> Marker:
        marker = Marker()
        marker.header.stamp = scan.header.stamp
        marker.header.frame_id = self._marker_frame_id(scan)
        marker.ns = "custom_fence_centers"
        marker.id = 6
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.09
        marker.scale.y = 0.09
        marker.scale.z = 0.09
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.points = [
            Point(x=fence.center_x, y=fence.center_y, z=0.18)
            for fence in fences
        ]
        return marker

    def _line_feature_marker(
        self, scan: LaserScan, line_features: List[LineFeatureDetection]
    ) -> Marker:
        marker = Marker()
        marker.header.stamp = scan.header.stamp
        marker.header.frame_id = self._custom_pattern_frame_id(scan)
        marker.ns = "custom_yellow_line_features"
        marker.id = 7
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.04
        marker.color.r = 1.0
        marker.color.g = 0.95
        marker.color.b = 0.0
        marker.color.a = 1.0

        for line_feature in line_features:
            start, end = line_feature.endpoints
            marker.points.append(Point(x=start[0], y=start[1], z=0.16))
            marker.points.append(Point(x=end[0], y=end[1], z=0.16))
        return marker

    def _line_feature_center_marker(
        self, scan: LaserScan, line_features: List[LineFeatureDetection]
    ) -> Marker:
        marker = Marker()
        marker.header.stamp = scan.header.stamp
        marker.header.frame_id = self._custom_pattern_frame_id(scan)
        marker.ns = "custom_yellow_line_centers"
        marker.id = 8
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.10
        marker.scale.y = 0.10
        marker.scale.z = 0.10
        marker.color.r = 1.0
        marker.color.g = 0.95
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.points = [
            Point(x=line.center_x, y=line.center_y, z=0.22)
            for line in line_features
        ]
        return marker

    def _scan_period_seconds(self, scan: LaserScan) -> float:
        if self.last_scan_time is None:
            return float(scan.scan_time)

        current = scan.header.stamp.sec + scan.header.stamp.nanosec * 1.0e-9
        previous = (
            self.last_scan_time.sec + self.last_scan_time.nanosec * 1.0e-9
        )
        if current > previous:
            return current - previous
        return float(scan.scan_time)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ScanFeatureMatcherNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
