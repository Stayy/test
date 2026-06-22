import math
import unittest

from scan_feature_matcher.matching import (
    FeatureMatch,
    MatchingConfig,
    Pose2D,
    ScanFeature,
    Transform2D,
    build_points,
    compose_pose,
    estimate_transform,
    normalize_angle,
)


class MatchingTest(unittest.TestCase):
    def test_build_points_applies_laser_mount_transform(self):
        points = build_points(
            [1.0],
            angle_min=0.0,
            angle_increment=1.0,
            range_min=0.05,
            range_max=10.0,
            laser_x=0.2,
            laser_y=-0.1,
            laser_yaw=math.pi / 2.0,
        )

        self.assertEqual(len(points), 1)
        self.assertAlmostEqual(points[0].x, 0.2)
        self.assertAlmostEqual(points[0].y, 0.9)

    def test_estimate_transform_recovers_known_scan_motion(self):
        expected = Transform2D(x=0.25, y=-0.12, yaw=0.18)
        previous_points = [(1.0, 0.2), (2.0, -0.4), (1.4, 1.1), (2.4, 0.7)]
        current_points = [
            _inverse_transform(expected, x, y) for x, y in previous_points
        ]
        matches = [
            FeatureMatch(
                previous=_feature(index, previous[0], previous[1]),
                current=_feature(index, current[0], current[1]),
                score=0.0,
            )
            for index, (previous, current) in enumerate(
                zip(previous_points, current_points)
            )
        ]

        estimate = estimate_transform(matches, MatchingConfig(min_matches=3))

        self.assertIsNotNone(estimate)
        self.assertEqual(estimate.inlier_count, 4)
        self.assertAlmostEqual(estimate.x, expected.x)
        self.assertAlmostEqual(estimate.y, expected.y)
        self.assertAlmostEqual(estimate.yaw, expected.yaw)

    def test_compose_pose_applies_delta_in_previous_body_frame(self):
        pose = Pose2D(x=1.0, y=2.0, yaw=math.pi / 2.0)
        delta = Transform2D(x=0.5, y=0.0, yaw=-0.1)

        result = compose_pose(pose, delta)

        self.assertAlmostEqual(result.x, 1.0)
        self.assertAlmostEqual(result.y, 2.5)
        self.assertAlmostEqual(
            result.yaw, normalize_angle(math.pi / 2.0 - 0.1)
        )


def _feature(index, x, y):
    return ScanFeature(
        index=index,
        kind="corner",
        x=x,
        y=y,
        range=math.hypot(x, y),
        bearing=math.atan2(y, x),
        response=1.0,
        descriptor=(0.0, 0.0, 0.0),
    )


def _inverse_transform(transform, x, y):
    translated_x = x - transform.x
    translated_y = y - transform.y
    cos_yaw = math.cos(transform.yaw)
    sin_yaw = math.sin(transform.yaw)
    return (
        cos_yaw * translated_x + sin_yaw * translated_y,
        -sin_yaw * translated_x + cos_yaw * translated_y,
    )


if __name__ == "__main__":
    unittest.main()
