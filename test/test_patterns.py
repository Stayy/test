import math
import unittest

from scan_feature_matcher.patterns import (
    PoleDetection,
    PolePatternConfig,
    detect_fence_patterns,
    detect_fences_from_scan,
    detect_line_features_from_scan,
)


class PatternDetectionTest(unittest.TestCase):
    def test_detects_five_pole_fence_from_scan_points(self):
        ranges, angle_min, angle_increment = _scan_for_points(
            [(2.0, -0.20), (2.0, -0.10), (2.0, 0.0), (2.0, 0.10), (2.0, 0.20)]
        )
        config = PolePatternConfig(
            pole_cluster_jump_threshold=0.05,
            fence_spacing=0.10,
            fence_spacing_tolerance=0.015,
            fence_collinearity_tolerance=0.01,
            fence_max_pattern_error=0.02,
        )

        poles, fences = detect_fences_from_scan(
            ranges,
            angle_min,
            angle_increment,
            range_min=0.05,
            range_max=10.0,
            config=config,
        )

        self.assertEqual(len(poles), 5)
        self.assertEqual(len(fences), 1)
        self.assertEqual(len(fences[0].poles), 5)
        self.assertAlmostEqual(fences[0].span, 0.40, places=2)
        self.assertAlmostEqual(abs(fences[0].yaw), math.pi / 2.0, places=2)

    def test_rejects_wrong_spacing(self):
        config = PolePatternConfig(
            fence_spacing=0.10,
            fence_spacing_tolerance=0.01,
            fence_collinearity_tolerance=0.01,
            fence_max_pattern_error=0.02,
        )
        poles = [
            _pole(index, 2.0, y)
            for index, y in enumerate([-0.24, -0.12, 0.0, 0.12, 0.24])
        ]

        fences = detect_fence_patterns(poles, config)

        self.assertEqual(fences, [])

    def test_detects_short_line_feature_from_sparse_scan_points(self):
        ranges, angle_min, angle_increment = _scan_for_points(
            [(2.0, -0.20), (2.0, -0.10), (2.0, 0.0), (2.0, 0.10), (2.0, 0.20)]
        )
        config = PolePatternConfig(
            line_cluster_jump_threshold=0.16,
            line_min_points=5,
            line_min_length=0.25,
            line_max_length=0.75,
            line_max_width=0.03,
        )

        features = detect_line_features_from_scan(
            ranges,
            angle_min,
            angle_increment,
            range_min=0.05,
            range_max=10.0,
            config=config,
        )

        self.assertEqual(len(features), 1)
        self.assertAlmostEqual(features[0].length, 0.40, places=2)
        self.assertAlmostEqual(features[0].width, 0.0, places=2)

    def test_detects_line_feature_from_separated_anchor_clusters(self):
        ranges, angle_min, angle_increment = _scan_for_points(
            [(2.0, -0.24), (2.0, -0.12), (2.0, 0.0), (2.0, 0.12), (2.0, 0.24)]
        )
        config = PolePatternConfig(
            line_cluster_jump_threshold=0.08,
            line_min_points=5,
            line_min_length=0.25,
            line_max_length=0.75,
            line_max_width=0.03,
            line_anchor_cluster_jump_threshold=0.03,
            line_min_anchor_count=5,
            line_group_max_anchor_gap=0.15,
        )

        features = detect_line_features_from_scan(
            ranges,
            angle_min,
            angle_increment,
            range_min=0.05,
            range_max=10.0,
            config=config,
        )

        self.assertEqual(len(features), 1)
        self.assertAlmostEqual(features[0].length, 0.48, places=2)
        self.assertEqual(features[0].point_count, 5)

    def test_detects_line_feature_with_endpoint_duplicate_return(self):
        ranges, angle_min, angle_increment = _scan_for_points(
            [
                (2.0, -0.24),
                (1.96, -0.22),
                (2.0, -0.12),
                (2.0, 0.0),
                (2.0, 0.12),
                (2.0, 0.24),
            ]
        )
        config = PolePatternConfig(
            line_cluster_jump_threshold=0.08,
            line_min_points=5,
            line_min_length=0.25,
            line_max_length=0.75,
            line_max_width=0.04,
            line_anchor_cluster_jump_threshold=0.03,
            line_min_anchor_count=5,
            line_min_anchor_spacing=0.10,
            line_group_max_anchor_gap=0.16,
            line_hypothesis_lateral_tolerance=0.04,
            line_isolation_enabled=True,
        )

        features = detect_line_features_from_scan(
            ranges,
            angle_min,
            angle_increment,
            range_min=0.05,
            range_max=10.0,
            config=config,
        )

        self.assertEqual(len(features), 1)
        self.assertEqual(features[0].point_count, 5)
        self.assertAlmostEqual(features[0].length, 0.48, places=2)

    def test_line_hypothesis_ignores_nearby_outlier(self):
        ranges, angle_min, angle_increment = _scan_for_points(
            [
                (2.0, -0.24),
                (2.0, -0.12),
                (2.0, 0.0),
                (2.0, 0.12),
                (2.0, 0.24),
                (1.90, 0.18),
            ]
        )
        config = PolePatternConfig(
            line_cluster_jump_threshold=0.08,
            line_min_points=5,
            line_min_length=0.25,
            line_max_length=0.75,
            line_max_width=0.04,
            line_anchor_cluster_jump_threshold=0.03,
            line_min_anchor_count=5,
            line_group_max_anchor_gap=0.16,
            line_hypothesis_lateral_tolerance=0.04,
        )

        features = detect_line_features_from_scan(
            ranges,
            angle_min,
            angle_increment,
            range_min=0.05,
            range_max=10.0,
            config=config,
        )

        self.assertEqual(len(features), 1)
        self.assertEqual(features[0].point_count, 5)
        self.assertLessEqual(features[0].width, 0.04)

    def test_detects_line_feature_from_slightly_merged_blobs(self):
        ranges, angle_min, angle_increment = _scan_for_points(
            [
                (2.00, -0.20),
                (2.02, -0.195),
                (2.00, -0.10),
                (2.02, -0.095),
                (2.00, 0.0),
                (2.02, 0.005),
                (2.00, 0.10),
                (2.02, 0.105),
                (2.00, 0.20),
                (2.02, 0.205),
            ]
        )
        config = PolePatternConfig(
            line_cluster_jump_threshold=0.08,
            line_min_points=5,
            line_min_length=0.25,
            line_max_length=0.75,
            line_max_width=0.06,
            line_anchor_cluster_jump_threshold=0.04,
            line_min_anchor_count=5,
            line_group_max_anchor_gap=0.14,
            line_hypothesis_lateral_tolerance=0.04,
        )

        features = detect_line_features_from_scan(
            ranges,
            angle_min,
            angle_increment,
            range_min=0.05,
            range_max=10.0,
            config=config,
        )

        self.assertEqual(len(features), 1)
        self.assertGreaterEqual(features[0].point_count, 5)
        self.assertAlmostEqual(features[0].length, 0.40, places=1)

    def test_rejects_curved_cluster_as_line_feature(self):
        points = [
            (math.cos(angle), math.sin(angle))
            for angle in (-0.30, -0.15, 0.0, 0.15, 0.30)
        ]
        ranges, angle_min, angle_increment = _scan_for_points(points)
        config = PolePatternConfig(
            line_cluster_jump_threshold=0.16,
            line_min_points=5,
            line_min_length=0.25,
            line_max_length=0.75,
            line_max_width=0.02,
        )

        features = detect_line_features_from_scan(
            ranges,
            angle_min,
            angle_increment,
            range_min=0.05,
            range_max=10.0,
            config=config,
        )

        self.assertEqual(features, [])

    def test_rejects_line_feature_when_anchor_spacing_is_under_ten_cm(self):
        ranges, angle_min, angle_increment = _scan_for_points(
            [(2.0, -0.16), (2.0, -0.08), (2.0, 0.0), (2.0, 0.08), (2.0, 0.16)]
        )
        config = PolePatternConfig(
            line_cluster_jump_threshold=0.10,
            line_min_points=5,
            line_min_length=0.25,
            line_max_length=0.75,
            line_max_width=0.03,
            line_anchor_cluster_jump_threshold=0.03,
            line_min_anchor_count=5,
            line_min_anchor_spacing=0.10,
            line_group_max_anchor_gap=0.15,
            line_isolation_enabled=True,
        )

        features = detect_line_features_from_scan(
            ranges,
            angle_min,
            angle_increment,
            range_min=0.05,
            range_max=10.0,
            config=config,
        )

        self.assertEqual(features, [])

    def test_rejects_wall_segments_because_line_is_not_isolated(self):
        wall_points = [(2.0, y) for y in (-0.55, -0.44, -0.33, -0.22, -0.11, 0.0, 0.11, 0.22, 0.33, 0.44, 0.55)]
        ranges, angle_min, angle_increment = _scan_for_points(wall_points)
        config = PolePatternConfig(
            line_cluster_jump_threshold=0.14,
            line_min_points=4,
            line_min_length=0.25,
            line_max_length=0.55,
            line_max_width=0.04,
            line_anchor_cluster_jump_threshold=0.03,
            line_min_anchor_count=4,
            line_group_max_anchor_gap=0.15,
            line_hypothesis_lateral_tolerance=0.04,
            line_isolation_enabled=True,
            line_isolation_lateral_tolerance=0.08,
            line_isolation_extension=0.18,
        )

        features = detect_line_features_from_scan(
            ranges,
            angle_min,
            angle_increment,
            range_min=0.05,
            range_max=10.0,
            config=config,
        )

        self.assertEqual(features, [])


def _scan_for_points(points):
    angle_min = -0.40
    angle_increment = 0.01
    ranges = [float("inf")] * 81
    for x, y in points:
        bearing = math.atan2(y, x)
        index = round((bearing - angle_min) / angle_increment)
        ranges[index] = math.hypot(x, y)
    return ranges, angle_min, angle_increment


def _pole(index, x, y):
    return PoleDetection(
        index=index,
        x=x,
        y=y,
        range=math.hypot(x, y),
        bearing=math.atan2(y, x),
        width=0.02,
        point_count=3,
        scan_indices=(index,),
    )


if __name__ == "__main__":
    unittest.main()
