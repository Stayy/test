import math
import unittest

from scan_feature_matcher.patterns import (
    PoleDetection,
    PolePatternConfig,
    detect_fence_patterns,
    detect_fences_from_scan,
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


def _scan_for_points(points):
    angle_min = -0.14
    angle_increment = 0.01
    ranges = [float("inf")] * 29
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
