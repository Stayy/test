import math
import unittest

from scan_feature_matcher.patterns import LineFeatureDetection
from scan_feature_matcher.tracking import (
    LineFeatureTracker,
    LineFeatureTrackingConfig,
)


class LineFeatureTrackerTest(unittest.TestCase):
    def test_confirms_after_required_consecutive_hits(self):
        tracker = LineFeatureTracker(
            LineFeatureTrackingConfig(confirmations_required=2)
        )

        first = tracker.update([_line(0.0, 0.0)])
        second = tracker.update([_line(0.02, 0.0)])

        self.assertEqual(first, [])
        self.assertEqual(len(second), 1)
        self.assertAlmostEqual(second[0].center_x, 0.011)

    def test_holds_confirmed_detection_through_short_miss(self):
        tracker = LineFeatureTracker(
            LineFeatureTrackingConfig(confirmations_required=1, hold_frames=2)
        )
        tracker.update([_line(0.0, 0.0)])

        first_miss = tracker.update([])
        second_miss = tracker.update([])
        third_miss = tracker.update([])

        self.assertEqual(len(first_miss), 1)
        self.assertEqual(len(second_miss), 1)
        self.assertEqual(third_miss, [])

    def test_discards_one_frame_false_positive(self):
        tracker = LineFeatureTracker(
            LineFeatureTrackingConfig(confirmations_required=2)
        )

        first = tracker.update([_line(0.0, 0.0)])
        second = tracker.update([])

        self.assertEqual(first, [])
        self.assertEqual(second, [])


def _line(x, y):
    length = 0.4
    yaw = 0.1
    return LineFeatureDetection(
        center_x=x,
        center_y=y,
        yaw=yaw,
        length=length,
        width=0.02,
        point_count=5,
        score=0.01,
        scan_indices=(1, 2, 3, 4, 5),
        endpoints=(
            (x - math.cos(yaw) * length * 0.5, y - math.sin(yaw) * length * 0.5),
            (x + math.cos(yaw) * length * 0.5, y + math.sin(yaw) * length * 0.5),
        ),
    )


if __name__ == "__main__":
    unittest.main()
