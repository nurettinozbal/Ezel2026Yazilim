import unittest

from fusion_health_summary import summarize


class TestFusionHealthSummary(unittest.TestCase):
    def test_aggregate_only(self) -> None:
        result = summarize(
            {"source_health": "fusion_not_ready", "matched_count": 0},
            {"detections": []},
            {
                "obstacles": [
                    {"id": "private-a", "hard_obstacle": True, "color": "unknown"},
                    {"id": "private-b", "hard_obstacle": True, "color": "orange"},
                ]
            },
        )
        self.assertEqual(result["colored_buoy_count"], 0)
        self.assertEqual(result["obstacle_count"], 2)
        self.assertEqual(result["hard_obstacle_count"], 2)
        self.assertEqual(result["unknown_obstacle_count"], 1)
        self.assertNotIn("private-a", str(result))


if __name__ == "__main__":
    unittest.main()
