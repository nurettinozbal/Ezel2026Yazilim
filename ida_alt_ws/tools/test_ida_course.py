"""Course generator geometry regression tests (ROS2 dependency-free)."""

import math
import random
import unittest

from ida_course import CourseSchema, _segments_intersect, min_clearance


def _orientation(a, b, c):
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def _properly_intersects(a, b, c, d):
    """True only for a proper crossing; touching endpoints is permitted."""
    return _orientation(a, b, c) * _orientation(a, b, d) < 0.0 and _orientation(
        c, d, a
    ) * _orientation(c, d, b) < 0.0


class CoursePairGeometryTests(unittest.TestCase):
    def setUp(self):
        self.schema = CourseSchema()
        self.segments = self.schema._build_segments(
            self.schema.p1_corners + [self.schema.p2_end]
        )
        total_m = sum(segment.length_m() for segment in self.segments)
        self.pairs = self.schema._walk_pairs(
            self.segments, total_m, random.Random(2026)
        )

    def test_pair_centers_stay_within_configured_spacing(self):
        gaps = [b.t - a.t for a, b in zip(self.pairs, self.pairs[1:])]
        self.assertTrue(gaps)
        for gap in gaps:
            self.assertGreaterEqual(gap, self.schema.pair_min_gap_m - 1e-7)
            self.assertLessEqual(gap, self.schema.pair_max_gap_m + 1e-7)

    def test_each_turn_has_one_bisector_pair(self):
        corners = self.schema.p1_corners[1:] + [self.schema.p2_end]
        for corner in corners:
            matches = [pair for pair in self.pairs if math.dist(pair.center, corner) < 1e-7]
            self.assertEqual(len(matches), 1, f"corner pair mismatch at {corner}")

        for index, corner in enumerate(self.schema.p1_corners[1:], start=0):
            pair = next(pair for pair in self.pairs if math.dist(pair.center, corner) < 1e-7)
            expected_n = self.schema._corner_normal(
                self.segments[index], self.segments[index + 1]
            )
            actual_n = (
                (pair.left[0] - pair.center[0]) / self.schema.half_width,
                (pair.left[1] - pair.center[1]) / self.schema.half_width,
            )
            self.assertAlmostEqual(actual_n[0], expected_n[0], places=7)
            self.assertAlmostEqual(actual_n[1], expected_n[1], places=7)

    def test_left_and_right_boundaries_do_not_cross(self):
        self.assertEqual(self.schema._pair_geometry_error(self.pairs), "")
        for current in self.pairs:
            self.assertAlmostEqual(
                math.dist(current.left, current.right),
                self.schema.corridor_width_m,
                places=7,
            )

    def test_default_turn_buoys_are_not_spawned_dip_to_dip(self):
        # 1 m çaplı sim dubaları için merkezler arasında en az 2 m bırakılır.
        all_buoys = [point for pair in self.pairs for point in (pair.left, pair.right)]
        closest = min(
            math.dist(a, b)
            for i, a in enumerate(all_buoys)
            for b in all_buoys[i + 1 :]
        )
        self.assertGreaterEqual(closest, 2.0)

    def test_seed_is_deterministic_and_can_change_spacing(self):
        total_m = sum(segment.length_m() for segment in self.segments)
        same = self.schema._walk_pairs(
            self.segments, total_m, random.Random(2026)
        )
        other = self.schema._walk_pairs(
            self.segments, total_m, random.Random(2027)
        )
        self.assertEqual(self.pairs, same)
        self.assertNotEqual([pair.t for pair in self.pairs], [pair.t for pair in other])

    def test_default_geometry_and_yellow_clearance_hold_across_seeds(self):
        for seed in range(100):
            data = self.schema.generate(seed)
            yellow = [buoy for buoy in data.buoys if buoy["color"] == "yellow"]
            self.assertEqual(len(yellow), 10, f"seed={seed}")
            self.assertGreaterEqual(min_clearance(data), 2.0 - 1e-7, f"seed={seed}")

    def _pairs_for_path(self, path, end, seed=2026):
        schema = CourseSchema(
            {"p1_corners": path, "p2_end": end, "p2_yellow_count": 0}
        )
        segments = schema._build_segments(path + [end])
        total_m = sum(segment.length_m() for segment in segments)
        return schema, schema._walk_pairs(segments, total_m, random.Random(seed))

    def _assert_full_geometry(self, schema, pairs):
        self.assertEqual(schema._pair_geometry_error(pairs), "")
        points = [point for pair in pairs for point in (pair.left, pair.right)]
        rounded = {(round(point[0], 7), round(point[1], 7)) for point in points}
        self.assertEqual(len(points), len(rounded), "duplicate buoy coordinate")

        boundaries = []
        for side in ("left", "right"):
            boundaries.extend(
                (side, index, getattr(a, side), getattr(b, side))
                for index, (a, b) in enumerate(zip(pairs, pairs[1:]))
            )
        for edge_index, (side_a, index_a, a, b) in enumerate(boundaries):
            for side_b, index_b, c, d in boundaries[edge_index + 1 :]:
                if side_a == side_b and abs(index_a - index_b) <= 1:
                    continue
                self.assertFalse(
                    _segments_intersect(a, b, c, d),
                    f"boundary crossing {side_a}:{index_a} {side_b}:{index_b}",
                )

        for gate_index, pair in enumerate(pairs):
            for side, edge_index, c, d in boundaries:
                if edge_index in (gate_index - 1, gate_index):
                    continue
                self.assertFalse(
                    _segments_intersect(pair.left, pair.right, c, d),
                    f"gate {gate_index} crosses {side}:{edge_index}",
                )

    def test_left_and_right_90_degree_turns_have_unique_safe_buoys(self):
        cases = (
            ([[0, 0], [20, 0], [20, 20]], [40, 20]),
            ([[0, 0], [20, 0], [20, -20]], [40, -20]),
        )
        for path, end in cases:
            schema, pairs = self._pairs_for_path(path, end)
            self._assert_full_geometry(schema, pairs)
            self.assertNotIn((15.0, 5.0), {
                (round(point[0], 7), round(point[1], 7))
                for pair in pairs
                for point in (pair.left, pair.right)
            })

    def test_obtuse_turn_has_no_non_adjacent_crossing(self):
        schema, pairs = self._pairs_for_path(
            [[0, 0], [20, 0], [30, 15]], [30, 35]
        )
        self._assert_full_geometry(schema, pairs)

    def test_overlapping_acute_and_near_u_corridors_are_rejected(self):
        impossible = (
            ([[0, 0], [20, 0], [5, 5]], [5, 25]),
            ([[0, 0], [20, 0], [1, 3]], [1, 23]),
        )
        for path, end in impossible:
            with self.assertRaisesRegex(ValueError, "kesişmeyen duba geometrisi"):
                self._pairs_for_path(path, end)

    def test_impossible_yellow_density_is_an_explicit_error(self):
        schema = CourseSchema(
            {
                "p1_corners": [[0, 0], [20, 0], [20, 20]],
                "p2_end": [40, 20],
                "p2_yellow_count": 10,
            }
        )
        with self.assertRaisesRegex(ValueError, "10 sarı duba yerleştirilemedi"):
            schema.generate(2026)


if __name__ == "__main__":
    unittest.main()
