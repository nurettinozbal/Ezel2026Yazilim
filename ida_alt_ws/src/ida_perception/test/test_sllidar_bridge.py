"""sllidar_bridge saf modül testleri (rclpy'siz).

laserscan_to_points: 360° tarama -> nokta listesi; inf/NaN, range filtreleri,
angle_offset, uzunluk uyuşmazlığı ve negatif açı normalize'i.
scan_to_obstacles: tam zincir — dolu scan -> kontrat formatı, boş scan -> [].
"""

import math
import unittest

from ida_perception.sllidar_bridge import (
    inside_front_fov,
    laserscan_to_points,
    obstacles_to_raw_clusters,
    points_to_raw_body_contract,
    scan_to_obstacles,
    transform_obstacles_to_base,
    transform_scan_angle_deg,
)


def make_scan(
    num_points: int = 1023,
    angle_min: float = -math.pi,
    angle_max: float = math.pi,
    range_min: float = 0.2,
    range_max: float = 18.0,
    ranges=None,
) -> dict:
    """Test için sensor_msgs/LaserScan formatında dict üretir."""
    if ranges is None:
        ranges = [5.0] * num_points
    increment = (angle_max - angle_min) / num_points if num_points else 0.0
    return {
        "angle_min": angle_min,
        "angle_max": angle_max,
        "angle_increment": increment,
        "ranges": ranges,
        "range_min": range_min,
        "range_max": range_max,
    }


class TestLaserscanToPoints(unittest.TestCase):
    def test_full_360_scan(self) -> None:
        # 360° tarama, her açıda bir nokta -> 1023 nokta korunur.
        points = laserscan_to_points(make_scan())
        self.assertEqual(len(points), 1023)
        # -π..+π ([-180°, 180°)) normalize sonrası 0..360 tam kapsama:
        # tüm açılar geçerli aralıkta ve yayılım ≈ 360°.
        angles = sorted(a for a, _ in points)
        self.assertTrue(all(0.0 <= a <= 360.0 for a in angles))
        self.assertGreater(angles[-1] - angles[0], 359.0)

    def test_offset_zero_points_forward(self) -> None:
        # angle_min=-π, offset 0: 0° (araç ileri yönü) ölçümü 0° civarında kalır
        # (1023 nokta -> çözünürlük ~0.35°, en yakın örnek 0.18°).
        points = laserscan_to_points(make_scan())
        self.assertTrue(any(abs(a) < 0.5 or abs(a - 360.0) < 0.5 for a, _ in points))

    def test_angle_offset_plus_90(self) -> None:
        # +90° offset: 0° ölçümü 90°'ye (sağ) kayar.
        scan = make_scan()
        points = laserscan_to_points(scan, angle_offset_deg=90.0)
        zero_index_angle = math.degrees(scan["angle_min"] + 512 * scan["angle_increment"])
        self.assertAlmostEqual(zero_index_angle, 0.0, delta=0.5)
        self.assertTrue(any(abs(a - 90.0) < 0.5 for a, _ in points))

    def test_mount_yaw_mirror_and_front_sector(self) -> None:
        self.assertEqual(transform_scan_angle_deg(0.0, 180.0, False), 180.0)
        self.assertEqual(transform_scan_angle_deg(30.0, 180.0, True), 150.0)
        self.assertTrue(inside_front_fov(79.0, 160.0))
        self.assertTrue(inside_front_fov(281.0, 160.0))  # -79 deg
        self.assertFalse(inside_front_fov(81.0, 160.0))
        self.assertFalse(inside_front_fov(180.0, 160.0))

    def test_front_sector_filters_rear_after_mount_rotation(self) -> None:
        # Raw 180 deg is this vehicle's bow; +180 mounting yaw maps it to 0.
        ranges = [float("inf")] * 360
        ranges[0] = 2.0      # raw -180 deg -> vehicle 0 deg (keep)
        ranges[180] = 3.0    # raw 0 deg -> vehicle 180 deg (reject)
        points = laserscan_to_points(
            make_scan(num_points=360, ranges=ranges),
            angle_offset_deg=180.0,
            front_fov_deg=160.0,
        )
        self.assertEqual(len(points), 1)
        self.assertAlmostEqual(points[0][0], 0.0, delta=1e-6)

    def test_inf_nan_filtered(self) -> None:
        ranges = [5.0] * 10
        ranges[2] = float("inf")
        ranges[4] = float("nan")
        scan = make_scan(num_points=10, ranges=ranges)
        points = laserscan_to_points(scan)
        self.assertEqual(len(points), 8)

    def test_nonpositive_filtered(self) -> None:
        ranges = [5.0, 0.0, -1.0, 5.0]
        points = laserscan_to_points(make_scan(num_points=4, ranges=ranges))
        self.assertEqual(len(points), 2)

    def test_range_min_max_filter(self) -> None:
        ranges = [0.1, 5.0, 20.0, 5.0]  # range_min 0.2 altı ve range_max 18 üstü elenir
        points = laserscan_to_points(
            make_scan(num_points=4, ranges=ranges, range_min=0.2, range_max=18.0)
        )
        self.assertEqual(len(points), 2)

    def test_param_filters(self) -> None:
        # min_distance_m ve max_distance_m parametreleri de uygulanır.
        ranges = [0.2, 3.0, 15.0, 25.0]
        points = laserscan_to_points(
            make_scan(num_points=4, ranges=ranges),
            min_distance_m=1.0,
            max_distance_m=20.0,
        )
        self.assertEqual(len(points), 2)

    def test_empty_ranges_no_exception(self) -> None:
        points = laserscan_to_points(make_scan(num_points=0, ranges=[]))
        self.assertEqual(points, [])

    def test_length_mismatch_no_exception(self) -> None:
        # Increment 360/1023 ama ranges 100 eleman: istisna yok, 100 nokta.
        scan = make_scan(num_points=100, ranges=[5.0] * 100)
        points = laserscan_to_points(scan)
        self.assertEqual(len(points), 100)

    def test_negative_angle_normalized(self) -> None:
        # angle_min=-π, index 0 -> -180° -> 180° (0..360 normalize).
        scan = make_scan(num_points=8)
        points = laserscan_to_points(scan)
        angles = [a for a, _ in points]
        self.assertTrue(all(0.0 <= a <= 360.0 for a in angles))
        self.assertTrue(any(abs(a - 180.0) < 1e-6 for a in angles))

    def test_missing_fields_empty(self) -> None:
        self.assertEqual(laserscan_to_points({}), [])
        self.assertEqual(laserscan_to_points({"angle_min": 0.0}), [])

    def test_raw_body_points_apply_offset_sign_and_even_limit(self) -> None:
        result = points_to_raw_body_contract(
            [(0.0, 2.0), (90.0, 1.0), (180.0, 2.0), (270.0, 1.0)],
            sensor_forward_offset_m=0.57,
            sensor_lateral_right_offset_m=0.1,
            limit=2,
        )
        self.assertEqual(len(result), 2)
        self.assertAlmostEqual(result[0]["forward_m"], 2.57)
        self.assertAlmostEqual(result[0]["lateral_left_m"], -0.1)
        self.assertAlmostEqual(result[1]["forward_m"], -1.43)
        self.assertAlmostEqual(result[1]["lateral_left_m"], -0.1)

    def test_raw_body_points_reject_invalid_limit(self) -> None:
        with self.assertRaises(ValueError):
            points_to_raw_body_contract([], limit=0)


class TestScanToObstacles(unittest.TestCase):
    def test_full_scan_single_cluster_forward(self) -> None:
        # Tüm mesafeler 5.0 m: 360° boyunca tek halka -> 0° civarı engel kümesi.
        scan = make_scan()
        obstacles = scan_to_obstacles(
            scan,
            angle_offset_deg=0.0,
            lidar_range_m=18.0,
            min_distance_m=0.3,
            cluster_gap_m=0.35,
            min_cluster_points=3,
            max_angle_gap_deg=8.0,
            stamp=123.0,
        )
        self.assertIsInstance(obstacles, list)
        for obs in obstacles:
            self.assertIn("distance", obs)
            self.assertIn("forward_m", obs)
            self.assertIn("lateral_m", obs)
            self.assertIn("width_m", obs)
            self.assertEqual(obs["stamp"], 123.0)

    def test_empty_scan_no_obstacles(self) -> None:
        scan = make_scan(num_points=0, ranges=[])
        obstacles = scan_to_obstacles(
            scan,
            angle_offset_deg=0.0,
            lidar_range_m=18.0,
            min_distance_m=0.3,
            cluster_gap_m=0.35,
            min_cluster_points=3,
            max_angle_gap_deg=8.0,
            stamp=0.0,
        )
        self.assertEqual(obstacles, [])

    def test_inf_scan_no_obstacles(self) -> None:
        # Tamamı inf: nokta yok -> engel yok.
        scan = make_scan(num_points=50, ranges=[float("inf")] * 50)
        obstacles = scan_to_obstacles(
            scan,
            angle_offset_deg=0.0,
            lidar_range_m=18.0,
            min_distance_m=0.3,
            cluster_gap_m=0.35,
            min_cluster_points=3,
            max_angle_gap_deg=8.0,
            stamp=0.0,
        )
        self.assertEqual(obstacles, [])

    def test_forward_cluster_detected(self) -> None:
        # 0° civarı kısa bir blok: nokta az olduğundan min_cluster altı kalabilir;
        # 5'er derecelik 30 noktalık blok tek engel olarak çıkar.
        ranges = [float("inf")] * 200
        # 200 nokta / 360° -> 1.8°/nokta; 0° merkezli ~40 nokta (=-36°..+36°).
        for i in range(80, 120):
            ranges[i] = 5.0
        scan = make_scan(num_points=200, ranges=ranges)
        obstacles = scan_to_obstacles(
            scan,
            angle_offset_deg=0.0,
            lidar_range_m=18.0,
            min_distance_m=0.3,
            cluster_gap_m=0.35,
            min_cluster_points=3,
            max_angle_gap_deg=8.0,
            stamp=0.0,
        )
        self.assertTrue(any(obs["forward_m"] > 4.0 for obs in obstacles))


class TestRawClusters(unittest.TestCase):
    def test_sensor_offset_translates_clusters_to_vehicle_centre(self) -> None:
        source = [{
            "distance": math.hypot(2.0, 1.0),
            "forward_m": 2.0,
            "lateral_m": 1.0,
            "width_m": 0.25,
            "stamp": 12.0,
        }]

        transformed = transform_obstacles_to_base(
            source,
            sensor_forward_offset_m=0.57,
            sensor_lateral_right_offset_m=0.0,
        )

        self.assertEqual(source[0]["forward_m"], 2.0)
        self.assertAlmostEqual(transformed[0]["forward_m"], 2.57)
        self.assertAlmostEqual(transformed[0]["lateral_m"], 1.0)
        self.assertAlmostEqual(transformed[0]["distance"], math.hypot(2.57, 1.0))
        self.assertEqual(transformed[0]["width_m"], 0.25)
        self.assertEqual(transformed[0]["stamp"], 12.0)

    def test_sensor_offset_rejects_nonfinite_configuration(self) -> None:
        with self.assertRaises(ValueError):
            transform_obstacles_to_base([], sensor_forward_offset_m=float("nan"))

    def test_left_positive_conversion_and_deterministic_ids(self) -> None:
        obstacles = [
            {"distance": 5.1, "forward_m": 5.0, "lateral_m": 1.0, "width_m": 0.2},
            {"distance": 4.1, "forward_m": 4.0, "lateral_m": -1.0, "width_m": 0.3},
        ]

        clusters = obstacles_to_raw_clusters(obstacles)
        reversed_clusters = obstacles_to_raw_clusters(list(reversed(obstacles)))

        self.assertEqual(clusters, reversed_clusters)
        self.assertEqual([item["id"] for item in clusters], ["s2_cluster_000", "s2_cluster_001"])
        self.assertEqual(clusters[0]["lateral_left_m"], 1.0)
        self.assertEqual(clusters[1]["lateral_left_m"], -1.0)
        self.assertNotIn("lateral_m", clusters[0])


if __name__ == "__main__":
    unittest.main()
