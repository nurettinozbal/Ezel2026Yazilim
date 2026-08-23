"""
system_check_node
──────────────────
idaws start ile birlikte çalışır. Diğer node'ların başlangıç logları bir kere
yazılıp log akışında kaybolur — "çalışıyor" yazması ile gerçekten veri akması
aynı şey değildir (ör. Pixhawk bağlanır ama LiDAR sessiz kalabilir). Bu node
Pixhawk/LiDAR/kamera/görev alt sistemlerinin GERÇEKTEN veri ürettiğini
periyodik olarak tek satırda özetler, böylece terminalde her an göz atıp
"şu an gerçekten çalışıyor mu" sorusuna cevap bulunabilir.
"""

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan, Image

STALE_SEC = 3.0  # bu süreden eski veri "kesildi" sayılır


class SystemCheckNode(Node):

    def __init__(self):
        super().__init__('system_check_node')

        self.declare_parameter('report_period_sec', 5.0)
        period = max(1.0, float(self.get_parameter('report_period_sec').value))

        self._last = {'pixhawk': None, 'lidar': None, 'camera': None}
        self._lidar_points = 0
        self._mode = '?'
        self._phase = '?'

        # /scan BEST_EFFORT yayınlanıyor (scan_filter_node); vision/image_annotated
        # da öyle (yolo_vision_node) — RELIABLE ile abone olursak QoS uyuşmazlığından
        # sessizce hiç veri almayız (bu projede daha önce iki kez bu yüzden yanlış
        # teşhis konuldu).
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(String, 'telemetry/heartbeat_status', self._cb_pixhawk, 10)
        self.create_subscription(String, 'telemetry/mode', self._cb_mode, 10)
        self.create_subscription(LaserScan, '/scan', self._cb_lidar, sensor_qos)
        self.create_subscription(Image, 'vision/image_annotated', self._cb_camera, image_qos)
        self.create_subscription(String, 'mission/phase', self._cb_phase, 10)

        self.create_timer(period, self._report)
        self.get_logger().info(f'system_check_node başlatıldı — durum her {period:.0f} sn\'de bir yazılır.')

    def _cb_pixhawk(self, msg):
        self._last['pixhawk'] = time.monotonic()

    def _cb_mode(self, msg):
        self._mode = msg.data

    def _cb_lidar(self, msg):
        self._last['lidar'] = time.monotonic()
        self._lidar_points = sum(1 for r in msg.ranges if msg.range_min < r < msg.range_max)

    def _cb_camera(self, msg):
        self._last['camera'] = time.monotonic()

    def _cb_phase(self, msg):
        self._phase = msg.data

    def _fresh(self, key):
        t = self._last[key]
        return t is not None and (time.monotonic() - t) < STALE_SEC

    def _report(self):
        pixhawk_ok = self._fresh('pixhawk')
        lidar_ok = self._fresh('lidar')
        camera_ok = self._fresh('camera')

        def mark(ok):
            return '✓' if ok else '✗'

        line = (
            f"{mark(pixhawk_ok)} Pixhawk(mod={self._mode})   "
            f"{mark(lidar_ok)} LiDAR({self._lidar_points} nokta)   "
            f"{mark(camera_ok)} Kamera   "
            f"| Görev: {self._phase}"
        )
        if pixhawk_ok and lidar_ok:
            self.get_logger().info(line)
        else:
            self.get_logger().warn(line)


def main(args=None):
    rclpy.init(args=args)
    node = SystemCheckNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
