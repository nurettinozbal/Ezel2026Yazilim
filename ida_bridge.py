import os
os.environ["MAVLINK20"] = "1"

import math
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

from pymavlink import mavutil

from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy
)


class IdaBridge(Node):

    def __init__(self):

        super().__init__('ida_bridge')

        # ====================================================
        # LOW LATENCY QoS
        # ====================================================

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # ====================================================
        # MOTOR KOMUTLARI
        # ====================================================

        self.cmd_subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            qos_profile
        )

        # ====================================================
        # LiDAR /scan
        # 360 derece taramayı doğrudan MAVLink OBSTACLE_DISTANCE
        # mesajına dönüştüreceğiz.
        # ====================================================

        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile
        )

        self.lidar_min_cm = 10
        self.lidar_max_cm = 1200

        # OBSTACLE_DISTANCE 72 eleman taşır.
        # 72 x 5 derece = 360 derece.
        self.lidar_increment_deg = 5
        self.lidar_bin_count = 72

        # max_distance + 1 = bu açıda engel yok.
        self.latest_obstacle_distances = [
            self.lidar_max_cm + 1
        ] * self.lidar_bin_count

        self.last_scan_time = 0.0

        # ====================================================
        # PIXHAWK MAVLINK
        # ====================================================

        self.get_logger().info(
            "Pixhawk baglantisi bekleniyor..."
        )

        self.master = mavutil.mavlink_connection(
            '/dev/ttyACM0',
            baud=115200
        )

        self.master.wait_heartbeat()

        self.get_logger().info(
            "Pixhawk Heartbeat alindi."
        )

        self.get_logger().info(
            "Sistem Hazir: Motor + 360 Derece LiDAR MAVLink Aktarimi Aktif!"
        )

        # ====================================================
        # FAILSAFE
        # ====================================================

        # RC override durumu: script cmd_vel aliyor mu, yoksa
        # kontrol kumandada mi? Baslangicta override YOK,
        # yani kontrol kumandada.
        self.override_active = False

        self.last_msg_time = self.get_clock().now()

        self.create_timer(
            0.1,
            self.failsafe_callback
        )

        # LiDAR verisini 5 Hz MAVLink'e gönder.
        self.create_timer(
            0.2,
            self.send_obstacle_distance
        )

    # ========================================================
    # MOTOR / RC OVERRIDE
    # ========================================================

    def set_rc(self, throttle, steering):

        rc_values = [65535] * 8

        rc_values[0] = steering
        rc_values[1] = throttle

        self.master.mav.rc_channels_override_send(
            self.master.target_system,
            self.master.target_component,
            *rc_values
        )

        self.override_active = True

    def release_rc(self):
        """
        Tum override kanallarini serbest birak (0 = release).
        Bu, ArduPilot'a "bu kanal icin RC receiver/kumanda
        degerini kullan" der; kontrol fiziksel kumandaya doner.
        """

        self.master.mav.rc_channels_override_send(
            self.master.target_system,
            self.master.target_component,
            *([0] * 8)
        )

        self.override_active = False

    # ========================================================
    # CMD_VEL
    # ========================================================

    def cmd_vel_callback(self, msg):

        self.last_msg_time = self.get_clock().now()

        throttle = int(
            1500 + (msg.linear.x * 250)
        )

        steering = int(
            1500 - (msg.angular.z * 200)
        )

        throttle = max(
            1300,
            min(1700, throttle)
        )

        steering = max(
            1300,
            min(1700, steering)
        )

        self.set_rc(
            throttle,
            steering
        )

    # ========================================================
    # LASERSCAN -> 72 AÇISAL DİLİM
    # ========================================================

    def scan_callback(self, msg):

        # Her yeni taramada tüm dilimleri "engel yok" olarak başlat.
        bins = [
            self.lidar_max_cm + 1
        ] * self.lidar_bin_count

        for i, distance_m in enumerate(msg.ranges):

            if not math.isfinite(distance_m):
                continue

            if distance_m < max(msg.range_min, self.lidar_min_cm / 100.0):
                continue

            if distance_m > min(msg.range_max, self.lidar_max_cm / 100.0):
                continue

            # ROS LaserScan açısı:
            # pozitif yön genellikle saat yönünün tersidir (sol).
            ros_angle_rad = (
                msg.angle_min
                + (i * msg.angle_increment)
            )

            ros_angle_deg = math.degrees(
                ros_angle_rad
            )

            # MAV_FRAME_BODY_FRD / radar gösterimimiz:
            # 0 = ileri
            # pozitif = sağ / saat yönü
            mav_angle_deg = (
                -ros_angle_deg
            ) % 360.0

            bin_index = int(
                mav_angle_deg
                // self.lidar_increment_deg
            ) % self.lidar_bin_count

            distance_cm = int(
                round(distance_m * 100.0)
            )

            distance_cm = max(
                self.lidar_min_cm,
                min(
                    self.lidar_max_cm,
                    distance_cm
                )
            )

            # Aynı 5 derecelik dilimde birden fazla ölçüm varsa
            # en yakın engeli sakla.
            if (
                bins[bin_index] > self.lidar_max_cm
                or distance_cm < bins[bin_index]
            ):
                bins[bin_index] = distance_cm

        self.latest_obstacle_distances = bins
        self.last_scan_time = time.monotonic()

    # ========================================================
    # 360 LiDAR -> MAVLink OBSTACLE_DISTANCE
    # ========================================================

    def send_obstacle_distance(self):

        # Son 1 saniyede /scan gelmediyse eski LiDAR verisini gönderme.
        if (
            self.last_scan_time == 0.0
            or (time.monotonic() - self.last_scan_time) > 1.0
        ):
            return

        try:

            self.master.mav.obstacle_distance_send(
                int(time.time() * 1_000_000),

                # Sensör tipi
                mavutil.mavlink.MAV_DISTANCE_SENSOR_LASER,

                # 72 adet uint16 mesafe, cm
                self.latest_obstacle_distances,

                # Açısal artış (derece)
                self.lidar_increment_deg,

                # min_distance (cm)
                self.lidar_min_cm,

                # max_distance (cm)
                self.lidar_max_cm,

                # increment_f
                float(self.lidar_increment_deg),

                # angle_offset
                0.0,

                # Tekne gövdesine göre:
                # X ileri, Y sağ, Z aşağı
                mavutil.mavlink.MAV_FRAME_BODY_FRD
            )

        except Exception as e:

            self.get_logger().error(
                f"OBSTACLE_DISTANCE gonderme hatasi: {e}"
            )

    # ========================================================
    # FAILSAFE
    # ========================================================

    def failsafe_callback(self):

        # Override aktif degilse yapacak bir sey yok,
        # kontrol zaten kumandada.
        if not self.override_active:
            return

        elapsed = (
            self.get_clock().now()
            - self.last_msg_time
        ).nanoseconds

        if elapsed > 500_000_000:

            # cmd_vel 500ms'den uzun suredir gelmiyor:
            # override'i tamamen birak, kontrol kumandaya donsun.
            self.release_rc()

            self.get_logger().warn(
                "cmd_vel zaman asimi: RC override birakildi, "
                "kontrol kumandaya devredildi."
            )


def main(args=None):

    rclpy.init(args=args)

    node = IdaBridge()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        try:
            node.release_rc()
        except Exception:
            pass

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':
    main()
