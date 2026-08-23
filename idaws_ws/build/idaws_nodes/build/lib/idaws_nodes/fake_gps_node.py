"""
fake_gps_node
─────────────
SADECE bench/kapalı alan testi içindir. Kapalı alanda/masa üstünde gerçek GPS
fix'i olmadan EKF origin/HOME_POSITION hiç oluşmaz, bu yüzden GUIDED modda
SET_POSITION_TARGET_LOCAL_NED komutları (bkz. pymavlink_controller_node.
_cb_velocity/_cb_setpoint_ned) otopilot tarafından sessizce reddedilir —
hiçbir hedefe hareket edilmez.

Bu node ham MAVLink bağlantısı AÇMAZ (Pixhawk'ın seri portu TEK bir bağlantı
kabul eder, pymavlink_controller_node zaten onu tutuyor). Bunun yerine sabit
bir konumu cmd/fake_gps (NavSatFix) topic'ine yayınlar; pymavlink_controller_
node buna abonedir ve her mesajı MAVLink GPS_INPUT'e çevirip Pixhawk'a
GÖNDERİR (bkz. pymavlink_controller_node._cb_fake_gps) — cmd/velocity/cmd/
setpoint_ned ile AYNI "ROS topic → node çevirir → MAVLink" deseni.

ÖN KOŞUL: otopilotta GPS_TYPE=14 (MAV) yer istasyonundan (Mission Planner/
QGC) BİR KEZ elle ayarlanmış olmalı — aksi hâlde pymavlink_controller_node'un
gönderdiği GPS_INPUT mesajları otopilot tarafından sessizce yok sayılır. Bu
node o parametreyi YAZMAZ.

⚠ SADECE bench/kapalı alan testi içindir — su üzerinde ASLA çalıştırmayın:
araç fiilen hareket ederken GPS'i sabit bir noktaya kilitlemek EKF'yi gerçek
konumdan koparır. Bu yüzden idaws_launch.py / sim_launch.py'ye DAHİL
EDİLMEDİ — yalnızca elle çalıştırılır:

    ros2 run idaws_nodes fake_gps_node --ros-args -p lat:=51.566151 -p lon:=-4.034345
"""

import random

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

# idaws_bringup/sim/scripts/start_sim.sh CUSTOM_LOCATION ile aynı — sim'de
# zaten kullanılan bir konum, yeni bir yer ezberlemeye gerek kalmasın diye.
DEFAULT_LAT = 51.566151
DEFAULT_LON = -4.034345
DEFAULT_ALT = 0.0
DEFAULT_HZ = 5.0
# Matematiksel olarak TAMAMEN sabit bir nokta gerçekçi değil — gerçek bir GPS
# alıcısı yerinde dursa bile birkaç cm-m titrer. EKF'nin glitch/innovation
# tutarlılık denetimi bu "fazla mükemmel" veriyi şüpheli bulup GPS/AHRS'i
# SYS_STATUS'ta "Bad" işaretleyebiliyor (sahada gözlendi: HUD "kilitli"
# gösteriyor ama SYS_STATUS GPS=Bad, AHRS=Bad çıkıyor, AUTO'ya bu yüzden
# reddediliyor) — bu yüzden her yayında küçük, gerçekçi bir gürültü eklenir.
JITTER_DEG_STD = 0.000003  # ~yaklaşık 30 cm standart sapma


class FakeGpsNode(Node):

    def __init__(self):
        super().__init__('fake_gps_node')

        self.declare_parameter('lat', DEFAULT_LAT)
        self.declare_parameter('lon', DEFAULT_LON)
        self.declare_parameter('alt', DEFAULT_ALT)
        self.declare_parameter('publish_hz', DEFAULT_HZ)

        self._lat = float(self.get_parameter('lat').value)
        self._lon = float(self.get_parameter('lon').value)
        self._alt = float(self.get_parameter('alt').value)
        hz = max(0.1, float(self.get_parameter('publish_hz').value))

        self.pub = self.create_publisher(NavSatFix, 'cmd/fake_gps', 10)
        self.create_timer(1.0 / hz, self._publish)

        self.get_logger().warn(
            f'⚠ fake_gps_node ÇALIŞIYOR — cmd/fake_gps üzerinden ({self._lat:.6f}, '
            f'{self._lon:.6f}) {hz:.0f} Hz\'de yayınlanıyor. pymavlink_controller_node '
            'bunu Pixhawk\'a GPS_INPUT olarak GÖNDERİR (GPS_TYPE=14 ayarlıysa kabul '
            'edilir). SADECE bench/kapalı alan testi içindir — SU ÜZERİNDEYSE HEMEN '
            'DURDURUN (Ctrl+C).'
        )

    def _publish(self):
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gps'
        msg.latitude = self._lat + random.gauss(0.0, JITTER_DEG_STD)
        msg.longitude = self._lon + random.gauss(0.0, JITTER_DEG_STD)
        msg.altitude = self._alt + random.gauss(0.0, 0.3)
        # status/covariance alanları pymavlink_controller_node._cb_fake_gps
        # tarafından okunmuyor — varsayılan (UNKNOWN) bırakılır.
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FakeGpsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # SIGINT'te rclpy kendi handler'ıyla context'i zaten kapatmış olabilir;
        # ikinci kez çağırmak RCLError fırlatır.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
