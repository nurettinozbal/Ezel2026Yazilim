"""gazebo_pose_sync_node — SITL telemetrisini Gazebo Classic model pose'una senkronlar.

ArduRover SITL (sim_vehicle.py) kendi fiziğini koşar; Gazebo'daki `ida_boat`
modeli gerçek konumu yalnızca İZLER. Bu node /telemetry/state (std_msgs/String
JSON, mavsdk_bridge'den gelir) dinler, lat/lon/heading'i Gazebo ENU world
frame'ine çevirir ve gazebo_msgs/SetEntityState servisiyle model pose'unu
yazar. Böylece modele bağlı kamera/lidar plugin'leri SITL ile senkron hareket
eder.

Koordinat sistemi (kritik):
  - Stack scenario koordinatları: x = kuzey, y = doğu (ida_planning.geo'daki
    latlon_to_local_m ile aynı mantık burada KENDİ İÇİNDE uygulanır — ida_control
    bağımsız kalsın, ida_planning importu yok).
  - Gazebo world frame: x = doğu, y = kuzey (ENU). Dönüşüm:
    Gazebo(x, y) = scenario(y_east, x_north).
  - Yaw: heading 0 = kuzey = Gazebo +y; yaw = radians(90 - heading_deg).
"""

import json
import math
from typing import Optional

import rclpy
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import SetEntityState
from rclpy.node import Node
from std_msgs.msg import String

# Stack scenario koordinat sistemiyle birebir aynı (ida_planning.geo):
EARTH_RADIUS_M = 6371000.0


class GazeboPoseSyncNode(Node):
    """SITL /telemetry/state -> Gazebo SetEntityState senkronizasyon node'u."""

    def __init__(self) -> None:
        super().__init__("ida_gazebo_pose_sync")
        self.declare_parameter("model_name", "ida_boat")
        self.declare_parameter("origin_lat", 40.8630501)
        self.declare_parameter("origin_lon", 29.2599517)
        self.declare_parameter("publish_hz", 20.0)
        self.declare_parameter("z_m", 0.3)

        self.model_name = str(self.get_parameter("model_name").value)
        self.origin_lat = float(self.get_parameter("origin_lat").value)
        self.origin_lon = float(self.get_parameter("origin_lon").value)
        publish_hz = float(self.get_parameter("publish_hz").value)
        self.z_m = float(self.get_parameter("z_m").value)

        # En son gelen SITL telemetrisi (ilk saniyeler boş olabilir).
        self._telemetry: Optional[dict] = None

        self.create_subscription(String, "/telemetry/state", self.on_telemetry, 20)

        # gazebo_ros_state plugin'i model_states + set_entity_state servislerini
        # expose eder. gazebo_ros_state plugin'inde servis /gazebo/set_entity_state
        # olarak yayınlanır (namespace=/gazebo); /set_entity_state (eski/başka
        # expose) çalışmayabilir — kullanıcı /gazebo/set_entity_state ile manuel
        # çağrıda modeli hareket ettirdi, o yüzden buna bağlanırız.
        self.cli = self.create_client(SetEntityState, "/gazebo/set_entity_state")

        self.create_timer(1.0 / max(0.1, publish_hz), self.tick)
        self.get_logger().info(
            f"Gazebo pose sync başladı (model={self.model_name}, "
            f"origin=({self.origin_lat:.6f}, {self.origin_lon:.6f}))"
        )

    def on_telemetry(self, msg: String) -> None:
        """/telemetry/state JSON'undan lat/lon/heading_deg alanlarını sakla."""
        try:
            payload = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        if not isinstance(payload, dict):
            return
        self._telemetry = payload

    def _latlon_to_local(self, lat: float, lon: float) -> tuple[float, float]:
        """lat/lon'u scenario koordinatlarına çevir: (x_north, y_east)."""
        dlat = math.radians(lat - self.origin_lat)
        dlon = math.radians(lon - self.origin_lon)
        x_north = dlat * EARTH_RADIUS_M
        y_east = dlon * EARTH_RADIUS_M * math.cos(math.radians(self.origin_lat))
        return x_north, y_east

    def tick(self) -> None:
        """Her tick'te son telemetriyi Gazebo ENU pose'una çevirip servise yaz."""
        if self._telemetry is None:
            return  # telemetri daha gelmedi (ilk saniyeler) — sessizce bekle
        try:
            lat = self._telemetry.get("lat")
            lon = self._telemetry.get("lon")
            if lat is None or lon is None:
                return  # eksik alan (farklı producer) — bu tick'i atla
            lat = float(lat)
            lon = float(lon)
            heading_deg = float(self._telemetry.get("heading_deg", 0.0))
        except (TypeError, ValueError):
            return
        if not math.isfinite(lat) or not math.isfinite(lon):
            return

        x_north, y_east = self._latlon_to_local(lat, lon)
        # ENU dönüşümü: scenario (x_north, y_east) -> Gazebo (x=y_east, y=x_north)
        gz_x = y_east
        gz_y = x_north
        yaw = math.radians(90.0 - heading_deg)

        state = EntityState()
        state.name = self.model_name
        state.pose.position.x = gz_x
        state.pose.position.y = gz_y
        state.pose.position.z = self.z_m
        # Euler (roll=0, pitch=0, yaw) -> quaternion.
        half_yaw = yaw / 2.0
        state.pose.orientation.w = math.cos(half_yaw)
        state.pose.orientation.x = 0.0
        state.pose.orientation.y = 0.0
        state.pose.orientation.z = math.sin(half_yaw)

        # Servis henüz hazır değilse (Gazebo açılıyor) beklemeden bu tick'i atla
        # — sonraki tick yeniden dener. wait_for_service kullanılmaz: her tick'te
        # 100ms bloklamak rclpy spin'ini yavaşlatır (review B3).
        if not self.cli.service_is_ready():
            return

        req = SetEntityState.Request()
        req.state = state
        # Sonuçla ilgilenilmez (spam önleme) — async call, callback hatayı loglar.
        future = self.cli.call_async(req)
        future.add_done_callback(self._on_result)

    def _on_result(self, future) -> None:
        """SetEntityState yanıtı: hata debug seviyesinde loglanır (spam yok)."""
        try:
            result = future.result()
            if result is not None and not result.success:
                self.get_logger().debug("SetEntityState servis hatası döndü")
        except Exception as exc:
            self.get_logger().debug(f"SetEntityState çağrısı başarısız: {exc}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GazeboPoseSyncNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
