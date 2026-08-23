"""generate_course — TEKNOFEST İDA parkur generator'ı.

``tools/ida_course.py``'daki parametrik şemadan (P1 N-şekil + P2 düz + P3
hedef üçgeni) üç çıktı üretir:

1. ``full_mission.yaml``     — metrik scenario (stack kontratı):
   origin/initial_pose/waypoints/buoys/targets; x = kuzey, y = doğu.
2. ``hakem_mission.txt``     — coğrafik görev dosyası (hakemden alıyormuşuz
   gibi): her waypoint `parkur<no> <lat> <lon>`, dd.ddddddd (7 ondalık),
   ``geo.local_m_to_latlon`` ile. Başlık satırı # ile.
3. ``ida_course_models.sdf`` — duba katmanı: her duba için SDF ``<model>``
   bloğu (armut-tip görsel + silindir collision), world'e include edilecek.
   Pozlar scenario ile BİREBİR; Gazebo ENU dönüşümü: world(x, y) =
   scenario(y_east, x_north); z = 0.25 (kenar/engel) / 0.45 (hedef).

Deterministik: tüm rastgelelik ``random.Random(seed)`` üzerinden; zaman
kullanılmaz (aynı seed -> aynı çıktı). Varsayılan seed CLI'da ``--seed``
ile, şema parametreleri ``--config <json>`` ile değiştirilebilir.

Kullanım:
    python3 tools/generate_course.py --seed 2026
"""

import argparse
import json
import sys
from pathlib import Path
from typing import Any, Dict, List

# tools/ paket değil; ida_course'a dosya dizini üzerinden eriş.
sys.path.insert(0, str(Path(__file__).resolve().parent))
from ida_course import (  # noqa: E402  (sys.path düzenlemesi import'tan önce)
    CourseSchema,
    DEFAULT_CONFIG,
    ScenarioData,
)

# Gazebo duba model şartnameleri (world'de kullanılacak boyutlar).
# - Kenar/engel duba: çap 30 cm, yükseklik 50 cm (armut: alt 0.15r/0.25h,
#   üst 0.08r/0.25h; collision tek silindir 0.15r/0.5h).
# - Hedef duba: çap 640 mm, yükseklik 950 mm (armut: alt 0.32r/0.475h,
#   üst 0.16r/0.475h; collision 0.32r/0.95h).
# ``z`` değerleri: dubanın su üstü yüksekliği (0.25 kenar/engel, 0.45 hedef)
# — scenario koordinatlarıyla birebir tutarlı (y ekseni pozları dönüşümünde).
BUOY_MODEL_SPEC = {
    "edge": {"r_bottom": 0.15, "r_top": 0.08, "h_half": 0.25, "z": 0.25},
    "yellow": {"r_bottom": 0.15, "r_top": 0.08, "h_half": 0.25, "z": 0.25},
    "target": {"r_bottom": 0.32, "r_top": 0.16, "h_half": 0.475, "z": 0.45},
}

# SDF renk sabitleri (RGBA) — şartname RAL değerleriyle uyumlu.
COLOR_RGBA = {
    "orange": (1.0, 0.5, 0.0, 1.0),
    "yellow": (1.0, 0.9, 0.0, 1.0),
    "red": (1.0, 0.1, 0.1, 1.0),
    "green": (0.0, 0.8, 0.2, 1.0),
    "black": (0.1, 0.1, 0.1, 1.0),
}

# dd.ddddddd (7 ondalık) — hakem görev dosyası ve scenario origin için.
_LAT_LON_FORMAT = "{:.7f}"

# Gazebo ENU dönüşümü: scenario (x=kuzey, y=doğu) -> world (x=doğu, y=kuzey).
def _world_pose(buoy: Dict[str, Any], spec: Dict[str, Any]) -> tuple[float, float, float]:
    """Scenario duba -> Gazebo world pose (x_east, y_north, z)."""
    x_north = float(buoy["x"])
    y_east = float(buoy["y"])
    return y_east, x_north, float(spec["z"])


# --- YAML emisyonu ----------------------------------------------------------

def _quote(value: Any) -> str:
    """YAML string değerini tırnaklar (sayı/None/float değilse).

    Tüm string'ler tırnaklanır — düz sayı görünümlü string'ler (örn. "45.0")
    tırnaksız YAML'da sayıya dönüşür, JSON-uyumlu scenario okuma
    (load_scenario) bozulur. Sayılar/None çıplak yazılır.
    """
    if isinstance(value, str):
        return f'"{value}"'
    if value is None:
        return "null"
    return str(value)


def emit_yaml(data: ScenarioData) -> str:
    """ScenarioData'yı stack kontratlı JSON-uyumlu YAML metnine çevirir.

    ``load_scenario`` önce ``json.loads`` dener, başarısızsa PyYAML'a düşer;
    bu yüzden emisyon JSON-uyumlu olmalı (tırnaklı anahtarlar, sayılar).
    """
    lines: List[str] = []
    lines.append("{")
    lines.append(f'  "origin": {{"lat": {_quote(data.origin["lat"])}, "lon": {_quote(data.origin["lon"])}}},')
    pose = data.initial_pose
    lines.append(
        f'  "initial_pose": {{"x": {_quote(pose["x"])}, "y": {_quote(pose["y"])}, '
        f'"heading_deg": {_quote(pose["heading_deg"])}}},'
    )

    lines.append('  "waypoints": [')
    for i, wp in enumerate(data.waypoints):
        comma = "," if i < len(data.waypoints) - 1 else ""
        lines.append(
            f'    {{"x": {_quote(wp["x"])}, "y": {_quote(wp["y"])}, '
            f'"parkur": {_quote(wp["parkur"])}}}{comma}'
        )
    lines.append("  ],")

    lines.append('  "buoys": [')
    for i, buoy in enumerate(data.buoys):
        comma = "," if i < len(data.buoys) - 1 else ""
        lines.append(
            f'    {{"id": "{buoy["id"]}", "x": {_quote(buoy["x"])}, '
            f'"y": {_quote(buoy["y"])}, "color": "{buoy["color"]}"}}{comma}'
        )
    lines.append("  ],")

    lines.append('  "obstacles": [],')

    lines.append('  "targets": [')
    for i, target in enumerate(data.targets):
        comma = "," if i < len(data.targets) - 1 else ""
        lines.append(
            f'    {{"id": "{target["id"]}", "x": {_quote(target["x"])}, '
            f'"y": {_quote(target["y"])}, "color": "{target["color"]}"}}{comma}'
        )
    lines.append("  ]")
    lines.append("}")
    return "\n".join(lines) + "\n"


# --- Hakem görev dosyası ------------------------------------------------------

def _latlon(d: ScenarioData, x: float, y: float) -> tuple[str, str]:
    """Scenario (x_kuzey, y_doğu) -> dd.ddddddd lat/lon string'leri."""
    lat, lon = _local_m_to_latlon(
        float(d.origin["lat"]),
        float(d.origin["lon"]),
        float(x),
        float(y),
    )
    return _LAT_LON_FORMAT.format(lat), _LAT_LON_FORMAT.format(lon)


def emit_referee_mission(d: ScenarioData) -> str:
    """Coğrafik görev dosyası: `parkur<no> <lat> <lon>` satırları.

    Başlık # ile (yorum); her waypoint geo.local_m_to_latlon ile çevrilir
    (7 ondalık). Parkur no: gn1..gn4 -> 1, gn5 -> 2 (P3 waypoint'i yok).
    """
    lines: List[str] = []
    lines.append(
        "# TEKNOFEST İDA hakem görevi (otomatik üretildi) — parkur<no> lat lon"
    )
    for wp in d.waypoints:
        lat, lon = _latlon(d, wp["x"], wp["y"])
        lines.append(f"parkur{int(wp['parkur'])} {lat} {lon}")
    return "\n".join(lines) + "\n"


# --- Gazebo duba SDF katmanı -----------------------------------------------------

def _sdf_model(name: str, pose: tuple[float, float, float], color: tuple[float, float, float, float], spec: Dict[str, Any]) -> str:
    """Tek duba için SDF ``<model>`` bloğu (armut-tip görsel + collision).

    Görsel iki silindir: alt geniş (r_bottom, h_half) + üst dar (r_top,
    h_half) — armut formu. Collision tek silindir (r_bottom, 2*h_half).
    Model adı duba id'si; ``<static>true</static>`` (parkur sabit).
    """
    r_bottom = spec["r_bottom"]
    r_top = spec["r_top"]
    h_half = spec["h_half"]
    ambient = f"{color[0]:.2f} {color[1]:.2f} {color[2]:.2f} {color[3]:.2f}"
    x, y, z = pose
    return f"""  <model name="{name}" static="true">
    <pose>{x:.3f} {y:.3f} {z:.3f} 0 0 0</pose>
    <link name="{name}_link">
      <collision name="{name}_collision">
        <geometry>
          <cylinder>
            <radius>{r_bottom}</radius>
            <length>{2.0 * h_half}</length>
          </cylinder>
        </geometry>
      </collision>
      <visual name="{name}_visual_bottom">
        <pose>0 0 {h_half / 2.0:.3f} 0 0 0</pose>
        <geometry>
          <cylinder>
            <radius>{r_bottom}</radius>
            <length>{h_half}</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>{ambient}</ambient>
          <diffuse>{ambient}</diffuse>
        </material>
      </visual>
      <visual name="{name}_visual_top">
        <pose>0 0 {h_half + h_half / 2.0:.3f} 0 0 0</pose>
        <geometry>
          <cylinder>
            <radius>{r_top}</radius>
            <length>{h_half}</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>{ambient}</ambient>
          <diffuse>{ambient}</diffuse>
        </material>
      </visual>
    </link>
  </model>"""


def emit_course_models_sdf(data: ScenarioData) -> str:
    """Duba katmanı SDF: world içine doğrudan gömülen ``<model>`` blokları.

    Kenar (turuncu) + engel (sarı) + hedef (kırmızı/siyah/yeşil) model
    blokları; pozlar scenario ile birebir (ENU dönüşümü uygulanır).
    Model adı duba id'si (örn. ``p1_o_l1``). ``<sdf version="1.6">``
    kökü world tarafından sağlanır (bu dosya katman olarak world'e gömülür).
    """
    blocks: List[str] = []
    blocks.append("<!-- ida_course_models.sdf — otomatik üretildi (tools/generate_course.py).")
    blocks.append("     Duba katmanı: world'e gömülür (ENU dönüşümü uygulanır);")
    blocks.append("     pozlar full_mission.yaml ile birebir. -->")
    for buoy in data.buoys:
        color = str(buoy["color"]).lower()
        if color == "orange":
            spec = BUOY_MODEL_SPEC["edge"]
        elif color == "yellow":
            spec = BUOY_MODEL_SPEC["yellow"]
        else:
            spec = BUOY_MODEL_SPEC["target"]
        blocks.append(
            _sdf_model(
                str(buoy["id"]),
                _world_pose(buoy, spec),
                COLOR_RGBA.get(color, COLOR_RGBA["orange"]),
                spec,
            )
        )
    for target in data.targets:
        blocks.append(
            _sdf_model(
                str(target["id"]),
                _world_pose(target, BUOY_MODEL_SPEC["target"]),
                COLOR_RGBA.get(str(target["color"]).lower(), COLOR_RGBA["orange"]),
                BUOY_MODEL_SPEC["target"],
            )
        )
    return "\n".join(blocks) + "\n"


_WORLD_TEMPLATE = """<?xml version="1.0" ?>
<!--
  sim_gazebo.world — TEKNOFEST İDA simülasyon dünyası (Gazebo Classic 11, SDF 1.6).

  Bu dünya YALNIZCA görselleştirmedir; FİZİK ArduRover SITL'de koşar.
  ida_boat modeli SITL konumunu İZLER (gazebo_pose_sync_node,
  /telemetry/state -> SetEntityState). Kamera/lidar plugin'leri modele bağlı
  olduğundan SITL ile senkron hareket eder ve /camera/image_raw ile /scan
  yayınlar.

  KOORDİNAT DÖNÜŞÜMÜ (kritik):
    - full_mission.yaml scenario koordinatları: x = kuzey, y = doğu.
    - Gazebo world frame: x = doğu, y = kuzey (ENU).
    - Bu world'deki tüm statik nesne pose'ları dönüştürülmüştür:
      world(x, y) = scenario(y_east, x_north).

  PARKUR KATMANI: tools/generate_course.py tarafından üretilir (tek kaynak).
  full_mission.yaml + hakem_mission.txt + aşağıdaki duba modelleri birlikte
  üretilir; yeni görev dosyası gelince generator yeniden çalıştırılır.
  DÜBA MODELLERİ buraya gömülüdür (ida_course_models.sdf katmanı).
-->
<sdf version="1.6">
  <world name="ida_sim">
    <!-- Güneş ışığı + ortam aydınlatması -->
    <include>
      <uri>model://sun</uri>
    </include>

    <light name="ambient_light" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 30 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 -0.5 -0.9</direction>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
    </light>

    <!-- Varsayılan fizik (yüzey yerçekimi, standart temas) -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <!-- gazebo_ros_state: /set_entity_state servisi + model_states yayınlar.
         gazebo_pose_sync_node SetEntityState ile ida_boat'u taşır; plugin yoksa
         servis yayınlanmaz -> model yerinde kalır. -->
    <plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so">
      <ros>
        <namespace>/gazebo</namespace>
      </ros>
      <update_rate>20</update_rate>
    </plugin>

    <!-- Su yüzeyi: z=0 düzleminde büyük mavi yarı saydam tabaka -->
    <model name="water_plane">
      <static>true</static>
      <link name="water_link">
        <collision name="water_collision">
          <pose>0 0 0 0 0 0</pose>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
        </collision>
        <visual name="water_visual">
          <pose>0 0 0 0 0 0</pose>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.2 0.4 0.8 0.6</ambient>
            <diffuse>0.2 0.5 0.9 0.6</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
          </material>
        </visual>
      </link>
    </model>

    <!--
      İDA tekne modeli (SITL konumunu izler).
      Gövde: 1.4 x 0.9 x 0.25 m; mast: kamera yüksekliği için dikey silindir.
      Camera: tekne önüne bakar (ENU'da ön +x'tir; sensor pose 0.5 0 0.5 0 0 0).
      Başlangıç pose 0 0 0.3 0 0 0 (z = su üstü); gerçek konum/yaw
      gazebo_pose_sync_node tarafından yazılır (heading 0 = kuzey = +y).
    -->
    <model name="ida_boat">
      <static>false</static>
      <pose>0 0 0.3 0 0 0</pose>
      <link name="boat_body">
        <pose>0 0 0.125 0 0 0</pose>
        <collision name="boat_body_collision">
          <geometry>
            <box>
              <size>1.4 0.9 0.25</size>
            </box>
          </geometry>
        </collision>
        <visual name="boat_body_visual">
          <geometry>
            <box>
              <size>1.4 0.9 0.25</size>
            </box>
          </geometry>
          <material>
            <ambient>1.0 0.55 0.15 1</ambient>
            <diffuse>1.0 0.6 0.2 1</diffuse>
          </material>
        </visual>
      </link>
      <link name="mast">
        <pose>0 0 0.25 0 0 0</pose>
        <visual name="mast_visual">
          <geometry>
            <cylinder>
              <radius>0.03</radius>
              <length>0.4</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>0.9 0.9 0.9 1</ambient>
            <diffuse>1 1 1 1</diffuse>
          </material>
        </visual>
      </link>
      <!-- Kamera: tekne önüne (ENU +x) ve mast tepesine monte -->
      <link name="camera_link">
        <pose>0.5 0 0.5 0 0 0</pose>
        <inertial>
          <mass>0.05</mass>
          <inertia>
            <ixx>0.0001</ixx>
            <iyy>0.0001</iyy>
            <izz>0.0001</izz>
          </inertia>
        </inertial>
        <sensor name="camera_sensor" type="camera">
          <camera>
            <horizontal_fov>1.57</horizontal_fov>
            <image>
              <width>640</width>
              <height>480</height>
            </image>
            <clip>
              <near>0.1</near>
              <far>100</far>
            </clip>
          </camera>
          <update_rate>10</update_rate>
          <plugin name="ida_camera" filename="libgazebo_ros_camera.so">
            <ros>
              <namespace>/</namespace>
              <remapping>image_raw:=camera/image_raw</remapping>
            </ros>
            <camera_name>ida_camera</camera_name>
            <image_width>640</image_width>
            <image_height>480</image_height>
            <update_rate>10</update_rate>
            <horizontal_fov>1.57</horizontal_fov>
          </plugin>
        </sensor>
      </link>
      <!-- 2D Lidar (ray): mast üstü, 360 derece -->
      <link name="lidar_link">
        <pose>0.2 0 0.2 0 0 0</pose>
        <inertial>
          <mass>0.05</mass>
          <inertia>
            <ixx>0.0001</ixx>
            <iyy>0.0001</iyy>
            <izz>0.0001</izz>
          </inertia>
        </inertial>
        <sensor name="lidar_sensor" type="ray">
          <ray>
            <scan>
              <horizontal>
                <samples>360</samples>
                <resolution>1</resolution>
                <min_angle>-3.14159</min_angle>
                <max_angle>3.14159</max_angle>
              </horizontal>
            </scan>
            <range>
              <min>0.3</min>
              <max>18.0</max>
              <resolution>0.01</resolution>
            </range>
            <noise>
              <type>gaussian</type>
              <mean>0.0</mean>
              <stddev>0.01</stddev>
            </noise>
          </ray>
          <update_rate>10</update_rate>
          <plugin name="ida_lidar" filename="libgazebo_ros_ray_sensor.so">
            <ros>
              <namespace>/</namespace>
              <remapping>scan:=scan</remapping>
            </ros>
            <output_type>sensor_msgs/LaserScan</output_type>
            <update_rate>10</update_rate>
            <min_angle>-3.14159</min_angle>
            <max_angle>3.14159</max_angle>
            <range_min>0.3</range_min>
            <range_max>18.0</range_max>
          </plugin>
        </sensor>
      </link>
    </model>

    <!-- ============================================================
         PARKUR DUBALARI (otomatik üretildi — tools/generate_course.py).
         Pozlar full_mission.yaml ile birebir; ENU dönüşümü uygulanmış.
         ============================================================ -->
{buoy_blocks}
  </world>
</sdf>
"""


def emit_world(buoy_blocks: str) -> str:
    """Tam world SDF metnini üretir (şablon + duba katmanı).

    ``sim_gazebo.world`` bu fonksiyonun çıktısıyla aynıdır; generator tek
    kaynak olduğundan duba katmanı world'e gömülür (ayrı include dosyası
    tutulmaz — yeni görev gelince world yeniden üretilir).
    """
    return _WORLD_TEMPLATE.format(buoy_blocks=buoy_blocks.rstrip() + "\n")


# --- geo.local_m_to_latlon ile birebir aynı (ida_planning'e bağımlılık yok) ---------

def _local_m_to_latlon(origin_lat: float, origin_lon: float, x_north: float, y_east: float) -> tuple[float, float]:
    """geo.local_m_to_latlon ile birebir aynı formül (bağımsız implementasyon).

    tools/ altındaki araçlar ida_planning'e import bağımlılığı kurmaz
    (ROS2 paket yolu gerekmez); formül kopyası kontratı korur ve round-trip
    testi (hakem_mission.txt -> local) bunu doğrular.
    """
    import math

    earth_radius_m = 6371000.0
    lat = origin_lat + math.degrees(x_north / earth_radius_m)
    lon = origin_lon + math.degrees(
        y_east / (earth_radius_m * math.cos(math.radians(origin_lat)))
    )
    return lat, lon


# --- CLI ------------------------------------------------------------------------

def _default_paths() -> Dict[str, Path]:
    """Çıktı yolları: repo kökü + src/ida_bringup altı (paket kontratı)."""
    repo_root = Path(__file__).resolve().parents[1]
    return {
        "scenario": repo_root / "src" / "ida_bringup" / "scenarios" / "full_mission.yaml",
        "referee": repo_root / "src" / "ida_bringup" / "scenarios" / "hakem_mission.txt",
        "world": repo_root / "src" / "ida_bringup" / "worlds" / "sim_gazebo.world",
    }


def main(argv: List[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="TEKNOFEST İDA parkur generator'ı (full_mission.yaml + hakem_mission.txt + ida_course_models.sdf)"
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=2026,
        help="Deterministik rastgelelik tohumu (varsayılan: 2026).",
    )
    parser.add_argument(
        "--config",
        type=str,
        default=None,
        help="Şema parametreleri için JSON dosyası (opsiyonel; varsayılanlar kullanılır).",
    )
    parser.add_argument(
        "--out-dir",
        type=str,
        default=None,
        help="Çıktı dizini (varsayılan: repo src/ida_bringup yolları).",
    )
    args = parser.parse_args(argv)

    config: Dict[str, Any] = {}
    if args.config:
        config_path = Path(args.config)
        config = json.loads(config_path.read_text(encoding="utf-8"))
        unknown = set(config) - set(DEFAULT_CONFIG)
        if unknown:
            parser.error(f"--config bilinmeyen anahtarlar içeriyor: {sorted(unknown)}")

    schema = CourseSchema(config)
    data = schema.generate(seed=args.seed)

    paths = _default_paths()
    if args.out_dir:
        out = Path(args.out_dir)
        paths = {
            "scenario": out / "full_mission.yaml",
            "referee": out / "hakem_mission.txt",
            "world": out / "sim_gazebo.world",
        }
    for path in paths.values():
        path.parent.mkdir(parents=True, exist_ok=True)

    paths["scenario"].write_text(emit_yaml(data), encoding="utf-8")
    paths["referee"].write_text(emit_referee_mission(data), encoding="utf-8")
    # World: şablon + gömülü duba katmanı (tek kaynak — sim_gazebo.world ile aynı).
    paths["world"].write_text(emit_world(emit_course_models_sdf(data)), encoding="utf-8")

    print(f"seed={args.seed}")
    print(f"  waypoints={len(data.waypoints)}  buoys={len(data.buoys)}  targets={len(data.targets)}")
    print(f"  -> {paths['scenario']}")
    print(f"  -> {paths['referee']}")
    print(f"  -> {paths['world']}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
