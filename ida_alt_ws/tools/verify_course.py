"""verify_course — generate_course.py çıktılarını kontrata karşı doğrular.

Çalıştır: python tools/verify_course.py [--seed 2026] [--repo <kök>]

Kontroller:
1. full_mission.yaml ``load_scenario`` ile okunur (JSON-uyumlu YAML).
2. waypoint sayısı 5 (gn1..gn5; P3 waypoint'i yok), parkur etiketleri.
3. buoys >= çift sayısı*2 + 10 (sarılar dahil); targets == 3.
4. Sarı dubalar birbirine VE turuncuya < 2 m yakın değil.
5. hakem_mission.txt ``local_m_to_latlon``'un tersiyle geri çevrilip
   senaryoyla tutarlı (hata < 1 m) — geo formülünün birebir aynısı.
6. Determinizm: aynı seed iki kez üretilirse aynı scenario (byte-identical).
7. SDF: her duba/target scenario pozuna birebir oturur (ENU dönüşümü).
"""

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any, Dict, List

# Windows konsolu cp1252 olabilir; Türkçe çıktı için UTF-8 zorla.
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")

sys.path.insert(0, str(Path(__file__).resolve().parent))
from ida_course import CourseSchema, count_orange_pairs, count_yellow, min_clearance  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src" / "ida_planning"))
from ida_planning.scenario import load_scenario  # noqa: E402

EARTH_RADIUS_M = 6371000.0


def _latlon_to_local_m(origin_lat: float, origin_lon: float, lat: float, lon: float) -> tuple[float, float]:
    """geo.latlon_to_local_m ile birebir aynı (ters çevirme doğrulaması)."""
    dlat = math.radians(lat - origin_lat)
    dlon = math.radians(lon - origin_lon)
    x_north = dlat * EARTH_RADIUS_M
    y_east = dlon * EARTH_RADIUS_M * math.cos(math.radians(origin_lat))
    return x_north, y_east


def _round_trip_err(origin: Dict[str, float], wp_x: float, wp_y: float, lat: float, lon: float) -> float:
    """Scenario noktası vs hakem dosyası lat/lon'undan geri çevrilen (m)."""
    rx, ry = _latlon_to_local_m(origin["lat"], origin["lon"], lat, lon)
    return math.hypot(rx - wp_x, ry - wp_y)


def _sdf_pose_for(name: str, sdf_text: str) -> tuple[float, float, float]:
    """SDF metninden model adına göre <pose> üçlüsünü (x, y, z) çıkarır."""
    marker = f'<model name="{name}"'
    idx = sdf_text.find(marker)
    if idx < 0:
        return (0.0, 0.0, 0.0)
    pose_open = sdf_text.find("<pose>", idx)
    pose_close = sdf_text.find("</pose>", pose_open)
    values = sdf_text[pose_open + len("<pose>") : pose_close].split()
    return float(values[0]), float(values[1]), float(values[2])


def main(argv: List[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--seed", type=int, default=2026)
    parser.add_argument("--repo", type=str, default=None)
    args = parser.parse_args(argv)

    repo = Path(args.repo) if args.repo else Path(__file__).resolve().parents[1]
    scenario_path = repo / "src" / "ida_bringup" / "scenarios" / "full_mission.yaml"
    referee_path = repo / "src" / "ida_bringup" / "scenarios" / "hakem_mission.txt"
    sdf_path = repo / "src" / "ida_bringup" / "worlds" / "sim_gazebo.world"

    failures: List[str] = []

    def check(ok: bool, msg: str) -> None:
        status = "OK " if ok else "FAIL"
        print(f"  [{status}] {msg}")
        if not ok:
            failures.append(msg)

    # 1) load_scenario (gerçek kontrat okuyucusu).
    scenario = load_scenario(str(scenario_path))
    check(isinstance(scenario, dict), "full_mission.yaml load_scenario ile okundu")

    # 2) Waypoint'ler.
    waypoints = scenario["waypoints"]
    check(len(waypoints) == 5, f"waypoint sayısı == 5 (aldık {len(waypoints)})")
    expected_wp = [(10.0, 0.0), (35.0, 30.0), (15.0, 60.0), (45.0, 90.0), (45.0, 140.0)]
    wp_match = all(
        math.isclose(w["x"], ex[0], abs_tol=1e-6) and math.isclose(w["y"], ex[1], abs_tol=1e-6)
        for w, ex in zip(waypoints, expected_wp)
    )
    check(wp_match, "waypoint koordinatları gn1..gn5 ile birebir")
    check(
        all(int(w.get("parkur", 0)) == 1 for w in waypoints[:-1])
        and int(waypoints[-1].get("parkur", 0)) == 2,
        "parkur etiketleri: gn1..gn4 -> 1, gn5 -> 2",
    )
    # initial_pose gn1'de (10,0), heading 0.
    pose = scenario.get("initial_pose", {})
    check(
        math.isclose(float(pose.get("x", 0.0)), 10.0, abs_tol=1e-6)
        and math.isclose(float(pose.get("y", 0.0)), 0.0, abs_tol=1e-6)
        and float(pose.get("heading_deg", -1.0)) == 0.0,
        "initial_pose (10, 0, heading 0) — gn1 hemen önü",
    )

    # 3) Duba/target sayıları.
    buoys = scenario["buoys"]
    targets = scenario["targets"]
    orange = [b for b in buoys if b.get("color") == "orange"]
    yellow = [b for b in buoys if b.get("color") == "yellow"]
    pairs = count_orange_pairs(ScenarioData) if False else len(orange) // 2
    check(len(orange) == pairs * 2, f"turuncu dubalar çift sayısında ({pairs} çift)")
    check(len(buoys) >= pairs * 2 + 10, f"buoys >= çift*2 + 10 (aldık {len(buoys)} >= {pairs * 2 + 10})")
    check(len(yellow) == 10, f"sarı engel == 10 (aldık {len(yellow)})")
    check(len(targets) == 3, f"targets == 3 (aldık {len(targets)})")
    check(
        sorted(t["color"] for t in targets) == ["black", "green", "red"],
        "hedef renkleri: kırmızı/siyah/yeşil",
    )
    check(scenario.get("obstacles", None) == [], "obstacles boş (sarılar buoys içinde)")

    # 4) Sarı mesafeleri (min_clearance >= 2 m).
    schema = CourseSchema()
    data = schema.generate(seed=args.seed)
    clearance = min_clearance(data)
    check(clearance >= 2.0, f"sarı-sarı & sarı-turuncu min mesafe >= 2 m (aldık {clearance:.3f} m)")

    # 5) hakem_mission.txt round-trip (hata < 1 m).
    referee_lines = [
        line.strip() for line in referee_path.read_text(encoding="utf-8").splitlines()
        if line.strip() and not line.strip().startswith("#")
    ]
    check(len(referee_lines) == 5, f"hakem dosyası 5 waypoint satırı (aldık {len(referee_lines)})")
    origin = scenario["origin"]
    max_err = 0.0
    ref_ok = True
    for i, line in enumerate(referee_lines):
        parts = line.split()
        if len(parts) != 3:
            ref_ok = False
            continue
        parkur_label, lat_s, lon_s = parts
        lat, lon = float(lat_s), float(lon_s)
        err = _round_trip_err(origin, waypoints[i]["x"], waypoints[i]["y"], lat, lon)
        max_err = max(max_err, err)
        if err >= 1.0:
            ref_ok = False
            print(f"    ! waypoint {i + 1} hata {err:.4f} m >= 1 m")
        if int(parkur_label.replace("parkur", "")) != int(waypoints[i].get("parkur", 0)):
            ref_ok = False
    check(ref_ok, f"hakem_mission.txt round-trip < 1 m (maks hata {max_err:.6f} m)")

    # 6) Determinizm: aynı seed iki kez -> byte-identical scenario.
    d1 = schema.generate(seed=args.seed)
    d2 = schema.generate(seed=args.seed)
    from generate_course import emit_yaml  # noqa: E402

    check(emit_yaml(d1) == emit_yaml(d2), "determinizm: aynı seed -> aynı scenario metni")

    # 7) SDF poz birebir (ENU dönüşümü).
    sdf_text = sdf_path.read_text(encoding="utf-8")
    # Duba z değerleri: kenar/engel 0.25, hedef 0.45.
    z_for = lambda color: 0.25 if color in ("orange", "yellow") else 0.45
    all_objects = [dict(b) for b in buoys] + [dict(t) for t in targets]
    sdf_ok = True
    for obj in all_objects:
        name = str(obj["id"])
        gx, gy, gz = _sdf_pose_for(name, sdf_text)
        exp_x = float(obj["y"])  # ENU: world x = scenario y_east
        exp_y = float(obj["x"])  # ENU: world y = scenario x_north
        exp_z = z_for(str(obj["color"]).lower())
        # SDF pozları 3 ondalıkla yazılır (0.001 m hassasiyet) — tol buna göre.
        if not (
            math.isclose(gx, exp_x, abs_tol=1e-3)
            and math.isclose(gy, exp_y, abs_tol=1e-3)
            and math.isclose(gz, exp_z, abs_tol=1e-3)
        ):
            sdf_ok = False
            print(f"    ! {name}: SDF pose ({gx}, {gy}, {gz}) != beklenen ({exp_x}, {exp_y}, {exp_z})")
    check(sdf_ok, "SDF pozları scenario ile birebir (ENU)")

    # 8) Her scenario nesnesinin world'de tam bir model bloğu var
    # (world'deki ida_boat/water_plane sayılmaz; duba/target isimleri aranır).
    missing = [str(obj["id"]) for obj in all_objects if f'<model name="{obj["id"]}"' not in sdf_text]
    check(not missing, f"world'de tüm duba/target modelleri mevcut ({len(all_objects) - len(missing)}/{len(all_objects)})")

    print()
    if failures:
        print(f"DOĞRULAMA BAŞARISIZ — {len(failures)} hata:")
        for f in failures:
            print(f"  - {f}")
        return 1
    print("DOĞRULAMA TAMAM — tüm kontroller geçti")
    return 0


if __name__ == "__main__":
    sys.exit(main())
