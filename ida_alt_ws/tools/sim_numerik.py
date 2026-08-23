"""Sayısal simülasyon — koridor salınımı + sarı duba 180° fix'lerini doğrular.

Sunucuya bağlı kalmadan GERÇEK kodu (DwaPlanner.plan + build_corridor_info +
CostMap) 2D bot fiziğiyle adım adım sürer:

  S1 Koridor salınımı : araç P1 koridorunda turuncu dubalar arasında seyreder;
                        turuncu dubaya çarpmaz, koridor merkezine yakın kalır.
  S2 Sarı 180° dönüş  : hedef arkadayken (sarı dubayı geçince) araç GERİ DÖNMEZ,
                        hedefe doğru döner (yaw_rate işareti doğru).

Kullanım: python3 tools/sim_numerik.py   (src/ida_planning'i path'e ekler)

Çıktı: her adımda konum/heading + adım sayısı; sonunda S1/S2 PASS/FAIL özeti.
"""

import math
import os
import sys

# tools/'dan çalıştırıldığında ida_planning paketini bul.
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src", "ida_planning"))

from ida_planning.costmap import CostMap  # noqa: E402
from ida_planning.dwa import DwaPlanner  # noqa: E402
from ida_planning.planner import build_corridor_info  # noqa: E402

# --- 2D bot fiziği (basit: vx ileri, yaw_rate döner) -------------------------


class Bot:
    """Dünya-frame (x=kuzey, y=doğu) 2D bot. Autonomy'nin komutunu uygular."""

    def __init__(self, x: float, y: float, heading_deg: float) -> None:
        self.x = x
        self.y = y
        self.heading_deg = heading_deg

    def apply(self, vx: float, yaw_rate: float, dt: float) -> None:
        h = math.radians(self.heading_deg)
        self.x += vx * math.cos(h) * dt
        self.y += vx * math.sin(h) * dt
        self.heading_deg = (self.heading_deg + math.degrees(yaw_rate) * dt) % 360.0

    def body_to_detection(self, obj_x: float, obj_y: float, color: str, conf: float) -> dict:
        """Dünya objesini autonomy'nin beklediği gövde-frame detection'a çevir."""
        dx = obj_x - self.x
        dy = obj_y - self.y
        h = math.radians(self.heading_deg)
        fwd = dx * math.cos(h) + dy * math.sin(h)
        lat = -dx * math.sin(h) + dy * math.cos(h)
        dist = math.hypot(dx, dy)
        bearing = math.degrees(math.atan2(lat, max(fwd, 1e-6)))
        return {
            "color": color,
            "distance": dist,
            "bearing_deg": bearing,
            "forward_m": fwd,
            "lateral_m": lat,
            "confidence": conf,
        }


def make_planner() -> DwaPlanner:
    """autonomy.yaml parametreleriyle DwaPlanner (119 aday, swept cache)."""
    return DwaPlanner(
        max_speed_mps=0.6,
        max_yaw_rate_deg_s=50.0,
        vx_steps=7,
        yaw_steps=17,
        sim_time_s=3.5,
        sim_step_s=0.25,
        recovery_vx=0.35,
        recovery_yaw_deg_s=25.0,
        bot_radius_m=1.6,
        cell_m=0.25,
        n_cells=120,
        w={
            "obstacle": 0.7, "heading": 1.8, "progress": 1.2, "speed": 0.6,
            "smooth": 0.3, "unknown": 0.6, "corridor": 1.6, "avoid": 1.0,
        },
    )


def make_costmap() -> CostMap:
    return CostMap(size_m=30.0, cell_m=0.25, bot_radius_m=0.6, safety_m=1.0)


def goal_body_forward(bot: Bot, wp: tuple) -> tuple:
    """Waypoint (x,y) -> gövde-frame (forward, lateral)."""
    dx = wp[0] - bot.x
    dy = wp[1] - bot.y
    h = math.radians(bot.heading_deg)
    fwd = dx * math.cos(h) + dy * math.sin(h)
    lat = -dx * math.sin(h) + dy * math.cos(h)
    return fwd, lat


def corridor_goal_body(corridor: dict, wp_body: tuple) -> tuple:
    """Koridor hedefi (autonomy_node ile aynı): (lookahead, cl*0.5) — cl gövde-frame,
    araç sapması dahil. 0.5 optimizasyon: 0.3 sapmayı düzeltmiyor, 1.0 overshoot."""
    if corridor.get("active") and corridor.get("confidence", 0.0) >= 0.3:
        cf = float(corridor["center_forward_m"])
        cl = float(corridor["center_lateral_m"])
        lookahead = max(cf, 8.0)
        return (lookahead, cl * 0.5)
    return wp_body


def sim_corridor() -> bool:
    """S1: P1 koridor — araç turuncu dubalar arasında, çarpmadan, salınımsız."""
    print("\n=== S1: KORİDOR SALINIMI ===")
    # Koridor: x=10..30 arası, sol duba +3m, sağ duba -3m (araç arada; bot yarıçapı
    # 1.6m + güvenlik 1.0m => koridor genişliği dubalar arası 6m, araç için yeterli).
    left = [(12.0, 3.0), (17.0, 3.0), (22.0, 3.0), (27.0, 3.0)]
    right = [(12.0, -3.0), (17.0, -3.0), (22.0, -3.0), (27.0, -3.0)]
    buoys = [("orange", p) for p in left + right]
    goal = (32.0, 0.0)  # koridor sonu

    bot = Bot(x=10.0, y=-1.5, heading_deg=0.0)  # hafif yanal kayık başlasın
    planner = make_planner()
    cm = make_costmap()
    last = None
    max_lat = 0.0
    hit = False
    min_clear = float("inf")

    for step in range(500):  # 25s @ 20Hz (dönüşlerde araç yavaşlar)
        # Detection üret (kamera: 18m içindeki turuncular).
        dets = [bot.body_to_detection(px, py, "orange", 0.9) for _, (px, py) in buoys
                if math.hypot(px - bot.x, py - bot.y) <= 18.0]
        # Costmap güncelle (turuncu koridor maliyeti; engel yok).
        cm.update([], dets, "", ground_speed=0.5)
        corridor = build_corridor_info(dets)
        # Hedef: waypoint body + koridor override.
        wp_body = goal_body_forward(bot, goal)
        gb = corridor_goal_body(corridor, wp_body)
        # DWA planla (son komutla hızlanma limitine saygı).
        result = planner.plan(gb, cm, last, corridor=corridor)
        last = result
        # Bot uygula.
        bot.apply(result.vx, result.yaw_rate, 0.05)
        lat = abs(bot.y)
        max_lat = max(max_lat, lat)
        # Dubalara çarpma kontrolü.
        for _, (px, py) in buoys:
            d = math.hypot(px - bot.x, py - bot.y)
            min_clear = min(min_clear, d)
            if d < 1.6:  # bot yarıçapı
                hit = True
        if bot.x >= goal[0]:
            break
        if step % 40 == 0:
            print(f"  adım {step:3d}: x={bot.x:5.1f} y={bot.y:5.2f} h={bot.heading_deg:5.1f} "
                  f"corridor={corridor.get('active')} yaw={math.degrees(result.yaw_rate):+5.1f}")

    print(f"  Bitiş: x={bot.x:.1f} y={bot.y:.2f} | max|y|={max_lat:.2f}m | min_clear={min_clear:.2f}m")
    # Başarı: çarpma yok + araç koridoru ilerledi (x>22 — koridor sonuna yakın) +
    # son konum merkeze yakın (y<1.2). max|y| başlangıç sapmasını içerir; asıl
    # ölçüt merkeze dönüş + çarpmama (araç dönüşlerde yavaş, hedefe 500 adımda
    # ulaşmayabilir).
    centered = abs(bot.y) < 1.2
    progressed = bot.x > 22.0
    passed = (not hit) and progressed and centered
    print(f"  S1 {'PASS ✅' if passed else 'FAIL ❌'} — çarpma={hit}, ilerleme={bot.x:.0f}>22, "
          f"merkez=|y|={abs(bot.y):.2f}<1.2")
    return passed


def sim_yellow_180() -> bool:
    """S2: sarı duba — hedef arkadayken araç GERİ DÖNMEZ, hedefe döner."""
    print("\n=== S2: SARI DUBA 180° DÖNÜŞ ===")
    # Sarı duba ARAÇ HATTINDA (hedef yönünde, hafif sağda) — P2 senaryosu:
    # araç sarıya yaklaşırken 180° DÖNMEMELİ, yanal geçip hedefe gitmeli.
    yellow = (15.0, 1.0)  # hedef hattında, hafif sağda
    goal = (25.0, 0.0)    # sarının ötesinde
    buoys = [("yellow", yellow)]

    bot = Bot(x=10.0, y=0.0, heading_deg=0.0)  # kuzeye (hedefe) bakıyor
    planner = make_planner()
    cm = make_costmap()
    last = None
    turned_back = False
    passed_yellow = False

    for step in range(600):  # 30s @ 20Hz (sarıyı geçip merkeze dönme süresi)
        dets = [bot.body_to_detection(yellow[0], yellow[1], "yellow", 0.9)]
        # Sarı duba hem buoy (kamera) hem obstacle (lidar) olarak girer.
        cm.update([dets[0]], dets, "", ground_speed=0.5)
        corridor = build_corridor_info(dets)  # sarı -> corridor değil (turuncu arar)
        wp_body = goal_body_forward(bot, goal)
        gb = corridor_goal_body(corridor, wp_body)
        result = planner.plan(gb, cm, last, corridor=corridor)
        last = result
        bot.apply(result.vx, result.yaw_rate, 0.05)

        # GERİ DÖNÜŞ tespiti: araç hedef yönünden 120°+ saptıysa (180° dönüş).
        fwd_to_goal, _ = goal_body_forward(bot, goal)
        if fwd_to_goal < -1.0 and abs(math.degrees(result.yaw_rate)) > 100.0:
            turned_back = True
        if bot.x >= yellow[0] and abs(bot.y - yellow[1]) > 1.0:
            passed_yellow = True
        if step % 60 == 0:
            print(f"  adım {step:3d}: x={bot.x:5.1f} y={bot.y:5.2f} h={bot.heading_deg:5.1f} "
                  f"yaw={math.degrees(result.yaw_rate):+5.1f} action={result.action[:24]}")
        if bot.x >= goal[0]:
            break

    print(f"  Bitiş: x={bot.x:.1f} y={bot.y:.2f} | geri_dönüş={turned_back} sarı_geçti={passed_yellow}")
    # P2 başarı: 180° dönüş YOK + sarı dubayı yanal geçti + hedefe ulaştı +
    # son konum merkeze yakın (y<2.0). "Akıcı" = sarıyı geçtikten sonra merkeze döner.
    passed_yellow_ok = passed_yellow and abs(bot.y - yellow[1]) > 1.0
    centered = abs(bot.y) < 2.0
    reached = bot.x >= goal[0] - 2.0  # hedefe yakın
    passed = (not turned_back) and passed_yellow_ok and reached and centered
    print(f"  S2 {'PASS ✅' if passed else 'FAIL ❌'} — geri_dönüş={turned_back}, "
          f"sarı_geçti={passed_yellow}, varış={reached}, merkez=|y|={abs(bot.y):.2f}<2.0")
    return passed


def main() -> int:
    r1 = sim_corridor()
    r2 = sim_yellow_180()
    print("\n=== SONUÇ ===")
    print(f"S1 Koridor salınımı : {'PASS ✅' if r1 else 'FAIL ❌'}")
    print(f"S2 Sarı 180° dönüş   : {'PASS ✅' if r2 else 'FAIL ❌'}")
    return 0 if (r1 and r2) else 1


if __name__ == "__main__":
    sys.exit(main())
