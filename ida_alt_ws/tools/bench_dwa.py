"""Benchmark: DWA plan() performansı (swept-cell cache + tek geçiş).

Costmap: 30 duba (turuncu koridor + sarı engeller karışık). DwaPlanner yaml
parametreleri: sim_time=3.5, sim_step=0.25, vx_steps=7, yaw_steps=17 (119 aday),
bot_radius=1.6, n_cells=120. 100 ardışık plan() ölçülür; ilk tick (cache soğuk)
ayrıca raporlanır; cache boyutu ve isabet oranı basılır.
"""

import os
import sys
import time

# tools/'dan çalıştırıldığında ida_planning paketini bul (src/ida_planning).
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src", "ida_planning"))

from ida_planning.costmap import COST_OBSTACLE, COST_ORANGE, CostMap
from ida_planning.dwa import DwaCommand, DwaPlanner


def seed_costmap(cm, n_buoys=30):
    cm.reset()
    for i in range(n_buoys // 2):
        fwd = 3.0 + i * 2.5
        cm._add_obstacle(fwd, -2.2, COST_ORANGE, "orange")
        cm._add_obstacle(fwd, 2.2, COST_ORANGE, "orange")
    for i in range(n_buoys // 3):
        fwd = 4.0 + i * 3.0
        cm._add_obstacle(fwd, 0.6 * (1 if i % 2 == 0 else -1), COST_OBSTACLE, "yellow")


def _warmup(cm, planner, last):
    """Ölçümden ÖNCE cache'i ısıt (soğuk ilk plan ayrıca raporlanır)."""
    planner.plan((8.0, 0.0), cm, last)


def main():
    cm = CostMap(size_m=30.0, cell_m=0.25, bot_radius_m=0.6, safety_m=0.5)
    cm.set_speed(0.8)
    seed_costmap(cm, 30)
    planner = DwaPlanner(
        max_speed_mps=0.8, max_yaw_rate_deg_s=45.0, vx_steps=7, yaw_steps=17,
        sim_time_s=3.5, sim_step_s=0.25, bot_radius_m=1.6, n_cells=120,
    )
    last = DwaCommand(vx=0.5, yaw_rate=0.0, action="start", score=0.5)
    goal = (8.0, 0.0)

    # SOĞUK ilk plan (cache boş): tek ölçüm.
    t0 = time.perf_counter()
    first = planner.plan(goal, cm, last)
    t_first = time.perf_counter() - t0

    # Ölçüm öncesi ekstra ısıtma (cache dolu + last_cmd senkron).
    _warmup(cm, planner, last)

    n_plans = 100
    t0 = time.perf_counter()
    for _ in range(n_plans):
        planner.plan(goal, cm, last)
        last = planner.last_cmd
    t_warm = (time.perf_counter() - t0) / n_plans

    cache = planner._swept_cache
    total_lookups = 0
    hits = 0
    # İsabet oranı: 100 tick'in her birinde 119 aday -> cache sorgusu sayısı.
    for _ in range(100):
        cands = planner.candidates(goal, planner.last_cmd)
        for cand in cands:
            total_lookups += 1
            key = (round(cand.vx, 5), round(cand.yaw_rate, 5), cm.n)
            if key in cache:
                hits += 1

    print(f"COLD_FIRST_PLAN_MS={t_first * 1000:.2f}")
    print(f"WARM_PLAN_MS={t_warm * 1000:.2f}  (avg of {n_plans})")
    print(f"CACHE_SIZE={len(cache)}  (max={planner._swept_cache_max})")
    print(f"HIT_RATE={hits / total_lookups:.4f}  ({hits}/{total_lookups})")
    print(f"FIRST_CMD={first.action}")


if __name__ == "__main__":
    main()
