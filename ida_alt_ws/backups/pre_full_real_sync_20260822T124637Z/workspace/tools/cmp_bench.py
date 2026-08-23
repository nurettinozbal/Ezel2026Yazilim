"""Old vs new score_candidate full-plan benchmark on identical input.

Legacy: build_swept_cells + collides_only_real + avg/unknown + avoid (eski akış).
New: _swept_cells cache + tek geçiş + avoid (uygulanan akış).
Aynı costmap/aday/goal; 100 plan ortalaması karşılaştırılır.
"""

import time

from ida_planning.costmap import COST_OBSTACLE, COST_ORANGE, COST_UNKNOWN, CostMap
from ida_planning.dwa import DwaPlanner, build_swept_cells
from ida_planning.geo import clamp


class LegacyDwaPlanner(DwaPlanner):
    """Eski score_candidate (cache'siz, ayrı pass'ler) — kıyas tabanı."""

    def score_candidate(self, cand, goal, last, costmap):
        swept_cols, swept_rows = build_swept_cells(
            self.sim_time_s, self.sim_step_s, cand.vx, cand.yaw_rate,
            self.offset_cells, costmap.n, self.cell_m,
        )
        if costmap.collides_only_real(swept_cols, swept_rows):
            cand.collides = True
            cand.score = float("-inf")
            return cand
        if swept_cols:
            total = 0.0
            n_real = 0
            unknown_count = 0
            for c, r in zip(swept_cols, swept_rows):
                v = costmap.inflated[r][c]
                if v >= 3:
                    total += v
                    n_real += 1
                if costmap.cells[r][c] == COST_UNKNOWN:
                    unknown_count += 1
            avg = total / max(1, n_real) if n_real > 0 else 0.0
        else:
            avg = 0.0
            unknown_count = 0
        avoid = 0.0
        min_dist = float("inf")
        inflated = costmap.inflated
        half_n = costmap.n / 2.0
        cell_m = costmap.cell_m
        for c, r in zip(swept_cols, swept_rows):
            row = inflated[r]
            if row[c] >= 8:
                dist = math.hypot((c - half_n) * cell_m, (half_n - r) * cell_m)
                min_dist = dist if dist < min_dist else min_dist
        if math.isfinite(min_dist):
            avoid = 1.0 / (min_dist + 0.5)
        cand.avoid = avoid
        cand.score = 1.0
        return cand


def seed(cm):
    cm.set_speed(0.8)
    cm.reset()
    for i in range(15):
        fwd = 3.0 + i * 2.5
        cm._add_obstacle(fwd, -2.2, COST_ORANGE, "orange")
        cm._add_obstacle(fwd, 2.2, COST_ORANGE, "orange")
    for i in range(10):
        fwd = 4.0 + i * 3.0
        cm._add_obstacle(fwd, 0.6 * (1 if i % 2 == 0 else -1), COST_OBSTACLE, "yellow")


def bench(make_planner, name):
    cm = CostMap(size_m=30.0, cell_m=0.25, bot_radius_m=0.6, safety_m=0.5)
    seed(cm)
    p = make_planner()
    last = p.last_cmd
    # cache'i ısıt (new) — legacy'de cache yok, her tick yeniden üretir.
    p.plan((8.0, 0.0), cm, last)
    n = 100
    t0 = time.perf_counter()
    for _ in range(n):
        p.plan((8.0, 0.0), cm, last)
        last = p.last_cmd
    ms = (time.perf_counter() - t0) / n * 1000
    print(f"{name}: {ms:.2f} ms/plan  (cache={len(p._swept_cache)})")
    return ms


if __name__ == "__main__":
    import math

    def mk_new():
        return DwaPlanner(
            max_speed_mps=0.8, max_yaw_rate_deg_s=45.0, vx_steps=7, yaw_steps=17,
            sim_time_s=3.5, sim_step_s=0.25, bot_radius_m=1.6, n_cells=120,
        )

    def mk_legacy():
        return LegacyDwaPlanner(
            max_speed_mps=0.8, max_yaw_rate_deg_s=45.0, vx_steps=7, yaw_steps=17,
            sim_time_s=3.5, sim_step_s=0.25, bot_radius_m=1.6, n_cells=120,
        )

    a = bench(mk_new, "NEW")
    b = bench(mk_legacy, "LEGACY")
    print(f"SPEEDUP: {(b / a):.2f}x  (legacy/new)")
