"""Eşdeğerlik kontrolü: ESKİ (build_swept_cells + collides_only_real + avg/unknown)
akışı ile YENİ (cache + tek geçiş) akışının aynı girişte bit-ayar aynı cand ürettiğini
doğrular. 3 costmap durumu x 20'şer aday; tüm sonuç alanları karşılaştırılır.
"""

import math

from ida_planning.costmap import COST_GOAL, COST_OBSTACLE, COST_ORANGE, COST_UNKNOWN, CostMap
from ida_planning.dwa import DwaPlanner, build_swept_cells


def old_flow(planner, cand, costmap):
    """Eski akış: build_swept_cells -> collides_only_real -> avg/unknown döngüsü."""
    swept_cols, swept_rows = build_swept_cells(
        planner.sim_time_s, planner.sim_step_s, cand.vx, cand.yaw_rate,
        planner.offset_cells, costmap.n, planner.cell_m,
    )
    if costmap.collides_only_real(swept_cols, swept_rows):
        return {"collides": True, "score": float("-inf"), "swept_cost": 0.0, "unknown_count": 0}
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
    return {"collides": False, "score": 1.0, "swept_cost": avg, "unknown_count": unknown_count}


def new_flow(planner, cand, costmap):
    """Yeni akış: _swept_cells (cache) + tek geçiş."""
    swept_cols, swept_rows = planner._swept_cells(cand.vx, cand.yaw_rate, costmap.n)
    collides = False
    total = 0.0
    n_real = 0
    unknown_count = 0
    inflated = costmap.inflated
    cells = costmap.cells
    n = costmap.n
    for c, r in zip(swept_cols, swept_rows):
        if not (0 <= c < n and 0 <= r < n):
            collides = True
            break
        v = inflated[r][c]
        if v >= 8:
            collides = True
            break
        if v >= 3:
            total += v
            n_real += 1
        if cells[r][c] == COST_UNKNOWN:
            unknown_count += 1
    if collides:
        return {"collides": True, "score": float("-inf"), "swept_cost": 0.0, "unknown_count": 0}
    avg = total / max(1, n_real) if n_real > 0 else 0.0
    return {"collides": False, "score": 1.0, "swept_cost": avg, "unknown_count": unknown_count}


def fill_buoy(costmap, forward, lateral, cost, tag):
    """Costmap'in _add_obstacle'ı yerine doğrudan hücre + şişirme yazar (deterministik)."""
    c, r = costmap.world_to_cell(forward, lateral)
    if not costmap.in_bounds(c, r):
        return
    costmap.cells[r][c] = cost
    costmap.tags[r][c] = tag
    costmap.inflated[r][c] = max(costmap.inflated[r][c], float(cost))


def make_costmap(seed):
    """3 costmap durumu: boş / turuncu koridor / sarı engel önü."""
    cm = CostMap(size_m=30.0, cell_m=0.25, bot_radius_m=0.6, safety_m=0.5)
    if seed == 1:  # turuncu koridor: solda ve sağda turuncu dubalar
        for fwd in (3.0, 6.0, 9.0):
            fill_buoy(cm, fwd, -2.0, COST_ORANGE, "orange")
            fill_buoy(cm, fwd, 2.0, COST_ORANGE, "orange")
    elif seed == 2:  # sarı engel önde
        fill_buoy(cm, 5.0, 0.0, COST_OBSTACLE, "yellow")
        fill_buoy(cm, 5.0, -1.5, COST_UNKNOWN, "unknown")
    return cm


def main():
    failures = 0
    for seed in range(3):
        costmap = make_costmap(seed)
        planner = DwaPlanner(
            max_speed_mps=0.8, max_yaw_rate_deg_s=45.0, vx_steps=7, yaw_steps=17,
            sim_time_s=3.5, sim_step_s=0.25, bot_radius_m=1.6, n_cells=120,
        )
        cands = planner.candidates((5.0, 0.0), None)
        for cand in cands[:20]:
            o = old_flow(planner, cand, costmap)
            n = new_flow(planner, cand, costmap)
            for k in o:
                a, b = o[k], n[k]
                if isinstance(a, float) and math.isfinite(a):
                    ok = abs(a - b) < 1e-12
                else:
                    ok = a == b
                if not ok:
                    failures += 1
                    print(f"FARK seed={seed} cand.vx={cand.vx:.4f} yaw={cand.yaw_rate:.4f} key={k} old={a} new={b}")
    if failures == 0:
        print("EQUIV_OK: 3 costmap states x 20 candidates = 60 candidates, all fields bit-identical.")
    else:
        print(f"EQUIV_ERROR: {failures} differences found")
        raise SystemExit(1)


if __name__ == "__main__":
    main()
