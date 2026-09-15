"""布局优化器：锁定物件不动，搜索其余家具的摆放并评分排序。

评分四要素（与需求对应）：
  1. 全部家具可达：每件家具到入户门必须存在 A* 网格路径；
  2. 最小通行余量：所有路径距障碍/墙的最小侧隙（越大越好）；
  3. 转向次数：所有家具路径的转弯总数（越少越好）；
  4. 占地紧凑度：未占用的中部空地 / 家具包围盒面积（越紧凑越高分）。
"""
import heapq
import math
import random

from geometry import (
    rect_poly, convex_overlap_depth, point_in_poly, polys_overlap,
    sub, scale, add, unit, poly_center, convex_hull, poly_area,
)
from planner import Plan

CELL = 100          # A* 网格 100mm
WALL_MARGIN = 60    # 通行余量门槛 mm
CORRIDOR_KEEP = 300  # 至少保留的通道宽度


# ---------------------------------------------------------------- A*

def build_grid(plan, blockers, bounds=None, cell=CELL):
    """栅格化可行区域：True = 可通行中心格（已扣除障碍与墙边余量）。"""
    spaces = plan.walkable_polys()
    if bounds is None:
        xs = [v[0] for sp in spaces for v in sp]
        ys = [v[1] for sp in spaces for v in sp]
        minx, maxx, miny, maxy = min(xs), max(xs), min(ys), max(ys)
    else:
        minx, miny, maxx, maxy = bounds
    nx = int((maxx - minx) / cell) + 1
    ny = int((maxy - miny) / cell) + 1

    walls = plan.wall_segments(gate_margin=WALL_MARGIN // 2)

    def free(x, y):
        if not any(point_in_poly((x, y), sp) for sp in spaces):
            return False
        for bk in blockers:
            if point_in_poly((x, y), bk["poly"]):
                return False
        for a, b, _ in walls:
            from geometry import point_seg_dist
            d, _ = point_seg_dist((x, y), a, b)
            if d < WALL_MARGIN:
                return False
        return True

    grid = [[free(minx + i * cell + cell / 2, miny + j * cell + cell / 2)
             for j in range(ny)] for i in range(nx)]
    return grid, (minx, miny, maxx, maxy)


def astar(grid, start, goal, cell=CELL):
    """8 邻域 A*。start/goal 为网格坐标 (i,j)。返回 (路径格点, 转向次数) 或 None。"""
    nx, ny = len(grid), len(grid[0]) if grid else 0

    def ok(i, j):
        return 0 <= i < nx and 0 <= j < ny and grid[i][j]

    if not (ok(*start) and ok(*goal)):
        return None

    def key(p):
        return p[0] * ny + p[1]

    openh = [(0, 0, start, None, 0)]
    came = {}
    gscore = {key(start): 0}
    counter = 0
    while openh:
        _, g, cur, prev_dir, turns = heapq.heappop(openh)
        if cur == goal:
            # 回溯
            path = [cur]
            k = key(cur)
            while k in came:
                p, pd = came[k]
                path.append(p)
                k = key(p)
            path.reverse()
            return path, turns
        if g > gscore.get(key(cur), 1e18):
            continue
        i, j = cur
        for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1),
                       (1, 1), (1, -1), (-1, 1), (-1, -1)):
            ni, nj = i + di, j + dj
            if not ok(ni, nj):
                continue
            if di and dj and not (ok(i + di, j) and ok(i, j + dj)):
                continue  # 不能贴角斜穿
            step = math.hypot(di, dj) * cell
            ng = g + step
            d = (di, dj)
            nturn = turns + (1 if prev_dir is not None and d != prev_dir else 0)
            nk = key((ni, nj))
            if ng + nturn * 50 < gscore.get(nk, 1e18):
                gscore[nk] = ng
                came[nk] = (cur, d)
                h = math.hypot(ni - goal[0], nj - goal[1]) * cell
                f = ng + h + nturn * 50
                counter += 1
                heapq.heappush(openh, (f, ng, (ni, nj), d, nturn))
    return None


def world_to_grid(x, y, origin, cell=CELL):
    minx, miny = origin[0], origin[1]
    return (int((x - minx) / cell), int((y - miny) / cell))


# ---------------------------------------------------------------- 布局生成

def candidate_positions(plan, furn, blockers, other_furniture, cell=150):
    """沿墙边/障碍边生成候选摆放位（贴墙优先，减少占地）。"""
    spaces = plan.walkable_polys()
    cands = []
    degs = [0, 90] if abs(furn["w"] - furn["d"]) > 5 else [0]

    # 贴各空间四边内缩
    for poly in spaces:
        n = len(poly)
        for ei in range(n):
            a, b = tuple(poly[ei]), tuple(poly[(ei + 1) % n])
            ev = sub(b, a)
            L = math.hypot(*ev)
            eu = unit(ev)
            nx = (-eu[1], eu[0])
            # 沿边每 cell 放一个，内缩 max(w,d)/2 + 余量
            off = max(furn["w"], furn["d"]) / 2.0 + WALL_MARGIN
            t = cell
            while t < L - cell:
                base = add(a, scale(eu, t))
                for s in (1, -1):
                    p = add(base, scale(nx, off * s))
                    for deg in degs:
                        cands.append((p[0], p[1], deg))
                t += cell
    # 障碍旁
    for bk in blockers + other_furniture:
        bp = bk["poly"]
        c = poly_center(bp)
        n = len(bp)
        for i in range(n):
            e = sub(bp[(i + 1) % n], bp[i])
            eu = unit(e)
            nrm = (-eu[1], eu[0])
            off = max(furn["w"], furn["d"]) / 2.0 + 40
            for s in (1, -1):
                for deg in degs:
                    p = add(c, scale(nrm, off * s))
                    cands.append((p[0], p[1], deg))
    return cands


def placement_valid(plan, furn, x, y, deg, blockers, placed_polys):
    poly = rect_poly(x, y, furn["w"], furn["d"], deg)
    # 必须整体在某空间内
    if not any(all(point_in_poly(v, sp) for v in poly) for sp in plan.walkable_polys()):
        return False, poly
    # 不与任何块体重叠
    for bk in blockers:
        if convex_overlap_depth(poly, bk["poly"]):
            return False, poly
    for pp in placed_polys:
        if convex_overlap_depth(poly, pp):
            return False, poly
    return True, poly


def min_gap_around(poly, plan, blockers, polys):
    """poly 四周到墙/其他家具的最小间隙（mm）。"""
    from geometry import point_seg_dist
    best = 1e9
    c = poly_center(poly)
    for a, b, _ in plan.wall_segments(gate_margin=0):
        d, _ = point_seg_dist(c, a, b)
        best = min(best, d)
    for bk in blockers + [{"poly": p} for p in polys]:
        d, _ = point_seg_dist(c, bk["poly"]) if "poly" in bk else point_seg_dist(c, bk)
        best = min(best, d)
    # 减去家具自身等效半宽
    return best


# ---------------------------------------------------------------- 主流程

def optimize_layout(data, furniture, entry_door_id=None, samples=60, seed=7):
    """furniture: [{id,name,w,d,h,locked,x,y,deg, clearance}]。

    返回 {ok, ranked:[{score, reachable, minClearance, turns, compactness,
                       placements, paths, unreachable:[]}]}
    """
    random.seed(seed)
    plan = Plan(data)
    locked = [f for f in furniture if f.get("locked")]
    movable = [f for f in furniture if not f.get("locked")]

    locked_polys = []
    locked_blockers = []
    for f in locked:
        if not f.get("x") or not f.get("y"):
            continue  # 锁定但无摆放位：忽略（前端会要求先放置）
        poly = rect_poly(f["x"], f["y"], f["w"], f["d"], f.get("deg", 0))
        locked_polys.append(poly)
        locked_blockers.append({
            "poly": poly, "id": f["id"], "name": f.get("name", f["id"]),
            "height": f.get("h", 2000), "kind": "locked",
        })

    base_blockers = plan.blocker_polys(extra=locked_blockers)

    # 入户门 -> 室内出发点
    door = None
    for d in plan.doors:
        if d.get("id") == entry_door_id:
            door = d
            break
    if door is None and plan.doors:
        door = plan.doors[0]
    spaces = plan.walkable_polys()
    start_world = entry_start_point(door, spaces)

    xs = [v[0] for sp in spaces for v in sp]
    ys = [v[1] for sp in spaces for v in sp]
    bounds = (min(xs), min(ys), max(xs), max(ys))

    # 每件家具的候选位
    options = {}
    for f in movable:
        cands = candidate_positions(plan, f, base_blockers, [])
        # 去重（近似）
        uniq = []
        seen = set()
        for x, y, deg in cands:
            k = (round(x / 50), round(y / 50), deg)
            if k not in seen:
                seen.add(k)
                uniq.append((x, y, deg))
        random.shuffle(uniq)
        options[f["id"]] = uniq[:samples]

    if not movable:
        return {"ok": True, "ranked": [_score_empty(plan, locked_blockers, start_world,
                                                    bounds, locked_polys, furniture)]}

    # 贪心 + 随机重启：按面积大的先放
    order = sorted(range(len(movable)),
                   key=lambda i: -(movable[i]["w"] * movable[i]["d"]))
    ranked = []
    seen_layouts = set()

    for attempt in range(max(1, samples)):
        placed_polys = list(locked_polys)
        placements = {f["id"]: None for f in movable}
        success = True
        for idx in order:
            f = movable[idx]
            shift = attempt % max(1, len(options[f["id"]]))
            chosen = None
            for k, (x, y, deg) in enumerate(options[f["id"]]):
                x, y, deg = options[f["id"]][(k + shift) % len(options[f["id"]])]
                ok, poly = placement_valid(plan, f, x, y, deg, base_blockers, placed_polys)
                if ok:
                    chosen = (x, y, deg, poly)
                    break
            if chosen is None:
                success = False
                break
            x, y, deg, poly = chosen
            placements[f["id"]] = {"id": f["id"], "x": round(x, 1),
                                   "y": round(y, 1), "deg": deg}
            placed_polys.append(poly)
        if not success:
            continue

        sig = tuple(sorted((p["id"], round(p["x"] / 100), round(p["y"] / 100), p["deg"])
                           for p in placements.values()))
        if sig in seen_layouts:
            continue
        seen_layouts.add(sig)

        scored = _score_layout(plan, data, furniture, movable, placements,
                               placed_polys, base_blockers, start_world, bounds)
        ranked.append(scored)
        if len(ranked) >= 12:
            break

    ranked.sort(key=lambda r: (
        -len(r["reachable"]),          # 可达家具最多
        r["unreachableCount"] * 10 ** 9,
        -r["minClearance"],             # 余量最大
        r["turns"],                     # 转向最少
        -r["compactness"],              # 越紧凑分越高
    ))
    return {"ok": True, "ranked": ranked[:8]}


def entry_start_point(door, spaces, inward=400):
    wu = unit((door.get("wx", 1.0), door.get("wy", 0.0)))
    nx = (-wu[1], wu[0])
    p = (door["x"], door["y"])
    for s in (1, -1):
        q = add(p, scale(nx, inward * s * (door.get("swing", 1) or 1)))
        if any(point_in_poly(q, sp) for sp in spaces):
            return q
    return q


def _score_layout(plan, data, furniture, movable, placements, placed_polys,
                  base_blockers, start_world, bounds):
    # 路径可达性：每件已放家具作为目标（格点取家具中心），其余家具视作阻挡
    all_polys = placed_polys
    reachable, unreachable, total_turns, min_gap = [], [], 0, 1e9
    paths_out = {}

    for f in furniture:
        if f.get("locked"):
            continue
        pl = placements[f["id"]]
        if pl is None:
            unreachable.append(f["id"])
            continue
        blockers = [bk for bk in base_blockers]
        # 把别的家具加入阻挡，当前家具本身不挡路
        for g in movable:
            if g["id"] == f["id"] or placements[g["id"]] is None:
                continue
            gp = placements[g["id"]]
            blockers.append({"poly": rect_poly(gp["x"], gp["y"], g["w"], g["d"], gp["deg"]),
                             "id": g["id"], "name": g.get("name", g["id"])})
        grid, bnds = build_grid(plan, blockers, bounds)
        origin = (bnds[0], bnds[1])
        s = world_to_grid(start_world[0], start_world[1], origin)
        gpos = world_to_grid(pl["x"], pl["y"], origin)
        # 找最近的可行目标格
        gpos = nearest_free(grid, gpos)
        spos = nearest_free(grid, s)
        result = astar(grid, spos, gpos) if spos and gpos else None
        if result is None:
            unreachable.append(f["id"])
        else:
            path_cells, turns = result
            reachable.append(f["id"])
            total_turns += turns
            paths_out[f["id"]] = [grid_to_world(c, origin) for c in path_cells]
            # 路径侧隙
            blockers_full = blockers
            for wx, wy in path_cells[::3]:
                wx, wy = grid_to_world((wx, wy), origin)
                from geometry import point_seg_dist
                for a, b, _ in plan.wall_segments():
                    d, _ = point_seg_dist((wx, wy), a, b)
                    min_gap = min(min_gap, d)

    # 紧凑度：家具占地面积 / 包围盒面积（越高=围合越紧、空地集中）
    if all_polys:
        xs = [v[0] for p in all_polys for v in p]
        ys = [v[1] for p in all_polys for v in p]
        box = (max(xs) - min(xs)) * (max(ys) - min(ys)) or 1
        used = sum(poly_area(p) for p in all_polys)
        compactness = used / box
    else:
        compactness = 0

    return {
        "reachable": reachable,
        "unreachable": unreachable,
        "unreachableCount": len(unreachable),
        "minClearance": None if min_gap == 1e9 else round(min_gap, 1),
        "turns": total_turns,
        "compactness": round(compactness, 3),
        "placements": [placements[f["id"]] for f in movable],
        "paths": paths_out,
        "score": round(len(reachable) * 1000 + (min_gap if min_gap < 1e9 else 0)
                       - total_turns * 20 + compactness * 500, 1),
    }


def nearest_free(grid, cell):
    nx, ny = len(grid), len(grid[0])
    i, j = cell
    if 0 <= i < nx and 0 <= j < ny and grid[i][j]:
        return cell
    for r in range(1, 8):
        for di in range(-r, r + 1):
            for dj in range(-r, r + 1):
                ni, nj = i + di, j + dj
                if 0 <= ni < nx and 0 <= nj < ny and grid[ni][nj]:
                    return ni, nj
    return None


def grid_to_world(cell, origin, csize=CELL):
    i, j = cell
    return (round(origin[0] + i * csize + csize / 2, 1),
            round(origin[1] + j * csize + csize / 2, 1))


def _score_empty(plan, blockers, start_world, bounds, locked_polys, furniture):
    return {
        "reachable": [f["id"] for f in furniture if f.get("locked")],
        "unreachable": [],
        "unreachableCount": 0,
        "minClearance": None,
        "turns": 0,
        "compactness": 1.0,
        "placements": [],
        "paths": {},
        "score": 0,
    }
