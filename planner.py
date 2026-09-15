"""方案模型：从平面 JSON 派生墙体、门洞、障碍，并执行姿态/扫掠/门扇/净高检查。"""
import math

from geometry import (
    rect_poly, point_in_poly, polys_overlap, convex_overlap_depth,
    segments_intersect, ray_hits_segment, segment_overlap_with_poly,
    sub, add, scale, dist, heading, unit, normal, poly_center,
    swept_envelope, poly_area, poly_contains_poly, EPS,
)

# 冲突类型
WALL = "wall"          # 与墙体冲突
OBSTACLE = "obstacle"  # 与固定障碍 / 其他家具冲突
SWING = "door_swing"   # 被门扇开启范围扫到
HEIGHT = "height"      # 抬起后超高（门楣 / 限高）
OUTSIDE = "outside"    # 完全离开可行区域（穿出建筑）


# ---------------------------------------------------------------- 方案实体

class Plan:
    def __init__(self, data):
        self.data = data
        self.spaces = data.get("spaces", [])
        self.doors = data.get("doors", [])
        self.obstacles = data.get("obstacles", [])

    # -- 墙体：每个空间的边，扣除门洞后得到墙段 --
    def wall_segments(self, gate_margin=0.0):
        """返回 [(a,b,space)]。门洞沿所有与它重叠的墙边剪开，
        因此共用隔墙上的门会同时剪开两侧空间的边。"""
        gates = []
        for door in self.doors:
            wu = unit((door.get("wx", 1.0), door.get("wy", 0.0)))
            p = (door["x"], door["y"])
            hw = door["width"] / 2.0 + gate_margin
            gates.append((add(p, scale(wu, -hw)), add(p, scale(wu, hw)), door))

        walls = []
        for sp in self.spaces:
            poly = sp["poly"]
            n = len(poly)
            for i in range(n):
                e1, e2 = tuple(poly[i]), tuple(poly[(i + 1) % n])
                ev = sub(e2, e1)
                L = math.hypot(*ev)
                eu = scale(ev, 1.0 / L) if L > EPS else (0, 0)
                cuts = []
                for ga, gb, door in gates:
                    # 门中心投影到该边；法向贴合且投影落在边上 => 沿边剪开半门宽
                    mp = (door["x"], door["y"])
                    if self._point_near_edge(mp, e1, e2, 70.0):
                        m = sub(mp, e1)
                        c = m[0] * eu[0] + m[1] * eu[1]
                        half = door["width"] / 2.0 + gate_margin
                        cuts.append((max(0.0, c - half), min(L, c + half)))
                if cuts:
                    cuts.sort()
                    merged = []
                    for s, t in cuts:
                        if merged and s <= merged[-1][1]:
                            merged[-1] = (merged[-1][0], max(merged[-1][1], t))
                        else:
                            merged.append((s, t))
                    cur = 0.0
                    for s, t in merged:
                        if s - cur > 1.0:
                            walls.append((add(e1, scale(eu, cur)),
                                          add(e1, scale(eu, s)), sp))
                        cur = max(cur, t)
                    if L - cur > 1.0:
                        walls.append((add(e1, scale(eu, cur)),
                                      add(e1, scale(eu, L)), sp))
                else:
                    walls.append((e1, e2, sp))
        return walls

    @staticmethod
    def _point_near_edge(p, a, b, tol):
        ab = sub(b, a)
        L2 = ab[0] ** 2 + ab[1] ** 2
        if L2 < EPS:
            return dist(p, a) <= tol
        t = max(0.0, min(1.0, (sub(p, a)[0] * ab[0] + sub(p, a)[1] * ab[1]) / L2))
        q = add(a, scale(ab, t))
        return dist(p, q) <= tol

    # -- 门：门扇矩形、扫掠扇形 --
    def door_leaf(self, door, angle=None):
        if angle is None:
            angle = door.get("openAngle", 90.0)
        return door_panel_poly(door, angle)

    def door_sweep(self, door, steps=24):
        """门扇从闭合到 openAngle 的扫掠区域（多边形列表：扇形三角扇 + 全开矩形）。"""
        return door_swing_polys(door, steps)

    # -- 固定障碍矩形（含锁定家具时由调用方追加）--
    def blocker_polys(self, extra=None, exclude_id=None):
        blockers = []
        for ob in self.obstacles:
            blockers.append({
                "poly": rect_poly(ob["x"], ob["y"], ob["w"], ob["d"], ob.get("deg", 0)),
                "height": ob.get("height", 2000),
                "id": ob.get("id", "ob"),
                "kind": "obstacle",
                "name": ob.get("name", "障碍"),
            })
        for b in extra or []:
            if b.get("id") == exclude_id:
                continue
            blockers.append(b)
        return blockers

    def walkable_polys(self):
        """可行区域多边形（所有空间并集，门是空间之间的连通口）。"""
        return [sp["poly"] for sp in self.spaces]


# ---------------------------------------------------------------- 门扇几何

def _door_frame(door):
    """门坐标系：wu=门洞方向，n=其法向；hinge=门轴（在 +wu 端）；
    swing=+1 时扇叶向 +n 侧开启，-1 时向 -n 侧开启。"""
    wu = unit((door.get("wx", 1.0), door.get("wy", 0.0)))
    n = (-wu[1], wu[0])
    side = door.get("swing", 1) or 1
    center = (door["x"], door["y"])
    hinge = add(center, scale(wu, door["width"] / 2.0))
    return wu, n, side, center, hinge


def door_panel_poly(door, angle=None):
    """开启 angle 度时的门扇矩形（40mm 厚）；0 度沿 -wu 闭合。"""
    if angle is None:
        angle = door.get("openAngle", 90.0)
    wu, n, side, _center, hinge = _door_frame(door)
    a = math.radians(angle)
    dirv = (-wu[0] * math.cos(a) + side * n[0] * math.sin(a),
            -wu[1] * math.cos(a) + side * n[1] * math.sin(a))
    thick = 40.0
    end = add(hinge, scale(dirv, door["width"]))
    hn = scale(n, thick * 0.5)
    return [add(hinge, hn), add(end, hn), add(end, scale(hn, -1)),
            add(hinge, scale(hn, -1))], hinge, dirv


def door_swing_polys(door, steps=24):
    """门扇闭合到 openAngle 的扫掠近似：绕铰链的三角扇。"""
    wu, n, side, _center, hinge = _door_frame(door)
    max_ang = abs(float(door.get("openAngle", 90.0)))
    tris = []

    def tip(rad):
        v = (-wu[0] * math.cos(rad) + side * n[0] * math.sin(rad),
             -wu[1] * math.cos(rad) + side * n[1] * math.sin(rad))
        return add(hinge, scale(v, door["width"]))

    prev = 0.0
    for k in range(1, steps + 1):
        cur = max_ang * k / steps
        tris.append([hinge, tip(math.radians(prev)), tip(math.radians(cur))])
        prev = cur
    return tris, hinge


def door_gate_segment(door):
    """门洞净开口线段。"""
    wu = unit((door.get("wx", 1.0), door.get("wy", 0.0)))
    p = (door["x"], door["y"])
    return add(p, scale(wu, -door["width"] / 2.0)), add(p, scale(wu, door["width"] / 2.0)), wu


# ---------------------------------------------------------------- 家具姿态

def furniture_poly(furn, pose):
    return rect_poly(pose["x"], pose["y"], furn["w"], furn["d"], pose.get("deg", 0))


def tilted_height(furn, pose):
    """倾斜搬运时的有效竖向高度（mm）。简化模型：h*cos+长边*sin，上限按对角。"""
    pitch = max(0.0, min(90.0, float(pose.get("pitch", 0))))
    h = float(furn["h"])
    long_side = max(float(furn["w"]), float(furn["d"]))
    if pitch <= 0:
        return h
    a = math.radians(pitch)
    return h * math.cos(a) + long_side * math.sin(a)


# ---------------------------------------------------------------- 使用包络（柜门/抽屉/椅子）

def _usage_polys(furn, x, y, deg):
    """家具在 (x,y,deg) 处的使用包络多边形列表（不含家具本体）。

    furn["usage"] 可含：
      doors:   [{edge:"front|back|left|right", open:0..180, reach:mm}]
      drawers: [{edge:..., reach:mm}]
      chair:   {reach:mm}（前方入座+后移区）
    """
    from geometry import convex_hull
    w, d = float(furn["w"]), float(furn["d"])
    a = math.radians(deg)
    ca, sa = math.cos(a), math.sin(a)

    def R(lx, ly):
        return (x + lx * ca - ly * sa, y + lx * sa + ly * ca)

    # 四边外法线（局部坐标）：front=-y, back=+y, left=-x, right=+x
    edges = {
        "front": ((0, -1), (0, -d / 2.0), w),
        "back": ((0, 1), (0, d / 2.0), w),
        "left": ((-1, 0), (-w / 2.0, 0), d),
        "right": ((1, 0), (w / 2.0, 0), d),
    }
    polys = []
    usage = furn.get("usage") or {}

    for dr in usage.get("doors", []):
        n, anchor, edge_len = edges.get(dr.get("edge", "front"), edges["front"])
        reach = float(dr.get("reach", 600))
        open_ang = min(179.0, max(0.0, float(dr.get("open", 90))))
        nx, ny = n
        # 切线 t：front/back 向右(+x)，left/right 向下(+y)；门轴在 -t 端
        tx, ty = (ny, -nx)
        hx0 = anchor[0] - tx * edge_len / 2.0
        hy0 = anchor[1] - ty * edge_len / 2.0
        ar = math.radians(open_ang)
        closed = (hx0 + tx * edge_len, hy0 + ty * edge_len)
        tip = (hx0 + (tx * math.cos(ar) - nx * math.sin(ar)) * edge_len,
               hy0 + (ty * math.cos(ar) - ny * math.sin(ar)) * edge_len)
        out_tip = (tip[0] + nx * reach * 0.25, tip[1] + ny * reach * 0.25)
        out_closed = (closed[0] + nx * reach * 0.25,
                      closed[1] + ny * reach * 0.25)
        polys.append([R(*p) for p in
                      convex_hull([(hx0, hy0), closed, tip, out_tip, out_closed])])

    for dr in usage.get("drawers", []):
        n, anchor, edge_len = edges.get(dr.get("edge", "front"), edges["front"])
        reach = float(dr.get("reach", 700))
        nx, ny = n
        ax, ay = anchor
        rect = [
            (ax - edge_len / 2, ay), (ax + edge_len / 2, ay),
            (ax + edge_len / 2 + nx * reach, ay + ny * reach),
            (ax - edge_len / 2 + nx * reach, ay + ny * reach),
        ]
        polys.append([R(*p) for p in rect])

    ch = usage.get("chair")
    if ch:
        reach = float(ch.get("reach", 700))
        seat = float(ch.get("seat", 500))
        nx, ny = edges["front"][0]
        ax, ay = 0, -d / 2.0
        rect = [
            (ax - w / 2.0, ay), (ax + w / 2.0, ay),
            (ax + w / 2.0 + nx * (reach + seat), ay + ny * (reach + seat)),
            (ax - w / 2.0 + nx * (reach + seat), ay + ny * (reach + seat)),
        ]
        polys.append([R(*p) for p in rect])

    return polys


def usage_polys(furn, x, y, deg):
    """公开接口：返回家具在指定摆放位的使用包络多边形（世界坐标）。"""
    return _usage_polys(furn, x, y, deg)


def usage_blockers(furniture_items):
    """把已放置家具的使用包络转成阻挡块（柜门/抽屉/椅子打开后的空间）。"""
    out = []
    for f in furniture_items:
        pos = f.get("placement")
        if not pos or not f.get("usage"):
            continue
        for k, poly in enumerate(_usage_polys(f, pos["x"], pos["y"], pos.get("deg", 0))):
            out.append({
                "poly": poly, "id": "%s:usage%d" % (f["id"], k),
                "name": "%s使用空间" % f.get("name", f["id"]),
                "height": 2000, "kind": "usage",
            })
    return out


# ---------------------------------------------------------------- 单姿态检查

def point_walkable(p, walkables):
    return any(point_in_poly(p, poly) for poly in walkables)


def _blocked_by_door_swing(plan, door, poly, passing_gate=False):
    """家具是否被门扇开启扫掠区挡住。

    门扇扫掠是绕铰链的扇形三角扇；三角跨过门洞延伸到对侧空间的部分不算阻挡
    （门后是另一个房间，搬运时家具可出现在门洞另一侧）。判断方式：三角上
    位于门“开启侧”且在任一空间内的点，落入家具多边形才算碰撞。家具处于
    门洞开口内（passing_gate）时，门线上的点全部忽略。
    """
    tris, hinge = door_swing_polys(door)
    ga, gb, wu = door_gate_segment(door)
    nrm = normal(wu)
    side = door.get("swing", 1) or 1
    spaces = plan.walkable_polys()

    def on_open_side(p):
        rel = sub(p, (door["x"], door["y"]))
        return (rel[0] * nrm[0] + rel[1] * nrm[1]) * side > -30

    def in_world(p):
        return any(point_in_poly(p, sp) for sp in spaces)

    def at_gate(p, tol=45.0):
        # 点在门洞开口线段附近
        from geometry import point_seg_dist
        d, _ = point_seg_dist(p, ga, gb)
        return d < tol

    for tri in tris:
        samples = list(tri)
        for i in range(3):
            samples.append(((tri[i][0] + tri[(i + 1) % 3][0]) / 2,
                            (tri[i][1] + tri[(i + 1) % 3][1]) / 2))
        for s in samples:
            if not on_open_side(s) or not in_world(s):
                continue
            if passing_gate and at_gate(s):
                continue
            if point_in_poly(s, poly):
                return True, hinge
    return False, hinge


def check_pose(plan, furn, pose, blockers=None, ignore_walls=False):
    """检查单个家具姿态，返回冲突列表。"""
    issues = []
    poly = furniture_poly(furn, pose)
    walls = [] if ignore_walls else plan.wall_segments()
    walkables = plan.walkable_polys()
    blockers = blockers if blockers is not None else plan.blocker_polys()

    # 1) 墙体碰撞（墙段已扣除门洞+余量；沿墙法向确有穿透才算，贴墙容差 20mm）
    for a, b, sp in walls:
        mid = add(a, scale(sub(b, a), 0.5))
        if point_in_poly(mid, poly):
            depth = convex_wall_depth(poly, a, b)
            if depth > 20.0:
                issues.append({
                    "type": WALL,
                    "point": {"x": mid[0], "y": mid[1]},
                    "depth": depth,
                    "remaining": depth,
                    "message": "与墙体冲突，侵入 %.0f mm" % depth,
                })

    # 2) 与障碍 / 其他家具
    for bk in blockers:
        if bk.get("id") == furn.get("id"):
            continue
        depth = convex_overlap_depth(poly, bk["poly"])
        if depth:
            c = poly_center(bk["poly"])
            issues.append({
                "type": OBSTACLE,
                "objectId": bk.get("id"),
                "objectName": bk.get("name", "障碍物"),
                "point": {"x": c[0], "y": c[1]},
                "depth": depth,
                "remaining": depth,
                "message": "与「%s」碰撞，重叠 %.0f mm" % (bk.get("name", "障碍物"), depth),
            })

    # 3) 离开可行区域（仅当全部空间都不包含中心时视为穿出；跨门居中允许）
    center = (pose["x"], pose["y"])
    if walkables and not point_walkable(center, walkables):
        # 家具整体不在任何空间内
        if not any(poly_contains_poly(wp, poly) or
                   all(point_in_poly(v, wp) for v in poly) for wp in walkables):
            issues.append({
                "type": OUTSIDE,
                "point": {"x": center[0], "y": center[1]},
                "depth": 0,
                "remaining": 0,
                "message": "家具位于建筑可行区域之外",
            })

    # 4) 门扇开启范围
    for door in plan.doors:
        ga0, gb0, _ = door_gate_segment(door)
        passing = bool(segment_overlap_with_poly((ga0, gb0), poly))
        if passing:
            # 正在过门：门已固定全开，只检查全开扇叶
            leaf, hinge, _ = door_panel_poly(door)
            depth = convex_overlap_depth(poly, leaf)
            if depth and depth > 20.0:
                issues.append({
                    "type": SWING,
                    "doorId": door.get("id"),
                    "point": {"x": hinge[0], "y": hinge[1]},
                    "depth": depth,
                    "remaining": depth,
                    "message": "撞到全开固定的门扇（重叠 %.0f mm）" % depth,
                })
        else:
            blocked, hinge = _blocked_by_door_swing(plan, door, poly, False)
            if blocked:
                issues.append({
                    "type": SWING,
                    "doorId": door.get("id"),
                    "point": {"x": hinge[0], "y": hinge[1]},
                    "depth": 0,
                    "remaining": 0,
                    "message": "位于门扇 %.0f° 开启扫掠范围内" % door.get("openAngle", 90),
                })
            leaf, hinge, _ = door_panel_poly(door)
            depth = convex_overlap_depth(poly, leaf)
            if depth and depth > 20.0:
                issues.append({
                    "type": SWING,
                    "doorId": door.get("id"),
                    "point": {"x": hinge[0], "y": hinge[1]},
                    "depth": depth,
                    "remaining": depth,
                    "message": "挡住门扇全开位置（重叠 %.0f mm）" % depth,
                })

    # 5) 净高 / 门楣
    eff_h = tilted_height(furn, pose)
    # 所在空间的净高（倾斜抬起后不能超过房间净高）
    center = (pose["x"], pose["y"])
    for sp in plan.spaces:
        if eff_h > (sp.get("height") or 99999) and point_in_poly(center, sp["poly"]):
            limit = sp["height"]
            issues.append({
                "type": HEIGHT,
                "point": {"x": center[0], "y": center[1]},
                "height": eff_h, "limit": limit,
                "remaining": limit - eff_h,
                "message": "「%s」净高 %.0f，抬起后 %.0f，超出 %.0f mm"
                           % (sp.get("name", "房间"), limit, eff_h, eff_h - limit),
            })
    # 与门开口重叠时检查门楣
    for door in plan.doors:
        ga, gb, _ = door_gate_segment(door)
        intervals = segment_overlap_with_poly((ga, gb), poly)
        clearance = door.get("clearHeight")
        if intervals and clearance and eff_h > clearance:
            issues.append({
                "type": HEIGHT,
                "doorId": door.get("id"),
                "point": {"x": door["x"], "y": door["y"]},
                "height": eff_h,
                "limit": clearance,
                "remaining": clearance - eff_h,
                "message": "门楣净高 %.0f，搬运高度 %.0f，超出 %.0f mm"
                           % (clearance, eff_h, eff_h - clearance),
            })

    return issues


def convex_wall_depth(poly, a, b):
    """估算家具多边形穿墙的法向穿透深度。"""
    nrm = normal(sub(b, a))
    # 取家具跨墙线两侧顶点，沿墙法向投影
    rel = [sub(v, a) for v in poly]
    signed = [r[0] * nrm[0] + r[1] * nrm[1] for r in rel]
    s_min, s_max = min(signed), max(signed)
    if s_min < 0 < s_max:
        return min(-s_min, s_max)
    return min(abs(s_min), abs(s_max))


# ---------------------------------------------------------------- 扫掠检查（相邻姿态之间）

def check_segment_between_poses(plan, furn, p0, p1, blockers=None):
    """两关键姿态之间的移动扫掠：转角扫掠包络 + 通道净宽 + 走廊侧隙。"""
    issues = []
    pa, pb = furniture_poly(furn, p0), furniture_poly(furn, p1)
    env = swept_envelope(pa, pb)
    walls = plan.wall_segments()
    blockers = blockers if blockers is not None else plan.blocker_polys()
    turn = abs((p1.get("deg", 0) - p0.get("deg", 0) + 180) % 360 - 180)

    # 1) 扫掠包络撞墙
    for a, b, sp in walls:
        mid = add(a, scale(sub(b, a), 0.5))
        if point_in_poly(mid, env):
            depth = convex_wall_depth(env, a, b)
            if depth > 20.0:
                issues.append({
                    "type": WALL,
                    "point": {"x": mid[0], "y": mid[1]},
                    "depth": depth,
                    "remaining": max(0.0, depth),
                    "turn": turn,
                    "message": ("转角扫掠撞墙" if turn > 8 else "移动中蹭墙")
                               + "，侵入 %.0f mm" % depth,
                })

    # 2) 扫掠包络撞障碍
    for bk in blockers:
        if bk.get("id") == furn.get("id"):
            continue
        depth = convex_overlap_depth(env, bk["poly"])
        if depth:
            c = poly_center(bk["poly"])
            issues.append({
                "type": OBSTACLE,
                "objectId": bk.get("id"),
                "objectName": bk.get("name", "障碍物"),
                "point": {"x": c[0], "y": c[1]},
                "depth": depth,
                "remaining": depth,
                "turn": turn,
                "message": "%s扫到「%s」，重叠 %.0f mm"
                           % ("转向" if turn > 8 else "移动",
                              bk.get("name", "障碍物"), depth),
            })

    # 3) 门扇：扫掠包络与门洞重叠即视为「门已固定全开」，只被全开扇叶阻挡；
    #    否则扫到开启扇形则阻挡（不能在扇区内停放）
    for door in plan.doors:
        ga0, gb0, _ = door_gate_segment(door)
        env_hits_gate = any(segments_intersect(ga0, gb0, env[i], env[(i + 1) % len(env)])
                            for i in range(len(env)))
        hinge = add((door["x"], door["y"]),
                    scale(unit((door.get("wx", 1.0), door.get("wy", 0.0))),
                          door["width"] / 2.0))
        if env_hits_gate:
            leaf, _, _ = door_panel_poly(door)
            depth = convex_overlap_depth(env, leaf)
            if depth and depth > 20.0:
                issues.append({
                    "type": SWING,
                    "doorId": door.get("id"),
                    "point": {"x": hinge[0], "y": hinge[1]},
                    "depth": depth,
                    "remaining": depth,
                    "message": "过门时撞到全开固定的门扇（重叠 %.0f mm）" % depth,
                })
        else:
            blocked, _h = _blocked_by_door_swing(plan, door, env, False)
            if blocked:
                issues.append({
                    "type": SWING,
                    "doorId": door.get("id"),
                    "point": {"x": hinge[0], "y": hinge[1]},
                    "depth": 0,
                    "remaining": 0,
                    "message": "搬运路径被门扇开启范围阻挡（需绕开或先推门贴墙）",
                })

    # 4) 通道净宽：沿移动轴线，在两端姿态中心处向两侧探测
    move = sub((p1["x"], p1["y"]), (p0["x"], p0["y"]))
    if move[0] ** 2 + move[1] ** 2 > EPS:
        clear0 = lateral_clearance(plan, blockers, (p0["x"], p0["y"]),
                                   (p1["x"], p1["y"]), pa)
        clear1 = lateral_clearance(plan, blockers, (p1["x"], p1["y"]),
                                   (p0["x"], p0["y"]), pb)
        clear = min(clear0, clear1)
        # 通道实际剩余 = 净宽 - 家具垂直于运动方向的占用
        # clear 是到障碍的双向侧隙之和方向的最小净空，直接返回侧隙
        issues.append({
            "_metric": "clearance",
            "sideGap": clear,
        })

    # 5) 门楣：移动路径穿过任一门洞且抬起超高
    h0, h1 = tilted_height(furn, p0), tilted_height(furn, p1)
    eff_h = max(h0, h1)
    for door in plan.doors:
        ga, gb, _ = door_gate_segment(door)
        # 路径中线是否跨过门线
        if segments_intersect((p0["x"], p0["y"]), (p1["x"], p1["y"]), ga, gb):
            clearance = door.get("clearHeight")
            if clearance and eff_h > clearance:
                issues.append({
                    "type": HEIGHT,
                    "doorId": door.get("id"),
                    "point": {"x": door["x"], "y": door["y"]},
                    "height": eff_h,
                    "limit": clearance,
                    "remaining": clearance - eff_h,
                    "message": "过门时抬起高度 %.0f > 门楣 %.0f，差 %.0f mm"
                               % (eff_h, clearance, eff_h - clearance),
                })
            # 门洞宽度余量
            gate_used = gate_projection(door, env)
            gap = door["width"] - gate_used
            if gap < 0:
                issues.append({
                    "type": WALL,
                    "subtype": "gate_width",
                    "point": {"x": door["x"], "y": door["y"]},
                    "depth": -gap,
                    "remaining": 0,
                    "message": "门洞净宽 %d，家具扫掠需 %.0f，差 %.0f mm"
                               % (door["width"], gate_used, -gap),
                })

    return issues, env


def gate_projection(door, poly):
    """家具多边形沿门洞方向的投影长度。"""
    wu = unit((door.get("wx", 1.0), door.get("wy", 0.0)))
    vals = [v[0] * wu[0] + v[1] * wu[1] for v in poly]
    return max(vals) - min(vals)


def lateral_clearance(plan, blockers, origin, toward, poly):
    """在 origin 处，沿 toward 方向的法向左右探测到墙/障碍的最小侧隙（mm）。"""
    move = sub(toward, origin)
    nrm = normal(move)
    gaps = []
    walls = plan.wall_segments()

    def ray_gap(sign):
        ray = scale(nrm, sign)
        best = 1e9
        for a, b, _sp in walls:
            t = ray_hits_segment(origin, ray, a, b, 5000.0)
            if t is not None:
                best = min(best, t)
        for bk in blockers:
            bp = bk["poly"]
            n = len(bp)
            for i in range(n):
                t = ray_hits_segment(origin, ray, bp[i], bp[(i + 1) % n], 5000.0)
                if t is not None:
                    best = min(best, t)
        return best

    gaps.append(ray_gap(1))
    gaps.append(ray_gap(-1))
    return min(gaps)


# ---------------------------------------------------------------- 整条路线

def analyze_route(plan, furn, poses, static_blockers=None):
    """播放路线：逐姿态、逐段检查，停在首个冲突。

    返回 {ok, conflictIndex, conflict, reports, envelopes}
    conflictIndex: 冲突所在姿态/段编号（k = 姿态 k 或 段 k-1~k）
    """
    blockers = static_blockers if static_blockers is not None else plan.blocker_polys()
    reports = []
    envelopes = []
    min_side_gap = None

    for k, pose in enumerate(poses):
        issues = check_pose(plan, furn, pose, blockers=blockers)
        metric = [i for i in issues if i.get("type")]
        if metric:
            return {
                "ok": False,
                "conflictIndex": k,
                "conflictKind": "pose",
                "conflict": metric[0],
                "allConflicts": metric,
                "reports": reports,
                "envelopes": envelopes,
                "minSideGap": min_side_gap,
            }
        reports.append({"index": k, "ok": True})
        if k > 0:
            seg_issues, env = check_segment_between_poses(
                plan, furn, poses[k - 1], pose, blockers=blockers)
            gap_metrics = [i for i in seg_issues if i.get("_metric") == "clearance"]
            hard = [i for i in seg_issues if not i.get("_metric")]
            env_pts = [{"x": x, "y": y} for x, y in env]
            envelopes.append(env_pts)
            for gm in gap_metrics:
                min_side_gap = gm["sideGap"] if min_side_gap is None else min(min_side_gap, gm["sideGap"])
            if hard:
                return {
                    "ok": False,
                    "conflictIndex": k,
                    "conflictKind": "segment",
                    "conflict": hard[0],
                    "allConflicts": hard,
                    "reports": reports,
                    "envelopes": envelopes,
                    "minSideGap": min_side_gap,
                }
            reports[-1]["segmentOk"] = True
            reports[-1]["sideGap"] = gap_metrics[0]["sideGap"] if gap_metrics else None

    return {
        "ok": True,
        "conflictIndex": None,
        "conflict": None,
        "allConflicts": [],
        "reports": reports,
        "envelopes": envelopes,
        "minSideGap": min_side_gap,
    }
