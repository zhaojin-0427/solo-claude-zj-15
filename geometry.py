"""纯 Python 2D 计算几何（单位：毫米），不依赖任何第三方库。

约定：世界坐标 x 向右、y 向下；角度单位为度，顺时针为正。
所有多边形按顶点序列表示（不强制绕向），矩形走局部坐标
(-w/2,-d/2) -> (w/2,-d/2) -> (w/2,d/2) -> (-w/2,d/2)。
"""
import math

EPS = 1e-6


# ---------------------------------------------------------------- 基础构造

def rect_poly(cx, cy, w, d, deg=0.0):
    """中心 (cx,cy)、宽 w、深 d、旋转 deg 度的矩形四角。"""
    a = math.radians(deg)
    ca, sa = math.cos(a), math.sin(a)
    hx, hy = w / 2.0, d / 2.0
    pts = []
    for lx, ly in ((-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy)):
        pts.append((cx + lx * ca - ly * sa, cy + lx * sa + ly * ca))
    return pts


def poly_area(poly):
    s = 0.0
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        s += x1 * y2 - x2 * y1
    return abs(s) * 0.5


def transform_point(p, cx, cy, deg):
    a = math.radians(deg)
    ca, sa = math.cos(a), math.sin(a)
    x, y = p
    return (cx + x * ca - y * sa, cy + x * sa + y * ca)


# ---------------------------------------------------------------- 点 / 线段

def sub(a, b):
    return (a[0] - b[0], a[1] - b[1])


def add(a, b):
    return (a[0] + b[0], a[1] + b[1])


def scale(a, k):
    return (a[0] * k, a[1] * k)


def dot(a, b):
    return a[0] * b[0] + a[1] * b[1]


def cross(a, b):
    return a[0] * b[1] - a[1] * b[0]


def dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def point_seg_dist(p, a, b):
    """点 p 到线段 ab 的距离与最近点。"""
    ab = sub(b, a)
    L2 = dot(ab, ab)
    if L2 < EPS:
        return dist(p, a), a
    t = max(0.0, min(1.0, dot(sub(p, a), ab) / L2))
    q = add(a, scale(ab, t))
    return dist(p, q), q


def _orientation(a, b, c):
    v = cross(sub(b, a), sub(c, a))
    if abs(v) < 1e-7:
        return 0
    return 1 if v > 0 else -1


def on_segment(p, a, b, tol=1e-6):
    if _orientation(a, b, p) != 0:
        return False
    return (min(a[0], b[0]) - tol <= p[0] <= max(a[0], b[0]) + tol and
            min(a[1], b[1]) - tol <= p[1] <= max(a[1], b[1]) + tol)


def segments_intersect(a, b, c, d):
    """线段 ab 与 cd 是否相交（含端点接触）。"""
    o1, o2 = _orientation(a, b, c), _orientation(a, b, d)
    o3, o4 = _orientation(c, d, a), _orientation(c, d, b)
    if o1 == 0 and on_segment(c, a, b):
        return True
    if o2 == 0 and on_segment(d, a, b):
        return True
    if o3 == 0 and on_segment(a, c, d):
        return True
    if o4 == 0 and on_segment(b, c, d):
        return True
    return o1 != o2 and o3 != o4


def ray_hits_segment(orig, ray, a, b, max_t=1e12):
    """射线 orig+t*ray（t>=0）与线段 ab，返回正参数 t，不相交返回 None。"""
    v = sub(b, a)
    denom = cross(ray, v)
    if abs(denom) < EPS:
        return None
    dif = sub(a, orig)
    t = cross(dif, v) / denom
    u = cross(dif, ray) / denom
    if t >= -1e-9 and -1e-9 <= u <= 1 + 1e-9 and t <= max_t:
        return max(0.0, t)
    return None


def nearest_point_on_poly(p, poly):
    """点到多边形边界（边的集合）的最近点与距离。"""
    best_d, best_q = float("inf"), None
    n = len(poly)
    for i in range(n):
        d, q = point_seg_dist(p, poly[i], poly[(i + 1) % n])
        if d < best_d:
            best_d, best_q = d, q
    return best_d, best_q


# ---------------------------------------------------------------- 多边形关系

def point_in_poly(p, poly):
    """射线法，边界算内部。"""
    x, y = p
    n = len(poly)
    inside = False
    for i in range(n):
        ax, ay = poly[i]
        bx, by = poly[(i + 1) % n]
        if on_segment(p, (ax, ay), (bx, by)):
            return True
        if (ay > y) != (by > y):
            xint = ax + (bx - ax) * (y - ay) / (by - ay)
            if x < xint:
                inside = not inside
    return inside


def poly_contains_poly(outer, inner):
    """outer 为凸多边形时：inner 所有顶点在 outer 内即包含。"""
    return all(point_in_poly(v, outer) for v in inner)


def _convex_axes(poly):
    axes = []
    n = len(poly)
    for i in range(n):
        p1, p2 = poly[i], poly[(i + 1) % n]
        e = sub(p2, p1)
        L = math.hypot(*e)
        if L > EPS:
            axes.append((-e[1] / L, e[0] / L))
    return axes


def _project(poly, axis):
    vals = [dot(v, axis) for v in poly]
    return min(vals), max(vals)


def convex_overlap_depth(a, b):
    """SAT 凸多边形相交：不相交返回 None；相交返回最小穿透深度（mm）。"""
    min_depth = float("inf")
    for axis in _convex_axes(a) + _convex_axes(b):
        a0, a1 = _project(a, axis)
        b0, b1 = _project(b, axis)
        overlap = min(a1, b1) - max(a0, b0)
        if overlap <= 0:
            return None
        min_depth = min(min_depth, overlap)
    return min_depth


def polys_overlap(a, b):
    return convex_overlap_depth(a, b) is not None


def seg_poly_intersect(a, b, poly):
    """线段是否穿过多边形（端点在内或与边相交）。"""
    if point_in_poly(a, poly) or point_in_poly(b, poly):
        return True
    n = len(poly)
    for i in range(n):
        if segments_intersect(a, b, poly[i], poly[(i + 1) % n]):
            return True
    return False


def segment_overlap_with_poly(seg, poly):
    """线段在多边形内部的参数区间并集 [(t0,t1),...]。"""
    a, b = seg
    n = len(poly)
    ts = [0.0, 1.0]
    for i in range(n):
        e1, e2 = poly[i], poly[(i + 1) % n]
        v = sub(b, a)
        denom = cross(v, sub(e2, e1))
        if abs(denom) < EPS:
            continue
        dif = sub(e1, a)
        t = cross(dif, sub(e2, e1)) / denom
        u = cross(dif, v) / denom
        if -1e-9 <= u <= 1 + 1e-9 and -1e-9 <= t <= 1 + 1e-9:
            ts.append(min(1.0, max(0.0, t)))
    ts.sort()
    intervals = []
    for i in range(len(ts) - 1):
        tm = (ts[i] + ts[i + 1]) / 2.0
        mid = add(a, scale(sub(b, a), tm))
        if point_in_poly(mid, poly):
            intervals.append((ts[i], ts[i + 1]))
    return intervals


def poly_center(poly):
    x = sum(p[0] for p in poly) / len(poly)
    y = sum(p[1] for p in poly) / len(poly)
    return (x, y)


def project_poly(poly, axis):
    return _project(poly, axis)


# ---------------------------------------------------------------- 凸包 / 包络

def convex_hull(points):
    pts = sorted(set((round(x, 3), round(y, 3)) for x, y in points))
    if len(pts) <= 1:
        return pts

    def cross3(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower = []
    for p in pts:
        while len(lower) >= 2 and cross3(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    upper = []
    for p in reversed(pts):
        while len(upper) >= 2 and cross3(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def swept_envelope(poly_a, poly_b):
    """两个姿态间的移动扫掠包络：两多边形顶点的凸包（含平移+旋转扫掠的保守包络）。"""
    return convex_hull(list(poly_a) + list(poly_b))


# ---------------------------------------------------------------- 向量工具

def unit(v):
    L = math.hypot(*v)
    return (v[0] / L, v[1] / L) if L > EPS else (0.0, 0.0)


def normal(v):
    u = unit(v)
    return (-u[1], u[0])


def heading(a, b):
    return unit(sub(b, a))


def clamp(v, lo, hi):
    return max(lo, min(hi, v))
