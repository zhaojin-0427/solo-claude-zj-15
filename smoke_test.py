"""手工冒烟测试：走廊+房间+门，一张桌子搬运路线。"""
from planner import Plan, analyze_route, check_pose
from optimizer import optimize_layout

# 走廊 2000x1600（0,0)-(2000,1600)，房间 3000x3000（2000,0)-(5000,3000)
plan_data = {
    "spaces": [
        {"id": "hall", "name": "走廊",
         "poly": [[0, 0], [2000, 0], [2000, 1600], [0, 1600]]},
        {"id": "room", "name": "房间",
         "poly": [[2000, 0], [5000, 0], [5000, 3000], [2000, 3000]]},
    ],
    "doors": [
        # 走廊进入房间的门，开在 x=2000 的隔墙上，门宽 850，门楣 2000
        {"id": "d1", "x": 2000, "y": 450, "wx": 0, "wy": 1, "width": 850,
         "openAngle": 90, "swing": 1, "clearHeight": 2000, "name": "房门"},
    ],
    "obstacles": [
        {"id": "ob1", "name": "固定柜", "x": 4200, "y": 2400, "w": 600, "d": 600, "deg": 0},
    ],
}

table = {"id": "t1", "name": "小书柜", "w": 700, "d": 400, "h": 1200}

# 门朝走廊开（swing=1），门扇扇形占走廊上方；从 y=300 一线一步推进门洞
# （推进过程的扫掠包络跨过门洞时视为门已被推贴墙，不要求在扇形内停放）
poses_ok = [
    {"x": 500, "y": 300, "deg": 0, "pitch": 0},
    {"x": 2000, "y": 300, "deg": 0, "pitch": 0},
    {"x": 2700, "y": 300, "deg": 0, "pitch": 0},
    {"x": 3200, "y": 1700, "deg": 90, "pitch": 0},
]

# 大书桌（1200 长）横过 850 门 -> 门洞净宽不足，应停在过门段
big_table = {"id": "big", "name": "大书桌", "w": 1200, "d": 600, "h": 750}
poses_gate = [
    {"x": 500, "y": 300, "deg": 0, "pitch": 0},
    {"x": 2000, "y": 300, "deg": 0, "pitch": 0},
    {"x": 3000, "y": 300, "deg": 0, "pitch": 0},
]

plan = Plan(plan_data)
r = analyze_route(plan, table, poses_ok)
print("正常路线:", "OK" if r["ok"] else "FAIL",
      "" if r["ok"] else r["conflict"]["message"], "最小侧隙:", r["minSideGap"])

# 大书桌门洞净宽不足
rg = analyze_route(plan, big_table, poses_gate)
print("大门洞路线:", "OK" if rg["ok"] else
      "停在第%d点 [%s] %s" % (rg["conflictIndex"], rg["conflict"]["type"],
                              rg["conflict"]["message"]))

# 故意撞固定柜
poses_bad = [
    {"x": 600, "y": 500, "deg": 0, "pitch": 0},
    {"x": 4200, "y": 2400, "deg": 0, "pitch": 0},
]
r2 = analyze_route(plan, table, poses_bad)
print("撞柜路线:", "OK" if r2["ok"] else "停在第%d姿态 (%s)" % (
    r2["conflictIndex"], r2["conflict"]["type"]), r2["conflict"]["message"])

# 抬高超门楣：pitch 60 的 750 高 1200 长桌 -> 750*cos60 + 1200*sin60 = 1414 < 2000 不超
# 改为高 1900 的大衣柜
wardrobe = {"id": "w1", "name": "衣柜", "w": 1000, "d": 600, "h": 2100}
poses_h = [
    {"x": 600, "y": 500, "deg": 0, "pitch": 0},
    {"x": 2000, "y": 450, "deg": 0, "pitch": 30},
]
r3 = analyze_route(plan, wardrobe, poses_h)
# 2100*cos30 + 1000*sin30 = 1818+500=2318 > 2000
print("门楣检查:", [c["message"] for c in r3.get("allConflicts", [])] or "通过")

# 门扇扫掠：在朝走廊开的扇形区内放物（数值搜索得到的扇内点）
p_swing = {"x": 1200, "y": 600, "deg": 0, "pitch": 0}
iss = check_pose(plan, {"id": "x", "w": 300, "d": 300, "h": 500}, p_swing)
print("门扇区放置:", [i["message"] for i in iss] or "通过")

# 优化器：锁定床，摆放书桌、椅子
furniture = [
    {"id": "bed", "name": "床(锁定)", "w": 1500, "d": 2000, "h": 500,
     "locked": True, "x": 4200, "y": 1200, "deg": 0},
    {"id": "desk", "name": "书桌", "w": 1200, "d": 600, "h": 750},
    {"id": "chair", "name": "椅子", "w": 450, "d": 450, "h": 900},
]
opt = optimize_layout(plan_data, furniture, entry_door_id="d1", samples=15)
print("优化方案数:", len(opt["ranked"]))
for i, c in enumerate(opt["ranked"][:3]):
    print("  #%d 可达%d 最小余量%s 转弯%d 紧凑%.2f placements=%d" % (
        i, len(c["reachable"]), c["minClearance"], c["turns"],
        c["compactness"], len(c["placements"])))
