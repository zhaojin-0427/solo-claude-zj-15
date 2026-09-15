"""家具入户推演台 —— 本机 HTTP 服务（Python 标准库实现，无需 Flask）。

启动：python3 app.py  →  http://127.0.0.1:8000
API：
  GET  /api/schemes              方案列表
  POST /api/schemes              新建/保存方案
  GET  /api/schemes/<id>         读取方案
  DELETE /api/schemes/<id>       删除方案
  POST /api/check/route          计算搬运路线（停在首个冲突）
  POST /api/check/pose           检查单个姿态（拖动时）
  POST /api/optimize             布局优化排序
"""
import json
import os
import sqlite3
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

from planner import (Plan, analyze_route, check_pose, furniture_poly,
                     tilted_height, usage_polys, usage_blockers)
from optimizer import optimize_layout

BASE = os.path.dirname(os.path.abspath(__file__))
DB_PATH = os.path.join(BASE, "data", "schemes.db")
STATIC = os.path.join(BASE, "static")

CONTENT_TYPES = {
    ".html": "text/html; charset=utf-8",
    ".css": "text/css; charset=utf-8",
    ".js": "application/javascript; charset=utf-8",
    ".svg": "image/svg+xml",
    ".json": "application/json; charset=utf-8",
    ".png": "image/png",
    ".ico": "image/x-icon",
}


# ---------------------------------------------------------------- 数据库

def db():
    os.makedirs(os.path.dirname(DB_PATH), exist_ok=True)
    conn = sqlite3.connect(DB_PATH)
    conn.row_factory = sqlite3.Row
    conn.execute("""
        CREATE TABLE IF NOT EXISTS schemes (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            name TEXT NOT NULL,
            data TEXT NOT NULL,
            updated_at REAL NOT NULL
        )
    """)
    return conn


def init_db():
    db().close()


# ---------------------------------------------------------------- 业务接口

def api_save_scheme(body):
    name = (body.get("name") or "未命名方案").strip()[:60]
    payload = body.get("data", body)
    sid = body.get("id")
    now = time.time()
    with db() as conn:
        if sid:
            conn.execute("UPDATE schemes SET name=?, data=?, updated_at=? WHERE id=?",
                         (name, json.dumps(payload, ensure_ascii=False), now, sid))
        else:
            cur = conn.execute("INSERT INTO schemes(name, data, updated_at) VALUES(?,?,?)",
                               (name, json.dumps(payload, ensure_ascii=False), now))
            sid = cur.lastrowid
    return {"ok": True, "id": sid, "name": name}


def api_list_schemes():
    with db() as conn:
        rows = conn.execute(
            "SELECT id,name,updated_at FROM schemes ORDER BY updated_at DESC").fetchall()
    return [{"id": r["id"], "name": r["name"], "updatedAt": r["updated_at"]} for r in rows]


def api_get_scheme(sid):
    with db() as conn:
        row = conn.execute("SELECT * FROM schemes WHERE id=?", (sid,)).fetchone()
    if not row:
        return None
    return {"id": row["id"], "name": row["name"],
            "data": json.loads(row["data"]), "updatedAt": row["updated_at"]}


def api_delete_scheme(sid):
    with db() as conn:
        conn.execute("DELETE FROM schemes WHERE id=?", (sid,))
    return {"ok": True}


def static_blockers_from_plan(plan_data, moving_furn_id=None):
    """把方案中已放置的其他家具（非当前搬运对象）及其使用包络作为固定阻挡。"""
    plan = Plan(plan_data)
    others = [f for f in plan_data.get("furniture", [])
              if f.get("id") != moving_furn_id]
    extra = []
    for f in others:
        pos = f.get("placement")
        if pos:
            extra.append({
                "poly": furniture_poly(f, pos),
                "id": f["id"], "name": f.get("name", f["id"]),
                "height": f.get("h", 2000), "kind": "furniture",
            })
    extra.extend(usage_blockers(others))
    return plan, extra


def api_usage(body):
    """返回若干家具在其摆放位的使用包络多边形，供前端叠加显示。"""
    out = {}
    for f in body.get("furniture", []):
        pos = f.get("placement")
        if pos and f.get("usage"):
            out[f["id"]] = [
                [{"x": round(x, 1), "y": round(y, 1)} for x, y in poly]
                for poly in usage_polys(f, pos["x"], pos["y"], pos.get("deg", 0))
            ]
    return {"polys": out}


def api_check_route(body):
    plan_data = body["plan"]
    furn = body["furniture"]
    poses = body["poses"]
    plan, extra = static_blockers_from_plan(plan_data, furn.get("id"))
    result = analyze_route(plan, furn, poses, static_blockers=plan.blocker_polys(extra=extra))
    # 补充抬升高度信息，便于前端标注
    result["heights"] = [round(tilted_height(furn, p), 1) for p in poses]
    return result


def api_check_pose(body):
    plan_data = body["plan"]
    furn = body["furniture"]
    pose = body["pose"]
    plan, extra = static_blockers_from_plan(plan_data, furn.get("id"))
    issues = check_pose(plan, furn, pose, blockers=plan.blocker_polys(extra=extra))
    return {"ok": not issues, "issues": issues,
            "effectiveHeight": round(tilted_height(furn, pose), 1)}


def api_optimize(body):
    plan_data = body["plan"]
    furniture = body["furniture"]
    entry = body.get("entryDoorId")
    samples = int(body.get("samples", 40))
    return optimize_layout(plan_data, furniture, entry_door_id=entry, samples=samples)


# ---------------------------------------------------------------- HTTP

class Handler(BaseHTTPRequestHandler):
    server_version = "FurnitureSim/1.0"

    def log_message(self, fmt, *args):
        pass

    def _send_json(self, obj, status=200):
        raw = json.dumps(obj, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(raw)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(raw)

    def _read_json(self):
        n = int(self.headers.get("Content-Length", 0))
        if not n:
            return {}
        raw = self.rfile.read(n)
        return json.loads(raw.decode("utf-8"))

    def do_GET(self):
        path = urlparse(self.path).path
        if path == "/api/schemes":
            return self._send_json(api_list_schemes())
        if path.startswith("/api/schemes/"):
            sid = path.rsplit("/", 1)[-1]
            try:
                sid = int(sid)
            except ValueError:
                return self._send_json({"error": "bad id"}, 400)
            row = api_get_scheme(sid)
            return self._send_json(row if row else {"error": "not found"},
                                   200 if row else 404)
        self._serve_static(path)

    def do_POST(self):
        path = urlparse(self.path).path
        try:
            body = self._read_json()
        except Exception as exc:
            return self._send_json({"error": "bad json: %s" % exc}, 400)
        try:
            if path == "/api/schemes":
                return self._send_json(api_save_scheme(body))
            if path == "/api/check/route":
                return self._send_json(api_check_route(body))
            if path == "/api/check/pose":
                return self._send_json(api_check_pose(body))
            if path == "/api/optimize":
                return self._send_json(api_optimize(body))
            if path == "/api/usage":
                return self._send_json(api_usage(body))
        except KeyError as exc:
            return self._send_json({"error": "缺少字段 %s" % exc}, 400)
        except Exception as exc:
            return self._send_json({"error": str(exc)}, 500)
        self._send_json({"error": "not found"}, 404)

    def do_DELETE(self):
        path = urlparse(self.path).path
        if path.startswith("/api/schemes/"):
            sid = path.rsplit("/", 1)[-1]
            try:
                return self._send_json(api_delete_scheme(int(sid)))
            except ValueError:
                return self._send_json({"error": "bad id"}, 400)
        self._send_json({"error": "not found"}, 404)

    def _serve_static(self, path):
        if path == "/":
            path = "/index.html"
        fp = os.path.normpath(os.path.join(STATIC, path.lstrip("/")))
        if not fp.startswith(STATIC) or not os.path.isfile(fp):
            self.send_response(404)
            self.send_header("Content-Type", "text/plain; charset=utf-8")
            self.end_headers()
            self.wfile.write("404".encode())
            return
        ext = os.path.splitext(fp)[1]
        with open(fp, "rb") as fh:
            raw = fh.read()
        self.send_response(200)
        self.send_header("Content-Type", CONTENT_TYPES.get(ext, "application/octet-stream"))
        self.send_header("Content-Length", str(len(raw)))
        self.end_headers()
        self.wfile.write(raw)


def main():
    init_db()
    port = int(os.environ.get("PORT", "8000"))
    httpd = ThreadingHTTPServer(("127.0.0.1", port), Handler)
    print("家具入户推演台  →  http://127.0.0.1:%d" % port)
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
