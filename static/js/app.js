/* 家具入户推演台 —— 前端主逻辑（原生 JS + SVG） */
(function () {
  const $ = id => document.getElementById(id);

  // ---------------- 状态 ----------------
  const state = {
    mode: "plan",
    spaces: [], doors: [], obstacles: [], furniture: [],
    selected: null,
    activeFurn: null,
    poses: [],
    routeResult: null,
    layoutResult: null,
    layoutSelected: 0,
    showGrid: true,
    schemeId: null,
    seq: 1,
    pendingPlace: null,     // 待放置家具 id（点击画布落位）
    playing: false,
  };

  const layer = {
    space: $("layerSpace"), ruler: $("layerRuler"), door: $("layerDoor"),
    obstacle: $("layerObstacle"),
    furniture: $("layerFurniture"), usage: $("layerUsage"), route: $("layerRoute"),
    layout: $("layerLayout"), overlay: $("layerOverlay"),
  };

  // ---------------- 数学辅助 ----------------
  const uid = p => p + "_" + (state.seq++) + "_" + Math.random().toString(36).slice(2, 6);
  const clamp = (v, a, b) => Math.max(a, Math.min(b, v));
  const round1 = v => Math.round(v * 10) / 10;
  function debounce(fn, ms) { let t; return (...a) => { clearTimeout(t); t = setTimeout(() => fn(...a), ms); }; }
  function unit(v) { const L = Math.hypot(v[0], v[1]) || 1; return [v[0] / L, v[1] / L]; }
  function ptsStr(pts) { return pts.map(p => `${round1(p[0])},${round1(p[1])}`).join(" "); }
  function flashHint(msg) {
    const h = $("modeHint"); h.textContent = msg; h.classList.add("show");
    clearTimeout(flashHint._t); flashHint._t = setTimeout(() => h.classList.remove("show"), 1500);
  }
  async function api(path, body) {
    const res = await fetch(path, {
      method: "POST", headers: { "Content-Type": "application/json" },
      body: JSON.stringify(body || {}),
    });
    return res.json();
  }

  function rectPoints(x, y, w, d, deg) {
    const a = (deg || 0) * Math.PI / 180, ca = Math.cos(a), sa = Math.sin(a);
    return [[-w / 2, -d / 2], [w / 2, -d / 2], [w / 2, d / 2], [-w / 2, d / 2]]
      .map(([lx, ly]) => [round1(x + lx * ca - ly * sa), round1(y + lx * sa + ly * ca)]);
  }
  function polyCenter(poly) {
    return [poly.reduce((s, p) => s + p[0], 0) / poly.length,
            poly.reduce((s, p) => s + p[1], 0) / poly.length];
  }

  // JS 版墙段派生（与 planner.py 同构）
  function pointNearEdge(p, a, b, tol) {
    const ab = [b[0] - a[0], b[1] - a[1]], L2 = ab[0] ** 2 + ab[1] ** 2;
    let t = ((p[0] - a[0]) * ab[0] + (p[1] - a[1]) * ab[1]) / L2;
    t = clamp(t, 0, 1);
    const q = [a[0] + ab[0] * t, a[1] + ab[1] * t];
    return Math.hypot(p[0] - q[0], p[1] - q[1]) <= tol;
  }
  function wallSegmentsJS(margin = 0) {
    const walls = [];
    state.spaces.forEach(sp => {
      const poly = sp.poly, n = poly.length;
      for (let i = 0; i < n; i++) {
        const e1 = poly[i], e2 = poly[(i + 1) % n];
        const ev = [e2[0] - e1[0], e2[1] - e1[1]];
        const L = Math.hypot(...ev), eu = [ev[0] / L, ev[1] / L];
        const cuts = [];
        state.doors.forEach(dr => {
          if (pointNearEdge([dr.x, dr.y], e1, e2, 70)) {
            const m = [dr.x - e1[0], dr.y - e1[1]];
            const c = m[0] * eu[0] + m[1] * eu[1];
            const half = dr.width / 2 + margin;
            cuts.push([Math.max(0, c - half), Math.min(L, c + half)]);
          }
        });
        cuts.sort((a, b) => a[0] - b[0]);
        const merged = [];
        cuts.forEach(c => {
          if (merged.length && c[0] <= merged[merged.length - 1][1])
            merged[merged.length - 1][1] = Math.max(merged[merged.length - 1][1], c[1]);
          else merged.push(c);
        });
        let cur = 0;
        const at = t => [round1(e1[0] + eu[0] * t), round1(e1[1] + eu[1] * t)];
        merged.forEach(([s, t]) => {
          if (s - cur > 1) walls.push([at(cur), at(s)]);
          cur = Math.max(cur, t);
        });
        if (L - cur > 1) walls.push([at(cur), at(L)]);
        if (!merged.length) walls.push([e1.slice(), e2.slice()]);
      }
    });
    return walls;
  }

  // 门几何（与后端同构：门轴在 +wu 端）
  function doorSwingPoints(dr, steps = 20) {
    const wu = unit([dr.wx ?? 1, dr.wy ?? 0]), n = [-wu[1], wu[0]];
    const side = dr.swing ?? 1;
    const hinge = [dr.x + wu[0] * dr.width / 2, dr.y + wu[1] * dr.width / 2];
    const tip = rad => [
      hinge[0] + (-wu[0] * Math.cos(rad) + side * n[0] * Math.sin(rad)) * dr.width,
      hinge[1] + (-wu[1] * Math.cos(rad) + side * n[1] * Math.sin(rad)) * dr.width];
    const pts = [hinge];
    const ang = Math.abs(dr.openAngle ?? 90);
    for (let k = 0; k <= steps; k++) pts.push(tip(ang * k / steps * Math.PI / 180));
    return pts;
  }
  function doorLeafPoints(dr) {
    const wu = unit([dr.wx ?? 1, dr.wy ?? 0]), n = [-wu[1], wu[0]], side = dr.swing ?? 1;
    const hinge = [dr.x + wu[0] * dr.width / 2, dr.y + wu[1] * dr.width / 2];
    const a = (dr.openAngle ?? 90) * Math.PI / 180;
    return [hinge, [
      hinge[0] + (-wu[0] * Math.cos(a) + side * n[0] * Math.sin(a)) * dr.width,
      hinge[1] + (-wu[1] * Math.cos(a) + side * n[1] * Math.sin(a)) * dr.width]];
  }

  // 使用包络（与 planner 同构）
  function usagePolysJS(f, pos) {
    const w = f.w, d = f.d, deg = pos.deg || 0;
    const a = deg * Math.PI / 180, ca = Math.cos(a), sa = Math.sin(a);
    const R = (lx, ly) => [round1(pos.x + lx * ca - ly * sa), round1(pos.y + lx * sa + ly * ca)];
    const edges = {
      front: { n: [0, -1], anc: [0, -d / 2], len: w },
      back: { n: [0, 1], anc: [0, d / 2], len: w },
      left: { n: [-1, 0], anc: [-w / 2, 0], len: d },
      right: { n: [1, 0], anc: [w / 2, 0], len: d },
    };
    function hull(pts) {
      pts = [...new Set(pts.map(p => p.map(v => Math.round(v) / 1).join(",")))]
        .map(s => s.split(",").map(Number));
      pts.sort((p, q) => p[0] - q[0] || p[1] - q[1]);
      const cr = (o, x, y) => (x[0] - o[0]) * (y[1] - o[1]) - (x[1] - o[1]) * (y[0] - o[0]);
      const lo = []; pts.forEach(p => { while (lo.length >= 2 && cr(lo[lo.length - 2], lo[lo.length - 1], p) <= 0) lo.pop(); lo.push(p); });
      const up = []; pts.slice().reverse().forEach(p => { while (up.length >= 2 && cr(up[up.length - 2], up[up.length - 1], p) <= 0) up.pop(); up.push(p); });
      return lo.slice(0, -1).concat(up.slice(0, -1));
    }
    const out = [];
    (f.usage?.doors || []).forEach(dr => {
      const e = edges[dr.edge || "front"], [nx, ny] = e.n, [tx, ty] = [ny, -nx];
      const hx = e.anc[0] - tx * e.len / 2, hy = e.anc[1] - ty * e.len / 2;
      const ang = Math.min(179, dr.open || 90) * Math.PI / 180;
      const closed = [hx + tx * e.len, hy + ty * e.len];
      const tip = [hx + (tx * Math.cos(ang) - nx * Math.sin(ang)) * e.len,
                   hy + (ty * Math.cos(ang) - ny * Math.sin(ang)) * e.len];
      const reach = dr.reach || 600;
      out.push(hull([[hx, hy], closed, tip,
        [tip[0] + nx * reach * .25, tip[1] + ny * reach * .25],
        [closed[0] + nx * reach * .25, closed[1] + ny * reach * .25]]).map(p => R(...p)));
    });
    (f.usage?.drawers || []).forEach(dr => {
      const e = edges[dr.edge || "front"], [nx, ny] = e.n, reach = dr.reach || 700;
      const [ax, ay] = e.anc, L = e.len;
      out.push([[ax - L / 2, ay], [ax + L / 2, ay],
        [ax + L / 2 + nx * reach, ay + ny * reach],
        [ax - L / 2 + nx * reach, ay + ny * reach]].map(p => R(...p)));
    });
    if (f.usage?.chair) {
      const tot = (f.usage.chair.reach ?? 300) + (f.usage.chair.seat ?? 500);
      out.push([[-w / 2, -d / 2], [w / 2, -d / 2],
        [w / 2, -d / 2 - tot], [-w / 2, -d / 2 - tot]].map(p => R(...p)));
    }
    return out;
  }

  function planData() {
    return {
      spaces: state.spaces, doors: state.doors, obstacles: state.obstacles,
      furniture: state.furniture.map(({ id, name, w, d, h, locked, placement, usage }) =>
        ({ id, name, w, d, h, locked, placement, usage: usage || undefined })),
    };
  }
  function activeFurniture() { return state.furniture.find(f => f.id === state.activeFurn); }
  function bounds() {
    const all = [...state.spaces.flatMap(s => s.poly), ...state.obstacles.map(o => [o.x, o.y])];
    if (!all.length) return null;
    const xs = all.map(p => p[0]), ys = all.map(p => p[1]);
    return { minx: Math.min(...xs) - 500, miny: Math.min(...ys) - 500,
             maxx: Math.max(...xs) + 500, maxy: Math.max(...ys) + 500 };
  }

  // ---------------- SVG 工具 ----------------
  function el(tag, attrs, parent) {
    const node = document.createElementNS("http://www.w3.org/2000/svg", tag);
    // HTML 元素（li/button/option/input/span/div）用 HTML 命名空间
    const htmlTags = ["LI", "BUTTON", "OPTION", "INPUT", "SPAN", "DIV", "LABEL",
      "SELECT", "B", "P", "UL", "H3", "TEXTAREA"];
    const dom = htmlTags.includes(tag.toUpperCase())
      ? document.createElement(tag) : node;
    if (attrs) for (const k in attrs) {
      if (k === "text") dom.textContent = attrs[k];
      else if (k === "innerHTML") dom.innerHTML = attrs[k];
      else dom.setAttribute(k, attrs[k]);
    }
    if (parent) parent.appendChild(dom);
    return dom;
  }
  function clear(node) { while (node.firstChild) node.removeChild(node.firstChild); }
  function poly(parent, points, attrs) { return el("polygon", Object.assign({ points: ptsStr(points) }, attrs), parent); }

  // ================= 编辑器回调 =================
  const editor = new Editor($("svg"), $("view"), {
    onCoord: w => { $("coordReadout").textContent = `X ${Math.round(w.x)} mm　Y ${Math.round(w.y)} mm`; },
    onViewport: () => drawScale(),
    onPick: (kind, id) => selectElement(kind, id),
    onDelete: (kind, id) => deleteElement(kind, id),
    onDrag: (kind, id, dx, dy, ev, w, drag) => dragElement(kind, id, dx, dy, ev, w, drag),
    onDrawStart: t => flashHint(t === "door" ? "沿墙边拖出门洞" : "拖动拉出矩形"),
    onDrawEnd: (tool, p0, p1) => finishDraw(tool, p0, p1),
    onPlaceClick: w => {
      if (state.pendingPlace) {
        const f = state.furniture.find(x => x.id === state.pendingPlace);
        if (f) {
          f.placement = { x: Math.round(w.x / 50) * 50, y: Math.round(w.y / 50) * 50, deg: 0 };
          flashHint(`已放置 ${f.name}`);
        }
        state.pendingPlace = null;
        currentTool = "select";
        editor.setTool("select");
        renderAll();
      }
    },
  });

  // ---------------- 绘制完成 ----------------
  function finishDraw(tool, p0, p1) {
    if (tool === "room") {
      const x = Math.min(p0.x, p1.x), y = Math.min(p0.y, p1.y);
      const w = Math.abs(p1.x - p0.x), h = Math.abs(p1.y - p0.y);
      if (w < 300 || h < 300) return;
      const sp = {
        id: uid("sp"), name: "空间" + state.spaces.length, height: 2600,
        poly: [[x, y], [x + w, y], [x + w, y + h], [x, y + h]],
      };
      state.spaces.push(sp);
      selectElement("space", sp.id);
    } else if (tool === "obstacle") {
      const x = (p0.x + p1.x) / 2, y = (p0.y + p1.y) / 2;
      const w = Math.abs(p1.x - p0.x), h = Math.abs(p1.y - p0.y);
      if (w < 80 || h < 80) return;
      const ob = { id: uid("ob"), name: "障碍" + (state.obstacles.length + 1),
        x: Math.round(x), y: Math.round(y), w: Math.round(w), d: Math.round(h),
        h: 2200, deg: 0 };
      state.obstacles.push(ob);
      selectElement("obstacle", ob.id);
    } else if (tool === "door") {
      const dx = p1.x - p0.x, dy = p1.y - p0.y;
      const width = Math.round(Math.hypot(dx, dy));
      if (width < 500) { flashHint("门洞太短"); return; }
      const dr = {
        id: uid("dr"), name: "门" + (state.doors.length + 1),
        x: Math.round((p0.x + p1.x) / 2), y: Math.round((p0.y + p1.y) / 2),
        wx: round1(dx / width), wy: round1(dy / width),
        width, clearHeight: 2050, openAngle: 90, swing: 1,
      };
      // 默认开启侧：自动选空间所在法向侧（若门贴在某空间边上）
      state.doors.push(dr);
      selectElement("door", dr.id);
    }
    renderAll();
  }

  // ---------------- 选择 / 删除 / 拖动 ----------------
  function selectElement(kind, id) {
    if (kind === "pose") {
      state.selected = { kind: "pose", id };
      renderRouteLayer(); updateSidebar();
      return;
    }
    if (state.mode !== "plan") return;
    state.selected = { kind, id };
    renderAll();
  }

  function deleteElement(kind, id) {
    if (kind === "space") state.spaces = state.spaces.filter(s => s.id !== id);
    if (kind === "door") state.doors = state.doors.filter(d => d.id !== id);
    if (kind === "obstacle") state.obstacles = state.obstacles.filter(o => o.id !== id);
    state.selected = null;
    renderAll(); updateSidebar();
  }

  function dragElement(kind, id, dx, dy, ev, w, drag) {
    if (kind === "space") {
      const sp = state.spaces.find(s => s.id === id);
      sp.poly = sp.poly.map(([x, y]) => [round1(x + dx), round1(y + dy)]);
      renderSpaces(); renderDoors(); renderObstacles(); renderRulers();
    } else if (kind === "door") {
      const dr = state.doors.find(d => d.id === id);
      dr.x = round1(dr.x + dx); dr.y = round1(dr.y + dy);
      renderSpaces(); renderDoors();
    } else if (kind === "obstacle") {
      const ob = state.obstacles.find(o => o.id === id);
      ob.x = round1(ob.x + dx); ob.y = round1(ob.y + dy);
      renderObstacles();
    } else if (kind === "pose") {
      const idx = Number(id);
      const pose = state.poses[idx];
      if (!pose) return;
      if (drag.hit.getAttribute("data-move") === "rotate") {
        const ang = Math.atan2(w.y - pose.y, w.x - pose.x) * 180 / Math.PI;
        pose.deg = Math.round(ang / 5) * 5;
      } else {
        pose.x = Math.round(w.x / 50) * 50;
        pose.y = Math.round(w.y / 50) * 50;
      }
      state.selected = { kind: "pose", id };
      renderRouteLayer(); updateSidebar();
      scheduleRouteCheck();
    }
  }

  // ================= 渲染 =================
  function renderAll() {
    renderSpaces(); renderDoors(); renderObstacles();
    renderFurnitureLayer(); renderUsage(); renderRouteLayer(); renderRulers(); renderLayout();
    drawScale(); updateSidebar();
  }

  function renderSpaces() {
    clear(layer.space);
    state.spaces.forEach(sp => {
      const g = el("g", { "data-hit": "space", "data-id": sp.id }, layer.space);
      poly(g, sp.poly, { class: "space-fill", fill: "#fff" });
      const c = polyCenter(sp.poly);
      el("text", { class: "space-label", x: c[0], y: c[1] - 8, text: sp.name }, g);
      el("text", { class: "space-dim", x: c[0], y: c[1] + 10,
        text: `净高 ${sp.height} mm` }, g);
      poly(g, sp.poly, { class: "hit-area", "stroke-width": 24, fill: "rgba(255,255,255,.01)" });
    });
    wallSegmentsJS().forEach(([a, b]) => {
      el("line", { class: "wall", x1: a[0], y1: a[1], x2: b[0], y2: b[1] }, layer.space);
    });
  }

  function renderDoors() {
    clear(layer.door);
    state.doors.forEach(dr => {
      const g = el("g", { "data-hit": "door", "data-id": dr.id }, layer.door);
      poly(g, doorSwingPoints(dr), { class: "door-swing" });
      const wu = unit([dr.wx ?? 1, dr.wy ?? 0]);
      const a = [dr.x - wu[0] * dr.width / 2, dr.y - wu[1] * dr.width / 2];
      const b = [dr.x + wu[0] * dr.width / 2, dr.y + wu[1] * dr.width / 2];
      el("line", { class: "gate", x1: a[0], y1: a[1], x2: b[0], y2: b[1] }, g);
      const [h, e] = doorLeafPoints(dr);
      el("line", { class: "door-leaf", x1: h[0], y1: h[1], x2: e[0], y2: e[1] }, g);
      el("circle", { cx: h[0], cy: h[1], r: 14, fill: "#0ea5e9" }, g);
      const n = [-wu[1], wu[0]];
      el("text", { class: "door-label", x: dr.x + n[0] * 40, y: dr.y + n[1] * 40 - 20,
        text: `${dr.name} ${dr.width}×${dr.clearHeight}` }, g);
      el("line", { x1: a[0], y1: a[1], x2: b[0], y2: b[1],
        class: "hit-area", "stroke-width": 70 }, g);
    });
  }

  function renderObstacles() {
    clear(layer.obstacle);
    state.obstacles.forEach(ob => {
      const pts = rectPoints(ob.x, ob.y, ob.w, ob.d, ob.deg || 0);
      const g = el("g", { "data-hit": "obstacle", "data-id": ob.id }, layer.obstacle);
      poly(g, pts, { class: "obstacle" });
      // 交叉线纹理
      el("line", { x1: pts[0][0], y1: pts[0][1], x2: pts[2][0], y2: pts[2][1],
        class: "obstacle-hatch" }, g);
      el("line", { x1: pts[1][0], y1: pts[1][1], x2: pts[3][0], y2: pts[3][1],
        class: "obstacle-hatch" }, g);
      el("text", { class: "obstacle-label", x: ob.x, y: ob.y + 4, text: ob.name }, g);
      poly(g, pts, { class: "hit-area", "stroke-width": 18, fill: "rgba(0,0,0,.001)" });
    });
  }

  function renderFurnitureLayer() {
    clear(layer.furniture);
    state.furniture.forEach(f => {
      const pos = f.placement;
      if (!pos) return;
      const pts = rectPoints(pos.x, pos.y, f.w, f.d, pos.deg || 0);
      const g = el("g", {}, layer.furniture);
      poly(g, pts, { class: "furn-body" + (f.locked ? " locked" : " placed") });
      el("text", { class: "furn-label", x: pos.x, y: pos.y - 4, text: f.name }, g);
      el("text", { class: "furn-dim", x: pos.x, y: pos.y + 12,
        text: `${f.w}×${f.d}×${f.h}` }, g);
      if (f.locked) el("text", { class: "furn-dim", x: pos.x, y: pos.y - 24, text: "🔒" }, g);
    });
  }

  function renderUsage() {
    clear(layer.usage);
    state.furniture.forEach(f => {
      if (!f.placement || !f.usage) return;
      usagePolysJS(f, f.placement).forEach(p => poly(layer.usage, p, { class: "usage-poly" }));
    });
  }

  function renderRulers() {
    clear(layer.ruler);
    state.spaces.forEach(sp => {
      sp.poly.forEach((a, i) => {
        const b = sp.poly[(i + 1) % sp.poly.length];
        const L = Math.round(Math.hypot(b[0] - a[0], b[1] - a[1]));
        const mx = (a[0] + b[0]) / 2, my = (a[1] + b[1]) / 2;
        // 尺寸文字放在空间内侧，避免压墙
        const c = polyCenter(sp.poly);
        const nx = -(b[1] - a[1]) / L, ny = (b[0] - a[0]) / L;
        const side = ((c[0] - mx) * nx + (c[1] - my) * ny) > 0 ? 1 : -1;
        el("text", { class: "ruler-text", x: mx + nx * 14 * side,
          y: my + ny * 14 * side + 3, "text-anchor": "middle", text: L + "" }, layer.ruler);
      });
    });
    drawScale();
  }

  function drawScale() {
    const pxPerMm = editor.scale;
    const targetPx = 120;
    const rawMm = targetPx / pxPerMm;
    const nice = [100, 200, 500, 1000, 2000, 5000].find(v => v >= rawMm) || 5000;
    const wPx = nice * pxPerMm;
    const lg = $("scaleLegend");
    lg.innerHTML = "";
    lg.textContent = `└${nice >= 1000 ? (nice / 1000) + " m" : nice + " mm"}┘`;
    lg.style.minWidth = Math.round(wPx + 20) + "px";
    lg.style.textAlign = "center";
  }

  // ---------------- 路线层 ----------------
  function renderRouteLayer() {
    clear(layer.route);
    clear(layer.overlay);
    const f = activeFurniture();
    if (state.mode !== "furn" || !f) return;

    const res = state.routeResult;
    const badIdx = res && !res.ok ? res.conflictIndex : null;

    // 相邻姿态的扫掠包络
    for (let i = 1; i < state.poses.length; i++) {
      const a = state.poses[i - 1], b = state.poses[i];
      const pa = rectPoints(a.x, a.y, f.w, f.d, a.deg || 0);
      const pb = rectPoints(b.x, b.y, f.w, f.d, b.deg || 0);
      const env = convexHull(pa.concat(pb));
      const stopped = badIdx != null && i > badIdx;
      const bad = badIdx === i && res.conflictKind === "segment";
      poly(layer.route, env, {
        class: "envelope" + (bad ? " bad" : ""),
        opacity: stopped ? .2 : .9,
      });
    }
    // 连线
    if (state.poses.length > 1) {
      const d = state.poses.map((p, i) => (i ? "L" : "M") + p.x + " " + p.y).join(" ");
      el("path", { class: "route-line", d }, layer.route);
    }
    // 姿态本体
    state.poses.forEach((p, i) => {
      const pts = rectPoints(p.x, p.y, f.w, f.d, p.deg || 0);
      const stopped = badIdx != null && i > badIdx;
      const bad = badIdx === i;
      const g = el("g", {}, layer.route);
      g.setAttribute("opacity", stopped ? .3 : 1);
      poly(g, pts, {
        class: "pose-body" + (bad ? " bad" : "") + (state.selected?.kind === "pose" &&
               Number(state.selected.id) === i ? " selected" : ""),
        "data-hit": "pose", "data-id": i, "data-move": "body",
      });
      // 旋转手柄
      const a = (p.deg || 0) * Math.PI / 180;
      const hx = p.x + Math.cos(a) * f.w / 2, hy = p.y + Math.sin(a) * f.w / 2;
      const rx = p.x + Math.cos(a) * (f.w / 2 + 45), ry = p.y + Math.sin(a) * (f.w / 2 + 45);
      el("line", { x1: hx, y1: hy, x2: rx, y2: ry, stroke: "#2563eb",
        "stroke-width": 1.5, "stroke-dasharray": "3 2" }, g);
      el("circle", { class: "pose-handle", cx: rx, cy: ry, r: 9,
        "data-hit": "pose", "data-id": i, "data-move": "rotate" }, g);
      el("text", { class: "pose-num", x: p.x, y: p.y + 4, text: i + 1 + "",
        "data-hit": "pose", "data-id": i, "data-move": "body" }, g);
    });

    // 冲突标记
    if (res && !res.ok && res.conflict) drawConflictMark(res.conflict);
  }

  function drawConflictMark(c) {
    if (!c.point) return;
    const { x, y } = c.point;
    el("circle", { class: "conflict-ring", cx: x, cy: y, r: 45 }, layer.overlay);
    el("circle", { class: "conflict-mark", cx: x, cy: y, r: 16 }, layer.overlay);
    el("text", { x: x, y: y + 5, "text-anchor": "middle", fill: "#fff",
      "font-size": 15, "font-weight": 700, text: "!" }, layer.overlay);
    const rem = c.remaining != null ? Math.abs(Math.round(c.remaining)) : null;
    const label = rem != null && c.type !== "door_swing" ? `差 ${rem} mm` : "冲突";
    el("rect", { class: "conflict-callout", x: x + 20, y: y - 44,
      width: 86, height: 22, rx: 5 }, layer.overlay);
    el("text", { class: "conflict-text", x: x + 63, y: y - 29,
      "text-anchor": "middle", text: label }, layer.overlay);
  }

  function convexHull(points) {
    const pts = points.slice().sort((a, b) => a[0] - b[0] || a[1] - b[1]);
    const cr = (o, a, b) => (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
    const lo = []; pts.forEach(p => { while (lo.length >= 2 && cr(lo[lo.length - 2], lo[lo.length - 1], p) <= 0) lo.pop(); lo.push(p); });
    const up = []; pts.slice().reverse().forEach(p => { while (up.length >= 2 && cr(up[up.length - 2], up[up.length - 1], p) <= 0) up.pop(); up.push(p); });
    return lo.slice(0, -1).concat(up.slice(0, -1));
  }

  // ---------------- 布局优化层 ----------------
  function renderLayout() {
    clear(layer.layout);
    if (state.mode !== "layout" || !state.layoutResult) return;
    const cand = state.layoutResult.ranked[state.layoutSelected];
    if (!cand) return;
    const byId = Object.fromEntries(cand.placements.map(p => [p.id, p]));
    state.furniture.forEach(f => {
      const p = byId[f.id];
      if (!p) return;
      poly(layer.layout, rectPoints(p.x, p.y, f.w, f.d, p.deg || 0), { class: "layout-furn" });
      el("text", { class: "furn-label", x: p.x, y: p.y + 4, text: f.name }, layer.layout);
    });
    Object.values(cand.paths || {}).forEach(path => {
      if (path.length < 2) return;
      const d = path.map((p, i) => (i ? "L" : "M") + p[0] + " " + p[1]).join(" ");
      el("path", { class: "layout-path", d }, layer.layout);
    });
  }

  // ================= 路线计算 / 播放 =================
  const scheduleRouteCheck = debounce(checkRoute, 250);

  async function checkPoseLive(pose) {
    const f = activeFurniture();
    if (!f) return;
    return api("/api/check/pose", { plan: planData(), furniture: f, pose });
  }

  async function checkRoute() {
    const f = activeFurniture();
    if (!f || !state.poses.length) { state.routeResult = null; renderRouteLayer(); return; }
    state.routeResult = await api("/api/check/route", {
      plan: planData(), furniture: f, poses: state.poses });
    renderRouteLayer(); updateSidebar();
    if (state.playing) drivePlayback();
  }

  function playRoute() {
    const f = activeFurniture();
    if (!f) { flashHint("请先选择家具"); return; }
    if (state.poses.length < 2) { flashHint("至少需要两个关键姿态"); return; }
    hideConflict();
    stopPlay();
    state.playing = true;
    drivePlayback._i = 0;
    checkRoute();
  }

  let playTimer = null;
  function stopPlay() {
    state.playing = false;
    clearTimeout(playTimer);
    const ghost = document.getElementById("playGhost");
    if (ghost) ghost.remove();
  }

  async function drivePlayback() {
    const f = activeFurniture();
    const res = state.routeResult;
    if (!res) { state.playing = false; return; }
    let ghost = document.getElementById("playGhost");
    if (!ghost) {
      ghost = el("g", { id: "playGhost" }, layer.route);
      el("polygon", { class: "moving-pose" }, ghost);
      el("text", { class: "furn-label", y: 4, "text-anchor": "middle", text: f.name }, ghost);
    }
    const polyNode = ghost.firstChild;
    const txtNode = ghost.lastChild;
    const limit = res.ok ? state.poses.length : res.conflictIndex + 1;
    // 逐段插值；冲突段只走到冲突处（简化：停在该段起点后推进入一点点）
    let i = (drivePlayback._i ?? 0);
    if (i >= limit) i = 0;
    drivePlayback._i = i;

    function frame(segIdx, t) {
      if (!state.playing) return;
      let x, y, deg;
      if (segIdx >= state.poses.length - 1) {
        const p = state.poses[segIdx];
        x = p.x; y = p.y; deg = p.deg || 0;
      } else {
        const a = state.poses[segIdx], b = state.poses[segIdx + 1];
        // 冲突段在 55% 处停下
        const tt = (!res.ok && res.conflictKind === "segment" &&
                    segIdx + 1 === res.conflictIndex) ? t * 0.55 : t;
        x = a.x + (b.x - a.x) * tt; y = a.y + (b.y - a.y) * tt;
        deg = (a.deg || 0) + ((b.deg || 0) - (a.deg || 0)) * tt;
      }
      polyNode.setAttribute("points", ptsStr(rectPoints(x, y, f.w, f.d, deg)));
      txtNode.setAttribute("x", x); txtNode.setAttribute("y", y - Math.max(f.w, f.d) / 2 - 8);
      if (t < 1) {
        playTimer = setTimeout(() => frame(segIdx, Math.min(1, t + 0.04)), 30);
      } else {
        const next = segIdx + 1;
        if (next >= limit) {
          state.playing = false;
          if (res.ok) showConflictOk(f, res);
          else showConflict(res);
          return;
        }
        // 若下一段是冲突段，播到一半停
        drivePlayback._i = next;
        playTimer = setTimeout(() => frame(next, 0), 250);
      }
    }
    frame(i, 0);
  }

  function showConflict(res) {
    const c = res.conflict;
    const bar = $("conflictBar");
    bar.classList.remove("ok"); bar.hidden = false;
    const typeName = { wall: "墙体", obstacle: "家具/障碍", door_swing: "门扇",
      height: "净高/门楣", outside: "越界" }[c.type] || "冲突";
    $("conflictIcon").textContent = "⚠️";
    $("conflictTitle").textContent =
      `第 ${res.conflictIndex + 1} 个${res.conflictKind === "segment" ? "段" : "姿态"}受阻：${typeName}`;
    let detail = c.message;
    if (c.type === "height")
      detail += `（剩余 ${Math.round(c.remaining)} mm）`;
    else if (c.remaining > 0)
      detail += `　剩余/侵入 ${Math.round(c.remaining)} mm`;
    if (res.minSideGap != null)
      detail += `　路线最小侧隙 ${Math.round(res.minSideGap)} mm`;
    $("conflictDetail").textContent = detail;
  }
  function showConflictOk(f, res) {
    const bar = $("conflictBar");
    bar.classList.add("ok"); bar.hidden = false;
    $("conflictIcon").textContent = "✅";
    $("conflictTitle").textContent = `路线可行：${f.name} 可搬入预定位置`;
    $("conflictDetail").textContent =
      (res.minSideGap != null ? `最小通行侧隙 ${Math.round(res.minSideGap)} mm` : "无侧隙数据")
      + `　共 ${state.poses.length} 个关键姿态`;
  }
  function hideConflict() { $("conflictBar").hidden = true; $("conflictBar").classList.remove("ok"); }

  // ================= 右侧栏 =================
  function updateSidebar() {
    updateProps();
    updateFurnList();
    updatePoseList();
    updateUsageControls();
    updateLockList();
  }

  // 户型属性
  function updateProps() {
    ["spaceForm", "doorForm", "obstacleForm"].forEach(id => $(id).hidden = true);
    const body = $("propBody");
    if (!state.selected || state.mode !== "plan") {
      body.innerHTML = state.spaces.length
        ? `<p class="muted">共 ${state.spaces.length} 个空间、${state.doors.length} 个门洞、${state.obstacles.length} 个固定障碍。<br>点击要素可编辑。</p>`
        : `<p class="muted">先用左侧「房间/走廊」工具画出空间，<br>再沿墙边拖出门洞，最后放置固定障碍。</p>`;
      return;
    }
    const { kind, id } = state.selected;
    if (kind === "space") {
      const sp = state.spaces.find(s => s.id === id); if (!sp) return;
      $("spaceForm").hidden = false;
      $("spName").value = sp.name;
      $("spHeight").value = sp.height;
      const w = Math.round(Math.hypot(sp.poly[1][0] - sp.poly[0][0], sp.poly[1][1] - sp.poly[0][1]));
      const h = Math.round(Math.hypot(sp.poly[3][0] - sp.poly[0][0], sp.poly[3][1] - sp.poly[0][1]));
      $("spaceDims").textContent = `开间 × 进深：${w} × ${h} mm`;
      body.innerHTML = "";
    } else if (kind === "door") {
      const dr = state.doors.find(d => d.id === id); if (!dr) return;
      $("doorForm").hidden = false;
      $("drName").value = dr.name; $("drWidth").value = dr.width;
      $("drClear").value = dr.clearHeight; $("drOpen").value = dr.openAngle;
      $("drOpenVal").textContent = dr.openAngle + "°";
      $("drSwing").value = String(dr.swing ?? 1);
      body.innerHTML = "";
    } else if (kind === "obstacle") {
      const ob = state.obstacles.find(o => o.id === id); if (!ob) return;
      $("obstacleForm").hidden = false;
      $("obName").value = ob.name; $("obW").value = ob.w;
      $("obD").value = ob.d; $("obH").value = ob.h;
      $("obDeg").value = ob.deg || 0;
      $("obDegVal").textContent = (ob.deg || 0) + "°";
      body.innerHTML = "";
    }
  }

  function updateFurnList() {
    const ul = $("furnList");
    ul.innerHTML = "";
    state.furniture.forEach(f => {
      const li = el("li", { }, ul);
      li.className = (f.id === state.activeFurn ? "active " : "") + (f.locked ? "locked" : "");
      const sp = el("span", { class: "fname", text: f.name }, li);
      sp.onclick = (e) => { e.stopPropagation(); setActiveFurn(f.id); };
      el("span", { class: "fdim", text: `${f.w}×${f.d}×${f.h}` }, li);
      if (f.locked) el("span", { class: "tag-lock", text: "锁定" }, li);
      const del = el("button", { class: "icon-btn", text: "🗑", title: "删除家具" }, li);
      del.onclick = (e) => { e.stopPropagation(); removeFurniture(f.id); };
      li.onclick = () => setActiveFurn(f.id);
    });
  }

  function updatePoseList() {
    const box = $("poseList");
    box.innerHTML = "";
    const f = activeFurniture();
    $("routeTitle").textContent = f ? `搬运路线 · ${f.name}` : "搬运路线";
    $("routeControls").style.opacity = f ? 1 : .45;
    // 抬起角滑块同步到选中姿态（不触发 input 事件以免循环）
    const selPose = state.selected?.kind === "pose"
      ? state.poses[Number(state.selected.id)]
      : state.poses[state.poses.length - 1];
    if (selPose) $("posePitch").value = selPose.pitch || 0;
    const res = state.routeResult;
    state.poses.forEach((p, i) => {
      const bad = res && !res.ok && res.conflictIndex === i;
      const li = el("li", { class: bad ? "bad" : "" }, box);
      el("span", { class: "pidx", text: i + 1 + "" }, li);
      const info = el("span", { class: "pinfo" }, li);
      el("div", { text: `(${Math.round(p.x)}, ${Math.round(p.y)})　${p.deg || 0}°${p.pitch ? "　抬" + p.pitch + "°" : ""}` }, info);
      const segIssue = res && !res.ok && res.conflictKind === "segment" && res.conflictIndex === i;
      el("div", { class: "pmeta",
        text: bad || segIssue ? "⛔ " + res.conflict.message :
          (i === state.poses.length - 1 ? "终点（预定位置）" : "关键姿态") }, info);
      const xbtn = el("button", { class: "icon-btn", text: "✕" }, li);
      xbtn.onclick = (e) => {
        e.stopPropagation();
        state.poses.splice(i, 1);
        state.selected = null;
        renderRouteLayer(); updateSidebar(); scheduleRouteCheck();
      };
      li.onclick = () => { state.selected = { kind: "pose", id: i }; renderRouteLayer(); };
    });
  }

  function updateUsageControls() {
    const f = activeFurniture();
    const box = $("usageControls");
    box.style.opacity = f && f.placement ? 1 : .5;
    if (!f) return;
    const u = f.usage || {};
    $("ugDoor").checked = !!u.doors?.length;
    $("ugDrawer").checked = !!u.drawers?.length;
    $("ugChair").checked = !!u.chair;
    $("ugDoorBox").hidden = !u.doors?.length;
    $("ugDrawerBox").hidden = !u.drawers?.length;
    $("ugChairBox").hidden = !u.chair;
    if (u.doors?.length) {
      const d = u.doors[0];
      $("ugDoorEdge").value = d.edge || "front";
      $("ugDoorOpen").value = d.open ?? 90;
      $("ugDoorReach").value = d.reach ?? 600;
    }
    if (u.drawers?.length) {
      $("ugDrawerEdge").value = u.drawers[0].edge || "front";
      $("ugDrawerReach").value = u.drawers[0].reach ?? 700;
    }
    if (u.chair) {
      $("ugChairReach").value = u.chair.reach ?? 300;
      $("ugChairSeat").value = u.chair.seat ?? 500;
    }
  }

  function updateLockList() {
    const sel = $("entryDoor");
    if (sel.options.length !== state.doors.length) {
      sel.innerHTML = "";
      state.doors.forEach(d => el("option", { value: d.id, text: d.name }, sel));
    }
    const ul = $("lockList");
    ul.innerHTML = "";
    state.furniture.forEach(f => {
      const li = el("li", null, ul);
      li.className = f.locked ? "locked" : "";
      const cb = el("input", { type: "checkbox" }, li);
      cb.checked = !!f.locked;
      cb.style.marginRight = "6px";
      cb.onchange = () => {
        if (cb.checked && !f.placement) {
          cb.checked = false;
          flashHint("请先在家具页放置 " + f.name);
          setMode("furn"); setActiveFurn(f.id);
          beginPlace(f.id);
          return;
        }
        f.locked = cb.checked;
        renderFurnitureLayer(); updateFurnList(); updateLockList();
      };
      el("span", { class: "fname", text: f.name }, li);
      el("span", { class: "fdim", text: f.placement ? "已放置" : "未放置" }, li);
    });
  }

  // ================= 家具与姿态操作 =================
  function addFurniture() {
    const name = $("fName").value.trim() || ("家具" + (state.furniture.length + 1));
    const f = {
      id: uid("f"), name,
      w: Number($("fW").value) || 1000,
      d: Number($("fD").value) || 600,
      h: Number($("fH").value) || 2000,
      locked: false,
    };
    state.furniture.push(f);
    $("fName").value = "";
    setActiveFurn(f.id);
    currentTool = "pose";
    editor.setTool("pose");
    document.querySelectorAll("#toolrail .tool").forEach(b =>
      b.classList.toggle("active", b.dataset.tool === "pose"));
    flashHint("已添加：" + name + "，点击画布添加关键姿态，拖蓝点旋转");
  }
  function removeFurniture(id) {
    state.furniture = state.furniture.filter(f => f.id !== id);
    if (state.activeFurn === id) { state.activeFurn = null; state.poses = []; state.routeResult = null; }
    renderAll();
  }
  function setActiveFurn(id) {
    state.activeFurn = id;
    state.poses = [];
    state.routeResult = null;
    state.selected = null;
    currentTool = "pose";
    editor.setTool("pose");
    document.querySelectorAll("#toolrail .tool").forEach(b =>
      b.classList.toggle("active", b.dataset.tool === "pose"));
    const f = activeFurniture();
    if (f) $("pitchVal").textContent = `水平搬运，有效高 = ${f.h} mm`;
    renderRouteLayer(); updateSidebar();
  }
  function beginPlace(id) {
    state.pendingPlace = id;
    editor.setTool("place");
    flashHint("在房间内点击放置家具");
  }

  function addPoseAt(w) {
    const f = activeFurniture();
    if (!f) return;
    const x = Math.round(w.x / 50) * 50, y = Math.round(w.y / 50) * 50;
    const last = state.poses[state.poses.length - 1];
    state.poses.push({ x, y, deg: last ? last.deg : 0, pitch: last ? last.pitch : 0 });
    state.selected = { kind: "pose", id: state.poses.length - 1 };
    renderRouteLayer(); updateSidebar(); scheduleRouteCheck();
  }

  // ================= 摆放优化 =================
  async function runOptimize() {
    if (!state.furniture.length) { flashHint("请先添加家具"); return; }
    const movable = state.furniture.filter(f => !f.locked);
    if (!movable.length) { flashHint("所有家具都已锁定，没有可调整对象"); return; }
    $("optStatus").innerHTML = '<span class="spinner"></span> 正在搜索摆放…';
    $("optResults").innerHTML = "";
    const res = await api("/api/optimize", {
      plan: planData(),
      furniture: state.furniture.map(f => ({
        id: f.id, name: f.name, w: f.w, d: f.d, h: f.h,
        locked: f.locked,
        x: f.placement ? f.placement.x : undefined,
        y: f.placement ? f.placement.y : undefined,
        deg: f.placement ? f.placement.deg : undefined,
      })),
      entryDoorId: $("entryDoor").value || undefined,
      samples: 40,
    });
    if (res.error) { $("optStatus").textContent = "出错：" + res.error; return; }
    state.layoutResult = res; state.layoutSelected = 0;
    renderLayout(); renderOptCards();
    const n = res.ranked.length;
    $("optStatus").innerHTML = n
      ? `共生成 <b>${n}</b> 个方案，点击卡片预览，「采用」写入摆放位。`
      : "未找到可行摆放：请检查锁定物件是否占满空间。";
  }

  function renderOptCards() {
    const box = $("optResults");
    box.innerHTML = "";
    state.layoutResult.ranked.forEach((c, i) => {
      const card = el("div", { class: "opt-card" + (i === state.layoutSelected ? " selected" : "") }, box);
      const allMovable = state.furniture.filter(f => !f.locked).length;
      el("div", { class: "rank", text: `方案 ${i + 1}` }, card);
      const m = el("div", { class: "opt-metrics" }, card);
      const m1 = el("div", null, m);
      el("span", { text: "可达家具：" }, m1);
      el("b", { class: c.unreachableCount ? "badge-bad" : "badge-ok",
        text: c.reachable.length + "/" + allMovable }, m1);
      const m2 = el("div", null, m);
      el("span", { text: "最小通行余量：" }, m2);
      el("b", { text: (c.minClearance ?? "—") + " mm" }, m2);
      const m3 = el("div", null, m);
      el("span", { text: "转向次数：" }, m3); el("b", { text: c.turns + "" }, m3);
      const m4 = el("div", null, m);
      el("span", { text: "占地紧凑度：" }, m4); el("b", { text: c.compactness + "" }, m4);
      if (c.unreachableCount)
        el("div", { class: "badge-bad", text: "不可达：" + c.unreachable.join("、") }, card);
      const btns = el("div", { style: "margin-top:8px;display:flex;gap:6px;" }, card);
      const use = el("button", { class: "primary small", text: "采用此方案" }, btns);
      use.onclick = (e) => { e.stopPropagation(); applyLayout(c); };
      card.onclick = () => { state.layoutSelected = i; renderLayout(); renderOptCards(); };
    });
  }

  function applyLayout(c) {
    c.placements.forEach(p => {
      const f = state.furniture.find(x => x.id === p.id);
      if (f) f.placement = { x: p.x, y: p.y, deg: p.deg || 0 };
    });
    state.layoutResult = null;
    renderFurnitureLayer(); renderUsage(); renderLayout();
    updateSidebar();
    flashHint("已写入摆放位，可在家具页设置使用包络");
  }

  // ================= 模式切换 =================
  function setMode(mode) {
    state.mode = mode;
    document.querySelectorAll(".tab").forEach(t => t.classList.toggle("active", t.dataset.mode === mode));
    document.querySelectorAll(".rail-group").forEach(g =>
      g.style.display = g.dataset.show === mode ? "flex" : "none");
    document.querySelectorAll(".panel").forEach(p =>
      p.hidden = p.dataset.panel !== mode);
    $("conflictBar").hidden = true;
    stopPlay();
    clear(layer.layout);
    if (mode === "plan") { currentTool = "select"; editor.setTool("select"); renderAll(); }
    if (mode === "furn") {
      currentTool = state.furniture.length ? "pose" : "select";
      editor.setTool(currentTool); renderAll();
    }
    if (mode === "layout") { currentTool = "select"; editor.setTool("select"); state.layoutResult = null; renderAll(); }
    document.querySelectorAll("#toolrail .tool").forEach(b =>
      b.classList.toggle("active", b.dataset.tool === currentTool));
  }
  let currentTool = "select";
  function selectTool() { return currentTool; }

  // ================= 方案存取 =================
  async function saveScheme() {
    const body = { id: state.schemeId || undefined, name: $("schemeName").value, data: planData() };
    const r = await api("/api/schemes", body);
    if (r.ok) { state.schemeId = r.id; $("saveHint").textContent = "已保存 " + new Date().toLocaleTimeString(); loadSchemeList(); }
  }
  async function loadSchemeList() {
    const list = await fetch("/api/schemes").then(r => r.json());
    const sel = $("schemeList");
    sel.innerHTML = "";
    el("option", { value: "", text: "— 已保存方案 —" }, sel);
    list.forEach(s => el("option", { value: s.id, text: s.name }, sel));
  }
  async function loadScheme(id) {
    const r = await fetch("/api/schemes/" + id).then(x => x.json());
    if (!r.data) return;
    applyData(r.data);
    state.schemeId = r.id; $("schemeName").value = r.name;
    flashHint("已读取：" + r.name);
    editor.fit(bounds());
  }
  function applyData(d) {
    state.spaces = d.spaces || []; state.doors = d.doors || [];
    state.obstacles = d.obstacles || []; state.furniture = d.furniture || [];
    state.activeFurn = null; state.poses = []; state.routeResult = null; state.layoutResult = null;
    state.seq = 1000 + Math.floor(Math.random() * 9000);
    renderAll();
  }
  function newScheme() {
    if (!confirm("新建空白方案？当前未保存内容将丢失。")) return;
    state.schemeId = null;
    applyData({ spaces: [], doors: [], obstacles: [], furniture: [] });
  }

  // ---------------- 示例户型 ----------------
  function loadDemo() {
    const data = {
      spaces: [
        { id: "sp_hall", name: "楼道走廊", height: 2600,
          poly: [[0, 0], [2000, 0], [2000, 1600], [0, 1600]] },
        { id: "sp_room", name: "卧室", height: 2700,
          poly: [[2000, 0], [5200, 0], [5200, 3200], [2000, 3200]] },
      ],
      doors: [
        { id: "dr1", name: "入户房门", x: 2000, y: 300, wx: 0, wy: 1,
          width: 850, clearHeight: 2050, openAngle: 90, swing: 1 },
      ],
      obstacles: [
        { id: "ob1", name: "管井", x: 4750, y: 350, w: 500, d: 500, h: 2600, deg: 0 },
      ],
      furniture: [
        { id: "f_bed", name: "单人床", w: 1200, d: 2000, h: 480, locked: true,
          placement: { x: 4500, y: 2200, deg: 0 } },
        { id: "f_ward", name: "双门衣柜", w: 1000, d: 600, h: 2100, locked: false,
          usage: { doors: [{ edge: "front", open: 90, reach: 600 }] } },
        { id: "f_desk", name: "书桌", w: 1200, d: 600, h: 750, locked: false,
          usage: { chair: { reach: 300, seat: 500 } } },
        { id: "f_chair", name: "椅子", w: 450, d: 450, h: 900, locked: false },
      ],
    };
    applyData(data);
    state.schemeId = null;
    $("schemeName").value = "示例：单间合租房";
    editor.fit(bounds());
    flashHint("示例户型已载入，切到②家具页选衣柜拼路线");
  }

  // ================= 事件绑定 =================
  function bind() {
    // 顶栏
    $("btnSave").onclick = saveScheme;
    $("btnNew").onclick = newScheme;
    $("btnLoadDemo").onclick = loadDemo;
    $("schemeList").onchange = e => e.target.value && loadScheme(e.target.value);
    document.querySelectorAll(".tab").forEach(t => t.onclick = () => setMode(t.dataset.mode));

    // 工具
    document.querySelectorAll("#toolrail .tool[data-tool]").forEach(btn => {
      btn.onclick = () => {
        document.querySelectorAll("#toolrail .tool").forEach(b => b.classList.remove("active"));
        btn.classList.add("active");
        currentTool = btn.dataset.tool;
        if (state.mode === "furn" && currentTool === "pose") {
          editor.setTool("pose");
          flashHint("点击画布添加关键姿态");
        } else {
          editor.setTool(currentTool);
        }
      };
    });
    $("btnGrid").onclick = () => {
      state.showGrid = !state.showGrid;
      $("gridRect").style.display = state.showGrid ? "" : "none";
    };
    $("btnFit").onclick = () => editor.fit(bounds());
    $("btnPlay").onclick = playRoute;
    $("btnStop").onclick = stopPlay;

    // 画布点击（家具模式在空白处加姿态）
    $("svg").addEventListener("mousedown", (e) => {
      if (e.target.closest("[data-hit]")) {
        window.__suppressCanvasClick = true;
        setTimeout(() => { window.__suppressCanvasClick = false; }, 0);
      }
    });
    $("svg").addEventListener("click", (e) => {
      if (window.__suppressCanvasClick) return;
      if (e.target.closest("[data-hit]")) return;
      const w = editor.toWorld(e.clientX, e.clientY);
      if (state.pendingPlace) return;
      if (state.mode === "furn" && currentTool === "pose") addPoseAt(w);
    });

    // 家具添加
    $("btnAddFurn").onclick = addFurniture;

    // 抬起角
    $("posePitch").oninput = e => {
      const f = activeFurniture();
      const pitch = Number(e.target.value);
      const sel = state.selected?.kind === "pose" ? Number(state.selected.id)
        : state.poses.length - 1;
      if (sel >= 0) state.poses[sel].pitch = pitch;
      if (f) {
        const H = f.h, longSide = Math.max(f.w, f.d);
        const eff = pitch ? H * Math.cos(pitch * Math.PI / 180) + longSide * Math.sin(pitch * Math.PI / 180) : H;
        $("pitchVal").textContent = pitch
          ? `抬起 ${pitch}°，有效竖向高 ≈ ${Math.round(eff)} mm`
          : `水平搬运，有效高 = ${f.h} mm`;
      }
      renderRouteLayer(); scheduleRouteCheck();
    };

    // 选中姿态的精确编辑
    $("poseList").addEventListener("change", () => {});

    // 使用包络控件
    const syncUsage = () => {
      const f = activeFurniture();
      if (!f) return;
      const u = {};
      if ($("ugDoor").checked) u.doors = [{ edge: $("ugDoorEdge").value,
        open: Number($("ugDoorOpen").value), reach: Number($("ugDoorReach").value) }];
      if ($("ugDrawer").checked) u.drawers = [{ edge: $("ugDrawerEdge").value,
        reach: Number($("ugDrawerReach").value) }];
      if ($("ugChair").checked) u.chair = { reach: Number($("ugChairReach").value),
        seat: Number($("ugChairSeat").value) };
      f.usage = Object.keys(u).length ? u : undefined;
      updateUsageControls(); renderUsage();
    };
    ["ugDoor", "ugDrawer", "ugChair", "ugDoorEdge", "ugDoorOpen", "ugDoorReach",
     "ugDrawerEdge", "ugDrawerReach", "ugChairReach", "ugChairSeat"]
      .forEach(id => $(id).addEventListener("change", syncUsage));

    // 放置 / 终点采用
    const placeBtn = el("button", { class: "small",
      style: "width:100%;margin-top:6px;",
      text: "📍 将当前家具放到房间（点击画布落位）" }, $("routeControls"));
    placeBtn.onclick = () => {
      const f = activeFurniture(); if (!f) return;
      beginPlace(f.id);
    };
    // 路线可行一键到位
    const adoptPose = el("button", { class: "primary small",
      style: "width:100%;margin-top:4px;", text: "✔ 用路线终点作为摆放位" }, $("routeControls"));
    adoptPose.onclick = () => {
      const f = activeFurniture();
      if (!f || !state.poses.length) return;
      const p = state.poses[state.poses.length - 1];
      f.placement = { x: p.x, y: p.y, deg: p.deg || 0 };
      renderFurnitureLayer(); renderUsage(); updateSidebar();
      flashHint("已将终点设为摆放位，可继续设置使用包络");
    };

    // 属性表单
    $("spName").oninput = e => withSel("space", s => { s.name = e.target.value; renderSpaces(); });
    $("spHeight").oninput = e => withSel("space", s => { s.height = Number(e.target.value); renderSpaces(); });
    $("drName").oninput = e => withSel("door", d => { d.name = e.target.value; renderDoors(); });
    $("drWidth").oninput = e => withSel("door", d => { d.width = Number(e.target.value); renderSpaces(); renderDoors(); });
    $("drClear").oninput = e => withSel("door", d => { d.clearHeight = Number(e.target.value); renderDoors(); });
    $("drOpen").oninput = e => withSel("door", d => {
      d.openAngle = Number(e.target.value); $("drOpenVal").textContent = d.openAngle + "°";
      renderDoors();
    });
    $("drSwing").onchange = e => withSel("door", d => { d.swing = Number(e.target.value); renderDoors(); });
    $("drFlip").onclick = () => withSel("door", d => {
      d.swing = -(d.swing ?? 1); $("drSwing").value = String(d.swing); renderDoors();
    });
    $("obName").oninput = e => withSel("obstacle", o => { o.name = e.target.value; renderObstacles(); });
    $("obW").oninput = e => withSel("obstacle", o => { o.w = Number(e.target.value); renderObstacles(); });
    $("obD").oninput = e => withSel("obstacle", o => { o.d = Number(e.target.value); renderObstacles(); });
    $("obH").oninput = e => withSel("obstacle", o => { o.h = Number(e.target.value); });
    $("obDeg").oninput = e => withSel("obstacle", o => {
      o.deg = Number(e.target.value); $("obDegVal").textContent = o.deg + "°";
      renderObstacles();
    });

    // 优化
    $("btnOptimize").onclick = runOptimize;
    $("conflictClose").onclick = hideConflict;

    // 键盘：Delete 删除选中，R 旋转
    window.addEventListener("keydown", e => {
      if ((e.key === "Delete" || e.key === "Backspace") &&
          !/INPUT|SELECT|TEXTAREA/.test(document.activeElement.tagName)) {
        if (state.selected && ["space", "door", "obstacle"].includes(state.selected.kind)) {
          deleteElement(state.selected.kind, state.selected.id);
        } else if (state.selected?.kind === "pose" && state.mode === "furn") {
          const i = Number(state.selected.id);
          state.poses.splice(i, 1);
          state.selected = null;
          renderRouteLayer(); updateSidebar(); scheduleRouteCheck();
        }
      }
      if (e.key === "r" || e.key === "R") {
        if (state.selected?.kind === "pose") {
          const p = state.poses[Number(state.selected.id)];
          p.deg = (p.deg + 5) % 360; renderRouteLayer(); scheduleRouteCheck();
        }
      }
    });
  }
  function withSel(kind, fn) {
    if (!state.selected || state.selected.kind !== kind) return;
    const map = { space: state.spaces, door: state.doors, obstacle: state.obstacles };
    const obj = map[kind].find(x => x.id === state.selected.id);
    if (obj) fn(obj);
  }

  // ---------------- 启动 ----------------
  window.__sim = { state, editor, planData, renderAll, setMode };
  bind();
  loadSchemeList();
  setMode("plan");
  renderAll();
  setTimeout(() => { editor.fit(bounds() || { minx: 0, miny: 0, maxx: 6000, maxy: 5000 }); }, 60);
})();
