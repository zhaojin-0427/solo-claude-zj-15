/* 轻量 SVG 编辑器：视口变换、捕捉、绘图工具与标尺。
   世界单位 mm；屏幕坐标 px。 */
(function (global) {
  const SVGNS = "http://www.w3.org/2000/svg";

  function el(tag, attrs, parent) {
    const node = document.createElementNS(SVGNS, tag);
    if (attrs) for (const k in attrs) {
      if (k === "text") node.textContent = attrs[k];
      else node.setAttribute(k, attrs[k]);
    }
    if (parent) parent.appendChild(node);
    return node;
  }

  function clear(node) {
    while (node.firstChild) node.removeChild(node.firstChild);
  }

  function ptsStr(pts) {
    return pts.map(p => `${round(p[0])},${round(p[1])}`).join(" ");
  }

  function round(v, d = 1) {
    const k = Math.pow(10, d);
    return Math.round(v * k) / k;
  }

  class Editor {
    constructor(svg, view, callbacks) {
      this.svg = svg;
      this.view = view;
      this.cb = callbacks || {};
      this.scale = 0.18;       // px per mm（初始）
      this.tx = 120;
      this.ty = 120;
      this.tool = "select";
      this.snap = true;
      this.snapStep = 50;
      this._drag = null;
      this._bind();
      this.applyTransform();
    }

    // ---------- 坐标变换 ----------
    toWorld(clientX, clientY) {
      const r = this.svg.getBoundingClientRect();
      const sx = clientX - r.left, sy = clientY - r.top;
      return { x: (sx - this.tx) / this.scale, y: (sy - this.ty) / this.scale };
    }
    toScreen(wx, wy) {
      return { x: wx * this.scale + this.tx, y: wy * this.scale + this.ty };
    }
    applyTransform() {
      this.view.setAttribute("transform",
        `translate(${this.tx},${this.ty}) scale(${this.scale})`);
    }
    fit(bbox, padMm = 600) {
      if (!bbox) bbox = { minx: 0, miny: 0, maxx: 6000, maxy: 6000 };
      const r = this.svg.getBoundingClientRect();
      const w = Math.max(100, bbox.maxx - bbox.minx);
      const h = Math.max(100, bbox.maxy - bbox.miny);
      this.scale = Math.min((r.width - 80) / (w + padMm * 2),
                            (r.height - 80) / (h + padMm * 2));
      this.tx = (r.width - w * this.scale) / 2 - bbox.minx * this.scale;
      this.ty = (r.height - h * this.scale) / 2 - bbox.miny * this.scale;
      this.applyTransform();
    }
    snapPt(p) {
      if (!this.snap) return p;
      const s = this.snapStep;
      return { x: Math.round(p.x / s) * s, y: Math.round(p.y / s) * s };
    }

    setTool(t) {
      this.tool = t;
      this.svg.className.baseVal = "tool-" + t;
    }

    // ---------- 鼠标交互 ----------
    _bind() {
      this.svg.addEventListener("mousedown", e => this._down(e));
      window.addEventListener("mousemove", e => this._move(e));
      window.addEventListener("mouseup", e => this._up(e));
      this.svg.addEventListener("wheel", e => this._wheel(e), { passive: false });
      this.svg.addEventListener("contextmenu", e => e.preventDefault());
    }

    _wheel(e) {
      e.preventDefault();
      const factor = e.deltaY < 0 ? 1.12 : 1 / 1.12;
      const w = this.toWorld(e.clientX, e.clientY);
      this.scale = Math.min(2, Math.max(0.02, this.scale * factor));
      const s2 = this.toScreen(w.x, w.y);
      this.tx += e.clientX - this.svg.getBoundingClientRect().left - s2.x;
      this.ty += e.clientY - this.svg.getBoundingClientRect().top - s2.y;
      this.applyTransform();
      this.cb.onViewport && this.cb.onViewport();
    }

    _down(e) {
      const w = this.snapPt(this.toWorld(e.clientX, e.clientY));
      const btn = e.button;
      if (btn === 1 || (btn === 0 && e.spaceKey) || this.tool === "pan" ||
          (e.getModifierState && e.getModifierState("Space"))) {
        this._drag = { kind: "pan", x: e.clientX, y: e.clientY, tx: this.tx, ty: this.ty };
        e.preventDefault();
        return;
      }
      if (btn === 2 || (btn === 0 && e.altKey)) {
        // Alt+左键或右键平移
        this._drag = { kind: "pan", x: e.clientX, y: e.clientY, tx: this.tx, ty: this.ty };
        return;
      }
      // 命中已有元素？（家具模式下姿态始终可拖；其余模式按工具）
      const target = e.target;
      const hit = target.closest ? target.closest("[data-hit]") : null;
      if (this.tool === "delete" && hit) {
        this.cb.onDelete && this.cb.onDelete(hit.getAttribute("data-hit"),
                                            hit.getAttribute("data-id"));
        return;
      }
      if (hit) {
        const ht = hit.getAttribute("data-hit");
        if (this.tool === "select" || this.tool === "place" ||
            (this.tool === "pose" && ht === "pose")) {
          this._drag = {
            kind: "element",
            hit,
            id: hit.getAttribute("data-id"),
            kindType: ht,
            x: w.x, y: w.y,
            move: hit.getAttribute("data-move"),
            startSX: e.clientX, startSY: e.clientY,
          };
          this.cb.onPick && this.cb.onPick(ht, this._drag.id, hit);
          return;
        }
      }
      if (btn === 0) {
        // 落位模式：单击空白处即放置，交给 click 事件处理
        if (this.tool === "place") { this.cb.onPlaceClick && this.cb.onPlaceClick(w); return; }
        // 绘图工具
        if (this.tool === "room" || this.tool === "obstacle") {
          this._draw = {
            tool: this.tool, x0: w.x, y0: w.y,
            rect: el("rect", { class: "sketch-rect", fill: "rgba(37,99,235,.08)",
                               stroke: "#2563eb", "stroke-dasharray": "6 4" }),
          };
          this.view.appendChild(this._draw.rect);
        } else if (this.tool === "door") {
          this._draw = {
            tool: "door", x0: w.x, y0: w.y,
            line: el("line", { stroke: "#0ea5e9", "stroke-width": 6,
                               "stroke-dasharray": "8 5" }),
          };
          this.view.appendChild(this._draw.line);
        }
        this.cb.onDrawStart && this.cb.onDrawStart(this.tool, w);
      }
    }

    _move(e) {
      const w = this.snapPt(this.toWorld(e.clientX, e.clientY));
      this.cb.onCoord && this.cb.onCoord(w);
      if (this._drag && this._drag.kind === "pan") {
        this.tx = this._drag.tx + (e.clientX - this._drag.x);
        this.ty = this._drag.ty + (e.clientY - this._drag.y);
        this.applyTransform();
        this.cb.onViewport && this.cb.onViewport();
        return;
      }
      if (this._drag && this._drag.kind === "element") {
        const dx = w.x - this._drag.x, dy = w.y - this._drag.y;
        if (!this._drag.moved) {
          const pdx = e.clientX - this._drag.startSX;
          const pdy = e.clientY - this._drag.startSY;
          if (Math.hypot(pdx, pdy) < 3) return;
          this._drag.moved = true;
        }
        this.cb.onDrag && this.cb.onDrag(this._drag.kindType, this._drag.id,
                                         dx, dy, e, w, this._drag);
        this.cb.onElementDragged && this.cb.onElementDragged();
        this._drag.x = w.x; this._drag.y = w.y;
        return;
      }
      if (this._draw) {
        const d = this._draw;
        if (d.tool === "room" || d.tool === "obstacle") {
          const x = Math.min(d.x0, w.x), y = Math.min(d.y0, w.y);
          d.rect.setAttribute("x", x); d.rect.setAttribute("y", y);
          d.rect.setAttribute("width", Math.abs(w.x - d.x0));
          d.rect.setAttribute("height", Math.abs(w.y - d.y0));
        } else if (d.tool === "door") {
          d.line.setAttribute("x1", d.x0); d.line.setAttribute("y1", d.y0);
          d.line.setAttribute("x2", w.x); d.line.setAttribute("y2", w.y);
        }
        this.cb.onDrawMove && this.cb.onDrawMove(d.tool, d.x0, d.y0, w);
      }
    }

    _up(e) {
      const w = this.snapPt(this.toWorld(e.clientX, e.clientY));
      if (this._drag) { this._drag = null; }
      if (this._draw) {
        const d = this._draw;
        d.rect && d.rect.remove();
        d.line && d.line.remove();
        const w2 = (w.x - d.x0) ** 2 + (w.y - d.y0) ** 2;
        if (w2 > 2500) {  // 至少 50mm
          this.cb.onDrawEnd && this.cb.onDrawEnd(d.tool,
            { x: d.x0, y: d.y0 }, { x: w.x, y: w.y });
        }
        this._draw = null;
      }
    }
  }

  global.Editor = Editor;
  global.svgEl = el;
  global.svgClear = clear;
  global.ptsStr = ptsStr;
  global.svgRound = round;
})(window);
