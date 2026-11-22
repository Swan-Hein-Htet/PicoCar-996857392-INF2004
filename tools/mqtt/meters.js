/* Tilt, hump and obstacle meters for the PicoCar pages (car_ui.html, manual.html).
   Fed from telemetry v2: pitch_x10 (0.1 deg, + = nose up), roll_deg,
   hump_now_mm (height above flat right now), plus the last measured hump
   from the car's log.  Uses the pages' colour variables. */
(function () {
  "use strict";
  const NS = "http://www.w3.org/2000/svg";
  const PITCH_MAX = 30, HUMP_MAX = 50, OBST_MAX = 400, CAR_W = 150;
  const PLANS = ["", "passing left", "passing right", "backing off"];

  const css = `
.meters { display: grid; grid-template-columns: repeat(auto-fit, minmax(110px, 1fr)); gap: 14px; align-items: end; }
.meter { text-align: center; min-width: 0; }
.meter svg { width: 100%; height: auto; max-height: 132px; display: block; overflow: visible; }
.meter svg text { font-family: var(--thin, system-ui); font-size: 9px; fill: var(--fg-3); letter-spacing: 0.08em; }
.meter .val { font-weight: 300; font-size: 26px; line-height: 1; margin-top: 4px; font-variant-numeric: tabular-nums; }
.meter .val small { font-size: 10px; font-weight: 600; letter-spacing: 0.2em; color: var(--fg-3); margin-left: 3px; }
.meter .val.live { color: var(--amber); }
.meter .cap { font-size: 10.5px; font-weight: 600; letter-spacing: 0.22em; text-transform: uppercase; color: var(--fg-3); margin-top: 5px; }
.meter .sub { font-size: 11.5px; color: var(--fg-2); margin-top: 3px; }
@media (max-width: 520px) { .meter.hump svg, .meter.obst svg { max-height: 110px; } }`;
  if (!document.getElementById("meters-css")) {
    const st = document.createElement("style"); st.id = "meters-css"; st.textContent = css; document.head.append(st);
  }

  function s(tag, attrs, parent) {
    const e = document.createElementNS(NS, tag);
    for (const [k, v] of Object.entries(attrs || {})) e.setAttribute(k, v);
    if (parent) parent.append(e);
    return e;
  }
  function h(tag, cls, text, parent) {
    const e = document.createElement(tag);
    if (cls) e.className = cls;
    if (text != null) e.textContent = text;
    if (parent) parent.append(e);
    return e;
  }
  function polar(cx, cy, r, deg) {                 // 0 deg = straight up
    const a = (deg - 90) * Math.PI / 180;
    return [cx + r * Math.cos(a), cy + r * Math.sin(a)];
  }
  function arcPath(cx, cy, r, from, to) {
    const [x0, y0] = polar(cx, cy, r, from), [x1, y1] = polar(cx, cy, r, to);
    return `M ${x0.toFixed(1)} ${y0.toFixed(1)} A ${r} ${r} 0 0 ${to > from ? 1 : 0} ${x1.toFixed(1)} ${y1.toFixed(1)}`;
  }

  /* An angle instrument: a +-30 deg scale over a car drawn by drawCar,
     which the needle angle rotates. */
  function angleMeter(parent, caption, drawCar) {
    const box = h("div", "meter", null, parent);
    const svg = s("svg", { viewBox: "0 0 180 104", role: "img", "aria-label": caption }, box);
    const cx = 90, cy = 78, r = 62;
    s("path", { d: arcPath(cx, cy, r, -PITCH_MAX, PITCH_MAX), fill: "none", stroke: "var(--hair-2)", "stroke-width": 1.5 }, svg);
    for (let d = -PITCH_MAX; d <= PITCH_MAX; d += 5) {
      const major = d % 15 === 0, len = major ? 4 : 2;
      const [x0, y0] = polar(cx, cy, r - len, d), [x1, y1] = polar(cx, cy, r + len, d);
      s("line", { x1: x0, y1: y0, x2: x1, y2: y1, stroke: d === 0 ? "var(--fg-2)" : "var(--hair-2)", "stroke-width": d === 0 ? 1.5 : 1 }, svg);
      if (major && d !== 0) { const [tx, ty] = polar(cx, cy, r + 13, d); const t = s("text", { x: tx, y: ty + 3, "text-anchor": "middle" }, svg); t.textContent = Math.abs(d); }
    }
    const live = s("path", { d: "", fill: "none", stroke: "var(--amber)", "stroke-width": 3, "stroke-linecap": "round" }, svg);
    s("line", { x1: 18, x2: 162, y1: cy + 8, y2: cy + 8, stroke: "var(--hair-2)", "stroke-dasharray": "3 4" }, svg);
    const car = s("g", { transform: `translate(${cx} ${cy}) rotate(0)` }, svg);
    drawCar(car);
    const needle = s("line", { x1: cx, y1: cy, x2: cx, y2: cy - r + 8, stroke: "var(--amber)", "stroke-width": 1.5, opacity: 0.8 }, svg);
    const val = h("div", "val", "--", box);
    h("div", "cap", caption, box);
    const sub = h("div", "sub", "", box);
    return function (deg, words) {
      if (deg == null) { val.textContent = "--"; sub.textContent = ""; live.setAttribute("d", ""); return; }
      const shown = Math.max(-PITCH_MAX, Math.min(PITCH_MAX, deg));
      car.setAttribute("transform", `translate(${cx} ${cy}) rotate(${-shown})`);
      needle.setAttribute("transform", `rotate(${-shown} ${cx} ${cy})`);
      live.setAttribute("d", Math.abs(shown) < 0.3 ? "" : arcPath(cx, cy, r, 0, -shown));
      val.textContent = "";
      val.append((deg > 0 ? "+" : "") + deg.toFixed(1));
      const u = document.createElement("small"); u.textContent = "DEG"; val.append(u);
      val.classList.toggle("live", Math.abs(deg) >= 3);
      sub.textContent = Math.abs(deg) < 1 ? "level" : words(deg);
    };
  }

  window.PicoMeters = function (host) {
    const wrap = h("div", "meters", null, host);
    /* Pitch: the car from the side, nose to the right. */
    const pitch = angleMeter(wrap, "Pitch", (g) => {
      s("rect", { x: -30, y: -14, width: 60, height: 10, rx: 2, fill: "none", stroke: "var(--fg)", "stroke-width": 1.5 }, g);
      s("path", { d: "M 30 -9 L 38 -9", stroke: "var(--fg)", "stroke-width": 1.5 }, g);   // nose
      s("circle", { cx: -18, cy: 0, r: 5, fill: "none", stroke: "var(--fg)", "stroke-width": 1.5 }, g);
      s("circle", { cx: 18, cy: 0, r: 5, fill: "none", stroke: "var(--fg)", "stroke-width": 1.5 }, g);
    });
    /* Roll: the car from behind. */
    const roll = angleMeter(wrap, "Roll", (g) => {
      s("rect", { x: -20, y: -18, width: 40, height: 14, rx: 3, fill: "none", stroke: "var(--fg)", "stroke-width": 1.5 }, g);
      s("rect", { x: -27, y: -8, width: 7, height: 12, rx: 2, fill: "var(--fg)" }, g);
      s("rect", { x: 20, y: -8, width: 7, height: 12, rx: 2, fill: "var(--fg)" }, g);
    });
    /* Hump: height above flat now, 0-50 mm, with the last hump marked. */
    const hb = h("div", "meter hump", null, wrap);
    const svg = s("svg", { viewBox: "0 0 120 104", role: "img", "aria-label": "Hump height" }, hb);
    const y = (mm) => 96 - (Math.min(HUMP_MAX, Math.max(0, mm)) / HUMP_MAX) * 88;
    s("rect", { x: 52, y: 8, width: 16, height: 88, fill: "none", stroke: "var(--hair-2)" }, svg);
    for (let mm = 0; mm <= HUMP_MAX; mm += 10) {
      s("line", { x1: 46, x2: 52, y1: y(mm), y2: y(mm), stroke: "var(--hair-2)" }, svg);
      const t = s("text", { x: 42, y: y(mm) + 3, "text-anchor": "end" }, svg); t.textContent = mm;
    }
    const fill = s("rect", { x: 53, width: 14, y: 96, height: 0, fill: "var(--amber)" }, svg);
    const mark = s("line", { x1: 48, x2: 74, y1: 96, y2: 96, stroke: "var(--fg)", "stroke-width": 2, visibility: "hidden" }, svg);
    const markT = s("text", { x: 77, y: 99 }, svg);
    const hval = h("div", "val", "--", hb);
    h("div", "cap", "Hump height", hb);
    const hsub = h("div", "sub", "", hb);

    /* Obstacle: top view, the measured width against the car's, both to
       scale (0-400 mm across). */
    const ob = h("div", "meter obst", null, wrap);
    const osv = s("svg", { viewBox: "0 0 120 104", role: "img", "aria-label": "Obstacle width" }, ob);
    const ox = (mm) => (mm / OBST_MAX) * 112;
    s("line", { x1: 60, x2: 60, y1: 13, y2: 100, stroke: "var(--hair-2)", "stroke-width": 3 }, osv);
    const box = s("rect", { x: 60, y: 14, width: 0, height: 24, fill: "var(--amber)", opacity: 0.85, visibility: "hidden" }, osv);
    const gap = s("line", { x1: 60, x2: 60, y1: 46, y2: 64, stroke: "var(--fg-3)", "stroke-dasharray": "2 3", visibility: "hidden" }, osv);
    s("rect", { x: 60 - ox(CAR_W) / 2, y: 70, width: ox(CAR_W), height: 26, rx: 3, fill: "none", stroke: "var(--fg)", "stroke-width": 1.5 }, osv);
    const ovt = s("text", { x: 60, y: 9, "text-anchor": "middle" }, osv);
    const oval = h("div", "val", "--", ob);
    h("div", "cap", "Obstacle width", ob);
    const osub = h("div", "sub", "", ob);

    return {
      update(rec, last, max) {
        const ok = !!rec && rec.version >= 2;
        pitch(ok ? rec.pitch_x10 / 10 : null, (d) => d > 0 ? "nose up" : "nose down");
        roll(ok ? rec.roll_deg : null, (d) => d > 0 ? "left side up" : "right side up");
        const now = ok ? rec.hump_now_mm : 0;
        const hNow = Math.max(0, Math.min(HUMP_MAX, now));
        fill.setAttribute("y", y(hNow)); fill.setAttribute("height", 96 - y(hNow));
        if (last != null) {
          mark.setAttribute("y1", y(last)); mark.setAttribute("y2", y(last)); mark.setAttribute("visibility", "visible");
          markT.setAttribute("y", y(last) + 3); markT.textContent = "last";
        } else { mark.setAttribute("visibility", "hidden"); markT.textContent = ""; }
        hval.textContent = "";
        const shown = now > 0 ? now : (last != null ? last : null);
        hval.append(shown == null ? "--" : String(shown));
        const u = document.createElement("small"); u.textContent = "MM"; hval.append(u);
        hval.classList.toggle("live", now > 0);
        hsub.textContent = !ok ? "needs the new firmware" : now > 0 ? "on a hump now" :
          last != null ? `last hump${max ? `, highest ${max} mm` : ""}` : "no hump yet";

        const w = rec ? (rec.obstacle_width_mm || 0) : 0;
        const near = rec ? (rec.obstacle_closest_mm || 0) : 0;
        const ow = Math.min(OBST_MAX, w);
        box.setAttribute("x", 60 - ox(ow) / 2); box.setAttribute("width", ox(ow));
        box.setAttribute("visibility", w > 0 ? "visible" : "hidden");
        gap.setAttribute("visibility", w > 0 ? "visible" : "hidden");
        ovt.textContent = w > 0 && near > 0 ? `${near} mm ahead` : "";
        oval.textContent = "";
        oval.append(w > 0 ? String(w) : "--");
        const ou = document.createElement("small"); ou.textContent = "MM"; oval.append(ou);
        oval.classList.toggle("live", !!rec && rec.mission_state === 7);
        osub.textContent = !rec ? "" : w > 0 ? (PLANS[rec.bypass_plan] || "measured") :
          "no obstacle yet";
      },
    };
  };
})();
