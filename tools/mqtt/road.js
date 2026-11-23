/* Road view for the PicoCar dashboard: a chase camera behind the car, the
   line it is following drawn as the road, junctions, obstacles, humps and
   the planned path, from telemetry v2.

   Position is dead reckoning from the wheel encoders (enc_left/right, 16-bit
   edge counts), so the road behind the car is where the car really went;
   the road ahead is a projection along the current curvature.  The line
   under the car is offset by line_error.  Junctions, obstacles and humps are
   pinned to the world where the car met them.

   Usage:  const road = PicoRoad(container);  road.update(stateSnapshot);
           PicoRoad.demo(cb)  feeds made-up snapshots to cb (5 Hz); returns
           a stop function.  car_ui.html?demo=road uses it. */
(function () {
  "use strict";

  /* Car geometry (mm), from the firmware. */
  const WHEEL_DIA = 65, EDGES_PER_REV = 1056, TRACK = 130;
  const MM_PER_EDGE = Math.PI * WHEEL_DIA / EDGES_PER_REV;
  const IR_AHEAD = 70, US_AHEAD = 60, CAR_W = 150, CAR_FRONT = 70, CAR_REAR = -110;
  const LINE_SPAN = 20;            /* mm of line offset at full line_error */
  const ROAD_HALF = 125;           /* drawn road half-width               */
  const LINE_HALF = 9;             /* the 18 mm line                      */
  /* Camera: behind and above, looking ahead and down. */
  const CAM_BACK = 620, CAM_H = 430, CAM_PITCH = 21 * Math.PI / 180, NEAR = 25;
  const FOG_FROM = 900, FOG_TO = 2600;
  /* States (firmware MISSION_x). */
  const S = { INIT: 0, CAL: 1, READY: 2, FOLLOW: 3, BARCODE: 4, MANOEUVRE: 5, HUMP: 6, OBSTACLE: 7, SEARCH: 8, FINISHED: 9, FAULT: 10 };
  const CMD = { 1: "left", 2: "right", 3: "straight", 4: "uturn" };

  const css = `
.road { position: relative; width: 100%; height: 340px; overflow: hidden; border: 1px solid var(--hair, rgba(255,255,255,.16));
  background: radial-gradient(120% 90% at 50% 100%, #10141a 0%, #07090c 55%, #000 100%); }
.road canvas { position: absolute; inset: 0; width: 100%; height: 100%; display: block; }
/* HUD: six slots over the scene (the page may put its own widgets in them). */
.road .rv-layer { position: absolute; inset: 0; display: grid; grid-template-columns: minmax(0, 1fr) auto minmax(0, 1fr);
  grid-template-rows: auto minmax(0, 1fr) auto; grid-template-areas: "tl tc tr" "ml . mr" "bl bc br"; gap: 10px 16px; padding: 16px 18px;
  pointer-events: none; }
.road .rv-tl { grid-area: tl; justify-self: start; align-self: start; }
.road .rv-tc { grid-area: tc; justify-self: center; align-self: start; text-align: center; }
.road .rv-tr { grid-area: tr; justify-self: end; align-self: start; text-align: right; display: flex; flex-direction: column; align-items: flex-end; gap: 8px; }
.road .rv-ml { grid-area: ml; justify-self: start; align-self: center; min-height: 0; }
.road .rv-mr { grid-area: mr; justify-self: end; align-self: center; min-height: 0; }
.road .rv-bl { grid-area: bl; justify-self: start; align-self: end; }
.road .rv-bc { grid-area: bc; justify-self: center; align-self: end; text-align: center; display: flex; flex-direction: column; align-items: center; gap: 8px; min-width: 0; }
.road .rv-br { grid-area: br; justify-self: end; align-self: end; }
.road .rv-card { display: flex; align-items: center; gap: 14px; padding: 12px 16px 12px 12px;
  background: rgba(10,12,16,.72); border: 1px solid rgba(255,255,255,.12); backdrop-filter: blur(6px); -webkit-backdrop-filter: blur(6px);
  max-width: 380px; }
.road .rv-icon { width: 44px; height: 44px; flex: none; display: grid; place-items: center; background: rgba(255,255,255,.06); }
.road .rv-icon svg { width: 30px; height: 30px; stroke: currentColor; fill: none; stroke-width: 2.2; stroke-linecap: round; stroke-linejoin: round; }
.road .rv-main { font-size: 17px; font-weight: 500; letter-spacing: .02em; line-height: 1.2; }
.road .rv-sub { font-size: 11px; font-weight: 600; letter-spacing: .2em; text-transform: uppercase; color: var(--fg-3, rgba(255,255,255,.48)); margin-top: 4px; }
.road .rv-card.warn { border-color: rgba(255,176,32,.55); } .road .rv-card.warn .rv-icon { color: var(--amber, #FFB020); }
.road .rv-card.bad { border-color: rgba(255,59,48,.6); } .road .rv-card.bad .rv-icon { color: var(--red, #FF3B30); }
.road .rv-card.go .rv-icon { color: #4DA3FF; }
.road .rv-speed b { display: block; font-weight: 200; font-size: 54px; line-height: 1; font-variant-numeric: tabular-nums; }
.road .rv-speed small { font-size: 10px; font-weight: 600; letter-spacing: .24em; color: var(--fg-3, rgba(255,255,255,.48)); }
.road .rv-gear { display: flex; gap: 10px; justify-content: flex-end; font-size: 12px; font-weight: 600; letter-spacing: .1em; color: rgba(255,255,255,.3); }
.road .rv-gear .on { color: #fff; }
.road .rv-tr .rv-chips { flex-direction: column; align-items: flex-end; }
.road .rv-chips { display: flex; justify-content: center; gap: 6px; flex-wrap: wrap; }
.road .rv-chip { font-size: 10px; font-weight: 600; letter-spacing: .18em; text-transform: uppercase; padding: 4px 9px; color: var(--fg-2, rgba(255,255,255,.72));
  background: rgba(10,12,16,.7); border: 1px solid rgba(255,255,255,.12); white-space: nowrap; }
.road .rv-chip.warn { color: var(--amber, #FFB020); border-color: rgba(255,176,32,.45); }
.road .rv-chip.bad { color: var(--red, #FF3B30); border-color: rgba(255,59,48,.5); }
.road .rv-empty[hidden] { display: none; }
.road .rv-empty { position: absolute; inset: 0; display: grid; place-items: center; color: var(--fg-3, rgba(255,255,255,.48)); font-size: 11px;
  font-weight: 600; letter-spacing: .24em; text-transform: uppercase; pointer-events: none; }
@media (max-width: 760px) {
  .road { height: 250px; }
  .road .rv-layer { padding: 10px; gap: 8px; }
  .road .rv-card { padding: 8px 12px 8px 8px; gap: 10px; }
  .road .rv-icon { width: 34px; height: 34px; } .road .rv-icon svg { width: 22px; height: 22px; }
  .road .rv-main { font-size: 14px; }
  .road .rv-speed b { font-size: 34px; }
}`;
  if (!document.getElementById("road-css")) {
    const st = document.createElement("style"); st.id = "road-css"; st.textContent = css; document.head.append(st);
  }

  const ICONS = {
    left: '<path d="M18 26V14a4 4 0 0 0-4-4H6"/><path d="M10 6 6 10l4 4"/>',
    right: '<path d="M10 26V14a4 4 0 0 1 4-4h8"/><path d="m18 6 4 4-4 4"/>',
    straight: '<path d="M14 26V5"/><path d="m9 10 5-5 5 5"/>',
    uturn: '<path d="M9 26V12a5 5 0 0 1 10 0v6"/><path d="m15 15 4 4 4-4"/>',
    obstacle: '<path d="M14 4 26 25H2Z"/><path d="M14 12v6"/><path d="M14 21.5v.5"/>',
    bypassL: '<path d="M18 26v-5c0-3-8-4-8-9V5"/><path d="m6 9 4-4 4 4"/><rect x="17" y="8" width="7" height="7"/>',
    bypassR: '<path d="M10 26v-5c0-3 8-4 8-9V5"/><path d="m14 9 4-4 4 4"/><rect x="4" y="8" width="7" height="7"/>',
    search: '<circle cx="12" cy="12" r="7"/><path d="m17 17 7 7"/>',
    hump: '<path d="M2 22h5c3 0 4-9 7-9s4 9 7 9h5"/>',
    barcode: '<path d="M5 6v16M9 6v16M12 6v16M17 6v16M20 6v16M23 6v16"/>',
    stop: '<rect x="6" y="6" width="16" height="16" rx="1"/>',
    flag: '<path d="M7 26V4"/><path d="M7 5h14l-3 5 3 5H7"/>',
    follow: '<path d="M14 26V5"/><path d="M8 26 11 5M20 26 17 5" opacity=".45"/>',
    cal: '<circle cx="14" cy="14" r="9"/><path d="M14 2v5M14 21v5M2 14h5M21 14h5"/>',
  };

  function el(tag, cls, parent) { const e = document.createElement(tag); if (cls) e.className = cls; if (parent) parent.append(e); return e; }
  const clamp = (v, a, b) => Math.max(a, Math.min(b, v));
  const wrap16 = (d) => ((d + 32768) & 0xffff) - 32768;
  const angDiff = (a, b) => { let d = a - b; while (d > Math.PI) d -= 2 * Math.PI; while (d < -Math.PI) d += 2 * Math.PI; return d; };

  /* opts.speed false: no speed number of its own (the page shows a dial);
     opts.horizon / opts.zoom move and scale the scene (fractions of height). */
  function PicoRoad(container, opts) {
    opts = opts || {};
    container.classList.add("road");
    const canvas = el("canvas", "", container);
    canvas.setAttribute("role", "img");
    canvas.setAttribute("aria-label", "Road view: the car, the line ahead, junctions and obstacles");
    const ctx = canvas.getContext("2d");
    const empty = el("div", "rv-empty", container); empty.textContent = "Waiting for the car";
    const layer = el("div", "rv-layer", container);
    const slots = {};
    ["tl", "tc", "tr", "ml", "mr", "bl", "bc", "br"].forEach((k) => { slots[k] = el("div", "rv-" + k, layer); });
    const card = el("div", "rv-card", slots.tl);
    card.setAttribute("role", "status");
    const icon = el("div", "rv-icon", card);
    const txt = el("div", "", card);
    const main = el("div", "rv-main", txt);
    const sub = el("div", "rv-sub", txt);
    const speedBox = el("div", "rv-speed", slots.tr);
    const speedV = el("b", "", speedBox); speedV.textContent = "0";
    el("small", "", speedBox).textContent = "MM/S";
    if (opts.speed === false) speedBox.hidden = true;
    const gear = el("div", "rv-gear", slots.tr);
    gear.setAttribute("aria-label", "Gear");
    const gears = {}; ["P", "R", "N", "D"].forEach((g) => { gears[g] = el("span", "", gear); gears[g].textContent = g; });
    const chips = el("div", "rv-chips", slots.tr);

    const reduce = window.matchMedia && window.matchMedia("(prefers-reduced-motion: reduce)").matches;

    /* ---- world model ---- */
    let W = null;
    function resetWorld() {
      W = {
        pose: { x: 0, y: 0, h: 0 },          /* from the encoders       */
        view: { x: 0, y: 0, h: 0 },          /* smoothed, drawn         */
        cam: { h: 0 },
        enc: null, trail: [], junctions: [], obstacles: [], humps: [],
        lineOff: 0, curv: 0, onLine: false, rec: null, s: null,
        state: -1, lstate: -1, pending: null, reads: null, turning: null, curObs: null,
        obsWidthAtEntry: 0, blink: 0, sonar: null,
      };
    }
    resetWorld();

    function addTrail(force) {
      const p = W.pose, last = W.trail[W.trail.length - 1];
      if (!force && last && Math.hypot(p.x - last.x, p.y - last.y) < 12) return;
      const nx = -Math.sin(p.h), ny = Math.cos(p.h);
      W.trail.push({ x: p.x, y: p.y, lx: p.x + nx * W.lineOff, ly: p.y + ny * W.lineOff });
      if (W.trail.length > 700) W.trail.splice(0, 100);
    }
    function ahead(mm, lat) {            /* world point from the current pose */
      const p = W.pose, c = Math.cos(p.h), s = Math.sin(p.h);
      return { x: p.x + c * mm - s * (lat || 0), y: p.y + s * mm + c * (lat || 0), h: p.h };
    }

    function update(snap) {
      W.s = snap;
      const r = snap && snap.rec;
      const stale = !snap || snap.age == null || snap.age > 3;
      empty.hidden = !!r;
      if (!r) { hud(null, true); return; }
      const state = r.mission_state;

      /* New calibration: a fresh world. */
      if ((state === S.CAL || state === S.INIT) && W.state !== S.CAL && W.state !== S.INIT && W.state !== -1) resetWorld();

      /* Dead reckoning. */
      if (W.enc) {
        const dl = wrap16(r.enc_left - W.enc.l), dr = wrap16(r.enc_right - W.enc.r);
        if (Math.abs(dl) < 6000 && Math.abs(dr) < 6000) {
          const ml = dl * MM_PER_EDGE, mr = dr * MM_PER_EDGE, d = (ml + mr) / 2, dh = (mr - ml) / TRACK;
          const hm = W.pose.h + dh / 2;
          W.pose.x += d * Math.cos(hm); W.pose.y += d * Math.sin(hm); W.pose.h += dh;
        }
      }
      W.enc = { l: r.enc_left, r: r.enc_right };

      /* Line under the car (+ = to the left), and curvature for the road ahead. */
      const tracking = r.line_state === 1 || r.line_state === 2;
      const target = tracking ? clamp(r.line_error, -127, 127) / 127 * LINE_SPAN : W.lineOff;
      W.lineOff += (target - W.lineOff) * 0.5;
      const v = (r.speed_left_mms + r.speed_right_mms) / 2;
      const k = v > 25 ? (r.speed_right_mms - r.speed_left_mms) / (TRACK * v) : 0;
      W.curv += (clamp(k, -1 / 220, 1 / 220) - W.curv) * 0.35;
      /* The road is where the car followed the line: not the detour round
         an obstacle, nor a search. */
      W.onLine = tracking && [S.READY, S.FOLLOW, S.BARCODE, S.MANOEUVRE, S.HUMP, S.FINISHED].includes(state);
      if (W.onLine) addTrail(false);

      /* Barcode read: the command waits for the next junction. */
      if (W.reads != null && snap.reads > W.reads && CMD[r.last_barcode]) W.pending = { cmd: CMD[r.last_barcode], letter: snap.letter };
      W.reads = snap.reads;

      /* Junction: pinned where the IR row crossed it. */
      if ((r.line_state === 2 && W.lstate !== 2) || (state === S.MANOEUVRE && W.state !== S.MANOEUVRE)) {
        const j = ahead(IR_AHEAD);
        if (!W.junctions.some((q) => Math.hypot(q.x - j.x, q.y - j.y) < 160)) W.junctions.push({ x: j.x, y: j.y, h: j.h, cmd: null });
      }
      if (state === S.MANOEUVRE && W.state !== S.MANOEUVRE) {
        W.turning = W.pending ? W.pending.cmd : "straight";
        const last = W.junctions[W.junctions.length - 1]; if (last) last.cmd = W.turning;
      }
      if (state !== S.MANOEUVRE && W.state === S.MANOEUVRE) { W.turning = null; W.pending = null; }
      if (state === S.MANOEUVRE && W.pending && W.pending.cmd === "uturn") W.turning = "uturn";

      /* Obstacle: pinned at the stop, measured by the scan. */
      W.sonar = snap.sonar != null && snap.sonar > 0 ? snap.sonar : null;
      if (state === S.OBSTACLE && W.state !== S.OBSTACLE) {
        const rng = r.obstacle_closest_mm > 0 && r.obstacle_closest_mm < 400 ? r.obstacle_closest_mm : (W.sonar && W.sonar < 400 ? W.sonar : 180);
        const o = { at: { ...W.pose }, range: rng, width: 120, depth: 120, measured: false, plan: 0, done: false };
        const c = ahead(US_AHEAD + rng + o.depth / 2); o.x = c.x; o.y = c.y; o.h = c.h;
        W.obstacles.push(o); W.curObs = o; W.obsWidthAtEntry = r.obstacle_width_mm;
        if (W.obstacles.length > 8) W.obstacles.shift();
      }
      const o = W.curObs;
      if (o && state === S.OBSTACLE) {
        /* The scan is done once T5 reaches PLAN (5) or later. */
        const scanned = r.scan_state >= 5 && r.scan_state <= 11 && r.obstacle_width_mm > 0;
        if (scanned && !o.measured) {
          o.measured = true; o.width = clamp(r.obstacle_width_mm, 40, 600); o.depth = clamp(o.width, 60, 250);
          o.plan = r.bypass_plan;
          const c = (() => { const a = o.at, cc = Math.cos(a.h), ss = Math.sin(a.h), b = (r.obstacle_bearing_x10deg * 10 - 90) * Math.PI / 180;
            const lat = clamp((US_AHEAD + o.range) * Math.sin(b), -200, 200), d = US_AHEAD + o.range + o.depth / 2;
            return { x: a.x + cc * d - ss * lat, y: a.y + ss * d + cc * lat }; })();
          o.x = c.x; o.y = c.y;
        }
        if (o.measured && (r.bypass_plan === 1 || r.bypass_plan === 2 || r.bypass_plan === 3)) o.plan = r.bypass_plan;
      }
      if (state !== S.OBSTACLE && W.state === S.OBSTACLE && o) { o.done = true; W.curObs = null; }

      /* Hump: pinned where the car climbed it. */
      if (state === S.HUMP && W.state !== S.HUMP) { const h = ahead(40); W.humps.push({ x: h.x, y: h.y, h: h.h, mm: 0 }); }
      if (W.humps.length && state === S.HUMP) { const h = W.humps[W.humps.length - 1]; h.mm = Math.max(h.mm, r.hump_now_mm || 0, snap.humpLast || 0); }

      W.state = state; W.lstate = r.line_state; W.rec = r;
      hud(r, stale);
      /* Animation frames can be held back (a background tab, a hidden
         pane): draw from the data instead so the view never sits empty. */
      if (performance.now() - lastDraw > 400) draw(performance.now());
    }

    /* ---- HUD text ---- */
    const WORD = { left: "Turn left", right: "Turn right", straight: "Straight on", uturn: "U-turn" };
    const ING = { left: "Turning left", right: "Turning right", straight: "Going straight on", uturn: "Making a U-turn" };
    let hudKey = "";
    function hud(r, stale) {
      let ic = "cal", m = "Awaiting vehicle", sb = "Power the car", tone = "";
      const chipList = [];
      if (r) {
        const st = r.mission_state, p = W.pending, o = W.curObs;
        ic = "follow"; m = "Following the line"; sb = "Lane centred";
        if (st === S.INIT) { ic = "cal"; m = "Starting up"; sb = "Vehicle booting"; }
        else if (st === S.CAL) { ic = "cal"; m = "Calibrating"; sb = "Sweeping the line sensors"; }
        else if (st === S.READY) { ic = "flag"; m = "Ready"; sb = "Press Go to start"; }
        else if (st === S.FINISHED) { ic = "flag"; m = "Course complete"; sb = "End marker reached"; tone = "go"; }
        else if (st === S.FAULT) { ic = "stop"; m = "Stopped"; sb = "Safe state: reset to recover"; tone = "bad"; }
        else if (st === S.SEARCH) { ic = "search"; m = "Searching for the line"; sb = "Line lost"; tone = "warn"; }
        else if (st === S.HUMP) { ic = "hump"; m = "Crossing a hump"; sb = (r.hump_now_mm || 0) + " mm high"; tone = "warn"; }
        else if (st === S.BARCODE) { ic = "barcode"; m = "Reading a barcode"; sb = "Holding speed"; }
        else if (st === S.MANOEUVRE) { const t = W.turning || "straight"; ic = t; m = ING[t] + (t === "uturn" ? "" : " at the junction"); sb = p && p.letter ? "Barcode " + p.letter : "Junction"; tone = "go"; }
        else if (st === S.OBSTACLE) {
          tone = "warn";
          if (o && (o.plan === 1 || o.plan === 2) && o.measured) { ic = o.plan === 1 ? "bypassL" : "bypassR"; m = "Going round on the " + (o.plan === 1 ? "left" : "right"); sb = `Obstacle ${o.width} mm wide`; }
          else if (r.bypass_plan === 3) { ic = "obstacle"; m = "No way round"; sb = "Backing off to look again"; tone = "bad"; }
          else { ic = "obstacle"; m = "Obstacle ahead"; sb = "Measuring its width"; }
        }
        else if (p) { ic = p.cmd; m = WORD[p.cmd] + (p.cmd === "uturn" ? "" : " at the next junction"); sb = p.letter ? "Barcode " + p.letter : "Barcode command"; tone = "go"; }
        else if (r.line_state === 2) { ic = "straight"; m = "Junction"; sb = "No command: straight on"; }
        if (stale) { sb = "Telemetry lost, last known"; tone = "bad"; }

        const sp = (Math.abs(r.speed_left_mms) + Math.abs(r.speed_right_mms)) / 2;
        speedV.textContent = String(Math.round(sp));
        const g = st === S.FAULT || st === S.FINISHED || st === S.READY || st <= S.CAL ? "P" :
          r.motion_state === 3 ? "R" : sp < 5 ? "N" : "D";
        Object.entries(gears).forEach(([k, e]) => e.classList.toggle("on", k === g));

        if (W.sonar != null && W.sonar < 1500 && st !== S.READY) chipList.push([`Sonar ${(W.sonar / 10).toFixed(0)} cm`, W.sonar < 250 ? "warn" : ""]);
        chipList.push(["Line " + (["calibrating", "tracking", "junction", "lost"][r.line_state] || "?"), r.line_state === 3 ? "warn" : ""]);
        if (r.line_state === 1) chipList.push([`Offset ${r.line_error > 0 ? "+" : ""}${r.line_error}`, ""]);
        if (r.hump_max_mm > 0) chipList.push([`Hump max ${r.hump_max_mm} mm`, ""]);
      } else {
        speedV.textContent = "0";
        Object.entries(gears).forEach(([k, e]) => e.classList.toggle("on", k === "P"));
      }
      const key = [ic, m, sb, tone, chipList.map((c) => c.join(":")).join("|")].join("/");
      if (key === hudKey) return;
      hudKey = key;
      card.className = "rv-card" + (tone ? " " + tone : "");
      icon.innerHTML = `<svg viewBox="0 0 28 28" aria-hidden="true">${ICONS[ic] || ICONS.follow}</svg>`;
      main.textContent = m; sub.textContent = sb;
      chips.textContent = "";
      chipList.forEach(([t, c]) => { const e = el("span", "rv-chip" + (c ? " " + c : ""), chips); e.textContent = t; });
    }

    /* ---- projection ---- */
    let CW = 0, CH = 0, F = 1, CX = 0, CY = 0, camX = 0, camY = 0, camC = 1, camS = 0;
    const cp = Math.cos(CAM_PITCH), sp = Math.sin(CAM_PITCH);
    function setCamera() {
      const v = W.view, h = W.cam.h;
      camC = Math.cos(h); camS = Math.sin(h);
      camX = v.x - camC * CAM_BACK; camY = v.y - camS * CAM_BACK;
    }
    /* World (x, y, z up) to screen; null behind the near plane. */
    function P(x, y, z) {
      const dx = x - camX, dy = y - camY;
      const fwd = dx * camC + dy * camS, lat = -dx * camS + dy * camC;
      const h = CAM_H - (z || 0);
      const zc = fwd * cp + h * sp, yc = h * cp - fwd * sp;
      if (zc < NEAR) return null;
      return { x: CX - F * lat / zc, y: CY + F * yc / zc, z: zc };
    }
    const fog = (z) => clamp(1 - (z - FOG_FROM) / (FOG_TO - FOG_FROM), 0, 1);

    function poly(pts, fill, stroke, lw) {
      if (pts.some((p) => !p)) return false;
      ctx.beginPath(); ctx.moveTo(pts[0].x, pts[0].y);
      for (let i = 1; i < pts.length; i++) ctx.lineTo(pts[i].x, pts[i].y);
      ctx.closePath();
      if (fill) { ctx.fillStyle = fill; ctx.fill(); }
      if (stroke) { ctx.strokeStyle = stroke; ctx.lineWidth = lw || 1; ctx.stroke(); }
      return true;
    }

    /* A ribbon along a world polyline: half-width w, colour by depth. */
    function ribbon(pts, w, colour, alpha, z) {
      for (let i = 0; i + 1 < pts.length; i++) {
        const a = pts[i], b = pts[i + 1];
        const dx = b.x - a.x, dy = b.y - a.y, L = Math.hypot(dx, dy);
        if (L < 0.5) continue;
        const nx = -dy / L * w, ny = dx / L * w;
        const q = [P(a.x + nx, a.y + ny, z), P(b.x + nx, b.y + ny, z), P(b.x - nx, b.y - ny, z), P(a.x - nx, a.y - ny, z)];
        if (q.some((p) => !p)) continue;
        const f = fog((q[0].z + q[1].z) / 2) * (typeof alpha === "function" ? alpha(i / (pts.length - 1)) : alpha);
        if (f <= 0.01) continue;
        ctx.globalAlpha = f;
        poly(q, colour, colour, 1);
      }
      ctx.globalAlpha = 1;
    }

    /* The road: driven trail plus the projection ahead. */
    function roadPoints() {
      const lane = W.trail.map((t) => ({ x: t.lx, y: t.ly }));
      const v = W.view, st = W.state;
      const nx = -Math.sin(v.h), ny = Math.cos(v.h);
      let here = { x: v.x + nx * W.lineOff, y: v.y + ny * W.lineOff }, hd = v.h;
      if (W.onLine || lane.length < 2) lane.push(here);
      else {
        /* Off the line: it carries on from where the car left it. */
        const a = lane[lane.length - 2], b = lane[lane.length - 1];
        here = b; hd = Math.atan2(b.y - a.y, b.x - a.x);
      }
      const fwd = [];
      const going = st === S.FOLLOW || st === S.BARCODE || st === S.HUMP || st === S.READY || st === S.MANOEUVRE || st === S.SEARCH || st === S.FINISHED;
      if (going || st === S.OBSTACLE || st === S.FAULT || st === -1) {
        let x = here.x, y = here.y, h = hd;
        const k = st === S.FOLLOW || st === S.BARCODE || st === S.HUMP ? W.curv : 0;
        for (let d = 0; d < 1800; d += 40) { fwd.push({ x, y }); h += k * 40; x += Math.cos(h) * 40; y += Math.sin(h) * 40; }
      }
      return { lane, fwd };
    }

    function drawGround() {
      /* Fine grid pinned to the world, so motion reads even on a straight. */
      const v = W.view, G = 100, R = 1600;
      const gx0 = Math.floor((v.x - R) / G) * G, gy0 = Math.floor((v.y - R) / G) * G;
      ctx.lineWidth = 1;
      for (let gx = gx0; gx <= v.x + R; gx += G) {
        for (let gy = gy0; gy <= v.y + R; gy += G) {
          const p = P(gx, gy, 0);
          if (!p) continue;
          const f = fog(p.z) * 0.5;
          if (f < 0.02) continue;
          const s = clamp(F * 2.2 / p.z, 0.6, 2.2);
          ctx.globalAlpha = f; ctx.fillStyle = "#5b6676";
          ctx.fillRect(p.x - s / 2, p.y - s / 2, s, s);
        }
      }
      ctx.globalAlpha = 1;
    }

    function drawJunction(j) {
      const c = Math.cos(j.h), s = Math.sin(j.h), L = 700;
      const a = { x: j.x - s * L, y: j.y + c * L }, b = { x: j.x + s * L, y: j.y - c * L };
      const seg = [a, { x: j.x, y: j.y }, b];
      ribbon(seg, ROAD_HALF, "#1a1f27", (t) => 1 - Math.abs(t - 0.5) * 1.2, 0);
      ribbon(seg, LINE_HALF, "#e9eef5", (t) => 0.75 - Math.abs(t - 0.5) * 1.1, 0.5);
      /* stop bar */
      const q = [P(j.x - c * 30 + -s * ROAD_HALF, j.y - s * 30 + c * ROAD_HALF, 0.6), P(j.x - c * 22 + -s * ROAD_HALF, j.y - s * 22 + c * ROAD_HALF, 0.6),
        P(j.x - c * 22 - -s * ROAD_HALF, j.y - s * 22 - c * ROAD_HALF, 0.6), P(j.x - c * 30 - -s * ROAD_HALF, j.y - s * 30 - c * ROAD_HALF, 0.6)];
      if (q.every(Boolean)) { ctx.globalAlpha = 0.35 * fog(q[0].z); poly(q, "#ffffff"); ctx.globalAlpha = 1; }
    }

    function drawHump(hm) {
      const c = Math.cos(hm.h), s = Math.sin(hm.h);
      for (let i = 0; i < 4; i++) {
        const d = i * 34 - 50;
        const q = [P(hm.x + c * d - s * ROAD_HALF, hm.y + s * d + c * ROAD_HALF, 0.4), P(hm.x + c * (d + 16) - s * ROAD_HALF, hm.y + s * (d + 16) + c * ROAD_HALF, 0.4),
          P(hm.x + c * (d + 16) + s * ROAD_HALF, hm.y + s * (d + 16) - c * ROAD_HALF, 0.4), P(hm.x + c * d + s * ROAD_HALF, hm.y + s * d - c * ROAD_HALF, 0.4)];
        if (!q.every(Boolean)) continue;
        ctx.globalAlpha = 0.55 * fog(q[0].z); poly(q, "#FFB020");
      }
      ctx.globalAlpha = 1;
      const p = P(hm.x + Math.cos(hm.h) * 90, hm.y + Math.sin(hm.h) * 90, 30);
      if (p && hm.mm > 0) label(p, `HUMP ${hm.mm} MM`, "#FFB020", fog(p.z));
    }

    function label(p, text, colour, alpha) {
      ctx.globalAlpha = alpha;
      ctx.font = "600 10.5px 'Avenir Next','Helvetica Neue','Segoe UI',system-ui,sans-serif";
      const w = ctx.measureText(text).width + 16;
      ctx.fillStyle = "rgba(8,10,14,0.78)"; ctx.fillRect(p.x - w / 2, p.y - 22, w, 18);
      ctx.strokeStyle = colour; ctx.lineWidth = 1; ctx.strokeRect(p.x - w / 2 + 0.5, p.y - 21.5, w - 1, 17);
      ctx.fillStyle = colour; ctx.textAlign = "center"; ctx.textBaseline = "middle";
      ctx.fillText(text, p.x, p.y - 13);
      ctx.globalAlpha = 1;
    }

    /* A box in the world (centre, heading, half sizes, height), visible faces only. */
    function box(cx, cy, h, hl, hw, z0, z1, faces) {
      const c = Math.cos(h), s = Math.sin(h);
      const corner = (u, v, z) => ({ x: cx + c * u - s * v, y: cy + s * u + c * v, z });
      const V = [[-hl, -hw], [hl, -hw], [hl, hw], [-hl, hw]];
      const bot = V.map(([u, v]) => corner(u, v, z0)), top = V.map(([u, v]) => corner(u, v, z1));
      const eye = { x: camX, y: camY, z: CAM_H };
      const list = [];
      for (let i = 0; i < 4; i++) {
        const a = bot[i], b = bot[(i + 1) % 4];
        const mx = (a.x + b.x) / 2, my = (a.y + b.y) / 2;
        const nx = mx - cx, ny = my - cy;                       /* outward */
        if (nx * (eye.x - mx) + ny * (eye.y - my) <= 0) continue;
        const kind = i === 0 ? "right" : i === 1 ? "front" : i === 2 ? "left" : "rear";
        list.push({ kind, pts: [a, b, top[(i + 1) % 4], top[i]], d: Math.hypot(eye.x - mx, eye.y - my) });
      }
      list.sort((p, q) => q.d - p.d);
      list.forEach((f) => { const q = f.pts.map((p) => P(p.x, p.y, p.z)); poly(q, faces[f.kind] || faces.side, faces.edge, 1); });
      if (eye.z > z1) { const q = top.map((p) => P(p.x, p.y, p.z)); poly(q, faces.top, faces.edge, 1); return q; }
      return null;
    }

    function drawObstacle(o, now) {
      const active = o === W.curObs;
      const near = active && W.state === S.OBSTACLE;
      const p0 = P(o.x, o.y, 0);
      if (!p0) return;
      const f = fog(p0.z) * (o.done ? 0.65 : 1);
      if (f < 0.02) return;
      ctx.globalAlpha = f;
      /* soft shadow */
      const sh = [P(o.x - 80, o.y - 80, 0), P(o.x + 80, o.y - 80, 0), P(o.x + 80, o.y + 80, 0), P(o.x - 80, o.y + 80, 0)];
      const tint = near ? "255,82,72" : "150,160,175";
      box(o.x, o.y, o.h, o.depth / 2, o.width / 2, 0, 100, {
        front: `rgba(${tint},0.30)`, side: `rgba(${tint},0.22)`, rear: `rgba(${tint},0.22)`, left: `rgba(${tint},0.22)`, right: `rgba(${tint},0.22)`,
        top: `rgba(${tint},0.40)`, edge: near ? "rgba(255,110,100,0.95)" : "rgba(200,210,225,0.7)" });
      void sh;
      /* scanning shimmer */
      if (near && !o.measured) {
        const k = (now / 900) % 1;
        const a = o.at, b = P(a.x + Math.cos(a.h) * US_AHEAD, a.y + Math.sin(a.h) * US_AHEAD, 70), t = P(o.x, o.y, 60);
        if (b && t) {
          const g = ctx.createLinearGradient(b.x, b.y, t.x, t.y);
          g.addColorStop(0, "rgba(255,176,32,0)"); g.addColorStop(clamp(k, 0.01, 0.99), "rgba(255,176,32,0.55)"); g.addColorStop(1, "rgba(255,176,32,0)");
          ctx.strokeStyle = g; ctx.lineWidth = 2; ctx.beginPath(); ctx.moveTo(b.x, b.y); ctx.lineTo(t.x, t.y); ctx.stroke();
        }
      }
      ctx.globalAlpha = 1;
      const lp = P(o.x, o.y, 120);
      if (lp && !o.done) {
        const dist = W.sonar != null && near && W.sonar < 600 ? `${Math.round(W.sonar / 10)} CM · ` : "";
        label(lp, o.measured ? `${dist}${o.width} MM WIDE` : `${dist}MEASURING`, near ? "#FF6A60" : "#C8D2E1", f);
      }
    }

    /* Planned path (blue), like an autopilot's. */
    function plannedPath() {
      const v = W.view, st = W.state, o = W.curObs;
      const c = Math.cos(v.h), s = Math.sin(v.h);
      const pts = [];
      if (st === S.OBSTACLE && o && (o.plan === 1 || o.plan === 2)) {
        const a = o.at, ac = Math.cos(a.h), as = Math.sin(a.h), sg = o.plan === 1 ? 1 : -1;
        const sh = o.width / 2 + CAR_W / 2 + 60;
        const along = (u, l) => ({ x: a.x + ac * u - as * l * sg, y: a.y + as * u + ac * l * sg });
        const oa = US_AHEAD + o.range;
        const ctrl = [along(0, 0), along(oa * 0.45, sh * 0.55), along(oa, sh), along(oa + o.depth, sh), along(oa + o.depth + sh * 0.7, sh * 0.45), along(oa + o.depth + sh * 1.3, 0), along(oa + o.depth + sh * 1.3 + 300, 0)];
        for (let i = 0; i + 1 < ctrl.length; i++) for (let t = 0; t < 1; t += 0.2) {
          const p = ctrl[i], q = ctrl[i + 1]; pts.push({ x: p.x + (q.x - p.x) * t, y: p.y + (q.y - p.y) * t });
        }
        return smooth(pts);
      }
      if (st === S.MANOEUVRE || (W.pending && (st === S.FOLLOW || st === S.BARCODE))) {
        const j = st === S.MANOEUVRE ? W.junctions[W.junctions.length - 1] : null;
        const cmd = st === S.MANOEUVRE ? W.turning : W.pending.cmd;
        if (j && cmd && cmd !== "straight" && cmd !== "uturn") {
          const jc = Math.cos(j.h), js = Math.sin(j.h), sg = cmd === "left" ? 1 : -1;
          pts.push({ x: v.x, y: v.y });
          const toJ = Math.max(0, (j.x - v.x) * jc + (j.y - v.y) * js);
          for (let d = 0; d <= toJ; d += 20) pts.push({ x: v.x + c * d, y: v.y + s * d });
          for (let d = 20; d <= 600; d += 20) pts.push({ x: j.x - js * d * sg, y: j.y + jc * d * sg });
          return pts;
        }
      }
      if (st === S.FOLLOW || st === S.BARCODE || st === S.HUMP || st === S.MANOEUVRE) {
        let x = v.x, y = v.y, h = v.h;
        for (let d = 0; d <= 700; d += 25) { pts.push({ x, y }); const k = st === S.MANOEUVRE ? 0 : W.curv; h += k * 25; x += Math.cos(h) * 25; y += Math.sin(h) * 25; }
      }
      return pts;
    }
    function smooth(pts) {
      const out = pts.slice();
      for (let pass = 0; pass < 3; pass++) for (let i = 1; i + 1 < out.length; i++) out[i] = { x: (out[i - 1].x + out[i].x * 2 + out[i + 1].x) / 4, y: (out[i - 1].y + out[i].y * 2 + out[i + 1].y) / 4 };
      return out;
    }

    function drawCar(now) {
      const v = W.view, r = W.rec;
      const body = { top: "#eef1f5", rear: "#c3c8cf", front: "#d6dade", left: "#aeb4bc", right: "#aeb4bc", side: "#aeb4bc", edge: "rgba(0,0,0,0.25)" };
      const c = Math.cos(v.h), s = Math.sin(v.h);
      const at = (u, l) => ({ x: v.x + c * u - s * l, y: v.y + s * u + c * l });
      /* shadow */
      const sh = [at(CAR_REAR - 10, -88), at(CAR_FRONT + 15, -88), at(CAR_FRONT + 15, 88), at(CAR_REAR - 10, 88)].map((p) => P(p.x, p.y, 0));
      if (sh.every(Boolean)) {
        const g = ctx.createRadialGradient((sh[0].x + sh[2].x) / 2, (sh[0].y + sh[2].y) / 2, 4, (sh[0].x + sh[2].x) / 2, (sh[0].y + sh[2].y) / 2, Math.abs(sh[2].x - sh[0].x) * 0.7);
        g.addColorStop(0, "rgba(0,0,0,0.7)"); g.addColorStop(1, "rgba(0,0,0,0)");
        poly(sh, g);
      }
      /* wheels */
      [[-1], [1]].forEach(([side]) => {
        const w = at(0, side * 70);
        box(w.x, w.y, v.h, 32, 12, 0, 52, { side: "#0c0d10", front: "#16181c", rear: "#16181c", left: "#0c0d10", right: "#0c0d10", top: "#1b1d22", edge: "rgba(255,255,255,0.08)" });
      });
      /* body and cabin */
      const mid = (CAR_FRONT + CAR_REAR) / 2, bc = at(mid, 0);
      box(bc.x, bc.y, v.h, (CAR_FRONT - CAR_REAR) / 2, CAR_W / 2 - 6, 16, 46, body);
      const cc = at(mid - 18, 0);
      const cabTop = box(cc.x, cc.y, v.h, 48, 44, 46, 68, { top: "#20262f", rear: "#161b22", front: "#2a323d", left: "#1b2029", right: "#1b2029", side: "#1b2029", edge: "rgba(255,255,255,0.18)" });
      void cabTop;
      /* tail lights: brighter when stopping or stopped */
      const braking = r && (r.motion_state === 6 || ((Math.abs(r.speed_left_mms) + Math.abs(r.speed_right_mms)) < 10 && W.state >= S.FOLLOW && W.state <= S.SEARCH));
      const tl = (l0, l1) => [at(CAR_REAR + 0.5, l0), at(CAR_REAR + 0.5, l1)].flatMap((p) => [P(p.x, p.y, 34), P(p.x, p.y, 42)]);
      [[-64, -36], [36, 64]].forEach(([a, b]) => {
        const q = tl(a, b); if (!q.every(Boolean)) return;
        ctx.shadowColor = "#ff2a2a"; ctx.shadowBlur = braking ? 18 : 6;
        poly([q[0], q[2], q[3], q[1]], braking ? "#ff3b3b" : "#a3171a");
        ctx.shadowBlur = 0;
      });
      /* indicators: the side of the turn coming up */
      const turn = W.state === S.MANOEUVRE ? W.turning : W.pending ? W.pending.cmd : null;
      const sideOf = (r && r.motion_state === 4) ? "left" : (r && r.motion_state === 5) ? "right" : turn;
      if ((sideOf === "left" || sideOf === "right") && Math.floor(now / 380) % 2 === 0) {
        const l = sideOf === "left" ? 1 : -1;
        const q = [at(CAR_REAR + 0.6, l * 70), at(CAR_REAR + 0.6, l * 62)].flatMap((p) => [P(p.x, p.y, 24), P(p.x, p.y, 44)]);
        if (q.every(Boolean)) { ctx.shadowColor = "#FFB020"; ctx.shadowBlur = 16; poly([q[0], q[2], q[3], q[1]], "#FFB020"); ctx.shadowBlur = 0; }
      }
    }

    let lastDraw = 0;
    function draw(now) {
      const tNow = performance.now(), steps = clamp((tNow - (lastDraw || tNow)) / 16.7, 1, 60);
      lastDraw = tNow;
      const dpr = Math.min(window.devicePixelRatio || 1, 2);
      const w = container.clientWidth, h = container.clientHeight;
      if (w !== CW || h !== CH) { CW = w; CH = h; canvas.width = Math.round(w * dpr); canvas.height = Math.round(h * dpr); }
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      ctx.clearRect(0, 0, CW, CH);
      F = Math.min(CH, CW * 0.7) * (opts.zoom || 1.15); CX = CW / 2;
      CY = CH * (CW < 560 ? 0.58 : (opts.horizon || 0.40));     /* narrow: car mid-screen */

      /* Ease the drawn pose toward the reckoned one; the camera's heading
         lags a little so turns read as the car turning. */
      const k = reduce ? 1 : 1 - Math.pow(0.84, steps), kc = reduce ? 1 : 1 - Math.pow(0.93, steps);
      W.view.x += (W.pose.x - W.view.x) * k; W.view.y += (W.pose.y - W.view.y) * k;
      W.view.h += angDiff(W.pose.h, W.view.h) * k;
      W.cam.h += angDiff(W.view.h, W.cam.h) * kc;
      setCamera();

      /* horizon glow */
      const hz = CY - F * Math.tan(CAM_PITCH);
      const g = ctx.createLinearGradient(0, hz - 40, 0, hz + 120);
      g.addColorStop(0, "rgba(77,163,255,0)"); g.addColorStop(0.35, "rgba(77,163,255,0.06)"); g.addColorStop(1, "rgba(77,163,255,0)");
      ctx.fillStyle = g; ctx.fillRect(0, hz - 40, CW, 160);

      drawGround();
      W.junctions.forEach(drawJunction);
      const { lane, fwd } = roadPoints();
      const all = lane.concat(fwd.slice(1));
      ribbon(all, ROAD_HALF, "#1a1f27", 1, 0);
      /* road edges */
      const edge = (side) => all.map((p, i) => {
        const q = all[Math.min(i + 1, all.length - 1)], o = all[Math.max(i - 1, 0)];
        const dx = q.x - o.x, dy = q.y - o.y, L = Math.hypot(dx, dy) || 1;
        return { x: p.x - dy / L * ROAD_HALF * side, y: p.y + dx / L * ROAD_HALF * side };
      });
      ribbon(edge(1), 2, "#3a4452", 1, 0.2); ribbon(edge(-1), 2, "#3a4452", 1, 0.2);
      /* the line: solid where driven, dashed-fading ahead */
      ribbon(lane, LINE_HALF, "#f2f5f9", 0.95, 0.4);
      ribbon(fwd, LINE_HALF, "#f2f5f9", (t) => 0.8 * (1 - t), 0.4);
      W.humps.forEach(drawHump);

      /* planned path */
      const path = plannedPath();
      if (path.length > 1) {
        ctx.shadowColor = "#4DA3FF"; ctx.shadowBlur = 14;
        ribbon(path, 34, "#2f7fe0", (t) => 0.42 * (1 - t * 0.85), 0.8);
        ctx.shadowBlur = 0;
        ribbon(path, 6, "#8cc4ff", (t) => 0.8 * (1 - t), 1);
      }

      /* obstacles behind the car first, then the car, then those in front */
      const v = W.view, hc = Math.cos(W.cam.h), hs = Math.sin(W.cam.h);
      const depth = (o) => (o.x - v.x) * hc + (o.y - v.y) * hs;
      const obs = W.obstacles.slice().sort((a, b) => depth(b) - depth(a));
      obs.filter((o) => depth(o) >= 0).forEach((o) => drawObstacle(o, now));
      drawCar(now);
      obs.filter((o) => depth(o) < 0).forEach((o) => drawObstacle(o, now));

      /* vignette */
      const vg = ctx.createRadialGradient(CX, CH * 0.7, CH * 0.3, CX, CH * 0.6, Math.max(CW, CH) * 0.75);
      vg.addColorStop(0, "rgba(0,0,0,0)"); vg.addColorStop(1, "rgba(0,0,0,0.55)");
      ctx.fillStyle = vg; ctx.fillRect(0, 0, CW, CH);
    }
    function frame(now) { draw(now); raf = requestAnimationFrame(frame); }
    let raf = requestAnimationFrame(frame);
    document.addEventListener("visibilitychange", () => {
      cancelAnimationFrame(raf);
      if (!document.hidden) raf = requestAnimationFrame(frame);
    });
    hud(null, true);
    container.__road = () => W;      /* for debugging in the console */
    return { update, reset: resetWorld, slots };
  }

  /* ---- demo feed: a made-up run through every kind of event ---- */
  PicoRoad.demo = function (cb) {
    const E = 1 / MM_PER_EDGE;
    let encL = 0, encR = 0, dist = 0, reads = 0, letter = null, lastBc = 0, plan = 0, width = 0, closest = 0, hump = 0, humpMax = 0, t = 0;
    const steps = [];
    const seg = (o) => steps.push(o);
    const run = (state, ms, vl, vr, extra) => seg(Object.assign({ state, ms, vl, vr }, extra || {}));
    const turn = (state, deg, dir, extra) => { const mm = Math.PI * TRACK * deg / 360, ms = mm / 70 * 1000; run(state, ms, -dir * 70, dir * 70, Object.assign({ motion: dir > 0 ? 4 : 5 }, extra || {})); };
    const build = () => {
      steps.length = 0;
      run(1, 1500, 0, 0, { motion: 0, line: 0 });
      run(2, 1200, 0, 0, { motion: 0 });
      run(3, 3500, 100, 100, { motion: 1, wob: 1, sonar: 1500 });
      run(4, 1500, 100, 100, { motion: 1, onEnd: () => { reads++; letter = "A"; lastBc = 1; } });
      run(3, 4200, 102, 98, { motion: 1, wob: 1 });
      run(3, 300, 100, 100, { motion: 1, line: 2 });
      run(5, 400, 0, 0, { motion: 6, line: 2 });
      run(5, 2000, 40, 40, { motion: 2 });
      run(5, 400, 0, 0, { motion: 6 });
      turn(5, 90, 1);
      run(5, 400, 0, 0, { motion: 6 });
      run(3, 3500, 100, 100, { motion: 1, wob: 1, sonarFrom: 900, sonarTo: 190 });
      run(7, 600, 0, 0, { motion: 6, sonar: 185, scan: 1, onStart: () => { closest = 185; plan = 0; } });
      turn(7, 28, -1, { sonar: 600, scan: 2 }); turn(7, 28, 1, { sonar: 180, scan: 2 }); turn(7, 28, 1, { sonar: 600, scan: 2 }); turn(7, 28, -1, { sonar: 185, scan: 2 });
      run(7, 500, 0, 0, { motion: 6, scan: 4, onEnd: () => { width = 110; plan = 1; } });
      turn(7, 60, 1, { scan: 6 }); run(7, 2300, 100, 100, { motion: 2, scan: 6 }); turn(7, 60, -1, { scan: 7 }); run(7, 3000, 100, 100, { motion: 2, scan: 7 });
      turn(7, 60, -1, { scan: 8 }); run(7, 2300, 100, 100, { motion: 2, scan: 8 }); turn(7, 60, 1, { scan: 9 });
      run(3, 2500, 100, 100, { motion: 1, wob: 1, sonar: 1400 });
      run(6, 2200, 100, 100, { motion: 1, hump: true });
      run(3, 2500, 98, 102, { motion: 1, wob: 1 });
      run(4, 1500, 100, 100, { motion: 1, onEnd: () => { reads++; letter = "B"; lastBc = 2; } });
      run(3, 3000, 100, 100, { motion: 1, wob: 1 });
      run(3, 300, 100, 100, { motion: 1, line: 2 });
      run(5, 400, 0, 0, { motion: 6, line: 2 }); run(5, 2000, 40, 40, { motion: 2 }); run(5, 400, 0, 0, { motion: 6 });
      turn(5, 90, -1); run(5, 400, 0, 0, { motion: 6 });
      run(3, 3000, 100, 100, { motion: 1, wob: 1 });
      run(9, 4000, 0, 0, { motion: 0 });
    };
    build();
    let i = 0, inStep = 0;
    const tick = 200;
    const id = setInterval(() => {
      let st = steps[i];
      if (inStep === 0 && st.onStart) st.onStart();
      inStep += tick; t += tick;
      const dt = tick / 1000;
      encL = (encL + Math.round(st.vl * dt * E)) & 0xffff; encR = (encR + Math.round(st.vr * dt * E)) & 0xffff;
      dist += Math.abs(st.vl + st.vr) / 2 * dt;
      const frac = Math.min(1, inStep / st.ms);
      let err = st.wob ? Math.round(30 * Math.sin(t / 700)) : 0;
      if (st.hump) { hump = Math.round(14 * Math.sin(Math.PI * frac)); humpMax = Math.max(humpMax, hump); } else hump = 0;
      let sonar = st.sonar != null ? st.sonar : st.sonarFrom != null ? Math.round(st.sonarFrom + (st.sonarTo - st.sonarFrom) * frac) : null;
      const rec = {
        mission_state: st.state, motion_state: st.motion ?? 1, speed_left_mms: Math.round(st.vl), speed_right_mms: Math.round(st.vr),
        enc_left: encL, enc_right: encR, line_state: st.line ?? (st.state === 7 ? 1 : 1), line_error: err, last_barcode: lastBc, reject_reason: 0,
        hump_max_mm: humpMax, obstacle_width_mm: width, obstacle_closest_mm: closest, obstacle_bearing_x10deg: 9, bypass_plan: plan,
        distance_mm: Math.round(dist), motion_class: 1, turn_rate_dps: 0, faults: 0, scan_state: st.scan || 0, pitch_x10: 0, roll_deg: 0, hump_now_mm: hump,
      };
      cb({ rec, age: 0.2, reads, letter, sonar, humpLast: humpMax || null });
      if (inStep >= st.ms) {
        if (st.onEnd) st.onEnd();
        inStep = 0; i++;
        if (i >= steps.length) { i = 0; reads = 0; letter = null; lastBc = 0; plan = 0; width = 0; closest = 0; humpMax = 0; }
      }
    }, tick);
    return () => clearInterval(id);
  };

  window.PicoRoad = PicoRoad;
})();
