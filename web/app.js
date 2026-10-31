// UI for the navigation port: two matplotlib-style plots, an rqt_graph-style node graph and the node logs.
(() => {
  "use strict";
  const $ = (id) => document.getElementById(id);
  const B = window.PROJECT_MAP;
  const prm = new Nav.PRM(B, { safety: 0.15, gridSize: 0.2, connectionDistance: 0.3 });
  const checkMap = new Nav.Map(B, 0.12);
  const C = { b: "#0000ff", c: "#00bfbf", y: "#bfbf00", r: "#ff0000", g: "#008000", c0: "#1f77b4", c1: "#ff7f0e", ink: "#1d1d1b" };

  let seed = 2; // the first run uses a seed we know localizes cleanly; "New run" draws fresh ones
  let goal = [-0.6, 1.0];
  let mission, running = true, speed = 3, printed = 0;
  function newMission(keepSeed) {
    if (!keepSeed) seed = (seed * 48271) % 2147483647;
    mission = new Nav.Mission(B, { seed, prm, goal, lookahead: +$("look").value, scanNoise: +$("noise").value });
    printed = 0; $("term").innerHTML = "";
  }

  // ------------------------------------------------------------ plots
  function Plot(canvas) {
    const ctx = canvas.getContext("2d");
    let W = 0, dpr = 1, pad = 34, s = 1;
    const [x0, y0, x1, y1] = [-1.3, -1.3, 1.3, 1.3];
    const fit = () => {
      dpr = Math.min(devicePixelRatio || 1, 2);
      W = canvas.clientWidth; canvas.width = canvas.height = Math.round(W * dpr);
      pad = W < 420 ? 28 : 36; s = (W - pad - 18) / (x1 - x0);
    };
    const X = (x) => (pad + (x - x0) * s) * dpr, Y = (y) => (18 + (y1 - y) * s) * dpr;
    const inv = (px, py) => [x0 + (px - pad) / s, y1 - (py - 18) / s];
    function frame() {
      ctx.setTransform(1, 0, 0, 1, 0, 0);
      ctx.clearRect(0, 0, canvas.width, canvas.height);
      // axes and ticks, matplotlib style
      ctx.strokeStyle = "#000"; ctx.lineWidth = 0.8 * dpr;
      ctx.strokeRect(X(x0), Y(y1), (x1 - x0) * s * dpr, (y1 - y0) * s * dpr);
      ctx.fillStyle = "#333"; ctx.font = `${10 * dpr}px "DejaVu Sans", Verdana, sans-serif`;
      for (let v = -1.2; v <= 1.21; v += 0.4) {
        const t = v.toFixed(1).replace("-0.0", "0.0");
        ctx.beginPath(); ctx.moveTo(X(v), Y(y0)); ctx.lineTo(X(v), Y(y0) + 4 * dpr); ctx.stroke();
        ctx.textAlign = "center"; ctx.fillText(t, X(v), Y(y0) + 14 * dpr);
        ctx.beginPath(); ctx.moveTo(X(x0), Y(v)); ctx.lineTo(X(x0) - 4 * dpr, Y(v)); ctx.stroke();
        ctx.textAlign = "right"; ctx.fillText(t, X(x0) - 6 * dpr, Y(v) + 3.5 * dpr);
      }
      // walls
      ctx.beginPath(); B.forEach(([x, y], i) => (i ? ctx.lineTo(X(x), Y(y)) : ctx.moveTo(X(x), Y(y)))); ctx.closePath();
      ctx.fillStyle = "#fff"; ctx.fill();
      ctx.strokeStyle = "#000"; ctx.lineWidth = 2.2 * dpr; ctx.lineJoin = "miter"; ctx.stroke();
    }
    return { ctx, fit, X, Y, inv, frame, get dpr() { return dpr; }, get s() { return s; }, canvas };
  }
  const world = Plot($("world")), belief = Plot($("belief"));

  function robotGlyph(P, x, y, th, fill, stroke) {
    const { ctx, X, Y, dpr, s } = P, r = 0.09 * s * dpr;
    ctx.beginPath(); ctx.arc(X(x), Y(y), r, 0, Math.PI * 2);
    ctx.fillStyle = fill; ctx.fill();
    if (stroke) { ctx.strokeStyle = stroke; ctx.lineWidth = 1.5 * dpr; ctx.stroke(); }
    ctx.beginPath(); ctx.moveTo(X(x), Y(y)); ctx.lineTo(X(x + 0.13 * Math.cos(th)), Y(y + 0.13 * Math.sin(th)));
    ctx.strokeStyle = stroke || "#fff"; ctx.lineWidth = 2 * dpr; ctx.stroke();
  }
  function star(P, x, y, r, color) {
    const { ctx, X, Y, dpr } = P; ctx.beginPath();
    for (let k = 0; k < 10; k++) { const a = -Math.PI / 2 + (k * Math.PI) / 5, rr = (k % 2 ? r * 0.45 : r) * dpr; ctx.lineTo(X(x) + rr * Math.cos(a), Y(y) + rr * Math.sin(a)); }
    ctx.closePath(); ctx.fillStyle = color; ctx.fill();
  }
  function line(P, pts, color, w, dash) {
    const { ctx, X, Y, dpr } = P; if (!pts || pts.length < 2) return;
    ctx.beginPath(); pts.forEach(([x, y], i) => (i ? ctx.lineTo(X(x), Y(y)) : ctx.moveTo(X(x), Y(y))));
    ctx.strokeStyle = color; ctx.lineWidth = w * dpr; ctx.setLineDash(dash ? dash.map((d) => d * dpr) : []); ctx.lineJoin = "round"; ctx.stroke(); ctx.setLineDash([]);
  }
  function dots(P, pts, color, r) { const { ctx, X, Y, dpr } = P; ctx.fillStyle = color; for (const [x, y] of pts) { ctx.beginPath(); ctx.arc(X(x), Y(y), r * dpr, 0, Math.PI * 2); ctx.fill(); } }

  function drawWorld() {
    const P = world, m = mission, r = m.robot, { ctx, X, Y, dpr } = P;
    P.frame();
    line(P, m.trueTrail, "#9a9a9a", 1.4);
    // LiDAR: faint rays and red hits
    const ox = r.x - Nav.LIDAR.offset * Math.cos(r.th), oy = r.y - Nav.LIDAR.offset * Math.sin(r.th);
    if ($("showRays").checked) {
      ctx.strokeStyle = "rgba(255,0,0,0.13)"; ctx.lineWidth = 0.7 * dpr; ctx.beginPath();
      m.scanZ.forEach((z, i) => { if (!Number.isFinite(z)) return; const a = r.th + i * Nav.LIDAR.step; ctx.moveTo(X(ox), Y(oy)); ctx.lineTo(X(ox + z * Math.cos(a)), Y(oy + z * Math.sin(a))); });
      ctx.stroke();
    }
    ctx.fillStyle = C.r;
    m.scanZ.forEach((z, i) => { if (!Number.isFinite(z)) return; const a = r.th + i * Nav.LIDAR.step; ctx.fillRect(X(ox + z * Math.cos(a)) - 1.3 * dpr, Y(oy + z * Math.sin(a)) - 1.3 * dpr, 2.6 * dpr, 2.6 * dpr); });
    // start and goal
    ctx.fillStyle = C.r; ctx.fillRect(X(m.start[0]) - 5 * dpr, Y(m.start[1]) - 5 * dpr, 10 * dpr, 10 * dpr);
    star(P, m.goal[0], m.goal[1], 10, C.g);
    if (m.estimate) robotGlyph(P, m.estimate[0], m.estimate[1], m.estimate[2], "rgba(255,127,14,0.18)", C.c1);
    robotGlyph(P, r.x, r.y, r.th, C.ink);
    const err = m.estimate ? Math.hypot(m.estimate[0] - r.x, m.estimate[1] - r.y) : null;
    $("worldRead").textContent = `t = ${m.t.toFixed(1)} s · v = ${m.cmd[0].toFixed(2)} m/s · ω = ${m.cmd[1].toFixed(2)} rad/s` + (err != null ? ` · estimate off by ${(err * 100).toFixed(1)} cm` : " · pose unknown to the robot");
  }

  function drawBelief() {
    const P = belief, m = mission, { ctx, X, Y, dpr } = P;
    P.frame();
    if ($("showRoad").checked) {
      ctx.strokeStyle = C.c; ctx.lineWidth = 0.6 * dpr; ctx.beginPath();
      prm.edges.forEach((nb, i) => nb.forEach((j) => { if (j > i) { ctx.moveTo(X(prm.nodes[i][0]), Y(prm.nodes[i][1])); ctx.lineTo(X(prm.nodes[j][0]), Y(prm.nodes[j][1])); } }));
      ctx.stroke();
      dots(P, prm.nodes, C.c, 2);
      if (m.rawPath && prm.expanded) dots(P, prm.expanded.map((i) => prm.nodes[i]), "rgba(0,191,191,0.35)", 5);
    }
    // particles as small arrows ('b' quiver)
    const L = 0.055;
    ctx.strokeStyle = C.b; ctx.lineWidth = 1 * dpr; ctx.beginPath();
    for (const [x, y, t] of m.pf.particles) {
      const ex = x + L * Math.cos(t), ey = y + L * Math.sin(t);
      ctx.moveTo(X(x), Y(y)); ctx.lineTo(X(ex), Y(ey));
      ctx.moveTo(X(ex), Y(ey)); ctx.lineTo(X(ex - 0.022 * Math.cos(t - 0.5)), Y(ey - 0.022 * Math.sin(t - 0.5)));
    }
    ctx.stroke();
    if (m.rawPath) { line(P, m.rawPath, C.c0, 1.6); dots(P, m.rawPath.slice(1, -1), C.b, 3); }
    if (m.path) { line(P, m.path, C.y, 2.6); dots(P, m.path.slice(1, -1), C.y, 1.8); }
    line(P, m.estTrail, "rgba(255,127,14,0.55)", 1.4);
    if (m.phase === "track" && m.pp.target && m.estimate) {
      line(P, [[m.estimate[0], m.estimate[1]], m.pp.target], C.c1, 1.2, [4, 3]);
      dots(P, [m.pp.target], C.c1, 4);
    }
    ctx.fillStyle = C.r; ctx.fillRect(X(m.start[0]) - 5 * dpr, Y(m.start[1]) - 5 * dpr, 10 * dpr, 10 * dpr);
    star(P, m.goal[0], m.goal[1], 10, C.g);
    if (m.estimate && m.localized) robotGlyph(P, m.estimate[0], m.estimate[1], m.estimate[2], C.c1, null);
    $("beliefRead").textContent = `${m.pf.particles.length} particles · ${m.pf.clusters} cluster${m.pf.clusters === 1 ? "" : "s"} · ` + (m.localized ? "localized" : "searching") + (m.phase === "track" ? ` · lookahead ${m.pp.lookahead.toFixed(2)} m` : "");
  }

  // ------------------------------------------------------------ ROS graph
  const svg = $("rosgraph"), NS = "http://www.w3.org/2000/svg";
  const el = (tag, a, p) => { const e = document.createElementNS(NS, tag); for (const k in a) e.setAttribute(k, a[k]); if (p) p.appendChild(e); return e; };
  const defs = el("defs", {}, svg);
  const mk = el("marker", { id: "arrow", viewBox: "0 0 10 10", refX: 9, refY: 5, markerWidth: 7, markerHeight: 7, orient: "auto-start-reverse" }, defs);
  el("path", { d: "M0 0 L10 5 L0 10 z", fill: "#8a877e" }, mk);
  const N = {
    coppeliasim: [30, 125, 150], particle_filter: [330, 30, 150], wall_follower: [330, 220, 150],
    probabilistic_roadmap: [620, 30, 205], pure_pursuit: [640, 220, 160],
  };
  const E = [
    ["coppeliasim", "particle_filter", "/scan /odometry", "M180 135 C 250 135, 260 52, 330 52", "scan"],
    ["coppeliasim", "wall_follower", "/scan /odometry", "M180 150 C 250 150, 260 232, 330 232", "scan"],
    ["particle_filter", "probabilistic_roadmap", "/pose", "M480 50 L 620 50", "pose"],
    ["particle_filter", "pure_pursuit", "/pose", "M470 66 C 560 110, 620 200, 660 220", "pose"],
    ["particle_filter", "wall_follower", "/pose", "M405 66 L 405 220", "pose"],
    ["probabilistic_roadmap", "pure_pursuit", "/path", "M735 66 L 735 220", "path"],
    ["wall_follower", "coppeliasim", "/cmd_vel", "M330 248 C 250 248, 200 210, 140 160", "cmdWF"],
    ["pure_pursuit", "coppeliasim", "/cmd_vel", "M720 256 C 700 300, 140 300, 100 160", "cmdPP"],
  ];
  const edgeEls = E.map(([a, b, label, d]) => {
    const g = el("g", { class: "edge" }, svg);
    const p = el("path", { d }, g);
    const t = el("text", {}, g); t.textContent = label;
    return { g, p, t, len: 0 };
  });
  const nodeEls = {};
  for (const k in N) {
    const [x, y, w] = N[k], g = el("g", { class: "node" }, svg);
    el("rect", { x, y, width: w, height: 36, rx: 18 }, g);
    const t = el("text", { x: x + w / 2, y: y + 22, "text-anchor": "middle" }, g); t.textContent = "/" + k;
    nodeEls[k] = g;
  }
  const lm = el("text", { x: 30, y: 30, "font-size": 11, fill: "#9c9a92" }, svg); lm.textContent = "/lifecycle_manager: all nodes active";
  requestAnimationFrame(() => edgeEls.forEach((e, i) => {
    e.len = e.p.getTotalLength();
    const mid = e.p.getPointAtLength(e.len * (i === 7 ? 0.5 : 0.5));
    const ix = mid.x + ([4, 5].includes(i) ? 6 : 0), iy = mid.y + ([4, 5].includes(i) ? 4 : i === 7 ? 14 : -6);
    e.t.setAttribute("x", ix); e.t.setAttribute("y", iy); if (![4, 5].includes(i)) e.t.setAttribute("text-anchor", "middle");
  }));
  const pulses = [];
  let lastTraffic = null, lastPulse = new Array(E.length).fill(0);
  function graphTick(now) {
    const m = mission, phase = m.phase;
    const active = { coppeliasim: true, particle_filter: true, wall_follower: phase === "explore", probabilistic_roadmap: !!m.rawPath, pure_pursuit: phase === "track" };
    for (const k in nodeEls) nodeEls[k].classList.toggle("on", active[k]);
    const live = { scan: true, pose: m.localized, path: !!m.rawPath, cmdWF: phase === "explore", cmdPP: phase === "track" };
    const tr = m.traffic;
    E.forEach((e, i) => {
      const key = e[4], hot = live[key];
      edgeEls[i].g.classList.toggle("hot", !!hot);
      const fired = lastTraffic && ((key === "scan" && tr.scan > lastTraffic.scan) || (key === "pose" && tr.pose > lastTraffic.pose) || (key === "path" && tr.path > lastTraffic.path) || ((key === "cmdWF" || key === "cmdPP") && tr.cmd_vel > lastTraffic.cmd_vel));
      if (hot && fired && now - lastPulse[i] > (key === "path" ? 0 : 140)) {
        lastPulse[i] = now;
        pulses.push({ i, t0: now, c: el("circle", { r: key === "path" ? 5 : 3.2, class: "pulse" }, svg) });
      }
    });
    lastTraffic = { ...tr };
    for (let k = pulses.length - 1; k >= 0; k--) {
      const p = pulses[k], e = edgeEls[p.i], f = (now - p.t0) / 500;
      if (f >= 1 || !e.len) { p.c.remove(); pulses.splice(k, 1); continue; }
      const pt = e.p.getPointAtLength(f * e.len); p.c.setAttribute("cx", pt.x); p.c.setAttribute("cy", pt.y);
    }
  }

  // ------------------------------------------------------------ logs and phase strip
  const esc = (s) => s.replace(/[&<>]/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;" }[c]));
  function logs() {
    const ev = mission.events, term = $("term");
    while (printed < ev.length) {
      const e = ev[printed++], warn = /contact with a wall|stuck|wrong corridor|lost the pose|timeout|no path/.test(e.msg);
      const d = document.createElement("div");
      d.innerHTML = `<span class="${warn ? "w" : "lvl"}">[${warn ? "WARN" : "INFO"}]</span> <span class="t">[${e.t.toFixed(2)}]</span> <span class="n">[${e.node}]</span>: ${esc(e.msg)}`;
      term.appendChild(d);
      while (term.children.length > 14) term.firstChild.remove();
    }
  }
  let warnedWrong = false;
  function phases() {
    const m = mission, order = ["explore", "plan", "track", "goal"];
    let cur = m.phase === "explore" ? (m.localized ? 1 : 0) : m.phase === "track" ? 2 : m.phase === "goal" ? 3 : 0;
    $("phases").querySelectorAll("li").forEach((li, i) => { li.classList.toggle("done", i < cur || (i === 3 && m.phase === "goal")); li.classList.toggle("now", i === cur && m.phase !== "goal"); });
    if (m.phase === "goal" && !warnedWrong) {
      warnedWrong = true;
      const err = Math.hypot(m.robot.x - m.goal[0], m.robot.y - m.goal[1]);
      if (err > 0.3) m.log("particle_filter", `wrong corridor: the robot believes it arrived, but it is ${err.toFixed(2)} m away. The maze is almost symmetric.`);
    }
    if (m.phase !== "goal") warnedWrong = false;
  }

  // after a clean arrival, wander on to another goal so the page keeps moving
  const tour = [[1.0, 1.0], [0.6, -1.0], [-1.0, 0.6], [1.0, -0.6], [-0.6, 1.0], [0.2, -0.6]];
  let tourIdx = 0, arrivedAt = null;
  function autoGoal(now) {
    const m = mission;
    if (m.phase !== "goal") { arrivedAt = null; return; }
    if (Math.hypot(m.robot.x - m.goal[0], m.robot.y - m.goal[1]) > 0.3) return; // wrong corridor: leave it on screen
    if (arrivedAt === null) arrivedAt = now;
    if (running && now - arrivedAt > 2500) {
      let g; do { g = tour[tourIdx++ % tour.length]; } while (Math.hypot(g[0] - m.goal[0], g[1] - m.goal[1]) < 0.3);
      goal = g; m.setGoal(g); arrivedAt = null;
    }
  }

  // ------------------------------------------------------------ loop
  let acc = 0, last = performance.now();
  function loop(now) {
    const dt = Math.min(0.1, (now - last) / 1000); last = now;
    if (running) {
      acc += dt * speed;
      let n = 0;
      while (acc >= mission.dt && n < 40) { mission.step(); acc -= mission.dt; n++; }
    }
    drawWorld(); drawBelief(); graphTick(now); logs(); phases(); autoGoal(now);
    requestAnimationFrame(loop);
  }

  // ------------------------------------------------------------ controls
  $("run").onclick = () => { running = !running; $("run").textContent = running ? "Pause" : "Run"; };
  $("restart").onclick = () => { newMission(false); running = true; $("run").textContent = "Pause"; };
  $("speed").onchange = (e) => (speed = +e.target.value);
  $("look").oninput = (e) => { const v = +e.target.value; $("lookV").textContent = v.toFixed(2) + " m"; mission.pp.lookahead = v; };
  $("noise").oninput = (e) => { const v = +e.target.value; $("noiseV").textContent = (v * 100).toFixed(1) + " cm"; mission.robot.scanNoise = v; };
  for (const P of [world, belief]) P.canvas.addEventListener("click", (e) => {
    const r = P.canvas.getBoundingClientRect(), [x, y] = P.inv(e.clientX - r.left, e.clientY - r.top);
    if (!checkMap.contains(x, y)) return;
    goal = [x, y];
    mission.setGoal(goal);
    if (!running) { running = true; $("run").textContent = "Pause"; }
  });
  const resize = () => { world.fit(); belief.fit(); };
  addEventListener("resize", resize);
  newMission(true); resize();
  requestAnimationFrame(loop);
})();
