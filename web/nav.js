// A JavaScript port of our ROS 2 navigation stack, so it can run in a browser.
// Each class mirrors a Python package in sim_ws/src with the parameters from amr_bringup/launch/project.launch.py:
//   Map            -> amr_localization/maps.py           (polygon, safety distance, ray casting)
//   WallFollower   -> amr_control/wall_follower.py       (state machine + PD on the side distance)
//   ParticleFilter -> amr_localization/particle_filter.py (1000 particles, 8 rays, DBSCAN pose)
//   PRM            -> amr_planning/prm.py                 (grid roadmap, A*, gradient smoothing)
//   PurePursuit    -> amr_control/pure_pursuit.py         (lookahead 0.3 m, turn in place above pi/3.5)
// The Robot class stands in for CoppeliaSim: differential drive kinematics, a 240-ray LiDAR and odometry noise.
(function (root) {
  "use strict";
  const TAU = Math.PI * 2;

  // ------------------------------------------------------------------ randomness (seeded, so runs are reproducible)
  function RNG(seed) {
    let a = seed >>> 0;
    const u = () => { a = (a + 0x6d2b79f5) >>> 0; let t = a; t = Math.imul(t ^ (t >>> 15), t | 1); t ^= t + Math.imul(t ^ (t >>> 7), t | 61); return ((t ^ (t >>> 14)) >>> 0) / 4294967296; };
    let spare = null;
    const normal = (mu = 0, s = 1) => {
      if (spare !== null) { const v = spare; spare = null; return mu + s * v; }
      let x, y, r; do { x = 2 * u() - 1; y = 2 * u() - 1; r = x * x + y * y; } while (r === 0 || r >= 1);
      const k = Math.sqrt((-2 * Math.log(r)) / r); spare = y * k; return mu + s * x * k;
    };
    return { u, normal, uniform: (lo, hi) => lo + (hi - lo) * u(), choice: (arr) => arr[Math.floor(u() * arr.length)] };
  }

  // ------------------------------------------------------------------ geometry
  function segIntersect(ax, ay, bx, by, cx, cy, dx, dy) {
    const rx = bx - ax, ry = by - ay, sx = dx - cx, sy = dy - cy;
    const den = rx * sy - ry * sx;
    if (Math.abs(den) < 1e-12) return null;
    const t = ((cx - ax) * sy - (cy - ay) * sx) / den, u = ((cx - ax) * ry - (cy - ay) * rx) / den;
    if (t < 0 || t > 1 || u < 0 || u > 1) return null;
    return [ax + t * rx, ay + t * ry, t];
  }
  function pointSegDist(px, py, ax, ay, bx, by) {
    const vx = bx - ax, vy = by - ay, L = vx * vx + vy * vy;
    let t = L ? ((px - ax) * vx + (py - ay) * vy) / L : 0; t = Math.max(0, Math.min(1, t));
    return Math.hypot(px - ax - t * vx, py - ay - t * vy);
  }
  const wrap = (a) => ((a % TAU) + TAU) % TAU;
  const wrapPi = (a) => ((a + Math.PI) % TAU + TAU) % TAU - Math.PI;

  // ------------------------------------------------------------------ Map (maps.py)
  class Map {
    constructor(boundary, safety = 0) {
      this.boundary = boundary;
      this.safety = safety;
      this.segments = boundary.map((p, i) => [p, boundary[(i + 1) % boundary.length]]).filter(([p, q]) => p[0] !== q[0] || p[1] !== q[1]);
      const xs = boundary.map((p) => p[0]), ys = boundary.map((p) => p[1]);
      this.bounds = [Math.min(...xs), Math.min(...ys), Math.max(...xs), Math.max(...ys)];
    }
    inside(x, y) {
      let c = false; const b = this.boundary;
      for (let i = 0, j = b.length - 1; i < b.length; j = i++) {
        const [xi, yi] = b[i], [xj, yj] = b[j];
        if ((yi > y) !== (yj > y) && x < ((xj - xi) * (y - yi)) / (yj - yi) + xi) c = !c;
      }
      return c;
    }
    wallDistance(x, y) { let d = Infinity; for (const [p, q] of this.segments) d = Math.min(d, pointSegDist(x, y, p[0], p[1], q[0], q[1])); return d; }
    // shapely: safe_polygon = polygon.buffer(-safety).contains(point)
    contains(x, y, safety = this.safety) { return this.inside(x, y) && this.wallDistance(x, y) > safety; }
    // shapely: LineString(segment).crosses(safe_polygon), checked along the segment
    crosses(a, b, safety = this.safety) {
      const n = Math.max(2, Math.ceil(Math.hypot(b[0] - a[0], b[1] - a[1]) / 0.01));
      for (let k = 0; k <= n; k++) { const t = k / n; if (!this.contains(a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, safety)) return true; }
      return false;
    }
    // closest intersection of a segment with the walls: [x, y, distance] or null
    collide(ax, ay, bx, by) {
      let best = null;
      for (const [p, q] of this.segments) {
        const hit = segIntersect(ax, ay, bx, by, p[0], p[1], q[0], q[1]);
        if (hit && (!best || hit[2] < best[2])) best = hit;
      }
      return best ? [best[0], best[1], best[2] * Math.hypot(bx - ax, by - ay)] : null;
    }
  }

  // ------------------------------------------------------------------ the robot and the simulator (CoppeliaSim's job)
  const LIDAR = { rays: 240, step: (1.5 * Math.PI) / 180, max: 8.0, min: 0.16, offset: 0.035 };
  class Robot {
    constructor(map, pose, rng, opts = {}) {
      this.map = map; this.rng = rng;
      [this.x, this.y, this.th] = pose;
      this.radius = 0.09;
      this.scanNoise = opts.scanNoise ?? 0.01;
      this.odomNoise = opts.odomNoise ?? 0.02;
      this.crashed = false;
    }
    // ideal ray casting from the LiDAR origin, which sits 3.5 cm behind the robot centre
    static cast(map, x, y, th, indices) {
      const ox = x - LIDAR.offset * Math.cos(th), oy = y - LIDAR.offset * Math.sin(th);
      return indices.map((i) => {
        const a = th + i * LIDAR.step;
        const hit = map.collide(ox, oy, ox + LIDAR.max * Math.cos(a), oy + LIDAR.max * Math.sin(a));
        return hit ? hit[2] : NaN;
      });
    }
    scan() {
      const all = Array.from({ length: LIDAR.rays }, (_, i) => i);
      return Robot.cast(this.map, this.x, this.y, this.th, all).map((d) => {
        if (!Number.isFinite(d)) return NaN;
        const z = d + this.rng.normal(0, this.scanNoise);
        return z < LIDAR.min || z > LIDAR.max ? NaN : z;
      });
    }
    // apply (v, w) for dt; returns odometry (z_v, z_w) with a little wheel noise
    drive(v, w, dt) {
      if (this.crashed) return [0, 0];
      // walls are solid: on contact the robot slides along them, the way it scrapes a wall in the simulator
      const dx = v * Math.cos(this.th) * dt, dy = v * Math.sin(this.th) * dt, ok = (x, y) => this.map.wallDistance(x, y) >= this.radius;
      this.bumped = false;
      if (ok(this.x + dx, this.y + dy)) { this.x += dx; this.y += dy; }
      else {
        this.bumped = v > 0.02;
        if (ok(this.x + dx, this.y)) this.x += dx; else if (ok(this.x, this.y + dy)) this.y += dy;
      }
      this.th = wrap(this.th + w * dt);
      return [v * (1 + this.rng.normal(0, this.odomNoise)), w * (1 + this.rng.normal(0, this.odomNoise))];
    }
  }

  // ------------------------------------------------------------------ WallFollower (wall_follower.py)
  class WallFollower {
    constructor(dt, rng) {
      this.dt = dt; this.rng = rng;
      this.LINEAR_SPEED_MAX = 0.22; this.TRACK = 0.16;
      this.frontThreshold = 0.23; this.expectedTurning = this.frontThreshold * Math.SQRT2 * 1.3;
      this.v = 0.15; this.w = 0; this.front = 0; this.right = 0; this.left = 0;
      this.lastRightErr = 0; this.lastLeftErr = 0; this.rightErr = 0; this.leftErr = 0;
      this.target = 0.2; this.pathWidth = 0.4; this.Kp = 4; this.Kd = 5;
      this.followed = "right"; this.state = "Front";
    }
    static safeMin(vals, dflt = 8.0) { const v = vals.filter(Number.isFinite); return v.length ? Math.min(...v) : dflt; }
    compute(z) {
      const n = z.length, q = (3 * n) >> 2, h = n >> 2;
      this.front = WallFollower.safeMin(z.slice(0, 5).concat(z.slice(-5)));
      this.right = WallFollower.safeMin(z.slice(q - 5, q + 5));
      this.left = WallFollower.safeMin(z.slice(h - 5, h + 5));
      this.rightErr = this.target - this.right;
      this.leftErr = -this.target + this.left;
      if (this.state === "Front") [this.v, this.w] = this.front_();
      else if (this.state === "Turn_Right") { [this.v, this.w] = [0, -0.5]; if (this.front >= this.expectedTurning) this.state = "Front"; }
      else if (this.state === "Turn_Left") { [this.v, this.w] = [0, 0.5]; if (this.front >= this.expectedTurning) this.state = "Front"; }
      this.lastRightErr = this.rightErr; this.lastLeftErr = this.leftErr;
      return [this.v, this.w];
    }
    front_() {
      if (this.left + this.right >= this.pathWidth) this.followed = this.right <= this.left ? "right" : "left";
      if (this.front <= this.frontThreshold) { this.turn_(); return [0, 0]; }
      const w = this.wVel();
      return [this.LINEAR_SPEED_MAX - Math.abs(w) * (this.TRACK / 2), w];
    }
    turn_() {
      this.rightErr = 0; this.leftErr = 0;
      const diff = this.right - this.left;
      if (Math.abs(diff) <= 0.05) this.state = this.rng.choice(["Turn_Left", "Turn_Right"]);
      else if (diff <= 0) { this.state = "Turn_Left"; this.followed = "right"; }
      else { this.state = "Turn_Right"; this.followed = "left"; }
    }
    wVel() {
      const [e, le] = this.followed === "right" ? [this.rightErr, this.lastRightErr] : [this.leftErr, this.lastLeftErr];
      return this.Kp * e + this.Kd * ((e - le) / this.dt);
    }
  }

  // ------------------------------------------------------------------ DBSCAN on (x, y, cos th, sin th), as sklearn with eps=0.2, min_samples=5
  function dbscan(P, eps = 0.2, minPts = 5) {
    const n = P.length, e2 = eps * eps, labels = new Int32Array(n).fill(-2), core = new Uint8Array(n);
    const F = P.map(([x, y, t]) => [x, y, Math.cos(t), Math.sin(t)]);
    const nb = (i) => { const r = [], a = F[i]; for (let j = 0; j < n; j++) { const b = F[j], d = (a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2 + (a[3] - b[3]) ** 2; if (d <= e2) r.push(j); } return r; };
    let c = 0;
    for (let i = 0; i < n; i++) {
      if (labels[i] !== -2) continue;
      const N = nb(i);
      if (N.length < minPts) { labels[i] = -1; continue; }
      core[i] = 1; labels[i] = c;
      const queue = N.slice();
      for (let k = 0; k < queue.length; k++) {
        const j = queue[k];
        if (labels[j] === -1) labels[j] = c;
        if (labels[j] !== -2) continue;
        labels[j] = c;
        const M = nb(j);
        if (M.length >= minPts) { core[j] = 1; for (const m of M) queue.push(m); }
      }
      c++;
    }
    return { labels, core, clusters: c };
  }

  // ------------------------------------------------------------------ ParticleFilter (particle_filter.py)
  class ParticleFilter {
    constructor(map, rng, opts = {}) {
      this.map = map; this.rng = rng;
      this.dt = opts.dt ?? 0.05;
      this.sigmaV = opts.sigmaV ?? 0.05; this.sigmaW = opts.sigmaW ?? 0.1; this.sigmaZ = opts.sigmaZ ?? 0.2;
      this.count = opts.particles ?? 1000; this.localizedCount = 25;
      this.rays = []; for (let i = 0; i < 240; i += 240 / 8) this.rays.push(i);
      this.iteration = 0; this.clusters = 0;
      this.particles = this.init(this.count);
    }
    init(n) {
      const [x0, y0, x1, y1] = this.map.bounds, P = [];
      while (P.length < n) {
        const x = this.rng.uniform(x0, x1), y = this.rng.uniform(y0, y1);
        if (this.map.contains(x, y)) P.push([x, y, this.rng.choice([0, Math.PI / 2, Math.PI, (3 * Math.PI) / 2])]);
      }
      return P;
    }
    move(v, w) {
      this.iteration++;
      for (const p of this.particles) {
        const [x0, y0] = p, nv = v + this.rng.normal(0, this.sigmaV), nw = w + this.rng.normal(0, this.sigmaW);
        p[0] += Math.cos(p[2]) * nv * this.dt; p[1] += Math.sin(p[2]) * nv * this.dt;
        p[2] = wrap(p[2] + nw * this.dt);
        const hit = this.map.collide(x0, y0, p[0], p[1]);
        if (hit) { p[0] = hit[0]; p[1] = hit[1]; }
      }
    }
    gaussian(mu, s, x) { return Math.exp(-0.5 * ((x - mu) / s) ** 2) / (s * Math.sqrt(TAU)); }
    likelihood(z, p) {
      const zh = Robot.cast(this.map, p[0], p[1], p[2], this.rays);
      let prob = 1;
      this.rays.forEach((r, k) => {
        const m = Number.isFinite(z[r]) ? z[r] : LIDAR.min, ph = Number.isFinite(zh[k]) ? zh[k] : LIDAR.min;
        prob *= this.gaussian(m, this.sigmaZ, ph);
      });
      return prob;
    }
    resample(z) {
      const w = this.particles.map((p) => this.likelihood(z, p));
      let tot = w.reduce((a, b) => a + b, 0);
      if (!(tot > 0)) { w.fill(1); tot = w.length; }
      const cum = []; let acc = 0; for (const x of w) { acc += x / tot; cum.push(acc); }
      const n = this.count, r0 = this.rng.uniform(0, 1 / n), out = [];
      let j = 0;
      for (let i = 0; i < n; i++) {
        const r = r0 + i / n;
        while (j < cum.length - 1 && cum[j] <= r) j++; // np.digitize(r, cum)
        out.push(this.particles[j].slice());
      }
      this.particles = out;
      this.lastWeights = w;
    }
    computePose() {
      const { labels, core, clusters } = dbscan(this.particles);
      this.clusters = clusters;
      if (clusters === 1) {
        let sx = 0, sy = 0, sc = 0, ss = 0, n = 0;
        this.particles.forEach((p, i) => { if (core[i]) { sx += p[0]; sy += p[1]; sc += Math.cos(p[2]); ss += Math.sin(p[2]); n++; } });
        this.count = this.localizedCount;
        return [true, [sx / n, sy / n, wrap(Math.atan2(ss / n, sc / n))]];
      }
      if (clusters > 1) this.count = Math.max(100 * clusters, 100);
      return [false, null];
    }
  }

  // ------------------------------------------------------------------ PRM (prm.py)
  class PRM {
    constructor(boundary, opts = {}) {
      this.map = new Map(boundary, opts.safety ?? 0.15);
      const g = opts.gridSize ?? 0.2, cd = opts.connectionDistance ?? 0.3;
      const [x0, y0, x1, y1] = this.map.bounds;
      this.nodes = [];
      for (let x = x0; x <= x1 + 1e-9; x += g) for (let y = y0; y <= y1 + 1e-9; y += g) {
        const p = [Math.round(x * 1e6) / 1e6, Math.round(y * 1e6) / 1e6];
        if (this.map.contains(p[0], p[1])) this.nodes.push(p);
      }
      this.edges = this.nodes.map(() => []);
      this.nodes.forEach((a, i) => this.nodes.forEach((b, j) => {
        if (i !== j && Math.hypot(a[0] - b[0], a[1] - b[1]) <= cd + 1e-9 && !this.map.crosses(a, b)) this.edges[i].push(j);
      }));
    }
    closest(p) { let bi = 0, bd = Infinity; this.nodes.forEach((n, i) => { const d = Math.hypot(n[0] - p[0], n[1] - p[1]); if (d < bd) { bd = d; bi = i; } }); return bi; }
    // A* from the node closest to the start to the node closest to the goal
    findPath(start, goal) {
      const s = this.closest(start), t = this.closest(goal), N = this.nodes;
      const h = (i) => Math.hypot(N[i][0] - goal[0], N[i][1] - goal[1]);
      const open = new Map_(), closed = new Set(), anc = new Map_();
      open.set(s, [h(s), 0]);
      this.expanded = [];
      while (open.size) {
        let cur = -1, bf = Infinity; for (const [k, [f]] of open) if (f < bf) { bf = f; cur = k; }
        const g = open.get(cur)[1];
        if (cur === t) {
          const path = [goal]; let c = t; while (c !== s) { path.push(N[c]); c = anc.get(c); } path.push(N[s], start);
          return path.reverse();
        }
        closed.add(cur); open.delete(cur); this.expanded.push(cur);
        for (const nb of this.edges[cur]) {
          if (closed.has(nb)) continue;
          const ng = g + Math.hypot(N[cur][0] - N[nb][0], N[cur][1] - N[nb][1]);
          if (!open.has(nb) || ng < open.get(nb)[1]) { anc.set(nb, cur); open.set(nb, [ng + h(nb), ng]); }
        }
      }
      return null;
    }
    static smooth(path, dataW = 0.1, smoothW = 0.1, extra = 5, tol = 1e-6) {
      let P = path;
      if (extra > 0) {
        P = [];
        for (let i = 0; i < path.length - 1; i++) {
          const a = path[i], b = path[i + 1];
          P.push(a);
          for (let j = 1; j <= extra; j++) P.push([a[0] + ((b[0] - a[0]) * j) / (extra + 1), a[1] + ((b[1] - a[1]) * j) / (extra + 1)]);
        }
        P.push(path[path.length - 1]);
      }
      const s = P.map((p) => p.slice());
      for (let it = 0; it < 5000; it++) {
        let change = 0;
        for (let i = 1; i < P.length - 1; i++) for (let d = 0; d < 2; d++) {
          const old = s[i][d];
          s[i][d] += dataW * (P[i][d] - s[i][d]) + smoothW * (s[i + 1][d] + s[i - 1][d] - 2 * s[i][d]);
          change += Math.abs(s[i][d] - old);
        }
        if (change < tol) break;
      }
      return s;
    }
  }
  const Map_ = root.Map || globalThis.Map; // the built-in Map, since our class shadows the name

  // ------------------------------------------------------------------ PurePursuit (pure_pursuit.py)
  class PurePursuit {
    constructor(lookahead = 0.3) { this.lookahead = lookahead; this.path = []; this.target = null; }
    compute(x, y, th) {
      if (!this.path.length) return [0, 0];
      let ci = 0, cd = Infinity;
      this.path.forEach((p, i) => { const d = (p[0] - x) ** 2 + (p[1] - y) ** 2; if (d < cd) { cd = d; ci = i; } });
      let L = this.path[this.path.length - 1];
      for (let i = ci; i < this.path.length; i++) { const p = this.path[i]; if ((p[0] - x) ** 2 + (p[1] - y) ** 2 >= this.lookahead ** 2) { L = p; break; } }
      this.target = L;
      const vx = L[0] - x, vy = L[1] - y, l = Math.hypot(vx, vy);
      const alpha = wrapPi(Math.atan2(vy, vx) - th);
      const vMin = 0.1, vMax = 0.22, maxAngle = Math.PI / 3.5;
      if (Math.abs(alpha) > maxAngle) return [0, 0.5 * Math.sign(alpha)];
      const v = Math.max(vMin, Math.min(vMax * (1 - Math.abs(alpha) / maxAngle), vMax));
      return [v, l > 0 ? (v * 2 * Math.sin(alpha)) / l : 0];
    }
  }

  // ------------------------------------------------------------------ the launch file: all nodes wired together
  class Mission {
    constructor(boundary, opts = {}) {
      this.opts = opts;
      this.rng = RNG(opts.seed ?? 1);
      this.dt = 0.05;
      this.map = new Map(boundary, 0.08);            // particle filter map (safety 0.08)
      this.start = opts.start ?? [-1, -1, Math.PI / 2];
      this.goal = opts.goal ?? [-0.6, 1.0];
      this.robot = new Robot(this.map, this.start, this.rng, { scanNoise: opts.scanNoise });
      this.pf = new ParticleFilter(this.map, this.rng, { dt: this.dt, particles: opts.particles ?? 1000, sigmaZ: opts.sigmaZ ?? 0.2 });
      this.wf = new WallFollower(this.dt, this.rng);
      this.prm = opts.prm || new PRM(boundary, { safety: 0.15, gridSize: 0.2, connectionDistance: 0.3 });
      this.pp = new PurePursuit(opts.lookahead ?? 0.3);
      this.stepsBetweenSense = 10;
      this.t = 0; this.steps = 0;
      this.localized = false; this.estimate = null;
      this.phase = "explore"; // explore -> plan -> track -> goal | crashed | failed
      this.events = [];
      this.trueTrail = [[this.robot.x, this.robot.y]]; this.estTrail = [];
      this.scanZ = this.robot.scan(); this.cmd = [0, 0]; this.odom = [0, 0];
      this.traffic = { scan: 0, odom: 0, pose: 0, path: 0, cmd_vel: 0 };
      this.log("lifecycle_manager", "configure → activate: particle_filter, probabilistic_roadmap, wall_follower, pure_pursuit");
      this.log("particle_filter", `global localization with ${this.pf.particles.length} particles`);
    }
    log(node, msg) { this.events.push({ t: this.t, node, msg }); }
    step() {
      if (this.phase === "goal" || this.phase === "crashed" || this.phase === "failed") return;
      // CoppeliaSim publishes /scan and /odom
      this.t += this.dt; this.steps++;
      this.scanZ = this.robot.scan(); this.traffic.scan++; this.traffic.odom++;
      // particle filter: motion step, then a measurement step every 10 steps (every step once localized)
      this.pf.move(this.odom[0], this.odom[1]);
      if (this.localized || this.steps % this.stepsBetweenSense === 0) {
        this.pf.resample(this.scanZ);
        const wasLocalized = this.localized, before = this.pf.clusters;
        const [ok, pose] = this.pf.computePose();
        this.localized = ok;
        if (ok) { this.estimate = pose; this.estTrail.push([pose[0], pose[1]]); this.traffic.pose++; }
        if (ok && !wasLocalized) {
          const err = Math.hypot(pose[0] - this.robot.x, pose[1] - this.robot.y);
          this.log("particle_filter", `localized after ${this.t.toFixed(1)} s, one cluster left; error ${(err * 100).toFixed(1)} cm; keeping 25 particles`);
        } else if (!ok && this.pf.clusters !== before && this.pf.clusters > 1) {
          this.log("particle_filter", `${this.pf.clusters} clusters, ${this.pf.count} particles`);
        }
        if (!ok && wasLocalized) { this.log("particle_filter", "lost the pose, back to exploring"); this.phase = "explore"; this.pp.path = []; }
      }
      // planning, once
      if (this.localized && this.phase === "explore") {
        const raw = this.prm.findPath([this.estimate[0], this.estimate[1]], this.goal);
        if (!raw) { this.phase = "failed"; this.log("probabilistic_roadmap", "no path found"); return; }
        this.rawPath = raw;
        this.path = PRM.smooth(raw, 0.1, 0.1, 5);
        this.pp.path = this.path; this.traffic.path++;
        const len = this.path.reduce((a, p, i) => (i ? a + Math.hypot(p[0] - this.path[i - 1][0], p[1] - this.path[i - 1][1]) : 0), 0);
        this.log("probabilistic_roadmap", `A* over ${this.prm.nodes.length} nodes expanded ${this.prm.expanded.length}; smoothed path ${len.toFixed(2)} m`);
        this.phase = "track";
        this.log("pure_pursuit", `tracking with lookahead ${this.pp.lookahead.toFixed(2)} m`);
      }
      // control: wall follower while lost, pure pursuit once there is a path
      if (this.phase === "explore") this.cmd = this.wf.compute(this.scanZ);
      else if (this.phase === "track") {
        const [x, y, th] = this.estimate;
        if (Math.hypot(x - this.goal[0], y - this.goal[1]) < 0.1) {
          this.cmd = [0, 0]; this.phase = "goal";
          const err = Math.hypot(this.robot.x - this.goal[0], this.robot.y - this.goal[1]);
          this.log("coppeliasim", `goal reached in ${this.t.toFixed(1)} s; true distance to goal ${(err * 100).toFixed(1)} cm`);
        } else this.cmd = this.pp.compute(x, y, th);
      }
      this.traffic.cmd_vel++;
      this.odom = this.robot.drive(this.cmd[0], this.cmd[1], this.dt);
      if (this.robot.bumped) { this.bumps = (this.bumps || 0) + 1; this.stuck = (this.stuck || 0) + 1; if (this.bumps === 1 || this.stuck === 1) this.log("coppeliasim", "contact with a wall"); } else this.stuck = 0;
      if (this.stuck > 100) { this.phase = "crashed"; this.log("coppeliasim", "stuck against a wall for 5 s"); }
      this.trueTrail.push([this.robot.x, this.robot.y]);
      if (this.t > 400) { this.phase = "failed"; this.log("coppeliasim", "timeout"); }
    }
  }

  // a new goal while driving: plan again from the current estimate
  Mission.prototype.setGoal = function (g) {
    this.goal = g;
    if (this.phase === "track" || this.phase === "goal") {
      this.phase = "explore";
      this.log("probabilistic_roadmap", `new goal (${g[0].toFixed(2)}, ${g[1].toFixed(2)})`);
    } else this.log("probabilistic_roadmap", `goal set to (${g[0].toFixed(2)}, ${g[1].toFixed(2)})`);
  };

  const api = { RNG, Map, Robot, WallFollower, ParticleFilter, PRM, PurePursuit, Mission, dbscan, LIDAR };
  if (typeof module !== "undefined" && module.exports) module.exports = api; else root.Nav = api;
})(typeof window !== "undefined" ? window : globalThis);
