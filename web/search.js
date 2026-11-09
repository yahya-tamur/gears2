// Background thread: looks for values that build ("Find values").
//
// The values marked "set" stay as they are; every other number may move
// along its slider's steps, within the slider's range. assess() (Rust) says
// how far a combination is from building without building it: 0 when every
// check passes, otherwise a score that shrinks as it gets closer.
//
// Best-first search: always look next around the combination with the
// smallest score + distance from the starting values (distance: how far each
// value moved, as a fraction of its slider's range, added up), so the first
// working combination found changes as little as it can. Around a
// combination, each free value is moved by 1, 2, 3, 5, 8, ... steps each way.
//
// Values that can't change the score (a helix angle never makes a gear fail
// to build) would only multiply the combinations to try, so first each free
// value's moves are tried from the start and from random points across the
// ranges, and only values whose moves change the score somewhere are
// searched. If that finds nothing, the search runs again over all of them.
//
// Best-first search can get stuck in a dip it can only leave through worse
// scores (fewer teeth than the gear set allows now, before a larger pressure
// angle lowers that minimum). Then simulated annealing takes over: random
// moves, keeping worse ones with a chance that shrinks as it cools. Once
// something works, each value is walked back toward its starting value as
// far as it keeps working, so as little as possible changes.

import init, { assess } from "./pkg/gears2.js";

const ready = init();
const MOVES = [1, 2, 3, 5, 8, 13, 21, 34];

function decimals(step) {
  const text = String(step);
  return text.includes("e-") ? Number(text.split("e-")[1]) : (text.split(".")[1] || "").length;
}

class Heap {
  constructor() { this.a = []; }
  get size() { return this.a.length; }
  push(x) {
    const a = this.a;
    a.push(x);
    for (let i = a.length - 1; i > 0;) {
      const p = (i - 1) >> 1;
      if (a[p].f <= a[i].f) break;
      [a[p], a[i]] = [a[i], a[p]];
      i = p;
    }
  }
  pop() {
    const a = this.a, top = a[0], last = a.pop();
    if (a.length) {
      a[0] = last;
      for (let i = 0;;) {
        const l = 2 * i + 1, r = l + 1;
        let m = i;
        if (l < a.length && a[l].f < a[m].f) m = l;
        if (r < a.length && a[r].f < a[m].f) m = r;
        if (m === i) break;
        [a[m], a[i]] = [a[i], a[m]];
        i = m;
      }
    }
    return top;
  }
}

function search(params, allFree, id) {
  const start = performance.now();
  const p = { ...params };
  let evals = 0;
  const scoreOf = (free, vals) => {
    for (let i = 0; i < free.length; i++) p[free[i].name] = vals[i];
    evals++;
    return assess(p);
  };
  const snap = (f, raw) => +(f.min + Math.round((raw - f.min) / f.step) * f.step).toFixed(decimals(f.step) + 2);

  // Which values can change the score?
  const v0 = allFree.map((f) => f.value);
  let seed = 12345;
  const random = () => ((seed = (seed * 1103515245 + 12345) % 2147483648) / 2147483648);
  const points = [v0];
  for (let k = 0; k < 60; k++) points.push(allFree.map((f) => snap(f, f.min + random() * (f.max - f.min))));
  const relevant = allFree.map(() => false);
  for (const point of points) {
    const b = scoreOf(allFree, point);
    allFree.forEach((f, i) => {
      if (relevant[i]) return;
      for (const k of MOVES) {
        for (const sign of [-1, 1]) {
          const raw = point[i] + sign * k * f.step;
          if (raw < f.min - 1e-9 || raw > f.max + 1e-9) continue;
          const vals = point.slice();
          vals[i] = snap(f, raw);
          if (scoreOf(allFree, vals) !== b) {
            relevant[i] = true;
            return;
          }
        }
      }
    });
  }
  for (let i = 0; i < allFree.length; i++) p[allFree[i].name] = v0[i];

  const run = (free, budget) => {
    const n = free.length;
    const w0 = free.map((f) => f.value);
    const dist = (vals) => vals.reduce((a, v, i) => a + Math.abs(v - w0[i]) / (free[i].max - free[i].min || 1), 0);
    const heap = new Heap();
    const seen = new Set([w0.join(",")]);
    const b0 = scoreOf(free, w0);
    heap.push({ f: b0, b: b0, vals: w0 });
    let best = { b: b0 };
    const until = evals + budget;
    while (heap.size) {
      const node = heap.pop();
      if (node.b === 0) return { found: true, values: node.vals };
      if (node.b < best.b) best = node;
      for (let i = 0; i < n; i++) {
        const f = free[i];
        for (const k of MOVES) {
          for (const sign of [-1, 1]) {
            const raw = node.vals[i] + sign * k * f.step;
            if (raw < f.min - 1e-9 || raw > f.max + 1e-9) continue;
            const vals = node.vals.slice();
            vals[i] = snap(f, raw);
            const key = vals.join(",");
            if (seen.has(key)) continue;
            seen.add(key);
            const b = scoreOf(free, vals);
            heap.push({ f: b + dist(vals), b, vals });
            if (evals % 5000 === 0) self.postMessage({ type: "progress", id, evals });
          }
        }
      }
      if (evals > until || performance.now() - start > 20000) break;
    }
    return { found: false, best: best.b };
  };

  const anneal = (free, from, budget) => {
    const n = free.length;
    let cur = from.slice(), b = scoreOf(free, cur);
    const until = evals + budget;
    let k = 0;
    while (evals < until && performance.now() - start < 20000) {
      const t = 2.0 * Math.pow(0.002, (k++ % 40000) / 40000); // cools, then reheats
      const i = Math.floor(random() * n), f = free[i];
      const step = MOVES[Math.floor(random() * random() * MOVES.length)] * (random() < 0.5 ? -1 : 1);
      const raw = cur[i] + step * f.step;
      if (raw < f.min - 1e-9 || raw > f.max + 1e-9) continue;
      const vals = cur.slice();
      vals[i] = snap(f, raw);
      const nb = scoreOf(free, vals);
      if (nb <= b || random() < Math.exp((b - nb) / t)) {
        cur = vals;
        b = nb;
        if (b === 0) return { found: true, values: cur };
      }
      if (evals % 5000 === 0) self.postMessage({ type: "progress", id, evals });
    }
    return { found: false };
  };
  // Walk each value back toward where it started while it still works.
  const polish = (free, vals) => {
    const w0 = free.map((f) => f.value);
    for (let changed = true; changed;) {
      changed = false;
      free.forEach((f, i) => {
        while (Math.abs(vals[i] - w0[i]) > 1e-9) {
          const toward = vals.slice();
          const next = vals[i] + Math.sign(w0[i] - vals[i]) * f.step;
          toward[i] = Math.abs(next - w0[i]) < f.step / 2 || Math.sign(w0[i] - next) !== Math.sign(w0[i] - vals[i]) ? w0[i] : snap(f, next);
          if (scoreOf(free, toward) !== 0) break;
          vals = toward;
          changed = true;
        }
      });
    }
    return vals;
  };

  const chosen = allFree.filter((_, i) => relevant[i]);
  const attempt = (free) => {
    let r = run(free, 150000);
    if (!r.found) r = anneal(free, free.map((f) => f.value), 400000);
    return r.found ? polish(free, r.values) : null;
  };
  let found = chosen.length ? attempt(chosen) : null;
  let values = null;
  if (found) {
    values = v0.slice();
    chosen.forEach((f, j) => { values[allFree.indexOf(f)] = found[j]; });
  } else {
    found = attempt(allFree);
    values = found;
  }
  const result = { found: !!found };
  return { found: result.found, values, evals, searched: chosen.map((f) => f.name) };
}

self.onmessage = async (event) => {
  const { id, params, free } = event.data;
  await ready;
  try {
    self.postMessage({ type: "done", id, ...search(params, free, id) });
  } catch (e) {
    self.postMessage({ type: "done", id, found: false, evals: 0, failed: String(e) });
  }
};
