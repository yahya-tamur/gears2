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
// A pass stops early once STALL combinations go by without getting any
// closer: wandering further from there usually doesn't help (one search got
// as close as it would in 1,500 combinations, then spent 148,000 more). If
// the quick passes find nothing, the slider-step pass runs again without
// stopping early (sometimes the answer is past a long stretch of nothing
// better), followed by the (quick) finer passes from where it got.
//
// A working range can be narrower than a slider step (one tooth height in
// 0.02 mm), so when the slider's steps find nothing, the search runs again
// with steps 10 times finer (whole numbers, like tooth counts, stay whole),
// from each of the few closest combinations the slider's steps found, in
// turn. No finer than that: a value that only works within a hair's width
// wouldn't survive printing anyway.
//
// Best-first search can get stuck in a dip it can only leave through worse
// scores (fewer teeth than the gear set allows now, before a larger pressure
// angle lowers that minimum). Then simulated annealing takes over: random
// moves, keeping worse ones with a chance that shrinks as it cools. Once
// something works, each value is walked back toward its starting value as
// far as it keeps working, so as little as possible changes.
//
// Each combination is scored in Rust (search_score, see settle() in
// web.rs), which first moves free values past a limit that can be worked
// out straight to it (on a slider step): a bore to at most the gear's flat
// end face, the ring to at least so wide, a tooth count to at least the gear
// set's minimum, and tooth height or pressure angle to the nearest value
// that lets the tooth counts work. Bores and the ring's outer diameter only
// matter for their own check, so they aren't searched at all.
//
// First, for spur, ring and planetary gears, an exhaustive search in Rust
// (search_exhaustive, see search.rs): only the gear-set values are hard to
// solve together, so it goes through all of their combinations, nearest
// first, and counts through tooth and planet counts for each. It finds the
// working combination nearest the starting values, or that there's none
// (on the sliders' steps, then 10 times finer ones). Only when it can't
// (too many combinations, a bevel pair, or free values it doesn't search,
// like the helix angle, and nothing found) does the general search below
// run.
//
// With logging on (searchLog(true) in the page's console), every distinct
// combination scored is kept, as settled, with its score, and sent back.
// (The exhaustive search logs a code instead of a score: 0 works, 1 fails a
// check, 2 makes no gear set.)

import init, { search_begin, search_score, search_exhaustive, search_take_log } from "./pkg/gears2.js";

// values only their own check depends on: settled, never searched (the same
// list as SOLVED in web.rs)
const SOLVED = new Set(["bore", "bore2", "planet_bore", "rim"]);

const ready = init();
const MOVES = [1, 2, 3, 5, 8, 13, 21, 34];
const STALL = 5000;
const NEAR = 6;

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

function search(params, allFree, id, logging) {
  const start = performance.now();
  // Scores already worked out are remembered: evals counts distinct
  // combinations, not repeats.
  search_begin(params, allFree.map((f) => f.name), Float64Array.from(allFree.flatMap((f) => [f.min, f.max, f.step, f.value])));
  let evals = 0;
  const memo = new Map(); // combination -> [score, values as settled]
  const width = allFree.length + 1;
  let log = logging ? new Float64Array(width * 65536) : null, logged = 0;
  const lookup = (free, vals) => {
    const full = allFree.map((f) => f.value);
    for (let i = 0; i < free.length; i++) full[allFree.indexOf(free[i])] = vals[i];
    const key = full.join(",");
    let r = memo.get(key);
    if (r === undefined) {
      evals++;
      r = search_score(Float64Array.from(full));
      if (memo.size > 200000) memo.clear(); // keep memory bounded
      memo.set(key, r);
      if (log) {
        if ((logged + 1) * width > log.length) {
          const bigger = new Float64Array(log.length * 2);
          bigger.set(log);
          log = bigger;
        }
        log.set(r.subarray(1), logged * width);
        log[logged * width + width - 1] = r[0];
        logged++;
      }
    }
    return r;
  };
  const scoreOf = (free, vals) => lookup(free, vals)[0];
  const logRows = (rows) => {
    if (!log || !rows.length) return;
    while ((logged * width + rows.length) > log.length) {
      const bigger = new Float64Array(log.length * 2);
      bigger.set(log);
      log = bigger;
    }
    log.set(rows, logged * width);
    logged += rows.length / width;
  };
  const logs = () => (log ? { names: allFree.map((f) => f.name), rows: log.slice(0, logged * width) } : null);

  // The exhaustive search, on the sliders' steps and then 10 times finer.
  let certain = true;
  for (const fine of [1, 10]) {
    const r = search_exhaustive(fine, !!log);
    logRows(search_take_log());
    if (r[0] !== 1) {
      certain = false;
      break;
    }
    evals += r[3];
    if (r[1] === 1) {
      const values = Array.from(r.subarray(4), (v, i) => +v.toFixed(decimals(allFree[i].step) + 3));
      return { found: true, values, evals, searched: allFree.map((f) => f.name), log: logs() };
    }
    certain = certain && r[2] === 1;
  }
  if (certain) return { found: false, values: null, evals, searched: allFree.map((f) => f.name), log: logs() };
  // a value's step in a pass `fine` times finer (whole numbers stay whole)
  const stepOf = (f, fine) => (f.whole ? f.step : f.step / fine);
  // the nearest value on a grid of the slider's steps divided by `fine`
  const snap = (f, raw, fine = 1) => {
    const step = stepOf(f, fine);
    return +(f.min + Math.round((raw - f.min) / step) * step).toFixed(decimals(f.step) + 2 + Math.round(Math.log10(fine)));
  };

  // Which values can change the score? (Solved values are never searched.)
  const searchable = allFree.filter((f) => !SOLVED.has(f.name));
  const v0 = searchable.map((f) => f.value);
  let seed = 12345;
  const random = () => ((seed = (seed * 1103515245 + 12345) % 2147483648) / 2147483648);
  const points = [v0];
  for (let k = 0; k < 60; k++) points.push(searchable.map((f) => snap(f, f.min + random() * (f.max - f.min))));
  const relevant = searchable.map(() => false);
  for (const point of points) {
    const b = scoreOf(searchable, point);
    searchable.forEach((f, i) => {
      if (relevant[i]) return;
      for (const k of MOVES) {
        for (const sign of [-1, 1]) {
          const raw = point[i] + sign * k * f.step;
          if (raw < f.min - 1e-9 || raw > f.max + 1e-9) continue;
          const vals = point.slice();
          vals[i] = snap(f, raw);
          if (scoreOf(searchable, vals) !== b) {
            relevant[i] = true;
            return;
          }
        }
      }
    });
  }

  // Best-first from `from`, on steps `fine` times finer than the sliders';
  // distances are from the starting values. Returns what works, or the
  // closest combinations it saw (up to NEAR, closest first, as `near`).
  // Stops after `budget` combinations, or `stall` without getting closer.
  const run = (free, budget, fine = 1, from = null, stall = STALL) => {
    const n = free.length;
    const w0 = free.map((f) => f.value);
    const dist = (vals) => vals.reduce((a, v, i) => a + Math.abs(v - w0[i]) / (free[i].max - free[i].min || 1), 0);
    const heap = new Heap();
    const s0 = from || w0;
    const seen = new Set([s0.join(",")]);
    const b0 = scoreOf(free, s0);
    let best = { f: b0 + dist(s0), b: b0, vals: s0 };
    heap.push(best);
    const near = [best]; // the closest few, by score then distance
    const better = (a, b) => a.b < b.b || (a.b === b.b && a.f < b.f);
    const until = evals + budget;
    let improved = evals;
    while (heap.size) {
      const node = heap.pop();
      if (node.b === 0) return { found: true, values: node.vals };
      for (let i = 0; i < n; i++) {
        const f = free[i];
        for (const k of MOVES) {
          for (const sign of [-1, 1]) {
            const raw = node.vals[i] + sign * k * stepOf(f, fine);
            if (raw < f.min - 1e-9 || raw > f.max + 1e-9) continue;
            const vals = node.vals.slice();
            vals[i] = snap(f, raw, fine);
            const key = vals.join(",");
            if (seen.has(key)) continue;
            seen.add(key);
            const b = scoreOf(free, vals);
            const child = { f: b + dist(vals), b, vals };
            heap.push(child);
            if (better(child, best)) {
              if (b < best.b) improved = evals;
              best = child;
            }
            if (near.length < NEAR || better(child, near[near.length - 1])) {
              near.push(child);
              near.sort((x, y) => (better(x, y) ? -1 : better(y, x) ? 1 : 0));
              if (near.length > NEAR) near.pop();
            }
            if (evals % 5000 === 0) self.postMessage({ type: "progress", id, evals });
          }
        }
      }
      if (evals > until || evals - improved > stall || performance.now() - start > 20000) break;
    }
    return { found: false, near: near.map((x) => x.vals) };
  };

  const anneal = (free, from, budget) => {
    const n = free.length;
    let cur = from.slice(), b = scoreOf(free, cur);
    let k = 0;
    for (let tries = 0; tries < budget && performance.now() - start < 20000; tries++) {
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
      if (tries % 5000 === 0) self.postMessage({ type: "progress", id, evals });
    }
    return { found: false };
  };
  // Walk each value back toward where it started while it still works.
  const polish = (free, vals, fine = 1) => {
    const w0 = free.map((f) => f.value);
    for (let changed = true; changed;) {
      changed = false;
      free.forEach((f, i) => {
        const step = stepOf(f, fine);
        while (Math.abs(vals[i] - w0[i]) > 1e-9) {
          const toward = vals.slice();
          const next = vals[i] + Math.sign(w0[i] - vals[i]) * step;
          toward[i] = Math.abs(next - w0[i]) < step / 2 || Math.sign(w0[i] - next) !== Math.sign(w0[i] - vals[i]) ? w0[i] : snap(f, next, fine);
          if (scoreOf(free, toward) !== 0) break;
          vals = toward;
          changed = true;
        }
      });
    }
    return vals;
  };

  const chosen = searchable.filter((_, i) => relevant[i]);
  const attempt = (free) => {
    // the sliders' steps, then 10 times finer from each of the closest;
    // quickly first, then the sliders' steps without stopping early
    let r, fine;
    for (const stall of [STALL, Infinity]) {
      fine = 1;
      r = run(free, 150000, 1, null, stall);
      for (const from of r.found ? [] : r.near) {
        fine = 10;
        r = run(free, 100000, fine, from);
        if (r.found) break;
      }
      if (r.found) break;
    }
    if (r.found) return polish(free, r.values, fine);
    r = anneal(free, free.map((f) => f.value), 400000);
    return r.found ? polish(free, r.values) : null;
  };
  // Nothing to search (only solved values free): the limits alone decide.
  let found = chosen.length ? attempt(chosen) : (scoreOf([], []) === 0 ? [] : null);
  let searched = chosen;
  if (!found && chosen.length < searchable.length) {
    found = attempt(searchable);
    searched = searchable;
  }
  if (!found) return { found: false, values: null, evals, searched: chosen.map((f) => f.name), log: logs() };
  // every free value, the settled ones included, as that combination sets them
  const values = Array.from(lookup(searched, found).subarray(1), (v, i) => +v.toFixed(decimals(allFree[i].step) + 3));
  return { found: true, values, evals, searched: chosen.map((f) => f.name), log: logs() };
}

self.onmessage = async (event) => {
  const { id, params, free, log } = event.data;
  await ready;
  try {
    const result = search(params, free, id, log);
    self.postMessage({ type: "done", id, ...result }, result.log ? [result.log.rows.buffer] : []);
  } catch (e) {
    self.postMessage({ type: "done", id, found: false, evals: 0, failed: String(e) });
  }
};
