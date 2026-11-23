// Every option's default value: what the page starts with, and what the API
// (api.js) fills in for options a program leaves out. null means "auto"
// (an optional value worked out from the others). Lengths in mm, angles in
// degrees.

export const DEFAULTS = {
  // gear set
  module: 2,
  pressure_angle: 20,
  helix_angle: 30,
  height: null,
  backlash: 0.2,
  clearance: null,
  fillet_radius: null,
  // gear
  kind: "planetary", // "spur", "ring", "planetary" or "bevel"
  teeth: 21,
  ring_teeth: 63,
  teeth2: 30,
  shaft_angle: 90,
  sun_teeth: 21,
  planet_teeth: 21,
  planets: 4,
  face_width: 12,
  width: 30,
  bore: 10,
  bore2: 8,
  backlash_inner: 0.2,
  backlash_outer: null,
  planet_bore: 8,
  rim: 140,
  hand: 1, // 1 counterclockwise, -1 clockwise
  end_bottom: null,
  end_top: null,
  layout: "assembled", // bevel pairs: "assembled" or "print"
  // STL smoothness
  max_deviation: 0.01,
  ds: 0.05,
  fillet_deviation: null,
  // STEP smoothness
  step_deviation: 0.0001,
};

// Options that may be null (auto).
export const OPTIONAL = new Set([
  "height", "clearance", "fillet_radius", "backlash_inner", "backlash_outer",
  "end_bottom", "end_top", "fillet_deviation",
]);
