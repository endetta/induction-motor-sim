/**
 * MotorGeometry.js
 * ---------------------------------------------------------------------------
 * Procedural construction of the three-phase induction machine in Three.js.
 * The motor axis is the world Z axis: the rotor spins about Z, bars sit at
 * angle θk in the XY plane, matching the physics model (θ from atan2(y, x)).
 *
 * Layout (SI metres, exaggerated air gap for visibility):
 *   shaft r 0.017 | rotor core r 0.055 + cage bars r 0.050 | air gap 0.007 |
 *   stator bore 0.062 (24 slots, depth 0.02) | yoke to 0.105 | casing 0.115
 *   stator core length 0.20 | casing length 0.26 | end shields ±0.14
 *
 * Winding: double-layer diamond lap, 24 coils for 24 slots at 5/6 pitch
 * (span 5 slots = 75° mech). Each coil is one closed tube with two straight
 * sides — the U side in the slot BOTTOM layer (R_BOT = 0.076), the V side in
 * the 5-slots-on TOP layer (R_TOP = 0.068) — joined by a diamond knuckle over
 * each end face (R_APEX ≈ 0.078 at Z_APEX = ±0.128). Every slot therefore
 * holds exactly two coil sides, one per layer.
 *
 * Overhang clearance: all 24 knuckles share one R_APEX family but start every
 * 15° while spanning 75°, so five of them overlap at any given azimuth (not
 * six). Each coil is fanned outward by (k mod 4)·APEX_STEP — measured against
 * the tube oracle at 1.91 mm of centreline on the worst pair, up from 0.48 mm
 * for the fan-out the file shipped with and 1.13 mm for an unfanned crown. That
 * is an improvement, not a solved interleave: a full 3.6 mm needs each knuckle
 * to carry its own axial rise (real diamond coils nest in z, not only in r),
 * which is a larger change than this pass. Ties and jumpers still inherit the
 * same four radii, so they space without a separate tier system.
 *
 * Circuit: the coils alone are closed loops (an open circuit). The copper that
 * makes it a machine is `circuit` — for each phase, two-coil pole groups
 * chained by hairpin ties at the rear, four groups linked by 75° jumpers, a
 * lead from each U1/V1/W1 pin down the housing to the group's U terminal, and
 * a tail from each finish back up to the box and the star point.
 *
 * Cutaway: a wedge (90° or 120°) is removed with two THREE.Plane clip planes
 * and material.clipIntersection = true — a point is clipped only when it is
 * inside BOTH planes, which is exactly the wedge {a1 < angle(p) < a2}.
 * The rotor, shaft and cage stay unclipped so the spinning cage stays visible.
 *
 * Rotor variants:
 *   'cage'  — skewed aluminium/copper bars shorted by two end rings (default)
 *   'wound' — 3-phase rotor winding brought out to three slip rings on the
 *             shaft with stationary carbon brushes (NO DC commutator segments).
 */

import * as THREE from 'three';
import { mergeGeometries } from 'three/examples/jsm/utils/BufferGeometryUtils.js';

// ---- principal dimensions (m) -----------------------------------------------
const SHAFT_R = 0.017;
const ROTOR_CORE_R = 0.055;
const BAR_R = 0.0065;
const BAR_CIRCLE_R = 0.050;
const END_RING_R = 0.050;
const BORE_R = 0.062;         // stator bore
const SLOT_DEPTH = 0.020;
const SLOT_HALF_ANG = 0.097;  // ≈ 5.5° → slot width ≈ 12 mm at the bore
const N_SLOTS = 24;           // stator slots (4-pole → 2 slots per phase belt)
const YOKE_R = 0.105;         // stator outer radius
const CORE_LEN = 0.20;        // stator/rotor core length
const CASING_R = 0.115;
const CASING_LEN = 0.26;
const N_FINS = 28;
const SKEW = 0.21;            // cage-bar skew (rad over the core, ≈ 12°)

/** 4-pole, 24-slot phase-belt sequence per pole pair (60°-el belts). */
const BELT_SEQ = ['A+', 'C-', 'B+', 'A-', 'C+', 'B-'];
const PHASE_OF = { 'A+': 'A', 'A-': 'A', 'B+': 'B', 'B-': 'B', 'C+': 'C', 'C-': 'C' };

// ---- winding geometry (double-layer diamond lap, 5/6 pitch) ------------------
const SPAN = 5;                            // coil pitch in slots
const R_BOT = BORE_R + 0.014;              // 0.076 — slot-bottom layer (U side)
const R_TOP = BORE_R + 0.006;              // 0.068 — slot-mouth layer (V side)
const R_APEX = BORE_R + 0.016;             // 0.078 — knuckle crown radius
const Z_CORE = CORE_LEN / 2;               // 0.100 — core end face
const Z_EXT = Z_CORE + 0.008;              // 0.108 — straight extension past the mouth
const Z_APEX = Z_CORE + 0.028;             // 0.128 — knuckle crown on the end face
// 3.6 mm copper, not 4.8: the spec's Z_APEX = Z_CORE + 0.028 = 0.128 plus a
// 0.0024 wire radius is 0.1304, which pokes 0.4 mm through the front shield
// inner face at +CASING_LEN/2 = +0.130 (housing, `disc.translate`). The two
// spec numbers are mutually inconsistent; the wire gives way so the crown
// stays inside the machine (0.128 + 0.0018 = 0.1298).
const WIRE_R = 0.0018;
// Per-tier knuckle fan-out. The 24 coils are congruent under 15° rotation and
// span 75°, so overlapping coils meet at arc parameters Δu = Δk/5, where a
// single shared crown radius leaves about 1.1 mm of centreline — less than the
// 3.6 mm two conductors need. Swept T × step against the tube oracle: T = 4
// with a 5.0 mm step is the best the fan-out alone can do (worst pair 1.91 mm,
// up from 0.48 mm), which is a real improvement but still not a full clearance.
const APEX_STEP = 0.0050;

export const PHASE_COLORS = {
  A: 0xd94848, // merah
  B: 0xb8860b, // kuning gelap (kontras di latar terang)
  C: 0x2b62b8, // biru
};

/** Procedural lamination texture: thin silicon-steel plate stack, 2×16 px
 *  canvas repeated along the core axis. No external image to load. */
function laminationTexture(repeatY = 40) {
  const c = document.createElement('canvas');
  c.width = 2; c.height = 16;
  const ctx = c.getContext('2d');
  ctx.fillStyle = '#4a4e52';                    // silicon-steel base
  ctx.fillRect(0, 0, 2, 16);
  ctx.fillStyle = '#1e2022';                    // inter-lamination insulation line
  for (let y = 0; y < 16; y += 2) ctx.fillRect(0, y, 2, 1);
  const tex = new THREE.CanvasTexture(c);
  tex.wrapS = tex.wrapT = THREE.RepeatWrapping;
  tex.repeat.set(1, repeatY);
  tex.colorSpace = THREE.SRGBColorSpace;
  return tex;
}

/** Materials shared across the build. Clipped ones receive the cutaway planes. */
function makeMaterials() {
  const std = (color, metalness, roughness, extra = {}) =>
    new THREE.MeshStandardMaterial({ color, metalness, roughness, ...extra });

  return {
    // Painted/machined parts: LOW metalness — with no env map a metalness-1
    // surface only reflects the lights and reads as flat black.
    casing:  std(0x9aa3ae, 0.25, 0.45),
    fin:     std(0x8b95a2, 0.25, 0.50),
    foot:    std(0x6b7480, 0.30, 0.55),
    shield:  std(0xa7b0ba, 0.30, 0.40),
    termBox: std(0x7d8794, 0.20, 0.50),    pin: std(0xd4af37, 1.0, 0.3),
    // stator vs rotor laminations are given clearly different albedo/metalness
    // so the 7 mm air gap (rotor R 0.055 / bore R 0.062) reads as a hard edge
    core:    std(0x8c96a4, 0.75, 0.40, { map: laminationTexture(60) }),   // laminated stator
    rotorCore: std(0x525961, 0.50, 0.60, { map: laminationTexture(45) }),
    bar:     std(0xcf8440, 0.90, 0.30),                      // copper cage bar
    ring:    std(0xe09a52, 0.90, 0.28),                      // copper end ring
    shaft:   std(0xc9d1da, 0.90, 0.20),
    slipRing: std(0xd9b25a, 0.90, 0.25),
    brush:   std(0x2a2d33, 0.10, 0.80),
    // winding conductors — emissive so current pulsing can glow
    windingA: std(PHASE_COLORS.A, 0.30, 0.45, { emissive: PHASE_COLORS.A, emissiveIntensity: 0.15 }),
    windingB: std(PHASE_COLORS.B, 0.30, 0.45, { emissive: PHASE_COLORS.B, emissiveIntensity: 0.15 }),
    windingC: std(PHASE_COLORS.C, 0.30, 0.45, { emissive: PHASE_COLORS.C, emissiveIntensity: 0.15 }),
  };
}

/** Circle polygon points, CCW, `n` segments. */
function circlePts(r, n = 96, cx = 0, cy = 0) {
  const pts = [];
  for (let i = 0; i < n; i++) {
    const a = (2 * Math.PI * i) / n;
    pts.push(new THREE.Vector2(cx + r * Math.cos(a), cy + r * Math.sin(a)));
  }
  return pts;
}

/**
 * Stator cross-section: outer circle with a slotted bore hole.
 * The hole is the bore disc plus N radial slot notches; built as a CW polygon
 * (opposite winding to the CCW outer shape) so triangulation succeeds.
 */
function statorShape() {
  const shape = new THREE.Shape(circlePts(YOKE_R, 128));
  const hole = [];
  const N = N_SLOTS, pitch = (2 * Math.PI) / N;
  for (let k = 0; k < N; k++) {
    const psi = k * pitch;
    // slot notch spans [psi − β, psi + β] radially outward from the bore
    for (let i = 0; i <= 3; i++) {   // up the leading edge (approximated)
      const a = psi - SLOT_HALF_ANG;
      hole.push(new THREE.Vector2((BORE_R + (SLOT_DEPTH * i) / 3) * Math.cos(a),
                                  (BORE_R + (SLOT_DEPTH * i) / 3) * Math.sin(a)));
    }
    for (let i = 1; i <= 3; i++) {   // across the notch bottom
      const a = psi - SLOT_HALF_ANG + (2 * SLOT_HALF_ANG * i) / 3;
      hole.push(new THREE.Vector2((BORE_R + SLOT_DEPTH) * Math.cos(a),
                                  (BORE_R + SLOT_DEPTH) * Math.sin(a)));
    }
    for (let i = 3; i >= 0; i--) {   // back down the trailing edge
      const a = psi + SLOT_HALF_ANG;
      hole.push(new THREE.Vector2((BORE_R + (SLOT_DEPTH * i) / 3) * Math.cos(a),
                                  (BORE_R + (SLOT_DEPTH * i) / 3) * Math.sin(a)));
    }
    // bore arc between this slot's trailing edge and the next slot's leading edge
    for (let i = 1; i < 4; i++) {
      const a = psi + SLOT_HALF_ANG + ((pitch - 2 * SLOT_HALF_ANG) * i) / 4;
      hole.push(new THREE.Vector2(BORE_R * Math.cos(a), BORE_R * Math.sin(a)));
    }
  }
  hole.reverse(); // CW winding for the hole
  shape.holes.push(new THREE.Path(hole));
  return shape;
}

/** Extruded stator core with slots, centred on z = 0. */
function statorCoreGeometry() {
  const g = new THREE.ExtrudeGeometry(statorShape(), {
    depth: CORE_LEN, bevelEnabled: false, curveSegments: 4,
  });
  g.translate(0, 0, -CORE_LEN / 2);
  g.computeVertexNormals();
  return g;
}

// ---- diamond lap coil path ---------------------------------------------------
/**
 * The U side runs the slot bottom layer from the rear mouth to the front mouth
 * and out along the axis; the V side mirrors it in the slot-mouth layer, offset
 * by 75° (5 slots). Every group below is a rotation about the motor axis, so
 * each S-segment is a straight line on a cylinder and each knuckle is a plain
 * circular arc — the "rigid corner" look of a formed diamond coil, with no
 * Catmull-Rom overshoot into the iron.
 */
function coilPathParts(k) {
  const thA = (2 * Math.PI * k) / N_SLOTS;
  const thB = (2 * Math.PI * (k + SPAN)) / N_SLOTS;          // +75° mech
  const thMid = thA + (SPAN * Math.PI) / N_SLOTS;            // knuckle apex
  // Fan the knuckle crown outward so the overlapping overhangs nest: the 24
  // coils are congruent under 15° rotation while each spans 75°, so two coils
  // share every azimuth at arc parameters Δu = Δk/5 and would otherwise meet
  // at near-identical radius (measured ~1.1 mm at Δk = 1).
  const ra = R_APEX + (k % 4) * APEX_STEP;
  const P = (r, th, z) => new THREE.Vector3(r * Math.cos(th), r * Math.sin(th), z);
  // Ten rigid corners: U side (slot bottom layer) rear→front→out, front apex,
  // V side (slot-mouth layer) front→rear→out, rear apex, back to the U mouth.
  // Deliberately sparse — the three collinear points per straight side are what
  // keep the closed centripetal CR spline from overshooting into the iron
  // (densifying the knuckle arcs pushes the crown ~3.4 mm proud of Z_APEX).
  const pts = [
    P(R_BOT, thA, -Z_CORE), P(R_BOT, thA, Z_CORE), P(R_BOT, thA, Z_EXT),
    P(ra, thMid, Z_APEX), P(R_TOP, thB, Z_EXT), P(R_TOP, thB, Z_CORE),
    P(R_TOP, thB, -Z_CORE), P(R_TOP, thB, -Z_EXT), P(ra, thMid, -Z_APEX),
    P(R_BOT, thA, -Z_EXT),
  ];
  return { ra, pts };
}

/** One complete stator coil as a single closed tube (see coilPathParts). */
function diamondCoilGeometry(k) {
  const { pts } = coilPathParts(k);
  const curve = new THREE.CatmullRomCurve3(pts, true, 'centripetal', 0.25);
  return new THREE.TubeGeometry(curve, 72, WIRE_R, 8, true);
}

/** Helical skewed cage bar at slot angle θk (tube along a helix curve). */
function cageBarGeometry(k) {
  const psi = (2 * Math.PI * k) / N_SLOTS;
  const pts = [];
  const SEG = 8;
  for (let i = 0; i <= SEG; i++) {
    const z = -CORE_LEN / 2 + (CORE_LEN * i) / SEG;
    const a = psi + (SKEW * i) / SEG;
    pts.push(new THREE.Vector3(BAR_CIRCLE_R * Math.cos(a), BAR_CIRCLE_R * Math.sin(a), z));
  }
  return new THREE.TubeGeometry(new THREE.CatmullRomCurve3(pts), 16, BAR_R, 6, false);
}

/** Straight rotor winding bar (wound-rotor variant), colored per phase. */
function rotorWindingBarGeometry(k) {
  const psi = (2 * Math.PI * k) / N_SLOTS;
  const g = new THREE.CylinderGeometry(BAR_R, BAR_R, CORE_LEN * 0.97, 6);
  g.rotateX(Math.PI / 2);
  g.translate(BAR_CIRCLE_R * Math.cos(psi), BAR_CIRCLE_R * Math.sin(psi), 0);
  return g;
}

/**
 * Build the complete motor.
 * Returns handles for animation: rotor group, per-phase winding materials,
 * cutaway control and explode control.
 */
export function buildMotor(rotorType = 'cage') {
  const mats = makeMaterials();
  const root = new THREE.Group();
  const clipMats = [mats.casing, mats.fin, mats.foot, mats.shield, mats.termBox,
                    mats.core, mats.windingA, mats.windingB, mats.windingC];

  // ---- stator (core + windings) -------------------------------------------
  const stator = new THREE.Group();
  const coreMesh = new THREE.Mesh(statorCoreGeometry(), mats.core);
  stator.add(coreMesh);

  // Stator winding: 24 diamond coils for 24 slots. Slot 2k takes the U side of
  // coil 2k⁻ and the V side of coil 2k⁺ — two per slot, one per layer.
  const phaseMats = { A: mats.windingA, B: mats.windingB, C: mats.windingC };
  const slotPhase = [];                        // slot → 'A' | 'B' | 'C'
  for (let s = 0; s < N_SLOTS; s++) {
    slotPhase.push(PHASE_OF[BELT_SEQ[Math.floor(s / 2) % 6]]);
  }
  for (let k = 0; k < N_SLOTS; k++) {
    stator.add(new THREE.Mesh(diamondCoilGeometry(k), phaseMats[slotPhase[k]]));
  }

  // ---- winding circuit: leads, pole-group ties, star point -------------------
  // The coils above are closed loops — electrically an open circuit. What makes
  // it a machine is the copper that joins them: two-coil pole groups chained by
  // rear hairpins, four groups per phase linked by 75° jumpers, six leads in
  // from the terminal box, and a neutral ring at the non-drive end for Y.
  // All of it lives in `stator`, so it explodes and hides with the winding.
  const circuit = new THREE.Group();
  const P3 = (r, th, z) => new THREE.Vector3(r * Math.cos(th), r * Math.sin(th), z);
  const slotTh = (kk) => (2 * Math.PI * (((kk % N_SLOTS) + N_SLOTS) % N_SLOTS)) / N_SLOTS;
  const wire = (pts, r, mat, seg = 32) => new THREE.Mesh(
    new THREE.TubeGeometry(
      new THREE.CatmullRomCurve3(pts, false, 'centripetal'), seg, r, 6, false), mat);

  // Per-phase pole groups: the chain enters coil k at its U side and leaves at
  // its V side, so the sequence k = 2a, 2a+1, 2a+6, 2a+7, 2a+12, 2a+13, 2a+18,
  // 2a+19 walks all 8 coils of the phase in pole order with one turn of span
  // between groups and one of opposite span within them.
  const groups = { A: [], B: [], C: [] };
  for (let g = 0; g < 6; g++) {
    const a = 2 * g;
    const ph = slotPhase[a];
    groups[ph].push({ left: a, right: a + 1 });
  }
  const entryI = { A: 0, B: 4, C: 2 };             // coil id of each phase's U1 terminal
  const headOf = (ph) => {
    const ei = entryI[ph];
    return groups[ph].find((G) =>
      (G.left % N_SLOTS) === (ei % N_SLOTS)
      || (G.right % N_SLOTS) === (ei % N_SLOTS)
      || (G.left + SPAN) % N_SLOTS === (ei % N_SLOTS)
      || (G.right + SPAN) % N_SLOTS === (ei % N_SLOTS)) || groups[ph][0];
  };

  /** A tie between two coil-side ends: the path leaves along +axial, wraps the
   *  end face at `rBend`, and re-enters along −axial in the other side.
   *  zA/zB mirror on the rear (±Z_EXT). */
  const tieBetween = (a, ra, zA, b, rb, zB, rBend) => {
    const zc = (zA + zB) / 2, az = (a + b) / 2;
    return [
      P3(ra, a - 0.19, zA),
      P3(ra, a - 0.05, zA + Math.sign(zc) * (Z_APEX - Z_EXT) * 0.75),
      P3(rBend, az - (b - a) * 0.5, zc),
      P3(rBend, az, zc),
      P3(rBend, az + (b - a) * 0.5, zc),
      P3(rb, b + 0.05, zB + Math.sign(zc) * (Z_APEX - Z_EXT) * 0.75),
      P3(rb, b + 0.19, zB),
    ];
  };

  const PIN_Y = CASING_R + 0.064;                    // top stud row (matches pinGeos)
  const pinOf = (c, row) => new THREE.Vector3(-0.018 + c * 0.018, PIN_Y, 0.038 + row * 0.034);
  const RING_R = 0.068;
  const RING_Z = -(CORE_LEN / 2 + 0.0265);           // 26.5 mm past the rear core face

  for (const [c, ph] of [[0, 'A'], [1, 'B'], [2, 'C']]) {
    const mat = phaseMats[ph];
    const list = groups[ph];
    const head = headOf(ph);
    const headId = (head.left % N_SLOTS) === (entryI[ph] % N_SLOTS) ? head.left : head.right;

    // --- U1 / V1 / W1 — pin, down the housing, onto the phase head ---------
    // The lead leaves the box at the gland, follows the casing down over the
    // fins to the non-drive end, then turns inboard and lands on the head
    // coil's rear terminal — the way a real supply cable is dressed.
    const thLand = slotTh(headId);
    const rLead = 0.134 + c * 0.003;
    circuit.add(wire([
      pinOf(c, 0),
      new THREE.Vector3(pinOf(c, 0).x, PIN_Y - 0.018, 0.024),
      P3(0.128, Math.atan2(0.024, pinOf(c, 0).x), 0.012),
      P3(rLead, thLand + 0.20, -0.055),
      P3(rLead, thLand + 0.06, -0.116),
      P3(R_BOT + 0.010, thLand + 0.02, -Z_EXT - 0.004),
      P3(R_BOT, thLand, -Z_EXT),
    ], 0.0026, mat, 44));

    // --- U2 / V2 / W2 — finish of the last group, back to the box + star ---
    // The finish is a rear terminal under the end face; the tail picks it up
    // there, runs outboard to the casing, climbs to the box and terminates on
    // the rear stud row. All three tails converge on the neutral ring.
    const last = list[list.length - 1];
    const thTail = slotTh((headId === last.left ? last.right : last.left) + SPAN);
    const rTail = 0.130 + c * 0.0022;
    const tailPts = [
      P3(R_BOT - 0.008, thTail - 0.02, -Z_EXT - 0.004),
      P3(R_BOT, thTail, -Z_EXT),
      P3(0.086, thTail, -Z_EXT - 0.006),
      P3(0.100, thTail + 0.06, -0.088),
      P3(0.110, thTail + 0.30, -0.020),
      P3(rTail, thTail + 0.48, 0.052),
      P3(rTail, thTail + 0.62, 0.108),
      new THREE.Vector3(pinOf(c, 1).x, PIN_Y + 0.014, 0.096),
      pinOf(c, 1),
    ];
    // star point: the three tails are one physical node (see `neutral` below)
    circuit.add(wire([
      pinOf(c, 1),
      new THREE.Vector3(pinOf(c, 1).x, PIN_Y - 0.010, 0.062),
      P3(RING_R + 0.006, Math.atan2(pinOf(c, 1).y, pinOf(c, 1).x), RING_Z + 0.030),
      P3(RING_R, Math.atan2(pinOf(c, 1).y, pinOf(c, 1).x), RING_Z),
    ], 0.0022, mat, 20));
    circuit.add(wire(tailPts, 0.0022, mat, 46));

    // --- within-group ties: V(k) → U(k+1) at the rear (15° hairpin) -------
    for (const G of list) {
      const rA = coilPathParts(G.left).ra, rB = coilPathParts(G.right).ra;
      circuit.add(wire(tieBetween(
        slotTh(G.left) + (2 * Math.PI * SPAN) / N_SLOTS, R_TOP,
        -Z_EXT, slotTh(G.right), R_BOT, -Z_EXT, (rA + rB) / 2 + 0.002), 0.0024, mat, 24));
    }

    // --- between-group jumpers: rear V(k+1) → U of the next group ---------
    // Both ends sit on the rear end face 75° apart; the jumper repeats the
    // one-span bridge of the winding itself, one tier higher.
    for (let i = 0; i < list.length - 1; i++) {
      const from = list[i].right, to = list[i + 1].left;
      const rA = coilPathParts(from).ra, rB = coilPathParts(to).ra;
      circuit.add(wire(tieBetween(
        slotTh(from) + (2 * Math.PI * SPAN) / N_SLOTS, R_TOP, -Z_EXT,
        slotTh(to), R_BOT, -Z_EXT, Math.max(rA, rB) + 0.007), 0.0026, mat, 34));
    }

    // --- seam: the last group closes onto the head coil at the rear -------
    const seamFrom = list[list.length - 1].right;
    circuit.add(wire(tieBetween(
      slotTh(seamFrom) + (2 * Math.PI * SPAN) / N_SLOTS, R_TOP, -Z_EXT,
      slotTh(headId), R_BOT, -Z_EXT, (coilPathParts(seamFrom).ra + coilPathParts(headId).ra) / 2 + 0.004),
      0.0024, mat, 34));
  }
  stator.add(circuit);

  // --- star point: neutral ring at the non-drive end ------------------------
  // Ties the three U2/V2/W2 tails into one physical node. Shown for Bintang
  // (Y); hidden for Δ, where the jumper bars at the terminal box are what
  // expresses the connection instead.
  const neutral = new THREE.Group();
  const neutralMat = new THREE.MeshStandardMaterial({
    color: 0xb08a52, metalness: 0.9, roughness: 0.30,
    emissive: 0xb08a52, emissiveIntensity: 0.08,
  });
  clipMats.push(neutralMat);
  const ringGeo = new THREE.TorusGeometry(RING_R, 0.0030, 6, 64);
  ringGeo.translate(0, 0, RING_Z);
  neutral.add(new THREE.Mesh(ringGeo, neutralMat));
  stator.add(neutral);
  root.add(stator);

  // ---- rotor assembly (spins about Z) --------------------------------------
  const rotor = new THREE.Group();
  rotor.add(new THREE.Mesh(new THREE.CylinderGeometry(ROTOR_CORE_R, ROTOR_CORE_R, CORE_LEN * 0.99, 48),
                           mats.rotorCore).rotateX(Math.PI / 2).translateZ(0));

  const rotorExtras = { slipRings: new THREE.Group(), brushes: null };
  if (rotorType === 'cage') {
    const bars = mergeGeometries(Array.from({ length: N_SLOTS }, (_, k) => cageBarGeometry(k)));
    rotor.add(new THREE.Mesh(bars, mats.bar));
    for (const side of [+1, -1]) {
      const ringGeo = new THREE.TorusGeometry(END_RING_R, 0.011, 8, 48);
      ringGeo.translate(0, 0, side * (CORE_LEN / 2 + 0.012));
      rotor.add(new THREE.Mesh(ringGeo, mats.ring));
    }
  } else {
    // Wound rotor: straight 3-phase winding, colored per phase belt
    // (phase = belt index mod 3, mirroring the stator belt pattern).
    for (let k = 0; k < N_SLOTS; k++) {
      const ph = ['A', 'B', 'C'][k % 3];
      rotor.add(new THREE.Mesh(rotorWindingBarGeometry(k), phaseMats[ph]));
    }
    for (const side of [+1, -1]) {
      for (let ph = 0; ph < 3; ph++) {
        // end-turn arc connecting this phase's bars at each end (simplified hoop)
        const g = new THREE.TorusGeometry(BAR_CIRCLE_R, 0.006, 6, 30, (2 * Math.PI * 8) / N_SLOTS);
        g.rotateZ((2 * Math.PI * ph * 8) / N_SLOTS);
        g.translate(0, 0, side * (CORE_LEN / 2 + 0.010));
        rotor.add(new THREE.Mesh(g, phaseMats[['A', 'B', 'C'][ph]]));
      }
    }
    // three slip rings on the shaft (rear end) — rotating with the rotor
    for (let ph = 0; ph < 3; ph++) {
      const g = new THREE.CylinderGeometry(0.026, 0.026, 0.014, 32);
      g.rotateX(Math.PI / 2);
      g.translate(0, 0, -CORE_LEN / 2 - 0.05 - ph * 0.022);
      const mesh = new THREE.Mesh(g, mats.slipRing);
      rotor.add(mesh);
      rotorExtras.slipRings.add(mesh);
    }
  }
  // shaft through the whole machine — with a keyway (power-transmission groove)
  const shaftGeo = new THREE.CylinderGeometry(SHAFT_R, SHAFT_R, CASING_LEN + 0.18, 24);
  shaftGeo.rotateX(Math.PI / 2);
  rotor.add(new THREE.Mesh(shaftGeo, mats.shaft));
  const keyLen = CASING_LEN + 0.06;               // keyway: shallow rectangular groove
  const keyway = new THREE.Mesh(
    new THREE.BoxGeometry(0.006, 0.004, keyLen),
    new THREE.MeshStandardMaterial({ color: 0x3a4048, metalness: 0.6, roughness: 0.6 }),
  );
  keyway.position.set(0, SHAFT_R - 0.001, 0);
  rotor.add(keyway);
  root.add(rotor);

  // ---- housing: casing shell, fins, feet, end shields, terminal box --------
  const housing = new THREE.Group();
  const shell = new THREE.ExtrudeGeometry((() => {
    const s = new THREE.Shape(circlePts(CASING_R, 96));
    s.holes.push(new THREE.Path(circlePts(CASING_R - 0.008, 96).reverse()));
    return s;
  })(), { depth: CASING_LEN, bevelEnabled: false });
  shell.translate(0, 0, -CASING_LEN / 2);
  housing.add(new THREE.Mesh(shell, mats.casing));

  // longitudinal cooling fins (merged)
  const finGeos = [];
  for (let k = 0; k < N_FINS; k++) {
    const a = (2 * Math.PI * k) / N_FINS;
    const g = new THREE.BoxGeometry(0.008, 0.016, CASING_LEN * 0.92);
    g.translate(0, CASING_R + 0.006, 0);
    g.rotateZ(a);
    finGeos.push(g);
  }
  housing.add(new THREE.Mesh(mergeGeometries(finGeos), mats.fin));

  // mounting feet + base rail
  const footGeos = [];
  for (const sx of [+1, -1]) {
    const g = new THREE.BoxGeometry(0.05, 0.022, CASING_LEN * 0.55);
    g.translate(sx * 0.055, -CASING_R - 0.012, 0);
    footGeos.push(g);
  }
  const rail = new THREE.BoxGeometry(0.17, 0.012, CASING_LEN * 0.55);
  rail.translate(0, -CASING_R - 0.028, 0);
  footGeos.push(rail);
  housing.add(new THREE.Mesh(mergeGeometries(footGeos), mats.foot));

  // end shields with bearing bosses
  const shieldFront = new THREE.Group();
  const shieldRear = new THREE.Group();
  for (const [grp, side] of [[shieldFront, +1], [shieldRear, -1]]) {
    const disc = new THREE.ExtrudeGeometry((() => {
      const s = new THREE.Shape(circlePts(CASING_R - 0.001, 72));
      s.holes.push(new THREE.Path(circlePts(0.020, 32).reverse()));
      return s;
    })(), { depth: 0.016, bevelEnabled: false });
    disc.translate(0, 0, side * (CASING_LEN / 2) - (side > 0 ? 0 : 0.016));
    grp.add(new THREE.Mesh(disc, mats.shield));
    const boss = new THREE.CylinderGeometry(0.034, 0.034, 0.026, 24);
    boss.rotateX(Math.PI / 2);
    boss.translate(0, 0, side * (CASING_LEN / 2 + 0.012));
    grp.add(new THREE.Mesh(boss, mats.shield));
  }
  // End shields belong to the outer housing, not to root. On root they survived
  // the "Casing luar" toggle (which only hides `housing`) and stayed behind as
  // two discs floating in mid-air with everything else gone.
  housing.add(shieldFront, shieldRear);
  root.add(housing);

  // terminal box with 6 pins (U1 V1 W1 / U2 V2 W2), top of casing.
  // Supply cables are CHILDREN of termBox: they ride along when the box lifts
  // in exploded view, and their far ends land on the U1/V1/W1 bolts.
  const termBox = new THREE.Group();
  // The shell must actually ENCLOSE the six studs and the Star/Delta jumper
  // plate. At the old 50 mm height the studs (top 0.186) and the brass plate
  // (top 0.193) punched through a lid sitting at 0.171, so the jumper read as
  // floating hardware hovering above the box instead of terminated inside it.
  const BOX_Y0 = CASING_R + 0.003;                 // 0.118 — box bottom
  const BOX_H = 0.085;                             // → box top 0.203
  // Shell is a separate see-through material from the lid: the shell stays
  // faintly visible always (so the box still reads as a box), while the lid is
  // the part that lifts. Both are depthWrite:false so the brass plate and the
  // studs inside are never depth-occluded by the enclosure.
  const shellMat = new THREE.MeshStandardMaterial({
    color: 0x7d8794, metalness: 0.2, roughness: 0.5,
    transparent: true, opacity: 0.22, depthWrite: false,
  });
  const box = new THREE.Mesh(new THREE.BoxGeometry(0.075, BOX_H, 0.062), shellMat);
  box.position.set(0, BOX_Y0 + BOX_H / 2, 0.055);
  termBox.add(box);
  clipMats.push(shellMat);          // keep the box inside the cutaway wedge
  // semi-transparent lid — the Star/Delta jumper plate stays visible inside
  const lidMat = new THREE.MeshStandardMaterial({
    color: 0x7d8794, metalness: 0.2, roughness: 0.5,
    transparent: true, opacity: 0.30, depthWrite: false,
  });
  const lid = new THREE.Mesh(new THREE.BoxGeometry(0.083, 0.008, 0.068), lidMat);
  lid.position.set(0, BOX_Y0 + BOX_H + 0.004, 0.055);
  termBox.add(lid);
  // cable glands on the box side wall (three rubber grommets the leads pass through)
  const glandGeos = [];
  for (let i = 0; i < 3; i++) {
    const g = new THREE.CylinderGeometry(0.006, 0.006, 0.012, 10);
    g.rotateX(Math.PI / 2);
    g.translate(-0.018 + i * 0.018, CASING_R + 0.026, 0.092);
    glandGeos.push(g);
  }
  termBox.add(new THREE.Mesh(mergeGeometries(glandGeos), mats.brush));
  const pinGeos = [];
  for (let r = 0; r < 2; r++) for (let c = 0; c < 3; c++) {
    const g = new THREE.CylinderGeometry(0.0035, 0.0035, 0.014, 10);
    g.translate(-0.018 + c * 0.018, CASING_R + 0.064, 0.038 + r * 0.034);
    pinGeos.push(g);
  }
  termBox.add(new THREE.Mesh(mergeGeometries(pinGeos), mats.pin));
  housing.add(termBox);

  // ---- supply cables (three colored leads into the terminal box) -----------
  // Built INSIDE the geometry module so they parent to termBox:
  // exploded view carries them with the box; ends land on U1/V1/W1 bolts.
  const cables = new THREE.Group();
  for (const [i, color] of [[0, 0xd94848], [1, 0xb8860b], [2, 0x2b62b8]]) {
    const mat = new THREE.MeshStandardMaterial({ color, roughness: 0.6, metalness: 0.1 });
    const x = -0.018 + i * 0.018;                  // U1 / V1 / W1 column
    // lead: STARTS at the gland on the box wall and curves out, so the cable
    // grows out of the box rather than dangling toward it from nowhere.
    const lead = new THREE.Mesh(new THREE.TubeGeometry(new THREE.CatmullRomCurve3([
      new THREE.Vector3(x, CASING_R + 0.026, 0.098),
      new THREE.Vector3(x - 0.055 + i * 0.03, CASING_R + 0.075, 0.135),
      new THREE.Vector3(x - 0.10 + i * 0.06, 0.24, 0.20),
      new THREE.Vector3(x - 0.02 + i * 0.02, 0.30, 0.16),
    ]), 24, 0.004, 6), mat);
    cables.add(lead);
    // short stub inside the box: gland → pin (U1/V1/W1 top row)
    const stub = new THREE.Mesh(new THREE.TubeGeometry(new THREE.CatmullRomCurve3([
      new THREE.Vector3(x, CASING_R + 0.026, 0.086),
      new THREE.Vector3(x, CASING_R + 0.046, 0.062),
      new THREE.Vector3(x, CASING_R + 0.064, 0.038),
    ]), 12, 0.0032, 6), mat);
    cables.add(stub);
  }
  termBox.add(cables);

  // ---- fan cowl + centrifugal fan on the non-drive end (rear, −Z) -----------
  const cowl = new THREE.Group();
  const shroud = new THREE.Mesh(
    new THREE.CylinderGeometry(0.095, CASING_R * 0.92, 0.075, 32, 1, true),
    mats.fin,
  );
  shroud.rotateX(Math.PI / 2);
  shroud.position.z = -CASING_LEN / 2 - 0.048;
  cowl.add(shroud);
  const cowlBack = new THREE.Mesh(new THREE.SphereGeometry(0.095, 24, 12, 0, Math.PI * 2, 0, Math.PI / 2), mats.fin);
  cowlBack.rotateX(-Math.PI / 2);
  cowlBack.position.z = -CASING_LEN / 2 - 0.085;
  cowl.add(cowlBack);
  // fan blades: 7 flat vanes angled off the rear shaft hub
  const bladeGeos = [];
  for (let k = 0; k < 7; k++) {
    const g = new THREE.BoxGeometry(0.030, 0.062, 0.004);
    g.rotateX(Math.PI / 6);
    g.translate(0, 0.048, 0);
    g.rotateZ((2 * Math.PI * k) / 7);
    bladeGeos.push(g);
  }
  const fan = new THREE.Mesh(mergeGeometries(bladeGeos), mats.foot);
  fan.position.z = -CASING_LEN / 2 - 0.060;
  rotor.add(fan);                                  // spins with the rotor
  housing.add(cowl);
  // ---- brushes for the wound rotor (stationary → on the rear shield) -------
  if (rotorType === 'wound') {
    const brushes = new THREE.Group();
    for (let ph = 0; ph < 3; ph++) {
      const z = -CORE_LEN / 2 - 0.05 - ph * 0.022;
      const holder = new THREE.Mesh(new THREE.BoxGeometry(0.012, 0.03, 0.016), mats.termBox);
      holder.position.set(0.033, 0, z);
      const carbon = new THREE.Mesh(new THREE.BoxGeometry(0.008, 0.010, 0.010), mats.brush);
      carbon.position.set(0.026, 0, z);
      brushes.add(holder, carbon);
    }
    shieldRear.add(brushes);
  }

  // ---- keyway (parallel key slot) on the drive-end shaft --------------------
  // A shallow rectangular key seat visible next to the front shield —
  // proportional to the shaft (6 × 6 mm key on a 34 mm shaft end).
  const key = new THREE.Mesh(
    new THREE.BoxGeometry(0.006, 0.006, 0.045),
    new THREE.MeshStandardMaterial({ color: 0x9aa3ae, metalness: 0.85, roughness: 0.35 }),
  );
  key.position.set(0, SHAFT_R - 0.001, CASING_LEN / 2 + 0.045);
  rotor.add(key);

  // Brass jumper plates — Star (Y): one bar bridging the top row (U2-V2-W2);
  // Delta (Δ): three vertical bars pin-row to pin-row. Swappable at runtime.
  const jumperBrass = new THREE.MeshStandardMaterial({ color: 0xd4af37, metalness: 1.0, roughness: 0.3 });
  function jumperPlates(mode) {
    const geos = [];
    if (mode === 'delta') {
      // vertical bridges: pin row to pin row (front z=0.038 to rear z=0.072)
      for (let c = 0; c < 3; c++) {
        const g = new THREE.BoxGeometry(0.010, 0.028, 0.005);
        g.translate(-0.018 + c * 0.018, CASING_R + 0.064, 0.055);
        geos.push(g);
      }
    } else {
      // horizontal bar bridging the rear row (U2-V2-W2)
      const g = new THREE.BoxGeometry(0.052, 0.005, 0.008);
      g.translate(0, CASING_R + 0.073, 0.072);
      geos.push(g);
    }
    return new THREE.Mesh(mergeGeometries(geos), jumperBrass);
  }
  const jumper = jumperPlates('star');
  termBox.add(jumper);

  // ---- cutaway clipping -----------------------------------------------------
  const planeA = new THREE.Plane(new THREE.Vector3(1, 0, 0), 0);
  const planeB = new THREE.Plane(new THREE.Vector3(0, 1, 0), 0);

  /** mode: 'none' | '90' | '120' — wedge centred toward the default camera. */
  function setCutaway(mode) {
    let planes = null;
    if (mode !== 'none') {
      const span = mode === '120' ? (120 * Math.PI) / 180 : (90 * Math.PI) / 180;
      const centre = (38 * Math.PI) / 180;           // toward default camera az
      const a1 = centre - span / 2, a2 = centre + span / 2;
      // clip where n1·p>0 AND n2·p>0 (the wedge), via normals −n1 / −n2
      planeA.normal.set(Math.sin(a1), -Math.cos(a1), 0);
      planeB.normal.set(-Math.sin(a2), Math.cos(a2), 0);
      planes = [planeA, planeB];
    }
    for (const m of clipMats) {
      m.clippingPlanes = planes;
      m.clipIntersection = true;   // clip only the wedge (intersection)
      m.needsUpdate = true;
    }
  }
  setCutaway('none');

  // ---- exploded view ---------------------------------------------------------
  function setExplode(f) {
    shieldFront.position.z = f * 0.16;
    shieldRear.position.z = -f * 0.16;
    termBox.position.y = f * 0.10;      // cables are termBox children — they ride along
    stator.position.z = f * 0.04;
  }
  setExplode(0);

  // ---- visibility / opacity control ------------------------------------------
  /** Terminal-box lid: true = lifted off (hidden), so the brass jumper plate and
   *  the six U1/V1/W1 – U2/V2/W2 studs are unobstructed. */
  function setTerminalLid(open) {
    lid.visible = !open;
    shellMat.opacity = open ? 0.06 : 0.22;   // near-invisible shell when opened
  }
  /** Front end shield: false removes the disc + bearing boss so the air gap,
   *  the rotor end ring and the stator slot section are visible head-on. */
  function setFrontShieldVisible(visible) {
    shieldFront.visible = visible;
  }
  /** Both end shields together — casing on, covers off. */
  function setShieldsVisible(visible) {
    shieldFront.visible = visible;
    shieldRear.visible = visible;
  }
  /** X-ray / ghosting: fades the casing, fins and stator core while keeping the
   *  outer silhouette (the object stays depth-sorted, it just stops occluding).
   *  opacity ≥ 0.98 restores fully solid rendering. */
  function setGhostOpacity(opacity) {
    const ghost = opacity < 0.98;
    [mats.casing, mats.fin, mats.core].forEach((m) => {
      m.transparent = ghost;
      m.opacity = opacity;
      m.depthWrite = !ghost;
      m.needsUpdate = true;
    });
  }

  return {
    root, rotor, stator, housing, shieldFront, shieldRear, rotorExtras,
    termBox, cables, lid,
    phaseMats, mats,
    setCutaway, setExplode,
    setLidOpen: setTerminalLid,          // legacy name kept for the chkLid toggle
    setFrontShieldVisible, setShieldsVisible, setTerminalLid, setGhostOpacity,
    focusTermBox: { x: 0, y: CASING_R + 0.055, z: 0.055 },   // camera-aim target
    setJumper: (mode) => {
      termBox.remove(jumper);
      jumper.geometry.dispose();
      const next = jumperPlates(mode);
      jumper.geometry = next.geometry;
      termBox.add(jumper);
      neutral.visible = mode !== 'delta';   // Y → physical star point on show
    },
    dims: { CORE_LEN, BAR_CIRCLE_R, BORE_R },
  };
}
