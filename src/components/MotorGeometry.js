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
 *   winding: 12 short-pitch coils (pitch 5/6), one continuous tube per coil;
 *   end-turn overhangs stacked in 3 z-levels (8.5 mm step) so coils whose arcs
 *   overlap angularly never intersect each other or the end shields.
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
    core:    std(0x8c96a4, 0.70, 0.50, { map: laminationTexture(60) }),   // laminated stator
    rotorCore: std(0x79828d, 0.70, 0.55, { map: laminationTexture(45) }),
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
  const bore = circlePts(BORE_R, 24 * 4); // placeholder, replaced below
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
  void bore;
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

/**
 * One complete stator coil as a single closed tube: conductor side in slot a,
 * front end-turn arcing over to slot a+5 (short pitch 5/6 — standard for
 * induction machines), second side, rear end-turn back. `level` (0..2) lifts
 * the overhang axially so coils whose end-turn arcs overlap angularly sit at
 * different z and never intersect.
 */
function coilGeometry(k, level) {
  const thA = (2 * Math.PI * k) / N_SLOTS;
  const thB = (2 * Math.PI * (k + 5)) / N_SLOTS;   // wraps mod 2π for k=19→slot 0
  const rSlot = BORE_R + SLOT_DEPTH * 0.5;         // conductor centre in the slot
  const rTurn = 0.076;                             // end-turn hoop radius
  const zSide = CORE_LEN / 2 - 0.005;              // where sides meet the overhang
  const zL = CORE_LEN / 2 + 0.005 + level * 0.0085;// front overhang z-level
  const span = thB - thA;                          // 75° = 5 slot pitches
  const P = (r, th, z) =>
    new THREE.Vector3(r * Math.cos(th), r * Math.sin(th), z);
  const pts = [
    // side in slot a (down → up)
    P(rSlot, thA, -zSide), P(rSlot, thA, -zSide / 2), P(rSlot, thA, 0),
    P(rSlot, thA, zSide / 2), P(rSlot, thA, zSide),
    // front end-turn: a → b, flaring to rTurn and lifting to zL
    P(rTurn, thA + span * 0.03, zSide + 0.002),
    P(rTurn, thA + span * 0.25, zL), P(rTurn, thA + span * 0.5, zL),
    P(rTurn, thA + span * 0.75, zL),
    P(rTurn, thB - span * 0.03, zSide + 0.002),
    // side in slot b (up → down)
    P(rSlot, thB, zSide), P(rSlot, thB, zSide / 2), P(rSlot, thB, 0),
    P(rSlot, thB, -zSide / 2), P(rSlot, thB, -zSide),
    // rear end-turn: b → a (mirror)
    P(rTurn, thB - span * 0.03, -zSide - 0.002),
    P(rTurn, thB - span * 0.25, -zL), P(rTurn, thB - span * 0.5, -zL),
    P(rTurn, thB - span * 0.75, -zL),
    P(rTurn, thA + span * 0.03, -zSide - 0.002),
  ];
  const curve = new THREE.CatmullRomCurve3(pts, true);
  return new THREE.TubeGeometry(curve, 96, 0.0039, 8, true);
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

  // stator winding: 12 short-pitch coils, odd slot a → a+5, one side per slot.
  // Phase of slot k: BELT_SEQ[floor(k/2) % 6] (verified: a and a+5 share phase).
  // Overhang z-level = coil index mod 3 — same-level coils start 90° apart,
  // clearing the 75° arc span; different levels are 8.5 mm apart axially.
  const phaseMats = { A: mats.windingA, B: mats.windingB, C: mats.windingC };
  for (let ci = 0; ci < N_SLOTS / 2; ci++) {
    const k = 2 * ci + 1;                          // odd slot: 1, 3, … 23
    const ph = PHASE_OF[BELT_SEQ[Math.floor(k / 2) % 6]];
    stator.add(new THREE.Mesh(coilGeometry(k, ci % 3), phaseMats[ph]));
  }
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
    root.add(grp);
  }
  root.add(housing);

  // terminal box with 6 pins (U1 V1 W1 / U2 V2 W2), top of casing
  const termBox = new THREE.Group();
  const box = new THREE.Mesh(new THREE.BoxGeometry(0.075, 0.05, 0.062), mats.termBox);
  box.position.set(0, CASING_R + 0.028, 0.055);
  termBox.add(box);
  const lid = new THREE.Mesh(new THREE.BoxGeometry(0.083, 0.008, 0.068), mats.foot);
  lid.position.set(0, CASING_R + 0.056, 0.055);
  termBox.add(lid);
  const pinGeos = [];
  for (let r = 0; r < 2; r++) for (let c = 0; c < 3; c++) {
    const g = new THREE.CylinderGeometry(0.0035, 0.0035, 0.014, 10);
    g.translate(-0.018 + c * 0.018, CASING_R + 0.064, 0.038 + r * 0.034);
    pinGeos.push(g);
  }
  termBox.add(new THREE.Mesh(mergeGeometries(pinGeos), mats.pin));
  housing.add(termBox);

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

  // Brass jumper plates — Star (Y): one bar bridging the top row (U2-V2-W2);
  // Delta (Δ): three vertical bars pin-row to pin-row. Swappable at runtime.
  const jumperBrass = new THREE.MeshStandardMaterial({ color: 0xd4af37, metalness: 1.0, roughness: 0.3 });
  function jumperPlates(mode) {
    const geos = [];
    if (mode === 'delta') {
      for (let c = 0; c < 3; c++) {
        const g = new THREE.BoxGeometry(0.010, 0.028, 0.005);
        g.translate(-0.018 + c * 0.018, CASING_R + 0.064, 0.055);
        geos.push(g);
      }
    } else {
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
  let explodeF = 0;
  function setExplode(f) {
    explodeF = f;
    shieldFront.position.z = f * 0.16;
    shieldRear.position.z = -f * 0.16;
    termBox.position.y = f * 0.10;
    box.position.y = CASING_R + 0.028 + f * 0.10;
    lid.position.y = CASING_R + 0.056 + f * 0.10;
    stator.position.z = f * 0.04;
  }
  setExplode(0);

  return {
    root, rotor, stator, housing, shieldFront, shieldRear, rotorExtras,
    phaseMats, mats,
    setCutaway, setExplode, setJumper: (mode) => {
      termBox.remove(jumper);
      jumper.geometry.dispose();
      const next = jumperPlates(mode);
      jumper.geometry = next.geometry;
      termBox.add(jumper);
    },
    dims: { CORE_LEN, BAR_CIRCLE_R, BORE_R },
  };
}
