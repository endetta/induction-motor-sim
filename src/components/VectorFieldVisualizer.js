/**
 * VectorFieldVisualizer.js
 * ---------------------------------------------------------------------------
 * Turns the physics model's per-bar quantities into 3D vectors, and animates
 * the rotating magnetic field.
 *
 * Layers (each independently toggleable):
 *   L1 stator currents  — winding emissive pulse ∝ |i_a|,|i_b|,|i_c|
 *   L1b phase flux vectors — B_A/B_B/B_C arrows along each phase axis, whose
 *        tip-sum is the resultant B_net = rotating RMF (sync speed)
 *   L2 RMF field lines  — closed flux loops in the FRONT cross-section plane
 *        (z = CORE_LEN/2 + 0.01) crossing the air gap over the cage bars;
 *        pole-count aware (P=4 → alternating N-S-N-S every 90°)
 *   L3 rotor bar currents — arrows along ±Z at the END-RING radius, on a
 *        selected subset of bars (8 of 24) to avoid visual clutter
 *   L4 Lorentz forces   — tangential arrows just outside the air gap, same subset
 *
 * Implementation notes:
 *  - No per-frame allocation: everything is built once and only position /
 *    length / direction / visibility are updated. `setLength` on ArrowHelper
 *    does not reallocate the head; instanced particles use setMatrixAt only.
 *  - Bar arrows point along ±Z (bars run axially); force arrows follow the
 *    bar-tangent ê_θ, so their sign shows the driving sense directly.
 */

import * as THREE from 'three';

const N_BARS = 24;
const N_SHOWN = 12;             // bars carrying visible arrows (subset, not all 24)
const BAR_CIRCLE_R = 0.050;
const CORE_LEN = 0.20;
const BAR_ARROW_LEN = 0.030;    // m at full scale
const FORCE_ARROW_LEN = 0.045;
const END_RING_Z = CORE_LEN / 2 + 0.012;   // end-ring plane (matches MotorGeometry)
const GAP_Z = CORE_LEN / 2 + 0.01;         // front cross-section flux plane

/** Bar angles θ_k = 2πk/N, matching the physics model's ordering. */
export function barAngles(n = N_BARS) {
  const th = [];
  for (let k = 0; k < n; k++) th.push((2 * Math.PI * k) / n);
  return th;
}

/**
 * Colour a scalar in [−1, 1] onto a blue→dark→red ramp for current sign.
 * Dark enough to read on a light background.
 */
const _c = new THREE.Color();
export function signedColor(v, out = _c) {
  const t = Math.max(-1, Math.min(1, v));
  if (t >= 0) out.setRGB(0.72 + 0.16 * t, 0.18 + 0.12 * (1 - t), 0.16 + 0.10 * (1 - t));
  else out.setRGB(0.14 + 0.10 * (1 + t), 0.30 + 0.14 * (1 + t), 0.72 + 0.16 * -t);
  return out;
}

export class VectorFieldVisualizer {
  /**
   * @param {THREE.Scene} scene
   * @param {object} motor  handle returned by buildMotor()
   */
  constructor(scene, motor) {
    this.scene = scene;
    this.motor = motor;
    this.theta = barAngles(N_BARS);
    // evenly spread subset: every N_BARS/N_SHOWN-th bar
    const stride = N_BARS / N_SHOWN;
    this.shown = [];
    for (let k = 0; k < N_SHOWN; k++) this.shown.push(Math.round(k * stride));

    // ---- L1b: per-phase flux vectors + resultant RMF arrow ------------------
    // B_A(t) = Bm·cos(ωe·t) along axis A (0°), B_B 120°, B_C 240°.
    // Resultant B_net = B_A + B_B + B_C = 1.5·Bm·(ωe·t) — the RMF itself.
    this.phaseGrp = new THREE.Group();
    this.phaseAxes = [0, (2 * Math.PI) / 3, (4 * Math.PI) / 3];
    const phaseCols = [0xd94848, 0xb8860b, 0x2b62b8];   // A red, B dark yellow, C blue
    this.phaseArrows = [];
    for (let i = 0; i < 3; i++) {
      const a = new THREE.ArrowHelper(
        new THREE.Vector3(Math.cos(this.phaseAxes[i]), Math.sin(this.phaseAxes[i]), 0),
        new THREE.Vector3(0, 0, GAP_Z), 0.05, phaseCols[i], 0.014, 0.009);
      a.position.set(0, 0, GAP_Z);
      this.phaseArrows.push(a);
      this.phaseGrp.add(a);
    }
    // resultant: B_net tip position, rebuilt from the three vectors each frame
    this.netArrow = new THREE.ArrowHelper(
      new THREE.Vector3(1, 0, 0), new THREE.Vector3(0, 0, GAP_Z), 0.075, 0x0e7fb8, 0.02, 0.013);
    this.phaseGrp.add(this.netArrow);
    scene.add(this.phaseGrp);
    this._netDir = new THREE.Vector3();

    // ---- L2: RMF flux lines — front cross-section, pole-count aware ---------
    // Closed loops ride in the plane z = GAP_Z so they visibly cross the air
    // gap and the cage bars (the old geometry was buried at z = 0 inside the
    // rotor). For P poles the group holds P loop-lobes: flux exits each N
    // lobe, crosses the gap, returns through the neighbouring S lobes. The
    // per-lobe lobes are unit-dipoles arranged on the field axis; the group
    // rotates at ωs and the lobe pattern is re-spaced when P changes.
    this.rmf = new THREE.Group();
    this.rmf.position.z = GAP_Z;
    this.rmfLoops = [];
    this._buildRmfLoops(4);
    scene.add(this.rmf);

    // ---- L3: rotor bar current arrows (axial, ±Z) — end-ring radius ---------
    this.barArrows = [];
    const barGrp = new THREE.Group();
    for (let i = 0; i < N_SHOWN; i++) {
      const a = this.theta[this.shown[i]];
      const arrow = new THREE.ArrowHelper(
        new THREE.Vector3(0, 0, 1), new THREE.Vector3(0, 0, 0), 0.03, 0xcc8f1f, 0.010, 0.006);
      // sit ON the end ring: end-ring radius, end-ring z plane
      arrow.position.set(BAR_CIRCLE_R * Math.cos(a), BAR_CIRCLE_R * Math.sin(a), END_RING_Z);
      barGrp.add(arrow);
      this.barArrows.push(arrow);
    }
    scene.add(barGrp);
    this.barGrp = barGrp;

    // ---- L4: Lorentz force arrows (tangential) — just outside the air gap ---
    this.forceArrows = [];
    const forceGrp = new THREE.Group();
    for (let i = 0; i < N_SHOWN; i++) {
      const a = this.theta[this.shown[i]];
      const arrow = new THREE.ArrowHelper(
        new THREE.Vector3(0, 0, 1), new THREE.Vector3(0, 0, 0), 0.04, 0xd95a1f, 0.012, 0.007);
      // slightly outside the air gap: bore radius + margin, front of core
      const rOut = 0.066;
      arrow.position.set(rOut * Math.cos(a), rOut * Math.sin(a), CORE_LEN / 2 + 0.03);
      forceGrp.add(arrow);
      this.forceArrows.push(arrow);
    }
    scene.add(forceGrp);
    this.forceGrp = forceGrp;

    // ---- L6: RMF flux particles — InstancedMesh riding the closed loops ----
    // Built once, updated per frame via setMatrixAt only (zero allocation).
    this._syncCurves();
    const N_PARTICLES = 120;
    this.particleOffsets = [];
    for (let i = 0; i < N_PARTICLES; i++) {
      this.particleOffsets.push({ curveIdx: i % this.rmfCurves.length, u: i / N_PARTICLES });
    }
    this.particleMesh = new THREE.InstancedMesh(
      new THREE.SphereGeometry(0.0022, 6, 6),
      new THREE.MeshBasicMaterial({ color: 0x0aa5cc, transparent: true, opacity: 0.9 }),
      N_PARTICLES,
    );
    this.rmf.add(this.particleMesh);   // rotates with the field group
    this._pDummy = new THREE.Object3D();

    // layer visibility state
    this.layers = { stator: true, rmf: true, barCurrent: true, force: true,
                    phaseField: true };
  }

  /** (Re)build the pole-count-aware flux loop pattern in the rmf group. */
  _buildRmfLoops(P) {
    // dispose previous
    for (const l of this.rmfLoops) { this.rmf.remove(l); l.geometry.dispose(); l.material.dispose(); }
    this.rmfLoops.length = 0;
    this._rmfP = P;
    // P poles → P/2 loop pairs; each pair is one closed oval through the axis
    const nLoops = P / 2;
    for (let i = 0; i < nLoops; i++) {
      const scale = 1 - i * 0.22;
      const pts = [];
      const Rpole = 0.098 * scale, Rside = 0.055 * scale;
      const SEG = 20;
      for (let s = 0; s < SEG; s++) {
        const a = (2 * Math.PI * s) / SEG;
        pts.push(new THREE.Vector3(
          Rpole * Math.cos(a),
          Rside * Math.sin(a) * (0.35 + 0.65 * Math.abs(Math.cos(a))), 0));
      }
      const curve = new THREE.CatmullRomCurve3(pts, true);
      const tube = new THREE.Mesh(
        new THREE.TubeGeometry(curve, 64, 0.0018, 5, true),
        new THREE.MeshBasicMaterial({ color: 0x0e7fb8, transparent: true, opacity: 0.7 }),
      );
      // P=4: second loop pair turned 90° so N-S alternate every 90° spatially
      tube.rotation.z = (2 * Math.PI * i * 2) / P;
      this.rmf.add(tube);
      this.rmfLoops.push(tube);
    }
  }

  /** Mirror the loop construction into particle-riding curves. */
  _syncCurves() {
    this.rmfCurves = [];
    const nLoops = this._rmfP / 2;
    for (let i = 0; i < nLoops; i++) {
      const scale = 1 - i * 0.22;
      const pts = [];
      const Rpole = 0.098 * scale, Rside = 0.055 * scale;
      const SEG = 20;
      for (let s = 0; s < SEG; s++) {
        const a = (2 * Math.PI * s) / SEG;
        pts.push(new THREE.Vector3(
          Rpole * Math.cos(a),
          Rside * Math.sin(a) * (0.35 + 0.65 * Math.abs(Math.cos(a))), 0));
      }
      this.rmfCurves.push(new THREE.CatmullRomCurve3(pts, true));
    }
  }

  /** Toggle a layer by name; returns the new state. */
  toggle(name, on) {
    const v = on === undefined ? !this.layers[name] : on;
    this.layers[name] = v;
    if (name === 'rmf') this.rmf.visible = v;
    if (name === 'barCurrent') this.barGrp.visible = v;
    if (name === 'force') this.forceGrp.visible = v;
    if (name === 'phaseField') this.phaseGrp.visible = v;
    return v;
  }

  /**
   * Per-frame update from the physics model.
   * @param {InductionMotorModel} motor
   * @param {number} animateScale  slow-motion multiplier applied by the caller
   */
  update(motor, _animateScale = 1, dt = 1 / 60) {
    const p = motor.params;
    const thetaField = motor.omega_e * motor.time;   // RMF angle = ωe·t (mechanically ωs·t)

    // ---- L1: stator winding current pulse -----------------------------------
    if (this.layers.stator) {
      // i_a = Im cos(ωe t) etc. — normalised so 1.0 = rated-ish conduction.
      const wA = Math.cos(thetaField);
      const wB = Math.cos(thetaField - (2 * Math.PI) / 3);
      const wC = Math.cos(thetaField - (4 * Math.PI) / 3);
      const k = 0.12 + 2.2 * Math.min(1, motor.Istator / Math.max(motor.ratedCurrent, 1e-6));
      this.motor.phaseMats.A.emissiveIntensity = Math.max(0, wA) * k;
      this.motor.phaseMats.B.emissiveIntensity = Math.max(0, wB) * k;
      this.motor.phaseMats.C.emissiveIntensity = Math.max(0, wC) * k;
    }

    // ---- L2: RMF rotation at synchronous speed + pole-count pattern ---------
    // The field rotates mechanically at ωs·t = (2/P)·ωe·t.
    const polePairs = p.P / 2;
    const fieldAngle = motor.omega_e * motor.time / polePairs;
    if (p.P !== this._rmfP) { this._buildRmfLoops(p.P); this._syncCurves(); }
    this.rmf.rotation.z = fieldAngle;

    // ---- L1b: per-phase flux vectors B_A/B_B/B_C and resultant B_net --------
    if (this.layers.phaseField) {
      const Bm = motor.Bm;
      // vector sum in the fixed XY frame: Σ Bm·cos(ωe·t − φi)·(axis_i)
      let nx = 0, ny = 0;
      for (let i = 0; i < 3; i++) {
        const b = Bm * Math.cos(thetaField - this.phaseAxes[i]);
        const ax = Math.cos(this.phaseAxes[i]), ay = Math.sin(this.phaseAxes[i]);
        const len = Math.abs(b) / Bm * 0.05;
        const av = this.phaseArrows[i];
        av.visible = len > 1e-4;
        if (av.visible) {
          // flip arrow when the instantaneous flux is negative
          av.setDirection(_dir.set(ax * Math.sign(b), ay * Math.sign(b), 0));
          av.setLength(len, Math.min(0.014, len * 0.3), Math.min(0.009, len * 0.2));
        }
        nx += b * ax; ny += b * ay;
      }
      const nMag = Math.hypot(nx, ny) / Bm;
      this.netArrow.visible = nMag > 1e-4;
      if (this.netArrow.visible) {
        const len = Math.min(nMag * 0.05, 0.075);
        this.netArrow.setDirection(this._netDir.set(nx / (nMag * Bm), ny / (nMag * Bm), 0));
        this.netArrow.setLength(len, Math.min(0.02, len * 0.28), Math.min(0.013, len * 0.18));
      }
    }

    // ---- L6: particles riding the closed flux loops -------------------------
    // Speed ∝ gap flux density B_gap (denser/thicker field → faster dots).
    if (this.layers.rmf) {
      const speed = motor.Bm * 0.25;
      const dummy = this._pDummy;
      const curves = this.rmfCurves;
      const nCurves = curves.length;
      for (let i = 0; i < this.particleOffsets.length; i++) {
        const po = this.particleOffsets[i];
        po.u = (po.u + speed * dt) % 1.0;
        // modulo: the curve set is rebuilt when P changes, so the stored
        // curveIdx (0..3 from construction) can exceed the live loop count
        const pt = curves[po.curveIdx % nCurves].getPointAt(po.u);
        dummy.position.copy(pt);
        dummy.updateMatrix();
        this.particleMesh.setMatrixAt(i, dummy.matrix);
      }
      this.particleMesh.instanceMatrix.needsUpdate = true;
    }

    // ---- L3 + L4: per-bar current and force (subset of bars) ----------------
    if (this.layers.barCurrent || this.layers.force) {
      const ir = motor.barCurrents(this.theta);
      // normalise against the maximum possible bar current (s=1) for a stable scale
      const iRef = motor.barEmf0 / Math.hypot(p.Rr, p.Xr60 * (p.f / 60)) || 1;
      const iScale = iRef > 0 ? 1 / iRef : 1;

      // force reference: r·L·i_max·B_max at full scale
      const fRef = Math.abs(p.rRotor * p.L * iRef * 1.5 * motor.Bm) || 1;

      for (let i = 0; i < N_SHOWN; i++) {
        const k = this.shown[i];
        const ik = ir[k];
        const a = this.theta[k];

        if (this.layers.barCurrent) {
          const mag = ik * iScale;
          const av = this.barArrows[i];
          const len = Math.min(Math.abs(mag), 1.4) * BAR_ARROW_LEN;
          if (len < 1e-4) {
            av.visible = false;
          } else {
            av.visible = true;
            av.setLength(len, Math.min(0.012, len * 0.35), Math.min(0.008, len * 0.22));
            av.setDirection(mag >= 0 ? UP : DOWN);
            av.setColor(signedColor(Math.max(-1, Math.min(1, mag))));
          }
        }

        if (this.layers.force) {
          // F = i·L×B : tangential, sign = current×flux direction at this bar.
          const B = motor.fluxDensityAt(a);
          const fTang = ik * p.L * B;
          const fNorm = fTang / fRef;
          const av = this.forceArrows[i];
          const len = Math.min(Math.abs(fNorm), 1.3) * FORCE_ARROW_LEN;
          if (len < 1e-4) {
            av.visible = false;
          } else {
            av.visible = true;
            // ê_θ at angle a is (−sin a, cos a, 0)
            _dir.set(-Math.sin(a), Math.cos(a), 0).multiplyScalar(Math.sign(fNorm));
            av.setDirection(_dir);
            av.setLength(len, Math.min(0.014, len * 0.30), Math.min(0.009, len * 0.20));
          }
        }
      }
    }
  }
}

const UP = new THREE.Vector3(0, 0, 1);
const DOWN = new THREE.Vector3(0, 0, -1);
const _dir = new THREE.Vector3();
