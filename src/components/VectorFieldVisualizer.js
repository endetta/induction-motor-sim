/**
 * VectorFieldVisualizer.js
 * ---------------------------------------------------------------------------
 * Turns the physics model's per-bar quantities into 3D vectors, and animates
 * the rotating magnetic field.
 *
 * Layers (each independently toggleable):
 *   L1 stator currents  — winding emissive pulse ∝ |i_a|,|i_b|,|i_c|
 *   L2 RMF field lines  — closed oval loops from N pole to S pole, rotating at ωs
 *   L3 rotor bar currents — arrows along ±Z whose length ∝ instantaneous i_r,k
 *   L4 Lorentz forces   — tangential arrows at each bar, length ∝ F_k
 *   L5 torque/speed     — ωs reference ring (HUD also draws the numeric readout)
 *
 * Implementation notes:
 *  - With N_BARS=24 arrows per layer there is no per-frame allocation: all
 *    ArrowHelpers are built once and only their length/direction/visibility are
 *    updated. `setLength` does not reallocate the arrow head.
 *  - Arrows point along ±Z for bar currents (the bars run axially); force
 *    arrows point along the bar-tangent ê_θ direction, so their sign shows the
 *    driving sense directly.
 */

import * as THREE from 'three';

const N_BARS = 24;
const BAR_CIRCLE_R = 0.050;
const CORE_LEN = 0.20;
const BAR_ARROW_LEN = 0.030;    // m at full scale
const FORCE_ARROW_LEN = 0.045;

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

    // ---- L2: rotating magnetic field — closed oval flux loops ---------------
    // Each loop is a closed CatmullRom through 16 points: two poles at radius
    // ±Rpole on the field axis, crossed through the centre — a dipole-ish oval
    // that reads as "out of N, into S" once arrow heads are added.
    this.rmf = new THREE.Group();
    this.rmfLoops = [];
    const nLoops = 4;
    for (let i = 0; i < nLoops; i++) {
      const scale = 1 - i * 0.16;
      const pts = [];
      const Rpole = 0.098 * scale, Rside = 0.055 * scale;
      const SEG = 20;
      for (let s = 0; s < SEG; s++) {
        const a = (2 * Math.PI * s) / SEG;
        // polar-ish oval: x = Rpole·cos a, y = Rside·sin a, pinched at the poles
        pts.push(new THREE.Vector3(Rpole * Math.cos(a), Rside * Math.sin(a) * (0.35 + 0.65 * Math.abs(Math.cos(a))), 0));
      }
      const curve = new THREE.CatmullRomCurve3(pts, true);
      const tube = new THREE.Mesh(
        new THREE.TubeGeometry(curve, 64, 0.0018, 5, true),
        new THREE.MeshBasicMaterial({ color: 0x0e7fb8, transparent: true, opacity: 0.7 }),
      );
      this.rmf.add(tube);
      this.rmfLoops.push(tube);

      // N / S arrow heads on the axis, showing the outward field direction
      const headN = new THREE.Mesh(new THREE.ConeGeometry(0.008, 0.02, 8),
        new THREE.MeshBasicMaterial({ color: 0xc22a35 }));
      headN.position.set(Rpole, 0, 0);
      headN.rotation.z = -Math.PI / 2;
      const headS = new THREE.Mesh(new THREE.ConeGeometry(0.008, 0.02, 8),
        new THREE.MeshBasicMaterial({ color: 0x2b62b8 }));
      headS.position.set(-Rpole, 0, 0);
      headS.rotation.z = Math.PI / 2;
      this.rmf.add(headN, headS);
    }
    scene.add(this.rmf);

    // ---- L3: rotor bar current arrows (axial, ±Z) --------------------------
    this.barArrows = [];
    const barGrp = new THREE.Group();
    for (let k = 0; k < N_BARS; k++) {
      const a = this.theta[k];
      const arrow = new THREE.ArrowHelper(
        new THREE.Vector3(0, 0, 1), new THREE.Vector3(0, 0, 0), 0.03, 0xcc8f1f, 0.010, 0.006);
      arrow.position.set(BAR_CIRCLE_R * Math.cos(a), BAR_CIRCLE_R * Math.sin(a), CORE_LEN / 2 + 0.02);
      barGrp.add(arrow);
      this.barArrows.push(arrow);
    }
    scene.add(barGrp);
    this.barGrp = barGrp;

    // ---- L4: Lorentz force arrows (tangential) ------------------------------
    this.forceArrows = [];
    const forceGrp = new THREE.Group();
    for (let k = 0; k < N_BARS; k++) {
      const a = this.theta[k];
      const arrow = new THREE.ArrowHelper(
        new THREE.Vector3(0, 0, 1), new THREE.Vector3(0, 0, 0), 0.04, 0xd95a1f, 0.012, 0.007);
      arrow.position.set(BAR_CIRCLE_R * Math.cos(a), BAR_CIRCLE_R * Math.sin(a), CORE_LEN / 2 + 0.055);
      forceGrp.add(arrow);
      this.forceArrows.push(arrow);
    }
    scene.add(forceGrp);
    this.forceGrp = forceGrp;

    // ---- L5: speed reference ring (ωs) -------------------------------------
    const wsRing = new THREE.Mesh(
      new THREE.TorusGeometry(0.108, 0.0016, 6, 96),
      new THREE.MeshBasicMaterial({ color: 0x188a4e, transparent: true, opacity: 0.6 }),
    );
    wsRing.position.z = CORE_LEN / 2 + 0.10;
    scene.add(wsRing);
    this.wsRing = wsRing;

    // ---- L6: RMF flux particles — InstancedMesh riding the closed loops ----
    // Built once, updated per frame via setMatrixAt only (zero allocation).
    // Curves mirror the loop construction above (same points, stored for u→pos).
    this.rmfCurves = [];
    for (let i = 0; i < nLoops; i++) {
      const scale = 1 - i * 0.16;
      const pts = [];
      const Rpole = 0.098 * scale, Rside = 0.055 * scale;
      const SEG = 20;
      for (let s = 0; s < SEG; s++) {
        const a = (2 * Math.PI * s) / SEG;
        pts.push(new THREE.Vector3(Rpole * Math.cos(a), Rside * Math.sin(a) * (0.35 + 0.65 * Math.abs(Math.cos(a))), 0));
      }
      this.rmfCurves.push(new THREE.CatmullRomCurve3(pts, true));
    }
    const N_PARTICLES = 120;
    this.particleOffsets = [];
    for (let i = 0; i < N_PARTICLES; i++) {
      this.particleOffsets.push({ curveIdx: i % nLoops, u: i / N_PARTICLES });
    }
    this.particleMesh = new THREE.InstancedMesh(
      new THREE.SphereGeometry(0.0022, 6, 6),
      new THREE.MeshBasicMaterial({ color: 0x0aa5cc, transparent: true, opacity: 0.9 }),
      N_PARTICLES,
    );
    this.rmf.add(this.particleMesh);   // rotates with the field group
    this._pDummy = new THREE.Object3D();

    // layer visibility state
    this.layers = { stator: true, rmf: true, barCurrent: true, force: true, speedRing: true };
  }

  /** Toggle a layer by name; returns the new state. */
  toggle(name, on) {
    const v = on === undefined ? !this.layers[name] : on;
    this.layers[name] = v;
    if (name === 'rmf') this.rmf.visible = v;
    if (name === 'barCurrent') this.barGrp.visible = v;
    if (name === 'force') this.forceGrp.visible = v;
    if (name === 'speedRing') this.wsRing.visible = v;
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

    // ---- L2: RMF rotation at synchronous speed ------------------------------
    // The field rotates mechanically at ωs·t = (2/P)·ωe·t.
    const polePairs = p.P / 2;
    const fieldAngle = motor.omega_e * motor.time / polePairs;
    this.rmf.rotation.z = fieldAngle;

    // ---- L6: particles riding the closed flux loops -------------------------
    // Speed ∝ gap flux density B_gap (denser/thicker field → faster dots).
    if (this.layers.rmf) {
      const speed = motor.Bm * 0.25;
      const dummy = this._pDummy;
      for (let i = 0; i < this.particleOffsets.length; i++) {
        const po = this.particleOffsets[i];
        po.u = (po.u + speed * dt) % 1.0;
        const pt = this.rmfCurves[po.curveIdx].getPointAt(po.u);
        dummy.position.copy(pt);
        dummy.updateMatrix();
        this.particleMesh.setMatrixAt(i, dummy.matrix);
      }
      this.particleMesh.instanceMatrix.needsUpdate = true;
    }

    // ---- L3 + L4: per-bar current and force ---------------------------------
    if (this.layers.barCurrent || this.layers.force) {
      const ir = motor.barCurrents(this.theta);
      // normalise against the maximum possible bar current (s=1) for a stable scale
      const iRef = motor.barEmf0 / Math.hypot(p.Rr, p.Xr60 * (p.f / 60)) || 1;
      const iMax = Math.max(...ir.map(Math.abs), 1e-9);
      const iScale = iRef > 0 ? 1 / iRef : 1;

      // force reference: r·L·i_max·B_max at full scale
      const fRef = Math.abs(p.rRotor * p.L * iRef * 1.5 * motor.Bm) || 1;

      for (let k = 0; k < N_BARS; k++) {
        const ik = ir[k];
        const a = this.theta[k];

        if (this.layers.barCurrent) {
          const mag = ik * iScale;
          const av = this.barArrows[k];
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
          const av = this.forceArrows[k];
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
      void iMax;
    }
  }
}

const UP = new THREE.Vector3(0, 0, 1);
const DOWN = new THREE.Vector3(0, 0, -1);
const _dir = new THREE.Vector3();
