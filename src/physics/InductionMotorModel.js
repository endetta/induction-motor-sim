/**
 * InductionMotorModel.js
 * ---------------------------------------------------------------------------
 * State machine + numerical integration for a three-phase induction motor.
 * SI units unless noted.
 *
 * == 1. ROTATING MAGNETIC FIELD ==============================================
 * Balanced three-phase currents in the stator winding:
 *     i_a(t) = Im·cos(ωe·t)
 *     i_b(t) = Im·cos(ωe·t − 2π/3)
 *     i_c(t) = Im·cos(ωe·t − 4π/3)
 * Each phase produces a pulsating mmf along its own axis; the three are
 * displaced 120° in space and 120° in time, so their sum is a travelling wave:
 *     B_net(θ,t) = (3/2)·Bm·[cos(ωe·t − θ)·a_r + sin(ωe·t − θ)·a_θ]
 * The (3/2) is the three-phase spatial-summation constant: √3/2 for the
 * individual phase contributions, giving 1.5× the per-phase amplitude.
 * Synchronous speed:   n_s = 120·f / P  (rpm)     ωs = 4πf / P  (rad/s)
 * In one supply cycle the field advances one pole pair (2/P of a revolution).
 *
 * == 2. SLIP, ROTOR EMF, ROTOR CURRENT =======================================
 *     s    = (ωs − ωr)/ωs                    slip
 *     E_r  = s·E_r0                          bar emf ∝ relative flux-cutting speed
 *     f_r  = s·f                             rotor frequency
 *     Z_r  = √(R_r² + (s·X_r0)²)             bar impedance at rotor frequency
 *     φ_r  = atan(s·X_r0 / R_r)              bar current lag
 *     i_r,k(t) = (E_r/Z_r)·cos(s·ωe·t − φ_r − θ_k)
 * At standstill (s = 1) bar emf and frequency are maximal — this is the
 * starting-current condition. At s = 0 the flux no longer cuts the bars.
 *
 * == 3. LORENTZ FORCE AND ELECTROMAGNETIC TORQUE =============================
 *     F_k = i_r,k·(L × B_net)          on bar k of active length L
 * Tangential component F_k = i_r,k·L·B_r(θ_k);  T_e = Σ r_rotor·F_k
 * The continuous equivalent (rotor referred to the stator) drives the
 * mechanical dynamics — it is the same physics expressed per-phase:
 *     I2'(s) = V_th / √((R_th + R_r'/s)² + (X_th + X_r')²)
 *     T_e(s) = 3·I2'²·(R_r'/s) / ωs
 * The rotor branch always shows the CONSTANT referred reactance X_r' — the
 * s·X_r0 seen at rotor frequency, divided by s when referred to the stator.
 * Peak (pull-out) at s_k = R_r'/√(R_th² + (X_th+X_r')²), where the Kloss
 * approximation T_e ≈ 2·T_max/(s/s_k + s_k/s) is exact; see klossTorque().
 *
 * Modelling note: the rotor's L/R electrical time constant (Xr'/(ωe·Rr'), a
 * few ms) is much shorter than the mechanical time constant J·ωs/T_rated, so
 * the quasi-static torque curve is used instead of integrating rotor flux
 * linkage. This is the standard assumption for start-up transient studies.
 *
 * == 4. MECHANICAL TRANSIENT (STARTING) ======================================
 *     J·dωr/dt = T_e − T_L − B·ωr
 * RK4 at a fixed 1 ms step. J (kg·m²) sets the run-up time; the transient runs
 * from s = 1 down to the operating slip. Rotor I²R loss is the difference
 * between air-gap power and mechanical power: P_ag − P_mech = s·P_ag.
 *
 * == 5. STATOR CURRENT =======================================================
 * Exact per-phase equivalent circuit (magnetizing branch ∥ rotor branch).
 * At s = 1 this approaches the familiar locked-rotor inrush
 *     I_start ≈ V_ph / √((R_s+R_r')² + (X_s+X_r')²)   ≈ 5–8 × I_rated
 * and falls to the magnetizing-dominated no-load current as s → 0.
 *
 * Reactances scale with line frequency (X = 2πfL), so lowering f while holding
 * V raises flux and saturates the iron — the physical V/f trade-off.
 */

import * as THREE from 'three';

/** Reference machine: 460 V / 60 Hz / 4-pole (Chapman, 25 hp design-B class). */
export const REF_F = 60;        // Hz  — frequency at which Xs60/Xr60/Xm60 apply
export const REF_VPH = 265.6;   // V   — rated phase voltage (460 V L-L / √3)

export const DEFAULT_PARAMS = {
  Vph: 230.9,      // V       supply phase voltage (400 V line-to-line / √3)
  f: 50,           // Hz      line frequency (10–60)
  P: 4,            // poles
  Rs: 0.641,       // Ω       stator resistance per phase
  Rr: 0.332,       // Ω       rotor resistance referred to stator
  Xs60: 1.106,     // Ω       stator leakage reactance at REF_F
  Xr60: 0.464,     // Ω       rotor leakage reactance referred to stator at REF_F
  Xm60: 26.3,      // Ω       magnetizing reactance at REF_F
  nBars: 24,       // rotor bars modelled for the Lorentz-force layer
  L: 0.20,         // m       active rotor-bar length
  rRotor: 0.05,    // m       rotor radius at the bar mid-plane
  J: 0.25,         // kg·m²   total inertia (rotor + coupled load)
  TL: 0.0,         // N·m     constant-magnitude load torque
  // No-load drag: friction + windage + core loss, lumped as viscous.
  // Sized from a no-load test losing ≈4 % of rated power at synchronous speed.
  Bfric: 0.02,     // N·m·s/rad
};

export const MotorState = Object.freeze({
  IDLE: 'idle',        // de-energized: ωr coasts down on load + friction
  RUNNING: 'running',  // energized: torque from the equivalent circuit
  BRAKING: 'braking',  // emergency braking: torque opposes rotation
});

/** ωs = 4πf/P (rad/s) — mechanical synchronous speed. */
export function omegaSync(f, P) { return (4 * Math.PI * f) / P; }

/** s = (ωs − ωr)/ωs. Band [−1, 2]: motoring 0…1, plugging 1…2, generating < 0. */
export function slipOf(wr, ws) {
  if (ws === 0) return 1;
  return THREE.MathUtils.clamp((ws - wr) / ws, -1.0, 2.0);
}

/** Leakage/magnetizing reactances at the current line frequency: X ∝ f. */
export function reactances(p) {
  const k = p.f / REF_F;
  return { Xs: p.Xs60 * k, Xr: p.Xr60 * k, Xm: p.Xm60 * k };
}

/**
 * Thevenin equivalent seen from the rotor branch (Chapman eq. 6-41..43):
 *   V_th = V_ph·Xm/√(Rs² + (Xs+Xm)²)
 *   R_th = Rs·Xm²/(Rs² + (Xs+Xm)²)
 *   X_th = Xm·(Rs² + Xs·(Xs+Xm))/(Rs² + (Xs+Xm)²)
 */
export function thevenin(p) {
  const { Xs, Xr, Xm } = reactances(p);
  const d = p.Rs * p.Rs + (Xs + Xm) * (Xs + Xm);
  return {
    Vth: (p.Vph * Xm) / Math.sqrt(d),
    Rth: (p.Rs * Xm * Xm) / d,
    Xth: (Xm * (p.Rs * p.Rs + Xs * (Xs + Xm))) / d,
    Xs, Xr, Xm,
  };
}

/**
 * Electromagnetic torque (N·m) at slip s — exact per-phase equivalent circuit.
 *   T_e = 3·I2'²·(R_r'/s)/ωs
 * Sign carries: s < 0 (ωr > ωs) gives negative torque = regenerative braking,
 * which is what keeps the rotor from running away past synchronous speed.
 */
export function torqueSlip(s, p) {
  // s → 0: the rotor branch degenerates (R_r'/s → ∞), I2' → 0 and Te → 0.
  // Handle the 0/0 limit analytically rather than numerically.
  if (Math.abs(s) < 1e-9) return 0;
  const { Vth, Rth, Xth, Xr } = thevenin(p);
  const denom = Math.hypot(Rth + p.Rr / s, Xth + Xr);
  const I2 = Vth / denom;
  return (3 * I2 * I2 * (p.Rr / s)) / omegaSync(p.f, p.P);
}

/**
 * Kloss closed-form torque curve — the classical two-constant approximation of
 * the equivalent-circuit result:
 *     T_e(s) = 2·T_max / (s/s_k + s_k/s)
 * Exact at s = s_k (returns T_max) and exact in the limit s → 0 and s → ∞ in
 * shape; typically within ~10 % of the exact circuit over the stable branch.
 * Provided for comparison in the HUD and as the analytic sanity check.
 */
export function klossTorque(s, p) {
  if (Math.abs(s) < 1e-9) return 0;
  const { tMax, sMax } = klossConstants(p);
  return (2 * tMax) / (s / sMax + sMax / s);
}

/**
 * Pull-out (breakdown) slip and maximum torque.
 *   s_k   = R_r' / √(R_th² + (X_th + X_r')²)
 *   T_max = 3·V_th² / (2·ωs·(R_th + √(R_th² + (X_th+X_r')²)))
 */
export function klossConstants(p) {
  const { Vth, Rth, Xth, Xr } = thevenin(p);
  const Zk = Math.hypot(Rth, Xth + Xr);
  return {
    sMax: p.Rr / Zk,
    tMax: (3 * Vth * Vth) / (2 * omegaSync(p.f, p.P) * (Rth + Zk)),
  };
}

/**
 * Stator current magnitude (A) at slip s — exact complex arithmetic.
 * Z_t = Rs + jXs + (jXm ∥ (Rr/s + jXr))  →  I = V_ph/|Z_t|.
 */
export function statorCurrent(s, p) {
  // s = 0 clamp: Rr/s would be ±Infinity and the complex division below turns
  // it into NaN (Infinity/Infinity). At s → 0⁺ the rotor branch opens and the
  // current falls to the magnetizing-dominated no-load value — clamping to
  // 1e-4 reproduces that limit; the sign is preserved for s < 0 (generating).
  const sSafe = Math.abs(s) < 1e-4 ? (s >= 0 ? 1e-4 : -1e-4) : s;
  const { Xs, Xr, Xm } = reactances(p);
  const aR = p.Rr / sSafe, aI = Xr;        // rotor branch Z2 = Rr/s + jXr
  // (jXm)·Z2 / (jXm + Z2), with jXm = (0, Xm): proper complex division.
  const nR = -Xm * aI, nI = Xm * aR;       // numerator (jXm)(Z2)
  const dR = aR, dI = aI + Xm;             // denominator (Z2 + jXm)
  const d2 = dR * dR + dI * dI || 1e-12;
  const parR = (nR * dR + nI * dI) / d2;   // (ac+bd)/|d|²
  const parI = (nI * dR - nR * dI) / d2;   // (bc−ad)/|d|²
  return p.Vph / (Math.hypot(p.Rs + parR, Xs + parI) || 1e-9);
}

/**
 * Operating slip for a load of `loadFrac` × T_max. Bisection is valid only on
 * the STABLE branch s ∈ (0, s_k], where torque rises monotonically — past
 * pull-out the curve turns back and two slips share each torque.
 * Default 0.45·T_max ≈ the rated point of a design-B machine (breakdown
 * torque ≈ 2.2 × rated), giving the reference slip used to report I_rated.
 */
export function operatingSlip(p, loadFrac = 0.45) {
  const { tMax, sMax } = klossConstants(p);
  const target = loadFrac * tMax;
  let lo = 1e-6, hi = sMax;
  for (let i = 0; i < 80; i++) {
    const mid = (lo + hi) / 2;
    if (torqueSlip(mid, p) < target) lo = mid; else hi = mid;
  }
  return (lo + hi) / 2;
}

/** Classical starting (locked-rotor) current, neglecting the magnetizing branch. */
export function lockedRotorCurrent(p) {
  const { Xs, Xr } = reactances(p);
  return p.Vph / Math.hypot(p.Rs + p.Rr, Xs + Xr);
}

/**
 * Simulation core. Owns the scalar state, integrates the mechanical ODE at a
 * fixed 1 ms step (`step()` accumulates wall time and drains it in sub-steps),
 * and exposes per-bar arrays for the 3D visualizers.
 */
export class InductionMotorModel {
  constructor(params = {}) {
    this.params = { ...DEFAULT_PARAMS, ...params };
    this.state = MotorState.IDLE;
    this.time = 0;        // simulated electrical time (s)
    this.omega_r = 0;     // rotor mechanical speed (rad/s)
    this.theta_r = 0;     // rotor mechanical angle (rad)
    this.Te = 0;          // last electromagnetic torque (N·m)
    this.TL = 0;          // load torque actually applied (N·m)
    this.slip = 1;
    this.Istator = 0;     // stator current magnitude (A)
    this.brakeTorque = 0; // braking torque magnitude (N·m) while BRAKING
    this.stalled = false; // true when breakaway torque < load torque at rest
    // Integrated rotor electrical phase: θ_rot = ∫ s(t)·ωe dt. Phase from
    // s(t)·ωe·t is only valid at constant slip — during run-up the ωe·t·ds/dt
    // term distorts it, so the phase is accumulated in _subStep instead.
    this.theta_rot_elec = 0;
    this.history = [];    // telemetry ring buffer [{t,wr,ws,s,I,Te}]
    this._accum = 0;      // fixed-step accumulator
  }

  get omega_e() { return 2 * Math.PI * this.params.f; }              // ωe = 2πf
  get omega_s() { return omegaSync(this.params.f, this.params.P); }  // ωs = 4πf/P
  get nSyncRpm() { return (120 * this.params.f) / this.params.P; }   // n_s = 120f/P
  get kloss() { return klossConstants(this.params); }

  /** Rotor bar open-circuit emf scale: E ∝ flux·speed ∝ V_ph at fixed V/f. */
  get barEmf0() { return this.params.Vph; }

  /** Air-gap flux-density amplitude (display units): B ∝ V_ph/f — the
   *  constant-flux law. Dropping f at fixed V saturates the iron. */
  get Bm() {
    const p = this.params;
    return 0.9 * (p.Vph / REF_VPH) * (REF_F / Math.max(p.f, 1));
  }

  /** Rated current estimate at the reference operating slip (for gauges). */
  get ratedCurrent() { return statorCurrent(operatingSlip(this.params), this.params); }

  /** Per-bar rotor currents (A) for bar angles θ_k.
   *  i_r,k = (E_r/Z_r)·cos(θ_rot − φ_r − θ_k),  E_r = s·E_r0,
   *  where θ_rot = ∫s·ωe dt is the INTEGRATED rotor electrical phase
   *  (accumulated in _subStep — s(t)·ωe·t would only be valid at constant s). */
  barCurrents(thetaKs) {
    const { Xr } = reactances(this.params);   // leakage reactance at line freq
    const Rr = this.params.Rr;
    const s = this.slip;
    const Er = s * this.barEmf0;                 // induced bar emf ∝ s
    const Zr = Math.hypot(Rr, s * Xr);           // bar impedance at rotor freq s·f
    const phi = Math.atan2(s * Xr, Rr);          // emf → current lag
    const Im = Er / (Zr || 1e-9);
    const wt = this.theta_rot_elec;              // integrated rotor-frequency phase
    return thetaKs.map((thk) => Im * Math.cos(wt - phi - thk));
  }

  /** Radial flux density at a bar position (display units), rotating at ωs:
   *  B_r(θ,t) = (3/2)·Bm·cos(ωe·t − θ). */
  fluxDensityAt(thetaK) {
    return 1.5 * this.Bm * Math.cos(this.omega_e * this.time - thetaK);
  }

  /** Bar angles θ_k equally spaced around the cage. */
  barAngles() {
    const n = this.params.nBars;
    return Array.from({ length: n }, (_, k) => (2 * Math.PI * k) / n);
  }

  /**
   * Discrete Lorentz torque: the explicit Σ r·F_k over the modelled bars.
   * Tangential force on an axial bar is F = i·L·B_r, so torque = r·F.
   * Time-averaged it tracks the continuous curve (same sign, similar shape);
   * it exists to drive the per-bar force arrows, not the dynamics.
   */
  discreteTorque() {
    const th = this.barAngles();
    const ir = this.barCurrents(th);
    let sum = 0;
    for (let k = 0; k < th.length; k++) {
      sum += this.params.rRotor * ir[k] * this.params.L * this.fluxDensityAt(th[k]);
    }
    return sum;
  }

  /** Load torque (resists motion) and viscous drag at rotor speed w. */
  _loadAt(w) {
    const dir = Math.sign(w) || 0;             // a stationary rotor carries no load torque
    return { TL: this.params.TL * dir, Bw: this.params.Bfric * w };
  }

  /** Electromagnetic torque at rotor speed w for the current operating state. */
  _torqueAt(w) {
    const p = this.params;
    if (this.state === MotorState.RUNNING) {
      return torqueSlip(slipOf(w, omegaSync(p.f, p.P)), p);
    }
    if (this.state === MotorState.BRAKING) {
      // Friction brake: opposes motion, produces nothing at standstill.
      return -Math.sign(w) * this.brakeTorque;
    }
    return 0;                                  // IDLE — de-energized
  }

  /** Telemetry snapshot for the HUD and plots. */
  telemetry() {
    const p = this.params;
    const { tMax, sMax } = this.kloss;
    return {
      t: this.time,
      wr: this.omega_r,
      wrpm: (this.omega_r * 60) / (2 * Math.PI),
      ws: this.omega_s,
      wspm: this.nSyncRpm,
      s: this.slip,
      I: this.Istator,
      Irated: this.ratedCurrent,
      Te: this.Te,
      Tmax: tMax,
      sMax,
      TL: this.TL,
      fr: this.slip * p.f,
      flux: this.Bm,
      stalled: this.stalled,
      state: this.state,
    };
  }

  /** Energize the stator. ωr keeps its physical value (restart from a coast). */
  start() {
    this.state = MotorState.RUNNING;
    this.stalled = false;
  }

  /** De-energize: torque collapses to zero, the rotor coasts down. */
  stop() {
    this.state = MotorState.IDLE;
    this.stalled = false;
  }

  /** Emergency braking — a large torque opposing rotation. */
  brake() {
    this.state = MotorState.BRAKING;
    this.brakeTorque = Math.max(20, 1.2 * this.kloss.tMax);
    this.stalled = false;
  }

  reset() {
    this.state = MotorState.IDLE;
    this.time = 0; this.omega_r = 0; this.theta_r = 0; this.slip = 1;
    this.Te = 0; this.TL = 0; this.Istator = 0; this.brakeTorque = 0;
    this.stalled = false; this.theta_rot_elec = 0;
    this.history.length = 0; this._accum = 0;
  }

  /** Live parameter edits (voltage, frequency, J, TL, Rr…) with no reset. */
  setParams(patch) { Object.assign(this.params, patch); }

  /** Advance by dtWall seconds, drained in fixed 1 ms physics sub-steps. */
  step(dtWall) {
    const H = 0.001;                       // 1 ms — 20 samples/cycle at 50 Hz
    this._accum = Math.min(this._accum + dtWall, 0.05);  // anti spiral-of-death
    while (this._accum >= H) { this._subStep(H); this._accum -= H; }
  }

  /** One fixed step: RK4 on ωr, explicit update of angle and telemetry. */
  _subStep(h) {
    const p = this.params;

    if (this.state !== MotorState.IDLE) this.time += h;   // field exists only when energized
    this.slip = slipOf(this.omega_r, this.omega_s);

    // Integrated rotor electrical phase: dθ/dt = s·ωe (rotor-frequency phase).
    // Accumulated — NOT s·ωe·t, which is only exact at constant slip and
    // distorts (ωe·t·ds/dt term) through the whole run-up transient.
    if (this.state !== MotorState.IDLE) {
      this.theta_rot_elec = (this.theta_rot_elec + this.slip * this.omega_e * h) % (2 * Math.PI);
    }

    // dω/dt = (T_e(ω) − T_L·sgn(ω) − B·ω)/J
    const deriv = (w) => {
      const { TL, Bw } = this._loadAt(w);
      return (this._torqueAt(w) - TL - Bw) / p.J;
    };

    this.Te = this._torqueAt(this.omega_r);
    this.TL = this._loadAt(this.omega_r).TL;

    // Breakaway check: a stationary rotor only starts turning if the starting
    // torque beats the load. Otherwise it stays put (a real stall, no creep).
    if (this.state === MotorState.RUNNING && this.omega_r === 0 && !this.stalled) {
      if (Math.abs(this.Te) <= Math.abs(p.TL)) this.stalled = true;
    }
    if (this.state === MotorState.RUNNING && this.stalled && Math.abs(this.Te) > Math.abs(p.TL)) {
      this.stalled = false;                 // load fell (slider) — the motor breaks away
    }

    if (this.state === MotorState.RUNNING && !this.stalled) {
      this.omega_r = rk4(this.omega_r, h, deriv);        // smooth acceleration
    } else if (this.state !== MotorState.RUNNING) {
      // IDLE / BRAKING: drag and brake torque are stiff about ω = 0, where
      // sgn(ω) flips. Semi-implicit Euler plus a zero-crossing latch stops the
      // speed from chattering through zero.
      let w2 = this.omega_r + deriv(this.omega_r) * h;
      if (this.omega_r !== 0 && Math.sign(w2) !== Math.sign(this.omega_r)) w2 = 0;
      if (this.state === MotorState.BRAKING && Math.abs(w2) < 0.5) w2 = 0;
      this.omega_r = w2;
    }

    // A coasting rotor loses its last fraction of a rad/s to viscous drag and
    // is reported as stopped (a purely viscous decay only approaches zero).
    if (this.state !== MotorState.RUNNING && Math.abs(this.omega_r) < 0.05) this.omega_r = 0;

    // Rotor mechanical angle, same positive sense as the RMF
    this.theta_r = (this.theta_r + this.omega_r * h) % (2 * Math.PI);
    if (this.theta_r < 0) this.theta_r += 2 * Math.PI;

    // Stator current from the equivalent circuit (0 when de-energized)
    this.Istator = this.state === MotorState.IDLE
      ? 0 : statorCurrent(Math.max(this.slip, 1e-4), p);

    // Telemetry @ ~120 Hz, ~12 s of history
    const last = this.history[this.history.length - 1];
    if (!last || this.time - last.t >= 1 / 120) {
      this.history.push({ t: this.time, wr: this.omega_r, ws: this.omega_s,
                          s: this.slip, I: this.Istator, Te: this.Te });
      if (this.history.length > 1500) this.history.shift();
    }
  }
}

/** Classic RK4 step for the scalar ODE y' = f(y). */
export function rk4(y, h, f) {
  const k1 = f(y);
  const k2 = f(y + (h / 2) * k1);
  const k3 = f(y + (h / 2) * k2);
  const k4 = f(y + h * k3);
  return y + (h / 6) * (k1 + 2 * k2 + 2 * k3 + k4);
}
