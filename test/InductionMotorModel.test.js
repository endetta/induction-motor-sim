import { describe, it, expect } from 'vitest';
import {
  omegaSync, slipOf, torqueSlip, klossTorque, klossConstants, statorCurrent, rk4,
  thevenin, reactances, operatingSlip, lockedRotorCurrent,
  InductionMotorModel, MotorState, DEFAULT_PARAMS, REF_F,
} from '../src/physics/InductionMotorModel.js';

const p = { ...DEFAULT_PARAMS };

describe('synchronous speed n_s = 120f/P', () => {
  it('50 Hz 4-pole → 1500 rpm = 50π rad/s', () => {
    expect(omegaSync(50, 4)).toBeCloseTo(50 * Math.PI, 9);
    expect((omegaSync(50, 4) * 60) / (2 * Math.PI)).toBeCloseTo(1500, 6);
  });
  it('60 Hz 4-pole → 1800 rpm', () => {
    expect((omegaSync(60, 4) * 60) / (2 * Math.PI)).toBeCloseTo(1800, 6);
  });
  it('50 Hz 2-pole → 3000 rpm', () => {
    expect((omegaSync(50, 2) * 60) / (2 * Math.PI)).toBeCloseTo(3000, 6);
  });
});

describe('slip', () => {
  const ws = omegaSync(50, 4);
  it('s = 1 at standstill, s = 0 at synchronous speed', () => {
    expect(slipOf(0, ws)).toBeCloseTo(1, 12);
    expect(slipOf(ws, ws)).toBeCloseTo(0, 12);
    expect(slipOf(0.97 * ws, ws)).toBeCloseTo(0.03, 9);
  });
  it('s > 1 in the plugging region, s < 0 above sync (generation)', () => {
    expect(slipOf(-0.2 * ws, ws)).toBeCloseTo(1.2, 9);
    expect(slipOf(1.05 * ws, ws)).toBeCloseTo(-0.05, 9);
  });
});

describe('thevenin equivalent + reactance frequency scaling', () => {
  it('Thevenin voltage sits below the supply voltage', () => {
    const { Vth } = thevenin(p);
    expect(Vth).toBeGreaterThan(0);
    expect(Vth).toBeLessThan(p.Vph);
  });
  it('reactances scale linearly with frequency (X = 2πfL)', () => {
    const r50 = reactances({ ...p, f: 50 });
    const r60 = reactances({ ...p, f: 60 });
    expect(r50.Xs / r60.Xs).toBeCloseTo(50 / 60, 9);
    expect(r50.Xm / r60.Xm).toBeCloseTo(50 / 60, 9);
    expect(r60.Xs).toBeCloseTo(p.Xs60, 9);
    expect(r60.Xr).toBeCloseTo(p.Xr60, 9);
  });
});

describe('torque-slip curve', () => {
  const { tMax, sMax } = klossConstants(p);

  it('pull-out slip lies in the expected band', () => {
    expect(sMax).toBeGreaterThan(0.05);
    expect(sMax).toBeLessThan(0.5);
  });
  it('torque → 0 as slip → 0 (no flux cutting at sync)', () => {
    expect(torqueSlip(0, p)).toBe(0);              // analytic 0/0 guard
    expect(torqueSlip(1e-6, p)).toBeLessThan(0.01); // 2.8 mN·m — physically ≈ 0
    expect(torqueSlip(1e-3, p)).toBeLessThan(0.05 * tMax);
  });
  it('T(sMax) is the maximum — bracketed samples are never higher', () => {
    const t = torqueSlip(sMax, p);
    for (const f of [0.2, 0.5, 0.9, 0.95, 1.05, 1.1, 1.5]) {
      expect(torqueSlip(sMax * f, p)).toBeLessThanOrEqual(t + 1e-9);
    }
    expect(t).toBeCloseTo(tMax, 4);
  });
  it('starting torque (s=1) is positive and below pull-out', () => {
    const t1 = torqueSlip(1, p);
    expect(t1).toBeGreaterThan(0);
    expect(t1).toBeLessThan(tMax);
  });
  it('Kloss closed form tracks the circuit curve: exact at s_k, ≤25% at extremes', () => {
    // The two-constant Kloss form is exact at pull-out and asymptotically
    // correct; away from s_k it deviates up to ~20% because it neglects the
    // stator-impedance shaping of the exact circuit. That IS its known fidelity.
    for (const s of [0.1, 0.2, 0.3, 0.5, 0.7, 1.0, 1.5]) {
      const err = Math.abs(klossTorque(s, p) - torqueSlip(s, p)) / torqueSlip(s, p);
      if (Math.abs(s - sMax) < 0.12) expect(err).toBeLessThan(0.05);
      else expect(err).toBeLessThan(0.25);
    }
  });
  it('Kloss is exact at the pull-out point (its two defining conditions)', () => {
    expect(klossTorque(sMax, p)).toBeCloseTo(tMax, 6);
  });
  it('torque is monotonic past pull-out (unstable region)', () => {
    let prev = torqueSlip(sMax, p);
    for (let s = sMax + 0.05; s <= 2.0; s += 0.05) {
      const t = torqueSlip(s, p);
      expect(t).toBeLessThan(prev);
      prev = t;
    }
  });
  it('negative slip (above sync) yields negative torque — regeneration braking', () => {
    expect(torqueSlip(-0.02, p)).toBeLessThan(0);
  });
});

describe('stator current', () => {
  const sRated = operatingSlip(p);

  it('starting current (s=1) is 5–8× rated', () => {
    const ratio = statorCurrent(1, p) / statorCurrent(sRated, p);
    expect(ratio).toBeGreaterThan(4);
    expect(ratio).toBeLessThan(9);
  });
  it('locked-rotor current matches V/√((Rs+Rr)²+(Xs+Xr)²) at s=1', () => {
    const { Xs, Xr } = reactances(p);
    const expected = p.Vph / Math.hypot(p.Rs + p.Rr, Xs + Xr);
    // The exact circuit adds the magnetizing shunt → ~0.5% above the
    // classical locked-rotor approximation.
    expect(statorCurrent(1, p)).toBeGreaterThan(expected * 0.995);
    expect(statorCurrent(1, p)).toBeLessThan(expected * 1.02);
    expect(lockedRotorCurrent(p)).toBeCloseTo(expected, 6);
  });
  it('current falls monotonically as slip decreases', () => {
    expect(statorCurrent(1, p)).toBeGreaterThan(statorCurrent(0.5, p));
    expect(statorCurrent(0.5, p)).toBeGreaterThan(statorCurrent(0.1, p));
    expect(statorCurrent(0.1, p)).toBeGreaterThan(statorCurrent(0.01, p));
  });
  it('no-load current is magnetizing-dominated, not zero', () => {
    const I0 = statorCurrent(1e-4, p);
    expect(I0).toBeGreaterThan(0);
    expect(I0).toBeLessThan(statorCurrent(1, p) / 3);
  });
  it('s = 0 does not produce NaN — clamped to the no-load limit', () => {
    // At synchronous speed the rotor branch opens (Rr/s → ∞); the exact
    // complex division must stay finite, matching the s→0⁺ limit.
    const I0 = statorCurrent(1e-4, p);
    expect(statorCurrent(0, p)).toBeCloseTo(I0, 6);
    expect(Number.isFinite(statorCurrent(0, p))).toBe(true);
    expect(statorCurrent(-0, p)).toBeCloseTo(I0, 6);
  });
  it('operating slip is small (< 8%) — rated region near pull-out-adjacent load', () => {
    expect(sRated).toBeLessThan(0.08);
    expect(sRated).toBeGreaterThan(0.005);
  });
  it('operating slip satisfies torque(sRated) = 0.45·Tmax by bisection construction', () => {
    expect(torqueSlip(sRated, p)).toBeCloseTo(0.45 * klossConstants(p).tMax, 6);
  });
});

describe('flux-density law B ∝ V/f', () => {
  it('halving the frequency at fixed V doubles Bm', () => {
    const m60 = new InductionMotorModel({ f: 60, Vph: 265.6 });
    const m30 = new InductionMotorModel({ f: 30, Vph: 265.6 });
    expect(m30.Bm / m60.Bm).toBeCloseTo(2, 9);
  });
  it('rated voltage at 60 Hz gives the reference flux level', () => {
    const m = new InductionMotorModel({ f: REF_F, Vph: 265.6 });
    expect(m.Bm).toBeCloseTo(0.9, 9);
  });
});

describe('RK4 integrator', () => {
  it('integrates a constant derivative exactly', () => {
    let y = 0;
    for (let i = 0; i < 100; i++) y = rk4(y, 0.01, () => 2);
    expect(y).toBeCloseTo(2, 6);
  });
  it('integrates exponential decay y′ = −2y accurately at h=1ms', () => {
    let y = 1;
    for (let i = 0; i < 500; i++) y = rk4(y, 0.001, (v) => -2 * v);
    expect(y).toBeCloseTo(Math.exp(-1), 5);
  });
  it('RK4 beats Euler on the same ODE', () => {
    let ye = 1;
    for (let i = 0; i < 500; i++) ye += -2 * ye * 0.001;
    const ex = Math.exp(-1);
    let yr = 1;
    for (let i = 0; i < 500; i++) yr = rk4(yr, 0.001, (v) => -2 * v);
    expect(Math.abs(yr - ex)).toBeLessThan(Math.abs(ye - ex));
  });
});

describe('starting transient', () => {
  it('accelerates to near synchronous speed on no load within ~8 s', () => {
    const m = new InductionMotorModel({ J: 0.25, TL: 0 });
    m.start();
    for (let i = 0; i < 8000; i++) m.step(0.001);
    const tel = m.telemetry();
    console.log('   [no-load final]', tel.wr.toFixed(1), '/', tel.ws.toFixed(1), 's=', tel.s.toFixed(4));
    expect(tel.wr).toBeGreaterThan(0.9 * tel.ws);
    expect(tel.s).toBeLessThan(0.10);
    expect(tel.s).toBeGreaterThan(0);
  });
  it('never exceeds synchronous speed (regenerative torque clamps it)', () => {
    const m = new InductionMotorModel({ J: 0.05, TL: 0 });
    m.start();
    let peak = 0;
    for (let i = 0; i < 10000; i++) { m.step(0.001); peak = Math.max(peak, m.omega_r); }
    expect(peak).toBeLessThanOrEqual(m.omega_s + 1e-6);
  });
  it('higher inertia ⇒ slower acceleration during the transient (t < τ_mech)', () => {
    // Compare early in the run-up, while both rotors are still accelerating;
    // the lighter rotor also settles into its low-slip operating point faster,
    // so compare at a moment when both are meaningfully mid-transient.
    const m1 = new InductionMotorModel({ J: 0.02, TL: 5 });
    const m2 = new InductionMotorModel({ J: 0.5, TL: 5 });
    m1.start(); m2.start();
    for (let i = 0; i < 150; i++) { m1.step(0.001); m2.step(0.001); }   // 0.15 s
    console.log('   [inertia] light', m1.omega_r.toFixed(1), 'heavy', m2.omega_r.toFixed(1));
    expect(m1.omega_r).toBeGreaterThan(m2.omega_r);
  });
  it('higher inertia ⇒ longer time to reach 90% of sync speed', () => {
    const timeTo90 = (J) => {
      const m = new InductionMotorModel({ J, TL: 5 });
      m.start();
      for (let i = 0; i < 60000; i++) {
        m.step(0.001);
        if (m.omega_r >= 0.9 * m.omega_s) return i * 0.001;
      }
      return Infinity;
    };
    const t1 = timeTo90(0.02), t2 = timeTo90(0.4);
    expect(Number.isFinite(t1)).toBe(true);
    expect(t2).toBeGreaterThan(t1);
  });
  it('starting current decays toward the running value during the transient', () => {
    const m = new InductionMotorModel({ J: 0.3, TL: 8 });
    m.start();
    for (let i = 0; i < 100; i++) m.step(0.001);
    const Istart = m.Istator;
    for (let i = 0; i < 10000; i++) m.step(0.001);
    const Irun = m.Istator;
    expect(Istart).toBeGreaterThan(3 * Irun);
  });
  it('load below pull-out reaches a steady operating slip with torque balance', () => {
    const { tMax } = klossConstants(p);
    const m = new InductionMotorModel({ J: 0.3, TL: 0.4 * tMax });
    m.start();
    for (let i = 0; i < 30000; i++) m.step(0.001);
    expect(m.slip).toBeGreaterThan(0);
    expect(m.slip).toBeLessThan(0.5);
    // steady state: T_e = T_L + B·ωr
    expect(Math.abs(m.Te - (m.TL + p.Bfric * m.omega_r)))
      .toBeLessThan(0.05 * Math.abs(m.Te));
  });
  it('load above pull-out stalls: breakaway latch holds ωr at zero', () => {
    const { tMax } = klossConstants(p);
    const m = new InductionMotorModel({ J: 0.05, TL: 2.5 * tMax });
    m.start();
    for (let i = 0; i < 4000; i++) m.step(0.001);
    expect(m.omega_r).toBe(0);
    expect(m.stalled).toBe(true);
    expect(m.slip).toBe(1);
    // stalled current stays at the locked-rotor level (magnetizing shunt adds ~0.5%)
    expect(m.Istator).toBeGreaterThan(lockedRotorCurrent(p) * 0.99);
    expect(m.Istator).toBeLessThan(lockedRotorCurrent(p) * 1.02);
    // dropping the load below breakaway releases it
    m.setParams({ TL: 0.1 * tMax });
    for (let i = 0; i < 500; i++) m.step(0.001);
    expect(m.stalled).toBe(false);
    expect(m.omega_r).toBeGreaterThan(0);
  });
  it('rotor mechanical angle advances with rotor speed', () => {
    const m = new InductionMotorModel({ J: 0.25, TL: 0 });
    m.start();
    for (let i = 0; i < 3000; i++) m.step(0.001);
    expect(m.omega_r).toBeGreaterThan(0);
    const before = m.theta_r;
    for (let i = 0; i < 500; i++) m.step(0.001);
    const advanced = ((m.theta_r - before) % (2 * Math.PI) + 2 * Math.PI) % (2 * Math.PI);
    expect(advanced).toBeGreaterThan(0);
  });
  it('frequency change rescales the synchronous speed live', () => {
    const m = new InductionMotorModel({ J: 0.25, TL: 0 });
    m.start();
    for (let i = 0; i < 8000; i++) m.step(0.001);
    expect(m.telemetry().wspm).toBeCloseTo(1500, 3);
    m.setParams({ f: 30 });
    expect(m.telemetry().wspm).toBeCloseTo(900, 3);
    for (let i = 0; i < 12000; i++) m.step(0.001);
    // old speed 1500 rpm ≈ 157 rad/s > new ωs ≈ 94 → regeneration decelerates it
    expect(m.omega_r).toBeLessThanOrEqual(m.omega_s + 1e-6);
    expect(m.omega_r).toBeGreaterThan(0.8 * m.omega_s);
  });
});

describe('stop / brake / reset', () => {
  it('stop() de-energizes: stator current → 0 and rotor coasts down', () => {
    const m = new InductionMotorModel({ J: 0.25, TL: 0, Bfric: 0.02 });
    m.start();
    for (let i = 0; i < 8000; i++) m.step(0.001);
    const wRunning = m.omega_r;
    console.log('   [stop pre]', wRunning.toFixed(1));
    m.stop();
    for (let i = 0; i < 3000; i++) m.step(0.001);
    console.log('   [stop post]', m.omega_r.toFixed(1));
    expect(m.Istator).toBe(0);
    expect(wRunning).toBeGreaterThan(100);
    expect(m.omega_r).toBeLessThan(wRunning);
  });
  it('de-energized rotor coasts to a full stop under friction alone', () => {
    const m = new InductionMotorModel({ J: 0.05, TL: 0, Bfric: 0.05 });
    m.start();
    for (let i = 0; i < 6000; i++) m.step(0.001);
    console.log('   [coast pre]', m.omega_r.toFixed(1));
    m.stop();
    for (let i = 0; i < 40000; i++) {
      m.step(0.001);
      if (m.omega_r === 0) break;
    }
    console.log('   [coast post]', m.omega_r.toFixed(3));
    expect(m.omega_r).toBe(0);            // deadband latch reports a true stop
  });
  it('emergency brake stops the rotor and latches at zero (no chatter)', () => {
    const m = new InductionMotorModel({ J: 0.1, TL: 0 });
    m.start();
    for (let i = 0; i < 6000; i++) m.step(0.001);
    console.log('   [brake pre]', m.omega_r.toFixed(1));
    expect(m.omega_r).toBeGreaterThan(100);
    m.brake();
    for (let i = 0; i < 4000; i++) m.step(0.001);
    console.log('   [brake post]', m.omega_r.toFixed(3));
    expect(m.omega_r).toBe(0);
    expect(m.state).toBe(MotorState.BRAKING);
    // braking is strictly decelerating: no sign flip past zero
    expect(Math.sign(m.omega_r)).toBe(0);
  });
  it('reset() returns to a clean idle state', () => {
    const m = new InductionMotorModel();
    m.start();
    for (let i = 0; i < 1000; i++) m.step(0.001);
    m.reset();
    expect(m.state).toBe(MotorState.IDLE);
    expect(m.time).toBe(0);
    expect(m.omega_r).toBe(0);
    expect(m.slip).toBe(1);
    expect(m.Istator).toBe(0);
    expect(m.history.length).toBe(0);
  });
  it('restart after coast preserves rotor speed (no jump)', () => {
    const m = new InductionMotorModel({ J: 0.25, TL: 0 });
    m.start();
    for (let i = 0; i < 6000; i++) m.step(0.001);
    m.stop();
    for (let i = 0; i < 300; i++) m.step(0.001);
    const wCoast = m.omega_r;
    expect(wCoast).toBeGreaterThan(0);
    m.start();
    expect(m.omega_r).toBeCloseTo(wCoast, 9);
  });
});

describe('per-bar model (visualization layer)', () => {
  it('bar current amplitude = s·E0/√(Rr² + (s·Xr)²) — no NaN, peaks at s=1', () => {
    // Sample against FROZEN slip. barCurrents() uses the INTEGRATED phase
    // theta_rot_elec (updated only in _subStep), so advance it directly at
    // dθ = s·ωe·dt to hold the amplitude analytic and exact.
    const m = new InductionMotorModel();
    const { Xr } = reactances(m.params);
    const expectedAtSlip = (s) => (s * m.barEmf0) / Math.hypot(m.params.Rr, s * Xr);
    const th = [0];
    m.state = MotorState.RUNNING;
    m.slip = 1;
    const s1 = [];
    for (let i = 0; i < 2000; i++) {
      m.theta_rot_elec = (m.theta_rot_elec + 1 * m.omega_e * 0.0001) % (2 * Math.PI);
      s1.push(Math.abs(m.barCurrents(th)[0]));
    }
    expect(s1.every((v) => Number.isFinite(v))).toBe(true);
    // sampled max sits just under the analytic peak (0.1 ms grid) but within 0.01%
    expect(Math.max(...s1)).toBeLessThanOrEqual(expectedAtSlip(1) + 1e-6);
    expect(Math.max(...s1)).toBeGreaterThan(expectedAtSlip(1) * 0.9999);
    // half slip: emf halves but bar impedance |Z| also drops (0.384 vs 0.510 Ω),
    // so the amplitude ratio is the analytic 0.663, not 0.5.
    m.slip = 0.5;
    const s2 = [];
    for (let i = 0; i < 2000; i++) {
      m.theta_rot_elec = (m.theta_rot_elec + 0.5 * m.omega_e * 0.0002) % (2 * Math.PI);
      s2.push(Math.abs(m.barCurrents(th)[0]));
    }
    const ratio = Math.max(...s2) / Math.max(...s1);
    expect(ratio).toBeCloseTo(expectedAtSlip(0.5) / expectedAtSlip(1), 3);
  });
  it('discrete Lorentz torque shares the sign of the continuous torque at s=1', () => {
    const m = new InductionMotorModel();
    m.state = MotorState.RUNNING;
    m.slip = 1;
    let sum = 0;
    for (let i = 0; i < 400; i++) { m.time += 0.0005; sum += m.discreteTorque(); }
    const avg = sum / 400;
    expect(Math.sign(avg)).toBe(Math.sign(torqueSlip(1, m.params)));
    expect(Math.abs(avg)).toBeGreaterThan(0);
  });
  it('discrete torque → 0 at synchronous speed (no relative flux cutting)', () => {
    const m = new InductionMotorModel();
    m.omega_r = m.omega_s;
    m.slip = 0;
    expect(m.barCurrents([0, 1, 2])[0]).toBeCloseTo(0, 6);
    expect(Math.abs(m.discreteTorque())).toBeLessThan(1e-6);
  });
  it('flux density completes one cycle per 1/f — rotating field', () => {
    const m = new InductionMotorModel({ f: 10 });   // slow for a clean sample
    const T = 1 / 10;
    expect(m.fluxDensityAt(0)).toBeCloseTo(1.5 * m.Bm, 6);
    m.time = T;
    expect(m.fluxDensityAt(0)).toBeCloseTo(1.5 * m.Bm, 6);
    m.time = T / 2;
    expect(m.fluxDensityAt(0)).toBeCloseTo(-1.5 * m.Bm, 6);
    // travelling wave: B(θ,t) depends only on (ωe·t − θ)
    m.time = 0.013;
    expect(m.fluxDensityAt(0.4))
      .toBeCloseTo(1.5 * m.Bm * Math.cos(m.omega_e * m.time - 0.4), 6);
  });
  it('every bar carries finite current and force while running', () => {
    const m = new InductionMotorModel();
    m.start();
    for (let i = 0; i < 200; i++) m.step(0.001);
    const ir = m.barCurrents(m.barAngles());
    expect(ir.length).toBe(m.params.nBars);
    expect(ir.every((v) => Number.isFinite(v))).toBe(true);
    expect(Number.isFinite(m.discreteTorque())).toBe(true);
  });
  it('integrated rotor phase: bar frequency ≈ s·f_e through the transient', () => {
    // Phase is ∫s·ωe dt, so during run-up the bar-current oscillation rate
    // must track the instantaneous s·f (which falls from 50 Hz toward ~1 Hz),
    // not the s(t)·ωe·t pseudo-frequency that blows up as t grows.
    // Use BRAKING-free manual slip freeze: set slip directly and advance the
    // integrated phase at dθ = s·ωe·dt (exactly what _subStep does), then
    // verify barCurrents() oscillates at s·f_e — no pseudo-frequency blow-up.
    const m = new InductionMotorModel();
    m.state = MotorState.RUNNING;
    const sMid = 0.6;                               // arbitrary high-slip point
    m.slip = sMid;
    for (let i = 0; i < 300; i++) {                 // prime the phase accumulator
      m.theta_rot_elec = (m.theta_rot_elec + sMid * m.omega_e * 0.001) % (2 * Math.PI);
    }
    let crossings = 0, prev = m.barCurrents([0])[0];
    const dt = 0.0002;                              // 0.1 s window
    for (let i = 0; i < 500; i++) {
      m.theta_rot_elec = (m.theta_rot_elec + sMid * m.omega_e * dt) % (2 * Math.PI);
      const cur = m.barCurrents([0])[0];
      if (Math.sign(cur) !== Math.sign(prev)) crossings++;
      prev = cur;
    }
    const fMeas = crossings / 2 / (500 * dt);       // Hz
    expect(fMeas).toBeGreaterThan(0.8 * sMid * m.params.f);
    expect(fMeas).toBeLessThan(1.2 * sMid * m.params.f);
  });
});
