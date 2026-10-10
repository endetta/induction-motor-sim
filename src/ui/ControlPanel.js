/**
 * ControlPanel.js
 * ---------------------------------------------------------------------------
 * Binds HTML controls ↔ the physics model, owns the telemetry dashboard and
 * drives the starting-sequence callouts.
 *
 * Exposed bindings (ids in index.html):
 *   sliders:  vph, freq, poles, J, TL, Rr, animSpeed, explode
 *   buttons:  btnStart, btnStop, btnBrake, btnReset, btnStepMode, btnStepNext,
 *             cutaway select, jumper select
 *   toggles:  chkCasing, chkFins, chkRMF, chkBarI, chkForce, chkStator,
 *             chkPhaseField, chkResultant, chkWire, chkLid,
 *             chkSlowMo (0.1× edukasi slow-mo)
 *   readouts: ro-*, canvas plots: plotSpeed, plotCurrent
 */

import { TelemetryPlot } from './TelemetryPlot.js';

const $ = (id) => document.getElementById(id);

export class ControlPanel {
  /**
   * @param {InductionMotorModel} model  physics core (already constructed)
   * @param {{onCutaway, onExplode, onJumper, onVisibility}} hooks 3D-side callbacks
   */
  constructor(model, hooks) {
    this.model = model;
    this.hooks = hooks;
    this.animSpeed = 1;
    this.slowMo = false;      // "Mode Edukasi Slow-Mo" — 0.1× time scale
    this.stepMode = false;    // "Jeda per Langkah" — physics held between Next
    this._runGate = null;     // while set, physics runs until it returns true
    this._bindSliders();
    this._bindButtons();
    this._bindToggles();
    this._buildPlots();
    this._sequenceSteps = [
      ['seq1', 'Arus 3 fasa mengalir ke belitan stator'],
      ['seq2', 'RMF terbentak & berputar pada ωs'],
      ['seq3', 'Rotor diam (s=1): fluks memotong konduktor maksimum'],
      ['seq4', 'GGL induksi besar → arus rotor starting tinggi'],
      ['seq5', 'Gaya Lorentz maksimal → torsi percepatan'],
      ['seq6', 'Rotor akselerasi (RK4) menuju kecepatan nominal'],
    ];
    this._seqShown = 0;
    this._seqRendered = 0;
    this._phaseStart = 0;   // electrical time when the run began (timed phases)
    this._tePeak = 0;       // tracked Te max during run-up (step-5 detector)
    this._bindSeqExtras();
  }

  _bindSeqExtras() {
    // Slow-Mo edukasi toggle + step-by-step guide buttons (ids in index.html)
    $('chkSlowMo').addEventListener('change', (e) => { this.slowMo = e.target.checked; });
    $('btnStepMode').addEventListener('click', () => {
      this.stepMode = !this.stepMode;
      $('btnStepMode').classList.toggle('active-step', this.stepMode);
      this._runGate = null;
    });
    $('btnStepNext').addEventListener('click', () => {
      if (!this.stepMode) return;
      const target = Math.min(this._seqShown + 1, 6);
      this._runGate = () => this._seqShown >= target;
    });
  }

  _bindSliders() {
    const m = this.model;
    const bind = (id, apply, fmt) => {
      const el = $(id);
      const out = $('out-' + id);
      const sync = () => {
        const v = parseFloat(el.value);
        apply(v);
        if (out) out.textContent = fmt ? fmt(v) : String(v);
      };
      el.addEventListener('input', sync);
      sync();   // initialise from the HTML default
    };
    bind('vph',  (v) => m.setParams({ Vph: v }),      (v) => v.toFixed(0) + ' V');
    bind('freq', (v) => m.setParams({ f: v }),        (v) => v.toFixed(0) + ' Hz');
    bind('poles',(v) => m.setParams({ P: v }),        (v) => v.toFixed(0) + ' kutub');
    bind('J',    (v) => m.setParams({ J: v }),        (v) => v.toFixed(2) + ' kg·m²');
    bind('TL',   (v) => m.setParams({ TL: v }),       (v) => v.toFixed(1) + ' N·m');
    bind('Rr',   (v) => m.setParams({ Rr: v }),       (v) => v.toFixed(2) + ' Ω');
    bind('animSpeed', (v) => { this.animSpeed = v; }, (v) => v.toFixed(2) + '×');
    bind('explode', (v) => this.hooks.onExplode(v),   (v) => v.toFixed(2));
  }

  _bindButtons() {
    const m = this.model;
    $('btnStart').addEventListener('click', () => {
      m.start();
      this._seqShown = 0;                    // replay the starting sequence
      this._seqRendered = 0;
      this._phaseStart = this.model.time;    // timed-phase anchor at energize
      this._tePeak = 0;
      this._runGate = null;                  // clear any held step
    });
    $('btnStop').addEventListener('click', () => m.stop());
    $('btnBrake').addEventListener('click', () => m.brake());
    $('btnReset').addEventListener('click', () => {
      m.reset();
      this.plotSpeed.clear();
      this.plotCurrent.clear();
      this._seqShown = 0;
      this._seqRendered = 0;
      this._phaseStart = 0;
      this._tePeak = 0;
      this._runGate = null;
      this._setSequence(0);
    });
    $('cutaway').addEventListener('change', (e) => this.hooks.onCutaway(e.target.value));
    $('jumper').addEventListener('change', (e) => this.hooks.onJumper(e.target.value));
  }

  _bindToggles() {
    const chk = (id, fn) => $(id).addEventListener('change', (e) => fn(e.target.checked));
    chk('chkCasing', (v) => this.hooks.onVisibility('casing', v));
    chk('chkFins',   (v) => this.hooks.onVisibility('fins', v));
    chk('chkStator', (v) => this.hooks.onVisibility('stator', v));
    chk('chkRMF',    (v) => this.hooks.onVisibility('rmf', v));
    chk('chkPhaseField', (v) => this.hooks.onVisibility('phaseField', v));
    chk('chkResultant', (v) => this.hooks.onVisibility('netField', v));
    chk('chkBarI',   (v) => this.hooks.onVisibility('barCurrent', v));
    chk('chkForce',  (v) => this.hooks.onVisibility('force', v));
    chk('chkWire',   (v) => this.hooks.onVisibility('wire', v));
    chk('chkLid',    (v) => this.hooks.onLidOpen(v));
    $('btnFocusTermBox').addEventListener('click', () => this.hooks.onFocusTermBox());
  }

  _buildPlots() {
    this.plotSpeed = new TelemetryPlot($('plotSpeed'), [
      { label: 'ωr (rotor)', color: '#0f6e3d', get: (r) => r.wr },
      { label: 'ωs (sinkron)', color: '#0b6a99', get: (r) => r.ws },
    ]);
    this.plotCurrent = new TelemetryPlot($('plotCurrent'), [
      { label: 'I stator', color: '#8f5204', get: (r) => r.I },
      { label: 'Te', color: '#b8420f', get: (r) => r.Te / 10 },
    ]);
  }

  /** Starting-sequence callouts: true transient-timer state machine, not
   *  instant step pinning. Phases 1–2 are timed from electrical time (fixed
   *  stator-field build-up); 3–6 are physical conditions on the live telemetry:
   *    1  t ∈ [0, 0.2 s]        stator currents flow in
   *    2  t ∈ [0.2, 0.5 s]      RMF vector established, begins rotating at ωs
   *    3  s > 0.95              rotor still ≈ standstill, max flux cutting
   *    4  I at/near its transient inrush peak (locked-rotor level)
   *    5  accelerating torque Te at its peak during run-up
   *    6  s < 0.1               steady state reached
   *  Conditions only advance — a later condition never hides an earlier one. */
  _updateSequence(tel) {
    const run = tel.state === 'running';
    if (!run) return;                        // IDLE/BRAKING: keep last shown state
    const n = this._seqShown;
    let next = n;
    if (n < 1) next = 1;                                   // energized → currents flow
    if (n < 2 && this.model.time - this._phaseStart >= 0.2) next = 2;   // RMF formed
    if (n < 3 && this.model.time - this._phaseStart >= 0.5) next = 3;   // field cuts rotor
    if (n < 4 && tel.s > 0.95 && tel.I > 0.5 * tel.Irated) next = 4;    // big induced current
    if (n < 5) {
      // Te peak: track the max accelerating torque seen since step 4 armed
      if (n >= 4 || tel.Te > 0.2 * tel.Tmax) {
        if (tel.Te > (this._tePeak || 0)) this._tePeak = tel.Te;
        // peak passed: torque now clearly below its tracked maximum
        if (this._tePeak > 0.3 * tel.Tmax && tel.Te < 0.8 * this._tePeak) next = Math.max(next, 5);
      }
    }
    if (n < 6 && tel.s < 0.1) next = 6;                    // steady state
    if (next !== n) {
      this._seqShown = next;
      // re-anchor the timed phases when a physical condition fires early
      if (next >= 3) this._phaseStart = this.model.time - 0.5;
      this._setSequence(next);
    }
  }

  _setSequence(n) {
    if (this._seqRendered === n) return;
    this._seqRendered = n;
    for (let i = 0; i < 6; i++) {
      const el = $(this._sequenceSteps[i][0]);
      if (!el) continue;
      el.classList.toggle('done', i < n);
      el.classList.toggle('active', i === n - 1);
    }
  }

  /** Advance-by-gate for step mode: true when physics may integrate this frame.
   *  In step mode physics is held until "Next" arms a run-until-next-phase gate. */
  physicsGate() {
    if (!this.stepMode) return true;
    if (!this._runGate) return false;
    if (this._runGate()) { this._runGate = null; return false; }
    return true;
  }

  /** Push one telemetry row into the plots (call at ~30 Hz). */
  sample(tel) {
    this.plotSpeed.push(tel);
    this.plotCurrent.push(tel);
  }

  /** Refresh numeric readouts + plots. Call every frame (cheap). */
  update(tel) {
    const fmt = (x, d = 1) => x.toFixed(d);
    $('ro-state').textContent =
      tel.state === 'running' ? (tel.stalled ? 'STALL' : 'RUNNING')
      : tel.state === 'braking' ? 'BRAKING' : 'IDLE';
    $('ro-state').className = 'ro-state ' + (tel.stalled ? 'stall' : tel.state);
    $('ro-rpm').textContent  = fmt(tel.wrpm, 0);
    $('ro-sync').textContent = fmt(tel.wspm, 0);
    $('ro-slip').textContent = (tel.s * 100).toFixed(2) + ' %';
    $('ro-fr').textContent   = fmt(tel.fr, 2) + ' Hz';
    $('ro-current').textContent = fmt(tel.I, 1);
    $('ro-iratio').textContent  = (tel.I / Math.max(tel.Irated, 1e-6)).toFixed(2) + '× In';
    $('ro-torque').textContent  = fmt(tel.Te, 1);
    $('ro-tmax').textContent    = fmt(tel.Tmax, 0);
    $('ro-flux').textContent    = tel.flux.toFixed(2);
    $('ro-t').textContent       = fmt(tel.t, 2);
    this.plotSpeed.draw();
    this.plotCurrent.draw();
    this._updateSequence(tel);
  }
}
