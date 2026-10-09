/**
 * ControlPanel.js
 * ---------------------------------------------------------------------------
 * Binds HTML controls ↔ the physics model, owns the telemetry dashboard and
 * drives the starting-sequence callouts.
 *
 * Exposed bindings (ids in index.html):
 *   sliders:  vph, freq, poles, J, TL, Rr, animSpeed, explode
 *   buttons:  btnStart, btnStop, btnBrake, btnReset, cutaway select
 *   toggles:  chkCasing, chkFins, chkRMF, chkBarI, chkForce, chkStator, chkRing
 *   readouts: ro-*, canvas plots: plotSpeed, plotCurrent
 */

import { TelemetryPlot } from './TelemetryPlot.js';

const $ = (id) => document.getElementById(id);

export class ControlPanel {
  /**
   * @param {InductionMotorModel} model  physics core (already constructed)
   * @param {{onCutaway, onExplode, onVisibility}} hooks 3D-side callbacks
   */
  constructor(model, hooks) {
    this.model = model;
    this.hooks = hooks;
    this.animSpeed = 1;
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
    });
    $('btnStop').addEventListener('click', () => m.stop());
    $('btnBrake').addEventListener('click', () => m.brake());
    $('btnReset').addEventListener('click', () => {
      m.reset();
      this.plotSpeed.clear();
      this.plotCurrent.clear();
      this._seqShown = 0;
      this._setSequence(0);
    });
    $('cutaway').addEventListener('change', (e) => this.hooks.onCutaway(e.target.value));
  }

  _bindToggles() {
    const chk = (id, fn) => $(id).addEventListener('change', (e) => fn(e.target.checked));
    chk('chkCasing', (v) => this.hooks.onVisibility('casing', v));
    chk('chkFins',   (v) => this.hooks.onVisibility('fins', v));
    chk('chkStator', (v) => this.hooks.onVisibility('stator', v));
    chk('chkRMF',    (v) => this.hooks.onVisibility('rmf', v));
    chk('chkBarI',   (v) => this.hooks.onVisibility('barCurrent', v));
    chk('chkForce',  (v) => this.hooks.onVisibility('force', v));
    chk('chkRing',   (v) => this.hooks.onVisibility('speedRing', v));
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

  /** Starting-sequence callouts: highlight steps 1..6 as physics reaches them. */
  _updateSequence(tel) {
    const run = tel.state === 'running';
    const done = (c) => { if (run && c && this._seqShown < 6) this._seqShown = Math.max(this._seqShown, 6); };
    void done;
    let n = 0;
    if (run || tel.state === 'braking') {
      n = 1;                                              // currents flowing
      if (this.model.time > 0.05) n = 2;                  // RMF established
      if (tel.s > 0.95) n = Math.max(n, 3);               // rotor still ≈ standstill
      if (tel.s > 0.90) n = Math.max(n, 4);               // big induced current
      if (tel.Te > 0.2 * tel.Tmax) n = Math.max(n, 5);    // force/torque built
      if (tel.s < 0.8) n = 6;                             // accelerating / near speed
      if (tel.s < 0.05) n = 6;                            // steady state reached
    }
    this._setSequence(n);
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
