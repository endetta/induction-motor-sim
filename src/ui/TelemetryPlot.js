/**
 * TelemetryPlot.js — lightweight canvas strip-chart, no dependencies.
 * Draws ωr (actual) and ωs (synchronous) traces plus stator current.
 * Time-window zoom: "Fokus Transien Starting (0–2 s)" vs "Kontinu (12 s)"
 * buttons so the S-curve speed rise and inrush decay read clearly.
 */
export class TelemetryPlot {
  /**
   * @param {HTMLCanvasElement} canvas
   * @param {{label:string, color:string, get:(row)=>number}[]} series
   * @param {number} height css px
   */
  constructor(canvas, series, height = 110) {
    this.canvas = canvas;
    this.ctx = canvas.getContext('2d');
    this.series = series;
    this.height = height;
    this.data = [];
    this.maxPoints = 360;              // ~12 s at 30 Hz sampling
    this.autoMax = 1;
    this.tStart = 0;                   // visible time window [tStart, tStart+span]
    this.span = 12;                    // 'continu' default; 'transient' → 2 s
    // time-window zoom buttons: starting transient (0–2 s) vs continuous (12 s)
    const bar = document.createElement('div');
    bar.style.cssText = 'display:flex;gap:4px;margin-top:4px';
    const mkBtn = (txt, spanSec) => {
      const b = document.createElement('button');
      b.textContent = txt;
      b.style.cssText = 'font:600 10px "JetBrains Mono",monospace;padding:2px 6px;border-radius:5px;' +
        'border:1px solid #d8dfe8;background:#eef1f6;cursor:pointer';
      b.addEventListener('click', () => { this.span = spanSec; this.tStart = 0; });
      return b;
    };
    bar.appendChild(mkBtn('Transien Starting (0–2 s)', 2));
    bar.appendChild(mkBtn('Kontinu (12 s)', 12));
    canvas.insertAdjacentElement('afterend', bar);
    // handle devicePixelRatio for crisp lines
    const dpr = window.devicePixelRatio || 1;
    const w = canvas.clientWidth || 320;
    canvas.width = w * dpr;
    canvas.height = height * dpr;
    this.ctx.scale(dpr, dpr);
    this.w = w;
    this.h = height;
  }

  push(row) {
    this.data.push(row);
    if (this.data.length > this.maxPoints) this.data.shift();
  }

  clear() { this.data.length = 0; this.draw(); }

  draw() {
    const { ctx, w, h } = this;
    ctx.clearRect(0, 0, w, h);
    // grid
    ctx.strokeStyle = 'rgba(26,35,48,0.10)';
    ctx.lineWidth = 1;
    for (let i = 1; i < 4; i++) {
      const y = (h * i) / 4;
      ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(w, y); ctx.stroke();
    }
    if (this.data.length < 2) return;
    // visible window: rows with t in [tEnd − span, tEnd] — transient focus shows
    // the S-curve speed rise and inrush decay; continuous shows the steady state
    const tEnd = this.data[this.data.length - 1].t;
    const t0 = Math.max(this.data[0].t, tEnd - this.span);
    const vis = [];
    for (const row of this.data) if (row.t >= t0) vis.push(row);
    if (vis.length < 2) return;
    // autoscale over visible series values
    let max = 1e-9;
    for (const s of this.series) {
      for (const row of vis) max = Math.max(max, Math.abs(s.get(row)));
    }
    max *= 1.15;
    // traces
    const tLo = vis[0].t, tHi = vis[vis.length - 1].t;
    const tSpan = Math.max(tHi - tLo, 1e-6);
    for (const s of this.series) {
      ctx.strokeStyle = s.color;
      ctx.lineWidth = 1.6;
      ctx.beginPath();
      for (let i = 0; i < vis.length; i++) {
        const x = ((vis[i].t - tLo) / tSpan) * w;
        const y = h - (Math.abs(s.get(vis[i])) / max) * (h - 6) - 3;
        if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
      }
      ctx.stroke();
    }
    // legend
    ctx.font = '10px "JetBrains Mono", ui-monospace, monospace';
    let lx = 6;
    for (const s of this.series) {
      ctx.fillStyle = s.color;
      ctx.fillText(s.label, lx, 11);
      lx += ctx.measureText(s.label).width + 14;
    }
    // max label
    ctx.fillStyle = 'rgba(26,35,48,0.45)';
    ctx.fillText(max.toFixed(0), w - 34, h - 5);
  }
}
