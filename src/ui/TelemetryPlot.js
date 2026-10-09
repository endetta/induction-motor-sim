/**
 * TelemetryPlot.js — lightweight canvas strip-chart, no dependencies.
 * Draws ωr (actual) and ωs (synchronous) traces plus stator current.
 */
export class TelemetryPlot {
  /**
   * @param {HTMLCanvasElement} canvas
   * @param {{label:string, color:string, get:(row)=>number, max?:number}[]} series
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
    // autoscale over visible series values
    let max = 1e-9;
    for (const s of this.series) {
      for (const row of this.data) max = Math.max(max, Math.abs(s.get(row)));
    }
    max *= 1.15;
    // traces
    for (const s of this.series) {
      ctx.strokeStyle = s.color;
      ctx.lineWidth = 1.6;
      ctx.beginPath();
      for (let i = 0; i < this.data.length; i++) {
        const x = (i / (this.maxPoints - 1)) * w;
        const y = h - (Math.abs(s.get(this.data[i])) / max) * (h - 6) - 3;
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
