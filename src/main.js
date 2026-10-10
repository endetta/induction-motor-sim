/**
 * main.js — entry point: scene, lighting, camera, render loop.
 * Wires the physics model (InductionMotorModel) to the 3D build (buildMotor)
 * and the vector layers (VectorFieldVisualizer), with the HTML control panel.
 */

import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { RoomEnvironment } from 'three/examples/jsm/environments/RoomEnvironment.js';
import { buildMotor } from './components/MotorGeometry.js';
import { VectorFieldVisualizer } from './components/VectorFieldVisualizer.js';
import { InductionMotorModel } from './physics/InductionMotorModel.js';
import { ControlPanel } from './ui/ControlPanel.js';

// ---- renderer / scene -------------------------------------------------------
const viewport = document.getElementById('viewport');
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
// cutaway clipping needs local clipping on every material
renderer.localClippingEnabled = true;
viewport.appendChild(renderer.domElement);

const scene = new THREE.Scene();
scene.background = new THREE.Color(0xe8ecf2);
scene.fog = new THREE.Fog(0xe8ecf2, 1.6, 4.5);

// ---- lighting ---------------------------------------------------------------
// Environment map: metalness parts (copper bars, shaft, stator core) only look
// metallic when something exists for them to reflect — a plain light setup
// renders them as flat black. RoomEnvironment is a cheap procedural room.
const pmrem = new THREE.PMREMGenerator(renderer);
scene.environment = pmrem.fromScene(new RoomEnvironment(renderer), 0.04).texture;
pmrem.dispose();

scene.add(new THREE.HemisphereLight(0xffffff, 0x9aa4b0, 0.9));
const key = new THREE.DirectionalLight(0xffffff, 2.1);
key.position.set(0.7, 1.2, 0.9);
key.castShadow = true;
key.shadow.mapSize.set(2048, 2048);
key.shadow.camera.left = key.shadow.camera.bottom = -0.5;
key.shadow.camera.right = key.shadow.camera.top = 0.5;
scene.add(key);
const rim = new THREE.DirectionalLight(0x9cc8ff, 0.7);
rim.position.set(-0.9, 0.4, -0.8);
scene.add(rim);

// ground plane for shadow grounding
const ground = new THREE.Mesh(
  new THREE.CircleGeometry(2.2, 48).rotateX(-Math.PI / 2),
  new THREE.ShadowMaterial({ opacity: 0.18 }),
);
ground.position.y = -0.30;
ground.receiveShadow = true;
scene.add(ground);

// ---- camera + controls ------------------------------------------------------
const camera = new THREE.PerspectiveCamera(42, 1, 0.02, 30);
camera.position.set(0.42, 0.26, 0.52);
const controls = new OrbitControls(camera, renderer.domElement);
controls.target.set(0, 0, 0);
controls.enableDamping = true;
controls.dampingFactor = 0.08;
controls.minDistance = 0.18;
controls.maxDistance = 2.5;

// ---- build the machine ------------------------------------------------------
const motor3d = buildMotor('cage');
motor3d.root.traverse((o) => { if (o.isMesh) { o.castShadow = true; o.receiveShadow = true; } });
scene.add(motor3d.root);

const viz = new VectorFieldVisualizer(scene, motor3d);

// ---- physics + UI -----------------------------------------------------------
const model = new InductionMotorModel();

const hooks = {
  onCutaway: (mode) => motor3d.setCutaway(mode),
  onExplode: (f) => motor3d.setExplode(f),
  onJumper: (mode) => motor3d.setJumper(mode),
  onLidOpen: (open) => motor3d.setLidOpen(open),
  onFocusTermBox: () => {
    // Fokus ke Terminal Box: aim the orbit target at the box and move close
    const t = motor3d.focusTermBox;
    controls.target.set(t.x, t.y, t.z);
    camera.position.set(0.22, t.y + 0.10, t.z + 0.28);
  },
  onVisibility: (name, on) => {
    switch (name) {
      case 'casing':  motor3d.housing.visible = on; break;
      case 'fins':    motor3d.housing.children[1].visible = on; break;   // fins mesh
      case 'stator':  motor3d.stator.visible = on; break;
      case 'rmf':     viz.toggle('rmf', on); break;
      case 'barCurrent': viz.toggle('barCurrent', on); break;
      case 'force':   viz.toggle('force', on); break;
      case 'phaseField': viz.toggle('phaseField', on); break;
      case 'netField': viz.netArrow.visible = on; break;
      case 'wire':    motor3d.cables.visible = on; break;
    }
  },
};
const panel = new ControlPanel(model, hooks);
// apply the default cutaway (90°) chosen in the HTML
motor3d.setCutaway(document.getElementById('cutaway').value);

// ---- UI event isolation ------------------------------------------------------
// Panel interactions (sliders, wheel, keys) must never reach the canvas /
// OrbitControls — stop propagation at the aside container.
const asideEl = document.querySelector('aside');
['pointerdown', 'pointerup', 'wheel', 'keydown'].forEach((evt) => {
  asideEl.addEventListener(evt, (e) => e.stopPropagation());
});

// ---- resize ------------------------------------------------------------------
function resize() {
  const w = viewport.clientWidth, h = viewport.clientHeight;
  // updateStyle must stay ON: with it off the canvas has no CSS size, so it
  // renders at its intrinsic (attribute) size and feeds that back through the
  // ResizeObserver → canvas grows every frame (explodes on HiDPI displays).
  renderer.setSize(w, h);
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
}
window.addEventListener('resize', resize);
new ResizeObserver(resize).observe(viewport);
resize();

// ---- main loop ----------------------------------------------------------------
const CLOCK_MAX = 0.05;          // clamp tab-switch stalls
const clock = new THREE.Clock();
let sampleAccum = 0;

function frame() {
  requestAnimationFrame(frame);
  const dtWall = Math.min(clock.getDelta(), CLOCK_MAX);

  // physics: wall time × animation speed → simulated time.
  // Slow-Mo edukasi: 0.1× time scale so the J=1.2 kg·m² run-up spans 3–5 s visual.
  // Step mode: physics held until the "Next" gate releases it.
  const scale = panel.slowMo ? 0.1 : panel.animSpeed;
  if (panel.physicsGate()) {
    if (model.state !== 'idle' || model.omega_r > 0) model.step(dtWall * scale);
    else model.step(dtWall * scale);     // keep electrical clock running too
  }

  // rotor orientation (mechanical angle from the model)
  motor3d.rotor.rotation.z = -model.theta_r;     // −Z: RH rotation matches +θ math
  // wound-rotor slip rings spin with the rotor automatically (children of rotor)

  // vector layers + emissive windings (slow-mo also scales the field animation)
  viz.update(model, panel.slowMo ? 0.1 : panel.animSpeed, dtWall);

  // RMF loops live in the front cross-section plane (z set inside the viz)
  // — no per-frame repositioning needed anymore.

  // telemetry: 30 Hz sampling into plots, HUD text every frame
  sampleAccum += dtWall;
  const tel = model.telemetry();
  if (sampleAccum > 1 / 30) {
    panel.sample(tel);
    sampleAccum = 0;
  }
  panel.update(tel);

  controls.update();
  renderer.render(scene, camera);
}
frame();
