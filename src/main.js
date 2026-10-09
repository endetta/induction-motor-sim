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

// ---- supply cables (three colored leads into the terminal box) --------------
const cables = new THREE.Group();
for (const [i, color] of [[0, 0xd94848], [1, 0xb8860b], [2, 0x2b62b8]]) {
  const curve = new THREE.CatmullRomCurve3([
    new THREE.Vector3(-0.02 + i * 0.02, 0.30, 0.16),
    new THREE.Vector3(-0.10 + i * 0.06, 0.24, 0.20),
    new THREE.Vector3(-0.02 + i * 0.02, 0.16, 0.10),
    new THREE.Vector3(0.008 + i * 0.018, motor3d.mats ? 0.17 : 0.17, 0.056),
  ]);
  const tube = new THREE.Mesh(
    new THREE.TubeGeometry(curve, 24, 0.004, 6),
    new THREE.MeshStandardMaterial({ color, roughness: 0.6, metalness: 0.1 }),
  );
  cables.add(tube);
}
scene.add(cables);

// ---- physics + UI -----------------------------------------------------------
const model = new InductionMotorModel();

const hooks = {
  onCutaway: (mode) => motor3d.setCutaway(mode),
  onExplode: (f) => motor3d.setExplode(f),
  onJumper: (mode) => motor3d.setJumper(mode),
  onVisibility: (name, on) => {
    switch (name) {
      case 'casing':  motor3d.housing.visible = on; break;
      case 'fins':    motor3d.housing.children[1].visible = on; break;   // fins mesh
      case 'stator':  motor3d.stator.visible = on; break;
      case 'rmf':     viz.toggle('rmf', on); break;
      case 'barCurrent': viz.toggle('barCurrent', on); break;
      case 'force':   viz.toggle('force', on); break;
      case 'speedRing': viz.toggle('speedRing', on); break;
      case 'wire':    cables.visible = on; break;
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

  // physics: wall time × animation speed → simulated time
  if (model.state !== 'idle' || model.omega_r > 0) model.step(dtWall * panel.animSpeed);
  else model.step(dtWall * panel.animSpeed);     // keep electrical clock running too

  // rotor orientation (mechanical angle from the model)
  motor3d.rotor.rotation.z = -model.theta_r;     // −Z: RH rotation matches +θ math
  // wound-rotor slip rings spin with the rotor automatically (children of rotor)

  // vector layers + emissive windings
  viz.update(model, panel.animSpeed, dtWall);

  // RMF loops sit centred in the air gap plane, facing camera-neutral XY
  viz.rmf.position.z = 0.0;

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
