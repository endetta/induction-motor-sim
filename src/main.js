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

// Inner-cavity fill: the key/rim lights both come from outside, so the stator
// bore, the air gap and the rotor cage sat in near-black. A short-range,
// shadow-free point light out in front of the bore mouth lights the cavity
// without lighting anything else.
// NOTE on the numbers: three r155+ uses physically-correct light units
// (renderer.useLegacyLights === false), so a point light's contribution is
// intensity / d^decay — not the legacy (1 − d/cutoff)^decay window. The old
// "1.8" that suited legacy lighting is ~240× too hot here (it would clip the
// bore to white). decay=1 with a small intensity keeps the falloff steep
// enough to stay inside the machine while still reading as a clear fill.
const innerBoreLight = new THREE.PointLight(0xffffff, 0.05, 0.45, 1);
innerBoreLight.position.set(0, 0, 0.08);   // at the front air-gap mouth
innerBoreLight.castShadow = false;
scene.add(innerBoreLight);
// twin on the non-drive end so the rear bore is not a black hole in cutaway
const innerBoreLightRear = new THREE.PointLight(0xffffff, 0.035, 0.45, 1);
innerBoreLightRear.position.set(0, 0, -0.08);
innerBoreLightRear.castShadow = false;
scene.add(innerBoreLightRear);

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

// ---- camera preset transitions ----------------------------------------------
// Preset views hand the camera a destination; the render loop lerps toward it.
// OrbitControls is NOT disabled — damping keeps it in sync because the lerp
// writes camera.position/controls.target directly before controls.update().
let targetCamPos = null;
let targetControlsLookAt = null;
const CAM_LERP = 0.08;
const CAM_EPS = 0.002;

export function transitionCamera(pos, lookAt) {
  targetCamPos = pos.clone();
  targetControlsLookAt = lookAt.clone();
}

const CAM_PRESETS = {
  axial:   { pos: [0, 0, 0.42],              lookAt: [0, 0, 0.08] },
  cutaway: { pos: [0.32, 0.18, 0.12],        lookAt: [0, 0, 0] },
  terminal:{ pos: [0, 0.24, 0.08],           lookAt: [0, 0.14, 0.055] },
};
export function cameraPreset(name) {
  const p = CAM_PRESETS[name];
  if (!p) return;
  transitionCamera(new THREE.Vector3(...p.pos), new THREE.Vector3(...p.lookAt));
}

// ---- physics + UI -----------------------------------------------------------
const model = new InductionMotorModel();

const hooks = {
  onCutaway: (mode) => motor3d.setCutaway(mode),
  onExplode: (f) => motor3d.setExplode(f),
  onJumper: (mode) => motor3d.setJumper(mode),
  onLidOpen: (open) => motor3d.setTerminalLid(open),
  onTerminalLid: (open) => motor3d.setTerminalLid(open),
  onFrontShield: (v) => motor3d.setFrontShieldVisible(v),
  onShields: (v) => motor3d.setShieldsVisible(v),
  onGhostOpacity: (v) => motor3d.setGhostOpacity(v),
  onCameraPreset: (name) => cameraPreset(name),
  onFocusTermBox: () => {
    // Fokus ke Terminal Box: gliding preset instead of a hard camera teleport
    const t = motor3d.focusTermBox;
    transitionCamera(
      new THREE.Vector3(0.22, t.y + 0.10, t.z + 0.28),
      new THREE.Vector3(t.x, t.y, t.z),
    );
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

  // camera preset transition: exponential ease toward the armed target. The
  // epsilon snap clears the target so OrbitControls regains full authority
  // (a lingering target would fight every manual drag, one lerp step per frame).
  if (targetCamPos && targetControlsLookAt) {
    camera.position.lerp(targetCamPos, CAM_LERP);
    controls.target.lerp(targetControlsLookAt, CAM_LERP);
    if (camera.position.distanceTo(targetCamPos) < CAM_EPS
        && controls.target.distanceTo(targetControlsLookAt) < CAM_EPS) {
      camera.position.copy(targetCamPos);
      controls.target.copy(targetControlsLookAt);
      targetCamPos = null;
      targetControlsLookAt = null;
    }
  }

  controls.update();
  renderer.render(scene, camera);
}
frame();
