import * as THREE from 'three';
import { OrbitControls } from './vendor/OrbitControls.js';

const COLORS = {
  observed: 0xff8a3d,
  state: 0xffd34d,
  prediction: 0x3dd7e7,
  velocity: 0x7ce05a,
  spin: 0xe36bda,
  ballistic: 0xff525d,
  neutral: 0xd8dcdb,
};

const dom = {
  scene: document.querySelector('#scene'),
  datasetName: document.querySelector('#dataset-name'),
  loading: document.querySelector('#loading'),
  loadingText: document.querySelector('#loading-text'),
  reloadButton: document.querySelector('#reload-button'),
  openButton: document.querySelector('#open-button'),
  fileInput: document.querySelector('#file-input'),
  fitButton: document.querySelector('#fit-button'),
  panelButton: document.querySelector('#panel-button'),
  inspector: document.querySelector('#inspector'),
  previousButton: document.querySelector('#previous-button'),
  playButton: document.querySelector('#play-button'),
  nextButton: document.querySelector('#next-button'),
  timeline: document.querySelector('#timeline'),
  speedSelect: document.querySelector('#speed-select'),
  loopCheckbox: document.querySelector('#loop-checkbox'),
  currentTime: document.querySelector('#current-time'),
  duration: document.querySelector('#duration'),
  trackState: document.querySelector('#track-state'),
  values: {
    frame: document.querySelector('#value-frame'),
    time: document.querySelector('#value-time'),
    target: document.querySelector('#value-target'),
    detections: document.querySelector('#value-detections'),
    latency: document.querySelector('#value-latency'),
    position: document.querySelector('#value-position'),
    velocity: document.querySelector('#value-velocity'),
    acceleration: document.querySelector('#value-acceleration'),
    speed: document.querySelector('#value-speed'),
    yaw: document.querySelector('#value-yaw'),
    radii: document.querySelector('#value-radii'),
    dza: document.querySelector('#value-dza'),
    horizon: document.querySelector('#value-horizon'),
  },
};

const renderer = new THREE.WebGLRenderer({ antialias: true, powerPreference: 'high-performance' });
renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
renderer.setSize(dom.scene.clientWidth, dom.scene.clientHeight);
renderer.outputColorSpace = THREE.SRGBColorSpace;
renderer.toneMapping = THREE.ACESFilmicToneMapping;
renderer.toneMappingExposure = 1.05;
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
dom.scene.appendChild(renderer.domElement);

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0b0d0f);
scene.fog = new THREE.Fog(0x0b0d0f, 7.5, 16);

const camera = new THREE.PerspectiveCamera(48, 1, 0.015, 80);
camera.up.set(0, 0, 1);
camera.position.set(-2.2, -3.2, 2.0);

const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.dampingFactor = 0.08;
controls.target.set(0.8, 0, -0.12);
controls.minDistance = 0.35;
controls.maxDistance = 25;
controls.zoomToCursor = true;
controls.update();

scene.add(new THREE.HemisphereLight(0xf5f1e9, 0x22282a, 1.7));
const keyLight = new THREE.DirectionalLight(0xffffff, 2.5);
keyLight.position.set(-2.5, -3.5, 5.0);
keyLight.castShadow = true;
keyLight.shadow.mapSize.set(2048, 2048);
keyLight.shadow.camera.near = 0.1;
keyLight.shadow.camera.far = 12;
keyLight.shadow.camera.left = -4;
keyLight.shadow.camera.right = 4;
keyLight.shadow.camera.top = 4;
keyLight.shadow.camera.bottom = -4;
scene.add(keyLight);
const rimLight = new THREE.DirectionalLight(0xff9d62, 0.9);
rimLight.position.set(3.5, 2.0, 2.5);
scene.add(rimLight);

const visualRoot = new THREE.Group();
scene.add(visualRoot);
const layers = {
  grid: new THREE.Group(),
  self: new THREE.Group(),
  observed: new THREE.Group(),
  state: new THREE.Group(),
  prediction: new THREE.Group(),
  velocity: new THREE.Group(),
  spin: new THREE.Group(),
  ballistic: new THREE.Group(),
};
for (const group of Object.values(layers)) visualRoot.add(group);

const materials = {
  observed: new THREE.MeshStandardMaterial({
    color: COLORS.observed, emissive: 0x5a2108, emissiveIntensity: 0.75,
    metalness: 0.18, roughness: 0.32,
  }),
  state: new THREE.MeshStandardMaterial({
    color: COLORS.state, emissive: 0x4e3b00, emissiveIntensity: 0.55,
    transparent: true, opacity: 0.72, metalness: 0.12, roughness: 0.45,
  }),
  prediction: new THREE.MeshStandardMaterial({
    color: COLORS.prediction, emissive: 0x063e45, emissiveIntensity: 0.85,
    transparent: true, opacity: 0.64, metalness: 0.2, roughness: 0.28,
  }),
  stateWire: new THREE.LineBasicMaterial({ color: COLORS.state, transparent: true, opacity: 0.92 }),
  predictionWire: new THREE.LineBasicMaterial({ color: COLORS.prediction, transparent: true, opacity: 0.92 }),
  ballistic: new THREE.MeshBasicMaterial({ color: COLORS.ballistic }),
};

const geometry = {
  smallArmor: new THREE.BoxGeometry(0.012, 0.135, 0.050),
  largeArmor: new THREE.BoxGeometry(0.012, 0.225, 0.050),
  trackerArmor: new THREE.BoxGeometry(0.010, 0.135, 0.055),
  chassis: new THREE.BoxGeometry(0.58, 0.48, 0.32),
  chassisEdges: new THREE.EdgesGeometry(new THREE.BoxGeometry(0.58, 0.48, 0.32)),
  impact: new THREE.SphereGeometry(0.027, 16, 10),
};

function makeGrid(size = 12, step = 0.25, groundZ = -0.38) {
  const minor = [];
  const major = [];
  const half = size / 2;
  const count = Math.round(size / step);
  for (let i = 0; i <= count; i += 1) {
    const coordinate = -half + i * step;
    const target = i % Math.round(1 / step) === 0 ? major : minor;
    target.push(-half, coordinate, groundZ, half, coordinate, groundZ);
    target.push(coordinate, -half, groundZ, coordinate, half, groundZ);
  }
  const minorGeometry = new THREE.BufferGeometry();
  minorGeometry.setAttribute('position', new THREE.Float32BufferAttribute(minor, 3));
  layers.grid.add(new THREE.LineSegments(
    minorGeometry,
    new THREE.LineBasicMaterial({ color: 0x2d3133, transparent: true, opacity: 0.5 }),
  ));
  const majorGeometry = new THREE.BufferGeometry();
  majorGeometry.setAttribute('position', new THREE.Float32BufferAttribute(major, 3));
  layers.grid.add(new THREE.LineSegments(
    majorGeometry,
    new THREE.LineBasicMaterial({ color: 0x555b5e, transparent: true, opacity: 0.62 }),
  ));
  const ground = new THREE.Mesh(
    new THREE.PlaneGeometry(size, size),
    new THREE.MeshStandardMaterial({ color: 0x121517, roughness: 0.92, metalness: 0.02 }),
  );
  ground.position.z = groundZ - 0.008;
  ground.receiveShadow = true;
  layers.grid.add(ground);
  const axes = new THREE.AxesHelper(0.65);
  axes.position.z = groundZ + 0.012;
  layers.grid.add(axes);
}

function makeLabel(text, color = '#f4f5f2', width = 220) {
  const canvas = document.createElement('canvas');
  canvas.width = width;
  canvas.height = 56;
  const context = canvas.getContext('2d');
  context.fillStyle = 'rgba(10, 12, 13, 0.86)';
  context.fillRect(0, 0, canvas.width, canvas.height);
  context.strokeStyle = 'rgba(170, 178, 179, 0.72)';
  context.strokeRect(1, 1, canvas.width - 2, canvas.height - 2);
  context.font = '600 22px Consolas, monospace';
  context.textAlign = 'center';
  context.textBaseline = 'middle';
  context.fillStyle = color;
  context.fillText(text, canvas.width / 2, canvas.height / 2 + 1);
  const texture = new THREE.CanvasTexture(canvas);
  texture.colorSpace = THREE.SRGBColorSpace;
  const material = new THREE.SpriteMaterial({ map: texture, depthTest: false, transparent: true });
  const sprite = new THREE.Sprite(material);
  sprite.scale.set(width / 420, 0.13, 1);
  sprite.renderOrder = 20;
  return sprite;
}

function makeSelfVehicle() {
  const bodyMaterial = new THREE.MeshStandardMaterial({
    color: 0x9ba2a2, metalness: 0.72, roughness: 0.38,
  });
  const darkMaterial = new THREE.MeshStandardMaterial({
    color: 0x25292b, metalness: 0.55, roughness: 0.5,
  });
  const cameraMaterial = new THREE.MeshStandardMaterial({
    color: 0x4bb4bf, emissive: 0x0a3b40, emissiveIntensity: 0.45,
    metalness: 0.35, roughness: 0.32,
  });
  const base = new THREE.Mesh(new THREE.BoxGeometry(0.56, 0.46, 0.16), bodyMaterial);
  base.position.set(-0.24, 0, -0.50);
  base.castShadow = true;
  base.receiveShadow = true;
  layers.self.add(base);

  const wheelGeometry = new THREE.CylinderGeometry(0.105, 0.105, 0.07, 24);
  for (const x of [-0.42, -0.07]) {
    for (const y of [-0.26, 0.26]) {
      const wheel = new THREE.Mesh(wheelGeometry, darkMaterial);
      wheel.position.set(x, y, -0.54);
      wheel.castShadow = true;
      layers.self.add(wheel);
    }
  }

  const mast = new THREE.Mesh(new THREE.CylinderGeometry(0.08, 0.10, 0.18, 24), darkMaterial);
  mast.rotation.x = Math.PI / 2;
  mast.position.set(-0.18, 0, -0.34);
  layers.self.add(mast);

  const yawGroup = new THREE.Group();
  yawGroup.name = 'gimbalYaw';
  yawGroup.position.set(-0.18, 0, -0.27);
  const turret = new THREE.Mesh(new THREE.BoxGeometry(0.26, 0.22, 0.10), bodyMaterial);
  turret.castShadow = true;
  yawGroup.add(turret);
  const pitchGroup = new THREE.Group();
  pitchGroup.name = 'gimbalPitch';
  const barrel = new THREE.Mesh(new THREE.BoxGeometry(0.42, 0.035, 0.035), darkMaterial);
  barrel.position.x = 0.26;
  pitchGroup.add(barrel);
  yawGroup.add(pitchGroup);
  layers.self.add(yawGroup);

  const gimbalLabel = makeLabel('gimbal_link', '#f1f3f2');
  gimbalLabel.position.set(-0.18, 0, -0.08);
  layers.self.add(gimbalLabel);

  const cameraBody = new THREE.Mesh(new THREE.BoxGeometry(0.12, 0.09, 0.075), cameraMaterial);
  cameraBody.position.set(-0.055, 0, 0);
  cameraBody.castShadow = true;
  layers.self.add(cameraBody);
  const lens = new THREE.Mesh(new THREE.CylinderGeometry(0.025, 0.031, 0.055, 24), darkMaterial);
  lens.rotation.z = -Math.PI / 2;
  lens.position.set(0.035, 0, 0);
  layers.self.add(lens);
  const cameraLabel = makeLabel('camera_link', '#67d9e3');
  cameraLabel.position.set(0, 0, 0.18);
  layers.self.add(cameraLabel);
  return { yawGroup, pitchGroup };
}

function cameraFrustum(frame) {
  const cameraData = frame?.camera;
  if (!cameraData || !cameraData.k || !cameraData.orientation_wxyz) return;
  const [w, x, y, z] = cameraData.orientation_wxyz;
  const quaternion = new THREE.Quaternion(x, y, z, w).normalize();
  const origin = vector(cameraData.position || [0, 0, 0]);
  const depth = 0.55;
  const [fx, , cx, , fy, cy] = cameraData.k;
  const imageWidth = cameraData.width;
  const imageHeight = cameraData.height;
  const opticalCorners = [
    [0, 0], [imageWidth, 0], [imageWidth, imageHeight], [0, imageHeight],
  ].map(([u, v]) => new THREE.Vector3(
    (u - cx) / fx * depth,
    (v - cy) / fy * depth,
    depth,
  ).applyQuaternion(quaternion).add(origin));
  const points = [];
  for (const corner of opticalCorners) points.push(origin, corner);
  for (let i = 0; i < 4; i += 1) {
    points.push(opticalCorners[i], opticalCorners[(i + 1) % 4]);
  }
  const positions = [];
  for (const point of points) positions.push(point.x, point.y, point.z);
  const frustumGeometry = new THREE.BufferGeometry();
  frustumGeometry.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
  const frustum = new THREE.LineSegments(
    frustumGeometry,
    new THREE.LineBasicMaterial({ color: 0x5fc8d2, transparent: true, opacity: 0.56 }),
  );
  frustum.userData.cameraFrustum = true;
  layers.self.add(frustum);
}

makeGrid();
const selfVehicle = makeSelfVehicle();

const velocityArrow = new THREE.ArrowHelper(
  new THREE.Vector3(1, 0, 0), new THREE.Vector3(), 0.4, COLORS.velocity, 0.12, 0.07,
);
layers.velocity.add(velocityArrow);
velocityArrow.visible = false;
const spinArrow = new THREE.ArrowHelper(
  new THREE.Vector3(0, 0, 1), new THREE.Vector3(), 0.4, COLORS.spin, 0.12, 0.07,
);
layers.spin.add(spinArrow);
spinArrow.visible = false;

let ballisticLine = null;
let impactMarker = null;
let frames = [];
let frameIndex = 0;
let playing = false;
let playbackSpeed = 1;
let playbackTime = 0;
let lastRenderedFrame = -1;
let initialFitDone = false;
let frustumInitialized = false;

function vector(value) {
  if (!Array.isArray(value) || value.length < 3) return new THREE.Vector3();
  return new THREE.Vector3(
    Number(value[0]) || 0,
    Number(value[1]) || 0,
    Number(value[2]) || 0,
  );
}

function quaternion(value) {
  if (!Array.isArray(value) || value.length < 4) return new THREE.Quaternion();
  const component = (index, fallback) => {
    const parsed = Number(value[index]);
    return Number.isFinite(parsed) ? parsed : fallback;
  };
  return new THREE.Quaternion(
    component(1, 0),
    component(2, 0),
    component(3, 0),
    component(0, 1),
  ).normalize();
}

function clearGroup(group) {
  while (group.children.length) {
    const child = group.children[group.children.length - 1];
    child.traverse((object) => {
      if (object.userData.disposeOnClear && object.geometry) object.geometry.dispose();
      if (object.userData.disposeOnClear && object.material) object.material.dispose();
    });
    group.remove(child);
  }
}

function plateMesh(type, material) {
  return new THREE.Mesh(type === 'large' ? geometry.largeArmor : geometry.smallArmor, material);
}

function armorQuaternion(normalValue, widthValue, heightValue) {
  const normal = vector(normalValue).normalize();
  let widthAxis = vector(widthValue).normalize();
  let heightAxis = vector(heightValue);
  heightAxis.addScaledVector(widthAxis, -heightAxis.dot(widthAxis)).normalize();
  if (normal.lengthSq() < 0.5 || widthAxis.lengthSq() < 0.5 || heightAxis.lengthSq() < 0.5) {
    return new THREE.Quaternion();
  }
  widthAxis = heightAxis.clone().cross(normal).normalize();
  const basis = new THREE.Matrix4().makeBasis(normal, widthAxis, heightAxis);
  return new THREE.Quaternion().setFromRotationMatrix(basis);
}

function radialArmorQuaternion(position, center) {
  const normal = position.clone().sub(center);
  normal.z = 0;
  if (normal.lengthSq() < 1e-8) normal.set(1, 0, 0);
  normal.normalize();
  const up = new THREE.Vector3(0, 0, 1);
  const widthAxis = up.clone().cross(normal).normalize();
  return new THREE.Quaternion().setFromRotationMatrix(
    new THREE.Matrix4().makeBasis(normal, widthAxis, up),
  );
}

function addObservedArmors(frame) {
  clearGroup(layers.observed);
  for (const detection of frame.detections || []) {
    if (!detection.pose_valid || !detection.position_control) continue;
    const plate = plateMesh(detection.type, materials.observed);
    plate.position.copy(vector(detection.position_control));
    plate.quaternion.copy(quaternion(detection.orientation_control_wxyz));
    plate.castShadow = true;
    layers.observed.add(plate);

    const normalGeometry = new THREE.BufferGeometry().setFromPoints([
      plate.position,
      new THREE.Vector3(0.12, 0, 0).applyQuaternion(plate.quaternion).add(plate.position),
    ]);
    const normal = new THREE.Line(normalGeometry, new THREE.LineBasicMaterial({ color: COLORS.observed }));
    normal.userData.disposeOnClear = true;
    layers.observed.add(normal);
  }
}

function targetGroup(center, orientation, material, wireMaterial) {
  const group = new THREE.Group();
  group.position.copy(center);
  group.quaternion.copy(orientation);
  const body = new THREE.Mesh(geometry.chassis, material);
  body.castShadow = true;
  body.receiveShadow = true;
  group.add(body);
  group.add(new THREE.LineSegments(geometry.chassisEdges, wireMaterial));
  return group;
}

function stateOrientation(state, predictedYaw = null) {
  if (predictedYaw !== null && Number.isFinite(predictedYaw)) {
    const base = quaternion(state.orientation_wxyz);
    const euler = new THREE.Euler().setFromQuaternion(base, 'ZYX');
    euler.z = predictedYaw;
    return new THREE.Quaternion().setFromEuler(euler);
  }
  if (state.orientation_wxyz) return quaternion(state.orientation_wxyz);
  return new THREE.Quaternion().setFromAxisAngle(
    new THREE.Vector3(0, 0, 1), Number(state.yaw) || 0,
  );
}

function addState(frame) {
  clearGroup(layers.state);
  const state = frame.state || {};
  if (!state.valid || !state.position) return;
  const center = vector(state.position);
  layers.state.add(targetGroup(center, stateOrientation(state), materials.state, materials.stateWire));
  for (const armor of frame.tracked_armors || []) {
    const plate = new THREE.Mesh(geometry.trackerArmor, materials.state);
    plate.position.copy(vector(armor.position));
    plate.quaternion.copy(armorQuaternion(
      armor.normal, armor.width_axis, armor.height_axis,
    ));
    layers.state.add(plate);
  }
}

function addPrediction(frame) {
  clearGroup(layers.prediction);
  const state = frame.state || {};
  const prediction = frame.prediction || {};
  if (!prediction.valid || !prediction.center) return;
  const center = vector(prediction.center);
  const predictedYaw = (Number(state.yaw) || 0) +
    (Number(state.yaw_rate) || 0) * (Number(prediction.time_s) || 0);
  layers.prediction.add(targetGroup(
    center, stateOrientation(state, predictedYaw), materials.prediction, materials.predictionWire,
  ));
  for (const armorValue of prediction.armors || []) {
    const position = vector(armorValue);
    const plate = new THREE.Mesh(geometry.trackerArmor, materials.prediction);
    plate.position.copy(position);
    plate.quaternion.copy(radialArmorQuaternion(position, center));
    layers.prediction.add(plate);
  }
}

function updateArrows(frame) {
  const state = frame.state || {};
  if (!state.valid || !state.position) {
    velocityArrow.visible = false;
    spinArrow.visible = false;
    return;
  }
  const center = vector(state.position);
  const velocity = vector(state.velocity);
  const speed = velocity.length();
  velocityArrow.visible = speed > 0.01;
  if (velocityArrow.visible) {
    velocityArrow.position.copy(center).add(new THREE.Vector3(0, 0, 0.22));
    velocityArrow.setDirection(velocity.normalize());
    const length = Math.max(0.08, speed * 0.48);
    velocityArrow.setLength(length, Math.min(0.14, length * 0.32), Math.min(0.085, length * 0.20));
  }

  const yawRate = Number(state.yaw_rate) || 0;
  spinArrow.visible = Math.abs(yawRate) > 0.03;
  if (spinArrow.visible) {
    const length = Math.max(0.10, Math.abs(yawRate) * 0.08);
    const direction = yawRate >= 0 ? 1 : -1;
    spinArrow.position.copy(center).add(new THREE.Vector3(
      0, 0, direction > 0 ? 0.22 : 0.22 + length,
    ));
    spinArrow.setDirection(new THREE.Vector3(0, 0, direction));
    spinArrow.setLength(length, Math.min(0.15, length * 0.28), Math.min(0.09, length * 0.18));
  }
}

function clearBallistic() {
  if (ballisticLine) {
    layers.ballistic.remove(ballisticLine);
    ballisticLine.geometry.dispose();
    ballisticLine.material.dispose();
    ballisticLine = null;
  }
  if (impactMarker) {
    layers.ballistic.remove(impactMarker);
    impactMarker = null;
  }
}

function addBallistic(frame) {
  clearBallistic();
  const command = frame.command || {};
  const speed = Number(command.bullet_speed_mps) || 0;
  if (speed <= 0 || Number(command.mode) < 0) return;
  const yaw = (Number(command.yaw_deg) || 0) * Math.PI / 180;
  const pitch = (Number(command.pitch_deg) || 0) * Math.PI / 180;
  const distance = Math.max(0.5, Number(command.distance_m) || 4.0);
  const horizontalSpeed = Math.max(0.1, speed * Math.cos(pitch));
  const maxTime = Math.min(1.25, Math.max(0.18, distance / horizontalSpeed * 1.15));
  const origin = new THREE.Vector3(0.04, 0, -0.015);
  const velocity = new THREE.Vector3(
    speed * Math.cos(pitch) * Math.cos(yaw),
    speed * Math.cos(pitch) * Math.sin(yaw),
    speed * Math.sin(pitch),
  );
  const points = [];
  for (let i = 0; i <= 72; i += 1) {
    const time = maxTime * i / 72;
    points.push(new THREE.Vector3(
      origin.x + velocity.x * time,
      origin.y + velocity.y * time,
      origin.z + velocity.z * time - 0.5 * 9.80665 * time * time,
    ));
  }
  ballisticLine = new THREE.Line(
    new THREE.BufferGeometry().setFromPoints(points),
    new THREE.LineBasicMaterial({ color: COLORS.ballistic, transparent: true, opacity: 0.95 }),
  );
  layers.ballistic.add(ballisticLine);
  impactMarker = new THREE.Mesh(
    geometry.impact,
    materials.ballistic,
  );
  impactMarker.position.copy(points[points.length - 1]);
  layers.ballistic.add(impactMarker);
}

function updateSelf(frame) {
  const command = frame.command || {};
  selfVehicle.yawGroup.rotation.z = (Number(command.yaw_deg) || 0) * Math.PI / 180;
  selfVehicle.pitchGroup.rotation.y = -(Number(command.pitch_deg) || 0) * Math.PI / 180;
  if (!frustumInitialized && frame.camera) {
    cameraFrustum(frame);
    frustumInitialized = true;
  }
}

function trackStateName(value) {
  if (value === 0) return 'DETECTING';
  if (value === 1) return 'TRACKING';
  if (value === 2) return 'TEMP LOST';
  return 'NO TARGET';
}

function fixed(value, digits = 3) {
  const number = Number(value);
  return Number.isFinite(number) ? number.toFixed(digits) : '-';
}

function formatVector(value) {
  if (!Array.isArray(value) || value.length < 3) return '-';
  return `[${fixed(value[0])}, ${fixed(value[1])}, ${fixed(value[2])}]`;
}

function updateInspector(frame) {
  const state = frame.state || {};
  const prediction = frame.prediction || {};
  const timing = frame.timing || {};
  const validPoses = (frame.detections || []).filter((detection) => detection.pose_valid).length;
  dom.values.frame.textContent = `${frame.seq ?? '-'} / src ${frame.source_frame ?? '-'}`;
  dom.values.time.textContent = `${fixed(frame.time_s, 3)} s`;
  dom.values.target.textContent = state.selected_id || '-';
  dom.values.detections.textContent = `${(frame.detections || []).length} / PnP ${validPoses}`;
  dom.values.latency.textContent = `${fixed(timing.processing_ms, 2)} ms`;
  dom.values.position.textContent = formatVector(state.position);
  dom.values.velocity.textContent = formatVector(state.velocity);
  dom.values.acceleration.textContent = formatVector(state.acceleration);
  dom.values.speed.textContent = state.velocity ? `${fixed(vector(state.velocity).length(), 3)} m/s` : '-';
  dom.values.yaw.textContent = state.valid
    ? `${fixed(state.yaw)} / ${fixed(state.yaw_rate)} rad/s` : '-';
  dom.values.radii.textContent = state.valid ? `${fixed(state.r1)} / ${fixed(state.r2)} m` : '-';
  dom.values.dza.textContent = state.valid ? `${fixed(state.dza)} m` : '-';
  dom.values.horizon.textContent = prediction.valid
    ? `${fixed((Number(prediction.time_s) || 0) * 1000, 1)} ms` : '-';
  const stateName = trackStateName(state.track_state);
  dom.trackState.textContent = stateName;
  dom.trackState.className = 'state-badge';
  if (state.track_state === 1) dom.trackState.classList.add('tracking');
  if (state.track_state === 2) dom.trackState.classList.add('temp-lost');
}

function updateFrame(index, force = false) {
  if (!frames.length) return;
  frameIndex = Math.max(0, Math.min(frames.length - 1, index));
  if (!force && frameIndex === lastRenderedFrame) return;
  lastRenderedFrame = frameIndex;
  const frame = frames[frameIndex];
  playbackTime = Number(frame.time_s) || 0;
  addObservedArmors(frame);
  addState(frame);
  addPrediction(frame);
  updateArrows(frame);
  addBallistic(frame);
  updateSelf(frame);
  updateInspector(frame);
  dom.timeline.value = String(frameIndex);
  dom.currentTime.textContent = `${fixed(playbackTime, 2)} s`;
  if (!initialFitDone && frame.state?.valid) {
    fitView();
    initialFitDone = true;
  }
}

function fitView() {
  const bounds = new THREE.Box3();
  const fitObjects = [layers.self, layers.observed, layers.state, layers.prediction];
  for (const object of fitObjects) bounds.expandByObject(object);
  if (bounds.isEmpty()) {
    controls.target.set(0.8, 0, -0.12);
    camera.position.set(-2.2, -3.2, 2.0);
    controls.update();
    return;
  }
  const center = bounds.getCenter(new THREE.Vector3());
  const size = bounds.getSize(new THREE.Vector3());
  const maxDimension = Math.max(size.x, size.y, size.z, 1.2);
  const verticalFov = THREE.MathUtils.degToRad(camera.fov);
  const horizontalFov = 2 * Math.atan(Math.tan(verticalFov / 2) * camera.aspect);
  const limitingFov = Math.min(verticalFov, horizontalFov);
  const distance = maxDimension * 0.72 / Math.tan(limitingFov / 2);
  const direction = new THREE.Vector3(-0.72, -1.0, 0.66).normalize();
  controls.target.copy(center);
  camera.position.copy(center).addScaledVector(direction, Math.max(distance, 2.2));
  camera.near = Math.max(0.01, distance / 120);
  camera.far = Math.max(40, distance * 12);
  camera.updateProjectionMatrix();
  controls.update();
}

function indexAtTime(time) {
  let low = 0;
  let high = frames.length - 1;
  while (low <= high) {
    const mid = Math.floor((low + high) / 2);
    if ((Number(frames[mid].time_s) || 0) <= time) low = mid + 1;
    else high = mid - 1;
  }
  return Math.max(0, Math.min(frames.length - 1, high));
}

function setPlaying(value) {
  playing = Boolean(value) && frames.length > 0;
  dom.playButton.textContent = playing ? 'Ⅱ' : '▶';
  dom.playButton.title = playing ? 'Pause' : 'Play';
  dom.playButton.setAttribute('aria-label', playing ? 'Pause' : 'Play');
}

function parseJsonl(text) {
  const parsed = [];
  let invalid = 0;
  for (const rawLine of text.split(/\r?\n/)) {
    const line = rawLine.trim();
    if (!line) continue;
    try {
      const frame = JSON.parse(line);
      if (frame && Number.isFinite(Number(frame.time_s))) parsed.push(frame);
      else invalid += 1;
    } catch {
      invalid += 1;
    }
  }
  parsed.sort((left, right) => Number(left.time_s) - Number(right.time_s));
  if (!parsed.length) throw new Error('No valid frames in JSONL');
  return { parsed, invalid };
}

function useDataset(text, name) {
  const { parsed, invalid } = parseJsonl(text);
  frames = parsed;
  frameIndex = 0;
  lastRenderedFrame = -1;
  initialFitDone = false;
  frustumInitialized = false;
  for (const child of [...layers.self.children]) {
    if (child.userData.cameraFrustum) layers.self.remove(child);
  }
  dom.timeline.max = String(frames.length - 1);
  dom.timeline.value = '0';
  const endTime = Number(frames[frames.length - 1].time_s) || 0;
  dom.duration.textContent = `${fixed(endTime, 2)} s`;
  dom.datasetName.textContent = `${name} · ${frames.length} frames${invalid ? ` · ${invalid} ignored` : ''}`;
  const firstTracked = frames.findIndex((frame) => frame.state?.valid);
  updateFrame(firstTracked >= 0 ? firstTracked : 0, true);
  setPlaying(false);
  hideLoading();
}

function showLoading(text) {
  dom.loading.classList.remove('hidden', 'error');
  dom.loadingText.textContent = text;
}

function hideLoading() {
  dom.loading.classList.add('hidden');
}

function showError(error) {
  dom.loading.classList.remove('hidden');
  dom.loading.classList.add('error');
  dom.loadingText.textContent = error instanceof Error ? error.message : String(error);
}

async function loadServerDataset() {
  showLoading('LOADING DIAGNOSTICS');
  try {
    const [metaResponse, dataResponse] = await Promise.all([
      fetch('/api/meta', { cache: 'no-store' }),
      fetch('/api/diagnostics', { cache: 'no-store' }),
    ]);
    if (!dataResponse.ok) throw new Error(`Diagnostics request failed (${dataResponse.status})`);
    const meta = metaResponse.ok ? await metaResponse.json() : { name: 'diagnostics.jsonl' };
    useDataset(await dataResponse.text(), meta.name || 'diagnostics.jsonl');
  } catch (error) {
    showError(error);
  }
}

dom.reloadButton.addEventListener('click', loadServerDataset);
dom.openButton.addEventListener('click', () => dom.fileInput.click());
dom.fileInput.addEventListener('change', async () => {
  const file = dom.fileInput.files?.[0];
  if (!file) return;
  showLoading('LOADING LOCAL JSONL');
  try {
    useDataset(await file.text(), file.name);
  } catch (error) {
    showError(error);
  } finally {
    dom.fileInput.value = '';
  }
});
dom.fitButton.addEventListener('click', fitView);
dom.panelButton.addEventListener('click', () => dom.inspector.classList.toggle('open'));
dom.playButton.addEventListener('click', () => setPlaying(!playing));
dom.previousButton.addEventListener('click', () => {
  setPlaying(false);
  updateFrame(frameIndex - 1, true);
});
dom.nextButton.addEventListener('click', () => {
  setPlaying(false);
  updateFrame(frameIndex + 1, true);
});
dom.timeline.addEventListener('input', () => {
  setPlaying(false);
  updateFrame(Number(dom.timeline.value), true);
});
dom.speedSelect.addEventListener('change', () => {
  playbackSpeed = Number(dom.speedSelect.value) || 1;
});
for (const checkbox of document.querySelectorAll('[data-layer]')) {
  checkbox.addEventListener('change', () => {
    const layer = layers[checkbox.dataset.layer];
    if (layer) layer.visible = checkbox.checked;
  });
}

window.addEventListener('keydown', (event) => {
  const target = event.target;
  if (target instanceof HTMLInputElement || target instanceof HTMLSelectElement) return;
  if (event.code === 'Space') {
    event.preventDefault();
    setPlaying(!playing);
  } else if (event.code === 'ArrowLeft') {
    setPlaying(false);
    updateFrame(frameIndex - 1, true);
  } else if (event.code === 'ArrowRight') {
    setPlaying(false);
    updateFrame(frameIndex + 1, true);
  } else if (event.code === 'KeyF') {
    fitView();
  }
});

function resize() {
  const width = dom.scene.clientWidth;
  const height = dom.scene.clientHeight;
  renderer.setSize(width, height, false);
  camera.aspect = Math.max(0.1, width / Math.max(height, 1));
  camera.updateProjectionMatrix();
}
window.addEventListener('resize', resize);
resize();

const clock = new THREE.Clock();
function animate() {
  requestAnimationFrame(animate);
  const delta = Math.min(clock.getDelta(), 0.1);
  if (playing && frames.length) {
    playbackTime += delta * playbackSpeed;
    const endTime = Number(frames[frames.length - 1].time_s) || 0;
    if (playbackTime > endTime) {
      if (dom.loopCheckbox.checked) {
        playbackTime = Number(frames[0].time_s) || 0;
      } else {
        playbackTime = endTime;
        setPlaying(false);
      }
    }
    updateFrame(indexAtTime(playbackTime));
  }
  controls.update();
  renderer.render(scene, camera);
}
animate();
loadServerDataset();
