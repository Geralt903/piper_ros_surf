import * as THREE from 'https://unpkg.com/three@0.165.0/build/three.module.js';
import { STLLoader } from 'https://unpkg.com/three@0.165.0/examples/jsm/loaders/STLLoader.js';

const canvas = document.querySelector('#scene');
const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
renderer.setClearColor(0x111315, 1);

const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(42, 1, 0.1, 100);
camera.position.set(3.2, 3.0, 4.8);
camera.lookAt(0, 0.7, 0);

scene.add(new THREE.HemisphereLight(0xffffff, 0x1b2024, 2.2));
const keyLight = new THREE.DirectionalLight(0xffffff, 1.8);
keyLight.position.set(4, 5, 3);
scene.add(keyLight);

const grid = new THREE.GridHelper(5, 20, 0x3b454d, 0x252b30);
scene.add(grid);

const arm = new THREE.Group();
arm.rotation.x = -Math.PI / 2;
arm.scale.setScalar(3.6);
scene.add(arm);

const robotJoints = [
  { name: 'joint1', parent: 'base_link', child: 'link1', type: 'revolute', xyz: [0, 0, 0.123], rpy: [0, 0, 0], axis: [0, 0, 1] },
  { name: 'joint2', parent: 'link1', child: 'link2', type: 'revolute', xyz: [0, 0, 0], rpy: [1.5708, -0.1359, -3.1416], axis: [0, 0, 1] },
  { name: 'joint3', parent: 'link2', child: 'link3', type: 'revolute', xyz: [0.28503, 0, 0], rpy: [0, 0, -1.7939], axis: [0, 0, 1] },
  { name: 'joint4', parent: 'link3', child: 'link4', type: 'revolute', xyz: [-0.021984, -0.25075, 0], rpy: [1.5708, 0, 0], axis: [0, 0, 1] },
  { name: 'joint5', parent: 'link4', child: 'link5', type: 'revolute', xyz: [0, 0, 0], rpy: [-1.5708, 0, 0], axis: [0, 0, 1] },
  { name: 'joint6', parent: 'link5', child: 'link6', type: 'revolute', xyz: [0.000088259, -0.091, 0], rpy: [1.5708, 0, 0], axis: [0, 0, 1] },
  { name: 'joint6_to_gripper_base', parent: 'link6', child: 'gripper_base', type: 'fixed', xyz: [0, 0, 0], rpy: [0, 0, 0], axis: [0, 0, 0] },
  { name: 'joint7', parent: 'gripper_base', child: 'link7', type: 'prismatic', xyz: [0, 0, 0.1358], rpy: [1.5708, 0, 0], axis: [0, 0, 1] },
  { name: 'joint8', parent: 'gripper_base', child: 'link8', type: 'prismatic', xyz: [0, 0, 0.1358], rpy: [1.5708, 0, -3.1416], axis: [0, 0, -1] },
];

const adjustableJoints = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7', 'joint8'];
const defaultConfig = {
  display: {
    joint_offsets: Object.fromEntries(adjustableJoints.map((name) => [name, 0])),
    joint_directions: Object.fromEntries(adjustableJoints.map((name) => [name, name === 'joint2' ? -1 : 1])),
  },
  ik: {
    base_height: 0.123,
    joint2_pitch_bias: -0.1359,
    joint3_yaw_bias: -1.7939,
    upper_arm_length: 0.28503,
    forearm_x: -0.021984,
    forearm_y: -0.25075,
    wrist_y: -0.091,
    tool_z: 0.1358,
  },
};

let config = structuredClone(defaultConfig);

const frameOrder = ['base_link', 'link1', 'link2', 'link3', 'link4', 'link5', 'link6', 'gripper_base', 'link7', 'link8'];
const markers = new Map();
const linkSegments = new Map();
const linkVisuals = new Map();
const stlLoader = new STLLoader();

const base = new THREE.Mesh(
  new THREE.CylinderGeometry(0.09, 0.12, 0.08, 40),
  new THREE.MeshStandardMaterial({ color: 0x2b3339, roughness: 0.55 })
);
base.rotation.x = Math.PI / 2;
base.position.z = 0.04;
arm.add(base);

for (const frameName of frameOrder) {
  const marker = new THREE.Mesh(
    new THREE.SphereGeometry(frameName === 'base_link' ? 0.025 : 0.018, 24, 16),
    new THREE.MeshStandardMaterial({
      color: frameName.startsWith('link') ? 0xd4d8db : 0xf2b84b,
      roughness: 0.42,
      metalness: 0.1,
    })
  );
  markers.set(frameName, marker);
  arm.add(marker);
}

for (const joint of robotJoints) {
  const segment = new THREE.Mesh(
    new THREE.CapsuleGeometry(0.012, 1, 8, 16),
    new THREE.MeshStandardMaterial({
      color: joint.name === 'joint7' || joint.name === 'joint8' ? 0xf2b84b : 0x42c27a,
      roughness: 0.5,
    })
  );
  linkSegments.set(joint.name, segment);
  arm.add(segment);
}

const meshColors = {
  base_link: 0x9aa7b2,
  link1: 0xb9c4ce,
  link2: 0xb4c1d8,
  link3: 0xaebbd0,
  link4: 0xc1cbd4,
  link5: 0xb9c4ce,
  link6: 0xd8e0e5,
  gripper_base: 0xaab5bf,
  link7: 0xf2b84b,
  link8: 0xf2b84b,
};

for (const frameName of frameOrder) {
  stlLoader.load(
    `/meshes/${frameName}.STL`,
    (geometry) => {
      geometry.computeVertexNormals();
      const mesh = new THREE.Mesh(
        geometry,
        new THREE.MeshStandardMaterial({
          color: meshColors[frameName] ?? 0xb9c4ce,
          roughness: 0.58,
          metalness: 0.08,
        })
      );
      linkVisuals.set(frameName, mesh);
      arm.add(mesh);
    },
    undefined,
    () => {
      // Some deployments may omit meshes; the abstract skeleton still remains usable.
    }
  );
}

const els = {
  rosState: document.querySelector('#rosState'),
  age: document.querySelector('#age'),
  serviceBadge: document.querySelector('#serviceBadge'),
  enableBtn: document.querySelector('#enableBtn'),
  disableBtn: document.querySelector('#disableBtn'),
  ctrlMode: document.querySelector('#ctrlMode'),
  armStatus: document.querySelector('#armStatus'),
  motionStatus: document.querySelector('#motionStatus'),
  errCode: document.querySelector('#errCode'),
  jointList: document.querySelector('#jointList'),
  calibrationList: document.querySelector('#calibrationList'),
  ikList: document.querySelector('#ikList'),
  zeroCurrentBtn: document.querySelector('#zeroCurrentBtn'),
  saveConfigBtn: document.querySelector('#saveConfigBtn'),
  resetConfigBtn: document.querySelector('#resetConfigBtn'),
  message: document.querySelector('#message'),
};

let latestPositions = [0, 0, 0, 0, 0, 0, 0];
let targetPositions = [...latestPositions];
let jointNames = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'gripper'];
let busy = false;

function makeOriginMatrix(joint) {
  const matrix = new THREE.Matrix4();
  const position = new THREE.Vector3(...joint.xyz);
  const rotation = new THREE.Quaternion().setFromEuler(new THREE.Euler(...joint.rpy, 'XYZ'));
  matrix.compose(position, rotation, new THREE.Vector3(1, 1, 1));
  return matrix;
}

function makeMotionMatrix(joint, value) {
  const matrix = new THREE.Matrix4();
  const axis = new THREE.Vector3(...joint.axis).normalize();
  if (joint.type === 'revolute') {
    matrix.makeRotationAxis(axis, value);
  } else if (joint.type === 'prismatic') {
    matrix.makeTranslation(axis.x * value, axis.y * value, axis.z * value);
  } else {
    matrix.identity();
  }
  return matrix;
}

function readJointValue(name) {
  const index = jointNames.indexOf(name);
  if (index >= 0) {
    const direction = config.display.joint_directions[name] ?? 1;
    const offset = config.display.joint_offsets[name] ?? 0;
    return (latestPositions[index] ?? 0) * direction + offset;
  }

  const gripperIndex = jointNames.indexOf('gripper');
  if (gripperIndex >= 0) {
    const gripperValue = Math.max(0, Math.min(0.07, latestPositions[gripperIndex] ?? 0));
    if (name === 'joint7') return gripperValue / 2;
    if (name === 'joint8') return -gripperValue / 2;
  }
  return 0;
}

function setSegmentBetween(segment, start, end) {
  const direction = new THREE.Vector3().subVectors(end, start);
  const length = direction.length();
  segment.visible = length > 0.001;
  if (!segment.visible) return;

  segment.position.copy(start).addScaledVector(direction, 0.5);
  segment.scale.set(1, Math.max(0.001, length), 1);
  segment.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), direction.normalize());
}

function resize() {
  const width = canvas.clientWidth;
  const height = canvas.clientHeight;
  renderer.setSize(width, height, false);
  camera.aspect = width / Math.max(1, height);
  camera.updateProjectionMatrix();
}

function setMessage(text, isError = false) {
  els.message.textContent = text;
  els.message.classList.toggle('error', isError);
}

function drawJoints() {
  els.jointList.innerHTML = '';
  for (let i = 0; i < jointNames.length; i += 1) {
    const value = targetPositions[i] ?? 0;
    const normalized = Math.max(0, Math.min(1, (value + Math.PI) / (Math.PI * 2)));
    const row = document.createElement('div');
    row.className = 'joint-row';
    row.innerHTML = `
      <span>${jointNames[i] ?? `joint${i + 1}`}</span>
      <div class="bar"><i style="width:${(normalized * 100).toFixed(1)}%"></i></div>
      <strong>${value.toFixed(3)}</strong>
    `;
    els.jointList.appendChild(row);
  }
}

function drawCalibration() {
  els.calibrationList.innerHTML = '';
  for (const name of adjustableJoints) {
    const row = document.createElement('div');
    row.className = 'calibration-row';
    row.innerHTML = `
      <label for="offset-${name}">${name}</label>
      <input id="offset-${name}" data-offset="${name}" type="number" step="0.001" value="${(config.display.joint_offsets[name] ?? 0).toFixed(4)}">
      <select data-direction="${name}" aria-label="${name} direction">
        <option value="1"${(config.display.joint_directions[name] ?? 1) > 0 ? ' selected' : ''}>正向</option>
        <option value="-1"${(config.display.joint_directions[name] ?? 1) < 0 ? ' selected' : ''}>反向</option>
      </select>
    `;
    els.calibrationList.appendChild(row);
  }
}

function drawIkParams() {
  els.ikList.innerHTML = '';
  for (const [name, value] of Object.entries(config.ik)) {
    const row = document.createElement('div');
    row.className = 'ik-row';
    row.innerHTML = `
      <label for="ik-${name}">${name}</label>
      <input id="ik-${name}" data-ik="${name}" type="number" step="0.0001" value="${Number(value).toFixed(5)}">
    `;
    els.ikList.appendChild(row);
  }
}

function readConfigFromInputs() {
  const next = structuredClone(config);
  els.calibrationList.querySelectorAll('[data-offset]').forEach((input) => {
    next.display.joint_offsets[input.dataset.offset] = Number(input.value) || 0;
  });
  els.calibrationList.querySelectorAll('[data-direction]').forEach((select) => {
    next.display.joint_directions[select.dataset.direction] = Number(select.value) < 0 ? -1 : 1;
  });
  els.ikList.querySelectorAll('[data-ik]').forEach((input) => {
    next.ik[input.dataset.ik] = Number(input.value) || 0;
  });
  config = next;
}

function setConfig(nextConfig) {
  config = structuredClone(defaultConfig);
  for (const name of adjustableJoints) {
    config.display.joint_offsets[name] = Number(nextConfig?.display?.joint_offsets?.[name] ?? config.display.joint_offsets[name]);
    config.display.joint_directions[name] = Number(nextConfig?.display?.joint_directions?.[name] ?? config.display.joint_directions[name]) < 0 ? -1 : 1;
  }
  for (const name of Object.keys(defaultConfig.ik)) {
    config.ik[name] = Number(nextConfig?.ik?.[name] ?? config.ik[name]);
  }
  drawCalibration();
  drawIkParams();
}

async function loadConfig() {
  try {
    const response = await fetch('/api/config', { cache: 'no-store' });
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    setConfig(await response.json());
  } catch (error) {
    setConfig(defaultConfig);
    setMessage(`参数加载失败: ${error.message}`, true);
  }
}

async function saveConfig() {
  readConfigFromInputs();
  els.saveConfigBtn.disabled = true;
  try {
    const response = await fetch('/api/config', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(config),
    });
    const data = await response.json();
    if (!response.ok || !data.ok) throw new Error(data.error || `HTTP ${response.status}`);
    setConfig(data.config);
    setMessage('参数已保存');
  } catch (error) {
    setMessage(`保存失败: ${error.message}`, true);
  } finally {
    els.saveConfigBtn.disabled = false;
  }
}

function zeroCurrentPose() {
  readConfigFromInputs();
  for (const name of adjustableJoints) {
    const index = jointNames.indexOf(name);
    if (index < 0) continue;
    const direction = config.display.joint_directions[name] ?? 1;
    config.display.joint_offsets[name] = -(targetPositions[index] ?? 0) * direction;
  }
  drawCalibration();
  setMessage('已用当前反馈计算显示 offset，点击保存参数后持久化');
}

function syncPositionBuffer() {
  while (latestPositions.length < targetPositions.length) {
    latestPositions.push(targetPositions[latestPositions.length] ?? 0);
  }
}

function applyArmPose(delta) {
  syncPositionBuffer();
  base.visible = !linkVisuals.has('base_link');
  for (let i = 0; i < latestPositions.length; i += 1) {
    latestPositions[i] += ((targetPositions[i] ?? 0) - latestPositions[i]) * Math.min(1, delta * 8);
  }

  const frames = new Map([['base_link', new THREE.Matrix4().identity()]]);
  for (const joint of robotJoints) {
    const parentFrame = frames.get(joint.parent) ?? new THREE.Matrix4().identity();
    const childFrame = parentFrame.clone()
      .multiply(makeOriginMatrix(joint))
      .multiply(makeMotionMatrix(joint, readJointValue(joint.name)));
    frames.set(joint.child, childFrame);
  }

  for (const [frameName, marker] of markers.entries()) {
    const frame = frames.get(frameName);
    if (!frame) continue;
    marker.position.setFromMatrixPosition(frame);
  }

  for (const [frameName, visual] of linkVisuals.entries()) {
    const frame = frames.get(frameName);
    if (!frame) continue;
    frame.decompose(visual.position, visual.quaternion, visual.scale);
  }

  for (const joint of robotJoints) {
    const segment = linkSegments.get(joint.name);
    const parentFrame = frames.get(joint.parent);
    const childFrame = frames.get(joint.child);
    if (!segment || !parentFrame || !childFrame) continue;
    setSegmentBetween(
      segment,
      new THREE.Vector3().setFromMatrixPosition(parentFrame),
      new THREE.Vector3().setFromMatrixPosition(childFrame),
    );
  }
}

async function refreshState() {
  try {
    const response = await fetch('/api/state', { cache: 'no-store' });
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    const data = await response.json();
    const jointState = data.joint_state ?? {};
    targetPositions = jointState.positions?.length ? jointState.positions : targetPositions;
    jointNames = jointState.names?.length ? jointState.names : jointNames;
    syncPositionBuffer();

    const age = jointState.age;
    const connected = typeof age === 'number' && age < 1.5;
    els.rosState.textContent = connected ? '已连接' : '等待数据';
    els.age.textContent = typeof age === 'number' ? `${age.toFixed(2)}s` : '--';

    els.serviceBadge.textContent = data.enable_service_available ? '使能服务在线' : '使能服务离线';
    els.serviceBadge.classList.toggle('ready', Boolean(data.enable_service_available));

    const status = data.arm_status ?? {};
    els.ctrlMode.textContent = status.ctrl_mode ?? '--';
    els.armStatus.textContent = status.arm_status ?? '--';
    els.motionStatus.textContent = status.motion_status ?? '--';
    els.errCode.textContent = status.err_code ?? '--';
    drawJoints();
  } catch (error) {
    els.rosState.textContent = '后端离线';
    els.serviceBadge.textContent = '连接失败';
    els.serviceBadge.classList.remove('ready');
  }
}

async function setEnable(enable) {
  if (busy) return;
  busy = true;
  els.enableBtn.disabled = true;
  els.disableBtn.disabled = true;
  setMessage(enable ? '正在发送使能命令...' : '正在发送失能命令...');

  try {
    const response = await fetch('/api/enable', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ enable }),
    });
    const data = await response.json();
    if (!response.ok || !data.ok) throw new Error(data.message || `HTTP ${response.status}`);
    setMessage(enable ? '使能命令已发送' : '失能命令已发送');
  } catch (error) {
    setMessage(error.message, true);
  } finally {
    busy = false;
    els.enableBtn.disabled = false;
    els.disableBtn.disabled = false;
  }
}

els.enableBtn.addEventListener('click', () => setEnable(true));
els.disableBtn.addEventListener('click', () => setEnable(false));
els.calibrationList.addEventListener('input', readConfigFromInputs);
els.calibrationList.addEventListener('change', readConfigFromInputs);
els.ikList.addEventListener('input', readConfigFromInputs);
els.saveConfigBtn.addEventListener('click', saveConfig);
els.resetConfigBtn.addEventListener('click', () => {
  setConfig(defaultConfig);
  setMessage('已恢复默认，点击保存参数后持久化');
});
els.zeroCurrentBtn.addEventListener('click', zeroCurrentPose);
window.addEventListener('resize', resize);

let last = performance.now();
function animate(now) {
  const delta = (now - last) / 1000;
  last = now;
  applyArmPose(delta);
  renderer.render(scene, camera);
  requestAnimationFrame(animate);
}

resize();
drawJoints();
setConfig(defaultConfig);
loadConfig();
refreshState();
setInterval(refreshState, 200);
requestAnimationFrame(animate);
