import * as THREE from 'https://unpkg.com/three@0.165.0/build/three.module.js';
import { STLLoader } from 'https://unpkg.com/three@0.165.0/examples/jsm/loaders/STLLoader.js';

const MODEL_VERSION = 4;
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

const axisGroup = new THREE.Group();
axisGroup.position.set(0, 0.02, 0);
scene.add(axisGroup);

function makeAxisLabel(text, color) {
  const canvas = document.createElement('canvas');
  canvas.width = 96;
  canvas.height = 96;
  const ctx = canvas.getContext('2d');
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  ctx.fillStyle = color;
  ctx.font = '700 54px ui-sans-serif, system-ui, sans-serif';
  ctx.textAlign = 'center';
  ctx.textBaseline = 'middle';
  ctx.fillText(text, canvas.width / 2, canvas.height / 2);

  const texture = new THREE.CanvasTexture(canvas);
  const material = new THREE.SpriteMaterial({ map: texture, transparent: true });
  const sprite = new THREE.Sprite(material);
  sprite.scale.set(0.16, 0.16, 0.16);
  return sprite;
}

function addAxis(direction, color, label) {
  const start = new THREE.Vector3(0, 0, 0);
  const end = direction.clone().multiplyScalar(0.55);
  const arrow = new THREE.ArrowHelper(direction.clone().normalize(), start, 0.55, color, 0.08, 0.045);
  axisGroup.add(arrow);

  const text = makeAxisLabel(label, `#${color.toString(16).padStart(6, '0')}`);
  text.position.copy(end.multiplyScalar(1.12));
  axisGroup.add(text);
}

addAxis(new THREE.Vector3(1, 0, 0), 0xef6963, 'X');
addAxis(new THREE.Vector3(0, 1, 0), 0x42c27a, 'Y');
addAxis(new THREE.Vector3(0, 0, 1), 0x5aa7ff, 'Z');

const arm = new THREE.Group();
arm.rotation.x = -Math.PI / 2;
arm.scale.setScalar(3.6);
scene.add(arm);

const adjustableJoints = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7', 'joint8'];
const defaultConfig = {
  display: {
    model_version: MODEL_VERSION,
    joint_offsets: Object.fromEntries(adjustableJoints.map((name) => [name, name === 'joint2' ? -3.1416 : 0])),
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

let robotJoints = [];
let baseLinkName = 'base_link';
const linkVisuals = new Map();
const stlLoader = new STLLoader();

const base = new THREE.Mesh(
  new THREE.CylinderGeometry(0.09, 0.12, 0.08, 40),
  new THREE.MeshStandardMaterial({ color: 0x2b3339, roughness: 0.55 })
);
base.rotation.x = Math.PI / 2;
base.position.z = 0.04;
arm.add(base);

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

function parseVector(value, fallback) {
  if (!value) return [...fallback];
  const values = value.trim().split(/\s+/).map(Number);
  return values.length === 3 && values.every(Number.isFinite) ? values : [...fallback];
}

function parseOrigin(element) {
  const origin = element?.querySelector(':scope > origin');
  return {
    xyz: parseVector(origin?.getAttribute('xyz'), [0, 0, 0]),
    rpy: parseVector(origin?.getAttribute('rpy'), [0, 0, 0]),
  };
}

function packageMeshToUrl(filename) {
  return filename.replace(/^package:\/\/piper_description\/meshes\//, '/meshes/');
}

function addLinkVisual(linkName, meshUrl, origin) {
  stlLoader.load(
    meshUrl,
    (geometry) => {
      geometry.computeVertexNormals();
      const visual = new THREE.Group();
      const mesh = new THREE.Mesh(
        geometry,
        new THREE.MeshStandardMaterial({
          color: meshColors[linkName] ?? 0xb9c4ce,
          roughness: 0.58,
          metalness: 0.08,
        })
      );

      const visualMatrix = makeOriginMatrix({ xyz: origin.xyz, rpy: origin.rpy });
      visualMatrix.decompose(mesh.position, mesh.quaternion, mesh.scale);
      visual.add(mesh);
      linkVisuals.set(linkName, visual);
      arm.add(visual);
    },
    undefined,
    () => {
      // Some deployments may omit meshes; the abstract skeleton still remains usable.
    }
  );
}

function parseRobotModel(xmlText) {
  const doc = new DOMParser().parseFromString(xmlText, 'application/xml');
  const parserError = doc.querySelector('parsererror');
  if (parserError) throw new Error('URDF XML 解析失败');

  const links = [...doc.querySelectorAll('robot > link')].map((link) => link.getAttribute('name')).filter(Boolean);
  const joints = [...doc.querySelectorAll('robot > joint')].map((joint) => {
    const origin = parseOrigin(joint);
    return {
      name: joint.getAttribute('name'),
      type: joint.getAttribute('type') ?? 'fixed',
      parent: joint.querySelector(':scope > parent')?.getAttribute('link'),
      child: joint.querySelector(':scope > child')?.getAttribute('link'),
      axis: parseVector(joint.querySelector(':scope > axis')?.getAttribute('xyz'), [0, 0, 1]),
      xyz: origin.xyz,
      rpy: origin.rpy,
    };
  }).filter((joint) => joint.name && joint.parent && joint.child);

  const childLinks = new Set(joints.map((joint) => joint.child));
  baseLinkName = links.find((link) => !childLinks.has(link)) ?? 'base_link';
  robotJoints = joints;

  for (const link of doc.querySelectorAll('robot > link')) {
    const linkName = link.getAttribute('name');
    const visual = link.querySelector(':scope > visual');
    const mesh = visual?.querySelector(':scope > geometry > mesh');
    const filename = mesh?.getAttribute('filename');
    if (!linkName || !filename) continue;
    addLinkVisual(linkName, packageMeshToUrl(filename), parseOrigin(visual));
  }
}

async function loadRobotModel() {
  const response = await fetch('/urdf/piper_description.urdf', { cache: 'no-store' });
  if (!response.ok) throw new Error(`URDF HTTP ${response.status}`);
  parseRobotModel(await response.text());
}

const els = {
  rosState: document.querySelector('#rosState'),
  age: document.querySelector('#age'),
  serviceBadge: document.querySelector('#serviceBadge'),
  enableBtn: document.querySelector('#enableBtn'),
  disableBtn: document.querySelector('#disableBtn'),
  stopCurrentBtn: document.querySelector('#stopCurrentBtn'),
  ctrlMode: document.querySelector('#ctrlMode'),
  armStatus: document.querySelector('#armStatus'),
  motionStatus: document.querySelector('#motionStatus'),
  errCode: document.querySelector('#errCode'),
  jointList: document.querySelector('#jointList'),
  feedbackPose: document.querySelector('#feedbackPose'),
  solvedPose: document.querySelector('#solvedPose'),
  cmdX: document.querySelector('#cmdX'),
  cmdY: document.querySelector('#cmdY'),
  cmdZ: document.querySelector('#cmdZ'),
  cmdRoll: document.querySelector('#cmdRoll'),
  cmdPitch: document.querySelector('#cmdPitch'),
  cmdYaw: document.querySelector('#cmdYaw'),
  cmdGripper: document.querySelector('#cmdGripper'),
  cmdSpeed: document.querySelector('#cmdSpeed'),
  cmdSpeedSlider: document.querySelector('#cmdSpeedSlider'),
  fillCurrentPoseBtn: document.querySelector('#fillCurrentPoseBtn'),
  sendMoveitPoseBtn: document.querySelector('#sendMoveitPoseBtn'),
  jogButtons: document.querySelectorAll('[data-jog-axis]'),
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
let latestEndPose = null;
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

function formatPosePosition(position) {
  if (!position) return '--';
  return `x ${Number(position.x).toFixed(3)}  y ${Number(position.y).toFixed(3)}  z ${Number(position.z).toFixed(3)} m`;
}

function fillCurrentPoseInputs() {
  const position = latestEndPose?.position;
  if (!position) {
    setMessage('还没有末端坐标反馈', true);
    return;
  }

  els.cmdX.value = Number(position.x).toFixed(3);
  els.cmdY.value = Number(position.y).toFixed(3);
  els.cmdZ.value = Number(position.z).toFixed(3);
  setMessage('已填入当前末端坐标');
}

function readCommandSpeed() {
  return Math.max(1, Math.min(100, Number(els.cmdSpeed.value) || 10));
}

function syncSpeedInputs(source) {
  const value = Math.max(1, Math.min(100, Math.round(Number(source.value) || 10)));
  els.cmdSpeed.value = String(value);
  els.cmdSpeedSlider.value = String(value);
}

function computeRobotFrames() {
  const frames = new Map([[baseLinkName, new THREE.Matrix4().identity()]]);
  for (const joint of robotJoints) {
    const parentFrame = frames.get(joint.parent) ?? new THREE.Matrix4().identity();
    const childFrame = parentFrame.clone()
      .multiply(makeOriginMatrix(joint))
      .multiply(makeMotionMatrix(joint, readJointValue(joint.name)));
    frames.set(joint.child, childFrame);
  }
  return frames;
}

function updateSolvedEndPose(frames) {
  const endFrame = frames.get('gripper_base') ?? frames.get('link6') ?? frames.get('link8') ?? frames.get('link7');
  if (!endFrame) {
    els.solvedPose.textContent = '--';
    return;
  }

  const position = new THREE.Vector3();
  endFrame.decompose(position, new THREE.Quaternion(), new THREE.Vector3());
  els.solvedPose.textContent = formatPosePosition(position);
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
  config.display.model_version = Number(nextConfig?.display?.model_version ?? config.display.model_version);
  for (const name of adjustableJoints) {
    config.display.joint_offsets[name] = Number(nextConfig?.display?.joint_offsets?.[name] ?? config.display.joint_offsets[name]);
    config.display.joint_directions[name] = Number(nextConfig?.display?.joint_directions?.[name] ?? config.display.joint_directions[name]) < 0 ? -1 : 1;
  }
  if (config.display.model_version < MODEL_VERSION) {
    config.display.model_version = MODEL_VERSION;
    for (const name of adjustableJoints) {
      config.display.joint_directions[name] = defaultConfig.display.joint_directions[name];
    }
    config.display.joint_offsets.joint2 = defaultConfig.display.joint_offsets.joint2;
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

  const frames = computeRobotFrames();

  for (const [frameName, visual] of linkVisuals.entries()) {
    const frame = frames.get(frameName);
    if (!frame) continue;
    frame.decompose(visual.position, visual.quaternion, visual.scale);
  }

  updateSolvedEndPose(frames);
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
    latestEndPose = data.end_pose ?? latestEndPose;
    els.feedbackPose.textContent = formatPosePosition(data.end_pose?.position);
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
  els.stopCurrentBtn.disabled = true;
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
    els.stopCurrentBtn.disabled = false;
  }
}

async function stopCurrentPosition() {
  if (busy) return;
  busy = true;
  els.stopCurrentBtn.disabled = true;
  setMessage('正在发送当前位置保持命令...');

  try {
    const response = await fetch('/api/stop_current', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({}),
    });
    const data = await response.json();
    if (!response.ok || !data.ok) throw new Error(data.error || data.message || `HTTP ${response.status}`);
    setMessage('已发送当前位置保持命令');
  } catch (error) {
    setMessage(`停止失败: ${error.message}`, true);
  } finally {
    busy = false;
    els.stopCurrentBtn.disabled = false;
  }
}

function readMoveitPoseCommand() {
  return {
    frame_id: 'base_link',
    group_name: 'arm',
    ik_link_name: 'link6',
    x: Number(els.cmdX.value) || 0,
    y: Number(els.cmdY.value) || 0,
    z: Number(els.cmdZ.value) || 0,
    roll: Number(els.cmdRoll.value) || 0,
    pitch: Number(els.cmdPitch.value) || 0,
    yaw: Number(els.cmdYaw.value) || 0,
    gripper: Number(els.cmdGripper.value) || 0,
    speed: readCommandSpeed(),
    avoid_collisions: true,
  };
}

async function sendMoveitPoseCommand() {
  if (busy) return;
  busy = true;
  els.sendMoveitPoseBtn.disabled = true;
  setMessage('正在请求 MoveIt IK...');

  try {
    const response = await fetch('/api/moveit_pose', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(readMoveitPoseCommand()),
    });
    const data = await response.json();
    if (!response.ok || !data.ok) throw new Error(data.error || data.message || `HTTP ${response.status}`);
    setMessage('MoveIt 已解算，关节目标已发送');
  } catch (error) {
    setMessage(`MoveIt 控制失败: ${error.message}`, true);
  } finally {
    busy = false;
    els.sendMoveitPoseBtn.disabled = false;
  }
}

async function jogPose(axis, value) {
  if (busy) return;
  busy = true;
  els.jogButtons.forEach((button) => {
    button.disabled = true;
  });
  setMessage(`正在微调 ${axis.toUpperCase()} ${value > 0 ? '+' : ''}${value.toFixed(1)}...`);

  const payload = {
    dx: axis === 'x' ? value : 0,
    dy: axis === 'y' ? value : 0,
    dz: axis === 'z' ? value : 0,
    speed: readCommandSpeed(),
  };

  try {
    const response = await fetch('/api/jog_pose', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(payload),
    });
    const data = await response.json();
    if (!response.ok || !data.ok) throw new Error(data.error || data.message || `HTTP ${response.status}`);
    setMessage(`已发送 ${axis.toUpperCase()} ${value > 0 ? '+' : ''}${value.toFixed(1)} 微调命令`);
  } catch (error) {
    setMessage(`微调失败: ${error.message}`, true);
  } finally {
    busy = false;
    els.jogButtons.forEach((button) => {
      button.disabled = false;
    });
  }
}

els.enableBtn.addEventListener('click', () => setEnable(true));
els.disableBtn.addEventListener('click', () => setEnable(false));
els.stopCurrentBtn.addEventListener('click', stopCurrentPosition);
els.fillCurrentPoseBtn.addEventListener('click', fillCurrentPoseInputs);
els.cmdSpeed.addEventListener('input', () => syncSpeedInputs(els.cmdSpeed));
els.cmdSpeedSlider.addEventListener('input', () => syncSpeedInputs(els.cmdSpeedSlider));
els.sendMoveitPoseBtn.addEventListener('click', sendMoveitPoseCommand);
els.jogButtons.forEach((button) => {
  button.addEventListener('click', () => {
    jogPose(button.dataset.jogAxis, Number(button.dataset.jogValue) || 0);
  });
});
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

async function start() {
  resize();
  drawJoints();
  setConfig(defaultConfig);
  try {
    await loadRobotModel();
  } catch (error) {
    setMessage(`URDF 加载失败: ${error.message}`, true);
  }
  await loadConfig();
  await refreshState();
  setInterval(refreshState, 200);
  requestAnimationFrame(animate);
}

start();
