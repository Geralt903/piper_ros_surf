#!/usr/bin/env python3
# coding=utf-8
"""Tiny HTTP frontend and ROS topic bridge for the Piper MuJoCo simulation."""

import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rospy
from geometry_msgs.msg import PoseStamped, WrenchStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray, Header


JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7", "joint8"]


INDEX_HTML = """<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Piper MuJoCo ROS Control</title>
  <style>
    :root { color-scheme: light dark; font-family: system-ui, sans-serif; }
    body { margin: 0; background: #f5f7f8; color: #172026; }
    header { padding: 16px 20px; background: #18232d; color: white; }
    main { display: grid; grid-template-columns: minmax(340px, 520px) 1fr; gap: 16px; padding: 16px; }
    section { background: white; border: 1px solid #d9e0e6; border-radius: 8px; padding: 14px; }
    h1 { font-size: 20px; margin: 0; }
    h2 { font-size: 15px; margin: 0 0 12px; }
    .joint { display: grid; grid-template-columns: 62px 1fr 72px; gap: 10px; align-items: center; margin: 8px 0; }
    input[type=range] { width: 100%; }
    input[type=number] { width: 100%; box-sizing: border-box; padding: 6px; }
    button { border: 1px solid #1b5d7a; background: #1f7a9d; color: white; padding: 8px 11px; border-radius: 6px; cursor: pointer; }
    button.secondary { background: #fff; color: #1f4d62; }
    button.warn { background: #9d5a1f; border-color: #754114; }
    .row { display: flex; gap: 8px; flex-wrap: wrap; margin: 10px 0; }
    .grid { display: grid; grid-template-columns: repeat(3, minmax(80px, 1fr)); gap: 8px; }
    .kv { display: grid; grid-template-columns: 160px 1fr; gap: 6px; font-family: ui-monospace, monospace; font-size: 13px; }
    .ok { color: #147d3f; }
    .bad { color: #9b1c1c; }
    pre { margin: 0; white-space: pre-wrap; word-break: break-word; font-size: 12px; }
    @media (max-width: 860px) { main { grid-template-columns: 1fr; } }
  </style>
</head>
<body>
  <header><h1>Piper MuJoCo ROS Control</h1></header>
  <main>
    <section>
      <h2>Joint Target</h2>
      <div id="joints"></div>
      <div class="row">
        <button onclick="sendJoint()">Publish joint target</button>
        <button class="secondary" onclick="setPreset('home')">Home</button>
        <button class="secondary" onclick="setPreset('reach')">Reach</button>
      </div>
      <h2>Tool Force</h2>
      <div class="grid">
        <label>Fx <input id="fx" type="number" value="0" step="1"></label>
        <label>Fy <input id="fy" type="number" value="0" step="1"></label>
        <label>Fz <input id="fz" type="number" value="0" step="1"></label>
      </div>
      <div class="row">
        <button onclick="sendForce()">Publish force</button>
        <button class="secondary" onclick="zeroForce()">Zero force</button>
      </div>
      <h2>Gripper</h2>
      <div class="joint">
        <label>open</label>
        <input id="gripper" type="range" min="0" max="0.03" step="0.0005" value="0.025">
        <input id="gripper_num" type="number" min="0" max="0.03" step="0.0005" value="0.025">
      </div>
      <div class="row">
        <button onclick="sendGripper()">Publish gripper</button>
        <button class="secondary" onclick="setGripper(0.03)">Open</button>
        <button class="secondary" onclick="setGripper(0.0)">Close</button>
      </div>
      <h2>Run Speed</h2>
      <div class="joint">
        <label>speed</label>
        <input id="speed" type="range" min="0" max="4" step="0.05" value="1">
        <input id="speed_num" type="number" min="0" max="4" step="0.05" value="1">
      </div>
      <div class="row">
        <button onclick="sendSpeed()">Set speed</button>
        <button class="secondary" onclick="setSpeed(0)">Pause</button>
        <button class="secondary" onclick="setSpeed(1)">1x</button>
        <button class="secondary" onclick="setSpeed(2)">2x</button>
      </div>
      <h2>Pose Target</h2>
      <div class="grid">
        <label>x <input id="px" type="number" value="0.16" step="0.01"></label>
        <label>y <input id="py" type="number" value="0" step="0.01"></label>
        <label>z <input id="pz" type="number" value="0.13" step="0.01"></label>
        <label>qx <input id="qx" type="number" value="0" step="0.01"></label>
        <label>qy <input id="qy" type="number" value="-0.8011" step="0.01"></label>
        <label>qz <input id="qz" type="number" value="0" step="0.01"></label>
        <label>qw <input id="qw" type="number" value="-0.5985" step="0.01"></label>
      </div>
      <div class="row">
        <button onclick="sendPose()">Publish pose target</button>
      </div>
    </section>

    <section>
      <h2>Status</h2>
      <div class="kv">
        <div>Bridge</div><div id="bridge" class="bad">connecting</div>
        <div>Joint names</div><div id="names"></div>
        <div>Joint position</div><div id="positions"></div>
        <div>Run speed</div><div id="speed_state"></div>
        <div>End effector</div><div id="ee"></div>
        <div>Hand camera</div><div id="camera"></div>
        <div>Stability</div><div id="stability"></div>
      </div>
      <h2>Raw State</h2>
      <pre id="raw"></pre>
    </section>
  </main>
  <script>
    const presets = {
      home: [0.0, 1.0, -1.2, 0.0, 0.7, 0.0, 0.025, -0.025],
      reach: [0.0, 0.85, -1.45, 0.0, 0.85, 0.0, 0.010, -0.010]
    };
    const jointRanges = [
      [-2.618, 2.618],
      [0.0, 3.14158],
      [-2.697, 0.0],
      [-1.832, 1.832],
      [-1.22, 1.22],
      [-3.14158, 3.14158],
      [0.0, 0.0475],
      [-0.0475, 0.0]
    ];

    function makeJointControls() {
      const root = document.getElementById('joints');
      for (let i = 0; i < presets.home.length; i++) {
        const name = `joint${i + 1}`;
        const [min, max] = jointRanges[i];
        const step = i >= 6 ? '0.0005' : '0.001';
        const row = document.createElement('div');
        row.className = 'joint';
        row.innerHTML = `<label>${name}</label>
          <input id="${name}" type="range" min="${min}" max="${max}" step="${step}" value="${presets.home[i]}">
          <input id="${name}_num" type="number" min="${min}" max="${max}" step="${step}" value="${presets.home[i]}">`;
        root.appendChild(row);
        const slider = document.getElementById(name);
        const number = document.getElementById(`${name}_num`);
        slider.addEventListener('input', () => { number.value = slider.value; });
        number.addEventListener('input', () => { slider.value = number.value; });
      }
    }

    function jointValues() {
      return Array.from({length: presets.home.length}, (_, i) => Number(document.getElementById(`joint${i + 1}`).value));
    }

    function setPreset(name) {
      presets[name].forEach((v, i) => {
        document.getElementById(`joint${i + 1}`).value = v;
        document.getElementById(`joint${i + 1}_num`).value = v;
      });
      sendJoint();
    }

    async function post(path, body) {
      const res = await fetch(path, {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify(body)
      });
      if (!res.ok) throw new Error(await res.text());
      return res.json();
    }

    async function sendJoint() {
      await post('/api/joint_target', {positions: jointValues()});
    }

    async function sendForce() {
      await post('/api/tool_force', {force: ['fx', 'fy', 'fz'].map(id => Number(document.getElementById(id).value))});
    }

    async function zeroForce() {
      ['fx', 'fy', 'fz'].forEach(id => document.getElementById(id).value = 0);
      await sendForce();
    }

    async function sendGripper() {
      await post('/api/gripper_target', {opening: Number(document.getElementById('gripper').value)});
    }

    async function setGripper(opening) {
      document.getElementById('gripper').value = opening;
      document.getElementById('gripper_num').value = opening;
      await sendGripper();
    }

    async function sendSpeed() {
      await post('/api/speed_scale', {speed_scale: Number(document.getElementById('speed').value)});
    }

    async function setSpeed(speed) {
      document.getElementById('speed').value = speed;
      document.getElementById('speed_num').value = speed;
      await sendSpeed();
    }

    async function sendPose() {
      await post('/api/pose_target', {
        position: ['px', 'py', 'pz'].map(id => Number(document.getElementById(id).value)),
        orientation: ['qx', 'qy', 'qz', 'qw'].map(id => Number(document.getElementById(id).value))
      });
    }

    async function pollState() {
      try {
        const state = await fetch('/api/state').then(r => r.json());
        document.getElementById('bridge').textContent = 'connected';
        document.getElementById('bridge').className = 'ok';
        document.getElementById('names').textContent = (state.joint_state.name || []).join(', ');
        document.getElementById('positions').textContent = (state.joint_state.position || []).map(v => v.toFixed(3)).join(', ');
        const speedScale = Number(state.speed_scale || 0);
        document.getElementById('speed_state').textContent = `${speedScale.toFixed(2)}x`;
        if (document.activeElement !== document.getElementById('speed') &&
            document.activeElement !== document.getElementById('speed_num')) {
          document.getElementById('speed').value = speedScale;
          document.getElementById('speed_num').value = speedScale.toFixed(2);
        }
        const p = state.end_effector_pose.position || {};
        document.getElementById('ee').textContent = `x=${fmt(p.x)} y=${fmt(p.y)} z=${fmt(p.z)}`;
        const c = state.hand_camera_pose.position || {};
        document.getElementById('camera').textContent = `x=${fmt(c.x)} y=${fmt(c.y)} z=${fmt(c.z)}`;
        document.getElementById('stability').textContent = (state.stability || []).map(v => Number(v).toFixed(4)).join(', ');
        document.getElementById('raw').textContent = JSON.stringify(state, null, 2);
      } catch (err) {
        document.getElementById('bridge').textContent = err.message;
        document.getElementById('bridge').className = 'bad';
      }
    }

    function fmt(v) { return Number(v || 0).toFixed(3); }
    makeJointControls();
    document.getElementById('gripper').addEventListener('input', () => {
      document.getElementById('gripper_num').value = document.getElementById('gripper').value;
    });
    document.getElementById('gripper_num').addEventListener('input', () => {
      document.getElementById('gripper').value = document.getElementById('gripper_num').value;
    });
    document.getElementById('speed').addEventListener('input', () => {
      document.getElementById('speed_num').value = document.getElementById('speed').value;
    });
    document.getElementById('speed_num').addEventListener('input', () => {
      document.getElementById('speed').value = document.getElementById('speed_num').value;
    });
    pollState();
    setInterval(pollState, 500);
  </script>
</body>
</html>
"""


class BridgeState:
    def __init__(self):
        self.lock = threading.Lock()
        self.joint_state = {}
        self.end_effector_pose = {}
        self.hand_camera_pose = {}
        self.stability = []
        self.speed_scale = 1.0

    def update_joint_state(self, msg):
        with self.lock:
            self.joint_state = {
                "name": list(msg.name),
                "position": list(msg.position),
                "velocity": list(msg.velocity),
                "effort": list(msg.effort),
            }

    def update_pose(self, msg):
        with self.lock:
            self.end_effector_pose = {
                "frame_id": msg.header.frame_id,
                "position": {
                    "x": msg.pose.position.x,
                    "y": msg.pose.position.y,
                    "z": msg.pose.position.z,
                },
                "orientation": {
                    "x": msg.pose.orientation.x,
                    "y": msg.pose.orientation.y,
                    "z": msg.pose.orientation.z,
                    "w": msg.pose.orientation.w,
                },
            }

    def update_hand_camera_pose(self, msg):
        with self.lock:
            self.hand_camera_pose = {
                "frame_id": msg.header.frame_id,
                "position": {
                    "x": msg.pose.position.x,
                    "y": msg.pose.position.y,
                    "z": msg.pose.position.z,
                },
                "orientation": {
                    "x": msg.pose.orientation.x,
                    "y": msg.pose.orientation.y,
                    "z": msg.pose.orientation.z,
                    "w": msg.pose.orientation.w,
                },
            }

    def update_stability(self, msg):
        with self.lock:
            self.stability = list(msg.data)

    def update_speed_scale(self, msg):
        with self.lock:
            self.speed_scale = float(msg.data)

    def snapshot(self):
        with self.lock:
            return {
                "joint_state": self.joint_state,
                "end_effector_pose": self.end_effector_pose,
                "hand_camera_pose": self.hand_camera_pose,
                "stability": self.stability,
                "speed_scale": self.speed_scale,
            }


class RosBridge:
    def __init__(self):
        self.state = BridgeState()
        self.joint_pub = rospy.Publisher("/long_arm/joint_target", JointState, queue_size=1)
        self.gripper_pub = rospy.Publisher("/long_arm/gripper_target", Float64, queue_size=1)
        self.pose_pub = rospy.Publisher("/long_arm/target_pose", PoseStamped, queue_size=1)
        self.force_pub = rospy.Publisher("/long_arm/tool_force", WrenchStamped, queue_size=1)
        self.speed_pub = rospy.Publisher("/long_arm/speed_scale", Float64, queue_size=1)
        rospy.Subscriber("/joint_states", JointState, self.state.update_joint_state, queue_size=1)
        rospy.Subscriber("/long_arm/end_effector_pose", PoseStamped, self.state.update_pose, queue_size=1)
        rospy.Subscriber("/long_arm/hand_camera_pose", PoseStamped, self.state.update_hand_camera_pose, queue_size=1)
        rospy.Subscriber("/long_arm/stability", Float64MultiArray, self.state.update_stability, queue_size=1)
        rospy.Subscriber("/long_arm/speed_scale_state", Float64, self.state.update_speed_scale, queue_size=1)

    def publish_joint_target(self, payload):
        positions = payload.get("positions")
        if not isinstance(positions, list) or len(positions) not in (6, len(JOINT_NAMES)):
            raise ValueError("positions must be a list of 6 arm joints or {} arm+gripper joints".format(len(JOINT_NAMES)))
        msg = JointState()
        msg.header = Header(stamp=rospy.Time.now())
        msg.name = JOINT_NAMES[: len(positions)]
        msg.position = [float(v) for v in positions]
        self.joint_pub.publish(msg)
        return {"ok": True, "topic": "/long_arm/joint_target", "position": msg.position}

    def publish_gripper_target(self, payload):
        if "opening" not in payload:
            raise ValueError("opening is required")
        msg = Float64()
        msg.data = float(payload["opening"])
        self.gripper_pub.publish(msg)
        return {"ok": True, "topic": "/long_arm/gripper_target", "opening": msg.data}

    def publish_pose_target(self, payload):
        position = payload.get("position")
        orientation = payload.get("orientation")
        if not isinstance(position, list) or len(position) != 3:
            raise ValueError("position must be a list of 3 numbers")
        if not isinstance(orientation, list) or len(orientation) != 4:
            raise ValueError("orientation must be a list of 4 numbers [x,y,z,w]")
        msg = PoseStamped()
        msg.header = Header(stamp=rospy.Time.now(), frame_id="world")
        msg.pose.position.x = float(position[0])
        msg.pose.position.y = float(position[1])
        msg.pose.position.z = float(position[2])
        msg.pose.orientation.x = float(orientation[0])
        msg.pose.orientation.y = float(orientation[1])
        msg.pose.orientation.z = float(orientation[2])
        msg.pose.orientation.w = float(orientation[3])
        self.pose_pub.publish(msg)
        return {"ok": True, "topic": "/long_arm/target_pose"}

    def publish_tool_force(self, payload):
        force = payload.get("force")
        if not isinstance(force, list) or len(force) != 3:
            raise ValueError("force must be a list of 3 numbers")
        msg = WrenchStamped()
        msg.header = Header(stamp=rospy.Time.now(), frame_id="world")
        msg.wrench.force.x = float(force[0])
        msg.wrench.force.y = float(force[1])
        msg.wrench.force.z = float(force[2])
        self.force_pub.publish(msg)
        return {"ok": True, "topic": "/long_arm/tool_force"}

    def publish_speed_scale(self, payload):
        if "speed_scale" not in payload:
            raise ValueError("speed_scale is required")
        msg = Float64()
        msg.data = max(0.0, min(4.0, float(payload["speed_scale"])))
        self.speed_pub.publish(msg)
        return {"ok": True, "topic": "/long_arm/speed_scale", "speed_scale": msg.data}


def make_handler(bridge):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            rospy.logdebug("web bridge: " + fmt, *args)

        def _send(self, status, body, content_type="application/json"):
            if isinstance(body, (dict, list)):
                body = json.dumps(body).encode("utf-8")
            elif isinstance(body, str):
                body = body.encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Access-Control-Allow-Headers", "Content-Type")
            self.send_header("Access-Control-Allow-Methods", "GET,POST,OPTIONS")
            self.end_headers()
            self.wfile.write(body)

        def do_OPTIONS(self):
            self._send(204, b"")

        def do_HEAD(self):
            if self.path == "/" or self.path == "/index.html" or self.path == "/api/state":
                self.send_response(200)
                self.send_header("Access-Control-Allow-Origin", "*")
                self.end_headers()
            else:
                self.send_response(404)
                self.end_headers()

        def do_GET(self):
            if self.path == "/" or self.path == "/index.html":
                self._send(200, INDEX_HTML, "text/html; charset=utf-8")
            elif self.path == "/api/state":
                self._send(200, bridge.state.snapshot())
            else:
                self._send(404, {"ok": False, "error": "not found"})

        def do_POST(self):
            try:
                length = int(self.headers.get("Content-Length", "0"))
                payload = json.loads(self.rfile.read(length).decode("utf-8") or "{}")
                if self.path == "/api/joint_target":
                    result = bridge.publish_joint_target(payload)
                elif self.path == "/api/gripper_target":
                    result = bridge.publish_gripper_target(payload)
                elif self.path == "/api/pose_target":
                    result = bridge.publish_pose_target(payload)
                elif self.path == "/api/tool_force":
                    result = bridge.publish_tool_force(payload)
                elif self.path == "/api/speed_scale":
                    result = bridge.publish_speed_scale(payload)
                else:
                    self._send(404, {"ok": False, "error": "not found"})
                    return
                self._send(200, result)
            except Exception as exc:
                self._send(400, {"ok": False, "error": str(exc)})

    return Handler


def main():
    rospy.init_node("piper_mujoco_web_bridge", anonymous=False)
    host = rospy.get_param("~host", "0.0.0.0")
    port = int(rospy.get_param("~port", 8088))
    bridge = RosBridge()
    server = ThreadingHTTPServer((host, port), make_handler(bridge))
    rospy.loginfo("Piper MuJoCo web frontend: http://localhost:%s", port)

    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    rospy.on_shutdown(server.shutdown)
    rospy.spin()


if __name__ == "__main__":
    main()
