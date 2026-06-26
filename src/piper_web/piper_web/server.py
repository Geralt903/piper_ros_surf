#!/usr/bin/env python3
import argparse
import json
import mimetypes
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import math
from pathlib import Path
import threading
import time
from urllib.parse import urlparse

from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK

from piper_msgs.msg import PiperStatusMsg
from piper_msgs.srv import Enable


class PiperWebNode(Node):
    def __init__(self, config_path):
        super().__init__('piper_web')
        self._config_path = config_path
        self._lock = threading.Lock()
        self._joint_state = {
            'names': [],
            'positions': [],
            'velocities': [],
            'efforts': [],
            'stamp': None,
            'age': None,
        }
        self._arm_status = None
        self._end_pose = None
        self._last_joint_time = None

        self.create_subscription(JointState, '/joint_states_single', self._on_joint_state, 10)
        self.create_subscription(PiperStatusMsg, '/arm_status', self._on_arm_status, 10)
        self.create_subscription(PoseStamped, '/end_pose_stamped', self._on_end_pose, 10)
        self._joint_command_pub = self.create_publisher(JointState, '/joint_states', 10)
        self._ik_client = self.create_client(GetPositionIK, '/compute_ik')
        self._enable_client = self.create_client(Enable, '/enable_srv')
        self.create_timer(0.25, self._refresh_age)
        self._config = self._load_config()

    def _on_joint_state(self, msg):
        now = time.time()
        with self._lock:
            self._last_joint_time = now
            self._joint_state = {
                'names': list(msg.name),
                'positions': list(msg.position),
                'velocities': list(msg.velocity),
                'efforts': list(msg.effort),
                'stamp': msg.header.stamp.sec + msg.header.stamp.nanosec / 1_000_000_000,
                'age': 0.0,
            }

    def _on_arm_status(self, msg):
        with self._lock:
            self._arm_status = {
                'ctrl_mode': msg.ctrl_mode,
                'arm_status': msg.arm_status,
                'mode_feedback': msg.mode_feedback,
                'teach_status': msg.teach_status,
                'motion_status': msg.motion_status,
                'trajectory_num': msg.trajectory_num,
                'err_code': msg.err_code,
            }

    def _on_end_pose(self, msg):
        with self._lock:
            self._end_pose = {
                'frame_id': msg.header.frame_id,
                'position': {
                    'x': msg.pose.position.x,
                    'y': msg.pose.position.y,
                    'z': msg.pose.position.z,
                },
                'orientation': {
                    'x': msg.pose.orientation.x,
                    'y': msg.pose.orientation.y,
                    'z': msg.pose.orientation.z,
                    'w': msg.pose.orientation.w,
                },
                'stamp': msg.header.stamp.sec + msg.header.stamp.nanosec / 1_000_000_000,
            }

    def _refresh_age(self):
        with self._lock:
            if self._last_joint_time is None:
                self._joint_state['age'] = None
            else:
                self._joint_state['age'] = time.time() - self._last_joint_time

    def snapshot(self):
        with self._lock:
            return {
                'joint_state': dict(self._joint_state),
                'arm_status': None if self._arm_status is None else dict(self._arm_status),
                'end_pose': None if self._end_pose is None else dict(self._end_pose),
                'enable_service_available': self._enable_client.service_is_ready(),
                'config': dict(self._config),
                'server_time': time.time(),
            }

    def config(self):
        with self._lock:
            return dict(self._config)

    def set_config(self, config):
        normalized = normalize_config(config)
        self._config_path.parent.mkdir(parents=True, exist_ok=True)
        self._config_path.write_text(json.dumps(normalized, indent=2, sort_keys=True), encoding='utf-8')
        with self._lock:
            self._config = normalized
        return normalized

    def _load_config(self):
        try:
            if self._config_path.is_file():
                return normalize_config(json.loads(self._config_path.read_text(encoding='utf-8')))
        except Exception as exc:
            self.get_logger().warn(f'failed to load config {self._config_path}: {exc}')
        return default_config()

    def set_enable(self, enabled, timeout_sec=3.0):
        if not self._enable_client.wait_for_service(timeout_sec=0.2):
            return False, 'enable service is not available'

        request = Enable.Request()
        request.enable_request = bool(enabled)
        future = self._enable_client.call_async(request)
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())

        if not done.wait(timeout_sec):
            return False, 'enable service call timed out'

        try:
            response = future.result()
        except Exception as exc:
            return False, str(exc)

        if not response.enable_response:
            return False, 'enable service returned false'
        return True, 'ok'

    def stop_current_position(self):
        with self._lock:
            names = list(self._joint_state.get('names', []))
            positions = list(self._joint_state.get('positions', []))
            age = self._joint_state.get('age')

        if not names or not positions:
            return False, 'no joint feedback is available', None
        if isinstance(age, (int, float)) and age > 1.5:
            return False, f'joint feedback is stale ({age:.2f}s)', None

        count = min(len(names), len(positions))
        command = JointState()
        command.header.stamp = self.get_clock().now().to_msg()
        command.name = names[:count]
        command.position = [float(value) for value in positions[:count]]
        command.velocity = [0.0] * count
        command.effort = [0.0] * count
        if count >= 7:
            command.effort[6] = 0.5
        self._joint_command_pub.publish(command)

        return True, 'ok', {
            'joint_names': command.name,
            'positions': command.position,
        }

    def moveit_pose_command(self, payload, timeout_sec=5.0):
        if not self._ik_client.wait_for_service(timeout_sec=0.2):
            return False, 'MoveIt /compute_ik service is not available', None

        request = GetPositionIK.Request()
        request.ik_request.group_name = str(payload.get('group_name', 'arm'))
        request.ik_request.ik_link_name = str(payload.get('ik_link_name', 'link6'))
        request.ik_request.avoid_collisions = bool(payload.get('avoid_collisions', True))
        request.ik_request.timeout = Duration(sec=2, nanosec=0)

        pose = PoseStamped()
        pose.header.frame_id = str(payload.get('frame_id', 'base_link'))
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(payload.get('x', 0.0))
        pose.pose.position.y = float(payload.get('y', 0.0))
        pose.pose.position.z = float(payload.get('z', 0.0))

        if all(key in payload for key in ('qx', 'qy', 'qz', 'qw')):
            qx = float(payload.get('qx', 0.0))
            qy = float(payload.get('qy', 0.0))
            qz = float(payload.get('qz', 0.0))
            qw = float(payload.get('qw', 1.0))
        else:
            roll = float(payload.get('roll', 0.0))
            pitch = float(payload.get('pitch', 0.0))
            yaw = float(payload.get('yaw', 0.0))
            qx, qy, qz, qw = quaternion_from_euler(roll, pitch, yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        request.ik_request.pose_stamped = pose

        with self._lock:
            seed_names = list(self._joint_state.get('names', []))
            seed_positions = list(self._joint_state.get('positions', []))
            config = dict(self._config)
        if seed_names and seed_positions:
            valid_seed_names = []
            valid_seed_positions = []
            for name, position in zip(seed_names, seed_positions):
                moveit_name = 'joint7' if name == 'gripper' else name
                if moveit_name not in {'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7'}:
                    continue
                valid_seed_names.append(moveit_name)
                valid_seed_positions.append(driver_joint_to_urdf(moveit_name, position, config))

            request.ik_request.robot_state.joint_state.name = valid_seed_names
            request.ik_request.robot_state.joint_state.position = valid_seed_positions

        future = self._ik_client.call_async(request)
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout_sec):
            return False, 'MoveIt IK request timed out', None

        try:
            response = future.result()
        except Exception as exc:
            return False, str(exc), None

        if response.error_code.val != response.error_code.SUCCESS:
            return False, f'MoveIt IK failed with code {response.error_code.val}', None

        joint_map = dict(zip(response.solution.joint_state.name, response.solution.joint_state.position))
        command = JointState()
        command.header.stamp = self.get_clock().now().to_msg()
        command.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7']
        gripper = float(payload.get('gripper', 0.0))
        command.position = [
            urdf_joint_to_driver(name, float(joint_map.get(name, 0.0)), config)
            for name in command.name[:6]
        ] + [gripper]
        speed = max(1.0, min(100.0, float(payload.get('speed', 10.0))))
        command.velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, speed]
        command.effort = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5]
        self._joint_command_pub.publish(command)

        return True, 'ok', {
            'joint_names': command.name,
            'positions': command.position,
        }

    def jog_pose_command(self, payload):
        with self._lock:
            end_pose = None if self._end_pose is None else dict(self._end_pose)
            joint_names = list(self._joint_state.get('names', []))
            joint_positions = list(self._joint_state.get('positions', []))

        if end_pose is None:
            return False, 'no end pose feedback is available', None

        position = end_pose.get('position') or {}
        orientation = end_pose.get('orientation') or {}
        gripper = 0.0
        if 'gripper' in joint_names:
            gripper_index = joint_names.index('gripper')
            if gripper_index < len(joint_positions):
                gripper = float(joint_positions[gripper_index])
        elif len(joint_positions) >= 7:
            gripper = float(joint_positions[6])

        target = {
            'frame_id': payload.get('frame_id', 'base_link'),
            'group_name': payload.get('group_name', 'arm'),
            'ik_link_name': payload.get('ik_link_name', 'link6'),
            'x': float(position.get('x', 0.0)) + float(payload.get('dx', 0.0)),
            'y': float(position.get('y', 0.0)) + float(payload.get('dy', 0.0)),
            'z': float(position.get('z', 0.0)) + float(payload.get('dz', 0.0)),
            'qx': float(orientation.get('x', 0.0)),
            'qy': float(orientation.get('y', 0.0)),
            'qz': float(orientation.get('z', 0.0)),
            'qw': float(orientation.get('w', 1.0)),
            'gripper': float(payload.get('gripper', gripper)),
            'speed': float(payload.get('speed', 10.0)),
            'avoid_collisions': bool(payload.get('avoid_collisions', True)),
        }
        return self.moveit_pose_command(target)


class PiperRequestHandler(BaseHTTPRequestHandler):
    server_version = 'PiperWeb/0.1'

    def log_message(self, fmt, *args):
        return

    def do_GET(self):
        parsed = urlparse(self.path)
        if parsed.path == '/api/state':
            self._send_json(self.server.ros_node.snapshot())
            return
        if parsed.path == '/api/config':
            self._send_json(self.server.ros_node.config())
            return

        if parsed.path.startswith('/meshes/'):
            mesh_name = parsed.path[8:]
            if '..' in mesh_name:
                self.send_error(404)
                return
            target = self.server.mesh_dir / mesh_name
            if not target.is_file():
                self.send_error(404)
                return
            content_type = mimetypes.guess_type(str(target))[0] or 'application/octet-stream'
            body = target.read_bytes()
            self.send_response(200)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(body)
            return

        if parsed.path == '/urdf/piper_description.urdf':
            target = self.server.urdf_path
            if not target.is_file():
                self.send_error(404)
                return
            body = target.read_bytes()
            self.send_response(200)
            self.send_header('Content-Type', 'application/xml')
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(body)
            return

        path = parsed.path if parsed.path != '/' else '/index.html'
        clean = path.lstrip('/')
        if '..' in clean:
            self.send_error(404)
            return
        target = self.server.static_dir / clean
        if not target.is_file():
            self.send_error(404)
            return

        content_type = mimetypes.guess_type(str(target))[0] or 'application/octet-stream'
        body = target.read_bytes()
        self.send_response(200)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(body)

    def do_POST(self):
        parsed = urlparse(self.path)
        if parsed.path == '/api/config':
            try:
                length = int(self.headers.get('Content-Length', '0'))
                payload = json.loads(self.rfile.read(length).decode('utf-8') or '{}')
                config = self.server.ros_node.set_config(payload)
            except Exception as exc:
                self._send_json({'ok': False, 'error': str(exc)}, status=400)
                return

            self._send_json({'ok': True, 'config': config})
            return

        if parsed.path == '/api/moveit_pose':
            try:
                length = int(self.headers.get('Content-Length', '0'))
                payload = json.loads(self.rfile.read(length).decode('utf-8') or '{}')
                ok, message, solution = self.server.ros_node.moveit_pose_command(payload)
            except Exception as exc:
                self._send_json({'ok': False, 'error': str(exc)}, status=400)
                return

            self._send_json(
                {'ok': ok, 'message': message, 'solution': solution},
                status=200 if ok else 503,
            )
            return

        if parsed.path == '/api/stop_current':
            ok, message, command = self.server.ros_node.stop_current_position()
            self._send_json(
                {'ok': ok, 'message': message, 'command': command},
                status=200 if ok else 503,
            )
            return

        if parsed.path == '/api/jog_pose':
            try:
                length = int(self.headers.get('Content-Length', '0'))
                payload = json.loads(self.rfile.read(length).decode('utf-8') or '{}')
                ok, message, solution = self.server.ros_node.jog_pose_command(payload)
            except Exception as exc:
                self._send_json({'ok': False, 'error': str(exc)}, status=400)
                return

            self._send_json(
                {'ok': ok, 'message': message, 'solution': solution},
                status=200 if ok else 503,
            )
            return

        if parsed.path != '/api/enable':
            self.send_error(404)
            return

        try:
            length = int(self.headers.get('Content-Length', '0'))
            payload = json.loads(self.rfile.read(length).decode('utf-8') or '{}')
            enabled = bool(payload['enable'])
        except Exception:
            self._send_json({'ok': False, 'error': 'invalid request'}, status=400)
            return

        ok, message = self.server.ros_node.set_enable(enabled)
        self._send_json({'ok': ok, 'message': message, 'enable': enabled}, status=200 if ok else 503)

    def _send_json(self, payload, status=200):
        body = json.dumps(payload).encode('utf-8')
        self.send_response(status)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(body)


def default_config():
    return {
        'display': {
            'model_version': 3,
            'joint_offsets': {
                'joint1': 0.0,
                'joint2': 0.0,
                'joint3': 0.0,
                'joint4': 0.0,
                'joint5': 0.0,
                'joint6': 0.0,
                'joint7': 0.0,
                'joint8': 0.0,
            },
            'joint_directions': {
                'joint1': 1,
                'joint2': -1,
                'joint3': 1,
                'joint4': 1,
                'joint5': 1,
                'joint6': 1,
                'joint7': 1,
                'joint8': 1,
            },
        },
        'ik': {
            'base_height': 0.123,
            'joint2_pitch_bias': -0.1359,
            'joint3_yaw_bias': -1.7939,
            'upper_arm_length': 0.28503,
            'forearm_x': -0.021984,
            'forearm_y': -0.25075,
            'wrist_y': -0.091,
            'tool_z': 0.1358,
        },
    }


def quaternion_from_euler(roll, pitch, yaw):
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def driver_joint_to_urdf(name, value, config):
    display = config.get('display', {}) if isinstance(config, dict) else {}
    offsets = display.get('joint_offsets', {}) if isinstance(display, dict) else {}
    directions = display.get('joint_directions', {}) if isinstance(display, dict) else {}
    direction = -1.0 if float(directions.get(name, 1.0)) < 0 else 1.0
    offset = float(offsets.get(name, 0.0))
    return float(value) * direction + offset


def urdf_joint_to_driver(name, value, config):
    display = config.get('display', {}) if isinstance(config, dict) else {}
    offsets = display.get('joint_offsets', {}) if isinstance(display, dict) else {}
    directions = display.get('joint_directions', {}) if isinstance(display, dict) else {}
    direction = -1.0 if float(directions.get(name, 1.0)) < 0 else 1.0
    offset = float(offsets.get(name, 0.0))
    return (float(value) - offset) / direction


def normalize_config(config):
    defaults = default_config()
    normalized = default_config()

    display = config.get('display', {}) if isinstance(config, dict) else {}
    display_model_version = int(display.get('model_version', 1)) if isinstance(display, dict) else 1
    normalized['display']['model_version'] = defaults['display']['model_version']
    offsets = display.get('joint_offsets', {}) if isinstance(display, dict) else {}
    directions = display.get('joint_directions', {}) if isinstance(display, dict) else {}
    for name in defaults['display']['joint_offsets']:
        normalized['display']['joint_offsets'][name] = float(offsets.get(name, defaults['display']['joint_offsets'][name]))
        direction = int(directions.get(name, defaults['display']['joint_directions'][name]))
        if display_model_version < defaults['display']['model_version'] and name == 'joint8':
            direction = defaults['display']['joint_directions'][name]
        normalized['display']['joint_directions'][name] = -1 if direction < 0 else 1

    ik = config.get('ik', {}) if isinstance(config, dict) else {}
    for name, value in defaults['ik'].items():
        normalized['ik'][name] = float(ik.get(name, value))
    return normalized


def start_http_server(node, host, port):
    static_dir = Path(get_package_share_directory('piper_web')) / 'static'
    description_dir = Path(get_package_share_directory('piper_description'))
    mesh_dir = description_dir / 'meshes'
    urdf_path = description_dir / 'urdf' / 'piper_description.urdf'
    httpd = ThreadingHTTPServer((host, port), PiperRequestHandler)
    httpd.ros_node = node
    httpd.static_dir = static_dir
    httpd.mesh_dir = mesh_dir
    httpd.urdf_path = urdf_path
    thread = threading.Thread(target=httpd.serve_forever, daemon=True)
    thread.start()
    node.get_logger().info(f'Piper web monitor: http://{host}:{port}')
    return httpd


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--host', default='0.0.0.0')
    parser.add_argument('--port', type=int, default=8080)
    parser.add_argument('--config', default=str(Path.home() / '.ros' / 'piper_web_config.json'))
    parsed, ros_args = parser.parse_known_args(args)

    rclpy.init(args=ros_args)
    node = PiperWebNode(Path(parsed.config).expanduser())
    httpd = start_http_server(node, parsed.host, parsed.port)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        httpd.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
