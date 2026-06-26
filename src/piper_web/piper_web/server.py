#!/usr/bin/env python3
import argparse
import json
import mimetypes
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import threading
import time
from urllib.parse import urlparse

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

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
        self._last_joint_time = None

        self.create_subscription(JointState, '/joint_states_single', self._on_joint_state, 10)
        self.create_subscription(PiperStatusMsg, '/arm_status', self._on_arm_status, 10)
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


def normalize_config(config):
    defaults = default_config()
    normalized = default_config()

    display = config.get('display', {}) if isinstance(config, dict) else {}
    offsets = display.get('joint_offsets', {}) if isinstance(display, dict) else {}
    directions = display.get('joint_directions', {}) if isinstance(display, dict) else {}
    for name in defaults['display']['joint_offsets']:
        normalized['display']['joint_offsets'][name] = float(offsets.get(name, defaults['display']['joint_offsets'][name]))
        direction = int(directions.get(name, defaults['display']['joint_directions'][name]))
        normalized['display']['joint_directions'][name] = -1 if direction < 0 else 1

    ik = config.get('ik', {}) if isinstance(config, dict) else {}
    for name, value in defaults['ik'].items():
        normalized['ik'][name] = float(ik.get(name, value))
    return normalized


def start_http_server(node, host, port):
    static_dir = Path(get_package_share_directory('piper_web')) / 'static'
    mesh_dir = Path(get_package_share_directory('piper_description')) / 'meshes'
    httpd = ThreadingHTTPServer((host, port), PiperRequestHandler)
    httpd.ros_node = node
    httpd.static_dir = static_dir
    httpd.mesh_dir = mesh_dir
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
