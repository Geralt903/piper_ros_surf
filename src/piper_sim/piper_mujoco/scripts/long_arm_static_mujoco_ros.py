#!/usr/bin/env python3
# coding=utf-8
"""ROS1 + MuJoCo simulation node using the real Piper model."""

import math
import os
import subprocess
import threading
import time

import mujoco
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, WrenchStamped
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Float64, Float64MultiArray, Header


G = 9.81
DEFAULT_JOINT_NAMES = (
    "joint1",
    "joint2",
    "joint3",
    "joint4",
    "joint5",
    "joint6",
    "joint7",
    "joint8",
)
SUPPORT_HALF_LENGTH = 0.11


def normalize_quat(q):
    norm = np.linalg.norm(q)
    if norm < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0])
    return q / norm


def quat_conjugate(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])


def quat_multiply(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array(
        [
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ]
    )


def quat_error_vector(target_wxyz, current_wxyz):
    error = quat_multiply(normalize_quat(target_wxyz), quat_conjugate(normalize_quat(current_wxyz)))
    if error[0] < 0.0:
        error = -error
    vector = error[1:]
    vector_norm = np.linalg.norm(vector)
    if vector_norm < 1e-9:
        return np.zeros(3)
    angle = 2.0 * math.atan2(vector_norm, error[0])
    return angle * vector / vector_norm


def ros_quat_to_wxyz(q):
    return normalize_quat(np.array([q.w, q.x, q.y, q.z]))


def wxyz_to_ros_quat(q, msg):
    q = normalize_quat(q)
    msg.w = q[0]
    msg.x = q[1]
    msg.y = q[2]
    msg.z = q[3]


def rotmat_to_wxyz(rot):
    trace = float(np.trace(rot))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        return normalize_quat(np.array([
            0.25 * s,
            (rot[2, 1] - rot[1, 2]) / s,
            (rot[0, 2] - rot[2, 0]) / s,
            (rot[1, 0] - rot[0, 1]) / s,
        ]))
    axis = int(np.argmax(np.diag(rot)))
    if axis == 0:
        s = math.sqrt(1.0 + rot[0, 0] - rot[1, 1] - rot[2, 2]) * 2.0
        q = np.array([(rot[2, 1] - rot[1, 2]) / s, 0.25 * s, (rot[0, 1] + rot[1, 0]) / s, (rot[0, 2] + rot[2, 0]) / s])
    elif axis == 1:
        s = math.sqrt(1.0 + rot[1, 1] - rot[0, 0] - rot[2, 2]) * 2.0
        q = np.array([(rot[0, 2] - rot[2, 0]) / s, (rot[0, 1] + rot[1, 0]) / s, 0.25 * s, (rot[1, 2] + rot[2, 1]) / s])
    else:
        s = math.sqrt(1.0 + rot[2, 2] - rot[0, 0] - rot[1, 1]) * 2.0
        q = np.array([(rot[1, 0] - rot[0, 1]) / s, (rot[0, 2] + rot[2, 0]) / s, (rot[1, 2] + rot[2, 1]) / s, 0.25 * s])
    return normalize_quat(q)


class PiperMujocoRos:
    def __init__(self):
        rospy.init_node("piper_real_mujoco_ros", anonymous=False)
        self.lock = threading.Lock()

        self.rate_hz = float(rospy.get_param("~rate_hz", 200.0))
        self.speed_scale = float(rospy.get_param("~speed_scale", 1.0))
        self.viewer_enabled = bool(rospy.get_param("~viewer", False))
        self.demo_enabled = bool(rospy.get_param("~demo", True))
        self.hold_time = float(rospy.get_param("~hold_time", 3.0))
        self.ee_body_name = rospy.get_param("~ee_body", "link6")
        self.world_frame = rospy.get_param("~world_frame", "world")
        self.joint_names = tuple(rospy.get_param("~joint_names", list(DEFAULT_JOINT_NAMES)))
        self.gripper_open = float(rospy.get_param("~gripper_open", 0.030))
        self.gripper_closed = float(rospy.get_param("~gripper_closed", 0.0))
        self.camera_name = rospy.get_param("~camera_name", "hand_camera")
        self.publish_camera_image_enabled = bool(rospy.get_param("~publish_camera_image", False))
        self.camera_width = int(rospy.get_param("~camera_width", 320))
        self.camera_height = int(rospy.get_param("~camera_height", 240))
        self.camera_rate_hz = float(rospy.get_param("~camera_rate_hz", 10.0))
        self.last_camera_pub = 0.0

        package_path = subprocess.check_output(
            "rospack find piper_description", shell=True
        ).strip().decode("utf-8")
        xml_path = rospy.get_param(
            "~xml_path",
            os.path.join(package_path, "mujoco_model", "piper_wall_mount_description.xml"),
        )
        urdf_path = rospy.get_param(
            "~urdf_path",
            os.path.join(package_path, "urdf", "piper_description.urdf"),
        )
        self.publish_robot_description(urdf_path)

        self.model = mujoco.MjModel.from_xml_path(os.path.abspath(xml_path))
        self.data = mujoco.MjData(self.model)
        self.ik_data = mujoco.MjData(self.model)
        self.ee_body_id = int(self.model.body(self.ee_body_name).id)
        self.camera_id = self.resolve_camera_id()
        self.camera_renderer = self.create_camera_renderer()
        self.tool_force = np.zeros(3)
        self.target_pose = None
        self.q_direct = None

        self.actuator_ids = self.resolve_actuators()
        self.q_home = np.array(
            rospy.get_param("~home_q", [0.0, 1.0, -1.2, 0.0, 0.7, 0.0, 0.025, -0.025]),
            dtype=float,
        )
        self.demo_sequence = [
            ("home", self.q_home, np.array([0.0, 0.0, 0.0])),
            ("reach", np.array([0.0, 0.85, -1.45, 0.0, 0.85, 0.0, 0.025, -0.025]), np.array([0.0, 0.0, 0.0])),
            ("force_hold", np.array([0.0, 0.85, -1.45, 0.0, 0.85, 0.0, 0.010, -0.010]), np.array([18.0, 0.0, 0.0])),
            ("lift_with_force", np.array([0.0, 1.15, -1.35, 0.0, 0.65, 0.0, 0.030, -0.030]), np.array([18.0, 0.0, 4.0])),
        ]
        self.q_target = self.q_home.copy()
        self.set_state(self.q_home)

        rospy.Subscriber("/long_arm/target_pose", PoseStamped, self.target_pose_cb, queue_size=1)
        rospy.Subscriber("/long_arm/tool_force", WrenchStamped, self.tool_force_cb, queue_size=1)
        rospy.Subscriber("/long_arm/joint_target", JointState, self.joint_target_cb, queue_size=1)
        rospy.Subscriber("/long_arm/gripper_target", Float64, self.gripper_target_cb, queue_size=1)
        rospy.Subscriber("/long_arm/speed_scale", Float64, self.speed_scale_cb, queue_size=1)

        self.joint_pub = rospy.Publisher("/joint_states", JointState, queue_size=10, tcp_nodelay=True)
        self.ee_pose_pub = rospy.Publisher("/long_arm/end_effector_pose", PoseStamped, queue_size=10)
        self.stability_pub = rospy.Publisher("/long_arm/stability", Float64MultiArray, queue_size=10)
        self.speed_scale_pub = rospy.Publisher("/long_arm/speed_scale_state", Float64, queue_size=10)
        self.camera_pose_pub = rospy.Publisher("/long_arm/hand_camera_pose", PoseStamped, queue_size=10)
        self.camera_image_pub = rospy.Publisher("/long_arm/hand_camera/image_raw", Image, queue_size=2)

        rospy.loginfo("Loaded real Piper MuJoCo model: %s", xml_path)
        rospy.loginfo("Published real URDF to /robot_description: %s", urdf_path)
        rospy.loginfo("Joints: %s, end effector body: %s", list(self.joint_names), self.ee_body_name)
        if self.camera_id is not None:
            rospy.loginfo("MuJoCo camera: %s", self.camera_name)

    def publish_robot_description(self, urdf_path):
        with open(urdf_path, "r") as f:
            rospy.set_param("/robot_description", f.read())

    def resolve_actuators(self):
        actuator_ids = []
        actuator_names = [self.model.actuator(i).name for i in range(self.model.nu)]
        for joint_name in self.joint_names:
            if joint_name not in actuator_names:
                raise RuntimeError("Actuator for joint '{}' not found".format(joint_name))
            actuator_ids.append(int(self.model.actuator(joint_name).id))
        return actuator_ids

    def resolve_camera_id(self):
        try:
            return int(self.model.camera(self.camera_name).id)
        except KeyError:
            rospy.logwarn("Camera '%s' not found in MuJoCo model; camera topics will stay empty.", self.camera_name)
            return None

    def create_camera_renderer(self):
        if not self.publish_camera_image_enabled or self.camera_id is None:
            return None
        try:
            return mujoco.Renderer(self.model, height=self.camera_height, width=self.camera_width)
        except Exception as exc:
            rospy.logwarn("Could not create MuJoCo camera renderer: %s", exc)
            return None

    def joint_qpos_addresses(self):
        return [int(self.model.joint(name).qposadr[0]) for name in self.joint_names]

    def joint_dof_addresses(self):
        return [int(self.model.joint(name).dofadr[0]) for name in self.joint_names]

    def set_state(self, q):
        for addr, value in zip(self.joint_qpos_addresses(), q):
            self.data.qpos[addr] = value
        self.data.qvel[:] = 0.0
        self.data.ctrl[:] = 0.0
        self.data.xfrc_applied[:] = 0.0
        mujoco.mj_forward(self.model, self.data)

    def set_ik_state(self, q):
        for addr, value in zip(self.joint_qpos_addresses(), q):
            self.ik_data.qpos[addr] = value
        self.ik_data.qvel[:] = 0.0
        self.ik_data.qacc[:] = 0.0
        mujoco.mj_forward(self.model, self.ik_data)

    def current_q(self):
        return np.array([self.data.qpos[addr] for addr in self.joint_qpos_addresses()])

    def current_qvel(self):
        return np.array([self.data.qvel[addr] for addr in self.joint_dof_addresses()])

    def body_pose(self, data):
        pos = data.xpos[self.ee_body_id].copy()
        quat = data.xquat[self.ee_body_id].copy()
        return pos, quat

    def body_jacobian(self, data):
        jacp = np.zeros((3, self.model.nv))
        jacr = np.zeros((3, self.model.nv))
        mujoco.mj_jacBody(self.model, data, jacp, jacr, self.ee_body_id)
        cols = self.joint_dof_addresses()
        return np.vstack([jacp[:, cols], jacr[:, cols]])

    def solve_ik(self, target_pos, target_quat_wxyz, q0):
        q = q0.astype(float).copy()
        ranges = np.array([self.model.joint(name).range for name in self.joint_names], dtype=float)
        lower = ranges[:, 0]
        upper = ranges[:, 1]
        weights = np.diag([1.0, 1.0, 1.0, 0.25, 0.25, 0.25])

        for _ in range(180):
            self.set_ik_state(q)
            pos, quat = self.body_pose(self.ik_data)
            err = np.concatenate([target_pos - pos, quat_error_vector(target_quat_wxyz, quat)])
            weighted_err = weights @ err
            if np.linalg.norm(weighted_err) < 1e-4:
                break
            jac = weights @ self.body_jacobian(self.ik_data)
            lhs = jac @ jac.T + (0.035 ** 2) * np.eye(6)
            dq = jac.T @ np.linalg.solve(lhs, weighted_err)
            step_norm = np.linalg.norm(dq)
            if step_norm > 0.10:
                dq *= 0.10 / step_norm
            q = np.clip(q + dq, lower, upper)
        return q

    def target_pose_cb(self, msg):
        target_pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        target_quat = ros_quat_to_wxyz(msg.pose.orientation)
        with self.lock:
            self.demo_enabled = False
            self.q_direct = None
            self.target_pose = (target_pos, target_quat)
            self.q_target = self.solve_ik(target_pos, target_quat, self.current_q())

    def tool_force_cb(self, msg):
        with self.lock:
            self.tool_force = np.array(
                [msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z],
                dtype=float,
            )

    def joint_target_cb(self, msg):
        targets = dict(zip(msg.name, msg.position))
        q = self.current_q()
        updated = False
        for i, name in enumerate(self.joint_names):
            if name in targets:
                q[i] = targets[name]
                updated = True
        if updated:
            with self.lock:
                self.demo_enabled = False
                self.q_direct = q

    def gripper_target_cb(self, msg):
        opening = float(np.clip(msg.data, self.gripper_closed, self.gripper_open))
        q = self.current_q()
        for i, name in enumerate(self.joint_names):
            if name == "joint7":
                q[i] = opening
            elif name == "joint8":
                q[i] = -opening
        with self.lock:
            self.demo_enabled = False
            if self.q_direct is not None:
                self.q_direct = q
            else:
                self.q_target = q

    def speed_scale_cb(self, msg):
        with self.lock:
            self.speed_scale = float(np.clip(msg.data, 0.0, 4.0))

    def apply_tool_reaction(self, tool_force):
        self.data.xfrc_applied[:] = 0.0
        # WrenchStamped is interpreted as tool-on-environment force; MuJoCo body
        # receives the equal and opposite reaction.
        self.data.xfrc_applied[self.ee_body_id, :3] = -tool_force

    def control_step(self, q_target, tool_force):
        for actuator_id, target in zip(self.actuator_ids, q_target):
            self.data.ctrl[actuator_id] = target
        self.apply_tool_reaction(tool_force)

    def demo_target(self):
        idx = min(
            int((self.data.time % (self.hold_time * len(self.demo_sequence))) // self.hold_time),
            len(self.demo_sequence) - 1,
        )
        _, q_target, force = self.demo_sequence[idx]
        return q_target, force

    def stability_values(self, tool_force):
        masses = self.model.body_mass[1:]
        coms = self.data.xipos[1:]
        total_mass = float(np.sum(masses))
        com = np.sum(masses[:, None] * coms, axis=0) / total_mass
        pos, _ = self.body_pose(self.data)
        reaction = -tool_force
        normal_force = total_mass * G + reaction[2]
        cop_x = float("nan")
        if abs(normal_force) > 1e-6:
            moment_y = total_mass * G * com[0] + pos[0] * reaction[2] - pos[2] * reaction[0]
            cop_x = moment_y / normal_force
        support_margin = SUPPORT_HALF_LENGTH - abs(cop_x)
        return np.array([com[0], com[1], com[2], cop_x, support_margin, total_mass])

    def publish(self, tool_force):
        now = rospy.Time.now()
        q = self.current_q()

        joint_msg = JointState()
        joint_msg.header = Header(stamp=now)
        joint_msg.name = list(self.joint_names)
        joint_msg.position = q.tolist()
        joint_msg.velocity = self.current_qvel().tolist()
        joint_msg.effort = self.data.qfrc_actuator[self.joint_dof_addresses()].tolist()
        self.joint_pub.publish(joint_msg)

        pos, quat = self.body_pose(self.data)
        pose_msg = PoseStamped()
        pose_msg.header = Header(stamp=now, frame_id=self.world_frame)
        pose_msg.pose.position.x = pos[0]
        pose_msg.pose.position.y = pos[1]
        pose_msg.pose.position.z = pos[2]
        wxyz_to_ros_quat(quat, pose_msg.pose.orientation)
        self.ee_pose_pub.publish(pose_msg)

        stability_msg = Float64MultiArray()
        stability_msg.data = self.stability_values(tool_force).tolist()
        self.stability_pub.publish(stability_msg)

        speed_msg = Float64()
        speed_msg.data = self.speed_scale
        self.speed_scale_pub.publish(speed_msg)

        self.publish_camera(now)

    def publish_camera(self, now):
        if self.camera_id is None:
            return

        pos = self.data.cam_xpos[self.camera_id].copy()
        rot = self.data.cam_xmat[self.camera_id].reshape(3, 3).copy()
        quat = rotmat_to_wxyz(rot)

        pose_msg = PoseStamped()
        pose_msg.header = Header(stamp=now, frame_id=self.world_frame)
        pose_msg.pose.position.x = pos[0]
        pose_msg.pose.position.y = pos[1]
        pose_msg.pose.position.z = pos[2]
        wxyz_to_ros_quat(quat, pose_msg.pose.orientation)
        self.camera_pose_pub.publish(pose_msg)

        if self.camera_renderer is None:
            return
        if self.camera_rate_hz > 0.0 and (now.to_sec() - self.last_camera_pub) < (1.0 / self.camera_rate_hz):
            return
        self.last_camera_pub = now.to_sec()
        image = self.render_camera_image()
        if image is None:
            return
        image_msg = Image()
        image_msg.header = Header(stamp=now, frame_id=self.camera_name)
        image_msg.height = int(image.shape[0])
        image_msg.width = int(image.shape[1])
        image_msg.encoding = "rgb8"
        image_msg.is_bigendian = 0
        image_msg.step = int(image.shape[1] * 3)
        image_msg.data = image.tobytes()
        self.camera_image_pub.publish(image_msg)

    def render_camera_image(self):
        try:
            self.camera_renderer.update_scene(self.data, camera=self.camera_name)
            return self.camera_renderer.render()
        except Exception as exc:
            rospy.logwarn_throttle(5.0, "Could not render camera image: %s", exc)
            return None

    def step_once(self):
        with self.lock:
            if self.demo_enabled:
                q_target, tool_force = self.demo_target()
            else:
                q_target = self.q_direct if self.q_direct is not None else self.q_target.copy()
                tool_force = self.tool_force.copy()
        self.control_step(q_target, tool_force)
        mujoco.mj_step(self.model, self.data)
        self.publish(tool_force)

    def run_without_viewer(self):
        while not rospy.is_shutdown():
            with self.lock:
                speed_scale = self.speed_scale
            if speed_scale <= 0.0:
                self.publish(self.tool_force.copy())
                rospy.sleep(0.05)
                continue
            step_start = time.time()
            self.step_once()
            sleep_time = (self.model.opt.timestep / speed_scale) - (time.time() - step_start)
            if sleep_time > 0.0:
                rospy.sleep(sleep_time)

    def run_with_viewer(self):
        import mujoco.viewer

        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            viewer.cam.azimuth = 130
            viewer.cam.elevation = -25
            viewer.cam.distance = 1.2
            viewer.cam.lookat[:] = np.array([0.0, 0.0, 0.20])
            while not rospy.is_shutdown() and viewer.is_running():
                with self.lock:
                    speed_scale = self.speed_scale
                if speed_scale <= 0.0:
                    self.publish(self.tool_force.copy())
                    viewer.sync()
                    rospy.sleep(0.05)
                    continue
                step_start = time.time()
                self.step_once()
                viewer.sync()
                sleep_time = (self.model.opt.timestep / speed_scale) - (time.time() - step_start)
                if sleep_time > 0.0:
                    rospy.sleep(sleep_time)

    def run(self):
        if self.viewer_enabled:
            self.run_with_viewer()
        else:
            self.run_without_viewer()


def main():
    PiperMujocoRos().run()


if __name__ == "__main__":
    main()
