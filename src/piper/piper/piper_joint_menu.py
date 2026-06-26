#!/usr/bin/env python3
import math
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class PiperJointMenu(Node):
    def __init__(self):
        super().__init__("piper_joint_menu")
        self.declare_parameter("topic", "/joint_states")
        self.declare_parameter("speed", 10.0)
        self.declare_parameter("gripper", 0.0)
        self.declare_parameter("gripper_effort", 0.5)

        self.topic = self.get_parameter("topic").value
        self.speed = float(self.get_parameter("speed").value)
        self.gripper = float(self.get_parameter("gripper").value)
        self.gripper_effort = float(self.get_parameter("gripper_effort").value)

        self.publisher = self.create_publisher(JointState, self.topic, 10)
        self.positions = [0.0] * 6

    def publish_command(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"]
        msg.position = self.positions + [self.gripper]
        msg.velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, self.speed]
        msg.effort = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, self.gripper_effort]
        self.publisher.publish(msg)

    def show_state(self):
        deg_values = [math.degrees(value) for value in self.positions]
        print("\nCurrent target:")
        for index, value in enumerate(deg_values, start=1):
            print(f"  joint{index}: {value:.2f} deg")
        print(f"  gripper: {self.gripper:.4f} m")
        print(f"  speed: {self.speed:.1f} %")
        print(f"  topic: {self.topic}")

    def set_single_joint(self):
        joint_text = input("Joint number 1-6: ").strip()
        if not joint_text.isdigit():
            print("Invalid joint number.")
            return

        joint_index = int(joint_text)
        if joint_index < 1 or joint_index > 6:
            print("Joint number must be 1-6.")
            return

        angle_text = input(f"Target angle for joint{joint_index} in degrees: ").strip()
        try:
            angle_deg = float(angle_text)
        except ValueError:
            print("Invalid angle.")
            return

        self.positions[joint_index - 1] = math.radians(angle_deg)
        self.publish_command()
        print("Published joint command.")

    def set_all_joints(self):
        text = input("Enter 6 joint angles in degrees, separated by spaces: ").strip()
        parts = text.split()
        if len(parts) != 6:
            print("Please enter exactly 6 values.")
            return

        try:
            values_deg = [float(part) for part in parts]
        except ValueError:
            print("Invalid angle list.")
            return

        self.positions = [math.radians(value) for value in values_deg]
        self.publish_command()
        print("Published joint command.")

    def set_gripper(self):
        text = input("Gripper opening in meters, usually 0.0-0.04 in RViz scale: ").strip()
        try:
            self.gripper = float(text)
        except ValueError:
            print("Invalid gripper value.")
            return

        self.publish_command()
        print("Published gripper command.")

    def set_speed(self):
        text = input("Speed percent 1-100: ").strip()
        try:
            speed = float(text)
        except ValueError:
            print("Invalid speed.")
            return

        self.speed = min(100.0, max(1.0, speed))
        print(f"Speed set to {self.speed:.1f} %.")

    def publish_current(self):
        self.publish_command()
        print("Republished current target.")

    def run_menu(self):
        print("Piper joint menu")
        print("Make sure the arm is enabled and the workspace is clear.")

        while rclpy.ok():
            self.show_state()
            print("\nMenu:")
            print("  1) Set one joint")
            print("  2) Set all 6 joints")
            print("  3) Set gripper")
            print("  4) Set speed")
            print("  5) Publish current target")
            print("  q) Quit")

            choice = input("> ").strip().lower()
            if choice == "1":
                self.set_single_joint()
            elif choice == "2":
                self.set_all_joints()
            elif choice == "3":
                self.set_gripper()
            elif choice == "4":
                self.set_speed()
            elif choice == "5":
                self.publish_current()
            elif choice in ("q", "quit", "exit"):
                break
            else:
                print("Unknown option.")


def main(args=None):
    rclpy.init(args=args)
    node = PiperJointMenu()
    try:
        node.run_menu()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
