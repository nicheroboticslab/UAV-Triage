#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition
)

import time
import random
from pynput import keyboard


class OffboardKeyboardPriority(Node):

    def __init__(self):
        super().__init__('offboard_keyboard_priority')

        # ---------------- Publishers ----------------
        self.offboard_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.traj_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.cmd_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', 10)

        # ---------------- Subscriber ----------------
        self.pos_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.pos_cb,
            10
        )

        # ---------------- State ----------------
        self.current_pos = [0.0, 0.0, -2.0]
        self.target_pos = self.current_pos.copy()

        self.last_key_time = time.time()
        self.keyboard_timeout = 10.0  # seconds
        self.step = 0.5

        # ---------------- Priority Waypoints ----------------
        self.priority_points = [
            [50.0, 0.0, -2.0],
            [50.0, 50.0, -2.0],
            [0.0, 50.0, -2.0],
            [0.0, 0.0, -2.0],
        ]

        self.current_priority_wp = None
        self.priority_switch_interval = 10.0
        self.last_priority_switch_time = 0.0
        self.in_priority_mode = False

        self.counter = 0

        # ---------------- Keyboard Listener ----------------
        listener = keyboard.Listener(on_press=self.on_key_press)
        listener.daemon = True
        listener.start()

        # ---------------- Timer ----------------
        self.timer = self.create_timer(0.05, self.loop)

        self.get_logger().info("OFFBOARD KEYBOARD + RANDOM PRIORITY MODE STARTED")

    # -----------------------------------------------------

    def pos_cb(self, msg):
        self.current_pos = [msg.x, msg.y, msg.z]

    # -----------------------------------------------------

    def on_key_press(self, key):
        try:
            if key == keyboard.Key.up:
                self.target_pos[0] += self.step
            elif key == keyboard.Key.down:
                self.target_pos[0] -= self.step
            elif key == keyboard.Key.left:
                self.target_pos[1] += self.step
            elif key == keyboard.Key.right:
                self.target_pos[1] -= self.step
            elif key.char == 'w':
                self.target_pos[2] -= self.step
            elif key.char == 's':
                self.target_pos[2] += self.step

            self.last_key_time = time.time()
            self.in_priority_mode = False  # immediate override
            self.get_logger().info(f"KEYBOARD OVERRIDE → {self.target_pos}")

        except AttributeError:
            pass

    # -----------------------------------------------------

    def choose_random_priority_wp(self):
        candidates = self.priority_points.copy()

        # Avoid choosing same waypoint consecutively
        if self.current_priority_wp in candidates:
            candidates.remove(self.current_priority_wp)

        self.current_priority_wp = random.choice(candidates)
        self.target_pos = self.current_priority_wp.copy()

        self.get_logger().info(
            f"RANDOM PRIORITY TARGET → {self.current_priority_wp}"
        )

    # -----------------------------------------------------

    def priority_control(self):
        now = time.time()

        # Switch waypoint every fixed interval
        if now - self.last_priority_switch_time >= self.priority_switch_interval:
            self.choose_random_priority_wp()
            self.last_priority_switch_time = now

    # -----------------------------------------------------

    def loop(self):
        timestamp = int(self.get_clock().now().nanoseconds / 1000)
        now = time.time()

        # 1. OFFBOARD HEARTBEAT
        offboard = OffboardControlMode()
        offboard.timestamp = timestamp
        offboard.position = True
        self.offboard_pub.publish(offboard)

        # 2. CONTROL ARBITRATION
        if now - self.last_key_time > self.keyboard_timeout:
            if not self.in_priority_mode:
                self.in_priority_mode = True
                self.last_priority_switch_time = 0.0  # force immediate switch
                self.get_logger().info("ENTERED RANDOM PRIORITY MODE")

            self.priority_control()
            mode = "PRIORITY-RANDOM"

        else:
            mode = "KEYBOARD"

        # 3. SEND SETPOINT
        sp = TrajectorySetpoint()
        sp.timestamp = timestamp
        sp.position = self.target_pos
        sp.yaw = 0.0
        self.traj_pub.publish(sp)

        # 4. ARM & OFFBOARD
        if self.counter == 100:
            self.send_cmd(176, 1.0, 6.0)
            self.get_logger().info("OFFBOARD ENABLED")

        if self.counter == 120:
            self.send_cmd(400, 1.0)
            self.get_logger().info("ARMED")

        if self.counter % 50 == 0:
            self.get_logger().info(
                f"MODE: {mode} | Target: {self.target_pos}"
            )

        self.counter += 1

    # -----------------------------------------------------

    def send_cmd(self, command, p1, p2=0.0):
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.command = command
        msg.param1 = p1
        msg.param2 = p2
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.cmd_pub.publish(msg)


def main():
    rclpy.init()
    node = OffboardKeyboardPriority()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

