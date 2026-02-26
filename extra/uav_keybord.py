#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition
)

import random
import time
from pynput import keyboard


class OffboardKeyboardControl(Node):

    def __init__(self):
        super().__init__('offboard_keyboard_control')

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
        self.keyboard_timeout = 2.0  # seconds
        self.step = 0.5  # meters per key press

        self.counter = 0

        # ---------------- Keyboard Listener ----------------
        listener = keyboard.Listener(on_press=self.on_key_press)
        listener.daemon = True
        listener.start()

        # ---------------- Timer ----------------
        self.timer = self.create_timer(0.05, self.loop)

        self.get_logger().info("OFFBOARD KEYBOARD CONTROL STARTED")

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
            self.get_logger().info(f"Keyboard control → {self.target_pos}")

        except AttributeError:
            pass

    # -----------------------------------------------------

    def random_control(self):
        self.target_pos[0] += random.uniform(-0.2, 0.2)
        self.target_pos[1] += random.uniform(-0.2, 0.2)
        self.target_pos[2] = -2.0  # keep altitude fixed

    # -----------------------------------------------------

    def loop(self):
        timestamp = int(self.get_clock().now().nanoseconds / 1000)

        # 1. OFFBOARD HEARTBEAT
        offboard = OffboardControlMode()
        offboard.timestamp = timestamp
        offboard.position = True
        self.offboard_pub.publish(offboard)

        # 2. CONTROL MODE DECISION
        if time.time() - self.last_key_time > self.keyboard_timeout:
            self.random_control()
            mode = "RANDOM"
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
            self.get_logger().info(f"MODE: {mode} | Target: {self.target_pos}")

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
    node = OffboardKeyboardControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
