#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    BatteryStatus
)
import time
import subprocess
from pynput import keyboard
import re
import math


class OffboardKeyboardAI(Node):

    def __init__(self):
        super().__init__('offboard_keyboard_ai')

        # ---------------- QoS Profile for PX4 ----------------
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # ---------------- Publishers ----------------
        self.offboard_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.traj_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.cmd_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', 10)

        # ---------------- Subscribers ----------------
        self.pos_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.pos_cb,
            qos_profile
        )
        
        self.battery_sub = self.create_subscription(
            BatteryStatus,
            '/fmu/out/battery_status_v1',
            self.battery_cb,
            qos_profile
        )

        # ---------------- State ----------------
        self.current_pos = [0.0, 0.0, -2.0]
        self.target_pos = self.current_pos.copy()
        
        # Battery state
        self.battery_percentage = 100.0
        self.battery_voltage = 0.0
        self.battery_current = 0.0

        self.last_key_time = time.time()
        self.keyboard_timeout = 10.0

        self.ai_interval = 60.0
        self.last_ai_time = 0.0

        self.step = 0.5
        self.counter = 0
        self.in_ai_mode = False

        # ---------------- Priority Waypoints ----------------
        self.priority_points = [
            [150.0, 0.0, -2.0],
            [50.0, 150.0, -2.0],
            [0.0, -150.0, -2.0],
            [-150.0, 0.0, -2.0],
        ]

        self.current_wp_index = None

        # ---------------- Circle (Loiter) Params ----------------
        self.circle_radius = 20.0      # meters
        self.circle_omega = 0.5       # rad/s
        self.circle_angle = 0.0
        self.circle_center = None

        # ---------------- Keyboard ----------------
        listener = keyboard.Listener(on_press=self.on_key_press)
        listener.daemon = True
        listener.start()

        # ---------------- Timer ----------------
        self.timer_period = 0.05  # 20 Hz
        self.timer = self.create_timer(self.timer_period, self.loop)

        self.get_logger().info("OFFBOARD + KEYBOARD + AI (OLLAMA) + BATTERY STARTED")

    # --------------------------------------------------

    def battery_cb(self, msg):
        """Callback for battery status updates"""
        self.battery_percentage = msg.remaining * 100.0
        self.battery_voltage = msg.voltage_v
        self.battery_current = msg.current_a
        
        # Warn on low battery
        if self.battery_percentage < 20.0 and self.counter % 100 == 0:
            self.get_logger().warn(f"⚠️  LOW BATTERY: {self.battery_percentage:.1f}%")
        elif self.battery_percentage < 10.0:
            self.get_logger().error(f"🚨 CRITICAL BATTERY: {self.battery_percentage:.1f}%")

    # --------------------------------------------------

    def pos_cb(self, msg):
        self.current_pos = [msg.x, msg.y, msg.z]

    # --------------------------------------------------

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
            elif key.char == 'b':
                self.get_logger().info(
                    f"🔋 BATTERY STATUS: {self.battery_percentage:.1f}% | "
                    f"Voltage: {self.battery_voltage:.2f}V | "
                    f"Current: {self.battery_current:.2f}A"
                )
                return

            self.last_key_time = time.time()
            self.in_ai_mode = False
            self.circle_center = None

            self.get_logger().info(f"KEYBOARD OVERRIDE → {self.target_pos}")

        except AttributeError:
            pass

    # --------------------------------------------------

    def query_ai_waypoint(self):
        prompt = f"""
Return ONLY ONE DIGIT: 0, 1, 2, or 3. No words, no explanation.
Current position: {self.current_pos}
Battery: {self.battery_percentage:.1f}%
Waypoints:
0: {self.priority_points[0]}
1: {self.priority_points[1]}
2: {self.priority_points[2]}
3: {self.priority_points[3]}
Last waypoint index: {self.current_wp_index}
"""

        try:
            result = subprocess.run(
                ["ollama", "run", "llama3"],
                input=prompt,
                text=True,
                capture_output=True,
                timeout=30
            )

            response = result.stdout.strip()
            match = re.search(r'\b[0-3]\b', response)
            if not match:
                raise ValueError(f"Invalid AI output: {response}")

            index = int(match.group())
            if index == self.current_wp_index:
                index = (index + 1) % 4

            self.current_wp_index = index
            self.circle_center = self.priority_points[index].copy()
            self.circle_angle = 0.0

            self.get_logger().info(
                f"AI SELECTED WAYPOINT {index} → CIRCLE CENTER {self.circle_center}"
            )

        except Exception as e:
            self.get_logger().warn(f"AI FAILED → HOLDING | {e}")

    # --------------------------------------------------

    def ai_control(self):
        now = time.time()
        if now - self.last_ai_time >= self.ai_interval:
            self.query_ai_waypoint()
            self.last_ai_time = now

    # --------------------------------------------------

    def update_circle_target(self):
        if self.circle_center is None:
            return

        self.circle_angle += self.circle_omega * self.timer_period

        cx, cy, cz = self.circle_center
        self.target_pos[0] = cx + self.circle_radius * math.cos(self.circle_angle)
        self.target_pos[1] = cy + self.circle_radius * math.sin(self.circle_angle)
        self.target_pos[2] = cz

    # --------------------------------------------------

    def loop(self):
        timestamp = int(self.get_clock().now().nanoseconds / 1000)
        now = time.time()

        # ---------------- Offboard heartbeat ----------------
        offboard = OffboardControlMode()
        offboard.timestamp = timestamp
        offboard.position = True
        self.offboard_pub.publish(offboard)

        # ---------------- Mode logic ----------------
        if now - self.last_key_time > self.keyboard_timeout:
            if not self.in_ai_mode:
                self.get_logger().info("ENTERED AI PRIORITY MODE")
                self.last_ai_time = 0.0
                self.in_ai_mode = True

            self.ai_control()
            self.update_circle_target()
            mode = "AI"

        else:
            mode = "KEYBOARD"

        # ---------------- Send trajectory ----------------
        sp = TrajectorySetpoint()
        sp.timestamp = timestamp
        sp.position = self.target_pos
        sp.yaw = self.circle_angle + math.pi / 2 if self.in_ai_mode else 0.0
        self.traj_pub.publish(sp)

        # ---------------- Arm & Offboard ----------------
        if self.counter == 100:
            self.send_cmd(176, 1.0, 6.0)
            self.get_logger().info("OFFBOARD ENABLED")

        if self.counter == 120:
            self.send_cmd(400, 1.0)
            self.get_logger().info("ARMED")

        if self.counter % 50 == 0:
            self.get_logger().info(
                f"MODE: {mode} | TARGET: {self.target_pos} | "
                f"BATTERY: {self.battery_percentage:.1f}% ({self.battery_voltage:.1f}V)"
            )

        self.counter += 1

    # --------------------------------------------------

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
    node = OffboardKeyboardAI()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
