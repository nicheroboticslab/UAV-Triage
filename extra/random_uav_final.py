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


class OffboardBasicMove(Node):

    def __init__(self):
        super().__init__('offboard_basic_move')

        # Publishers
        self.offboard_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.traj_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.cmd_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', 10)

        # Subscriber
        self.pos_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.pos_cb,
            10
        )

        self.current_pos = [0.0, 0.0, 0.0]

        # Simple waypoint list
        self.waypoints = [
            [100.0, 0.0, -2.0],
            [0.0, 110.0, -2.0],
            [223.0, 46.0, -2.0],
            [110.0, 32.0, -2.0],
        ]
        self.wp_index = 0

        self.counter = 0
        self.timer = self.create_timer(0.05, self.loop)

        self.get_logger().info("OFFBOARD BASIC MOVE STARTED")

    # -------------------------------------------------

    def pos_cb(self, msg):
        self.current_pos = [msg.x, msg.y, msg.z]

    # -------------------------------------------------

    def loop(self):
        timestamp = int(self.get_clock().now().nanoseconds / 1000)

        # 1. OFFBOARD HEARTBEAT (ALWAYS)
        offboard = OffboardControlMode()
        offboard.timestamp = timestamp
        offboard.position = True
        self.offboard_pub.publish(offboard)

        # 2. ALWAYS SEND A SETPOINT
        sp = TrajectorySetpoint()
        sp.timestamp = timestamp
        sp.position = self.waypoints[self.wp_index]
        sp.yaw = 0.0
        self.traj_pub.publish(sp)

        # 3. ARM & OFFBOARD AFTER STREAMING SETPOINTS
        if self.counter == 100:
            self.send_cmd(176, 1.0, 6.0)  # OFFBOARD
            self.get_logger().info("OFFBOARD ENABLED")

        if self.counter == 120:
            self.send_cmd(400, 1.0)  # ARM
            self.get_logger().info("ARMED")

        # 4. CHANGE WAYPOINT EVERY 5 SECONDS
        if self.counter > 150 and self.counter % 100 == 0:
            self.wp_index = (self.wp_index + 1) % len(self.waypoints)
            self.get_logger().info(
                f"Switching to waypoint {self.wp_index}: {self.waypoints[self.wp_index]}"
            )

        self.counter += 1

    # -------------------------------------------------

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
    node = OffboardBasicMove()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

