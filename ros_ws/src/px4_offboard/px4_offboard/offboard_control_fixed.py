#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand
)

class OffboardFly(Node):

    def __init__(self):
        super().__init__('offboard_fly')

        # Publishers
        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            10
        )

        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            10
        )

        self.command_pub = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            10
        )

        self.counter = 0

        # Timer: 20 Hz
        self.timer = self.create_timer(0.05, self.timer_callback)

        self.get_logger().info('Offboard node started')

    def timer_callback(self):
        timestamp = int(self.get_clock().now().nanoseconds / 1000)

        # 1. Publish Offboard Control Mode
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = timestamp
        offboard_msg.position = True
        offboard_msg.velocity = False
        offboard_msg.acceleration = False
        offboard_msg.attitude = False
        offboard_msg.body_rate = False
        self.offboard_mode_pub.publish(offboard_msg)

        # 2. Publish Trajectory Setpoint (Takeoff & hover)
        traj = TrajectorySetpoint()
        traj.timestamp = timestamp
        traj.x = 0.0
        traj.y = 0.0
        traj.z = -2.0   # NED frame → -2 = 2 meters UP
        traj.yaw = 0.0
        self.trajectory_pub.publish(traj)

        # 3. After some setpoints → switch to OFFBOARD + ARM
        if self.counter == 20:
            self.publish_vehicle_command(176, 1.0)  # OFFBOARD mode
            self.publish_vehicle_command(400, 1.0)  # ARM
            self.get_logger().info('Sent OFFBOARD & ARM commands')

        self.counter += 1

    def publish_vehicle_command(self, command, param1):
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.param1 = param1
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.command_pub.publish(msg)


def main():
    rclpy.init()
    node = OffboardFly()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
