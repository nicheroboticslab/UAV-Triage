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
        # Timer: 20 Hz (0.05s interval)
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.get_logger().info('Offboard node started')

    def timer_callback(self):
        # Current microsecond timestamp
        timestamp = int(self.get_clock().now().nanoseconds / 1000)

        # 1. ALWAYS Publish Offboard Control Mode (Required for heartbeats)
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = timestamp
        offboard_msg.position = True
        offboard_msg.velocity = False
        offboard_msg.acceleration = False
        offboard_msg.attitude = False
        offboard_msg.body_rate = False
        self.offboard_mode_pub.publish(offboard_msg)

        # 2. ALWAYS Publish Trajectory Setpoint (Hover at 2m)
        traj = TrajectorySetpoint()
        traj.timestamp = timestamp
        # position array: [X (North), Y (East), Z (Down)]
        traj.position = [1.0, 2.0, -2.0]
        traj.yaw = 0.0
        # Explicitly set other fields to NaN so PX4 ignores them
        traj.velocity = [float('nan')] * 3
        traj.acceleration = [float('nan')] * 3
        self.trajectory_pub.publish(traj)

        # 3. Handle Arming and Mode Switching Sequence
        # Switch to OFFBOARD mode after 10 heartbeats (~0.5s)
        if self.counter == 10:
            # 176 = VEHICLE_CMD_DO_SET_MODE, param1=1 (Custom), param2=6 (Offboard)
            self.publish_vehicle_command(176, 1.0, 6.0)
            self.get_logger().info('Requested OFFBOARD mode')

        # ARM the vehicle after 20 heartbeats (~1.0s)
        elif self.counter == 20:
            # 400 = VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0 (Arm)
            self.publish_vehicle_command(400, 1.0)
            self.get_logger().info('Sent ARM command')

        self.counter += 1

    def publish_vehicle_command(self, command, param1, param2=0.0):
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.confirmation = 0
        self.command_pub.publish(msg)

def main():
    rclpy.init()
    node = OffboardFly()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
