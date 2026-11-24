#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import math
import time


class RectangleFlight(Node):

    def __init__(self):
        super().__init__('rectangle_flight')

        # QoS for PX4
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers
        self.offboard_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.traj_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)
        self.cmd_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos)

        # Subscriber
        self.local_pos_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            self.position_callback,
            qos
        )

        self.current_position = None

        # Timer at 10 Hz
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.counter = 0
        self.phase = 0  # 0: takeoff, 1+: rectangle points

        # Rectangle points (x, y, z)
        self.waypoints = [
            (0.0, 0.0, -3.0),
            (5.0, 0.0, -3.0),
            (5.0, 8.0, -3.0),
            (0.0, 8.0, -3.0),
            (0.0, 0.0, -3.0)
        ]

        self.current_wp = 0

    def position_callback(self, msg):
        self.current_position = msg
        self.get_logger().info(f"Pos: x={msg.x:.2f}, y={msg.y:.2f}, z={msg.z:.2f}")

    def publish_offboard_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_pub.publish(msg)

    def publish_trajectory(self, x, y, z):
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = 0.0
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.traj_pub.publish(msg)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.cmd_pub.publish(msg)

    def arm(self):
        self.get_logger().info("Arming...")
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

    def set_offboard_mode(self):
        self.get_logger().info("Switching to OFFBOARD mode...")
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)

    def reached(self, target):
        if self.current_position is None:
            return False

        dx = self.current_position.x - target[0]
        dy = self.current_position.y - target[1]
        dz = self.current_position.z - target[2]

        return math.sqrt(dx*dx + dy*dy + dz*dz) < 1.0

    def timer_callback(self):

        self.publish_offboard_mode()

        # Send initial setpoints before switching mode
        if self.counter < 20:
            self.publish_trajectory(0.0, 0.0, -3.0)
            self.counter += 1
            return

        if self.counter == 20:
            self.set_offboard_mode()
            self.arm()
            self.counter += 1
            return

        # Waypoint navigation
        if self.current_wp < len(self.waypoints):
            target = self.waypoints[self.current_wp]
            self.publish_trajectory(*target)
            

            if self.reached(target):
                if self.wp_reached_time is None:
                    self.wp_reached_time = time.time()

                if time.time() - self.wp_reached_time > 2.0:
                    self.get_logger().info(f"Reached waypoint {self.current_wp}")
                    self.current_wp += 1
                    self.wp_reached_time = None
            else:
                self.wp_reached_time = None

        else:
            self.get_logger().info("Mission complete - holding position")
            self.publish_trajectory(0.0, 0.0, -3.0)


def main(args=None):
    rclpy.init(args=args)
    node = RectangleFlight()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()