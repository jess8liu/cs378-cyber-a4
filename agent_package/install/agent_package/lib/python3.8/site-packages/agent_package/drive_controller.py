#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
import math
# TODO: include needed ROS msg type headers and libraries
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive


class DriveControllerService(Node):

    def __init__(self):
        self.drive_publisher = self.create_publisher(AckermannDriveStamped, '/drive_controller', 10)
        self.get_logger().info('Creating /scan subscriber')
        self.scan_subscription = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.get_logger().info('Creating /ego_racecar/odom subscriber')
        self.odom_subscription = self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10)

    def odom_callback(self, odom_msg):
        self.speed = odom_msg.twist.twist.linear.x

    def scan_callback(self, scan_msg):
        farthest_dist = max(scan_msg.ranges)
        farthest_angle = scan_msg.ranges.indexOf(farthest_dist)

        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = farthest_dist / 10
        drive_msg.drive.steering_angle = farthest_angle
        self.get_logger().info(f'Setting drive to speed: {drive_msg.drive.speed}, angle: {drive_msg.drive.steering_angle}')
        self.drive_publisher.publish(drive_msg)


def main(args=None):
    rclpy.init(args=args)

    drive_contoller_service = DriveControllerService()

    rclpy.spin(drive_contoller_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()