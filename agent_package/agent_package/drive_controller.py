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
        super().__init__('drive_controller')
        self.avoidance_correction = 0.
        self.drive_publisher = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.get_logger().info('Creating /scan subscriber')
        self.scan_subscription = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.get_logger().info('Creating /ego_racecar/odom subscriber')
        self.odom_subscription = self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10)
        self.collision_avoidance_subscription = self.create_subscription(AckermannDrive, '/collision_avoidance', self.collision_avoidance_callback, 10)

    def odom_callback(self, odom_msg):
        self.speed = odom_msg.twist.twist.linear.x

    def scan_callback(self, scan_msg):
        # self.get_logger().info(f'Max dist {scan_msg.range_max} farthest = {max(scan_msg.ranges)}')
        scan_msg.ranges = [(dist if dist < scan_msg.range_max - 5. else float(0)) for dist in scan_msg.ranges]
        farthest_dist = max(scan_msg.ranges)
        farthest_angle = scan_msg.angle_min + scan_msg.ranges.index(farthest_dist) * scan_msg.angle_increment

        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = farthest_dist / 10
        drive_msg.drive.steering_angle = self.avoidance_correction if self.avoidance_correction != 0 else farthest_angle
        drive_msg.drive.speed = .25
        #self.get_logger().info(f'total={len(scan_msg.ranges)} left_index={((-1 * math.pi / 2) - scan_msg.angle_min) // scan_msg.angle_increment} right_index={-1 * ((scan_msg.angle_max - (math.pi / 2)) // scan_msg.angle_increment)}')
        # left_dist = scan_msg.ranges[int(((-1 * math.pi / 2) - scan_msg.angle_min) // scan_msg.angle_increment)]
        # right_dist = scan_msg.ranges[int(-1 * ((scan_msg.angle_max - (math.pi / 2)) // scan_msg.angle_increment))]
        # angle = (math.atan(right_dist - left_dist))
        # self.get_logger().info(f'{angle=} dist_diff={right_dist - left_dist}')
        drive_msg.drive.steering_angle = self.avoidance_correction if self.avoidance_correction != 0 else farthest_angle
        # self.get_logger().info(f'Setting drive to speed: {drive_msg.drive.speed}, angle: {drive_msg.drive.steering_angle} avoidance = {self.avoidance_correction != 0}')
        self.drive_publisher.publish(drive_msg)

    def collision_avoidance_callback(self, avoidance_msg):
        self.avoidance_correction = avoidance_msg.steering_angle
        self.get_logger().info(f'Setting avoidance angle to {self.avoidance_correction}')


def main(args=None):
    rclpy.init(args=args)

    drive_contoller_service = DriveControllerService()

    rclpy.spin(drive_contoller_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()