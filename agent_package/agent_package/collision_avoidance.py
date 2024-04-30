#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
import math
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

class CollisionAvoidanceService(Node):

    def __init__(self):
        super().__init__('collision_avoidance')
        self.speed = 0.
        self.correcting = True
        self.collision_avoidance_publisher = self.create_publisher(AckermannDrive, '/collision_avoidance', 10)
        self.get_logger().info('Creating /scan subscriber')
        self.scan_subscription = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.get_logger().info('Creating /ego_racecar/odom subscriber')
        self.odom_subscription = self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10)

    def odom_callback(self, odom_msg):
        self.speed = odom_msg.twist.twist.linear.x

    def scan_callback(self, scan_msg):
        closest_dist = min(scan_msg.ranges)
        if closest_dist < 2 and not self.correcting:
            closest_angle = scan_msg.angle_min + scan_msg.ranges.index(closest_dist) * scan_msg.angle_increment
            self.correcting = True
            msg = AckermannDrive()
            msg.steering_angle = (-1 if closest_angle > 0 else 1) * math.pi / 2
            self.collision_avoidance_publisher.publish(msg)
        elif self.correcting:
            self.correcting = False
            msg = AckermannDrive()
            msg.steering_angle = 0.
            self.collision_avoidance_publisher.publish(msg)

    # def scan_callback(self, scan_msg):
    #     min_ttc = float('inf')
    #     min_info = {}
    #     for i, dist in enumerate(scan_msg.ranges):
    #         range_rate = max(self.speed * math.cos(scan_msg.angle_min + i * scan_msg.angle_increment), 0)
    #         if range_rate > 0 and dist / range_rate < min_ttc:
    #             min_ttc = dist / range_rate
    #             min_info['angle'] = scan_msg.angle_min + i * scan_msg.angle_increment
    #             min_info['x_velocity'] = self.speed
    #             min_info['range_rate'] = range_rate
    #             min_info['distance'] = dist
    #             min_info['min_ttc'] = min_ttc
    #     if (min_ttc < 1.5 or min(scan_msg.ranges) < 1) and not self.correcting:
    #         self.correcting = True
    #         msg = AckermannDrive()
    #         msg.steering_angle = (-1 if min_info['angle'] > 0 else 1) * math.pi / 2
    #         self.collision_avoidance_publisher.publish(msg)
    #     elif self.correcting:
    #         self.correcting = False
    #         msg = AckermannDrive()
    #         msg.steering_angle = 0.
    #         self.collision_avoidance_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    collision_avoidance_service = CollisionAvoidanceService()

    rclpy.spin(collision_avoidance_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()