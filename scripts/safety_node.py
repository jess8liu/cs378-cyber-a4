#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
import math
# TODO: include needed ROS msg type headers and libraries
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive


class SafetyNode(Node):
    """
    The class that handles emergency braking.
    """
    def __init__(self):
        super().__init__('safety_node')
        """
        One publisher should publish to the /drive topic with a AckermannDriveStamped drive message.

        You should also subscribe to the /scan topic to get the LaserScan messages and
        the /ego_racecar/odom topic to get the current speed of the vehicle.

        The subscribers should use the provided odom_callback and scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        """
        self.speed = 0.

        # TODO: create ROS subscribers and publishers.
        self.drive_publisher = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.get_logger().info('Creating /scan subscriber')
        self.scan_subscription = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.get_logger().info('Creating /ego_racecar/odom subscriber')
        self.odom_subscription = self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10)

    def odom_callback(self, odom_msg):
        # [DEBUG] print odom_msg values
        # self.get_logger().info('Position: "%s"' % odom_msg.pose.pose.position)
        # self.get_logger().info('Orientation: "%s"' % odom_msg.pose.pose.orientation)

        # TODO: update current speed
        self.speed = odom_msg.twist.twist.linear.x

    def scan_callback(self, scan_msg):
        # [DEBUG] print scan_msg values
        #self.get_logger().info('Ranges: "%s"' % scan_msg.ranges)

        # TODO: calculate TTC and determinte if brake is needed
        min_ttc = float('inf')
        min_info = {}
        for i, dist in enumerate(scan_msg.ranges):
            range_rate = max(self.speed * math.cos(scan_msg.angle_min + i * scan_msg.angle_increment), 0)
            if range_rate > 0 and dist / range_rate < min_ttc:
                min_ttc = dist / range_rate
                min_info['angle'] = scan_msg.angle_min + i * scan_msg.angle_increment
                min_info['x_velocity'] = self.speed
                min_info['range_rate'] = range_rate
                min_info['distance'] = dist
                min_info['min_ttc'] = min_ttc
        if min_ttc < 1.5:
            brake = True
            self.get_logger().info(f'braking, min ITTC: {min_ttc} {min_info}')
        else:
            brake = False
        
        # publish command to brake
        # find more details about AckermannDrive.msg here: https://docs.ros.org/en/jade/api/ackermann_msgs/html/msg/AckermannDrive.html
        if brake:
            msg = AckermannDriveStamped()
            msg.drive.speed = 0.
            msg.drive.steering_angle = 0.
            self.drive_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    safety_node = SafetyNode()
    rclpy.spin(safety_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    safety_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
