import rclpy
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive



class AgentService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.drive_controller_subscription = self.create_subscription(AckermannDriveStamped, '/drive_controller', self.drive_controller_callback, 10)
        self.get_logger().info('Creating /drive_controller subscriber')
        self.drive_publisher = self.create_publisher(AckermannDriveStamped, '/drive', 10)

    def drive_controller_callback(self, drive_msg):
        self.drive_publisher.publish(drive_msg)

def main(args=None):
    rclpy.init(args=args)

    agent_service = AgentService()

    rclpy.spin(agent_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()