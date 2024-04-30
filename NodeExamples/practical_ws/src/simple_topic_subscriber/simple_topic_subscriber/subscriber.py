#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__('simple_subscriber') # Run the parent constructor
        # Set up the subscriber
        self.subscriber = self.create_subscription(String, 'message', self.listener_callback, 10)
        self.subscriber # This prevents a warning

    def listener_callback(self, m):
        # Process the message
        self.get_logger().info(f'Received: {m.data}')


def main(args=None):
    rclpy.init(args=args)

    simple_subscriber = SimpleSubscriber()

    # Spin - to let ROS process the messages
    rclpy.spin(simple_subscriber)

    # Node exit
    simple_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
