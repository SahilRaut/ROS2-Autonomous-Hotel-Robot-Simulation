#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher') # Run the parent constructor
        # Set up the publisher
        self.publisher = self.create_publisher(String, 'message', 10)
        # Set up a timer
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        # Initialize internal counter
        self.count = 0

    def timer_callback(self):
        # Generate a message
        msg = String()
        msg.data = f'Message Number: {self.count}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Sent: {msg.data}')
        self.count += 1


def main(args=None):
    rclpy.init(args=args)

    simple_publisher = SimplePublisher()

    # Spin - to let ROS process the messages
    rclpy.spin(simple_publisher)

    # Node exit
    simple_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
