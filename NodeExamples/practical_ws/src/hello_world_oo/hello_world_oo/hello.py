#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class Hello(Node):
    def __init__(self):
        super().__init__('hello_node') # Run the parent constructor
        print('Hello World! - Object oriented') # Print the message


def main(args=None):
    rclpy.init(args=args)

    hello = Hello()

    # Node exit
    hello.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
