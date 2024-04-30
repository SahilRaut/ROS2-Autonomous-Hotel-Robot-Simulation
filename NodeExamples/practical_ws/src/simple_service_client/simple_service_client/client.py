#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rsa_interfaces.srv import Id

class SimpleClient(Node):
    def __init__(self):
        super().__init__('simple_client') # Run the parent constructor
        # Set up the client
        self.cbgroup = ReentrantCallbackGroup()
        self.client = self.create_client(Id, 'gen_id', callback_group = self.cbgroup)
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Service not available, waiting...')
        # Set up a timer
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback, callback_group = self.cbgroup)

    async def timer_callback(self):
        # Request a new id
        self.get_logger().info(f'Requesting new id number')
        call = await self.client.call_async(Id.Request())  # Waits for the result
        # Print a message
        self.id = call.id
        self.get_logger().info(f'Received id: {call.id}')

def main(args=None):
    rclpy.init(args=args)

    simple_client = SimpleClient()

    # Spin - to let ROS process events
    rclpy.spin(simple_client)

    # Node exit
    simple_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
