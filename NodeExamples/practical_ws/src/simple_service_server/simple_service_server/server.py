#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rsa_interfaces.srv import Id

class SimpleServer(Node):
    def __init__(self):
        super().__init__('simple_server') # Run the parent constructor
        # Set up the service server
        self.server = self.create_service(Id, 'gen_id', self.gen_id_callback)
        self.id = 1000

    def gen_id_callback(self, req, resp):
        self.get_logger().info(f'Id number requested, sending id: {self.id}')
        resp.id = self.id
        self.id += 1   # Update for the next id
        return resp


def main(args=None):
    rclpy.init(args=args)

    simple_server = SimpleServer()

    # Spin - to let ROS process the messages
    rclpy.spin(simple_server)

    # Node exit
    simple_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
