#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from conveyorbelt_msgs.srv import ConveyorBeltControl

class ConveyorServiceServerNode(Node):

    def __init__(self):
        super().__init__('conveyor_service_server_node')
        self.srv = self.create_service(ConveyorBeltControl, 'CONVEYORPOWER', self.handle_conveyor_control)

    def handle_conveyor_control(self, request, response):
        self.get_logger().info(f'Received request: power={request.power}')
        response.success = True
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ConveyorServiceServerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
