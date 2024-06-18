#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from custom_interfaces.srv import Cnt
import subprocess

class CounterServer(Node):
    def __init__(self):
        super().__init__('counter_server')
        self.counter = 0
        self.srv = self.create_service(Cnt, 'increment_counter', self.increment_counter_callback)
        self.get_logger().info('Counter Server Node has been started.')

    def increment_counter_callback(self, request, response):
        self.counter += 1  # Increment the counter
        self.get_logger().info(f'Counter incremented to {self.counter}')
        response.new_value = self.counter  # Set the response new_value to the new counter value

        # If counter reaches 3, execute the command
        if self.counter == 3:
            self.execute_command()

        return response

    def execute_command(self):
        command = "gnome-terminal -- bash -c 'source /opt/ros/humble/setup.bash; ros2 run ros2_execution send_goal.py; exec bash'"
        subprocess.Popen(command, shell=True)
        self.get_logger().info("Executed command: ros2 launch ros2_execution send_goal.py")

def main(args=None):
    rclpy.init(args=args)
    node = CounterServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
