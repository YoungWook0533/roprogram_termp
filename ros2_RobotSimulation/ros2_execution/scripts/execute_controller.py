#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from custom_interfaces.srv import Cnt
import subprocess

class ExecuteProgram(Node):
    def __init__(self):
        super().__init__('execute_program_node')
        self.subscription = self.create_subscription(
            String,
            'detected_object',
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning
        self.counter_client = self.create_client(Cnt, 'increment_counter')
        while not self.counter_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.get_logger().info("Node has been started. Waiting for object messages...")

    def execute_command(self, program_name):
        command = [
            "ros2", "run", "ros2_execution", "ros2_execution.py",
            "--ros-args", "-p", f"PROGRAM_FILENAME:={program_name}",
            "-p", "ROBOT_MODEL:=irb120",
            "-p", "EE_MODEL:=schunk"
        ]
        subprocess.run(command)
        self.get_logger().info(f"Executed command for {program_name}")
        self.send_counter_request()

    def listener_callback(self, msg):
        object_name = msg.data
        if object_name == 'cracker box':
            self.get_logger().info('Received cracker box')
            self.execute_command("toPos1")
            self.send_counter_request()
        elif object_name == 'sugar box':
            self.get_logger().info('Received sugar box')
            self.execute_command("toPos2")
            self.send_counter_request()
        elif object_name == 'spam can':
            self.get_logger().info('Received spam can')
            self.execute_command("toPos3")
            self.send_counter_request()
        else:
            self.get_logger().info(f"Received unknown object: {object_name}")

    def send_counter_request(self):
        req = Cnt.Request()
        self.future = self.counter_client.call_async(req)
        self.future.add_done_callback(self.counter_response_callback)

    def counter_response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Counter incremented, new value: {response.new_value}")
            print(f"Counter incremented, new value: {response.new_value}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {str(e)}")
            print(f"Service call failed: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = ExecuteProgram()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()