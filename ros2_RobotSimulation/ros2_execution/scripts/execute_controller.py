#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import subprocess

class ExecuteProgram(Node):
    def __init__(self):
        super().__init__('execute_program_node')
        self.get_logger().info("Node has been started. Please enter a number (1, 2, or 3) in the terminal.")
        self.run()

    def run(self):
        while True:
            try:
                user_input = int(input("Enter a number (1, 2, or 3): "))
                if user_input == 1:
                    self.execute_command("toPos1")
                elif user_input == 2:
                    self.execute_command("toPos2")
                elif user_input == 3:
                    self.execute_command("toPos3")
                else:
                    print("Invalid input. Please enter 1, 2, or 3.")
            except ValueError:
                print("Invalid input. Please enter a number.")

    def execute_command(self, program_name):
        command = [
            "ros2", "run", "ros2_execution", "ros2_execution.py",
            "--ros-args", "-p", f"PROGRAM_FILENAME:={program_name}",
            "-p", "ROBOT_MODEL:=irb120",
            "-p", "EE_MODEL:=schunk"
        ]
        subprocess.run(command)
        self.get_logger().info(f"Executed command for {program_name}")

def main(args=None):
    rclpy.init(args=args)
    node = ExecuteProgram()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
