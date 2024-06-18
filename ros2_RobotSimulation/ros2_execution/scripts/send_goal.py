#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition
import math

class GoalPublisher(Node):

    def __init__(self):
        super().__init__('goal_publisher')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.publisher_amr1 = self.create_publisher(PoseStamped, 'factory_amr1/goal_update', qos_profile)
        self.publisher_amr2 = self.create_publisher(PoseStamped, 'factory_amr2/goal_update', qos_profile)
        self.publisher_amr3 = self.create_publisher(PoseStamped, 'factory_amr3/goal_update', qos_profile)

        self.amr1_client = ActionClient(self, NavigateToPose, 'factory_amr1/navigate_to_pose')
        self.amr2_client = ActionClient(self, NavigateToPose, 'factory_amr2/navigate_to_pose')
        self.amr3_client = ActionClient(self, NavigateToPose, 'factory_amr3/navigate_to_pose')

        self.luggage_publisher_amr1 = self.create_publisher(JointTrajectory, 'factory_amr1/factory_amr1_luggage_controller/joint_trajectory', 10)
        self.luggage_publisher_amr2 = self.create_publisher(JointTrajectory, 'factory_amr2/factory_amr2_luggage_controller/joint_trajectory', 10)
        self.luggage_publisher_amr3 = self.create_publisher(JointTrajectory, 'factory_amr3/factory_amr3_luggage_controller/joint_trajectory', 10)

        self.amr1_goal_active = True
        self.amr2_goal_active = True
        self.amr3_goal_active = True

        self.timer = self.create_timer(3.0, self.timer_callback)

        # Create individual timers for luggage opening
        self.create_timer(90.0, self.luggage_timer_callback_factory_amr2)
        self.create_timer(140.0, self.luggage_timer_callback_factory_amr3)
        self.create_timer(210.0, self.luggage_timer_callback_factory_amr1)

    def timer_callback(self):
        if self.amr1_goal_active:
            self.send_goal('factory_amr1', -2.0, 4.5, 0.0, 1.57, self.amr1_client)
        if self.amr2_goal_active:
            self.send_goal('factory_amr2', -2.5, -1.5, 0.0, 0.0, self.amr2_client)
        if self.amr3_goal_active:
            self.send_goal('factory_amr3', 2.0, 5.0, 0.0, -1.57, self.amr3_client)

    def send_goal(self, namespace, x, y, z, w, client):
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map'
        goal_msg.pose.position.x = x
        goal_msg.pose.position.y = y
        goal_msg.pose.orientation.z = z
        goal_msg.pose.orientation.w = w

        goal_action = NavigateToPose.Goal()
        goal_action.pose = goal_msg

        client.wait_for_server()
        self.send_goal_future = client.send_goal_async(goal_action)
        self.send_goal_future.add_done_callback(lambda future: self.goal_response_callback(future, namespace, x, y))

    def goal_response_callback(self, future, namespace, goal_x, goal_y):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')

    def luggage_timer_callback_factory_amr1(self):
        self.publish_luggage_joint('factory_amr1')
        self.deactivate_lifecycle_nodes('factory_amr1')
        self.amr1_goal_active = False

    def luggage_timer_callback_factory_amr2(self):
        self.publish_luggage_joint('factory_amr2')
        self.deactivate_lifecycle_nodes('factory_amr2')
        self.amr2_goal_active = False

    def luggage_timer_callback_factory_amr3(self):
        self.publish_luggage_joint('factory_amr3')
        self.deactivate_lifecycle_nodes('factory_amr3')
        self.amr3_goal_active = False

    def publish_luggage_joint(self, namespace):
        joint_trajectory = JointTrajectory()
        joint_trajectory.joint_names = ['luggage_joint']
        point = JointTrajectoryPoint()
        point.positions = [0.3]
        point.time_from_start.sec = 3
        joint_trajectory.points = [point]

        if namespace == 'factory_amr1':
            self.luggage_publisher_amr1.publish(joint_trajectory)
            self.get_logger().info(f'{namespace} luggage joint opened')
        elif namespace == 'factory_amr2':
            self.luggage_publisher_amr2.publish(joint_trajectory)
            self.get_logger().info(f'{namespace} luggage joint opened')
        elif namespace == 'factory_amr3':
            self.luggage_publisher_amr3.publish(joint_trajectory)
            self.get_logger().info(f'{namespace} luggage joint opened')

    def deactivate_lifecycle_nodes(self, namespace):
        node_names = ['planner_server', 'controller_server', 'behavior_server', 'bt_navigator']
        for node_name in node_names:
            service_name = f'/{namespace}/{node_name}/change_state'
            client = self.create_client(ChangeState, service_name)
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'Waiting for service {service_name} to be available...')
            req = ChangeState.Request()
            req.transition.id = Transition.TRANSITION_DEACTIVATE
            future = client.call_async(req)
            future.add_done_callback(lambda future: self.deactivate_callback(future, node_name, namespace))

    def deactivate_callback(self, future, node_name, namespace):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'{namespace} {node_name} successfully deactivated')
            else:
                self.get_logger().error(f'Failed to deactivate {namespace} {node_name}')
        except Exception as e:
            self.get_logger().error(f'Service call failed for {namespace} {node_name}: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = GoalPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
