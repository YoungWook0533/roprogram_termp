#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from conveyorbelt_msgs.srv import ConveyorBeltControl

class ConveyorControlNode(Node):

    def __init__(self):
        super().__init__('conveyor_control_node')

        # 서비스 클라이언트 생성
        self.client = self.create_client(ConveyorBeltControl, 'CONVEYORPOWER')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.counter = 0  # 호출 횟수를 추적하기 위한 카운터
        self.timer_period = 4.0  # 초기 타이머 주기 설정
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def timer_callback(self):
        self.counter += 1
        request = ConveyorBeltControl.Request()
        
        if self.counter % 2 == 1:
            request.power = 5.0
            self.timer_period = 4.0  # 홀수 번째 요청 후 4초 대기
        else:
            request.power = 0.0
            self.timer_period = 20.0  # 짝수 번째 요청 후 20초 대기

        self.future = self.client.call_async(request)
        self.future.add_done_callback(self.response_callback)
        
        # 타이머 재설정
        self.timer.cancel()
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Response: {response.success}')
        except Exception as e:
            self.get_logger().info(f'Service call failed {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = ConveyorControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
