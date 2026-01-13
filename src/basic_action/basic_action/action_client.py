import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
import sys

from example_interfaces.action import Fibonacci

class FibonacciActionClient(Node):

    def __init__(self):
        super().__init__('fibonacci_action_client')
        
        # 액션 클라이언트 생성
        self._action_client = ActionClient(self, Fibonacci, 'fibonacci')

    def send_goal(self, order):
        # 액션 서버 대기
        self.get_logger().info('액션 서버를 찾는 중...')
        self._action_client.wait_for_server()

        # Goal 메시지 생성
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self.get_logger().info('액션 요청 전송 (order=%d)...' % order)

        # 액션 요청 전송 (비동기)
        # feedback_callback: 중간 결과를 받을 때 실행될 함수
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        # 요청이 수락되었는지 확인하는 콜백 등록
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    # 서버가 요청을 수락/거절했을 때 호출됨
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('액션 요청이 거절되었습니다.')
            return

        self.get_logger().info('액션 요청이 수락되었습니다. 결과 대기 중...')

        # 최종 결과를 기다림
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    # 서버로부터 피드백(중간 결과)이 올 때마다 호출됨
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('피드백 수신: {0}'.format(feedback.sequence))

    # 작업이 완료되어 최종 결과가 왔을 때 호출됨
    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('최종 결과 수신: {0}'.format(result.sequence))
        
        # 노드 종료
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)

    action_client = FibonacciActionClient()

    # 명령줄 인자로 숫자 받기 (기본값 10)
    order = 10
    if len(sys.argv) > 1:
        order = int(sys.argv[1])

    action_client.send_goal(order)

    rclpy.spin(action_client)

if __name__ == '__main__':
    main()
