import time
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

# ROS 2 기본 제공 액션 인터페이스 사용
from example_interfaces.action import Fibonacci

class FibonacciActionServer(Node):

    def __init__(self):
        super().__init__('fibonacci_action_server')
        
        # 액션 서버 생성
        # 매개변수 1: 액션 타입 (Fibonacci)
        # 매개변수 2: 액션 이름 ('fibonacci')
        # 매개변수 3: 실행 콜백 함수 (self.execute_callback)
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)
            
        self.get_logger().info('액션 서버가 준비되었습니다. (fibonacci)')

    def execute_callback(self, goal_handle):
        self.get_logger().info('액션 요청을 받았습니다. 실행 중...')
        
        # 1. Goal(목표) 데이터 가져오기
        order = goal_handle.request.order
        
        # 2. Feedback(중간 결과) 객체 생성
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]

        # 3. 피보나치 수열 계산 및 피드백 전송
        for i in range(1, order):
            # 다음 숫자 계산
            feedback_msg.sequence.append(
                feedback_msg.sequence[i] + feedback_msg.sequence[i-1])
            
            # 피드백 전송
            self.get_logger().info('피드백 전송: {0}'.format(feedback_msg.sequence))
            goal_handle.publish_feedback(feedback_msg)
            
            # 작업을 시뮬레이션하기 위해 1초 대기
            time.sleep(1)

        # 4. 작업 완료 처리
        goal_handle.succeed()

        # 5. Result(최종 결과) 반환
        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence
        
        self.get_logger().info('작업 완료! 최종 결과 반환.')
        return result

def main(args=None):
    rclpy.init(args=args)

    fibonacci_action_server = FibonacciActionServer()

    rclpy.spin(fibonacci_action_server)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
