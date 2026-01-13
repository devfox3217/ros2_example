from example_interfaces.srv import AddTwoInts  # 사용할 서비스 타입 import

import rclpy
from rclpy.node import Node

class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        
        # 서비스 서버 생성
        # 매개변수 1: 서비스 타입 (AddTwoInts)
        # 매개변수 2: 서비스 이름 ('add_two_ints')
        # 매개변수 3: 콜백 함수 (self.add_two_ints_callback)
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)
        self.get_logger().info('서비스 서버가 준비되었습니다. (add_two_ints)')

    # 클라이언트로부터 요청(request)이 오면 실행되는 콜백 함수
    # request: 요청 데이터 (a, b)
    # response: 응답 데이터 (sum)
    def add_two_ints_callback(self, request, response):
        # 요청받은 두 수를 더해서 응답에 저장
        response.sum = request.a + request.b
        
        # 로그 출력
        self.get_logger().info('요청 받음: a=%d, b=%d' % (request.a, request.b))
        self.get_logger().info('응답 보냄: sum=%d' % response.sum)
        
        return response

def main(args=None):
    rclpy.init(args=args)

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
