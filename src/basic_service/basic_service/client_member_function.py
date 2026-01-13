import sys
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        
        # 서비스 클라이언트 생성
        # 매개변수 1: 서비스 타입 (AddTwoInts)
        # 매개변수 2: 서비스 이름 ('add_two_ints') - 서버와 이름이 같아야 함
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        
        # 서비스 서버가 뜰 때까지 1초 간격으로 대기
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('서비스 서버를 찾을 수 없습니다. 다시 대기 중...')
        
        # 요청 객체 생성
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        
        # 비동기(Async) 방식으로 서비스 요청 전송
        # 결과는 Future 객체로 반환됨
        self.future = self.cli.call_async(self.req)
        
        # Future 객체가 완료될 때까지 대기하지 않고 바로 반환
        # (실제 대기는 main 함수에서 처리)
        rclpy.spin_until_future_complete(self, self.future)
        
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClientAsync()
    
    # 명령줄 인자로 전달된 두 수를 가져옴 (예: ros2 run ... 10 20)
    if len(sys.argv) < 3:
        print("사용법: ros2 run basic_service client <숫자1> <숫자2>")
        return

    a = int(sys.argv[1])
    b = int(sys.argv[2])
    
    response = minimal_client.send_request(a, b)
    
    minimal_client.get_logger().info(
        '요청 결과: %d + %d = %d' %
        (a, b, response.sum))

    minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
