import rclpy  # ROS 2 Python 클라이언트 라이브러리
from rclpy.node import Node  # Node 클래스를 사용하기 위해 import
from std_msgs.msg import String  # 문자열 메시지 타입을 사용하기 위해 import

# Node 클래스를 상속받아 MinimalPublisher 클래스를 정의합니다.
class MinimalPublisher(Node):

    def __init__(self):
        # 부모 클래스(Node)의 생성자를 호출하며 노드 이름을 'minimal_publisher'로 설정합니다.
        super().__init__('minimal_publisher')
        
        # 퍼블리셔(Publisher)를 생성합니다.
        # 매개변수 1: 메시지 타입 (String)
        # 매개변수 2: 토픽 이름 ('topic') - 이 이름으로 메시지를 발행합니다.
        # 매개변수 3: 큐 사이즈 (10) - 통신 상태가 나쁠 때 보관할 메시지 개수
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        
        # 타이머 주기 설정 (0.5초)
        timer_period = 0.5  # seconds
        
        # 타이머를 생성합니다. 0.5초마다 self.timer_callback 함수를 실행합니다.
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # 메시지에 포함될 카운터 변수 초기화
        self.i = 0

    # 타이머에 의해 주기적으로 호출되는 콜백 함수입니다.
    def timer_callback(self):
        msg = String()  # String 메시지 객체 생성
        msg.data = 'Hello World: %d' % self.i  # 메시지 데이터 채우기
        
        # 메시지를 발행(publish)합니다.
        self.publisher_.publish(msg)
        
        # 터미널에 로그를 출력합니다.
        self.get_logger().info('Publishing: "%s"' % msg.data)
        
        self.i += 1  # 카운터 증가

def main(args=None):
    # rclpy 라이브러리 초기화
    rclpy.init(args=args)

    # MinimalPublisher 노드 객체 생성
    minimal_publisher = MinimalPublisher()

    # 노드가 종료될 때까지 실행 상태를 유지하며 콜백을 처리합니다. (Spin)
    rclpy.spin(minimal_publisher)

    # 노드를 명시적으로 파괴합니다.
    # (선택 사항 - 가비지 컬렉터가 자동으로 처리하지만 명시적으로 적어주는 것이 좋습니다)
    minimal_publisher.destroy_node()
    
    # rclpy 라이브러리 종료
    rclpy.shutdown()

if __name__ == '__main__':
    main()
