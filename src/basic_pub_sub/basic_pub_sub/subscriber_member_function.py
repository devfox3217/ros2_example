import rclpy  # ROS 2 Python 클라이언트 라이브러리
from rclpy.node import Node  # Node 클래스를 사용하기 위해 import
from std_msgs.msg import String  # 문자열 메시지 타입을 사용하기 위해 import

# Node 클래스를 상속받아 MinimalSubscriber 클래스를 정의합니다.
class MinimalSubscriber(Node):

    def __init__(self):
        # 부모 클래스(Node)의 생성자를 호출하며 노드 이름을 'minimal_subscriber'로 설정합니다.
        super().__init__('minimal_subscriber')
        
        # 서브스크라이버(Subscriber)를 생성합니다.
        # 매개변수 1: 메시지 타입 (String)
        # 매개변수 2: 구독할 토픽 이름 ('topic') - Publisher가 발행하는 토픽 이름과 같아야 합니다.
        # 매개변수 3: 콜백 함수 (self.listener_callback) - 메시지를 받을 때마다 실행될 함수
        # 매개변수 4: 큐 사이즈 (10)
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        
        # 사용하지 않는 변수 경고를 방지하기 위한 코드 (기능에는 영향 없음)
        self.subscription

    # 메시지를 수신했을 때 호출되는 콜백 함수입니다.
    # msg 매개변수로 수신된 메시지 데이터가 전달됩니다.
    def listener_callback(self, msg):
        # 수신된 메시지 내용을 터미널에 로그로 출력합니다.
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    # rclpy 라이브러리 초기화
    rclpy.init(args=args)

    # MinimalSubscriber 노드 객체 생성
    minimal_subscriber = MinimalSubscriber()

    # 노드가 종료될 때까지 대기하며 메시지가 들어오면 콜백 함수를 실행합니다. (Spin)
    rclpy.spin(minimal_subscriber)

    # 노드를 명시적으로 파괴합니다.
    minimal_subscriber.destroy_node()
    
    # rclpy 라이브러리 종료
    rclpy.shutdown()

if __name__ == '__main__':
    main()
