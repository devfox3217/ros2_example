import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import sys

class ImagePublisher(Node):

    def __init__(self):
        super().__init__('image_publisher')
        
        # 1. 카메라 연결 시도
        # 0번 장치(/dev/video0)를 엽니다.
        self.cap = cv2.VideoCapture(0)
        
        if not self.cap.isOpened():
            self.get_logger().error('카메라를 열 수 없습니다! (/dev/video0)')
            # 연결 실패 시 예외를 발생시켜 main 함수에서 처리하도록 함
            raise RuntimeError("Camera not found")
        
        self.get_logger().info('카메라가 성공적으로 연결되었습니다.')

        # 2. Publisher 생성
        self.publisher_ = self.create_publisher(Image, 'image_raw', 10)
        
        # 3. CvBridge 초기화
        self.br = CvBridge()
        
        # 4. 타이머 설정 (0.1초마다 실행)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        # 카메라로부터 프레임 읽기
        ret, frame = self.cap.read()
        
        if ret:
            # 읽은 프레임을 ROS 메시지로 변환하여 발행
            msg = self.br.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher_.publish(msg)
        else:
            self.get_logger().warn('프레임을 읽을 수 없습니다.')

    def __del__(self):
        # 노드 종료 시 카메라 자원 해제
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()

def main(args=None):
    rclpy.init(args=args)

    try:
        # 노드 생성 시도
        image_publisher = ImagePublisher()
        
        # 성공하면 Spin (무한 루프)
        rclpy.spin(image_publisher)
        
        image_publisher.destroy_node()
        
    except RuntimeError as e:
        # 카메라 연결 실패 등의 런타임 에러 처리
        print(f"[ERROR] {e}")
    except KeyboardInterrupt:
        pass
    finally:
        # ROS 2 종료
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
