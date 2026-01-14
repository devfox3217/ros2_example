import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
import sys

class ImagePublisher(Node):

    def __init__(self):
        super().__init__('image_publisher')
        
        # 1. 카메라 연결 시도
        self.cap = cv2.VideoCapture(0)
        
        if not self.cap.isOpened():
            self.get_logger().error('카메라를 열 수 없습니다! (/dev/video0)')
            raise RuntimeError("Camera not found")
        
        # 카메라 해상도 설정 (하드웨어 지원 시 적용됨)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        self.get_logger().info('카메라가 성공적으로 연결되었습니다.')

        # 2. Publisher 생성 (QoS: Best Effort 적용)
        # qos_profile_sensor_data는 'Best Effort'와 'Volatile' 설정을 포함하여
        # 데이터 유실을 감수하고 최신 데이터를 빠르게 전송하는 데 최적화되어 있습니다.
        self.publisher_ = self.create_publisher(Image, 'image_raw', qos_profile_sensor_data)
        
        # 압축 이미지 Publisher 추가
        self.compressed_publisher_ = self.create_publisher(CompressedImage, 'image_raw/compressed', qos_profile_sensor_data)
        
        # 3. CvBridge 초기화
        self.br = CvBridge()
        
        # 4. 타이머 설정 (0.033초 -> 약 30 FPS)
        timer_period = 0.033
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        
        if ret:
            # (옵션) 소프트웨어 리사이즈: 하드웨어 설정이 안 먹힐 경우 강제로 줄임
            # frame = cv2.resize(frame, (320, 240))

            # 1. Raw Image 발행
            msg = self.br.cv2_to_imgmsg(frame, encoding='bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera_frame'
            self.publisher_.publish(msg)
            
            # 2. Compressed Image 발행 (JPEG 압축)
            # 네트워크 대역폭을 획기적으로 줄여줍니다.
            success, encoded_img = cv2.imencode('.jpg', frame)
            if success:
                cmp_msg = CompressedImage()
                cmp_msg.header = msg.header
                cmp_msg.format = "jpeg"
                cmp_msg.data = encoded_img.tobytes()
                self.compressed_publisher_.publish(cmp_msg)
                
        else:
            self.get_logger().warn('프레임을 읽을 수 없습니다.')

    def __del__(self):
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()

def main(args=None):
    rclpy.init(args=args)

    try:
        image_publisher = ImagePublisher()
        rclpy.spin(image_publisher)
        image_publisher.destroy_node()
    except RuntimeError as e:
        print(f"[ERROR] {e}")
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
