import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

class MinimalParam(Node):

    def __init__(self):
        super().__init__('minimal_param_node')

        # 1. 파라미터 선언 (이름: 'my_parameter', 기본값: 'world')
        # descriptor를 사용하여 파라미터에 대한 설명이나 제약조건을 추가할 수도 있습니다.
        self.declare_parameter('my_parameter', 'world')

        # 2. 파라미터 변경 콜백 등록
        # 외부에서 파라미터 값을 변경하려 할 때 이 함수가 호출됩니다.
        self.add_on_set_parameters_callback(self.parameter_callback)

        # 3. 타이머 생성 (1초마다 실행)
        self.timer = self.create_timer(1, self.timer_callback)

    def timer_callback(self):
        # 현재 파라미터 값을 읽어옵니다.
        my_param = self.get_parameter('my_parameter').get_parameter_value().string_value

        self.get_logger().info('Hello %s!' % my_param)

    def parameter_callback(self, params):
        # 변경 요청된 파라미터들을 확인합니다.
        for param in params:
            if param.name == 'my_parameter':
                if param.type_ == param.Type.STRING:
                    self.get_logger().info('파라미터 변경 감지! 새 값: %s' % param.value)
                else:
                    self.get_logger().warn('잘못된 타입입니다! 문자열이어야 합니다.')
                    return SetParametersResult(successful=False)

        # 변경 승인
        return SetParametersResult(successful=True)

def main(args=None):
    rclpy.init(args=args)

    node = MinimalParam()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
