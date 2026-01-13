from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. 파라미터 노드 실행
        Node(
            package='basic_param',
            executable='param_node',
            name='minimal_param_node',
            output='screen'
        ),
        
        # 2. RQT Parameter Reconfigure 실행
        # (파라미터 조절 GUI를 바로 띄웁니다)
        Node(
            package='rqt_reconfigure',
            executable='rqt_reconfigure',
            name='rqt_reconfigure',
            output='screen'
        ),
    ])
