import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 패키지 경로 찾기
    pkg_share = get_package_share_directory('rplidar_s2')
    
    # RViz 설정 파일 경로
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'rplidar.rviz')

    return LaunchDescription([
        # 1. RPLIDAR 드라이버 노드 실행 (sllidar_ros2 패키지 사용)
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            output='screen',
            parameters=[{
                'channel_type': 'serial',
                'serial_port': '/dev/ttyUSB0', 
                'serial_baudrate': 1000000, # S2 모델은 보통 1M baudrate 사용
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': True,
                'scan_mode': 'DenseBoost', # S2 전용 모드 (필요시 수정)
            }]
        ),

        # 2. RViz2 실행
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen'
        )
    ])
