from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Publisher 노드 실행
        Node(
            package='basic_pub_sub',      # 실행할 패키지 이름
            executable='talker',          # 실행할 노드 이름 (setup.py의 entry_points에 등록된 이름)
            name='my_talker',             # 노드 이름 변경 (선택 사항)
            output='screen'               # 로그를 터미널에 출력
        ),
        # Subscriber 노드 실행
        Node(
            package='basic_pub_sub',
            executable='listener',
            name='my_listener',
            output='screen'
        ),
    ])
