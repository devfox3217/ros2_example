from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown

def generate_launch_description():
    # 1. 카메라 노드 정의
    img_publisher_node = Node(
        package='basic_camera',
        executable='img_publisher',
        name='image_publisher',
        output='screen'
    )

    # 2. RQT Image View 노드 정의
    # arguments=['/image_raw']를 주어 실행 시 바로 해당 토픽을 보게 설정
    rqt_node = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='rqt_image_view',
        arguments=['/image_raw'],
        output='screen'
    )

    # 3. 이벤트 핸들러 정의
    # 카메라 노드(img_publisher_node)가 종료(Exit)되면
    # 런치 파일 전체를 종료(Shutdown)시키는 이벤트를 발생시킵니다.
    shutdown_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=img_publisher_node,
            on_exit=[EmitEvent(event=Shutdown())],
        )
    )

    return LaunchDescription([
        img_publisher_node,
        rqt_node,
        shutdown_handler
    ])
