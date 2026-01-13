#!/bin/bash

# 1. 패키지 빌드
echo "패키지 'basic_camera' 빌드 중..."
colcon build --packages-select basic_camera

# 빌드 성공 여부 확인
if [ $? -ne 0 ]; then
    echo "빌드 실패! 종료합니다."
    exit 1
fi

# 2. 설정 스크립트(setup.bash) 적용
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
else
    echo "오류: install/setup.bash 파일을 찾을 수 없습니다."
    exit 1
fi

# 3. 런치 파일 실행
echo "카메라 런치 파일 실행 (camera_launch.py)..."
echo "카메라가 연결되어 있다면 RQT Image View가 자동으로 실행됩니다."
echo "카메라 연결 실패 시 프로그램이 종료됩니다."

ros2 launch basic_camera camera_launch.py

# =================================================================
# [참고]
# 이 스크립트는 'img_publisher'와 'rqt_image_view'를 동시에 실행합니다.
# 카메라 연결에 실패하여 'img_publisher'가 종료되면,
# 런치 파일 설정에 의해 'rqt_image_view'도 함께 종료됩니다.
# =================================================================
