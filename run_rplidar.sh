#!/bin/bash

# 0. 드라이버 패키지 확인 및 설치
DRIVER_DIR="src/sllidar_ros2"
if [ ! -d "$DRIVER_DIR" ]; then
    echo "RPLIDAR 드라이버(sllidar_ros2)가 없습니다. 다운로드합니다..."
    git clone https://github.com/Slamtec/sllidar_ros2.git "$DRIVER_DIR"
else
    echo "RPLIDAR 드라이버가 이미 존재합니다."
fi

# 1. USB 권한 설정
USB_PORT="/dev/ttyUSB0"
if [ -e "$USB_PORT" ]; then
    echo "USB 포트($USB_PORT) 권한 설정 중..."
    sudo chmod 666 "$USB_PORT"
else
    echo "경고: $USB_PORT를 찾을 수 없습니다. 라이다가 연결되어 있나요?"
    echo "연결되지 않았다면 드라이버 실행 시 에러가 발생할 수 있습니다."
fi

# 2. 패키지 빌드 (드라이버 포함)
echo "패키지 빌드 중 (rplidar_s2, sllidar_ros2)..."
colcon build --packages-select rplidar_s2 sllidar_ros2

# 빌드 성공 여부 확인
if [ $? -ne 0 ]; then
    echo "빌드 실패! 종료합니다."
    exit 1
fi

# 3. 설정 스크립트 적용
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
else
    echo "오류: install/setup.bash 파일을 찾을 수 없습니다."
    exit 1
fi

# 4. 런치 파일 실행
echo "RPLIDAR S2 실행 및 시각화 (RViz)..."
ros2 launch rplidar_s2 rplidar_launch.py

# =================================================================
# [참고]
# - 처음 실행 시 드라이버를 다운로드하고 빌드하느라 시간이 걸릴 수 있습니다.
# - '/dev/ttyUSB0' 권한 설정을 위해 sudo 비밀번호를 물어볼 수 있습니다.
# - 실행 후 RViz 창이 뜨면 붉은색 점들(LaserScan)이 보여야 합니다.
# =================================================================
