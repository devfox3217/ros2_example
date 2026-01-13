#!/bin/bash

# 1. 패키지 빌드
echo "패키지 'basic_launch' 빌드 중..."
colcon build --packages-select basic_launch

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
echo "런치 파일 실행 (pub_sub_launch.py)..."
# ros2 launch <패키지명> <런치파일명>
ros2 launch basic_launch pub_sub_launch.py

# =================================================================
# [수동 실행 가이드]
#
# 1. 패키지 빌드
#    $ colcon build --packages-select basic_launch
#
# 2. 환경 설정
#    $ source install/setup.bash
#
# 3. 런치 파일 실행
#    $ ros2 launch basic_launch pub_sub_launch.py
# =================================================================
