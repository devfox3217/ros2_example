#!/bin/bash

# 1. 패키지 빌드
echo "패키지 'basic_param' 빌드 중..."
colcon build --packages-select basic_param

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

# 3. 런치 파일 실행 (노드 + GUI)
echo "런치 파일 실행 (param_launch.py)..."
echo "잠시 후 RQT GUI 창이 뜨면 왼쪽에서 '/minimal_param_node'를 선택하세요."
ros2 launch basic_param param_launch.py

# =================================================================
# [참고]
# 이 스크립트는 'param_node'와 'rqt_reconfigure'를 동시에 실행합니다.
# GUI 창에서 'my_parameter' 값을 변경하면 터미널 로그가 바뀌는 것을 확인할 수 있습니다.
# =================================================================
