#!/bin/bash

# 1. 패키지 빌드
echo "패키지 'basic_action' 빌드 중..."
colcon build --packages-select basic_action

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

# 3. 인자 확인
if [ -z "$1" ]; then
    echo "사용법: $0 [server|client] <args...>"
    echo "  - server: 액션 서버 실행"
    echo "  - client: 액션 클라이언트 실행 (예: $0 client 10)"
    exit 1
fi

NODE_TYPE=$1

# 4. 선택한 노드 실행
if [ "$NODE_TYPE" == "server" ]; then
    echo "Action Server 시작..."
    ros2 run basic_action server
elif [ "$NODE_TYPE" == "client" ]; then
    ORDER=${2:-10} # 두 번째 인자가 없으면 기본값 10
    echo "Action Client 시작 (목표: $ORDER)..."
    ros2 run basic_action client $ORDER
else
    echo "잘못된 인자입니다: $NODE_TYPE"
    echo "사용법: $0 [server|client]"
    exit 1
fi

# =================================================================
# [수동 실행 가이드]
#
# 1. 패키지 빌드
#    $ colcon build --packages-select basic_action
#
# 2. 환경 설정
#    $ source install/setup.bash
#
# 3. 서버 실행
#    $ ros2 run basic_action server
#
# 4. 클라이언트 실행
#    $ ros2 run basic_action client 10
# =================================================================
