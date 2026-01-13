#!/bin/bash

# 1. 패키지 빌드
echo "패키지 'basic_service' 빌드 중..."
colcon build --packages-select basic_service

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
    echo "  - server: 서비스 서버 실행"
    echo "  - client: 서비스 클라이언트 실행 (예: $0 client 10 20)"
    exit 1
fi

NODE_TYPE=$1

# 4. 선택한 노드 실행
if [ "$NODE_TYPE" == "server" ]; then
    echo "Service Server 시작..."
    ros2 run basic_service server
elif [ "$NODE_TYPE" == "client" ]; then
    # 클라이언트 실행 시 추가 인자(숫자 두 개)가 필요함
    if [ -z "$2" ] || [ -z "$3" ]; then
        echo "오류: 클라이언트 실행 시 두 개의 숫자가 필요합니다."
        echo "예시: $0 client 10 20"
        exit 1
    fi

    A=$2
    B=$3
    echo "Service Client 시작 (요청: $A + $B)..."
    ros2 run basic_service client $A $B
else
    echo "잘못된 인자입니다: $NODE_TYPE"
    echo "사용법: $0 [server|client]"
    exit 1
fi

# =================================================================
# [수동 실행 가이드]
#
# 1. 패키지 빌드
#    $ colcon build --packages-select basic_service
#
# 2. 환경 설정
#    $ source install/setup.bash
#
# 3. 서버 실행
#    $ ros2 run basic_service server
#
# 4. 클라이언트 실행
#    $ ros2 run basic_service client 10 20
# =================================================================
