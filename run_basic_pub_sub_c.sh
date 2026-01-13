#!/bin/bash

# 1. 패키지 빌드
echo "패키지 'basic_pub_sub_c' (C++ 버전) 빌드 중..."
colcon build --packages-select basic_pub_sub_c

# 빌드 성공 여부 확인
if [ $? -ne 0 ]; then
    echo "빌드 실패! 종료합니다."
    exit 1
fi

# 2. 설정 스크립트(setup.bash) 적용
# 새로운 패키지나 변경 사항을 인식하기 위해 빌드 후 반드시 실행해야 합니다.
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
else
    echo "오류: install/setup.bash 파일을 찾을 수 없습니다."
    exit 1
fi

# 3. 인자 확인
if [ -z "$1" ]; then
    echo "사용법: $0 [talker|listener]"
    exit 1
fi

NODE_TYPE=$1

# 4. 선택한 노드 실행
if [ "$NODE_TYPE" == "talker" ]; then
    echo "Publisher (talker) 시작..."
    ros2 run basic_pub_sub_c talker
elif [ "$NODE_TYPE" == "listener" ]; then
    echo "Subscriber (listener) 시작..."
    ros2 run basic_pub_sub_c listener
else
    echo "잘못된 인자입니다: $NODE_TYPE"
    echo "사용법: $0 [talker|listener]"
    exit 1
fi

# =================================================================
# [수동 실행 가이드]
# 이 스크립트 없이 터미널에서 직접 실행하려면 다음 명령어를 순서대로 입력하세요.
#
# 1. 워크스페이스 루트 폴더로 이동 (예: ~/ros2_example)
#
# 2. 패키지 빌드 (코드 수정 시 필수)
#    $ colcon build --packages-select basic_pub_sub_c
#
# 3. 환경 설정 적용 (새 터미널 열 때마다 필수)
#    $ source install/setup.bash
#
# 4. 노드 실행
#    - Publisher 실행:
#      $ ros2 run basic_pub_sub_c talker
#
#    - Subscriber 실행:
#      $ ros2 run basic_pub_sub_c listener
# =================================================================
