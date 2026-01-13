#!/bin/bash

# 인자 확인: 패키지 이름이 없으면 사용법 출력
if [ -z "$1" ]; then
    echo "사용법: $0 <패키지이름> [p|c]"
    echo "  p: ament_python (기본값, Python용)"
    echo "  c: ament_cmake (C++용)"
    exit 1
fi

PKG_NAME=$1
TYPE=${2:-p} # 두 번째 인자가 없으면 기본값으로 'p' (Python) 사용

# src 디렉토리가 없으면 생성
if [ ! -d "src" ]; then
    echo "'src' 디렉토리 생성 중..."
    mkdir src
fi

cd src

# 이미 같은 이름의 패키지가 있는지 확인
if [ -d "$PKG_NAME" ]; then
    echo "패키지 '$PKG_NAME'가 이미 존재합니다. 다시 생성하려면 먼저 삭제해주세요."
    exit 1
fi

# 패키지 생성 명령어 실행
if [ "$TYPE" == "p" ]; then
    echo "Python 패키지 생성 중 (ament_python): $PKG_NAME"
    ros2 pkg create --build-type ament_python "$PKG_NAME"
elif [ "$TYPE" == "c" ]; then
    echo "C++ 패키지 생성 중 (ament_cmake): $PKG_NAME"
    ros2 pkg create --build-type ament_cmake "$PKG_NAME"
else
    echo "잘못된 타입입니다: $TYPE. Python은 'p', C++는 'c'를 사용하세요."
    exit 1
fi

echo "패키지 생성 완료: $PKG_NAME"

# =================================================================
# [수동 생성 가이드]
# 이 스크립트 없이 터미널에서 직접 패키지를 생성하려면 다음 명령어를 사용하세요.
#
# 1. src 디렉토리로 이동
#    $ cd src
#
# 2. 패키지 생성
#    - Python 패키지 생성:
#      $ ros2 pkg create --build-type ament_python <패키지이름>
#
#    - C++ 패키지 생성:
#      $ ros2 pkg create --build-type ament_cmake <패키지이름>
#
#    (옵션) 의존성 추가와 함께 생성하기:
#      $ ros2 pkg create --build-type ament_python <패키지이름> --dependencies rclpy std_msgs
# =================================================================
# ros2 스터디 팀 화이팅
