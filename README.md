# ROS 2 Example Project (Humble)

이 프로젝트는 ROS 2 Humble 버전을 기준으로 다양한 기능을 학습하기 위해 만들어진 예제 모음입니다.
Topic, Service, Action, Launch, Parameter, Camera 등 ROS 2의 핵심 개념을 단계별로 실습할 수 있습니다.

## 1. 환경 설정 및 설치

### 필수 요구 사항
*   Ubuntu 22.04 LTS
*   ROS 2 Humble Hawksbill
*   Python 3

### 설치 방법
이 저장소를 클론한 후, 필요한 의존성 패키지를 설치하고 빌드합니다.

```bash
# 1. 저장소 클론
git clone <REPO_URL> ros2_example
cd ros2_example

# 2. 의존성 설치 (rosdep 사용)
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -y

# 3. 전체 빌드
colcon build

# 4. 환경 설정
source install/setup.bash
```

---

## 2. 예제 실행 방법

각 예제는 루트 디렉토리에 있는 쉘 스크립트(`run_*.sh`)를 통해 쉽게 실행할 수 있습니다.

### Step 1: Topic 통신 (Publisher / Subscriber)
가장 기본적인 메시지 송수신 예제입니다.

*   **Python 버전**:
    ```bash
    ./run_basic_pub_sub.sh talker   # Publisher 실행
    ./run_basic_pub_sub.sh listener # Subscriber 실행
    ```
*   **C++ 버전**:
    ```bash
    ./run_basic_pub_sub_c.sh talker
    ./run_basic_pub_sub_c.sh listener
    ```

### Step 2: Service 통신 (Server / Client)
요청(Request)과 응답(Response) 방식의 통신 예제입니다. (두 수 더하기)

```bash
./run_basic_service.sh server           # 서버 실행
./run_basic_service.sh client 10 20     # 클라이언트 실행 (10 + 20 요청)
```

### Step 3: Action 통신 (Server / Client)
오래 걸리는 작업과 피드백을 다루는 통신 예제입니다. (피보나치 수열 계산)

```bash
./run_basic_action.sh server      # 서버 실행
./run_basic_action.sh client 10   # 클라이언트 실행 (항 10개 요청)
```

### Step 4: Launch 파일
여러 노드를 명령어 하나로 동시에 실행하는 예제입니다. (Talker + Listener)

```bash
./run_basic_launch.sh
```

### Step 5: Parameter (파라미터)
실행 중에 노드의 설정을 변경하는 예제입니다. RQT GUI가 함께 실행됩니다.

```bash
./run_basic_param.sh
```

### Step 6: Camera (카메라)
USB 카메라 영상을 읽어서 토픽으로 발행하는 예제입니다.
카메라가 연결되어 있으면 RQT Image View가 자동으로 실행되어 화면을 보여줍니다.

```bash
./run_basic_camera.sh
```

---

## 3. 유틸리티 스크립트

*   **create_pkg.sh**: 새로운 패키지를 쉽게 생성해주는 스크립트입니다.
    ```bash
    ./create_pkg.sh <패키지명> [p|c]  # p: Python, c: C++
    ```

## 4. 라이선스
Apache License 2.0
