# 핸드 프리셋 컨트롤러 테스트 매뉴얼

## 1. 목적과 테스트 범위

`feature-preset-controller` 브랜치의 `ffw_preset_controller`를 일반 AI Worker
Docker 환경에서 검증하는 절차다. A2 또는 LG2 리더의 그리퍼 입력으로
SH5/BH5의 좌우 20자유도 핸드를 제어하고, YAML에 정의한 프리셋을 서비스로 변경한다.

먼저 리더 입력과 서비스 준비 상태를 확인한 뒤, follower와 연동해 핸드 출력을
확인한다. 핸드 명령은 같은 쪽 리더 팔 궤적이 발행될 때만 출력된다.
BG2/SG2는 그리퍼 모델이므로 팔 연동 시험에는 사용할 수 있지만, 20자유도
핸드 프리셋의 실제 동작 확인에는 SH5/BH5가 필요하다.

이 문서의 기준 코드는 `ai_worker` 커밋 `abb20ac`이다. 의존성 확인에 사용한
버전은 `cyclo_control/main`의 `584a343`, `robotis_interfaces/main`의 `c79afb6`이다.
다른 커밋으로 시험하면 결과와 함께 실제 커밋을 기록한다.

## 2. 컨테이너와 소스 준비

예시는 공식 `robotis/ai-worker:2.3.0` 이미지 기반의 일반 컨테이너를 가정한다.
실제 컨테이너 이름이 다르면 아래 `ai_worker`를 바꾼다. 별도의 개발 컨테이너나
로컬 스냅샷 이미지는 필요하지 않다.

**호스트 터미널:**

```bash
docker exec -it ai_worker bash
```

이하 별도 표시가 없는 명령은 모두 컨테이너 안에서 실행한다.
일반 `docker/docker-compose.yml`은 host 네트워크와 `/dev` 마운트를 사용한다.
다른 방식으로 생성한 컨테이너라면 해당 리더/follower 장치에 접근할 수 있는지 먼저 확인한다.
같은 장치를 두 개의 하드웨어 노드에서 동시에 열지 않는다.

작업 중인 변경을 보존한 상태에서 아래 저장소를 준비한다.
이미 준비된 테스트 브랜치를 받았다면 해당 커밋을 그대로 사용해도 된다.

```bash
cd /root/ros2_ws/src/ai_worker
git status --short
git fetch origin
git switch feature-preset-controller
git pull --ff-only

cd /root/ros2_ws/src/robotis_interfaces
git status --short
git fetch origin
git switch main
git pull --ff-only

cd /root/ros2_ws/src/cyclo_control
git status --short
git fetch origin
git switch main
git pull --ff-only
```

`robotis_interfaces`, `cyclo_control`, `robotis_hand` 및 하드웨어 드라이버 소스가
없는 환경은 먼저 저장소의 `ai_worker_ci.repos`와 일반 개발 환경 설정에 맞춰
의존성을 준비한다. `robotis_interfaces`에는 `SetPreset.srv`가 있어야 한다.

이전 설치의 자체 서비스 생성물이 섞이지 않도록 별도 빌드/설치 경로를 사용한다.

```bash
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
cd /root/ros2_ws
colcon build --symlink-install \
  --build-base build_hand_test \
  --install-base install_hand_test \
  --packages-up-to ffw_preset_controller ffw_bringup
source /root/ros2_ws/install_hand_test/setup.bash
```

**새 컨테이너 터미널을 열 때마다** 다음을 실행한다. 아래 경로를 이번 테스트의
공통 설치 경로로 사용한다.

```bash
source /root/ros2_ws/install_hand_test/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ROS_DOMAIN_ID=30
```

설치와 서비스 정의를 확인한다.

```bash
ros2 pkg prefix ffw_preset_controller
ros2 pkg prefix ffw_bringup
ros2 pkg prefix robotis_interfaces
ros2 interface show robotis_interfaces/srv/SetPreset
```

첫 번째 명령의 경로가 `install_hand_test/ffw_preset_controller`인지 확인한다.
`SetPreset` 요청에는 `target_arm`, `left_preset_id`, `right_preset_id`가 있고,
응답에는 `accepted`, `transition_id`, `message`가 있어야 한다.
이 패키지는 자체 `.srv`를 생성하지 않는다.

## 3. Zenoh 준비

테스트에 사용할 ROS 노드는 같은 도메인과 연결 가능한 Zenoh 라우터를 사용해야 한다.
기존 라우터가 없다면 전용 터미널 하나에서 실행하고 유지한다.

```bash
ros2 run rmw_zenoh_cpp rmw_zenohd
```

같은 호스트의 host-network 컨테이너들은 같은 호스트의 `localhost`를 사용한다.
서로 다른 호스트 또는 bridge-network 컨테이너는 도메인 번호만 맞춰서는 연결되지 않는다.
그 경우 외부 노드를 실행할 터미널에 실제 라우터 주소를 지정한다.
아래 주소는 예시이므로 반드시 테스트 환경의 주소로 바꾼다.

```bash
export ZENOH_CONFIG_OVERRIDE='mode="client";connect/endpoints=["tcp/192.168.0.10:7447"]'
```

환경변수 변경은 이후 실행하는 노드에 적용된다. `ros2` CLI의 기존 데몬 설정을
피하려면 이 문서의 조회 명령처럼 `--no-daemon`을 사용한다.

## 4. 리더 장치와 설정 확인

### A2

이 브랜치의 A2 포트는 현재 다음 파일에서 **`/dev/ttyACM0`으로 고정**되어 있다.

`ffw_description/ros2_control/ffw_a2_leader/ffw_a2_leader.ros2_control.xacro`

```bash
ls -l /dev/ttyACM0
```

해당 장치가 실제 A2인지 확인한다. 다른 경로를 사용해야 하면 위 xacro의
`port_name`을 수정하고 `ffw_description`을 다시 빌드한다.
A2 리더 launch에는 현재 별도의 `port_name` 인자가 없다.

### LG2

좌우 리더 장치와 기존 LG2의 포트 설정을 확인한다. 일반 udev 설정의 장치명은
`/dev/left_leader`, `/dev/right_leader`이다.

```bash
ls -l /dev/left_leader /dev/right_leader
```

리더 입력 보정값은 아래 YAML에 있다.

- A2: `ffw_bringup/config/ffw_a2_leader/ffw_a2_hand_preset_controller.yaml`
  — release `-0.4 rad`, grasp `0.7 rad`.
- LG2: `ffw_bringup/config/ffw_lg2_leader/ffw_lg2_hand_preset_controller.yaml`
  — release `-0.3 rad`, grasp `0.8 rad`.

## 5. 1차 시험: 리더만 연결하여 명령 생성 확인

다른 하드웨어 제어 노드가 같은 리더 장치를 사용하지 않는 상태에서 실행한다.
아래 두 launch 중 **연결한 리더에 해당하는 하나만** 실행한다.

```bash
# A2
ros2 launch ffw_bringup ffw_a2_leader_ai.launch.py use_hand_preset:=true style:=advanced
```

```bash
# LG2
ros2 launch ffw_bringup ffw_lg2_leader_ai.launch.py use_hand_preset:=true style:=advanced
```

`use_hand_preset`의 기본값은 `false`이다. `true`로 실행하면 핸드 프리셋 노드를
함께 시작하고, 리더의 팔 궤적 출력에서 그리퍼 관절을 제외한다.
원본 `/leader/joint_states`에는 그리퍼 관절이 계속 포함된다.

이 매뉴얼은 `style:=advanced`로 조이스틱 버튼을 사용해 teleop source와 팔 enable을
전환한다. 양쪽 버튼을 0.5초 이상 누르면 source를 전환한다. 최신 main의 기본값은
`style:=default`이며 양쪽 그리퍼 트리거로 teleop을 시작/정지한다.
핸드 입력으로 트리거를 조작하는 시험에서는 위 예시처럼 `advanced`를 명시한다.
`advanced`에서 source 선택만으로 핸드는 활성화되지 않는다. 각 팔을 enable하면
해당 핸드도 함께 활성화된다. `default`에서는 양쪽 트리거를 `trigger_duration`
(기본 2초) 동안 유지하면 teleop source 선택 후 양팔과 양손이 함께 활성화된다.
양쪽 트리거를 모두 놓은 뒤 같은 동작을 반복하면 양팔과 양손이 일시정지/재개된다.

하드웨어 bringup은 리더를 초기화하고 A2 트리거의 중립 위치를 명령한다.
실제 follower까지 연결한 상태에서는 유효 리더 입력과 같은 쪽 팔 궤적이
수신되면 핸드 명령을 발행하므로, 핸드와 주변 물체가 접촉하지 않도록 준비한다.

새 터미널에서 환경을 설정한 후 확인한다.

```bash
ros2 node list --no-daemon
ros2 topic echo /leader/joint_states sensor_msgs/msg/JointState --once --no-daemon
ros2 service type /hand_preset_controller/set_preset
```

기대 결과:

- `/hand_preset_controller` 노드가 보인다.
- `/leader/joint_states`에 `gripper_l_joint1`, `gripper_r_joint1`이 포함된다.
- 트리거를 움직이면 해당 관절의 `position`이 변한다.
- 서비스 타입은 `robotis_interfaces/srv/SetPreset`이다.

이 시점에는 리더 joint state만 들어오고 팔 궤적이 나오지 않으면 핸드 출력도
없어야 한다. A2와 LG2의 팔 raw 출력은 `/source`의 유효한 `teleop` heartbeat가 필요하고,
joint 모드에서는 follower 피드백도 필요하다.
6절의 follower bringup 후 아래 절차로 A2의 teleop source를 선택한다.

```bash
ros2 service call /set_source std_srvs/srv/SetBool "{data: true}"
ros2 topic echo /source std_msgs/msg/String --no-daemon
```

`teleop`을 확인한 뒤 양팔과 핸드를 활성화하려면 다음으로 양팔을 enable한다.

```bash
ros2 service call /set_enabled robotis_interfaces/srv/SetTeleoperation \
  "{target_arm: both, enabled: true}"
```

A2/LG2의 raw 팔 궤적은 개별 팔 disable 상태에서도 발행될 수 있다.
핸드는 `/control/status`의 `requested_arms`와 `active_arms`에 해당 팔이 모두
포함되어야 출력한다. 아직 활성화되지 않았거나 disable한 팔의 핸드는 출력하지 않는다.
Cyclo가 error 상태이면 양손 출력을 중지한다. 프리셋 선택 서비스는 비활성 상태에서도
설정을 바꿀 수 있지만, 서비스 호출만으로 핸드를 활성화하지는 않는다.
현재 팔 토픽이 나오는지 확인하려면 A2는 다음을 사용한다.

```bash
ros2 topic echo /leader/joint_trajectory_command_broadcaster_left/raw_joint_trajectory \
  trajectory_msgs/msg/JointTrajectory --once --no-daemon
```

LG2도 같은 경로의 `raw_joint_trajectory`를 사용한다.
좌우 핸드 출력은 각각 다음 토픽에서 확인한다.

```bash
ros2 topic echo /leader/joint_trajectory_command_broadcaster_left_hand/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory --once --no-daemon

ros2 topic echo /leader/joint_trajectory_command_broadcaster_right_hand/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory --once --no-daemon
```

각 메시지는 `finger_l_joint1`~`finger_l_joint20` 또는
`finger_r_joint1`~`finger_r_joint20`과 20개 `positions`를 포함해야 한다.
연속 변화를 보려면 `--once`를 빼고 트리거를 천천히 조작한다.
`publish_rate`는 기본 최대 50 Hz이며 팔 궤적 수신 주기와 장비 부하에 따라
실제 발행 주기는 낮아질 수 있다. 한쪽 팔 궤적만 수신되면 해당 손만 출력해야 한다.
팔 궤적이 멈춘 뒤에는 리더 joint state가 계속 들어와도 핸드 출력이 멈춰야 한다.

follower가 없으면 관절 제한 정보를 받지 못했다는 경고가 나올 수 있다.
현재 구현은 이 경우에도 **관절 제한으로 clamp하지 않은 명령을 발행**한다.
실제 핸드 동작 시험 전에는 6절의 제한 정보 수신을 확인한다.

## 6. 2차 시험: SH5/BH5 실제 핸드 연동

기존 follower가 실행 중이면 중복 실행하지 않고 해당 시스템을 사용한다.
새로 실행해야 한다면 장치 경로와 로봇 모델을 확인한 후 아래 중 하나를 사용한다.
예시는 follower 포트가 `/dev/follower`이고 카메라·LiDAR가 필요 없는 경우다.

```bash
# SH5
ros2 launch ffw_bringup ffw_sh5_follower_ai.launch.py \
  port_name:=/dev/follower launch_cameras:=false launch_lidar:=false start_rviz:=false
```

```bash
# BH5
ros2 launch ffw_bringup ffw_bh5_follower_ai.launch.py \
  port_name:=/dev/follower launch_cameras:=false start_rviz:=false
```

이 launch들은 핸드 외에 로봇 전체를 bringup하며 초기 위치 동작도 포함할 수 있다.
일반 로봇 운용 절차에 따라 시작한다. 리더와 follower를 다른 컨테이너에서
실행한다면 두 컨테이너의 토픽이 서로 보이는지 먼저 확인한다.

```bash
ros2 control list_controllers -c /controller_manager
ros2 topic echo /robot_description std_msgs/msg/String --once --no-daemon
```

`hand_l_controller`, `hand_r_controller`가 `active`여야 한다.
핸드 프리셋 노드 로그에 `Hand joint limits loaded from robot description`이
출력되는지 확인한다. 일부 핸드 관절의 제한이 없다는 경고가 있으면 해당 관절은
clamp되지 않으므로 모델/토픽 설정을 확인한다.

이후 리더 트리거를 천천히 조작하여 좌우 핸드가 각각 release→grasp 자세로
연속 이동하는지 확인한다. 프리셋 변경은 현재 트리거 위치에 대응하는 자세를
바꾸므로 첫 시험은 물체를 잡지 않은 상태에서 진행한다.

## 7. 프리셋 선택 서비스 시험

기본 프리셋 파일은 `ffw_bringup/config/common/hand_presets.yaml`이다.
기본 선택은 `envelope`이며 제공되는 ID는 다음과 같다.

- `1`: `envelope`
- `2`: `pinch`
- `3`: `3finger`

### 양손을 pinch로 변경

```bash
ros2 service call /hand_preset_controller/set_preset \
  robotis_interfaces/srv/SetPreset \
  "{target_arm: both, left_preset_id: 2, right_preset_id: 2}"
```

`accepted: true`와 0보다 큰 `transition_id`가 반환되어야 한다.
이 값은 선택 요청이 수락됐다는 의미이며 실제 동작 완료 응답은 아니다.
기본 `preset_switch_duration`은 1초이며, 입력이 계속 수신될 때 이전/새 프리셋을
이 시간 동안 보간한다. 노드 재시작 시 `transition_id`는 다시 시작한다.

### 한쪽만 변경

```bash
ros2 service call /hand_preset_controller/set_preset \
  robotis_interfaces/srv/SetPreset \
  "{target_arm: left, left_preset_id: 3}"
```

왼손만 `3finger`로 바뀌고 오른손은 기존 프리셋을 유지해야 한다.
`target_arm: left`일 때 `right_preset_id`는 사용하지 않는다.

### 좌우에 서로 다른 프리셋 적용

```bash
ros2 service call /hand_preset_controller/set_preset \
  robotis_interfaces/srv/SetPreset \
  "{target_arm: both, left_preset_id: 1, right_preset_id: 2}"
```

왼손은 `envelope`, 오른손은 `pinch`가 되어야 한다.
이후 양쪽 트리거를 각각 움직이며 자세를 확인한다.

### 잘못된 ID 요청

```bash
ros2 service call /hand_preset_controller/set_preset \
  robotis_interfaces/srv/SetPreset \
  "{target_arm: both, left_preset_id: 2, right_preset_id: 999}"
```

`accepted: false`가 반환되고 양손 모두 기존 프리셋을 유지해야 한다.
한쪽 ID만 유효해도 요청 전체가 거절된다. ID `0` 역시 유효한 프리셋이 아니다.

핸드 서비스는 **`/hand_preset_controller/set_preset`**이다.
Cyclo의 **`/set_preset`**은 팔 프리셋용이므로 구분한다.
별도 `GetHandPresets` 서비스는 없으며 사용 가능한 프리셋은 YAML과 시작 로그에서 확인한다.

## 8. 사용자 YAML 변경과 다시 읽기

### 8.1. RViz 슬라이더로 자세를 만들고 프리셋 기록하기

`robotis_hand_description`의 핸드 표시 launch를 실행하고, 함께 열리는
`joint_state_publisher_gui` 슬라이더로 자세를 만든다. RViz에서 자세를 확인한 뒤
`record_hand_preset`으로 `/joint_states`의 20개 관절 위치를 한 번 읽어 저장한다.
실제 핸드 하드웨어는 필요하지 않다. 이 절차는 현재 ROS domain ID와 RMW 설정을
그대로 사용하며, 두 터미널 모두 같은 환경을 source한다.
슬라이더의 `/joint_states`가 다른 follower의 joint state와 섞이지 않도록
자세 편집 중에는 해당 토픽을 발행하는 follower launch를 종료한다.

**터미널 1 — 핸드와 슬라이더 실행:**

```bash
ros2 launch robotis_hand_description hx5_d20_right.launch.py
```

왼손으로 자세를 만들려면 `hx5_d20_left.launch.py`를 사용한다.
슬라이더를 조절하여 먼저 grasp 자세를 만든다.

**터미널 2 — grasp 기록:**

```bash
ros2 run ffw_preset_controller record_hand_preset \
  --file /root/ros2_ws/src/ai_worker/ffw_bringup/config/common/hand_presets.yaml \
  --name my_preset --id 4 --pose grasp
```

**터미널 1**의 슬라이더를 release 자세로 바꾼 뒤, **터미널 2**에서 같은 이름과
ID로 release를 기록한다.

```bash
ros2 run ffw_preset_controller record_hand_preset \
  --file /root/ros2_ws/src/ai_worker/ffw_bringup/config/common/hand_presets.yaml \
  --name my_preset --id 4 --pose release
```

필수 인자와 저장 동작:

- `--file`: 수정할 기존 YAML 파일. 위 예시는 bringup의 소스 설정 파일이다.
- `--name`: 추가하거나 수정할 프리셋 이름.
- `--id`: 중복되지 않는 정수 ID(1~65535). 기존 이름을 수정할 때는 기존 ID와 같아야 한다.
  예시의 `4`가 다른 프리셋에 사용 중이면 사용하지 않는 번호를 선택한다.
- `--pose grasp|release`: 이번에 기록할 자세. 두 자세는 같은 이름과 ID로 각각 기록한다.
- 새 프리셋은 YAML의 `default_preset`을 복사하여 만든다. 아직 기록하지 않은 반대 자세는
  템플릿 값을 유지하므로 두 자세를 모두 기록한다. 다른 템플릿은 `--base pinch`처럼 지정한다.
- 관절 이름으로 좌우를 자동 판별하고, 메시지의 관절 순서와 관계없이 손가락별로 정렬한다.
  왼손을 기록하면 `mirrored_joints`의 부호 반전을 되돌려 오른손 기준으로 저장한다.
  런타임에서는 기존 프리셋과 동일하게 왼손 자세를 미러링하여 사용한다.
- 같은 자세에 `left_grasp` 또는 `left_release` override가 있으면 해당 override를 제거하여
  새로 기록한 공통 자세가 양손에 적용되게 한다. 반대 자세와 다른 프리셋은 유지한다.
- 20개 관절의 유한한 위치 값이 모두 수신되어야 저장한다. 기본 대기 시간은 10초이며,
  실패하면 YAML을 수정하지 않는다. 저장 전에 전체 YAML을 검증하고 파일을 원자적으로 교체한다.
  `.bak` 등 백업 파일은 생성하지 않는다.

선택 인자:

- 양손이 모두 포함된 토픽에서는 `--side left` 또는 `--side right`로 기록할 손을 선택한다.
- 다른 토픽을 읽으려면 `--topic /원하는/joint_states`를 지정한다.
- 표시 launch에 `prefix:=hand_`를 전달했다면 기록 명령에도 `--prefix hand_`를 전달한다.
- 수신 대기 시간은 `--timeout 20`처럼 변경한다.

기록 완료 후 터미널 1의 표시 launch를 `Ctrl+C`로 종료하고 follower/리더 시험을 재개한다.
실행 중인 프리셋 컨트롤러가 **기록한 파일과 같은 경로**를 읽도록
`hand_presets_file`을 지정해야 한다. 설치 경로에 복사된 YAML은 소스 파일을 기록해도
자동으로 바뀌지 않으므로 bringup을 다시 빌드하거나 저장한 소스 경로를 직접 지정한다.
아래 8.2절의 절차로 파일을 지정하고 reload한 뒤, 7절의 선택 서비스에서 새 ID를 사용한다.

### 8.2. 사용할 파일 지정, 수동 편집과 reload

기본 YAML을 복사하여 편집용 파일로 사용한다. 컨테이너 삭제 후에도 보존하려면
`/workspace` 등 실제 영속 마운트 경로에 저장한다.

```bash
cp /root/ros2_ws/src/ai_worker/ffw_bringup/config/common/hand_presets.yaml \
  /workspace/hand_presets_test.yaml
```

실행 중인 리더 launch를 종료한 뒤, 편집용 파일을 지정해 다시 실행한다.

```bash
ros2 launch ffw_bringup ffw_a2_leader_ai.launch.py \
  use_hand_preset:=true style:=advanced \
  hand_presets_file:=/workspace/hand_presets_test.yaml \
  hand_initial_preset:=envelope
```

LG2 시험이면 launch 파일 이름만 `ffw_lg2_leader_ai.launch.py`로 바꾼다.
`hand_initial_preset`은 숫자 ID가 아닌 YAML의 프리셋 **이름**이다.

YAML 형식:

- 각 프리셋에 중복되지 않는 정수 `id`를 지정한다. 허용 범위는 1~65535다.
- `release`, `grasp`에 `thumb`, `index`, `middle`, `ring`, `little`을 모두 작성한다.
- 손가락마다 관절 위치 4개를 **라디안**으로 작성한다. 전체는 20개다.
- 기본 값은 오른손 기준이며 `mirrored_joints`에 지정한 관절은 왼손에서 부호가 반전된다.
- 왼손 자세를 별도로 지정하려면 `left_release`, `left_grasp`를 사용한다.
- `default_preset`은 정의된 프리셋 이름이어야 한다.

작은 각도 변경으로 시작하고, 저장 후 다음 서비스를 호출한다.

```bash
ros2 service call /hand_preset_controller/reload_presets std_srvs/srv/Trigger '{}'
```

정상 파일은 `success: true`가 반환되고 현재 선택된 프리셋에 변경값이 반영된다.
잘못된 파일은 `success: false`가 반환되며 이전 유효 프리셋을 유지한다.
예를 들어 편집용 파일의 `pinch.id`를 `1`로 바꿔 중복 ID를 만든 뒤 다시 읽으면
거절되어야 한다. 시험 후 `2`로 복구하고 다시 읽는다.

현재 선택된 프리셋을 파일에서 삭제하면 reload 시 `default_preset`으로 전환된다.
실제 핸드가 움직일 수 있으므로 해당 시험은 먼저 출력 메시지로 확인한다.

## 9. 문제 해결과 시험 판정

### `/joint_states` 또는 raw arm trajectory가 보이지 않음

리더 입력은 `/leader/joint_states`이며 `/joint_states`는 follower 피드백이다.
핸드 출력은 이름에 **`_left_hand` / `_right_hand`**가 들어간 토픽이다.
`..._left/raw_joint_trajectory`는 팔 제어용이므로 핸드 명령 확인에 사용하지 않는다.

핸드 출력은 같은 쪽 팔 궤적의 수신으로 동작하므로, 팔 출력이 없으면 먼저
팔 broadcaster의 조건을 확인한다. A2는 follower/Cyclo와 `/source`의 유효한
`teleop` heartbeat, joint 모드의 follower 피드백이 필요하다.
리더 joint state 수신이나 프리셋 선택 서비스 성공만으로 핸드 출력이 생기지 않는다.

### 서비스가 없거나 예전 타입이 보임

`use_hand_preset:=true`, 노드 실행 여부, `ROS_DOMAIN_ID`, Zenoh 연결과
`ros2 pkg prefix ffw_preset_controller`를 확인한다.
예전 `SelectHandPreset`, `GetHandPresets`는 이 버전에서 사용하지 않는다.
테스트 설치 경로를 source한 새 터미널에서 다시 확인한다.

### 프리셋을 바꿨지만 움직이지 않음

`accepted: true`만으로 물리 동작을 판정하지 않는다. 핸드 명령 토픽의
`positions` 변화, follower 컨트롤러의 `active` 상태, 해당 토픽의 수신 연결을 확인한다.
선택한 프리셋들의 현재 트리거 위치에 대응하는 값이 같으면 눈에 띄는 변화가
작을 수 있으므로 트리거 전체 범위에서 비교한다.

### `no recent ... state` 경고

기본 0.5초 이상 해당 손의 유효한 리더 그리퍼 입력이 없으면 그 손의 새 명령
발행을 멈춘다. 통신 복구 시 새 입력으로 재개한다.
이는 비상정지가 아니며 follower가 이전 명령을 취소하거나 힘을 해제한다는 의미도 아니다.

### 양쪽 조이스틱을 눌러도 `/toggle_source`가 없다는 경우

`/toggle_source`와 `/set_source`는 follower의 Cyclo 노드가 제공한다.
follower launch에서 컨트롤러 활성화가 실패하면 Cyclo가 시작되지 않을 수 있다.
BH5 Gazebo는 실제 핸드의 압력 센서 state interface를 제공하지 않으므로
`pressure_l_broadcaster`, `pressure_r_broadcaster`를 실행하지 않는다.
이전 launch에서 압력 broadcaster 활성화 실패가 보이면 수정된 bringup을 빌드하고
Gazebo launch를 다시 시작한다. `/set_enabled`는 A2 리더가 제공하므로 리더도
정상적으로 실행되어 있어야 한다.

### Gazebo에서 `could not connect to display`

GUI를 사용하려면 컨테이너에 호스트의 `DISPLAY`, `/tmp/.X11-unix` 소켓과
유효한 X11 인증 파일을 전달하고, 컨테이너의 `XAUTHORITY`를 해당 파일로 설정한다.
IPC가 분리된 컨테이너에서는 `QT_X11_NO_MITSHM=1`을 설정한다.

이 저장소의 로컬 개발 컨테이너(`docker-compose.isolated.yml`)는 호스트 데스크톱
터미널에서 `./docker/start-isolated.sh`로 시작한다. 이 스크립트는 현재 세션의
인증 정보를 갱신하며, 로그아웃/로그인 후 GUI 인증이 실패하면 다시 실행한다.
이 설정은 로컬 스냅샷 이미지 `ai-worker-dev:display-ready`를 사용하므로 일반
컨테이너를 준비하는 팀원에게 해당 이미지가 있다고 가정하지 않는다.
GPU 전달 없이 소프트웨어 렌더링을 사용하므로 화면 갱신이 느릴 수 있다.

GUI 실행 명령:

```bash
ros2 launch ffw_bringup ffw_bg2_follower_ai_gazebo.launch.py gui:=true rviz:=false
```

화면이 필요 없으면 아래 headless 실행으로 팔 연동을 시험할 수 있다.

```bash
ros2 launch ffw_bringup ffw_bg2_follower_ai_gazebo.launch.py gui:=false rviz:=false
```

BG2에는 핸드가 없다. SH5/BH5 Gazebo launch는 `rviz:=false`로 RViz를 끌 수 있다.
현재 `gui` 인자는 제공하지 않으므로 `gui:=false`는 적용되지 않는다.

### 종료

리더/핸드 노드와 follower launch 터미널에서 각각 `Ctrl+C`로 종료한다.
명령 발행 중지만으로 핸드가 펼쳐지거나 토크가 꺼진다고 가정하지 말고,
실제 로봇은 일반 정지 절차를 따른다.

## 10. 인계할 테스트 결과

다음 항목의 성공/실패와 관찰 결과를 기록한다.

- 사용 이미지, 세 저장소 커밋, 리더 종류, follower 종류, 연결 포트.
- 리더 트리거 입력과 좌우 20관절 출력 확인.
- 팔 궤적이 없으면 핸드도 발행하지 않는지, 좌우가 각각 대응하는지 확인.
- 팔 궤적 중단 시 핸드 발행 중단, 재개 시 새 입력으로 다시 출력하는지 확인.
- `envelope`, `pinch`, `3finger`에서 release/중간/grasp 자세 확인.
- 왼손만 변경, 양손 서로 다른 프리셋, 부드러운 전환 확인.
- 잘못된 ID 요청 시 양손 상태 유지 확인.
- 사용자 YAML reload 성공 및 오류 시 이전 프리셋 유지 확인.
- RViz 슬라이더로 grasp/release를 기록하고 좌우 자동 판별과 미러링 결과 확인.
- 실제 로봇 시험이라면 핸드 관절 제한 수신과 follower 동작 여부.
- 실패 시 실행 명령, 서비스 응답, 관련 로그, 토픽 publisher/subscriber 수.

코드 수준에서는 빌드, 단위 테스트, 컨테이너 내부 실제 ROS 서비스 호출과
명령 메시지 수신을 확인했다. A2 하드웨어 joint state 수신과 핸드 명령 발행도
확인했다. SH5/BH5 **실제 핸드의 전체 프리셋 동작 검증은 인계받는 테스트의 범위**다.

자동 테스트를 재실행하려면:

```bash
cd /root/ros2_ws
colcon test --build-base build_hand_test --install-base install_hand_test \
  --packages-select ffw_preset_controller --event-handlers console_direct+
colcon test-result --test-result-base build_hand_test/ffw_preset_controller --verbose
```
