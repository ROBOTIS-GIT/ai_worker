# FFW Follower 모듈화 개발 문서

- 상태: URDF/Gazebo/description launch 모듈화 구현 완료, ros2_control/bringup launch 통합 대기
- 최종 수정: 2026-09-03

## 1. 목적

로봇별로 나뉜 follower launch와 description을 하나의 조합형 구조로 통합한다.

최종 실행 인터페이스는 다음과 같다.

```bash
ros2 launch ffw_bringup ffw_follower_ai.launch.py robot:=sg2
```

`robot` 값은 설정 파일에서 body, base, gripper 조합으로 해석한다. Launch와 Xacro 안에 로봇 이름별 분기를 중복해서 만들지 않는다.

## 2. 대상 로봇과 조합

| Robot | Body | Base | Gripper | 카메라 | Base 센서 |
|---|---|---|---|---|---|
| `bg2` | `ffw` | `ffw_base` | `rh_p12_rn_a` (2지) | Head ZED Mini, Wrist D405 | 없음 |
| `sg2` | `ffw` | `ffw_swerve` | `rh_p12_rn_a` (2지) | Head ZED Mini, Wrist D405 | IMU, Dual LiDAR |
| `bh5` | `ffw` | `ffw_base` | `hx5_d20_rev2` (5지) | Head ZED Mini, Wrist D405 | 없음 |
| `sh5` | `ffw` | `ffw_swerve` | `hx5_d20_rev2` (5지) | Head ZED Mini, Wrist D405 | IMU, Dual LiDAR |
| `f1` | `ffw_f` | `ffw_base_f` | `hx2_d1` (2지) | Head D455, Wrist D401 | 없음 |
| `f2` | `ffw_f` | `ffw_swerve_f` | `hx2_d1` (2지) | Head D455, Wrist D401 | IMU, Dual LiDAR |

현재 존재하는 위 6개 조합만 허용한다. 별도 로봇 정의 없이 body, base, gripper를 임의로 조합하는 기능은 제공하지 않는다.

## 3. 로봇 설정

로봇 조합의 단일 기준은 `ffw_description/config/follower_robots.yaml`이다.

```yaml
robots:
  bg2: {model: ffw_bg2_rev4_follower, body: ffw,   base: ffw_base,     gripper: rh_p12_rn_a}
  sg2: {model: ffw_sg2_rev1_follower, body: ffw,   base: ffw_swerve,   gripper: rh_p12_rn_a}
  bh5: {model: ffw_bh5_rev1_follower, body: ffw,   base: ffw_base,     gripper: hx5_d20_rev2}
  sh5: {model: ffw_sh5_rev1_follower, body: ffw,   base: ffw_swerve,   gripper: hx5_d20_rev2}
  f1:  {model: ffw_f1_follower,        body: ffw_f, base: ffw_base_f,   gripper: hx2_d1}
  f2:  {model: ffw_f2_follower,        body: ffw_f, base: ffw_swerve_f, gripper: hx2_d1}
```

카메라 종류, LiDAR 사용 여부, controller 목록처럼 component에서 결정할 수 있는 값은 중복해서 저장하지 않는다.

- `body=ffw`: ZED Mini head, D405 wrist
- `body=ffw_f`: D455 head, D401 wrist
- `base=ffw_swerve` 또는 `ffw_swerve_f`: steering/drive, IMU, dual LiDAR 활성화
- `gripper=hx5_d20_rev2`: 5지 hand controller, effort controller, pressure broadcaster 활성화

## 4. 모듈 경계

### 4.1 Body

Body는 다음 요소를 소유한다.

- Left/right arm 7축
- Lift
- Head 2축
- Body, arm, head geometry와 inertial
- Head/wrist camera description

공통 Xacro는 다음 위치에서 관리한다.

- `urdf/follower/body/ffw.urdf.xacro`: BG2/SG2/BH5/SH5 body
- `urdf/follower/body/ffw_f.urdf.xacro`: F1/F2 body

두 body는 geometry, inertial, joint origin과 카메라 구성이 다르므로 하나의 거대한 parameterized macro로 합치지 않는다.

### 4.2 Base

Base 구현은 물리 값이 같은 로봇끼리 다음 네 파일로 나눈다.

- `ffw_base`: BG2/BH5
- `ffw_base_f`: F1
- `ffw_swerve`: SG2/SH5
- `ffw_swerve_f`: F2

Legacy와 F-series는 base mesh, inertial과 wheel origin이 다르므로 하나의 parameterized base로 합치지 않는다. Wheel radius는 둘 다 `0.0865`이다. `ffw_swerve`는 SG2 정의를 기준으로 하며, 기존 SH5와 비교하면 LiDAR 두 링크에 각각 0.1 kg의 inertial과 시각 cylinder가 추가된다. 그 외 SH5의 link/joint origin과 물성치는 동일하다.

F2 controller의 `module_x_offsets`는 URDF steering 축 X 좌표 `[0.137119, 0.137119, -0.288133]`에 맞춘다. F2 steering 3축의 `Acceleration Limit`은 중복 없이 `12592`를 사용한다.

Swerve base가 소유하는 항목은 다음과 같다.

- `left/right/rear_wheel_steer`
- `left/right/rear_wheel_drive`
- IMU와 LED interface
- Left/right LiDAR link 및 Gazebo sensor
- Swerve controller와 초기 steering controller
- Dual LiDAR launch와 laser merger

### 4.3 Gripper

Gripper 모델은 다음과 같다.

- `rh_p12_rn_a`: legacy 2지
- `hx2_d1`: F-series 2지
- `hx5_d20_rev2`: BH5/SH5 5지 및 pressure sensor

5지 hand description의 소유권은 `robotis_hand_description` 패키지에 둔다.

- SH5처럼 BH5도 hand URDF와 ros2_control joint 정의를 `robotis_hand_description`에서 include한다.
- Gazebo는 `gazebo/follower/gripper/hx5_d20_rev2.gazebo.xacro`가 담당한다. 외부 좌우 Gazebo macro는 한 로봇에서 함께 호출하면 `hand_trans1~20` 이름이 중복되므로, 동일한 물리값을 사용하면서 이름을 `hand_l_*`, `hand_r_*`로 구분한다.
- 기존 BH5 내부 HX5-D20 URDF와 ros2_control hand 정의는 제거했으며 BH5/SH5 모두 외부 rev2 description을 사용한다.
- 중복된 로컬 HX5-D20 mesh는 제거하고 `robotis_hand_description`의 mesh를 사용한다.
- `ffw_description/package.xml`에 `robotis_hand_description` 실행 의존성을 선언한다.
- BH5와 SH5 모두 `hx5_d20_rev2`를 사용한다.

모든 gripper controller는 arm controller에서 분리한다.

목표 controller 구성:

```text
arm_l_controller: arm_l_joint1~7
arm_r_controller: arm_r_joint1~7

2지:
  gripper_l_controller: gripper_l_joint1
  gripper_r_controller: gripper_r_joint1

5지:
  hand_l_controller: finger_l_joint1~20
  hand_r_controller: finger_r_joint1~20
  effort_l_controller
  effort_r_controller
  pressure_l_broadcaster: BH5/SH5
  pressure_r_broadcaster: BH5/SH5
```

Controller는 분리하지만 같은 `/dev/follower` 버스를 사용하는 joint를 별도 ros2_control hardware system으로 분리하지 않는다.

## 5. ros2_control 구조

의미상 component 경계와 실제 통신 버스 경계를 구분한다.

```text
ros2_control/follower/
├── ffw_follower.ros2_control.xacro
├── body/
│   ├── ffw.ros2_control.xacro
│   └── ffw_f.ros2_control.xacro
├── base/
│   ├── ffw_base.ros2_control.xacro
│   ├── ffw_base_f.ros2_control.xacro
│   ├── ffw_swerve.ros2_control.xacro
│   └── ffw_swerve_f.ros2_control.xacro
└── gripper/
    ├── rh_p12_rn_a.ros2_control.xacro
    ├── hx2_d1.ros2_control.xacro
    └── hx5_d20_rev2.ros2_control.xacro
```

Body와 Gripper 파일은 hardware system을 만들지 않고 joint/GPIO 정의만 제공한다. Base 파일은 `/dev/follower`에 들어갈 steering 정의와 별도 drive/sensor system을 제공한다. 최상위 파일이 선택된 정의를 다음 통신 경계로 조립한다.

```text
follower hardware system (/dev/follower)
├── body joints
├── 선택된 gripper joints
└── swerve일 때 steering joints

base hardware system (/dev/ttyUSB0, swerve 전용)
└── drive joints

sensor hardware system (/dev/ttyUSB1, swerve 전용)
└── IMU / LED
```

Swerve의 steering joint와 drive joint는 서로 다른 hardware system에 있지만 모두 base component가 소유한다. Base Xacro는 필요한 joint 정의를 각 hardware system에 제공한다.

동일한 serial port를 두 Dynamixel hardware system이 동시에 열지 않도록 gripper 전용 hardware system은 만들지 않는다.

현재 저장소에서 `/dev/ttyUSB1` sensor system은 SG2/SH5/F2에만 정의되어 있다. 실제 BG2에는 LED가 있으나 main에는 통신 포트와 ros2_control 연결이 없으므로 하드웨어 연결을 확인한 뒤 반영한다.

`use_sim`은 최상위 Xacro argument로 한 번만 받고 ros2_control 매크로 parameter로 전달한다. 각 매크로 내부에서는 전역 `$(arg use_sim)`을 다시 읽지 않고 전달받은 `${use_sim}`을 사용하도록 모든 로봇을 통일한다.

`ffw_robot_manager`의 배터리 전압 조회를 위해 모든 로봇의 `dxl1`, `dxl61`에서 `Present Input Voltage`를 제공한다.

Head 2축(`dxl62`) 제어 gain은 SG2 기준인 P/I/D `800/200/200`, Feedforward 1st/2nd `20/20`으로 모든 로봇을 통일한다.

BH5도 다른 follower와 동일하게 상태 topic과 제어 service를 `ffw_follower/*` 이름으로 제공한다. `ffw_robot_manager`는 `/ffw_follower/dxl_state`를 구독한다.

현재 하드코딩된 포트는 launch argument가 실제 ros2_control parameter까지 전달되도록 수정한다. 기본값은 현재 값을 유지한다.

## 6. Leader trajectory 분리

현재 2지 follower는 arm 7축과 gripper 1축을 하나의 trajectory로 받는다. Gripper controller 분리 후에는 leader 출력도 분리해야 한다.

```text
arm left trajectory      -> arm_l_controller
arm right trajectory     -> arm_r_controller
gripper left trajectory  -> gripper_l_controller 또는 hand_l_controller
gripper right trajectory -> gripper_r_controller 또는 hand_r_controller
```

기존 `joint_trajectory_command_broadcaster`가 arm과 gripper group을 각각 발행하도록 확장한다. 단순히 follower launch에서 topic만 분리하면 publisher가 없으므로 동작하지 않는다.

현재 BH5/SH5 launch가 참조하는 `left_hand`와 `right_hand` trajectory publisher는 저장소 내 broadcaster 구현에서 확인되지 않으므로 통합 과정에서 함께 정리한다.

## 7. 공통 초기 자세 파일

초기 자세는 로봇별로 유지하지 않고 하나의 공통 파일에 모든 executor 설정을 넣는다.

예정 파일:

```text
ffw_bringup/config/common/ffw_follower_initial_positions.yaml
```

파일에는 다음 section을 모두 둔다.

```text
body:
  arm_l_joint_trajectory_executor
  arm_r_joint_trajectory_executor
  head_joint_trajectory_executor
  lift_joint_trajectory_executor

2지:
  gripper_l_joint_trajectory_executor
  gripper_r_joint_trajectory_executor

5지:
  hand_l_joint_trajectory_executor
  hand_r_joint_trajectory_executor

swerve:
  swerve_steering_joint_trajectory_executor
```

Launch는 선택된 component에 필요한 executor만 실행한다. 사용하지 않는 section이 공통 YAML에 있어도 해당 node를 실행하지 않으므로 영향을 주지 않는다.

기존 로봇별 초기 자세 값의 완전한 보존은 목표가 아니다. 공통 자세는 다음 조건만 만족하면 된다.

- 모든 대상 로봇의 joint limit 안에 있을 것
- 시작 동작 중 명백한 self-collision이 없을 것
- Swerve steering은 `[0.0, 0.0, 0.0]`으로 초기화할 것

## 8. 통합 URDF 구조

최상위 description은 `ffw_description/urdf/follower/ffw_follower.urdf.xacro` 하나로 통합한다.

```text
ffw_follower.urdf.xacro
├── body 선택
├── base 선택
├── gripper 선택
├── ros2_control hardware system 조립
└── Gazebo component 조립
```

최상위 Xacro는 `robot` argument를 받아 `follower_robots.yaml`을 읽고 body/base/gripper 모듈을 선택한다. Launch는 로봇 이름만 넘기며 조합 규칙을 중복해서 갖지 않는다.

`ffw_follower.urdf.xacro` 소스 파일은 하나만 저장한다. Launch 실행 시 Xacro 결과를 XML 문자열로 만들어 `robot_description` parameter로 사용하며, 생성 결과를 별도 `.urdf` 파일로 저장하지 않는다.

```text
ros2 launch ... robot:=sg2
        ↓
follower_robots.yaml 조회
        ↓
body=ffw, base=ffw_swerve, gripper=rh_p12_rn_a
        ↓
ffw_follower.urdf.xacro 실행
        ↓
robot_description 문자열
        ├── robot_state_publisher
        └── ros2_control_node
```

Launch는 기존 `Command([xacro, ...])` 패턴으로 공통 Xacro에 `robot`을 전달한다.

```python
robot_description_content = Command([
    FindExecutable(name='xacro'),
    ' ',
    PathJoinSubstitution([
        FindPackageShare('ffw_description'),
        'urdf', 'follower', 'ffw_follower.urdf.xacro',
    ]),
    ' robot:=', LaunchConfiguration('robot'),
    ' use_sim:=', LaunchConfiguration('use_sim'),
    ' use_mock_hardware:=', LaunchConfiguration('use_mock_hardware'),
])
```

공통 Xacro는 설정에 선택된 파일을 동적으로 include하고, 각 파일이 제공하는 동일한 macro 이름을 호출한다. Include만으로 link나 joint가 생성되지는 않는다.

```xml
<xacro:arg name="robot" default=""/>
<xacro:property name="robot_name" value="$(arg robot)"/>
<xacro:property name="robot_config"
                value="${xacro.load_yaml('.../follower_robots.yaml')['robots'][robot_name]}"/>

<xacro:include filename=".../body/${robot_config['body']}.urdf.xacro"/>
<xacro:include filename=".../base/${robot_config['base']}.urdf.xacro"/>
<xacro:include filename=".../gripper/${robot_config['gripper']}.urdf.xacro"/>

<xacro:follower_base prefix=""/>
<xacro:follower_body parent="base_link" prefix="">
  <origin xyz="${body_mount_xyz}" rpy="${body_mount_rpy}"/>
</xacro:follower_body>
<xacro:follower_gripper prefix=""/>
```

기존 로봇별 최상위 follower URDF는 제거했다. 기존 `ffw_bringup` launch 변경은 원복했으며, ros2_control/config/launch 통합 단계에서 공통 Xacro로 한꺼번에 전환한다.

Gazebo 설정도 URDF와 같은 component 이름과 폴더 구조를 사용한다.

```text
gazebo/follower/
├── body/
│   ├── ffw.gazebo.xacro
│   └── ffw_f.gazebo.xacro
├── base/
│   ├── ffw_base.gazebo.xacro
│   ├── ffw_base_f.gazebo.xacro
│   ├── ffw_swerve.gazebo.xacro
│   └── ffw_swerve_f.gazebo.xacro
└── gripper/
    ├── rh_p12_rn_a.gazebo.xacro
    ├── hx2_d1.gazebo.xacro
    └── hx5_d20_rev2.gazebo.xacro
```

각 component는 `follower_body_gazebo`, `follower_base_gazebo`, `follower_gripper_gazebo`라는 공통 호출 이름만 제공한다. 물리값과 transmission 정의는 공통 파일로 빼지 않고 각 구현 파일에 둔다. 현재 값이 같더라도 `ffw`와 `ffw_f`, 각 Base와 Gripper가 독립적으로 변경될 수 있도록 서로 include하지 않는다. Swerve Base가 dual LiDAR sensor를 소유하고 Gripper가 자기 link와 transmission 설정을 소유한다. 최상위 Xacro에는 공통 `gz_ros2_control` plugin만 직접 둔다. 기존 로봇별 Gazebo 파일은 제거했다.

Description 확인용 launch도 하나로 통합한다.

```bash
ros2 launch ffw_description ffw_follower_description.launch.py robot:=sg2
```

`robot`은 기본값 없는 필수 인자이며 `use_gui`만 선택적으로 받는다. RViz 설정은 로봇별 파일을 유지하고 `robot` 이름으로 `ffw_<robot>.rviz`를 선택한다.

## 9. 통합 Launch 책임

`ffw_follower_ai.launch.py`는 다음만 담당한다.

1. 필수 `robot` argument 전달
2. 공통 Xacro 실행
3. Xacro와 같은 `robot` 값으로 controller 설정 선택
4. 공통 controller 및 executor 실행
5. 선택된 base와 gripper의 추가 controller/node 실행

공통 launch argument:

- `robot`: `bg2`, `sg2`, `bh5`, `sh5`, `f1`, `f2`
- `start_rviz`
- `use_sim`
- `use_mock_hardware`
- `mock_sensor_commands`
- `launch_cameras`
- `launch_lidar`
- `init_position`
- `use_head_eef_tracker`
- 하드웨어 포트 override

잘못된 `robot` 값은 기본 로봇으로 대체하지 않고 즉시 오류로 종료한다.

## 10. 작업 순서

1. 기존 6개 Xacro의 link, joint, ros2_control interface 목록을 기준 결과로 저장한다.
2. `follower_robots.yaml`을 추가하고 6개 로봇만 검증한다.
3. BH5의 hand URDF와 ros2_control 정의를 SH5처럼 `robotis_hand_description` include 방식으로 전환한다. (완료)
4. 기존 body macro를 유지하면서 base/swerve base와 gripper 조립부를 모듈화한다. (완료)
5. 공통 `ffw_follower.urdf.xacro`를 만든다. (완료)
6. ros2_control joint 정의를 body/base/gripper macro로 나누고 hardware system을 조립한다. (완료)
7. 2지 gripper를 arm controller에서 분리한다.
8. Leader broadcaster의 arm/gripper trajectory 출력을 분리한다.
9. 공통 initial position YAML을 만들고 component별 executor를 연결한다.
10. Gazebo description을 component 기준으로 정리한다. (완료)
11. `ffw_follower_ai.launch.py`를 추가하고 6개 로봇을 검증한다.
12. 상위 launch, Docker alias와 s6 service runner를 새 launch로 변경한다.
13. 기존 로봇별 follower launch를 제거한다. 최상위 URDF wrapper는 제거 완료했다.

Launch 통합 전까지 6개 조합을 직접 Xacro로 생성해 URDF와 Gazebo 구성을 검증한다. 기존 6개 launch는 이 단계에서 수정하지 않는다.

## 11. 검증 기준

각 로봇에 대해 다음 명령 형태로 mock hardware smoke test를 수행한다.

```bash
ros2 launch ffw_bringup ffw_follower_ai.launch.py \
  robot:=sg2 \
  use_mock_hardware:=true \
  launch_cameras:=false \
  launch_lidar:=false
```

URDF 조합 자체는 ROS 환경에서 다음 형태로 여섯 로봇을 생성·검증한다.

```bash
for robot in bg2 sg2 bh5 sh5 f1 f2; do
  xacro ffw_description/urdf/follower/ffw_follower.urdf.xacro robot:=$robot \
    | check_urdf /dev/stdin
done
```

완료 조건:

- 6개 robot profile이 모두 Xacro 생성에 성공한다.
- 지원하지 않는 robot 값은 명확한 오류를 출력한다.
- 모든 arm controller는 7개 arm joint만 가진다.
- 모든 gripper는 별도 controller로 spawn된다.
- `base` 타입에는 steering/drive interface와 swerve controller가 없다.
- `swerve` base에는 steering 3축과 drive 3축이 모두 존재한다.
- Swerve steering 초기화 후 drive controller 전환이 성공한다.
- 하나의 공통 initial position YAML로 6개 로봇이 초기화된다.
- Legacy/F-series 카메라 조합이 기존 하드웨어와 일치한다.
- BH5와 SH5의 hand URDF 및 ros2_control 정의가 `robotis_hand_description`에서 제공된다.
- BH5와 SH5의 pressure broadcaster가 유지된다.
- 기존 내부 launch 및 Docker 호출이 새 launch를 사용한다.
- 최종 상태에는 follower launch가 `ffw_follower_ai.launch.py` 하나만 남는다.

## 12. 확인된 정리 대상

- 2지 gripper가 포함된 arm controller 및 arm init trajectory
- 실제 hardware parameter에 연결되지 않은 `port_name` launch argument
- 저장소에서 publisher가 확인되지 않는 BH5/SH5 hand trajectory topic
- 기존 launch를 직접 호출하는 상위 launch, Docker alias와 s6 service runner

## 13. 비목표

- 현재 로봇별 초기 자세 수치의 완전한 보존
- BG2 rev2/rev3 지원
- Mobile-base 단독 구동 지원
- 정의되지 않은 body/base/gripper 조합 지원
- Gripper마다 별도의 serial hardware system 생성
- 실제 차이가 있는 geometry, inertial, wheel tuning을 하나의 공통 숫자로 강제 통일
- 필요성이 확인되기 전 controller YAML 병합기 또는 설정 생성기 추가
