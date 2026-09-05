# FFW Follower 모듈화 개발 문서

- 상태: follower description/ros2_control/controller/실제 로봇 launch 모듈화 완료, Gazebo launch 통합 및 leader trajectory 분리 대기
- 최종 수정: 2026-09-05

## 1. 목적

로봇별로 나뉜 follower launch와 description을 하나의 조합형 구조로 통합한다.

최종 실행 인터페이스는 다음과 같다.

```bash
ros2 launch ffw_bringup ffw_follower_ai.launch.py robot:=sg2
```

`robot` 값은 설정 파일에서 body, base, end tool 조합으로 해석한다. Launch와 Xacro 안에 로봇 이름별 분기를 중복해서 만들지 않는다.

## 2. 대상 로봇과 조합

| Robot | Body | Base | End tool | 카메라 | Base 센서 |
|---|---|---|---|---|---|
| `bg2` | `ffw_body` | `ffw_basic` | `rh_p12_rn_a` | Head ZED Mini, Wrist D405 | LED |
| `sg2` | `ffw_body` | `ffw_swerve` | `rh_p12_rn_a` | Head ZED Mini, Wrist D405 | IMU, Dual LiDAR |
| `bh5` | `ffw_body` | `ffw_basic` | `hx5_d20_rev2` | Head ZED Mini, Wrist D405 | LED |
| `sh5` | `ffw_body` | `ffw_swerve` | `hx5_d20_rev2` | Head ZED Mini, Wrist D405 | IMU, Dual LiDAR |
| `f1` | `ffw_body_f` | `ffw_basic` | `hx2_d1` | Head D455, Wrist D401 | LED |
| `f2` | `ffw_body_f` | `ffw_swerve_f` | `hx2_d1` | Head D455, Wrist D401 | IMU, Dual LiDAR |

현재 존재하는 위 6개 조합만 허용한다. 별도 로봇 정의 없이 body, base, end tool을 임의로 조합하는 기능은 제공하지 않는다.

## 3. 로봇 설정

로봇 조합의 단일 기준은 `ffw_description/config/follower_robots.yaml`이다.

```yaml
bg2:
  body: ffw_body
  base: ffw_basic
  end_tool: rh_p12_rn_a
  head_camera_type: zed
sg2:
  body: ffw_body
  base: ffw_swerve
  end_tool: rh_p12_rn_a
  head_camera_type: zed
```

Head camera 종류는 각 로봇의 `head_camera_type`에서 직접 읽는다. Controller 실행 목록은 각 component의 controller YAML이 소유한다. LiDAR 지원 여부는 `ffw_swerve`로 시작하는 base 모델명으로 판단하고, 실제 LiDAR 실행 여부는 `launch_lidar` launch argument로 정한다.

- `body=ffw_body`: ZED Mini head, D405 wrist
- `body=ffw_body_f`: D455 head, D401 wrist
- `base=ffw_swerve*`: steering/drive, IMU, dual LiDAR 활성화
- `end_tool=hx5_d20_rev2`: 5지 hand controller, effort controller, pressure broadcaster 활성화

로봇 조립도와 component controller 설정은 책임이 다르므로 파일을 분리한다.

- `ffw_description/config/follower_robots.yaml`: 로봇별 body/base/end tool 조립도와 head camera 종류
- `ffw_bringup/config/follower/controllers/`: component별 controller 설정과 `controller_spawn` 실행 정보

`controller_spawn`에는 로봇 이름을 넣지 않는다. 기존 component로 로봇 조합을 변경할 때는 `follower_robots.yaml`만 수정한다.

## 4. 모듈 경계

### 4.1 Body

Body는 다음 요소를 소유한다.

- Left/right arm 7축
- Lift
- Head 2축
- Body, arm, head geometry와 inertial
- Head/wrist camera description

공통 Xacro는 다음 위치에서 관리한다.

- `urdf/follower/body/ffw_body.urdf.xacro`: BG2/SG2/BH5/SH5 body
- `urdf/follower/body/ffw_body_f.urdf.xacro`: F1/F2 body

두 body는 geometry, inertial, joint origin과 카메라 구성이 다르므로 하나의 거대한 parameterized macro로 합치지 않는다.

### 4.2 Base

Base는 모델별 파일로 나눈다.

- `ffw_basic`: BG2/BH5/F1
- `ffw_swerve`: SG2/SH5
- `ffw_swerve_f`: F2

Basic base는 geometry와 inertial이 동일하므로 BG2/BH5/F1이 `ffw_basic`을 공유한다. Swerve는 Legacy와 F-series의 mesh, inertial과 wheel origin이 달라 별도 모델을 유지한다. Wheel radius는 둘 다 `0.0865`이다. `ffw_swerve`는 SG2 정의를 기준으로 하며, 기존 SH5와 비교하면 LiDAR 두 링크에 각각 0.1 kg의 inertial과 시각 cylinder가 추가된다. 그 외 SH5의 link/joint origin과 물성치는 동일하다.

F2 controller의 `module_x_offsets`는 URDF steering 축 X 좌표 `[0.137119, 0.137119, -0.288133]`에 맞춘다. F2 steering 3축의 `Acceleration Limit`은 중복 없이 `12592`를 사용한다.

Swerve base가 소유하는 항목은 다음과 같다.

- `left/right/rear_wheel_steer`
- `left/right/rear_wheel_drive`
- IMU와 LED interface
- Left/right LiDAR link 및 Gazebo sensor
- Swerve controller와 초기 steering controller
- Dual LiDAR launch와 laser merger

### 4.3 End tool

End tool은 모델별 파일로 나눈다.

- `rh_p12_rn_a`: legacy 2지
- `hx2_d1`: F-series 2지
- `hx5_d20_rev2`: BH5/SH5 5지 및 pressure sensor

5지 hand description의 소유권은 `robotis_hand_description` 패키지에 둔다.

- SH5처럼 BH5도 hand URDF와 ros2_control joint 정의를 `robotis_hand_description`에서 include한다.
- Gazebo는 `gazebo/follower/end_tool/hx5_d20_rev2.gazebo.xacro`가 담당한다. 외부 좌우 Gazebo macro는 한 로봇에서 함께 호출하면 `hand_trans1~20` 이름이 중복되므로, 동일한 물리값을 사용하면서 이름을 `hand_l_*`, `hand_r_*`로 구분한다.
- 기존 BH5 내부 HX5-D20 URDF와 ros2_control hand 정의는 제거했으며 BH5/SH5 모두 외부 rev2 description을 사용한다.
- 중복된 로컬 HX5-D20 mesh는 제거하고 `robotis_hand_description`의 mesh를 사용한다.
- `ffw_description/package.xml`에 `robotis_hand_description` 실행 의존성을 선언한다.
- BH5와 SH5 모두 `hx5_d20_rev2`를 사용한다.

모든 gripper controller는 arm controller에서 분리한다.

Controller 구성:

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
│   ├── ffw_body.ros2_control.xacro
│   └── ffw_body_f.ros2_control.xacro
├── base/
│   ├── ffw_basic.ros2_control.xacro
│   ├── ffw_swerve.ros2_control.xacro
│   └── ffw_swerve_f.ros2_control.xacro
└── end_tool/
    ├── rh_p12_rn_a.ros2_control.xacro
    ├── hx2_d1.ros2_control.xacro
    └── hx5_d20_rev2.ros2_control.xacro
```

Body와 End tool 파일은 hardware system을 만들지 않고 joint/GPIO 정의만 제공한다. Base 파일은 `/dev/follower`에 들어갈 steering 정의와 별도 drive/sensor system을 제공한다. 최상위 파일이 선택된 정의를 다음 통신 경계로 조립한다.

```text
follower hardware system (/dev/follower)
├── body joints
├── 선택된 gripper joints
└── swerve일 때 steering joints

base hardware system (/dev/ttyUSB0, swerve 전용)
└── drive joints

sensor hardware system (/dev/ttyUSB1)
├── basic: LED
└── swerve: IMU / LED
```

Simulation에서는 실제 `/dev/ttyUSB1` sensor hardware system을 만들지 않는다. Dual LiDAR는 Swerve Gazebo sensor와 bridge가 제공한다.

Swerve의 steering joint와 drive joint는 서로 다른 hardware system에 있지만 모두 base component가 소유한다. Base Xacro는 필요한 joint 정의를 각 hardware system에 제공한다.

동일한 serial port를 두 Dynamixel hardware system이 동시에 열지 않도록 gripper 전용 hardware system은 만들지 않는다.

`basic` base는 `/dev/ttyUSB1`의 LED(ID 91)만 사용하며 IMU는 정의하지 않는다. 따라서 BG2, BH5와 F1에서도 `ffw_sensor/set_dxl_data`를 통한 LED 제어가 가능하다.

`use_sim`은 최상위 Xacro argument로 한 번만 받고 ros2_control 매크로 parameter로 전달한다. 각 매크로 내부에서는 전역 `$(arg use_sim)`을 다시 읽지 않고 전달받은 `${use_sim}`을 사용하도록 모든 로봇을 통일한다.

`ffw_robot_manager`의 배터리 전압 조회를 위해 모든 로봇의 `dxl1`, `dxl61`에서 `Present Input Voltage`를 제공한다.

Head 2축(`dxl62`) 제어 gain은 SG2 기준인 P/I/D `800/200/200`, Feedforward 1st/2nd `20/20`으로 모든 로봇을 통일한다.

BH5도 다른 follower와 동일하게 상태 topic과 제어 service를 `ffw_follower/*` 이름으로 제공한다. `ffw_robot_manager`는 `/ffw_follower/dxl_state`를 구독한다.

현재 하드코딩된 포트는 launch argument가 실제 ros2_control parameter까지 전달되도록 수정한다. 기본값은 현재 값을 유지한다.

### 5.1 Controller config 구조

Controller config도 body/base/end tool component별로 분리한다. 같은 설정을 사용하는 component도 추후 독립적으로 변경할 수 있도록 각각 자기 파일을 가진다.

```text
ffw_bringup/config/follower/
├── controllers/
│   ├── body/
│   │   ├── ffw_body.controller.yaml
│   │   └── ffw_body_f.controller.yaml
│   ├── base/
│   │   ├── ffw_basic.controller.yaml
│   │   ├── ffw_swerve.controller.yaml
│   │   └── ffw_swerve_f.controller.yaml
│   └── end_tool/
│       ├── rh_p12_rn_a.controller.yaml
│       ├── hx2_d1.controller.yaml
│       └── hx5_d20_rev2.controller.yaml
└── initial_positions.yaml
```

`ffw_body`와 `ffw_body_f` body config는 arm 7축, head, lift, `joint_state_broadcaster`, `ffw_robot_manager`와 `update_rate`를 가진다. 별도 `core.yaml`은 만들지 않는다. `basic` base config는 현재 실행할 controller 없이 공통 로딩 형식만 유지한다. `swerve` config는 steering 초기화와 drive controller만 소유한다.

Launch는 선택된 body/base/end tool의 `*.controller.yaml`과 그 안의 `controller_spawn`을 읽는다. Controller 설정은 `ros2_control_node`에 전달하고, `controller_spawn`에 지정된 controller와 executor를 실행한다. `use_sim_time`과 `robot_description`은 Launch가 직접 전달한다. `ffw_robot_manager`는 모든 실제 follower에서 동일하게 실행한다. 배터리 전압 interface가 없는 경우 해당 배터리만 자동으로 건너뛴다.

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

파일:

```text
ffw_bringup/config/follower/initial_positions.yaml
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
├── end tool 선택
├── ros2_control hardware system 조립
└── Gazebo component 조립
```

최상위 Xacro는 `robot` argument를 받아 `follower_robots.yaml`을 읽고 body/base/end tool 모듈을 선택한다. Launch는 로봇 이름만 넘기며 조합 규칙을 중복해서 갖지 않는다.

`ffw_follower.urdf.xacro` 소스 파일은 하나만 저장한다. Launch 실행 시 Xacro 결과를 XML 문자열로 만들어 `robot_description` parameter로 사용하며, 생성 결과를 별도 `.urdf` 파일로 저장하지 않는다.

```text
ros2 launch ... robot:=sg2
        ↓
follower_robots.yaml 조회
        ↓
body=ffw_body, base=ffw_swerve, end_tool=rh_p12_rn_a
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
                value="${xacro.load_yaml('.../follower_robots.yaml')[robot_name]}"/>
<xacro:property name="body" value="${robot_config['body']}"/>
<xacro:property name="base" value="${robot_config['base']}"/>
<xacro:property name="end_tool" value="${robot_config['end_tool']}"/>

<xacro:include filename=".../body/${body}.urdf.xacro"/>
<xacro:include filename=".../base/${base}.urdf.xacro"/>
<xacro:include filename=".../end_tool/${end_tool}.urdf.xacro"/>

<xacro:follower_base prefix=""/>
<xacro:follower_body parent="base_link" prefix="">
  <origin xyz="${body_mount_xyz}" rpy="${body_mount_rpy}"/>
</xacro:follower_body>
<xacro:follower_end_tool prefix=""/>
```

기존 로봇별 최상위 follower URDF는 제거했다. 기존 `ffw_bringup` launch 변경은 원복했으며, ros2_control/config/launch 통합 단계에서 공통 Xacro로 한꺼번에 전환한다.

Gazebo 설정도 URDF와 같은 component 이름과 폴더 구조를 사용한다.

```text
gazebo/follower/
├── body/
│   ├── ffw_body.gazebo.xacro
│   └── ffw_body_f.gazebo.xacro
├── base/
│   ├── ffw_basic.gazebo.xacro
│   ├── ffw_swerve.gazebo.xacro
│   └── ffw_swerve_f.gazebo.xacro
└── end_tool/
    ├── rh_p12_rn_a.gazebo.xacro
    ├── hx2_d1.gazebo.xacro
    └── hx5_d20_rev2.gazebo.xacro
```

각 component는 `follower_body_gazebo`, `follower_base_gazebo`, `follower_end_tool_gazebo`라는 공통 호출 이름만 제공한다. 물리값과 transmission 정의는 공통 파일로 빼지 않고 각 구현 파일에 둔다. 현재 값이 같더라도 각 Body/Base/End tool 모델이 독립적으로 변경될 수 있도록 서로 include하지 않는다. Swerve Base가 dual LiDAR sensor를 소유하고 End tool이 자기 link와 transmission 설정을 소유한다. 최상위 Xacro에는 공통 `gz_ros2_control` plugin만 직접 둔다. 기존 로봇별 Gazebo 파일은 제거했다.

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
5. 선택된 base와 end tool의 추가 controller/node 실행

Gazebo launch 통합은 보류하며 기존 로봇별 Gazebo launch를 유지한다. 기존 launch가 참조하던 로봇별 URDF는 모듈화 과정에서 제거되었으므로, Gazebo launch 통합 전까지는 실행 대상이 아니다.

공통 launch argument:

- `robot`: `bg2`, `sg2`, `bh5`, `sh5`, `f1`, `f2`
- `start_rviz`
- `use_sim`
- `use_mock_hardware`
- `mock_sensor_commands`
- `launch_cameras`
- `launch_lidar`: Swerve의 dual LiDAR driver와 merger 실행 여부, 기본값 `true`
- `init_position`
- `use_head_eef_tracker`
- `port_name`: follower hardware 포트 override

잘못된 `robot` 값은 기본 로봇으로 대체하지 않고 즉시 오류로 종료한다.

## 10. 작업 순서

1. 기존 6개 Xacro의 link, joint, ros2_control interface 목록을 기준 결과로 저장한다.
2. `follower_robots.yaml`을 추가하고 6개 로봇만 검증한다.
3. BH5의 hand URDF와 ros2_control 정의를 SH5처럼 `robotis_hand_description` include 방식으로 전환한다. (완료)
4. 기존 body macro를 유지하면서 base와 end tool 조립부를 모듈화한다. (완료)
5. 공통 `ffw_follower.urdf.xacro`를 만든다. (완료)
6. ros2_control joint 정의를 body/base/end tool macro로 나누고 hardware system을 조립한다. (완료)
7. 2지 gripper를 arm controller에서 분리한다. (controller와 launch 분리 완료, leader 출력 분리 대기)
8. Leader broadcaster의 arm/gripper trajectory 출력을 분리한다.
9. 공통 initial position YAML을 만들고 component별 executor를 연결한다. (완료)
10. Gazebo description을 component 기준으로 정리한다. (완료)
11. 실제 로봇용 `ffw_follower_ai.launch.py`를 추가하고 6개 로봇을 검증한다. (완료)
12. Gazebo launch를 통합한다. (보류)
13. Docker alias와 s6 service runner를 새 launch로 변경한다. 상위 launch 전환은 보류한다.
14. 기존 로봇별 실제 로봇 launch를 제거한다. Gazebo launch는 통합할 때 정리한다. (완료)

6개 조합은 Xacro/check_urdf와 mock hardware launch로 검증한다. SG2는 공통 초기 자세 실행과 steering 초기화 후 drive controller 전환까지 확인한다.

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
- `ffw_basic`에는 steering/drive interface와 swerve controller가 없다.
- `ffw_swerve*` base에는 steering 3축과 drive 3축이 모두 존재한다.
- Swerve steering 초기화 후 drive controller 전환이 성공한다.
- 하나의 공통 initial position YAML로 6개 로봇이 초기화된다.
- Legacy/F-series 카메라 조합이 기존 하드웨어와 일치한다.
- BH5와 SH5의 hand URDF 및 ros2_control 정의가 `robotis_hand_description`에서 제공된다.
- BH5와 SH5의 pressure broadcaster가 유지된다.
- 기존 내부 launch 및 Docker 호출이 새 launch를 사용한다.
- 실제 로봇은 공통 `ffw_follower_ai.launch.py`를 사용하고, Gazebo는 통합 전까지 기존 로봇별 launch를 사용한다.

## 12. 확인된 정리 대상

- 저장소에서 publisher가 확인되지 않는 BH5/SH5 hand trajectory topic

## 13. 비목표

- 현재 로봇별 초기 자세 수치의 완전한 보존
- BG2 rev2/rev3 지원
- Mobile-base 단독 구동 지원
- 정의되지 않은 body/base/end tool 조합 지원
- End tool마다 별도의 serial hardware system 생성
- 실제 차이가 있는 geometry, inertial, wheel tuning을 하나의 공통 숫자로 강제 통일
- 필요성이 확인되기 전 controller YAML 병합기 또는 설정 생성기 추가
