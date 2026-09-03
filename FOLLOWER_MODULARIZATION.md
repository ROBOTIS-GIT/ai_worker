# FFW Follower 모듈화 개발 문서

- 상태: 설계 초안
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
| `bg2` | `legacy` | `base` | `rh_p12_rn_a` (2지) | Head ZED Mini, Wrist D405 | 없음 |
| `sg2` | `legacy` | `swerve` | `rh_p12_rn_a` (2지) | Head ZED Mini, Wrist D405 | IMU, Dual LiDAR |
| `bh5` | `legacy` | `base` | `hx5_d20` (5지) | Head ZED Mini, Wrist D405 | 없음 |
| `sh5` | `legacy` | `swerve` | `hx5_d20_rev2` (5지) | Head ZED Mini, Wrist D405 | IMU, Dual LiDAR |
| `f1` | `f_series` | `base` | `hx2_d1` (2지) | Head D455, Wrist D401 | 없음 |
| `f2` | `f_series` | `swerve` | `hx2_d1` (2지) | Head D455, Wrist D401 | IMU, Dual LiDAR |

현재 존재하는 위 6개 조합만 허용한다. 별도 로봇 정의 없이 body, base, gripper를 임의로 조합하는 기능은 제공하지 않는다.

## 3. 로봇 설정

로봇 조합의 단일 기준은 `ffw_bringup/config/common/robot_profiles.yaml`로 둔다.

```yaml
robots:
  bg2: {body: legacy,   base: base,   gripper: rh_p12_rn_a}
  sg2: {body: legacy,   base: swerve, gripper: rh_p12_rn_a}
  bh5: {body: legacy,   base: base,   gripper: hx5_d20}
  sh5: {body: legacy,   base: swerve, gripper: hx5_d20_rev2}
  f1:  {body: f_series, base: base,   gripper: hx2_d1}
  f2:  {body: f_series, base: swerve, gripper: hx2_d1}
```

카메라 종류, LiDAR 사용 여부, controller 목록처럼 component에서 결정할 수 있는 값은 중복해서 저장하지 않는다.

- `body=legacy`: ZED Mini head, D405 wrist
- `body=f_series`: D455 head, D401 wrist
- `base=swerve`: steering/drive, IMU, dual LiDAR 활성화
- `gripper=hx5_*`: 5지 hand controller, effort controller 활성화
- `gripper=hx5_d20_rev2`: pressure broadcaster 활성화

## 4. 모듈 경계

### 4.1 Body

Body는 다음 요소를 소유한다.

- Left/right arm 7축
- Lift
- Head 2축
- Body, arm, head geometry와 inertial
- Head/wrist camera description

기존 공통 Xacro를 재사용한다.

- `ffw_follower_body.xacro`: legacy body
- `ffw_f_follower_body.xacro`: F-series body

두 body는 geometry, inertial, joint origin과 카메라 구성이 다르므로 하나의 거대한 parameterized macro로 합치지 않는다.

### 4.2 Base

Base의 논리 타입은 다음 두 가지다.

- `base`: 이동 구동축이 없고 world에 고정된 기본 base
- `swerve`: 3 steering joint와 3 drive joint를 가진 이동 base

Legacy와 F-series는 base mesh, inertial, wheel origin과 wheel radius가 다르다. 따라서 논리 타입은 두 개로 유지하되 내부 구현은 body family에 맞는 물리 데이터를 선택한다.

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
- `hx5_d20`: BH5 5지
- `hx5_d20_rev2`: SH5 5지 및 pressure sensor

5지 hand description의 소유권은 `robotis_hand_description` 패키지에 둔다.

- SH5처럼 BH5도 hand URDF, Gazebo, ros2_control joint 정의를 `robotis_hand_description`에서 include한다.
- 현재 BH5가 `ffw_description` 내부에 직접 보유한 `common/hx5_d20` URDF와 follower ros2_control의 hand joint 정의는 외부 패키지 전환 후 제거한다.
- `ffw_description/package.xml`에 `robotis_hand_description` 실행 의존성을 선언한다.
- BH5의 `hx5_d20`과 SH5의 `hx5_d20_rev2` 모델 구분은 유지한다.

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
  pressure broadcaster: 지원 모델만 사용
```

Controller는 분리하지만 같은 `/dev/follower` 버스를 사용하는 joint를 별도 ros2_control hardware system으로 분리하지 않는다.

## 5. ros2_control 구조

의미상 component 경계와 실제 통신 버스 경계를 구분한다.

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

최상위 description은 하나만 둔다.

```text
ffw_follower.urdf.xacro
├── body 선택
├── base 선택
├── gripper 선택
├── ros2_control hardware system 조립
└── Gazebo component 조립
```

최상위 Xacro는 로봇 이름을 직접 해석하지 않고 launch에서 전달받은 `body`, `base`, `gripper` argument만 사용한다. 로봇 조합 규칙은 `robot_profiles.yaml`에만 둔다.

`ffw_follower.urdf.xacro` 소스 파일은 하나만 저장한다. Launch를 실행할 때마다 선택된 component argument로 Xacro를 펼치고, 생성된 XML 문자열을 `robot_description` parameter로 사용한다. 생성 결과를 별도 `.urdf` 파일로 저장하지 않는다.

```text
ros2 launch ... robot:=sg2
        ↓
robot_profiles.yaml 조회
        ↓
body=legacy, base=swerve, gripper=rh_p12_rn_a
        ↓
ffw_follower.urdf.xacro 실행
        ↓
robot_description 문자열
        ├── robot_state_publisher
        └── ros2_control_node
```

Launch argument는 `generate_launch_description()` 시점에 아직 문자열로 확정되지 않으므로 `OpaqueFunction` 안에서 `robot`을 해석한다. 기존 launch의 `Command([xacro, ...])` 패턴을 그대로 재사용한다.

```python
def launch_setup(context):
    robot = LaunchConfiguration('robot').perform(context)
    robot_profiles_path = os.path.join(
        get_package_share_directory('ffw_bringup'),
        'config', 'common', 'robot_profiles.yaml'
    )

    with open(robot_profiles_path) as file:
        profiles = yaml.safe_load(file)['robots']

    if robot not in profiles:
        raise RuntimeError(f'Unsupported robot: {robot}')

    profile = profiles[robot]
    robot_description_content = Command([
        FindExecutable(name='xacro'),
        ' ',
        PathJoinSubstitution([
            FindPackageShare('ffw_description'),
            'urdf',
            'ffw_follower.urdf.xacro',
        ]),
        ' body:=', profile['body'],
        ' base:=', profile['base'],
        ' gripper:=', profile['gripper'],
        ' use_sim:=', LaunchConfiguration('use_sim'),
        ' use_mock_hardware:=', LaunchConfiguration('use_mock_hardware'),
        ' port_name:=', LaunchConfiguration('port_name'),
    ])

    robot_description = {'robot_description': robot_description_content}
    # 같은 robot_description을 robot_state_publisher와 ros2_control_node에 전달한다.
```

공통 Xacro는 필요한 매크로 정의를 include한 후 선택된 매크로만 호출한다. Include만으로 link나 joint가 생성되지는 않는다.

```xml
<xacro:arg name="body" default="legacy"/>
<xacro:arg name="base" default="base"/>
<xacro:arg name="gripper" default="rh_p12_rn_a"/>

<xacro:if value="${body == 'legacy'}">
  <xacro:ffw_follower_body parent="base_link" prefix="">
    <origin xyz="..." rpy="..."/>
  </xacro:ffw_follower_body>
</xacro:if>

<xacro:if value="${base == 'swerve'}">
  <xacro:swerve_base family="${body}"/>
</xacro:if>

<xacro:if value="${gripper == 'rh_p12_rn_a'}">
  <xacro:rh_p12_rn_a .../>
</xacro:if>
```

기존 로봇별 URDF는 전환 기간에 새 공통 Xacro를 호출하는 얇은 wrapper로 사용할 수 있다. 6개 로봇 검증이 끝나면 제거한다.

## 9. 통합 Launch 책임

`ffw_follower_ai.launch.py`는 다음만 담당한다.

1. 필수 `robot` argument 검증
2. `robot_profiles.yaml` 조회
3. 공통 Xacro에 component argument 전달
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
2. `robot_profiles.yaml`을 추가하고 6개 로봇만 검증한다.
3. BH5의 hand URDF와 ros2_control 정의를 SH5처럼 `robotis_hand_description` include 방식으로 전환한다.
4. 기존 body macro를 유지하면서 base/swerve base와 gripper 조립부를 모듈화한다.
5. 공통 `ffw_follower.urdf.xacro`를 만든다.
6. ros2_control joint 정의를 body/base/gripper macro로 나누고 hardware system을 조립한다.
7. 2지 gripper를 arm controller에서 분리한다.
8. Leader broadcaster의 arm/gripper trajectory 출력을 분리한다.
9. 공통 initial position YAML을 만들고 component별 executor를 연결한다.
10. Gazebo description을 component 기준으로 정리한다.
11. `ffw_follower_ai.launch.py`를 추가하고 6개 로봇을 검증한다.
12. 상위 launch, Docker alias와 s6 service runner를 새 launch로 변경한다.
13. 기존 로봇별 follower launch와 최상위 URDF wrapper를 제거한다.

Launch 통합 전까지 기존 6개 launch를 새 URDF와 ros2_control 구조의 검증 수단으로 사용한다.

## 11. 검증 기준

각 로봇에 대해 다음 명령 형태로 mock hardware smoke test를 수행한다.

```bash
ros2 launch ffw_bringup ffw_follower_ai.launch.py \
  robot:=sg2 \
  use_mock_hardware:=true \
  launch_cameras:=false \
  launch_lidar:=false
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
- SH5 pressure broadcaster가 유지된다.
- 기존 내부 launch 및 Docker 호출이 새 launch를 사용한다.
- 최종 상태에는 follower launch가 `ffw_follower_ai.launch.py` 하나만 남는다.

## 12. 확인된 정리 대상

- 기존 follower ros2_control에 섞여 있는 swerve steering joint
- 2지 gripper가 포함된 arm controller 및 arm init trajectory
- 실제 hardware parameter에 연결되지 않은 `port_name` launch argument
- HX2-D1 URDF와 맞지 않는 F1/F2 Gazebo의 RH-P12 link 참조
- 저장소에서 publisher가 확인되지 않는 BH5/SH5 hand trajectory topic
- `ffw_description` 내부에 중복 보관된 BH5 HX5-D20 URDF와 ros2_control hand joint 정의
- 기존 launch를 직접 호출하는 상위 launch, Docker alias와 s6 service runner

## 13. 비목표

- 현재 로봇별 초기 자세 수치의 완전한 보존
- 정의되지 않은 body/base/gripper 조합 지원
- Gripper마다 별도의 serial hardware system 생성
- 실제 차이가 있는 geometry, inertial, wheel tuning을 하나의 공통 숫자로 강제 통일
- 필요성이 확인되기 전 controller YAML 병합기 또는 설정 생성기 추가
