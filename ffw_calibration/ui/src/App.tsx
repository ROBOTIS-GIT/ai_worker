import { Fragment, useEffect, useRef, useState, type CSSProperties } from 'react'

const TABS = ['Information', 'Calibration', 'Result', 'Test'] as const
type Tab = typeof TABS[number]

const JOINT_ORDER = [
  // RIGHT ARM
  'arm_r_joint2',
  'arm_r_joint4',
  'arm_r_joint6',
  'arm_r_joint7',
  'arm_r_joint1',
  'arm_r_joint3',
  'arm_r_joint5',
  // LEFT ARM
  'arm_l_joint2',
  'arm_l_joint4',
  'arm_l_joint6',
  'arm_l_joint7',
  'arm_l_joint1',
  'arm_l_joint3',
  'arm_l_joint5',
] as const

/** Head joints (UI only; calibration flow not wired yet) */
const HEAD_JOINT_ORDER = ['head_joint1', 'head_joint2'] as const

type SafetyStage = 'before_joint1' | 'before_joint3' | 'before_joint5'

type PendingSafety = {
  arm: 'right' | 'left'
  stage: SafetyStage
}

const SAFETY_STAGE_TITLE: Record<SafetyStage, string> = {
  before_joint1: 'Joint 1 준비',
  before_joint3: 'Joint 3 준비',
  before_joint5: 'Joint 5 준비',
}

const nextJointKeyAfterSafety = (
  arm: 'right' | 'left',
  stage: SafetyStage,
): (typeof JOINT_ORDER)[number] => {
  if (stage === 'before_joint1') {
    return arm === 'right' ? 'arm_r_joint1' : 'arm_l_joint1'
  }
  if (stage === 'before_joint3') {
    return arm === 'right' ? 'arm_r_joint3' : 'arm_l_joint3'
  }
  return arm === 'right' ? 'arm_r_joint5' : 'arm_l_joint5'
}

const previousJointKeyBeforeSafety = (
  arm: 'right' | 'left',
  stage: SafetyStage,
): (typeof JOINT_ORDER)[number] => {
  if (stage === 'before_joint1') {
    return arm === 'right' ? 'arm_r_joint7' : 'arm_l_joint7'
  }
  if (stage === 'before_joint3') {
    return arm === 'right' ? 'arm_r_joint1' : 'arm_l_joint1'
  }
  return arm === 'right' ? 'arm_r_joint3' : 'arm_l_joint3'
}

const previousSafetyStage = (stage: SafetyStage): SafetyStage | null => {
  if (stage === 'before_joint3') return 'before_joint1'
  if (stage === 'before_joint5') return 'before_joint3'
  return null
}

const safetyStageForJointKey = (
  joint: string,
): { arm: 'right' | 'left'; stage: SafetyStage } | null => {
  if (joint === 'arm_r_joint1') return { arm: 'right', stage: 'before_joint1' }
  if (joint === 'arm_r_joint3') return { arm: 'right', stage: 'before_joint3' }
  if (joint === 'arm_r_joint5') return { arm: 'right', stage: 'before_joint5' }
  if (joint === 'arm_l_joint1') return { arm: 'left', stage: 'before_joint1' }
  if (joint === 'arm_l_joint3') return { arm: 'left', stage: 'before_joint3' }
  if (joint === 'arm_l_joint5') return { arm: 'left', stage: 'before_joint5' }
  return null
}

type Status = 'pending' | 'captured' | 'skipped'

type CalibrationStatusMsg = {
  phase?: string
  arm?: string
  joint?: string
  progress?: number
  message?: string
  result?: string
}

type RosbridgeMessage =
  | {
      op: 'publish'
      topic: string
      msg?: {
        name?: string[]
        position?: number[]
        effort?: number[]
        /** std_msgs/Float64MultiArray */
        data?: number[]
      } & CalibrationStatusMsg
    }
  | {
      op: 'service_response'
      id?: string
      values?: {
        success?: boolean
        joint_names?: string[]
        target_rads?: number[]
        message?: string
        measured_rad?: number
        target_rad?: number
        delta_rad?: number
        delta_pulse?: number
        old_offset?: number
        new_offset?: number
        gpio_keys?: string[]
        offsets?: number[]
        result?: boolean
        duration_sec?: number
        defined?: boolean[]
        right_targets?: number[]
        left_targets?: number[]
        default_duration_sec?: number
        pose_duration_sec?: number[]
        before_joint1_duration_sec?: number
        before_joint3_duration_sec?: number
        before_joint5_duration_sec?: number
        before_joint1_right?: number[]
        before_joint1_left?: number[]
        before_joint3_right?: number[]
        before_joint3_left?: number[]
        before_joint5_right?: number[]
        before_joint5_left?: number[]
        zero_pose_duration_sec?: number
        et_pose_duration_sec?: number
        zero_pose_right?: number[]
        zero_pose_left?: number[]
        lift_up_defined?: boolean
        lift_up_duration_sec?: number
        lift_up_right?: number[]
        lift_up_left?: number[]
        lift_down_defined?: boolean
        lift_down_duration_sec?: number
        lift_down_right?: number[]
        lift_down_left?: number[]
      }
    }

type RosServiceValues = {
  success?: boolean
  joint_names?: string[]
  target_rads?: number[]
  message?: string
  measured_rad?: number
  target_rad?: number
  delta_rad?: number
  delta_pulse?: number
  old_offset?: number
  new_offset?: number
  gpio_keys?: string[]
  offsets?: number[]
  result?: boolean
  duration_sec?: number
  defined?: boolean[]
  right_targets?: number[]
  left_targets?: number[]
  default_duration_sec?: number
  pose_duration_sec?: number[]
  before_joint1_duration_sec?: number
  before_joint3_duration_sec?: number
  before_joint5_duration_sec?: number
  before_joint1_right?: number[]
  before_joint1_left?: number[]
  before_joint3_right?: number[]
  before_joint3_left?: number[]
  before_joint5_right?: number[]
  before_joint5_left?: number[]
  zero_pose_duration_sec?: number
  et_pose_duration_sec?: number
  zero_pose_right?: number[]
  zero_pose_left?: number[]
  lift_up_defined?: boolean
  lift_up_duration_sec?: number
  lift_up_right?: number[]
  lift_up_left?: number[]
  lift_down_defined?: boolean
  lift_down_duration_sec?: number
  lift_down_right?: number[]
  lift_down_left?: number[]
}

type StatusListener = {
  phase: string
  onProgress?: (progress: number, msg: CalibrationStatusMsg) => void
  resolve: (msg: CalibrationStatusMsg) => void
  reject: (err: Error) => void
}

type TaskWaiter = {
  promise: Promise<CalibrationStatusMsg>
  cancel: () => void
}

const ZERO_POSE_BAR_STEPS = 12


/** SAFETY: joint 1–7 vs 0 rad, ±tol */
const SAFETY_ZERO_RAD_TOL = 0.5
/** SAFETY 토크 해제 `apply_effort` effort→0 ramp duration_sec */
const SAFETY_TORQUE_RELEASE_SEC = 3
/** SAFETY「이동」traj 이후 effort 램프(초) */
const JOINT7_SAFETY_EFFORT_RAMP_SEC = 5
/** `apply_effort` target 7×0 */
const SAFETY_ZERO_EFFORT_TARGET: readonly number[] = [0, 0, 0, 0, 0, 0, 0]

const buildAllZeroTrajectory8 = (): number[] => [0, 0, 0, 0, 0, 0, 0, 0]

type Traj8Pair = { right: number[]; left: number[] }

type SafetyPrepStagePack = Traj8Pair & { durationSec: number }

type SafetyPrepPosesPack = {
  beforeJoint1: SafetyPrepStagePack
  beforeJoint3: SafetyPrepStagePack
  beforeJoint5: SafetyPrepStagePack
}

type MotionPosesPack = {
  zeroPoseDurationSec: number
  etPoseDurationSec: number
  zeroPose: Traj8Pair
  liftUp?: { durationSec: number; right: number[]; left: number[] }
  liftDown?: { durationSec: number; right: number[]; left: number[] }
}

const parseTraj8 = (values: unknown): number[] => {
  if (!Array.isArray(values) || values.length < TEST_TRAJ_AXES) {
    return buildAllZeroTrajectory8()
  }
  return values.slice(0, TEST_TRAJ_AXES).map(v => (typeof v === 'number' ? v : 0))
}

const getSafetyPrepStagePack = (
  pack: SafetyPrepPosesPack | null,
  stage: SafetyStage,
): SafetyPrepStagePack | null => {
  if (!pack) return null
  if (stage === 'before_joint5') return pack.beforeJoint5
  if (stage === 'before_joint3') return pack.beforeJoint3
  return pack.beforeJoint1
}

const getSafetyPrepBaseTraj8 = (
  pack: SafetyPrepPosesPack | null,
  stage: SafetyStage,
  arm: 'right' | 'left',
): number[] => {
  const slot = getSafetyPrepStagePack(pack, stage)
  if (!slot) return buildAllZeroTrajectory8()
  return [...(arm === 'right' ? slot.right : slot.left)]
}

const getSafetyPrepDurationSec = (
  pack: SafetyPrepPosesPack | null,
  stage: SafetyStage,
): number => {
  const slot = getSafetyPrepStagePack(pack, stage)
  return slot?.durationSec ?? DEFAULT_SAFETY_PREP_MOVE_SEC
}

/** SAFETY: effort 램프·모터 homing 대상 순서 (joint 번호 1..7) */
const SAFETY_PREP_CALIB_JOINT_NUMS = [2, 4, 6, 7, 1, 3, 5] as const

const SET_DXL_DATA_SERVICE = '/dynamixel_hardware_interface/set_dxl_data'
const SET_DXL_DATA_TYPE = 'dynamixel_interfaces/srv/SetDataToDxl'
/** Homing 단계: effort 0 직후, Torque OFF↔Homing↔Torque ON 사이, Torque ON 직후 */
const DXL_HOMING_STEP_PAUSE_MS = 500

const sleepMs = (ms: number) =>
  new Promise<void>(resolve => {
    window.setTimeout(resolve, ms)
  })

const jointNameToDxlId = (joint: string): number | null => {
  const m = joint.match(/^arm_([rl])_joint(\d+)$/)
  if (!m) return null
  const n = Number(m[2])
  if (!Number.isFinite(n) || n < 1 || n > 7) return null
  return m[1] === 'r' ? n : 30 + n
}

/** 준비 관절(1|3|5)보다 캘리 순서상 앞선 관절 이름 (해당 팔) */
const safetyPrepPreviousJointNames = (
  arm: 'right' | 'left',
  prepNum: 1 | 3 | 5,
): string[] => {
  const prefix = arm === 'right' ? 'arm_r' : 'arm_l'
  const prepIdx = SAFETY_PREP_CALIB_JOINT_NUMS.indexOf(prepNum)
  if (prepIdx < 0) return []
  return SAFETY_PREP_CALIB_JOINT_NUMS.slice(0, prepIdx).map(n => `${prefix}_joint${n}`)
}

/** Test 탭 관절별 포즈 기본 `duration_sec` (YAML 미로드 시) */
const DEFAULT_TEST_JOINT_TRAJECTORY_SEC = 6
const TEST_JOINT_POSE_SLOT_COUNT = 28
const TEST_TRAJ_AXES = 8
const TEST_TRAJ_PACKED_SIZE = TEST_JOINT_POSE_SLOT_COUNT * TEST_TRAJ_AXES
const TEST_ET_POSE_PHASE = 'move_to_pose:et_pose'
const DEFAULT_ET_POSE_DURATION_SEC = 10
const DEFAULT_ZERO_POSE_DURATION_SEC = 5
const DEFAULT_SAFETY_PREP_MOVE_SEC = 3

const TEST_JOINT_NUMS = [1, 2, 3, 4, 5, 6, 7] as const
type TestJointNum = (typeof TEST_JOINT_NUMS)[number]
type TestTabSelection = 'custom' | TestJointNum
type TestPoseIndex = 1 | 2 | 3 | 4

const TEST_JOINT_POSE_LABELS: Record<TestPoseIndex, string> = {
  1: 'Pose 1',
  2: 'Pose 2',
  3: 'Pose 3',
  4: 'Pose 4',
}

const TRAJ8_AXIS_LABELS = ['J1', 'J2', 'J3', 'J4', 'J5', 'J6', 'J7', 'G'] as const
const DEG_TO_RAD = Math.PI / 180

const TEST_CUSTOM_ARM_JOINT_NUMS = [1, 2, 3, 4, 5, 6, 7] as const
type TestCustomArmJointNum = (typeof TEST_CUSTOM_ARM_JOINT_NUMS)[number]
/** Custom ± step: time from start = 각도(°) × 100ms */
const TEST_CUSTOM_MS_PER_DEG = 100

const testCustomTimeFromStartSec = (deg: number) =>
  (deg * TEST_CUSTOM_MS_PER_DEG) / 1000

const TEST_CUSTOM_STEP_OPTIONS = [
  { deg: 0.1, timeFromStartSec: testCustomTimeFromStartSec(0.1) },
  { deg: 1, timeFromStartSec: testCustomTimeFromStartSec(1) },
  { deg: 5, timeFromStartSec: testCustomTimeFromStartSec(5) },
  { deg: 10, timeFromStartSec: testCustomTimeFromStartSec(10) },
] as const

type TestCustomStepDeg = (typeof TEST_CUSTOM_STEP_OPTIONS)[number]['deg']

const formatTestCustomTimeFromStart = (sec: number) => {
  if (sec === 0) return '0초'
  if (sec < 1) return `${Math.round(sec * 1000)}ms`
  return `${sec}초`
}

const customArmJointRosName = (arm: 'r' | 'l', jointNum: TestCustomArmJointNum) =>
  `arm_${arm}_joint${jointNum}` as const

const buildCustomArmTargets8 = (
  arm: 'r' | 'l',
  positions: Record<string, number>,
  jointNum?: TestCustomArmJointNum,
  deltaRad = 0,
): number[] => {
  const joints = TEST_CUSTOM_ARM_JOINT_NUMS.map(n => {
    const name = customArmJointRosName(arm, n)
    const base = positions[name] ?? 0
    return n === jointNum ? base + deltaRad : base
  })
  return [...joints, 0]
}

/** Test 탭 이미지·궤적용 관절명 (오른팔) */
const testJointRosName = (n: TestJointNum) => `arm_r_joint${n}` as const

type TestArmTraj = { right: number[]; left: number[] }

type TestJointPosePack = {
  defaultDurationSec: number
  poseDurationSec: number[]
  defined: boolean[]
  right: number[]
  left: number[]
}

const testJointPoseSlotIndex = (jointNum: TestJointNum, poseIdx: TestPoseIndex) =>
  (jointNum - 1) * 4 + (poseIdx - 1)

const testJointPoseTrajOffset = (jointNum: TestJointNum, poseIdx: TestPoseIndex) =>
  testJointPoseSlotIndex(jointNum, poseIdx) * TEST_TRAJ_AXES

const buildEtPoseArmTargets8 = (
  targetRads: Record<string, number>,
  arm: 'r' | 'l',
): number[] => {
  const p = arm === 'r' ? 'arm_r' : 'arm_l'
  return [
    targetRads[`${p}_joint1`] ?? 0,
    targetRads[`${p}_joint2`] ?? 0,
    targetRads[`${p}_joint3`] ?? 0,
    targetRads[`${p}_joint4`] ?? 0,
    targetRads[`${p}_joint5`] ?? 0,
    targetRads[`${p}_joint6`] ?? 0,
    targetRads[`${p}_joint7`] ?? 0,
    0,
  ]
}

const STATUS_COLOR: Record<Status, string> = {
  pending: '#3a3a3a',
  captured: '#22c55e',
  skipped: '#737373',
}

/** 오른쪽 리스트 토크 점: arm_r/l_joint1..7 만 */
const ARM_TORQUE_JOINT_RE = /^arm_[rl]_joint[1-7]$/

/** joint_states.effort(실측) 또는 effort 명령 값의 절대값이 이 값 이하면 OFF */
const TORQUE_EFFORT_ABS_EPS = 0.05

const RIGHT_ARM_EFFORT_CMD_TOPIC = '/arm_r_effort_controller/commands'
const LEFT_ARM_EFFORT_CMD_TOPIC = '/arm_l_effort_controller/commands'

function normalizeEffortCommand7(raw: unknown): number[] | null {
  if (!Array.isArray(raw)) return null
  const out: number[] = []
  for (let i = 0; i < 7; i++) {
    const v = raw[i]
    out.push(typeof v === 'number' && Number.isFinite(v) ? v : 0)
  }
  return out
}

const formatRad4 = (v: number) => v.toFixed(4)

const RAD_TO_DEG = 180 / Math.PI
type ResultAngleUnit = 'rad' | 'deg'

const formatResultAngle = (rad: number, unit: ResultAngleUnit) =>
  unit === 'deg' ? (rad * RAD_TO_DEG).toFixed(2) : rad.toFixed(4)

const formatResultDelta = (rad: number, unit: ResultAngleUnit) => {
  const sign = rad >= 0 ? '+' : ''
  return `${sign}${formatResultAngle(rad, unit)}`
}

/** arm_r_joint3 → "3" (JOINT 열 표시용) */
const jointDisplayNumber = (joint: string): string => {
  const m = joint.match(/joint(\d+)/i)
  return m ? m[1] : joint
}

function App() {
  const [statuses, setStatuses] = useState<Status[]>(
    () => JOINT_ORDER.map(() => 'pending')
  )
  const [currentIndex, setCurrentIndex] = useState(0)
  const [activeTab, setActiveTab] = useState<Tab>('Calibration')
  const [calibrationStarted, setCalibrationStarted] = useState(false)
  const [startZeroPosePressed, setStartZeroPosePressed] = useState(false)
  const [startZeroPoseProgress, setStartZeroPoseProgress] = useState(0)
  /** Start: `zero_effort` 서비스 호출 동안만 true (추가 zero 궤적 없음) */
  const [startSequenceBusy, setStartSequenceBusy] = useState(false)
  /** SAFETY prep */
  const [pendingSafety, setPendingSafety] = useState<PendingSafety | null>(null)
  /** SAFETY: apply_effort 로 effort→0 완료 */
  const [safetyEffortZeroed, setSafetyEffortZeroed] = useState(false)
  /** SAFETY: DXL Homing Offset 적용 완료 (이동 버튼 활성) */
  const [safetyTorqueReleased, setSafetyTorqueReleased] = useState(false)
  const [safetyTorqueReleaseBusy, setSafetyTorqueReleaseBusy] = useState(false)
  const [safetyHomingBusy, setSafetyHomingBusy] = useState(false)
  const [safetyArmMoveBusy, setSafetyArmMoveBusy] = useState(false)
  const [safetyApplyProgress, setSafetyApplyProgress] = useState(0)
  const [jointImgFailed, setJointImgFailed] = useState<Record<string, boolean>>({})
  const [jointPositions, setJointPositions] = useState<Record<string, number>>({})
  const [targetRads, setTargetRads] = useState<Record<string, number>>({})
  // 캘리브레이션이 끝난 joint 의 그 시점 measured 값. captured 이후에는
  // 실시간 값 대신 이 값을 표시해서 떨림을 막는다.
  const [capturedPositions, setCapturedPositions] = useState<Record<string, number>>({})
  // Result 탭: 캡처 시점 기준 rad (서비스 응답 measured / target / delta_rad)
  type CaptureResult = {
    measuredRad: number
    targetRad: number
    deltaRad: number
  }
  const [captureResults, setCaptureResults] = useState<Record<string, CaptureResult>>({})
  const [testTabSelection, setTestTabSelection] = useState<TestTabSelection>(1)
  const [testSelectedPoseIdx, setTestSelectedPoseIdx] = useState<TestPoseIndex>(1)
  const [testCustomStepDeg, setTestCustomStepDeg] = useState<TestCustomStepDeg>(1)
  const [testCustomArm, setTestCustomArm] = useState<'right' | 'left'>('right')
  const [activeTestTraj, setActiveTestTraj] = useState<string | null>(null)
  const jointPositionsRef = useRef(jointPositions)
  const capturedPositionsRef = useRef(capturedPositions)
  const captureResultsRef = useRef(captureResults)
  useEffect(() => {
    jointPositionsRef.current = jointPositions
  }, [jointPositions])
  useEffect(() => {
    capturedPositionsRef.current = capturedPositions
  }, [capturedPositions])
  useEffect(() => {
    captureResultsRef.current = captureResults
  }, [captureResults])

  /** Homing 은 DXL 레지스터에 반영 — 궤적 목표는 baseEight 그대로 (delta 보정 없음). */
  const mergeCapturedIntoTrajectory8 = (
    _arm: 'right' | 'left',
    baseEight: number[],
  ): number[] => [...baseEight]

  const callSetDataToDxl = (
    id: number,
    itemName: string,
    itemData: number,
  ) =>
    callRosService(SET_DXL_DATA_SERVICE, SET_DXL_DATA_TYPE, {
      id,
      item_name: itemName,
      item_data: itemData >>> 0,
    }, 10)

  const buildMotorHomingTargets = async (
    jointNames: string[],
  ): Promise<{ joint: string; id: number; offset: number }[]> => {
    if (jointNames.length === 0) {
      return []
    }
    const offResp = await callRosService(
      '/calibration/get_offsets',
      'ffw_calibration/srv/GetHomingOffsets',
      {},
      15,
    )
    if (!offResp?.success) {
      throw new Error(offResp?.message ?? 'Homing offset 조회 실패')
    }
    const names = offResp.joint_names ?? []
    const offsets = offResp.offsets ?? []
    const targets: { joint: string; id: number; offset: number }[] = []
    for (const joint of jointNames) {
      const idx = names.indexOf(joint)
      if (idx < 0) continue
      const offset = offsets[idx]
      if (typeof offset !== 'number' || !Number.isFinite(offset)) continue
      const id = jointNameToDxlId(joint)
      if (id == null) continue
      targets.push({ joint, id, offset })
    }
    return targets
  }

  /** effort 0 직후 호출 전 500ms 대기는 호출 측. 내부: Torque OFF → 500ms → Homing → 500ms → Torque ON → 500ms */
  const applyMotorHomingToJoints = async (jointNames: string[]) => {
    const targets = await buildMotorHomingTargets(jointNames)
    const dxlIds = [...new Set(targets.map(t => t.id))]

    for (const id of dxlIds) {
      const resp = await callSetDataToDxl(id, 'Torque Enable', 0)
      if (resp?.result === false) {
        throw new Error(`DXL ${id} Torque Enable OFF 실패`)
      }
    }
    await sleepMs(DXL_HOMING_STEP_PAUSE_MS)
    for (const t of targets) {
      const resp = await callSetDataToDxl(t.id, 'Homing Offset', t.offset)
      if (resp?.result === false) {
        throw new Error(`${t.joint} Homing Offset 적용 실패`)
      }
    }
    await sleepMs(DXL_HOMING_STEP_PAUSE_MS)
    for (const id of dxlIds) {
      const resp = await callSetDataToDxl(id, 'Torque Enable', 1)
      if (resp?.result === false) {
        throw new Error(`DXL ${id} Torque Enable ON 실패`)
      }
    }
    await sleepMs(DXL_HOMING_STEP_PAUSE_MS)
  }

  const applySafetyMotorHoming = async (
    arm: 'right' | 'left',
    prepNum: 1 | 3 | 5,
  ) => {
    const joints = safetyPrepPreviousJointNames(arm, prepNum).filter(
      j => captureResultsRef.current[j] != null,
    )
    await applyMotorHomingToJoints(joints)
  }

  const calibratedJointNames = () =>
    JOINT_ORDER.filter(j => captureResultsRef.current[j] != null)

  const rosSocketRef = useRef<WebSocket | null>(null)
  const serviceResolversRef = useRef<
    Record<string, (values: RosServiceValues) => void>
  >({})
  const statusListenersRef = useRef<StatusListener[]>([])
  const finalizeRanRef = useRef(false)
  const pendingSafetyRef = useRef<PendingSafety | null>(null)
  const lastRightEffortCmdRef = useRef<number[] | null>(null)
  const lastLeftEffortCmdRef = useRef<number[] | null>(null)
  const testJointPosePackRef = useRef<TestJointPosePack | null>(null)
  const safetyPrepPosesRef = useRef<SafetyPrepPosesPack | null>(null)
  const motionPosesRef = useRef<MotionPosesPack | null>(null)
  const [motionPosesPack, setMotionPosesPack] = useState<MotionPosesPack | null>(null)
  const [resultAngleUnit, setResultAngleUnit] = useState<ResultAngleUnit>('rad')
  const [finalizeUi, setFinalizeUi] = useState<{
    progress: number
    detail: string
    failed?: boolean
  } | null>(null)

  const waitForCalibrationTask = (
    phase: string,
    onProgress?: (progress: number, msg: CalibrationStatusMsg) => void,
  ): TaskWaiter => {
    let resolveFn: (msg: CalibrationStatusMsg) => void = () => {}
    let rejectFn: (err: Error) => void = () => {}
    const promise = new Promise<CalibrationStatusMsg>((resolve, reject) => {
      resolveFn = resolve
      rejectFn = reject
    })
    const listener: StatusListener = {
      phase,
      onProgress,
      resolve: resolveFn,
      reject: rejectFn,
    }
    statusListenersRef.current.push(listener)
    const cancel = () => {
      statusListenersRef.current =
        statusListenersRef.current.filter(l => l !== listener)
    }
    return { promise, cancel }
  }
  // 양팔 1..7: effort_controller/commands 우선, 없으면 joint_states effort
  const [torqueStates, setTorqueStates] = useState<Record<string, boolean>>(() =>
    JOINT_ORDER.reduce<Record<string, boolean>>((acc, j) => {
      acc[j] = true
      return acc
    }, {})
  )

  const isDone = currentIndex >= JOINT_ORDER.length
  const currentJoint = !calibrationStarted ? null : isDone ? null : JOINT_ORDER[currentIndex]
  const firstLeftJointIdx = JOINT_ORDER.findIndex(j => j.startsWith('arm_l_'))
  /** Result 탭: 양팔 각각 joint1 → joint7 순 */
  const resultArmJoints = (arm: 'r' | 'l') =>
    ([1, 2, 3, 4, 5, 6, 7] as const).flatMap(n => {
      const joint = `arm_${arm}_joint${n}` as (typeof JOINT_ORDER)[number]
      const idx = JOINT_ORDER.indexOf(joint)
      return idx >= 0 ? [{ joint, idx }] : []
    })
  const stretchPanels =
    activeTab === 'Information' ||
    activeTab === 'Calibration' ||
    activeTab === 'Result' ||
    activeTab === 'Test'

  const INFORMATION_ASIDE_W = 300

  const getTestJointPoseTraj = (
    jointNum: TestJointNum,
    poseIdx: TestPoseIndex,
  ): TestArmTraj => {
    const pack = testJointPosePackRef.current
    const slot = testJointPoseSlotIndex(jointNum, poseIdx)
    const base = testJointPoseTrajOffset(jointNum, poseIdx)
    if (
      pack &&
      pack.defined[slot] &&
      pack.right.length >= base + TEST_TRAJ_AXES &&
      pack.left.length >= base + TEST_TRAJ_AXES
    ) {
      return {
        right: pack.right.slice(base, base + TEST_TRAJ_AXES),
        left: pack.left.slice(base, base + TEST_TRAJ_AXES),
      }
    }
    const zero = buildAllZeroTrajectory8()
    return { right: [...zero], left: [...zero] }
  }

  const callRosService = (
    service: string,
    type: string,
    args: Record<string, unknown> = {},
    // 한 번의 요청에 허용할 최대 대기 시간(초). rosbridge 의 default 5s 를
    // 덮어쓰기 위해 함께 메시지에 포함한다.
    timeoutSec = 10,
  ) => {
    const socket = rosSocketRef.current
    if (!socket || socket.readyState !== WebSocket.OPEN) {
      return Promise.reject(new Error('ROS bridge is not connected'))
    }

    const id = `${service}-${Date.now()}-${Math.random().toString(16).slice(2)}`
    return new Promise<RosServiceValues>((resolve, reject) => {
      const localTimeout = window.setTimeout(() => {
        delete serviceResolversRef.current[id]
        reject(new Error(`${service} timeout`))
        // 클라이언트는 rosbridge 보다 약간 더 기다려준다 (네트워크 지연 여유).
      }, (timeoutSec + 5) * 1000)

      serviceResolversRef.current[id] = values => {
        window.clearTimeout(localTimeout)
        resolve(values)
      }

      socket.send(JSON.stringify({
        op: 'call_service',
        id,
        service,
        type,
        args,
        timeout: timeoutSec,
      }))
    })
  }

  useEffect(() => {
    const host = window.location.hostname || 'localhost'
    const url = window.location.protocol === 'https:'
      ? `wss://${window.location.host}/ws/`
      : `ws://${host}:9090`
    const socket = new WebSocket(url)
    rosSocketRef.current = socket

    socket.onopen = () => {
      socket.send(JSON.stringify({
        op: 'subscribe',
        topic: '/joint_states',
        type: 'sensor_msgs/msg/JointState',
        throttle_rate: 100,
      }))
      socket.send(JSON.stringify({
        op: 'subscribe',
        topic: RIGHT_ARM_EFFORT_CMD_TOPIC,
        type: 'std_msgs/msg/Float64MultiArray',
      }))
      socket.send(JSON.stringify({
        op: 'subscribe',
        topic: LEFT_ARM_EFFORT_CMD_TOPIC,
        type: 'std_msgs/msg/Float64MultiArray',
      }))
      socket.send(JSON.stringify({
        op: 'subscribe',
        topic: '/calibration/status',
        type: 'ffw_calibration/msg/CalibrationStatus',
      }))
      socket.send(JSON.stringify({
        op: 'call_service',
        id: 'get_calibration_config',
        service: '/calibration/get_config',
        type: 'ffw_calibration/srv/GetCalibrationConfig',
        args: {},
      }))
      socket.send(JSON.stringify({
        op: 'call_service',
        id: 'get_test_joint_poses',
        service: '/calibration/get_test_joint_poses',
        type: 'ffw_calibration/srv/GetTestJointPoses',
        args: {},
      }))
      socket.send(JSON.stringify({
        op: 'call_service',
        id: 'get_safety_prep_poses',
        service: '/calibration/get_safety_prep_poses',
        type: 'ffw_calibration/srv/GetSafetyPrepPoses',
        args: {},
      }))
      socket.send(JSON.stringify({
        op: 'call_service',
        id: 'get_motion_poses',
        service: '/calibration/get_motion_poses',
        type: 'ffw_calibration/srv/GetMotionPoses',
        args: {},
      }))
    }

    socket.onmessage = event => {
      const data = JSON.parse(event.data) as RosbridgeMessage

      if (data.op === 'publish' && data.topic === '/joint_states' && data.msg) {
        const names = data.msg.name ?? []
        const positions = data.msg.position ?? []
        const efforts = data.msg.effort
        setJointPositions(current => {
          const updated = { ...current }
          names.forEach((name, index) => {
            const position = positions[index]
            if (typeof position === 'number') updated[name] = position
          })
          return updated
        })
        const hasEffort =
          Array.isArray(efforts) && efforts.length === names.length
        setTorqueStates(prev => {
          const next = { ...prev }
          names.forEach((name, index) => {
            if (!ARM_TORQUE_JOINT_RE.test(name)) return
            const m = name.match(/^arm_(r|l)_joint(\d+)$/)
            if (!m) return
            const armIdx = Number(m[2]) - 1
            const cmdVec =
              m[1] === 'r'
                ? lastRightEffortCmdRef.current
                : lastLeftEffortCmdRef.current
            if (
              cmdVec &&
              armIdx >= 0 &&
              armIdx < cmdVec.length &&
              typeof cmdVec[armIdx] === 'number'
            ) {
              next[name] =
                Math.abs(cmdVec[armIdx]) > TORQUE_EFFORT_ABS_EPS
              return
            }
            if (hasEffort) {
              const e = efforts[index]
              if (typeof e === 'number' && Number.isFinite(e)) {
                next[name] = Math.abs(e) > TORQUE_EFFORT_ABS_EPS
              }
            }
          })
          return next
        })
      }

      if (
        data.op === 'publish' &&
        data.topic === RIGHT_ARM_EFFORT_CMD_TOPIC &&
        data.msg
      ) {
        const cmd = normalizeEffortCommand7(data.msg.data)
        if (cmd) {
          lastRightEffortCmdRef.current = cmd
        }
      }

      if (
        data.op === 'publish' &&
        data.topic === LEFT_ARM_EFFORT_CMD_TOPIC &&
        data.msg
      ) {
        const cmd = normalizeEffortCommand7(data.msg.data)
        if (cmd) {
          lastLeftEffortCmdRef.current = cmd
        }
      }

      if (
        data.op === 'service_response' &&
        data.id === 'get_calibration_config' &&
        data.values?.success
      ) {
        const joints = data.values.joint_names ?? []
        const targets = data.values.target_rads ?? []
        setTargetRads(
          Object.fromEntries(
            joints.map((joint, index) => [joint, targets[index] ?? 0])
          )
        )
      }

      if (
        data.op === 'service_response' &&
        data.id === 'get_test_joint_poses' &&
        data.values?.success
      ) {
        const defined = data.values.defined ?? []
        const right = data.values.right_targets ?? []
        const left = data.values.left_targets ?? []
        if (
          defined.length >= TEST_JOINT_POSE_SLOT_COUNT &&
          right.length >= TEST_TRAJ_PACKED_SIZE &&
          left.length >= TEST_TRAJ_PACKED_SIZE
        ) {
          const defaultDurationSec =
            typeof data.values.default_duration_sec === 'number'
              ? data.values.default_duration_sec
              : DEFAULT_TEST_JOINT_TRAJECTORY_SEC
          const poseDurationRaw = data.values.pose_duration_sec ?? []
          const poseDurationSec = Array.from(
            { length: TEST_JOINT_POSE_SLOT_COUNT },
            (_, i) =>
              typeof poseDurationRaw[i] === 'number'
                ? poseDurationRaw[i]
                : defaultDurationSec,
          )
          testJointPosePackRef.current = {
            defaultDurationSec,
            poseDurationSec,
            defined: defined.slice(0, TEST_JOINT_POSE_SLOT_COUNT),
            right: right.slice(0, TEST_TRAJ_PACKED_SIZE),
            left: left.slice(0, TEST_TRAJ_PACKED_SIZE),
          }
        }
      }

      if (
        data.op === 'service_response' &&
        data.id === 'get_safety_prep_poses' &&
        data.values?.success
      ) {
        safetyPrepPosesRef.current = {
          beforeJoint1: {
            durationSec:
              typeof data.values.before_joint1_duration_sec === 'number'
                ? data.values.before_joint1_duration_sec
                : DEFAULT_SAFETY_PREP_MOVE_SEC,
            right: parseTraj8(data.values.before_joint1_right),
            left: parseTraj8(data.values.before_joint1_left),
          },
          beforeJoint3: {
            durationSec:
              typeof data.values.before_joint3_duration_sec === 'number'
                ? data.values.before_joint3_duration_sec
                : DEFAULT_SAFETY_PREP_MOVE_SEC,
            right: parseTraj8(data.values.before_joint3_right),
            left: parseTraj8(data.values.before_joint3_left),
          },
          beforeJoint5: {
            durationSec:
              typeof data.values.before_joint5_duration_sec === 'number'
                ? data.values.before_joint5_duration_sec
                : DEFAULT_SAFETY_PREP_MOVE_SEC,
            right: parseTraj8(data.values.before_joint5_right),
            left: parseTraj8(data.values.before_joint5_left),
          },
        }
      }

      if (
        data.op === 'service_response' &&
        data.id === 'get_motion_poses' &&
        data.values?.success
      ) {
        const pack: MotionPosesPack = {
          zeroPoseDurationSec:
            typeof data.values.zero_pose_duration_sec === 'number'
              ? data.values.zero_pose_duration_sec
              : DEFAULT_ZERO_POSE_DURATION_SEC,
          etPoseDurationSec:
            typeof data.values.et_pose_duration_sec === 'number'
              ? data.values.et_pose_duration_sec
              : DEFAULT_ET_POSE_DURATION_SEC,
          zeroPose: {
            right: parseTraj8(data.values.zero_pose_right),
            left: parseTraj8(data.values.zero_pose_left),
          },
        }
        if (data.values.lift_up_defined) {
          pack.liftUp = {
            durationSec: data.values.lift_up_duration_sec ?? 0,
            right: parseTraj8(data.values.lift_up_right),
            left: parseTraj8(data.values.lift_up_left),
          }
        }
        if (data.values.lift_down_defined) {
          pack.liftDown = {
            durationSec: data.values.lift_down_duration_sec ?? 0,
            right: parseTraj8(data.values.lift_down_right),
            left: parseTraj8(data.values.lift_down_left),
          }
        }
        motionPosesRef.current = pack
        setMotionPosesPack(pack)
      }

      if (data.op === 'service_response' && data.id) {
        const resolver = serviceResolversRef.current[data.id]
        if (resolver) {
          delete serviceResolversRef.current[data.id]
          resolver(data.values ?? {})
        }
      }

      if (
        data.op === 'publish' &&
        data.topic === '/calibration/status' &&
        data.msg
      ) {
        const msg = data.msg as CalibrationStatusMsg
        const phase = msg.phase ?? ''
        const progress = typeof msg.progress === 'number' ? msg.progress : 0
        const result = msg.result ?? ''
        if (phase === 'finalize_calibration') {
          setFinalizeUi({
            progress: result === 'done' ? 1 : progress,
            detail: msg.message ?? '',
            failed: result === 'failed',
          })
        }
        statusListenersRef.current = statusListenersRef.current.filter(l => {
          if (l.phase !== phase) return true
          l.onProgress?.(progress, msg)
          if (result === 'done') {
            l.resolve(msg)
            return false
          }
          if (result === 'failed') {
            l.reject(new Error(msg.message || `${phase} failed`))
            return false
          }
          return true
        })
      }
    }

    return () => {
      if (socket.readyState === WebSocket.OPEN) {
        socket.send(JSON.stringify({ op: 'unsubscribe', topic: '/joint_states' }))
        socket.send(
          JSON.stringify({ op: 'unsubscribe', topic: RIGHT_ARM_EFFORT_CMD_TOPIC }),
        )
        socket.send(
          JSON.stringify({ op: 'unsubscribe', topic: LEFT_ARM_EFFORT_CMD_TOPIC }),
        )
        socket.send(JSON.stringify({ op: 'unsubscribe', topic: '/calibration/status' }))
      }
      socket.close()
      rosSocketRef.current = null
    }
  }, [])

  useEffect(() => {
    if (!isDone) {
      finalizeRanRef.current = false
      setFinalizeUi(null)
      return
    }
    if (!calibrationStarted || finalizeRanRef.current) return
    finalizeRanRef.current = true

    setFinalizeUi({ progress: 0, detail: '저장 중…' })

    const zeroEffortPayload = (arm: 'right' | 'left') => ({
      arm,
      target: [...SAFETY_ZERO_EFFORT_TARGET],
      duration_sec: SAFETY_TORQUE_RELEASE_SEC,
      effort_joint_2467_preset: false,
      effort_hold_2467_ramp_joint1: false,
      effort_hold_all_ramp_joint3: false,
      safety_prep_joint_number: 0,
    })

    const allJointsOnEffortPayload = (arm: 'right' | 'left') => ({
      arm,
      target: [] as number[],
      duration_sec: JOINT7_SAFETY_EFFORT_RAMP_SEC,
      effort_joint_2467_preset: false,
      effort_hold_2467_ramp_joint1: false,
      effort_hold_all_ramp_joint3: false,
      safety_prep_joint_number: 0,
    })

    const motion = motionPosesRef.current
    const zeroTraj8 = motion
      ? [...motion.zeroPose.right]
      : [...buildAllZeroTrajectory8()]
    const zeroTraj8Left = motion ? [...motion.zeroPose.left] : [...buildAllZeroTrajectory8()]
    const zeroDurationSec =
      motion?.zeroPoseDurationSec ?? DEFAULT_ZERO_POSE_DURATION_SEC
    const trajBothZeroArgs = {
      arm: 'both' as const,
      duration_sec: zeroDurationSec,
      use_mid_zero_target: false,
      right_targets: zeroTraj8,
      left_targets: zeroTraj8Left,
    }
    const trajTimeoutSec = Math.max(15, zeroDurationSec + 25)

    let cancelled = false

    void (async () => {
      const bump = (progress: number, detail: string) => {
        if (!cancelled) {
          setFinalizeUi({ progress, detail, failed: false })
        }
      }
      try {
        const respFinalize = await callRosService(
          '/calibration/finalize_calibration',
          'std_srvs/srv/Trigger',
          {},
          90,
        )
        if (cancelled) return
        if (!respFinalize?.success) {
          throw new Error(respFinalize?.message ?? 'finalize_calibration failed')
        }
        bump(0.08, '저장 완료 · 마무리 중…')

        const wRRelease = waitForCalibrationTask('apply_effort')
        const rRRelease = await callRosService(
          '/calibration/apply_effort',
          'ffw_calibration/srv/ApplyEffort',
          zeroEffortPayload('right'),
          SAFETY_TORQUE_RELEASE_SEC + 60,
        )
        if (cancelled) return
        if (!rRRelease?.success) {
          throw new Error(rRRelease?.message ?? '오른팔 effort 해제 실패')
        }
        await wRRelease.promise

        bump(0.22, '마무리 중…')
        const wLRelease = waitForCalibrationTask('apply_effort')
        const rLRelease = await callRosService(
          '/calibration/apply_effort',
          'ffw_calibration/srv/ApplyEffort',
          zeroEffortPayload('left'),
          SAFETY_TORQUE_RELEASE_SEC + 60,
        )
        if (cancelled) return
        if (!rLRelease?.success) {
          throw new Error(rLRelease?.message ?? '왼팔 effort 해제 실패')
        }
        await wLRelease.promise

        bump(0.28, '마무리 중…')
        await sleepMs(DXL_HOMING_STEP_PAUSE_MS)
        await applyMotorHomingToJoints(calibratedJointNames())

        bump(0.36, '마무리 중…')
        const wTraj = waitForCalibrationTask('move_trajectory', prog => {
          if (!cancelled) {
            bump(0.36 + prog * 0.28, '마무리 중…')
          }
        })
        const trajResp = await callRosService(
          '/calibration/move_arm_trajectory',
          'ffw_calibration/srv/MoveArmTrajectory',
          trajBothZeroArgs,
          trajTimeoutSec,
        )
        if (cancelled) return
        if (!trajResp?.success) {
          throw new Error(trajResp?.message ?? '양팔 0 궤적 실패')
        }
        await wTraj.promise

        bump(0.68, '마무리 중…')
        const wROn = waitForCalibrationTask('apply_effort')
        const rROn = await callRosService(
          '/calibration/apply_effort',
          'ffw_calibration/srv/ApplyEffort',
          allJointsOnEffortPayload('right'),
          JOINT7_SAFETY_EFFORT_RAMP_SEC + 60,
        )
        if (cancelled) return
        if (!rROn?.success) {
          throw new Error(rROn?.message ?? '오른팔 effort 램프 실패')
        }
        await wROn.promise

        bump(0.84, '마무리 중…')
        const wLOn = waitForCalibrationTask('apply_effort')
        const rLOn = await callRosService(
          '/calibration/apply_effort',
          'ffw_calibration/srv/ApplyEffort',
          allJointsOnEffortPayload('left'),
          JOINT7_SAFETY_EFFORT_RAMP_SEC + 60,
        )
        if (cancelled) return
        if (!rLOn?.success) {
          throw new Error(rLOn?.message ?? '왼팔 effort 램프 실패')
        }
        await wLOn.promise

        if (cancelled) return
        bump(1, '마무리 완료')
        setActiveTab('Result')
      } catch (error) {
        if (!cancelled) {
          console.error(error)
          window.alert(
            error instanceof Error ? error.message : '마무리 실패',
          )
          setFinalizeUi(prev => ({
            progress: prev?.progress ?? 0,
            detail: error instanceof Error ? error.message : '마무리 실패',
            failed: true,
          }))
        }
      }
    })()

    return () => {
      cancelled = true
    }
  }, [calibrationStarted, isDone])

  useEffect(() => {
    pendingSafetyRef.current = pendingSafety
  }, [pendingSafety])

  useEffect(() => {
    if (pendingSafety) {
      setSafetyArmMoveBusy(false)
      setSafetyEffortZeroed(false)
      setSafetyTorqueReleased(false)
      setSafetyTorqueReleaseBusy(false)
      setSafetyHomingBusy(false)
    }
    setSafetyApplyProgress(0)
  }, [pendingSafety])

  // 진행률 바는 ROS /calibration/status 토픽에서 직접 동기화한다.
  // fake 타이머는 더 이상 사용하지 않는다.

  const clearJointCapture = (jointKey: string) => {
    setCapturedPositions(prev => {
      const next = { ...prev }
      delete next[jointKey]
      return next
    })
    setCaptureResults(prev => {
      const next = { ...prev }
      delete next[jointKey]
      return next
    })
  }

  const resetJointToPending = (jointKey: string) => {
    const idx = JOINT_ORDER.indexOf(jointKey as (typeof JOINT_ORDER)[number])
    if (idx < 0) return
    setStatuses(prev => {
      const n = [...prev]
      n[idx] = 'pending'
      return n
    })
    clearJointCapture(jointKey)
  }

  const enterSafetyPrep = (arm: 'right' | 'left', stage: SafetyStage) => {
    const prepJoint = nextJointKeyAfterSafety(arm, stage)
    const prepIdx = JOINT_ORDER.indexOf(prepJoint)
    setPendingSafety({ arm, stage })
    if (prepIdx >= 0) {
      setCurrentIndex(prepIdx)
    }
  }

  const skipSafetyPrep = () => {
    if (!pendingSafety) return
    if (safetyArmMoveBusy || safetyTorqueReleaseBusy || safetyHomingBusy) return

    const { arm, stage } = pendingSafety
    const skippedJoint = nextJointKeyAfterSafety(arm, stage)
    const skippedIdx = JOINT_ORDER.indexOf(skippedJoint)

    setSafetyArmMoveBusy(false)
    setSafetyTorqueReleaseBusy(false)
    setSafetyHomingBusy(false)
    clearJointCapture(skippedJoint)

    if (skippedIdx >= 0) {
      setStatuses(prev => {
        const n = [...prev]
        n[skippedIdx] = 'skipped'
        return n
      })
    }

    if (stage === 'before_joint1') {
      enterSafetyPrep(arm, 'before_joint3')
      return
    }
    if (stage === 'before_joint3') {
      enterSafetyPrep(arm, 'before_joint5')
      return
    }

    setPendingSafety(null)
    const nextIdx = skippedIdx >= 0 ? skippedIdx + 1 : currentIndex + 1
    setCurrentIndex(Math.min(nextIdx, JOINT_ORDER.length))
  }

  const advance = (status: Status) => {
    if (isDone) return
    const leavingJoint = JOINT_ORDER[currentIndex]
    setStatuses(prev => {
      const next = [...prev]
      next[currentIndex] = status
      return next
    })
    if (leavingJoint === 'arm_r_joint7') {
      enterSafetyPrep('right', 'before_joint1')
      return
    }
    if (leavingJoint === 'arm_l_joint7') {
      enterSafetyPrep('left', 'before_joint1')
      return
    }
    if (leavingJoint === 'arm_r_joint1') {
      enterSafetyPrep('right', 'before_joint3')
      return
    }
    if (leavingJoint === 'arm_l_joint1') {
      enterSafetyPrep('left', 'before_joint3')
      return
    }
    if (leavingJoint === 'arm_r_joint3') {
      enterSafetyPrep('right', 'before_joint5')
      return
    }
    if (leavingJoint === 'arm_l_joint3') {
      enterSafetyPrep('left', 'before_joint5')
      return
    }
    setCurrentIndex(i => i + 1)
  }

  const canGoBack =
    calibrationStarted &&
    (pendingSafety !== null || isDone || currentIndex > 0)

  const goToPreviousCalibrationStep = () => {
    if (!calibrationStarted || !canGoBack) return

    if (pendingSafety) {
      const { arm, stage } = pendingSafety
      setSafetyArmMoveBusy(false)
      const prevStage = previousSafetyStage(stage)
      if (prevStage != null) {
        resetJointToPending(nextJointKeyAfterSafety(arm, stage))
        resetJointToPending(nextJointKeyAfterSafety(arm, prevStage))
        enterSafetyPrep(arm, prevStage)
        return
      }
      setPendingSafety(null)
      resetJointToPending(nextJointKeyAfterSafety(arm, stage))
      const prevJoint = previousJointKeyBeforeSafety(arm, stage)
      const prevIdx = JOINT_ORDER.indexOf(prevJoint)
      if (prevIdx >= 0) {
        setCurrentIndex(prevIdx)
        resetJointToPending(prevJoint)
      }
      return
    }

    if (isDone) {
      const last = JOINT_ORDER.length - 1
      const j = JOINT_ORDER[last]
      const lastSafety = safetyStageForJointKey(j)
      if (lastSafety) {
        resetJointToPending(j)
        enterSafetyPrep(lastSafety.arm, lastSafety.stage)
        return
      }
      setCurrentIndex(last)
      resetJointToPending(j)
      return
    }

    if (currentIndex <= 0) return

    const leavingJoint = JOINT_ORDER[currentIndex]
    const leavingSafety = safetyStageForJointKey(leavingJoint)
    if (leavingSafety) {
      resetJointToPending(leavingJoint)
      enterSafetyPrep(leavingSafety.arm, leavingSafety.stage)
      return
    }

    const newIdx = currentIndex - 1
    const prevJoint = JOINT_ORDER[newIdx]
    const prevSafety = safetyStageForJointKey(prevJoint)

    resetJointToPending(leavingJoint)
    resetJointToPending(prevJoint)

    if (prevSafety) {
      enterSafetyPrep(prevSafety.arm, prevSafety.stage)
      return
    }

    setCurrentIndex(newIdx)
  }

  const handleTestJointPose = async (jointNum: TestJointNum, poseIdx: TestPoseIndex) => {
    const busyKey = `j${jointNum}_p${poseIdx}`
    if (activeTestTraj) return
    setTestTabSelection(jointNum)
    setTestSelectedPoseIdx(poseIdx)
    setActiveTestTraj(busyKey)

    const traj = getTestJointPoseTraj(jointNum, poseIdx)
    const waiter = waitForCalibrationTask('move_trajectory')
    const slot = testJointPoseSlotIndex(jointNum, poseIdx)
    const pack = testJointPosePackRef.current
    const trajDurationSec =
      pack?.poseDurationSec[slot] ?? pack?.defaultDurationSec ?? DEFAULT_TEST_JOINT_TRAJECTORY_SEC
    const trajTimeoutSec = Math.max(15, trajDurationSec + 25)
    try {
      const startResp = await callRosService(
        '/calibration/move_arm_trajectory',
        'ffw_calibration/srv/MoveArmTrajectory',
        {
          arm: 'both',
          duration_sec: trajDurationSec,
          use_mid_zero_target: false,
          right_targets: [...traj.right],
          left_targets: [...traj.left],
        },
        trajTimeoutSec,
      )
      if (!startResp?.success) {
        waiter.cancel()
        throw new Error(
          startResp?.message ?? `Joint ${jointNum} ${TEST_JOINT_POSE_LABELS[poseIdx]} failed`,
        )
      }
      await waiter.promise
    } catch (error) {
      waiter.cancel()
      console.error(error)
      window.alert(
        error instanceof Error
          ? error.message
          : `Joint ${jointNum} ${TEST_JOINT_POSE_LABELS[poseIdx]} failed`,
      )
    } finally {
      setActiveTestTraj(null)
    }
  }

  const handleTestCustomArmNudge = async (
    arm: 'right' | 'left',
    jointNum: TestCustomArmJointNum,
    direction: -1 | 1,
  ) => {
    const busyKey = `custom_${arm}_j${jointNum}_${direction > 0 ? 'p' : 'm'}`
    if (activeTestTraj) return
    const step = TEST_CUSTOM_STEP_OPTIONS.find(o => o.deg === testCustomStepDeg)
    if (!step) return

    setActiveTestTraj(busyKey)
    const deltaRad = direction * step.deg * DEG_TO_RAD
    const armChar = arm === 'right' ? 'r' : 'l'
    const targets8 = buildCustomArmTargets8(
      armChar,
      jointPositionsRef.current,
      jointNum,
      deltaRad,
    )
    const durationSec = step.timeFromStartSec
    const isInstantStep = durationSec <= 0
    const trajTimeoutSec = isInstantStep ? 10 : Math.max(15, durationSec + 25)
    const waiter = waitForCalibrationTask('move_trajectory')
    try {
      const startResp = await callRosService(
        '/calibration/move_arm_trajectory',
        'ffw_calibration/srv/MoveArmTrajectory',
        arm === 'right'
          ? {
              arm: 'right',
              duration_sec: durationSec,
              use_mid_zero_target: false,
              right_targets: targets8,
              left_targets: [] as number[],
            }
          : {
              arm: 'left',
              duration_sec: durationSec,
              use_mid_zero_target: false,
              right_targets: [] as number[],
              left_targets: targets8,
            },
        trajTimeoutSec,
      )
      if (!startResp?.success) {
        waiter.cancel()
        throw new Error(
          startResp?.message ??
            `${arm === 'right' ? 'RIGHT' : 'LEFT'} J${jointNum} nudge failed`,
        )
      }
      // 0초(instant): 서비스가 "task started"만 반환. 완료 status까지 기다리면
      // activeTestTraj 때문에 버튼이 ~250ms 잠김.
      if (isInstantStep) {
        waiter.cancel()
        const jointName = customArmJointRosName(armChar, jointNum)
        setJointPositions(prev => ({
          ...prev,
          [jointName]: (prev[jointName] ?? 0) + deltaRad,
        }))
      } else {
        await waiter.promise
      }
    } catch (error) {
      waiter.cancel()
      console.error(error)
      window.alert(
        error instanceof Error
          ? error.message
          : `${arm === 'right' ? 'RIGHT' : 'LEFT'} J${jointNum} nudge failed`,
      )
    } finally {
      setActiveTestTraj(null)
    }
  }

  const handleTestEtPose = async () => {
    if (activeTestTraj) return
    if (Object.keys(targetRads).length === 0) {
      window.alert('캘리브레이션 목표 자세를 불러오지 못했습니다. ROS 연결을 확인하세요.')
      return
    }

    const etDurationSec =
      motionPosesRef.current?.etPoseDurationSec ?? DEFAULT_ET_POSE_DURATION_SEC

    setActiveTestTraj('et_pose')
    const waiter = waitForCalibrationTask(TEST_ET_POSE_PHASE)
    const timeoutSec = Math.max(20, etDurationSec + 25)
    try {
      const startResp = await callRosService(
        '/calibration/move_to_pose',
        'ffw_calibration/srv/MoveToPose',
        {
          duration_sec: etDurationSec,
          right_positions: buildEtPoseArmTargets8(targetRads, 'r'),
          left_positions: buildEtPoseArmTargets8(targetRads, 'l'),
          enable: [],
          label: 'et_pose',
        },
        timeoutSec,
      )
      if (!startResp?.success) {
        waiter.cancel()
        throw new Error(startResp?.message ?? 'ET Pose failed to start')
      }
      await waiter.promise
    } catch (error) {
      waiter.cancel()
      console.error(error)
      window.alert(error instanceof Error ? error.message : 'ET Pose 실패')
    } finally {
      setActiveTestTraj(null)
    }
  }

  const handleTestZeroPose = async () => {
    if (activeTestTraj) return
    const durationSec =
      motionPosesRef.current?.zeroPoseDurationSec ?? DEFAULT_ZERO_POSE_DURATION_SEC
    setActiveTestTraj('zero_pose')
    const waiter = waitForCalibrationTask('zero_pose')
    const timeoutSec = Math.max(15, durationSec + 25)
    try {
      const startResp = await callRosService(
        '/calibration/move_to_zero_pose',
        'ffw_calibration/srv/MoveToZeroPose',
        {
          arm: 'both',
          duration_sec: durationSec,
          enable: [],
        },
        timeoutSec,
      )
      if (!startResp?.success) {
        waiter.cancel()
        throw new Error(startResp?.message ?? 'Zero Pose failed to start')
      }
      await waiter.promise
    } catch (error) {
      waiter.cancel()
      console.error(error)
      window.alert(error instanceof Error ? error.message : 'Zero Pose 실패')
    } finally {
      setActiveTestTraj(null)
    }
  }

  const handleTestLiftPose = async (direction: 'up' | 'down') => {
    if (activeTestTraj) return
    const motion = motionPosesRef.current
    const block = direction === 'up' ? motion?.liftUp : motion?.liftDown
    if (!block) {
      window.alert('Lift pose가 config/common/robot_poses.yaml (common)에 정의되지 않았습니다.')
      return
    }
    const busyKey = direction === 'up' ? 'lift_up' : 'lift_down'
    setActiveTestTraj(busyKey)
    const durationSec = block.durationSec
    const isInstant = durationSec <= 0
    const trajTimeoutSec = isInstant ? 10 : Math.max(15, durationSec + 25)
    const waiter = waitForCalibrationTask('move_trajectory')
    try {
      const startResp = await callRosService(
        '/calibration/move_arm_trajectory',
        'ffw_calibration/srv/MoveArmTrajectory',
        {
          arm: 'both',
          duration_sec: durationSec,
          use_mid_zero_target: false,
          right_targets: [...block.right],
          left_targets: [...block.left],
        },
        trajTimeoutSec,
      )
      if (!startResp?.success) {
        waiter.cancel()
        throw new Error(startResp?.message ?? `Lift ${direction} failed to start`)
      }
      if (isInstant) {
        waiter.cancel()
      } else {
        await waiter.promise
      }
    } catch (error) {
      waiter.cancel()
      console.error(error)
      window.alert(
        error instanceof Error ? error.message : `Lift ${direction} 실패`,
      )
    } finally {
      setActiveTestTraj(null)
    }
  }

  const handleCalibrate = async () => {
    if (!currentJoint) return
    const capturedJoint = currentJoint
    try {
      const result = await callRosService(
        '/calibration/capture_joint',
        'ffw_calibration/srv/CaptureJoint',
        { joint: capturedJoint },
      )
      if (!result?.success) {
        throw new Error(result?.message ?? 'Calibration failed')
      }
      // capture 직후 측정값을 고정 저장 (서비스 응답에 measured_rad 가 있으면
      // 그걸, 없으면 마지막으로 받은 joint_states 값 사용).
      const measuredFromService =
        typeof result?.measured_rad === 'number' ? result.measured_rad : undefined
      const measuredFromTopic = jointPositions[capturedJoint]
      const frozen = measuredFromService ?? measuredFromTopic
      if (typeof frozen === 'number') {
        setCapturedPositions(prev => ({ ...prev, [capturedJoint]: frozen }))
      }
      const targetRad =
        typeof result?.target_rad === 'number'
          ? result.target_rad
          : targetRads[capturedJoint]
      if (typeof frozen === 'number' && typeof targetRad === 'number') {
        const deltaRad =
          typeof result?.delta_rad === 'number'
            ? result.delta_rad
            : targetRad - frozen
        setCaptureResults(prev => ({
          ...prev,
          [capturedJoint]: {
            measuredRad: frozen,
            targetRad,
            deltaRad,
          },
        }))
      }
      advance('captured')
    } catch (error) {
      console.error(error)
      window.alert(error instanceof Error ? error.message : 'Calibrate 실패')
    }
  }

  const renderJoint = (joint: string, i: number) => {
    const isCurrent = calibrationStarted && i === currentIndex
    const status = statuses[i]
    const shortName = joint.replace(/^arm_[rl]_/, '')

    const dotColor = isCurrent ? '#3b82f6' : STATUS_COLOR[status]
    const textColor =
      isCurrent ? '#fff'
        : status === 'captured' ? '#d1d5db'
        : status === 'skipped' ? '#6b7280'
        : '#9ca3af'

    return (
      <div
        key={joint}
        style={{
          display: 'flex',
          alignItems: 'center',
          gap: '14px',
          width: '100%',
          boxSizing: 'border-box',
          padding: '8px 14px',
          borderRadius: '8px',
          backgroundColor: isCurrent ? 'rgba(59, 130, 246, 0.12)' : 'transparent',
          borderLeft: isCurrent ? '3px solid #3b82f6' : '3px solid transparent',
          opacity: status === 'skipped' ? 0.45 : 1,
          transition: 'background-color 0.15s, border-color 0.15s, opacity 0.15s',
        }}
      >
        <span
          style={{
            width: '10px',
            height: '10px',
            borderRadius: '50%',
            backgroundColor: status === 'skipped' ? 'transparent' : dotColor,
            border: status === 'skipped' ? '2px solid #737373' : 'none',
            boxSizing: 'border-box',
            flexShrink: 0,
            boxShadow: isCurrent ? '0 0 0 3px rgba(59, 130, 246, 0.25)' : 'none',
          }}
        />
        <span
          style={{
            fontFamily: 'ui-monospace, SFMono-Regular, Menlo, monospace',
            fontSize: '14px',
            color: textColor,
            fontWeight: isCurrent ? 600 : 400,
          }}
        >
          {shortName}
        </span>
      </div>
    )
  }

  const renderHeadJoint = (joint: string) => {
    const shortName = joint.replace(/^head_/, '')

    return (
      <div
        key={joint}
        style={{
          display: 'flex',
          alignItems: 'center',
          gap: '14px',
          width: '100%',
          boxSizing: 'border-box',
          padding: '8px 14px',
          borderRadius: '8px',
          borderLeft: '3px solid transparent',
          opacity: 0.72,
        }}
      >
        <span
          style={{
            width: '10px',
            height: '10px',
            borderRadius: '50%',
            backgroundColor: 'transparent',
            border: '2px solid #525252',
            boxSizing: 'border-box',
            flexShrink: 0,
          }}
        />
        <span
          style={{
            fontFamily: 'ui-monospace, SFMono-Regular, Menlo, monospace',
            fontSize: '14px',
            color: '#9ca3af',
            fontWeight: 400,
          }}
        >
          {shortName}
        </span>
      </div>
    )
  }

  const sectionHeader = (label: string, compact = false) => (
    <div
      style={{
        fontSize: '12px',
        fontWeight: 700,
        letterSpacing: '0.15em',
        color: '#9ca3af',
        padding: compact ? '0 4px' : '0 14px',
        marginBottom: '4px',
        flexShrink: 0,
      }}
    >
      {label}
    </div>
  )

  /** 팔별 joint list: 확대/축소 시에도 패널 높이를 균등하게 채움 */
  const jointArmListBlock: CSSProperties = {
    flex: 1,
    minHeight: 0,
    display: 'flex',
    flexDirection: 'column',
  }
  const jointArmListRows: CSSProperties = {
    flex: 1,
    minHeight: 0,
    display: 'flex',
    flexDirection: 'column',
    gap: '2px',
  }
  const jointArmListRowSlot: CSSProperties = {
    flex: 1,
    display: 'flex',
    alignItems: 'center',
    minHeight: 0,
    width: '100%',
  }

  /** 오른쪽 상태 패널: 숫자 열(STATE·CAP·Δ)은 고정 폭으로 붙여 표시 */
  const STATE_PANEL_JOINT_COL_W = 40
  const STATE_PANEL_NUM_COL_W = 68
  const STATE_PANEL_DELTA_COL_W = 58
  const STATE_PANEL_GRID = `26px ${STATE_PANEL_JOINT_COL_W}px ${STATE_PANEL_NUM_COL_W}px ${STATE_PANEL_NUM_COL_W}px ${STATE_PANEL_DELTA_COL_W}px`
  /** grid + columnGap(8) + row padding(10) + aside padding(16) + border(2) */
  const CALIB_STATE_ASIDE_W =
    26 +
    STATE_PANEL_JOINT_COL_W +
    STATE_PANEL_NUM_COL_W * 2 +
    STATE_PANEL_DELTA_COL_W +
    8 +
    10 +
    16 +
    2
  const calibCenterShell: CSSProperties = {
    flex: 1,
    minHeight: 0,
    width: '100%',
    alignSelf: 'stretch',
    display: 'flex',
    flexDirection: 'column',
  }
  const calibCardShell: CSSProperties = {
    flex: 1,
    minHeight: 0,
    width: '100%',
    display: 'flex',
    flexDirection: 'column',
    padding: '28px 32px',
    backgroundColor: '#fff',
    border: '1px solid #fecaca',
    borderTop: '6px solid #ef4444',
    borderRadius: '12px',
    boxShadow: '0 12px 28px rgba(15, 23, 42, 0.08)',
    boxSizing: 'border-box',
  }
  const statePanelRow: CSSProperties = {
    display: 'grid',
    gridTemplateColumns: STATE_PANEL_GRID,
    columnGap: '2px',
    alignItems: 'center',
    boxSizing: 'border-box',
    width: '100%',
    maxWidth: '100%',
    padding: '6px 6px 6px 4px',
    borderLeft: '3px solid transparent',
    overflow: 'visible',
  }
  const statePanelCell: CSSProperties = {
    display: 'flex',
    alignItems: 'center',
    justifyContent: 'center',
    minWidth: 0,
    maxWidth: '100%',
    overflow: 'hidden',
    textAlign: 'center',
  }
  const statePanelHeaderCell: CSSProperties = {
    ...statePanelCell,
    fontSize: '10px',
    fontWeight: 700,
    letterSpacing: '0.06em',
    color: '#6b7280',
    lineHeight: 1.2,
  }
  const statePanelMonoCell: CSSProperties = {
    ...statePanelCell,
    fontFamily: 'ui-monospace, SFMono-Regular, Menlo, monospace',
    fontSize: '12px',
    fontVariantNumeric: 'tabular-nums',
    lineHeight: 1.2,
  }
  const statePanelJointCell: CSSProperties = {
    ...statePanelMonoCell,
    overflow: 'visible',
    maxWidth: 'none',
    padding: '0 4px',
    boxSizing: 'border-box',
  }
  const statePanelValueText: CSSProperties = {
    overflow: 'hidden',
    textOverflow: 'ellipsis',
    whiteSpace: 'nowrap',
    maxWidth: '100%',
    display: 'block',
  }

  const STATE_PANEL_HEADERS: { label: string; title?: string }[] = [
    { label: 'T' },
    { label: 'JOINT' },
    { label: 'STATE', title: '실시간 각도 (rad)' },
    { label: 'CAP', title: '캡처 각도 (rad)' },
    { label: 'Δ', title: '목표까지 (rad)' },
  ]

  const stateColumnHeader = () => (
    <div style={{ ...statePanelRow, marginBottom: '4px', flexShrink: 0 }}>
      {STATE_PANEL_HEADERS.map(({ label, title }) => (
        <div
          key={label}
          style={
            label === 'JOINT'
              ? { ...statePanelHeaderCell, ...statePanelJointCell, letterSpacing: '0.04em' }
              : statePanelHeaderCell
          }
          title={title}
        >
          {label}
        </div>
      ))}
    </div>
  )

  const renderJointState = (joint: string, i: number) => {
    const isCurrent = calibrationStarted && i === currentIndex
    const status = statuses[i]
    const shortName = jointDisplayNumber(joint)

    // STATE 만 실시간. CAP·Δ 는 Calibrate 시점에 고정.
    const measuredLive = jointPositions[joint]
    const target = targetRads[joint]
    const stateValue =
      typeof measuredLive === 'number' ? formatRad4(measuredLive) : '--'

    const captureRad =
      status === 'captured'
        ? (captureResults[joint]?.measuredRad ??
          capturedPositions[joint])
        : null

    let diff: number | null = null
    if (status === 'captured') {
      const frozenDelta = captureResults[joint]?.deltaRad
      if (typeof frozenDelta === 'number') {
        diff = frozenDelta
      } else if (typeof captureRad === 'number' && typeof target === 'number') {
        diff = target - captureRad
      }
    } else if (
      status !== 'skipped' &&
      typeof measuredLive === 'number' &&
      typeof target === 'number'
    ) {
      diff = target - measuredLive
    }
    const diffValue =
      diff !== null ? `${diff >= 0 ? '+' : ''}${formatRad4(diff)}` : '--'
    const captureValue =
      status === 'skipped'
        ? 'SKIP'
        : typeof captureRad === 'number'
          ? formatRad4(captureRad)
          : '--'
    // delta 부호별 색: + 면 초록, - 면 빨강, 0/없음 은 회색
    const diffColor =
      diff === null ? '#9ca3af'
        : diff > 0 ? '#16a34a'
        : diff < 0 ? '#dc2626'
        : '#6b7280'

    const nameColor =
      isCurrent ? '#0f172a'
        : status === 'captured' ? '#374151'
        : status === 'skipped' ? '#9ca3af'
        : '#374151'

    const valueColor =
      isCurrent ? '#0f172a'
        : status === 'skipped' ? '#9ca3af'
        : '#1f2937'

    const torqueOn = torqueStates[joint] ?? false
    const torqueColor = torqueOn ? '#22c55e' : '#ef4444'

    return (
      <div
        key={joint}
        style={{
          ...statePanelRow,
          width: '100%',
          borderRadius: '8px',
          backgroundColor: isCurrent ? 'rgba(59, 130, 246, 0.12)' : 'transparent',
          borderLeftColor: isCurrent ? '#3b82f6' : 'transparent',
          opacity: status === 'skipped' ? 0.45 : 1,
          transition: 'background-color 0.15s, border-color 0.15s, opacity 0.15s',
        }}
      >
        <div style={statePanelCell}>
          <span
            title={`토크 ${torqueOn ? 'ON' : 'OFF'}`}
            style={{
              width: '10px',
              height: '10px',
              borderRadius: '50%',
              backgroundColor: torqueColor,
              boxShadow: `0 0 0 2px ${torqueOn ? 'rgba(34,197,94,0.18)' : 'rgba(239,68,68,0.18)'}`,
              flexShrink: 0,
            }}
          />
        </div>
        <div
          style={{
            ...statePanelJointCell,
            color: nameColor,
            fontWeight: isCurrent ? 600 : 400,
          }}
        >
          {shortName}
        </div>
        <div style={{ ...statePanelMonoCell, color: valueColor }} title={stateValue}>
          <span style={statePanelValueText}>{stateValue}</span>
        </div>
        <div
          title={
            status === 'captured'
              ? `Calibrate 시 캡처: ${captureValue} rad`
              : undefined
          }
          style={{
            ...statePanelMonoCell,
            color:
              status === 'captured'
                ? '#2563eb'
                : status === 'skipped'
                  ? '#9ca3af'
                  : '#d1d5db',
            fontWeight: status === 'captured' ? 600 : 400,
          }}
        >
          <span style={statePanelValueText}>{captureValue}</span>
        </div>
        <div
          title={
            status === 'captured'
              ? `Calibrate 시 Δ: ${diffValue} rad`
              : diff !== null
                ? `목표까지: ${diffValue} rad`
                : undefined
          }
          style={{
            ...statePanelMonoCell,
            color: diffColor,
            fontWeight: status === 'captured' ? 600 : 500,
          }}
        >
          <span style={statePanelValueText}>{diffValue}</span>
        </div>
      </div>
    )
  }

  /** Result 탭 오른쪽 패널: 토크 · 관절명 · 실시간 STATE 만 */
  const RESULT_PANEL_GRID = `26px ${STATE_PANEL_JOINT_COL_W}px ${STATE_PANEL_NUM_COL_W}px`
  const RESULT_STATE_ASIDE_W =
    26 + STATE_PANEL_JOINT_COL_W + STATE_PANEL_NUM_COL_W + 4 + 10 + 16 + 6
  const resultPanelRow: CSSProperties = {
    display: 'grid',
    gridTemplateColumns: RESULT_PANEL_GRID,
    columnGap: '2px',
    alignItems: 'center',
    boxSizing: 'border-box',
    width: '100%',
    maxWidth: '100%',
    padding: '6px 6px 6px 4px',
    borderLeft: '3px solid transparent',
    overflow: 'visible',
  }
  const resultPanelCell: CSSProperties = {
    display: 'flex',
    alignItems: 'center',
    justifyContent: 'center',
    minWidth: 0,
    maxWidth: '100%',
    overflow: 'hidden',
    textAlign: 'center',
  }

  const resultPanelHeaderLabel = (title: string, unit?: string) => (
    <span
      style={{
        display: 'flex',
        flexDirection: 'column',
        alignItems: 'center',
        lineHeight: 1.2,
        gap: '2px',
      }}
    >
      <span>{title}</span>
      {unit ? (
        <span style={{ fontSize: '9px', fontWeight: 600, letterSpacing: '0.08em', opacity: 0.85 }}>
          {unit}
        </span>
      ) : null}
    </span>
  )

  const resultColumnHeader = () => (
    <div style={{ ...resultPanelRow, marginBottom: '4px', flexShrink: 0 }}>
      <div style={resultPanelCell}>T</div>
      <div
        style={{
          ...statePanelHeaderCell,
          ...statePanelJointCell,
          letterSpacing: '0.04em',
        }}
      >
        JOINT
      </div>
      <div
        style={{
          ...resultPanelCell,
          fontSize: '10px',
          fontWeight: 700,
          letterSpacing: '0.06em',
          color: '#6b7280',
        }}
        title="실시간 joint_states (rad)"
      >
        {resultPanelHeaderLabel('STATE', '(rad)')}
      </div>
    </div>
  )

  const renderResultJointState = (joint: string, i: number) => {
    const isCurrent = calibrationStarted && i === currentIndex
    const status = statuses[i]
    const shortName = jointDisplayNumber(joint)
    const live = jointPositions[joint]
    const stateStr = typeof live === 'number' ? formatRad4(live) : '--'
    const torqueOn = torqueStates[joint] ?? false
    const torqueColor = torqueOn ? '#22c55e' : '#ef4444'
    const nameColor =
      isCurrent ? '#0f172a'
        : status === 'skipped' ? '#9ca3af'
        : '#374151'

    return (
      <div
        key={joint}
        style={{
          ...resultPanelRow,
          width: '100%',
          borderRadius: '8px',
          backgroundColor: isCurrent ? 'rgba(59, 130, 246, 0.08)' : 'transparent',
          borderLeftColor: isCurrent ? '#3b82f6' : 'transparent',
          opacity: status === 'skipped' ? 0.45 : 1,
        }}
      >
        <div style={resultPanelCell}>
          <span
            title={`토크 ${torqueOn ? 'ON' : 'OFF'}`}
            style={{
              width: '10px',
              height: '10px',
              borderRadius: '50%',
              backgroundColor: torqueColor,
              boxShadow: `0 0 0 2px ${torqueOn ? 'rgba(34,197,94,0.18)' : 'rgba(239,68,68,0.18)'}`,
            }}
          />
        </div>
        <div
          style={{
            ...statePanelJointCell,
            color: nameColor,
            fontWeight: isCurrent ? 600 : 400,
          }}
        >
          {shortName}
        </div>
        <div
          style={{
            ...resultPanelCell,
            fontFamily: 'ui-monospace, SFMono-Regular, Menlo, monospace',
            fontSize: '12px',
            fontVariantNumeric: 'tabular-nums',
            color: '#0f172a',
          }}
          title={stateStr}
        >
          <span style={statePanelValueText}>{stateStr}</span>
        </div>
      </div>
    )
  }

  const jointNamesForSafetyArm = (arm: 'right' | 'left') => {
    const p = arm === 'right' ? 'arm_r_' : 'arm_l_'
    return [1, 2, 3, 4, 5, 6, 7].map(n => `${p}joint${n}`)
  }

  const renderSafetyPrepPage = () => {
    if (!pendingSafety) return null
    const { arm: psArm, stage } = pendingSafety
    const names = jointNamesForSafetyArm(psArm)
    const armLabel = psArm === 'right' ? 'RIGHT · 오른팔' : 'LEFT · 왼팔'

    const rows = names.map(joint => {
      const v = jointPositions[joint]
      const ok = typeof v === 'number' && Math.abs(v) <= SAFETY_ZERO_RAD_TOL
      return { joint, v, ok }
    })
    const allOk = rows.every(r => r.ok)

    const safetyPrepNumForPending = (): 1 | 3 | 5 => {
      const stage = pendingSafetyRef.current?.stage
      if (stage === 'before_joint3') return 3
      if (stage === 'before_joint5') return 5
      return 1
    }

    const handleSafetyEffortZero = async () => {
      if (!pendingSafety || safetyTorqueReleaseBusy || safetyHomingBusy) return
      const armForStatus = pendingSafety.arm
      setSafetyTorqueReleaseBusy(true)
      setSafetyApplyProgress(0)
      const waiterRelease = waitForCalibrationTask('apply_effort', (prog, msg) => {
        if ((msg.arm ?? '') !== armForStatus) return
        setSafetyApplyProgress(
          Math.min(ZERO_POSE_BAR_STEPS, Math.round(prog * ZERO_POSE_BAR_STEPS)),
        )
      })
      try {
        const resp = await callRosService(
          '/calibration/apply_effort',
          'ffw_calibration/srv/ApplyEffort',
          {
            arm: armForStatus,
            target: [...SAFETY_ZERO_EFFORT_TARGET],
            duration_sec: SAFETY_TORQUE_RELEASE_SEC,
            effort_joint_2467_preset: false,
            effort_hold_2467_ramp_joint1: false,
            effort_hold_all_ramp_joint3: false,
            safety_prep_joint_number: 0,
          },
          SAFETY_TORQUE_RELEASE_SEC + 60,
        )
        if (!resp?.success) {
          throw new Error(resp?.message ?? '준비 실패')
        }
        await waiterRelease.promise

        if (pendingSafetyRef.current?.arm === armForStatus) {
          setSafetyEffortZeroed(true)
          setSafetyApplyProgress(0)
        }
      } catch (error) {
        console.error(error)
        window.alert(
          error instanceof Error ? error.message : '준비 실패',
        )
        setSafetyApplyProgress(0)
      } finally {
        waiterRelease.cancel()
        setSafetyTorqueReleaseBusy(false)
      }
    }

    const handleSafetyHomingApply = async () => {
      if (
        !pendingSafety ||
        !safetyEffortZeroed ||
        safetyTorqueReleased ||
        safetyHomingBusy ||
        safetyTorqueReleaseBusy
      ) {
        return
      }
      const armForStatus = pendingSafety.arm
      setSafetyHomingBusy(true)
      try {
        await sleepMs(DXL_HOMING_STEP_PAUSE_MS)
        if (
          pendingSafetyRef.current?.arm === armForStatus &&
          pendingSafetyRef.current?.stage != null
        ) {
          await applySafetyMotorHoming(armForStatus, safetyPrepNumForPending())
        }

        if (pendingSafetyRef.current?.arm === armForStatus) {
          setSafetyTorqueReleased(true)
        }
      } catch (error) {
        console.error(error)
        window.alert(
          error instanceof Error ? error.message : 'Homing 적용 실패',
        )
      } finally {
        setSafetyHomingBusy(false)
      }
    }

    const handleSafetyMove = async () => {
      if (!pendingSafety || !allOk || !safetyTorqueReleased) return
      const armForStatus = pendingSafety.arm
      const stageForAsync = pendingSafety.stage
      setSafetyArmMoveBusy(true)
      setSafetyApplyProgress(0)
      const barSteps = ZERO_POSE_BAR_STEPS
      const half = Math.floor(barSteps / 2)
      const baseTraj = getSafetyPrepBaseTraj8(
        safetyPrepPosesRef.current,
        stageForAsync,
        armForStatus,
      )
      const trajTargets = mergeCapturedIntoTrajectory8(armForStatus, baseTraj)
      const trajDurationSec = getSafetyPrepDurationSec(
        safetyPrepPosesRef.current,
        stageForAsync,
      )
      const trajServiceTimeoutSec = Math.max(15, trajDurationSec + 25)
      const waiterTraj = waitForCalibrationTask('move_trajectory', prog => {
        setSafetyApplyProgress(Math.min(half, Math.round(prog * half)))
      })
      const waiterEffort = waitForCalibrationTask('apply_effort', (prog, msg) => {
        if ((msg.arm ?? '') !== armForStatus) return
        setSafetyApplyProgress(
          Math.min(barSteps, half + Math.round(prog * Math.max(1, barSteps - half))),
        )
      })
      try {
        const trajArgs =
          armForStatus === 'right'
            ? {
                arm: 'right',
                duration_sec: trajDurationSec,
                use_mid_zero_target: false,
                right_targets: trajTargets,
                left_targets: [] as number[],
              }
            : {
                arm: 'left',
                duration_sec: trajDurationSec,
                use_mid_zero_target: false,
                right_targets: [] as number[],
                left_targets: trajTargets,
              }
        const trajResp = await callRosService(
          '/calibration/move_arm_trajectory',
          'ffw_calibration/srv/MoveArmTrajectory',
          trajArgs,
          trajServiceTimeoutSec,
        )
        if (!trajResp?.success) {
          waiterTraj.cancel()
          waiterEffort.cancel()
          throw new Error(trajResp?.message ?? '궤적 시작 실패')
        }
        await waiterTraj.promise

        const safetyPrepApply = (prepNum: 1 | 3 | 5) => ({
          arm: armForStatus,
          target: [] as number[],
          duration_sec: JOINT7_SAFETY_EFFORT_RAMP_SEC,
          effort_joint_2467_preset: false,
          effort_hold_2467_ramp_joint1: false,
          effort_hold_all_ramp_joint3: false,
          safety_prep_joint_number: prepNum,
        })
        const applyPayload =
          stageForAsync === 'before_joint1'
            ? safetyPrepApply(1)
            : stageForAsync === 'before_joint3'
              ? safetyPrepApply(3)
              : safetyPrepApply(5)
        const resp = await callRosService(
          '/calibration/apply_effort',
          'ffw_calibration/srv/ApplyEffort',
          applyPayload,
          JOINT7_SAFETY_EFFORT_RAMP_SEC + 60,
        )
        if (!resp?.success) {
          throw new Error(resp?.message ?? '이동 명령 실패')
        }
        if (
          pendingSafetyRef.current?.arm === armForStatus &&
          pendingSafetyRef.current?.stage === stageForAsync
        ) {
          setSafetyApplyProgress(ZERO_POSE_BAR_STEPS)
          setPendingSafety(null)
        }
      } catch (error) {
        console.error(error)
        window.alert(error instanceof Error ? error.message : '이동 실패')
        setSafetyApplyProgress(0)
      } finally {
        waiterTraj.cancel()
        waiterEffort.cancel()
        setSafetyArmMoveBusy(false)
      }
    }

    const safetyTitle = SAFETY_STAGE_TITLE[stage]
    const showSafetyProgress =
      safetyArmMoveBusy || safetyTorqueReleaseBusy || safetyHomingBusy

    const renderSafetyProgress = () => (
      <div
        role="status"
        aria-live="polite"
        style={{
          width: '600px',
          minWidth: '600px',
          maxWidth: '600px',
          padding: '14px 18px',
          backgroundColor: '#eff6ff',
          border: '1px solid #bfdbfe',
          borderRadius: '10px',
          boxSizing: 'border-box',
        }}
      >
        <div
          style={{
            display: 'flex',
            justifyContent: 'space-between',
            alignItems: 'center',
            marginBottom: '8px',
            color: '#1e40af',
            fontSize: '12px',
            fontWeight: 700,
            letterSpacing: '0.08em',
          }}
        >
          <span>진행률</span>
          <span>
            {safetyApplyProgress}/{ZERO_POSE_BAR_STEPS}
          </span>
        </div>
        <div
          style={{
            display: 'grid',
            gridTemplateColumns: `repeat(${ZERO_POSE_BAR_STEPS}, 1fr)`,
            gap: '6px',
          }}
        >
          {Array.from({ length: ZERO_POSE_BAR_STEPS }).map((_, i) => (
            <div
              key={i}
              style={{
                height: '12px',
                borderRadius: '4px',
                backgroundColor: i < safetyApplyProgress ? '#2563eb' : '#dbeafe',
                transition: 'background-color 0.15s',
              }}
            />
          ))}
        </div>
      </div>
    )

    const renderSafetyPrepButton = (
      sublabel: string,
      opts: {
        onClick: () => void
        disabled: boolean
        busy?: boolean
        backgroundColor: string
        boxShadow?: string
      },
    ) => (
      <button
        type="button"
        disabled={opts.disabled}
        onClick={opts.onClick}
        style={{
          padding: '10px 22px',
          fontWeight: 700,
          backgroundColor: opts.disabled ? '#d1d5db' : opts.backgroundColor,
          color: '#fff',
          border: 'none',
          borderRadius: '8px',
          cursor: opts.disabled ? 'not-allowed' : 'pointer',
          boxShadow: opts.disabled ? 'none' : (opts.boxShadow ?? 'none'),
          display: 'flex',
          flexDirection: 'column',
          alignItems: 'center',
          lineHeight: 1.25,
          minWidth: '108px',
        }}
      >
        <span style={{ fontSize: '17px' }}>준비</span>
        <span
          style={{
            fontSize: '12px',
            fontWeight: 600,
            opacity: 0.92,
            marginTop: '3px',
          }}
        >
          {opts.busy ? `${sublabel}…` : sublabel}
        </span>
      </button>
    )

    return (
      <div style={calibCenterShell}>
        <div style={calibCardShell}>
          <div
            style={{
              display: 'flex',
              alignItems: 'stretch',
              gap: '24px',
              marginBottom: '12px',
              minHeight: '72px',
            }}
          >
            <div style={{ flexShrink: 0 }}>
              <div
                style={{
                  fontSize: '12px',
                  fontWeight: 700,
                  letterSpacing: '0.18em',
                  color: '#b91c1c',
                  marginBottom: '10px',
                }}
              >
                SAFETY · {safetyTitle}
              </div>
              <h2
                style={{
                  margin: 0,
                  fontSize: '26px',
                  fontWeight: 800,
                  color: '#0f172a',
                }}
              >
                {armLabel}
              </h2>
            </div>
            <div
              style={{
                flex: 1,
                minWidth: '280px',
                display: 'flex',
                alignItems: 'center',
                justifyContent: 'center',
              }}
            >
              {showSafetyProgress ? renderSafetyProgress() : null}
            </div>
          </div>
          <div style={{ display: 'flex', flexWrap: 'wrap', gap: '12px', marginBottom: '20px' }}>
            <button
              type="button"
              disabled={
                !canGoBack ||
                safetyArmMoveBusy ||
                safetyTorqueReleaseBusy ||
                safetyHomingBusy
              }
              onClick={goToPreviousCalibrationStep}
              style={{
                padding: '14px 28px',
                fontSize: '17px',
                fontWeight: 700,
                backgroundColor:
                  !canGoBack ||
                  safetyArmMoveBusy ||
                  safetyTorqueReleaseBusy ||
                  safetyHomingBusy
                    ? '#d1d5db'
                    : '#64748b',
                color: '#fff',
                border: 'none',
                borderRadius: '8px',
                cursor:
                  !canGoBack ||
                  safetyArmMoveBusy ||
                  safetyTorqueReleaseBusy ||
                  safetyHomingBusy
                    ? 'not-allowed'
                    : 'pointer',
              }}
            >
              이전
            </button>
            <button
              type="button"
              disabled={
                safetyArmMoveBusy ||
                safetyTorqueReleaseBusy ||
                safetyHomingBusy
              }
              onClick={skipSafetyPrep}
              style={{
                padding: '14px 28px',
                fontSize: '17px',
                fontWeight: 700,
                backgroundColor:
                  safetyArmMoveBusy ||
                  safetyTorqueReleaseBusy ||
                  safetyHomingBusy
                    ? '#d1d5db'
                    : '#525252',
                color: '#fff',
                border: 'none',
                borderRadius: '8px',
                cursor:
                  safetyArmMoveBusy ||
                  safetyTorqueReleaseBusy ||
                  safetyHomingBusy
                    ? 'not-allowed'
                    : 'pointer',
              }}
            >
              Skip
            </button>
            {!safetyTorqueReleased ? (
              <>
                {!safetyEffortZeroed
                  ? renderSafetyPrepButton('effort 0', {
                      onClick: () => void handleSafetyEffortZero(),
                      disabled:
                        safetyTorqueReleaseBusy ||
                        safetyArmMoveBusy ||
                        safetyHomingBusy,
                      busy: safetyTorqueReleaseBusy,
                      backgroundColor: '#0f766e',
                    })
                  : renderSafetyPrepButton('Homing 적용', {
                      onClick: () => void handleSafetyHomingApply(),
                      disabled:
                        safetyHomingBusy ||
                        safetyArmMoveBusy ||
                        safetyTorqueReleaseBusy,
                      busy: safetyHomingBusy,
                      backgroundColor: '#047857',
                    })}
              </>
            ) : null}
            {renderSafetyPrepButton('이동', {
              onClick: () => void handleSafetyMove(),
              disabled:
                !safetyTorqueReleased ||
                !allOk ||
                safetyArmMoveBusy ||
                safetyTorqueReleaseBusy ||
                safetyHomingBusy,
              busy: safetyArmMoveBusy,
              backgroundColor: '#2563eb',
              boxShadow: '0 4px 12px rgba(37, 99, 235, 0.35)',
            })}
          </div>

          <p
            style={{
              margin: '0 0 20px',
              fontSize: '14px',
              lineHeight: 1.55,
              color: '#b45309',
            }}
          >
            {!safetyEffortZeroed
              ? '「Skip」을 누르면 모터가 풀리며 중력으로 로봇팔이 떨어질 수 있습니다. 충돌에 주의하세요'
              : !safetyTorqueReleased
                ? '「Homing 적용」은 현재 관절의 Calibration을 위해, 이미 완료한 이전 관절들의 보정을 반영합니다.'
                : '「이동」을 누르면 로봇이 움직입니다. 충돌에 주의하며, 이상 거동 시 즉시 E-STOP을 누르세요.'}
          </p>

          <div
            style={{
              flex: 1,
              minHeight: 0,
              display: 'flex',
              flexDirection: 'column',
              marginBottom: '16px',
            }}
          >
            <div
              style={{
                border: '1px solid #e5e7eb',
                borderRadius: '10px',
                overflow: 'auto',
                flex: 1,
                minHeight: 0,
              }}
            >
              <div
                style={{
                  display: 'grid',
                  gridTemplateColumns: '1fr 1fr 100px',
                  padding: '10px 14px',
                  fontSize: '11px',
                  fontWeight: 700,
                  letterSpacing: '0.12em',
                  color: '#64748b',
                  backgroundColor: '#f8fafc',
                  borderBottom: '1px solid #e5e7eb',
                }}
              >
                <span>JOINT</span>
                <span style={{ textAlign: 'right' }}>POSITION (rad)</span>
                <span style={{ textAlign: 'center' }}>OK</span>
              </div>
              {rows.map((r, i) => (
                <div
                  key={r.joint}
                  style={{
                    display: 'grid',
                    gridTemplateColumns: '1fr 1fr 100px',
                    padding: '12px 14px',
                    alignItems: 'center',
                    fontSize: '15px',
                    borderTop: i === 0 ? 'none' : '1px solid #f1f5f9',
                    backgroundColor: r.ok ? '#f0fdf4' : '#fff',
                  }}
                >
                  <span
                    style={{
                      fontFamily: 'ui-monospace, SFMono-Regular, Menlo, monospace',
                      fontWeight: 600,
                      color: '#0f172a',
                    }}
                  >
                    {r.joint.replace(/^arm_[rl]_/, '')}
                  </span>
                  <span
                    style={{
                      textAlign: 'right',
                      fontFamily: 'ui-monospace, SFMono-Regular, Menlo, monospace',
                      color: typeof r.v === 'number' ? '#0f172a' : '#94a3b8',
                    }}
                  >
                    {typeof r.v === 'number' ? r.v.toFixed(4) : '—'}
                  </span>
                  <span
                    style={{
                      textAlign: 'center',
                      fontWeight: 800,
                      color: r.ok ? '#15803d' : '#dc2626',
                    }}
                  >
                    {r.ok ? '●' : '✕'}
                  </span>
                </div>
              ))}
            </div>
          </div>
        </div>
      </div>
    )
  }

  const resultTableHeaderCell = (title: string, unit?: string) => (
    <span
      style={{
        display: 'flex',
        flexDirection: 'column',
        alignItems: 'center',
        justifyContent: 'center',
        lineHeight: 1.2,
        gap: '2px',
      }}
    >
      <span>{title}</span>
      {unit ? (
        <span
          style={{
            fontSize: '10px',
            fontWeight: 600,
            letterSpacing: '0.1em',
            opacity: 0.85,
          }}
        >
          {unit}
        </span>
      ) : null}
    </span>
  )

  const renderDeltaTable = (label: string, arm: 'r' | 'l') => {
    const angleUnitLabel = resultAngleUnit === 'deg' ? '(deg)' : '(rad)'
    return (
    <div
      style={{
        flex: 1,
        minWidth: '360px',
        minHeight: 0,
        display: 'flex',
        flexDirection: 'column',
      }}
    >
      <div
        style={{
          fontSize: '12px',
          fontWeight: 700,
          letterSpacing: '0.18em',
          color: '#6b7280',
          marginBottom: '12px',
        }}
      >
        {label}
      </div>
      <div
        style={{
          flex: 1,
          border: '1px solid #e5e7eb',
          borderRadius: '10px',
          overflow: 'hidden',
          backgroundColor: '#fff',
          display: 'flex',
          flexDirection: 'column',
        }}
      >
        <div
          style={{
            display: 'grid',
            gridTemplateColumns: '1.1fr 1fr 1fr 1fr 1.1fr',
            padding: '12px 18px 10px',
            fontSize: '11px',
            fontWeight: 700,
            letterSpacing: '0.14em',
            color: '#6b7280',
            backgroundColor: '#f9fafb',
            borderBottom: '1px solid #e5e7eb',
            textAlign: 'center',
            alignItems: 'center',
          }}
        >
          {resultTableHeaderCell('JOINT')}
          {resultTableHeaderCell('CAPTURE', angleUnitLabel)}
          {resultTableHeaderCell('TARGET', angleUnitLabel)}
          {resultTableHeaderCell('Δ', angleUnitLabel)}
          {resultTableHeaderCell('STATUS')}
        </div>
        <div style={{ flex: 1, display: 'flex', flexDirection: 'column' }}>
          {resultArmJoints(arm).map(({ joint, idx }, rowIdx) => {
            const status = statuses[idx]
            const shortName = jointDisplayNumber(joint)
            const result = captureResults[joint]
            const capture = result
              ? formatResultAngle(result.measuredRad, resultAngleUnit)
              : '--'
            const target = result
              ? formatResultAngle(result.targetRad, resultAngleUnit)
              : '--'
            const delta = result
              ? formatResultDelta(result.deltaRad, resultAngleUnit)
              : '--'
            const deltaColor = !result
              ? '#9ca3af'
              : result.deltaRad > 0
                ? '#16a34a'
                : result.deltaRad < 0
                  ? '#dc2626'
                  : '#0f172a'
            const statusColor =
              status === 'captured' ? '#16a34a'
              : status === 'skipped' ? '#9ca3af'
              : '#d4d4d4'
            const statusLabel =
              status === 'captured' ? 'CALIBRATED'
              : status === 'skipped' ? 'SKIPPED'
              : 'PENDING'
            return (
              <div
                key={joint}
                style={{
                  flex: 1,
                  display: 'grid',
                  gridTemplateColumns: '1.1fr 1fr 1fr 1fr 1.1fr',
                  padding: '14px 20px',
                  fontSize: '14px',
                  color: '#0f172a',
                  borderTop: rowIdx === 0 ? 'none' : '1px solid #f1f5f9',
                  alignItems: 'center',
                  textAlign: 'center',
                }}
              >
                <span
                  style={{
                    fontFamily:
                      'ui-monospace, SFMono-Regular, Menlo, monospace',
                    fontWeight: 600,
                  }}
                >
                  {shortName}
                </span>
                <span
                  style={{
                    fontFamily:
                      'ui-monospace, SFMono-Regular, Menlo, monospace',
                    color: '#2563eb',
                    fontVariantNumeric: 'tabular-nums',
                    fontWeight: status === 'captured' ? 600 : 400,
                  }}
                >
                  {capture}
                </span>
                <span
                  style={{
                    fontFamily:
                      'ui-monospace, SFMono-Regular, Menlo, monospace',
                    fontVariantNumeric: 'tabular-nums',
                  }}
                >
                  {target}
                </span>
                <span
                  style={{
                    fontFamily:
                      'ui-monospace, SFMono-Regular, Menlo, monospace',
                    fontWeight: 600,
                    color: deltaColor,
                  }}
                >
                  {delta}
                </span>
                <span
                  style={{
                    display: 'flex',
                    justifyContent: 'center',
                    alignItems: 'center',
                    gap: '6px',
                    fontSize: '11px',
                    fontWeight: 700,
                    letterSpacing: '0.1em',
                    color: statusColor,
                  }}
                >
                  <span
                    style={{
                      width: '8px',
                      height: '8px',
                      borderRadius: '50%',
                      backgroundColor: statusColor,
                      display: 'inline-block',
                    }}
                  />
                  {statusLabel}
                </span>
              </div>
            )
          })}
        </div>
      </div>
    </div>
    )
  }

  const renderInformationAside = () => (
    <aside
      style={{
        width: `${INFORMATION_ASIDE_W}px`,
        flexShrink: 0,
        minHeight: 0,
        alignSelf: 'stretch',
        display: 'flex',
        flexDirection: 'column',
        backgroundColor: '#fff',
        border: '1px solid #e5e5e5',
        borderRadius: '12px',
        padding: '22px 24px',
        boxSizing: 'border-box',
        boxShadow: '0 1px 4px rgba(0, 0, 0, 0.04)',
      }}
    >
      <div
        style={{
          fontSize: '12px',
          fontWeight: 700,
          letterSpacing: '0.14em',
          color: '#888',
          marginBottom: '14px',
        }}
      >
        진행 순서
      </div>
      <ol
        style={{
          margin: '0 0 28px',
          paddingLeft: '20px',
          color: '#333',
          fontSize: '14px',
          lineHeight: 1.85,
        }}
      >
        <li style={{ marginBottom: '10px' }}>
          <strong>Calibration</strong>
          <div style={{ fontSize: '13px', color: '#666', marginTop: '4px' }}>
            Zero Pose → Start
          </div>
        </li>
        <li style={{ marginBottom: '10px' }}>
          <strong>관절별 Calibrate</strong>
          <div style={{ fontSize: '13px', color: '#666', marginTop: '4px' }}>
            화면 안내 따라 진행
          </div>
        </li>
        <li style={{ marginBottom: '10px' }}>
          <strong>준비</strong> (해당 관절만)
          <div style={{ fontSize: '13px', color: '#666', marginTop: '4px' }}>
            Homing 적용 → 이동
          </div>
        </li>
        <li>
          <strong>Result</strong> · <strong>Test</strong>
          <div style={{ fontSize: '13px', color: '#666', marginTop: '4px' }}>
            결과 확인 · 포즈 테스트(선택)
          </div>
        </li>
      </ol>

      <div
        style={{
          fontSize: '12px',
          fontWeight: 700,
          letterSpacing: '0.14em',
          color: '#888',
          marginBottom: '12px',
        }}
      >
        시작 전 확인
      </div>
      <ul
        style={{
          margin: 0,
          paddingLeft: '20px',
          color: '#333',
          fontSize: '14px',
          lineHeight: 1.75,
        }}
      >
        <li style={{ marginBottom: '6px' }}>작업 공간 비우기</li>
        <li style={{ marginBottom: '6px' }}>E-STOP 위치 확인</li>
        <li>2인 1조 (한 명은 E-STOP 대기)</li>
      </ul>
    </aside>
  )

  const renderTestTab = () => {
    const testBusy = activeTestTraj !== null
    const testCustomSelected = testTabSelection === 'custom'

    const testTopButton = (
      label: string,
      opts?: { onClick?: () => void; running?: boolean },
    ) => {
      const wired = !!opts?.onClick
      const disabled = !wired || (testBusy && !opts?.running)
      return (
        <button
          type="button"
          disabled={disabled}
          title={wired ? undefined : '연결 예정'}
          onClick={opts?.onClick}
          style={{
            flex: 1,
            minWidth: '120px',
            padding: '12px 20px',
            fontSize: '15px',
            fontWeight: 700,
            backgroundColor: disabled && !opts?.running ? '#e2e8f0' : '#0f172a',
            color: disabled && !opts?.running ? '#64748b' : '#fff',
            border: 'none',
            borderRadius: '8px',
            cursor: disabled ? 'not-allowed' : 'pointer',
            opacity: disabled && !opts?.running ? 0.65 : 1,
          }}
        >
          {opts?.running ? `${label}…` : label}
        </button>
      )
    }

    const renderCompactTraj = (traj: TestArmTraj) => {
      const arms: { label: string; targets: number[] }[] = [
        { label: 'R', targets: traj.right },
        { label: 'L', targets: traj.left },
      ]
      return (
        <div
          style={{
            display: 'flex',
            gap: '6px',
            padding: '5px 6px',
            backgroundColor: '#f8fafc',
            border: '1px solid #e2e8f0',
            borderRadius: '5px',
            fontSize: '9px',
            fontFamily: 'ui-monospace, SFMono-Regular, Menlo, monospace',
            lineHeight: 1.2,
            flex: 1,
            width: '100%',
            height: '100%',
            minHeight: 0,
            overflowY: 'auto',
            alignSelf: 'stretch',
          }}
        >
          {arms.map(({ label, targets }) => (
            <div key={label} style={{ flex: 1, minWidth: 0 }}>
              <div
                style={{
                  fontWeight: 700,
                  color: '#64748b',
                  marginBottom: '2px',
                  letterSpacing: '0.06em',
                  fontSize: '8px',
                }}
              >
                {label}
              </div>
              {TRAJ8_AXIS_LABELS.map((axis, i) => (
                <div
                  key={axis}
                  style={{
                    display: 'flex',
                    justifyContent: 'space-between',
                    gap: '4px',
                    color: '#0f172a',
                  }}
                >
                  <span style={{ color: '#94a3b8' }}>{axis}</span>
                  <span style={{ fontVariantNumeric: 'tabular-nums' }}>
                    {formatRad4(targets[i] ?? 0)}
                  </span>
                </div>
              ))}
            </div>
          ))}
        </div>
      )
    }

    const renderTestCustomPanel = () => {
      const armChar = testCustomArm === 'right' ? 'r' : 'l'

      return (
        <div
          style={{
            flex: 1,
            minHeight: 0,
            width: '100%',
            display: 'flex',
            gap: '12px',
            alignItems: 'stretch',
          }}
        >
          <div
            style={{
              flex: 1,
              minWidth: 0,
              minHeight: 0,
              border: '1px dashed #d1d5db',
              borderRadius: '12px',
              backgroundColor: '#fafafa',
            }}
          />

          <div
            style={{
              flex: 1,
              minWidth: 0,
              minHeight: 0,
              display: 'flex',
              flexDirection: 'column',
              gap: '14px',
              border: '1px solid #e5e7eb',
              borderRadius: '12px',
              padding: '14px 16px',
              backgroundColor: '#fff',
            }}
          >
            <div style={{ flexShrink: 0 }}>
              <div
                style={{
                  fontSize: '11px',
                  fontWeight: 700,
                  letterSpacing: '0.14em',
                  color: '#6b7280',
                  marginBottom: '8px',
                }}
              >
                ARM
              </div>
              <div style={{ display: 'flex', gap: '8px' }}>
                {(['left', 'right'] as const).map(side => {
                  const selected = testCustomArm === side
                  return (
                    <button
                      key={side}
                      type="button"
                      onClick={() => setTestCustomArm(side)}
                      style={{
                        flex: 1,
                        padding: '10px 14px',
                        fontSize: '14px',
                        fontWeight: selected ? 700 : 500,
                        backgroundColor: selected ? '#2563eb' : '#fff',
                        color: selected ? '#fff' : '#374151',
                        border: `1px solid ${selected ? '#2563eb' : '#e5e7eb'}`,
                        borderRadius: '8px',
                        cursor: 'pointer',
                      }}
                    >
                      {side === 'right' ? 'RIGHT' : 'LEFT'}
                    </button>
                  )
                })}
              </div>
            </div>

            <div style={{ flexShrink: 0 }}>
              <div
                style={{
                  fontSize: '11px',
                  fontWeight: 700,
                  letterSpacing: '0.14em',
                  color: '#6b7280',
                  marginBottom: '8px',
                }}
              >
                STEP
              </div>
              <div
                style={{
                  display: 'flex',
                  flexWrap: 'wrap',
                  gap: '8px',
                }}
              >
                {TEST_CUSTOM_STEP_OPTIONS.map(opt => {
                  const selected = testCustomStepDeg === opt.deg
                  return (
                    <button
                      key={opt.deg}
                      type="button"
                      onClick={() => setTestCustomStepDeg(opt.deg)}
                      style={{
                        display: 'flex',
                        alignItems: 'center',
                        gap: '10px',
                        padding: '10px 14px',
                        fontSize: '14px',
                        fontWeight: selected ? 700 : 500,
                        backgroundColor: selected ? '#2563eb' : '#fff',
                        color: selected ? '#fff' : '#374151',
                        border: `1px solid ${selected ? '#2563eb' : '#e5e7eb'}`,
                        borderRadius: '8px',
                        cursor: 'pointer',
                      }}
                    >
                      <span>{opt.deg}°</span>
                      <span
                        style={{
                          fontSize: '12px',
                          fontWeight: 500,
                          color: selected ? 'rgba(255,255,255,0.85)' : '#64748b',
                        }}
                      >
                        time from start{' '}
                        {formatTestCustomTimeFromStart(opt.timeFromStartSec)}
                      </span>
                    </button>
                  )
                })}
              </div>
            </div>

            <div
              style={{
                flex: 1,
                minHeight: 0,
                display: 'flex',
                flexDirection: 'column',
                gap: '8px',
                justifyContent: 'space-between',
              }}
            >
              {TEST_CUSTOM_ARM_JOINT_NUMS.map(jointNum => {
                const joint = customArmJointRosName(armChar, jointNum)
                const pos = jointPositions[joint]
                const value =
                  typeof pos === 'number' ? formatRad4(pos) : '--'
                const nudgeDisabled = testBusy
                return (
                  <div
                    key={joint}
                    style={{
                      display: 'flex',
                      alignItems: 'center',
                      gap: '10px',
                      flex: 1,
                      minHeight: 0,
                    }}
                  >
                    <div
                      style={{
                        width: '28px',
                        flexShrink: 0,
                        fontSize: '12px',
                        fontWeight: 600,
                        color: '#64748b',
                      }}
                    >
                      J{jointNum}
                    </div>
                    <button
                      type="button"
                      disabled={nudgeDisabled}
                      onClick={() =>
                        void handleTestCustomArmNudge(testCustomArm, jointNum, -1)
                      }
                      style={{
                        width: '40px',
                        height: '40px',
                        flexShrink: 0,
                        fontSize: '20px',
                        fontWeight: 700,
                        lineHeight: 1,
                        backgroundColor: nudgeDisabled ? '#e2e8f0' : '#fff',
                        color: nudgeDisabled ? '#94a3b8' : '#0f172a',
                        border: '1px solid #e5e7eb',
                        borderRadius: '8px',
                        cursor: nudgeDisabled ? 'not-allowed' : 'pointer',
                      }}
                    >
                      −
                    </button>
                    <div
                      style={{
                        flex: 1,
                        textAlign: 'center',
                        fontFamily:
                          'ui-monospace, SFMono-Regular, Menlo, monospace',
                        fontSize: '14px',
                        fontWeight: 600,
                        color: '#0f172a',
                        fontVariantNumeric: 'tabular-nums',
                      }}
                    >
                      {value}
                    </div>
                    <button
                      type="button"
                      disabled={nudgeDisabled}
                      onClick={() =>
                        void handleTestCustomArmNudge(testCustomArm, jointNum, 1)
                      }
                      style={{
                        width: '40px',
                        height: '40px',
                        flexShrink: 0,
                        fontSize: '20px',
                        fontWeight: 700,
                        lineHeight: 1,
                        backgroundColor: nudgeDisabled ? '#e2e8f0' : '#fff',
                        color: nudgeDisabled ? '#94a3b8' : '#0f172a',
                        border: '1px solid #e5e7eb',
                        borderRadius: '8px',
                        cursor: nudgeDisabled ? 'not-allowed' : 'pointer',
                      }}
                    >
                      +
                    </button>
                  </div>
                )
              })}
            </div>
          </div>
        </div>
      )
    }

    const renderPoseCard = (poseIdx: TestPoseIndex) => {
      if (testTabSelection === 'custom') return null
      const jointNum = testTabSelection
      const busyKey = `j${jointNum}_p${poseIdx}`
      const isRunning = activeTestTraj === busyKey
      const poseMoveDisabled = testBusy && !isRunning
      const selected = testSelectedPoseIdx === poseIdx
      const traj = getTestJointPoseTraj(jointNum, poseIdx)
      const photoFile = `${testJointRosName(jointNum)}_${poseIdx}.png`
      const photoKey = photoFile
      const photoFailed = !!jointImgFailed[photoKey]

      return (
        <div
          key={poseIdx}
          style={{
            display: 'flex',
            flexDirection: 'column',
            gap: '5px',
            padding: '6px',
            width: '100%',
            height: '100%',
            minWidth: 0,
            minHeight: 0,
            boxSizing: 'border-box',
            borderRadius: '8px',
            border: selected ? '2px solid #2563eb' : '1px solid #e5e7eb',
            backgroundColor: '#fff',
            boxShadow: selected
              ? '0 0 0 3px rgba(37, 99, 235, 0.12)'
              : '0 1px 3px rgba(15, 23, 42, 0.06)',
          }}
        >
          <div
            style={{
              display: 'flex',
              gap: '8px',
              alignItems: 'stretch',
              flex: 1,
              minHeight: 0,
              overflow: 'hidden',
            }}
          >
            <div
              style={{
                width: '48%',
                flexShrink: 0,
                minWidth: 0,
                alignSelf: 'stretch',
                minHeight: 0,
                borderRadius: '6px',
                overflow: 'hidden',
                border: '1px solid #e5e7eb',
                backgroundColor: '#f8fafc',
                display: 'flex',
                alignItems: 'center',
                justifyContent: 'center',
              }}
            >
              {photoFailed ? (
                <div style={{ textAlign: 'center', color: '#64748b', padding: '4px' }}>
                  <div
                    style={{
                      fontSize: '9px',
                      fontWeight: 700,
                      letterSpacing: '0.12em',
                      marginBottom: '4px',
                    }}
                  >
                    PHOTO
                  </div>
                  <div style={{ fontSize: '10px', wordBreak: 'break-all' }}>
                    {photoFile}
                  </div>
                </div>
              ) : (
                <img
                  src={`/${photoFile}`}
                  alt={`${testJointRosName(jointNum)} ${TEST_JOINT_POSE_LABELS[poseIdx]}`}
                  onError={() =>
                    setJointImgFailed(prev => ({ ...prev, [photoKey]: true }))
                  }
                  style={{
                    width: '100%',
                    height: '100%',
                    objectFit: 'contain',
                  }}
                />
              )}
            </div>

            <div style={{ flex: 1, minWidth: 0, minHeight: 0, display: 'flex' }}>
              {renderCompactTraj(traj)}
            </div>
          </div>

          <button
            type="button"
            disabled={poseMoveDisabled}
            onClick={() => void handleTestJointPose(jointNum, poseIdx)}
            style={{
              flexShrink: 0,
              padding: '7px 8px',
              fontSize: '12px',
              fontWeight: 600,
              backgroundColor: poseMoveDisabled ? '#94a3b8' : '#0f172a',
              color: '#fff',
              border: 'none',
              borderRadius: '6px',
              cursor: poseMoveDisabled ? 'not-allowed' : 'pointer',
              boxShadow: poseMoveDisabled ? 'none' : '0 2px 6px rgba(15, 23, 42, 0.12)',
              opacity: poseMoveDisabled && !isRunning ? 0.65 : 1,
            }}
          >
            {isRunning
              ? `${TEST_JOINT_POSE_LABELS[poseIdx]}…`
              : TEST_JOINT_POSE_LABELS[poseIdx]}
          </button>
        </div>
      )
    }

    return (
      <div
        style={{
          flex: 1,
          minHeight: 0,
          alignSelf: 'stretch',
          width: '100%',
          display: 'flex',
          flexDirection: 'column',
          gap: '24px',
        }}
      >
        <div
          style={{
            display: 'flex',
            gap: '12px',
            flexWrap: 'wrap',
            flexShrink: 0,
          }}
        >
          {testTopButton('Zero Pose', {
            onClick: () => void handleTestZeroPose(),
            running: activeTestTraj === 'zero_pose',
          })}
          {testTopButton('ET Pose', {
            onClick: () => void handleTestEtPose(),
            running: activeTestTraj === 'et_pose',
          })}
          {testTopButton('Lift Up', {
            onClick: motionPosesPack?.liftUp
              ? () => void handleTestLiftPose('up')
              : undefined,
            running: activeTestTraj === 'lift_up',
          })}
          {testTopButton('Lift Down', {
            onClick: motionPosesPack?.liftDown
              ? () => void handleTestLiftPose('down')
              : undefined,
            running: activeTestTraj === 'lift_down',
          })}
        </div>

        <p
          style={{
            margin: 0,
            flexShrink: 0,
            fontSize: '14px',
            color: '#475569',
            lineHeight: 1.6,
          }}
        >
          로봇이 이동합니다. 충돌 위험이 있는 경우{' '}
          <strong style={{ color: '#b91c1c' }}>E-STOP</strong>을 누르세요.
        </p>

        <div
          style={{
            flex: 1,
            minHeight: 0,
            display: 'flex',
            gap: '20px',
            alignItems: 'stretch',
          }}
        >
          <div
            style={{
              display: 'flex',
              flexDirection: 'column',
              gap: '6px',
              minWidth: '100px',
              flexShrink: 0,
              alignSelf: 'stretch',
            }}
          >
            <div
              style={{
                flexShrink: 0,
                display: 'flex',
                flexDirection: 'column',
                gap: '6px',
                paddingBottom: '14px',
                marginBottom: '14px',
                borderBottom: '1px solid #e5e7eb',
              }}
            >
              <div
                style={{
                  fontSize: '11px',
                  fontWeight: 700,
                  letterSpacing: '0.14em',
                  color: '#6b7280',
                  marginBottom: '2px',
                }}
              >
                CUSTOM
              </div>
              <button
                type="button"
                onClick={() => {
                  setTestTabSelection('custom')
                  setTestSelectedPoseIdx(1)
                }}
                style={{
                  padding: '10px 14px',
                  fontSize: '14px',
                  fontWeight: testCustomSelected ? 700 : 500,
                  textAlign: 'left',
                  backgroundColor: testCustomSelected ? '#2563eb' : '#fff',
                  color: testCustomSelected ? '#fff' : '#374151',
                  border: `1px solid ${testCustomSelected ? '#2563eb' : '#e5e7eb'}`,
                  borderRadius: '8px',
                  cursor: 'pointer',
                }}
              >
                Custom
              </button>
            </div>

            <div
              style={{
                fontSize: '11px',
                fontWeight: 700,
                letterSpacing: '0.14em',
                color: '#6b7280',
                marginBottom: '6px',
                flexShrink: 0,
              }}
            >
              JOINT
            </div>
            <div
              style={{
                flex: 1,
                minHeight: 0,
                display: 'flex',
                flexDirection: 'column',
                gap: '6px',
                justifyContent: 'space-between',
              }}
            >
              {TEST_JOINT_NUMS.map(n => {
                const selected = !testCustomSelected && n === testTabSelection
                return (
                  <button
                    key={n}
                    type="button"
                    onClick={() => {
                      setTestTabSelection(n)
                      setTestSelectedPoseIdx(1)
                    }}
                    style={{
                      padding: '10px 14px',
                      fontSize: '14px',
                      fontWeight: selected ? 700 : 500,
                      textAlign: 'left',
                      backgroundColor: selected ? '#2563eb' : '#fff',
                      color: selected ? '#fff' : '#374151',
                      border: `1px solid ${selected ? '#2563eb' : '#e5e7eb'}`,
                      borderRadius: '8px',
                      cursor: 'pointer',
                      flex: 1,
                      minHeight: 0,
                    }}
                  >
                    joint{n}
                  </button>
                )
              })}
            </div>
          </div>

          <div
            style={{
              flex: 1,
              minWidth: '320px',
              minHeight: 0,
              width: '100%',
              display: 'flex',
              flexDirection: 'column',
            }}
          >
            {testCustomSelected ? (
              renderTestCustomPanel()
            ) : (
              <div
                style={{
                  flex: 1,
                  minHeight: 0,
                  display: 'grid',
                  gridTemplateColumns: 'repeat(2, minmax(0, 1fr))',
                  gridTemplateRows: 'repeat(2, minmax(0, 1fr))',
                  gap: '12px',
                  width: '100%',
                  alignItems: 'stretch',
                }}
              >
                {renderPoseCard(1)}
                {renderPoseCard(2)}
                {renderPoseCard(3)}
                {renderPoseCard(4)}
              </div>
            )}
          </div>
        </div>

      </div>
    )
  }

  const renderStartPage = () => {
    const isStartZeroPoseComplete = startZeroPoseProgress >= ZERO_POSE_BAR_STEPS

    const handleStartZeroPoseButton = async () => {
      setStartZeroPosePressed(true)
      setStartZeroPoseProgress(0)

      const waiter = waitForCalibrationTask('zero_pose', progress => {
        setStartZeroPoseProgress(
          Math.min(ZERO_POSE_BAR_STEPS, Math.round(progress * ZERO_POSE_BAR_STEPS)),
        )
      })
      try {
        const startResp = await callRosService(
          '/calibration/move_to_zero_pose',
          'ffw_calibration/srv/MoveToZeroPose',
          {
            arm: 'both',
            duration_sec:
              motionPosesRef.current?.zeroPoseDurationSec ??
              DEFAULT_ZERO_POSE_DURATION_SEC,
            enable: [],
          },
        )
        if (!startResp?.success) {
          waiter.cancel()
          throw new Error(startResp?.message ?? 'Zero pose failed to start')
        }
        await waiter.promise
        setStartZeroPoseProgress(ZERO_POSE_BAR_STEPS)
      } catch (error) {
        waiter.cancel()
        console.error(error)
        window.alert(error instanceof Error ? error.message : 'Zero Pose 실패')
        setStartZeroPosePressed(false)
        setStartZeroPoseProgress(0)
      }
    }

    const handleStartButton = async () => {
      setStartSequenceBusy(true)
      try {
        const zeroResp = await callRosService(
          '/calibration/zero_effort',
          'std_srvs/srv/Trigger',
          {},
        )
        if (!zeroResp?.success) {
          throw new Error(zeroResp?.message ?? 'Failed to zero effort')
        }
        setCalibrationStarted(true)
      } catch (error) {
        console.error(error)
        window.alert(error instanceof Error ? error.message : 'Start 실패')
      } finally {
        setStartSequenceBusy(false)
      }
    }

    return (
    <div style={calibCenterShell}>
      <div style={calibCardShell}>
        <div
          style={{
            fontSize: '12px',
            fontWeight: 700,
            letterSpacing: '0.18em',
            color: '#b91c1c',
            marginBottom: '12px',
          }}
        >
          SAFETY · 시작 전 준비
        </div>

        <h2
          style={{
            margin: '0 0 18px',
            fontSize: '30px',
            fontWeight: 800,
            color: '#0f172a',
            letterSpacing: '0.01em',
          }}
        >
          Calibration 시작 전
        </h2>

        <div
          style={{
            margin: '0 0 12px',
            padding: '14px 16px',
            borderRadius: '8px',
            backgroundColor: '#fef2f2',
            border: '1px solid #fecaca',
            borderLeft: '4px solid #dc2626',
            color: '#991b1b',
            fontSize: '15px',
            lineHeight: 1.55,
            fontWeight: 700,
            display: 'flex',
            alignItems: 'flex-start',
            gap: '10px',
          }}
        >
          <span
            aria-hidden
            style={{
              flexShrink: 0,
              width: '22px',
              height: '22px',
              borderRadius: '50%',
              backgroundColor: '#dc2626',
              color: '#fff',
              display: 'inline-flex',
              alignItems: 'center',
              justifyContent: 'center',
              fontSize: '14px',
              fontWeight: 800,
              marginTop: '1px',
            }}
          >
            !
          </span>
          <span>
            <span style={{ letterSpacing: '0.08em' }}>WARNING · </span>
            로봇이 움직일 수 있습니다. E-STOP 준비·작업 반경 확인 후 진행하세요.
          </span>
        </div>

        <div
          style={{
            margin: '0 0 12px',
            padding: '16px 18px',
            borderRadius: '8px',
            backgroundColor: '#f3f4f6',
            border: '1px solid #e5e7eb',
            color: '#1f2937',
            fontSize: '17px',
            lineHeight: 1.7,
            fontWeight: 600,
          }}
        >
          <p style={{ margin: '0 0 10px', lineHeight: 1.65 }}>
            캘리브레이션 전 <strong>Zero Pose</strong>를 실행하세요.
          </p>
          <p style={{ margin: 0, lineHeight: 1.65 }}>
            Zero Pose 완료 후 <strong>Start</strong>가 활성화됩니다.
          </p>
        </div>

        {startZeroPosePressed && (
          <div
            style={{
              marginTop: '20px',
              padding: '18px',
              backgroundColor: '#f8fafc',
              border: '1px solid #e5e7eb',
              borderRadius: '10px',
            }}
          >
            <div
              style={{
                display: 'flex',
                justifyContent: 'space-between',
                alignItems: 'center',
                marginBottom: '12px',
                color: '#475569',
                fontSize: '13px',
                fontWeight: 700,
                letterSpacing: '0.1em',
              }}
            >
              <span>
                {isStartZeroPoseComplete ? 'ZERO POSE COMPLETE' : 'MOVING TO ZERO POSE'}
              </span>
              <span>{startZeroPoseProgress}/{ZERO_POSE_BAR_STEPS}</span>
            </div>

            <div
              style={{
                display: 'grid',
                gridTemplateColumns: `repeat(${ZERO_POSE_BAR_STEPS}, 1fr)`,
                gap: '6px',
              }}
            >
              {Array.from({ length: ZERO_POSE_BAR_STEPS }).map((_, i) => (
                <div
                  key={i}
                  style={{
                    height: '18px',
                    borderRadius: '4px',
                    backgroundColor: i < startZeroPoseProgress
                      ? (isStartZeroPoseComplete ? '#10b981' : '#3b82f6')
                      : '#e5e7eb',
                    transition: 'background-color 0.2s',
                  }}
                />
              ))}
            </div>
          </div>
        )}

        {isStartZeroPoseComplete && (
          <div
            style={{
              marginTop: '16px',
              padding: '12px 16px',
              borderRadius: '8px',
              backgroundColor: '#ecfdf5',
              border: '1px solid #a7f3d0',
              color: '#065f46',
              fontSize: '15px',
              fontWeight: 600,
              display: 'flex',
              alignItems: 'center',
              gap: '10px',
            }}
          >
            <span
              style={{
                display: 'inline-flex',
                width: '22px',
                height: '22px',
                borderRadius: '50%',
                backgroundColor: '#10b981',
                color: '#fff',
                alignItems: 'center',
                justifyContent: 'center',
                fontSize: '13px',
                fontWeight: 800,
              }}
            >
              ✓
            </span>
            Zero Pose 완료 · <strong>Start</strong>를 누르면 모터가 비활성화 됩니다.
          </div>
        )}

        <div style={{ display: 'flex', gap: '16px', marginTop: '24px' }}>
          <button
            onClick={handleStartZeroPoseButton}
            disabled={startZeroPosePressed}
            style={{
              padding: '16px 32px',
              fontSize: '20px',
              fontWeight: 700,
              backgroundColor: startZeroPosePressed ? '#9ca3af' : '#ef4444',
              color: '#fff',
              border: 'none',
              borderRadius: '8px',
              cursor: startZeroPosePressed ? 'not-allowed' : 'pointer',
              boxShadow: startZeroPosePressed ? 'none' : '0 4px 12px rgba(239, 68, 68, 0.35)',
            }}
          >
            {!startZeroPosePressed
              ? 'Zero Pose'
              : isStartZeroPoseComplete
                ? 'Zero Pose 완료'
                : 'Zero Pose 이동 중'}
          </button>

          <button
            onClick={handleStartButton}
            disabled={!isStartZeroPoseComplete || startSequenceBusy}
            style={{
              padding: '16px 32px',
              fontSize: '20px',
              fontWeight: 700,
              backgroundColor:
                isStartZeroPoseComplete && !startSequenceBusy ? '#16a34a' : '#d1d5db',
              color: '#fff',
              border: 'none',
              borderRadius: '8px',
              cursor:
                isStartZeroPoseComplete && !startSequenceBusy ? 'pointer' : 'not-allowed',
              boxShadow:
                isStartZeroPoseComplete && !startSequenceBusy
                  ? '0 4px 12px rgba(22, 163, 74, 0.35)'
                  : 'none',
            }}
          >
            {startSequenceBusy ? 'Start…' : 'Start'}
          </button>
        </div>

        {startSequenceBusy && (
          <div
            style={{
              marginTop: '16px',
              padding: '14px 18px',
              backgroundColor: '#f0fdf4',
              border: '1px solid #bbf7d0',
              borderRadius: '10px',
              color: '#166534',
              fontSize: '14px',
              fontWeight: 600,
            }}
          >
            시작 중…
          </div>
        )}
      </div>
    </div>
    )
  }


  return (
    <div
      style={{
        padding: '24px 32px',
        height: '100vh',
        boxSizing: 'border-box',
        display: 'flex',
        flexDirection: 'column',
        overflow: 'hidden',
      }}
    >
      <header
        style={{
          display: 'grid',
          gridTemplateColumns: 'auto 1fr auto',
          alignItems: 'center',
          columnGap: '48px',
          marginBottom: '24px',
          width: '100%',
          flexShrink: 0,
        }}
      >
        <div
          style={{
            display: 'flex',
            alignItems: 'baseline',
            gap: '14px',
            flexShrink: 0,
            whiteSpace: 'nowrap',
          }}
        >
          <h1
            style={{
              margin: 0,
              fontSize: '36px',
              fontWeight: 700,
              letterSpacing: '0.02em',
            }}
          >
            AI WORKER
          </h1>
          <span style={{ fontSize: '16px', color: '#888', fontWeight: 400 }}>
            calibration
          </span>
        </div>

        <nav
          aria-label="캘리브레이션 단계"
          style={{
            display: 'flex',
            alignItems: 'center',
            justifyContent: 'center',
            flexWrap: 'nowrap',
            minWidth: 0,
            width: '100%',
          }}
        >
          {TABS.map((tab, i) => {
            const isActive = tab === activeTab
            return (
              <Fragment key={tab}>
                {i > 0 && (
                  <span
                    aria-hidden
                    style={{
                      flexShrink: 0,
                      color: '#e0dcdc',
                      fontSize: '20px',
                      lineHeight: 1,
                      userSelect: 'none',
                      padding: '0 4px',
                    }}
                  >
                    |
                  </span>
                )}
                <div
                  style={{
                    flex: 1,
                    minWidth: 0,
                    display: 'flex',
                    justifyContent: 'center',
                    alignItems: 'center',
                  }}
                >
                  <button
                    type="button"
                    onClick={() => setActiveTab(tab)}
                    style={{
                      width: '100%',
                      maxWidth: '200px',
                      padding: '8px 12px',
                      fontSize: '20px',
                      fontWeight: isActive ? 600 : 400,
                      color: isActive ? '#3b3b3b' : '#888',
                      backgroundColor: 'transparent',
                      border: 'none',
                      outline: 'none',
                      cursor: 'pointer',
                      transition: 'color 0.15s',
                      whiteSpace: 'nowrap',
                      textAlign: 'center',
                    }}
                  >
                    {tab}
                  </button>
                </div>
              </Fragment>
            )
          })}
        </nav>

        <div
          style={{
            flexShrink: 0,
            display: 'flex',
            justifyContent: 'flex-end',
            alignItems: 'center',
          }}
        >
          <div
            style={{
              display: 'inline-flex',
              alignItems: 'center',
              gap: '10px',
              padding: '7px 14px',
              borderRadius: '10px',
              backgroundColor: '#f8fafc',
              border: '1px solid #e5e7eb',
              boxShadow: '0 1px 3px rgba(15, 23, 42, 0.05)',
            }}
          >
            <span
              style={{
                fontSize: '10px',
                fontWeight: 700,
                letterSpacing: '0.16em',
                color: '#9ca3af',
              }}
            >
              VERSION
            </span>
            <span
              aria-hidden
              style={{
                width: '1px',
                height: '14px',
                backgroundColor: '#e2e8f0',
                flexShrink: 0,
              }}
            />
            <span
              style={{
                fontFamily: 'ui-monospace, SFMono-Regular, Menlo, monospace',
                fontSize: '13px',
                fontWeight: 600,
                fontVariantNumeric: 'tabular-nums',
                letterSpacing: '0.03em',
                color: '#475569',
              }}
            >
              v{__APP_VERSION__}
            </span>
          </div>
        </div>
      </header>

      <main
        style={{
          flex: 1,
          minHeight: 0,
          display: 'flex',
          gap: '48px',
          alignItems: stretchPanels ? 'stretch' : 'flex-start',
        }}
      >
        {/* LEFT: joint list */}
        <section
          style={{
            minWidth: '220px',
            flexShrink: 0,
            minHeight: 0,
            alignSelf: 'stretch',
            backgroundColor: '#1a1a1a',
            border: '1px solid #2a2a2a',
            borderRadius: '12px',
            padding: '12px 8px',
            display: 'flex',
            flexDirection: 'column',
            boxSizing: 'border-box',
          }}
        >
          <div style={jointArmListBlock}>
            {sectionHeader('RIGHT')}
            <div style={jointArmListRows}>
              {JOINT_ORDER.slice(0, firstLeftJointIdx).map((j, i) => (
                <div key={j} style={jointArmListRowSlot}>
                  {renderJoint(j, i)}
                </div>
              ))}
            </div>
          </div>

          <div
            style={{
              height: '1px',
              flexShrink: 0,
              backgroundColor: '#2a2a2a',
              margin: '10px 14px',
            }}
          />

          <div style={jointArmListBlock}>
            {sectionHeader('LEFT')}
            <div style={jointArmListRows}>
              {JOINT_ORDER.slice(firstLeftJointIdx).map((j, i) => (
                <div key={j} style={jointArmListRowSlot}>
                  {renderJoint(j, i + firstLeftJointIdx)}
                </div>
              ))}
            </div>
          </div>

          <div
            style={{
              height: '1px',
              flexShrink: 0,
              backgroundColor: '#2a2a2a',
              margin: '10px 14px',
            }}
          />

          <div
            style={{
              flexShrink: 0,
              display: 'flex',
              flexDirection: 'column',
            }}
          >
            {sectionHeader('HEAD')}
            {HEAD_JOINT_ORDER.map(j => (
              <div key={j}>{renderHeadJoint(j)}</div>
            ))}
          </div>
        </section>

        {/* RIGHT: tab content */}
        <section
          style={{
            flex: 1,
            minWidth: 0,
            minHeight: 0,
            alignSelf: stretchPanels ? 'stretch' : 'flex-start',
            display: 'flex',
            flexDirection: 'column',
          }}
        >
          {activeTab === 'Calibration' && !calibrationStarted && renderStartPage()}

          {activeTab === 'Calibration' &&
            calibrationStarted &&
            isDone && (
            <div
              style={{
                flex: 1,
                minHeight: 0,
                width: '100%',
                display: 'flex',
                alignItems: 'center',
                justifyContent: 'center',
                padding: '32px 24px',
                boxSizing: 'border-box',
              }}
            >
              {finalizeUi !== null ? (
                <div
                  role="status"
                  style={{
                    width: '100%',
                    maxWidth: '440px',
                    padding: '28px 32px',
                    borderRadius: '16px',
                    border: `1px solid ${finalizeUi.failed ? '#fecaca' : '#fcd34d'}`,
                    backgroundColor: finalizeUi.failed
                      ? 'rgba(239, 68, 68, 0.08)'
                      : '#fffbeb',
                    boxShadow: '0 16px 40px rgba(15, 23, 42, 0.08)',
                    textAlign: 'center',
                  }}
                >
                  <div
                    style={{
                      fontSize: '13px',
                      fontWeight: 800,
                      letterSpacing: '0.2em',
                      color: finalizeUi.failed ? '#b91c1c' : '#b45309',
                      marginBottom: '10px',
                    }}
                  >
                    CALIBRATION
                  </div>
                  <div
                    style={{
                      fontSize: '22px',
                      fontWeight: 800,
                      color: '#0f172a',
                      letterSpacing: '0.02em',
                      marginBottom: '12px',
                    }}
                  >
                    마무리 중
                  </div>
                  <div
                    style={{
                      fontSize: '15px',
                      color: '#475569',
                      lineHeight: 1.55,
                      marginBottom: '22px',
                    }}
                  >
                    {finalizeUi.detail}
                  </div>
                  <div
                    style={{
                      height: '12px',
                      borderRadius: '8px',
                      backgroundColor: '#e2e8f0',
                      overflow: 'hidden',
                      maxWidth: '320px',
                      margin: '0 auto',
                    }}
                  >
                    <div
                      style={{
                        height: '100%',
                        width: `${Math.round(Math.min(1, Math.max(0, finalizeUi.progress)) * 100)}%`,
                        backgroundColor: finalizeUi.failed ? '#dc2626' : '#2563eb',
                        transition: 'width 0.2s ease-out',
                      }}
                    />
                  </div>
                </div>
              ) : (
                <div
                  style={{
                    fontSize: '17px',
                    fontWeight: 600,
                    color: '#64748b',
                    letterSpacing: '0.04em',
                  }}
                >
                  마무리 중…
                </div>
              )}
            </div>
          )}

          {activeTab === 'Calibration' &&
            calibrationStarted &&
            !isDone &&
            (pendingSafety
              ? renderSafetyPrepPage()
              : (
            <div style={calibCenterShell}>
              {(() => {
                const joint = currentJoint!

                const renderPhoto = (idx: 1 | 2) => {
                  const fileName = `${joint}_${idx}.png`
                  const key = `${joint}_${idx}`
                  const failed = !!jointImgFailed[key]

                  return (
                    <div
                      key={idx}
                      style={{
                        flex: 1,
                        minWidth: '260px',
                        aspectRatio: '4 / 3',
                        borderRadius: '10px',
                        overflow: 'hidden',
                        border: '1px solid #e5e7eb',
                        backgroundColor: '#f8fafc',
                        display: 'flex',
                        alignItems: 'center',
                        justifyContent: 'center',
                      }}
                    >
                      {failed ? (
                        <div style={{ textAlign: 'center', color: '#64748b', padding: '16px' }}>
                          <div
                            style={{
                              fontSize: '11px',
                              fontWeight: 700,
                              letterSpacing: '0.16em',
                              marginBottom: '6px',
                            }}
                          >
                            REFERENCE PHOTO
                          </div>
                          <div
                            style={{
                              fontSize: '12px',
                              fontFamily: 'ui-monospace, SFMono-Regular, Menlo, monospace',
                            }}
                          >
                            public/{fileName}
                          </div>
                        </div>
                      ) : (
                        <img
                          src={`/${fileName}`}
                          alt={`${joint} reference ${idx}`}
                          onError={() =>
                            setJointImgFailed(prev => ({ ...prev, [key]: true }))
                          }
                          style={{
                            width: '100%',
                            height: '100%',
                            objectFit: 'contain',
                            display: 'block',
                          }}
                        />
                      )}
                    </div>
                  )
                }

                return (
                  <div
                    style={{
                      display: 'flex',
                      gap: '16px',
                      flexWrap: 'wrap',
                      marginBottom: '18px',
                    }}
                  >
                    {renderPhoto(1)}
                    {renderPhoto(2)}
                  </div>
                )
              })()}

              <p
                style={{
                  margin: '0 0 28px',
                  padding: '16px 18px',
                  borderRadius: '8px',
                  backgroundColor: '#f3f4f6',
                  border: '1px solid #e5e7eb',
                  color: '#1f2937',
                  fontSize: '18px',
                  lineHeight: 1.6,
                  fontWeight: 600,
                }}
              >
                해당 Joint를 사진과 같이 맞춘 뒤 <strong>Calibrate</strong>를 누르세요.{' '}
                <strong>Skip</strong>을 누르면 해당 Joint는 생략합니다.
              </p>

              <div style={{ marginBottom: '24px' }}>
                <div style={{ fontSize: '14px', color: '#888', marginBottom: '8px' }}>
                  현재 관절
                </div>
                <div style={{ fontSize: '28px', fontFamily: 'monospace' }}>
                  {currentJoint ?? 'All done'}
                </div>
              </div>

              <div style={{ display: 'flex', gap: '16px', flexWrap: 'wrap' }}>
                <button
                  type="button"
                  disabled={!canGoBack}
                  onClick={goToPreviousCalibrationStep}
                  style={{
                    padding: '16px 32px',
                    fontSize: '20px',
                    fontWeight: 600,
                    backgroundColor: !canGoBack ? '#d1d5db' : '#64748b',
                    color: '#fff',
                    border: 'none',
                    borderRadius: '8px',
                    cursor: !canGoBack ? 'not-allowed' : 'pointer',
                  }}
                >
                  이전
                </button>
                <button
                  onClick={handleCalibrate}
                  disabled={isDone}
                  style={{
                    padding: '16px 32px',
                    fontSize: '20px',
                    fontWeight: 600,
                    backgroundColor: isDone ? '#333' : '#16a34a',
                    color: '#fff',
                    border: 'none',
                    borderRadius: '8px',
                    cursor: isDone ? 'not-allowed' : 'pointer',
                  }}
                >
                  Calibrate
                </button>
                <button
                  onClick={() => advance('skipped')}
                  disabled={isDone}
                  style={{
                    padding: '16px 32px',
                    fontSize: '20px',
                    fontWeight: 600,
                    backgroundColor: isDone ? '#333' : '#525252',
                    color: '#fff',
                    border: 'none',
                    borderRadius: '8px',
                    cursor: isDone ? 'not-allowed' : 'pointer',
                  }}
                >
                  Skip
                </button>
              </div>
            </div>
            ))}

          {activeTab === 'Information' && (
            <div
              style={{
                flex: 1,
                minWidth: 0,
                width: '100%',
                alignSelf: 'stretch',
                minHeight: 0,
                overflowY: 'auto',
              }}
            >
              <h2
                style={{
                  margin: '0 0 8px',
                  fontSize: '22px',
                  fontWeight: 800,
                  color: '#1a1a1a',
                }}
              >
                안내
              </h2>
              <p
                style={{
                  margin: '0 0 28px',
                  color: '#666',
                  fontSize: '15px',
                  lineHeight: 1.6,
                }}
              >
                화면 상단 탭 순서대로 진행하세요.
              </p>

              <div
                style={{
                  display: 'grid',
                  gridTemplateColumns: 'repeat(3, minmax(0, 1fr))',
                  gap: '12px',
                  marginBottom: '20px',
                }}
              >
                {(
                  [
                    { tab: 'Calibration', desc: 'Zero Pose → 캘리브레이션' },
                    { tab: 'Result', desc: '캡처·목표·Δ 확인' },
                    { tab: 'Test', desc: '포즈 이동 테스트' },
                  ] as const
                ).map(({ tab, desc }) => (
                  <div
                    key={tab}
                    style={{
                      padding: '14px 16px',
                      backgroundColor: '#fafafa',
                      border: '1px solid #e5e5e5',
                      borderRadius: '10px',
                    }}
                  >
                    <div
                      style={{
                        fontSize: '13px',
                        fontWeight: 700,
                        color: '#1a1a1a',
                        marginBottom: '6px',
                      }}
                    >
                      {tab}
                    </div>
                    <div style={{ fontSize: '13px', color: '#666', lineHeight: 1.45 }}>
                      {desc}
                    </div>
                  </div>
                ))}
              </div>


              <div
                style={{
                  padding: '22px 24px',
                  marginBottom: '16px',
                  backgroundColor: '#fef2f2',
                  border: '1px solid #fca5a5',
                  borderTop: '3px solid #dc2626',
                  borderRadius: '12px',
                  boxShadow: '0 1px 4px rgba(220, 38, 38, 0.08)',
                }}
              >
                <div
                  style={{
                    display: 'flex',
                    alignItems: 'center',
                    gap: '10px',
                    marginBottom: '14px',
                  }}
                >
                  <span
                    style={{
                      width: '28px',
                      height: '28px',
                      borderRadius: '50%',
                      backgroundColor: '#dc2626',
                      color: '#fff',
                      fontSize: '14px',
                      fontWeight: 800,
                      display: 'flex',
                      alignItems: 'center',
                      justifyContent: 'center',
                      flexShrink: 0,
                    }}
                  >
                    !
                  </span>
                  <span
                    style={{
                      fontSize: '14px',
                      fontWeight: 700,
                      letterSpacing: '0.06em',
                      color: '#991b1b',
                    }}
                  >
                    안전 (필수)
                  </span>
                </div>
                <ul
                  style={{
                    margin: 0,
                    paddingLeft: '20px',
                    color: '#333',
                    fontSize: '14px',
                    lineHeight: 1.8,
                  }}
                >
                  <li style={{ marginBottom: '6px' }}>
                    <strong>2인 1조</strong> — 한 명은 <strong>E-STOP</strong> 옆 대기
                  </li>
                  <li style={{ marginBottom: '6px' }}>
                    이상 동작 시 <strong>즉시 E-STOP</strong>
                  </li>
                  <li style={{ marginBottom: '6px' }}>
                    이동·캘리브 전 작업 반경 확인 (사람·장애물)
                  </li>
                  <li>진행 중 다른 조작 금지</li>
                </ul>
              </div>

              <div
                style={{
                  padding: '22px 24px',
                  backgroundColor: '#fff',
                  border: '1px solid #e5e5e5',
                  borderRadius: '12px',
                  boxShadow: '0 1px 4px rgba(0, 0, 0, 0.04)',
                }}
              >
                <div
                  style={{
                    fontSize: '12px',
                    fontWeight: 700,
                    letterSpacing: '0.14em',
                    color: '#888',
                    marginBottom: '12px',
                  }}
                >
                  알아두세요
                </div>
                <ul
                  style={{
                    margin: 0,
                    paddingLeft: '20px',
                    color: '#333',
                    fontSize: '14px',
                    lineHeight: 1.8,
                  }}
                >
                  <li style={{ marginBottom: '6px' }}>
                    <strong>Calibrate</strong>만 누르면 끝이 아닙니다. 이후{' '}
                    <strong>준비</strong> 화면에서 <strong>Homing 적용</strong>까지 해야 로봇에
                    반영됩니다.
                  </li>
                  <li style={{ marginBottom: '6px' }}>
                    <strong>1·3·5번 관절</strong>은 캘리브 전에 <strong>준비</strong> 과정이
                    있습니다. 화면 안내를 순서대로 따라 주세요.
                  </li>
                  <li style={{ marginBottom: '6px' }}>
                    준비에서 <strong>이동</strong> 전, 표에 나온 각도가{' '}
                    <strong>0에 가까운지</strong> 확인하세요.
                  </li>
                  <li>
                    <strong>이전</strong>을 누르면 이미 한 작업이 지워질 수 있어, 처음부터 다시
                    해야 할 수 있습니다.
                  </li>
                </ul>
              </div>
            </div>
          )}

          {activeTab === 'Result' && (
            <div
              style={{
                width: '100%',
                alignSelf: 'stretch',
                height: '100%',
                display: 'flex',
                flexDirection: 'column',
                flex: 1,
                minHeight: 0,
                boxSizing: 'border-box',
              }}
            >
              <div
                style={{
                  display: 'flex',
                  alignItems: 'center',
                  gap: '16px',
                  marginBottom: '28px',
                  flexWrap: 'wrap',
                }}
              >
                <h2
                  style={{
                    margin: 0,
                    fontSize: '22px',
                    fontWeight: 800,
                    color: '#0f172a',
                  }}
                >
                  Calibration Result
                </h2>
                <div
                  role="group"
                  aria-label="각도 단위"
                  style={{
                    display: 'flex',
                    border: '1px solid #e5e7eb',
                    borderRadius: '8px',
                    overflow: 'hidden',
                    backgroundColor: '#f8fafc',
                  }}
                >
                  {(['rad', 'deg'] as const).map(unit => {
                    const selected = resultAngleUnit === unit
                    return (
                      <button
                        key={unit}
                        type="button"
                        onClick={() => setResultAngleUnit(unit)}
                        style={{
                          padding: '8px 16px',
                          fontSize: '13px',
                          fontWeight: selected ? 700 : 500,
                          letterSpacing: '0.06em',
                          textTransform: 'uppercase',
                          border: 'none',
                          cursor: 'pointer',
                          backgroundColor: selected ? '#0f172a' : 'transparent',
                          color: selected ? '#fff' : '#64748b',
                          transition: 'background-color 0.15s, color 0.15s',
                        }}
                      >
                        {unit}
                      </button>
                    )
                  })}
                </div>
              </div>
              <div
                style={{
                  display: 'flex',
                  gap: '32px',
                  flex: 1,
                  minHeight: 0,
                  width: '100%',
                }}
              >
                {renderDeltaTable('RIGHT ARM', 'r')}
                {renderDeltaTable('LEFT ARM', 'l')}
              </div>
            </div>
          )}

          {activeTab === 'Test' && renderTestTab()}
        </section>

        {activeTab === 'Calibration' && (
          <aside
            style={{
              width: `${CALIB_STATE_ASIDE_W}px`,
              flexShrink: 0,
              minHeight: 0,
              alignSelf: 'stretch',
              display: 'flex',
              flexDirection: 'column',
              backgroundColor: '#fff',
              border: '1px solid #e5e7eb',
              borderRadius: '12px',
              padding: '10px 8px',
              boxSizing: 'border-box',
            }}
          >
            <div style={jointArmListBlock}>
              {sectionHeader('RIGHT', true)}
              {stateColumnHeader()}
              <div style={jointArmListRows}>
                {JOINT_ORDER.slice(0, firstLeftJointIdx).map((j, i) => (
                  <div key={j} style={jointArmListRowSlot}>
                    {renderJointState(j, i)}
                  </div>
                ))}
              </div>
            </div>

            <div
              style={{
                height: '1px',
                flexShrink: 0,
                backgroundColor: '#e5e7eb',
                margin: '8px 4px',
              }}
            />

            <div style={jointArmListBlock}>
              {sectionHeader('LEFT', true)}
              {stateColumnHeader()}
              <div style={jointArmListRows}>
                {JOINT_ORDER.slice(firstLeftJointIdx).map((j, i) => (
                  <div key={j} style={jointArmListRowSlot}>
                    {renderJointState(j, i + firstLeftJointIdx)}
                  </div>
                ))}
              </div>
            </div>
          </aside>
        )}

        {activeTab === 'Information' && renderInformationAside()}

        {activeTab === 'Test' && (
          <aside
            style={{
              width: `${RESULT_STATE_ASIDE_W}px`,
              flexShrink: 0,
              minHeight: 0,
              alignSelf: 'stretch',
              display: 'flex',
              flexDirection: 'column',
              backgroundColor: '#fff',
              border: '1px solid #e5e7eb',
              borderRadius: '12px',
              padding: '10px 8px',
              boxSizing: 'border-box',
            }}
          >
            <div style={jointArmListBlock}>
              {sectionHeader('RIGHT', true)}
              {resultColumnHeader()}
              <div style={jointArmListRows}>
                {resultArmJoints('r').map(({ joint, idx }) => (
                  <div key={joint} style={jointArmListRowSlot}>
                    {renderResultJointState(joint, idx)}
                  </div>
                ))}
              </div>
            </div>

            <div
              style={{
                height: '1px',
                flexShrink: 0,
                backgroundColor: '#e5e7eb',
                margin: '8px 4px',
              }}
            />

            <div style={jointArmListBlock}>
              {sectionHeader('LEFT', true)}
              {resultColumnHeader()}
              <div style={jointArmListRows}>
                {resultArmJoints('l').map(({ joint, idx }) => (
                  <div key={joint} style={jointArmListRowSlot}>
                    {renderResultJointState(joint, idx)}
                  </div>
                ))}
              </div>
            </div>
          </aside>
        )}
      </main>

    </div>
  )
}

export default App
