import { Fragment, useEffect, useRef, useState } from 'react'

const TABS = ['Information', 'Calibration', 'Result & Test'] as const
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

type SafetyStage = 'before_joint1' | 'before_joint3' | 'before_joint5'

type PendingSafety = {
  arm: 'right' | 'left'
  stage: SafetyStage
}

const SAFETY_STAGE_TITLE: Record<SafetyStage, string> = {
  before_joint1: 'Joint 1 준비 단계',
  before_joint3: 'Joint 3 준비 단계',
  before_joint5: 'Joint 5 준비 단계',
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

/** 시작 화면 첫 Zero Pose 버튼의 양팔 zero 궤적 `duration_sec` */
const START_PAGE_ZERO_POSE_DURATION_SEC = 5

/** SAFETY: joint 1–7 vs 0 rad, ±tol */
const SAFETY_ZERO_RAD_TOL = 0.5
/** SAFETY「이동」`move_arm_trajectory` duration_sec */
const SAFETY_MOVE_TRAJECTORY_SEC = 3
/** SAFETY 토크 해제 `apply_effort` effort→0 ramp duration_sec */
const SAFETY_TORQUE_RELEASE_SEC = 3
/** SAFETY「이동」traj 이후 effort 램프(초) */
const JOINT7_SAFETY_EFFORT_RAMP_SEC = 5
/** `apply_effort` target 7×0 */
const SAFETY_ZERO_EFFORT_TARGET: readonly number[] = [0, 0, 0, 0, 0, 0, 0]

/** 8축: 4번만 −90° */
const TRAJ_J4_HOLD_RAD = -Math.PI / 2
/** 8축: 6번만 +90° */
const TRAJ_J5_PREP_J6_RAD = Math.PI / 2

/** Joint 1–7·그리퍼 순서: 6번만 +90°, 나머지 0 */
const buildJoint5PrepTrajectoryTargets = (): number[] => [
  0, 0, 0, 0, 0, TRAJ_J5_PREP_J6_RAD, 0, 0,
]

const buildJoint4HoldTrajectoryTargets = (): number[] => [
  0, 0, 0, TRAJ_J4_HOLD_RAD, 0, 0, 0, 0,
]

const buildAllZeroTrajectory8 = (): number[] => [0, 0, 0, 0, 0, 0, 0, 0]

/** SAFETY: effort 램프·모터 homing 대상 순서 (joint 번호 1..7) */
const SAFETY_PREP_CALIB_JOINT_NUMS = [2, 4, 6, 7, 1, 3, 5] as const

const SET_DXL_DATA_SERVICE = '/dynamixel_hardware_interface/set_dxl_data'
const SET_DXL_DATA_TYPE = 'dynamixel_interfaces/srv/SetDataToDxl'
/** effort 해제 직후 / DXL torque on 직전 */
const SAFETY_DXL_TORQUE_PAUSE_MS = 500

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

/** Result 탭 Test Pose: `move_arm_trajectory` 단일 궤적 `time_from_start`(초) */
const TEST_POSE_TRAJECTORY_SEC = 8

const TEST_POSE_PRESETS: Record<
  'test_pose_1' | 'test_pose_2',
  { label: string; right: number[]; left: number[] }
> = {
  test_pose_1: {
    label: 'Test Pose 1',
    right: [0, 0, 0, 0, 0, 0, 0, 0],
    left: [0, 0, 0, 0, 0, 0, 0, 0],
  },
  test_pose_2: {
    label: 'Test Pose 2',
    right: [
      0.6227722314317596, -0.6074444077778085, 0.052143362563206445,
      -2.2258061232220654, -0.1902136176978195, 0.13343236009624007,
      0.6396127502330864, 0.0,
    ],
    left: [
      0.6227722314317596, 0.6074444077778085, 0.052143362563206445,
      -2.2258061232220654, 0.1902136176978195, 0.13343236009624007,
      -0.6396127502330864, 0.0,
    ],
  },
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
  /** torque ramp to 0 done */
  const [safetyTorqueReleased, setSafetyTorqueReleased] = useState(false)
  const [safetyTorqueReleaseBusy, setSafetyTorqueReleaseBusy] = useState(false)
  const [safetyArmMoveBusy, setSafetyArmMoveBusy] = useState(false)
  const [safetyApplyProgress, setSafetyApplyProgress] = useState(0)
  const [jointImgFailed, setJointImgFailed] = useState<Record<string, boolean>>({})
  const [rosConnected, setRosConnected] = useState(false)
  const [jointPositions, setJointPositions] = useState<Record<string, number>>({})
  const [targetRads, setTargetRads] = useState<Record<string, number>>({})
  // 캘리브레이션이 끝난 joint 의 그 시점 measured 값. captured 이후에는
  // 실시간 값 대신 이 값을 표시해서 떨림을 막는다.
  const [capturedPositions, setCapturedPositions] = useState<Record<string, number>>({})
  // Result & Test 탭: 캡처 시점 기준 rad (서비스 응답 measured / target / delta_rad)
  type CaptureResult = {
    measuredRad: number
    targetRad: number
    deltaRad: number
  }
  const [captureResults, setCaptureResults] = useState<Record<string, CaptureResult>>({})
  const [activeTestPose, setActiveTestPose] =
    useState<keyof typeof TEST_POSE_PRESETS | null>(null)
  const jointPositionsRef = useRef(jointPositions)
  const capturedPositionsRef = useRef(capturedPositions)
  const captureResultsRef = useRef(captureResults)
  /** SAFETY 준비: DXL Homing Offset 레지스터에 반영된 관절 (궤적은 base 만 사용) */
  const safetyMotorHomingJointsRef = useRef<Set<string>>(new Set())
  useEffect(() => {
    jointPositionsRef.current = jointPositions
  }, [jointPositions])
  useEffect(() => {
    capturedPositionsRef.current = capturedPositions
  }, [capturedPositions])
  useEffect(() => {
    captureResultsRef.current = captureResults
  }, [captureResults])

  const trajEightNamesForArm = (arm: 'right' | 'left') => {
    const p = arm === 'right' ? 'arm_r' : 'arm_l'
    const grip = arm === 'right' ? 'gripper_r_joint1' : 'gripper_l_joint1'
    return [...[1, 2, 3, 4, 5, 6, 7].map(n => `${p}_joint${n}`), grip]
  }

  /**
   * 캘리된 축만 capture 시 delta_rad 를 baseEight 에 반영 (절대 measured 로 덮지 않음).
   * joint_state 명령 보정은 서버 zero_pose 와 같이 base - delta_rad.
   */
  const mergeCapturedIntoTrajectory8 = (
    arm: 'right' | 'left',
    baseEight: number[],
  ): number[] => {
    const results = captureResultsRef.current
    const homingOnMotor = safetyMotorHomingJointsRef.current
    return trajEightNamesForArm(arm).map((name, i) => {
      const base = baseEight[i] ?? 0
      if (homingOnMotor.has(name)) {
        return base
      }
      const delta = results[name]?.deltaRad
      if (typeof delta === 'number' && Number.isFinite(delta)) {
        return base - delta
      }
      return base
    })
  }

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

  const applySafetyMotorHoming = async (
    arm: 'right' | 'left',
    prepNum: 1 | 3 | 5,
  ) => {
    const candidateJoints = safetyPrepPreviousJointNames(arm, prepNum).filter(
      j => captureResultsRef.current[j] != null,
    )
    const targets: { joint: string; id: number; offset: number }[] = []
    if (candidateJoints.length > 0) {
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
      for (const joint of candidateJoints) {
        const idx = names.indexOf(joint)
        if (idx < 0) continue
        const offset = offsets[idx]
        if (typeof offset !== 'number' || !Number.isFinite(offset)) continue
        const id = jointNameToDxlId(joint)
        if (id == null) continue
        targets.push({ joint, id, offset })
      }
    }

    const dxlIds = targets.map(t => t.id)
    await sleepMs(SAFETY_DXL_TORQUE_PAUSE_MS)
    for (const id of dxlIds) {
      const resp = await callSetDataToDxl(id, 'Torque Enable', 0)
      if (resp?.result === false) {
        throw new Error(`DXL ${id} Torque Enable OFF 실패`)
      }
    }
    for (const t of targets) {
      const resp = await callSetDataToDxl(t.id, 'Homing Offset', t.offset)
      if (resp?.result === false) {
        throw new Error(`${t.joint} Homing Offset 적용 실패`)
      }
      safetyMotorHomingJointsRef.current.add(t.joint)
    }
    await sleepMs(SAFETY_DXL_TORQUE_PAUSE_MS)
    for (const id of dxlIds) {
      const resp = await callSetDataToDxl(id, 'Torque Enable', 1)
      if (resp?.result === false) {
        throw new Error(`DXL ${id} Torque Enable ON 실패`)
      }
    }
  }

  const rosSocketRef = useRef<WebSocket | null>(null)
  const serviceResolversRef = useRef<
    Record<string, (values: RosServiceValues) => void>
  >({})
  const statusListenersRef = useRef<StatusListener[]>([])
  const finalizeRanRef = useRef(false)
  const pendingSafetyRef = useRef<PendingSafety | null>(null)
  const lastRightEffortCmdRef = useRef<number[] | null>(null)
  const lastLeftEffortCmdRef = useRef<number[] | null>(null)
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
  // 양팔 1..7: /joint_states 의 effort 우선, 없으면 effort_controller/commands
  const [torqueStates, setTorqueStates] = useState<Record<string, boolean>>(() =>
    JOINT_ORDER.reduce<Record<string, boolean>>((acc, j) => {
      acc[j] = true
      return acc
    }, {})
  )

  const isDone = currentIndex >= JOINT_ORDER.length
  const currentJoint = !calibrationStarted ? null : isDone ? null : JOINT_ORDER[currentIndex]
  const firstLeftJointIdx = JOINT_ORDER.findIndex(j => j.startsWith('arm_l_'))
  const stretchPanels =
    activeTab === 'Calibration' || activeTab === 'Result & Test'

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
      setRosConnected(true)
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
            if (hasEffort) {
              const e = efforts[index]
              if (typeof e === 'number' && Number.isFinite(e)) {
                next[name] = Math.abs(e) > TORQUE_EFFORT_ABS_EPS
                return
              }
            }
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

    socket.onerror = () => setRosConnected(false)
    socket.onclose = () => setRosConnected(false)

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

    setFinalizeUi({ progress: 0, detail: 'calibration 저장 중…' })

    const zeroEffortPayload = (arm: 'right' | 'left') => ({
      arm,
      target: [...SAFETY_ZERO_EFFORT_TARGET],
      duration_sec: SAFETY_TORQUE_RELEASE_SEC,
      instant_zero_trajectory_first: false,
      effort_joint_2467_preset: false,
      effort_hold_2467_ramp_joint1: false,
      effort_hold_all_ramp_joint3: false,
      safety_prep_joint_number: 0,
    })

    const allJointsOnEffortPayload = (arm: 'right' | 'left') => ({
      arm,
      target: [] as number[],
      duration_sec: JOINT7_SAFETY_EFFORT_RAMP_SEC,
      instant_zero_trajectory_first: false,
      effort_joint_2467_preset: false,
      effort_hold_2467_ramp_joint1: false,
      effort_hold_all_ramp_joint3: false,
      safety_prep_joint_number: 0,
    })

    const zeroTraj8 = [...buildAllZeroTrajectory8()]
    const trajBothZeroArgs = {
      arm: 'both' as const,
      duration_sec: SAFETY_MOVE_TRAJECTORY_SEC,
      use_mid_zero_target: false,
      right_targets: zeroTraj8,
      left_targets: zeroTraj8,
    }
    const trajTimeoutSec = Math.max(15, SAFETY_MOVE_TRAJECTORY_SEC + 25)

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
        bump(0.08, respFinalize?.message ?? '저장 완료 · 오른팔 effort 해제 중')

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

        bump(0.22, '왼팔 effort 해제 중')
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

        bump(0.36, '양팔 전축 0 궤적 이동 중')
        const wTraj = waitForCalibrationTask('move_trajectory', prog => {
          if (!cancelled) {
            bump(0.36 + prog * 0.28, '양팔 전축 0 궤적 이동 중')
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

        bump(0.68, '오른팔 전 관절 effort ON 램프 중')
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

        bump(0.84, '왼팔 전 관절 effort ON 램프 중')
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
        setActiveTab('Result & Test')
      } catch (error) {
        if (!cancelled) {
          console.error(error)
          window.alert(
            error instanceof Error ? error.message : 'calibration 마무리 실패',
          )
          setFinalizeUi(prev => ({
            progress: prev?.progress ?? 0,
            detail: error instanceof Error ? error.message : 'calibration 마무리 실패',
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
      setSafetyTorqueReleased(false)
      setSafetyTorqueReleaseBusy(false)
      safetyMotorHomingJointsRef.current = new Set()
    }
    setSafetyApplyProgress(0)
  }, [pendingSafety])

  // 진행률 바는 ROS /calibration/status 토픽에서 직접 동기화한다.
  // fake 타이머는 더 이상 사용하지 않는다.

  const advance = (status: Status) => {
    if (isDone) return
    const leavingJoint = JOINT_ORDER[currentIndex]
    setStatuses(prev => {
      const next = [...prev]
      next[currentIndex] = status
      return next
    })
    if (leavingJoint === 'arm_r_joint7') {
      setPendingSafety({ arm: 'right', stage: 'before_joint1' })
      return
    }
    if (leavingJoint === 'arm_l_joint7') {
      setPendingSafety({ arm: 'left', stage: 'before_joint1' })
      return
    }
    if (leavingJoint === 'arm_r_joint1') {
      setPendingSafety({ arm: 'right', stage: 'before_joint3' })
      return
    }
    if (leavingJoint === 'arm_l_joint1') {
      setPendingSafety({ arm: 'left', stage: 'before_joint3' })
      return
    }
    if (leavingJoint === 'arm_r_joint3') {
      setPendingSafety({ arm: 'right', stage: 'before_joint5' })
      return
    }
    if (leavingJoint === 'arm_l_joint3') {
      setPendingSafety({ arm: 'left', stage: 'before_joint5' })
      return
    }
    setCurrentIndex(i => i + 1)
  }

  const canGoBack =
    calibrationStarted &&
    (pendingSafety !== null || isDone || currentIndex > 0)

  const goToPreviousCalibrationStep = () => {
    if (!calibrationStarted || !canGoBack) return

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

    if (pendingSafety) {
      const jointAt = JOINT_ORDER[currentIndex]
      setSafetyArmMoveBusy(false)
      setPendingSafety(null)
      setStatuses(prev => {
        const n = [...prev]
        n[currentIndex] = 'pending'
        return n
      })
      clearJointCapture(jointAt)
      return
    }

    if (isDone) {
      const last = JOINT_ORDER.length - 1
      const j = JOINT_ORDER[last]
      setCurrentIndex(last)
      setStatuses(prev => {
        const n = [...prev]
        n[last] = 'pending'
        return n
      })
      clearJointCapture(j)
      return
    }

    if (currentIndex <= 0) return

    const newIdx = currentIndex - 1
    const prevJoint = JOINT_ORDER[newIdx]
    const leavingJoint = JOINT_ORDER[currentIndex]

    setCurrentIndex(newIdx)
    setStatuses(prev => {
      const n = [...prev]
      n[newIdx] = 'pending'
      n[currentIndex] = 'pending'
      return n
    })
    clearJointCapture(prevJoint)
    clearJointCapture(leavingJoint)
  }

  const handleTestPose = async (key: keyof typeof TEST_POSE_PRESETS) => {
    if (activeTestPose) return
    const preset = TEST_POSE_PRESETS[key]
    setActiveTestPose(key)

    const waiter = waitForCalibrationTask('move_trajectory')
    const trajTimeoutSec = Math.max(15, TEST_POSE_TRAJECTORY_SEC + 25)
    try {
      const startResp = await callRosService(
        '/calibration/move_arm_trajectory',
        'ffw_calibration/srv/MoveArmTrajectory',
        {
          arm: 'both',
          duration_sec: TEST_POSE_TRAJECTORY_SEC,
          use_mid_zero_target: false,
          right_targets: mergeCapturedIntoTrajectory8('right', [...preset.right]),
          left_targets: mergeCapturedIntoTrajectory8('left', [...preset.left]),
        },
        trajTimeoutSec,
      )
      if (!startResp?.success) {
        waiter.cancel()
        throw new Error(startResp?.message ?? `${preset.label} failed to start`)
      }
      await waiter.promise
    } catch (error) {
      waiter.cancel()
      console.error(error)
      window.alert(error instanceof Error ? error.message : `${preset.label} failed`)
    } finally {
      setActiveTestPose(null)
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
      // Result & Test 탭용 rad 요약 저장
      if (
        typeof result?.measured_rad === 'number' &&
        typeof result?.target_rad === 'number' &&
        typeof result?.delta_rad === 'number'
      ) {
        setCaptureResults(prev => ({
          ...prev,
          [capturedJoint]: {
            measuredRad: result.measured_rad as number,
            targetRad: result.target_rad as number,
            deltaRad: result.delta_rad as number,
          },
        }))
      }
      advance('captured')
    } catch (error) {
      console.error(error)
      window.alert(error instanceof Error ? error.message : 'Calibration failed')
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
          padding: '8px 14px',
          margin: '2px 0',
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

  const sectionHeader = (label: string) => (
    <div
      style={{
        fontSize: '12px',
        fontWeight: 700,
        letterSpacing: '0.15em',
        color: '#9ca3af',
        padding: '0 14px',
        marginBottom: '4px',
      }}
    >
      {label}
    </div>
  )

  const STATE_GRID_COLUMNS = '36px 36px 1fr 1fr'

  const stateColumnHeader = () => (
    <div
      style={{
        display: 'grid',
        gridTemplateColumns: STATE_GRID_COLUMNS,
        alignItems: 'center',
        gap: '3px',
        padding: '4px 14px 6px 8px',
        fontSize: '10px',
        fontWeight: 700,
        letterSpacing: '0.14em',
        color: '#6b7280',
      }}
    >
      <span style={{ textAlign: 'center' }}>T</span>
      <span>JOINT</span>
      <span style={{ textAlign: 'right' }}>STATE</span>
      <span style={{ textAlign: 'right' }}>Δ TARGET</span>
    </div>
  )

  const renderJointState = (joint: string, i: number) => {
    const isCurrent = calibrationStarted && i === currentIndex
    const status = statuses[i]
    const shortName = joint.replace(/^arm_[rl]_/, '')

    // captured 된 joint 는 그 시점의 값을 고정 표시. 그 외에는 실시간 값.
    const measured =
      status === 'captured' && typeof capturedPositions[joint] === 'number'
        ? capturedPositions[joint]
        : jointPositions[joint]
    const target = targetRads[joint]
    const diff = typeof measured === 'number' && typeof target === 'number'
      ? target - measured
      : null
    const stateValue =
      typeof measured === 'number' ? measured.toFixed(4) : '--'
    const diffValue =
      diff !== null ? `${diff >= 0 ? '+' : ''}${diff.toFixed(4)}` : '--'
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
          display: 'grid',
          gridTemplateColumns: STATE_GRID_COLUMNS,
          alignItems: 'center',
          gap: '3px',
          padding: '8px 14px 8px 8px',
          margin: '2px 0',
          borderRadius: '8px',
          backgroundColor: isCurrent ? 'rgba(59, 130, 246, 0.12)' : 'transparent',
          borderLeft: isCurrent ? '3px solid #3b82f6' : '3px solid transparent',
          opacity: status === 'skipped' ? 0.45 : 1,
          transition: 'background-color 0.15s, border-color 0.15s, opacity 0.15s',
        }}
      >
        <span
          title={`토크 ${torqueOn ? 'ON' : 'OFF'} · joint_states effort(실측), 없으면 effort 명령 토픽`}
          style={{
            justifySelf: 'center',
            width: '10px',
            height: '10px',
            borderRadius: '50%',
            backgroundColor: torqueColor,
            boxShadow: `0 0 0 2px ${torqueOn ? 'rgba(34,197,94,0.18)' : 'rgba(239,68,68,0.18)'}`,
          }}
        />
        <span
          style={{
            fontFamily: 'ui-monospace, SFMono-Regular, Menlo, monospace',
            fontSize: '13px',
            color: nameColor,
            fontWeight: isCurrent ? 600 : 400,
          }}
        >
          {shortName}
        </span>
        <span
          style={{
            fontFamily: 'ui-monospace, SFMono-Regular, Menlo, monospace',
            fontSize: '13px',
            color: valueColor,
            textAlign: 'right',
            fontVariantNumeric: 'tabular-nums',
          }}
        >
          {stateValue}
        </span>
        <span
          style={{
            fontFamily: 'ui-monospace, SFMono-Regular, Menlo, monospace',
            fontSize: '13px',
            color: diffColor,
            textAlign: 'right',
            fontVariantNumeric: 'tabular-nums',
            fontWeight: 500,
          }}
        >
          {diffValue}
        </span>
      </div>
    )
  }

  const jointNamesForSafetyArm = (arm: 'right' | 'left') => {
    const p = arm === 'right' ? 'arm_r_' : 'arm_l_'
    return [1, 2, 3, 4, 5, 6, 7].map(n => `${p}joint${n}`)
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

    const handleSafetyTorqueRelease = async () => {
      if (!pendingSafety || safetyTorqueReleaseBusy) return
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
            instant_zero_trajectory_first: false,
            effort_joint_2467_preset: false,
            effort_hold_2467_ramp_joint1: false,
            effort_hold_all_ramp_joint3: false,
            safety_prep_joint_number: 0,
          },
          SAFETY_TORQUE_RELEASE_SEC + 60,
        )
        if (!resp?.success) {
          throw new Error(resp?.message ?? '토크 해제 실패')
        }
        await waiterRelease.promise

        const prepNum: 1 | 3 | 5 =
          pendingSafetyRef.current?.stage === 'before_joint3'
            ? 3
            : pendingSafetyRef.current?.stage === 'before_joint5'
              ? 5
              : 1
        if (
          pendingSafetyRef.current?.arm === armForStatus &&
          pendingSafetyRef.current?.stage != null
        ) {
          await applySafetyMotorHoming(armForStatus, prepNum)
        }

        if (pendingSafetyRef.current?.arm === armForStatus) {
          setSafetyTorqueReleased(true)
          setSafetyApplyProgress(0)
        }
      } catch (error) {
        console.error(error)
        window.alert(
          error instanceof Error ? error.message : '토크 해제 · Homing 적용 실패',
        )
        setSafetyApplyProgress(0)
      } finally {
        waiterRelease.cancel()
        setSafetyTorqueReleaseBusy(false)
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
      let baseTraj: number[]
      if (stageForAsync === 'before_joint1') {
        baseTraj = buildAllZeroTrajectory8()
      } else if (stageForAsync === 'before_joint3') {
        baseTraj = buildJoint4HoldTrajectoryTargets()
      } else {
        baseTraj = buildJoint5PrepTrajectoryTargets()
      }
      const trajTargets = mergeCapturedIntoTrajectory8(armForStatus, baseTraj)
      const trajDurationSec = SAFETY_MOVE_TRAJECTORY_SEC
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
          instant_zero_trajectory_first: false,
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
          const nextJoint = nextJointKeyAfterSafety(armForStatus, stageForAsync)
          const ni = JOINT_ORDER.indexOf(nextJoint)
          if (ni >= 0) {
            setCurrentIndex(ni)
          }
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

    return (
      <div
        style={{
          flex: 1,
          width: '100%',
          minHeight: 0,
          maxWidth: '960px',
          display: 'flex',
          flexDirection: 'column',
        }}
      >
        <div
          style={{
            flex: 1,
            minHeight: 0,
            display: 'flex',
            flexDirection: 'column',
            padding: '28px 32px',
            backgroundColor: '#fff',
            border: '1px solid #fecaca',
            borderTop: '6px solid #ef4444',
            borderRadius: '12px',
            boxShadow: '0 12px 28px rgba(15, 23, 42, 0.08)',
          }}
        >
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
              margin: '0 0 12px',
              fontSize: '26px',
              fontWeight: 800,
              color: '#0f172a',
            }}
          >
            {armLabel}
          </h2>
          <div style={{ display: 'flex', flexWrap: 'wrap', gap: '12px', marginBottom: '20px' }}>
            <button
              type="button"
              disabled={!canGoBack || safetyArmMoveBusy || safetyTorqueReleaseBusy}
              onClick={goToPreviousCalibrationStep}
              style={{
                padding: '14px 28px',
                fontSize: '17px',
                fontWeight: 700,
                backgroundColor:
                  !canGoBack || safetyArmMoveBusy || safetyTorqueReleaseBusy
                    ? '#d1d5db'
                    : '#64748b',
                color: '#fff',
                border: 'none',
                borderRadius: '8px',
                cursor:
                  !canGoBack || safetyArmMoveBusy || safetyTorqueReleaseBusy
                    ? 'not-allowed'
                    : 'pointer',
              }}
            >
              이전
            </button>
            {!safetyTorqueReleased ? (
              <button
                type="button"
                disabled={safetyTorqueReleaseBusy || safetyArmMoveBusy}
                onClick={() => void handleSafetyTorqueRelease()}
                style={{
                  padding: '14px 28px',
                  fontSize: '17px',
                  fontWeight: 700,
                  backgroundColor:
                    safetyTorqueReleaseBusy || safetyArmMoveBusy ? '#d1d5db' : '#0f766e',
                  color: '#fff',
                  border: 'none',
                  borderRadius: '8px',
                  cursor:
                    safetyTorqueReleaseBusy || safetyArmMoveBusy ? 'not-allowed' : 'pointer',
                }}
              >
                {safetyTorqueReleaseBusy ? 'effort 해제 · Homing…' : '모터 토크 해제'}
              </button>
            ) : null}
            <button
              type="button"
              disabled={
                !safetyTorqueReleased || !allOk || safetyArmMoveBusy || safetyTorqueReleaseBusy
              }
              onClick={() => void handleSafetyMove()}
              style={{
                padding: '14px 28px',
                fontSize: '17px',
                fontWeight: 700,
                backgroundColor:
                  !safetyTorqueReleased || !allOk || safetyArmMoveBusy || safetyTorqueReleaseBusy
                    ? '#d1d5db'
                    : '#2563eb',
                color: '#fff',
                border: 'none',
                borderRadius: '8px',
                cursor:
                  !safetyTorqueReleased || !allOk || safetyArmMoveBusy || safetyTorqueReleaseBusy
                    ? 'not-allowed'
                    : 'pointer',
                boxShadow:
                  !safetyTorqueReleased || !allOk || safetyArmMoveBusy || safetyTorqueReleaseBusy
                    ? 'none'
                    : '0 4px 12px rgba(37, 99, 235, 0.35)',
              }}
            >
              {safetyArmMoveBusy ? '…' : '이동'}
            </button>
          </div>

          {(safetyArmMoveBusy || safetyTorqueReleaseBusy) && (
            <div
              style={{
                marginBottom: '18px',
                padding: '16px 18px',
                backgroundColor: '#eff6ff',
                border: '1px solid #bfdbfe',
                borderRadius: '10px',
              }}
            >
              <div
                style={{
                  display: 'flex',
                  justifyContent: 'space-between',
                  alignItems: 'center',
                  marginBottom: '10px',
                  color: '#1e40af',
                  fontSize: '13px',
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
                      height: '14px',
                      borderRadius: '4px',
                      backgroundColor: i < safetyApplyProgress ? '#2563eb' : '#dbeafe',
                      transition: 'background-color 0.15s',
                    }}
                  />
                ))}
              </div>
            </div>
          )}

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
            duration_sec: START_PAGE_ZERO_POSE_DURATION_SEC,
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
        window.alert(error instanceof Error ? error.message : 'Zero pose failed')
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
        window.alert(error instanceof Error ? error.message : 'Start failed')
      } finally {
        setStartSequenceBusy(false)
      }
    }

    return (
    <div style={{ maxWidth: '760px' }}>
      <div
        style={{
          padding: '28px 32px',
          backgroundColor: '#fff',
          border: '1px solid #fecaca',
          borderTop: '6px solid #ef4444',
          borderRadius: '12px',
          boxShadow: '0 12px 28px rgba(15, 23, 42, 0.08)',
        }}
      >
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
            로봇이 갑자기 움직일 수 있습니다. 항상 로봇과 충분한 거리를 두고
            작업하고, 작업 영역에 사람·장애물이 없는지 확인하세요. 사고 위험에
            주의하세요.
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
          <div>- Calibration 진행 전에 먼저 <strong>Zero Pose</strong> 버튼을 눌러주세요.</div>
          <div>- Zero Pose 는 <strong>궤적만</strong> 보내 zero 로 이동합니다 (effort 변경 없음).</div>
          <div>
            - <strong>Start</strong> 는 양팔 effort 를 <strong>램프 없이 즉시 0</strong>으로만 보냅니다.
            자세는 위 <strong>Zero Pose</strong> 단계에서 이미 맞춰 두었으므로, Start 직후에는
            추가 zero 궤적을 보내지 않아 관절 화면으로 빨리 넘어갑니다.
          </div>
          <div>- Zero Pose 완료 후 <strong>Start</strong> 버튼이 활성화됩니다.</div>
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
            Zero Pose 완료 · Start를 누르면 모터 토크가 풀립니다
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
            <code style={{ fontSize: '13px' }}>/calibration/zero_effort</code> 호출 중…
          </div>
        )}
      </div>
    </div>
    )
  }


  return (
    <div
      style={{
        padding: '32px',
        minHeight: '100vh',
        boxSizing: 'border-box',
        display: 'flex',
        flexDirection: 'column',
      }}
    >
      <header
        style={{
          display: 'flex',
          alignItems: 'baseline',
          gap: '64px',
          marginBottom: '40px',
        }}
      >
        <div style={{ display: 'flex', alignItems: 'baseline', gap: '14px' }}>
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

        <nav style={{ display: 'flex', alignItems: 'center', gap: '100px', marginLeft: '100px' }}>
          {TABS.map((tab, i) => {
            const isActive = tab === activeTab
            return (
              <Fragment key={tab}>
                {i > 0 && (
                  <span
                    style={{
                      color: '#e0dcdc',
                      fontSize: '20px',
                      userSelect: 'none',
                    }}
                  >
                    |
                  </span>
                )}
                <button
                  onClick={() => setActiveTab(tab)}
                  style={{
                    padding: '8px 4px',
                    fontSize: '20px',
                    fontWeight: isActive ? 600 : 400,
                    color: isActive ? '#3b3b3b' : '#888',
                    backgroundColor: 'transparent',
                    border: 'none',
                    outline: 'none',
                    cursor: 'pointer',
                    transition: 'color 0.15s',
                  }}
                >
                  {tab}
                </button>
              </Fragment>
            )
          })}
        </nav>

        <div
          title={rosConnected ? 'ROS connected' : 'ROS disconnected'}
          style={{
            marginLeft: 'auto',
            display: 'flex',
            alignItems: 'center',
            gap: '8px',
            fontSize: '12px',
            fontWeight: 700,
            letterSpacing: '0.12em',
            color: rosConnected ? '#16a34a' : '#dc2626',
          }}
        >
          <span
            style={{
              width: '9px',
              height: '9px',
              borderRadius: '50%',
              backgroundColor: rosConnected ? '#22c55e' : '#ef4444',
              boxShadow: `0 0 0 2px ${rosConnected ? 'rgba(34,197,94,0.18)' : 'rgba(239,68,68,0.18)'}`,
            }}
          />
          ROS
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
            alignSelf: stretchPanels ? 'stretch' : 'flex-start',
            backgroundColor: '#1a1a1a',
            border: '1px solid #2a2a2a',
            borderRadius: '12px',
            padding: '12px 8px',
            display: 'flex',
            flexDirection: 'column',
          }}
        >
          {sectionHeader('RIGHT')}
          {JOINT_ORDER.slice(0, firstLeftJointIdx).map((j, i) => renderJoint(j, i))}

          <div
            style={{
              height: '1px',
              backgroundColor: '#2a2a2a',
              margin: '10px 14px',
            }}
          />

          {sectionHeader('LEFT')}
          {JOINT_ORDER.slice(firstLeftJointIdx).map((j, i) =>
            renderJoint(j, i + firstLeftJointIdx),
          )}
          <div style={{ flex: 1, minHeight: 0 }} aria-hidden />
        </section>

        {/* RIGHT: tab content */}
        <section
          style={{
            flex: 1,
            minWidth: 0,
            minHeight: 0,
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
                    calibration 중
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
                  <div
                    style={{
                      fontSize: '12px',
                      color: '#64748b',
                      marginTop: '12px',
                    }}
                  >
                    마무리: 저장 → 양팔 effort 해제 ({SAFETY_TORQUE_RELEASE_SEC}s) → 전축 0 궤적 (
                    {SAFETY_MOVE_TRAJECTORY_SEC}s) → 전 관절 ON effort 램프 ({JOINT7_SAFETY_EFFORT_RAMP_SEC}s) ·
                    /calibration/status 반영
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
                  calibration 중…
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
            <div style={{ flex: 1, minHeight: 0, maxWidth: '900px', width: '100%' }}>
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
                화면과 같이 조정하고 Calibrate 버튼을 누르세요. 해당 관절
                Calibrate를 하지 않으면 Skip을 누르세요.
              </p>

              <div style={{ marginBottom: '24px' }}>
                <div style={{ fontSize: '14px', color: '#888', marginBottom: '8px' }}>
                  Current joint
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
            <div style={{ maxWidth: '760px' }}>
              {/* Overview */}
              <section style={{ marginBottom: '28px' }}>
                <div
                  style={{
                    fontSize: '12px',
                    fontWeight: 700,
                    letterSpacing: '0.18em',
                    color: '#6b7280',
                    marginBottom: '10px',
                  }}
                >
                  OVERVIEW
                </div>
                <p
                  style={{
                    margin: 0,
                    color: '#1a1a1a',
                    fontSize: '16px',
                    lineHeight: 1.7,
                  }}
                >
                  AI WORKER 양팔 14개 관절의{' '}
                  <strong>Homing Offset</strong>을 보정하는 도구입니다.
                  각 관절을 지정된 기준 자세에 맞춘 뒤 캡처하면, 현재 위치와
                  목표값의 차이를 Dynamixel pulse 단위로 계산해 설정 파일에
                  자동 반영합니다.
                </p>
              </section>

              {/* Procedure */}
              <section style={{ marginBottom: '28px' }}>
                <div
                  style={{
                    fontSize: '12px',
                    fontWeight: 700,
                    letterSpacing: '0.18em',
                    color: '#6b7280',
                    marginBottom: '10px',
                  }}
                >
                  PROCEDURE
                </div>
                <ol
                  style={{
                    margin: 0,
                    paddingLeft: '22px',
                    color: '#1a1a1a',
                    fontSize: '16px',
                    lineHeight: 1.8,
                  }}
                >
                  <li>
                    좌측 리스트에서 <strong>파란색으로 강조된 관절</strong>이
                    현재 작업 대상입니다.
                  </li>
                  <li>
                    한 번에 하나의 관절에 대해 작업을 수행하며, 순서는 좌측 리스트와 같습니다.
                  </li>
                  <li>
                    필요하면 <strong>이전</strong>으로 직전 단계(관절·안전 확인)로
                    돌아가며, 되돌린 관절의 캡처는 초기화됩니다.
                  </li>
                  <li>
                    해당 관절을 작업 지시서의 기준 자세로 정렬합니다.
                  </li>
                  <li>
                    자세를 확인한 뒤 <strong>Calibrate</strong> 버튼으로
                    현재 위치를 캡처합니다. 보정이 필요 없으면{' '}
                    <strong>Skip</strong>으로 건너뜁니다.
                  </li>
                  <li>
                    완료된 보정 결과는 <strong>Result &amp; Test</strong>{' '}
                    탭에서 확인할 수 있습니다.
                  </li>
                </ol>
              </section>

              {/* Safety */}
              <div
                style={{
                  padding: '20px 22px',
                  backgroundColor: 'rgba(239, 68, 68, 0.08)',
                  borderLeft: '4px solid #ef4444',
                  borderRadius: '8px',
                }}
              >
                <div
                  style={{
                    fontSize: '13px',
                    fontWeight: 700,
                    letterSpacing: '0.18em',
                    color: '#b91c1c',
                    marginBottom: '12px',
                  }}
                >
                  SAFETY · 안전 수칙
                </div>
                <ul
                  style={{
                    margin: 0,
                    paddingLeft: '22px',
                    color: '#1a1a1a',
                    fontSize: '15px',
                    lineHeight: 1.8,
                  }}
                >
                  <li>
                    잘못된 조작은{' '}
                    <strong style={{ color: '#b91c1c' }}>
                      로봇 파손 및 인명 사고
                    </strong>
                    로 이어질 수 있습니다.
                  </li>
                  <li>
                    반드시{' '}
                    <strong style={{ color: '#b91c1c' }}>2인 1조</strong>로
                    진행하며, 한 명은 항상{' '}
                    <strong style={{ color: '#b91c1c' }}>
                      E-STOP 옆에서 대기
                    </strong>
                    합니다.
                  </li>
                  <li>
                    Right/Left <strong>Joint 7</strong> 직후 <strong>SAFETY · Joint 1 준비 단계</strong>,
                    joint1 직후 <strong>SAFETY · Joint 3 준비 단계</strong>,
                    joint3 직후 <strong>SAFETY · Joint 5 준비 단계</strong>에서 해당 팔 관절 1~7 을 확인한 뒤{' '}
                    <strong>이동</strong>으로 이 팔만 <code style={{ fontSize: '13px' }}>move_arm_trajectory</code>(
                    <strong>{SAFETY_MOVE_TRAJECTORY_SEC}초</strong>) 후{' '}
                    <code style={{ fontSize: '13px' }}>apply_effort</code>를 적용합니다. 궤적은 Joint 1 준비에서 전축 0
                    목표, Joint 3 준비에서 4번 −90°(나머지 0), Joint 5 준비에서 6번 +90°(나머지 0)입니다.{' '}
                    <strong>모터 토크 해제</strong> 후 캘리된 준비-이전 관절에 DXL Homing Offset을 적용하고, 이동 궤적은
                    모터에 homing 반영된 축은 base 만, 나머지 캘리 축은 base − delta_rad 입니다. effort 는 캘리 순서 <strong>2→4→6→7→1→3→5</strong> 기준 해당 준비 관절<strong>보다 앞선 축만</strong>{' '}
                    <strong>{JOINT7_SAFETY_EFFORT_RAMP_SEC}초</strong>에 ON 목표까지 선형 램프하고, 준비 관절(1·3·5)은 그
                    구간 동안 <strong>0</strong>, 순서상 뒤 축은 램프 시작 시점 값을 유지합니다(Joint 1 준비: 2·4·6·7, Joint 3
                    준비: 2·4·6·7·1, Joint 5 준비: 2·4·6·7·1·3). 완료 후 각각 해당 팔{' '}
                    <strong>joint1</strong> · <strong>joint3</strong> · <strong>joint5</strong> 단계로 이어집니다.
                  </li>
                  <li>
                    이상 거동 발견 시 <strong style={{ color: '#b91c1c' }}>
                    즉시 E-STOP
                    </strong>을 누르고 작업을 중단합니다.
                  </li>
                </ul>
              </div>
            </div>
          )}

          {activeTab === 'Result & Test' && (() => {
            const armJoints = (arm: 'r' | 'l') =>
              JOINT_ORDER
                .map((j, i) => ({ joint: j, idx: i }))
                .filter(({ joint }) => joint.startsWith(`arm_${arm}_`))

            const renderDeltaTable = (label: string, arm: 'r' | 'l') => (
              <div
                style={{
                  flex: 1,
                  minWidth: '320px',
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
                    marginBottom: '8px',
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
                      padding: '10px 14px',
                      fontSize: '11px',
                      fontWeight: 700,
                      letterSpacing: '0.14em',
                      color: '#6b7280',
                      backgroundColor: '#f9fafb',
                      borderBottom: '1px solid #e5e7eb',
                      textAlign: 'center',
                    }}
                  >
                    <span>JOINT</span>
                    <span>MEASURED (rad)</span>
                    <span>TARGET (rad)</span>
                    <span>Δ (rad)</span>
                    <span>STATUS</span>
                  </div>
                  <div style={{ flex: 1, display: 'flex', flexDirection: 'column' }}>
                    {armJoints(arm).map(({ joint, idx }, rowIdx) => {
                      const status = statuses[idx]
                      const shortName = joint.replace(/^arm_[rl]_/, '')
                      const result = captureResults[joint]
                      const before = result ? formatRad4(result.measuredRad) : '--'
                      const after = result ? formatRad4(result.targetRad) : '--'
                      const delta = result
                        ? `${result.deltaRad >= 0 ? '+' : ''}${formatRad4(result.deltaRad)}`
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
                            padding: '12px 14px',
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
                              color: '#6b7280',
                            }}
                          >
                            {before}
                          </span>
                          <span
                            style={{
                              fontFamily:
                                'ui-monospace, SFMono-Regular, Menlo, monospace',
                            }}
                          >
                            {after}
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

            const testButton = (
              key: keyof typeof TEST_POSE_PRESETS,
            ) => {
              const preset = TEST_POSE_PRESETS[key]
              const isRunning = activeTestPose === key
              const isAnotherRunning =
                activeTestPose !== null && activeTestPose !== key
              const disabled = !rosConnected || isAnotherRunning || isRunning
              return (
                <button
                  onClick={() => handleTestPose(key)}
                  disabled={disabled}
                  style={{
                    flex: 1,
                    minWidth: '220px',
                    padding: '20px 28px',
                    fontSize: '18px',
                    fontWeight: 700,
                    backgroundColor: disabled ? '#94a3b8' : '#0f172a',
                    color: '#fff',
                    border: 'none',
                    borderRadius: '10px',
                    cursor: disabled ? 'not-allowed' : 'pointer',
                    letterSpacing: '0.04em',
                    boxShadow: disabled
                      ? 'none'
                      : '0 6px 14px rgba(15, 23, 42, 0.18)',
                    opacity: disabled && !isRunning ? 0.7 : 1,
                  }}
                >
                  {isRunning ? `${preset.label} 이동 중…` : preset.label}
                </button>
              )
            }

            return (
              <div
                style={{
                  maxWidth: '1100px',
                  height: '100%',
                  display: 'flex',
                  flexDirection: 'column',
                }}
              >
                <section
                  style={{
                    marginBottom: '32px',
                    flex: 1,
                    display: 'flex',
                    flexDirection: 'column',
                    minHeight: 0,
                  }}
                >
                  <h2
                    style={{
                      margin: '0 0 6px',
                      fontSize: '22px',
                      fontWeight: 800,
                      color: '#0f172a',
                    }}
                  >
                    Calibration Result
                  </h2>
                  <p
                    style={{
                      margin: '0 0 20px',
                      fontSize: '14px',
                      color: '#6b7280',
                    }}
                  >
                    캡처 시점의 측정 각·목표 각·보정량입니다. (단위: rad)
                  </p>

                  <div
                    style={{
                      display: 'flex',
                      gap: '20px',
                      flexWrap: 'wrap',
                      flex: 1,
                      minHeight: 0,
                    }}
                  >
                    {renderDeltaTable('RIGHT ARM', 'r')}
                    {renderDeltaTable('LEFT ARM', 'l')}
                  </div>
                </section>

                <section
                  style={{
                    padding: '24px',
                    backgroundColor: '#fff',
                    border: '1px solid #e5e7eb',
                    borderRadius: '12px',
                  }}
                >
                  <div
                    style={{
                      fontSize: '12px',
                      fontWeight: 700,
                      letterSpacing: '0.18em',
                      color: '#6b7280',
                      marginBottom: '6px',
                    }}
                  >
                    TEST POSE
                  </div>
                  <p
                    style={{
                      margin: '0 0 18px',
                      fontSize: '14px',
                      color: '#475569',
                    }}
                  >
                    캘리브레이션이 정상 적용되었는지 확인하기 위해 미리 정의된
                    포즈로 이동합니다. <code style={{ fontSize: '13px' }}>move_arm_trajectory</code>
                    로 양팔 궤적만 발행하며(hold·effort 램프 없음), 세그먼트 시간은{' '}
                    <strong>{TEST_POSE_TRAJECTORY_SEC}초</strong>입니다. 이미 캘리브된 축은 preset 목표에{' '}
                    <strong>delta_rad</strong>만 반영(base − delta)됩니다. 실행 전 작업 영역에 사람과 장애물이 없는지
                    반드시 확인하세요.
                  </p>
                  <div
                    style={{
                      display: 'flex',
                      gap: '14px',
                      flexWrap: 'wrap',
                    }}
                  >
                    {testButton('test_pose_1')}
                    {testButton('test_pose_2')}
                  </div>
                </section>
              </div>
            )
          })()}
        </section>

        {activeTab === 'Calibration' && (
          <aside
            style={{
              width: '300px',
              flexShrink: 0,
              alignSelf: 'flex-start',
              backgroundColor: '#fff',
              border: '1px solid #e5e7eb',
              borderRadius: '12px',
              padding: '12px 8px',
            }}
          >
            {sectionHeader('RIGHT')}
            {stateColumnHeader()}
            {JOINT_ORDER.slice(0, firstLeftJointIdx).map((j, i) => renderJointState(j, i))}

            <div
              style={{
                height: '1px',
                backgroundColor: '#e5e7eb',
                margin: '10px 14px',
              }}
            />

            {sectionHeader('LEFT')}
            {stateColumnHeader()}
            {JOINT_ORDER.slice(firstLeftJointIdx).map((j, i) =>
              renderJointState(j, i + firstLeftJointIdx),
            )}
          </aside>
        )}
      </main>

    </div>
  )
}

export default App
