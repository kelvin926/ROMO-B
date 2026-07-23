import { useEffect, useMemo, useRef, useState } from "react";
import {
  ArrowCounterClockwise,
  ArrowsClockwise,
  ArrowDown,
  ArrowLeft,
  ArrowRight,
  ArrowUp,
  Broadcast,
  CarProfile,
  CheckCircle,
  Circle,
  Crosshair,
  FloppyDisk,
  Gauge,
  HandPalm,
  Link,
  LinkBreak,
  LockKey,
  LockKeyOpen,
  MapPin,
  MapTrifold,
  Path,
  Play,
  Power,
  Pulse,
  ListChecks,
  Robot,
  ShieldCheck,
  SteeringWheel,
  Stop,
  StopCircle,
  Trash,
  Warning,
  WifiHigh,
  WifiSlash,
  Wrench,
  PlayCircle,
  PlugsConnected,
  SlidersHorizontal,
} from "@phosphor-icons/react";

const DEMO_TASKS = [
  ["robot_control", "로봇만 연결", "자율주행", "PCU와 웹 직접제어만 실행하며 LiDAR·Nav2·RViz는 켜지 않습니다.", "none", true],
  ["live_mapping", "실시간 매핑", "매핑", "Mid-360, IMU와 휠 오도메트리로 지도와 rosbag을 만듭니다.", "none", true],
  ["save_live_mapping", "현재 지도 저장", "매핑", "PCD, 포즈 그래프와 Nav2 점유 지도를 저장합니다.", "map", false],
  ["record_mapping_bag", "매핑 bag 기록", "매핑", "매핑과 플랫폼 관련 토픽을 모두 기록합니다.", "none", false],
  ["field_navigation", "Nav2 실주행", "자율주행", "위치추정, Nav2, 장애물 회피와 RViz를 실행합니다.", "map", true],
  ["autoware_field", "Autoware 실주행", "자율주행", "Autoware Universe 실주행 스택을 실행합니다.", "map", true],
  ["autoware_planning_sim", "Autoware 계획 시뮬레이션", "자율주행", "로봇 구동 없이 Autoware 경로 계획을 실행합니다.", "map", true],
  ["localization_replay", "위치추정 재생", "자율주행", "선택한 bag과 지도로 위치추정을 재생합니다.", "map_bag", false],
  ["doctor_preflight", "노트북 사전 점검", "점검", "노트북과 소프트웨어 준비 상태를 확인합니다.", "none", false],
  ["doctor_hardware", "하드웨어 정밀 점검", "점검", "PCU와 Mid-360 하드웨어 준비 상태를 확인합니다.", "none", false],
  ["mapping_calibration", "매핑 보정 확인", "점검", "LiDAR, IMU, 휠 오도메트리와 시간 동기를 비교합니다.", "none", false],
  ["nav2_preflight", "Nav2 지도 사전 점검", "점검", "선택한 지도의 lifecycle과 경로 계획을 확인합니다.", "map", false],
  ["autoware_validation", "Autoware 전체 점검", "점검", "격리된 Autoware 시험 전체를 실행합니다.", "map", false],
  ["build_project", "ROMO-B 워크스페이스 빌드", "빌드 및 데이터", "저장소 ROS 2 패키지를 빌드합니다.", "none", false],
  ["prepare_autoware_map", "Autoware 지도 준비", "빌드 및 데이터", "Lanelet2와 PCD 지도 묶음을 생성합니다.", "map", false],
  ["openarm_auto_calibration", "OpenArm 자동 영점 보정", "OpenArm", "원본 OpenArm-v1 기계 한계 캘리브레이션을 한쪽 팔에서 실행합니다.", "none", true],
].map(([id, label, group, description, selection, primary]) => ({
  id, label, group, description, selection, primary, caution: "", running: id === "field_navigation", owned_by_ui: id === "field_navigation", pids: id === "field_navigation" ? [24831] : [], pid: id === "field_navigation" ? 24831 : null, elapsed_sec: id === "field_navigation" ? 128.4 : null, exit_code: null, log_path: id === "field_navigation" ? "/home/hyunseo/ROMO-B/data/local/logs/operator-field.log" : "", message: id === "field_navigation" ? "실행 중" : "실행 대기",
}));

const OPENARM_LIMITS = [
  ["J1", "DM8009", -80, 200],
  ["J2", "DM8009", -100, 100],
  ["J3", "DM4340", -90, 90],
  ["J4", "DM4340", 0, 140],
  ["J5", "DM4310", -90, 90],
  ["J6", "DM4310", -45, 45],
  ["J7", "DM4310", -90, 90],
  ["GRIP", "DM4310", -60, 0],
];
const OPENARM_VELOCITY_LIMITS_DEG_S = [960, 960, 312, 312, 1200, 1200, 1200, 1718.87];

function normalizeOpenArmPoseArchive(raw) {
  const source = raw?.schema_version === 1 && raw?.poses ? raw.poses : raw;
  if (!source || typeof source !== "object" || Array.isArray(source)) {
    throw new Error("자세 JSON 형식이 올바르지 않습니다");
  }
  const entries = Object.entries(source);
  if (entries.length > 200) throw new Error("한 번에 최대 200개 자세만 가져올 수 있습니다");
  const normalized = {};
  entries.forEach(([rawName, pose]) => {
    const name = String(rawName).trim();
    if (!name || name.length > 80 || ["__proto__", "prototype", "constructor"].includes(name)) {
      throw new Error("자세 이름은 1~80자의 일반 문자열이어야 합니다");
    }
    if (!pose || !Array.isArray(pose.left) || !Array.isArray(pose.right) || pose.left.length !== 8 || pose.right.length !== 8) {
      throw new Error(`'${name}' 자세에는 왼팔·오른팔 각 8개 각도가 필요합니다`);
    }
    const clampSide = (values) => values.map((rawValue, index) => {
      const value = Number(rawValue);
      if (!Number.isFinite(value)) throw new Error(`'${name}' 자세에 숫자가 아닌 각도가 있습니다`);
      const lower = OPENARM_LIMITS[index][2];
      const upper = OPENARM_LIMITS[index][3];
      return Math.max(lower, Math.min(upper, value));
    });
    normalized[name] = {
      left: clampSide(pose.left),
      right: clampSide(pose.right),
      saved_at: typeof pose.saved_at === "string" ? pose.saved_at : new Date().toISOString(),
    };
  });
  return normalized;
}

function demoOpenArmMotors(side) {
  const positions = side === "left"
    ? [15, -22, 30, 67, 10, -8, 18, -25]
    : [-15, 22, -30, 67, -10, 8, -18, -25];
  return OPENARM_LIMITS.map(([label, motorType, lower, upper], index) => ({
    key: index === 7 ? "gripper" : `joint_${index + 1}`,
    label,
    motor_type: motorType,
    send_id: index + 1,
    receive_id: index + 17,
    lower_deg: lower,
    upper_deg: upper,
    hard_lower_deg: lower,
    hard_upper_deg: upper,
    velocity_limit_deg_s: OPENARM_VELOCITY_LIMITS_DEG_S[index],
    kp: index < 2 ? 20 : index < 4 ? 12 : 5,
    kd: index < 2 ? 1 : index < 4 ? 0.8 : 0.3,
    online: true,
    age_sec: 0.012 + index * 0.001,
    position_deg: positions[index],
    target_deg: positions[index],
    velocity_rad_s: index % 2 ? -0.008 : 0.006,
    torque_nm: 0.18 + index * 0.03,
    mos_temp_c: 31 + index,
    rotor_temp_c: 29 + index,
    status_byte: 0,
    fault_code: 0,
  }));
}

const DEMO_OPENARM = {
  backend: "내장 Classic SocketCAN",
  library_version: "openarm_can 1.2.9 protocol",
  connected: true,
  any_enabled: true,
  all_enabled: true,
  command_rate_hz: 100,
  trajectory_profile: "quintic_s_curve",
  target_speed_deg_s: 30,
  gain_scale: 1,
  maximum_target_speed_deg_s: 1718.87,
  maximum_gain_scale: 1.5,
  health_level: "READY",
  all_control_ready: true,
  issues: [],
  last_error: "",
  last_calibration: "2026-07-22 14:30:08",
  calibration: {
    status: "verified",
    message: "선택한 관절의 0점 검증을 통과했습니다",
    ready: false,
    verified: true,
    selection_key: "left,right:0,1,2,3,4,5,6,7",
    target_sides: ["left", "right"],
    target_joints: ["J1", "J2", "J3", "J4", "J5", "J6", "J7", "GRIP"],
    checked_at: "2026-07-22 14:29:58",
    written_at: "2026-07-22 14:30:06",
    verified_at: "2026-07-22 14:30:08",
    preflight: [],
    results: [],
    tolerance_deg: 2,
    max_velocity_rad_s: 0.05,
    automatic_tool: { available: true, enabled: true, version: "v1", reason: "원본 OpenArm 기계 한계 자동 보정 도구가 준비되었습니다" },
  },
  arms: {
    left: { interface: "can1", connected: true, enabled: true, online_count: 8, all_online: true, stale_count: 0, fault_count: 0, max_temp_c: 38, oldest_feedback_age_sec: 0.019, command_interlocked: false, interlock_reason: "", control_ready: true, can_error_count: 0, rx_count: 18420, tx_count: 9034, motors: demoOpenArmMotors("left") },
    right: { interface: "can0", connected: true, enabled: true, online_count: 8, all_online: true, stale_count: 0, fault_count: 0, max_temp_c: 38, oldest_feedback_age_sec: 0.019, command_interlocked: false, interlock_reason: "", control_ready: true, can_error_count: 0, rx_count: 18394, tx_count: 9034, motors: demoOpenArmMotors("right") },
  },
  interfaces: {
    left: { name: "can1", exists: true, is_can: true, up: true, operstate: "unknown", statistics: { rx_packets: 18420, tx_packets: 9034, rx_errors: 0, tx_errors: 0 } },
    right: { name: "can0", exists: true, is_can: true, up: true, operstate: "unknown", statistics: { rx_packets: 18394, tx_packets: 9034, rx_errors: 0, tx_errors: 0 } },
  },
  available_interfaces: [{ name: "can0", up: true }, { name: "can1", up: true }],
  log: ["[14:30:08] 현재 자세 유지로 모터 enable: left, right", "[14:30:06] 모터 상태 갱신 요청: left, right", "[14:30:05] CAN 연결: 왼팔 can1, 오른팔 can0"],
};

const DEMO_STATE = {
  version: "0.7.1",
  platform: {
    state: 2,
    state_name: "ARMED_AUTO",
    connected: true,
    auto_mode: true,
    estop: false,
    steer_mode: 0,
    steer_mode_name: "2WIS",
    wheel_speed_mps: [0.21, 0.22, 0.2, 0.21],
    wheel_steer_deg: [7.4, 6.9, 0, 0],
    pcu_alive: 184,
    hlv_alive: 183,
    command_timed_out: false,
    feedback_timed_out: false,
  },
  command: {
    active: false,
    mode: "2wis",
    speed_mps: 0.2,
    steer_deg: 7,
    pivot_rate_radps: 0,
    safe_linear_mps: 0.2,
    safe_angular_radps: 0.076,
  },
  commands: {
    nav: { linear_mps: 0.24, angular_radps: 0.08 },
    selected: { linear_mps: 0.24, angular_radps: 0.08 },
    smoothed: { linear_mps: 0.22, angular_radps: 0.075 },
    safe: { linear_mps: 0.2, angular_radps: 0.076 },
  },
  motion: {
    wheel_odom_speed_mps: 0.205,
    wheel_odom_yaw_rate_radps: 0.075,
    odom_x_m: 4.18,
    odom_y_m: 1.04,
    odom_yaw_deg: 88.9,
  },
  localization: {
    available: true,
    frame_id: "map",
    x_m: 17.42,
    y_m: -4.83,
    yaw_deg: 89.2,
    xy_std_m: 0.08,
    yaw_std_deg: 1.7,
  },
  sensors: {
    lidar_raw: { frame_id: "livox_frame", points: 18420, fields: ["x", "y", "z", "intensity"] },
    lidar_filtered: { frame_id: "livox_frame", points: 6830, fields: ["x", "y", "z", "intensity"] },
    imu: { frame_id: "livox_frame", angular_velocity_radps: [0.002, -0.004, 0.075], linear_acceleration_mps2: [0.01, 0.03, 9.79] },
  },
  navigation: {
    plan_points: 86,
    plan_length_m: 12.64,
    waypoint_count: 4,
    goal_state: "ACTIVE",
    goal: { x_m: 22.1, y_m: -4.4, yaw_deg: 90 },
    last_action: "Navigation stack ready",
    last_action_success: true,
  },
  diagnostics: {
    level: 0,
    summary: "All platform systems nominal",
    items: [
      { name: "ROMO-B / PCU serial bridge", level: 0, message: "Armed 2WIS", values: { device: "/dev/romo_b_pcu", sensor_calibrated: "true", auto_confirmed: "true", reverse_enabled: "true" } },
      { name: "Localization", level: 0, message: "NDT tracking" },
      { name: "Nav2", level: 0, message: "Active" },
    ],
    bridge_values: { device: "/dev/romo_b_pcu", receive_only: "false", sensor_calibrated: "true", auto_confirmed: "true", manual_zero_sent: "false", reverse_enabled: "true", commanded_steer_mode: "2WIS" },
  },
  health: {
    platform: { online: true, age_sec: 0.03, rate_hz: 20.0 },
    lidar: { online: true, age_sec: 0.06, rate_hz: 10.0 },
    lidar_raw: { online: true, age_sec: 0.04, rate_hz: 10.0 },
    lidar_filtered: { online: true, age_sec: 0.06, rate_hz: 10.0 },
    imu: { online: true, age_sec: 0.03, rate_hz: 100.0 },
    localization: { online: true, age_sec: 0.08, rate_hz: 10.0 },
    odometry: { online: true, age_sec: 0.02, rate_hz: 20.0 },
    cmd_nav: { online: true, age_sec: 0.04, rate_hz: 20.0 },
    cmd_selected: { online: true, age_sec: 0.04, rate_hz: 20.0 },
    cmd_smoothed: { online: true, age_sec: 0.04, rate_hz: 20.0 },
    cmd_safe: { online: true, age_sec: 0.04, rate_hz: 20.0 },
  },
  services: {
    arm: true,
    waypoint_save: true,
    waypoint_reload: true,
    waypoint_clear: true,
    waypoint_execute: true,
    waypoint_cancel: true,
    navigate_to_pose: true,
  },
  readiness: {
    bridge_armed: true,
    pcu_auto_confirmed: true,
    ready_to_arm: true,
    control_ready: true,
    checks: [
      { key: "serial", label: "PCU serial feedback", ok: true, detail: "/dev/romo_b_pcu live" },
      { key: "tx", label: "Command transmission", ok: true, detail: "receive_only must be false" },
      { key: "estop", label: "Physical E-stop", ok: true, detail: "PCU feedback must be clear" },
      { key: "initial_mode", label: "Initial steering mode", ok: true, detail: "Arm transition starts in 2WIS" },
      { key: "stopped", label: "Wheel standstill", ok: true, detail: "all wheels below 0.02 m/s" },
      { key: "calibration", label: "LiDAR transform approved", ok: true, detail: "navigation arm requirement" },
      { key: "manual_zero", label: "Manual zero handshake", ok: true, detail: "required before Auto rising edge" },
      { key: "auto_confirmed", label: "Auto control usable", ok: true, detail: "PCU Auto feedback plus bridge confirmation" },
    ],
  },
  runtime: { field_running: true, field_pids: [24831], owned_by_ui: true, log_path: "/home/hyunseo/ROMO-B/data/local/logs/operator-field.log" },
  operations: {
    tasks: DEMO_TASKS,
    artifacts: {
      maps: [{ id: "mapping-20260721-134953", path: "/home/hyunseo/ROMO-B/data/local/maps/mapping-20260721-134953", ready_nav2: true, has_pcd: true, has_pose_graph: true, has_nav2: true, has_autoware: true }],
      bags: [{ id: "mapping-20260721-134953", path: "/home/hyunseo/ROMO-B/data/local/bags/mapping-20260721-134953", has_metadata: true, db3_count: 1 }],
    },
    terminal_only: ["노트북 패키지 설치", "udev·네트워크 적용", "Autoware 소스 설치"],
  },
  graph: { node_count: 31, topic_count: 84, nodes: ["/romo_b_serial_bridge", "/controller_server", "/planner_server", "/lidar_localization_node", "/livox_lidar_publisher"] },
  host: { hostname: "hyunseo-2204", load_1m: 2.14, memory_used_gb: 9.8, memory_total_gb: 31.1, uptime_hours: 18.4, gpu: { available: true, name: "NVIDIA GPU", utilization_percent: 28, memory_used_mb: 1140, memory_total_mb: 4096, temperature_c: 51 } },
  openarm: DEMO_OPENARM,
};

const EMPTY_STATE = {
  ...DEMO_STATE,
  platform: {
    ...DEMO_STATE.platform,
    state: 0,
    state_name: "DISCONNECTED",
    connected: false,
    auto_mode: false,
    wheel_speed_mps: [0, 0, 0, 0],
    wheel_steer_deg: [0, 0, 0, 0],
  },
  localization: { available: false, x_m: 0, y_m: 0, yaw_deg: 0 },
  diagnostics: { level: 3, summary: "ROS 2 데이터 대기 중", items: [] },
  health: {
    platform: { online: false, age_sec: null, rate_hz: 0 },
    lidar: { online: false, age_sec: null, rate_hz: 0 },
    lidar_raw: { online: false, age_sec: null, rate_hz: 0 },
    lidar_filtered: { online: false, age_sec: null, rate_hz: 0 },
    imu: { online: false, age_sec: null, rate_hz: 0 },
    localization: { online: false, age_sec: null, rate_hz: 0 },
    odometry: { online: false, age_sec: null, rate_hz: 0 },
    cmd_nav: { online: false, age_sec: null, rate_hz: 0 },
    cmd_selected: { online: false, age_sec: null, rate_hz: 0 },
    cmd_smoothed: { online: false, age_sec: null, rate_hz: 0 },
    cmd_safe: { online: false, age_sec: null, rate_hz: 0 },
  },
  services: {
    arm: false,
    waypoint_save: false,
    waypoint_reload: false,
    waypoint_clear: false,
    waypoint_execute: false,
    waypoint_cancel: false,
    navigate_to_pose: false,
  },
  readiness: { ...DEMO_STATE.readiness, bridge_armed: false, pcu_auto_confirmed: false, ready_to_arm: false, control_ready: false, checks: DEMO_STATE.readiness.checks.map((item) => ({ ...item, ok: false })) },
  runtime: { field_running: false, field_pids: [], owned_by_ui: false, log_path: "" },
  operations: { ...DEMO_STATE.operations, tasks: DEMO_TASKS.map((task) => ({ ...task, running: false, owned_by_ui: false, pids: [], pid: null, elapsed_sec: null, log_path: "", message: "실행 대기" })) },
  openarm: {
    ...DEMO_OPENARM,
    connected: false,
    any_enabled: false,
    all_enabled: false,
    health_level: "DISCONNECTED",
    all_control_ready: false,
    issues: [],
    arms: {
      left: { ...DEMO_OPENARM.arms.left, connected: false, enabled: false, online_count: 0, all_online: false, stale_count: 8, max_temp_c: null, oldest_feedback_age_sec: null, control_ready: false, rx_count: 0, tx_count: 0, motors: DEMO_OPENARM.arms.left.motors.map((motor) => ({ ...motor, online: false, age_sec: null, position_deg: null, target_deg: null, velocity_rad_s: null, torque_nm: null, mos_temp_c: null, rotor_temp_c: null })) },
      right: { ...DEMO_OPENARM.arms.right, connected: false, enabled: false, online_count: 0, all_online: false, stale_count: 8, max_temp_c: null, oldest_feedback_age_sec: null, control_ready: false, rx_count: 0, tx_count: 0, motors: DEMO_OPENARM.arms.right.motors.map((motor) => ({ ...motor, online: false, age_sec: null, position_deg: null, target_deg: null, velocity_rad_s: null, torque_nm: null, mos_temp_c: null, rotor_temp_c: null })) },
    },
    log: [],
  },
  graph: { node_count: 1, topic_count: 0, nodes: ["/romo_b_operator_ui"] },
  host: { hostname: "hyunseo-2204", load_1m: 0, memory_used_gb: 0, memory_total_gb: 0, uptime_hours: 0, gpu: { available: false } },
};

const TABS = [
  { id: "unified", label: "통합 운용", icon: Robot },
  { id: "main", label: "차량 상세", icon: Gauge },
  { id: "algorithm", label: "플랫폼 제어 알고리즘", icon: Wrench },
  { id: "navigation", label: "자율주행", icon: MapPin },
  { id: "operations", label: "실행 관리", icon: PlayCircle },
  { id: "openarm", label: "양팔 상세", icon: PlugsConnected },
  { id: "system", label: "시스템 상태", icon: ListChecks },
  { id: "diagnostics", label: "진단", icon: Pulse },
];

function format(value, digits = 2) {
  const number = Number(value);
  return Number.isFinite(number) ? number.toFixed(digits) : "—";
}

async function postJson(path, payload = {}) {
  const response = await fetch(path, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(payload),
  });
  const result = await response.json();
  if (!response.ok) throw new Error(result.message || "요청 처리에 실패했습니다");
  return result;
}

function useRobotState(demo) {
  const [state, setState] = useState(demo ? DEMO_STATE : EMPTY_STATE);
  const [streamOnline, setStreamOnline] = useState(demo);

  useEffect(() => {
    if (demo) return undefined;
    let mounted = true;
    fetch("/api/state")
      .then((response) => response.json())
      .then((data) => mounted && setState(data))
      .catch(() => {});
    const events = new EventSource("/api/events");
    events.onmessage = (event) => {
      if (mounted) {
        setState(JSON.parse(event.data));
        setStreamOnline(true);
      }
    };
    events.onerror = () => mounted && setStreamOnline(false);
    return () => {
      mounted = false;
      events.close();
    };
  }, [demo]);
  return [state, streamOnline];
}

function StatusDot({ active, danger = false }) {
  return (
    <span className={`status-dot ${active ? (danger ? "danger" : "active") : ""}`}>
      <Circle weight="fill" />
    </span>
  );
}

function Metric({ label, value, unit, accent = false }) {
  return (
    <div className={`metric ${accent ? "metric-accent" : ""}`}>
      <span>{label}</span>
      <strong>{value}</strong>
      {unit && <small>{unit}</small>}
    </div>
  );
}

function HoldButton({ children, payload, disabled, className = "", onSend }) {
  const timer = useRef(null);
  const send = (active) => onSend({ ...payload, active });
  const stop = () => {
    if (timer.current) window.clearInterval(timer.current);
    timer.current = null;
    send(false);
  };
  const start = (event) => {
    if (disabled) return;
    event.preventDefault();
    event.currentTarget.setPointerCapture?.(event.pointerId);
    send(true);
    timer.current = window.setInterval(() => send(true), 100);
  };
  useEffect(() => () => timer.current && window.clearInterval(timer.current), []);
  return (
    <button
      type="button"
      className={`hold-button ${className}`}
      disabled={disabled}
      onPointerDown={start}
      onPointerUp={stop}
      onPointerCancel={stop}
      onPointerLeave={(event) => event.buttons === 1 && stop()}
    >
      {children}
    </button>
  );
}

function CommandPanel({ state, onPost, demo }) {
  const [mode, setMode] = useState("2wis");
  const [speed, setSpeed] = useState(0.2);
  const [steer, setSteer] = useState(0);
  const [pivotRate, setPivotRate] = useState(0.45);
  const bridgeArmed = state.platform.state === 2;
  const pcuFeedbackFresh = Boolean(
    state.health?.platform?.online && state.platform.connected
  );
  const controlReady = Boolean(state.readiness?.control_ready);
  const controlLabel = controlReady
    ? "주행 준비 완료"
    : bridgeArmed && state.platform.auto_mode && state.platform.estop
      ? "비상정지 작동"
      : bridgeArmed
        ? "AUTO 전환 중"
        : "주행 비활성";
  const canDrive = demo || controlReady;
  const steerLimit = mode === "4wis" ? 18 : 22;
  const firstBlocker = state.readiness?.checks?.find((item) => !item.ok);

  useEffect(() => {
    setSteer((value) => Math.max(-steerLimit, Math.min(steerLimit, value)));
  }, [steerLimit]);

  useEffect(() => {
    const backendMode = state.command?.mode;
    if (!state.command?.active && ["2wis", "4wis", "pivot"].includes(backendMode)) {
      setMode(backendMode);
    }
  }, [state.command?.mode, state.command?.active]);

  const drive = (payload) => {
    if (demo) return;
    postJson("/api/drive", payload).catch((error) => onPost(error.message, false));
  };

  const selectMode = (nextMode) => {
    setMode(nextMode);
    // Apply steering geometry immediately at zero speed. This lets the PCU
    // confirm 4WIS/Pivot and update all four feedback cards before movement.
    drive({ mode: nextMode, active: false, speed_mps: 0, steer_deg: 0, pivot_rate_radps: 0 });
  };

  return (
    <section className="panel command-panel">
      <div className="panel-title">
        <div>
          <span className="eyebrow">플랫폼 명령</span>
          <h2>차량 직접 제어</h2>
        </div>
        <span className={`mode-chip ${controlReady ? "armed" : ""}`}>
          {controlLabel}
        </span>
      </div>

      <div className="command-status-grid">
        <button
          className={`state-control ${bridgeArmed ? "selected" : ""}`}
          disabled={!demo && !state.services.arm}
          onClick={() => onPost("arm", !bridgeArmed)}
        >
          <Power weight="bold" />
          <span>{bridgeArmed ? "HLV 수동 전환 요청" : "HLV Auto / Arm 요청"}</span>
        </button>
        <div className={`readonly-control ${pcuFeedbackFresh && state.platform.estop ? "alert" : ""}`}>
          <ShieldCheck weight="bold" />
          <span>물리 비상정지</span>
          <strong>{!pcuFeedbackFresh ? "확인 불가" : state.platform.estop ? "작동 중" : "해제됨"}</strong>
        </div>
      </div>

      <div className="field-group">
        <div className="field-label-row">
          <label>조향 모드</label>
          <span>PCU: {state.platform.steer_mode_name}</span>
        </div>
        <div className="segmented-control">
          <button className={mode === "2wis" ? "active" : ""} onClick={() => selectMode("2wis")}>2WIS</button>
          <button className={mode === "4wis" ? "active" : ""} onClick={() => selectMode("4wis")}>4WIS</button>
          <button className={mode === "pivot" ? "active" : ""} onClick={() => selectMode("pivot")}>Pivot</button>
        </div>
      </div>

      {mode !== "pivot" ? (
        <>
          <div className="slider-field">
            <div className="field-label-row">
              <label htmlFor="speed">주행 속도 크기</label>
              <output>{format(speed, 2)} m/s</output>
            </div>
            <input id="speed" type="range" min="0" max="1.5" step="0.01" value={speed} onChange={(event) => setSpeed(Number(event.target.value))} />
            <div className="range-labels"><span>0</span><span>매뉴얼 최고속도: 전진·후진 ±1.5 m/s</span></div>
          </div>
          <div className="slider-field">
            <div className="field-label-row">
              <label htmlFor="steer">중앙 조향각</label>
              <output className={Math.abs(steer) > 18 ? "warning-text" : ""}>{format(steer, 1)}°</output>
            </div>
            <input id="steer" type="range" min={-steerLimit} max={steerLimit} step="0.5" value={steer} onChange={(event) => setSteer(Number(event.target.value))} />
            <div className="range-labels"><span>우회전 −{steerLimit}°</span><span>ROS 0°</span><span>좌회전 +{steerLimit}°</span></div>
          </div>
          <div className="drive-pad">
            <HoldButton
              disabled={!canDrive}
              payload={{ mode, speed_mps: speed, steer_deg: steer }}
              onSend={drive}
              className="drive-forward"
            >
              <ArrowUp weight="bold" />
              <span>누르는 동안 전진</span>
              <small>+{format(speed, 2)} m/s · {mode.toUpperCase()} · {format(steer, 1)}°</small>
            </HoldButton>
            <HoldButton
              disabled={!canDrive}
              payload={{ mode, speed_mps: -speed, steer_deg: steer }}
              onSend={drive}
              className="drive-reverse"
            >
              <ArrowDown weight="bold" />
              <span>누르는 동안 후진</span>
              <small>−{format(speed, 2)} m/s · {mode.toUpperCase()} · {format(steer, 1)}°</small>
            </HoldButton>
            <button className="center-steer" onClick={() => setSteer(0)}>
              <Crosshair weight="bold" /> 조향 중앙 정렬
            </button>
          </div>
        </>
      ) : (
        <div className="pivot-control">
          <div className="slider-field">
            <div className="field-label-row">
              <label htmlFor="pivot">제자리 회전 속도</label>
              <output>{format(pivotRate, 2)} rad/s</output>
            </div>
            <input id="pivot" type="range" min="0.1" max="0.75" step="0.05" value={pivotRate} onChange={(event) => setPivotRate(Number(event.target.value))} />
          </div>
          <div className="pivot-buttons">
            <HoldButton disabled={!canDrive} payload={{ mode: "pivot", pivot_rate_radps: pivotRate }} onSend={drive}>
              <ArrowLeft weight="bold" /> 누르는 동안 반시계 회전
            </HoldButton>
            <HoldButton disabled={!canDrive} payload={{ mode: "pivot", pivot_rate_radps: -pivotRate }} onSend={drive}>
              누르는 동안 시계 회전 <ArrowRight weight="bold" />
            </HoldButton>
          </div>
        </div>
      )}

      {!canDrive && (
        <div className="control-hint"><HandPalm weight="fill" /> {firstBlocker ? `${firstBlocker.label}: ${firstBlocker.detail}` : "주행하려면 PCU Auto 확인이 필요합니다."}</div>
      )}
    </section>
  );
}

function WheelCard({ name, speed, steer, steerEnabled }) {
  return (
    <div className="wheel-card">
      <span>{name}</span>
      <strong>{format(speed, 2)} <small>m/s</small></strong>
      <em>{steerEnabled ? `${format(steer, 1)}°` : "2WIS 고정축"}</em>
    </div>
  );
}

function FeedbackPanel({ state }) {
  const platform = state.platform;
  const feedbackFresh = Boolean(state.health?.platform?.online && platform.connected);
  const allWheelSteering = platform.steer_mode !== 0;
  return (
    <section className="panel feedback-panel">
      <div className="panel-title">
        <div>
          <span className="eyebrow">플랫폼 상태 피드백</span>
          <h2>실시간 PCU 상태</h2>
        </div>
        <div className="alive-pair">
          <span>PCU <strong>{platform.pcu_alive}</strong></span>
          <span>HLV <strong>{platform.hlv_alive}</strong></span>
        </div>
      </div>
      <div className="feedback-flags">
        <div><StatusDot active={feedbackFresh} /><span>통신 상태</span><strong>{feedbackFresh ? "연결됨" : "끊김"}</strong></div>
        <div><StatusDot active={state.readiness?.bridge_armed} /><span>HLV 요청</span><strong>{state.readiness?.bridge_armed ? "ARMED" : "수동"}</strong></div>
        <div><StatusDot active={feedbackFresh && platform.auto_mode} /><span>PCU 피드백</span><strong>{!feedbackFresh ? "확인 불가" : platform.auto_mode ? "AUTO" : "수동"}</strong></div>
        <div><StatusDot active={feedbackFresh} danger={feedbackFresh && platform.estop} /><span>비상정지</span><strong>{!feedbackFresh ? "확인 불가" : platform.estop ? "작동 중" : "해제"}</strong></div>
      </div>
      <div className="vehicle-feedback">
        <div className="wheel-column">
          <WheelCard name="FL" speed={platform.wheel_speed_mps[0]} steer={platform.wheel_steer_deg[0]} steerEnabled />
          <WheelCard name="RL" speed={platform.wheel_speed_mps[2]} steer={platform.wheel_steer_deg[2]} steerEnabled={allWheelSteering} />
        </div>
        <div className="vehicle-center">
          <span className="front-label">전방</span>
          <CarProfile size={94} weight="duotone" />
          <strong>ROMO-B</strong>
          <small>{platform.steer_mode_name}</small>
          <span className="rear-label">후방</span>
        </div>
        <div className="wheel-column">
          <WheelCard name="FR" speed={platform.wheel_speed_mps[1]} steer={platform.wheel_steer_deg[1]} steerEnabled />
          <WheelCard name="RR" speed={platform.wheel_speed_mps[3]} steer={platform.wheel_steer_deg[3]} steerEnabled={allWheelSteering} />
        </div>
      </div>
      <div className="feedback-metrics">
        <Metric label="최종 안전 명령" value={format(state.command.safe_linear_mps, 2)} unit="m/s" accent />
        <Metric label="휠 오도메트리" value={format(state.motion.wheel_odom_speed_mps, 2)} unit="m/s" />
        <Metric label="회전 속도" value={format(state.motion.wheel_odom_yaw_rate_radps, 2)} unit="rad/s" />
      </div>
    </section>
  );
}

function AutoReadiness({ state }) {
  return (
    <section className="panel readiness-panel">
      <div className="panel-title compact-title">
        <div><span className="eyebrow">Auto 전환 조건표</span><h2>PCU Auto 진입 및 주행 준비 상태</h2></div>
        <span className={`mode-chip ${state.readiness?.control_ready ? "armed" : ""}`}>
          {state.readiness?.control_ready ? "준비 완료" : "확인 필요"}
        </span>
      </div>
      <div className="readiness-grid">
        {(state.readiness?.checks || []).map((item) => (
          <div className={`readiness-item ${item.ok ? "pass" : "blocked"}`} key={item.key}>
            {item.ok ? <CheckCircle weight="fill" /> : <Warning weight="fill" />}
            <div><strong>{item.label}</strong><span>{item.detail}</span></div>
            <em>{item.ok ? "정상" : "대기"}</em>
          </div>
        ))}
      </div>
      <div className="readiness-explainer">
        <strong>표시 구분</strong>
        <span>RC/본체 스위치로 PCU 조건을 맞춘 뒤 웹의 HLV Arm을 누르면 Auto 전환을 요청합니다. PCU 피드백 AUTO와 브리지 ARMED가 모두 확인되어야 주행 준비 완료로 표시됩니다.</span>
      </div>
    </section>
  );
}

function UnifiedArmRow({ side, arm }) {
  const label = side === "left" ? "왼팔" : "오른팔";
  const feedbackReady = Boolean(arm?.all_online);
  const faulted = Number(arm?.fault_count || 0) > 0 || Boolean(arm?.command_interlocked);
  return (
    <div className={`unified-arm-row ${faulted ? "fault" : feedbackReady ? "ready" : "waiting"}`}>
      <div className="unified-arm-name">
        <StatusDot active={feedbackReady} danger={faulted} />
        <div><strong>{label}</strong><span>{arm?.interface || (side === "left" ? "can1" : "can0")}</span></div>
      </div>
      <div><span>피드백</span><strong>{arm?.online_count || 0}/8</strong></div>
      <div><span>모터</span><strong>{arm?.enabled ? "ENABLE" : "DISABLE"}</strong></div>
      <div><span>최고 온도</span><strong>{arm?.max_temp_c === null || arm?.max_temp_c === undefined ? "—" : `${format(arm.max_temp_c, 0)}°C`}</strong></div>
      <em>{faulted ? "확인 필요" : arm?.control_ready ? "제어 가능" : feedbackReady ? "활성 대기" : "연결 대기"}</em>
    </div>
  );
}

function UnifiedView({ state, onPost, demo, onNavigate }) {
  const openarm = state.openarm || EMPTY_STATE.openarm;
  const leftArm = openarm.arms?.left || {};
  const rightArm = openarm.arms?.right || {};
  const bridgeArmed = state.platform?.state === 2;
  const platformOnline = Boolean(state.platform?.connected && state.health?.platform?.online);
  const openarmOnlineCount = Number(leftArm.online_count || 0) + Number(rightArm.online_count || 0);
  const openarmAllOnline = Boolean(leftArm.all_online && rightArm.all_online);
  const openarmFaults = Number(leftArm.fault_count || 0) + Number(rightArm.fault_count || 0);
  const tasks = state.operations?.tasks || [];
  const primaryTask = tasks.find((task) => task.primary && task.running);
  const fieldTask = tasks.find((task) => task.id === "field_navigation");
  const mappingTask = tasks.find((task) => task.id === "live_mapping");
  const robotTask = tasks.find((task) => task.id === "robot_control");
  const [busyAction, setBusyAction] = useState("");

  const invokeOpenArm = async (action, payload, label) => {
    if (busyAction) return;
    if (demo) return onPost(`데모: ${label}`, true, true);
    setBusyAction(action);
    try {
      const result = await postJson(`/api/openarm/${action}`, payload);
      onPost(result.message, true, true);
    } catch (error) {
      onPost(error.message, false, true);
    } finally {
      setBusyAction("");
    }
  };

  const blockers = [
    ...(state.readiness?.checks || []).filter((item) => !item.ok).map((item) => ({
      source: "ROMO-B",
      title: item.label,
      detail: item.detail,
    })),
    ...(openarm.issues || []).map((issue) => ({ source: "OpenArm", title: issue, detail: "양팔 상세 화면에서 CAN 및 모터 상태를 확인하세요." })),
  ];
  if (!state.health?.lidar?.online) blockers.push({ source: "센서", title: "Mid-360 데이터 대기", detail: "LiDAR 연결 및 실행 상태를 확인하세요." });
  if (!state.health?.localization?.online) blockers.push({ source: "자율주행", title: "위치추정 대기", detail: "지도와 초기 위치를 확인하세요." });

  const unifiedLabel = platformOnline && openarm.connected
    ? "차량·양팔 연결됨"
    : platformOnline || openarm.connected
      ? "일부 하드웨어 연결됨"
      : "하드웨어 연결 대기";
  const overallHealthy = platformOnline && !state.platform.estop && openarm.connected && openarmFaults === 0;

  return (
    <div className="unified-layout" data-testid="unified-page">
      <section className="panel unified-command-center">
        <div className="panel-title unified-title">
          <div><span className="eyebrow">ROMO-B + OPENARM OPERATOR STATION</span><h2>통합 로봇 운용</h2><p>이동 플랫폼과 양팔의 연결·상태·핵심 제어를 한 화면에서 확인합니다.</p></div>
          <span className={`mode-chip ${overallHealthy ? "armed" : blockers.length ? "fault" : ""}`}>{unifiedLabel}</span>
        </div>

        <div className="unified-overview-strip">
          <div><StatusDot active={platformOnline} /><span>차량 통신</span><strong>{platformOnline ? "PCU 정상" : "연결 대기"}</strong><small>{format(state.health?.platform?.rate_hz, 1)} Hz</small></div>
          <div><StatusDot active={state.readiness?.control_ready} danger={state.platform?.estop} /><span>차량 제어</span><strong>{state.platform?.estop ? "E-STOP" : state.readiness?.control_ready ? "주행 준비" : bridgeArmed ? "AUTO 전환 중" : "수동"}</strong><small>{state.platform?.steer_mode_name || "2WIS"}</small></div>
          <div><StatusDot active={openarm.connected} /><span>양팔 CAN</span><strong>{openarm.connected ? `${openarmOnlineCount}/16 수신` : "연결 대기"}</strong><small>{leftArm.interface || "can1"} · {rightArm.interface || "can0"}</small></div>
          <div><StatusDot active={openarm.all_control_ready} danger={openarmFaults > 0} /><span>양팔 제어</span><strong>{openarmFaults ? `${openarmFaults}축 오류` : openarm.all_enabled ? "16축 활성" : openarmAllOnline ? "활성 대기" : "피드백 대기"}</strong><small>{openarm.health_level || "DISCONNECTED"}</small></div>
          <div><StatusDot active={state.health?.lidar?.online} /><span>센서·위치</span><strong>{state.health?.localization?.online ? "지도 추적 중" : "위치추정 대기"}</strong><small>LiDAR {format(state.health?.lidar?.rate_hz, 1)} Hz</small></div>
          <div><StatusDot active={Boolean(primaryTask)} /><span>실행 스택</span><strong>{primaryTask?.label || "정지"}</strong><small>{primaryTask?.running ? `PID ${primaryTask.pid || "확인 중"}` : "실행 관리에서 시작"}</small></div>
        </div>

        <div className="unified-control-grid">
          <article className="unified-subsystem vehicle-subsystem">
            <div className="unified-subsystem-head">
              <span className="unified-icon"><CarProfile weight="duotone" /></span>
              <div><span>이동 플랫폼</span><h3>ROMO-B</h3></div>
              <span className={`mode-chip ${state.readiness?.control_ready ? "armed" : state.platform?.estop ? "fault" : ""}`}>{state.platform?.state_name || "DISCONNECTED"}</span>
            </div>
            <div className="unified-metrics">
              <Metric label="최종 속도 명령" value={format(state.commands?.safe?.linear_mps ?? state.command?.safe_linear_mps, 2)} unit="m/s" accent />
              <Metric label="실측 주행 속도" value={format(state.motion?.wheel_odom_speed_mps, 2)} unit="m/s" />
              <Metric label="조향 모드" value={state.platform?.steer_mode_name || "2WIS"} />
              <Metric label="PCU / HLV Alive" value={`${state.platform?.pcu_alive ?? 0} / ${state.platform?.hlv_alive ?? 0}`} />
            </div>
            <div className="unified-wheel-line">
              {(["FL", "FR", "RL", "RR"]).map((name, index) => <span key={name}><b>{name}</b>{format(state.platform?.wheel_speed_mps?.[index], 2)} m/s <em>{format(state.platform?.wheel_steer_deg?.[index], 1)}°</em></span>)}
            </div>
            <div className="unified-actions">
              <button className={bridgeArmed ? "danger" : "primary"} disabled={!demo && !state.services?.arm} onClick={() => onPost("arm", !bridgeArmed)}>{bridgeArmed ? <LockKey weight="bold" /> : <Power weight="bold" />}{bridgeArmed ? "차량 수동 전환" : "차량 Auto / Arm"}</button>
              <button onClick={() => onNavigate("main")}><SteeringWheel weight="bold" />직접 제어 열기</button>
              <button onClick={() => onNavigate("navigation")}><MapPin weight="bold" />목표·경로 열기</button>
            </div>
          </article>

          <article className="unified-subsystem openarm-subsystem">
            <div className="unified-subsystem-head">
              <span className="unified-icon"><PlugsConnected weight="duotone" /></span>
              <div><span>양팔 매니퓰레이터</span><h3>OpenArm-v1</h3></div>
              <span className={`mode-chip ${openarm.health_level === "READY" ? "armed" : openarmFaults ? "fault" : ""}`}>{openarm.health_level || "DISCONNECTED"}</span>
            </div>
            <div className="unified-arm-list">
              <UnifiedArmRow side="left" arm={leftArm} />
              <UnifiedArmRow side="right" arm={rightArm} />
            </div>
            <div className="unified-arm-facts">
              <span><b>{format(openarm.command_rate_hz, 0)} Hz</b> 제어 주기</span>
              <span><b>{openarmFaults}축</b> Motor fault</span>
              <span><b>{openarm.last_calibration || "기록 없음"}</b> 마지막 영점</span>
            </div>
            <div className="unified-actions openarm-quick-actions">
              {!openarm.connected ? (
                <button className="primary" disabled={Boolean(busyAction)} onClick={() => invokeOpenArm("connect", { left_interface: leftArm.interface || "can1", right_interface: rightArm.interface || "can0" }, "양팔 CAN 연결")}><Link weight="bold" />양팔 CAN 연결</button>
              ) : (
                <button className="danger" disabled={Boolean(busyAction)} onClick={() => invokeOpenArm("disconnect", {}, "모터 비활성화 후 CAN 연결 해제")}><LinkBreak weight="bold" />CAN 연결 해제</button>
              )}
              {!openarm.all_enabled ? (
                <button disabled={Boolean(busyAction) || !openarm.connected || !openarmAllOnline || openarmFaults > 0} onClick={() => invokeOpenArm("enable", { side: "both", enabled: true }, "현재 자세 유지로 양팔 모터 활성화")}><LockKeyOpen weight="bold" />양팔 활성화</button>
              ) : (
                <button className="danger" disabled={Boolean(busyAction)} onClick={() => invokeOpenArm("enable", { side: "both", enabled: false }, "양팔 모터 비활성화")}><LockKey weight="bold" />양팔 비활성화</button>
              )}
              <button disabled={Boolean(busyAction) || !openarm.connected || !openarm.any_enabled || !openarmAllOnline || openarmFaults > 0} onClick={() => invokeOpenArm("hold", { side: "both" }, "양팔 현재 자세 유지")}><HandPalm weight="bold" />현재 자세 유지</button>
              <button onClick={() => onNavigate("openarm")}><SlidersHorizontal weight="bold" />16축 상세 제어</button>
            </div>
          </article>
        </div>
      </section>

      <section className="panel unified-mission-panel">
        <div className="panel-title compact-title"><div><span className="eyebrow">MISSION & RUNTIME</span><h2>자율주행·매핑 실행 상태</h2></div><PlayCircle size={27} weight="duotone" /></div>
        <div className="unified-mission-body">
          <div className="unified-mission-primary">
            <StatusDot active={Boolean(primaryTask)} />
            <div><span>현재 주 실행</span><strong>{primaryTask?.label || "실행 중인 주 스택 없음"}</strong><small>{primaryTask?.message || "실행 관리에서 로봇 연결, 매핑 또는 자율주행을 시작할 수 있습니다."}</small></div>
            <button onClick={() => onNavigate("operations")}>실행 관리 <ArrowRight weight="bold" /></button>
          </div>
          <div className="unified-task-grid">
            <div className={robotTask?.running ? "running" : ""}><Robot weight="duotone" /><span>로봇만 연결</span><strong>{robotTask?.running ? "실행 중" : "대기"}</strong></div>
            <div className={mappingTask?.running ? "running" : ""}><MapTrifold weight="duotone" /><span>실시간 매핑</span><strong>{mappingTask?.running ? "실행 중" : "대기"}</strong></div>
            <div className={fieldTask?.running ? "running" : ""}><Path weight="duotone" /><span>Nav2 실주행</span><strong>{fieldTask?.running ? "실행 중" : "대기"}</strong></div>
            <div className={state.navigation?.goal_state === "ACTIVE" ? "running" : ""}><MapPin weight="duotone" /><span>주행 목표</span><strong>{state.navigation?.goal_state || "없음"}</strong></div>
          </div>
        </div>
      </section>

      <section className="panel unified-attention-panel">
        <div className="panel-title compact-title"><div><span className="eyebrow">OPERATOR ATTENTION</span><h2>확인할 항목</h2></div><span className={`mode-chip ${blockers.length === 0 ? "armed" : "fault"}`}>{blockers.length ? `${blockers.length}건` : "모두 정상"}</span></div>
        <div className="unified-attention-list">
          {blockers.length ? blockers.slice(0, 5).map((item, index) => (
            <div key={`${item.source}-${item.title}-${index}`}><Warning weight="fill" /><span>{item.source}</span><div><strong>{item.title}</strong><small>{item.detail}</small></div></div>
          )) : <div className="unified-all-clear"><CheckCircle weight="fill" /><div><strong>즉시 확인할 문제가 없습니다</strong><small>차량, 양팔, LiDAR와 위치추정의 현재 상태가 정상입니다.</small></div></div>}
        </div>
        <button className="unified-diagnostics-link" onClick={() => onNavigate("diagnostics")}><Pulse weight="bold" />전체 진단 보기 <ArrowRight weight="bold" /></button>
      </section>
    </div>
  );
}

function MainView({ state, onPost, demo }) {
  return (
    <div className="main-grid">
      <CommandPanel state={state} onPost={onPost} demo={demo} />
      <FeedbackPanel state={state} />
      <AutoReadiness state={state} />
    </div>
  );
}

function computeWheelTargets(speed, steerDeg, mode = "2wis") {
  if (mode === "pivot") {
    return { fl: [speed, 30], fr: [-speed, -30], rl: [speed, -30], rr: [-speed, 30] };
  }
  const wheelbase = mode === "4wis" ? 0.323 / 2 : 0.323;
  const track = 0.39;
  if (Math.abs(steerDeg) < 0.01) {
    return { fl: [speed, 0], fr: [speed, 0], rl: [speed, 0], rr: [speed, 0] };
  }
  const steer = (steerDeg * Math.PI) / 180;
  const radius = wheelbase / Math.tan(steer);
  const innerRadius = radius - Math.sign(radius) * track / 2;
  const outerRadius = radius + Math.sign(radius) * track / 2;
  const innerAngle = (Math.atan(wheelbase / innerRadius) * 180) / Math.PI;
  const outerAngle = (Math.atan(wheelbase / outerRadius) * 180) / Math.PI;
  const innerSpeed = speed * Math.hypot(innerRadius, wheelbase) / Math.abs(radius);
  const outerSpeed = speed * Math.hypot(outerRadius, wheelbase) / Math.abs(radius);
  const leftIsInner = steerDeg < 0;
  const values = {
    fl: leftIsInner ? [innerSpeed, innerAngle] : [outerSpeed, outerAngle],
    fr: leftIsInner ? [outerSpeed, outerAngle] : [innerSpeed, innerAngle],
    rl: leftIsInner ? [speed * Math.abs(innerRadius / radius), 0] : [speed * Math.abs(outerRadius / radius), 0],
    rr: leftIsInner ? [speed * Math.abs(outerRadius / radius), 0] : [speed * Math.abs(innerRadius / radius), 0],
  };
  if (mode === "4wis") {
    values.rl[1] = -values.fl[1];
    values.rr[1] = -values.fr[1];
  }
  return values;
}

function AlgorithmView() {
  const [mode, setMode] = useState("2wis");
  const [speed, setSpeed] = useState(0.2);
  const [steer, setSteer] = useState(8);
  const targets = useMemo(() => computeWheelTargets(speed, steer, mode), [speed, steer, mode]);
  const steerLimit = mode === "4wis" ? 18 : 22;
  return (
    <section className="panel algorithm-panel">
      <div className="panel-title wide-title">
        <div><span className="eyebrow">플랫폼 제어 알고리즘</span><h2>{mode.toUpperCase()} 기구학 미리보기</h2></div>
        <div className="geometry-chips"><span>L 0.323 m</span><span>W 0.390 m</span></div>
      </div>
      <div className="algorithm-layout">
        <div className="algorithm-inputs">
          <p>부호가 있는 중심 속도와 조향각을 입력하면 매뉴얼의 차량 치수에 따라 네 바퀴의 목표 선속도와 조향각을 계산합니다.</p>
          <div className="segmented-control algorithm-mode">
            {[
              ["2wis", "2WIS"], ["4wis", "4WIS"], ["pivot", "Pivot"],
            ].map(([value, label]) => <button className={mode === value ? "active" : ""} onClick={() => setMode(value)} key={value}>{label}</button>)}
          </div>
          <div className="slider-field">
            <div className="field-label-row"><label>중심 속도</label><output>{format(speed, 2)} m/s</output></div>
            <input type="range" min="-1.5" max="1.5" step="0.01" value={speed} onChange={(event) => setSpeed(Number(event.target.value))} />
          </div>
          <div className="slider-field">
            <div className="field-label-row"><label>중앙 조향각</label><output>{format(steer, 1)}°</output></div>
            <input type="range" min={-steerLimit} max={steerLimit} step="0.5" value={steer} disabled={mode === "pivot"} onChange={(event) => setSteer(Number(event.target.value))} />
          </div>
          <div className="formula-card"><span>ROS angular.z</span><strong>{format(mode === "pivot" ? -speed / Math.hypot(0.323 / 2, 0.39 / 2) : speed * Math.tan((steer * Math.PI) / 180) / (mode === "4wis" ? 0.323 / 2 : 0.323), 3)} rad/s</strong><small>{mode === "4wis" ? "ω = v · tan(δ) / (L/2)" : mode === "pivot" ? "PCU 양수 속도 = 시계 방향" : "ω = v · tan(δ) / L"}</small></div>
        </div>
        <div className="algorithm-vehicle">
          <div className="prediction-grid">
            {Object.entries(targets).map(([wheel, values]) => (
              <div className={`prediction-card ${wheel}`} key={wheel}>
                <span>{wheel.toUpperCase()}</span>
                <strong>{format(values[0], 3)} m/s</strong>
                <small>{format(values[1], 1)}°</small>
              </div>
            ))}
            <div className="prediction-center"><CarProfile size={126} weight="duotone" /><strong>전방</strong></div>
          </div>
        </div>
      </div>
    </section>
  );
}

function ServiceButton({ icon: Icon, children, action, disabled, onAction, tone = "default" }) {
  return <button className={`service-button ${tone}`} disabled={disabled} onClick={() => onAction(action)}><Icon weight="bold" /><span>{children}</span></button>;
}

function NavigationView({ state, onPost, demo }) {
  const [initialPose, setInitialPose] = useState({ x_m: 0, y_m: 0, yaw_deg: 0, xy_std_m: 0.35, yaw_std_deg: 12 });
  const [goal, setGoal] = useState({ x_m: 0, y_m: 0, yaw_deg: 0 });
  const action = (name) => {
    if (demo) return onPost(`Demo: waypoint ${name}`, true, true);
    postJson(`/api/waypoints/${name}`).then((result) => onPost(result.message, true, true)).catch((error) => onPost(error.message, false, true));
  };
  const sendPose = (kind, values) => {
    if (demo) return onPost(`Demo: ${kind} pose sent`, true, true);
    postJson(`/api/navigation/${kind}`, values)
      .then((result) => onPost(result.message, true, true))
      .catch((error) => onPost(error.message, false, true));
  };
  const cancelGoal = () => {
    if (demo) return onPost("Demo: goal cancel requested", true, true);
    postJson("/api/navigation/cancel")
      .then((result) => onPost(result.message, true, true))
      .catch((error) => onPost(error.message, false, true));
  };
  const poseInputs = (values, setter) => (
    <div className="pose-inputs">
      {["x_m", "y_m", "yaw_deg"].map((key) => (
        <label key={key}><span>{key === "yaw_deg" ? "방향각 (도)" : key === "x_m" ? "지도 X (m)" : "지도 Y (m)"}</span><input type="number" step={key === "yaw_deg" ? "1" : "0.1"} value={values[key]} onChange={(event) => setter({ ...values, [key]: Number(event.target.value) })} /></label>
      ))}
    </div>
  );
  const services = state.services;
  return (
    <div className="navigation-grid">
      <section className="panel route-panel">
        <div className="panel-title"><div><span className="eyebrow">웨이포인트 자율주행</span><h2>경로 관리</h2></div><Path size={30} weight="duotone" /></div>
        <div className="route-summary">
          <div><span>웨이포인트</span><strong>{state.navigation.waypoint_count}</strong></div>
          <div><span>경로 점 개수</span><strong>{state.navigation.plan_points}</strong></div>
          <div><span>경로 길이</span><strong>{format(state.navigation.plan_length_m, 1)} <small>m</small></strong></div>
        </div>
        <div className="service-grid">
          <ServiceButton icon={FloppyDisk} action="save" disabled={!demo && !services.waypoint_save} onAction={action}>RViz 지점 저장</ServiceButton>
          <ServiceButton icon={ArrowCounterClockwise} action="reload" disabled={!demo && !services.waypoint_reload} onAction={action}>YAML 다시 불러오기</ServiceButton>
          <ServiceButton icon={Trash} action="clear" disabled={!demo && !services.waypoint_clear} onAction={action}>편집 지점 지우기</ServiceButton>
          <ServiceButton icon={Play} action="execute" disabled={!demo && !services.waypoint_execute} onAction={action} tone="primary">경로 주행 시작</ServiceButton>
          <ServiceButton icon={Stop} action="cancel" disabled={!demo && !services.waypoint_cancel} onAction={action} tone="danger">경로 주행 취소</ServiceButton>
        </div>
        <div className={`operation-result ${state.navigation.last_action_success ? "success" : "failure"}`}>
          {state.navigation.last_action_success ? <CheckCircle weight="fill" /> : <Warning weight="fill" />}
          <span>{state.navigation.last_action}</span>
        </div>
      </section>
      <section className="panel localization-panel">
        <div className="panel-title"><div><span className="eyebrow">지도 위치추정</span><h2>현재 지도 좌표</h2></div><Crosshair size={30} weight="duotone" /></div>
        <div className="pose-card">
          <Metric label="지도 X" value={format(state.localization.x_m, 2)} unit="m" />
          <Metric label="지도 Y" value={format(state.localization.y_m, 2)} unit="m" />
          <Metric label="로봇 방향" value={format(state.localization.yaw_deg, 1)} unit="도" accent />
        </div>
        <div className="pose-card secondary-pose">
          <Metric label="XY 표준편차" value={format(state.localization.xy_std_m, 2)} unit="m" />
          <Metric label="방향 표준편차" value={format(state.localization.yaw_std_deg, 1)} unit="도" />
          <Metric label="목표 상태" value={state.navigation.goal_state} />
        </div>
        <div className="localization-note"><MapPin weight="fill" /><div><strong>2D Pose Estimate</strong><span>문·기둥·코너 가까이에서 실제 전방 방향으로 지정하세요. 현재 프로필은 클릭 위치 주변 15 m 밖의 정합을 거부합니다.</span></div></div>
      </section>
      <section className="panel pose-command-panel">
        <div className="panel-title"><div><span className="eyebrow">웹 자율주행 제어</span><h2>초기 위치 및 목표 지점</h2></div><MapTrifold size={30} weight="duotone" /></div>
        <div className="pose-command-grid">
          <div className="pose-command-card">
            <div><strong>초기 위치 지정</strong><span>위치추정 오차 범위와 함께 `/initialpose`를 전송합니다.</span></div>
            {poseInputs(initialPose, setInitialPose)}
            <div className="inline-fields">
              <label><span>XY 오차 범위 (m)</span><input type="number" min="0.05" max="2" step="0.05" value={initialPose.xy_std_m} onChange={(event) => setInitialPose({ ...initialPose, xy_std_m: Number(event.target.value) })} /></label>
              <label><span>방향 오차 범위 (도)</span><input type="number" min="2" max="45" step="1" value={initialPose.yaw_std_deg} onChange={(event) => setInitialPose({ ...initialPose, yaw_std_deg: Number(event.target.value) })} /></label>
            </div>
            <button className="service-button primary" type="button" onClick={() => sendPose("initial-pose", initialPose)}><Crosshair weight="bold" />초기 위치 전송</button>
          </div>
          <div className="pose-command-card">
            <div><strong>목표 지점으로 주행</strong><span>지도 좌표계로 Nav2 `NavigateToPose` 목표를 전송합니다.</span></div>
            {poseInputs(goal, setGoal)}
            <div className="goal-actions">
              <button className="service-button primary" type="button" disabled={!demo && !services.navigate_to_pose} onClick={() => sendPose("goal", goal)}><Play weight="bold" />목표 전송</button>
              <button className="service-button danger" type="button" onClick={cancelGoal}><Stop weight="bold" />목표 취소</button>
            </div>
          </div>
        </div>
      </section>
    </div>
  );
}

function HealthCard({ icon: Icon, title, health, expected }) {
  const online = Boolean(health?.online);
  return (
    <div className={`health-card ${online ? "online" : "offline"}`}>
      <Icon weight="duotone" />
      <div><span>{title}</span><strong>{online ? "정상 수신" : "대기 중"}</strong><small>{format(health?.rate_hz, 1)} Hz · 기준 {expected}</small></div>
      <StatusDot active={online} />
    </div>
  );
}

function SystemView({ state, onPost, demo }) {
  const runtimeAction = (action) => {
    if (demo) return onPost(`Demo: field ${action}`, true, true);
    postJson(`/api/runtime/field/${action}`)
      .then((result) => onPost(result.message, true, true))
      .catch((error) => onPost(error.message, false, true));
  };
  const stages = [
    ["Nav2", "nav", "/cmd_vel_nav"],
    ["명령 선택기", "selected", "/cmd_vel_selected"],
    ["속도 평활기", "smoothed", "/cmd_vel_smoothed"],
    ["충돌 감시기", "safe", "/cmd_vel_safe"],
  ];
  const sensorRows = [
    ["Mid-360 원본", "lidar_raw", state.sensors?.lidar_raw],
    ["Mid-360 필터링", "lidar_filtered", state.sensors?.lidar_filtered],
    ["Livox IMU", "imu", state.sensors?.imu],
    ["휠 오도메트리", "odometry", { frame_id: "odom", points: "위치 + 속도" }],
  ];
  return (
    <div className="system-layout">
      <section className="panel runtime-panel">
        <div className="panel-title"><div><span className="eyebrow">프로세스 제어</span><h2>실주행 전체 스택</h2></div><Power size={30} weight="duotone" /></div>
        <div className={`runtime-state ${state.runtime?.field_running ? "running" : "stopped"}`}>
          <StatusDot active={state.runtime?.field_running} />
          <div><span>전체 ROS 2 스택</span><strong>{state.runtime?.field_running ? "실행 중" : "종료됨"}</strong><small>{state.runtime?.field_running ? `PID ${state.runtime.field_pids?.join(", ") || "확인 중"}` : "이 페이지에서 실행할 수 있습니다"}</small></div>
        </div>
        <div className="runtime-actions">
          <button className="service-button primary" disabled={state.runtime?.field_running} onClick={() => runtimeAction("start")}><PlayCircle weight="bold" />자율주행 + LiDAR + RViz 실행</button>
          <button className="service-button danger" disabled={!state.runtime?.field_running} onClick={() => runtimeAction("stop")}><StopCircle weight="bold" />속도 0 및 전체 스택 종료</button>
        </div>
        <div className="runtime-details"><span>실행 주체</span><strong>{state.runtime?.owned_by_ui ? "웹 화면" : state.runtime?.field_running ? "외부 터미널" : "—"}</strong><span>로그</span><code>{state.runtime?.log_path || "다음 실행 시 생성"}</code></div>
      </section>

      <section className="panel pipeline-panel">
        <div className="panel-title"><div><span className="eyebrow">명령 흐름 확인</span><h2>속도 명령 처리 단계</h2></div><Path size={30} weight="duotone" /></div>
        <div className="pipeline-table">
          {stages.map(([label, key, topic], index) => {
            const command = state.commands?.[key] || {};
            const health = state.health?.[`cmd_${key}`];
            return <div className="pipeline-row" key={key}><span>{index + 1}</span><div><strong>{label}</strong><code>{topic}</code></div><em>{format(command.linear_mps, 3)} m/s</em><em>{format(command.angular_radps, 3)} rad/s</em><StatusDot active={health?.online} /></div>;
          })}
        </div>
      </section>

      <section className="panel sensor-panel">
        <div className="panel-title"><div><span className="eyebrow">센서 목록</span><h2>실시간 입력</h2></div><Broadcast size={30} weight="duotone" /></div>
        <div className="sensor-table">
          {sensorRows.map(([label, key, data]) => <div className="sensor-row" key={key}><StatusDot active={state.health?.[key]?.online} /><div><strong>{label}</strong><span>{data?.frame_id || "frame unavailable"}</span></div><em>{format(state.health?.[key]?.rate_hz, 1)} Hz</em><code>{data?.points ?? (data?.angular_velocity_radps ? `${data.angular_velocity_radps.join(", ")} rad/s` : "—")}</code></div>)}
        </div>
      </section>

      <section className="panel host-panel">
        <div className="panel-title"><div><span className="eyebrow">노트북 자원</span><h2>{state.host?.hostname || "노트북"}</h2></div><Gauge size={30} weight="duotone" /></div>
        <div className="host-metrics">
          <Metric label="시스템 부하 (1분)" value={format(state.host?.load_1m, 2)} />
          <Metric label="메모리" value={format(state.host?.memory_used_gb, 1)} unit={`/ ${format(state.host?.memory_total_gb, 1)} GB`} />
          <Metric label="가동 시간" value={format(state.host?.uptime_hours, 1)} unit="시간" />
          <Metric label="GPU 사용률" value={state.host?.gpu?.available ? format(state.host.gpu.utilization_percent, 0) : "없음"} unit={state.host?.gpu?.available ? "%" : ""} accent={state.host?.gpu?.available} />
          <Metric label="GPU 메모리" value={state.host?.gpu?.available ? format(state.host.gpu.memory_used_mb, 0) : "없음"} unit={state.host?.gpu?.available ? `/ ${format(state.host.gpu.memory_total_mb, 0)} MB` : ""} />
          <Metric label="GPU 온도" value={state.host?.gpu?.available ? format(state.host.gpu.temperature_c, 0) : "없음"} unit={state.host?.gpu?.available ? "°C" : ""} />
        </div>
        <div className="gpu-name">{state.host?.gpu?.available ? state.host.gpu.name : "NVIDIA 상태 정보를 사용할 수 없습니다"}</div>
      </section>

      <section className="panel graph-panel">
        <div className="panel-title"><div><span className="eyebrow">ROS 그래프</span><h2>노드 {state.graph?.node_count || 0}개 · 토픽 {state.graph?.topic_count || 0}개</h2></div><ListChecks size={30} weight="duotone" /></div>
        <div className="node-list">{(state.graph?.nodes || []).map((node) => <code key={node}>{node}</code>)}</div>
      </section>
    </div>
  );
}

function OperationsView({ state, onPost, demo }) {
  const operations = state.operations || { tasks: [], artifacts: { maps: [], bags: [] }, terminal_only: [] };
  const maps = operations.artifacts?.maps || [];
  const bags = operations.artifacts?.bags || [];
  const [selectedMap, setSelectedMap] = useState("");
  const [selectedBag, setSelectedBag] = useState("");
  const [selectedTask, setSelectedTask] = useState("field_navigation");
  const [log, setLog] = useState({ log_path: "", tail: "작업을 선택하면 실행 로그가 여기에 표시됩니다." });
  const [useRviz, setUseRviz] = useState(true);
  const [receiveOnly, setReceiveOnly] = useState(true);
  const [replayRate, setReplayRate] = useState(1.0);
  const [maxSpeed, setMaxSpeed] = useState(0.5);
  const mapIds = maps.map((item) => item.id).join("|");
  const bagIds = bags.map((item) => item.id).join("|");

  useEffect(() => {
    if (maps.length && !maps.some((item) => item.id === selectedMap)) {
      setSelectedMap((maps.find((item) => item.ready_nav2) || maps[0]).id);
    }
  }, [mapIds, selectedMap]);

  useEffect(() => {
    if (bags.length && !bags.some((item) => item.id === selectedBag)) {
      setSelectedBag(bags[0].id);
    }
  }, [bagIds, selectedBag]);

  useEffect(() => {
    if (demo) {
      setLog({ log_path: "/home/hyunseo/ROMO-B/data/local/logs/operator-demo.log", tail: "[정보] 웹 실행 관리 준비 완료\n[정보] 허용된 저장소 명령만 실행됩니다\n[정보] 작업을 선택하고 실행 버튼을 누르세요" });
      return undefined;
    }
    let mounted = true;
    const refresh = () => fetch(`/api/operations/${selectedTask}/log`)
      .then((response) => response.json())
      .then((data) => mounted && setLog(data))
      .catch((error) => mounted && setLog({ log_path: "", tail: error.message }));
    refresh();
    const timer = window.setInterval(refresh, 1000);
    return () => { mounted = false; window.clearInterval(timer); };
  }, [selectedTask, demo]);

  const primaryRunning = operations.tasks.find((task) => task.primary && task.running);
  const action = (task, verb) => {
    setSelectedTask(task.id);
    if (demo) return onPost(`Demo: ${task.label} ${verb}`, true, true);
    const payload = {
      map_id: selectedMap,
      bag_id: selectedBag,
      use_rviz: useRviz,
      receive_only: receiveOnly,
      rate: replayRate,
      max_speed_mps: task.id === "robot_control" ? 1.5 : maxSpeed,
    };
    postJson(`/api/operations/${task.id}/${verb}`, payload)
      .then((result) => onPost(result.message, true, true))
      .catch((error) => onPost(error.message, false, true));
  };
  const selectionMissing = (task) => (
    (task.selection === "map" && !selectedMap)
    || (task.selection === "map_bag" && (!selectedMap || !selectedBag))
  );
  const groups = ["매핑", "자율주행", "점검", "빌드 및 데이터"];
  const selectedMapInfo = maps.find((item) => item.id === selectedMap);
  const selectedBagInfo = bags.find((item) => item.id === selectedBag);

  return (
    <div className="operations-layout">
      <section className="panel operations-config">
        <div className="panel-title"><div><span className="eyebrow">웹 작업 제어</span><h2>지도·bag 및 실행 설정</h2></div><MapTrifold size={30} weight="duotone" /></div>
        <div className="artifact-selectors">
          <label><span>지도 선택</span><select value={selectedMap} onChange={(event) => setSelectedMap(event.target.value)}><option value="">지도 없음</option>{maps.map((item) => <option value={item.id} key={item.id}>{item.id}{item.ready_nav2 ? " · Nav2 준비됨" : " · 미완성"}{item.has_autoware ? " · Autoware" : ""}</option>)}</select></label>
          <label><span>Rosbag 선택</span><select value={selectedBag} onChange={(event) => setSelectedBag(event.target.value)}><option value="">bag 없음</option>{bags.map((item) => <option value={item.id} key={item.id}>{item.id}{item.has_metadata ? " · 준비됨" : " · 기록 중/미완성"}</option>)}</select></label>
        </div>
        <div className="artifact-facts">
          <span className={selectedMapInfo?.has_pcd ? "pass" : ""}>PCD {selectedMapInfo?.has_pcd ? "있음" : "없음"}</span>
          <span className={selectedMapInfo?.has_pose_graph ? "pass" : ""}>포즈 그래프 {selectedMapInfo?.has_pose_graph ? "있음" : "없음"}</span>
          <span className={selectedMapInfo?.has_nav2 ? "pass" : ""}>NAV2 {selectedMapInfo?.has_nav2 ? "있음" : "없음"}</span>
          <span className={selectedMapInfo?.has_autoware ? "pass" : ""}>AUTOWARE {selectedMapInfo?.has_autoware ? "있음" : "없음"}</span>
          <span className={selectedBagInfo?.has_metadata ? "pass" : ""}>BAG {selectedBagInfo?.has_metadata ? `DB3 ${selectedBagInfo.db3_count || 0}개` : "없음"}</span>
          <span className="pass">직접제어 ±1.5 m/s</span>
        </div>
        <div className="launch-options">
          <label><input type="checkbox" checked={useRviz} onChange={(event) => setUseRviz(event.target.checked)} /> RViz 열기</label>
          <label><input type="checkbox" checked={receiveOnly} onChange={(event) => setReceiveOnly(event.target.checked)} /> Autoware 수신 전용</label>
          <label><span>재생 배속</span><input type="number" min="0.1" max="4" step="0.1" value={replayRate} onChange={(event) => setReplayRate(Number(event.target.value))} />×</label>
          <label><span>Nav2 최대 속도</span><input type="number" min="0.05" max="1.5" step="0.05" value={maxSpeed} onChange={(event) => setMaxSpeed(Number(event.target.value))} />m/s</label>
        </div>
        {primaryRunning && <div className="primary-running"><StatusDot active /><div><span>현재 실행 중인 주 스택</span><strong>{primaryRunning.label}</strong></div><em>PID {primaryRunning.pid || "확인 중"}</em></div>}
      </section>

      <section className="panel operation-log-panel">
        <div className="panel-title"><div><span className="eyebrow">실시간 프로세스 출력</span><h2>{operations.tasks.find((task) => task.id === selectedTask)?.label || "실행 로그"}</h2></div><Pulse size={30} weight="duotone" /></div>
        <div className="log-meta"><span>{log.log_path || "아직 로그 파일이 없습니다"}</span><button type="button" onClick={() => setLog({ ...log })}>실시간 · 1초</button></div>
        <pre className="operation-log">{log.tail || "출력 대기 중…"}</pre>
      </section>

      {groups.map((group) => (
        <section className="panel operation-group" key={group}>
          <div className="panel-title compact-title"><div><span className="eyebrow">허용된 저장소 작업</span><h2>{group}</h2></div><Wrench size={27} weight="duotone" /></div>
          <div className="operation-cards">
            {operations.tasks.filter((task) => task.group === group).map((task) => {
              const conflict = task.primary && primaryRunning && primaryRunning.id !== task.id;
              const failed = task.exit_code !== null && task.exit_code !== undefined && task.exit_code !== 0;
              const passed = task.exit_code === 0;
              return (
                <article className={`operation-card ${selectedTask === task.id ? "selected" : ""} ${task.running ? "running" : ""}`} key={task.id} onClick={() => setSelectedTask(task.id)}>
                  <div className="operation-card-head"><div><strong>{task.label}</strong><span>{task.description}</span></div><em className={task.running ? "running" : failed ? "failed" : passed ? "passed" : "ready"}>{task.running ? "실행 중" : failed ? `오류 ${task.exit_code}` : passed ? "완료" : "실행 대기"}</em></div>
                  {task.caution && <p><Warning weight="fill" />{task.caution}</p>}
                  <div className="operation-card-foot">
                    <span>{task.running ? `${format(task.elapsed_sec, 0)}초 · PID ${task.pid || "…"}` : task.message || "실행 대기"}</span>
                    {task.running ? (
                      <button className="task-stop" type="button" onClick={(event) => { event.stopPropagation(); action(task, "stop"); }}><StopCircle weight="bold" />종료</button>
                    ) : (
                      <button className="task-run" type="button" disabled={selectionMissing(task) || conflict} title={conflict ? `먼저 ${primaryRunning.label} 작업을 종료하세요` : ""} onClick={(event) => { event.stopPropagation(); action(task, "start"); }}><PlayCircle weight="bold" />실행</button>
                    )}
                  </div>
                </article>
              );
            })}
          </div>
        </section>
      ))}

      <section className="panel terminal-only-panel">
        <div className="panel-title compact-title"><div><span className="eyebrow">관리자 대화형 작업</span><h2>터미널 확인이 필요한 작업</h2></div><ShieldCheck size={27} weight="duotone" /></div>
        <div className="terminal-only-list">{(operations.terminal_only || []).map((item) => <span key={item}><Warning weight="fill" />{item}</span>)}</div>
      </section>
    </div>
  );
}

function OpenArmJointControl({ side, index, motor, value, disabled, onChange, onApply, onLimitsApply }) {
  const hasFault = Number(motor.fault_code || 0) !== 0;
  const lower = Number(motor.lower_deg);
  const upper = Number(motor.upper_deg);
  const hardLower = Number(motor.hard_lower_deg ?? lower);
  const hardUpper = Number(motor.hard_upper_deg ?? upper);
  const [limitDraft, setLimitDraft] = useState({ lower, upper });
  useEffect(() => setLimitDraft({ lower, upper }), [lower, upper]);
  const bounded = Math.max(lower, Math.min(upper, Number(value) || 0));
  const update = (next) => onChange(Math.max(lower, Math.min(upper, Number(next) || 0)));
  return (
    <article className={`openarm-joint ${motor.online ? "online" : "offline"} ${hasFault ? "fault" : ""}`} data-testid={`openarm-${side}-joint-${index + 1}`}>
      <div className="openarm-joint-head">
        <div>
          <StatusDot active={motor.online} danger={hasFault} />
          <strong>{motor.label}</strong>
          <span>{motor.motor_type}</span>
        </div>
        <em>{hasFault ? `FAULT ${motor.fault_code}` : motor.online ? "ONLINE" : "대기"}</em>
      </div>
      <div className="openarm-joint-feedback">
        <div><span>현재각</span><strong>{format(motor.position_deg, 1)}°</strong></div>
        <div><span>속도</span><strong>{format(motor.velocity_rad_s, 2)} <small>rad/s</small></strong></div>
        <div><span>토크</span><strong>{format(motor.torque_nm, 2)} <small>Nm</small></strong></div>
        <div><span>온도</span><strong>{format(motor.mos_temp_c, 0)} / {format(motor.rotor_temp_c, 0)}°C</strong></div>
        <div><span>피드백 지연</span><strong>{motor.age_sec === null || motor.age_sec === undefined ? "—" : `${format(Number(motor.age_sec) * 1000, 0)} ms`}</strong></div>
        <div><span>Fault code</span><strong>{motor.fault_code === null || motor.fault_code === undefined ? "—" : motor.fault_code}</strong></div>
      </div>
      <label className="openarm-target-label">
        <span>목표 각도</span>
        <input
          aria-label={`${side} ${motor.label} 목표 각도`}
          type="number"
          min={lower}
          max={upper}
          step="0.5"
          value={bounded}
          disabled={disabled}
          onChange={(event) => update(event.target.value)}
        />
      </label>
      <input
        className="openarm-joint-slider"
        aria-label={`${side} ${motor.label} 슬라이더`}
        type="range"
        min={lower}
        max={upper}
        step="0.5"
        value={bounded}
        disabled={disabled}
        onChange={(event) => update(event.target.value)}
      />
      <div className="openarm-joint-actions">
        <button disabled={disabled} onClick={() => update(bounded - 1)}>−1°</button>
        <span>{format(lower, 0)}° … {format(upper, 0)}°</span>
        <button disabled={disabled} onClick={() => update(bounded + 1)}>+1°</button>
        <button className="apply" disabled={disabled} onClick={onApply}>이 관절 적용</button>
      </div>
      <div className="openarm-limit-editor">
        <div><strong>운용 각도 범위</strong><small>하드 한계 {format(hardLower, 1)}° … {format(hardUpper, 1)}° · 축 최고 {format(motor.velocity_limit_deg_s, 0)}°/s</small></div>
        <label><span>하한</span><input aria-label={`${side} ${motor.label} 운용 하한`} type="number" min={hardLower} max={hardUpper} step="0.5" value={limitDraft.lower} onChange={(event) => setLimitDraft((current) => ({ ...current, lower: event.target.value }))} /></label>
        <label><span>상한</span><input aria-label={`${side} ${motor.label} 운용 상한`} type="number" min={hardLower} max={hardUpper} step="0.5" value={limitDraft.upper} onChange={(event) => setLimitDraft((current) => ({ ...current, upper: event.target.value }))} /></label>
        <button onClick={() => onLimitsApply(Number(limitDraft.lower), Number(limitDraft.upper), false)}>범위 저장</button>
        <button onClick={() => onLimitsApply(hardLower, hardUpper, true)}>원본 복원</button>
      </div>
    </article>
  );
}

function OpenArmView({ state, onPost, demo }) {
  const openarm = state.openarm || EMPTY_STATE.openarm;
  const automaticTool = openarm.calibration?.automatic_tool || { available: false, enabled: false, version: "v1", reason: "자동 캘리브레이션 도구 상태를 확인 중입니다" };
  const automaticTask = (state.operations?.tasks || []).find((task) => task.id === "openarm_auto_calibration") || {};
  const poseFromFeedback = (side) => (openarm.arms?.[side]?.motors || []).map((motor) => {
    const fallback = Math.max(Number(motor.lower_deg || 0), Math.min(Number(motor.upper_deg || 0), 0));
    return Number.isFinite(Number(motor.position_deg)) ? Number(motor.position_deg) : fallback;
  });
  const [interfaces, setInterfaces] = useState({
    left: openarm.arms?.left?.interface || "can1",
    right: openarm.arms?.right?.interface || "can0",
  });
  const [drafts, setDrafts] = useState({ left: poseFromFeedback("left"), right: poseFromFeedback("right") });
  const [dirty, setDirty] = useState({ left: false, right: false });
  const [speed, setSpeed] = useState(Number(openarm.target_speed_deg_s || 15));
  const [gain, setGain] = useState(Number(openarm.gain_scale || 1));
  const [calibrationSide, setCalibrationSide] = useState("both");
  const [calibrationJoint, setCalibrationJoint] = useState("all");
  const [calibrationText, setCalibrationText] = useState("");
  const [calibrationAcknowledged, setCalibrationAcknowledged] = useState(false);
  const [calibrationReport, setCalibrationReport] = useState(openarm.calibration || null);
  const [automaticSide, setAutomaticSide] = useState("right");
  const [automaticText, setAutomaticText] = useState("");
  const [automaticAcknowledged, setAutomaticAcknowledged] = useState(false);
  const [automaticLog, setAutomaticLog] = useState("아직 자동 캘리브레이션 실행 로그가 없습니다.");
  const [demoAutomaticRunning, setDemoAutomaticRunning] = useState(false);
  const [poseName, setPoseName] = useState("기본 자세");
  const [selectedPose, setSelectedPose] = useState("");
  const poseImportRef = useRef(null);
  const [savedPoses, setSavedPoses] = useState(() => {
    try {
      return JSON.parse(window.localStorage.getItem("romo-b-openarm-poses") || "{}");
    } catch {
      return {};
    }
  });

  useEffect(() => {
    if (!openarm.connected) {
      setInterfaces({
        left: openarm.arms?.left?.interface || "can1",
        right: openarm.arms?.right?.interface || "can0",
      });
    }
  }, [openarm.connected, openarm.arms?.left?.interface, openarm.arms?.right?.interface]);

  useEffect(() => {
    setDrafts((current) => ({
      left: dirty.left ? current.left : poseFromFeedback("left"),
      right: dirty.right ? current.right : poseFromFeedback("right"),
    }));
  }, [openarm.arms?.left?.rx_count, openarm.arms?.right?.rx_count, dirty.left, dirty.right]);

  useEffect(() => {
    if (openarm.calibration) setCalibrationReport(openarm.calibration);
  }, [openarm.calibration]);

  useEffect(() => {
    if (demo) return undefined;
    let mounted = true;
    const refresh = () => fetch("/api/operations/openarm_auto_calibration/log")
      .then((response) => response.json())
      .then((data) => mounted && setAutomaticLog(data.tail || "출력 대기 중…"))
      .catch((error) => mounted && setAutomaticLog(error.message));
    refresh();
    const timer = window.setInterval(refresh, 1000);
    return () => { mounted = false; window.clearInterval(timer); };
  }, [demo]);

  const notifyResult = (message, success = true) => onPost(message, success, true);
  const invoke = async (action, payload, demoMessage) => {
    if (demo) {
      notifyResult(`데모: ${demoMessage}`, true);
      return { accepted: true };
    }
    try {
      const result = await postJson(`/api/openarm/${action}`, payload);
      notifyResult(result.message, true);
      return result;
    } catch (error) {
      notifyResult(error.message, false);
      return null;
    }
  };
  const setJointDraft = (side, index, value) => {
    setDrafts((current) => ({ ...current, [side]: current[side].map((item, itemIndex) => itemIndex === index ? value : item) }));
    setDirty((current) => ({ ...current, [side]: true }));
  };
  const syncFeedback = (side = "both") => {
    const sides = side === "both" ? ["left", "right"] : [side];
    setDrafts((current) => {
      const next = { ...current };
      sides.forEach((selected) => { next[selected] = poseFromFeedback(selected); });
      return next;
    });
    setDirty((current) => ({
      ...current,
      ...(sides.includes("left") ? { left: false } : {}),
      ...(sides.includes("right") ? { right: false } : {}),
    }));
    notifyResult("현재 모터 피드백을 편집값으로 불러왔습니다", true);
  };
  const applyPose = async (side) => {
    const sides = side === "both" ? ["left", "right"] : [side];
    const targets = Object.fromEntries(sides.map((selected) => [selected, drafts[selected]]));
    const result = await invoke("pose", { targets, speed_deg_s: speed, gain_scale: gain }, `${side === "both" ? "양팔" : side === "left" ? "왼팔" : "오른팔"} 자세 적용`);
    if (result) setDirty((current) => ({ ...current, ...(sides.includes("left") ? { left: false } : {}), ...(sides.includes("right") ? { right: false } : {}) }));
  };
  const applyJoint = (side, index) => invoke("joint", { side, joint: index, target_deg: drafts[side][index], speed_deg_s: speed, gain_scale: gain }, `${side === "left" ? "왼팔" : "오른팔"} ${(openarm.arms?.[side]?.motors || [])[index]?.label} 적용`);
  const applyJointLimits = (side, index, lowerDeg, upperDeg, reset = false) => invoke(
    "limits",
    { side, joint: index, lower_deg: lowerDeg, upper_deg: upperDeg, reset },
    `${side === "left" ? "왼팔" : "오른팔"} ${(openarm.arms?.[side]?.motors || [])[index]?.label} ${reset ? "원본 범위 복원" : "운용 범위 저장"}`,
  );
  const enable = (side, enabled) => invoke("enable", { side, enabled }, `${side === "both" ? "양팔" : side === "left" ? "왼팔" : "오른팔"} 모터 ${enabled ? "활성화" : "비활성화"}`);
  const copyPose = (from, to) => {
    setDrafts((current) => ({ ...current, [to]: [...current[from]] }));
    setDirty((current) => ({ ...current, [to]: true }));
    notifyResult(`${from === "left" ? "왼팔" : "오른팔"} 편집값을 ${to === "left" ? "왼팔" : "오른팔"}에 복사했습니다`, true);
  };
  const savePose = () => {
    const name = poseName.trim();
    if (!name) return notifyResult("저장할 자세 이름을 입력하세요", false);
    const next = { ...savedPoses, [name]: { left: drafts.left, right: drafts.right, saved_at: new Date().toISOString() } };
    setSavedPoses(next);
    setSelectedPose(name);
    window.localStorage.setItem("romo-b-openarm-poses", JSON.stringify(next));
    notifyResult(`브라우저에 '${name}' 자세를 저장했습니다`, true);
  };
  const loadPose = () => {
    const pose = savedPoses[selectedPose];
    if (!pose) return notifyResult("불러올 저장 자세를 선택하세요", false);
    setDrafts({ left: [...pose.left], right: [...pose.right] });
    setDirty({ left: true, right: true });
    notifyResult(`'${selectedPose}' 자세를 편집값으로 불러왔습니다`, true);
  };
  const deletePose = () => {
    if (!selectedPose) return notifyResult("삭제할 저장 자세를 선택하세요", false);
    const next = { ...savedPoses };
    delete next[selectedPose];
    setSavedPoses(next);
    window.localStorage.setItem("romo-b-openarm-poses", JSON.stringify(next));
    notifyResult(`'${selectedPose}' 자세를 삭제했습니다`, true);
    setSelectedPose("");
  };
  const exportPoses = () => {
    const archive = {
      schema_version: 1,
      type: "romo_b_openarm_poses",
      exported_at: new Date().toISOString(),
      poses: savedPoses,
    };
    const blob = new Blob([`${JSON.stringify(archive, null, 2)}\n`], { type: "application/json" });
    const url = window.URL.createObjectURL(blob);
    const link = document.createElement("a");
    link.href = url;
    link.download = `romo-b-openarm-poses-${new Date().toISOString().slice(0, 10)}.json`;
    document.body.appendChild(link);
    link.click();
    link.remove();
    window.setTimeout(() => window.URL.revokeObjectURL(url), 0);
    notifyResult(`OpenArm 자세 ${Object.keys(savedPoses).length}개를 JSON으로 내보냈습니다`, true);
  };
  const importPoses = async (event) => {
    const file = event.target.files?.[0];
    event.target.value = "";
    if (!file) return;
    if (file.size > 1024 * 1024) return notifyResult("자세 JSON은 1 MB 이하여야 합니다", false);
    try {
      const imported = normalizeOpenArmPoseArchive(JSON.parse(await file.text()));
      const next = { ...savedPoses, ...imported };
      setSavedPoses(next);
      window.localStorage.setItem("romo-b-openarm-poses", JSON.stringify(next));
      notifyResult(`OpenArm 자세 ${Object.keys(imported).length}개를 가져왔습니다`, true);
    } catch (error) {
      notifyResult(error.message || "자세 JSON을 가져오지 못했습니다", false);
    }
  };
  const calibrationPayload = (includeConfirmation = false) => {
    const payload = { side: calibrationSide };
    if (calibrationJoint !== "all") payload.joint = Number(calibrationJoint);
    if (includeConfirmation) payload.confirmation = calibrationText;
    return payload;
  };
  const calibrationSelectionKey = `${calibrationSide === "both" ? "left,right" : calibrationSide}:${calibrationJoint === "all" ? "0,1,2,3,4,5,6,7" : calibrationJoint}`;
  const selectedCalibrationEnabled = calibrationSide === "both" ? openarm.any_enabled : openarm.arms?.[calibrationSide]?.enabled;
  const checkCalibration = async () => {
    if (demo) {
      const report = {
        ...DEMO_OPENARM.calibration,
        status: "ready",
        message: "영점 저장 준비가 완료되었습니다",
        ready: true,
        verified: false,
        selection_key: calibrationSelectionKey,
        target_sides: calibrationSide === "both" ? ["left", "right"] : [calibrationSide],
        target_joints: calibrationJoint === "all" ? OPENARM_LIMITS.map(([label]) => label) : [OPENARM_LIMITS[Number(calibrationJoint)][0]],
        preflight: [],
        results: [],
      };
      setCalibrationReport(report);
      notifyResult("데모: 영점 저장 준비 점검을 통과했습니다", true);
      return report;
    }
    const result = await invoke("calibration-check", calibrationPayload(), "영점 저장 준비 점검");
    if (result?.calibration) setCalibrationReport(result.calibration);
    return result;
  };
  const verifyCalibration = async () => {
    if (demo) {
      const report = { ...calibrationReport, status: "verified", message: "선택한 관절의 0점 검증을 통과했습니다", ready: false, verified: true, verified_at: new Date().toLocaleString("ko-KR") };
      setCalibrationReport(report);
      notifyResult("데모: 선택한 관절의 0점 검증을 통과했습니다", true);
      return report;
    }
    const result = await invoke("verify-zero", calibrationPayload(), "영점 사후 검증");
    if (result?.calibration) setCalibrationReport(result.calibration);
    return result;
  };
  const calibrate = async () => {
    if (demo) {
      const report = { ...calibrationReport, status: "awaiting_verification", message: "영점 저장 완료 · 새 피드백으로 0점을 검증하세요", ready: false, verified: false, written_at: new Date().toLocaleString("ko-KR") };
      setCalibrationReport(report);
      notifyResult("데모: 공식 Disable → Set Zero → Disable 프레임을 전송했습니다", true);
      window.setTimeout(verifyCalibration, 250);
      return report;
    }
    const result = await invoke("calibrate-zero", calibrationPayload(true), "공식 3단계 영점 저장");
    if (!result) return null;
    if (result.calibration) setCalibrationReport(result.calibration);
    await invoke("refresh", { side: calibrationSide }, "영점 저장 후 모터 피드백 갱신");
    await new Promise((resolve) => window.setTimeout(resolve, 400));
    return verifyCalibration();
  };
  const startAutomaticCalibration = async () => {
    const running = demo ? demoAutomaticRunning : automaticTask.running;
    if (running) return;
    if (demo) {
      setDemoAutomaticRunning(true);
      setAutomaticLog(`[데모] ${automaticSide === "left" ? "왼팔" : "오른팔"} OpenArm-v1 자동 캘리브레이션 시작\n[데모] 기계 한계 검색과 영점 저장 출력이 여기에 실시간 표시됩니다.`);
      notifyResult("데모: OpenArm 원본 자동 캘리브레이션을 시작했습니다", true);
      return;
    }
    try {
      const result = await postJson("/api/operations/openarm_auto_calibration/start", {
        side: automaticSide,
        interface: interfaces[automaticSide],
        confirmation: automaticText,
      });
      notifyResult(result.message, result.accepted !== false);
    } catch (error) {
      notifyResult(error.message, false);
    }
  };
  const stopAutomaticCalibration = async () => {
    if (demo) {
      setDemoAutomaticRunning(false);
      setAutomaticLog((current) => `${current}\n[데모] SIGINT 요청 → 모터 disable 후 종료`);
      notifyResult("데모: 자동 캘리브레이션 종료를 요청했습니다", true);
      return;
    }
    try {
      const result = await postJson("/api/operations/openarm_auto_calibration/stop", {});
      notifyResult(result.message, true);
    } catch (error) {
      notifyResult(error.message, false);
    }
  };
  const calibrationReadyForWrite = calibrationReport?.ready
    && calibrationReport?.selection_key === calibrationSelectionKey;
  const calibrationDisabled = calibrationText !== "OPENARM ZERO"
    || !calibrationAcknowledged
    || !calibrationReadyForWrite
    || (!demo && (!openarm.connected || selectedCalibrationEnabled));
  const automaticRunning = demo ? demoAutomaticRunning : Boolean(automaticTask.running);
  const automaticInterface = openarm.interfaces?.[automaticSide] || {};
  const automaticInterfaceReady = demo || (automaticInterface.exists && automaticInterface.is_can && automaticInterface.up);
  const automaticStartDisabled = (!demo && (!automaticTool.available || !automaticInterfaceReady || openarm.connected || openarm.any_enabled))
    || automaticRunning
    || !automaticAcknowledged
    || automaticText !== "OPENARM AUTO CAL";
  const healthLabels = {
    DISCONNECTED: "CAN 연결 안 됨",
    STANDBY: "정상 · 모터 비활성",
    READY: "정상 · 제어 가능",
    WARNING: "일부 축 확인 필요",
    BLOCKED: "명령 차단",
  };
  const healthLabel = healthLabels[openarm.health_level] || "상태 확인 중";
  const maximumSpeed = Math.max(1, Number(openarm.maximum_target_speed_deg_s || 1718.87));
  const updateSpeed = (value) => setSpeed(Math.max(1, Math.min(maximumSpeed, Number(value) || 1)));

  return (
    <div className="openarm-layout" data-testid="openarm-page">
      <section className="panel openarm-connect-panel">
        <div className="panel-title">
          <div><span className="eyebrow">OPENARM-V1 · CLASSIC CAN 1 Mbps</span><h2>양팔 CAN 연결 및 전원 제어</h2></div>
          <span className={`mode-chip ${openarm.connected ? "armed" : ""}`}>{openarm.connected ? "CAN 연결됨" : "연결 대기"}</span>
        </div>
        <div className="openarm-connection-grid">
          {(["left", "right"]).map((side) => {
            const arm = openarm.arms?.[side];
            const iface = openarm.interfaces?.[side] || {};
            return (
              <div className="openarm-interface" key={side}>
                <div><StatusDot active={arm?.connected && iface.up} /><strong>{side === "left" ? "왼팔" : "오른팔"}</strong><span>{arm?.online_count || 0}/8 모터</span></div>
                <label><span>SocketCAN 인터페이스</span><input aria-label={`${side} CAN 인터페이스`} value={interfaces[side]} disabled={openarm.connected} onChange={(event) => setInterfaces((current) => ({ ...current, [side]: event.target.value }))} /></label>
                <dl><dt>링크</dt><dd>{iface.exists ? iface.up ? "UP" : "DOWN" : "없음"}</dd><dt>RX / TX</dt><dd>{arm?.rx_count || 0} / {arm?.tx_count || 0}</dd><dt>모터 출력</dt><dd>{arm?.enabled ? "ENABLE" : "DISABLE"}</dd></dl>
              </div>
            );
          })}
        </div>
        <div className="openarm-connect-actions">
          <button data-testid="openarm-connect" className="service-button primary" disabled={openarm.connected} onClick={() => invoke("connect", { left_interface: interfaces.left, right_interface: interfaces.right }, "양팔 CAN 연결")}><Link weight="bold" />CAN 연결</button>
          <button className="service-button danger" disabled={!openarm.connected} onClick={() => invoke("disconnect", {}, "모터 비활성화 후 CAN 연결 해제")}><LinkBreak weight="bold" />CAN 연결 해제</button>
          <button className="service-button" disabled={!openarm.connected} onClick={() => invoke("refresh", { side: "both" }, "16개 모터 상태 갱신")}><ArrowsClockwise weight="bold" />전체 상태 갱신</button>
          <button className="service-button" disabled={!openarm.connected || !openarm.arms?.left?.all_online || !openarm.arms?.right?.all_online || openarm.arms?.left?.fault_count > 0 || openarm.arms?.right?.fault_count > 0 || openarm.all_enabled} onClick={() => enable("both", true)}><LockKeyOpen weight="bold" />양팔 모터 활성화</button>
          <button className="service-button danger" disabled={!openarm.connected || !openarm.any_enabled} onClick={() => enable("both", false)}><LockKey weight="bold" />양팔 모터 비활성화</button>
          <button className="service-button" disabled={!openarm.connected} onClick={() => invoke("clear-errors", { side: "both" }, "양팔 모터 오류 해제")}><Wrench weight="bold" />오류 해제</button>
        </div>
        <p className="openarm-safety-note"><ShieldCheck weight="fill" />CAN 연결만으로 모터가 활성화되거나 움직이지 않습니다. 활성화 버튼은 16축 피드백을 확인한 뒤 현재 자세를 유지하며 시작합니다.</p>
      </section>

      <section className="panel openarm-health-panel">
        <div className="panel-title compact-title">
          <div><span className="eyebrow">실시간 CAN 피드백 · 명령 허용 상태</span><h2>양팔 연결·제어 상태</h2></div>
          <span className={`mode-chip ${openarm.health_level === "READY" ? "armed" : openarm.health_level === "BLOCKED" || openarm.health_level === "WARNING" ? "fault" : ""}`}>{healthLabel}</span>
        </div>
        <div className="openarm-health-grid">
          {(["left", "right"]).map((side) => {
            const arm = openarm.arms?.[side] || {};
            return (
              <article className={`openarm-health-card ${arm.control_ready ? "ready" : arm.command_interlocked || arm.fault_count ? "blocked" : ""}`} key={side}>
                <div className="openarm-health-head"><span><StatusDot active={arm.all_online} danger={arm.command_interlocked || arm.fault_count > 0} /><strong>{side === "left" ? "왼팔" : "오른팔"}</strong></span><em>{arm.control_ready ? "정상 · 제어 중" : arm.command_interlocked ? "명령 차단" : arm.enabled ? "피드백 확인 중" : arm.all_online ? "정상 · 비활성" : "피드백 대기"}</em></div>
                <dl>
                  <div><dt>피드백</dt><dd>{arm.online_count || 0}/8 · stale {arm.stale_count ?? 8}</dd></div>
                  <div><dt>최장 지연</dt><dd>{arm.oldest_feedback_age_sec === null || arm.oldest_feedback_age_sec === undefined ? "—" : `${format(Number(arm.oldest_feedback_age_sec) * 1000, 0)} ms`}</dd></div>
                  <div><dt>Motor fault</dt><dd>{arm.fault_count || 0}축</dd></div>
                  <div><dt>최고 온도</dt><dd>{arm.max_temp_c === null || arm.max_temp_c === undefined ? "—" : `${format(arm.max_temp_c, 0)}°C`}</dd></div>
                  <div><dt>CAN error</dt><dd>{arm.can_error_count || 0}건</dd></div>
                  <div><dt>명령 상태</dt><dd>{arm.command_interlocked ? "차단됨" : arm.enabled ? "제어 프레임 전송" : "모터 비활성"}</dd></div>
                </dl>
                {arm.interlock_reason && <p><Warning weight="fill" />{arm.interlock_reason}</p>}
              </article>
            );
          })}
        </div>
        {(openarm.issues || []).length > 0 && <div className="openarm-issue-list">{openarm.issues.map((issue) => <span key={issue}><Warning weight="fill" />{issue}</span>)}</div>}
        <p className="openarm-health-note">피드백이 끊기거나 motor fault가 확인되면 해당 팔의 새 자세 명령과 주기 전송을 차단합니다. 상태가 정상으로 돌아온 뒤 <b>현재 자세 유지</b>를 눌러야 명령이 다시 시작됩니다.</p>
      </section>

      <section className="panel openarm-settings-panel">
        <div className="panel-title compact-title"><div><span className="eyebrow">동작 설정</span><h2>100 Hz S-curve 속도와 MIT gain</h2></div><SlidersHorizontal size={25} weight="duotone" /></div>
        <div className="openarm-setting-controls">
          <label className="openarm-speed-control"><span>관절 최대 속도 <strong>{format(speed, 0)}°/s</strong></span><div><input aria-label="OpenArm 최대 속도 슬라이더" type="range" min="1" max={maximumSpeed} step="1" value={speed} onChange={(event) => updateSpeed(event.target.value)} /><input aria-label="OpenArm 최대 속도" type="number" min="1" max={maximumSpeed} step="1" value={speed} onChange={(event) => updateSpeed(event.target.value)} /></div><small>각 축은 원본 속도 상한으로 다시 제한됩니다. J3/J4 312 · J1/J2 960 · J5~J7 1200 · gripper 1719°/s</small></label>
          <label><span>gain 배율 <strong>{format(gain, 2)}×</strong></span><input type="range" min="0.05" max={openarm.maximum_gain_scale || 1.5} step="0.05" value={gain} onChange={(event) => setGain(Number(event.target.value))} /></label>
        </div>
        <div className="openarm-facts"><span>제어 주기 <b>{format(openarm.command_rate_hz, 0)} Hz</b></span><span>궤적 <b>{openarm.trajectory_profile === "quintic_s_curve" ? "5차 S-curve" : openarm.trajectory_profile || "확인 중"}</b></span><span>프로토콜 <b>{openarm.library_version}</b></span><span>마지막 영점 <b>{openarm.last_calibration || "기록 없음"}</b></span><span>전체 상태 <b>{healthLabel}</b></span><span>확인 항목 <b>{(openarm.issues || []).length}건</b></span></div>
        {openarm.last_error && <div className="openarm-error"><Warning weight="fill" />{openarm.last_error}</div>}
      </section>

      {(["left", "right"]).map((side) => {
        const arm = openarm.arms?.[side] || { motors: [] };
        const disabled = !demo && !arm.control_ready;
        const holdDisabled = !demo && (!arm.connected || !arm.enabled || !arm.all_online || arm.fault_count > 0);
        return (
          <section className="panel openarm-arm-panel" key={side} data-testid={`openarm-${side}-arm`}>
            <div className="panel-title">
              <div><span className="eyebrow">{side === "left" ? "LEFT ARM · CAN1" : "RIGHT ARM · CAN0"}</span><h2>{side === "left" ? "왼팔 8축 제어" : "오른팔 8축 제어"}</h2></div>
              <span className={`mode-chip ${arm.control_ready ? "armed" : arm.command_interlocked || arm.fault_count ? "fault" : ""}`}>{arm.command_interlocked ? "명령 차단" : arm.enabled ? arm.control_ready ? "모터 활성" : "상태 확인" : arm.all_online ? "피드백 정상" : `${arm.online_count || 0}/8 대기`}</span>
            </div>
            <div className="openarm-arm-toolbar">
              <button disabled={!arm.connected || !arm.all_online || arm.fault_count > 0 || arm.enabled} onClick={() => enable(side, true)}><Power weight="bold" />활성화</button>
              <button disabled={!arm.connected || !arm.enabled} onClick={() => enable(side, false)}><StopCircle weight="bold" />비활성화</button>
              <button disabled={holdDisabled} onClick={() => invoke("hold", { side }, `${side === "left" ? "왼팔" : "오른팔"} 현재 자세 유지`)}><HandPalm weight="bold" />{arm.command_interlocked ? "인터록 해제 + 자세 유지" : "현재 자세 유지"}</button>
              <button disabled={disabled} onClick={() => applyPose(side)}><Play weight="fill" />8축 편집값 적용</button>
              <button disabled={!arm.all_online} onClick={() => syncFeedback(side)}><ArrowCounterClockwise weight="bold" />피드백 불러오기</button>
            </div>
            <div className="openarm-joints-grid">
              {(arm.motors || []).map((motor, index) => <OpenArmJointControl key={motor.key} side={side} index={index} motor={motor} value={drafts[side]?.[index] ?? 0} disabled={disabled} onChange={(value) => setJointDraft(side, index, value)} onApply={() => applyJoint(side, index)} onLimitsApply={(lowerDeg, upperDeg, reset) => applyJointLimits(side, index, lowerDeg, upperDeg, reset)} />)}
            </div>
          </section>
        );
      })}

      <section className="panel openarm-pose-panel">
        <div className="panel-title"><div><span className="eyebrow">양팔 자세 작업</span><h2>동시 적용 · 복사 · 브라우저 저장</h2></div><FloppyDisk size={27} weight="duotone" /></div>
        <div className="openarm-pose-actions">
          <button className="service-button primary" disabled={!demo && !openarm.all_enabled} onClick={() => applyPose("both")}><Play weight="fill" />양팔 동시 적용</button>
          <button className="service-button" onClick={() => syncFeedback("both")}><ArrowCounterClockwise weight="bold" />양팔 피드백 불러오기</button>
          <button className="service-button" onClick={() => copyPose("left", "right")}><ArrowRight weight="bold" />왼팔 → 오른팔 복사</button>
          <button className="service-button" onClick={() => copyPose("right", "left")}><ArrowLeft weight="bold" />오른팔 → 왼팔 복사</button>
          <button className="service-button" onClick={() => { setDrafts({ left: Array(8).fill(0), right: Array(8).fill(0) }); setDirty({ left: true, right: true }); }}><Crosshair weight="bold" />편집값 모두 0°</button>
        </div>
        <div className="openarm-preset-row">
          <label><span>자세 이름</span><input value={poseName} onChange={(event) => setPoseName(event.target.value)} /></label>
          <button onClick={savePose}><FloppyDisk weight="bold" />현재 편집값 저장</button>
          <label><span>저장된 자세</span><select value={selectedPose} onChange={(event) => setSelectedPose(event.target.value)}><option value="">선택</option>{Object.keys(savedPoses).sort().map((name) => <option key={name} value={name}>{name}</option>)}</select></label>
          <button disabled={!selectedPose} onClick={loadPose}><Play weight="bold" />불러오기</button>
          <button className="danger" disabled={!selectedPose} onClick={deletePose}><Trash weight="bold" />삭제</button>
        </div>
        <div className="openarm-preset-tools">
          <div><strong>{Object.keys(savedPoses).length}개</strong><span>이 브라우저에 저장된 자세</span></div>
          <button disabled={!Object.keys(savedPoses).length} onClick={exportPoses}><ArrowDown weight="bold" />자세 JSON 내보내기</button>
          <button onClick={() => poseImportRef.current?.click()}><ArrowUp weight="bold" />자세 JSON 가져오기</button>
          <input ref={poseImportRef} type="file" accept="application/json,.json" hidden aria-label="OpenArm 자세 JSON 파일" onChange={importPoses} />
          <p>다른 노트북이나 연구원에게 전달할 때 JSON 파일을 사용하세요. 같은 이름을 가져오면 기존 자세를 덮어씁니다.</p>
        </div>
      </section>

      <section className="panel openarm-calibration-panel">
        <div className="panel-title"><div><span className="eyebrow">단계형 수동 영점 캘리브레이션</span><h2>현재 기구 자세를 0°로 저장하고 피드백 검증</h2></div><Crosshair size={27} weight="duotone" /></div>
        <div className="calibration-steps" aria-label="수동 캘리브레이션 단계">
          <span className={calibrationReport?.status === "ready" ? "active" : ""}><b>1</b>정지·피드백 준비점검</span>
          <span className={calibrationReport?.status === "awaiting_verification" ? "active" : ""}><b>2</b>Disable → Zero → Disable</span>
          <span className={calibrationReport?.status === "verified" ? "passed" : calibrationReport?.status === "failed" ? "failed" : ""}><b>3</b>새 피드백 0점 확인</span>
        </div>
        <div className="openarm-calibration-form">
          <label><span>대상 팔</span><select value={calibrationSide} onChange={(event) => setCalibrationSide(event.target.value)}><option value="both">양팔</option><option value="left">왼팔</option><option value="right">오른팔</option></select></label>
          <label><span>대상 관절</span><select value={calibrationJoint} onChange={(event) => setCalibrationJoint(event.target.value)}><option value="all">전체 8축</option>{OPENARM_LIMITS.map(([label], index) => <option key={label} value={index}>{label}</option>)}</select></label>
          <label className="calibration-confirm"><span>확인 문구: OPENARM ZERO</span><input value={calibrationText} placeholder="OPENARM ZERO" onChange={(event) => setCalibrationText(event.target.value)} /></label>
          <button onClick={checkCalibration}><ListChecks weight="bold" />1 준비 상태 확인</button>
        </div>
        <label className="calibration-acknowledgement"><input type="checkbox" checked={calibrationAcknowledged} onChange={(event) => setCalibrationAcknowledged(event.target.checked)} /><span>선택한 팔을 정확한 0° 자세로 직접 지지했고, 모터가 비활성·정지 상태임을 확인했습니다.</span></label>
        <div className="calibration-actions">
          <button className="danger" disabled={calibrationDisabled} onClick={calibrate}><Warning weight="fill" />2 영점 저장 + 자동 검증</button>
          <button disabled={!demo && (!openarm.connected || selectedCalibrationEnabled)} onClick={verifyCalibration}><CheckCircle weight="bold" />3 다시 검증</button>
          <span className={`calibration-status ${calibrationReport?.status || "idle"}`}><StatusDot active={calibrationReport?.status === "ready" || calibrationReport?.status === "verified"} danger={calibrationReport?.status === "blocked" || calibrationReport?.status === "failed"} /><b>{calibrationReport?.message || "준비 점검 전입니다"}</b></span>
        </div>
        {(calibrationReport?.preflight?.length > 0 || calibrationReport?.results?.length > 0) && (
          <div className="calibration-results">
            <div className="calibration-result-head"><span>팔 / 축</span><span>현재각</span><span>속도</span><span>피드백</span><span>Fault</span><span>판정</span></div>
            {(calibrationReport.results?.length ? calibrationReport.results : calibrationReport.preflight).map((item) => (
              <div className="calibration-result-row" key={`${item.side}-${item.joint}`}>
                <strong>{item.side === "left" ? "왼팔" : "오른팔"} {item.label}</strong>
                <span>{item.position_deg === null ? "—" : `${format(item.position_deg, 2)}°`}</span>
                <span>{item.velocity_rad_s === null ? "—" : `${format(item.velocity_rad_s, 3)} rad/s`}</span>
                <span>{item.online ? `${format(item.age_sec, 3)}초` : "오프라인"}</span>
                <span>{item.fault_code || 0}</span>
                <em className={item.passed === true ? "pass" : item.passed === false ? "fail" : item.online ? "ready" : "fail"}>{item.passed === true ? "통과" : item.passed === false ? item.reason || "실패" : item.online ? "확인됨" : "대기"}</em>
              </div>
            ))}
          </div>
        )}
        <p className="openarm-calibration-note"><Warning weight="fill" />이 수동 방식은 사람이 잡아 둔 현재 자세를 모터 0°로 영구 저장합니다. 준비점검을 다시 하면 저장 버튼이 잠기며, 저장 뒤에는 반드시 새 피드백의 각도·속도·Fault를 검증합니다.</p>
      </section>

      <section className="panel openarm-auto-calibration-panel" data-testid="openarm-auto-calibration">
        <div className="panel-title"><div><span className="eyebrow">OPENARM 원본 코드 · ROBOT VERSION V1</span><h2>기계 한계 자동 캘리브레이션</h2></div><span className={`mode-chip ${automaticTool.available ? "armed" : "fault"}`}>{automaticTool.available ? "도구 준비됨" : "도구 미준비"}</span></div>
        <div className="automatic-calibration-warning">
          <Warning size={24} weight="fill" />
          <div><strong>선택한 한쪽 팔이 여러 기계 한계까지 자동으로 움직입니다.</strong><span>팔을 지지하고 작업 반경을 완전히 비우세요. 웹 CAN 연결을 해제한 뒤 원본 <code>openarm-can-zero-position-calibration</code>을 독립 실행하며, 종료 요청 시 SIGINT 후 원본 정리 코드가 전체 모터를 비활성화합니다.</span></div>
        </div>
        <div className="automatic-calibration-grid">
          <label><span>자동 보정할 팔</span><select value={automaticSide} disabled={automaticRunning} onChange={(event) => setAutomaticSide(event.target.value)}><option value="left">왼팔</option><option value="right">오른팔</option></select></label>
          <label><span>SocketCAN 인터페이스</span><input value={interfaces[automaticSide]} disabled readOnly /></label>
          <label><span>고정된 로봇 버전</span><input value={`OpenArm ${automaticTool.version || "v1"}`} disabled readOnly /></label>
          <label><span>확인 문구: OPENARM AUTO CAL</span><input value={automaticText} disabled={automaticRunning} placeholder="OPENARM AUTO CAL" onChange={(event) => setAutomaticText(event.target.value)} /></label>
        </div>
        <label className="calibration-acknowledgement automatic"><input type="checkbox" checked={automaticAcknowledged} disabled={automaticRunning} onChange={(event) => setAutomaticAcknowledged(event.target.checked)} /><span>팔을 사람이 지지하고 주변을 비웠으며, 선택한 CAN 배정과 OpenArm-v1 하드웨어를 확인했습니다.</span></label>
        <div className="automatic-calibration-actions">
          <button className="danger" disabled={automaticStartDisabled} onClick={startAutomaticCalibration}><PlayCircle weight="fill" />원본 자동 캘리브레이션 시작</button>
          <button disabled={!automaticRunning} onClick={stopAutomaticCalibration}><StopCircle weight="fill" />자동 보정 종료 요청</button>
          <div><span>실행 상태</span><strong>{automaticRunning ? `실행 중 · PID ${automaticTask.pid || (demo ? "DEMO" : "확인 중")}` : automaticTask.exit_code === 0 ? "최근 작업 완료" : automaticTask.exit_code ? `최근 종료 코드 ${automaticTask.exit_code}` : "대기"}</strong></div>
          <div><span>도구 상태</span><strong>{automaticTool.reason}</strong></div>
        </div>
        {openarm.connected && !demo && <p className="automatic-calibration-blocked"><LinkBreak weight="bold" />현재 웹 CAN이 연결되어 있습니다. 위의 <b>CAN 연결 해제</b>를 먼저 누르세요.</p>}
        {!openarm.connected && !automaticInterfaceReady && !demo && <p className="automatic-calibration-blocked"><Warning weight="bold" /><b>{interfaces[automaticSide]}</b> SocketCAN 인터페이스가 없거나 UP 상태가 아닙니다. USB-CAN 연결과 인터페이스 배정을 확인하세요.</p>}
        <div className="automatic-log-meta"><span>{automaticTask.log_path || "자동 캘리브레이션 로그 경로 대기"}</span><em>1초마다 갱신</em></div>
        <pre className="automatic-calibration-log">{automaticLog}</pre>
      </section>

      <section className="panel openarm-log-panel">
        <div className="panel-title compact-title"><div><span className="eyebrow">CAN 이벤트</span><h2>OpenArm 최근 로그</h2></div><Pulse size={24} weight="duotone" /></div>
        <pre className="openarm-log">{(openarm.log || []).length ? openarm.log.join("\n") : "CAN 작업 기록이 없습니다."}</pre>
      </section>
    </div>
  );
}

function DiagnosticsView({ state }) {
  return (
    <div className="diagnostics-layout">
      <section className="health-grid">
        <HealthCard icon={Broadcast} title="PCU 시리얼 피드백" health={state.health.platform} expected="20 Hz" />
        <HealthCard icon={Robot} title="휠 오도메트리" health={state.health.odometry} expected="20 Hz" />
        <HealthCard icon={Crosshair} title="LiDAR 위치추정" health={state.health.localization} expected="10 Hz" />
        <HealthCard icon={WifiHigh} title="Mid-360 원본 포인트" health={state.health.lidar_raw} expected="10 Hz" />
        <HealthCard icon={WifiHigh} title="필터링 포인트" health={state.health.lidar_filtered} expected="10 Hz" />
        <HealthCard icon={Pulse} title="Mid-360 IMU" health={state.health.imu} expected="100 Hz" />
        <HealthCard icon={Path} title="최종 안전 명령" health={state.health.cmd_safe} expected="20 Hz" />
      </section>
      <section className="panel diagnostic-list">
        <div className="panel-title"><div><span className="eyebrow">ROS 진단</span><h2>{state.diagnostics.summary}</h2></div></div>
        {state.diagnostics.items.length ? state.diagnostics.items.map((item) => (
          <div className="diagnostic-row" key={item.name}>
            <StatusDot active={item.level === 0} danger={item.level >= 2} />
            <div><strong>{item.name}</strong><span>{item.message}</span>{item.hardware_id && <code>{item.hardware_id}</code>}<div className="diagnostic-values">{Object.entries(item.values || {}).map(([key, value]) => <span key={key}><b>{key}</b>{value}</span>)}</div></div>
            <em>{["정상", "주의", "오류", "오래됨"][item.level] || item.level}</em>
          </div>
        )) : <div className="empty-state"><Pulse size={36} weight="duotone" /><span>`/diagnostics` 토픽 대기 중</span></div>}
      </section>
    </div>
  );
}

export function App() {
  const demo = new URLSearchParams(window.location.search).get("demo") === "1";
  const [state, streamOnline] = useRobotState(demo);
  const [tab, setTab] = useState("unified");
  const [toast, setToast] = useState(null);
  const [clock, setClock] = useState(new Date());

  useEffect(() => {
    const timer = window.setInterval(() => setClock(new Date()), 1000);
    return () => window.clearInterval(timer);
  }, []);

  const notify = (message, success = true, isMessage = false) => {
    if (message === "arm") {
      if (demo) {
        setToast({ message: "데모: Arm 요청", success: true });
        return;
      }
      postJson("/api/arm", { armed: success })
        .then((result) => setToast({ message: result.message, success: true }))
        .catch((error) => setToast({ message: error.message, success: false }));
      return;
    }
    setToast({ message, success: isMessage ? success : Boolean(success) });
    window.setTimeout(() => setToast(null), 3500);
  };

  const programStop = () => {
    if (demo) return notify("데모: 차량 정지·수동 전환 및 OpenArm 양팔 비활성화 요청", true, true);
    postJson("/api/program-stop")
      .then((result) => notify(`차량 정지·수동 전환 및 OpenArm 비활성화를 요청했습니다 · ${result.openarm?.message || "OpenArm 상태 확인"}`, true, true))
      .catch((error) => notify(error.message, false, true));
  };

  const connected = state.platform.connected
    && Boolean(state.health?.platform?.online)
    && (streamOnline || demo);
  return (
    <div className="app-shell">
      <header className="app-header">
        <div className="brand-block">
          <div className="brand-mark"><Robot weight="duotone" /></div>
          <div><span>ROMO-B + OPENARM</span><strong>통합 로봇 운용 콘솔</strong></div>
        </div>
        <div className="header-status">
          <span className={`connection-pill ${connected ? "online" : "offline"}`}>
            {connected ? <WifiHigh weight="bold" /> : <WifiSlash weight="bold" />}
            {connected ? "/dev/romo_b_pcu 연결됨" : "PCU 연결 대기 중"}
          </span>
          <span className={`connection-pill openarm-connection-pill ${state.openarm?.connected ? "online" : "offline"}`}>
            {state.openarm?.connected ? <PlugsConnected weight="bold" /> : <LinkBreak weight="bold" />}
            {state.openarm?.connected ? `OpenArm ${Number(state.openarm?.arms?.left?.online_count || 0) + Number(state.openarm?.arms?.right?.online_count || 0)}/16` : "OpenArm CAN 대기"}
          </span>
          <div className="clock"><strong>{clock.toLocaleTimeString("ko-KR", { hour12: false })}</strong><span>{clock.toLocaleDateString("ko-KR")}</span></div>
          <button className="program-stop" title="차량 속도 0·PCU 수동 전환·OpenArm 양팔 비활성화를 함께 요청합니다" onClick={programStop}><Stop weight="fill" /><span>전체 정지</span></button>
        </div>
      </header>

      <nav className="tab-bar" aria-label="로봇 제어 화면">
        {TABS.map(({ id, label, icon: Icon }) => (
          <button key={id} className={tab === id ? "active" : ""} onClick={() => setTab(id)}><Icon weight={tab === id ? "fill" : "regular"} />{label}</button>
        ))}
      </nav>

      <main className="content">
        <div className="status-ribbon">
          <div><StatusDot active={connected} /><span>PCU 연결</span><strong>{connected ? "정상" : "끊김"}</strong></div>
          <div><StatusDot active={state.readiness?.bridge_armed} /><span>HLV 브리지</span><strong>{state.readiness?.bridge_armed ? "ARMED" : "수동"}</strong></div>
          <div><StatusDot active={connected && state.platform.auto_mode} /><span>PCU 피드백</span><strong>{!connected ? "확인 불가" : state.platform.auto_mode ? "AUTO" : "수동"}</strong></div>
          <div><StatusDot active={connected} danger={connected && state.platform.estop} /><span>물리 비상정지</span><strong>{!connected ? "확인 불가" : state.platform.estop ? "작동 중" : "해제"}</strong></div>
          <div><StatusDot active={state.health.lidar?.online} /><span>Mid-360</span><strong>{format(state.health.lidar?.rate_hz, 1)} Hz</strong></div>
          <div><StatusDot active={state.health.localization?.online} /><span>위치추정</span><strong>{state.health.localization?.online ? "추적 중" : "대기 중"}</strong></div>
          <div><StatusDot active={state.openarm?.connected} /><span>OpenArm CAN</span><strong>{state.openarm?.connected ? "연결됨" : "대기 중"}</strong></div>
          <div><StatusDot active={state.openarm?.all_control_ready} danger={(state.openarm?.arms?.left?.fault_count || 0) + (state.openarm?.arms?.right?.fault_count || 0) > 0} /><span>양팔 피드백</span><strong>{Number(state.openarm?.arms?.left?.online_count || 0) + Number(state.openarm?.arms?.right?.online_count || 0)}/16</strong></div>
        </div>
        {tab === "unified" && <UnifiedView state={state} onPost={notify} demo={demo} onNavigate={setTab} />}
        {tab === "main" && <MainView state={state} onPost={notify} demo={demo} />}
        {tab === "algorithm" && <AlgorithmView />}
        {tab === "navigation" && <NavigationView state={state} onPost={notify} demo={demo} />}
        {tab === "operations" && <OperationsView state={state} onPost={notify} demo={demo} />}
        {tab === "openarm" && <OpenArmView state={state} onPost={notify} demo={demo} />}
        {tab === "system" && <SystemView state={state} onPost={notify} demo={demo} />}
        {tab === "diagnostics" && <DiagnosticsView state={state} />}
      </main>

      <footer className="app-footer">
        <span>ROS 2 Humble · 115200 8N1 · UI {state.version}</span>
        <span><StatusDot active={streamOnline || demo} />{demo ? "데모 상태 정보" : streamOnline ? "실시간 상태 정보" : "상태 정보 재연결 중"}</span>
      </footer>
      {toast && <div className={`toast ${toast.success ? "success" : "failure"}`}>{toast.success ? <CheckCircle weight="fill" /> : <Warning weight="fill" />}<span>{toast.message}</span></div>}
    </div>
  );
}
