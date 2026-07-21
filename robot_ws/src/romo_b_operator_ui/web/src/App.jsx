import { useEffect, useMemo, useRef, useState } from "react";
import {
  ArrowCounterClockwise,
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
].map(([id, label, group, description, selection, primary]) => ({
  id, label, group, description, selection, primary, caution: "", running: id === "field_navigation", owned_by_ui: id === "field_navigation", pids: id === "field_navigation" ? [24831] : [], pid: id === "field_navigation" ? 24831 : null, elapsed_sec: id === "field_navigation" ? 128.4 : null, exit_code: null, log_path: id === "field_navigation" ? "/home/hyunseo/ROMO-B/data/local/logs/operator-field.log" : "", message: id === "field_navigation" ? "실행 중" : "실행 대기",
}));

const DEMO_STATE = {
  version: "0.3.0",
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
  graph: { node_count: 1, topic_count: 0, nodes: ["/romo_b_operator_ui"] },
  host: { hostname: "hyunseo-2204", load_1m: 0, memory_used_gb: 0, memory_total_gb: 0, uptime_hours: 0, gpu: { available: false } },
};

const TABS = [
  { id: "main", label: "메인 제어", icon: Gauge },
  { id: "algorithm", label: "플랫폼 제어 알고리즘", icon: Wrench },
  { id: "navigation", label: "자율주행", icon: MapPin },
  { id: "operations", label: "실행 관리", icon: PlayCircle },
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
        <div className={`readonly-control ${state.platform.estop ? "alert" : ""}`}>
          <ShieldCheck weight="bold" />
          <span>물리 비상정지</span>
          <strong>{state.platform.estop ? "작동 중" : "해제됨"}</strong>
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
            <input id="speed" type="range" min="0" max="0.5" step="0.01" value={speed} onChange={(event) => setSpeed(Number(event.target.value))} />
            <div className="range-labels"><span>0</span><span>전진·후진 최대 ±0.5</span></div>
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
        <div><StatusDot active={platform.connected} /><span>통신 상태</span><strong>{platform.connected ? "연결됨" : "끊김"}</strong></div>
        <div><StatusDot active={state.readiness?.bridge_armed} /><span>HLV 요청</span><strong>{state.readiness?.bridge_armed ? "ARMED" : "수동"}</strong></div>
        <div><StatusDot active={platform.auto_mode} /><span>PCU 피드백</span><strong>{platform.auto_mode ? "AUTO" : "수동"}</strong></div>
        <div><StatusDot active danger={platform.estop} /><span>비상정지</span><strong>{platform.estop ? "작동 중" : "해제"}</strong></div>
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
            <input type="range" min="-0.5" max="0.5" step="0.01" value={speed} onChange={(event) => setSpeed(Number(event.target.value))} />
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
      max_speed_mps: maxSpeed,
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
        </div>
        <div className="launch-options">
          <label><input type="checkbox" checked={useRviz} onChange={(event) => setUseRviz(event.target.checked)} /> RViz 열기</label>
          <label><input type="checkbox" checked={receiveOnly} onChange={(event) => setReceiveOnly(event.target.checked)} /> Autoware 수신 전용</label>
          <label><span>재생 배속</span><input type="number" min="0.1" max="4" step="0.1" value={replayRate} onChange={(event) => setReplayRate(Number(event.target.value))} />×</label>
          <label><span>최대 속도</span><input type="number" min="0.05" max="1.5" step="0.05" value={maxSpeed} onChange={(event) => setMaxSpeed(Number(event.target.value))} />m/s</label>
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
  const [tab, setTab] = useState("main");
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
    if (demo) return notify("데모: 속도 0 및 수동 전환 요청", true, true);
    postJson("/api/program-stop")
      .then(() => notify("속도 0 및 수동 전환을 요청했습니다", true, true))
      .catch((error) => notify(error.message, false, true));
  };

  const connected = state.platform.connected && (streamOnline || demo);
  return (
    <div className="app-shell">
      <header className="app-header">
        <div className="brand-block">
          <div className="brand-mark"><Robot weight="duotone" /></div>
          <div><span>ROMO-B</span><strong>통합 제어 콘솔</strong></div>
        </div>
        <div className="header-status">
          <span className={`connection-pill ${connected ? "online" : "offline"}`}>
            {connected ? <WifiHigh weight="bold" /> : <WifiSlash weight="bold" />}
            {connected ? "/dev/romo_b_pcu 연결됨" : "PCU 연결 대기 중"}
          </span>
          <div className="clock"><strong>{clock.toLocaleTimeString("ko-KR", { hour12: false })}</strong><span>{clock.toLocaleDateString("ko-KR")}</span></div>
          <button className="program-stop" title="속도 0 명령을 보내고 PCU 수동 전환을 요청합니다" onClick={programStop}><Stop weight="fill" /><span>정지 + 수동</span></button>
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
          <div><StatusDot active={state.platform.auto_mode} /><span>PCU 피드백</span><strong>{state.platform.auto_mode ? "AUTO" : "수동"}</strong></div>
          <div><StatusDot active danger={state.platform.estop} /><span>물리 비상정지</span><strong>{state.platform.estop ? "작동 중" : "해제"}</strong></div>
          <div><StatusDot active={state.health.lidar?.online} /><span>Mid-360</span><strong>{format(state.health.lidar?.rate_hz, 1)} Hz</strong></div>
          <div><StatusDot active={state.health.localization?.online} /><span>위치추정</span><strong>{state.health.localization?.online ? "추적 중" : "대기 중"}</strong></div>
        </div>
        {tab === "main" && <MainView state={state} onPost={notify} demo={demo} />}
        {tab === "algorithm" && <AlgorithmView />}
        {tab === "navigation" && <NavigationView state={state} onPost={notify} demo={demo} />}
        {tab === "operations" && <OperationsView state={state} onPost={notify} demo={demo} />}
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
