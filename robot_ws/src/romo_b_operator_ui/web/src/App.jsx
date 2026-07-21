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

const DEMO_STATE = {
  version: "0.2.0",
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
  diagnostics: { level: 3, summary: "Waiting for ROS 2 data", items: [] },
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
  graph: { node_count: 1, topic_count: 0, nodes: ["/romo_b_operator_ui"] },
  host: { hostname: "hyunseo-2204", load_1m: 0, memory_used_gb: 0, memory_total_gb: 0, uptime_hours: 0, gpu: { available: false } },
};

const TABS = [
  { id: "main", label: "Main", icon: Gauge },
  { id: "algorithm", label: "Platform control algorithm", icon: Wrench },
  { id: "navigation", label: "Navigation", icon: MapPin },
  { id: "system", label: "System control", icon: ListChecks },
  { id: "diagnostics", label: "Diagnostics", icon: Pulse },
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
  if (!response.ok) throw new Error(result.message || "Request failed");
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
    ? "CONTROL READY"
    : bridgeArmed && state.platform.auto_mode && state.platform.estop
      ? "E-STOP BLOCKED"
      : bridgeArmed
        ? "AUTO REQUESTING"
        : "DISARMED";
  const canDrive = demo || controlReady;
  const steerLimit = mode === "4wis" ? 18 : 22;
  const firstBlocker = state.readiness?.checks?.find((item) => !item.ok);

  useEffect(() => {
    setSteer((value) => Math.max(-steerLimit, Math.min(steerLimit, value)));
  }, [steerLimit]);

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
          <span className="eyebrow">Platform status command</span>
          <h2>Vehicle command</h2>
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
          <span>{bridgeArmed ? "HLV: request Manual" : "HLV: request Auto / Arm"}</span>
        </button>
        <div className={`readonly-control ${state.platform.estop ? "alert" : ""}`}>
          <ShieldCheck weight="bold" />
          <span>Physical E-stop</span>
          <strong>{state.platform.estop ? "ACTIVE" : "RELEASED"}</strong>
        </div>
      </div>

      <div className="field-group">
        <div className="field-label-row">
          <label>Steering mode</label>
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
              <label htmlFor="speed">Drive speed magnitude</label>
              <output>{format(speed, 2)} m/s</output>
            </div>
            <input id="speed" type="range" min="0" max="0.5" step="0.01" value={speed} onChange={(event) => setSpeed(Number(event.target.value))} />
            <div className="range-labels"><span>0</span><span>signed max ±0.5</span></div>
          </div>
          <div className="slider-field">
            <div className="field-label-row">
              <label htmlFor="steer">Center steering</label>
              <output className={Math.abs(steer) > 18 ? "warning-text" : ""}>{format(steer, 1)}°</output>
            </div>
            <input id="steer" type="range" min={-steerLimit} max={steerLimit} step="0.5" value={steer} onChange={(event) => setSteer(Number(event.target.value))} />
            <div className="range-labels"><span>Right −{steerLimit}°</span><span>ROS 0°</span><span>Left +{steerLimit}°</span></div>
          </div>
          <div className="drive-pad">
            <HoldButton
              disabled={!canDrive}
              payload={{ mode, speed_mps: speed, steer_deg: steer }}
              onSend={drive}
              className="drive-forward"
            >
              <ArrowUp weight="bold" />
              <span>Hold forward</span>
              <small>+{format(speed, 2)} m/s · {mode.toUpperCase()} · {format(steer, 1)}°</small>
            </HoldButton>
            <HoldButton
              disabled={!canDrive}
              payload={{ mode, speed_mps: -speed, steer_deg: steer }}
              onSend={drive}
              className="drive-reverse"
            >
              <ArrowDown weight="bold" />
              <span>Hold reverse</span>
              <small>−{format(speed, 2)} m/s · {mode.toUpperCase()} · {format(steer, 1)}°</small>
            </HoldButton>
            <button className="center-steer" onClick={() => setSteer(0)}>
              <Crosshair weight="bold" /> Center steering
            </button>
          </div>
        </>
      ) : (
        <div className="pivot-control">
          <div className="slider-field">
            <div className="field-label-row">
              <label htmlFor="pivot">Pivot rate</label>
              <output>{format(pivotRate, 2)} rad/s</output>
            </div>
            <input id="pivot" type="range" min="0.1" max="0.75" step="0.05" value={pivotRate} onChange={(event) => setPivotRate(Number(event.target.value))} />
          </div>
          <div className="pivot-buttons">
            <HoldButton disabled={!canDrive} payload={{ mode: "pivot", pivot_rate_radps: pivotRate }} onSend={drive}>
              <ArrowLeft weight="bold" /> Hold CCW
            </HoldButton>
            <HoldButton disabled={!canDrive} payload={{ mode: "pivot", pivot_rate_radps: -pivotRate }} onSend={drive}>
              Hold CW <ArrowRight weight="bold" />
            </HoldButton>
          </div>
        </div>
      )}

      {!canDrive && (
        <div className="control-hint"><HandPalm weight="fill" /> {firstBlocker ? `${firstBlocker.label}: ${firstBlocker.detail}` : "PCU Auto confirmation is required before motion."}</div>
      )}
    </section>
  );
}

function WheelCard({ name, speed, steer, steerEnabled }) {
  return (
    <div className="wheel-card">
      <span>{name}</span>
      <strong>{format(speed, 2)} <small>m/s</small></strong>
      <em>{steerEnabled ? `${format(steer, 1)}°` : "fixed in 2WIS"}</em>
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
          <span className="eyebrow">Platform status feedback</span>
          <h2>Live PCU feedback</h2>
        </div>
        <div className="alive-pair">
          <span>PCU <strong>{platform.pcu_alive}</strong></span>
          <span>HLV <strong>{platform.hlv_alive}</strong></span>
        </div>
      </div>
      <div className="feedback-flags">
        <div><StatusDot active={platform.connected} /><span>Communication</span><strong>{platform.connected ? "ONLINE" : "OFFLINE"}</strong></div>
        <div><StatusDot active={state.readiness?.bridge_armed} /><span>HLV request</span><strong>{state.readiness?.bridge_armed ? "ARMED" : "MANUAL"}</strong></div>
        <div><StatusDot active={platform.auto_mode} /><span>PCU feedback</span><strong>{platform.auto_mode ? "AUTO" : "MANUAL"}</strong></div>
        <div><StatusDot active danger={platform.estop} /><span>E-stop</span><strong>{platform.estop ? "ACTIVE" : "CLEAR"}</strong></div>
      </div>
      <div className="vehicle-feedback">
        <div className="wheel-column">
          <WheelCard name="FL" speed={platform.wheel_speed_mps[0]} steer={platform.wheel_steer_deg[0]} steerEnabled />
          <WheelCard name="RL" speed={platform.wheel_speed_mps[2]} steer={platform.wheel_steer_deg[2]} steerEnabled={allWheelSteering} />
        </div>
        <div className="vehicle-center">
          <span className="front-label">FRONT</span>
          <CarProfile size={94} weight="duotone" />
          <strong>ROMO-B</strong>
          <small>{platform.steer_mode_name}</small>
          <span className="rear-label">REAR</span>
        </div>
        <div className="wheel-column">
          <WheelCard name="FR" speed={platform.wheel_speed_mps[1]} steer={platform.wheel_steer_deg[1]} steerEnabled />
          <WheelCard name="RR" speed={platform.wheel_speed_mps[3]} steer={platform.wheel_steer_deg[3]} steerEnabled={allWheelSteering} />
        </div>
      </div>
      <div className="feedback-metrics">
        <Metric label="Safe command" value={format(state.command.safe_linear_mps, 2)} unit="m/s" accent />
        <Metric label="Wheel odometry" value={format(state.motion.wheel_odom_speed_mps, 2)} unit="m/s" />
        <Metric label="Yaw rate" value={format(state.motion.wheel_odom_yaw_rate_radps, 2)} unit="rad/s" />
      </div>
    </section>
  );
}

function AutoReadiness({ state }) {
  return (
    <section className="panel readiness-panel">
      <div className="panel-title compact-title">
        <div><span className="eyebrow">Auto transition truth table</span><h2>PCU Auto entry & control readiness</h2></div>
        <span className={`mode-chip ${state.readiness?.control_ready ? "armed" : ""}`}>
          {state.readiness?.control_ready ? "READY" : "CHECK REQUIRED"}
        </span>
      </div>
      <div className="readiness-grid">
        {(state.readiness?.checks || []).map((item) => (
          <div className={`readiness-item ${item.ok ? "pass" : "blocked"}`} key={item.key}>
            {item.ok ? <CheckCircle weight="fill" /> : <Warning weight="fill" />}
            <div><strong>{item.label}</strong><span>{item.detail}</span></div>
            <em>{item.ok ? "PASS" : "WAIT"}</em>
          </div>
        ))}
      </div>
      <div className="readiness-explainer">
        <strong>표시 구분</strong>
        <span>RC/본체 스위치는 PCU 조건을 만들고, 웹의 HLV Arm은 Auto 상승 에지를 요청합니다. 실제 주행 가능 여부는 PCU 피드백 AUTO와 브리지 ARMED가 모두 확인될 때만 READY로 표시됩니다.</span>
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
        <div><span className="eyebrow">Platform control algorithm</span><h2>{mode.toUpperCase()} kinematic preview</h2></div>
        <div className="geometry-chips"><span>L 0.323 m</span><span>W 0.390 m</span></div>
      </div>
      <div className="algorithm-layout">
        <div className="algorithm-inputs">
          <p>Signed 중심 속도와 조향각을 입력하면 매뉴얼 기하에 따라 네 바퀴의 목표 선속도와 조향각을 계산합니다.</p>
          <div className="segmented-control algorithm-mode">
            {[
              ["2wis", "2WIS"], ["4wis", "4WIS"], ["pivot", "Pivot"],
            ].map(([value, label]) => <button className={mode === value ? "active" : ""} onClick={() => setMode(value)} key={value}>{label}</button>)}
          </div>
          <div className="slider-field">
            <div className="field-label-row"><label>Center speed</label><output>{format(speed, 2)} m/s</output></div>
            <input type="range" min="-0.5" max="0.5" step="0.01" value={speed} onChange={(event) => setSpeed(Number(event.target.value))} />
          </div>
          <div className="slider-field">
            <div className="field-label-row"><label>Center steering</label><output>{format(steer, 1)}°</output></div>
            <input type="range" min={-steerLimit} max={steerLimit} step="0.5" value={steer} disabled={mode === "pivot"} onChange={(event) => setSteer(Number(event.target.value))} />
          </div>
          <div className="formula-card"><span>ROS angular.z</span><strong>{format(mode === "pivot" ? -speed / Math.hypot(0.323 / 2, 0.39 / 2) : speed * Math.tan((steer * Math.PI) / 180) / (mode === "4wis" ? 0.323 / 2 : 0.323), 3)} rad/s</strong><small>{mode === "4wis" ? "ω = v · tan(δ) / (L/2)" : mode === "pivot" ? "PCU positive speed = clockwise" : "ω = v · tan(δ) / L"}</small></div>
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
            <div className="prediction-center"><CarProfile size={126} weight="duotone" /><strong>FRONT</strong></div>
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
        <label key={key}><span>{key === "yaw_deg" ? "Yaw (deg)" : key === "x_m" ? "Map X (m)" : "Map Y (m)"}</span><input type="number" step={key === "yaw_deg" ? "1" : "0.1"} value={values[key]} onChange={(event) => setter({ ...values, [key]: Number(event.target.value) })} /></label>
      ))}
    </div>
  );
  const services = state.services;
  return (
    <div className="navigation-grid">
      <section className="panel route-panel">
        <div className="panel-title"><div><span className="eyebrow">Waypoint navigation</span><h2>Route operations</h2></div><Path size={30} weight="duotone" /></div>
        <div className="route-summary">
          <div><span>Waypoints</span><strong>{state.navigation.waypoint_count}</strong></div>
          <div><span>Plan poses</span><strong>{state.navigation.plan_points}</strong></div>
          <div><span>Plan length</span><strong>{format(state.navigation.plan_length_m, 1)} <small>m</small></strong></div>
        </div>
        <div className="service-grid">
          <ServiceButton icon={FloppyDisk} action="save" disabled={!demo && !services.waypoint_save} onAction={action}>Save RViz points</ServiceButton>
          <ServiceButton icon={ArrowCounterClockwise} action="reload" disabled={!demo && !services.waypoint_reload} onAction={action}>Reload YAML</ServiceButton>
          <ServiceButton icon={Trash} action="clear" disabled={!demo && !services.waypoint_clear} onAction={action}>Clear editor</ServiceButton>
          <ServiceButton icon={Play} action="execute" disabled={!demo && !services.waypoint_execute} onAction={action} tone="primary">Execute route</ServiceButton>
          <ServiceButton icon={Stop} action="cancel" disabled={!demo && !services.waypoint_cancel} onAction={action} tone="danger">Cancel route</ServiceButton>
        </div>
        <div className={`operation-result ${state.navigation.last_action_success ? "success" : "failure"}`}>
          {state.navigation.last_action_success ? <CheckCircle weight="fill" /> : <Warning weight="fill" />}
          <span>{state.navigation.last_action}</span>
        </div>
      </section>
      <section className="panel localization-panel">
        <div className="panel-title"><div><span className="eyebrow">Map localization</span><h2>Current map pose</h2></div><Crosshair size={30} weight="duotone" /></div>
        <div className="pose-card">
          <Metric label="Map X" value={format(state.localization.x_m, 2)} unit="m" />
          <Metric label="Map Y" value={format(state.localization.y_m, 2)} unit="m" />
          <Metric label="Heading" value={format(state.localization.yaw_deg, 1)} unit="deg" accent />
        </div>
        <div className="pose-card secondary-pose">
          <Metric label="XY std" value={format(state.localization.xy_std_m, 2)} unit="m" />
          <Metric label="Yaw std" value={format(state.localization.yaw_std_deg, 1)} unit="deg" />
          <Metric label="Goal state" value={state.navigation.goal_state} />
        </div>
        <div className="localization-note"><MapPin weight="fill" /><div><strong>2D Pose Estimate</strong><span>문·기둥·코너 가까이에서 실제 전방 방향으로 지정하세요. 현재 프로필은 클릭 위치 주변 15 m 밖의 정합을 거부합니다.</span></div></div>
      </section>
      <section className="panel pose-command-panel">
        <div className="panel-title"><div><span className="eyebrow">Browser-only navigation</span><h2>Initial pose & direct goal</h2></div><MapTrifold size={30} weight="duotone" /></div>
        <div className="pose-command-grid">
          <div className="pose-command-card">
            <div><strong>Set initial pose</strong><span>Publish `/initialpose` with localization covariance</span></div>
            {poseInputs(initialPose, setInitialPose)}
            <div className="inline-fields">
              <label><span>XY std (m)</span><input type="number" min="0.05" max="2" step="0.05" value={initialPose.xy_std_m} onChange={(event) => setInitialPose({ ...initialPose, xy_std_m: Number(event.target.value) })} /></label>
              <label><span>Yaw std (deg)</span><input type="number" min="2" max="45" step="1" value={initialPose.yaw_std_deg} onChange={(event) => setInitialPose({ ...initialPose, yaw_std_deg: Number(event.target.value) })} /></label>
            </div>
            <button className="service-button primary" type="button" onClick={() => sendPose("initial-pose", initialPose)}><Crosshair weight="bold" />Publish initial pose</button>
          </div>
          <div className="pose-command-card">
            <div><strong>Navigate to goal</strong><span>Send Nav2 `NavigateToPose` in the map frame</span></div>
            {poseInputs(goal, setGoal)}
            <div className="goal-actions">
              <button className="service-button primary" type="button" disabled={!demo && !services.navigate_to_pose} onClick={() => sendPose("goal", goal)}><Play weight="bold" />Send goal</button>
              <button className="service-button danger" type="button" onClick={cancelGoal}><Stop weight="bold" />Cancel goal</button>
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
      <div><span>{title}</span><strong>{online ? "ONLINE" : "WAITING"}</strong><small>{format(health?.rate_hz, 1)} Hz · expected {expected}</small></div>
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
    ["Mux", "selected", "/cmd_vel_selected"],
    ["Smoother", "smoothed", "/cmd_vel_smoothed"],
    ["Collision monitor", "safe", "/cmd_vel_safe"],
  ];
  const sensorRows = [
    ["Mid-360 raw", "lidar_raw", state.sensors?.lidar_raw],
    ["Mid-360 filtered", "lidar_filtered", state.sensors?.lidar_filtered],
    ["Livox IMU", "imu", state.sensors?.imu],
    ["Wheel odometry", "odometry", { frame_id: "odom", points: "pose + twist" }],
  ];
  return (
    <div className="system-layout">
      <section className="panel runtime-panel">
        <div className="panel-title"><div><span className="eyebrow">Process control</span><h2>Field navigation stack</h2></div><Power size={30} weight="duotone" /></div>
        <div className={`runtime-state ${state.runtime?.field_running ? "running" : "stopped"}`}>
          <StatusDot active={state.runtime?.field_running} />
          <div><span>Complete ROS 2 stack</span><strong>{state.runtime?.field_running ? "RUNNING" : "STOPPED"}</strong><small>{state.runtime?.field_running ? `PID ${state.runtime.field_pids?.join(", ") || "detecting"}` : "Ready to launch from this page"}</small></div>
        </div>
        <div className="runtime-actions">
          <button className="service-button primary" disabled={state.runtime?.field_running} onClick={() => runtimeAction("start")}><PlayCircle weight="bold" />Start navigation + LiDAR + RViz</button>
          <button className="service-button danger" disabled={!state.runtime?.field_running} onClick={() => runtimeAction("stop")}><StopCircle weight="bold" />Zero & stop field stack</button>
        </div>
        <div className="runtime-details"><span>Owner</span><strong>{state.runtime?.owned_by_ui ? "WEB UI" : state.runtime?.field_running ? "EXTERNAL TERMINAL" : "—"}</strong><span>Log</span><code>{state.runtime?.log_path || "created on next web launch"}</code></div>
      </section>

      <section className="panel pipeline-panel">
        <div className="panel-title"><div><span className="eyebrow">Command observability</span><h2>Velocity pipeline</h2></div><Path size={30} weight="duotone" /></div>
        <div className="pipeline-table">
          {stages.map(([label, key, topic], index) => {
            const command = state.commands?.[key] || {};
            const health = state.health?.[`cmd_${key}`];
            return <div className="pipeline-row" key={key}><span>{index + 1}</span><div><strong>{label}</strong><code>{topic}</code></div><em>{format(command.linear_mps, 3)} m/s</em><em>{format(command.angular_radps, 3)} rad/s</em><StatusDot active={health?.online} /></div>;
          })}
        </div>
      </section>

      <section className="panel sensor-panel">
        <div className="panel-title"><div><span className="eyebrow">Sensor inventory</span><h2>Live inputs</h2></div><Broadcast size={30} weight="duotone" /></div>
        <div className="sensor-table">
          {sensorRows.map(([label, key, data]) => <div className="sensor-row" key={key}><StatusDot active={state.health?.[key]?.online} /><div><strong>{label}</strong><span>{data?.frame_id || "frame unavailable"}</span></div><em>{format(state.health?.[key]?.rate_hz, 1)} Hz</em><code>{data?.points ?? (data?.angular_velocity_radps ? `${data.angular_velocity_radps.join(", ")} rad/s` : "—")}</code></div>)}
        </div>
      </section>

      <section className="panel host-panel">
        <div className="panel-title"><div><span className="eyebrow">Laptop resources</span><h2>{state.host?.hostname || "Host"}</h2></div><Gauge size={30} weight="duotone" /></div>
        <div className="host-metrics">
          <Metric label="Load (1m)" value={format(state.host?.load_1m, 2)} />
          <Metric label="Memory" value={format(state.host?.memory_used_gb, 1)} unit={`/ ${format(state.host?.memory_total_gb, 1)} GB`} />
          <Metric label="Uptime" value={format(state.host?.uptime_hours, 1)} unit="h" />
          <Metric label="GPU use" value={state.host?.gpu?.available ? format(state.host.gpu.utilization_percent, 0) : "N/A"} unit={state.host?.gpu?.available ? "%" : ""} accent={state.host?.gpu?.available} />
          <Metric label="GPU memory" value={state.host?.gpu?.available ? format(state.host.gpu.memory_used_mb, 0) : "N/A"} unit={state.host?.gpu?.available ? `/ ${format(state.host.gpu.memory_total_mb, 0)} MB` : ""} />
          <Metric label="GPU temp" value={state.host?.gpu?.available ? format(state.host.gpu.temperature_c, 0) : "N/A"} unit={state.host?.gpu?.available ? "°C" : ""} />
        </div>
        <div className="gpu-name">{state.host?.gpu?.available ? state.host.gpu.name : "NVIDIA telemetry unavailable"}</div>
      </section>

      <section className="panel graph-panel">
        <div className="panel-title"><div><span className="eyebrow">ROS graph</span><h2>{state.graph?.node_count || 0} nodes · {state.graph?.topic_count || 0} topics</h2></div><ListChecks size={30} weight="duotone" /></div>
        <div className="node-list">{(state.graph?.nodes || []).map((node) => <code key={node}>{node}</code>)}</div>
      </section>
    </div>
  );
}

function DiagnosticsView({ state }) {
  return (
    <div className="diagnostics-layout">
      <section className="health-grid">
        <HealthCard icon={Broadcast} title="PCU serial feedback" health={state.health.platform} expected="20 Hz" />
        <HealthCard icon={Robot} title="Wheel odometry" health={state.health.odometry} expected="20 Hz" />
        <HealthCard icon={Crosshair} title="LiDAR localization" health={state.health.localization} expected="10 Hz" />
        <HealthCard icon={WifiHigh} title="Mid-360 raw cloud" health={state.health.lidar_raw} expected="10 Hz" />
        <HealthCard icon={WifiHigh} title="Filtered cloud" health={state.health.lidar_filtered} expected="10 Hz" />
        <HealthCard icon={Pulse} title="Mid-360 IMU" health={state.health.imu} expected="100 Hz" />
        <HealthCard icon={Path} title="Safe command" health={state.health.cmd_safe} expected="20 Hz" />
      </section>
      <section className="panel diagnostic-list">
        <div className="panel-title"><div><span className="eyebrow">ROS diagnostics</span><h2>{state.diagnostics.summary}</h2></div></div>
        {state.diagnostics.items.length ? state.diagnostics.items.map((item) => (
          <div className="diagnostic-row" key={item.name}>
            <StatusDot active={item.level === 0} danger={item.level >= 2} />
            <div><strong>{item.name}</strong><span>{item.message}</span>{item.hardware_id && <code>{item.hardware_id}</code>}<div className="diagnostic-values">{Object.entries(item.values || {}).map(([key, value]) => <span key={key}><b>{key}</b>{value}</span>)}</div></div>
            <em>{["OK", "WARN", "ERROR", "STALE"][item.level] || item.level}</em>
          </div>
        )) : <div className="empty-state"><Pulse size={36} weight="duotone" /><span>Waiting for `/diagnostics`</span></div>}
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
        setToast({ message: "Demo: arm request", success: true });
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
    if (demo) return notify("Demo: zero command and Manual requested", true, true);
    postJson("/api/program-stop")
      .then(() => notify("Zero command and Manual requested", true, true))
      .catch((error) => notify(error.message, false, true));
  };

  const connected = state.platform.connected && (streamOnline || demo);
  return (
    <div className="app-shell">
      <header className="app-header">
        <div className="brand-block">
          <div className="brand-mark"><Robot weight="duotone" /></div>
          <div><span>ROMO-B</span><strong>Operator Console</strong></div>
        </div>
        <div className="header-status">
          <span className={`connection-pill ${connected ? "online" : "offline"}`}>
            {connected ? <WifiHigh weight="bold" /> : <WifiSlash weight="bold" />}
            {connected ? "/dev/romo_b_pcu connected" : "Waiting for PCU"}
          </span>
          <div className="clock"><strong>{clock.toLocaleTimeString("ko-KR", { hour12: false })}</strong><span>{clock.toLocaleDateString("ko-KR")}</span></div>
          <button className="program-stop" title="Publish zero command and explicitly request PCU Manual" onClick={programStop}><Stop weight="fill" /><span>ZERO + MANUAL</span></button>
        </div>
      </header>

      <nav className="tab-bar" aria-label="Operator console views">
        {TABS.map(({ id, label, icon: Icon }) => (
          <button key={id} className={tab === id ? "active" : ""} onClick={() => setTab(id)}><Icon weight={tab === id ? "fill" : "regular"} />{label}</button>
        ))}
      </nav>

      <main className="content">
        <div className="status-ribbon">
          <div><StatusDot active={connected} /><span>PCU link</span><strong>{connected ? "ONLINE" : "OFFLINE"}</strong></div>
          <div><StatusDot active={state.readiness?.bridge_armed} /><span>HLV bridge</span><strong>{state.readiness?.bridge_armed ? "ARMED" : "MANUAL"}</strong></div>
          <div><StatusDot active={state.platform.auto_mode} /><span>PCU feedback</span><strong>{state.platform.auto_mode ? "AUTO" : "MANUAL"}</strong></div>
          <div><StatusDot active danger={state.platform.estop} /><span>Physical E-stop</span><strong>{state.platform.estop ? "ACTIVE" : "CLEAR"}</strong></div>
          <div><StatusDot active={state.health.lidar?.online} /><span>Mid-360</span><strong>{format(state.health.lidar?.rate_hz, 1)} Hz</strong></div>
          <div><StatusDot active={state.health.localization?.online} /><span>Localization</span><strong>{state.health.localization?.online ? "TRACKING" : "WAITING"}</strong></div>
        </div>
        {tab === "main" && <MainView state={state} onPost={notify} demo={demo} />}
        {tab === "algorithm" && <AlgorithmView />}
        {tab === "navigation" && <NavigationView state={state} onPost={notify} demo={demo} />}
        {tab === "system" && <SystemView state={state} onPost={notify} demo={demo} />}
        {tab === "diagnostics" && <DiagnosticsView state={state} />}
      </main>

      <footer className="app-footer">
        <span>ROS 2 Humble · 115200 8N1 · UI {state.version}</span>
        <span><StatusDot active={streamOnline || demo} />{demo ? "Demo telemetry" : streamOnline ? "Live telemetry" : "Telemetry reconnecting"}</span>
      </footer>
      {toast && <div className={`toast ${toast.success ? "success" : "failure"}`}>{toast.success ? <CheckCircle weight="fill" /> : <Warning weight="fill" />}<span>{toast.message}</span></div>}
    </div>
  );
}
