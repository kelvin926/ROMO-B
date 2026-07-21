import { useEffect, useMemo, useRef, useState } from "react";
import {
  ArrowCounterClockwise,
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
  Path,
  Play,
  Power,
  Pulse,
  Robot,
  ShieldCheck,
  SteeringWheel,
  Stop,
  Trash,
  Warning,
  WifiHigh,
  WifiSlash,
  Wrench,
} from "@phosphor-icons/react";

const DEMO_STATE = {
  version: "0.1.0",
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
  motion: { wheel_odom_speed_mps: 0.205, wheel_odom_yaw_rate_radps: 0.075 },
  localization: { available: true, x_m: 17.42, y_m: -4.83, yaw_deg: 89.2 },
  navigation: {
    plan_points: 86,
    waypoint_count: 4,
    last_action: "Navigation stack ready",
    last_action_success: true,
  },
  diagnostics: {
    level: 0,
    summary: "All platform systems nominal",
    items: [
      { name: "ROMO-B / PCU serial bridge", level: 0, message: "Armed 2WIS" },
      { name: "Localization", level: 0, message: "NDT tracking" },
      { name: "Nav2", level: 0, message: "Active" },
    ],
  },
  health: {
    platform: { online: true, age_sec: 0.03, rate_hz: 20.0 },
    lidar: { online: true, age_sec: 0.06, rate_hz: 10.0 },
    localization: { online: true, age_sec: 0.08, rate_hz: 10.0 },
    odometry: { online: true, age_sec: 0.02, rate_hz: 20.0 },
  },
  services: {
    arm: true,
    waypoint_save: true,
    waypoint_reload: true,
    waypoint_clear: true,
    waypoint_execute: true,
    waypoint_cancel: true,
  },
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
    localization: { online: false, age_sec: null, rate_hz: 0 },
    odometry: { online: false, age_sec: null, rate_hz: 0 },
  },
  services: {
    arm: false,
    waypoint_save: false,
    waypoint_reload: false,
    waypoint_clear: false,
    waypoint_execute: false,
    waypoint_cancel: false,
  },
};

const TABS = [
  { id: "main", label: "Main", icon: Gauge },
  { id: "algorithm", label: "Platform control algorithm", icon: Wrench },
  { id: "navigation", label: "Navigation", icon: MapPin },
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
  const armed = state.platform.state === 2 && !state.platform.estop;
  const canDrive = demo || (armed && state.platform.connected);

  const drive = (payload) => {
    if (demo) return;
    postJson("/api/drive", payload).catch((error) => onPost(error.message, false));
  };

  return (
    <section className="panel command-panel">
      <div className="panel-title">
        <div>
          <span className="eyebrow">Platform status command</span>
          <h2>Vehicle command</h2>
        </div>
        <span className={`mode-chip ${armed ? "armed" : ""}`}>
          {armed ? "ARMED" : "MANUAL"}
        </span>
      </div>

      <div className="command-status-grid">
        <button
          className={`state-control ${armed ? "selected" : ""}`}
          disabled={!demo && !state.services.arm}
          onClick={() => onPost("arm", !armed)}
        >
          <Power weight="bold" />
          <span>{armed ? "Switch to Manual" : "Request Auto / Arm"}</span>
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
          <button className={mode === "2wis" ? "active" : ""} onClick={() => setMode("2wis")}>2WIS</button>
          <button disabled title="4WIS is not enabled in the ROMO-B bridge">4WIS</button>
          <button className={mode === "pivot" ? "active" : ""} onClick={() => setMode("pivot")}>Pivot</button>
        </div>
      </div>

      {mode === "2wis" ? (
        <>
          <div className="slider-field">
            <div className="field-label-row">
              <label htmlFor="speed">Forward speed</label>
              <output>{format(speed, 2)} m/s</output>
            </div>
            <input id="speed" type="range" min="0" max="0.5" step="0.01" value={speed} onChange={(event) => setSpeed(Number(event.target.value))} />
            <div className="range-labels"><span>0</span><span>navigation max 0.5</span></div>
          </div>
          <div className="slider-field">
            <div className="field-label-row">
              <label htmlFor="steer">Center steering</label>
              <output className={Math.abs(steer) > 18 ? "warning-text" : ""}>{format(steer, 1)}°</output>
            </div>
            <input id="steer" type="range" min="-22" max="22" step="0.5" value={steer} onChange={(event) => setSteer(Number(event.target.value))} />
            <div className="range-labels"><span>Left −22°</span><span>0°</span><span>Right +22°</span></div>
          </div>
          <div className="drive-pad">
            <HoldButton
              disabled={!canDrive}
              payload={{ mode: "2wis", speed_mps: speed, steer_deg: steer }}
              onSend={drive}
              className="drive-forward"
            >
              <ArrowUp weight="bold" />
              <span>Hold to drive</span>
              <small>{format(speed, 2)} m/s · {format(steer, 1)}°</small>
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
        <div className="control-hint"><HandPalm weight="fill" /> Connect the PCU, release physical E-stop, then request Auto.</div>
      )}
    </section>
  );
}

function WheelCard({ name, speed, steer, front }) {
  return (
    <div className="wheel-card">
      <span>{name}</span>
      <strong>{format(speed, 2)} <small>m/s</small></strong>
      <em>{front ? `${format(steer, 1)}°` : "fixed"}</em>
    </div>
  );
}

function FeedbackPanel({ state }) {
  const platform = state.platform;
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
        <div><StatusDot active={platform.auto_mode} /><span>PCU Auto</span><strong>{platform.auto_mode ? "AUTO" : "MANUAL"}</strong></div>
        <div><StatusDot active={platform.estop} danger /><span>E-stop</span><strong>{platform.estop ? "ACTIVE" : "CLEAR"}</strong></div>
      </div>
      <div className="vehicle-feedback">
        <div className="wheel-column">
          <WheelCard name="FL" speed={platform.wheel_speed_mps[0]} steer={platform.wheel_steer_deg[0]} front />
          <WheelCard name="RL" speed={platform.wheel_speed_mps[2]} steer={platform.wheel_steer_deg[2]} />
        </div>
        <div className="vehicle-center">
          <span className="front-label">FRONT</span>
          <CarProfile size={94} weight="duotone" />
          <strong>ROMO-B</strong>
          <small>{platform.steer_mode_name}</small>
          <span className="rear-label">REAR</span>
        </div>
        <div className="wheel-column">
          <WheelCard name="FR" speed={platform.wheel_speed_mps[1]} steer={platform.wheel_steer_deg[1]} front />
          <WheelCard name="RR" speed={platform.wheel_speed_mps[3]} steer={platform.wheel_steer_deg[3]} />
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

function MainView({ state, onPost, demo }) {
  return (
    <div className="main-grid">
      <CommandPanel state={state} onPost={onPost} demo={demo} />
      <FeedbackPanel state={state} />
    </div>
  );
}

function computeWheelTargets(speed, steerDeg) {
  const wheelbase = 0.323;
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
  return {
    fl: leftIsInner ? [innerSpeed, innerAngle] : [outerSpeed, outerAngle],
    fr: leftIsInner ? [outerSpeed, outerAngle] : [innerSpeed, innerAngle],
    rl: leftIsInner ? [speed * Math.abs(innerRadius / radius), 0] : [speed * Math.abs(outerRadius / radius), 0],
    rr: leftIsInner ? [speed * Math.abs(outerRadius / radius), 0] : [speed * Math.abs(innerRadius / radius), 0],
  };
}

function AlgorithmView() {
  const [speed, setSpeed] = useState(0.2);
  const [steer, setSteer] = useState(8);
  const targets = useMemo(() => computeWheelTargets(speed, steer), [speed, steer]);
  return (
    <section className="panel algorithm-panel">
      <div className="panel-title wide-title">
        <div><span className="eyebrow">Platform control algorithm</span><h2>2WIS kinematic preview</h2></div>
        <div className="geometry-chips"><span>L 0.323 m</span><span>W 0.390 m</span></div>
      </div>
      <div className="algorithm-layout">
        <div className="algorithm-inputs">
          <p>센터 속도와 조향각을 입력하면 각 휠의 목표 선속도와 전륜 조향각을 계산합니다.</p>
          <div className="slider-field">
            <div className="field-label-row"><label>Center speed</label><output>{format(speed, 2)} m/s</output></div>
            <input type="range" min="0" max="0.5" step="0.01" value={speed} onChange={(event) => setSpeed(Number(event.target.value))} />
          </div>
          <div className="slider-field">
            <div className="field-label-row"><label>Center steering</label><output>{format(steer, 1)}°</output></div>
            <input type="range" min="-22" max="22" step="0.5" value={steer} onChange={(event) => setSteer(Number(event.target.value))} />
          </div>
          <div className="formula-card"><span>ROS angular.z</span><strong>{format(speed * Math.tan((steer * Math.PI) / 180) / 0.323, 3)} rad/s</strong><small>ω = v · tan(δ) / L</small></div>
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
  const action = (name) => {
    if (demo) return onPost(`Demo: waypoint ${name}`, true, true);
    postJson(`/api/waypoints/${name}`).then((result) => onPost(result.message, true, true)).catch((error) => onPost(error.message, false, true));
  };
  const services = state.services;
  return (
    <div className="navigation-grid">
      <section className="panel route-panel">
        <div className="panel-title"><div><span className="eyebrow">Waypoint navigation</span><h2>Route operations</h2></div><Path size={30} weight="duotone" /></div>
        <div className="route-summary">
          <div><span>Waypoints</span><strong>{state.navigation.waypoint_count}</strong></div>
          <div><span>Plan poses</span><strong>{state.navigation.plan_points}</strong></div>
          <div><span>Speed cap</span><strong>0.50 <small>m/s</small></strong></div>
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
        <div className="localization-note"><MapPin weight="fill" /><div><strong>2D Pose Estimate</strong><span>문·기둥·코너 가까이에서 실제 전방 방향으로 지정하세요. 현재 프로필은 클릭 위치 주변 15 m 밖의 정합을 거부합니다.</span></div></div>
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

function DiagnosticsView({ state }) {
  return (
    <div className="diagnostics-layout">
      <section className="health-grid">
        <HealthCard icon={Broadcast} title="PCU serial feedback" health={state.health.platform} expected="20 Hz" />
        <HealthCard icon={Robot} title="Wheel odometry" health={state.health.odometry} expected="20 Hz" />
        <HealthCard icon={Crosshair} title="LiDAR localization" health={state.health.localization} expected="10 Hz" />
        <HealthCard icon={WifiHigh} title="Mid-360 point cloud" health={state.health.lidar} expected="10 Hz" />
      </section>
      <section className="panel diagnostic-list">
        <div className="panel-title"><div><span className="eyebrow">ROS diagnostics</span><h2>{state.diagnostics.summary}</h2></div></div>
        {state.diagnostics.items.length ? state.diagnostics.items.map((item) => (
          <div className="diagnostic-row" key={item.name}>
            <StatusDot active={item.level === 0} danger={item.level >= 2} />
            <div><strong>{item.name}</strong><span>{item.message}</span></div>
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
          <button className="program-stop" onClick={programStop}><Stop weight="fill" /><span>PROGRAM STOP</span></button>
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
          <div><StatusDot active={state.platform.auto_mode} /><span>Control</span><strong>{state.platform.state_name}</strong></div>
          <div><StatusDot active={!state.platform.estop} danger={state.platform.estop} /><span>Physical E-stop</span><strong>{state.platform.estop ? "ACTIVE" : "CLEAR"}</strong></div>
          <div><StatusDot active={state.health.lidar?.online} /><span>Mid-360</span><strong>{format(state.health.lidar?.rate_hz, 1)} Hz</strong></div>
          <div><StatusDot active={state.health.localization?.online} /><span>Localization</span><strong>{state.health.localization?.online ? "TRACKING" : "WAITING"}</strong></div>
        </div>
        {tab === "main" && <MainView state={state} onPost={notify} demo={demo} />}
        {tab === "algorithm" && <AlgorithmView />}
        {tab === "navigation" && <NavigationView state={state} onPost={notify} demo={demo} />}
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
