# Prototype Instructions

Run the local server yourself and open the preview in the browser available to this environment. Do not give the user server-start instructions when you can run it.

Before making substantial visual changes, use the Product Design plugin's `get-context` skill when the visual source is unclear or no longer matches the current goal. When the user gives durable prototype-specific design feedback, preferences, or decisions, record them in `AGENTS.md`.

When implementing from a selected generated mock, treat that image as the source of truth for layout, component anatomy, density, spacing, color, typography, visible content, and hierarchy.

Durable operator-console requirements: keep the verified-manual visual language, expose dense PCU/ROS telemetry and Auto-entry blockers, show all four steering feedback values whenever 4WIS or Pivot is active, and support hold-to-run forward/reverse control in 2WIS, 4WIS, and Pivot. The selected steering mode is maintained independently of the hold-to-run motion command; releasing motion must not make the PCU fall back to 2WIS while the browser control node is healthy. The browser is the primary operator surface, including field-stack start/stop, arm/manual, initial pose, goal, route, diagnostics, live mapping, map save, bag recording, replay, validation, build, and Autoware controls. Every process control must use an allow-listed argument vector, expose running state/PID/exit code/log output, prevent conflicting primary stacks, and constrain artifact selection to `data/local`.

All operator-facing labels, descriptions, warnings, buttons, and status explanations must be written in Korean while preserving standard technical names such as ROS 2, Nav2, Autoware, PCU, 2WIS, 4WIS, Pivot, topic names, and units.

OpenArm-v1 양팔은 차량 UI와 분리된 전용 탭에서 조작한다. 이 탭은 터미널 없이 좌·우 SocketCAN 연결/해제, 16개 모터 상태, 모터 enable/disable, 현재 자세 유지, 오류 해제, 관절별 목표, 양팔 동시 자세, 속도·gain, 자세 저장/복사, 현재 자세 영점 저장을 제공해야 한다. CAN 연결만으로 모터를 enable하거나 움직이지 않으며, enable은 8축 피드백이 모두 들어온 뒤 현재 자세 유지로 시작한다. 영점 저장은 모터가 disable된 상태와 명시적 확인 문구를 요구한다.
