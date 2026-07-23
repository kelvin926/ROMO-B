# Prototype Instructions

Run the local server yourself and open the preview in the browser available to this environment. Do not give the user server-start instructions when you can run it.

Before making substantial visual changes, use the Product Design plugin's `get-context` skill when the visual source is unclear or no longer matches the current goal. When the user gives durable prototype-specific design feedback, preferences, or decisions, record them in `AGENTS.md`.

When implementing from a selected generated mock, treat that image as the source of truth for layout, component anatomy, density, spacing, color, typography, visible content, and hierarchy.

Durable operator-console requirements: keep the verified-manual visual language, expose dense PCU/ROS telemetry and Auto-entry blockers, show all four steering feedback values whenever 4WIS or Pivot is active, and support hold-to-run forward/reverse control in 2WIS, 4WIS, and Pivot. The selected steering mode is maintained independently of the hold-to-run motion command; releasing motion must not make the PCU fall back to 2WIS while the browser control node is healthy. The browser is the primary operator surface, including field-stack start/stop, arm/manual, initial pose, goal, route, diagnostics, live mapping, map save, bag recording, replay, validation, build, and Autoware controls. Every process control must use an allow-listed argument vector, expose running state/PID/exit code/log output, prevent conflicting primary stacks, and constrain artifact selection to `data/local`.

All operator-facing labels, descriptions, warnings, buttons, and status explanations must be written in Korean while preserving standard technical names such as ROS 2, Nav2, Autoware, PCU, 2WIS, 4WIS, Pivot, topic names, and units.

첫 화면은 ROMO-B와 OpenArm-v1의 연결, 건전성, 핵심 운용 제어와 실행 상태를 함께 보여주는 통합 운용 화면으로 유지한다. 차량 직접 주행과 OpenArm 16축 조작처럼 밀도가 높은 기능은 각각 차량 상세와 양팔 상세 탭에 유지하되, 통합 화면에서 차량 Arm/수동 전환, OpenArm CAN 연결/해제, 양팔 enable/disable, 현재 자세 유지와 상세 화면 이동을 바로 할 수 있어야 한다.

OpenArm-v1 양팔 상세 탭은 터미널 없이 좌·우 SocketCAN 연결/해제, 16개 모터 상태, 모터 enable/disable, 현재 자세 유지, 오류 해제, 관절별 목표, 양팔 동시 자세, 속도·gain, 자세 저장/복사, 현재 자세 영점 저장을 제공해야 한다. CAN 연결만으로 모터를 enable하거나 움직이지 않으며, enable은 8축 피드백이 모두 들어온 뒤 현재 자세 유지로 시작한다. 영점 저장은 모터가 disable된 상태와 명시적 확인 문구를 요구한다.

OpenArm 일반 자세 명령은 100 Hz 5차 S-curve와 속도 feed-forward를 사용해 시작·종료가 부드러워야 한다. UI는 원본 관절/모터 속도 상한까지 선택할 수 있게 하되 축별 하드 속도 한계를 항상 적용한다. 좌·우 각 관절의 운용 각도 하한·상한은 웹에서 별도로 저장하고 원본 기계 한계로 복원할 수 있어야 하며, 하드 각도 한계 밖의 값은 거부하고 범위 편집 자체로 팔을 움직이지 않는다.

OpenArm-v1은 명령과 피드백에 런타임 부호 변환을 적용하지 않고 원본 캘리브레이션 관절 좌표를 그대로 사용한다. 공식 bimanual URDF와 자동 캘리브레이션을 따라 왼팔 J1은 -200°..80°, J2는 -190°..10°이고 오른팔 J1은 -80°..200°, J2는 -10°..190°이다. J3~J7과 gripper는 좌우 동일 범위다. 화면에서 좌우 영점·범위 차이를 명확히 표시하고 저장 자세에는 좌표 schema version을 기록한다.
