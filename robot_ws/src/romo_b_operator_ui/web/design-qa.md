# ROMO-B operator console design QA

## Source truth

- `/tmp/codex-remote-attachments/019f6f1c-7733-7d01-b5df-92b771c6c176/9868f797-4d3c-4f52-a92e-e1f9dde2251b/1-Photo-1.jpg`
- `/tmp/codex-remote-attachments/019f6f1c-7733-7d01-b5df-92b771c6c176/9868f797-4d3c-4f52-a92e-e1f9dde2251b/2-Photo-2.jpg`
- `/tmp/codex-remote-attachments/019f6f1c-7733-7d01-b5df-92b771c6c176/9868f797-4d3c-4f52-a92e-e1f9dde2251b/3-Photo-3.jpg`

## Captured implementation

- Main with live PCU feedback: `qa-operator-main-v2.png`
- System control and telemetry: `qa-operator-system-v2.png`
- Browser-only navigation: `qa-operator-navigation-v2.png`
- Mobile Main: `qa-operator-mobile-v2.png`
- Desktop capture: 1218 x 1451 full-page image at the normal app viewport.
- Responsive capture: 390 x 844 viewport with no document-level horizontal overflow.

## Full-view comparison

The manual photographs and the live Main capture were inspected together. The
implementation preserves the manual's defining operator hierarchy: top tabs,
persistent stop control, command and feedback halves, independent HLV and PCU
Auto state, E-stop feedback, steering-mode selection, speed and steering input,
four-wheel feedback, Alive counters, and a four-wheel kinematic view. The
monochrome LabVIEW surface was translated to higher-contrast industrial status
tokens without changing the information architecture.

System control and Navigation intentionally extend the manual rather than
imitate it. They use the same typography, border, spacing, status-color, and
icon tokens, so runtime ownership, the velocity pipeline, sensor rates, laptop
resources, the ROS graph, initial pose, and direct goals remain one coherent
console.

## Focused comparison and fixes

1. HLV Arm and PCU Auto were previously conflated. They now appear as separate
   command, feedback, ribbon, readiness, and diagnostic values. The live check
   correctly showed `HLV ARMED`, `PCU AUTO`, and physical E-stop `ACTIVE` at the
   same time.
2. Rear-wheel feedback previously rendered `fixed` unconditionally. It now
   renders `fixed in 2WIS` only when the *actual PCU feedback mode* is 2WIS;
   4WIS and Pivot display all four measured steering angles. Selecting a mode
   also requests that geometry at zero speed so feedback updates before motion.
3. Command controls now include signed forward/reverse operation for 2WIS and
   4WIS, plus Pivot CW/CCW hold controls. The 4WIS steering limit is 18 degrees
   and the 2WIS limit is 22 degrees, matching the manual.
4. The page no longer depends on a second command-line UI process. The enabled
   user service owns the browser console, while System control exposes the
   field-stack start/zero-stop state and its process/log ownership.
5. The final status-dot pass corrected active physical E-stop to red in both
   the ribbon and feedback panel. Offline sensors remain neutral gray and
   healthy live paths remain green.

## Mandatory QA passes

- Typography: Manrope variable font loads locally; heading, telemetry, label,
  and code styles preserve a readable hierarchy without clipped dynamic text.
- Spacing/layout: desktop command, feedback, readiness, system, and navigation
  panels align to the same grid and retain consistent 7 px industrial radii.
- Viewport resilience: 1218 px desktop and 390 x 844 mobile were checked. The
  mobile tab row intentionally scrolls horizontally, controls stack to one
  column, the stop action becomes icon-only, and document horizontal overflow
  remains absent.
- Colors/tokens: semantic green, amber, red, and neutral states are consistent;
  active E-stop and destructive zero/stop actions are visually distinct.
- Assets/icons: the source uses no product imagery. All controls use one
  Phosphor icon family; no placeholder art, custom SVG replacement, or fake
  dashboard imagery is present.
- Copy: labels distinguish selected command mode from actual PCU feedback and
  explain every Auto-entry blocker.
- Accessibility: native buttons, sliders, labels, outputs, and number inputs
  are keyboard reachable; active/disabled states are not color-only; practical
  mobile tap targets and visible focus styling are retained.

## Interactions tested

- Main, 4WIS selection, Navigation, and System control tab switching.
- Signed `Hold forward` and `Hold reverse` controls rendered for both 2WIS and
  4WIS; Pivot exposes CW and CCW controls. No motion was sent because the live
  physical E-stop feedback was active.
- Live PCU state, Alive counters, odometry, command pipeline, ROS graph, laptop
  CPU/memory/GPU telemetry, initial-pose form, and direct-goal form rendered.
- Field-stack ownership detection showed the externally launched stack and PID
  without starting a duplicate process.
- User service restart and HTTP state endpoint recovery were verified.
- No application console error was emitted. The only warning was the Codex
  Electron host's generic development Content-Security-Policy warning.

## Findings

- P0: none.
- P1: none.
- P2: none.
- P3: none requiring another iteration for the laptop-first operator workflow.

final result: passed
