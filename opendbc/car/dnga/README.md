# DNGA Yaris Cross HEV

V4.2 keeps the cleaned V4 file structure while returning longitudinal behavior
toward the last known-good pre-feedback V3.3R baseline
(`5a5ba4c9cb13b6135dc01fc7e3ce5815f4eb35d3`).

The 0x275-family signals are read-only physical feedback. They do not own ACC
engagement, the SET/set-speed display, LKAS state, steering authority, or the
hydraulic brake request.

## Structure

- `carcontroller.py`: steering limits, CAN scheduling, HUD, and actuator feedback.
- `longitudinal.py`: lead qualification, desired-speed shaping, hydraulic braking,
  stop-and-go, and brake-to-propulsion handoff.
- `dnga_hybrid_feedback.py`: read-only decoding/freshness/agreement for 0x275,
  0x2C9, 0x12A, 0x125, and 0x08C.
- `carstate.py`: vehicle state, buttons, ACC MAIN/SET/RES, and LKAS latches.
- `interface.py`: car parameters plus passive bus-1 HEV and bus-2 stock-camera observation.
- `dngacan.py`: outgoing 0x1D0/0x271/0x273/0x274 encoding and checksums.
- `values.py`: vehicle identification and active control tuning.

The authoritative DBC is `opendbc/dnga_hev.dbc`.

## V4.2 control contract

ACC/session presentation follows openpilot engagement plus the CarState
ACC MAIN/SET/RES latch. CANCEL and the brake pedal normally clear that latch in
CarState; the controller-level cancel/pedal/gas gates can suspend actuation
without being allowed to rewrite the 0x273 session on their own. Driver
accelerator override therefore preserves the visible ACC session. HEV feedback
cannot clear or latch the cruise session and cannot change LKAS/HUD engagement
state.

Normal 0x273 mode follows the stock-observed V3.3 state table:

- disabled: IS_ACCEL=0, IS_DECEL=0
- normal 0x01, including a lowered lead-deceleration target: IS_ACCEL=1, IS_DECEL=0
- moving brake 0x21 / crawl 0x31: IS_ACCEL=0, IS_DECEL=1
- standstill hold 0x30: IS_ACCEL=1, IS_DECEL=1

For a trusted lead, V4.2 restores V3.3-style below-current-speed 0x273 shaping
before and after hydraulic braking. Generic no-lead negative targets remain
disabled. Positive propulsion still requires planner/PID agreement.

Hydraulic braking remains planner-primary. A fresh checksum-valid stock-camera
0x271/0x273 braking pair is direct brake-only evidence and is not vetoed by a
temporarily positive downstream PID. The geometry-only fallback requires a
trusted closing lead plus matching negative planner intent. V4.1's stronger
0.24 final-crawl floor is retained for stop completion.

## HEV feedback scope

The HEV observer is used only at a deceleration-to-propulsion boundary. A prior
negative target, hydraulic brake, or SNG release marks a handoff pending. Positive
desired-speed buildup remains at zero until three consecutive 20 Hz samples
confirm:

- all five HEV channels are fresh and mutually consistent;
- 0x275 brake request and 0x08C friction are clear; and
- 0x275/0x2C9/0x12A/0x125 show that strong negative torque has faded.

That three-sample (~0.15 s) confirmation is signal debounce, not a blind release
timer. Stale or disagreeing feedback never disables ACC, LKAS, steering, or
hydraulic braking; it only prevents a pending brake-to-propulsion transition
from building a positive speed target.

The stock-observed 1.20 s 0x271 FC/04/C8 release sequence is retained as protocol
framing, but the timer itself no longer blocks 0x273 normal mode or positive
target buildup after physical HEV feedback proves the handoff ready.

The active Panda safety policy is unchanged.

## Deliberately retained safeguards

Lead qualification, brake entry/rate limiting, measured-deceleration governor,
low-speed ECU wake, stopped-lead confirmation, 0x21 -> 0x31 -> 0x30 stop states,
lead-release hold, and overshoot recovery remain. These are independent of the
0x275 observer and should be changed only from road-log evidence.

V4.0's differential equality harness was removed from V4.2 because V4.2
intentionally changes longitudinal behavior. The original harness remains on
the V4.0 branch for auditing the refactor itself. Native-device syntax checks,
Panda checks, and controlled vehicle testing are required before relying on
V4.2 stop completion.
