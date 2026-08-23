from opendbc.car import Bus
from opendbc.car.dnga.dngacan import (
  create_can_steer_command,
  dnga_create_accel_command,
  dnga_create_brake_command,
  dnga_create_hud,
)
from opendbc.car.dnga.values import NOT_CAN_CONTROLLED
from opendbc.car.dnga.dnga_hybrid_feedback import hybrid_feedback_snapshot
from opendbc.can import CANPacker
from numpy import clip, interp
from opendbc.car.common.conversions import Conversions as CV
from cereal import messaging
from opendbc.car.interfaces import CarControllerBase


# V3.3R4 hybrid-feedback-supervisor build: offline/replay/bench validation.
#
# R4 retains every R3 stopping and R2 handoff safeguard, but changes the final
# brake-to-propulsion handoff from timer/aEgo inference to read-only bus-1
# feedback.  0x275/0x2C9/0x12A/0x125/0x08C are never transmitted here.
# Brake request and friction activity must clear before IS_ACCEL can arm at
# current speed; positive target ramp waits until strong negative hybrid torque
# has faded.  Stale/disagreeing feedback or persistent positive torque under
# friction braking latches a fault and requires a fresh SET/RES engagement.
#
# Changes from V3.3R2 are deliberately limited to trusted-lead stopping:
#   * A checksum-validated, fresh factory-camera 0x271/0x273 deceleration pair
#     is a brake-only lower-bound observer below 28.8 km/h. It can request
#     earlier/stronger braking but can never request propulsion.
#   * A radar-geometry fallback enters the same path when a trusted slow lead
#     is closing inside 20 m and downstream control already requests braking.
#   * The stop guard begins from the stock-observed initial request (bounded at
#     0.36 m/s^2), tracks upward at no more than 0.05 per 20 Hz update, and is
#     capped at the recorded stock maximum of 0.87 m/s^2.
#   * The 0.5-pump encoding is used only from the stock-observed 0.75 request
#     threshold upward. All other brake modes retain their previous limits.
#   * Once the guarded stop has begun, positive downstream PI cannot release it
#     while the selected lead is still closing or completing a stop.
#
# Retained V3.3R2 containment:
#   * A persistent deceleration latch prohibits IS_ACCEL and positive target
#     buildup from the first fresh negative planner request through a verified
#     neutral handoff.
#   * Every hydraulic release while control remains allowed uses the
#     stock-observed 0x01 + FC/04 + C8 stage and retains IS_DECEL for at least
#     the observed 1.2-second pump release.
#   * The latch clears only after fresh planner/PID positive agreement and
#     measured deceleration have all remained neutral for 0.50 seconds.
#   * The old V3.0 PI-positive/negative-aEgo handoff trigger is prohibited.
#   * Driver gas, brake, cancel, or cruise-latch loss sends exact disabled
#     longitudinal frames; the encoder no longer uses stale `enabled` state.
#   * Ambiguous 0x273 target-speed and curve regen are disabled. V3.0's early,
#     progressive hydraulic entry/ramp remains the braking baseline.
#   * Low-speed positive-target authority and buildup rate are reduced, and
#     cannot arm until the deceleration latch and neutral dwell have cleared.
#
# Retained V3.3R1 safeguards:
#   * 0x30 is sent only at confirmed physical standstill; 0x31 owns creep.
#   * The trusted stopped-lead crawl floor survives all caps and filtering.
#   * The measured-deceleration governor limits the applied command after the
#     target filter, so its requested step is not attenuated a second time.
#   * The launch guard requires both no positive intent and no positive target.
#   * Generic no-lead braking/regen and curve hydraulic braking are disabled.
#   * Trusted-lead TTC/closing deterioration uses a faster, still-capped
#     hydraulic escalation path for the logged 10:08 failure mode.
#
# MPC, longitudinal PID, distance bars, steering limits, CAN encoders, and the
# model runner are unchanged. The paired interface.py adds the read-only raw
# feedback observer, while the paired Panda policy independently enforces the
# same freshness/agreement boundary. RPM remains diagnostic-only here.
#
# V2.5R keeps V2.5P/Q's explicit stock-observed 0x273 state mapping and
# 0x21 -> 0x31 -> 0x30 stop-and-go states, but changes command arbitration:
#
#   * A fresh longitudinalPlan is the primary direction request. A temporary
#     positive downstream PID correction cannot release hydraulic braking or
#     request propulsion while the planner still asks to slow for a lead.
#   * Normal lead following uses 0x01 coasting/regen first. Hydraulic braking
#     starts weak and progressively builds only after sustained planner decel.
#   * Normal lead hydraulic authority is intentionally below the observed stock
#     maximum. A separate urgent path keeps enough authority for a genuinely
#     fast closing event.
#   * After braking, lead propulsion requires sustained positive planner demand
#     and remains capped to a low rate, preventing rev/jump/brake cycling.
#   * The selected 1/2/3-bar value changes local hydraulic sensitivity.
#     carstate.py now publishes the same selection through distanceLines, so
#     the MPC uses the matching aggressive/standard/relaxed time gap.
#
# Steering limits and interface longitudinal gains are deliberately unchanged
# in this patch. They should be changed separately only after the longitudinal
# actuator handoff is stable.
V25L_BRAKE_MIN = 0.05

# Progressive pedal-like shaping at the 20 Hz longitudinal CAN rate.
V25R_BRAKE_STEP_UP = 0.015
V25R_BRAKE_STEP_UP_URGENT = 0.030
V25R_BRAKE_STEP_DOWN = 0.015
V25R_BRAKE_FILTER_UP = 0.08
V25R_BRAKE_FILTER_DOWN = 0.08

V25R_LEAD_ENTRY_FRAMES = 4          # 0.20 seconds
V25R_URGENT_ENTRY_FRAMES = 2        # 0.10 seconds
V25L_LEAD_TRUST_FRAMES = 10         # 0.50 seconds
V25L_URGENT_LEAD_FRAMES = 3         # 0.15 seconds
V25L_REENTRY_BLOCK_FRAMES = 80      # 0.80 seconds at 100 Hz
V25L_PROPULSION_DWELL_FRAMES = 60   # 0.60 seconds after hydraulic release
V25L_MIN_MOVING_SPEED = 0.15
V25L_MIN_ENTRY_SPEED = 0.50

# Planner/radar freshness and hysteresis.
V25R_PLAN_MAX_AGE_FRAMES = 50       # 0.50 seconds at 100 Hz
V25R_RADAR_MAX_AGE_FRAMES = 50
V25R_RELEASE_CONFIRM_FRAMES = 10    # 0.50 seconds at 20 Hz
V25R_LEAD_ACCEL_CONFIRM_FRAMES = 3  # 0.15 seconds at 20 Hz
V25R_LEAD_LOSS_FRAMES = 3           # ignore one 20 Hz lead dropout
V25R_PLANNER_DECEL_DEADBAND = 0.03
V25R_PLANNER_ACCEL_ENTRY = 0.05
V25R_PLANNER_RELEASE = 0.05

# Normal lead braking is deliberately gentler than stock. Urgent closing can
# still use more authority, but remains below V2.5R's 0.75 m/s^2 maximum.
V25R_URGENT_PLANNER_DECEL = 0.45
V25R_URGENT_HARD_DECEL = 0.70
V25R_URGENT_CLOSING_SPEED = 2.5     # m/s
V25R_URGENT_TTC = 7.0               # seconds
V25R_URGENT_BRAKE_MAX = 0.45

# Planner-primary target with only a small same-direction PID allowance.
V25R_PID_BRAKE_BLEND = 0.25
V25R_PID_BRAKE_ALLOWANCE = 0.08

# V3.0 early-progressive highway lead braking.
V30_HIGHWAY_MIN_SPEED = 8.0
V30_NORMAL_ENTRY_MIN = 0.12
V30_NORMAL_ENTRY_MAX = 0.20
V30_NORMAL_BRAKE_CAP_MIN = 0.25
V30_NORMAL_BRAKE_CAP_MAX = 0.30
V30_BRAKE_STEP_UP = 0.006
V30_BRAKE_STEP_UP_URGENT = 0.015
V30_HANDOFF_FRAMES = 3
V30_HANDOFF_STEP_DOWN = 0.030

# V3.2R isolated low-speed ECU wake/handoff.
#
# This is intentionally separate from the V3.0 highway and curve logic:
#   * V3.0 hydraulic entry and caps stay unchanged; handoff is guarded below.
#   * The outgoing fake lead bit is allowed only below 30.6 km/h.
#   * It first arms at exactly zero desired-speed offset.
#   * Positive target still requires measured deceleration to fade, retaining
#     V3.0's -0.10 m/s² aEgo gate instead of pushing through active regen.
V32R_LOW_SPEED_MAX = 8.5
V32R_REGEN_NEUTRAL_DWELL_FRAMES = 50   # 0.50 s at 100 Hz
V32R_LOW_SPEED_ARM_FRAMES = 30         # 0.30 s lead-bit-only neutral stage
V32R_LOW_SPEED_ACCEL_CAP = 0.05
V32R_LOW_SPEED_OFFSET_CAP = 0.08
V32R_LOW_SPEED_OFFSET_STEP_UP = 0.001
V32R_DEPARTING_LEAD_VREL = 0.40
V32R_DEPARTING_LEAD_DREL = 3.0
V32R_DEPARTING_ACCEL_CAP = 0.08
V32R_DEPARTING_OFFSET_CAP = 0.10
V32R_DEPARTING_OFFSET_STEP_UP = 0.0015
V32R_NONBLOCKING_LEAD_DISTANCE = 20.0
V32R_NONBLOCKING_LEAD_VREL = -0.50
V32R_LOW_SPEED_OVERSHOOT_AEGO = 0.30
V32R_OVERSHOOT_BLOCK_FRAMES = 50

# V3.3R stock-derived safety layers.
#
# The confirmed engine RPM signal is diagnostic only. These safeguards use
# planner direction, target slope, hydraulic state, trusted lead motion, and
# actual vehicle acceleration because the logged launch happened at 0 RPM.
V33R_LOW_SPEED_ENGAGEMENT_MAX = 4.17
V33R_LOW_SPEED_ENGAGEMENT_GUARD_FRAMES = 50

V33R_STOP_LEAD_TRUST_FRAMES = 3
V33R_STOP_HOLD_MIN_DISTANCE = 1.0
V33R_STOP_HOLD_MAX_DISTANCE = 12.0
V33R_STOPPED_LEAD_MAX_SPEED = 0.50
V33R_HOLD_RESUME_LEAD_SPEED = 0.35
V33R_HOLD_RESUME_FRAMES = 6
V33R_CREEP_GUARD_MIN_EGO = 0.0
V33R_CREEP_GUARD_MAX_EGO = 1.00
V33R_CREEP_GUARD_MIN_CLOSING = 0.05
V33R_CREEP_ENTRY_FRAMES = 2
V33R_CREEP_BRAKE_FLOOR = 0.18

V33R_RELEASE_FREEZE_FRAMES = 30
V33R_RELEASE_LEAD_HOLD_FRAMES = 80
V33R_RELEASE_PROPULSION_BLOCK_FRAMES = 60
V33R_TARGET_SLOPE_UNLOCK_FRAMES = 30
V33R_TARGET_RETURN_STEP = 0.010
V33R_TARGET_SLOPE_BRAKE = 0.02
V33R_TARGET_SLOPE_AEGO = -0.08
V33R_PROPULSION_AEGO_MIN = -0.05

V33R_OVERSHOOT_AEGO = 0.25
V33R_OVERSHOOT_CONFIRM_FRAMES = 2
V33R_OVERSHOOT_BLOCK_FRAMES = 60

V33R_EARLY_HIGHWAY_MIN_SPEED = 8.0
V33R_EARLY_HIGHWAY_CLOSING = 1.0
V33R_EARLY_HIGHWAY_TTC = 18.0
V33R_EARLY_HIGHWAY_TIME_GAP = 3.0
V33R_EARLY_HIGHWAY_PLANNER_BRAKE = 0.05

V33R_AEGO_FILTER_ALPHA = 0.20
V33R_DECEL_GOVERNOR_START = -0.90
V33R_DECEL_GOVERNOR_CRITICAL_TTC = 3.0
V33R_DECEL_GOVERNOR_CRITICAL_CLOSING = 3.0
V33R_DECEL_GOVERNOR_STEP_DOWN = 0.015

# V3.3R2 persistent deceleration latch and stock-observed release stage.
# Longitudinal commands are emitted at 20 Hz while `frame` advances at 100 Hz.
V33R2_DECEL_LATCH_BRAKE = 0.02
V33R2_DECEL_CLEAR_PLANNER_ACCEL = 0.05
V33R2_DECEL_CLEAR_PID_ACCEL = 0.05
V33R2_DECEL_CLEAR_AEGO = -0.02
V33R2_DECEL_CLEAR_FRAMES = 10          # 0.50 s at 20 Hz
V33R2_RELEASE_PUMP_FRAMES = 120        # 1.20 s at 100 Hz
V33R2_REENTRY_BLOCK_FRAMES = 10        # 0.10 s, not the old blind 0.80 s

# V3.3R3 low-speed missed-stop guard. The authority ceiling and pump split are
# taken from the simultaneously captured factory-camera requests in the 13:21
# and 13:24 approaches. Factory traffic is observation-only: the controller
# re-encodes its own bounded command and never forwards raw camera frames.
V33R3_STOCK_FRAME_MAX_AGE = 25          # 0.25 s at 100 Hz
V33R3_STOP_GUARD_MAX_SPEED = 8.0        # 28.8 km/h
V33R3_STOP_GUARD_MAX_DISTANCE = 20.0
V33R3_STOP_GUARD_MIN_CLOSING = 0.20
V33R3_STOP_GUARD_MIN_PID_BRAKE = 0.05
V33R3_STOP_GUARD_MIN_STOCK_BRAKE = 0.08
V33R3_PREDICTIVE_MIN_CLOSING = 0.50
V33R3_PREDICTIVE_MAX_TTC = 25.0
V33R3_PREDICTIVE_MAX_LEAD_SPEED = 5.5
V33R3_PREDICTIVE_ENTRY_FRAMES = 2       # 0.10 s at 20 Hz
V33R3_PREDICTIVE_STANDSTILL_GAP = 5.0
V33R3_PREDICTIVE_REACTION_TIME = 0.35
V33R3_PREDICTIVE_INITIAL_BRAKE = 0.13
V33R3_STOCK_INITIAL_BRAKE_MAX = 0.36
V33R3_STOP_BRAKE_MAX = 0.87
V33R3_STOP_BRAKE_FILTER_UP = 0.30
V33R3_STOP_BRAKE_STEP_UP = 0.05
V33R3_PUMP_05_THRESHOLD = 0.75
V33R3_STOP_COMPLETION_MAX_LEAD_SPEED = 1.0
V33R3_STOP_COMPLETION_MAX_DISTANCE = 12.0

# V3.3R4 feedback-supervised handoff. Controller decisions run at 20 Hz while
# frame numbers advance at 100 Hz.
V33R4_BRAKE_CLEAR_FRAMES = 5              # 0.25 s stable zero pressure/request
V33R4_TORQUE_READY_FRAMES = 5             # 0.25 s before positive target ramp
V33R4_ENTRY_OVERLAP_FRAMES = 55           # stock max 0.20 s; allow 0.55 s
V33R4_OVERLAP_FAULT_FRAMES = 3            # stock return max 2; fault at 0.15 s
V33R4_DISAGREE_FAULT_FRAMES = 3
V33R4_ENTRY_TORQUE_RISE_RAW = 150

# V3.3R1 10:08 trusted-lead emergency escalation. The logged approach began
# normally at 19.59 m / 1.09 m/s closing, then deteriorated to 9.57 m / 3.03
# m/s while both planner and downstream control requested strong deceleration.
# Enter the faster path only with a fully trusted, selected lead, TTC/closing
# agreement, and clearly negative planner demand. Retain the existing 0.45
# m/s^2 hydraulic ceiling; this changes response time, not maximum authority.
V33R_EMERGENCY_CLOSING_SPEED = 2.0
V33R_EMERGENCY_TTC = 8.0
V33R_EMERGENCY_PLANNER_BRAKE = 0.20
V33R_EMERGENCY_BRAKE_FILTER_UP = 0.20
V33R_EMERGENCY_BRAKE_STEP_UP = 0.030

# Legacy V3.3R1 curve-regen calibration retained only for source comparison.
# V3.3R2 forces `curve_regen_allowed = False` until actual negative MG torque
# or delivered-regeneration feedback is decoded on this HEV.
V33R_CURVE_REGEN_MIN_SPEED = V32R_LOW_SPEED_MAX
V33R_CURVE_REGEN_ENTRY = 0.08
V33R_CURVE_REGEN_PID_ENTRY = 0.05
V33R_CURVE_REGEN_CONFIRM_FRAMES = 3
V33R_CURVE_REGEN_OFFSET_MAX = 0.20
V33R_CURVE_REGEN_OFFSET_STEP_DOWN = 0.010

# Stock-like 0x01 desired-speed shaping. Positive acceleration follows the
# previously successful continuous target strategy, while negative lead demand
# ramps into a conservative current-speed-relative offset. V3.3R1 prohibits
# generic no-lead braking but permits a smaller, slower offset only while the
# fresh longitudinal planner source is a confirmed vision turn.
V25L_DECEL_DEADBAND = 0.02
V25L_DECEL_OFFSET_STEP_DOWN = 0.02
V25L_DECEL_OFFSET_STEP_UP = 0.08
V25L_ACCEL_ENTRY = 0.05
V25L_ACCEL_CAP = 0.25
V25L_ACCEL_OFFSET_STEP_UP = 0.004
V25L_ACCEL_OFFSET_STEP_DOWN = 0.04

# V2.5V regen-to-propulsion neutral handoff. The DNGA 0x273 normal mode
# covers both lowered-speed regen and positive propulsion, so prevent the
# requested speed from crossing directly from below vEgo to above vEgo.
V25V_REGEN_RELEASE_DWELL_FRAMES = 30  # 0.30 seconds at 100 Hz
V25V_REGEN_OFFSET_EPS = 0.01          # m/s
V25V_ACCEL_AEGO_MIN = -0.10           # wait until measured decel has faded
V25L_ACCEL_OFFSET_MAX = 0.55

# Legacy V2.5O no-lead hydraulic constants are retained for source history.
# V3.3R1 cannot enter this mode.
V25O_NOLEAD_BRAKE_ENTRY = 0.40
V25O_NOLEAD_BRAKE_RELEASE = 0.16
V25O_NOLEAD_BRAKE_MAX = 0.10
V25O_NOLEAD_ENTRY_FRAMES = 10
V25O_NOLEAD_MIN_SPEED = 3.0        # 10.8 km/h; never stop the car without a lead

# Legacy V2.5W curve-only hydraulic constants are retained for source history.
# V3.3R1 cannot enter this mode; its curve allowance is 0x273 regen only.
V25W_CURVE_BRAKE_ENTRY = 0.16
V25W_CURVE_BRAKE_RELEASE = 0.08
V25W_CURVE_ENTRY_FRAMES = 3       # 0.15 seconds at 20 Hz
V25W_CURVE_PID_ENTRY = 0.25

# Stock-observed stop-and-go states. 0x31 is used during the final crawl and
# 0x30 holds standstill at the minimum known brake command (0x04C3).
V25O_SNG_ARM_SPEED = 2.5            # 9.0 km/h
V25O_SNG_APPROACH_SPEED = 1.2       # 4.3 km/h
V25O_SNG_ARM_BRAKE = 0.18
V25O_SNG_HOLD_BRAKE = 0.05
V25O_SNG_RELEASE_ACCEL = 0.08
V25O_SNG_RELEASE_FRAMES = 5         # 0.25 seconds at 20 Hz

# V2.5U low-speed stopped-lead protection. This bypasses only the normal
# hydraulic reentry delay when the vehicle is already crawling toward a
# nearly stopped lead at close range.
V25U_STOP_LEAD_MAX_SPEED = 0.50       # estimated lead speed, m/s
V25U_STOP_LEAD_MAX_DISTANCE = 8.0     # metres
V25U_STOP_LEAD_MIN_CLOSING = 0.15     # m/s
V25U_STOP_LEAD_MIN_BRAKE = 0.08       # planner decel, m/s^2

# V2.5X-SC1 stop-completion-only correction.
#
# Once the existing stop-and-go path is armed and already in the final crawl
# toward a trusted stopped lead, do not let the hydraulic command taper below
# the level that failed to overcome vehicle creep in the 2026-07-31 log.
#
# These constants do not change hydraulic entry, following distance, regen,
# acceleration, resume, lead trust, or the standstill/0x30 transition.
V25X_SC_MAX_EGO_SPEED = 0.80           # 2.9 km/h: final crawl only
V25X_SC_MAX_LEAD_SPEED = 0.50          # lead is effectively stopped
V25X_SC_MAX_LEAD_DISTANCE = 8.0        # retain existing stopped-lead limit
V25X_SC_MIN_BRAKE = 0.18               # m/s^2 hydraulic completion floor

V25O_BRAKE_MODE_NONE = 0
V25O_BRAKE_MODE_LEAD = 1
V25O_BRAKE_MODE_NOLEAD = 2
V25W_BRAKE_MODE_CURVE = 3


def v25r_rate_limit_brake(target, last, urgent=False):
  step_up = V25R_BRAKE_STEP_UP_URGENT if urgent else V25R_BRAKE_STEP_UP
  if target > last:
    return min(target, last + step_up)
  return max(target, last - V25R_BRAKE_STEP_DOWN)


def v25r_normalize_plan_source(source):
  """Return a stable lowercase longitudinalPlan source name."""
  try:
    source = str(source).strip().lower()
  except Exception:
    return ""
  if "." in source:
    source = source.rsplit(".", 1)[-1]
  return source


def v25r_distance_profile(distance_val):
  """Return planner-entry, normal brake cap, and lead accel cap.

  CS.op_distance_val follows the local SetDistance enum:
    0 = aggressive / 1 bar
    1 = normal / 2 bars
    2 = far / 3 bars
  """
  try:
    distance_val = int(distance_val)
  except Exception:
    distance_val = 1

  # V2.5X evenly scaled bar profile:
  #   1 bar: closest, latest hydraulic entry, strongest normal authority
  #   2 bars: balanced midpoint
  #   3 bars: farthest, slightly earlier entry, gentlest normal authority
  #
  # Keep the current 3-bar hydraulic threshold at 0.16 so the previous
  # too-early 3-bar braking does not return. Scale 1 and 2 bars around it.
  if distance_val <= 0:
    return 0.18, 0.42, 0.12
  if distance_val >= 2:
    return 0.16, 0.36, 0.08
  return 0.17, 0.39, 0.10


def v25l_low_speed_brake_cap(v_ego):
  """Taper moving brake authority smoothly toward zero vehicle speed."""
  return float(interp(
    v_ego,
    [0.15, 0.30, 0.60, 1.00, 1.50, 2.50],
    [0.05, 0.08, 0.15, 0.25, 0.40, 0.75],
  ))


def v25l_high_speed_brake_cap(v_ego):
  """Keep hydraulic authority conservative at road/highway speed."""
  return float(interp(
    v_ego,
    [0.0, 15.0, 25.0, 35.0, 45.0],
    [0.65, 0.65, 0.60, 0.52, 0.48],
  ))


def v25w_curve_brake_cap(v_ego):
  """Moderate curve-only hydraulic authority selected by planner source."""
  return float(interp(
    v_ego,
    [3.0, 8.0, 15.0, 25.0, 35.0],
    [0.12, 0.17, 0.23, 0.28, 0.30],
  ))


def v25l_hydraulic_entry(v_ego):
  """Legacy speed-dependent hydraulic entry curve."""
  return float(interp(
    v_ego,
    [0.0, 4.0, 15.0, 25.0, 35.0],
    [0.25, 0.25, 0.38, 0.47, 0.52],
  ))


def v30_progressive_hydraulic_entry(v_ego):
  return float(interp(
    v_ego,
    [8.0, 15.0, 25.0, 35.0],
    [V30_NORMAL_ENTRY_MIN, 0.14, 0.17, V30_NORMAL_ENTRY_MAX],
  ))


def v30_progressive_brake_cap(v_ego):
  return float(interp(
    v_ego,
    [8.0, 15.0, 25.0, 35.0],
    [V30_NORMAL_BRAKE_CAP_MIN, 0.27, 0.29, V30_NORMAL_BRAKE_CAP_MAX],
  ))


def v25l_powertrain_decel_cap(v_ego):
  """Maximum 0x01 desired-speed offset below vEgo for regen/coasting."""
  return float(interp(
    v_ego,
    [0.0, 4.0, 15.0, 25.0, 35.0],
    [0.08, 0.15, 0.28, 0.40, 0.45],
  ))


def v25l_encode_hev_brake(brake_cmd):
  """Return (negative pump reaction, legacy combined raw magnitude)."""
  brake_cmd = float(clip(brake_cmd, V25L_BRAKE_MIN, V33R3_STOP_BRAKE_MAX))
  # Factory 0x271 uses FC/04 below 0.75 and FB/05 from 0.75 upward in the
  # recorded strong-stop envelope. Do not use the larger pump below that
  # observed transition.
  pump = 0.5 if brake_cmd >= V33R3_PUMP_05_THRESHOLD else 0.4

  magnitude_byte = int(round(200.0 - 100.0 * brake_cmd))
  magnitude_byte = int(clip(magnitude_byte, 0, 255))
  pump_byte = int(round(pump * 10.0))

  # Preserve the current DBC's 16-bit MAGNITUDE packing exactly.
  combined_magnitude = (pump_byte << 8) | magnitude_byte
  return -pump, combined_magnitude


def v33r3_relative_stop_request(v_ego, d_rel, closing_speed):
  """Bounded relative-motion request for a trusted slow/stopping lead.

  This is a fallback when the passive factory request is unavailable. It uses
  only the closing energy inside the remaining gap; it does not infer regen,
  friction pressure, or positive drive torque.
  """
  usable_distance = max(
    0.5,
    d_rel - V33R3_PREDICTIVE_STANDSTILL_GAP -
    V33R3_PREDICTIVE_REACTION_TIME * max(0.0, v_ego),
  )
  relative_decel = closing_speed * closing_speed / (2.0 * usable_distance)
  return float(clip(
    relative_decel,
    V33R3_PREDICTIVE_INITIAL_BRAKE,
    V33R3_STOP_BRAKE_MAX,
  ))


def apply_dnga_steer_torque_limits(apply_torque, apply_torque_last, driver_torque, blinkerOn, LIMITS):
  reduced_torque_mult = 10 if blinkerOn else 1.5

  driver_max_torque = 255 + driver_torque * reduced_torque_mult
  driver_min_torque = -255 - driver_torque * reduced_torque_mult

  max_steer_allowed = clip(driver_max_torque, 0, 255)
  min_steer_allowed = clip(driver_min_torque, -255, 0)

  apply_torque = clip(apply_torque, min_steer_allowed, max_steer_allowed)

  if apply_torque_last > 0:
    apply_torque = clip(
      apply_torque,
      max(apply_torque_last - LIMITS.STEER_DELTA_DOWN, -LIMITS.STEER_DELTA_UP),
      apply_torque_last + LIMITS.STEER_DELTA_UP
    )
  else:
    apply_torque = clip(
      apply_torque,
      apply_torque_last - LIMITS.STEER_DELTA_UP,
      min(apply_torque_last + LIMITS.STEER_DELTA_DOWN, LIMITS.STEER_DELTA_UP)
    )

  return int(round(float(apply_torque)))


class CarControllerParams():
  def __init__(self, CP):
    self.STEER_BP = CP.lateralParams.torqueBP
    self.STEER_LIM_TORQ = CP.lateralParams.torqueV

    if CP.carFingerprint in NOT_CAN_CONTROLLED:
      self.STEER_DELTA_UP = 20
      self.STEER_DELTA_DOWN = 30
    else:
      self.STEER_DELTA_UP = 22
      self.STEER_DELTA_DOWN = 35


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP, CP_SP):
    super().__init__(dbc_names, CP, CP_SP)
    self.last_steer = 0
    self.steer_rate_limited = False

    self.params = CarControllerParams(CP)
    self.packer = CANPacker(dbc_names[Bus.pt])

    self.stockLdw = False

    self.prev_enabled = False
    self.block_brake_until_frame = 0

    # V2.5O hydraulic state, stop-and-go latch, and continuous desired-speed
    # offset for acceleration, coasting, and regenerative deceleration.
    self.v25l_apply_brake = 0.0
    self.v25l_brake_target = 0.0
    self.v25l_brake_active = False
    self.v25l_brake_entry_counter = 0
    self.v25o_nolead_entry_counter = 0
    self.v25o_brake_mode = V25O_BRAKE_MODE_NONE
    self.v25o_sng_armed = False
    self.v25o_stop_hold = False
    self.v25o_sng_release_frames = 0
    self.v25l_lead_counter = 0
    self.v25l_brake_reentry_frame = 0
    self.v25l_propulsion_block_until_frame = 0
    self.v25l_speed_offset = 0.0
    self.v25v_regen_release_until_frame = 0

    # V2.5R planner-primary handoff state. Subscribers are non-blocking and
    # avoid changing DragonPilot's CarController.update call signature.
    self.v25r_release_counter = 0
    self.v25r_lead_accel_counter = 0
    self.v25r_lead_loss_counter = 0
    self.v25r_urgent_brake = False
    self.v30_handoff_counter = 0
    self.v30_handoff_active = False

    # V3.2R low-speed-only handoff state.
    self.v32r_neutral_until_frame = 0
    self.v32r_low_speed_arm_start_frame = -1000000
    self.v32r_overshoot_block_until_frame = 0

    self.v33r_low_speed_guard_until_frame = 0
    self.v33r_stopped_lead_counter = 0
    self.v33r_hold_resume_counter = 0
    self.v33r_release_freeze_until_frame = 0
    self.v33r_release_lead_until_frame = 0
    self.v33r_target_slope_unlock_frame = 0
    self.v33r_overshoot_counter = 0
    self.v33r_overshoot_block_until_frame = 0
    self.v33r_filtered_aego = 0.0
    self.v33r_curve_regen_counter = 0

    # V3.3R2 handoff state. Once set, the deceleration latch is cleared only
    # by fresh, sustained positive intent after all braking/release stages and
    # measured deceleration have ended.
    self.v33r2_decel_latched = False
    self.v33r2_decel_clear_counter = 0
    self.v33r2_release_pump_until_frame = 0

    # V3.3R3 stop-guard state. The entry counter filters the radar-only
    # fallback; checksum-validated factory braking may enter immediately.
    self.v33r3_predictive_entry_counter = 0
    self.v33r3_stop_guard_latched = False

    # V3.3R4 read-only hybrid-feedback supervisor. Fault state survives an
    # ordinary disengagement and is cleared only by a new, feedback-clean
    # engagement edge (fresh SET/RES through the existing cruise latch).
    self.v33r4_fault_latched = False
    self.v33r4_fault_reason = ""
    self.v33r4_feedback_disagree_counter = 0
    self.v33r4_brake_clear_counter = 0
    self.v33r4_torque_ready_counter = 0
    self.v33r4_decel_entry_frame = -1000000
    self.v33r4_decel_entry_torque = 0
    self.v33r4_decel_torque_cleared = False
    self.v33r4_positive_overlap_counter = 0

    self.v25r_plan_source = ""
    self.v25r_plan_accel = 0.0
    self.v25r_plan_accel_next = 0.0
    self.v25r_plan_has_lead = False
    self.v25r_plan_frame = -1000000

    self.v25r_lead0_status = False
    self.v25r_lead0_drel = 0.0
    self.v25r_lead0_vrel = 0.0
    self.v25r_lead1_status = False
    self.v25r_lead1_drel = 0.0
    self.v25r_lead1_vrel = 0.0
    self.v25r_radar_frame = -1000000

    try:
      self.v25r_plan_sm = messaging.SubMaster(["longitudinalPlan"])
    except Exception:
      self.v25r_plan_sm = None

    try:
      self.v25r_radar_sm = messaging.SubMaster(["radarState"])
    except Exception:
      self.v25r_radar_sm = None

  def _v33r4_latch_fault(self, CS, reason):
    """Fail non-propulsive and require the existing SET/RES latch to re-arm."""
    self.v33r4_fault_latched = True
    self.v33r4_fault_reason = str(reason)
    self.v33r4_brake_clear_counter = 0
    self.v33r4_torque_ready_counter = 0
    self.v33r4_positive_overlap_counter = 0
    self.v25l_speed_offset = 0.0
    if hasattr(CS, "is_cruise_latch"):
      CS.is_cruise_latch = False
    CS.hybrid_feedback_fault = True
    CS.hybrid_feedback_fault_reason = self.v33r4_fault_reason

  def update(self, CC, CC_SP, CS, now_nanos):
    del CC_SP, now_nanos
    enabled = CC.enabled
    active = CC.latActive
    long_active = CC.longActive
    frame = self.frame
    actuators = CC.actuators
    pcm_cancel_cmd = CC.cruiseControl.cancel
    hud_control = CC.hudControl
    hud_alert = hud_control.visualAlert
    left_line = hud_control.leftLaneVisible
    right_line = hud_control.rightLaneVisible
    lead = hud_control.leadVisible
    left_lane_depart = hud_control.leftLaneDepart
    right_lane_depart = hud_control.rightLaneDepart
    dragonconf = None

    can_sends = []  # Create the list that will hold all CAN messages for this control cycle

    # -----------------------------
    # Steering
    # -----------------------------
    steer_max_interp = interp(CS.out.vEgo, self.params.STEER_BP, self.params.STEER_LIM_TORQ)  # Get speed-based steering torque limit
    steer_max_interp = max(1.0, steer_max_interp)  # Avoid divide-by-zero if the torque table returns 0

    new_steer = int(round(actuators.torque * steer_max_interp))  # Convert normalized OP steer command into raw torque

    isBlinkerOn = CS.out.leftBlinker != CS.out.rightBlinker  # True when only one blinker is active

    apply_steer = apply_dnga_steer_torque_limits(  # Apply steering torque and rate limits
      new_steer,  # Requested raw steering torque
      self.last_steer,  # Last applied raw steering torque
      CS.out.steeringTorqueEps,  # Driver torque from EPS
      isBlinkerOn,  # Blinker state for driver override allowance
      self.params  # Steering limit parameters
    )

    if not active and not self.stockLdw:
      apply_steer = 0

    self.steer_rate_limited = (new_steer != apply_steer) and (apply_steer != 0)  # Mark whether steering was rate limited
    self.steer_rate_limited &= not CS.out.steeringPressed  # Do not show rate limit if driver is steering

    # -----------------------------
    # Longitudinal base values
    # -----------------------------
    apply_accel = clip(actuators.accel, -3.0, 1.5)  # Clamp openpilot accel request
    apply_brake = -apply_accel if apply_accel < 0.0 else 0.0  # Convert negative accel into positive brake amount

    engagement_edge = enabled and not self.prev_enabled
    if engagement_edge:  # Detect the first frame after OP engagement
      self.block_brake_until_frame = frame + 50  # Let engagement settle for 0.5 seconds
      self.v25l_apply_brake = 0.0
      self.v25l_brake_target = 0.0
      self.v25l_brake_active = False
      self.v25l_brake_entry_counter = 0
      self.v25o_nolead_entry_counter = 0
      self.v25o_brake_mode = V25O_BRAKE_MODE_NONE
      self.v25o_sng_armed = False
      self.v25o_stop_hold = False
      self.v25o_sng_release_frames = 0
      self.v25l_lead_counter = 0
      self.v25l_brake_reentry_frame = frame
      self.v25l_propulsion_block_until_frame = frame
      self.v25l_speed_offset = 0.0
      self.v25v_regen_release_until_frame = frame
      self.v25r_release_counter = 0
      self.v25r_lead_accel_counter = 0
      self.v25r_lead_loss_counter = 0
      self.v25r_urgent_brake = False
      self.v30_handoff_counter = 0
      self.v30_handoff_active = False
      self.v32r_neutral_until_frame = frame
      self.v32r_low_speed_arm_start_frame = -1000000
      self.v32r_overshoot_block_until_frame = frame
      self.v33r_low_speed_guard_until_frame = (
        frame + V33R_LOW_SPEED_ENGAGEMENT_GUARD_FRAMES
      )
      self.v33r_stopped_lead_counter = 0
      self.v33r_hold_resume_counter = 0
      self.v33r_release_freeze_until_frame = frame
      self.v33r_release_lead_until_frame = frame
      self.v33r_target_slope_unlock_frame = frame
      self.v33r_overshoot_counter = 0
      self.v33r_overshoot_block_until_frame = frame
      self.v33r_filtered_aego = float(CS.out.aEgo)
      self.v33r_curve_regen_counter = 0
      self.v33r2_decel_latched = False
      self.v33r2_decel_clear_counter = 0
      self.v33r2_release_pump_until_frame = frame
      self.v33r3_predictive_entry_counter = 0
      self.v33r3_stop_guard_latched = False
      self.v33r4_feedback_disagree_counter = 0
      self.v33r4_brake_clear_counter = 0
      self.v33r4_torque_ready_counter = 0
      self.v33r4_decel_entry_frame = -1000000
      self.v33r4_decel_entry_torque = 0
      self.v33r4_decel_torque_cleared = False
      self.v33r4_positive_overlap_counter = 0

    self.prev_enabled = enabled  # Save enabled state for the next control cycle

    # -----------------------------
    # Steering command, 50 Hz
    # -----------------------------
    if (frame % 2) == 0:  # Send steering every 2 frames
      steer_req = (active or self.stockLdw) and CS.lkas_latch  # Request steering when OP/LDA is active and LKAS latch is on
      can_sends.append(  # Add steering CAN message
        create_can_steer_command(  # Build STEERING_LKAS frame
          self.packer,  # CAN packer
          apply_steer,  # Raw torque command
          steer_req,  # Steering request bit
          (frame // 2) % 16  # Steering counter
        )
      )

    # -----------------------------
    # Longitudinal / HUD, 20 Hz
    # -----------------------------
    if (frame % 5) == 0:  # Send ACC/HUD messages every 5 frames

      t_lookup = 0.35 + 0.07 * CS.out.vEgo

      # Read longitudinalPlan. It is the primary longitudinal direction
      # request; the downstream PID may only add a small same-direction amount.
      if self.v25r_plan_sm is not None:
        try:
          self.v25r_plan_sm.update(0)
          if self.v25r_plan_sm.updated["longitudinalPlan"]:
            long_plan = self.v25r_plan_sm["longitudinalPlan"]
            self.v25r_plan_source = v25r_normalize_plan_source(
              getattr(long_plan, "longitudinalPlanSource", "")
            )
            self.v25r_plan_has_lead = bool(
              getattr(long_plan, "hasLead", False)
            )
            plan_accels = getattr(long_plan, "accels", [])
            if len(plan_accels) > 0:
              self.v25r_plan_accel = float(plan_accels[0])
              self.v25r_plan_accel_next = (
                float(plan_accels[1]) if len(plan_accels) > 1
                else self.v25r_plan_accel
              )
            self.v25r_plan_frame = frame
        except Exception:
          pass

      # Radar values are used only to classify urgency. Planner source remains
      # the authority for whether a lead is longitudinally relevant.
      if self.v25r_radar_sm is not None:
        try:
          self.v25r_radar_sm.update(0)
          if self.v25r_radar_sm.updated["radarState"]:
            radar_state = self.v25r_radar_sm["radarState"]
            lead0 = radar_state.leadOne
            lead1 = radar_state.leadTwo
            self.v25r_lead0_status = bool(lead0.status)
            self.v25r_lead0_drel = float(lead0.dRel)
            self.v25r_lead0_vrel = float(lead0.vRel)
            self.v25r_lead1_status = bool(lead1.status)
            self.v25r_lead1_drel = float(lead1.dRel)
            self.v25r_lead1_vrel = float(lead1.vRel)
            self.v25r_radar_frame = frame
        except Exception:
          pass

      plan_fresh = (
        frame - self.v25r_plan_frame <= V25R_PLAN_MAX_AGE_FRAMES
      )
      radar_fresh = (
        frame - self.v25r_radar_frame <= V25R_RADAR_MAX_AGE_FRAMES
      )
      planner_source_lead = (
        plan_fresh and self.v25r_plan_source in ("lead0", "lead1")
      )
      # V2.5T: longitudinalPlanSource identifies which trajectory currently
      # limits the MPC, not whether a lead exists. The MPC can legitimately
      # report source="cruise" while hasLead remains true. Treat hasLead as the
      # lead-presence authority and retain source only for selecting lead0/lead1.
      planner_reports_lead = (
        plan_fresh and
        (self.v25r_plan_has_lead or planner_source_lead)
      )

      # Blend current and next planner acceleration only for anticipation.
      planner_accel_request = (
        0.70 * self.v25r_plan_accel +
        0.30 * self.v25r_plan_accel_next
        if plan_fresh else apply_accel
      )
      planner_brake_request = max(0.0, -planner_accel_request)

      if (
        self.v25r_plan_source == "lead1" and
        self.v25r_lead1_status
      ):
        selected_lead_status = True
        selected_lead_drel = self.v25r_lead1_drel
        selected_lead_vrel = self.v25r_lead1_vrel
      else:
        # Camera-only logs can select lead1 while radarState.leadOne remains
        # the only populated estimate. Use leadOne for urgency in that case.
        selected_lead_status = self.v25r_lead0_status
        selected_lead_drel = self.v25r_lead0_drel
        selected_lead_vrel = self.v25r_lead0_vrel

      if not radar_fresh:
        selected_lead_status = bool(lead)
        selected_lead_drel = 0.0
        selected_lead_vrel = 0.0

      closing_speed = max(0.0, -selected_lead_vrel)
      ttc = (
        selected_lead_drel / closing_speed
        if selected_lead_status and closing_speed > 0.1
        else 999.0
      )

      selected_lead_speed = max(
        0.0,
        CS.out.vEgo + selected_lead_vrel,
      )

      # Qualify camera leads before allowing hydraulic 0x21.
      if lead:
        self.v25l_lead_counter = min(
          self.v25l_lead_counter + 1,
          V25L_LEAD_TRUST_FRAMES,
        )
      else:
        self.v25l_lead_counter = 0

      trusted_lead = self.v25l_lead_counter >= V25L_LEAD_TRUST_FRAMES
      urgent_lead = self.v25l_lead_counter >= V25L_URGENT_LEAD_FRAMES

      if lead:
        self.v25r_lead_loss_counter = 0
      else:
        self.v25r_lead_loss_counter = min(
          self.v25r_lead_loss_counter + 1,
          V25R_LEAD_LOSS_FRAMES,
        )

      # V2.5T: hasLead establishes lead relevance. longitudinalPlanSource can
      # flicker between lead0/lead1/cruise while the same physical lead remains
      # valid, so using source as a gate can suppress hydraulic braking and SNG.
      relevant_lead = (
        trusted_lead and planner_reports_lead
        if plan_fresh else trusted_lead
      )
      relevant_urgent_lead = (
        urgent_lead and planner_reports_lead
        if plan_fresh else urgent_lead
      )
      nolead_planner_context = (
        not planner_reports_lead if plan_fresh else not trusted_lead
      )
      curve_planner_context = (
        plan_fresh and
        not planner_reports_lead and
        self.v25r_plan_source == "turn"
      )

      distance_val = int(clip(
        getattr(CS, "op_distance_val", 1),
        0,
        2,
      ))
      lead_entry_planner, lead_normal_cap, lead_accel_cap = (
        v25r_distance_profile(distance_val)
      )
      lead_hydraulic_entry = lead_entry_planner
      if CS.out.vEgo >= V30_HIGHWAY_MIN_SPEED:
        lead_hydraulic_entry = max(
          lead_entry_planner,
          v30_progressive_hydraulic_entry(CS.out.vEgo),
        )

      # The same setting is now delivered to the MPC by
      # carState.distanceLines; no Params write is needed here.

      v33r_emergency_closing = (
        relevant_lead and
        selected_lead_status and
        closing_speed >= V33R_EMERGENCY_CLOSING_SPEED and
        ttc <= V33R_EMERGENCY_TTC and
        planner_brake_request >= V33R_EMERGENCY_PLANNER_BRAKE
      )
      urgent_closing = (
        relevant_urgent_lead and
        (
          planner_brake_request >= V25R_URGENT_PLANNER_DECEL or
          (
            closing_speed >= V25R_URGENT_CLOSING_SPEED and
            ttc <= V25R_URGENT_TTC
          )
        )
      ) or v33r_emergency_closing

      stopped_lead_approach = (
        relevant_lead and
        selected_lead_status and
        CS.out.vEgo <= V25O_SNG_ARM_SPEED and
        0.0 < selected_lead_drel <= V25U_STOP_LEAD_MAX_DISTANCE and
        selected_lead_speed <= V25U_STOP_LEAD_MAX_SPEED and
        closing_speed >= V25U_STOP_LEAD_MIN_CLOSING
      )

      # V2.5X-SC1: after the normal logic has already armed stop-and-go, keep a
      # narrowly scoped final-crawl qualifier that does not require a minimum
      # closing speed. The old closing-speed condition naturally becomes false
      # near zero and could let braking taper before physical standstill.
      stop_completion_active = (
        self.v25o_sng_armed and
        self.v25o_brake_mode == V25O_BRAKE_MODE_LEAD and
        relevant_lead and
        selected_lead_status and
        not CS.out.standstill and
        CS.out.vEgo <= V25X_SC_MAX_EGO_SPEED and
        0.0 < selected_lead_drel <= V25X_SC_MAX_LEAD_DISTANCE and
        selected_lead_speed <= V25X_SC_MAX_LEAD_SPEED
      )

      stopped_lead_candidate = (
        selected_lead_status and
        bool(lead) and
        (planner_reports_lead if plan_fresh else True) and
        V33R_STOP_HOLD_MIN_DISTANCE <= selected_lead_drel <=
          V33R_STOP_HOLD_MAX_DISTANCE and
        selected_lead_speed <= V33R_STOPPED_LEAD_MAX_SPEED
      )
      if stopped_lead_candidate:
        self.v33r_stopped_lead_counter = min(
          self.v33r_stopped_lead_counter + 1,
          V33R_STOP_LEAD_TRUST_FRAMES,
        )
      else:
        self.v33r_stopped_lead_counter = 0

      v33r_trusted_stopped_lead = (
        self.v33r_stopped_lead_counter >=
        V33R_STOP_LEAD_TRUST_FRAMES
      )
      v33r_creep_stop_guard = (
        v33r_trusted_stopped_lead and
        not CS.out.standstill and
        V33R_CREEP_GUARD_MIN_EGO < CS.out.vEgo <=
          V33R_CREEP_GUARD_MAX_EGO and
        closing_speed >= V33R_CREEP_GUARD_MIN_CLOSING
      )
      v33r_early_highway_entry = (
        relevant_lead and
        selected_lead_status and
        CS.out.vEgo >= V33R_EARLY_HIGHWAY_MIN_SPEED and
        closing_speed >= V33R_EARLY_HIGHWAY_CLOSING and
        ttc <= V33R_EARLY_HIGHWAY_TTC and
        selected_lead_drel <= max(
          35.0,
          CS.out.vEgo * V33R_EARLY_HIGHWAY_TIME_GAP,
        ) and
        planner_brake_request >= V33R_EARLY_HIGHWAY_PLANNER_BRAKE
      )

      self.v33r_filtered_aego += V33R_AEGO_FILTER_ALPHA * (
        float(CS.out.aEgo) - self.v33r_filtered_aego
      )
      v33r_critical_closing = (
        selected_lead_status and
        closing_speed >= V33R_DECEL_GOVERNOR_CRITICAL_CLOSING and
        ttc <= V33R_DECEL_GOVERNOR_CRITICAL_TTC
      )

      # -----------------------------
      # ACC_BRAKE / 0x271 state map from logs
      # -----------------------------
      # 0x00 + pump 0.0 + mag 200 = disabled / neutral / no brake
      # 0x01 + pump 0.0 + mag 200 = enabled / ready / no brake
      # 0x21 + pump -0.4 + mag 0x04xx = active braking request
      # 0x31 = final stop approach; 0x30 = standstill hold (verified in rlogs)

      if not enabled:  # OP is not engaged
        brake_state = 0x00  # Use disabled/neutral brake state
        pump_reaction = 0.0  # No pump reaction
        brake_mag = 200  # Stock neutral magnitude 0x00C8

      else:  # OP is engaged but not necessarily braking
        brake_state = 0x01  # Use enabled/no-brake state found in logs
        pump_reaction = 0.0  # No pump reaction while not braking
        brake_mag = 200  # Stock neutral magnitude 0x00C8

      # -----------------------------
      # V2.5R planner-primary hydraulic braking and stop-and-go
      # -----------------------------
      brake_request = apply_brake

      base_control_allowed = (
        enabled and
        long_active and
        CS.out.cruiseState.enabled and
        not pcm_cancel_cmd and
        not CS.out.gasPressed and
        not CS.out.brakePressed
      )
      r4_feedback = hybrid_feedback_snapshot(CS, frame)
      r4_feedback_clean = (
        r4_feedback["fresh"] and r4_feedback["consistent"]
      )
      r4_rearm_ok = (
        r4_feedback_clean and
        r4_feedback["brakes_clear"] and
        r4_feedback["torque_ramp_ready"] and
        not r4_feedback["positive_vote"]
      )

      # A fault is not cleared by timers or by the stale outer enabled flag.
      # A new SET/RES engagement edge may clear it only while all observed
      # feedback is fresh, mutually consistent, brake-clear, and non-positive.
      if engagement_edge and self.v33r4_fault_latched:
        if r4_rearm_ok:
          self.v33r4_fault_latched = False
          self.v33r4_fault_reason = ""
        else:
          self._v33r4_latch_fault(CS, "feedback_not_safe_to_rearm")

      if base_control_allowed and not r4_feedback["fresh"]:
        self._v33r4_latch_fault(CS, "hybrid_feedback_stale")
      elif base_control_allowed and not r4_feedback["consistent"]:
        self.v33r4_feedback_disagree_counter = min(
          self.v33r4_feedback_disagree_counter + 1,
          V33R4_DISAGREE_FAULT_FRAMES,
        )
        if (
          self.v33r4_feedback_disagree_counter >=
          V33R4_DISAGREE_FAULT_FRAMES
        ):
          self._v33r4_latch_fault(CS, "hybrid_torque_feedback_disagrees")
      else:
        self.v33r4_feedback_disagree_counter = 0

      control_allowed = (
        base_control_allowed and not self.v33r4_fault_latched
      )
      CS.hybrid_feedback_fault = self.v33r4_fault_latched
      CS.hybrid_feedback_fault_reason = self.v33r4_fault_reason
      moving_allowed = control_allowed and not CS.out.standstill

      # Passive factory-camera request observer. interface.py accepts only
      # checksum-valid bus-2 frames and timestamps them with this same 100 Hz
      # frame counter. A valid pair is brake-only evidence; it is never used to
      # enable control, clear a driver override, or request acceleration.
      stock_brake_rx_frame = int(getattr(
        CS, "stock_acc_brake_rx_frame", -1000000
      ))
      stock_acc_rx_frame = int(getattr(
        CS, "stock_acc_request_rx_frame", -1000000
      ))
      stock_brake_fresh = (
        0 <= frame - stock_brake_rx_frame <=
        V33R3_STOCK_FRAME_MAX_AGE
      )
      stock_acc_fresh = (
        0 <= frame - stock_acc_rx_frame <=
        V33R3_STOCK_FRAME_MAX_AGE
      )
      stock_brake_request = float(clip(
        getattr(CS, "stock_acc_brake_decel", 0.0),
        0.0,
        V33R3_STOP_BRAKE_MAX,
      ))
      stock_brake_pair_valid = (
        stock_brake_fresh and
        stock_acc_fresh and
        int(getattr(CS, "stock_acc_brake_state", 0)) == 0x21 and
        bool(getattr(CS, "stock_acc_request_enabled", False)) and
        bool(getattr(CS, "stock_acc_request_lead", False)) and
        bool(getattr(CS, "stock_acc_request_is_decel", False)) and
        not bool(getattr(CS, "stock_acc_request_is_accel", False)) and
        stock_brake_request >= V33R3_STOP_GUARD_MIN_STOCK_BRAKE
      )
      v33r3_stock_brake_context = (
        moving_allowed and
        stock_brake_pair_valid and
        relevant_lead and
        selected_lead_status and
        CS.out.vEgo <= V33R3_STOP_GUARD_MAX_SPEED and
        0.0 < selected_lead_drel <= V33R3_STOP_GUARD_MAX_DISTANCE and
        closing_speed >= V33R3_STOP_GUARD_MIN_CLOSING
      )
      v33r3_stock_brake_entry = (
        v33r3_stock_brake_context and
        brake_request >= V33R3_STOP_GUARD_MIN_PID_BRAKE
      )
      v33r3_stock_brake_guard = (
        v33r3_stock_brake_context and
        (
          v33r3_stock_brake_entry or
          self.v33r3_stop_guard_latched
        )
      )

      # Radar-only fallback for the same failure class. Entry requires a
      # trusted selected lead, low road speed, bounded geometry, and an already
      # negative downstream command. After entry, relative stopping energy may
      # keep increasing the brake request even if PI later winds positive under
      # the still-active negative wheel torque.
      v33r3_predictive_stop_context = (
        moving_allowed and
        relevant_lead and
        selected_lead_status and
        CS.out.vEgo <= V33R3_STOP_GUARD_MAX_SPEED and
        0.0 < selected_lead_drel <= V33R3_STOP_GUARD_MAX_DISTANCE and
        closing_speed >= V33R3_PREDICTIVE_MIN_CLOSING and
        ttc <= V33R3_PREDICTIVE_MAX_TTC and
        selected_lead_speed <= V33R3_PREDICTIVE_MAX_LEAD_SPEED
      )
      v33r3_predictive_stop_entry = (
        v33r3_predictive_stop_context and
        brake_request >= V33R3_STOP_GUARD_MIN_PID_BRAKE
      )
      if v33r3_predictive_stop_entry:
        self.v33r3_predictive_entry_counter = min(
          self.v33r3_predictive_entry_counter + 1,
          V33R3_PREDICTIVE_ENTRY_FRAMES,
        )
      else:
        self.v33r3_predictive_entry_counter = 0
      v33r3_predictive_stop_confirmed = (
        self.v33r3_predictive_entry_counter >=
        V33R3_PREDICTIVE_ENTRY_FRAMES
      )
      v33r3_stop_guard_entry = (
        v33r3_stock_brake_entry or
        v33r3_predictive_stop_confirmed
      )
      v33r3_relative_brake_request = (
        v33r3_relative_stop_request(
          CS.out.vEgo,
          selected_lead_drel,
          closing_speed,
        )
        if v33r3_predictive_stop_context else
        0.0
      )
      v33r3_stop_guard_request = (
        stock_brake_request
        if v33r3_stock_brake_guard else
        v33r3_relative_brake_request
      )
      v33r3_stop_completion_guard = (
        self.v33r3_stop_guard_latched and
        relevant_lead and
        selected_lead_status and
        not CS.out.standstill and
        CS.out.vEgo <= V25O_SNG_ARM_SPEED and
        0.0 < selected_lead_drel <=
          V33R3_STOP_COMPLETION_MAX_DISTANCE and
        selected_lead_speed <=
          V33R3_STOP_COMPLETION_MAX_LEAD_SPEED
      )
      v33r3_stop_guard_authority = (
        v33r3_stock_brake_guard or
        v33r3_predictive_stop_context or
        v33r3_stop_completion_guard
      )

      def start_v33r_staged_release():
        # Copy the stock-observed moving release sequence. Keep the pump
        # reaction at FC/04/C8 and retain deceleration mode for the full
        # observed 1.2-second pressure-release interval.
        self.v33r2_decel_latched = True
        self.v33r2_decel_clear_counter = 0
        self.v33r2_release_pump_until_frame = max(
          self.v33r2_release_pump_until_frame,
          frame + V33R2_RELEASE_PUMP_FRAMES,
        )
        self.v33r_release_freeze_until_frame = max(
          self.v33r_release_freeze_until_frame,
          frame + V33R_RELEASE_FREEZE_FRAMES,
        )
        self.v33r_release_lead_until_frame = max(
          self.v33r_release_lead_until_frame,
          frame + V33R_RELEASE_LEAD_HOLD_FRAMES,
        )
        self.v33r_target_slope_unlock_frame = max(
          self.v33r_target_slope_unlock_frame,
          frame + V33R_TARGET_SLOPE_UNLOCK_FRAMES,
        )
        self.v25l_propulsion_block_until_frame = max(
          self.v25l_propulsion_block_until_frame,
          frame + V33R_RELEASE_PROPULSION_BLOCK_FRAMES,
        )
        self.v25l_speed_offset = min(0.0, self.v25l_speed_offset)

      def clear_hydraulic(reentry=True, propulsion_dwell=True):
        self.v25l_apply_brake = 0.0
        self.v25l_brake_target = 0.0
        self.v25l_brake_active = False
        self.v25l_brake_entry_counter = 0
        self.v25o_nolead_entry_counter = 0
        self.v25o_brake_mode = V25O_BRAKE_MODE_NONE
        self.v25r_release_counter = 0
        self.v25r_urgent_brake = False
        self.v30_handoff_counter = 0
        self.v30_handoff_active = False
        self.v33r3_predictive_entry_counter = 0
        self.v33r3_stop_guard_latched = False
        if reentry:
          # The old 0.80-second blind block overlapped the logged relaunches.
          # Keep only a short transition guard so renewed negative planner
          # demand can restore hydraulic authority promptly.
          self.v25l_brake_reentry_frame = (
            frame + V33R2_REENTRY_BLOCK_FRAMES
          )
        if propulsion_dwell:
          self.v25l_propulsion_block_until_frame = (
            frame + V25L_PROPULSION_DWELL_FRAMES
          )

      # Release 0x30 only when the lead itself moves and the planner remains
      # positive. A transient downstream PID output cannot release the hold.
      if self.v25o_stop_hold:
        hold_safety_exit = not control_allowed
        hold_resume_candidate = (
          selected_lead_status and
          bool(lead) and
          (planner_reports_lead if plan_fresh else True) and
          selected_lead_speed >= V33R_HOLD_RESUME_LEAD_SPEED and
          (
            planner_accel_request >= V25O_SNG_RELEASE_ACCEL
            if plan_fresh else
            apply_accel >= V25O_SNG_RELEASE_ACCEL
          )
        )
        if hold_resume_candidate:
          self.v33r_hold_resume_counter = min(
            self.v33r_hold_resume_counter + 1,
            V33R_HOLD_RESUME_FRAMES,
          )
        else:
          self.v33r_hold_resume_counter = 0

        hold_resume = (
          self.v33r_hold_resume_counter >= V33R_HOLD_RESUME_FRAMES
        )

        if hold_safety_exit or hold_resume:
          self.v25o_stop_hold = False
          self.v33r_hold_resume_counter = 0
          clear_hydraulic(reentry=False, propulsion_dwell=False)
          if hold_safety_exit:
            self.v25o_sng_armed = False
            self.v25o_sng_release_frames = 0
            self.v33r_release_lead_until_frame = frame
            self.v33r_release_freeze_until_frame = frame
          else:
            self.v25o_sng_release_frames = V25O_SNG_RELEASE_FRAMES
            start_v33r_staged_release()
        else:
          self.v25l_brake_active = True
          self.v25o_brake_mode = V25O_BRAKE_MODE_LEAD
          self.v25l_apply_brake = V25O_SNG_HOLD_BRAKE
          self.v25l_brake_target = V25O_SNG_HOLD_BRAKE
          self.v25l_speed_offset = 0.0

      sng_release_active = False
      if self.v25o_sng_release_frames > 0:
        if not control_allowed:
          self.v25o_sng_release_frames = 0
          self.v25o_sng_armed = False
        else:
          sng_release_active = True
          self.v25o_sng_release_frames -= 1
          self.v25l_brake_active = False
          self.v25o_brake_mode = V25O_BRAKE_MODE_NONE
          self.v25l_apply_brake = 0.0
          self.v25l_brake_target = 0.0
          self.v25l_speed_offset = 0.0

      planner_allows_hold = (
        planner_accel_request < V25O_SNG_RELEASE_ACCEL
        if plan_fresh else
        apply_accel < V25O_SNG_RELEASE_ACCEL
      )
      direct_standstill_hold = (
        control_allowed and
        CS.out.standstill and
        v33r_trusted_stopped_lead and
        planner_allows_hold
      )
      approached_standstill_hold = (
        self.v25o_sng_armed and
        CS.out.standstill and
        control_allowed and
        planner_allows_hold
      )
      if (
        not sng_release_active and
        not self.v25o_stop_hold and
        (direct_standstill_hold or approached_standstill_hold)
      ):
        self.v25o_stop_hold = True
        self.v25o_sng_armed = True
        self.v25l_brake_active = True
        self.v25o_brake_mode = V25O_BRAKE_MODE_LEAD
        self.v25l_apply_brake = V25O_SNG_HOLD_BRAKE
        self.v25l_brake_target = V25O_SNG_HOLD_BRAKE
        self.v25l_speed_offset = 0.0
        self.v33r_release_lead_until_frame = max(
          self.v33r_release_lead_until_frame,
          frame + V33R_RELEASE_LEAD_HOLD_FRAMES,
        )

      soft_releasing_hydraulic = False
      hydraulic_handoff_release = False

      if not sng_release_active and not self.v25o_stop_hold and self.v25l_brake_active:
        safety_hard_release = not moving_allowed
        force_soft_release = False

        v30_handoff_candidate = (
          self.v25o_brake_mode == V25O_BRAKE_MODE_LEAD and
          CS.out.vEgo >= V30_HIGHWAY_MIN_SPEED and
          plan_fresh and
          relevant_lead and
          not urgent_closing and
          not self.v25r_urgent_brake and
          not self.v25o_sng_armed and
          not stop_completion_active and
          planner_accel_request >= V33R2_DECEL_CLEAR_PLANNER_ACCEL and
          apply_accel >= V33R2_DECEL_CLEAR_PID_ACCEL and
          CS.out.aEgo >= V33R2_DECEL_CLEAR_AEGO
        )

        if (
          v30_handoff_candidate and
          not self.v30_handoff_active
        ):
          self.v30_handoff_counter = min(
            self.v30_handoff_counter + 1,
            V30_HANDOFF_FRAMES,
          )
          if (
            self.v30_handoff_counter >=
            V30_HANDOFF_FRAMES
          ):
            self.v30_handoff_active = True
        elif not self.v30_handoff_active:
          self.v30_handoff_counter = 0

        if self.v30_handoff_active:
          # Complete the hydraulic release only after planner, downstream PID,
          # and measured motion all agree braking has ended. V3.0's previous
          # PI-positive / still-decelerating trigger is intentionally gone.
          force_soft_release = True
          hydraulic_handoff_release = True

        if self.v25o_brake_mode == V25O_BRAKE_MODE_LEAD:
          # Raw lead loss is a safety exit. A downstream positive PID request
          # is intentionally ignored while the lead planner still wants decel.
          if self.v25r_lead_loss_counter >= V25R_LEAD_LOSS_FRAMES:
            safety_hard_release = True
          elif plan_fresh and not planner_reports_lead:
            self.v25o_sng_armed = False
            safety_hard_release = True

        elif self.v25o_brake_mode in (
          V25O_BRAKE_MODE_NOLEAD,
          V25W_BRAKE_MODE_CURVE,
        ):
          # These legacy modes are prohibited in V3.3R1. Release immediately
          # if one is ever observed after a controller hot-reload/restart.
          safety_hard_release = True

        if self.v25o_brake_mode == V25O_BRAKE_MODE_LEAD:
          hold_brake_to_standstill = (
            (
              self.v25o_sng_armed and
              (
                stopped_lead_approach or
                stop_completion_active
              ) and
              CS.out.vEgo <= V25O_SNG_ARM_SPEED
            ) or
            v33r3_stop_completion_guard
          )
          low_demand = (
            False
            if (
              hold_brake_to_standstill or
              v33r3_stop_guard_authority
            ) else
            (
              planner_brake_request < V25R_PLANNER_RELEASE
              if plan_fresh else
              brake_request < 0.12
            )
          )
        elif self.v25o_brake_mode == V25W_BRAKE_MODE_CURVE:
          low_demand = (
            planner_brake_request < V25W_CURVE_BRAKE_RELEASE
            if plan_fresh else
            brake_request < V25O_NOLEAD_BRAKE_RELEASE
          )
        else:
          low_demand = (
            planner_brake_request < 0.08
            if plan_fresh else
            brake_request < V25O_NOLEAD_BRAKE_RELEASE
          )

        if safety_hard_release:
          self.v25o_sng_armed = False
          if control_allowed:
            start_v33r_staged_release()
          clear_hydraulic()

        elif force_soft_release or low_demand:
          self.v25r_release_counter = min(
            self.v25r_release_counter + 1,
            V25R_RELEASE_CONFIRM_FRAMES,
          )
          soft_releasing_hydraulic = True
          self.v25l_brake_target = V25L_BRAKE_MIN
          release_step = (
            V30_HANDOFF_STEP_DOWN
            if hydraulic_handoff_release else
            V25R_BRAKE_STEP_DOWN
          )
          self.v25l_apply_brake = max(
            0.0,
            self.v25l_apply_brake - release_step,
          )

          if (
            self.v25r_release_counter >= V25R_RELEASE_CONFIRM_FRAMES and
            self.v25l_apply_brake <= V25L_BRAKE_MIN
          ):
            self.v25o_sng_armed = False
            was_v30_handoff = self.v30_handoff_active
            start_v33r_staged_release()
            clear_hydraulic()
            if was_v30_handoff:
              self.v25l_brake_reentry_frame = (
                frame + V33R2_REENTRY_BLOCK_FRAMES
              )
        else:
          self.v25r_release_counter = 0

      if not sng_release_active and not self.v25o_stop_hold and not self.v25l_brake_active:
        stopped_lead_reentry = (
          (
            stopped_lead_approach and
            planner_brake_request >= V25U_STOP_LEAD_MIN_BRAKE
          ) or
          v33r_creep_stop_guard or
          v33r3_stop_guard_entry
        )
        lead_entry = (
          moving_allowed and
          (relevant_lead or v33r_creep_stop_guard) and
          (
            planner_brake_request >= lead_hydraulic_entry or
            stopped_lead_reentry or
            v33r_early_highway_entry or
            v33r3_stop_guard_entry
          ) and
          CS.out.vEgo > (
            V33R_CREEP_GUARD_MIN_EGO
            if v33r_creep_stop_guard else
            V25L_MIN_ENTRY_SPEED
          ) and
          (
            frame > self.block_brake_until_frame or
            v33r_creep_stop_guard
          ) and
          (
            frame >= self.v25l_brake_reentry_frame or
            stopped_lead_reentry or
            v33r3_stop_guard_entry
          )
        )
        urgent_entry = (
          moving_allowed and
          urgent_closing and
          (
            planner_brake_request >= 0.25 or
            v33r_emergency_closing
          ) and
          CS.out.vEgo > V25L_MIN_ENTRY_SPEED and
          frame > self.block_brake_until_frame
        )
        if lead_entry:
          required_entry_frames = (
            1
            if v33r3_stop_guard_entry else
            (
              V33R_CREEP_ENTRY_FRAMES
              if v33r_creep_stop_guard else
              V25R_LEAD_ENTRY_FRAMES
            )
          )
          self.v25l_brake_entry_counter = min(
            self.v25l_brake_entry_counter + 1,
            required_entry_frames,
          )
        else:
          required_entry_frames = V25R_LEAD_ENTRY_FRAMES
          self.v25l_brake_entry_counter = 0

        if urgent_entry:
          self.v25o_nolead_entry_counter = min(
            self.v25o_nolead_entry_counter + 1,
            V25R_URGENT_ENTRY_FRAMES,
          )
        else:
          self.v25o_nolead_entry_counter = 0

        urgent_confirmed = (
          urgent_entry and
          self.v25o_nolead_entry_counter >= V25R_URGENT_ENTRY_FRAMES
        )
        normal_confirmed = (
          self.v25l_brake_entry_counter >= required_entry_frames
        )

        if urgent_confirmed or normal_confirmed:
          self.v25l_brake_active = True
          self.v25o_brake_mode = V25O_BRAKE_MODE_LEAD
          self.v25r_urgent_brake = bool(urgent_confirmed)
          entry_brake = V25L_BRAKE_MIN
          if v33r3_stock_brake_guard:
            entry_brake = min(
              V33R3_STOCK_INITIAL_BRAKE_MAX,
              max(V25L_BRAKE_MIN, stock_brake_request),
            )
          elif v33r3_predictive_stop_confirmed:
            entry_brake = V33R3_PREDICTIVE_INITIAL_BRAKE
          self.v25l_apply_brake = entry_brake
          self.v25l_brake_target = entry_brake
          self.v33r3_stop_guard_latched = bool(
            v33r3_stop_guard_entry
          )
          self.v25l_brake_entry_counter = 0
          self.v25o_nolead_entry_counter = 0
          self.v25r_release_counter = 0
          self.v25l_speed_offset = 0.0
          if v33r_creep_stop_guard:
            self.v25o_sng_armed = True
            self.v25l_apply_brake = max(
              self.v25l_apply_brake,
              V33R_CREEP_BRAKE_FLOOR,
            )
            self.v25l_brake_target = max(
              self.v25l_brake_target,
              V33R_CREEP_BRAKE_FLOOR,
            )

      if (
        self.v25l_brake_active and
        (
          v33r3_stock_brake_guard or
          v33r3_predictive_stop_confirmed
        )
      ):
        self.v33r3_stop_guard_latched = True

      if self.v25l_brake_active and not self.v25o_stop_hold:
        if not soft_releasing_hydraulic:
          speed_scale = interp(
            CS.out.vEgo,
            [0.0, 140.0 * CV.KPH_TO_MS],
            [1.0, 1.0 / 1.5]
          )

          if self.v25o_brake_mode == V25O_BRAKE_MODE_NOLEAD:
            brake_cap = V25O_NOLEAD_BRAKE_MAX
          elif self.v25o_brake_mode == V25W_BRAKE_MODE_CURVE:
            brake_cap = v25w_curve_brake_cap(CS.out.vEgo)
          else:
            if urgent_closing or planner_brake_request >= V25R_URGENT_HARD_DECEL:
              self.v25r_urgent_brake = True

            if v33r3_stop_guard_authority:
              requested_cap = V33R3_STOP_BRAKE_MAX
            elif self.v25r_urgent_brake:
              requested_cap = V25R_URGENT_BRAKE_MAX
            elif CS.out.vEgo >= V30_HIGHWAY_MIN_SPEED:
              requested_cap = min(
                lead_normal_cap,
                v30_progressive_brake_cap(CS.out.vEgo),
              )
            else:
              requested_cap = lead_normal_cap

            brake_cap = min(
              requested_cap,
              v25l_low_speed_brake_cap(CS.out.vEgo),
              v25l_high_speed_brake_cap(CS.out.vEgo),
            )
            if v33r3_stop_guard_authority:
              # The guard's 0.87 ceiling is itself the stock-derived envelope;
              # do not apply the older 0.45/0.65 generic caps on top of it.
              brake_cap = V33R3_STOP_BRAKE_MAX
            elif self.v33r3_stop_guard_latched:
              # If guard geometry clears before planner demand, decay from the
              # prior command through the normal rate limiter instead of
              # clipping abruptly back to the legacy cap.
              brake_cap = max(
                brake_cap,
                min(V33R3_STOP_BRAKE_MAX, self.v25l_apply_brake),
              )
          crawl_floor_active = (
            self.v25o_brake_mode == V25O_BRAKE_MODE_LEAD and
            (
              v33r_creep_stop_guard or
              stop_completion_active or
              v33r3_stop_completion_guard
            )
          )
          brake_floor = (
            max(V33R_CREEP_BRAKE_FLOOR, V25X_SC_MIN_BRAKE)
            if crawl_floor_active else
            V25L_BRAKE_MIN
          )
          if crawl_floor_active:
            # Apply the floor to the cap itself, so the later filter/clip
            # cannot silently reduce the final-crawl command below 0.18.
            brake_cap = max(brake_cap, brake_floor)

          if plan_fresh:
            target_request = planner_brake_request
            # Downstream PID may add only a small same-direction correction.
            if apply_accel < 0.0:
              pid_extra = max(0.0, brake_request - planner_brake_request)
              target_request += min(
                V25R_PID_BRAKE_ALLOWANCE,
                V25R_PID_BRAKE_BLEND * pid_extra,
              )
          else:
            target_request = brake_request

          if v33r3_stop_guard_authority:
            target_request = max(
              target_request,
              v33r3_stop_guard_request,
              V33R_CREEP_BRAKE_FLOOR
              if v33r3_stop_completion_guard else
              V25L_BRAKE_MIN,
            )

          if self.v25o_brake_mode in (
            V25O_BRAKE_MODE_NOLEAD,
            V25W_BRAKE_MODE_CURVE,
          ):
            target_request = min(
              target_request,
              planner_brake_request + V25L_BRAKE_MIN,
            )

          raw_target_brake = float(clip(
            target_request * speed_scale,
            brake_floor,
            brake_cap,
          ))
          filter_alpha = (
            (
              V33R3_STOP_BRAKE_FILTER_UP
              if v33r3_stop_guard_authority else
              (
                V33R_EMERGENCY_BRAKE_FILTER_UP
                if v33r_emergency_closing else
                V25R_BRAKE_FILTER_UP
              )
            )
            if raw_target_brake > self.v25l_brake_target else
            V25R_BRAKE_FILTER_DOWN
          )
          self.v25l_brake_target += filter_alpha * (
            raw_target_brake - self.v25l_brake_target
          )
          self.v25l_brake_target = float(clip(
            self.v25l_brake_target,
            brake_floor,
            brake_cap,
          ))
          if self.v25l_brake_target > self.v25l_apply_brake:
            v30_step_up = (
              V33R3_STOP_BRAKE_STEP_UP
              if v33r3_stop_guard_authority else
              (
                V33R_EMERGENCY_BRAKE_STEP_UP
                if v33r_emergency_closing else
                (
                  V30_BRAKE_STEP_UP_URGENT
                  if self.v25r_urgent_brake else
                  V30_BRAKE_STEP_UP
                )
              )
            )
            rate_limited_brake = min(
              self.v25l_brake_target,
              self.v25l_apply_brake + v30_step_up,
            )
          else:
            rate_limited_brake = max(
              self.v25l_brake_target,
              self.v25l_apply_brake - V25R_BRAKE_STEP_DOWN,
            )
          if (
            self.v33r_filtered_aego <= V33R_DECEL_GOVERNOR_START and
            not v33r_critical_closing and
            not v33r_emergency_closing and
            not v33r3_stop_guard_authority
          ):
            # Act on the rate-limited output after target filtering. In V3.3R
            # the filter reduced a requested 0.015 step to about 0.0012.
            rate_limited_brake = min(
              rate_limited_brake,
              max(
                brake_floor,
                self.v25l_apply_brake -
                V33R_DECEL_GOVERNOR_STEP_DOWN,
              ),
            )
          if v33r3_stop_guard_authority:
            # A stock/geometry floor may bypass target filtering, but its rise
            # is still bounded to 0.05 per 20 Hz update. This prevents the old
            # 0.08 target filter from recreating the 13:24 authority delay.
            bounded_guard_floor = min(
              v33r3_stop_guard_request,
              self.v25l_apply_brake + V33R3_STOP_BRAKE_STEP_UP,
            )
            rate_limited_brake = max(
              rate_limited_brake,
              bounded_guard_floor,
            )
          self.v25l_apply_brake = float(clip(
            rate_limited_brake,
            brake_floor,
            brake_cap,
          ))

        if (
          self.v25o_brake_mode == V25O_BRAKE_MODE_LEAD and
          relevant_lead and
          CS.out.vEgo <= V25O_SNG_ARM_SPEED and
          (
            planner_brake_request >= V25O_SNG_ARM_BRAKE or
            (
              stopped_lead_approach and
              planner_brake_request >= V25U_STOP_LEAD_MIN_BRAKE
            ) or
            v33r_creep_stop_guard or
            v33r3_stop_completion_guard
          )
        ):
          self.v25o_sng_armed = True

        self.v25l_propulsion_block_until_frame = max(
          self.v25l_propulsion_block_until_frame,
          frame + V25L_PROPULSION_DWELL_FRAMES,
        )

      if (
        self.v25o_sng_armed and
        not self.v25o_stop_hold and
        self.v25o_sng_release_frames == 0 and
        CS.out.vEgo > V25O_SNG_APPROACH_SPEED and
        planner_accel_request >= V25R_PLANNER_ACCEL_ENTRY and
        not v33r3_stop_guard_authority
      ):
        self.v25o_sng_armed = False

      hydraulic_req = (
        self.v25l_brake_active and
        self.v25l_apply_brake >= V25L_BRAKE_MIN
      )
      release_pump_active = (
        control_allowed and
        not hydraulic_req and
        (
          frame < self.v33r2_release_pump_until_frame or
          (
            self.v33r2_decel_latched and
            r4_feedback_clean and
            not r4_feedback["brakes_clear"]
          )
        )
      )

      # Toyota briefly overlaps positive hybrid torque with brake entry. In
      # the passive stock capture the longest voted positive-torque + friction
      # overlap was 0.20 s. Permit a wider 0.55 s entry envelope only while
      # torque does not rise materially; once torque has cleared, any return of
      # voted propulsion under friction is a fault.
      r4_negative_intent = (
        control_allowed and
        (
          hydraulic_req or
          release_pump_active or
          self.v33r2_decel_latched or
          (plan_fresh and planner_brake_request >= V33R2_DECEL_LATCH_BRAKE)
        )
      )
      if r4_negative_intent and self.v33r4_decel_entry_frame < 0:
        self.v33r4_decel_entry_frame = frame
        self.v33r4_decel_entry_torque = r4_feedback["torque_actual"]
        self.v33r4_decel_torque_cleared = (
          r4_feedback["torque_actual"] <= 80
        )
      if r4_negative_intent and r4_feedback["torque_actual"] <= 80:
        self.v33r4_decel_torque_cleared = True

      r4_positive_under_friction = (
        control_allowed and
        r4_feedback_clean and
        r4_feedback["friction"] > 0 and
        r4_feedback["positive_vote"]
      )
      r4_overlap_age = frame - self.v33r4_decel_entry_frame
      r4_overlap_rising = (
        r4_feedback["torque_actual"] >
        max(80, self.v33r4_decel_entry_torque + V33R4_ENTRY_TORQUE_RISE_RAW)
      )
      r4_overlap_unsafe = (
        r4_positive_under_friction and
        (
          not r4_negative_intent or
          self.v33r4_decel_torque_cleared or
          r4_overlap_age > V33R4_ENTRY_OVERLAP_FRAMES or
          r4_overlap_rising
        )
      )
      if r4_overlap_unsafe:
        self.v33r4_positive_overlap_counter = min(
          self.v33r4_positive_overlap_counter + 1,
          V33R4_OVERLAP_FAULT_FRAMES,
        )
      else:
        self.v33r4_positive_overlap_counter = 0
      if (
        self.v33r4_positive_overlap_counter >=
        V33R4_OVERLAP_FAULT_FRAMES
      ):
        self._v33r4_latch_fault(CS, "positive_torque_under_friction_braking")
        control_allowed = False

      if hydraulic_req:
        if self.v25o_stop_hold and CS.out.standstill:
          brake_state = 0x30
          self.v25l_apply_brake = V25O_SNG_HOLD_BRAKE
        elif self.v25o_stop_hold:
          # A standstill latch must never emit 0x30 while the car is moving.
          # Keep the verified moving stop state and crawl floor until physical
          # standstill returns or the normal hold-resume condition releases it.
          brake_state = 0x31
          self.v25l_apply_brake = max(
            self.v25l_apply_brake,
            V33R_CREEP_BRAKE_FLOOR,
          )
          self.v25l_brake_target = max(
            self.v25l_brake_target,
            V33R_CREEP_BRAKE_FLOOR,
          )
        elif (
          self.v25o_brake_mode == V25O_BRAKE_MODE_LEAD and
          self.v25o_sng_armed and
          CS.out.vEgo <= V25O_SNG_APPROACH_SPEED
        ):
          brake_state = 0x31
        else:
          brake_state = 0x21

        pump_reaction, brake_mag = v25l_encode_hev_brake(
          self.v25l_apply_brake
        )

      elif sng_release_active or release_pump_active:
        # Verified stock moving/standstill release combination: state 0x01,
        # pump -0.4/+0.4, neutral magnitude byte C8. V3.3R2 keeps this stage
        # for the full observed moving-brake release interval.
        brake_state = 0x01
        pump_reaction = -0.4
        brake_mag = (4 << 8) | 200

      # -----------------------------
      # V2.5R planner-primary desired-speed arbitration
      # -----------------------------
      # The planner selects direction. The downstream PID cannot request gas
      # while the lead planner still requests slowing. This is the key fix for
      # brake -> rev/no acceleration -> jump -> brake oscillation.
      release_freeze_active = (
        control_allowed and frame < self.v33r_release_freeze_until_frame
      )
      release_lead_active = (
        control_allowed and frame < self.v33r_release_lead_until_frame
      )
      low_speed_engagement_guard = (
        control_allowed and
        CS.out.vEgo < V33R_LOW_SPEED_ENGAGEMENT_MAX and
        frame < self.v33r_low_speed_guard_until_frame
      )

      # V3.3R2 containment: target-speed/curve regen uses the same 0x273
      # IS_ACCEL state that was present during the launches. Until actual
      # negative MG torque or regen feedback is decoded, do not create any
      # negative 0x273 target. The retained early-progressive 0x21 path owns
      # qualified lead braking.
      self.v33r_curve_regen_counter = 0
      curve_regen_allowed = False
      decel_offset_allowed = False

      positive_agreement = (
        plan_fresh and
        planner_accel_request >= V33R2_DECEL_CLEAR_PLANNER_ACCEL and
        apply_accel >= V33R2_DECEL_CLEAR_PID_ACCEL
      )
      departing_lead = (
        selected_lead_status and
        selected_lead_vrel >= V32R_DEPARTING_LEAD_VREL and
        selected_lead_drel >= V32R_DEPARTING_LEAD_DREL
      )
      distant_nonclosing_lead = (
        selected_lead_status and
        selected_lead_drel >= V32R_NONBLOCKING_LEAD_DISTANCE and
        selected_lead_vrel >= V32R_NONBLOCKING_LEAD_VREL
      )
      lead_nonblocking_for_propulsion = (
        not selected_lead_status or
        departing_lead or
        distant_nonclosing_lead
      )

      decel_latch_request = (
        control_allowed and
        (
          hydraulic_req or
          sng_release_active or
          release_pump_active or
          (
            plan_fresh and
            planner_brake_request >= V33R2_DECEL_LATCH_BRAKE
          ) or
          self.v25l_speed_offset < -V25V_REGEN_OFFSET_EPS
        )
      )
      if not control_allowed:
        self.v33r2_decel_latched = False
        self.v33r2_decel_clear_counter = 0
        self.v33r4_brake_clear_counter = 0
        self.v33r4_torque_ready_counter = 0
        self.v33r4_decel_entry_frame = -1000000
        self.v33r4_decel_torque_cleared = False
        self.v33r4_positive_overlap_counter = 0
        self.v33r2_release_pump_until_frame = frame
        self.v33r3_predictive_entry_counter = 0
        self.v33r3_stop_guard_latched = False
      elif decel_latch_request:
        if not self.v33r2_decel_latched:
          self.v33r4_decel_entry_frame = frame
          self.v33r4_decel_entry_torque = r4_feedback["torque_actual"]
          self.v33r4_decel_torque_cleared = (
            r4_feedback["torque_actual"] <= 80
          )
        self.v33r2_decel_latched = True
        self.v33r2_decel_clear_counter = 0
        self.v33r4_brake_clear_counter = 0
        self.v33r4_torque_ready_counter = 0
      elif self.v33r2_decel_latched:
        # R4 removes aEgo from the release decision. Net wheel acceleration can
        # conceal simultaneous positive drive and negative brake/regen torque.
        # Require the read-only brake request and friction channel to remain
        # clear before leaving the deceleration latch.
        decel_clear_candidate = (
          positive_agreement and
          not self.v25o_stop_hold and
          not hydraulic_req and
          not sng_release_active and
          not release_pump_active and
          r4_feedback_clean and
          r4_feedback["brakes_clear"] and
          (
            not relevant_lead or
            lead_nonblocking_for_propulsion
          )
        )
        if decel_clear_candidate:
          self.v33r4_brake_clear_counter = min(
            self.v33r4_brake_clear_counter + 1,
            V33R4_BRAKE_CLEAR_FRAMES,
          )
        else:
          self.v33r4_brake_clear_counter = 0
        self.v33r2_decel_clear_counter = self.v33r4_brake_clear_counter

        if (
          self.v33r4_brake_clear_counter >= V33R4_BRAKE_CLEAR_FRAMES
        ):
          self.v33r2_decel_latched = False
          self.v33r2_decel_clear_counter = 0
          self.v33r4_brake_clear_counter = 0
          self.v33r4_torque_ready_counter = 0
          self.v33r4_decel_entry_frame = -1000000
          self.v33r4_decel_torque_cleared = False
          self.v33r4_positive_overlap_counter = 0

      target_slope_lock = (
        hydraulic_req or
        release_pump_active or
        self.v33r2_decel_latched or
        release_freeze_active or
        self.v25l_speed_offset < -V25V_REGEN_OFFSET_EPS
      )
      if target_slope_lock:
        self.v33r_target_slope_unlock_frame = max(
          self.v33r_target_slope_unlock_frame,
          frame + V33R_TARGET_SLOPE_UNLOCK_FRAMES,
        )

      # After physical brake feedback clears, R4 first arms IS_ACCEL with an
      # exact current-speed target. Positive target offset remains zero until
      # the request/actual torque vote is no longer strongly negative and the
      # planner/PID have agreed on acceleration for 0.25 s.
      r4_accel_arm_ready = (
        control_allowed and
        r4_feedback_clean and
        r4_feedback["brakes_clear"]
      )
      r4_torque_ready_candidate = (
        r4_accel_arm_ready and
        r4_feedback["torque_ramp_ready"] and
        not hydraulic_req and
        not release_pump_active and
        not self.v33r2_decel_latched and
        positive_agreement
      )
      if r4_torque_ready_candidate:
        self.v33r4_torque_ready_counter = min(
          self.v33r4_torque_ready_counter + 1,
          V33R4_TORQUE_READY_FRAMES,
        )
      else:
        self.v33r4_torque_ready_counter = 0
      r4_propulsion_ramp_ready = (
        self.v33r4_torque_ready_counter >= V33R4_TORQUE_READY_FRAMES
      )

      propulsion_blocked = (
        hydraulic_req or
        release_pump_active or
        self.v33r2_decel_latched or
        not r4_propulsion_ramp_ready or
        not plan_fresh or
        frame < self.v25l_propulsion_block_until_frame or
        frame < self.v33r_target_slope_unlock_frame or
        frame < self.v33r_overshoot_block_until_frame or
        release_freeze_active or
        low_speed_engagement_guard
      )

      if plan_fresh:
        if planner_accel_request < 0.0:
          # Hydraulic arbitration above consumes qualified lead deceleration.
          # Do not mirror it into the ambiguous target-speed regen channel.
          effective_accel = 0.0
        elif planner_accel_request > 0.0 and apply_accel > 0.0:
          # Positive propulsion requires planner/PID agreement.
          effective_accel = min(
            planner_accel_request + 0.08,
            0.75 * planner_accel_request + 0.25 * apply_accel,
          )
        else:
          # Planner positive but PID still negative: coast rather than fight.
          effective_accel = 0.0
      else:
        effective_accel = (
          apply_accel
          if apply_accel >= 0.0 or decel_offset_allowed else
          0.0
        )

      if not decel_offset_allowed and self.v25l_speed_offset < 0.0:
        # Lead loss or turn-source loss is a safety-neutral transition for
        # 0x273. Do not retain a negative target on an ordinary open road.
        self.v25l_speed_offset = 0.0

      # Preserve the existing post-deceleration dwell, now driven by the
      # persistent latch and verified pump-release stage instead of a negative
      # 0x273 target. Low speed retains an additional neutral wake delay.
      regen_or_brake_active = (
        hydraulic_req or
        sng_release_active or
        release_pump_active or
        self.v33r2_decel_latched
      )
      if regen_or_brake_active:
        self.v25v_regen_release_until_frame = max(
          self.v25v_regen_release_until_frame,
          frame + V25V_REGEN_RELEASE_DWELL_FRAMES,
        )
        if CS.out.vEgo < V32R_LOW_SPEED_MAX:
          self.v32r_neutral_until_frame = max(
            self.v32r_neutral_until_frame,
            frame + V32R_REGEN_NEUTRAL_DWELL_FRAMES,
          )
          self.v32r_low_speed_arm_start_frame = -1000000

      if planner_reports_lead and plan_fresh:
        if (
          planner_accel_request >= V25R_PLANNER_ACCEL_ENTRY and
          apply_accel > 0.0
        ):
          self.v25r_lead_accel_counter = min(
            self.v25r_lead_accel_counter + 1,
            V25R_LEAD_ACCEL_CONFIRM_FRAMES,
          )
        else:
          self.v25r_lead_accel_counter = 0
      else:
        self.v25r_lead_accel_counter = V25R_LEAD_ACCEL_CONFIRM_FRAMES

      lead_accel_confirmed = (
        self.v25r_lead_accel_counter >= V25R_LEAD_ACCEL_CONFIRM_FRAMES
      )

      # V3.2R low-speed lead classification is used only to decide whether the
      # outgoing ECU wake bit may be used. It does not alter planner/radar lead
      # state and cannot enter hydraulic braking. V3.3R2 computes the common
      # positive-intent and lead-release predicates above the decel latch.
      low_speed_propulsion_request = (
        control_allowed and
        not hydraulic_req and
        not sng_release_active and
        not release_pump_active and
        not self.v33r2_decel_latched and
        not self.v25o_stop_hold and
        CS.out.vEgo < V32R_LOW_SPEED_MAX and
        positive_agreement and
        lead_nonblocking_for_propulsion
      )

      low_speed_arm_eligible = (
        low_speed_propulsion_request and
        frame >= self.v32r_neutral_until_frame and
        frame >= self.v32r_overshoot_block_until_frame and
        self.v25l_speed_offset >= -V25V_REGEN_OFFSET_EPS
      )
      if not low_speed_arm_eligible:
        self.v32r_low_speed_arm_start_frame = -1000000
      elif self.v32r_low_speed_arm_start_frame < 0:
        self.v32r_low_speed_arm_start_frame = frame

      low_speed_arm_active = (
        low_speed_arm_eligible and
        frame - self.v32r_low_speed_arm_start_frame <
        V32R_LOW_SPEED_ARM_FRAMES
      )
      low_speed_arm_complete = (
        low_speed_arm_eligible and
        not low_speed_arm_active
      )

      unexpected_positive_accel = (
        control_allowed and
        CS.out.aEgo >= V33R_OVERSHOOT_AEGO and
        self.v25l_speed_offset <= V25V_REGEN_OFFSET_EPS and
        not positive_agreement
      )
      if unexpected_positive_accel:
        self.v33r_overshoot_counter = min(
          self.v33r_overshoot_counter + 1,
          V33R_OVERSHOOT_CONFIRM_FRAMES,
        )
      else:
        self.v33r_overshoot_counter = 0

      if (
        self.v33r_overshoot_counter >=
        V33R_OVERSHOOT_CONFIRM_FRAMES
      ):
        self.v25l_speed_offset = 0.0
        self.v33r2_decel_latched = True
        self.v33r2_decel_clear_counter = 0
        self.v33r_overshoot_block_until_frame = (
          frame + V33R_OVERSHOOT_BLOCK_FRAMES
        )
        self.v33r_target_slope_unlock_frame = max(
          self.v33r_target_slope_unlock_frame,
          frame + V33R_TARGET_SLOPE_UNLOCK_FRAMES,
        )
        self.v33r_release_lead_until_frame = max(
          self.v33r_release_lead_until_frame,
          frame + V33R_RELEASE_LEAD_HOLD_FRAMES,
        )
        self.v33r_overshoot_counter = 0

        # Never add no-lead braking. A persistent close stopped lead may
        # re-enter the verified 0x31/0x30 stop path.
        if (
          v33r_trusted_stopped_lead and
          CS.out.vEgo <= V33R_CREEP_GUARD_MAX_EGO
        ):
          self.v25o_sng_armed = True
          self.v25l_brake_active = True
          self.v25o_brake_mode = V25O_BRAKE_MODE_LEAD
          self.v25l_apply_brake = max(
            self.v25l_apply_brake,
            V33R_CREEP_BRAKE_FLOOR,
          )
          self.v25l_brake_target = max(
            self.v25l_brake_target,
            V33R_CREEP_BRAKE_FLOOR,
          )
          hydraulic_req = True

      low_speed_overshoot = (
        CS.out.vEgo < V32R_LOW_SPEED_MAX and
        self.v25l_speed_offset > 0.0 and
        CS.out.aEgo >= V32R_LOW_SPEED_OVERSHOOT_AEGO
      )
      if low_speed_overshoot:
        self.v25l_speed_offset = 0.0
        self.v33r2_decel_latched = True
        self.v33r2_decel_clear_counter = 0
        self.v32r_overshoot_block_until_frame = (
          frame + V32R_OVERSHOOT_BLOCK_FRAMES
        )
        self.v32r_neutral_until_frame = max(
          self.v32r_neutral_until_frame,
          frame + V32R_REGEN_NEUTRAL_DWELL_FRAMES,
        )
        self.v32r_low_speed_arm_start_frame = -1000000

      if not control_allowed:
        self.v25l_speed_offset = 0.0
        self.v33r_release_freeze_until_frame = frame
        self.v33r_release_lead_until_frame = frame
        self.v33r_target_slope_unlock_frame = frame
        self.v33r_overshoot_counter = 0
        self.v33r_curve_regen_counter = 0
        self.v33r2_decel_latched = False
        self.v33r2_decel_clear_counter = 0
        self.v33r2_release_pump_until_frame = frame

      elif (
        hydraulic_req or
        sng_release_active or
        release_pump_active or
        self.v33r2_decel_latched
      ):
        self.v25l_speed_offset = 0.0

      elif (
        decel_offset_allowed and
        effective_accel <= -V25L_DECEL_DEADBAND and
        frame > self.block_brake_until_frame and
        frame >= self.v25l_brake_reentry_frame
      ):
        effective_brake = max(0.0, -effective_accel)
        decel_offset_cap = v25l_powertrain_decel_cap(CS.out.vEgo)
        decel_offset_step_down = V25L_DECEL_OFFSET_STEP_DOWN
        if curve_regen_allowed and not relevant_lead:
          decel_offset_cap = min(
            decel_offset_cap,
            V33R_CURVE_REGEN_OFFSET_MAX,
          )
          decel_offset_step_down = V33R_CURVE_REGEN_OFFSET_STEP_DOWN
        target_offset = -min(
          decel_offset_cap,
          effective_brake * t_lookup,
        )

        if self.v25l_speed_offset > 0.0:
          self.v25l_speed_offset = 0.0
        elif target_offset < self.v25l_speed_offset:
          self.v25l_speed_offset = max(
            target_offset,
            self.v25l_speed_offset - decel_offset_step_down,
          )
        else:
          if (
            frame >= self.v33r_target_slope_unlock_frame and
            CS.out.aEgo >= V33R_PROPULSION_AEGO_MIN
          ):
            self.v25l_speed_offset = min(
              target_offset,
              self.v25l_speed_offset + V33R_TARGET_RETURN_STEP,
            )

      elif (
        effective_accel >= V25L_ACCEL_ENTRY and
        not propulsion_blocked and
        frame >= self.v25v_regen_release_until_frame and
        (
          not planner_reports_lead or
          not plan_fresh or
          lead_accel_confirmed or
          low_speed_propulsion_request
        ) and
        (
          CS.out.vEgo >= V32R_LOW_SPEED_MAX or
          low_speed_arm_complete
        )
      ):
        # Retain V3.0's highway cap/ramp selection. The low-speed path receives
        # the smaller V3.3R2 authority and can run only after the latch clears.
        if CS.out.vEgo < V32R_LOW_SPEED_MAX:
          if departing_lead:
            accel_cap = max(
              lead_accel_cap,
              V32R_DEPARTING_ACCEL_CAP,
            )
            offset_cap = V32R_DEPARTING_OFFSET_CAP
            accel_step_up = V32R_DEPARTING_OFFSET_STEP_UP
          else:
            accel_cap = V32R_LOW_SPEED_ACCEL_CAP
            offset_cap = V32R_LOW_SPEED_OFFSET_CAP
            accel_step_up = V32R_LOW_SPEED_OFFSET_STEP_UP
        else:
          accel_cap = (
            lead_accel_cap
            if planner_reports_lead and plan_fresh else
            V25L_ACCEL_CAP
          )
          offset_cap = (
            0.30
            if planner_reports_lead and plan_fresh else
            V25L_ACCEL_OFFSET_MAX
          )
          accel_step_up = V25L_ACCEL_OFFSET_STEP_UP

        target_offset = min(
          offset_cap,
          min(effective_accel, accel_cap) * t_lookup,
        )

        if self.v25l_speed_offset < 0.0:
          self.v25l_speed_offset = min(
            0.0,
            self.v25l_speed_offset + V33R_TARGET_RETURN_STEP,
          )
        elif target_offset > self.v25l_speed_offset:
          self.v25l_speed_offset = min(
            target_offset,
            self.v25l_speed_offset + accel_step_up,
          )
        else:
          self.v25l_speed_offset = max(
            target_offset,
            self.v25l_speed_offset - V25L_ACCEL_OFFSET_STEP_DOWN,
          )

      else:
        # Planner neutral, unconfirmed acceleration, or regen-release dwell:
        # return to a true zero offset before any positive propulsion.
        if self.v25l_speed_offset < 0.0:
          if (
            frame >= self.v33r_target_slope_unlock_frame and
            CS.out.aEgo >= V33R_PROPULSION_AEGO_MIN
          ):
            self.v25l_speed_offset = min(
              0.0,
              self.v25l_speed_offset + V33R_TARGET_RETURN_STEP,
            )
        else:
          self.v25l_speed_offset = 0.0

      if release_freeze_active:
        self.v25l_speed_offset = min(0.0, self.v25l_speed_offset)

      if low_speed_arm_active:
        # ECU wake stage: lead bit only, no positive desired-speed target.
        self.v25l_speed_offset = 0.0

      # Longitudinal engagement follows independent physical/cruise override
      # state, not the stale outer `enabled` bit. This makes gas, brake,
      # CANCEL, and cruise-latch loss encode an exact disabled command.
      longitudinal_enabled = control_allowed and self.CP.openpilotLongitudinalControl
      if not longitudinal_enabled:
        brake_state = 0x00
        pump_reaction = 0.0
        brake_mag = 200
        des_speed = 0.0
      elif self.v25o_stop_hold or sng_release_active:
        # Stock standstill and standstill-release logs keep ACC_CMD at zero.
        des_speed = 0.0
      elif (
        hydraulic_req or
        release_pump_active or
        self.v33r2_decel_latched
      ):
        # Never combine braking intent with a lowered or positive 0x273 target.
        des_speed = CS.out.vEgo
      else:
        des_speed = max(0.0, CS.out.vEgo + self.v25l_speed_offset)

      # Select the two independent 0x273 longitudinal mode bits explicitly.
      # Stock-observed state table:
      #   disabled                 = 0x00: IS_ACCEL=0, IS_DECEL=0
      #   normal 0x01, including
      #   lowered ACC_CMD regen    = 0x40: IS_ACCEL=1, IS_DECEL=0
      #   moving 0x21 / crawl 0x31 = 0x20: IS_ACCEL=0, IS_DECEL=1
      #   standstill 0x30          = 0x60: IS_ACCEL=1, IS_DECEL=1
      #   0x30 pressure release    = 0x20 during the FC/04/C8 staging frames
      low_speed_handoff_blocked = (
        longitudinal_enabled and
        CS.out.vEgo < V32R_LOW_SPEED_MAX and
        not low_speed_arm_complete
      )
      if not longitudinal_enabled:
        acc_cmd_is_accel = False
        acc_cmd_is_decel = False
      elif brake_state == 0x30:
        acc_cmd_is_accel = True
        acc_cmd_is_decel = True
      elif (
        brake_state in (0x21, 0x31) or
        sng_release_active or
        release_pump_active or
        self.v33r2_decel_latched or
        low_speed_handoff_blocked or
        not r4_accel_arm_ready or
        not plan_fresh
      ):
        acc_cmd_is_accel = False
        acc_cmd_is_decel = True
      else:
        # IS_ACCEL is reachable only after the persistent latch and all neutral
        # dwell/override gates have cleared.
        acc_cmd_is_accel = True
        acc_cmd_is_decel = False

      low_speed_accel_unlock = (
        longitudinal_enabled and
        low_speed_propulsion_request and
        not hydraulic_req and
        not sng_release_active and
        not release_pump_active and
        not self.v33r2_decel_latched and
        r4_propulsion_ramp_ready and
        not self.v25o_stop_hold and
        frame >= self.v32r_neutral_until_frame and
        frame >= self.v32r_overshoot_block_until_frame and
        (
          low_speed_arm_active or
          self.v25l_speed_offset > 0.0
        )
      )
      # Cancel/disabled frames cannot preserve a stale lead bit.
      lead_for_acc_cmd = bool(
        longitudinal_enabled and
        (
          lead or
          low_speed_accel_unlock or
          release_lead_active or
          self.v33r2_decel_latched or
          self.v25o_stop_hold
        )
      )

      can_sends.append(  # Add ACC_CMD_HUD message
        dnga_create_accel_command(  # Build 0x273 ACC command frame
          self.packer,  # CAN packer
          CS.cruise_speed,  # OP cruise set speed
          CS.out.cruiseState.available,  # ACC ready/available bit
          longitudinal_enabled,  # Independently override-gated longitudinal state
          lead_for_acc_cmd,  # Real lead or isolated low-speed ECU wake
          des_speed,  # Desired speed command
          acc_cmd_is_accel,  # Explicit stock IS_ACCEL mode bit
          acc_cmd_is_decel,  # Explicit stock IS_DECEL mode bit
          CS.op_distance_val  # Follow distance setting
        )
      )

      can_sends.append(  # Add ACC_BRAKE message
        dnga_create_brake_command(  # Build 0x271 brake command
          self.packer,  # CAN packer
          brake_state,  # 0x00, 0x01, 0x21, 0x31, or 0x30
          pump_reaction,  # 0.0 or -0.4
          brake_mag,  # 200 or conservative 0x04xx value
          (frame // 5) % 8  # ACC_BRAKE counter
        )
      )

      can_sends.append(  # Add LKAS_HUD message
        dnga_create_hud(  # Build 0x274 HUD command
          self.packer,  # CAN packer
          CS.out.cruiseState.available and CS.lkas_latch,  # LKAS ready state
          enabled,  # LKAS engaged display state
          left_line,  # Left lane visible
          right_line,  # Right lane visible
          self.stockLdw,  # Lane departure warning flag
          CS.stock_fcw,  # Stock FCW state
          CS.stock_aeb,  # Stock AEB state
          CS.stock_adas_frontDepartureHUD,  # Front departure HUD state
          CS.stock_lkc_off,  # LKC off state
          CS.stock_fcw_off  # FCW off state
        )
      )

    self.last_steer = apply_steer  # Store applied steering torque for next frame

    new_actuators = actuators.as_builder()  # Copy actuator output object
    new_actuators.torque = float(apply_steer / steer_max_interp)
    new_actuators.torqueOutputCan = apply_steer  # Report actual normalized steering command

    self.frame += 1
    return new_actuators, can_sends  # Return actuator feedback and CAN messages
