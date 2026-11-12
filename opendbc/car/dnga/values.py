from collections import defaultdict
from enum import IntFlag

from opendbc.car import Bus, CarSpecs, PlatformConfig, Platforms
from opendbc.car.docs_definitions import CarDocs, SupportType


HUD_MULTIPLIER = 1.04


class DngaFlags(IntFlag):
  HYBRID = 1
  SNG = 2


class CarControllerParams:
  STEER_MAX = 255
  STEER_STEP = 2
  ACC_CONTROL_STEP = 5
  ACC_STEP = 5
  ACCEL_MIN = -3.5
  ACCEL_MAX = 1.6

  def __init__(self, CP):
    self.STEER_BP = CP.lateralParams.torqueBP
    self.STEER_LIM_TORQ = CP.lateralParams.torqueV
    if CP.carFingerprint in NOT_CAN_CONTROLLED:
      self.STEER_DELTA_UP = 20
      self.STEER_DELTA_DOWN = 30
    else:
      self.STEER_DELTA_UP = 22
      self.STEER_DELTA_DOWN = 35


class CAR(Platforms):
  TOYOTA_YARIS_CROSS_HEV = PlatformConfig(
    [
      CarDocs(
        "Toyota Yaris Cross Hybrid AC200",
        package="All",
        support_type=SupportType.COMMUNITY,
        support_link=None,
      ),
    ],
    CarSpecs(
      mass=1250.0,
      wheelbase=2.620,
      steerRatio=17.0,
      centerToFrontRatio=0.44,
      tireStiffnessFactor=0.7933,
    ),
    {
      Bus.pt: "dnga_hev",
      Bus.cam: "dnga_hev",
      Bus.alt: "dnga_hev",
    },
    flags=DngaFlags.HYBRID | DngaFlags.SNG,
  )


DBC = CAR.create_dbc_map()
ACC_CAR = {CAR.TOYOTA_YARIS_CROSS_HEV}
SNG_CAR = CAR.with_flags(DngaFlags.SNG)
HYBRID_CAR = CAR.with_flags(DngaFlags.HYBRID)
NOT_CAN_CONTROLLED = set()

# Retained from the DragonPilot port for the later V3.3R4 controller adaptation.
BRAKE_SCALE = defaultdict(lambda: 1.0, {CAR.TOYOTA_YARIS_CROSS_HEV: 2.0})
GAS_SCALE = defaultdict(lambda: 2600.0, {CAR.TOYOTA_YARIS_CROSS_HEV: 0.4})


class LongitudinalParams:
  """Tuning in m/s and m/s²; deadlines use 100 Hz frames, counts use 20 Hz updates."""

  # Lead braking and planner/PID arbitration.
  BRAKE_MIN = 0.05
  BRAKE_STEP_DOWN = 0.015
  BRAKE_FILTER_UP = 0.08
  BRAKE_FILTER_DOWN = 0.08
  LEAD_ENTRY_COUNT = 4
  URGENT_ENTRY_COUNT = 2
  LEAD_TRUST_COUNT = 10
  URGENT_LEAD_COUNT = 3

  # The 0.8.13 MPC often schedules lead decel well before accels[0:2] become
  # negative. Use a short, bounded lookahead to begin the smooth 0x273 decel
  # path earlier and to admit hydraulic entry earlier without changing urgent
  # or emergency classification.
  LEAD_LOOKAHEAD_COUNT = 13
  LEAD_LOOKAHEAD_WEIGHT = 0.75
  LEAD_LOOKAHEAD_MAX_BRAKE = 0.30
  LEAD_LOOKAHEAD_TARGET_CAP = 0.20
  LEAD_LOOKAHEAD_MIN_CLOSING = 0.50
  LEAD_LOOKAHEAD_MAX_TTC = 20.0
  MIN_ENTRY_SPEED = 0.5
  PLAN_MAX_AGE_FRAMES = 50
  RADAR_MAX_AGE_FRAMES = 50
  RELEASE_CONFIRM_COUNT = 10
  LEAD_ACCEL_CONFIRM_COUNT = 3
  LEAD_LOSS_COUNT = 3
  PLANNER_ACCEL_ENTRY = 0.05
  PLANNER_RELEASE = 0.05
  URGENT_PLANNER_DECEL = 0.45
  URGENT_HARD_DECEL = 0.7
  URGENT_CLOSING_SPEED = 2.5
  URGENT_TTC = 7.0
  URGENT_BRAKE_MAX = 0.45
  PID_BRAKE_BLEND = 0.25
  PID_BRAKE_ALLOWANCE = 0.08

  # Progressive highway braking and release.
  HIGHWAY_MIN_SPEED = 8.0
  HIGHWAY_ENTRY_MIN = 0.12
  HIGHWAY_ENTRY_MAX = 0.2
  HIGHWAY_BRAKE_CAP_MIN = 0.25
  HIGHWAY_BRAKE_CAP_MAX = 0.3
  BRAKE_STEP_UP = 0.006
  BRAKE_STEP_UP_URGENT = 0.015
  HANDOFF_PID_ACCEL = 0.15
  HANDOFF_AEGO_MAX = -0.08
  HANDOFF_COUNT = 3
  HANDOFF_STEP_DOWN = 0.03

  # Low-speed ECU wake, target limits, and overshoot recovery.
  LOW_SPEED_MAX = 8.5
  LOW_SPEED_NEUTRAL_DWELL_FRAMES = 50
  LOW_SPEED_ARM_FRAMES = 30
  LOW_SPEED_ACCEL_CAP = 0.05
  LOW_SPEED_OFFSET_CAP = 0.08
  LOW_SPEED_OFFSET_STEP_UP = 0.001
  DEPARTING_LEAD_VREL = 0.4
  DEPARTING_LEAD_DREL = 3.0
  DEPARTING_ACCEL_CAP = 0.08
  DEPARTING_OFFSET_CAP = 0.1
  DEPARTING_OFFSET_STEP_UP = 0.0015
  NONBLOCKING_LEAD_DISTANCE = 20.0
  NONBLOCKING_LEAD_VREL = -0.5
  LOW_SPEED_OVERSHOOT_AEGO = 0.3
  LOW_SPEED_OVERSHOOT_BLOCK_FRAMES = 50
  LOW_SPEED_ENGAGEMENT_MAX = 4.17
  LOW_SPEED_ENGAGEMENT_GUARD_FRAMES = 50

  # Stopped-lead confirmation and creep protection.
  STOP_LEAD_TRUST_COUNT = 3
  STOP_HOLD_MIN_DISTANCE = 1.0
  STOP_HOLD_MAX_DISTANCE = 12.0
  STOPPED_LEAD_MAX_SPEED = 0.5
  HOLD_RESUME_LEAD_SPEED = 0.35
  HOLD_RESUME_COUNT = 6
  CREEP_GUARD_MIN_EGO = 0.0
  CREEP_GUARD_MAX_EGO = 1.0
  CREEP_GUARD_MIN_CLOSING = 0.05
  CREEP_ENTRY_COUNT = 2
  CREEP_BRAKE_FLOOR = 0.24

  # Brake release framing and desired-speed return.
  RELEASE_LEAD_HOLD_FRAMES = 80
  TARGET_RETURN_STEP = 0.01
  OVERSHOOT_AEGO = 0.25
  OVERSHOOT_CONFIRM_COUNT = 2
  OVERSHOOT_BLOCK_FRAMES = 60

  # Early entry and measured-deceleration governor.
  EARLY_HIGHWAY_MIN_SPEED = 8.0
  EARLY_HIGHWAY_CLOSING = 1.0
  EARLY_HIGHWAY_TTC = 18.0
  EARLY_HIGHWAY_TIME_GAP = 3.0
  EARLY_HIGHWAY_PLANNER_BRAKE = 0.05
  AEGO_FILTER_ALPHA = 0.2
  DECEL_GOVERNOR_START = -0.9
  DECEL_GOVERNOR_CRITICAL_TTC = 3.0
  DECEL_GOVERNOR_CRITICAL_CLOSING = 3.0
  DECEL_GOVERNOR_STEP_DOWN = 0.015
  RELEASE_PUMP_FRAMES = 120
  REENTRY_BLOCK_FRAMES = 10

  # Passive factory braking and relative-motion stop guard.
  STOCK_FRAME_MAX_AGE = 25
  STOP_GUARD_MAX_SPEED = 8.0
  STOP_GUARD_MAX_DISTANCE = 20.0

  # A checksum-valid stock 0x271+0x273 pair is direct brake evidence, so allow
  # it to start the normal lead brake path earlier and do not require an
  # additional closing-speed threshold. Predictive geometry still keeps its
  # own stricter closing/TTC checks below.
  # A validated camera 0x271+0x273 brake pair remains useful at highway
  # speed. V4.3.4's 20 m/s ceiling caused OP to keep ACCEL asserted while the
  # factory camera and the HEV were already braking above ~72 km/h.
  STOCK_BRAKE_MAX_SPEED = 40.0
  STOCK_BRAKE_MAX_DISTANCE = 120.0
  STOCK_BRAKE_MAX_OPENING = 1.00
  STOCK_BRAKE_RELEASE_MAX_OPENING = 1.50
  STOCK_BRAKE_RELEASE_COUNT = 3
  STOP_GUARD_MIN_STOCK_BRAKE = 0.08

  # The validated OEM pair confirms that braking should continue, but normal
  # following should stay smoother than the factory controller. Use a small
  # fraction of the OEM magnitude plus closing-energy geometry, with a mild
  # initial entry and slower rise. Urgent/critical paths retain faster authority.
  STOCK_SMOOTH_INITIAL_BRAKE = 0.08
  STOCK_SMOOTH_ENTRY_MAX = 0.12
  STOCK_SMOOTH_FOLLOW_RATIO = 0.40
  STOCK_SMOOTH_MAX_BRAKE = 0.36
  STOCK_SMOOTH_FILTER_UP = 0.12
  STOCK_SMOOTH_STEP_UP = 0.012
  STOCK_URGENT_STEP_UP = 0.03
  STOCK_BRAKE_CAP_MARGIN = 0.05
  PREDICTIVE_MIN_PLANNER_BRAKE = 0.05
  PREDICTIVE_MIN_CLOSING = 0.7
  PREDICTIVE_MAX_TTC = 15.0
  PREDICTIVE_MAX_LEAD_SPEED = 5.5
  PREDICTIVE_ENTRY_COUNT = 3
  PREDICTIVE_STANDSTILL_GAP = 5.0
  PREDICTIVE_REACTION_TIME = 0.35
  PREDICTIVE_INITIAL_BRAKE = 0.16
  STOCK_INITIAL_BRAKE_MAX = 0.36
  STOP_BRAKE_MAX = 0.87  # absolute decode/safety ceiling, not the normal stop-guard cap
  STOP_BRAKE_FILTER_UP = 0.3
  STOP_BRAKE_STEP_UP = 0.05
  PUMP_05_THRESHOLD = 0.75
  STOP_COMPLETION_MAX_LEAD_SPEED = 1.0
  STOP_COMPLETION_MAX_DISTANCE = 12.0

  # Read-only HEV handoff feedback. This is a signal debounce, not a blind dwell.
  HYBRID_READY_COUNT = 3

  # Urgent trusted-lead escalation.
  EMERGENCY_CLOSING_SPEED = 2.0
  EMERGENCY_TTC = 8.0
  EMERGENCY_PLANNER_BRAKE = 0.2
  EMERGENCY_BRAKE_FILTER_UP = 0.2
  EMERGENCY_BRAKE_STEP_UP = 0.03

  # V3.3-style desired-speed shaping. Negative 0x273 targets are allowed
  # only for a trusted lead, an explicitly selected vision turn, or when the
  # user has set cruise below current speed.
  DECEL_DEADBAND = 0.02
  DECEL_OFFSET_STEP_DOWN = 0.02
  DECEL_OFFSET_STEP_UP = 0.08

  # Vision-turn anticipation. The 0.8.13 plan has 17 accel points through
  # 2.5 s; the first 13 cover ~1.4 s. Bring a bounded portion of that future
  # decel forward into 0x273, then add a narrowly scoped curve-only 0x21
  # supplement when the planner is explicitly source=turn and the car remains
  # meaningfully above the vision turn speed.
  CURVE_LOOKAHEAD_COUNT = 13
  CURVE_LOOKAHEAD_WEIGHT = 0.60
  CURVE_MAX_DECEL = 0.35
  CURVE_HYDRAULIC_MIN_SPEED = 5.6
  CURVE_SPEED_ERROR_ENTRY = 0.70
  CURVE_SPEED_ERROR_RELEASE = 0.20
  CURVE_BRAKE_ENTRY = 0.12
  CURVE_BRAKE_RELEASE = 0.06
  CURVE_ENTRY_COUNT = 3

  # Allow a user-commanded lower cruise setpoint to use the same smooth
  # below-vEgo 0x273 path once the car is meaningfully above that set speed.
  CRUISE_DECEL_MARGIN = 0.30

  # Near the selected cruise speed, the higher-level planner can still ask for
  # a tiny positive acceleration while the downstream speed PID is already
  # asking to slow. On the HEV this disagreement can hold the powertrain in an
  # unwanted propulsion/rev state. Confirm that mismatch briefly, then bias the
  # normal 0x273 target a few tenths of km/h below vEgo until PID agrees again.
  NEAR_SET_COAST_MIN_SPEED = 5.0
  NEAR_SET_COAST_BAND = 0.60
  NEAR_SET_COAST_PID_ENTRY = -0.05
  NEAR_SET_COAST_PID_RELEASE = 0.02
  NEAR_SET_COAST_PLAN_MAX = 0.12
  NEAR_SET_COAST_COUNT = 3
  NEAR_SET_COAST_DECEL = 0.08

  ACCEL_ENTRY = 0.05
  ACCEL_CAP = 0.25
  ACCEL_OFFSET_STEP_UP = 0.004
  ACCEL_OFFSET_STEP_DOWN = 0.04
  SPEED_OFFSET_EPS = 0.01
  ACCEL_OFFSET_MAX = 0.55

  # Stop-and-go hold, release, and final-crawl floor.
  SNG_ARM_SPEED = 2.5
  SNG_APPROACH_SPEED = 1.2
  SNG_ARM_BRAKE = 0.18
  SNG_HOLD_BRAKE = 0.05
  SNG_RELEASE_ACCEL = 0.08
  SNG_RELEASE_COUNT = 5
  STOP_LEAD_MAX_SPEED = 0.5
  STOP_LEAD_MAX_DISTANCE = 8.0
  STOP_LEAD_MIN_CLOSING = 0.15
  STOP_LEAD_MIN_BRAKE = 0.08
  STOP_COMPLETION_MAX_EGO_SPEED = 0.8
  CRAWL_MAX_LEAD_SPEED = 0.5
  CRAWL_MAX_LEAD_DISTANCE = 8.0
  STOP_COMPLETION_BRAKE_FLOOR = 0.24


class BrakeState:
  DISABLED = 0x00
  READY = 0x01
  BRAKING = 0x21
  CREEPING = 0x31
  HOLD = 0x30
