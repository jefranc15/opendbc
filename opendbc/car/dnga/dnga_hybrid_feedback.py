"""Read-only DNGA hybrid/brake feedback decoding.

The engineering names and units of these fields are not yet Techstream-
validated. Longitudinal control uses these read-only observations only to
confirm the physical brake-to-propulsion handoff. Nothing in this module
transmits a CAN message or owns ACC, HUD, LKA, or cruise-session state.
"""

HYBRID_FEEDBACK_MAX_AGE_FRAMES = 25  # 0.25 s at the 100 Hz car loop
HYBRID_TORQUE_RELEASE_MIN_RAW = -100  # strong negative torque has faded
HYBRID_TORQUE_RELEASE_MIN_11BIT = -8
HYBRID_BRAKE_REQUEST_CLEAR_MIN = -100  # 0x275 word 0; active is ~-500..
HYBRID_FRICTION_CLEAR_MAX = 0  # 0x08C byte 2 stock handoff value
# The asynchronously sampled passive stock capture reached 354.7 and 92
# respectively during short driver-brake transitions. These are gross-
# plausibility limits used only to reject obviously inconsistent feedback.
HYBRID_ACTUAL_12A_MAX_ERROR = 400  # actual - (73/6)*0x12A
HYBRID_12A_125_MAX_ERROR = 100


def signed_value(raw, bits):
  sign = 1 << (bits - 1)
  return raw - (1 << bits) if raw & sign else raw


def toyota_checksum_valid(addr, dat):
  """Validate Toyota's last-byte additive checksum for any 11-bit ID."""
  dat = bytes(dat)
  if len(dat) < 2:
    return False
  expected = ((addr & 0xFF) + ((addr >> 8) & 0xFF) + len(dat) + sum(dat[:-1])) & 0xFF
  return dat[-1] == expected


def decode_hybrid_feedback_frame(addr, dat):
  """Return a validated field dictionary, or None for malformed data."""
  dat = bytes(dat)
  expected_lengths = {
    0x08C: 8,
    0x125: 7,
    0x12A: 7,
    0x275: 8,
    0x2C9: 8,
  }
  if addr not in expected_lengths or len(dat) != expected_lengths[addr]:
    return None
  if not toyota_checksum_valid(addr, dat):
    return None

  if addr == 0x275:
    return {
      "hybrid_brake_request_raw_275": signed_value(int.from_bytes(dat[0:2], "big"), 16),
      "hybrid_torque_request_raw_275": signed_value(int.from_bytes(dat[2:4], "big"), 16),
    }
  if addr == 0x2C9:
    return {
      "hybrid_torque_actual_raw_2c9": signed_value(int.from_bytes(dat[5:7], "big"), 16),
    }
  if addr == 0x12A:
    return {
      "hybrid_torque_raw_12a": signed_value(int.from_bytes(dat[2:4], "big") & 0x7FF, 11),
    }
  if addr == 0x125:
    return {
      "hybrid_torque_raw_125": signed_value(int.from_bytes(dat[3:5], "big") & 0x7FF, 11),
    }
  return {"hybrid_friction_raw_08c": dat[2]}


def initialize_hybrid_feedback_state(car_state):
  """Initialize the dynamic CarState attributes consumed by the longitudinal observer."""
  car_state.hybrid_brake_request_raw_275 = 0
  car_state.hybrid_torque_request_raw_275 = 0
  car_state.hybrid_torque_actual_raw_2c9 = 0
  car_state.hybrid_torque_raw_12a = 0
  car_state.hybrid_torque_raw_125 = 0
  car_state.hybrid_friction_raw_08c = 0
  car_state.hybrid_feedback_rx_frame_275 = -1000000
  car_state.hybrid_feedback_rx_frame_2c9 = -1000000
  car_state.hybrid_feedback_rx_frame_12a = -1000000
  car_state.hybrid_feedback_rx_frame_125 = -1000000
  car_state.hybrid_feedback_rx_frame_08c = -1000000


def apply_hybrid_feedback_frame(car_state, frame, addr, dat):
  """Decode one bus-1 frame into CarState and stamp its 100 Hz frame age."""
  decoded = decode_hybrid_feedback_frame(addr, dat)
  if decoded is None:
    return False
  for field, value in decoded.items():
    setattr(car_state, field, value)
  setattr(car_state, "hybrid_feedback_rx_frame_{:03x}".format(addr), int(frame))
  return True


def hybrid_feedback_snapshot(car_state, frame):
  """Classify feedback freshness, agreement, brake release, and torque readiness."""
  frame = int(frame)
  ages = {}
  fresh = True
  for suffix in ("275", "2c9", "12a", "125", "08c"):
    rx_frame = int(getattr(car_state, "hybrid_feedback_rx_frame_{}".format(suffix), -1000000))
    age = frame - rx_frame
    ages[suffix] = age
    fresh = fresh and 0 <= age <= HYBRID_FEEDBACK_MAX_AGE_FRAMES

  brake_request = int(getattr(car_state, "hybrid_brake_request_raw_275", 0))
  torque_request = int(getattr(car_state, "hybrid_torque_request_raw_275", 0))
  torque_actual = int(getattr(car_state, "hybrid_torque_actual_raw_2c9", 0))
  torque_12a = int(getattr(car_state, "hybrid_torque_raw_12a", 0))
  torque_125 = int(getattr(car_state, "hybrid_torque_raw_125", 0))
  friction = int(getattr(car_state, "hybrid_friction_raw_08c", 0))

  # 73/6 (12.1667) is the fit observed between 0x2C9 and 0x12A.
  actual_12a_error = abs((torque_actual * 6 - torque_12a * 73) / 6.0)
  duplicate_error = abs(torque_12a - torque_125)
  consistent = actual_12a_error <= HYBRID_ACTUAL_12A_MAX_ERROR and duplicate_error <= HYBRID_12A_125_MAX_ERROR
  brakes_clear = brake_request >= HYBRID_BRAKE_REQUEST_CLEAR_MIN and friction <= HYBRID_FRICTION_CLEAR_MAX
  torque_ramp_ready = (
    torque_request >= HYBRID_TORQUE_RELEASE_MIN_RAW
    and torque_actual >= HYBRID_TORQUE_RELEASE_MIN_RAW
    and torque_12a >= HYBRID_TORQUE_RELEASE_MIN_11BIT
    and torque_125 >= HYBRID_TORQUE_RELEASE_MIN_11BIT
  )

  return {
    "ages": ages,
    "fresh": bool(fresh),
    "consistent": bool(consistent),
    "brakes_clear": bool(brakes_clear),
    "torque_ramp_ready": bool(torque_ramp_ready),
    "brake_request": brake_request,
    "torque_request": torque_request,
    "torque_actual": torque_actual,
    "torque_12a": torque_12a,
    "torque_125": torque_125,
    "friction": friction,
    "actual_12a_error": actual_12a_error,
    "duplicate_error": duplicate_error,
  }
