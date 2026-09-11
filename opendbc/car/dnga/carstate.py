import math

import numpy as np

from opendbc.can import CANDefine, CANParser
from opendbc.car import Bus, DT_CTRL, create_button_events, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.dnga.dnga_hybrid_feedback import apply_hybrid_feedback_frame, initialize_hybrid_feedback_state
from opendbc.car.dnga.values import DBC, HUD_MULTIPLIER
from opendbc.car.interfaces import CarStateBase


ButtonType = structs.CarState.ButtonEvent.Type
SEC_HOLD_TO_STEP_SPEED = 0.6
SHORT_PRESS_SEC = 1.0
CRUISE_MIN_KPH = 30.0
CRUISE_MAX_KPH = 125.0


def decode_engine_rpm_037(raw):
  """Decode the scanner-validated signed 16-bit 0x037 RPM field."""
  rpm_signed = raw if raw < 0x8000 else raw - 0x10000
  return max(0, rpm_signed)


def dnga_rx_checksum_valid(addr, dat):
  """Validate the additive DNGA checksum without trusting the DBC parser."""
  dat = bytes(dat)
  if len(dat) != 8:
    return False
  expected = (addr + len(dat[:-1]) + 1 + 2 + sum(dat[:-1])) & 0xFF
  return dat[-1] == expected


def decode_stock_acc_brake_271(dat):
  """Return a validated (state, pump, magnitude, decel) camera request."""
  dat = bytes(dat)
  if not dnga_rx_checksum_valid(0x271, dat):
    return None

  state = dat[1]
  pump_inverse = dat[3]
  pump_level = dat[4]
  magnitude = dat[5]

  if state in (0x21, 0x31, 0x30):
    if dat[2] != 0x00 or ((pump_inverse + pump_level) & 0xFF) != 0x00 or pump_level not in (4, 5, 6, 7, 8) or magnitude > 200:
      return None
  elif state in (0x00, 0x01):
    if magnitude != 200:
      return None
  else:
    return None

  pump = pump_level / 10.0
  decel = max(0.0, min(0.87, (200 - magnitude) / 100.0))
  return state, pump, magnitude, decel


def decode_stock_acc_cmd_273(dat):
  """Return validated camera engagement/lead/mode bits and desired speed."""
  dat = bytes(dat)
  if not dnga_rx_checksum_valid(0x273, dat):
    return None
  if not (dat[1] & 0x02) or not (dat[6] & 0x40):
    return None

  enabled = bool(dat[1] & 0x20)
  lead = bool(dat[1] & 0x08)
  is_decel = bool(dat[4] & 0x20)
  is_accel = bool(dat[4] & 0x40)
  acc_cmd_kph = ((dat[2] << 8) | dat[3]) * 0.01
  return enabled, lead, is_accel, is_decel, acc_cmd_kph


class CarState(CarStateBase):
  def __init__(self, CP, CP_SP):
    super().__init__(CP, CP_SP)

    can_define = CANDefine(DBC[CP.carFingerprint][Bus.pt])
    self.shifter_values = can_define.dv["TRANSMISSION"]["GEAR"]

    self.is_cruise_latch = False
    self.cruise_speed = CRUISE_MIN_KPH * CV.KPH_TO_MS
    self.is_plus_btn_latch = False
    self.is_minus_btn_latch = False
    self.plus_hold_time = 0.0
    self.minus_hold_time = 0.0
    self.plus_did_hold_step = False
    self.minus_did_hold_step = False

    self.prev_distance_btn = False
    # Local enum retained from the DragonPilot controller:
    # 0 = 1 bar/aggressive, 1 = 2 bars/normal, 2 = 3 bars/far.
    self.op_distance_val = 1

    self.stock_lkc_off = True
    self.stock_fcw_off = True
    self.lkas_rdy = True
    self.lkas_latch = True
    self.lkas_btn_rising_edge_seen = False
    self.stock_aeb = False
    self.stock_fcw = False
    self.stock_adas_frontDepartureHUD = False

    self.stock_acc_engaged = False
    self.stock_acc_cmd = 0.0
    self.stock_brake_mag = 0
    self.stock_acc_set_speed = 0.0
    self.stock_acc_brake_state = 0
    self.stock_acc_brake_pump = 0.0
    self.stock_acc_brake_decel = 0.0
    self.stock_acc_brake_rx_frame = -1000000
    self.stock_acc_request_enabled = False
    self.stock_acc_request_lead = False
    self.stock_acc_request_is_accel = False
    self.stock_acc_request_is_decel = False
    self.stock_acc_request_rx_frame = -1000000

    # ACC MAIN is momentary on this Yaris Cross. Availability is therefore a
    # local latch; reading it back from 0x273 would create feedback from our own
    # transmitted ACC_CMD_HUD later in the controller port.
    self.acc_main_button = False
    self.prev_acc_main_button = False
    self.acc_main_latch = False

    # Raw bus-1 signals retained from the tested DragonPilot port.
    self.gas_raw_277 = 0
    self.gas_raw_277_seen = False
    self.engine_rpm_raw_037 = 0

    initialize_hybrid_feedback_state(self)

  def update_raw_can(self, can_packets, frame):
    """Observe raw bus-1 hybrid data and stock-camera bus-2 frames."""
    for _, frames in can_packets:
      for msg in frames:
        if msg.src == 1 and msg.address == 0x277:
          dat = bytes(msg.dat)
          if len(dat) >= 3:
            self.gas_raw_277 = int.from_bytes(dat[1:3], "big")
            self.gas_raw_277_seen = True

        elif msg.src == 1 and msg.address == 0x037:
          dat = bytes(msg.dat)
          if len(dat) >= 5:
            self.engine_rpm_raw_037 = decode_engine_rpm_037(int.from_bytes(dat[3:5], "big"))

        elif msg.src == 1 and msg.address in (0x08C, 0x125, 0x12A, 0x275, 0x2C9):
          apply_hybrid_feedback_frame(self, frame, msg.address, msg.dat)

        elif msg.src == 2 and msg.address == 0x271:
          decoded = decode_stock_acc_brake_271(msg.dat)
          if decoded is not None:
            state, pump, magnitude, decel = decoded
            self.stock_acc_brake_state = state
            self.stock_acc_brake_pump = pump
            self.stock_brake_mag = magnitude
            self.stock_acc_brake_decel = decel
            self.stock_acc_brake_rx_frame = frame

        elif msg.src == 2 and msg.address == 0x273:
          decoded = decode_stock_acc_cmd_273(msg.dat)
          if decoded is not None:
            enabled, lead, is_accel, is_decel, acc_cmd_kph = decoded
            self.stock_acc_request_enabled = enabled
            self.stock_acc_request_lead = lead
            self.stock_acc_request_is_accel = is_accel
            self.stock_acc_request_is_decel = is_decel
            self.stock_acc_cmd = acc_cmd_kph
            self.stock_acc_request_rx_frame = frame

  def _update_cruise_speed_button(self, pressed, was_pressed, is_plus):
    hold_attr = "plus_hold_time" if is_plus else "minus_hold_time"
    stepped_attr = "plus_did_hold_step" if is_plus else "minus_did_hold_step"
    hold_time = getattr(self, hold_attr)
    did_hold_step = getattr(self, stepped_attr)

    if pressed:
      if not was_pressed:
        hold_time = 0.0
        did_hold_step = False
      else:
        hold_time += DT_CTRL
        while hold_time >= SEC_HOLD_TO_STEP_SPEED:
          kph = self.cruise_speed * CV.MS_TO_KPH
          if is_plus:
            kph += 5.0 - (kph % 5.0)
          else:
            kph = max(CRUISE_MIN_KPH, (np.floor(kph / 5.0) - 1.0) * 5.0)
          self.cruise_speed = kph * CV.KPH_TO_MS
          hold_time -= SEC_HOLD_TO_STEP_SPEED
          did_hold_step = True
    elif was_pressed:
      # A short release moves 1 km/h. A held press already moved in 5 km/h steps.
      if not did_hold_step:
        self.cruise_speed += CV.KPH_TO_MS if is_plus else -CV.KPH_TO_MS
      hold_time = 0.0
      did_hold_step = False

    setattr(self, hold_attr, hold_time)
    setattr(self, stepped_attr, did_hold_step)

  def update(self, can_parsers):
    cp = can_parsers[Bus.pt]
    cp_cam = can_parsers[Bus.cam]

    ret = structs.CarState()
    ret_sp = structs.CarStateSP()

    # The rear DNGA wheel-speed field has not been reliable on this car. Keep
    # the tested DragonPilot behavior of using the front signal for all wheels.
    self.parse_wheel_speeds(
      ret,
      cp.vl["WHEEL_SPEED"]["WHEELSPEED_F"],
      cp.vl["WHEEL_SPEED"]["WHEELSPEED_F"],
      cp.vl["WHEEL_SPEED"]["WHEELSPEED_F"],
      cp.vl["WHEEL_SPEED"]["WHEELSPEED_F"],
    )
    ret.standstill = ret.vEgoRaw < 0.01

    can_gear = int(cp.vl["TRANSMISSION"]["GEAR"])
    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(can_gear))

    ret.doorOpen = any((
      cp.vl["METER_CLUSTER"]["MAIN_DOOR"],
      cp.vl["METER_CLUSTER"]["LEFT_FRONT_DOOR"],
      cp.vl["METER_CLUSTER"]["RIGHT_BACK_DOOR"],
      cp.vl["METER_CLUSTER"]["LEFT_BACK_DOOR"],
    ))
    ret.seatbeltUnlatched = bool(
      cp.vl["METER_CLUSTER"]["SEAT_BELT_WARNING"] or
      cp.vl["METER_CLUSTER"]["SEAT_BELT_WARNING2"]
    )

    if ret.doorOpen or ret.seatbeltUnlatched:
      self.is_cruise_latch = False

    if self.gas_raw_277_seen:
      ret.gasPressed = self.gas_raw_277 > 100
    else:
      # Fallback retained only until the bus-1 0x277 observer has seen data.
      ret.gasPressed = not bool(cp.vl["GAS_PEDAL_2"]["GAS_PEDAL_STEP"])

    ret.brakePressed = bool(cp.vl["BRAKE"]["BRAKE_ENGAGED"])

    ret.steeringAngleDeg = cp.vl["STEERING_MODULE"]["STEER_ANGLE"]
    ret.steeringTorque = cp.vl["STEERING_MODULE"]["MAIN_TORQUE"]
    ret.steeringTorqueEps = cp.vl["EPS_SHAFT_TORQUE"]["STEERING_TORQUE"]
    ret.steeringPressed = abs(ret.steeringTorque) > 20

    ret.vEgoCluster = cp.vl["BUTTONS"]["UI_SPEED"] * CV.KPH_TO_MS * HUD_MULTIPLIER

    if cp_cam.can_valid:
      self.stock_adas_frontDepartureHUD = bool(cp_cam.vl["LKAS_HUD"]["FRONT_DEPART"])
      self.stock_aeb = bool(cp_cam.vl["LKAS_HUD"]["AEB_BRAKE"])
      self.stock_fcw = bool(cp_cam.vl["LKAS_HUD"]["AEB_ALARM"])
      self.stock_lkc_off = bool(cp_cam.vl["LKAS_HUD"]["LDA_OFF"])
      self.lkas_rdy = bool(cp_cam.vl["LKAS_HUD"]["LKAS_SET"])
      self.stock_fcw_off = bool(cp_cam.vl["LKAS_HUD"]["FCW_DISABLE"])
      self.stock_acc_set_speed = cp_cam.vl["ACC_CMD_HUD"]["SET_SPEED"]

    ret.stockAeb = self.stock_aeb
    ret.stockFcw = self.stock_fcw

    lkc_button = bool(cp.vl["BUTTONS"]["LKC_BTN"])
    if lkc_button and not self.lkas_btn_rising_edge_seen:
      self.lkas_btn_rising_edge_seen = True
    elif self.lkas_btn_rising_edge_seen and not lkc_button:
      self.lkas_latch = not self.lkas_latch
      self.lkas_btn_rising_edge_seen = False

    self.acc_main_button = bool(cp.vl["PCM_BUTTONS"]["ACC_MAIN"])
    if self.acc_main_button and not self.prev_acc_main_button:
      self.acc_main_latch = not self.acc_main_latch
      if not self.acc_main_latch:
        self.is_cruise_latch = False
    self.prev_acc_main_button = self.acc_main_button

    distance_btn = bool(cp.vl["BUTTONS"]["DISTANCE_BTN"])
    ret.buttonEvents = create_button_events(
      int(distance_btn), int(self.prev_distance_btn), {1: ButtonType.gapAdjustCruise}
    )
    if distance_btn and not self.prev_distance_btn:
      self.op_distance_val -= 1
      if self.op_distance_val < 0:
        self.op_distance_val = 2
    self.prev_distance_btn = distance_btn

    minus_button = bool(cp.vl["PCM_BUTTONS"]["SET_MINUS"])
    plus_button = bool(cp.vl["PCM_BUTTONS"]["RES_PLUS"])

    if self.is_cruise_latch:
      self._update_cruise_speed_button(plus_button, self.is_plus_btn_latch, True)
      self._update_cruise_speed_button(minus_button, self.is_minus_btn_latch, False)

    # SET/RES engages on button release when ACC MAIN is available.
    if self.acc_main_latch and not self.is_cruise_latch:
      if self.is_plus_btn_latch and not plus_button:
        self.is_cruise_latch = True
      elif self.is_minus_btn_latch and not minus_button:
        self.cruise_speed = max(CRUISE_MIN_KPH * CV.KPH_TO_MS, ret.vEgoCluster)
        self.is_cruise_latch = True

    self.is_plus_btn_latch = plus_button
    self.is_minus_btn_latch = minus_button

    if bool(cp.vl["PCM_BUTTONS"]["CANCEL"]) or ret.brakePressed:
      self.is_cruise_latch = False

    self.cruise_speed = float(np.clip(
      self.cruise_speed,
      CRUISE_MIN_KPH * CV.KPH_TO_MS,
      CRUISE_MAX_KPH * CV.KPH_TO_MS,
    ))

    if not self.acc_main_latch:
      self.is_cruise_latch = False

    ret.cruiseState.available = self.acc_main_latch
    ret.cruiseState.enabled = self.is_cruise_latch
    ret.cruiseState.speed = self.cruise_speed
    ret.cruiseState.speedCluster = self.cruise_speed
    ret.cruiseState.standstill = False
    ret.cruiseState.nonAdaptive = False

    ret.leftBlinker = bool(cp.vl["METER_CLUSTER"]["LEFT_SIGNAL"])
    ret.rightBlinker = bool(cp.vl["METER_CLUSTER"]["RIGHT_SIGNAL"])
    ret.genericToggle = bool(cp.vl["RIGHT_STALK"]["GENERIC_TOGGLE"])

    if self.CP.enableBsm:
      ret.leftBlindspot = bool(cp.vl["BSM"]["BSM_CHIME"])
      ret.rightBlindspot = bool(cp.vl["BSM"]["BSM_CHIME"])

    return ret, ret_sp

  @staticmethod
  def get_can_parsers(CP, CP_SP):
    pt_messages = [
      ("WHEEL_SPEED", 50),
      ("BRAKE", 100),
      ("STEERING_MODULE", 100),
      ("EPS_SHAFT_TORQUE", 40),
      ("BUTTONS", 50),
      ("METER_CLUSTER", 15),
      ("PCM_BUTTONS", 30),
      ("TRANSMISSION", math.nan),
      ("RIGHT_STALK", math.nan),
      ("BSM", math.nan),
      ("GAS_PEDAL_2", math.nan),
      ("PCM_BUTTONS_HYBRID", math.nan),
    ]
    cam_messages = [
      ("LKAS_HUD", 20),
      ("ACC_CMD_HUD", 20),
      ("STEERING_LKAS", 40),
      ("ACC_BRAKE", 20),
    ]
    return {
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], pt_messages, 0),
      Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.cam], cam_messages, 2),
    }
