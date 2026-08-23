from opendbc.car.common.conversions import Conversions as CV


class SetDistance:
  aggressive = 0
  normal = 1
  far = 2


def compute_set_distance(state):
  if state == SetDistance.aggressive:
    return 2
  if state == SetDistance.normal:
    return 1
  return 0


def lkc_checksum(addr, dat):
  return (addr + len(dat) + 1 + 1 + sum(dat)) & 0xFF


def dnga_checksum(addr, dat):
  return (addr + len(dat) + 1 + 2 + sum(dat)) & 0xFF


def create_can_steer_command(packer, steer, steer_req, raw_cnt):
  values = {
    "STEER_REQ": steer_req,
    "STEER_CMD": -steer if steer_req else 0,
    "COUNTER": raw_cnt,
    "SET_ME_1": 1,
    "SET_ME_1_2": 1,
  }
  # Current opendbc CANPacker returns (address, data, bus).
  dat = packer.make_can_msg("STEERING_LKAS", 0, values)[1]
  values["CHECKSUM"] = lkc_checksum(0x1D0, dat[:-1])
  return packer.make_can_msg("STEERING_LKAS", 0, values)


def dnga_create_brake_command(packer, brake_state, pump_reaction, brake_mag, idx):
  values = {
    "COUNTER": idx,
    "BRAKE_STATE": brake_state,
    "UNKNOWN_BYTE_2": 0x00,
    "PUMP_REACTION2": pump_reaction,
    "MAGNITUDE": brake_mag,
  }
  dat = packer.make_can_msg("ACC_BRAKE", 0, values)[1]
  values["CHECKSUM"] = dnga_checksum(0x271, dat[:-1])
  return packer.make_can_msg("ACC_BRAKE", 0, values)


def dnga_create_accel_command(packer, set_speed, acc_rdy, enabled, is_lead,
                              des_speed, is_accel, is_decel, set_distance):
  """Build ACC_CMD_HUD/0x273 with explicit stock longitudinal state bits."""
  values = {
    "SET_SPEED": set_speed * CV.MS_TO_KPH,
    "FOLLOW_DISTANCE": compute_set_distance(set_distance) if acc_rdy else 3,
    "IS_LEAD": is_lead,
    "IS_ACCEL": bool(enabled and is_accel),
    "IS_DECEL": bool(enabled and is_decel),
    "SET_ME_1_2": acc_rdy,
    "SET_ME_1": 1,
    "SET_0_WHEN_ENGAGE": not enabled,
    "SET_1_WHEN_ENGAGE": enabled,
    "ACC_CMD": des_speed * CV.MS_TO_KPH if enabled else 0,
  }
  dat = packer.make_can_msg("ACC_CMD_HUD", 0, values)[1]
  values["CHECKSUM"] = dnga_checksum(0x273, dat[:-1])
  return packer.make_can_msg("ACC_CMD_HUD", 0, values)


def dnga_create_hud(packer, lkas_rdy, enabled, llane_visible, rlane_visible,
                    ldw, fcw, aeb, front_depart, ldp_off, fcw_off):
  values = {
    "LKAS_SET": lkas_rdy,
    "LKAS_ENGAGED": enabled,
    "LDA_ALERT": ldw,
    "LDA_OFF": ldp_off,
    "LANE_RIGHT_DETECT": rlane_visible,
    "LANE_LEFT_DETECT": llane_visible,
    "SET_ME_X02": 0x2,
    "AEB_ALARM": fcw,
    "AEB_BRAKE": aeb,
    "FRONT_DEPART": front_depart,
    "FCW_DISABLE": fcw_off,
  }
  dat = packer.make_can_msg("LKAS_HUD", 0, values)[1]
  values["CHECKSUM"] = dnga_checksum(0x274, dat[:-1])
  return packer.make_can_msg("LKAS_HUD", 0, values)
