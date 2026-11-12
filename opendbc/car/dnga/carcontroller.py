from numpy import clip, interp

from opendbc.can import CANPacker
from opendbc.car import Bus
from opendbc.car.dnga.dngacan import (
  create_can_steer_command,
  dnga_create_accel_command,
  dnga_create_brake_command,
  dnga_create_hud,
)
from opendbc.car.dnga.dnga_hybrid_feedback import hybrid_brake_feedback_state, hybrid_feedback_snapshot
from opendbc.car.dnga.longitudinal import LongitudinalController, R2_MOVING_MIN_SPEED
from opendbc.car.dnga.values import BrakeState, CarControllerParams
from opendbc.car.interfaces import CarControllerBase


def apply_dnga_steer_torque_limits(apply_torque, apply_torque_last, driver_torque, blinker_on, limits):
  reduced_torque_mult = 10 if blinker_on else 1.5
  driver_max_torque = 255 + driver_torque * reduced_torque_mult
  driver_min_torque = -255 - driver_torque * reduced_torque_mult

  max_steer_allowed = clip(driver_max_torque, 0, 255)
  min_steer_allowed = clip(driver_min_torque, -255, 0)
  apply_torque = clip(apply_torque, min_steer_allowed, max_steer_allowed)

  if apply_torque_last > 0:
    apply_torque = clip(
      apply_torque,
      max(apply_torque_last - limits.STEER_DELTA_DOWN, -limits.STEER_DELTA_UP),
      apply_torque_last + limits.STEER_DELTA_UP,
    )
  else:
    apply_torque = clip(
      apply_torque,
      apply_torque_last - limits.STEER_DELTA_UP,
      min(apply_torque_last + limits.STEER_DELTA_DOWN, limits.STEER_DELTA_UP),
    )
  return int(round(float(apply_torque)))


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP, CP_SP):
    super().__init__(dbc_names, CP, CP_SP)
    self.last_steer = 0
    self.steer_rate_limited = False
    self.params = CarControllerParams(CP)
    self.packer = CANPacker(dbc_names[Bus.pt])
    self.stockLdw = False
    self.longitudinal = LongitudinalController()

  def _apply_brake_propulsion_interlock(self, command, CS, frame):
    """Final physical-brake acceleration veto for the selected controller.

    0x275 brake request plus 0x08C friction remain a hard physical-braking
    acceleration veto. Raw torque request/actual channels have no authority.
    """
    feedback = hybrid_feedback_snapshot(CS, frame)
    fresh, physical_braking = hybrid_brake_feedback_state(feedback)

    if command.brake_state == BrakeState.HOLD:
      # Preserve the inherited confirmed-standstill HOLD mode bits and target.
      return command

    hydraulic = command.brake_state in (BrakeState.BRAKING, BrakeState.CREEPING)
    release_pump = command.pump < 0.0

    # One controller object retains the historical low-speed SNG logic.
    # Only its authorized moving-brake/release phase owns a below-vEgo
    # 0x273 target. All SNG/CREEPING/low-speed commands keep the original
    # neutral hydraulic target at current speed.
    moving_brake_phase = (
      CS.out.vEgo >= R2_MOVING_MIN_SPEED
      and not CS.out.standstill
      and not self.longitudinal.sng_armed
      and not self.longitudinal.stop_hold
      and self.longitudinal.sng_release_count == 0
      and (self.longitudinal.r2_moving or frame < self.longitudinal.r2_release_until_frame)
    )

    if physical_braking:
      # Brake-specific 0x275 / 0x08C feedback blocks renewed propulsion.
      self.longitudinal.handoff_pending = True
      self.longitudinal.handoff_ready_counter = 0
      self.longitudinal.kinematic_ready_counter = 0

    if hydraulic or release_pump:
      command.is_accel = False
      command.is_decel = True
      if not moving_brake_phase:
        # Preserve prior SNG and legacy hydraulic CAN behavior.
        command.speed = float(CS.out.vEgo)
        self.longitudinal.speed_offset = 0.0
    elif not fresh or physical_braking or self.longitudinal.handoff_pending:
      # After 0x271 releases, neutralize both 0x273 mode bits until the
      # inherited HEV handoff and brake-feedback gates allow propulsion.
      command.is_accel = False
      command.is_decel = False
      command.speed = float(CS.out.vEgo)
      self.longitudinal.speed_offset = 0.0

    return command

  def update(self, CC, CC_SP, CS, now_nanos):
    del CC_SP, now_nanos

    enabled = CC.enabled
    lat_active = CC.latActive
    long_active = CC.longActive
    actuators = CC.actuators
    pcm_cancel_cmd = CC.cruiseControl.cancel
    hud_control = CC.hudControl

    can_sends = []

    steer_max_interp = interp(CS.out.vEgo, self.params.STEER_BP, self.params.STEER_LIM_TORQ)
    steer_max_interp = max(1.0, steer_max_interp)
    new_steer = int(round(actuators.torque * steer_max_interp))
    blinker_on = CS.out.leftBlinker != CS.out.rightBlinker

    apply_steer = apply_dnga_steer_torque_limits(
      new_steer,
      self.last_steer,
      CS.out.steeringTorqueEps,
      blinker_on,
      self.params,
    )
    if not lat_active and not self.stockLdw:
      apply_steer = 0

    self.steer_rate_limited = new_steer != apply_steer and apply_steer != 0
    self.steer_rate_limited &= not CS.out.steeringPressed

    if self.frame % CarControllerParams.STEER_STEP == 0:
      steer_req = (lat_active or self.stockLdw) and CS.lkas_latch
      can_sends.append(
        create_can_steer_command(
          self.packer,
          apply_steer,
          steer_req,
          self.frame // CarControllerParams.STEER_STEP % 16,
        )
      )

    # Keep the visible SET/RES session alive during a driver-gas override even
    # when sunnypilot drops longActive. The DNGA longitudinal controller still
    # blocks all OP longitudinal actuation while gas is pressed.
    longitudinal_enabled = (
      enabled
      and self.CP.openpilotLongitudinalControl
      and (long_active or CS.out.gasPressed)
    )
    command = self.longitudinal.update(
      longitudinal_enabled,
      CS,
      self.frame,
      actuators.accel,
      pcm_cancel_cmd,
      hud_control.leadVisible,
    )

    if command is not None:
      command = self._apply_brake_propulsion_interlock(command, CS, self.frame)
      can_sends.append(
        dnga_create_accel_command(
          self.packer,
          CS.cruise_speed,
          CS.out.cruiseState.available,
          command.enabled,
          command.lead,
          command.speed,
          command.is_accel,
          command.is_decel,
          CS.op_distance_val,
        )
      )
      can_sends.append(
        dnga_create_brake_command(
          self.packer,
          command.brake_state,
          command.pump,
          command.magnitude,
          self.frame // CarControllerParams.ACC_STEP % 8,
        )
      )
      can_sends.append(
        dnga_create_hud(
          self.packer,
          CS.out.cruiseState.available and CS.lkas_latch,
          enabled,
          hud_control.leftLaneVisible,
          hud_control.rightLaneVisible,
          self.stockLdw,
          CS.stock_fcw,
          CS.stock_aeb,
          CS.stock_adas_frontDepartureHUD,
          CS.stock_lkc_off,
          CS.stock_fcw_off,
        )
      )

    self.last_steer = apply_steer

    new_actuators = actuators.as_builder()
    new_actuators.torque = float(apply_steer / steer_max_interp)
    new_actuators.torqueOutputCan = apply_steer

    self.frame += 1
    return new_actuators, can_sends
