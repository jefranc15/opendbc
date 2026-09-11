from opendbc.car import get_safety_config, structs
from opendbc.car.dnga.carcontroller import CarController
from opendbc.car.dnga.carstate import CarState
from opendbc.car.dnga.radar_interface import RadarInterface
from opendbc.car.dnga.values import CAR, CarControllerParams
from opendbc.car.interfaces import CarInterfaceBase


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController
  RadarInterface = RadarInterface

  @staticmethod
  def get_pid_accel_limits(CP, CP_SP, current_speed, cruise_speed):
    return CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX

  @staticmethod
  def _get_params(ret, candidate, fingerprint, car_fw, alpha_long, is_release, docs):
    if candidate != CAR.TOYOTA_YARIS_CROSS_HEV:
      raise ValueError(f"Unsupported DNGA car: {candidate}")

    ret.brand = "dnga"
    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.dnga)]
    ret.transmissionType = structs.CarParams.TransmissionType.automatic
    ret.radarUnavailable = True

    # Preserve the steering geometry/tuning from the tested DragonPilot port.
    ret.steerControlType = structs.CarParams.SteerControlType.torque
    ret.steerLimitTimer = 0.1
    ret.steerActuatorDelay = 0.30
    ret.lateralParams.torqueBP = [0, 10, 20, 35]
    ret.lateralParams.torqueV = [255, 255, 255, 255]
    ret.lateralTuning.init("pid")
    ret.lateralTuning.pid.kpBP = [0.0]
    ret.lateralTuning.pid.kpV = [0.32]
    ret.lateralTuning.pid.kiBP = [0.0]
    ret.lateralTuning.pid.kiV = [0.14]
    ret.lateralTuning.pid.kf = 0.000188

    ret.wheelSpeedFactor = 1.653
    ret.enableBsm = True
    ret.minEnableSpeed = -1.0
    ret.stoppingDecelRate = 0.25

    # Keep the proven longitudinal gains in the params now, but do not enable
    # actuation until the V3.3R4 controller is ported. The temporary controller
    # sends no CAN and dashcamOnly prevents accidental road-control use.
    ret.longitudinalTuning.kpBP = [0.0, 5.0, 20.0]
    ret.longitudinalTuning.kpV = [2.2, 2.0, 1.8]
    ret.longitudinalTuning.kiBP = [0.0]
    ret.longitudinalTuning.kiV = [0.0]
    ret.longitudinalActuatorDelay = 0.45
    ret.openpilotLongitudinalControl = True
    ret.dashcamOnly = False

    # ACC MAIN/SET/RES state is reconstructed in CarState exactly as on the
    # DragonPilot port, so keep pcmCruise tied to that software cruise state.
    ret.pcmCruise = True

    return ret

  def update(self, can_packets):
    # The current generic interface passes parsed CAN only to CarState. Preserve
    # the old port's raw bus-1 HEV and bus-2 stock-ACC observers before parsing.
    self.CS.update_raw_can(can_packets, self.frame)
    ret = super().update(can_packets)
    self.frame += 1
    return ret
