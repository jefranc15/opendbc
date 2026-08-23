from opendbc.car.interfaces import CarControllerBase


class CarController(CarControllerBase):
  """Non-transmitting bring-up controller.

  This intentionally sends no CAN while the DragonPilot V3.3R4 controller is
  being adapted to the current sunnypilot/opendbc controller API. Keeping a
  real CarController class here lets fingerprinting and CarState be exercised
  without accidentally falling back to BukaPilot longitudinal behavior.
  """

  def update(self, CC, CC_SP, CS, now_nanos):
    new_actuators = CC.actuators.as_builder()
    new_actuators.torqueOutputCan = 0
    self.frame += 1
    return new_actuators, []
