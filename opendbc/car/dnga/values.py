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
  ACCEL_MIN = -3.5
  ACCEL_MAX = 1.6

  def __init__(self, CP):
    pass


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
