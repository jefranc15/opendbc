from enum import IntFlag

from opendbc.car import Bus, CarSpecs, PlatformConfig, Platforms
from opendbc.car.docs_definitions import CarDocs, SupportType


HUD_MULTIPLIER = 1.04


class DngaFlags(IntFlag):
  HYBRID = 1
  SNG = 2


class CarControllerParams:
  STEER_MAX = 255

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
