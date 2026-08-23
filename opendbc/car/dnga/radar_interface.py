from opendbc.car.interfaces import RadarInterfaceBase


class RadarInterface(RadarInterfaceBase):
  """The Yaris Cross DNGA port uses model/camera leads, not a CAN radar."""
