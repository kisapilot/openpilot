import math

from cereal import car
from opendbc.can.parser import CANParser
from opendbc.car import structs
from opendbc.car.interfaces import RadarInterfaceBase
from opendbc.car.hyundai.values import DBC, CANFD_CAR
from openpilot.common.params import Params

RADAR_START_ADDR = 0x500
RADAR_MSG_COUNT = 32

USE_RADAR_TRACK = Params().get_bool("UseRadarTrack") or (Params().get_bool("ExperimentalLongitudinalEnabled") and int(Params().get("KISALongAlt", encoding="utf8")) not in (1, 2))

# POC for parsing corner radars: https://github.com/commaai/openpilot/pull/24221/

def get_radar_can_parser(CP):
  if USE_RADAR_TRACK or CP.carFingerprint in CANFD_CAR:
    if DBC[CP.carFingerprint]['radar'] is None:
      return None

    messages = [(f"RADAR_TRACK_{addr:x}", 50) for addr in range(RADAR_START_ADDR, RADAR_START_ADDR + RADAR_MSG_COUNT)]
    return CANParser(DBC[CP.carFingerprint]['radar'], messages, 1)

  else:
    messages = [
      ("SCC11", 50),
    ]
    return CANParser(DBC[CP.carFingerprint]['pt'], messages, CP.sccBus)


class RadarInterface(RadarInterfaceBase):
  def __init__(self, CP):
    super().__init__(CP)

    self.CP = CP
    
    if USE_RADAR_TRACK or self.CP.carFingerprint in CANFD_CAR:
      self.updated_messages = set()
      self.trigger_msg = RADAR_START_ADDR + RADAR_MSG_COUNT - 1
      self.track_id = 0

      self.radar_off_can = CP.radarUnavailable
      self.rcp = get_radar_can_parser(CP)
    else:
      self.rcp = get_radar_can_parser(CP)
      self.updated_messages = set()
      self.trigger_msg = 0x420
      self.track_id = 0
      self.radar_off_can = CP.radarUnavailable

  def update(self, can_strings):
    if USE_RADAR_TRACK or self.CP.carFingerprint in CANFD_CAR:
      if self.radar_off_can or (self.rcp is None):
        return super().update(None)
    else:
      if self.radar_off_can:
        return super().update(None)

    vls = self.rcp.update_strings(can_strings)
    self.updated_messages.update(vls)

    if self.trigger_msg not in self.updated_messages:
      return None

    rr = self._update(self.updated_messages)
    self.updated_messages.clear()

    return rr

  def _update(self, updated_messages):
    ret = structs.RadarData()

    if USE_RADAR_TRACK or self.CP.carFingerprint in CANFD_CAR:
      if self.rcp is None:
        return ret

      errors = []

      if not self.rcp.can_valid:
        errors.append("canError")
      ret.errors = errors

      for addr in range(RADAR_START_ADDR, RADAR_START_ADDR + RADAR_MSG_COUNT):
        msg = self.rcp.vl[f"RADAR_TRACK_{addr:x}"]

        if addr not in self.pts:
          self.pts[addr] = car.RadarData.RadarPoint.new_message()
          self.pts[addr].trackId = self.track_id
          self.track_id += 1

        valid = msg['STATE'] in (3, 4)
        if valid:
          azimuth = math.radians(msg['AZIMUTH'])
          self.pts[addr].measured = True
          self.pts[addr].dRel = math.cos(azimuth) * msg['LONG_DIST']
          self.pts[addr].yRel = 0.5 * -math.sin(azimuth) * msg['LONG_DIST']
          self.pts[addr].vRel = msg['REL_SPEED']
          self.pts[addr].aRel = msg['REL_ACCEL']
          self.pts[addr].yvRel = float('nan')

        else:
          del self.pts[addr]

    else:
      cpt = self.rcp.vl
      errors = []
      if not self.rcp.can_valid:
        errors.append("canError")
      ret.errors = errors

      valid = cpt["SCC11"]['ACC_ObjStatus']

      for ii in range(1):
        if valid:
          if ii not in self.pts:
            self.pts[ii] = car.RadarData.RadarPoint.new_message()
            self.pts[ii].trackId = self.track_id
            self.track_id += 1
          self.pts[ii].dRel = cpt["SCC11"]['ACC_ObjDist']  # from front of car
          self.pts[ii].yRel = -cpt["SCC11"]['ACC_ObjLatPos']  # in car frame's y axis, left is negative
          self.pts[ii].vRel = cpt["SCC11"]['ACC_ObjRelSpd']
          self.pts[ii].aRel = float('nan')
          self.pts[ii].yvRel = float('nan')
          self.pts[ii].measured = True
        else:
          if ii in self.pts:
            del self.pts[ii]

    ret.points = list(self.pts.values())
    return ret
