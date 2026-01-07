import copy
import numpy as np
from opendbc.car import CanBusBase
from opendbc.car.crc import CRC16_XMODEM
from opendbc.car.hyundai.values import HyundaiFlags, Buttons, CANFD_CAR
from random import randint

from opendbc.car.common.conversions import Conversions as CV

_state = {"wait_timer": 0}

def hyundai_crc8(data: bytes) -> int:
  poly = 0x2F
  crc = 0xFF
  for byte in data:
    crc ^= byte
    for _ in range(8):
      crc = ((crc << 1) ^ poly) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
  return crc ^ 0xFF

class CanBus(CanBusBase):
  def __init__(self, CP, fingerprint=None, lka_steering=None) -> None:
    super().__init__(CP, fingerprint)

    if lka_steering is None:
      lka_steering = CP.flags & HyundaiFlags.CANFD_LKA_STEERING.value if CP is not None else False

    # On the CAN-FD platforms, the LKAS camera is on both A-CAN and E-CAN. LKA steering cars
    # have a different harness than the LFA steering variants in order to split
    # a different bus, since the steering is done by different ECUs.
    self._a, self._e = 1, 0
    if lka_steering:
      self._a, self._e = 0, 1

    self._a += self.offset
    self._e += self.offset
    self._cam = 2 + self.offset

  @property
  def ECAN(self):
    return self._e

  @property
  def ACAN(self):
    return self._a

  @property
  def CAM(self):
    return self._cam


def create_steering_messages(packer, CP, CAN, enabled, lat_active, apply_torque, apply_angle, max_torque):
  common_values = {
    "LKA_MODE": 2,
    "LKA_ICON": 2 if enabled else 1,
    "TORQUE_REQUEST": apply_torque,
    "LKA_ASSIST": 0,
    "STEER_REQ": 1 if lat_active else 0,
    "STEER_MODE": 0,
    "HAS_LANE_SAFETY": 0,  # hide LKAS settings
    "NEW_SIGNAL_2": 0,
    "DAMP_FACTOR": 100,  # can potentially tuned for better perf [3, 200]
  }

  lkas_values = copy.copy(common_values)
  lkas_values["LKA_AVAILABLE"] = 0

  lfa_values = copy.copy(common_values)
  lfa_values["NEW_SIGNAL_1"] = 0

  ret = []
  if CP.flags & HyundaiFlags.CANFD_LKA_STEERING: # hda2
    lkas_msg = "LKAS_ALT" if CP.flags & HyundaiFlags.CANFD_LKA_STEERING_ALT else "LKAS"
    if CP.openpilotLongitudinalControl:
      ret.append(packer.make_can_msg("LFA", CAN.ECAN, lfa_values))
    if CP.isAngleControl: # hda2 angle control
      lkas_values["LKA_MODE"] = 0
      lkas_values["TORQUE_REQUEST"] = 0
      lkas_values["STEER_REQ"] = 0
      lkas_values["LKA_AVAILABLE"] = 3 if lat_active else 0
      lkas_values["LKAS_ANGLE_ACTIVE"] = 2 if lat_active else 0
      lkas_values["ADAS_StrAnglReqVal"] = apply_angle if lat_active else 0
      lkas_values["LKAS_ANGLE_MAX_TORQUE"] = max_torque if lat_active else 0
      lkas_values["LKAS_SIGNAL_1"] = 10
      lkas_values["LKAS_SIGNAL_2"] = 1
      lkas_values["LKAS_SIGNAL_3"] = 1
      lkas_values["LKAS_SIGNAL_4"] = 1
      lkas_values["LKAS_SIGNAL_5"] = 1
      lkas_values["NEW_SIGNAL_3"] = 9
    ret.append(packer.make_can_msg(lkas_msg, CAN.ACAN, lkas_values))
  elif CP.isAngleControl: # non-hda2 angle control
    lfa_values["LKA_MODE"] = 0
    lfa_values["NEW_SIGNAL_1"] = 3 if lat_active else 0
    lfa_values["TORQUE_REQUEST"] = -1024
    lfa_values["LKA_ASSIST"] = 1
    lfa_values["STEER_REQ"] = 0
    lfa_values["NEW_SIGNAL_3"] = 0
    lfa_values["NEW_SIGNAL_5"] = 1
    ret.append(packer.make_can_msg("LFA", CAN.ECAN, lfa_values))
    ang_values = {
      "ADAS_ActvACILvl2Sta": 2 if lat_active else 1,
      "ADAS_StrAnglReqVal": np.clip(apply_angle, -119.9, 119.9) if lat_active else 0,
      "LKAS_ANGLE_MAX_TORQUE": max_torque if lat_active else 0,
    }
    ret.append(packer.make_can_msg("ADAS_CMD_35_10ms", CAN.ECAN, ang_values))
  else:
    lfa_values["LKA_MODE"] = 0
    lfa_values["NEW_SIGNAL_1"] = 3 if lat_active else 0
    lfa_values["NEW_SIGNAL_3"] = 31 if lat_active else 100
    lfa_values["NEW_SIGNAL_5"] = 1
    ret.append(packer.make_can_msg("LFA", CAN.ECAN, lfa_values))

  return ret


def create_steering_messages_adrv(packer, CP, CAN, lat_active, apply_torque, apply_angle, max_torque, frame, lfa_alt, mdps_info, lfa_info, csw_info, ccnc_161):
  # some from carrot
  emergency_steering = False
  if ccnc_161:
    values = ccnc_161
    emergency_steering = values["ALERTS_1"] in [11, 12, 13, 14, 15, 21, 22, 23, 24, 25, 26]

  ret = []
  values = mdps_info
  if CP.isAngleControl:
    if lfa_alt:
      values["LKA_ANGLE_ACTIVE"] = lfa_alt["ADAS_ActvACILvl2Sta"]
  else:
    if lfa_info:
      values["LKA_ACTIVE"] = 1 if lfa_info["STEER_REQ"] == 1 else 0

  if frame % 1000 < 40:
    values["STEERING_COL_TORQUE"] += 220
  ret.append(packer.make_can_msg("MDPS", CAN.CAM, values))

  if frame % 10 == 0:
    if csw_info:
      values = csw_info
      if frame % 1000 < 40:
        values["TOUCH_DETECT"] = 3
        values["TOUCH1"] = 50
        values["TOUCH2"] = 50
        values["CHECKSUM_"] = 0
        dat = packer.make_can_msg("HOD_FD_01_100ms", 0, values)[1]
        values["CHECKSUM_"] = hyundai_crc8(dat[1:8])
      ret.append(packer.make_can_msg("HOD_FD_01_100ms", CAN.CAM, values))

  if CP.isAngleControl:
    if emergency_steering:
      values = lfa_alt
    else:
      values = {}
      values["ADAS_ActvACILvl2Sta"] = 2 if lat_active else 1
      values["ADAS_StrAnglReqVal"] = apply_angle
      values["LKAS_ANGLE_MAX_TORQUE"] = max_torque if lat_active else 0
    ret.append(packer.make_can_msg("ADAS_CMD_35_10ms", CAN.ECAN, values))

    values = lfa_info
    if not emergency_steering:
      values["LKA_MODE"] = 0
      values["LKA_ICON"] = 2 if lat_active else 1
      values["TORQUE_REQUEST"] = -1024
      values["VALUE63"] = 0
      values["STEER_REQ"] = 0
      values["HAS_LANE_SAFETY"] = 0
      values["LKA_ACTIVE"] = 3 if lat_active else 0
      values["VALUE64"] = 0
      values["LKAS_ANGLE_CMD"] = -25.6
      values["LKAS_ANGLE_ACTIVE"] = 0
      values["LKAS_ANGLE_MAX_TORQUE"] = 0
      values["NEW_SIGNAL_1"] = 10
  else:
    values = {}
    values["LKA_MODE"] = 2
    values["LKA_ICON"] = 2 if lat_active else 1
    values["TORQUE_REQUEST"] = apply_torque
    values["STEER_REQ"] = 1 if lat_active else 0
    values["VALUE64"] = 0
    values["HAS_LANE_SAFETY"] = 0
    values["LKA_ACTIVE"] = 0
    values["DAMP_FACTOR"] = 0 if lat_active else 100  

  ret.append(packer.make_can_msg("LFA", CAN.ECAN, values))

  return ret


def create_suppress_lfa(packer, CAN, lfa_block_msg, lka_steering_alt, enabled):
  suppress_msg = "CAM_0x362" if lka_steering_alt else "CAM_0x2a4"

  #msg_bytes = 32 if lka_steering_alt else 24

  #values = {f"BYTE{i}": lfa_block_msg[f"BYTE{i}"] for i in range(3, msg_bytes) if i != 7}
  values = lfa_block_msg

  values["COUNTER"] = lfa_block_msg["COUNTER"]
  
  values["LEFT_LANE_LINE_PROB"] = lfa_block_msg["LEFT_LANE_LINE_PROB"] # maybe double lane above 20
  values["RIGHT_LANE_LINE_PROB"] = lfa_block_msg["RIGHT_LANE_LINE_PROB"] # maybe double lane above 20
  values["LEFT_LANE_TYPE"] = 0   # lfa_block_msg["LEFT_LANE_TYPE"]
  values["RIGHT_LANE_TYPE"] = 0  # lfa_block_msg["RIGHT_LANE_TYPE"]
  values["LEFT_LANE_COLOR"] = lfa_block_msg["LEFT_LANE_COLOR"]
  values["RIGHT_LANE_COLOR"] = lfa_block_msg["RIGHT_LANE_COLOR"]
  values["LEFT_GUARD"] = lfa_block_msg["LEFT_GUARD"]
  values["RIGHT_GUARD"] = lfa_block_msg["RIGHT_GUARD"]
  values["LEFT_BLOCKED"] = lfa_block_msg["LEFT_BLOCKED"]
  values["RIGHT_BLOCKED"] = lfa_block_msg["RIGHT_BLOCKED"]
  values["DISTANCE_1"] = lfa_block_msg["DISTANCE_1"]
  values["DISTANCE_2"] = lfa_block_msg["DISTANCE_2"]
  values["DISTANCE_3"] = lfa_block_msg["DISTANCE_3"]
  values["DISTANCE_4"] = lfa_block_msg["DISTANCE_4"]
  values["DISTANCE_5"] = lfa_block_msg["DISTANCE_5"]
  values["DISTANCE_6"] = lfa_block_msg["DISTANCE_6"]
  values["DISTANCE_7"] = lfa_block_msg["DISTANCE_7"]
  values["DISTANCE_8"] = lfa_block_msg["DISTANCE_8"]
  values["SET_ME_0"] = 0
  values["SET_ME_0_2"] = 0
  values["LEFT_LANE_LINE"] = 0 if enabled else 3
  values["RIGHT_LANE_LINE"] = 0 if enabled else 3
  return packer.make_can_msg(suppress_msg, CAN.ACAN, values)


def create_buttons(packer, CP, CAN, CS, btn, reset = None, lda_btn = None):
  if CS.cruise_buttons[-1] != Buttons.NONE or CS.main_buttons[-1] or CS.lfa_buttons[-1]:
    _state['wait_timer'] = 20 if CP.carFingerprint not in CANFD_CAR else 30
    return []
  elif _state['wait_timer'] > 0:
    _state['wait_timer'] -= 1
    return []

  if reset:
    values = CS.cruise_btn_info
    bus = CAN.ECAN if CP.flags & HyundaiFlags.CANFD_LKA_STEERING else CAN.CAM
  elif lda_btn:
    values = CS.cruise_btn_info
    values["LDA_BTN"] = 1
    values["SET_ME_1"] = 1
    values["COUNTER"] = (values["COUNTER"] + 1) % 0x10
    bus = CAN.ECAN if CP.flags & HyundaiFlags.CANFD_LKA_STEERING else CAN.CAM
    values["_CHECKSUM"] = 0
    dat = packer.make_can_msg("CRUISE_BUTTONS", bus, values)[1]
    values["_CHECKSUM"] = hyundai_crc8(dat[1:8])
  else:
    values = CS.cruise_btn_info
    values["CRUISE_BUTTONS"] = btn
    values["SET_ME_1"] = 1
    values["COUNTER"] = (values["COUNTER"] + 1) % 0x10

    bus = CAN.ECAN if CP.flags & HyundaiFlags.CANFD_LKA_STEERING else CAN.CAM

    values["_CHECKSUM"] = 0
    dat = packer.make_can_msg("CRUISE_BUTTONS", bus, values)[1]
    values["_CHECKSUM"] = hyundai_crc8(dat[1:8])

  return packer.make_can_msg("CRUISE_BUTTONS", bus, values)


def create_acc_cancel(packer, CP, CAN, cruise_info_copy):
  # TODO: why do we copy different values here?
  if CP.flags & HyundaiFlags.CANFD_CAMERA_SCC.value:
    values = {s: cruise_info_copy[s] for s in [
      "COUNTER",
      "CHECKSUM",
      "NEW_SIGNAL_1",
      "MainMode_ACC",
      "ACCMode",
      "ZEROS_9",
      "CRUISE_STANDSTILL",
      "ZEROS_5",
      "DISTANCE_SETTING",
      "VSetDis",
    ]}
  else:
    values = {s: cruise_info_copy[s] for s in [
      "COUNTER",
      "CHECKSUM",
      "ACCMode",
      "VSetDis",
      "CRUISE_STANDSTILL",
    ]}
  values.update({
    "ACCMode": 4,
    "aReqRaw": 0.0,
    "aReqValue": 0.0,
  })
  return packer.make_can_msg("SCC_CONTROL", CAN.ECAN, values)


def create_lfahda_cluster(packer, CAN, enabled, long_active, lat_active, adrv):
  if adrv:
    values = {
      "HDA_CntrlModSta": 2 if long_active else 0,
      "HDA_LFA_SymSta": 2 if lat_active else 0,
    }
  else:
    values = {
      "HDA_CntrlModSta": 2 if enabled else 0,
      "HDA_LFA_SymSta": 2 if enabled else 0,
    }
  return packer.make_can_msg("LFAHDA_CLUSTER", CAN.ECAN, values)


def create_acc_control(packer, CAN, enabled, accel_last, accel, stopping, gas_override, set_speed, hud_control):
  jerk = 5
  jn = jerk / 50
  if not enabled or gas_override:
    a_val, a_raw = 0, 0
  else:
    a_raw = accel
    a_val = np.clip(accel, accel_last - jn, accel_last + jn)

  values = {
    "ACCMode": 0 if not enabled else (2 if gas_override else 1),
    "MainMode_ACC": 1,
    "StopReq": 1 if stopping else 0,
    "aReqValue": a_val,
    "aReqRaw": a_raw,
    "VSetDis": set_speed,
    "JerkLowerLimit": jerk if enabled else 1,
    "JerkUpperLimit": 3.0,

    "ACC_ObjDist": 1,
    "ObjValid": 0,
    "OBJ_STATUS": 2,
    "SET_ME_2": 0x4,
    "SET_ME_3": 0x3,
    "SET_ME_TMP_64": 0x64,
    "DISTANCE_SETTING": hud_control.leadDistanceBars,
  }

  return packer.make_can_msg("SCC_CONTROL", CAN.ECAN, values)


def create_spas_messages(packer, CAN, left_blink, right_blink):
  ret = []

  values = {
  }
  ret.append(packer.make_can_msg("SPAS1", CAN.ECAN, values))

  blink = 0
  if left_blink:
    blink = 3
  elif right_blink:
    blink = 4
  values = {
    "BLINKER_CONTROL": blink,
  }
  ret.append(packer.make_can_msg("SPAS2", CAN.ECAN, values))

  return ret


def create_fca_warning_light(CP, packer, CAN, frame):
  ret = []

  if CP.flags & HyundaiFlags.CAMERA_SCC.value or CP.adrvControl:
    return ret

  if frame % 2 == 0:
    values = {
      'AEB_SETTING': 0x1,  # show AEB disabled icon
      'SET_ME_2': 0x2,
      'SET_ME_FF': 0xff,
      'SET_ME_FC': 0xfc,
      'SET_ME_9': 0x9,
    }
    ret.append(packer.make_can_msg("ADRV_0x160", CAN.ECAN, values))
  return ret


def hkg_can_fd_checksum(address: int, sig, d: bytearray) -> int:
  crc = 0
  for i in range(2, len(d)):
    crc = ((crc << 8) ^ CRC16_XMODEM[(crc >> 8) ^ d[i]]) & 0xFFFF
  crc = ((crc << 8) ^ CRC16_XMODEM[(crc >> 8) ^ ((address >> 0) & 0xFF)]) & 0xFFFF
  crc = ((crc << 8) ^ CRC16_XMODEM[(crc >> 8) ^ ((address >> 8) & 0xFF)]) & 0xFFFF
  if len(d) == 8:
    crc ^= 0x5F29
  elif len(d) == 16:
    crc ^= 0x041D
  elif len(d) == 24:
    crc ^= 0x819D
  elif len(d) == 32:
    crc ^= 0x9F5B
  return crc


def create_tcs_messages(packer, CAN, CS):
  ret = []
  if CS.tcs_373_info:
    values = CS.tcs_373_info
    values["DriverBraking"] = 0
    values["DriverBrakingLowSens"] = 0
    ret.append(packer.make_can_msg("TCS", CAN.CAM, values))
  return ret


def create_steering_wheel(packer, CP, CAN, cs_wheel_info):
  values = cs_wheel_info
  values["HOD_Dir_Status"] = 3
  values["TOUCH1"] = randint(35, 50)
  values["TOUCH2"] = randint(35, 50)
  values["COUNTER"] = (values["COUNTER"] + 1) % 0x10

  bus = CAN.ECAN if CP.flags & HyundaiFlags.CANFD_LKA_STEERING else CAN.CAM

  values["CHECKSUM_"] = 0
  dat = packer.make_can_msg("HOD_FD_01_100ms", bus, values)[1]
  values["CHECKSUM_"] = hyundai_crc8(dat[1:8])

  return packer.make_can_msg("HOD_FD_01_100ms", bus, values)

# carrot
def create_acc_control_scc_adrv(packer, CAN, enabled, accel_last, accel, stopping, gas_override, set_speed, hud_control, hyundai_jerk, CS):
  acc_mode = 0 if not enabled else (2 if gas_override else 1)

  jerk_u = hyundai_jerk.jerk_u
  jerk_l = hyundai_jerk.jerk_l
  jerk = 5
  jn = jerk / 50
  if not enabled or gas_override:
    a_val, a_raw = 0, 0
  else:
    a_raw = accel
    a_val = np.clip(accel, accel_last - jn, accel_last + jn)

  values = CS.cruise_info
  if False:
    values["ACCMode"] = acc_mode
    values["MainMode_ACC"] = 1
    values["StopReq"] = 1 if stopping else 0  # 1: Stop control is required, 2: Not used, 3: Error Indicator
    values["aReqValue"] = a_val
    values["aReqRaw"] = a_raw
    values["VSetDis"] = set_speed
    #values["JerkLowerLimit"] = jerk if enabled else 1
    #values["JerkUpperLimit"] = 3.0
    values["JerkLowerLimit"] = jerk_l if enabled else 1
    values["JerkUpperLimit"] = 2.0 if stopping else jerk_u
    values["DISTANCE_SETTING"] = hud_control.leadDistanceBars # + 5
    #values["DISTANCE_SETTING"] = hud_control.leadDistanceBars  + 5

    #values["ACC_ObjDist"] = 1
    #values["ObjValid"] = 0
    #values["OBJ_STATUS"] =  2
    values["NSCCOper"] = 1 if enabled else 0 # 0: off, 1: Ready, 2: Act, 3: Error Indicator
    values["NSCCOnOff"] = 2  # 0: Default, 1: Off, 2: On, 3: Invalid
    #values["SET_ME_3"] = 0x3  # objRelsped와 충돌
    #values["ACC_ObjLatPos"] = - hud_control.leadDPath
    values["DriveMode"] = 0 # 0: Default, 1: Comfort Mode, 2:Normal mode, 3:Dynamic mode, reserved

    hud_lead_info = 0
    if hud_control.leadVisible:
      hud_lead_info = 1 if values["ACC_ObjRelSpd"] > 0 else 2
    values["HUD_LEAD_INFO"] = hud_lead_info  #1: in-path object detected(uncontrollable), 2: controllable long, 3: controllable long & lat, ... reserved

    values["DriverAlert"] = 0   # 1: SCC Disengaged, 2: No SCC Engage condition, 3: SCC Disenganed when the vehicle stops

    values["TARGET_DISTANCE"] = CS.out.vEgo * 1.0 + 4.0


    # 이거안하면 정지중 뒤로 밀리는 현상 발생하는듯.. (신호정지중에 뒤로 밀리는 경험함.. 시험해봐야)
    if values["InfoDisplay"] != 5: #5: Front Car Departure Notice
      values["InfoDisplay"] = 4 if stopping and CS.out.aEgo > -0.3 else 0  # 1: SCC Mode, 2: Convention Cruise Mode, 3: Object disappered at low speed, 4: Available to resume acceleration control, 5: Front vehicle departure notice, 6: Reserved, 7: Invalid

    values["TakeOverReq"] = 0    # 1: Takeover request, 2: Not used, 3: Error indicator , 이것이 켜지면 가속을 안하는듯함.
    #values["NEW_SIGNAL_4"] = 9 if hud_control.leadVisible else 0
    # AccelLimitBandUpper, Lower
    values["SysFailState"] = 0    # 1: Performance degredation, 2: system temporairy unavailble, 3: SCC Service required , 눈이 묻어 레이더오류시... 2가 됨. 이때 가속을 안함...

    values["AccelLimitBandUpper"] = 0.0   # 이값이 1.26일때 가속을 안하는 증상이 보임.. 
    values["AccelLimitBandLower"] = 0.0

  return packer.make_can_msg("SCC_CONTROL", CAN.ECAN, values)


# carrot
def create_ccnc_messages(CP, packer, CAN, frame, CC, CS, hud_control, left_lane_warning, right_lane_warning, canfd_debug, MainMode_ACC_trigger, LFA_trigger, hdp_use):
  ret = []
  if CP.flags & HyundaiFlags.CAMERA_SCC.value or CP.adrvControl:
    if frame % 2 == 0:
      if CS.adrv_160_info:
        values = CS.adrv_160_info
        values["LFA_FAULT"] = 0
        ret.append(packer.make_can_msg("ADRV_0x160", CAN.ECAN, values))

      if CS.cruise_buttons_msg:
        values = CS.cruise_buttons_msg
        if MainMode_ACC_trigger > 0:
          values["ADAPTIVE_CRUISE_MAIN_BTN"] = 1
        elif LFA_trigger > 0:
          values["LFA_BTN"] = 1
        ret.append(packer.make_can_msg(CS.cruise_btns_msg_canfd, CAN.CAM, values))


    if frame % 5 == 0:
      if CS.adrv_161_info:
        main_enabled = CS.out.cruiseState.available
        cruise_enabled = CS.acc_active
        lat_enabled = CC.enabled
        lat_active = CC.latActive
        nav_active = True

        hdp_active = True if hdp_use and cruise_enabled else False

        values = CS.adrv_161_info

        values["SETSPEED"] = (6 if hdp_active else 3 if cruise_enabled else 1) if main_enabled else 0
        values["SETSPEED_HUD"] = (5 if hdp_active else 3 if cruise_enabled else 1) if main_enabled else 0
        set_speed_in_units = hud_control.setSpeed * (CV.MS_TO_KPH if CS.is_metric else CV.MS_TO_MPH)
        values["vSetDis"] = int(set_speed_in_units + 0.5)

        values["DISTANCE"] = 4 if hdp_active else hud_control.leadDistanceBars
        values["DISTANCE_LEAD"] = 2 if cruise_enabled and hud_control.leadVisible else 1 if main_enabled and hud_control.leadVisible else 0
        values["DISTANCE_CAR"] = 3 if hdp_active else 2 if cruise_enabled else 1 if main_enabled else 0
        values["DISTANCE_SPACING"] = 5 if hdp_active else 1 if cruise_enabled else 0

        values["TARGET"] = 1 if main_enabled else 0
        values["TARGET_DISTANCE"] = int(hud_control.leadDistance)

        #values["BACKGROUND"] = 6 if CS.paddle_button_prev > 0 else 1 if cruise_enabled else 3 if main_enabled else 7
        values["CENTERLINE"] = 1 if lat_enabled else 0
        values["CAR_CIRCLE"] = 2 if hdp_active else 1 if cruise_enabled else 0

        values["NAV_ICON"] = 2 if nav_active else 0
        values["HDA_ICON"] = 5 if hdp_active else 2 if cruise_enabled else 1 if main_enabled else 0
        values["LFA_ICON"] = 5 if hdp_active else 2 if lat_active else 1 if lat_enabled else 0
        values["LKA_ICON"] = 4 if lat_active else 3 if lat_enabled else 0
        values["FCA_ALT_ICON"] = 0

        if values["ALERTS_2"] in [1, 2, 5, 10, 21, 22]:  # 10,21,22: 운전자모니터 알람/경고
          values["ALERTS_2"] = 0
          values["DAW_ICON"] = 0

        values["SOUNDS_1"] = 0  # 운전자모니터경고음.
        values["SOUNDS_2"] = 0  # 2: STEER중지 경고후에도 사운드가 나옴.
        values["SOUNDS_4"] = 0  # 차선변경알림? 에이 그냥0으로..

        if values["ALERTS_3"] in [3, 4, 13, 17, 19, 26, 7, 8, 9, 10]:
          values["ALERTS_3"] = 0
          values["SOUNDS_3"] = 0

        if values["ALERTS_5"] in [1, 2, 4, 5]:
          values["ALERTS_5"] = 0

        if values["ALERTS_5"] in [11]:
          values["ALERTS_5"] = 0

        curvature = round(CS.out.steeringAngleDeg / 3)

        values["LANELINE_CURVATURE"] = (min(abs(curvature), 15) + (-1 if curvature < 0 else 0)) if lat_active else 0
        values["LANELINE_CURVATURE_DIRECTION"] = 1 if curvature < 0 and lat_active else 0

        # lane_color = 6 if lat_active else 2 
        lane_color = 2 # 6: green, 2: white, 4: yellow
        if hud_control.leftLaneDepart:
          values["LANELINE_LEFT"] = 4 if (frame // 50) % 2 == 0 else 1
        else:
          values["LANELINE_LEFT"] = lane_color if hud_control.leftLaneVisible else 0
        if hud_control.rightLaneDepart:
          values["LANELINE_RIGHT"] = 4 if (frame // 50) % 2 == 0 else 1
        else:
          values["LANELINE_RIGHT"] = lane_color if hud_control.rightLaneVisible else 0
        #values["LANELINE_LEFT_POSITION"] = 15
        #values["LANELINE_RIGHT_POSITION"] = 15

        values["LCA_LEFT_ARROW"] = 2 if CS.out.leftBlinker else 0
        values["LCA_RIGHT_ARROW"] = 2 if CS.out.rightBlinker else 0

        values["LCA_LEFT_ICON"] = 1 if CS.out.leftBlindspot else 2
        values["LCA_RIGHT_ICON"] = 1 if CS.out.rightBlindspot else 2

        ret.append(packer.make_can_msg("ADRV_0x161", CAN.ECAN, values))

      if CS.adrv_200_info:
        values = CS.adrv_200_info
        ret.append(packer.make_can_msg("ADRV_0x200", CAN.ECAN, values))

      if CS.adrv_1ea_info:
        values = CS.adrv_1ea_info
        # values["HDA_MODE1"] = 8
        # values["HDA_MODE2"] = 2
        if values['LF_DETECT'] == 0 and hud_control.leadLeftDist > 0:
          values['LF_DETECT'] = 3 if hud_control.leadLeftDist > 30 else 4
          values['LF_DETECT_DISTANCE'] = hud_control.leadLeftDist
          values['LF_DETECT_LATERAL'] = hud_control.leadLeftLat
        if values['RF_DETECT'] == 0 and hud_control.leadRightDist > 0:
          values['RF_DETECT'] = 3 if hud_control.leadRightDist > 30 else 4
          values['RF_DETECT_DISTANCE'] = hud_control.leadRightDist
          values['RF_DETECT_LATERAL'] = hud_control.leadRightLat

        #values["CURRENT_LANE_NUMBER"]  차량이 현재 몇번째 차선에 있는 나타낸 표시 같음 약간의 시간차 있음. SET_ME_FF 대신 이거 같음
        #values["TOTAL_LANE_COUNT"]  전체 레인 숫자 레인 갯수가 늘어나면 이것도 늘어남 추측이 맞는듯

        ret.append(packer.make_can_msg("ADRV_0x1ea", CAN.ECAN, values))

      if CS.adrv_162_info:
        values = CS.adrv_162_info
        if hud_control.leadDistance > 0:
          values["FF_DISTANCE"] = hud_control.leadDistance
          #values["FF_DETECT"] = 11 if hud_control.leadRelSpeed > -0.1 else 12  # bicycle
          #values["FF_DETECT"] = 5 if hud_control.leadRelSpeed > -0.1 else 6 # truck
          ff_type = 3 if hud_control.leadRadar == 1 else 13
          values["FF_DETECT"] = ff_type if hud_control.leadRelSpeed > -0.1 else ff_type + 1
          #values["FF_DETECT_LAT"] = - hud_control.leadDPath

        if values['LF_DETECT'] == 0 and hud_control.leadLeftDist > 0:
          values['LF_DETECT'] = 3 if hud_control.leadLeftDist > 30 else 4
          values['LF_DETECT_DISTANCE'] = hud_control.leadLeftDist
          values['LF_DETECT_LATERAL'] = hud_control.leadLeftLat
        if values['RF_DETECT'] == 0 and hud_control.leadRightDist > 0:
          values['RF_DETECT'] = 3 if hud_control.leadRightDist > 30 else 4
          values['RF_DETECT_DISTANCE'] = hud_control.leadRightDist
          values['RF_DETECT_LATERAL'] = hud_control.leadRightLat
        if values['LR_DETECT'] == 0 and hud_control.leadLeftDist2 > 0:
          values['LR_DETECT'] = 4
          values['LR_DETECT_DISTANCE'] = 2
          values['LR_DETECT_LATERAL'] = hud_control.leadLeftLat2
        if values['RR_DETECT'] == 0 and hud_control.leadRightDist2 > 0:
          values['RR_DETECT'] = 4
          values['RR_DETECT_DISTANCE'] = 2
          values['RR_DETECT_LATERAL'] = hud_control.leadRightLat2

        if (left_lane_warning and not CS.out.leftBlinker) or (right_lane_warning and not CS.out.rightBlinker):
          values["VIBRATE"] = 1
        ret.append(packer.make_can_msg("ADRV_0x162", CAN.ECAN, values))

    if frame % 20 == 0:
      if CS.adrv_345_info:
        values = CS.adrv_345_info
        # values['SET_ME_15'] = 0x15
        ret.append(packer.make_can_msg("ADRV_0x345", CAN.ECAN, values))

    if frame % 100 == 0:
      if CS.adrv_1da_info:
        values = CS.adrv_1da_info
        # values['SET_ME_22'] = 0x22
        # values['SET_ME_41'] = 0x41
        ret.append(packer.make_can_msg("ADRV_0x1da", CAN.ECAN, values))

    if frame % 20 == 0: # 아직 시험중..
      if CS.hda_4a3_info:
        values = CS.hda_4a3_info
        if canfd_debug == 5:
          values["SIGNAL_0"] = 5
          values["NEW_SIGNAL_1"] = 4
          values["SPEED_LIMIT"] = 80
          values["NEW_SIGNAL_3"] = 154
          values["NEW_SIGNAL_4"] = 9
          values["NEW_SIGNAL_5"] = 0
          values["NEW_SIGNAL_6"] = 256

        ret.append(packer.make_can_msg("HDA_4A3_INFO", CAN.CAM, values))

  return ret

def create_adrv_messages(CP, packer, CAN, frame):
  # messages needed to car happy after disabling
  # the ADAS Driving ECU to do longitudinal control

  ret = []

  if not CP.flags & HyundaiFlags.CAMERA_SCC.value:
    values = {}

    ret.extend(create_fca_warning_light(CP, packer, CAN, frame))
    if frame % 5 == 0:
      values = {
        'HDA_MODE1': 0x8,
        'HDA_MODE2': 0x1,
        #'SET_ME_1C': 0x1c,
        'SET_ME_FF': 0xff,
        #'SET_ME_TMP_F': 0xf,
        #'SET_ME_TMP_F_2': 0xf,
        #'DATA26': 1,  #1
        #'DATA32': 5,  #5
      }
      ret.append(packer.make_can_msg("ADRV_0x1ea", CAN.ECAN, values))

      values = {
        'SET_ME_E1': 0xe1,
        #'SET_ME_3A': 0x3a,
        'TauGapSet' : 1,
        'NEW_SIGNAL_2': 3,
      }
      ret.append(packer.make_can_msg("ADRV_0x200", CAN.ECAN, values))

    if frame % 20 == 0:
      values = {
        'SET_ME_15': 0x15,
      }
      ret.append(packer.make_can_msg("ADRV_0x345", CAN.ECAN, values))

    if frame % 100 == 0:
      values = {
        'SET_ME_22': 0x22,
        'SET_ME_41': 0x41,
      }
      ret.append(packer.make_can_msg("ADRV_0x1da", CAN.ECAN, values))

  return ret