from collections import deque
import copy
import math

from cereal import car
import cereal.messaging as messaging
from openpilot.common.conversions import Conversions as CV
from opendbc.can.parser import CANParser
from opendbc.can.can_define import CANDefine
from openpilot.selfdrive.car.hyundai.hyundaicanfd import CanBus
from openpilot.selfdrive.car.hyundai.values import HyundaiFlags, CAR, DBC, CAN_GEARS, CAMERA_SCC_CAR, \
                                                   CANFD_CAR, Buttons, CarControllerParams, LEGACY_SAFETY_MODE_CAR_ALT, ANGLE_CONTROL_CAR
from openpilot.selfdrive.car.interfaces import CarStateBase
from openpilot.common.params import Params

GearShifter = car.CarState.GearShifter

PREV_BUTTON_SAMPLES = 8
CLUSTER_SAMPLE_RATE = 20  # frames
STANDSTILL_THRESHOLD = 12 * 0.03125 * CV.KPH_TO_MS


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    can_define = CANDefine(DBC[CP.carFingerprint]["pt"])

    self.cruise_buttons = deque([Buttons.NONE] * PREV_BUTTON_SAMPLES, maxlen=PREV_BUTTON_SAMPLES)
    self.main_buttons = deque([Buttons.NONE] * PREV_BUTTON_SAMPLES, maxlen=PREV_BUTTON_SAMPLES)

    self.gear_msg_canfd = "GEAR_ALT" if CP.flags & HyundaiFlags.CANFD_ALT_GEARS else \
                          "GEAR_ALT_2" if CP.flags & HyundaiFlags.CANFD_ALT_GEARS_2 else \
                          "GEAR_SHIFTER"
    if CP.carFingerprint in CANFD_CAR:
      self.shifter_values = can_define.dv[self.gear_msg_canfd]["GEAR"]
    elif self.CP.carFingerprint in CAN_GEARS["use_cluster_gears"]:
      self.shifter_values = can_define.dv["CLU15"]["CF_Clu_Gear"]
    elif self.CP.carFingerprint in CAN_GEARS["use_tcu_gears"]:
      self.shifter_values = can_define.dv["TCU12"]["CUR_GR"]
    else:  # preferred and elect gear methods use same definition
      self.shifter_values = can_define.dv["LVR12"]["CF_Lvr_Gear"]

    self.accelerator_msg_canfd = "ACCELERATOR" if CP.flags & HyundaiFlags.EV else \
                                 "ACCELERATOR_ALT" if CP.flags & HyundaiFlags.HYBRID else \
                                 "ACCELERATOR_BRAKE_ALT"
    self.cruise_btns_msg_canfd = "CRUISE_BUTTONS_ALT" if CP.flags & HyundaiFlags.CANFD_ALT_BUTTONS else \
                                 "CRUISE_BUTTONS"
    self.is_metric = False
    self.buttons_counter = 0

    self.cruise_info = {}
    self.cruise_btn_info = {}

    # On some cars, CLU15->CF_Clu_VehicleSpeed can oscillate faster than the dash updates. Sample at 5 Hz
    self.cluster_speed = 0
    self.cluster_speed_counter = CLUSTER_SAMPLE_RATE

    self.params = CarControllerParams(CP)

    self.lkas_button_on = True
    self.cruise_main_button = 0
    self.mdps_error_cnt = 0
    self.cruiseState_standstill = False

    self.driverAcc_time = 0

    self.prev_cruise_buttons = 0
    self.prev_gap_button = 0
    
    self.steer_anglecorrection = float(int(Params().get("KisaSteerAngleCorrection", encoding="utf8")) * 0.1)
    self.gear_correction = Params().get_bool("JustDoGearD")
    self.set_spd_five = Params().get_bool("SetSpeedFive")
    self.brake_check = False
    self.cancel_check = False
    
    self.cruise_gap = int(Params().get("KisaCruiseGapSet", encoding="utf8"))
    self.is_highway = False
    self.is_set_speed_in_mph = False
    self.cruise_active = False

    # atom
    self.cruise_buttons_time = 0
    self.time_delay_int = 0
    self.VSetDis = 0
    self.clu_Vanz = 0

    # acc button 
    self.prev_acc_active = False
    self.prev_acc_set_btn = False
    self.prev_acc_reset_btn = False
    self.prev_cruise_btn = False
    self.prev_main_btn = False
    self.acc_active = False
    self.cruise_set_speed_kph = 0
    self.cruise_set_mode = int(Params().get("CruiseStatemodeSelInit", encoding="utf8"))
    self.gasPressed = False
    self.cruiseGapSet = 4.0

    self.long_alt = int(Params().get("KISALongAlt", encoding="utf8"))
    self.exp_engage_available = False

    self.exp_long_alt = CP.sccBus <= 0 and CP.carFingerprint in LEGACY_SAFETY_MODE_CAR_ALT and self.CP.openpilotLongitudinalControl
    self.exp_long = (CP.sccBus <= 0 and self.CP.openpilotLongitudinalControl and self.long_alt not in (1, 2)) or self.exp_long_alt
    self.lead_distance = 0
    self.DistSet = 0
    self.obj_valid = 0

    self.sm = messaging.SubMaster(['controlsState'])

  #@staticmethod
  def cruise_speed_button_alt(self):
    self.sm.update(0)
    set_speed_kph = self.cruise_set_speed_kph
    if 1 < round(self.sm['controlsState'].vCruise) < 255:
      set_speed_kph = round(self.sm['controlsState'].vCruise)
      self.cruise_set_speed_kph = set_speed_kph

    if self.cruise_buttons[-1]:
      self.cruise_buttons_time += 1
    else:
      self.cruise_buttons_time = 0
     
    # long press should set scc speed with cluster scc number
    if self.cruise_buttons_time >= 70:
      self.cruise_set_speed_kph = self.VSetDis
      return self.cruise_set_speed_kph

    if self.acc_active and not self.cruise_buttons[-1] and not self.prev_main_btn:
      if not self.prev_acc_set_btn: # first scc active
        self.prev_acc_set_btn = self.acc_active
        self.prev_main_btn = self.acc_active
        self.cruise_set_speed_kph = max(int(round(self.clu_Vanz)), (30 if self.is_metric else 20))
        return self.cruise_set_speed_kph
    elif self.prev_cruise_btn == self.cruise_buttons[-1]:
      return self.cruise_set_speed_kph
    elif self.prev_cruise_btn != self.cruise_buttons[-1]:
      self.prev_cruise_btn = self.cruise_buttons[-1]
      if not self.cruise_active:
        if self.cruise_buttons[-1] == Buttons.GAP_DIST:  # mode change
          self.cruise_set_mode += 1
          if self.cruise_set_mode > 5:
            self.cruise_set_mode = 0
          return None
        elif not self.prev_acc_reset_btn: # first scc active
          self.prev_acc_reset_btn = True
          if self.cruise_buttons[-1] == Buttons.SET_DECEL:
            self.cruise_set_speed_kph = max(int(round(self.clu_Vanz)), (30 if self.is_metric else 20))
          elif self.cruise_buttons[-1] == Buttons.RES_ACCEL:
            self.cruise_set_speed_kph = max(set_speed_kph, int(round(self.clu_Vanz)), (30 if self.is_metric else 20))
          elif self.cruise_buttons[-1] == Buttons.CANCEL:
            self.cruise_set_speed_kph = max(set_speed_kph, int(round(self.clu_Vanz)), (30 if self.is_metric else 20))
          return self.cruise_set_speed_kph
      elif self.cruise_buttons[-1] == Buttons.RES_ACCEL and not self.cruiseState_standstill:   # up 
        if self.set_spd_five:
          set_speed_kph += 5
          if set_speed_kph % 5 != 0:
            set_speed_kph = int(round(set_speed_kph/5)*5)
        else:
          set_speed_kph += 1
      elif self.cruise_buttons[-1] == Buttons.SET_DECEL and not self.cruiseState_standstill:  # dn
        if self.set_spd_five:
          set_speed_kph -= 5
          if set_speed_kph % 5 != 0:
            set_speed_kph = int(round(set_speed_kph/5)*5)
        else:
          set_speed_kph -= 1
      elif self.cruise_buttons[-1] == Buttons.CANCEL and not self.cruiseState_standstill:  # dn
        set_speed_kph = 255

      if set_speed_kph <= 30 and not self.is_set_speed_in_mph:
        set_speed_kph = 30
      elif set_speed_kph <= 20 and self.is_set_speed_in_mph:
        set_speed_kph = 20
      self.cruise_set_speed_kph = set_speed_kph
    else:
      self.prev_cruise_btn = False 

    return set_speed_kph

  def cruise_speed_button_long(self):
    self.sm.update(0)
    set_speed_kph = self.cruise_set_speed_kph
    if 0 < round(self.sm['controlsState'].vCruise) < 255:
      set_speed_kph = round(self.sm['controlsState'].vCruise)

    if self.cruise_buttons[-1]:
      self.cruise_buttons_time += 1
    else:
      self.cruise_buttons_time = 0

    # long press should set scc speed with cluster scc number
    if self.cruise_buttons_time >= 70 and self.cruise_buttons[-1] in (1,2):
      self.cruise_buttons_time = 0
      if self.is_metric:
        if self.cruise_buttons[-1] == 1:
          set_speed_kph += 10
        elif self.cruise_buttons[-1] == 2:
          set_speed_kph -= 10
      else:
        if self.cruise_buttons[-1] == 1:
          set_speed_kph += 5
        elif self.cruise_buttons[-1] == 2:
          set_speed_kph -= 5
      set_speed_kph = max(10, set_speed_kph) if self.is_metric else max(5, set_speed_kph)
      self.cruise_set_speed_kph = int(round(set_speed_kph/10)*10) if self.is_metric else int(round(set_speed_kph/5)*5)
      return self.cruise_set_speed_kph

    if self.prev_cruise_btn == self.cruise_buttons[-1]:
      return self.cruise_set_speed_kph
    elif self.prev_cruise_btn != self.cruise_buttons[-1]:
      self.prev_cruise_btn = self.cruise_buttons[-1]
      if self.cruise_buttons[-1] == Buttons.GAP_DIST and not self.acc_active:  # mode change
        self.cruise_set_mode += 1
        if self.cruise_set_mode > 5:
          self.cruise_set_mode = 0
        return None
      elif not self.prev_acc_set_btn: # first scc active
        self.prev_acc_set_btn = self.exp_engage_available
        if self.cruise_buttons[-1] == Buttons.SET_DECEL:
          self.cruise_set_speed_kph = max(int(round(self.clu_Vanz)), 10 if self.is_metric else 5)
        elif self.cruise_buttons[-1] == Buttons.RES_ACCEL:
          self.cruise_set_speed_kph = max(set_speed_kph, int(round(self.clu_Vanz)), 10 if self.is_metric else 5)
        return self.cruise_set_speed_kph

      if self.cruise_buttons[-1] == Buttons.RES_ACCEL:   # up 
        if self.set_spd_five:
          set_speed_kph += 5
          if set_speed_kph % 5 != 0:
            set_speed_kph = int(round(set_speed_kph/5)*5)
        else:
          set_speed_kph += 1
        if set_speed_kph <= 10 and not self.is_set_speed_in_mph:
          set_speed_kph = 10
        elif set_speed_kph <= 5 and self.is_set_speed_in_mph:
          set_speed_kph = 5

      elif self.cruise_buttons[-1] == Buttons.SET_DECEL:  # dn
        if self.set_spd_five:
          set_speed_kph -= 5
          if set_speed_kph % 5 != 0:
            set_speed_kph = int(round(set_speed_kph/5)*5)
        else:
          set_speed_kph -= 1
        if set_speed_kph <= 10 and not self.is_set_speed_in_mph:
          set_speed_kph = 10
        elif set_speed_kph <= 5 and self.is_set_speed_in_mph:
          set_speed_kph = 5

      self.cruise_set_speed_kph = set_speed_kph
    else:
      self.prev_cruise_btn = False

    return set_speed_kph

  def cruise_speed_button(self):
    self.sm.update(0)
    set_speed_kph = self.cruise_set_speed_kph
    if 1 < round(self.sm['controlsState'].vCruise) < 255:
      set_speed_kph = round(self.sm['controlsState'].vCruise)

    if self.cruise_buttons[-1]:
      self.cruise_buttons_time += 1
    else:
      self.cruise_buttons_time = 0
     
    # long press should set scc speed with cluster scc number
    if self.cruise_buttons_time >= 60:
      self.cruise_set_speed_kph = self.VSetDis
      return self.cruise_set_speed_kph

    if self.prev_cruise_btn == self.cruise_buttons[-1]:
      return self.cruise_set_speed_kph
    elif self.prev_cruise_btn != self.cruise_buttons[-1]:
      self.prev_cruise_btn = self.cruise_buttons[-1]
      if not self.cruise_active:
        if self.cruise_buttons[-1] == Buttons.GAP_DIST:  # mode change
          self.cruise_set_mode += 1
          if self.cruise_set_mode > 5:
            self.cruise_set_mode = 0
          return None
        elif not self.prev_acc_set_btn: # first scc active
          self.prev_acc_set_btn = self.acc_active
          if self.cruise_buttons[-1] == Buttons.SET_DECEL:
            self.cruise_set_speed_kph = max(int(round(self.clu_Vanz)), (30 if self.is_metric else 20))
          elif self.cruise_buttons[-1] == Buttons.RES_ACCEL:
            self.cruise_set_speed_kph = max(set_speed_kph, int(round(self.clu_Vanz)), (30 if self.is_metric else 20))
          return self.cruise_set_speed_kph

      elif self.cruise_buttons[-1] == Buttons.RES_ACCEL and not self.cruiseState_standstill:   # up 
        if self.set_spd_five:
          set_speed_kph += 5
          if set_speed_kph % 5 != 0:
            set_speed_kph = int(round(set_speed_kph/5)*5)
        else:
          set_speed_kph += 1
      elif self.cruise_buttons[-1] == Buttons.SET_DECEL and not self.cruiseState_standstill:  # dn
        if self.set_spd_five:
          set_speed_kph -= 5
          if set_speed_kph % 5 != 0:
            set_speed_kph = int(round(set_speed_kph/5)*5)
        else:
          set_speed_kph -= 1

      if set_speed_kph <= 30 and not self.is_set_speed_in_mph:
        set_speed_kph = 30
      elif set_speed_kph <= 20 and self.is_set_speed_in_mph:
        set_speed_kph = 20

      self.cruise_set_speed_kph = set_speed_kph
    else:
      self.prev_cruise_btn = False

    return set_speed_kph

  def get_tpms(self, unit, fl, fr, rl, rr):
    factor = 0.72519 if unit == 1 else 0.1 if unit == 2 else 1 # 0:psi, 1:kpa, 2:bar
    tpms = car.CarState.TPMS.new_message()
    tpms.unit = unit
    tpms.fl = fl * factor
    tpms.fr = fr * factor
    tpms.rl = rl * factor
    tpms.rr = rr * factor
    return tpms

  def update(self, cp, cp_cam):
    if self.CP.carFingerprint in CANFD_CAR:
      return self.update_canfd(cp, cp_cam)

    cp_scc = cp_cam if self.CP.sccBus == 2 else cp

    self.prev_cruise_main_button = self.cruise_main_button
    self.prev_lkas_button_on = self.lkas_button_on

    ret = car.CarState.new_message()
    cp_cruise = cp_cam if self.CP.carFingerprint in CAMERA_SCC_CAR or self.CP.sccBus == 2 else cp
    self.is_metric = cp.vl["CLU11"]["CF_Clu_SPEED_UNIT"] == 0
    speed_conv = CV.KPH_TO_MS if self.is_metric else CV.MPH_TO_MS

    ret.doorOpen = any([cp.vl["CGW1"]["CF_Gway_DrvDrSw"], cp.vl["CGW1"]["CF_Gway_AstDrSw"],
                        cp.vl["CGW2"]["CF_Gway_RLDrSw"], cp.vl["CGW2"]["CF_Gway_RRDrSw"]])

    ret.seatbeltUnlatched = cp.vl["CGW1"]["CF_Gway_DrvSeatBeltSw"] == 0

    ret.wheelSpeeds = self.get_wheel_speeds(
      cp.vl["WHL_SPD11"]["WHL_SPD_FL"],
      cp.vl["WHL_SPD11"]["WHL_SPD_FR"],
      cp.vl["WHL_SPD11"]["WHL_SPD_RL"],
      cp.vl["WHL_SPD11"]["WHL_SPD_RR"],
    )
    ret.vEgoRaw = (ret.wheelSpeeds.fl + ret.wheelSpeeds.fr + ret.wheelSpeeds.rl + ret.wheelSpeeds.rr) / 4.
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = ret.wheelSpeeds.fl <= STANDSTILL_THRESHOLD and ret.wheelSpeeds.rr <= STANDSTILL_THRESHOLD

    ret.vEgoOP = ret.vEgo
    ret.vEgo = cp.vl["CLU11"]["CF_Clu_Vanz"] * CV.MPH_TO_MS if bool(cp.vl["CLU11"]["CF_Clu_SPEED_UNIT"]) else cp.vl["CLU11"]["CF_Clu_Vanz"] * CV.KPH_TO_MS


    self.cluster_speed_counter += 1
    if self.cluster_speed_counter > CLUSTER_SAMPLE_RATE:
      self.cluster_speed = cp.vl["CLU15"]["CF_Clu_VehicleSpeed"]
      self.cluster_speed_counter = 0

      # Mimic how dash converts to imperial.
      # Sorento is the only platform where CF_Clu_VehicleSpeed is already imperial when not is_metric
      # TODO: CGW_USM1->CF_Gway_DrLockSoundRValue may describe this
      if not self.is_metric and self.CP.carFingerprint not in (CAR.KIA_SORENTO,):
        self.cluster_speed = math.floor(self.cluster_speed * CV.KPH_TO_MPH + CV.KPH_TO_MPH)

    ret.vEgoCluster = self.cluster_speed * speed_conv
    
    ret.standStill = self.CP.standStill

    ret.steeringAngleDeg = cp.vl["SAS11"]["SAS_Angle"] - self.steer_anglecorrection
    ret.steeringRateDeg = cp.vl["SAS11"]["SAS_Speed"]
    ret.yawRate = cp.vl["ESP12"]["YAW_RATE"]
    ret.leftBlinker, ret.rightBlinker = self.update_blinker_from_lamp(
      50, cp.vl["CGW1"]["CF_Gway_TurnSigLh"], cp.vl["CGW1"]["CF_Gway_TurnSigRh"])
    ret.steeringTorque = cp.vl["MDPS12"]["CR_Mdps_StrColTq"]
    ret.steeringTorqueEps = cp.vl["MDPS12"]["CR_Mdps_OutTq"]
    ret.steeringPressed = self.update_steering_pressed(abs(ret.steeringTorque) > self.params.STEER_THRESHOLD, 5)
    self.mdps_error_cnt += 1 if cp.vl["MDPS12"]["CF_Mdps_ToiUnavail"] != 0 else -self.mdps_error_cnt
    ret.steerFaultTemporary = self.mdps_error_cnt > 100 #cp.vl["MDPS12"]["CF_Mdps_ToiUnavail"] != 0

    self.Mdps_ToiUnavail = cp.vl["MDPS12"]["CF_Mdps_ToiUnavail"]
    self.driverOverride = cp.vl["TCS13"]["DriverOverride"]
    if self.driverOverride:
      self.driverAcc_time = 100
    elif self.driverAcc_time:
      self.driverAcc_time -= 1

    self.clu_Vanz = cp.vl["CLU11"]["CF_Clu_Vanz"]
    self.is_set_speed_in_mph = bool(cp.vl["CLU11"]["CF_Clu_SPEED_UNIT"])
    ret.isMph = self.is_set_speed_in_mph

    self.cruise_main_button = cp.vl["CLU11"]["CF_Clu_CruiseSwMain"]
    self.prev_cruise_buttons = self.cruise_buttons[-1]
    self.cruise_buttons[-1] = cp.vl["CLU11"]["CF_Clu_CruiseSwState"]
    ret.cruiseButtons = self.cruise_buttons[-1]

    if self.prev_gap_button != self.cruise_buttons[-1]:
      if self.cruise_buttons == 3:
        self.cruise_gap -= 1
      if self.cruise_gap < 1:
        self.cruise_gap = 4
      self.prev_gap_button = self.cruise_buttons[-1]

    # TODO: Find brake pressure
    ret.brake = 0
    ret.brakePressed = cp.vl["TCS13"]["DriverBraking"] != 0

    if self.CP.autoHoldAvailable:
      ret.brakeHoldActive = cp.vl["TCS15"]["AVH_LAMP"] == 2  # 0 OFF, 1 ERROR, 2 ACTIVE, 3 READY
      ret.autoHold = ret.brakeHoldActive

    ret.parkingBrake = cp.vl["TCS13"]["PBRAKE_ACT"] == 1

    if ret.brakePressed:
      self.brake_check = True
    if self.cruise_buttons[-1] == 4:
      self.cancel_check = True
    ret.brakeLights = bool(cp.vl["TCS13"]["BrakeLight"] or ret.brakePressed)

    # cruise state
    if (self.CP.openpilotLongitudinalControl and (self.CP.sccBus <= 0 and self.long_alt not in (1, 2))) or self.exp_long_alt:
      # These are not used for engage/disengage since openpilot keeps track of state using the buttons
      #ret.cruiseState.available = cp.vl["TCS13"]["ACCEnable"] == 0 or cp.vl["EMS16"]["CRUISE_LAMP_M"] != 0
      #ret.cruiseState.enabled = cp.vl["TCS13"]["ACC_REQ"] == 1 or cp.vl["LVR12"]["CF_Lvr_CruiseSet"] != 0
      ret.cruiseState.standstill = False
      if ret.brakePressed and self.acc_active:
        self.brake_check = True
        self.acc_active = False
      set_speed = self.cruise_speed_button_long()
      if self.cruise_buttons[-1] == 1 or self.cruise_buttons[-1] == 2:
        self.brake_check = False
        self.exp_engage_available = True
        self.acc_active = self.exp_engage_available
      elif self.cruise_buttons[-1] == 4:
        self.exp_engage_available = False
        self.acc_active = False
      speed_conv = CV.MPH_TO_MS if self.is_set_speed_in_mph else CV.KPH_TO_MS
      ret.cruiseState.speed = set_speed * speed_conv if self.acc_active else 0
      ret.cruiseState.speedCluster = set_speed * speed_conv if self.acc_active else 0
      ret.cruiseState.available = self.exp_engage_available
      ret.cruiseState.enabled = ret.cruiseState.available
      ret.cruiseAccStatus = self.acc_active
      ret.cruiseGapSet = self.cruise_gap
    else:
      ret.cruiseState.available = cp_scc.vl["SCC11"]["MainMode_ACC"] != 0
      ret.cruiseState.enabled = cp_scc.vl["SCC12"]["ACCMode"] != 0
      ret.cruiseState.standstill = cp_scc.vl["SCC11"]["SCCInfoDisplay"] == 4.
      ret.cruiseState.nonAdaptive = cp_cruise.vl["SCC11"]["SCCInfoDisplay"] == 2.  # Shows 'Cruise Control' on dash

      self.acc_active = cp_scc.vl["SCC12"]['ACCMode'] != 0
      if self.acc_active:
        self.brake_check = False
        self.cancel_check = False
      elif not ret.cruiseState.available:
        self.prev_acc_set_btn = False
        self.prev_main_btn = False
      self.cruiseState_standstill = ret.cruiseState.standstill

      set_speed = self.cruise_speed_button()
      if ret.cruiseState.enabled and (self.brake_check == False or self.cancel_check == False):
        speed_conv = CV.MPH_TO_MS if self.is_set_speed_in_mph else CV.KPH_TO_MS
        ret.cruiseState.speed = set_speed * speed_conv if not self.exp_long else \
                                          cp.vl["LVR12"]["CF_Lvr_CruiseSet"] * speed_conv
      else:
        ret.cruiseState.speed = 0
      self.cruise_active = self.acc_active

      ret.cruiseState.gapSet = cp_scc.vl["SCC11"]['TauGapSet']
      self.cruiseGapSet = ret.cruiseState.gapSet
      ret.cruiseGapSet = self.cruiseGapSet

      self.VSetDis = cp_scc.vl["SCC11"]["VSetDis"]
      ret.vSetDis = self.VSetDis
      lead_objspd = cp_scc.vl["SCC11"]["ACC_ObjRelSpd"]
      ret.radarVRel = lead_objspd
      self.lead_objspd = lead_objspd * CV.MS_TO_KPH

      ret.accFaulted = cp.vl["TCS13"]["ACCEnable"] != 0  # 0 ACC CONTROL ENABLED, 1-3 ACC CONTROL DISABLED
      ret.espDisabled = cp.vl["TCS11"]["TCS_PAS"] == 1
      ret.espActive = cp.vl["TCS11"]["ABS_ACT"] == 1

    ret.cruiseState.accActive = self.acc_active
    ret.cruiseState.cruiseSwState = self.cruise_buttons[-1]
    ret.cruiseState.modeSel = self.cruise_set_mode

    if self.CP.flags & (HyundaiFlags.HYBRID | HyundaiFlags.EV):
      if self.CP.flags & HyundaiFlags.HYBRID:
        ret.gas = cp.vl["E_EMS11"]["CR_Vcu_AccPedDep_Pos"] / 254.
        ret.engineRpm = cp.vl["E_EMS11"]["N"] # kisa
        ret.chargeMeter = 0
      else:
        ret.gas = cp.vl["E_EMS11"]["Accel_Pedal_Pos"] / 254.
        ret.engineRpm = cp.vl["ELECT_GEAR"]["Elect_Motor_Speed"] * 30 # kisa, may multiply deceleration ratio in line with engine rpm
        ret.chargeMeter = cp.vl["EV_Info"]["EV_Charge_Level"] # kisa
      ret.gasPressed = ret.gas > 0
    else:
      ret.gas = cp.vl["EMS12"]["PV_AV_CAN"] / 100.
      ret.gasPressed = bool(cp.vl["EMS16"]["CF_Ems_AclAct"])
      if self.CP.emsAvailable:
        ret.engineRpm = cp.vl["EMS_366"]["N"]
      else:
        ret.engineRpm = 0
      ret.chargeMeter = 0

    ret.espDisabled = (cp.vl["TCS15"]["ESC_Off_Step"] != 0)

    self.parkBrake = cp.vl["TCS13"]["PBRAKE_ACT"] == 1
    self.gasPressed = ret.gasPressed

    # kisa
    ret.tpms = self.get_tpms(
      cp.vl["TPMS11"]["UNIT"],
      cp.vl["TPMS11"]["PRESSURE_FL"],
      cp.vl["TPMS11"]["PRESSURE_FR"],
      cp.vl["TPMS11"]["PRESSURE_RL"],
      cp.vl["TPMS11"]["PRESSURE_RR"],
    )

    # Gear Selection via Cluster - For those Kia/Hyundai which are not fully discovered, we can use the Cluster Indicator for Gear Selection,
    # as this seems to be standard over all cars, but is not the preferred method.
    if self.CP.flags & (HyundaiFlags.HYBRID | HyundaiFlags.EV):
      gear = cp.vl["ELECT_GEAR"]["Elect_Gear_Shifter"]
      if self.CP.carFingerprint == CAR.HYUNDAI_NEXO_FE:
        gear = cp.vl["EMS20"]["Elect_Gear_Shifter_NEXO"] # kisa, NEXO gear by multikyd
      else:
        gear = cp.vl["ELECT_GEAR"]["Elect_Gear_Shifter"]
      ret.gearStep = cp.vl["ELECT_GEAR"]["Elect_Gear_Step"] # kisa
    elif self.CP.carFingerprint in CAN_GEARS["use_cluster_gears"]:
      gear = cp.vl["CLU15"]["CF_Clu_Gear"]
      ret.gearStep = 0
    elif self.CP.carFingerprint in CAN_GEARS["use_tcu_gears"]:
      gear = cp.vl["TCU12"]["CUR_GR"]
      ret.gearStep = 0
    else:
      gear = cp.vl["LVR12"]["CF_Lvr_Gear"]
      ret.gearStep = cp.vl["LVR11"]["CF_Lvr_GearInf"] # kisa

    if self.gear_correction:
      ret.gearShifter = GearShifter.drive
    else:
      ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(gear))

    if not self.CP.openpilotLongitudinalControl or self.CP.sccBus == 2:
      aeb_src = "FCA11" if self.CP.flags & HyundaiFlags.USE_FCA.value else "SCC12"
      aeb_sig = "FCA_CmdAct" if self.CP.flags & HyundaiFlags.USE_FCA.value else "AEB_CmdAct"
      aeb_warning = cp_cruise.vl[aeb_src]["CF_VSM_Warn"] != 0
      scc_warning = cp_cruise.vl["SCC12"]["TakeOverReq"] == 1  # sometimes only SCC system shows an FCW
      aeb_braking = cp_cruise.vl[aeb_src]["CF_VSM_DecCmdAct"] != 0 or cp_cruise.vl[aeb_src][aeb_sig] != 0
      ret.stockFcw = (aeb_warning or scc_warning) and not aeb_braking
      ret.stockAeb = aeb_warning and aeb_braking

    if self.CP.enableBsm:
      ret.leftBlindspot = cp.vl["LCA11"]["CF_Lca_IndLeft"] != 0
      ret.rightBlindspot = cp.vl["LCA11"]["CF_Lca_IndRight"] != 0

    # save the entire LKAS11 and CLU11
    self.lkas11 = copy.copy(cp_cam.vl["LKAS11"])
    self.clu11 = copy.copy(cp.vl["CLU11"])
    self.steer_state = cp.vl["MDPS12"]["CF_Mdps_ToiActive"]  # 0 NOT ACTIVE, 1 ACTIVE
    self.prev_cruise_buttons = self.cruise_buttons[-1]
    self.cruise_buttons.extend(cp.vl_all["CLU11"]["CF_Clu_CruiseSwState"])
    self.main_buttons.extend(cp.vl_all["CLU11"]["CF_Clu_CruiseSwMain"])

    self.brake_error = cp.vl["TCS13"]["ACCEnable"] == 3 # 0 ACC CONTROL ENABLED, 1-3 ACC CONTROL DISABLED
    self.lkas_error = cp_cam.vl["LKAS11"]["CF_Lkas_LdwsSysState"] == 7
    if not self.lkas_error:
      self.lkas_button_on = cp_cam.vl["LKAS11"]["CF_Lkas_LdwsSysState"]

    if not self.exp_long:
      ret.cruiseAccStatus = cp_scc.vl["SCC12"]["ACCMode"] == 1
      ret.driverAcc = bool(self.driverOverride)
      ret.aReqValue = cp_scc.vl["SCC12"]["aReqValue"]
      self.highway_cam = cp_scc.vl["SCC11"]["Navi_SCC_Camera_Act"]
      self.lead_distance = cp_scc.vl["SCC11"]["ACC_ObjDist"]
      ret.radarDRel = self.lead_distance
      self.scc11 = copy.copy(cp_scc.vl["SCC11"])
      self.scc12 = copy.copy(cp_scc.vl["SCC12"])
      if self.CP.scc13Available:
        self.scc13 = copy.copy(cp_scc.vl["SCC13"])
      if self.CP.scc14Available:
        self.scc14 = copy.copy(cp_scc.vl["SCC14"])

    self.mdps12 = copy.copy(cp.vl["MDPS12"])

    return ret

  def update_canfd(self, cp, cp_cam):
    ret = car.CarState.new_message()

    self.is_metric = cp.vl["CRUISE_BUTTONS_ALT"]["DISTANCE_UNIT"] != 1
    self.is_set_speed_in_mph = not self.is_metric
    speed_factor = CV.KPH_TO_MS if self.is_metric else CV.MPH_TO_MS

    self.clu_Vanz = cp.vl["CRUISE_BUTTONS_ALT"]["CLUSTER_SPEED"]

    if self.CP.flags & (HyundaiFlags.EV | HyundaiFlags.HYBRID):
      offset = 255. if self.CP.flags & HyundaiFlags.EV else 1023.
      ret.gas = cp.vl[self.accelerator_msg_canfd]["ACCELERATOR_PEDAL"] / offset
      ret.gasPressed = ret.gas > 1e-5
    else:
      ret.gasPressed = bool(cp.vl[self.accelerator_msg_canfd]["ACCELERATOR_PEDAL_PRESSED"])

    ret.brakePressed = cp.vl["TCS"]["DriverBraking"] == 1
    ret.brakeLights = bool(cp.vl["BRAKE"]["BRAKE_LIGHT"])

    ret.doorOpen = cp.vl["DOORS_SEATBELTS"]["DRIVER_DOOR"] == 1
    ret.seatbeltUnlatched = cp.vl["DOORS_SEATBELTS"]["DRIVER_SEATBELT"] == 0

    gear = cp.vl[self.gear_msg_canfd]["GEAR"]
    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(gear))

    # kisa
    ret.tpms = self.get_tpms(
      cp.vl["TPMS"]["UNIT"],
      cp.vl["TPMS"]["PRESSURE_FL"],
      cp.vl["TPMS"]["PRESSURE_FR"],
      cp.vl["TPMS"]["PRESSURE_RL"],
      cp.vl["TPMS"]["PRESSURE_RR"],
    )

    # TODO: figure out positions
    ret.wheelSpeeds = self.get_wheel_speeds(
      cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_1"],
      cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_2"],
      cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_3"],
      cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_4"],
    )
    ret.vEgoRaw = (ret.wheelSpeeds.fl + ret.wheelSpeeds.fr + ret.wheelSpeeds.rl + ret.wheelSpeeds.rr) / 4.
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)

    ret.vEgoOP = ret.vEgo
    ret.vEgo = cp.vl["CRUISE_BUTTONS_ALT"]["CLUSTER_SPEED"] * CV.KPH_TO_MS if self.is_metric else cp.vl["CRUISE_BUTTONS_ALT"]["CLUSTER_SPEED"] * CV.MPH_TO_MS

    ret.standstill = ret.wheelSpeeds.fl <= STANDSTILL_THRESHOLD and ret.wheelSpeeds.rr <= STANDSTILL_THRESHOLD

    ret.standStill = self.CP.standStill

    ret.steeringRateDeg = cp.vl["STEERING_SENSORS"]["STEERING_RATE"]
    ret.steeringAngleDeg = cp.vl["STEERING_SENSORS"]["STEERING_ANGLE"] * -1
    ret.steeringTorque = cp.vl["MDPS"]["STEERING_COL_TORQUE"]
    ret.steeringTorqueEps = cp.vl["MDPS"]["STEERING_OUT_TORQUE"]
    ret.steeringPressed = self.update_steering_pressed(abs(ret.steeringTorque) > self.params.STEER_THRESHOLD, 5)
    ret.steerFaultTemporary = cp.vl["MDPS"]["LKA_FAULT"] != 0

    # TODO: alt signal usage may be described by cp.vl['BLINKERS']['USE_ALT_LAMP']
    left_blinker_sig, right_blinker_sig = "LEFT_LAMP", "RIGHT_LAMP"
    if self.CP.carFingerprint in (CAR.HYUNDAI_KONA_EV_2ND_GEN, CAR.HYUNDAI_IONIQ_5_PE, CAR.KIA_EV9):
      left_blinker_sig, right_blinker_sig = "LEFT_LAMP_ALT", "RIGHT_LAMP_ALT"
    ret.leftBlinker, ret.rightBlinker = self.update_blinker_from_lamp(50, cp.vl["BLINKERS"][left_blinker_sig],
                                                                      cp.vl["BLINKERS"][right_blinker_sig])
    if self.CP.enableBsm:
      if self.CP.carFingerprint in ANGLE_CONTROL_CAR:
        ret.leftBlindspot = cp.vl["BLINDSPOTS_REAR_CORNERS"]["INDICATOR_LEFT_FOUR"] != 0
        ret.rightBlindspot = cp.vl["BLINDSPOTS_REAR_CORNERS"]["INDICATOR_RIGHT_FOUR"] != 0
      else:
        ret.leftBlindspot = cp.vl["BLINDSPOTS_REAR_CORNERS"]["FL_INDICATOR"] != 0
        ret.rightBlindspot = cp.vl["BLINDSPOTS_REAR_CORNERS"]["FR_INDICATOR"] != 0

    if self.CP.openpilotLongitudinalControl:
      # These are not used for engage/disengage since openpilot keeps track of state using the buttons
      # ret.cruiseState.enabled = cp.vl["TCS"]["ACC_REQ"] == 1
      ret.cruiseState.standstill = False
      if ret.brakePressed and self.acc_active and not ret.standstill:
        self.brake_check = True
        self.acc_active = False
      set_speed = self.cruise_speed_button_long()
      if self.cruise_buttons[-1] == 1 or self.cruise_buttons[-1] == 2:
        self.brake_check = False
        self.exp_engage_available = True
        self.acc_active = self.exp_engage_available
      elif self.cruise_buttons[-1] == 4:
        self.exp_engage_available = False
        self.acc_active = False
      speed_conv = CV.MPH_TO_MS if self.is_set_speed_in_mph else CV.KPH_TO_MS
      ret.cruiseState.speed = set_speed * speed_conv if self.acc_active else 0
      ret.cruiseState.speedCluster = set_speed * speed_conv if self.acc_active else 0
      ret.cruiseState.available = self.exp_engage_available
      ret.cruiseState.enabled = ret.cruiseState.available
      ret.cruiseAccStatus = self.acc_active
      ret.cruiseGapSet = self.cruise_gap
    else:
      cp_cruise_info = cp_cam if self.CP.flags & HyundaiFlags.CANFD_CAMERA_SCC else cp
      # cruise state
      # CAN FD cars enable on main button press, set available if no TCS faults preventing engagement
      ret.cruiseState.available = cp_cruise_info.vl["SCC_CONTROL"]["MainMode_ACC"] != 0
      ret.cruiseState.enabled = cp_cruise_info.vl["SCC_CONTROL"]["ACCMode"] in (1, 2)
      ret.cruiseState.standstill = cp_cruise_info.vl["SCC_CONTROL"]["CRUISE_STANDSTILL"] == 1
      ret.cruiseState.speed = cp_cruise_info.vl["SCC_CONTROL"]["VSetDis"] * speed_factor
      self.VSetDis = cp_cruise_info.vl["SCC_CONTROL"]["VSetDis"]
      ret.vSetDis = self.VSetDis
      self.cruiseState_standstill = ret.cruiseState.standstill
      self.cruise_info = copy.copy(cp_cruise_info.vl["SCC_CONTROL"])

      self.acc_active = cp_cruise_info.vl["SCC_CONTROL"]["ACCMode"] in (1, 2)
      ret.cruiseState.accActive = self.acc_active
      if self.acc_active:
        self.brake_check = False
        self.cancel_check = False
      elif not ret.cruiseState.available:
        self.prev_acc_set_btn = False
        self.prev_main_btn = False
      elif not self.acc_active:
        self.prev_acc_reset_btn = False

      set_speed = self.cruise_speed_button_alt()
      if ret.cruiseState.enabled and (self.brake_check == False or self.cancel_check == False):
        speed_conv = CV.MPH_TO_MS if self.is_set_speed_in_mph else CV.KPH_TO_MS
        ret.cruiseState.speed = set_speed * speed_factor
      else:
        ret.cruiseState.speed = 0
      self.cruise_active = self.acc_active
      ret.cruiseAccStatus = self.acc_active

      ret.brakeHoldActive = cp.vl["ESP_STATUS"]["AUTO_HOLD"] == 1 and cp_cruise_info.vl["SCC_CONTROL"]["ACCMode"] not in (1, 2)
      ret.autoHold = ret.brakeHoldActive

      if self.CP.adrvAvailable:
        ret.cruiseState.gapSet = cp.vl["ADRV_0x200"]["TauGapSet"]
      self.cruiseGapSet = ret.cruiseState.gapSet
      ret.cruiseGapSet = self.cruiseGapSet
      self.DistSet = cp_cruise_info.vl["SCC_CONTROL"]["DISTANCE_SETTING"] - 5 if cp_cruise_info.vl["SCC_CONTROL"]["DISTANCE_SETTING"] > 5 else cp_cruise_info.vl["SCC_CONTROL"]["DISTANCE_SETTING"]
      ret.cruiseState.modeSel = self.cruise_set_mode

    if not self.exp_long:
      self.lead_distance = cp_cruise_info.vl["SCC_CONTROL"]["ACC_ObjDist"]
      ret.radarDRel = self.lead_distance
      ret.aReqValue = cp_cruise_info.vl["SCC_CONTROL"]["aReqValue"]
      lead_objspd = cp_cruise_info.vl["SCC_CONTROL"]["ACC_ObjRelSpd"]
      ret.radarVRel = lead_objspd
      self.lead_objspd = lead_objspd * CV.MS_TO_KPH
      self.obj_valid = cp_cruise_info.vl["SCC_CONTROL"]["ObjValid"]
      self.scc_control = copy.copy(cp_cruise_info.vl["SCC_CONTROL"])
      self.driverOverride = cp.vl["TCS"]["DriverOverride"]
      if self.driverOverride:
        self.driverAcc_time = 100
      elif self.driverAcc_time:
        self.driverAcc_time -= 1
      ret.driverAcc = bool(self.driverOverride)

    # Manual Speed Limit Assist is a feature that replaces non-adaptive cruise control on EV CAN FD platforms.
    # It limits the vehicle speed, overridable by pressing the accelerator past a certain point.
    # The car will brake, but does not respect positive acceleration commands in this mode
    # TODO: find this message on ICE & HYBRID cars + cruise control signals (if exists)
    if self.CP.flags & HyundaiFlags.EV:
      ret.cruiseState.nonAdaptive = cp.vl["MANUAL_SPEED_LIMIT_ASSIST"]["MSLA_ENABLED"] == 1

    self.prev_cruise_buttons = self.cruise_buttons[-1]
    self.cruise_buttons.extend(cp.vl_all[self.cruise_btns_msg_canfd]["CRUISE_BUTTONS"])
    self.main_buttons.extend(cp.vl_all[self.cruise_btns_msg_canfd]["ADAPTIVE_CRUISE_MAIN_BTN"])
    self.buttons_counter = cp.vl[self.cruise_btns_msg_canfd]["COUNTER"]
    ret.accFaulted = cp.vl["TCS"]["ACCEnable"] != 0  # 0 ACC CONTROL ENABLED, 1-3 ACC CONTROL DISABLED
    ret.cruiseButtons = self.cruise_buttons[-1]

    if self.cruise_btns_msg_canfd == "CRUISE_BUTTONS":
      self.cruise_btn_info = copy.copy(cp_cruise_info.vl[self.cruise_btns_msg_canfd])

    if self.CP.flags & HyundaiFlags.CANFD_HDA2:
      self.hda2_lfa_block_msg = copy.copy(cp_cam.vl["CAM_0x362"] if self.CP.flags & HyundaiFlags.CANFD_HDA2_ALT_STEERING
                                          else cp_cam.vl["CAM_0x2a4"])

    return ret

  def get_can_parser(self, CP):
    if CP.carFingerprint in CANFD_CAR:
      return self.get_can_parser_canfd(CP)

    messages = [
      # address, frequency
      ("MDPS12", 50),
      ("TCS11", 100),
      ("TCS13", 50),
      ("TCS15", 10),
      ("CLU11", 50),
      ("CLU15", 5),
      ("ESP12", 100),
      ("CGW1", 10),
      ("CGW2", 5),
      ("CGW4", 5),
      ("WHL_SPD11", 50),
      ("SAS11", 100),
      ("TPMS11", 0),
    ]

    if CP.sccBus == 0 and CP.pcmCruise:
      messages += [
        ("SCC11", 50),
        ("SCC12", 50),
      ]
      if CP.flags & HyundaiFlags.USE_FCA.value:
        messages.append(("FCA11", 50))

    if CP.enableBsm:
      messages.append(("LCA11", 50))

    if CP.flags & (HyundaiFlags.HYBRID | HyundaiFlags.EV):
      messages.append(("E_EMS11", 50))
      if CP.flags & HyundaiFlags.EV:
        messages.append(("EV_Info", 0))
    else:
      messages += [
        ("EMS12", 100),
        ("EMS16", 100),
      ]
      if CP.emsAvailable:
        messages += [
          ("EMS_366", 100),
        ]

    if CP.flags & (HyundaiFlags.HYBRID | HyundaiFlags.EV):
      messages.append(("ELECT_GEAR", 20))
      if CP.carFingerprint == CAR.HYUNDAI_NEXO_FE:
        messages.append(("EMS20", 20))
    elif CP.carFingerprint in CAN_GEARS["use_cluster_gears"]:
      pass
    elif CP.carFingerprint in CAN_GEARS["use_tcu_gears"]:
      messages.append(("TCU12", 100))
    else:
      messages += [
        ("LVR11", 100),
        ("LVR12", 100),
      ]

    return CANParser(DBC[CP.carFingerprint]["pt"], messages, 0)

  @staticmethod
  def get_cam_can_parser(CP):
    if CP.carFingerprint in CANFD_CAR:
      return CarState.get_cam_can_parser_canfd(CP)

    messages = [
      ("LKAS11", 100)
    ]

    if CP.openpilotLongitudinalControl and CP.sccBus == 2:
      messages += [
        ("SCC11", 50),
        ("SCC12", 50),
      ]

      if CP.scc13Available:
        messages += [
          ("SCC13", 50),
        ]

      if CP.scc14Available:
        messages += [
          ("SCC14", 50),
        ]

      if CP.flags & HyundaiFlags.USE_FCA.value:
        messages.append(("FCA11", 50))

    return CANParser(DBC[CP.carFingerprint]["pt"], messages, 2)

  def get_can_parser_canfd(self, CP):
    messages = [
      (self.gear_msg_canfd, 100),
      (self.accelerator_msg_canfd, 100),
      ("WHEEL_SPEEDS", 100),
      ("STEERING_SENSORS", 100),
      ("MDPS", 100),
      ("BRAKE", 100),
      ("ESP_STATUS", 100),
      ("TCS", 50),
      ("CRUISE_BUTTONS_ALT", 50),
      ("TPMS", 5),
      ("BLINKERS", 4),
      ("DOORS_SEATBELTS", 4),
    ]

    if CP.flags & HyundaiFlags.EV:
      messages += [
        ("MANUAL_SPEED_LIMIT_ASSIST", 10),
      ]

    if not (CP.flags & HyundaiFlags.CANFD_ALT_BUTTONS):
      messages += [
        ("CRUISE_BUTTONS", 50)
      ]

    if CP.enableBsm:
      messages += [
        ("BLINDSPOTS_REAR_CORNERS", 20),
      ]

    if not (CP.flags & HyundaiFlags.CANFD_CAMERA_SCC.value) and not CP.openpilotLongitudinalControl:
      messages += [
        ("SCC_CONTROL", 50),
      ]

    if CP.adrvAvailable:
      messages += [
        ("ADRV_0x200", 20),
      ]

    return CANParser(DBC[CP.carFingerprint]["pt"], messages, CanBus(CP).ECAN)

  @staticmethod
  def get_cam_can_parser_canfd(CP):
    messages = []
    if CP.flags & HyundaiFlags.CANFD_HDA2:
      block_lfa_msg = "CAM_0x362" if CP.flags & HyundaiFlags.CANFD_HDA2_ALT_STEERING else "CAM_0x2a4"
      messages += [(block_lfa_msg, 20)]
    elif CP.flags & HyundaiFlags.CANFD_CAMERA_SCC:
      messages += [
        ("SCC_CONTROL", 50),
      ]

    return CANParser(DBC[CP.carFingerprint]["pt"], messages, CanBus(CP).CAM)
