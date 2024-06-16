from cereal import car, messaging
from openpilot.common.conversions import Conversions as CV
from openpilot.common.numpy_fast import clip, interp
from openpilot.common.realtime import DT_CTRL
from opendbc.can.packer import CANPacker
from openpilot.selfdrive.car import apply_driver_steer_torque_limits, common_fault_avoidance, apply_std_steer_angle_limits
from openpilot.selfdrive.car.hyundai import hyundaicanfd, hyundaican
from openpilot.selfdrive.car.hyundai.hyundaicanfd import CanBus
from openpilot.selfdrive.car.hyundai.values import HyundaiFlags, Buttons, CarControllerParams, CANFD_CAR, CAR, LEGACY_SAFETY_MODE_CAR_ALT, ANGLE_CONTROL_CAR
from openpilot.selfdrive.car.interfaces import CarControllerBase

from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
from openpilot.selfdrive.car.hyundai.carstate import GearShifter
from openpilot.selfdrive.controls.lib.desire_helper import LANE_CHANGE_SPEED_MIN
from openpilot.selfdrive.car.hyundai.kisa_cruise_control  import KisaCruiseControl

from openpilot.common.params import Params
from random import randint
from decimal import Decimal

VisualAlert = car.CarControl.HUDControl.VisualAlert
LongCtrlState = car.CarControl.Actuators.LongControlState

# EPS faults if you apply torque while the steering angle is above 90 degrees for more than 1 second
# All slightly below EPS thresholds to avoid fault
MAX_ANGLE = int(Params().get("AvoidLKASFaultMaxAngle", encoding="utf8")) # 85
MAX_ANGLE_FRAMES = int(Params().get("AvoidLKASFaultMaxFrame", encoding="utf8")) # 89
MAX_ANGLE_CONSECUTIVE_FRAMES = 2


def process_hud_alert(enabled, fingerprint, hud_control):
  sys_warning = (hud_control.visualAlert in (VisualAlert.steerRequired, VisualAlert.ldw))

  # initialize to no line visible
  # TODO: this is not accurate for all cars
  sys_state = 1
  if hud_control.leftLaneVisible and hud_control.rightLaneVisible or sys_warning:  # HUD alert only display when LKAS status is active
    sys_state = 3 if enabled or sys_warning else 4
  elif hud_control.leftLaneVisible:
    sys_state = 5
  elif hud_control.rightLaneVisible:
    sys_state = 6

  # initialize to no warnings
  left_lane_warning = 0
  right_lane_warning = 0
  if hud_control.leftLaneDepart:
    left_lane_warning = 1 if fingerprint in (CAR.GENESIS_DH, CAR.GENESIS_G90, CAR.GENESIS_G80) else 2
  if hud_control.rightLaneDepart:
    right_lane_warning = 1 if fingerprint in (CAR.GENESIS_DH, CAR.GENESIS_G90, CAR.GENESIS_G80) else 2

  return sys_warning, sys_state, left_lane_warning, right_lane_warning


class CarController(CarControllerBase):
  def __init__(self, dbc_name, CP, VM):
    self.CP = CP
    self.CAN = CanBus(CP)
    self.params = CarControllerParams(CP)
    self.packer = CANPacker(dbc_name)
    self.angle_limit_counter = 0
    self.frame = 0

    self.accel_last = 0
    self.apply_steer_last = 0
    self.car_fingerprint = CP.carFingerprint
    self.last_button_frame = 0

    self.apply_angle_last = 0
    self.lkas_max_torque = 0

    self.scc12_cnt = 0
    self.aq_value = 0
    self.aq_value_raw = 0

    self.resume_cnt = 0
    self.last_lead_distance = 0
    self.resume_wait_timer = 0

    self.last_resume_frame = 0
    self.accel = 0

    self.lanechange_manual_timer = 0
    self.emergency_manual_timer = 0
    self.driver_steering_torque_above_timer = 150
    self.driver_steering_angle_above_timer = 150
    
    self.mode_change_timer = 0

    self.acc_standstill_timer = 0
    self.acc_standstill = False

    self.need_brake = False
    self.need_brake_timer = 0

    self.cancel_counter = 0

    self.v_cruise_kph_auto_res = 0

    self.c_params = Params()
    self.mode_change_switch = int(self.c_params.get("CruiseStatemodeSelInit", encoding="utf8"))
    self.kisa_variablecruise = self.c_params.get_bool("KisaVariableCruise")
    self.kisa_autoresume = self.c_params.get_bool("KisaAutoResume")
    self.kisa_cruisegap_auto_adj = self.c_params.get_bool("CruiseGapAdjust")
    self.kisa_cruise_auto_res = self.c_params.get_bool("CruiseAutoRes")
    self.kisa_cruise_auto_res_option = int(self.c_params.get("AutoResOption", encoding="utf8"))
    self.kisa_cruise_auto_res_condition = int(self.c_params.get("AutoResCondition", encoding="utf8"))

    self.kisa_turnsteeringdisable = self.c_params.get_bool("KisaTurnSteeringDisable")
    self.kisa_maxanglelimit = float(int(self.c_params.get("KisaMaxAngleLimit", encoding="utf8")))
    self.ufc_mode_enabled = self.c_params.get_bool("UFCModeEnabled")
    self.ldws_fix = self.c_params.get_bool("LdwsCarFix")
    self.radar_helper_option = int(self.c_params.get("RadarLongHelper", encoding="utf8"))
    self.stopping_dist_adj_enabled = self.c_params.get_bool("StoppingDistAdj")
    self.standstill_resume_alt = self.c_params.get_bool("StandstillResumeAlt")
    self.auto_res_delay = int(self.c_params.get("AutoRESDelay", encoding="utf8")) * 100
    self.auto_res_delay_timer = 0
    self.stopped = False
    self.stoppingdist = float(Decimal(self.c_params.get("StoppingDist", encoding="utf8"))*Decimal('0.1'))

    self.longcontrol = self.CP.openpilotLongitudinalControl
    #self.scc_live is true because CP.radarUnavailable is False
    self.scc_live = not self.CP.radarUnavailable

    self.KCC = KisaCruiseControl()

    self.dRel = 0
    self.vRel = 0
    self.yRel = 0

    self.cruise_gap_prev = 0
    self.cruise_gap_set_init = False
    self.cruise_gap_adjusting = False
    self.cruise_speed_adjusting = False
    self.standstill_fault_reduce_timer = 0
    self.standstill_res_button = False
    self.standstill_res_count = int(self.c_params.get("RESCountatStandstill", encoding="utf8"))

    self.standstill_status = 0
    self.standstill_status_canfd = False
    self.standstill_status_timer = 0
    self.switch_timer = 0
    self.switch_timer2 = 0
    self.auto_res_timer = 0
    self.auto_res_limit_timer = 0
    self.auto_res_limit_sec = int(self.c_params.get("AutoResLimitTime", encoding="utf8")) * 100
    self.auto_res_starting = False
    self.res_speed = 0
    self.res_speed_timer = 0
    self.autohold_popup_timer = 0
    self.autohold_popup_switch = False

    self.steerMax_base = int(self.c_params.get("SteerMaxBaseAdj", encoding="utf8"))
    self.steerDeltaUp_base = int(self.c_params.get("SteerDeltaUpBaseAdj", encoding="utf8"))
    self.steerDeltaDown_base = int(self.c_params.get("SteerDeltaDownBaseAdj", encoding="utf8"))
    self.steerMax_Max = int(self.c_params.get("SteerMaxAdj", encoding="utf8"))
    self.steerDeltaUp_Max = int(self.c_params.get("SteerDeltaUpAdj", encoding="utf8"))
    self.steerDeltaDown_Max = int(self.c_params.get("SteerDeltaDownAdj", encoding="utf8"))
    self.model_speed = 255.0
    self.steerMax_range = [self.steerMax_Max, self.steerMax_base, self.steerMax_base]
    self.steerDeltaUp_range = [self.steerDeltaUp_Max, self.steerDeltaUp_base, self.steerDeltaUp_base]
    self.steerDeltaDown_range = [self.steerDeltaDown_Max, self.steerDeltaDown_base, self.steerDeltaDown_base]
    self.steerMax = 0
    self.steerDeltaUp = 0
    self.steerDeltaDown = 0

    self.variable_steer_max = self.c_params.get_bool("KisaVariableSteerMax")
    self.variable_steer_delta = self.c_params.get_bool("KisaVariableSteerDelta")
    self.osm_spdlimit_enabled = self.c_params.get_bool("OSMSpeedLimitEnable")
    self.stock_safety_decel_enabled = self.c_params.get_bool("UseStockDecelOnSS")
    self.joystick_debug_mode = self.c_params.get_bool("JoystickDebugMode")
    #self.stopsign_enabled = self.c_params.get_bool("StopAtStopSign")

    self.smooth_start = False

    self.cc_timer = 0
    self.on_speed_control = False
    self.on_speed_bump_control = False
    self.curv_speed_control = False
    self.cut_in_control = False
    self.driver_scc_set_control = False
    self.vFuture = 0
    self.vFutureA = 0
    self.cruise_init = False
    self.change_accel_fast = False

    self.to_avoid_lkas_fault_enabled = self.c_params.get_bool("AvoidLKASFaultEnabled")
    self.to_avoid_lkas_fault_max_angle = int(self.c_params.get("AvoidLKASFaultMaxAngle", encoding="utf8"))
    self.to_avoid_lkas_fault_max_frame = int(self.c_params.get("AvoidLKASFaultMaxFrame", encoding="utf8"))
    self.enable_steer_more = self.c_params.get_bool("AvoidLKASFaultBeyond")
    self.no_mdps_mods = self.c_params.get_bool("NoSmartMDPS")

    self.user_specific_feature = int(self.c_params.get("UserSpecificFeature", encoding="utf8"))

    self.gap_by_spd_on = self.c_params.get_bool("CruiseGapBySpdOn")
    self.gap_by_spd_spd = list(map(int, self.c_params.get("CruiseGapBySpdSpd", encoding="utf8").split(',')))
    self.gap_by_spd_gap = list(map(int, self.c_params.get("CruiseGapBySpdGap", encoding="utf8").split(',')))
    self.gap_by_spd_on_buffer1 = 0
    self.gap_by_spd_on_buffer2 = 0
    self.gap_by_spd_on_buffer3 = 0
    self.gap_by_spd_gap1 = False
    self.gap_by_spd_gap2 = False
    self.gap_by_spd_gap3 = False
    self.gap_by_spd_gap4 = False
    self.gap_by_spd_on_sw = False
    self.gap_by_spd_on_sw_trg = True
    self.gap_by_spd_on_sw_cnt = 0
    self.gap_by_spd_on_sw_cnt2 = 0

    self.prev_cruiseButton = 0
    self.lead_visible = False
    self.lead_debounce = 0
    self.radarDisableOverlapTimer = 0
    self.objdiststat = 0
    self.fca11supcnt = self.fca11inc = self.fca11alivecnt = self.fca11cnt13 = 0
    self.fca11maxcnt = 0xD

    self.steer_timer_apply_torque = 1.0
    self.DT_STEER = 0.005             # 0.01 1sec, 0.005  2sec

    self.lkas_onoff_counter = 0
    self.lkas_temp_disabled = False
    self.lkas_temp_disabled_timer = 0

    self.try_early_stop = self.c_params.get_bool("KISAEarlyStop")
    self.try_early_stop_retrieve = False
    self.try_early_stop_org_gap = 4.0

    self.ed_rd_diff_on = False
    self.ed_rd_diff_on_timer = 0
    self.ed_rd_diff_on_timer2 = 0

    self.vrel_delta = 0
    self.vrel_delta_prev = 0
    self.vrel_delta_timer = 0
    self.vrel_delta_timer2 = 0
    self.vrel_delta_timer3 = 0

    self.e2e_standstill_enable = self.c_params.get_bool("DepartChimeAtResume")
    self.e2e_standstill = False
    self.e2e_standstill_stat = False
    self.e2e_standstill_timer = 0
    self.e2e_standstill_timer2 = 0
    self.e2e_standstill_timer_buf = 0

    self.experimental_long_enabled = self.c_params.get_bool("ExperimentalLongitudinalEnabled")
    self.experimental_mode = self.c_params.get_bool("ExperimentalMode")
    self.live_torque_params = self.c_params.get_bool("KisaLiveTorque")
    self.gapsettingdance = int(self.c_params.get("KisaCruiseGapSet", encoding="utf8"))
    self.prev_gapButton = 0

    self.kisa_long_alt = True if int(self.c_params.get("KISALongAlt", encoding="utf8")) in (1, 2) else False

    self.btnsignal = 0
    self.nt_interval = int(self.c_params.get("KISACruiseSpammingInterval", encoding="utf8"))
    self.btn_count = int(self.c_params.get("KISACruiseSpammingBtnCount", encoding="utf8"))
    self.second2 = 0
    self.experimental_mode_temp = self.experimental_mode
    self.exp_mode_push = False
    self.exp_mode_push_cnt = 0

    self.counter_init = False
    self.radarDisableOverlapTimer = 0

    self.e2e_x = 0
    self.l_stat = 0

    self.refresh_time = 1

    # self.usf = 0
    self.stock_lfa_counter = 0

    self.str_log2 = ''

    if self.car_fingerprint in ANGLE_CONTROL_CAR:
      pass
    else:
      if CP.lateralTuning.which() == 'pid':
        self.str_log2 = 'T={:0.2f}/{:0.3f}/{:0.5f}/{:0.2f}'.format(CP.lateralTuning.pid.kpV[1], CP.lateralTuning.pid.kiV[1], CP.lateralTuning.pid.kf, CP.lateralTuning.pid.kd)
      elif CP.lateralTuning.which() == 'indi':
        self.str_log2 = 'T={:03.1f}/{:03.1f}/{:03.1f}/{:03.1f}'.format(CP.lateralTuning.indi.innerLoopGainV[0], CP.lateralTuning.indi.outerLoopGainV[0], \
        CP.lateralTuning.indi.timeConstantV[0], CP.lateralTuning.indi.actuatorEffectivenessV[0])
      elif CP.lateralTuning.which() == 'lqr':
        self.str_log2 = 'T={:04.0f}/{:05.3f}/{:07.5f}'.format(CP.lateralTuning.lqr.scale, CP.lateralTuning.lqr.ki, CP.lateralTuning.lqr.dcGain)
      elif CP.lateralTuning.which() == 'torque':
        self.str_log2 = 'T={:0.2f}/{:0.2f}/{:0.2f}/{:0.3f}'.format(CP.lateralTuning.torque.kp, CP.lateralTuning.torque.kf, CP.lateralTuning.torque.ki, CP.lateralTuning.torque.friction)

    self.sm = messaging.SubMaster(['controlsState', 'radarState', 'lateralPlan', 'longitudinalPlan', 'liveTorqueParameters'])


  def smooth_steer( self, apply_torque, CS ):
    if self.CP.smoothSteer.maxSteeringAngle and abs(CS.out.steeringAngleDeg) > self.CP.smoothSteer.maxSteeringAngle:
      if self.CP.smoothSteer.maxDriverAngleWait and CS.out.steeringPressed:
        self.steer_timer_apply_torque -= self.CP.smoothSteer.maxDriverAngleWait # 0.002 #self.DT_STEER   # 0.01 1sec, 0.005  2sec   0.002  5sec
      elif self.CP.smoothSteer.maxSteerAngleWait:
        self.steer_timer_apply_torque -= self.CP.smoothSteer.maxSteerAngleWait # 0.001  # 10 sec
    elif self.CP.smoothSteer.driverAngleWait and CS.out.steeringPressed:
      self.steer_timer_apply_torque -= self.CP.smoothSteer.driverAngleWait #0.001
    else:
      if self.steer_timer_apply_torque >= 1:
          return int(round(float(apply_torque)))
      self.steer_timer_apply_torque += self.DT_STEER

    if self.steer_timer_apply_torque < 0:
      self.steer_timer_apply_torque = 0
    elif self.steer_timer_apply_torque > 1:
      self.steer_timer_apply_torque = 1

    apply_torque *= self.steer_timer_apply_torque

    return  int(round(float(apply_torque)))


  def update(self, CC, CS, now_nanos):

    self.sm.update(0)

    actuators = CC.actuators
    hud_control = CC.hudControl

    self.vFuture = hud_control.vFuture
    self.vFutureA = hud_control.vFutureA

    if self.frame % 10 == 0:
      self.model_speed = self.sm['lateralPlan'].modelSpeed

    self.dRel = self.sm['radarState'].leadOne.dRel #Vision Lead
    self.vRel = self.sm['radarState'].leadOne.vRel #Vision Lead
    self.yRel = self.sm['radarState'].leadOne.yRel #Vision Lead

    if len(self.sm['longitudinalPlan'].e2eX) > 12:
      self.e2e_x = self.sm['longitudinalPlan'].e2eX[12]

    if abs(CS.out.steeringTorque) > 170 and CS.out.vEgo < LANE_CHANGE_SPEED_MIN and self.CP.carFingerprint not in CANFD_CAR:
      self.driver_steering_torque_above_timer -= 1
      if self.driver_steering_torque_above_timer <= 0:
        self.driver_steering_torque_above_timer = 0
    else:
      self.driver_steering_torque_above_timer += 10
      if self.driver_steering_torque_above_timer >= 150:
        self.driver_steering_torque_above_timer = 150

    # steering torque
    if self.CP.smoothSteer.method == 1:
      new_steer = int(round(actuators.steer * self.params.STEER_MAX))
      new_steer = self.smooth_steer( new_steer, CS )
    elif 0 <= self.driver_steering_torque_above_timer < 150 and not self.user_specific_feature == 60:
      new_steer = int(round(actuators.steer * self.params.STEER_MAX * (self.driver_steering_torque_above_timer / 150)))
    else:
      new_steer = int(round(actuators.steer * self.params.STEER_MAX))

    # if self.CP.carFingerprint in LEGACY_SAFETY_MODE_CAR_ALT:
    #   new_steer = int(round(interp(abs(actuators.steer), [0.9, 1.0], [new_steer, min(new_steer, 255)])))

    apply_steer = apply_driver_steer_torque_limits(new_steer, self.apply_steer_last, CS.out.steeringTorque, self.params)

    if self.joystick_debug_mode:
      lat_active = CC.latActive
    # disable when temp fault is active, or below LKA minimum speed
    elif self.kisa_maxanglelimit == 90:
      lat_active = CC.latActive and abs(CS.out.steeringAngleDeg) < self.kisa_maxanglelimit and (CS.out.gearShifter == GearShifter.drive or self.user_specific_feature == 11)
    elif self.kisa_maxanglelimit > 90:
      str_angle_limit = interp(CS.out.vEgo * CV.MS_TO_KPH, [0, 20], [self.kisa_maxanglelimit+60, self.kisa_maxanglelimit])
      lat_active = CC.latActive and abs(CS.out.steeringAngleDeg) < str_angle_limit and (CS.out.gearShifter == GearShifter.drive or self.user_specific_feature == 11)
    else:
      lat_active = CC.latActive and (CS.out.gearShifter == GearShifter.drive or self.user_specific_feature == 11)

    if (( CS.out.leftBlinker and not CS.out.rightBlinker) or ( CS.out.rightBlinker and not CS.out.leftBlinker)) and CS.out.vEgo < LANE_CHANGE_SPEED_MIN and self.kisa_turnsteeringdisable:
      self.lanechange_manual_timer = 50
    if CS.out.leftBlinker and CS.out.rightBlinker:
      self.emergency_manual_timer = 50
    if self.lanechange_manual_timer:
      lat_active = False
    if self.lanechange_manual_timer > 0:
      self.lanechange_manual_timer -= 1
    if self.emergency_manual_timer > 0:
      self.emergency_manual_timer -= 1

    if self.no_mdps_mods and CS.out.vEgo < self.CP.minSteerSpeed:
      lat_active = False

    # >90 degree steering fault prevention
    if self.to_avoid_lkas_fault_enabled or self.CP.carFingerprint in CANFD_CAR:
      self.angle_limit_counter, apply_steer_req = common_fault_avoidance(abs(CS.out.steeringAngleDeg) >= self.to_avoid_lkas_fault_max_angle, lat_active,
                                                                         self.angle_limit_counter, self.to_avoid_lkas_fault_max_frame,
                                                                         MAX_ANGLE_CONSECUTIVE_FRAMES)
      apply_angle = apply_std_steer_angle_limits(actuators.steeringAngleDeg, self.apply_angle_last, CS.out.vEgoRaw, self.params)

      # Figure out torque value.  On Stock when LKAS is active, this is variable,
      # but 0 when LKAS is not actively steering, so because we're "tricking" ADAS
      # into thinking LKAS is always active, we need to make sure we're applying
      # torque when the driver is not actively steering. The default value chosen
      # here is based on observations of the stock LKAS system when it's engaged
      # CS.out.steeringPressed and steeringTorque are based on the
      # STEERING_COL_TORQUE value

      lkas_max_torque = 180
      if abs(CS.out.steeringTorque) > 200:
        self.driver_steering_angle_above_timer -= 1
        if self.driver_steering_angle_above_timer <= 30:
          self.driver_steering_angle_above_timer = 30
      else:
        self.driver_steering_angle_above_timer += 1
        if self.driver_steering_angle_above_timer >= 150:
          self.driver_steering_angle_above_timer = 150

      ego_weight = interp(CS.out.vEgo, [0, 5, 10, 20], [0.2, 0.3, 0.5, 1.0])

      if 0 <= self.driver_steering_angle_above_timer < 150:
        self.lkas_max_torque = int(round(lkas_max_torque * (self.driver_steering_angle_above_timer / 150) * ego_weight))
      else:
        self.lkas_max_torque = lkas_max_torque * ego_weight

      # Hold torque with induced temporary fault when cutting the actuation bit
      torque_fault = lat_active and not apply_steer_req
    else:
      torque_fault = False

    if not lat_active:
      apply_angle = CS.out.steeringAngleDeg
      apply_steer = 0
      self.lkas_max_torque = 0

    self.apply_angle_last = apply_angle

    self.apply_steer_last = apply_steer

    # accel + longitudinal
    accel = clip(actuators.accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX)
    stopping = actuators.longControlState == LongCtrlState.stopping
    set_speed_in_units = hud_control.setSpeed * (CV.MS_TO_KPH if CS.is_metric else CV.MS_TO_MPH)

    # HUD messages
    sys_warning, sys_state, left_lane_warning, right_lane_warning = process_hud_alert(CC.enabled, self.car_fingerprint,
                                                                                      hud_control)

    can_sends = []

    # *** common hyundai stuff ***

    # tester present - w/ no response (keeps relevant ECU disabled)
    if self.frame % 100 == 0 and not (self.CP.flags & HyundaiFlags.CANFD_CAMERA_SCC.value) and self.CP.openpilotLongitudinalControl and \
     self.experimental_long_enabled and self.CP.carFingerprint not in LEGACY_SAFETY_MODE_CAR_ALT:
      # for longitudinal control, either radar or ADAS driving ECU
      addr, bus = 0x7d0, 0
      if self.CP.flags & HyundaiFlags.CANFD_HDA2.value:
        addr, bus = 0x730, self.CAN.ECAN
      can_sends.append([addr, 0, b"\x02\x3E\x80\x00\x00\x00\x00\x00", bus])

      # for blinkers
      if self.CP.flags & HyundaiFlags.ENABLE_BLINKERS:
        can_sends.append([0x7b1, 0, b"\x02\x3E\x80\x00\x00\x00\x00\x00", self.CAN.ECAN])

    if self.CP.openpilotLongitudinalControl and self.experimental_long_enabled and self.CP.carFingerprint in LEGACY_SAFETY_MODE_CAR_ALT and False: # ToDo
      addr, bus = 0x7d0, 0
      self.radarDisableOverlapTimer += 1
      if self.radarDisableOverlapTimer >= 30:
        if self.radarDisableOverlapTimer > 36:
          if self.frame % 41 == 0 or self.radarDisableOverlapTimer == 37:
            can_sends.append([addr, 0, b"\x02\x10\x03\x00\x00\x00\x00\x00", bus])
          elif self.frame % 43 == 0 or self.radarDisableOverlapTimer == 37:
            can_sends.append([addr, 0, b"\x03\x28\x03\x01\x00\x00\x00\x00", bus])
          elif self.frame % 19 == 0 or self.radarDisableOverlapTimer == 37:
            self.counter_init = True
            can_sends.append([addr, 0, b"\x02\x10\x85\x00\x00\x00\x00\x00", bus])  # radar disable
      else:
        self.counter_init = False
        can_sends.append([addr, 0, b"\x02\x10\x90\x00\x00\x00\x00\x00", bus])
        can_sends.append([addr, 0, b"\x03\x29\x03\x01\x00\x00\x00\x00", bus])

      if (self.frame % 50 == 0 or self.radarDisableOverlapTimer == 37) and self.radarDisableOverlapTimer >= 30:
        can_sends.append([addr, 0, b"\x02\x3E\x00\x00\x00\x00\x00\x00", bus])

    # common
    if CS.cruise_active and CS.lead_distance > 149 and self.dRel < ((CS.out.vEgo * CV.MS_TO_KPH)+5) < 100 and \
    self.vRel*3.6 < -(CS.out.vEgo * CV.MS_TO_KPH * 0.16) and CS.out.vEgo > 7 and abs(CS.out.steeringAngleDeg) < 10 and not self.longcontrol:
      self.need_brake_timer += 1
      if self.need_brake_timer > 100:
        self.need_brake = True
    elif not CS.cruise_active and 1 < self.dRel < (CS.out.vEgo * CV.MS_TO_KPH * 0.5) < 13 and self.vRel*3.6 < -(CS.out.vEgo * CV.MS_TO_KPH * 0.6) and \
      5 < (CS.out.vEgo * CV.MS_TO_KPH) < 20 and not (CS.out.brakeLights or CS.out.brakePressed or CS.out.gasPressed): # generate an event to avoid collision when SCC is not activated at low speed.
      self.need_brake_timer += 1
      if self.need_brake_timer > 50:
        self.need_brake = True
    else:
      self.need_brake = False
      self.need_brake_timer = 0
    if CS.out.autoHold and not self.autohold_popup_switch:
      self.autohold_popup_timer = 100
      self.autohold_popup_switch = True
    elif CS.out.autoHold and self.autohold_popup_switch and self.autohold_popup_timer:
      self.autohold_popup_timer -= 1
    elif not CS.out.autoHold and self.autohold_popup_switch:
      self.autohold_popup_switch = False
      self.autohold_popup_timer = 0

    if not CC.enabled:
      self.cruise_init = False
      self.lkas_temp_disabled = False
      self.e2e_standstill = False
      self.e2e_standstill_stat = False
      self.e2e_standstill_timer = 0
      self.e2e_standstill_timer2 = 0
      self.e2e_standstill_timer_buf = 0
    if CS.cruise_buttons[-1] == 4:
      self.cancel_counter += 1
      self.auto_res_starting = False
      self.standstill_res_button = False
    elif CS.cruise_buttons[-1] == 3:
      self.try_early_stop_retrieve = False
      self.try_early_stop_org_gap = CS.cruiseGapSet
      self.gap_by_spd_gap1 = False
      self.gap_by_spd_gap2 = False
      self.gap_by_spd_gap3 = False
      self.gap_by_spd_gap4 = False
    elif CS.cruise_active:
      self.cruise_init = True
      self.cancel_counter = 0
      self.auto_res_limit_timer = 0
      self.auto_res_delay_timer = 0          
      self.e2e_standstill = False
      self.e2e_standstill_stat = False
      self.e2e_standstill_timer = 0
      self.e2e_standstill_timer2 = 0
      self.e2e_standstill_timer_buf = 0
      if self.res_speed_timer > 0:
        self.res_speed_timer -= 1
        self.auto_res_starting = False
      else:
        self.auto_res_starting = False
        self.v_cruise_kph_auto_res = 0
        self.res_speed = 0
    else:
      if CS.out.brakeLights:
        self.auto_res_limit_timer = 0
        self.auto_res_delay_timer = 0
      else:
        if self.auto_res_limit_timer < self.auto_res_limit_sec:
          self.auto_res_limit_timer += 1
        if self.auto_res_delay_timer < self.auto_res_delay:
          self.auto_res_delay_timer += 1

      if self.e2e_standstill_enable:
        try:
          if self.e2e_standstill:
            self.e2e_standstill_timer += 1
            if self.e2e_standstill_timer > 100:
              self.e2e_standstill = False
              self.e2e_standstill_timer = 0
          elif CS.clu_Vanz >= 1:
            self.e2e_standstill = False
            self.e2e_standstill_stat = False
            self.e2e_standstill_timer = 0
            self.e2e_standstill_timer2 = 0
            self.e2e_standstill_timer_buf = 0
          elif self.e2e_standstill_stat and self.e2e_x > (40 if 0 < self.dRel < 15 else 25) and CS.clu_Vanz < 1:
            self.e2e_standstill_timer2 += 1
            if self.e2e_standstill_timer2 > 20 and not CS.out.gasPressed:
              self.e2e_standstill_timer2 = 0
              self.e2e_standstill = True
              self.e2e_standstill_stat = False
              self.e2e_standstill_timer = 0
              self.e2e_standstill_timer_buf += 500
          elif 0 < self.e2e_x < 10 and CS.clu_Vanz == 0:
            self.e2e_standstill_timer += 1
            self.e2e_standstill_timer2 = 0
            if self.e2e_standstill_timer > (100 + self.e2e_standstill_timer_buf):
              self.e2e_standstill_stat = True
          else:
            self.e2e_standstill_timer = 0
            self.e2e_standstill_timer_buf = 0
        except:
          pass

    if CS.out.brakeLights and CS.out.vEgo == 0 and not CS.out.cruiseState.standstill:
      self.standstill_status_timer += 1
      if self.standstill_status_timer > 200:
        self.standstill_status = 1
        self.standstill_status_timer = 0
    if self.standstill_status == 1 and CS.out.vEgo > 1:
      self.standstill_status = 0
      self.standstill_fault_reduce_timer = 0
      self.last_resume_frame = self.frame
      self.res_switch_timer = 0
      self.resume_cnt = 0

    if CS.out.vEgo <= 1:
      if stopping and CS.out.vEgo < 0.1 and not CS.out.gasPressed:
        self.acc_standstill_timer += 1
        if self.acc_standstill_timer >= 200:
          self.acc_standstill_timer = 200
          self.acc_standstill = True
      else:
        self.acc_standstill_timer = 0
        self.acc_standstill = False
    elif CS.out.gasPressed or CS.out.vEgo > 1:
      self.acc_standstill = False
      self.acc_standstill_timer = 0      
    else:
      self.acc_standstill = False
      self.acc_standstill_timer = 0

    if CS.cruise_active: # to toggle lkas, hold gap button for 1 sec
      if CS.cruise_buttons[-1] == 3:
        self.lkas_onoff_counter += 1
        self.gap_by_spd_on_sw = True
        self.gap_by_spd_on_sw_cnt2 = 0
        if self.lkas_onoff_counter > 100:
          self.lkas_onoff_counter = 0
          self.lkas_temp_disabled = not self.lkas_temp_disabled
          if self.lkas_temp_disabled:
            self.lkas_temp_disabled_timer = 0
          else:
            self.lkas_temp_disabled_timer = 15
      else:
        if self.lkas_temp_disabled_timer:
          self.lkas_temp_disabled_timer -= 1
        self.lkas_onoff_counter = 0
        if self.gap_by_spd_on_sw:
          self.gap_by_spd_on_sw = False
          self.gap_by_spd_on_sw_cnt += 1
          if self.gap_by_spd_on_sw_cnt > 4: #temporary disable of auto gap if you press gap button 5 times quickly.
            self.gap_by_spd_on_sw_trg = not self.gap_by_spd_on_sw_trg
            self.gap_by_spd_on_sw_cnt = 0
            self.gap_by_spd_on_sw_cnt2 = 0
        elif self.gap_by_spd_on_sw_cnt:
          self.gap_by_spd_on_sw_cnt2 += 1
          if self.gap_by_spd_on_sw_cnt2 > 20:
            self.gap_by_spd_on_sw_cnt = 0
            self.gap_by_spd_on_sw_cnt2 = 0
      self.second2 += 1
      if self.second2 > 100:
        self.second2 = 100
      if CS.cruise_buttons[-1] == 3: # push gap 2 times quickly, this is toggle.
        self.exp_mode_push = True
        self.second2 = 0
      elif self.exp_mode_push:
        self.exp_mode_push = False
        self.exp_mode_push_cnt += 1
      elif self.exp_mode_push_cnt == 2 and self.second2 > 50:
        self.exp_mode_push_cnt = 0
        self.experimental_mode_temp = not self.experimental_mode_temp
        if self.experimental_mode_temp:
          self.c_params.put_bool("ExperimentalMode", True)
        else:
          self.c_params.put_bool("ExperimentalMode", False)
      elif self.second2 > 50 and self.exp_mode_push_cnt > 0:
        self.exp_mode_push_cnt = 0
    else:
      self.lkas_onoff_counter = 0
      if self.lkas_temp_disabled_timer:
        self.lkas_temp_disabled_timer -= 1
      self.gap_by_spd_on_sw_cnt = 0
      self.gap_by_spd_on_sw_cnt2 = 0
      self.gap_by_spd_on_sw = False
      self.gap_by_spd_on_sw_trg = True

    if CS.out.cruiseState.modeSel == 0 and self.mode_change_switch == 5:
      self.mode_change_timer = 50
      self.mode_change_switch = 0
    elif CS.out.cruiseState.modeSel == 1 and self.mode_change_switch == 0:
      self.mode_change_timer = 50
      self.mode_change_switch = 1
    elif CS.out.cruiseState.modeSel == 2 and self.mode_change_switch == 1:
      self.mode_change_timer = 50
      self.mode_change_switch = 2
    elif CS.out.cruiseState.modeSel == 3 and self.mode_change_switch == 2:
      self.mode_change_timer = 50
      self.mode_change_switch = 3
    elif CS.out.cruiseState.modeSel == 4 and self.mode_change_switch == 3:
      self.mode_change_timer = 50
      self.mode_change_switch = 4
    elif CS.out.cruiseState.modeSel == 5 and self.mode_change_switch == 4:
      self.mode_change_timer = 50
      self.mode_change_switch = 5
    if self.mode_change_timer > 0:
      self.mode_change_timer -= 1

    # CAN-FD platforms
    if self.CP.carFingerprint in CANFD_CAR:
      hda2 = self.CP.flags & HyundaiFlags.CANFD_HDA2
      #hda2_long = hda2 and self.CP.openpilotLongitudinalControl

      angle_control = self.CP.carFingerprint in ANGLE_CONTROL_CAR
      # steering control
      can_sends.extend(hyundaicanfd.create_steering_messages(self.packer, self.CP, self.CAN, CC.enabled,
                                                             apply_steer_req, CS.out.steeringPressed,
                                                             apply_steer, apply_angle, self.lkas_max_torque, angle_control))

      # prevent LFA from activating on HDA2 by sending "no lane lines detected" to ADAS ECU
      if self.frame % 5 == 0 and hda2:
        if not CC.enabled:
          self.stock_lfa_counter += 1 if self.stock_lfa_counter < 100 else 0
        elif self.stock_lfa_counter:
          self.stock_lfa_counter -= 1
        can_sends.append(hyundaicanfd.create_suppress_lfa(self.packer, self.CAN, CS.hda2_lfa_block_msg,
                                                          self.CP.flags & HyundaiFlags.CANFD_HDA2_ALT_STEERING, CC.enabled, bool(self.stock_lfa_counter)))

      # LFA and HDA icons
      updateLfaHdaIcons = (not hda2) or angle_control
      if self.frame % 5 == 0 and updateLfaHdaIcons:
        can_sends.append(hyundaicanfd.create_lfahda_cluster(self.packer, self.CAN, CC.enabled))

      # blinkers
      if hda2 and self.CP.flags & HyundaiFlags.ENABLE_BLINKERS:
        can_sends.extend(hyundaicanfd.create_spas_messages(self.packer, self.CAN, self.frame, CC.leftBlinker, CC.rightBlinker))

      if self.CP.openpilotLongitudinalControl:
        if hda2:
          can_sends.extend(hyundaicanfd.create_adrv_messages(self.packer, self.CAN, self.frame))
        if self.frame % 2 == 0:
          can_sends.append(hyundaicanfd.create_acc_control(self.packer, self.CAN, CC.enabled, self.accel_last, accel, stopping, CC.cruiseControl.override,
                                                           set_speed_in_units, hud_control))
          self.accel_last = accel
      else:
        # button presses
        can_sends.extend(self.create_button_messages(CC, CS, use_clu11=False))
    else:
      clu11_speed = CS.clu11["CF_Clu_Vanz"]

      can_sends.append(hyundaican.create_lkas11(self.packer, self.frame, self.CP, apply_steer, lat_active and not self.lkas_temp_disabled,
                                                torque_fault, CS.lkas11, sys_warning, sys_state, CC.enabled,
                                                hud_control.leftLaneVisible, hud_control.rightLaneVisible,
                                                left_lane_warning, right_lane_warning, 0, self.ldws_fix))

      if CS.out.cruiseState.standstill:
        self.standstill_status = 1
        if self.kisa_autoresume:
          # run only first time when the car stopped
          if self.last_lead_distance == 0:
            # get the lead distance from the Radar
            self.last_lead_distance = CS.lead_distance
            self.resume_cnt = 0
            self.switch_timer = 0
            self.standstill_fault_reduce_timer += 1
          elif self.switch_timer > 0:
            self.switch_timer -= 1
            self.standstill_fault_reduce_timer += 1
          # at least 0.1 sec delay after entering the standstill
          elif 10 < self.standstill_fault_reduce_timer and CS.lead_distance != self.last_lead_distance and abs(CS.lead_distance - self.last_lead_distance) > 0.1:
            self.acc_standstill_timer = 0
            self.acc_standstill = False
            if self.standstill_resume_alt: # for D.Fyffe, code from neokii
              self.standstill_res_button = True
              can_sends.append(hyundaican.create_clu11(self.packer, self.resume_cnt, CS.clu11, Buttons.RES_ACCEL, clu11_speed, self.CP.sccBus))
              self.resume_cnt += 1
              if self.resume_cnt >= int(randint(4, 5) * 2):
                self.resume_cnt = 0
                self.switch_timer = int(randint(20, 25) * 2)
            else:
              if (self.frame - self.last_resume_frame) * DT_CTRL > 0.1:
                self.standstill_res_button = True
                # send 25 messages at a time to increases the likelihood of resume being accepted, value 25 is not acceptable at some cars.
                can_sends.extend([hyundaican.create_clu11(self.packer, self.frame, CS.clu11, Buttons.RES_ACCEL)] * self.standstill_res_count) if not self.longcontrol \
                else can_sends.extend([hyundaican.create_clu11(self.packer, self.frame, CS.clu11, Buttons.RES_ACCEL, clu11_speed, self.CP.sccBus)] * self.standstill_res_count)
                self.last_resume_frame = self.frame
            self.standstill_fault_reduce_timer += 1
          # gap save after 1sec
          elif 100 < self.standstill_fault_reduce_timer and self.cruise_gap_prev == 0 and CS.cruiseGapSet != 1.0 and self.kisa_autoresume and self.kisa_cruisegap_auto_adj and not self.gap_by_spd_on: 
            self.cruise_gap_prev = CS.cruiseGapSet
            self.cruise_gap_set_init = True
          # gap adjust to 1 for fast start
          elif 110 < self.standstill_fault_reduce_timer and CS.cruiseGapSet != 1.0 and self.kisa_autoresume and self.kisa_cruisegap_auto_adj and not self.gap_by_spd_on:
            can_sends.append(hyundaican.create_clu11(self.packer, self.frame, CS.clu11, Buttons.GAP_DIST)) if not self.longcontrol \
              else can_sends.append(hyundaican.create_clu11(self.packer, self.frame, CS.clu11, Buttons.GAP_DIST, clu11_speed, self.CP.sccBus))
            self.resume_cnt += 1
            if self.resume_cnt >= int(randint(4, 5) * 2):
              self.resume_cnt = 0
              self.switch_timer = int(randint(20, 25) * 2)
            self.cruise_gap_adjusting = True
          elif self.kisa_autoresume:
            self.cruise_gap_adjusting = False
            self.standstill_res_button = False
            self.standstill_fault_reduce_timer += 1
      # reset lead distnce after the car starts moving
      elif self.last_lead_distance != 0:
        self.last_lead_distance = 0
        self.standstill_res_button = False
      elif self.kisa_variablecruise and CS.acc_active:
        btn_signal = self.KCC.update(CS)
        self.btnsignal = btn_signal
        self.on_speed_control = self.KCC.onSpeedControl
        self.on_speed_bump_control = self.KCC.onSpeedBumpControl
        self.curv_speed_control = self.KCC.curvSpeedControl
        self.cut_in_control = self.KCC.cutInControl
        self.driver_scc_set_control = self.KCC.driverSccSetControl
        if self.kisa_cruisegap_auto_adj and not self.gap_by_spd_on:
          # gap restore
          if self.switch_timer > 0:
            self.switch_timer -= 1
          elif self.dRel > 15 and self.vRel*3.6 < 5 and self.cruise_gap_prev != CS.cruiseGapSet and self.cruise_gap_set_init and self.kisa_autoresume:
            can_sends.append(hyundaican.create_clu11(self.packer, self.frame, CS.clu11, Buttons.GAP_DIST)) if not self.longcontrol \
              else can_sends.append(hyundaican.create_clu11(self.packer, self.frame, CS.clu11, Buttons.GAP_DIST, clu11_speed, self.CP.sccBus))
            self.cruise_gap_adjusting = True
            self.resume_cnt += 1
            if self.resume_cnt >= int(randint(4, 5) * 2):
              self.resume_cnt = 0
              self.switch_timer = int(randint(20, 25) * 2)
          elif self.cruise_gap_prev == CS.cruiseGapSet and CS.cruiseGapSet != 1.0 and self.kisa_autoresume:
            self.cruise_gap_set_init = False
            self.cruise_gap_prev = 0
            self.cruise_gap_adjusting = False
          else:
            self.cruise_gap_adjusting = False
        if not self.cruise_gap_adjusting:
          if not self.gap_by_spd_on or not self.gap_by_spd_on_sw_trg:
            if 0 < CS.lead_distance <= 149 and CS.lead_objspd < 0 and self.try_early_stop and CS.cruiseGapSet != 4.0 and CS.clu_Vanz > 30 and \
             0 < self.e2e_x < 120 and CS.lead_objspd < -4:
              if not self.try_early_stop_retrieve:
                self.try_early_stop_org_gap = CS.cruiseGapSet
                self.try_early_stop_retrieve = True
              if self.switch_timer > 0:
                self.switch_timer -= 1
              else:
                can_sends.append(hyundaican.create_clu11(self.packer, self.frame, CS.clu11, Buttons.GAP_DIST)) if not self.longcontrol \
                  else can_sends.append(hyundaican.create_clu11(self.packer, self.frame, CS.clu11, Buttons.GAP_DIST, clu11_speed, self.CP.sccBus))
                self.resume_cnt += 1
                if self.resume_cnt >= int(randint(4, 5) * 2):
                  self.resume_cnt = 0
                  self.switch_timer = int(randint(20, 25) * 2)
            elif btn_signal != None:
              if self.switch_timer > 0:
                self.switch_timer -= 1
              else:
                btn_count = 1
                btn_count = int(interp(self.KCC.t_interval, [self.nt_interval+3,70],[1,self.btn_count])) if CS.is_set_speed_in_mph else int(interp(self.KCC.t_interval, [self.nt_interval,40],[1,self.btn_count]))
                can_sends.extend([hyundaican.create_clu11(self.packer, self.resume_cnt, CS.clu11, btn_signal)] * btn_count) if not self.longcontrol \
                else can_sends.extend([hyundaican.create_clu11(self.packer, self.frame, CS.clu11, btn_signal, clu11_speed, self.CP.sccBus)] * btn_count)
                self.resume_cnt += 1
                if self.resume_cnt >= int(randint(4, 5) * 2):
                  self.resume_cnt = 0
                  self.switch_timer = int(randint(20, 25) * 2)
            elif 0 < CS.lead_distance <= 149 and not self.cruise_gap_set_init and self.try_early_stop and self.try_early_stop_retrieve and \
            CS.cruiseGapSet != self.try_early_stop_org_gap and \
            (CS.clu_Vanz <= 20 or (CS.lead_objspd >= 0 and self.e2e_x > 50 and CS.clu_Vanz > 20)):
              if self.switch_timer > 0:
                self.switch_timer -= 1
              else:
                can_sends.append(hyundaican.create_clu11(self.packer, self.frame, CS.clu11, Buttons.GAP_DIST)) if not self.longcontrol \
                  else can_sends.append(hyundaican.create_clu11(self.packer, self.frame, CS.clu11, Buttons.GAP_DIST, clu11_speed, self.CP.sccBus))
                self.resume_cnt += 1
                if self.resume_cnt >= int(randint(4, 5) * 2):
                  self.resume_cnt = 0
                  self.switch_timer = int(randint(20, 25) * 2)
              if CS.cruiseGapSet == self.try_early_stop_org_gap:
                self.try_early_stop_retrieve = False
            else:
              self.resume_cnt = 0
          elif self.gap_by_spd_on and self.gap_by_spd_on_sw_trg:
            if 0 < CS.lead_distance <= 149 and CS.lead_objspd < 0 and self.try_early_stop and CS.cruiseGapSet != 4.0 and CS.clu_Vanz > 30 and \
             0 < self.e2e_x < 120 and CS.lead_objspd < -4:
              if not self.try_early_stop_retrieve:
                self.try_early_stop_org_gap = CS.cruiseGapSet
                self.try_early_stop_retrieve = True
              if self.switch_timer > 0:
                self.switch_timer -= 1
              else:
                can_sends.append(hyundaican.create_clu11(self.packer, self.frame, CS.clu11, Buttons.GAP_DIST)) if not self.longcontrol \
                  else can_sends.append(hyundaican.create_clu11(self.packer, self.frame, CS.clu11, Buttons.GAP_DIST, clu11_speed, self.CP.sccBus))
                self.resume_cnt += 1
                if self.resume_cnt >= int(randint(4, 5) * 2):
                  self.resume_cnt = 0
                  self.switch_timer = int(randint(20, 25) * 2)
                self.switch_timer2 = int(randint(20, 25) * 2)
            elif self.switch_timer > 0 and not self.try_early_stop_retrieve:
              self.switch_timer -= 1
            elif CS.cruiseGapSet != self.gap_by_spd_gap[0] and ((CS.clu_Vanz < self.gap_by_spd_spd[0]+self.gap_by_spd_on_buffer1) or self.gap_by_spd_gap1) and not self.try_early_stop_retrieve and not CS.lead_objspd < 0:
              self.gap_by_spd_gap1 = True
              self.gap_by_spd_gap2 = False
              self.gap_by_spd_gap3 = False
              self.gap_by_spd_gap4 = False
              self.gap_by_spd_on_buffer1 = 0
              self.gap_by_spd_on_buffer2 = 0
              can_sends.append(hyundaican.create_clu11(self.packer, self.frame, CS.clu11, Buttons.GAP_DIST)) if not self.longcontrol \
                else can_sends.append(hyundaican.create_clu11(self.packer, self.frame, CS.clu11, Buttons.GAP_DIST, clu11_speed, self.CP.sccBus))
              self.resume_cnt += 1
              if self.resume_cnt >= int(randint(4, 5) * 2):
                self.resume_cnt = 0
                self.switch_timer = int(randint(20, 25) * 2)
            elif CS.cruiseGapSet != self.gap_by_spd_gap[1] and ((self.gap_by_spd_spd[0] <= CS.clu_Vanz < self.gap_by_spd_spd[1]+self.gap_by_spd_on_buffer2) or self.gap_by_spd_gap2) and not self.try_early_stop_retrieve and not CS.lead_objspd < 0:
              self.gap_by_spd_gap1 = False
              self.gap_by_spd_gap2 = True
              self.gap_by_spd_gap3 = False
              self.gap_by_spd_gap4 = False
              self.gap_by_spd_on_buffer1 = -5
              self.gap_by_spd_on_buffer3 = 0
              can_sends.append(hyundaican.create_clu11(self.packer, self.frame, CS.clu11, Buttons.GAP_DIST)) if not self.longcontrol \
                else can_sends.append(hyundaican.create_clu11(self.packer, self.frame, CS.clu11, Buttons.GAP_DIST, clu11_speed, self.CP.sccBus))
              self.resume_cnt += 1
              if self.resume_cnt >= int(randint(4, 5) * 2):
                self.resume_cnt = 0
                self.switch_timer = int(randint(20, 25) * 2)
            elif CS.cruiseGapSet != self.gap_by_spd_gap[2] and ((self.gap_by_spd_spd[1] <= CS.clu_Vanz < self.gap_by_spd_spd[2]+self.gap_by_spd_on_buffer3) or self.gap_by_spd_gap3) and not self.try_early_stop_retrieve and not CS.lead_objspd < 0:
              self.gap_by_spd_gap1 = False
              self.gap_by_spd_gap2 = False
              self.gap_by_spd_gap3 = True
              self.gap_by_spd_gap4 = False
              self.gap_by_spd_on_buffer2 = -5
              can_sends.append(hyundaican.create_clu11(self.packer, self.frame, CS.clu11, Buttons.GAP_DIST)) if not self.longcontrol \
                else can_sends.append(hyundaican.create_clu11(self.packer, self.frame, CS.clu11, Buttons.GAP_DIST, clu11_speed, self.CP.sccBus))
              self.resume_cnt += 1
              if self.resume_cnt >= int(randint(4, 5) * 2):
                self.resume_cnt = 0
                self.switch_timer = int(randint(20, 25) * 2)
            elif CS.cruiseGapSet != self.gap_by_spd_gap[3] and ((self.gap_by_spd_spd[2] <= CS.clu_Vanz) or self.gap_by_spd_gap4) and not self.try_early_stop_retrieve and not CS.lead_objspd < 0:
              self.gap_by_spd_gap1 = False
              self.gap_by_spd_gap2 = False
              self.gap_by_spd_gap3 = False
              self.gap_by_spd_gap4 = True
              self.gap_by_spd_on_buffer3 = -5
              can_sends.append(hyundaican.create_clu11(self.packer, self.frame, CS.clu11, Buttons.GAP_DIST)) if not self.longcontrol \
                else can_sends.append(hyundaican.create_clu11(self.packer, self.frame, CS.clu11, Buttons.GAP_DIST, clu11_speed, self.CP.sccBus))
              self.resume_cnt += 1
              if self.resume_cnt >= int(randint(4, 5) * 2):
                self.resume_cnt = 0
                self.switch_timer = int(randint(20, 25) * 2)
            elif btn_signal != None:
              if self.switch_timer2 > 0 and self.try_early_stop_retrieve:
                self.switch_timer2 -= 1
              elif self.switch_timer > 0:
                self.switch_timer -= 1
              else:
                btn_count = 1
                btn_count = int(interp(self.KCC.t_interval, [self.nt_interval+3,70],[1,self.btn_count])) if CS.is_set_speed_in_mph else int(interp(self.KCC.t_interval, [self.nt_interval,40],[1,self.btn_count]))
                can_sends.extend([hyundaican.create_clu11(self.packer, self.resume_cnt, CS.clu11, btn_signal)] * btn_count) if not self.longcontrol \
                else can_sends.extend([hyundaican.create_clu11(self.packer, self.frame, CS.clu11, btn_signal, clu11_speed, self.CP.sccBus)] * btn_count)
                self.resume_cnt += 1
                if self.resume_cnt >= int(randint(4, 5) * 2):
                  self.resume_cnt = 0
                  self.switch_timer = int(randint(20, 25) * 2)
              self.gap_by_spd_gap1 = False
              self.gap_by_spd_gap2 = False
              self.gap_by_spd_gap3 = False
              self.gap_by_spd_gap4 = False
            elif 0 < CS.lead_distance <= 149 and not self.cruise_gap_set_init and self.try_early_stop and self.try_early_stop_retrieve and \
            CS.cruiseGapSet != self.try_early_stop_org_gap and \
            (CS.clu_Vanz <= 20 or (CS.lead_objspd >= 0 and self.e2e_x > 50 and CS.clu_Vanz > 20)):
              if self.switch_timer > 0:
                self.switch_timer -= 1
              else:
                can_sends.append(hyundaican.create_clu11(self.packer, self.frame, CS.clu11, Buttons.GAP_DIST)) if not self.longcontrol \
                  else can_sends.append(hyundaican.create_clu11(self.packer, self.frame, CS.clu11, Buttons.GAP_DIST, clu11_speed, self.CP.sccBus))
                self.resume_cnt += 1
                if self.resume_cnt >= int(randint(4, 5) * 2):
                  self.resume_cnt = 0
                  self.switch_timer = int(randint(20, 25) * 2)
                self.switch_timer2 = int(randint(20, 25) * 2)
              if CS.cruiseGapSet == self.try_early_stop_org_gap:
                self.try_early_stop_retrieve = False
              self.gap_by_spd_gap1 = False
              self.gap_by_spd_gap2 = False
              self.gap_by_spd_gap3 = False
              self.gap_by_spd_gap4 = False
            elif 0 < CS.lead_distance <= 149 and not self.cruise_gap_set_init and self.try_early_stop and self.try_early_stop_retrieve and \
            CS.cruiseGapSet == self.try_early_stop_org_gap and \
            (CS.clu_Vanz <= 20 or (CS.lead_objspd >= 0 and self.e2e_x > 50 and CS.clu_Vanz > 20)):
              self.try_early_stop_retrieve = False
              self.gap_by_spd_gap1 = False
              self.gap_by_spd_gap2 = False
              self.gap_by_spd_gap3 = False
              self.gap_by_spd_gap4 = False
            else:
              self.resume_cnt = 0
              self.gap_by_spd_gap1 = False
              self.gap_by_spd_gap2 = False
              self.gap_by_spd_gap3 = False
              self.gap_by_spd_gap4 = False
      else:
        self.on_speed_control = False
        self.on_speed_bump_control = False
        self.curv_speed_control = False
        self.cut_in_control = False
        self.driver_scc_set_control = False
        self.cruise_gap_adjusting = False
        self.standstill_res_button = False
        self.auto_res_starting = False
        self.btnsignal = 0

      kisa_cruise_auto_res_condition = False
      kisa_cruise_auto_res_condition = not self.kisa_cruise_auto_res_condition or CS.out.gasPressed
      t_speed = 20 if CS.is_set_speed_in_mph else 30
      if self.auto_res_timer > 0:
        self.auto_res_timer -= 1
      elif self.model_speed > (60 if CS.is_set_speed_in_mph else 95) and self.cancel_counter == 0 and not CS.cruise_active and not CS.out.brakeLights and round(CS.VSetDis) >= t_speed and \
      (1 < CS.lead_distance < 149 or round(CS.clu_Vanz) > t_speed) and round(CS.clu_Vanz) >= 3 and self.cruise_init and \
      self.kisa_cruise_auto_res and kisa_cruise_auto_res_condition and (self.auto_res_limit_sec == 0 or self.auto_res_limit_timer < self.auto_res_limit_sec) and \
      (self.auto_res_delay == 0 or self.auto_res_delay_timer >= self.auto_res_delay):
        if self.kisa_cruise_auto_res_option == 0:
          can_sends.append(hyundaican.create_clu11(self.packer, self.frame, CS.clu11, Buttons.RES_ACCEL)) if not self.longcontrol \
          else can_sends.append(hyundaican.create_clu11(self.packer, self.frame, CS.clu11, Buttons.RES_ACCEL, clu11_speed, self.CP.sccBus))  # auto res
          self.auto_res_starting = True
          self.res_speed = round(CS.VSetDis) if CS.is_set_speed_in_mph or self.osm_spdlimit_enabled else round(CS.clu_Vanz*1.1)
          self.res_speed_timer = 300
          self.resume_cnt += 1
          if self.resume_cnt >= int(randint(4, 5) * 2):
            self.resume_cnt = 0
            self.auto_res_timer = int(randint(20, 25) * 2)
        elif self.kisa_cruise_auto_res_option == 1:
          can_sends.append(hyundaican.create_clu11(self.packer, self.frame, CS.clu11, Buttons.SET_DECEL)) if not self.longcontrol \
          else can_sends.append(hyundaican.create_clu11(self.packer, self.frame, CS.clu11, Buttons.SET_DECEL, clu11_speed, self.CP.sccBus)) # auto res but set_decel to set current speed
          self.auto_res_starting = True
          self.v_cruise_kph_auto_res = round(CS.clu_Vanz)
          self.res_speed_timer = 50
          self.resume_cnt += 1
          if self.resume_cnt >= int(randint(4, 5) * 2):
            self.resume_cnt = 0
            self.auto_res_timer = int(randint(20, 25) * 2)
        elif self.kisa_cruise_auto_res_option == 2:
          if not self.longcontrol:
            can_sends.append(hyundaican.create_clu11(self.packer, self.frame, CS.clu11, Buttons.RES_ACCEL)) if 1 < CS.lead_distance < 149 \
            else can_sends.append(hyundaican.create_clu11(self.packer, self.frame, CS.clu11, Buttons.SET_DECEL))
          else:
            can_sends.append(hyundaican.create_clu11(self.packer, self.frame, CS.clu11, Buttons.RES_ACCEL, clu11_speed, self.CP.sccBus)) if 1 < CS.lead_distance < 149 \
            else can_sends.append(hyundaican.create_clu11(self.packer, self.frame, CS.clu11, Buttons.SET_DECEL, clu11_speed, self.CP.sccBus))
          self.auto_res_starting = True
          self.v_cruise_kph_auto_res = round(CS.clu_Vanz)
          self.res_speed_timer = 50
          self.resume_cnt += 1
          if self.resume_cnt >= int(randint(4, 5) * 2):
            self.resume_cnt = 0
            self.auto_res_timer = int(randint(20, 25) * 2)

      if not self.CP.openpilotLongitudinalControl and self.CP.carFingerprint in CANFD_CAR:
        can_sends.extend(self.create_button_messages(CC, CS, use_clu11=True))

      if self.CP.openpilotLongitudinalControl and self.experimental_long_enabled:
        if self.prev_gapButton != CS.cruise_buttons[-1]:  # gap change.
          if CS.cruise_buttons[-1] == 3:
            self.gapsettingdance -= 1
          if self.gapsettingdance < 1:
            self.gapsettingdance = 4
          self.prev_gapButton = CS.cruise_buttons[-1]

      if self.frame % 2 == 0 and self.CP.openpilotLongitudinalControl and self.experimental_long_enabled:
        # TODO: unclear if this is needed
        jerk = 3.0 if actuators.longControlState == LongCtrlState.pid else 1.0
        use_fca = self.CP.flags & HyundaiFlags.USE_FCA.value
        can_sends.extend(hyundaican.create_acc_commands(self.packer, CC.enabled, accel, jerk, int(self.frame / 2),
                                                        hud_control, set_speed_in_units, stopping,
                                                        CC.cruiseControl.override, use_fca, self.gapsettingdance))
      # 20 Hz LFA MFA message
      if self.frame % 5 == 0 and self.CP.flags & HyundaiFlags.SEND_LFA.value:
        can_sends.append(hyundaican.create_lfahda_mfc(self.packer, CC.enabled))

      # 5 Hz ACC options
      if self.frame % 20 == 0 and self.CP.openpilotLongitudinalControl and self.experimental_long_enabled:
        can_sends.extend(hyundaican.create_acc_opt(self.packer))

      # 2 Hz front radar options
      if self.frame % 50 == 0 and self.CP.openpilotLongitudinalControl and self.experimental_long_enabled:
        can_sends.append(hyundaican.create_frt_radar_opt(self.packer))


      if self.CP.sccBus == 2 and self.longcontrol and self.kisa_long_alt:
        if self.frame % 2 == 0:
          lead_objspd = CS.lead_objspd  # vRel (km/h)
          aReqValue = CS.scc12["aReqValue"]
          faccel = actuators.accel if CC.longActive and not CS.out.gasPressed else 0
          accel = actuators.oaccel if CC.longActive and not CS.out.gasPressed else 0
          radar_recog = (0 < CS.lead_distance <= 149)
          if self.joystick_debug_mode:
            accel = actuators.accel
          elif self.radar_helper_option == 0: # Vision Only
            if 0 < CS.lead_distance <= self.stoppingdist:
              stock_weight = interp(CS.lead_distance, [2.0, self.stoppingdist], [1., 0.])
              accel = accel * (1. - stock_weight) + aReqValue * stock_weight
              self.l_stat = 1
            elif 0.1 < self.dRel < (self.stoppingdist + 2.0) and self.vRel < 0:
              accel = self.accel - (DT_CTRL * interp(CS.out.vEgo, [1.0, 3.0], [0.5, 2.0]))
              self.stopped = False
              self.l_stat = 2
            elif 0.1 < self.dRel < (self.stoppingdist + 2.0):
              accel = min(-0.5, faccel*0.5)
              if stopping:
                self.stopped = True
                self.l_stat = 3
              else:
                self.stopped = False
                self.l_stat = 4
            else:
              self.stopped = False
              accel = faccel
              self.l_stat = 5
          elif self.radar_helper_option == 1: # Radar Only
            accel = aReqValue
            self.l_stat = 6
          elif self.radar_helper_option >= 2: # KISA Custom(Radar+Vision), more smooth slowdown for cut-in or encountering being decellerated car.
            if self.experimental_mode_temp:
              self.stopped = False
              if stopping:
                self.smooth_start = True
                accel = min(-0.5, accel, faccel*0.5)
                self.l_stat = 7
              elif self.smooth_start and CS.clu_Vanz < round(CS.VSetDis)*0.9:
                accel = interp(CS.clu_Vanz, [0, round(CS.VSetDis)], [min(accel*0.6, faccel*0.6), aReqValue])
                self.l_stat = 8
              else:
                self.smooth_start = False
                accel = faccel
                self.l_stat = 9
            elif 0 < CS.lead_distance <= 149 and CS.clu_Vanz > 3 and self.smooth_start:
              self.smooth_start = False
              accel = aReqValue
              self.l_stat = 10
            elif 0 < CS.lead_distance <= 149 and not self.smooth_start: # prevent moving forward at exp stop
              stock_weight = 0.0
              self.smooth_start = False
              self.vrel_delta_timer2 += 1
              if self.vrel_delta_timer2 > 10:
                self.vrel_delta_timer2 = 0
                self.vrel_delta = (self.vRel*3.6) - self.vrel_delta_prev
                self.vrel_delta_prev = self.vRel*3.6
                self.l_stat = 11
              if accel > 0 and self.change_accel_fast and CS.out.vEgo < 11.:
                if aReqValue >= accel:
                  self.change_accel_fast = False
                  self.l_stat = 12
                else:
                  accel = (aReqValue + accel) / 2
                  self.l_stat = 13
              elif aReqValue < 0 and accel > 0 and accel - aReqValue > 0.3 and lead_objspd > 0 and CS.out.vEgo < 11.:
                self.change_accel_fast = True
                self.l_stat = 14
              elif 0.1 < self.dRel < 6 and CS.lead_distance < 30.0 and lead_objspd > 0 and aReqValue - accel > 0.8: # in case radar detection works during vision breaking at stop.
                accel = interp(aReqValue, [0.0, 1.8], [0.0, -0.7])
                self.change_accel_fast = False
                self.l_stat = 15
              elif 0.1 < self.dRel <= 10.0 and CS.lead_distance - self.dRel >= 5.0 and aReqValue >= 0:
                self.change_accel_fast = False
                self.l_stat = 16
                pass
              elif aReqValue >= 0.0:
                self.ed_rd_diff_on_timer = 0
                self.ed_rd_diff_on_timer2 = 0
                dRel1 = self.dRel if self.dRel > 0 else CS.lead_distance
                if ((CS.lead_distance - dRel1 > 3.0) or self.KCC.cutInControl) and accel < 0:
                  if aReqValue < accel:
                    accel = interp(lead_objspd, [-1, 0, 5], [aReqValue, aReqValue, accel])
                    self.l_stat = 17
                  else:
                    accel = interp(self.dRel, [0, 40], [accel*0.1, accel*0.7])
                    self.l_stat = 18
                else:
                  if aReqValue < accel:
                    accel = interp(CS.lead_distance, [5.0, 13.0, 20.0], [aReqValue, accel*0.8+aReqValue*0.2, aReqValue])
                    self.l_stat = 19
                  else:
                    accel = aReqValue
                    self.l_stat = 20
              elif aReqValue < 0.0 and CS.lead_distance < self.stoppingdist+0.5 and accel >= aReqValue and lead_objspd <= 0 and self.stopping_dist_adj_enabled:
                self.l_stat = 21
                if CS.lead_distance < 1.5:
                  accel = self.accel - (DT_CTRL * 2.5)
                  self.l_stat = 22
                elif CS.lead_distance < self.stoppingdist+0.8:
                  accel = self.accel - (DT_CTRL * interp(CS.out.vEgo, [0.0, 1.0, 2.0], [0.02, 0.8, 4.5]))
                  self.l_stat = 23
              elif aReqValue < 0.0:
                dRel2 = self.dRel if self.dRel > 0 else CS.lead_distance
                dist_by_drel = interp(CS.lead_distance, [10, 50], [3.0, 9.0])
                d_ratio = interp(CS.clu_Vanz, [40, 110], [0.3, 0.19])
                d_ratio2 = interp(CS.clu_Vanz, [40, 110], [1.0, 2.0])
                if ((CS.lead_distance - dRel2 > dist_by_drel) or self.KCC.cutInControl) and accel < 0 and not self.ed_rd_diff_on:
                  self.ed_rd_diff_on = True
                  self.ed_rd_diff_on_timer = min(400, int(self.dRel * 5 * d_ratio2))
                  self.ed_rd_diff_on_timer2 = min(400, int(self.dRel * 5 * d_ratio2))
                  stock_weight = 1.0
                  self.l_stat = 24
                elif ((dRel2 - CS.lead_distance > dist_by_drel) or self.KCC.cutInControl) and not self.ed_rd_diff_on:
                  self.ed_rd_diff_on = True
                  self.ed_rd_diff_on_timer = min(400, int(self.dRel * 10 * d_ratio2))
                  self.ed_rd_diff_on_timer2 = min(400, int(self.dRel * 10 * d_ratio2))
                  stock_weight = 1.0
                  self.l_stat = 25
                elif self.ed_rd_diff_on_timer or (self.KCC.cut_in_run_timer and dRel2 < CS.clu_Vanz * d_ratio): # damping btw ED and RD for few secs.
                  stock_weight = interp(self.ed_rd_diff_on_timer, [0, self.ed_rd_diff_on_timer2], [0.1, 1.0])
                  self.ed_rd_diff_on_timer -= 1
                  self.l_stat = 26
                  if aReqValue <= accel:
                    stock_weight = 1.0
                    self.l_stat = 27
                else:
                  accel = interp(CS.clu_Vanz, [4.0, 10.0, 30.0], [max(accel, faccel), ((min(accel, faccel)*0.6)+(max(accel, faccel)*0.4)), min(accel, faccel)])
                  if not self.KCC.cutInControl:
                    self.ed_rd_diff_on = False
                    self.l_stat = 28
                  self.ed_rd_diff_on_timer = 0
                  self.ed_rd_diff_on_timer2 = 0
                  stock_weight = interp(abs(lead_objspd), [1.0, 5.0, 10.0, 20.0, 50.0], [0.15, 0.4, 1.0, 0.9, 0.2])
                  self.l_stat = 29
                  if aReqValue <= accel:
                    self.vrel_delta_timer = 0
                    self.vrel_delta_timer3 = 0
                    stock_weight = min(1.0, interp(CS.out.vEgo, [8.0, 30.0], [stock_weight, stock_weight*5.0]))
                    self.l_stat = 30
                    if not self.stopping_dist_adj_enabled:
                      stock_weight = min(1.0, interp(CS.lead_distance, [0.0, 10.0], [stock_weight*5.0, stock_weight]))
                      self.l_stat = 31
                  elif aReqValue > accel:
                    if self.vrel_delta < -5 and self.vrel_delta_timer == 0:
                      self.vrel_delta_timer = min(400, int(self.dRel*10))
                      self.vrel_delta_timer3 = min(400, int(self.dRel*10))
                      stock_weight = 1.0
                      self.l_stat = 32
                    elif self.vrel_delta_timer > 0:
                      self.vrel_delta_timer -= 1
                      stock_weight = interp(self.vrel_delta_timer, [0, self.vrel_delta_timer3], [0.1, 1.0])
                      self.l_stat = 33
                    else:
                      self.vrel_delta_timer = 0
                      self.vrel_delta_timer3 = 0
                      stock_weight = interp(abs(lead_objspd), [1.0, 15.0], [1.0, 0.1])
                      self.l_stat = 34
                accel = accel * (1.0 - stock_weight) + aReqValue * stock_weight
                accel = min(accel, -0.5) if CS.lead_distance <= self.stoppingdist+0.5 and not CS.out.standstill else accel
              # elif aReqValue < 0.0:
              #   stock_weight = interp(CS.lead_distance, [6.0, 10.0, 18.0, 25.0, 32.0], [1.0, 0.85, 1.0, 0.4, 1.0])
              #   accel = accel * (1.0 - stock_weight) + aReqValue * stock_weight
              else:
                stock_weight = 0.0
                self.change_accel_fast = False
                accel = accel * (1.0 - stock_weight) + aReqValue * stock_weight
                self.l_stat = 35
            elif 0.1 < self.dRel < (self.stoppingdist + 1.5) and int(self.vRel*3.6) < 0:
              accel = self.accel - (DT_CTRL * interp(CS.out.vEgo, [0.0, 1.0, 2.0], [0.04, 0.6, 1.1]))
              self.stopped = False
              self.l_stat = 36
            elif 0.1 < self.dRel < (self.stoppingdist + 1.5):
              accel = min(-0.6, faccel*0.5)
              if stopping:
                self.stopped = True
                self.l_stat = 37
              else:
                self.stopped = False
                self.l_stat = 38
            elif 0.1 < self.dRel < 90:
              self.stopped = False
              ddrel_weight = interp(self.dRel, [self.stoppingdist+1.5, 30], [1.0, 1.0])
              accel = faccel*ddrel_weight
              self.l_stat = 39
            else:
              self.stopped = False
              if stopping:
                self.smooth_start = True
                accel = min(-0.5, accel, faccel*0.5)
                self.l_stat = 40
              elif self.smooth_start and CS.clu_Vanz < round(CS.VSetDis)*0.9:
                accel = interp(CS.clu_Vanz, [0, round(CS.VSetDis)], [min(accel*0.6, faccel*0.6), aReqValue])
                self.l_stat = 41
              else:
                self.l_stat = 42
                if self.smooth_start:
                  self.smooth_start = False
                  self.l_stat = 43
                accel = aReqValue
          else:
            self.stopped = False
            stock_weight = 0.
            self.l_stat = 44

          if self.stock_safety_decel_enabled:
            if CS.highway_cam == 2 and accel > aReqValue:
              accel = aReqValue
          accel = clip(accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX)
          self.aq_value = accel
          self.aq_value_raw = aReqValue
          can_sends.append(hyundaican.create_scc11(self.packer, self.frame, set_speed_in_units, hud_control.leadVisible, self.scc_live, self.dRel, self.vRel, self.yRel, 
          self.car_fingerprint, CS.out.vEgo * CV.MS_TO_KPH, self.acc_standstill, self.gapsettingdance, self.stopped, radar_recog, CS.scc11))
          if (CS.brake_check or CS.cancel_check) and self.car_fingerprint != CAR.KIA_NIRO_EV:
            can_sends.append(hyundaican.create_scc12(self.packer, accel, CC.enabled, self.scc_live, CS.out.gasPressed, 1, 
            CS.out.stockAeb, self.car_fingerprint, CS.out.vEgo * CV.MS_TO_KPH, self.stopped, self.acc_standstill, radar_recog, self.scc12_cnt, CS.scc12))
          else:
            can_sends.append(hyundaican.create_scc12(self.packer, accel, CC.enabled, self.scc_live, CS.out.gasPressed, CS.out.brakePressed, 
            CS.out.stockAeb, self.car_fingerprint, CS.out.vEgo * CV.MS_TO_KPH, self.stopped, self.acc_standstill, radar_recog, self.scc12_cnt, CS.scc12))
          self.scc12_cnt += 1
          if self.CP.scc14Available:
            can_sends.append(hyundaican.create_scc14(self.packer, CC.enabled, CS.scc14, CS.out.stockAeb, hud_control.leadVisible, self.dRel, 
             CS.out.vEgo, self.acc_standstill, self.car_fingerprint))
          self.accel = accel

        if self.frame % 20 == 0 and self.CP.scc13Available:
          can_sends.append(hyundaican.create_scc13(self.packer, CS.scc13))
        if self.frame % 50 == 0:
          can_sends.append(hyundaican.create_scc42a(self.packer))

    if self.CP.carFingerprint in CANFD_CAR:
      if self.car_fingerprint in ANGLE_CONTROL_CAR:
        str_log1 = 'EN/LA/LO={}/{}{}/{}  MD={}  BS={:1.0f}  CV={:03.0f}/{:0.4f}  TQ={:03.0f}/{:03.0f}  VF={:03.0f}  ST={:03.0f}/{:01.0f}/{:01.0f}'.format(
          int(CC.enabled), int(CC.latActive), int(lat_active), int(CC.longActive), CS.out.cruiseState.modeSel, self.CP.sccBus, self.model_speed, abs(self.sm['controlsState'].curvature), self.lkas_max_torque, abs(CS.out.steeringTorque), self.vFuture, self.params.STEER_MAX, self.params.STEER_DELTA_UP, self.params.STEER_DELTA_DOWN)
      else:
        str_log1 = 'EN/LA/LO={}/{}{}/{}  MD={}  BS={:1.0f}  CV={:03.0f}/{:0.4f}  TQ={:03.0f}/{:03.0f}  VF={:03.0f}  ST={:03.0f}/{:01.0f}/{:01.0f}'.format(
          int(CC.enabled), int(CC.latActive), int(lat_active), int(CC.longActive), CS.out.cruiseState.modeSel, self.CP.sccBus, self.model_speed, abs(self.sm['controlsState'].curvature), abs(new_steer), abs(CS.out.steeringTorque), self.vFuture, self.params.STEER_MAX, self.params.STEER_DELTA_UP, self.params.STEER_DELTA_DOWN)
      if CS.out.cruiseState.accActive:
        str_log2 = 'AQ={:+04.2f}  SS={:03.0f}  VF={:03.0f}/{:03.0f}  TS/VS={:03.0f}/{:03.0f}  RD/ED/C/T={:04.1f}/{:04.1f}/{}/{}  C={:1.0f}/{:1.0f}/{}'.format(
        self.aq_value if self.longcontrol else CS.scc_control["aReqValue"], set_speed_in_units, self.vFuture, self.vFutureA, self.KCC.ctrl_speed, round(CS.VSetDis), CS.lead_distance, self.dRel, int(self.KCC.cut_in), self.KCC.cut_in_run_timer, 0, CS.cruiseGapSet, self.btnsignal if self.btnsignal is not None else 0, self.KCC.t_interval)
      else:
        str_log2 = 'MDPS={}  LKAS={:1.0f}  LEAD={}  AQ={:+04.2f}  VF={:03.0f}/{:03.0f}  CG={:1.0f}'.format(
        int(not CS.out.steerFaultTemporary), 0, int(bool(0 < CS.lead_distance < 149)), self.aq_value if self.longcontrol else CS.scc_control["aReqValue"], self.vFuture, self.vFutureA, CS.cruiseGapSet)
    else:
      str_log1 = 'EN/LA/LO={}/{}{}/{}  MD={}  BS={:1.0f}  CV={:03.0f}/{:0.4f}  TQ={:03.0f}/{:03.0f}  VF={:03.0f}  ST={:03.0f}/{:01.0f}/{:01.0f}'.format(
        int(CC.enabled), int(CC.latActive), int(lat_active), int(CC.longActive), CS.out.cruiseState.modeSel, self.CP.sccBus, self.model_speed, abs(self.sm['controlsState'].curvature), abs(new_steer), abs(CS.out.steeringTorque), self.vFuture, self.params.STEER_MAX, self.params.STEER_DELTA_UP, self.params.STEER_DELTA_DOWN)
      if CS.out.cruiseState.accActive:
        str_log2 = 'AQ={:+04.2f}  SS={:03.0f}  VF={:03.0f}/{:03.0f}  TS/VS={:03.0f}/{:03.0f}  RD/ED/C/T={:04.1f}/{:04.1f}/{}/{}/{}  C={:1.0f}/{:1.0f}/{}/{:1.0f}'.format(
        self.aq_value if self.longcontrol else CS.scc12["aReqValue"], set_speed_in_units, self.vFuture, self.vFutureA, self.KCC.ctrl_speed, round(CS.VSetDis), CS.lead_distance, self.dRel, int(self.KCC.cut_in), self.KCC.cut_in_run_timer, self.ed_rd_diff_on_timer, CS.cruiseGapSet, self.btnsignal if self.btnsignal is not None else 0, self.KCC.t_interval, self.l_stat)
      else:
        str_log2 = 'MDPS={}  LKAS={:1.0f}  LEAD={}  AQ={:+04.2f}  VF={:03.0f}/{:03.0f}  CG={:1.0f}'.format(
        int(not CS.out.steerFaultTemporary), CS.lkas_button_on, int(bool(0 < CS.lead_distance < 149)), self.aq_value if self.longcontrol else CS.scc12["aReqValue"], self.vFuture, self.vFutureA, CS.cruiseGapSet)

    self.cc_timer += 1
    if self.cc_timer > 100:
      self.cc_timer = 0
      # self.radar_helper_option = int(self.c_params.get("RadarLongHelper", encoding="utf8"))
      # self.stopping_dist_adj_enabled = self.c_params.get_bool("StoppingDistAdj")
      # self.standstill_res_count = int(self.c_params.get("RESCountatStandstill", encoding="utf8"))
      # self.kisa_cruisegap_auto_adj = self.c_params.get_bool("CruiseGapAdjust")
      # self.to_avoid_lkas_fault_enabled = self.c_params.get_bool("AvoidLKASFaultEnabled")
      # self.to_avoid_lkas_fault_max_angle = int(self.c_params.get("AvoidLKASFaultMaxAngle", encoding="utf8"))
      # self.to_avoid_lkas_fault_max_frame = int(self.c_params.get("AvoidLKASFaultMaxFrame", encoding="utf8"))
      # self.e2e_long_enabled = self.c_params.get_bool("E2ELong")
      # self.stopsign_enabled = self.c_params.get_bool("StopAtStopSign")
      # self.gap_by_spd_on = self.c_params.get_bool("CruiseGapBySpdOn")
      # self.experimental_mode = self.c_params.get_bool("ExperimentalMode")
      # self.usf = int(Params().get("UserSpecificFeature", encoding="utf8"))
      if self.c_params.get_bool("KisaLiveTunePanelEnable"):
        if self.car_fingerprint in ANGLE_CONTROL_CAR:
          pass
        elif self.CP.lateralTuning.which() == 'pid':
          self.str_log2 = 'T={:0.2f}/{:0.3f}/{:0.1f}/{:0.5f}'.format(float(Decimal(self.c_params.get("PidKp", encoding="utf8"))*Decimal('0.01')), \
          float(Decimal(self.c_params.get("PidKi", encoding="utf8"))*Decimal('0.001')), float(Decimal(self.c_params.get("PidKd", encoding="utf8"))*Decimal('0.01')), \
          float(Decimal(self.c_params.get("PidKf", encoding="utf8"))*Decimal('0.00001')))
        elif self.CP.lateralTuning.which() == 'indi':
          self.str_log2 = 'T={:03.1f}/{:03.1f}/{:03.1f}/{:03.1f}'.format(float(Decimal(self.c_params.get("InnerLoopGain", encoding="utf8"))*Decimal('0.1')), \
          float(Decimal(self.c_params.get("OuterLoopGain", encoding="utf8"))*Decimal('0.1')), float(Decimal(self.c_params.get("TimeConstant", encoding="utf8"))*Decimal('0.1')), \
          float(Decimal(self.c_params.get("ActuatorEffectiveness", encoding="utf8"))*Decimal('0.1')))
        elif self.CP.lateralTuning.which() == 'lqr':
          self.str_log2 = 'T={:04.0f}/{:05.3f}/{:07.5f}'.format(float(Decimal(self.c_params.get("Scale", encoding="utf8"))*Decimal('1.0')), \
          float(Decimal(self.c_params.get("LqrKi", encoding="utf8"))*Decimal('0.001')), float(Decimal(self.c_params.get("DcGain", encoding="utf8"))*Decimal('0.00001')))
        elif self.CP.lateralTuning.which() == 'torque':
          self.str_log2 = 'T={:0.1f}/{:0.1f}/{:0.1f}/{:0.1f}/{:0.3f}'.format(float(Decimal(self.c_params.get("TorqueMaxLatAccel", encoding="utf8"))*Decimal('0.1')), \
          float(Decimal(self.c_params.get("TorqueKp", encoding="utf8"))*Decimal('0.1')), \
          float(Decimal(self.c_params.get("TorqueKf", encoding="utf8"))*Decimal('0.1')), float(Decimal(self.c_params.get("TorqueKi", encoding="utf8"))*Decimal('0.1')), \
          float(Decimal(self.c_params.get("TorqueFriction", encoding="utf8")) * Decimal('0.001')))
      elif self.CP.lateralTuning.which() == 'torque' and self.live_torque_params:
        torque_params = self.sm['liveTorqueParameters']
        self.str_log2 = 'T={:0.2f}/{:0.2f}/{:0.3f}'.format(torque_params.latAccelFactorFiltered, torque_params.latAccelOffsetFiltered, torque_params.frictionCoefficientFiltered)

    new_actuators = actuators.as_builder()
    new_actuators.steer = apply_steer / self.params.STEER_MAX
    new_actuators.steerOutputCan = apply_steer
    new_actuators.steeringAngleDeg = apply_angle
    new_actuators.accel = self.accel if self.CP.sccBus == 2 else accel

    new_actuators.aqValue = self.aq_value
    new_actuators.aqValueRaw = self.aq_value_raw
    new_actuators.safetySpeed = self.KCC.safetycam_speed
    new_actuators.lkasTemporaryOff = self.lkas_temp_disabled
    new_actuators.gapBySpdOnTemp = (self.gap_by_spd_on_sw_trg and self.gap_by_spd_on)
    new_actuators.expModeTemp = self.experimental_mode_temp
    new_actuators.btnPressing = self.btnsignal if self.btnsignal is not None else 0

    new_actuators.autoResvCruisekph = self.v_cruise_kph_auto_res
    new_actuators.resSpeed = self.res_speed

    new_actuators.kisaLog1 = str_log1 + '  ' + self.str_log2
    new_actuators.kisaLog2 = str_log2

    self.frame += 1
    return new_actuators, can_sends

  def create_button_messages(self, CC: car.CarControl, CS: car.CarState, use_clu11: bool):
    can_sends = []
    if use_clu11:
      if CC.cruiseControl.cancel:
        can_sends.append(hyundaican.create_clu11(self.packer, self.frame, CS.clu11, Buttons.CANCEL, self.CP))
      elif CC.cruiseControl.resume:
        # send resume at a max freq of 10Hz
        if (self.frame - self.last_button_frame) * DT_CTRL > 0.1:
          # send 25 messages at a time to increases the likelihood of resume being accepted
          can_sends.extend([hyundaican.create_clu11(self.packer, self.frame, CS.clu11, Buttons.RES_ACCEL, self.CP)] * self.standstill_res_count)
          if (self.frame - self.last_button_frame) * DT_CTRL >= 0.15:
            self.last_button_frame = self.frame
    else:
      if (self.frame - self.last_button_frame) * DT_CTRL > (randint(1, 3) * 0.1 * self.refresh_time) and CS.acc_active: # 0.25
        resume_on = CS.out.cruiseState.standstill and abs(CS.lead_distance - self.last_lead_distance) >= 0.1 and self.standstill_status_canfd
        standstill = CS.out.cruiseState.standstill and 6 > CS.lead_distance > 0
        if standstill and self.last_lead_distance == 0:
          self.last_lead_distance = CS.lead_distance
          self.standstill_status_canfd = True
        elif resume_on:
          self.refresh_time = randint(1,2)
          if self.CP.flags & HyundaiFlags.CANFD_ALT_BUTTONS:
            # TODO: resume for alt button cars
            pass
          else:
            for _ in range(self.standstill_res_count):
              can_sends.append(hyundaicanfd.create_buttons(self.packer, self.CP, self.CAN, CS.buttons_counter+1, Buttons.RES_ACCEL, CS.cruise_btn_info))
            self.last_button_frame = self.frame
            self.standstill_res_button = True
            self.cruise_gap_adjusting = False
            self.cruise_gap_set_init = False
            self.last_lead_distance = 0
            self.standstill_status_canfd = False
        elif standstill and not self.gap_by_spd_on and self.kisa_cruisegap_auto_adj and 1.0 not in (CS.cruiseGapSet, CS.DistSet):
          if self.cruise_gap_prev == 0 and (CS.DistSet if CS.DistSet > 0 else CS.cruiseGapSet) != 1.0 and not self.cruise_gap_set_init:
            self.cruise_gap_prev = CS.DistSet if CS.DistSet > 0 else CS.cruiseGapSet
            self.cruise_gap_set_init = True
            self.refresh_time = 1
          elif 1.0 not in (CS.cruiseGapSet, CS.DistSet) and self.cruise_gap_set_init:
            for _ in range(self.btn_count):
              can_sends.append(hyundaicanfd.create_buttons(self.packer, self.CP, self.CAN, CS.buttons_counter, Buttons.GAP_DIST, CS.cruise_btn_info))
            self.last_button_frame = self.frame
            self.cruise_gap_adjusting = True
            self.refresh_time = interp((CS.DistSet if CS.DistSet > 0 else CS.cruiseGapSet), [2,3,4], [randint(7, 9) * 0.1, randint(4, 6) * 0.1, randint(1, 3) * 0.1])
        elif self.kisa_variablecruise or self.try_early_stop or self.gap_by_spd_on and not resume_on:
          self.cruise_gap_adjusting = False
          self.cruise_gap_set_init = False
          self.standstill_res_button = False
          btn_signal = self.KCC.update(CS)
          self.btnsignal = btn_signal
          self.on_speed_control = self.KCC.onSpeedControl
          self.on_speed_bump_control = self.KCC.onSpeedBumpControl
          self.curv_speed_control = self.KCC.curvSpeedControl
          self.cut_in_control = self.KCC.cutInControl
          self.driver_scc_set_control = self.KCC.driverSccSetControl
          if btn_signal is not None:
            if btn_signal == 3 and self.KCC.ctrl_gap != (CS.DistSet if CS.DistSet > 0 else CS.cruiseGapSet):
              for _ in range(self.btn_count):
                can_sends.append(hyundaicanfd.create_buttons(self.packer, self.CP, self.CAN, CS.buttons_counter, btn_signal, CS.cruise_btn_info))
              self.last_button_frame = self.frame
              self.cruise_gap_adjusting = True
            elif btn_signal in (1,2) and self.KCC.ctrl_speed != round(CS.VSetDis):
              for _ in range(self.btn_count):
                can_sends.append(hyundaicanfd.create_buttons(self.packer, self.CP, self.CAN, CS.buttons_counter, btn_signal, CS.cruise_btn_info))
              self.last_button_frame = self.frame
              self.cruise_speed_adjusting = True
            self.refresh_time = 0
          elif (self.KCC.ctrl_gap == (CS.DistSet if CS.DistSet > 0 else CS.cruiseGapSet)) or (self.KCC.ctrl_speed == round(CS.VSetDis)):
            if self.KCC.ctrl_gap == (CS.DistSet if CS.DistSet > 0 else CS.cruiseGapSet) and self.cruise_gap_adjusting:
              self.cruise_gap_adjusting = False
              self.last_button_frame = self.frame
              self.refresh_time = 1
            if self.KCC.ctrl_speed == round(CS.VSetDis) and self.cruise_speed_adjusting:
              self.cruise_speed_adjusting = False
              self.last_button_frame = self.frame
              self.refresh_time = 1
        else:
          self.cruise_gap_set_init = False
          self.on_speed_control = False
          self.on_speed_bump_control = False
          self.curv_speed_control = False
          self.cut_in_control = False
          self.driver_scc_set_control = False
          self.cruise_gap_adjusting = False
          self.cruise_speed_adjusting = False
          self.standstill_res_button = False
          self.auto_res_starting = False
          self.btnsignal = 0
      elif (self.frame - self.last_button_frame) * DT_CTRL > 1.0 and not CS.acc_active:
        self.last_button_frame = self.frame
        self.on_speed_control = False
        self.on_speed_bump_control = False
        self.curv_speed_control = False
        self.cut_in_control = False
        self.driver_scc_set_control = False
        self.cruise_gap_adjusting = False
        self.cruise_speed_adjusting = False
        self.standstill_res_button = False
        self.auto_res_starting = False
        self.btnsignal = 0

    return can_sends
