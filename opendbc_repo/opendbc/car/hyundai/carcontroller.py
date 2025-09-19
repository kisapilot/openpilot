import numpy as np
from opendbc.can import CANPacker
from opendbc.car import Bus, DT_CTRL, make_tester_present_msg, structs
from opendbc.car.lateral import apply_driver_steer_torque_limits, common_fault_avoidance, apply_std_steer_angle_limits
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.hyundai import hyundaicanfd, hyundaican
from opendbc.car.hyundai.hyundaicanfd import CanBus
from opendbc.car.hyundai.values import HyundaiFlags, Buttons, CarControllerParams, CAR
from opendbc.car.interfaces import CarControllerBase

from cereal import car, messaging
from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
from openpilot.selfdrive.controls.lib.desire_helper import LANE_CHANGE_SPEED_MIN
from opendbc.car.hyundai.kisa_cruise_control  import KisaCruiseControl

from openpilot.common.params import Params
from random import randint

GearShifter = structs.CarState.GearShifter
VisualAlert = structs.CarControl.HUDControl.VisualAlert
LongCtrlState = structs.CarControl.Actuators.LongControlState

# EPS faults if you apply torque while the steering angle is above 90 degrees for more than 1 second
# All slightly below EPS thresholds to avoid fault
MAX_ANGLE = Params().get("AvoidLKASFaultMaxAngle", return_default=True) # 85
MAX_ANGLE_FRAMES = Params().get("AvoidLKASFaultMaxFrame", return_default=True) # 89
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
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.CAN = CanBus(CP)
    self.params = CarControllerParams(CP)
    self.packer = CANPacker(dbc_names[Bus.pt])
    self.angle_limit_counter = 0

    self.accel_last = 0
    self.apply_torque_last = 0
    self.car_fingerprint = CP.carFingerprint
    self.last_button_frame = 0
    self.last_button_frame2 = 0

    self.apply_angle_now = 0
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
    self.mode_change_switch = self.c_params.get("CruiseStatemodeSelInit", return_default=True)
    self.kisa_variablecruise = self.c_params.get_bool("KisaVariableCruise")
    self.kisa_autoresume = self.c_params.get_bool("KisaAutoResume")
    self.kisa_cruisegap_auto_adj = self.c_params.get_bool("CruiseGapAdjust")
    self.kisa_cruise_auto_res = self.c_params.get_bool("CruiseAutoRes")
    self.kisa_cruise_auto_res_option = self.c_params.get("AutoResOption", return_default=True)
    self.kisa_cruise_auto_res_condition = self.c_params.get("AutoResCondition", return_default=True)

    self.kisa_turnsteeringdisable = self.c_params.get_bool("KisaTurnSteeringDisable")
    self.kisa_maxanglelimit = float(self.c_params.get("KisaMaxAngleLimit", return_default=True))
    self.ufc_mode_enabled = self.c_params.get_bool("UFCModeEnabled")
    self.ldws_fix = self.c_params.get_bool("LdwsCarFix")
    self.radar_helper_option = self.c_params.get("RadarLongHelper", return_default=True)
    self.stopping_dist_adj_enabled = self.c_params.get_bool("StoppingDistAdj")
    self.standstill_resume_alt = self.c_params.get_bool("StandstillResumeAlt")
    self.auto_res_delay = self.c_params.get("AutoRESDelay", return_default=True) * 100
    self.auto_res_delay_timer = 0
    self.stopped = False
    self.stoppingdist = self.c_params.get("StoppingDist", return_default=True) * 0.1

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
    self.standstill_res_count = self.c_params.get("RESCountatStandstill", return_default=True)

    self.standstill_status = False
    self.standstill_status_canfd = False
    self.standstill_status_timer = 0
    self.switch_timer = 0
    self.switch_timer2 = 0
    self.auto_res_timer = 0
    self.auto_res_limit_timer = 0
    self.auto_res_limit_sec = self.c_params.get("AutoResLimitTime", return_default=True) * 100
    self.auto_res_starting = False
    self.res_speed = 0
    self.res_speed_timer = 0
    self.autohold_popup_timer = 0
    self.autohold_popup_switch = False

    self.model_speed = 255.0

    self.osm_spdlimit_enabled = self.c_params.get_bool("OSMSpeedLimitEnable")
    self.stock_safety_decel_enabled = self.c_params.get_bool("UseStockDecelOnSS")

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
    self.to_avoid_lkas_fault_max_angle = self.c_params.get("AvoidLKASFaultMaxAngle", return_default=True)
    self.to_avoid_lkas_fault_max_frame = self.c_params.get("AvoidLKASFaultMaxFrame", return_default=True)
    self.no_mdps_mods = self.c_params.get_bool("NoSmartMDPS")

    self.user_specific_feature = self.c_params.get("UserSpecificFeature", return_default=True)

    self.gap_by_spd_on = self.c_params.get_bool("CruiseGapBySpdOn")
    self.gap_by_spd_spd = list(map(int, self.c_params.get("CruiseGapBySpdSpd", return_default=True).split(',')))
    self.gap_by_spd_gap = list(map(int, self.c_params.get("CruiseGapBySpdGap", return_default=True).split(',')))
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
    self.road_limit_spd_enabled = self.c_params.get_bool("CruiseSetwithRoadLimitSpeedEnabled")
    self.road_spd_on_sw = False
    self.road_spd_on_sw_trg = True
    self.road_spd_on_sw_cnt = 0
    self.road_spd_on_sw_cnt2 = 0

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
    self.standstill_manual_start = False
    self.standstill_manual_start_cnt = 0

    self.alpha_long_enabled = self.c_params.get_bool("AlphaLongitudinalEnabled")
    self.experimental_mode = self.c_params.get_bool("ExperimentalMode")
    self.live_torque_params = self.c_params.get_bool("KisaLiveTorque")
    self.gapsettingdance = self.c_params.get("KisaCruiseGapSet", return_default=True)
    self.prev_gapButton = 0

    self.kisa_long_alt = True if self.c_params.get("KISALongAlt", return_default=True) in (1, 2) else False

    self.btnsignal = 0
    self.nt_interval = self.c_params.get("KISACruiseSpammingInterval", return_default=True)
    self.btn_count = self.c_params.get("KISACruiseSpammingBtnCount", return_default=True)
    self.second2 = 0
    self.pause_time = 0
    self.gap_now = 0
    self.gap_prev = 0
    self.cruise_set_now = 0
    self.cruise_set_prev = 0

    self.experimental_mode_temp = False
    self.exp_mode_push = False
    self.exp_mode_push_cnt = 0

    self.counter_init = False
    self.radarDisableOverlapTimer = 0

    self.e2e_x = 0
    self.l_stat = 0

    self.refresh_time = 0.25
    self.refresh_count = 0
    self.refresh_time2 = 0.25
    self.acc_activated = False
    self.refresh_time3 = 10
    self.btn_reset = False
    self.lfa_init = False

    self.regenbrake = self.c_params.get_bool("RegenBrakeFeatureOn")
    rgn_option_list = list(self.c_params.get("RegenBrakeFeature", return_default=True))
    self.regen_stop = True if '1' in rgn_option_list and self.regenbrake else False
    self.regen_dist = True if '2' in rgn_option_list and self.regenbrake else False
    self.regen_e2e = True if '3' in rgn_option_list and self.regenbrake else False
    self.regen_stop_activated = False
    self.regen_stop_pre_activated = False
    self.regen_stop_timer = 0

    # self.usf = 0

    self.str_log1 = ''
    self.str_log2 = ''
    self.str_log3 = ''

    if self.CP.isAngleControl:
      pass
    else:
      if CP.lateralTuning.which() == 'pid':
        self.str_log3 = 'T={:0.2f}/{:0.3f}/{:0.5f}/{:0.2f}'.format(CP.lateralTuning.pid.kpV[1], CP.lateralTuning.pid.kiV[1], CP.lateralTuning.pid.kf, CP.lateralTuning.pid.kd)
      elif CP.lateralTuning.which() == 'indi':
        self.str_log3 = 'T={:03.1f}/{:03.1f}/{:03.1f}/{:03.1f}'.format(CP.lateralTuning.indi.innerLoopGainV[0], CP.lateralTuning.indi.outerLoopGainV[0], \
        CP.lateralTuning.indi.timeConstantV[0], CP.lateralTuning.indi.actuatorEffectivenessV[0])
      elif CP.lateralTuning.which() == 'lqr':
        self.str_log3 = 'T={:04.0f}/{:05.3f}/{:07.5f}'.format(CP.lateralTuning.lqr.scale, CP.lateralTuning.lqr.ki, CP.lateralTuning.lqr.dcGain)
      elif CP.lateralTuning.which() == 'torque':
        self.str_log3 = 'T={:0.2f}/{:0.2f}/{:0.2f}/{:0.3f}'.format(CP.lateralTuning.torque.kp, CP.lateralTuning.torque.kf, CP.lateralTuning.torque.ki, CP.lateralTuning.torque.friction)

    self.sm = messaging.SubMaster(['controlsState', 'selfdriveState', 'radarState', 'lateralPlan', 'longitudinalPlan', 'liveTorqueParameters', 'carState', 'liveENaviData', 'liveMapData'])


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

    if abs(CS.out.steeringTorque) > 170 and CS.out.vEgo < LANE_CHANGE_SPEED_MIN and not self.CP.isCanFD:
      self.driver_steering_torque_above_timer -= 1
      if self.driver_steering_torque_above_timer <= 0:
        self.driver_steering_torque_above_timer = 0
    else:
      self.driver_steering_torque_above_timer += 10
      if self.driver_steering_torque_above_timer >= 150:
        self.driver_steering_torque_above_timer = 150

    # steering torque
    if self.CP.smoothSteer.method == 1:
      new_torque = int(round(actuators.torque * self.params.STEER_MAX))
      new_torque = self.smooth_steer(new_torque, CS)
    elif 0 <= self.driver_steering_torque_above_timer < 150 and not self.user_specific_feature == 60:
      new_torque = int(round(actuators.torque * self.params.STEER_MAX * (self.driver_steering_torque_above_timer / 150)))
    else:
      new_torque = int(round(actuators.torque * self.params.STEER_MAX))

    apply_torque = apply_driver_steer_torque_limits(new_torque, self.apply_torque_last, CS.out.steeringTorque, self.params)

    if self.kisa_maxanglelimit == 90:
      lat_active = CC.latActive and abs(CS.out.steeringAngleDeg) < self.kisa_maxanglelimit and (CS.out.gearShifter == GearShifter.drive or self.user_specific_feature == 11)
    elif self.kisa_maxanglelimit > 90:
      str_angle_limit = np.interp(CS.out.vEgo * CV.MS_TO_KPH, [0, 20], [self.kisa_maxanglelimit+60, self.kisa_maxanglelimit])
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
    if self.to_avoid_lkas_fault_enabled or self.CP.isCanFD:
      self.angle_limit_counter, apply_steer_req = common_fault_avoidance(abs(CS.out.steeringAngleDeg) >= self.to_avoid_lkas_fault_max_angle, lat_active,
                                                                         self.angle_limit_counter, self.to_avoid_lkas_fault_max_frame,
                                                                         MAX_ANGLE_CONSECUTIVE_FRAMES)
      apply_angle = apply_std_steer_angle_limits(actuators.steeringAngleDeg, self.apply_angle_last, CS.out.vEgoRaw, CS.out.steeringAngleDeg, lat_active, CarControllerParams.ANGLE_LIMITS)
      self.apply_angle_now = apply_angle

      lkas_max_torque = CarControllerParams.LKAS_MAX_TORQUE
      if abs(CS.out.steeringTorque) > 200:
        angle_above_timer_step = int(np.interp(self.lkas_max_torque, [150, 250], [1, 10]))
        self.driver_steering_angle_above_timer -= angle_above_timer_step
        if self.driver_steering_angle_above_timer <= 20:
          self.driver_steering_angle_above_timer = 20
      else:
        angle_above_timer_step2 = int(np.interp(self.lkas_max_torque, [30, 250], [10, 1]))
        self.driver_steering_angle_above_timer += angle_above_timer_step2 if not CS.wheel_touched else +1
        if self.driver_steering_angle_above_timer >= 150:
          self.driver_steering_angle_above_timer = 150

      curv_weight = 1.0 if (CS.out.vEgo * CV.MS_TO_KPH < 40 or CS.out.leftBlinker or CS.out.rightBlinker) else np.interp(self.model_speed, [50, 100, 255], [1.5, 1.0, 0.5])
      ego_weight = min(1.0, np.interp(CS.out.vEgo * CV.MS_TO_KPH, [0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100], [0.2, 0.25, 0.32, 0.45, 0.6, 0.7, 0.8, 0.9, 1.0, 1.0, 1.0]))
      ego_weight = min(1.0, ego_weight * curv_weight)

      if 0 <= self.driver_steering_angle_above_timer < 150:
        self.lkas_max_torque = int(round(lkas_max_torque * (self.driver_steering_angle_above_timer / 150) * ego_weight))
      else:
        self.lkas_max_torque = lkas_max_torque * ego_weight

      # Hold torque with induced temporary fault when cutting the actuation bit
      torque_fault = lat_active and not apply_steer_req

      self.apply_angle_last = apply_angle
    else:
      torque_fault = False

    if not lat_active:
      apply_angle = CS.out.steeringAngleDeg
      apply_torque = 0
      self.lkas_max_torque = 0
      self.apply_angle_last = apply_angle

    self.apply_torque_last = apply_torque

    # accel + longitudinal
    accel = float(np.clip(actuators.accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))
    stopping = actuators.longControlState == LongCtrlState.stopping
    set_speed_in_units = hud_control.setSpeed * (CV.MS_TO_KPH if CS.is_metric else CV.MS_TO_MPH)

    can_sends = []

    # *** common hyundai stuff ***

    # tester present - w/ no response (keeps relevant ECU disabled)
    if self.frame % 100 == 0 and not (self.CP.flags & HyundaiFlags.CANFD_CAMERA_SCC) and self.CP.openpilotLongitudinalControl and self.alpha_long_enabled:
      # for longitudinal control, either radar or ADAS driving ECU
      addr, bus = 0x7d0, self.CAN.ECAN if self.CP.flags & HyundaiFlags.CANFD else 0
      if self.CP.flags & HyundaiFlags.CANFD_LKA_STEERING.value:
        addr, bus = 0x730, self.CAN.ECAN
      can_sends.append(make_tester_present_msg(addr, bus, suppress_response=True))

      # for blinkers
      if self.CP.flags & HyundaiFlags.ENABLE_BLINKERS:
        can_sends.append(make_tester_present_msg(0x7b1, self.CAN.ECAN, suppress_response=True))

    # *** CAN/CAN FD common msgs ***
    self.create_common_msgs(CS, CC, lat_active, stopping, set_speed_in_units, new_torque)
    # *** CAN/CAN FD specific ***
    if self.CP.flags & HyundaiFlags.CANFD:
      can_sends.extend(self.create_canfd_msgs(lat_active, apply_torque, set_speed_in_units, accel,
                                              stopping, hud_control, CS, CC, apply_angle, self.lkas_max_torque))
    else:
      can_sends.extend(self.create_can_msgs(lat_active, apply_torque, torque_fault, set_speed_in_units, accel,
                                            stopping, hud_control, actuators, CS, CC))

    new_actuators = actuators.as_builder()
    new_actuators.torque = apply_torque / self.params.STEER_MAX
    new_actuators.torqueOutputCan = apply_torque
    new_actuators.accel = self.accel if self.CP.sccBus == 2 else accel

    new_actuators.steeringAngleDeg = apply_angle if self.to_avoid_lkas_fault_enabled or self.CP.isCanFD or not lat_active else 0
    new_actuators.aqValue = self.aq_value
    new_actuators.aqValueRaw = self.aq_value_raw
    new_actuators.safetySpeed = self.KCC.safetycam_speed
    new_actuators.lkasTemporaryOff = self.lkas_temp_disabled
    new_actuators.gapBySpdOnTemp = (self.gap_by_spd_on_sw_trg and self.gap_by_spd_on)
    new_actuators.expModeTemp = self.experimental_mode_temp
    new_actuators.btnPressing = self.btnsignal if self.btnsignal is not None and self.refresh_time < 1.0 else 0

    new_actuators.autoResvCruisekph = self.v_cruise_kph_auto_res
    new_actuators.resSpeed = self.res_speed
    new_actuators.setLoadspeedTempStop = (self.road_spd_on_sw_trg and self.road_limit_spd_enabled)

    new_actuators.kisaLog1 = self.str_log1 + '  ' + self.str_log3
    new_actuators.kisaLog2 = self.str_log2

    new_actuators.needBrake = self.need_brake #bool
    new_actuators.lkasTempDisabled = self.lkas_temp_disabled #bool
    new_actuators.lanechangeManualTimer = self.lanechange_manual_timer #int
    new_actuators.emergencyManualTimer = self.emergency_manual_timer #int
    new_actuators.standstillResButton = self.standstill_res_button #bool
    new_actuators.cruiseGapAdjusting = self.cruise_gap_adjusting #bool
    new_actuators.onSpeedBumpControl = self.on_speed_bump_control #bool
    new_actuators.onSpeedControl = self.on_speed_control #bool
    new_actuators.curvSpeedControl = self.curv_speed_control #bool
    new_actuators.cutInControl = self.cut_in_control #bool
    new_actuators.driverSccSetControl = self.driver_scc_set_control #bool
    new_actuators.autoholdPopupTimer = self.autohold_popup_timer #int
    new_actuators.autoResStarting = self.auto_res_starting #bool
    new_actuators.e2eStandstill = self.e2e_standstill or self.standstill_manual_start #bool
    new_actuators.modeChangeTimer = self.mode_change_timer #int
    new_actuators.lkasTempDisabledTimer = self.lkas_temp_disabled_timer #int
    new_actuators.standStill = True if CS.out.cruiseState.standstill or (self.standstill_status or self.standstill_status_canfd) else False

    if self.standstill_manual_start:
      self.standstill_manual_start = False

    self.frame += 1
    return new_actuators, can_sends

  def create_can_msgs(self, apply_steer_req, apply_torque, torque_fault, set_speed_in_units, accel, stopping, hud_control, actuators, CS, CC):
    can_sends = []

    # HUD messages
    sys_warning, sys_state, left_lane_warning, right_lane_warning = process_hud_alert(CC.enabled, self.car_fingerprint,
                                                                                      hud_control)

    can_sends.append(hyundaican.create_lkas11(self.packer, self.frame, self.CP, apply_torque, apply_steer_req and not self.lkas_temp_disabled,
                                              torque_fault, CS.lkas11, sys_warning, sys_state, CC.enabled,
                                              hud_control.leftLaneVisible, hud_control.rightLaneVisible,
                                              left_lane_warning, right_lane_warning, self.ldws_fix))

    if CS.out.cruiseState.standstill:
      self.standstill_status = True
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
            can_sends.append(hyundaican.create_clu11(self.packer, self.resume_cnt, CS.clu11, Buttons.RES_ACCEL, self.CP))
            self.resume_cnt += 1
            if self.resume_cnt >= int(randint(4, 5) * 2):
              self.resume_cnt = 0
              self.switch_timer = int(randint(20, 25) * 2)
          else:
            if (self.frame - self.last_resume_frame) * DT_CTRL > 0.1:
              self.standstill_res_button = True
              # send 25 messages at a time to increases the likelihood of resume being accepted, value 25 is not acceptable at some cars.
              can_sends.extend([hyundaican.create_clu11(self.packer, self.frame, CS.clu11, Buttons.RES_ACCEL, self.CP)] * self.standstill_res_count)
              self.last_resume_frame = self.frame
          self.standstill_fault_reduce_timer += 1
        # gap save after 1sec
        elif 100 < self.standstill_fault_reduce_timer and self.cruise_gap_prev == 0 and CS.cruiseGapSet != 1.0 and self.kisa_autoresume and self.kisa_cruisegap_auto_adj and not self.gap_by_spd_on:
          self.cruise_gap_prev = CS.cruiseGapSet
          self.cruise_gap_set_init = True
        # gap adjust to 1 for fast start
        elif 110 < self.standstill_fault_reduce_timer and CS.cruiseGapSet != 1.0 and self.kisa_autoresume and self.kisa_cruisegap_auto_adj and not self.gap_by_spd_on:
          can_sends.append(hyundaican.create_clu11(self.packer, self.frame, CS.clu11, Buttons.GAP_DIST, self.CP))
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
      btn_signal = self.KCC.update(CS, self.gap_by_spd_on_sw_trg, sm=self.sm)
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
          can_sends.append(hyundaican.create_clu11(self.packer, self.frame, CS.clu11, Buttons.GAP_DIST, self.CP))
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
              can_sends.append(hyundaican.create_clu11(self.packer, self.frame, CS.clu11, Buttons.GAP_DIST, self.CP))
              self.resume_cnt += 1
              if self.resume_cnt >= int(randint(4, 5) * 2):
                self.resume_cnt = 0
                self.switch_timer = int(randint(20, 25) * 2)
          elif btn_signal != None:
            if self.switch_timer > 0:
              self.switch_timer -= 1
            else:
              btn_count = 1
              btn_count = int(np.interp(self.KCC.t_interval, [self.nt_interval+3,70],[1,self.btn_count])) if not CS.is_metric else int(np.interp(self.KCC.t_interval, [self.nt_interval,40],[1,self.btn_count]))
              can_sends.extend([hyundaican.create_clu11(self.packer, self.resume_cnt, CS.clu11, btn_signal, self.CP)] * btn_count)
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
              can_sends.append(hyundaican.create_clu11(self.packer, self.frame, CS.clu11, Buttons.GAP_DIST, self.CP))
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
              can_sends.append(hyundaican.create_clu11(self.packer, self.frame, CS.clu11, Buttons.GAP_DIST, self.CP))
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
            can_sends.append(hyundaican.create_clu11(self.packer, self.frame, CS.clu11, Buttons.GAP_DIST, self.CP))
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
            can_sends.append(hyundaican.create_clu11(self.packer, self.frame, CS.clu11, Buttons.GAP_DIST, self.CP))
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
            can_sends.append(hyundaican.create_clu11(self.packer, self.frame, CS.clu11, Buttons.GAP_DIST, self.CP))
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
            can_sends.append(hyundaican.create_clu11(self.packer, self.frame, CS.clu11, Buttons.GAP_DIST, self.CP))
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
              btn_count = int(np.interp(self.KCC.t_interval, [self.nt_interval+3,70],[1,self.btn_count])) if not CS.is_metric else int(np.interp(self.KCC.t_interval, [self.nt_interval,40],[1,self.btn_count]))
              can_sends.extend([hyundaican.create_clu11(self.packer, self.resume_cnt, CS.clu11, btn_signal, self.CP)] * btn_count)
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
              can_sends.append(hyundaican.create_clu11(self.packer, self.frame, CS.clu11, Buttons.GAP_DIST, self.CP))
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
    t_speed = 20 if not CS.is_metric else 30
    if self.auto_res_timer > 0:
      self.auto_res_timer -= 1
    elif self.model_speed > (60 if not CS.is_metric else 95) and self.cancel_counter == 0 and not CS.acc_active and not CS.out.brakeLights and round(CS.VSetDis) >= t_speed and \
     (1 < CS.lead_distance < 149 or round(CS.clu_Vanz) > t_speed) and round(CS.clu_Vanz) >= 3 and self.cruise_init and \
     self.kisa_cruise_auto_res and kisa_cruise_auto_res_condition and (self.auto_res_limit_sec == 0 or self.auto_res_limit_timer < self.auto_res_limit_sec) and \
     (self.auto_res_delay == 0 or self.auto_res_delay_timer >= self.auto_res_delay):
      if self.kisa_cruise_auto_res_option == 0:
        can_sends.append(hyundaican.create_clu11(self.packer, self.frame, CS.clu11, Buttons.RES_ACCEL, self.CP))
        self.auto_res_starting = True
        self.res_speed = round(CS.VSetDis) if not CS.is_metric or self.osm_spdlimit_enabled else round(CS.clu_Vanz*1.1)
        self.res_speed_timer = 300
        self.resume_cnt += 1
        if self.resume_cnt >= int(randint(4, 5) * 2):
          self.resume_cnt = 0
          self.auto_res_timer = int(randint(20, 25) * 2)
      elif self.kisa_cruise_auto_res_option == 1:
        can_sends.append(hyundaican.create_clu11(self.packer, self.frame, CS.clu11, Buttons.SET_DECEL, self.CP)) # auto res but set_decel to set current speed
        self.auto_res_starting = True
        self.v_cruise_kph_auto_res = round(CS.clu_Vanz)
        self.res_speed_timer = 50
        self.resume_cnt += 1
        if self.resume_cnt >= int(randint(4, 5) * 2):
          self.resume_cnt = 0
          self.auto_res_timer = int(randint(20, 25) * 2)
      elif self.kisa_cruise_auto_res_option == 2:
        can_sends.append(hyundaican.create_clu11(self.packer, self.frame, CS.clu11, Buttons.RES_ACCEL, self.CP)) if 1 < CS.lead_distance < 149 \
        else can_sends.append(hyundaican.create_clu11(self.packer, self.frame, CS.clu11, Buttons.SET_DECEL, self.CP))
        self.auto_res_starting = True
        self.v_cruise_kph_auto_res = round(CS.clu_Vanz)
        self.res_speed_timer = 50
        self.resume_cnt += 1
        if self.resume_cnt >= int(randint(4, 5) * 2):
          self.resume_cnt = 0
          self.auto_res_timer = int(randint(20, 25) * 2)

    if self.CP.openpilotLongitudinalControl and self.alpha_long_enabled and self.CP.sccBus != 2:
      if self.prev_gapButton != CS.cruise_buttons[-1]:  # gap change.
        if CS.cruise_buttons[-1] == 3:
          self.gapsettingdance -= 1
        if self.gapsettingdance < 1:
          self.gapsettingdance = 4
        self.prev_gapButton = CS.cruise_buttons[-1]

    if self.CP.sccBus == 2 and self.longcontrol and self.kisa_long_alt:
      if self.frame % 2 == 0:
        lead_objspd = CS.lead_objspd  # vRel (km/h)
        aReqValue = CS.scc12["aReqValue"]
        faccel = actuators.accel if CC.longActive and not CS.out.gasPressed else 0
        accel = actuators.oaccel if CC.longActive and not CS.out.gasPressed else 0
        radar_recog = (0 < CS.lead_distance <= 149)
        if self.radar_helper_option == 0: # Vision Only
          if 0 < CS.lead_distance <= self.stoppingdist:
            stock_weight = np.interp(CS.lead_distance, [2.0, self.stoppingdist], [1., 0.])
            accel = accel * (1. - stock_weight) + aReqValue * stock_weight
            self.l_stat = 1
          elif 0.1 < self.dRel < (self.stoppingdist + 2.0) and self.vRel < 0:
            accel = self.accel - (DT_CTRL * np.interp(CS.out.vEgo, [1.0, 3.0], [0.5, 2.0]))
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
              accel = np.interp(CS.clu_Vanz, [0, round(CS.VSetDis)], [min(accel*0.6, faccel*0.6), aReqValue])
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
              accel = np.interp(aReqValue, [0.0, 1.8], [0.0, -0.7])
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
                  accel = np.interp(lead_objspd, [-1, 0, 5], [aReqValue, aReqValue, accel])
                  self.l_stat = 17
                else:
                  accel = np.interp(self.dRel, [0, 40], [accel*0.1, accel*0.7])
                  self.l_stat = 18
              else:
                if aReqValue < accel:
                  accel = np.interp(CS.lead_distance, [5.0, 13.0, 20.0], [aReqValue, accel*0.8+aReqValue*0.2, aReqValue])
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
                accel = self.accel - (DT_CTRL * np.interp(CS.out.vEgo, [0.0, 1.0, 2.0], [0.02, 0.8, 4.5]))
                self.l_stat = 23
            elif aReqValue < 0.0:
              dRel2 = self.dRel if self.dRel > 0 else CS.lead_distance
              dist_by_drel = np.interp(CS.lead_distance, [10, 50], [3.0, 9.0])
              d_ratio = np.interp(CS.clu_Vanz, [40, 110], [0.3, 0.19])
              d_ratio2 = np.interp(CS.clu_Vanz, [40, 110], [1.0, 2.0])
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
                stock_weight = np.interp(self.ed_rd_diff_on_timer, [0, self.ed_rd_diff_on_timer2], [0.1, 1.0])
                self.ed_rd_diff_on_timer -= 1
                self.l_stat = 26
                if aReqValue <= accel:
                  stock_weight = 1.0
                  self.l_stat = 27
              else:
                accel = np.interp(CS.clu_Vanz, [4.0, 10.0, 30.0], [max(accel, faccel), ((min(accel, faccel)*0.6)+(max(accel, faccel)*0.4)), min(accel, faccel)])
                if not self.KCC.cutInControl:
                  self.ed_rd_diff_on = False
                  self.l_stat = 28
                self.ed_rd_diff_on_timer = 0
                self.ed_rd_diff_on_timer2 = 0
                stock_weight = np.interp(abs(lead_objspd), [1.0, 5.0, 10.0, 20.0, 50.0], [0.15, 0.4, 1.0, 0.9, 0.2])
                self.l_stat = 29
                if aReqValue <= accel:
                  self.vrel_delta_timer = 0
                  self.vrel_delta_timer3 = 0
                  stock_weight = min(1.0, np.interp(CS.out.vEgo, [8.0, 30.0], [stock_weight, stock_weight*5.0]))
                  self.l_stat = 30
                  if not self.stopping_dist_adj_enabled:
                    stock_weight = min(1.0, np.interp(CS.lead_distance, [0.0, 10.0], [stock_weight*5.0, stock_weight]))
                    self.l_stat = 31
                elif aReqValue > accel:
                  if self.vrel_delta < -5 and self.vrel_delta_timer == 0:
                    self.vrel_delta_timer = min(400, int(self.dRel*10))
                    self.vrel_delta_timer3 = min(400, int(self.dRel*10))
                    stock_weight = 1.0
                    self.l_stat = 32
                  elif self.vrel_delta_timer > 0:
                    self.vrel_delta_timer -= 1
                    stock_weight = np.interp(self.vrel_delta_timer, [0, self.vrel_delta_timer3], [0.1, 1.0])
                    self.l_stat = 33
                  else:
                    self.vrel_delta_timer = 0
                    self.vrel_delta_timer3 = 0
                    stock_weight = np.interp(abs(lead_objspd), [1.0, 15.0], [1.0, 0.1])
                    self.l_stat = 34
              accel = accel * (1.0 - stock_weight) + aReqValue * stock_weight
              accel = min(accel, -0.5) if CS.lead_distance <= self.stoppingdist+0.5 and not CS.out.standstill else accel
            # elif aReqValue < 0.0:
            #   stock_weight = np.interp(CS.lead_distance, [6.0, 10.0, 18.0, 25.0, 32.0], [1.0, 0.85, 1.0, 0.4, 1.0])
            #   accel = accel * (1.0 - stock_weight) + aReqValue * stock_weight
            else:
              stock_weight = 0.0
              self.change_accel_fast = False
              accel = accel * (1.0 - stock_weight) + aReqValue * stock_weight
              self.l_stat = 35
          elif 0.1 < self.dRel < (self.stoppingdist + 1.5) and int(self.vRel*3.6) < 0:
            accel = self.accel - (DT_CTRL * np.interp(CS.out.vEgo, [0.0, 1.0, 2.0], [0.04, 0.6, 1.1]))
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
            ddrel_weight = np.interp(self.dRel, [self.stoppingdist+1.5, 30], [1.0, 1.0])
            accel = faccel*ddrel_weight
            self.l_stat = 39
          else:
            self.stopped = False
            if stopping:
              self.smooth_start = True
              accel = min(-0.5, accel, faccel*0.5)
              self.l_stat = 40
            elif self.smooth_start and CS.clu_Vanz < round(CS.VSetDis)*0.9:
              accel = np.interp(CS.clu_Vanz, [0, round(CS.VSetDis)], [min(accel*0.6, faccel*0.6), aReqValue])
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
        accel = float(np.clip(accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))
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

    if self.frame % 2 == 0 and self.CP.openpilotLongitudinalControl and self.alpha_long_enabled and self.CP.sccBus != 2:
      # TODO: unclear if this is needed
      jerk = 3.0 if actuators.longControlState == LongCtrlState.pid else 1.0
      use_fca = self.CP.flags & HyundaiFlags.USE_FCA.value
      can_sends.extend(hyundaican.create_acc_commands(self.packer, CC.enabled, accel, jerk, int(self.frame / 2),
                                                      hud_control, set_speed_in_units, stopping,
                                                      CC.cruiseControl.override, use_fca, self.CP, self.gapsettingdance))

    # 20 Hz LFA MFA message
    if self.frame % 5 == 0 and self.CP.flags & HyundaiFlags.SEND_LFA.value:
      can_sends.append(hyundaican.create_lfahda_mfc(self.packer, CC.enabled))

    # 5 Hz ACC options
    if self.frame % 20 == 0 and self.CP.openpilotLongitudinalControl and self.CP.sccBus != 2:
      can_sends.extend(hyundaican.create_acc_opt(self.packer, self.CP))

    # 2 Hz front radar options
    if self.frame % 50 == 0 and self.CP.openpilotLongitudinalControl and self.CP.sccBus != 2:
      can_sends.append(hyundaican.create_frt_radar_opt(self.packer))

    return can_sends

  def create_canfd_msgs(self, apply_steer_req, apply_torque, set_speed_in_units, accel, stopping, hud_control, CS, CC, apply_angle, lkas_max_torque):
    can_sends = []

    lka_steering = self.CP.flags & HyundaiFlags.CANFD_LKA_STEERING
    lka_steering_long = lka_steering and self.CP.openpilotLongitudinalControl

    # lfa init
    # if not self.lfa_init:
    #   self.lfa_init = True
    #   for _ in range(22):
    #     can_sends.append(hyundaicanfd.create_buttons(self.packer, self.CP, self.CAN, CS, 0, False, True))

    # steering control
    can_sends.extend(hyundaicanfd.create_steering_messages(self.packer, self.CP, self.CAN, CC.enabled, apply_steer_req, apply_torque,
                                                            apply_angle, lkas_max_torque, self.frame, CS.adrv_160, CS.adrv_1ea, CS.lfa_alt_info, CS.mdps_info, CS.lfa_info, CS.csw_info, CS.ccnc_161, CS.lfa_hda_info))

    # prevent LFA from activating on LKA steering cars by sending "no lane lines detected" to ADAS ECU
    if self.frame % 5 == 0 and lka_steering:
      can_sends.append(hyundaicanfd.create_suppress_lfa(self.packer, self.CAN, CS.lfa_block_msg,
                                                        self.CP.flags & HyundaiFlags.CANFD_LKA_STEERING_ALT, CC.enabled))

    # LFA and HDA icons
    if self.frame % 5 == 0 and (not lka_steering or lka_steering_long) and not self.CP.adrvControl:
      can_sends.append(hyundaicanfd.create_lfahda_cluster(self.packer, self.CAN, CC.enabled))

    if self.CP.adrvControl:
      if self.frame % 5 == 0:
        can_sends.extend(hyundaicanfd.create_ccnc(self.packer, self.CAN, self.frame, CC.enabled, apply_steer_req, CS.ccnc_161, CS.ccnc_162, CS.adrv_1ea))

    # blinkers
    if lka_steering and self.CP.flags & HyundaiFlags.ENABLE_BLINKERS:
      can_sends.extend(hyundaicanfd.create_spas_messages(self.packer, self.CAN, CC.leftBlinker, CC.rightBlinker))

    if self.CP.openpilotLongitudinalControl:
      if lka_steering:
        can_sends.extend(hyundaicanfd.create_adrv_messages(self.packer, self.CAN, self.frame))
      else:
        can_sends.extend(hyundaicanfd.create_fca_warning_light(self.packer, self.CAN, self.frame))
      if self.frame % 2 == 0:
        can_sends.append(hyundaicanfd.create_acc_control(self.packer, self.CAN, CC.enabled, self.accel_last, accel, stopping, CC.cruiseControl.override,
                                                         set_speed_in_units, hud_control))
        self.accel_last = accel
    else:
      # button presses
      if (self.frame - self.last_button_frame) * DT_CTRL > self.refresh_time and CS.acc_active: # 0.25
        if not self.acc_activated:
          self.acc_activated = True
          self.standstill_status_canfd = False
        resume_on = CS.out.cruiseState.standstill and abs(CS.lead_distance - self.last_lead_distance) >= 0.1 and self.standstill_status_canfd
        standstill = CS.out.cruiseState.standstill and 10.0 > CS.lead_distance > 0 and CS.out.vEgo <= 0.3
        if standstill and (self.last_lead_distance == 0 or self.last_lead_distance > CS.lead_distance):
          self.last_lead_distance = CS.lead_distance
          self.standstill_status_canfd = True
          self.refresh_time = 0
        elif resume_on:
          self.standstill_manual_start_cnt += 1
          if self.standstill_manual_start_cnt > 4:
            self.standstill_manual_start_cnt = 0
            self.standstill_manual_start = True
          self.refresh_time = randint(self.nt_interval, self.nt_interval+2) * 0.01
          if self.CP.flags & HyundaiFlags.CANFD_ALT_BUTTONS:
            # TODO: resume for alt button cars
            pass
          elif self.standstill_res_button:
            self.standstill_res_button = False
            can_sends.append(hyundaicanfd.create_buttons(self.packer, self.CP, self.CAN, CS, 0, True))
          else:
            for i in range(self.standstill_res_count+1):
              can_sends.append(hyundaicanfd.create_buttons(self.packer, self.CP, self.CAN, CS, Buttons.RES_ACCEL, (i == 0 or i == self.standstill_res_count)))
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
            self.refresh_time = 0.25
          elif 1.0 not in (CS.cruiseGapSet, CS.DistSet) and self.cruise_gap_set_init:
            for i in range(self.btn_count+1):
              can_sends.append(hyundaicanfd.create_buttons(self.packer, self.CP, self.CAN, CS, Buttons.GAP_DIST, (i == 0 or i == self.btn_count)))
            self.last_button_frame = self.frame
            self.cruise_gap_adjusting = True
            self.refresh_time = randint(10,20) * 0.01
        elif self.kisa_variablecruise or self.try_early_stop or self.gap_by_spd_on and not resume_on:
          self.cruise_gap_adjusting = False
          self.cruise_gap_set_init = False
          self.standstill_res_button = False
          self.auto_res_starting = False
          btn_signal = self.KCC.update(CS, self.gap_by_spd_on_sw_trg, sm=self.sm)
          self.btnsignal = btn_signal
          self.on_speed_control = self.KCC.onSpeedControl
          self.on_speed_bump_control = self.KCC.onSpeedBumpControl
          self.curv_speed_control = self.KCC.curvSpeedControl
          self.cut_in_control = self.KCC.cutInControl
          self.driver_scc_set_control = self.KCC.driverSccSetControl
          if btn_signal is not None:
            if btn_signal == 3 and self.KCC.ctrl_gap != (CS.DistSet if CS.DistSet > 0 else CS.cruiseGapSet):
              self.gap_now = CS.DistSet if CS.DistSet > 0 else CS.cruiseGapSet
              self.pause_time = self.pause_time + 1 if self.gap_now == self.gap_prev else 0
              self.gap_prev = self.gap_now
              pause_time = np.interp(self.KCC.t_interval, [8, 80], [50, 100])
              if self.pause_time > pause_time:
                self.last_button_frame = self.frame
                self.cruise_gap_adjusting = False
                self.refresh_count += 0.5
                self.refresh_time = self.refresh_count
                self.pause_time = 0
                self.btn_reset = True
              else:
                for _ in range(self.btn_count if not self.btn_reset else self.standstill_res_count):
                  can_sends.append(hyundaicanfd.create_buttons(self.packer, self.CP, self.CAN, CS, btn_signal, self.btn_reset or self.frame % 8 == 0))
                self.last_button_frame = self.frame
                self.cruise_gap_adjusting = True
                self.refresh_time = 0 if not self.btn_reset else 0.25
                self.btn_reset = False
            elif btn_signal in (1,2) and self.KCC.ctrl_speed != round(CS.VSetDis):
              self.cruise_set_now = round(CS.VSetDis)
              self.pause_time = self.pause_time + 1 if self.cruise_set_now == self.cruise_set_prev else 0
              self.cruise_set_prev = self.cruise_set_now
              pause_time = np.interp(self.KCC.t_interval, [7, 70], [45, 100])
              if self.pause_time > pause_time:
                self.last_button_frame = self.frame
                self.cruise_speed_adjusting = False
                self.refresh_count += 0.5
                self.refresh_time = self.refresh_count
                self.pause_time = 0
                self.btn_reset = True
              else:
                for _ in range(self.btn_count if not self.btn_reset else self.standstill_res_count):
                  can_sends.append(hyundaicanfd.create_buttons(self.packer, self.CP, self.CAN, CS, btn_signal, self.btn_reset or self.frame % 8 == 0))
                self.last_button_frame = self.frame
                self.cruise_speed_adjusting = True
                self.refresh_time = 0 if not self.btn_reset else 0.25
                self.btn_reset = False
          elif (self.KCC.ctrl_gap == (CS.DistSet if CS.DistSet > 0 else CS.cruiseGapSet)) or (self.KCC.ctrl_speed == round(CS.VSetDis)):
            if self.KCC.ctrl_gap == (CS.DistSet if CS.DistSet > 0 else CS.cruiseGapSet) and self.cruise_gap_adjusting:
              self.cruise_gap_adjusting = False
              self.last_button_frame = self.frame
              self.refresh_time = randint(15,30) * 0.01
              self.pause_time = 0
              self.refresh_count = 0
            if self.KCC.ctrl_speed == round(CS.VSetDis) and self.cruise_speed_adjusting:
              self.cruise_speed_adjusting = False
              self.last_button_frame = self.frame
              self.refresh_time = randint(15,30) * 0.01
              self.pause_time = 0
              self.refresh_count = 0
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
          self.pause_time = 0
          self.refresh_count = 0
          self.refresh_time = 0.25
          self.btn_reset = False
        if self.standstill_status_canfd and CS.out.vEgo > 0.3:
          self.standstill_status_canfd = False
          self.standstill_res_button = False
          self.standstill_manual_start_cnt = 0
          self.standstill_manual_start = False
          self.last_lead_distance = 0
      elif (self.frame - self.last_button_frame) * DT_CTRL > self.refresh_time2 and not CS.acc_active:
        self.last_button_frame = self.frame
        if self.acc_activated:
          self.acc_activated = False
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
          self.refresh_time = 0.25
          self.refresh_time2 = 0.25
          self.refresh_count = 0
          self.last_lead_distance = 0
          self.standstill_manual_start_cnt = 0
          self.standstill_manual_start = False
          self.btn_reset = False
        if self.standstill_status_canfd and CS.out.vEgo > 0.3:
          self.standstill_status_canfd = False

        if not CS.regen_level_auto and (self.regen_stop or self.regen_dist or self.regen_e2e):
          if self.regen_stop:
            if CS.regen_level != 20 and not (CS.out.brakePressed or CS.out.gasPressed) and not self.regen_stop_pre_activated:
              self.regen_stop_timer += 1
              if self.regen_stop_timer > 5:
                self.regen_stop_timer = 0
                self.regen_stop_pre_activated = True
            elif CS.regen_level != 20 and self.regen_stop_pre_activated and not (CS.out.brakePressed or CS.out.gasPressed) and (self.dRel < 10 or CS.clu_Vanz < 15):
              self.regen_stop_activated = True
              can_sends.append(hyundaicanfd.create_buttons(self.packer, self.CP, self.CAN, CS, 0, False, False, True, False, True))
              self.refresh_time2 = 1.0
            elif CS.regen_level != 15 and CS.out.gasPressed and self.regen_stop_activated:
              self.regen_stop_pre_activated = False
              if CS.regen_level > 15:
                can_sends.append(hyundaicanfd.create_buttons(self.packer, self.CP, self.CAN, CS, 0, False, False, True, True, False))
              elif CS.regen_level < 15:
                can_sends.append(hyundaicanfd.create_buttons(self.packer, self.CP, self.CAN, CS, 0, False, False, True, False, True))
              self.refresh_time2 = 0.5
            elif CS.regen_level == 15:
              self.regen_stop_activated = False
            elif CS.out.brakePressed or CS.out.gasPressed:
              self.regen_stop_timer = 0
              self.regen_stop_pre_activated = False

        kisa_cruise_auto_res_condition = False
        kisa_cruise_auto_res_condition = CS.acc_active_standby and CS.regen_level < 20 and (not self.kisa_cruise_auto_res_condition or CS.out.gasPressed)
        t_speed = 20 if not CS.is_metric else 30
        if self.model_speed > (60 if not CS.is_metric else 95) and self.cancel_counter == 0 and not CS.acc_active and not CS.out.brakeLights and round(CS.VSetDis) >= t_speed and \
        (1 < CS.lead_distance < 149 or round(CS.clu_Vanz) > t_speed) and round(CS.clu_Vanz) >= 3 and self.cruise_init and \
        self.kisa_cruise_auto_res and kisa_cruise_auto_res_condition and (self.auto_res_limit_sec == 0 or self.auto_res_limit_timer < self.auto_res_limit_sec) and \
        (self.auto_res_delay == 0 or self.auto_res_delay_timer >= self.auto_res_delay):
          if self.kisa_cruise_auto_res_option == 0:
            for _ in range(self.standstill_res_count):
              can_sends.append(hyundaicanfd.create_buttons(self.packer, self.CP, self.CAN, CS, Buttons.CANCEL))
            self.last_button_frame = self.frame
            self.auto_res_starting = True
            self.res_speed = round(CS.VSetDis) if not CS.is_metric or self.osm_spdlimit_enabled else round(CS.clu_Vanz*1.1)
            self.res_speed_timer = 300
            self.refresh_time2 = randint(10,30) * 0.01
          elif self.kisa_cruise_auto_res_option == 1:
            for _ in range(self.standstill_res_count):
              can_sends.append(hyundaicanfd.create_buttons(self.packer, self.CP, self.CAN, CS, Buttons.SET_DECEL))
            self.last_button_frame = self.frame
            self.auto_res_starting = True
            self.v_cruise_kph_auto_res = round(CS.clu_Vanz)
            self.res_speed_timer = 50
            self.refresh_time2 = randint(10,30) * 0.01
          elif self.kisa_cruise_auto_res_option == 2:
            if 1 < CS.lead_distance < 149:
              for _ in range(self.standstill_res_count):
                can_sends.append(hyundaicanfd.create_buttons(self.packer, self.CP, self.CAN, CS, Buttons.CANCEL))
            else:
              for _ in range(self.standstill_res_count):
                can_sends.append(hyundaicanfd.create_buttons(self.packer, self.CP, self.CAN, CS, Buttons.SET_DECEL))
            self.last_button_frame = self.frame
            self.auto_res_starting = True
            self.v_cruise_kph_auto_res = round(CS.clu_Vanz)
            self.res_speed_timer = 50
            self.refresh_time2 = randint(10,30) * 0.01

      if self.frame % 10 == 0 and not self.CP.adrvControl:
        if self.CP.capacitiveSteeringWheel and self.btnsignal is None and not self.standstill_res_button and not self.cruise_gap_adjusting and \
        (not (self.on_speed_control or self.on_speed_bump_control or self.curv_speed_control or self.cut_in_control or self.driver_scc_set_control)):
          blinker = CS.out.leftBlinker or CS.out.rightBlinker
          if (self.frame - self.last_button_frame2) * DT_CTRL > self.refresh_time3:
            self.last_button_frame2 = self.frame
            for _ in range(randint(5,10) if not blinker else 1):
              can_sends.append(hyundaicanfd.create_steering_wheel(self.packer, self.CP, self.CAN, CS.csw_info))
          elif blinker:
            self.refresh_time3 = randint(8,12) * 0.1
          else:
            self.refresh_time3 = randint(8,12)


    return can_sends


  def create_common_msgs(self, CS, CC, lat_active, stopping, set_speed_in_units, new_torque):
    # common
    if CS.acc_active and CS.lead_distance > 149 and self.dRel < ((CS.out.vEgo * CV.MS_TO_KPH)+5) < 100 and \
     self.vRel*3.6 < -(CS.out.vEgo * CV.MS_TO_KPH * 0.16) and CS.out.vEgo > 7 and abs(CS.out.steeringAngleDeg) < 10 and not self.longcontrol:
      self.need_brake_timer += 1
      if self.need_brake_timer > 100:
        self.need_brake = True
    elif not CS.acc_active and 1 < self.dRel < (CS.out.vEgo * CV.MS_TO_KPH * 0.5) < 13 and self.vRel*3.6 < -(CS.out.vEgo * CV.MS_TO_KPH * 0.6) and \
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
    elif CS.acc_active:
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

    if (CS.out.brakeLights and CS.out.vEgo == 0 and not CS.out.cruiseState.standstill) or CS.i_pedel_stop:
      self.standstill_status_timer += 1
      if self.standstill_status_timer > 200:
        self.standstill_status = True
        self.standstill_status_timer = 0
    if self.standstill_status and CS.out.vEgo > 1:
      self.standstill_status = False
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

    if CS.acc_active: # to toggle lkas, hold gap button for 1 sec
      if CS.cruise_buttons[-1] == 3:
        self.lkas_onoff_counter += 1
        self.gap_by_spd_on_sw = True
        self.gap_by_spd_on_sw_cnt2 = 0
        self.road_spd_on_sw = True
        self.road_spd_on_sw_cnt2 = 0
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
        if self.road_spd_on_sw:
          self.road_spd_on_sw = False
          self.road_spd_on_sw_cnt += 1
          if self.road_spd_on_sw_cnt > 2: #temporary disable setting of road limit spped if you press gap button 3 times quickly.
            self.road_spd_on_sw_trg = not self.road_spd_on_sw_trg
            self.road_spd_on_sw_cnt = 0
            self.road_spd_on_sw_cnt2 = 0
        elif self.road_spd_on_sw_cnt:
          self.road_spd_on_sw_cnt2 += 1
          if self.road_spd_on_sw_cnt2 > 20:
            self.road_spd_on_sw_cnt = 0
            self.road_spd_on_sw_cnt2 = 0
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
      self.road_spd_on_sw_cnt = 0
      self.road_spd_on_sw_cnt2 = 0
      self.road_spd_on_sw = False
      self.road_spd_on_sw_trg = True

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

    if self.CP.isCanFD:
      if self.CP.isAngleControl:
        self.str_log1 = 'EN/LA/LO={}/{}{}/{}  MD={}  BS={:1.0f}  CV={:03.0f}/{:0.4f}  TQ={:03.0f}/{:03.0f}  A={:0.1f}/{:0.1f}/{:0.1f}'.format(
          int(CC.enabled), int(CC.latActive), int(lat_active), int(CC.longActive), CS.out.cruiseState.modeSel, self.CP.sccBus, self.model_speed, abs(self.sm['controlsState'].curvature), self.lkas_max_torque, abs(CS.out.steeringTorque), self.apply_angle_last, CS.stock_str_angle, self.apply_angle_now)
      else:
        self.str_log1 = 'EN/LA/LO={}/{}{}/{}  MD={}  BS={:1.0f}  CV={:03.0f}/{:0.4f}  TQ={:03.0f}/{:03.0f}  VF={:03.0f}  ST={:03.0f}/{:01.0f}/{:01.0f}'.format(
          int(CC.enabled), int(CC.latActive), int(lat_active), int(CC.longActive), CS.out.cruiseState.modeSel, self.CP.sccBus, self.model_speed, abs(self.sm['controlsState'].curvature), abs(new_torque), abs(CS.out.steeringTorque), self.vFuture, self.params.STEER_MAX, self.params.STEER_DELTA_UP, self.params.STEER_DELTA_DOWN)
      if CS.acc_active:
        self.str_log2 = 'AQ={:+04.2f}  SS={:03.0f}/{:03.0f}  VF={:03.0f}/{:03.0f}  TS/VS={:03.0f}/{:03.0f}  RD/ED/C/T={:04.1f}/{:04.1f}/{}/{}  C={:1.0f}/{:1.0f}/{}'.format(
        self.aq_value if self.longcontrol else CS.scc_control["aReqValue"], set_speed_in_units, self.sm['carState'].vCruise, self.vFuture, self.vFutureA, self.KCC.ctrl_speed, round(CS.VSetDis), CS.lead_distance, self.dRel, int(self.KCC.cut_in), self.KCC.cut_in_run_timer, 0, CS.cruiseGapSet, self.btnsignal if self.btnsignal is not None else 0, self.KCC.t_interval)
      else:
        self.str_log2 = 'MDPS={}  LKAS={:1.0f}  LEAD={}  AQ={:+04.2f}  VF={:03.0f}/{:03.0f}  CG={:1.0f}  M/C={:1.0f}/{:1.0f}'.format(
        int(not CS.out.steerFaultTemporary), 0, int(bool(0 < CS.lead_distance < 149)), self.aq_value if self.longcontrol else CS.scc_control["aReqValue"], self.vFuture, self.vFutureA, CS.cruiseGapSet, CS.main_buttons[-1], CS.cruise_buttons[-1])
    else:
      self.str_log1 = 'EN/LA/LO={}/{}{}/{}  MD={}  BS={:1.0f}  CV={:03.0f}/{:0.4f}  TQ={:03.0f}/{:03.0f}  VF={:03.0f}  ST={:03.0f}/{:01.0f}/{:01.0f}'.format(
        int(CC.enabled), int(CC.latActive), int(lat_active), int(CC.longActive), CS.out.cruiseState.modeSel, self.CP.sccBus, self.model_speed, abs(self.sm['controlsState'].curvature), abs(new_torque), abs(CS.out.steeringTorque), self.vFuture, self.params.STEER_MAX, self.params.STEER_DELTA_UP, self.params.STEER_DELTA_DOWN)
      if CS.acc_active:
        self.str_log2 = 'AQ={:+04.2f}  SS={:03.0f}/{:03.0f}  VF={:03.0f}/{:03.0f}  TS/VS={:03.0f}/{:03.0f}  RD/ED/C/T={:04.1f}/{:04.1f}/{}/{}/{}  C={:1.0f}/{:1.0f}/{}/{:1.0f}'.format(
        self.aq_value if self.longcontrol else CS.scc12["aReqValue"], set_speed_in_units, self.sm['carState'].vCruise, self.vFuture, self.vFutureA, self.KCC.ctrl_speed, round(CS.VSetDis), CS.lead_distance, self.dRel, int(self.KCC.cut_in), self.KCC.cut_in_run_timer, self.ed_rd_diff_on_timer, CS.cruiseGapSet, self.btnsignal if self.btnsignal is not None else 0, self.KCC.t_interval, self.l_stat)
      else:
        self.str_log2 = 'MDPS={}  LKAS={:1.0f}  LEAD={}  AQ={:+04.2f}  VF={:03.0f}/{:03.0f}  CG={:1.0f}  M/C={:1.0f}/{:1.0f}'.format(
        int(not CS.out.steerFaultTemporary), CS.lkas_button_on, int(bool(0 < CS.lead_distance < 149)), self.aq_value if self.longcontrol else CS.scc12["aReqValue"], self.vFuture, self.vFutureA, CS.cruiseGapSet, CS.main_buttons[-1], CS.cruise_buttons[-1])

    self.cc_timer += 1
    if self.cc_timer > 100:
      self.cc_timer = 0
      self.radar_helper_option = self.c_params.get("RadarLongHelper", return_default=True)
      self.stopping_dist_adj_enabled = self.c_params.get_bool("StoppingDistAdj")
      self.standstill_res_count = self.c_params.get("RESCountatStandstill", return_default=True)
      self.kisa_cruisegap_auto_adj = self.c_params.get_bool("CruiseGapAdjust")
      self.to_avoid_lkas_fault_enabled = self.c_params.get_bool("AvoidLKASFaultEnabled")
      self.to_avoid_lkas_fault_max_angle = self.c_params.get("AvoidLKASFaultMaxAngle", return_default=True)
      self.to_avoid_lkas_fault_max_frame = self.c_params.get("AvoidLKASFaultMaxFrame", return_default=True)
      self.stopsign_enabled = self.c_params.get_bool("StopAtStopSign")
      self.gap_by_spd_on = self.c_params.get_bool("CruiseGapBySpdOn")
      self.experimental_mode = self.c_params.get_bool("ExperimentalMode")
      self.usf = self.c_params.get("UserSpecificFeature", return_default=True)
      self.regenbrake = self.c_params.get_bool("RegenBrakeFeatureOn")
      rgn_option_list = list(self.c_params.get("RegenBrakeFeature", return_default=True))
      self.regen_stop = True if '1' in rgn_option_list and self.regenbrake else False
      if self.c_params.get_bool("KisaLiveTunePanelEnable"):
        if self.CP.isAngleControl:
          pass
        elif self.CP.lateralTuning.which() == 'pid':
          self.str_log3 = 'T={:0.2f}/{:0.3f}/{:0.1f}/{:0.5f}'.format(self.c_params.get("PidKp", return_default=True)*0.01, \
          self.c_params.get("PidKi", return_default=True)*0.001, self.c_params.get("PidKd", return_default=True)*0.01, self.c_params.get("PidKf", return_default=True)*0.00001)
        elif self.CP.lateralTuning.which() == 'indi':
          self.str_log3 = 'T={:03.1f}/{:03.1f}/{:03.1f}/{:03.1f}'.format(self.c_params.get("InnerLoopGain", return_default=True)*0.1, \
          self.c_params.get("OuterLoopGain", return_default=True)*0.1, self.c_params.get("TimeConstant", return_default=True)*0.1, self.c_params.get("ActuatorEffectiveness", return_default=True)*0.1)
        elif self.CP.lateralTuning.which() == 'lqr':
          self.str_log3 = 'T={:04.0f}/{:05.3f}/{:07.5f}'.format(self.c_params.get("Scale", return_default=True)*1.0, self.c_params.get("LqrKi", return_default=True)*0.001, self.c_params.get("DcGain", return_default=True)*0.00001)
        elif self.CP.lateralTuning.which() == 'torque':
          self.str_log3 = 'T={:0.1f}/{:0.1f}/{:0.1f}/{:0.1f}/{:0.1f}/{:0.3f}'.format(self.c_params.get("TorqueMaxLatAccel", return_default=True)*0.1, \
          self.c_params.get("TorqueKp", return_default=True)*0.1, self.c_params.get("TorqueKf", return_default=True)*0.1, self.c_params.get("TorqueKi", return_default=True)*0.1, \
           self.c_params.get("TorqueKd", return_default=True)*0.1, self.c_params.get("TorqueFriction", return_default=True)*0.001)
      elif self.CP.lateralTuning.which() == 'torque' and self.live_torque_params:
        torque_params = self.sm['liveTorqueParameters']
        self.str_log3 = 'T={:0.2f}/{:0.2f}/{:0.3f}'.format(torque_params.latAccelFactorFiltered, torque_params.latAccelOffsetFiltered, torque_params.frictionCoefficientFiltered)


  def smooth_steer(self, apply_torque, CS):
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

    return int(round(float(apply_torque)))
