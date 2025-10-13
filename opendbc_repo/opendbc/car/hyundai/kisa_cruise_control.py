# created by atom, managed and updated by multikyd
import numpy as np
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.hyundai.values import Buttons, CANFD_CAR
from cereal import log
import cereal.messaging as messaging
from random import randint
from openpilot.common.params import Params

LaneChangeState = log.LateralPlan.LaneChangeState

class KisaCruiseControl():
  def __init__(self):
    self.btn_cnt = 0
    self.seq_command = 0
    self.target_speed = 0
    self.target_gap = 0
    self.set_point = 0
    self.gap_point = 0
    self.wait_timer2 = 0
    self.wait_timer3 = 0

    self.gasPressed_old = 0

    self.params = Params()

    self.map_spdlimit_offset = self.params.get("KisaSpeedLimitOffset", return_default=True)
    self.safetycam_decel_dist_gain = self.params.get("SafetyCamDecelDistGain", return_default=True)

    self.map_speed_block = False
    self.map_speed_dist = 0
    self.map_speed = 0
    self.map_speed_prev = 0
    self.map_speed_dist_extend = False
    self.onSpeedControl = False
    self.onSpeedBumpControl = False
    self.onSpeedBumpControl2 = False
    self.curvSpeedControl = False
    self.cutInControl = False
    self.driverSccSetControl = False
    self.ctrl_speed = 0
    self.ctrl_gap = 0
    self.vision_curv_speed_c = list(map(int, self.params.get("VCurvSpeedC", return_default=True).split(',')))
    self.vision_curv_speed_t = list(map(int, self.params.get("VCurvSpeedT", return_default=True).split(',')))
    self.vision_curv_speed_cmph = list(map(int, self.params.get("VCurvSpeedCMPH", return_default=True).split(',')))
    self.vision_curv_speed_tmph = list(map(int, self.params.get("VCurvSpeedTMPH", return_default=True).split(',')))

    self.osm_curv_speed_c = list(map(int, self.params.get("OCurvSpeedC", return_default=True).split(',')))
    self.osm_curv_speed_t = list(map(int, self.params.get("OCurvSpeedT", return_default=True).split(',')))
    self.osm_custom_spdlimit_c = list(map(int, self.params.get("OSMCustomSpeedLimitC", return_default=True).split(',')))
    self.osm_custom_spdlimit_t = list(map(int, self.params.get("OSMCustomSpeedLimitT", return_default=True).split(',')))

    self.osm_wait_timer = 0
    self.stock_navi_info_enabled = self.params.get_bool("StockNaviSpeedEnabled")
    self.osm_speedlimit_enabled = self.params.get_bool("OSMSpeedLimitEnable")
    self.curv_decel_option = self.params.get("CurvDecelOption", return_default=True)
    self.cut_in = False
    self.cut_in_run_timer = 0

    self.decel_on_speedbump = self.params.get_bool("KISASpeedBump")
    self.navi_sel = self.params.get("KISANaviSelect", return_default=True)

    self.t_interval = 7
    self.t_interval2 = self.params.get("KISACruiseSpammingInterval", return_default=True)
    self.faststart = False
    self.faststart_curv = False
    self.safetycam_speed = 0
    self.decelonstop = False

    self.use_radar_value = self.params.get_bool("UseRadarValue")

    self.e2e_x = 0

    self.kisa_cruisegap_auto_adj = self.params.get_bool("CruiseGapAdjust")
    self.cruise_gap_prev = 0
    self.cruise_gap_set_init = False
    self.cruise_gap_adjusting = False

    self.try_early_stop = self.params.get_bool("KISAEarlyStop")
    self.try_early_stop_retrieve = False
    self.try_early_stop_org_gap = 4.0

    self.is_canfd = False

    self.cruise_road_limitspd_offset = self.params.get("CruiseSetwithRoadLimitSpeedOffset", return_default=True)

  def button_status(self, CS):
    if not CS.acc_active or CS.cruise_buttons[-1] != Buttons.NONE or CS.main_buttons[-1] or CS.lfa_buttons[-1]:
      self.wait_timer2 = 80 if CS.CP.carFingerprint not in CANFD_CAR else 100
    elif self.wait_timer2:
      self.wait_timer2 -= 1
    else:
      return 1
    return 0

  # buttn acc,dec control
  def switch(self, seq_cmd):
      self.case_name = "case_" + str(seq_cmd)
      self.case_func = getattr( self, self.case_name, lambda:"default")
      return self.case_func()

  def reset_btn(self):
      if self.seq_command != 4:
        self.seq_command = 0

  def case_default(self):
      self.seq_command = 0
      return None

  def case_0(self):
      self.btn_cnt = 0
      self.target_gap = self.gap_point
      self.target_speed = self.set_point
      delta_speed = round(self.target_speed - self.VSetDis)
      if self.target_gap and self.target_gap != self.DistSet:
        self.seq_command = 3  # case_3 번으로 이동.
      elif delta_speed > 0:
        self.seq_command = 1  # case_1 번으로 이동.
      elif delta_speed < 0:
        self.seq_command = 2  # case_2 번으로 이동.
      return None

  def case_1(self):  # res
      btn_signal = Buttons.RES_ACCEL
      self.btn_cnt += 1
      if self.target_speed == self.VSetDis:
        self.btn_cnt = 0
        self.seq_command = 4      # case_4 번으로 이동.
      elif self.btn_cnt > (2 if self.is_canfd else 5):
        self.btn_cnt = 0
        self.seq_command = 4      # case_4 번으로 이동.
      return btn_signal

  def case_2(self):  # dec
      btn_signal = Buttons.SET_DECEL
      self.btn_cnt += 1
      if self.target_speed == self.VSetDis:
        self.btn_cnt = 0
        self.seq_command = 4      # case_4 번으로 이동.
      elif self.btn_cnt > (2 if self.is_canfd else 5):
        self.btn_cnt = 0
        self.seq_command = 4      # case_4 번으로 이동.
      return btn_signal

  def case_3(self):  # gap
      btn_signal = Buttons.GAP_DIST
      self.btn_cnt += 1
      if self.target_gap == self.DistSet:
        self.btn_cnt = 0
        self.seq_command = 4      # case_4 번으로 이동.
      elif self.btn_cnt > (2 if self.is_canfd else 5):
        self.btn_cnt = 0
        self.seq_command = 4      # case_4 번으로 이동.
      return btn_signal

  def case_4(self):  # None  버튼 off 유지시간. 크르즈 속도제어오류 발생시 아래의 수치를 조금 변경하여 보십시오. 수치가 크면 속도변경이 느려지고, 작으면 빨라집니다.
      btn_signal = None  # Buttons.NONE

      self.btn_cnt += 1
      #if self.btn_cnt == 1:
      #  btn_signal = Buttons.NONE
      if self.btn_cnt > self.t_interval:    # 버튼 클릭후 일정시간 기다린다.  (반드시 필요함)
        self.seq_command = 0   # case_0 번으로 이동.  (다음 명령을 실행)
      return btn_signal

  def ascc_button_control(self, CS, set_speed, set_gap):
    self.gap_point = set_gap
    self.set_point = max(20 if not CS.is_metric else 30, set_speed)
    self.DistSet = CS.DistSet if CS.DistSet > 0 else CS.cruiseGapSet
    self.VSetDis = round(CS.VSetDis)
    btn_signal = self.switch(self.seq_command)

    return btn_signal

  def get_navi_speed(self, CS, navi_data, osm_data, car_state, cruiseState_speed):
    cruise_set_speed_kph = cruiseState_speed
    v_ego_kph = CS.out.vEgo * CV.MS_TO_KPH
    v_ego_mph = CS.out.vEgo * CV.MS_TO_MPH

    if not car_state.pauseSpdLimit:
      if self.navi_sel == 2:
        if navi_data.wazeRoadSpeedLimit > 9 or navi_data.wazeAlertDistance > 0:
          self.map_speed = navi_data.wazeRoadSpeedLimit
          self.map_speed_dist = max(0, navi_data.wazeAlertDistance)
          self.map_speed_dist_extend = navi_data.wazeAlertExtend
          if self.map_speed < 9:
            self.map_speed = self.map_speed_prev
          else:
            self.map_speed_prev = self.map_speed
          spdTarget = self.map_speed
          cam_distance_calc = 0
          cam_distance_calc = np.interp(self.map_speed * CV.MPH_TO_KPH if not CS.is_metric else 1, [30, 60, 110], [2.5, 3.0, 3.7])
          consider_speed = np.interp((v_ego_kph - self.map_speed * CV.MPH_TO_KPH if not CS.is_metric else 1), [0, 50], [1, 1.8])
          min_control_dist = np.interp(self.map_speed * CV.MPH_TO_KPH if not CS.is_metric else 1, [30, 110], [40, 250])
          final_cam_decel_start_dist = cam_distance_calc*consider_speed*v_ego_kph * (1 + self.safetycam_decel_dist_gain*0.01)
          if 0 < self.map_speed and self.map_speed_dist != 0:
            if self.map_speed_dist < final_cam_decel_start_dist:
              spdTarget = self.map_speed
            elif self.map_speed_dist < min_control_dist and self.map_speed_dist != 0:
              spdTarget = self.map_speed
            elif self.map_speed_dist_extend:
              spdTarget = self.map_speed
          else:
            self.onSpeedControl = False
            return cruise_set_speed_kph
          if self.map_spdlimit_offset > 0:
            cruise_set_speed_kph = spdTarget + round(spdTarget*0.01*self.map_spdlimit_offset)
          elif self.map_spdlimit_offset == -1:
            cruise_set_speed_kph = int(np.interp(spdTarget, self.osm_custom_spdlimit_c, self.osm_custom_spdlimit_t))
          if cruise_set_speed_kph+1.5 < v_ego_mph and not CS.is_metric and not CS.out.gasPressed:
            self.onSpeedControl = True
          elif cruise_set_speed_kph+1.5 < v_ego_kph and not not CS.is_metric and not CS.out.gasPressed:
            self.onSpeedControl = True
          else:
            self.onSpeedControl = False
      elif self.osm_speedlimit_enabled:  # osm speedlimit
        if osm_data.speedLimit > 21 or osm_data.speedLimitAhead > 21:
          spdTarget = osm_data.speedLimit
          self.map_speed = osm_data.speedLimitAhead
          self.map_speed_dist = max(0, osm_data.speedLimitAheadDistance)
          cam_distance_calc = 0
          cam_distance_calc = np.interp(self.map_speed * CV.MPH_TO_KPH if not CS.is_metric else 1, [30, 60, 110], [2.5, 3.0, 3.7])
          consider_speed = np.interp((v_ego_kph - self.map_speed * CV.MPH_TO_KPH if not CS.is_metric else 1), [0, 50], [1, 1.8])
          min_control_dist = np.interp(self.map_speed * CV.MPH_TO_KPH if not CS.is_metric else 1, [30, 110], [40, 250])
          final_cam_decel_start_dist = cam_distance_calc*consider_speed*v_ego_kph * (1 + self.safetycam_decel_dist_gain*0.01)
          if ((21 < self.map_speed < spdTarget) or (21 < self.map_speed and spdTarget == 0)) and self.map_speed_dist != 0:
            if self.map_speed_dist < final_cam_decel_start_dist:
              spdTarget = self.map_speed
            elif self.map_speed_dist < min_control_dist:
              spdTarget = self.map_speed
            elif spdTarget == 0:
              self.onSpeedControl = False
              return cruise_set_speed_kph
          elif spdTarget > 21:
            pass
          else:
            self.onSpeedControl = False
            return cruise_set_speed_kph
          if self.map_spdlimit_offset > 0:
            cruise_set_speed_kph = spdTarget + round(spdTarget*0.01*self.map_spdlimit_offset)
          elif self.map_spdlimit_offset == -1:
            cruise_set_speed_kph = int(np.interp(spdTarget, self.osm_custom_spdlimit_c, self.osm_custom_spdlimit_t))
          if cruise_set_speed_kph+1.5 < v_ego_mph and not CS.is_metric and not CS.out.gasPressed:
            self.onSpeedControl = True
          elif cruise_set_speed_kph+1.5 < v_ego_kph and not not CS.is_metric and not CS.out.gasPressed:
            self.onSpeedControl = True
          else:
            self.onSpeedControl = False
      elif self.decel_on_speedbump and navi_data.safetySign in ("22", "SpeedBump") and self.navi_sel == 1:
        sb_consider_speed = np.interp((v_ego_kph - (20 if not CS.is_metric else 30)), [0, 10, 25, 50], [1.5, 1.9, 2.0, 2.1])
        sb_final_decel_start_dist = sb_consider_speed*v_ego_kph
        min_dist_v = np.interp(CS.out.vEgo, [8.3, 13.8], [20, 40])
        if min_dist_v < navi_data.safetyDistance < sb_final_decel_start_dist:
          cruise_set_speed_kph == 20 if not CS.is_metric else 30
          self.onSpeedBumpControl = True
          self.onSpeedBumpControl2 = False
        elif navi_data.safetyDistance >= sb_final_decel_start_dist:
          cruise_set_speed_kph == 35 if not CS.is_metric else 60
          self.onSpeedBumpControl = False
          self.onSpeedBumpControl2 = True
        else:
          self.onSpeedBumpControl = False
          self.onSpeedBumpControl2 = False
      elif self.navi_sel == 1 and navi_data.speedLimit > 21 and navi_data.safetySign not in ("20", "21"):  # navi app speedlimit
        self.onSpeedBumpControl = False
        self.onSpeedBumpControl2 = False
        self.map_speed_dist = max(0, navi_data.safetyDistance - 30)
        self.map_speed = navi_data.speedLimit
        if self.map_speed_dist > 1250:
          self.map_speed_block = True
        elif 50 < self.map_speed_dist <= 1250 and self.map_speed_block:
          self.map_speed_block = True
        else:
          self.map_speed_block = False
        cam_distance_calc = 0
        cam_distance_calc = np.interp(self.map_speed, [30, 60, 110], [2.2, 2.9, 3.6])
        consider_speed = np.interp((v_ego_kph - self.map_speed), [0, 50], [1, 1.7])
        min_control_dist = np.interp(self.map_speed, [30, 110], [40, 250])
        final_cam_decel_start_dist = cam_distance_calc*consider_speed*v_ego_kph * (1 + self.safetycam_decel_dist_gain*0.01)
        if self.map_speed_dist < final_cam_decel_start_dist:
          spdTarget = self.map_speed
        elif self.map_speed_dist >= final_cam_decel_start_dist and self.map_speed_block:
          spdTarget = self.map_speed
        elif self.map_speed_dist < min_control_dist:
          spdTarget = self.map_speed
        elif self.onSpeedControl and self.map_speed > 21:
          spdTarget = self.map_speed
        else:
          return cruise_set_speed_kph
        if self.map_spdlimit_offset > 0:
          cruise_set_speed_kph = spdTarget + round(spdTarget*0.01*self.map_spdlimit_offset)
        elif self.map_spdlimit_offset == -1:
          cruise_set_speed_kph = int(np.interp(spdTarget, self.osm_custom_spdlimit_c, self.osm_custom_spdlimit_t))
        if cruise_set_speed_kph+1.5 < v_ego_mph and not CS.is_metric and not CS.out.gasPressed:
          self.onSpeedControl = True
        elif cruise_set_speed_kph+1.5 < v_ego_kph and not not CS.is_metric and not CS.out.gasPressed:
          self.onSpeedControl = True
        else:
          self.onSpeedControl = False
      else:
        spdTarget = cruise_set_speed_kph
        self.onSpeedControl = False
        self.map_speed = 0
        self.map_speed_prev = 0
        self.map_speed_dist = 0
        self.map_speed_block = False
        self.onSpeedBumpControl = False
        self.onSpeedBumpControl2 = False
    else:
      spdTarget = cruise_set_speed_kph
      self.onSpeedControl = False
      self.map_speed = 0
      self.map_speed_prev = 0
      self.map_speed_dist = 0
      self.map_speed_dist_extend = False
      if not car_state.pauseSpdLimit:
        self.map_speed_block = False
      self.onSpeedBumpControl = False
      self.onSpeedBumpControl2 = False

    # elif safetyDistance >= 50:
    #   if speedLimit <= 60:
    #     spdTarget = np.interp(safetyDistance, [50, 600], [ speedLimit, speedLimit + 50 ])
    #   else:
    #     spdTarget = np.interp(safetyDistance, [150, 900], [ speedLimit, speedLimit + 30 ])
    # else:
    #   spdTarget = speedLimit

    # if v_ego_kph < speedLimit:
    #   v_ego_kph = speedLimit

    # print('cruise_set_speed_kph={}'.format(cruise_set_speed_kph))

    return cruise_set_speed_kph

  def auto_speed_control(self, CS, controls_state, selfdrive_state, radar_state, lat_plan, long_plan, navi_speed, osm_data):
    modelSpeed = lat_plan.modelSpeed
    min_control_speed = 20 if not CS.is_metric else 30
    var_speed = navi_speed
    lead_0 = radar_state.leadOne
    lead_1 = radar_state.leadTwo

    if len(long_plan.e2eX) > 12:
      self.e2e_x = long_plan.e2eX[12]

    cut_in_model = True if lead_1.status and (lead_0.dRel - lead_1.dRel) > 3.0 and ((0 < lead_0.dRel < 85) or (0 < lead_1.dRel < 85)) else False
    dist_sel = lead_1.dRel if 0 < lead_1.dRel < 85 else lead_0.dRel if 0 < lead_0.dRel < 85 else CS.lead_distance
    cut_in_ed_rd_diff = True if 0 < CS.lead_distance <= 90 and (CS.lead_distance - dist_sel) > np.interp(CS.lead_distance, [10, 50], [3.0, 9.0]) else False

    if CS.CP.sccBus == 0 and CS.CP.openpilotLongitudinalControl:
      self.cut_in = cut_in_model or cut_in_ed_rd_diff
    else:
      self.cut_in = cut_in_ed_rd_diff

    self.driverSccSetControl = False

    if CS.driverAcc_time and CS.out.cruiseState.modeSel in (1,2,4):
      self.t_interval = randint(self.t_interval2+3, self.t_interval2+5) if not CS.is_metric else randint(self.t_interval2, self.t_interval2+2)
      self.driverSccSetControl = True
      return min(max(CS.clu_Vanz + (3 if not CS.is_metric else 5), 30 if not CS.is_metric else 50), navi_speed)
    # elif self.gasPressed_old:
    #   clu_Vanz = CS.clu_Vanz
    #   ctrl_speed = max(min_control_speed, ctrl_speed, clu_Vanz)
    #   CS.set_cruise_speed(ctrl_speed)
    elif controls_state.resSpeed > 21:
      self.t_interval = randint(self.t_interval2+3, self.t_interval2+5) if not CS.is_metric else randint(self.t_interval2, self.t_interval2+2)
      res_speed = max(min_control_speed, controls_state.resSpeed)
      return min(res_speed, navi_speed)
    elif CS.out.cruiseState.modeSel in (1,2,3,4):
      if selfdrive_state.experimentalMode and CS.CP.sccBus == 0:
        self.t_interval = randint(self.t_interval2+3, self.t_interval2+5) if not CS.is_metric else randint(self.t_interval2, self.t_interval2+2)
        var_speed = min(round(controls_state.vFuture), navi_speed)
      elif CS.out.brakeLights and CS.out.vEgo == 0 and CS.out.cruiseState.modeSel in (1,2,4):
        self.faststart = True
        self.t_interval = randint(self.t_interval2+3, self.t_interval2+5) if not CS.is_metric else randint(self.t_interval2, self.t_interval2+2)
        var_speed = min(navi_speed, 30 if not CS.is_metric else 45)
      elif self.onSpeedBumpControl2 and not lead_0.status:
        var_speed = min(navi_speed, 30 if not CS.is_metric else 60)
        self.t_interval = randint(self.t_interval2+3, self.t_interval2+5) if not CS.is_metric else randint(self.t_interval2, self.t_interval2+2)
      elif self.onSpeedBumpControl:
        var_speed = min(navi_speed, 20 if not CS.is_metric else 30)
        self.t_interval = randint(self.t_interval2+3, self.t_interval2+5) if not CS.is_metric else randint(self.t_interval2, self.t_interval2+2)
      elif self.faststart and round(controls_state.vFuture) <= (25 if not CS.is_metric else 40):
        var_speed = min(navi_speed, 30 if not CS.is_metric else 45)
      elif (lead_0.status or lead_1.status) and round(controls_state.vFuture) >= (min_control_speed-(4 if not CS.is_metric else 7)) and CS.out.cruiseState.modeSel in (1,2,4):
        self.faststart = False
        # dRel = CS.lead_distance if 0 < CS.lead_distance < 149 and not self.cut_in_run_timer else int(lead_0.dRel)
        # vRel = CS.lead_objspd * (CV.KPH_TO_MPH if not CS.is_metric else 1) if 0 < CS.lead_distance < 149 and \
        #  not self.cut_in_run_timer else int(lead_0.vRel * (CV.MS_TO_MPH if not CS.is_metric else CV.MS_TO_KPH))
        dRel = CS.lead_distance if self.use_radar_value and CS.lead_distance < 149 else lead_0.dRel
        vRel = int(CS.lead_objspd * (CV.MS_TO_MPH if not CS.is_metric else CV.MS_TO_KPH)) if self.use_radar_value and CS.lead_distance < 149 else int(lead_0.vRel * (CV.MS_TO_MPH if not CS.is_metric else CV.MS_TO_KPH))
        if self.cut_in_run_timer > 0:
          self.cut_in_run_timer -= 1
        elif self.cut_in:
          self.cut_in_run_timer = 1500
        d_ratio = np.interp(CS.clu_Vanz, [40, 110], [0.3, 0.19])
        if self.cut_in_run_timer and dRel < CS.clu_Vanz * d_ratio: # keep decel when cut_in, max running time 15sec
          self.t_interval = randint(self.t_interval2+3, self.t_interval2+5) if not CS.is_metric else randint(self.t_interval2, self.t_interval2+2)
          self.cutInControl = True
          var_speed = min(round(controls_state.vFutureA), navi_speed)
        elif vRel > (-3 if not CS.is_metric else -5):
          var_speed = min(round(controls_state.vFuture) + max(0, int(dRel*(0.11 if not CS.is_metric else 0.16)+vRel)), navi_speed)
          ttime = randint(50, 70) if not CS.is_metric else randint(25, 45)
          self.t_interval = int(np.interp(dRel, [15, 50], [self.t_interval2, ttime])) if not (self.onSpeedControl or self.curvSpeedControl or self.cut_in) else randint(self.t_interval2+3, self.t_interval2+5) if not CS.is_metric else randint(self.t_interval2, self.t_interval2+2)
          self.cutInControl = False
        else:
          var_speed = min(round(controls_state.vFuture), navi_speed)
          self.t_interval = randint(self.t_interval2+3, self.t_interval2+5) if not CS.is_metric else randint(self.t_interval2, self.t_interval2+2)
          self.cut_in_run_timer = 0
          self.cutInControl = False
      elif lead_0.status and round(controls_state.vFuture) < min_control_speed and CS.out.cruiseState.modeSel in (1,2,4):
        self.faststart = False
        var_speed = min(round(controls_state.vFuture), navi_speed)
        self.t_interval = randint(self.t_interval2+3, self.t_interval2+5) if not CS.is_metric else randint(self.t_interval2, self.t_interval2+2)
        self.cutInControl = False
      elif CS.out.cruiseState.modeSel == 3: # curv only
        vRel = int(CS.lead_objspd * (CV.MS_TO_MPH if not CS.is_metric else CV.MS_TO_KPH)) if self.use_radar_value and CS.lead_distance < 149 else int(lead_0.vRel * (CV.MS_TO_MPH if not CS.is_metric else CV.MS_TO_KPH))
        if (CS.out.brakeLights and CS.out.vEgo == 0) or (self.faststart_curv and round(controls_state.vFuture) < (15 if not CS.is_metric else 25)):
          self.faststart_curv = True
          var_speed = min(navi_speed, 20 if not CS.is_metric else 30)
        elif self.faststart_curv and (15 if not CS.is_metric else 25) <= round(controls_state.vFuture) <= (25 if not CS.is_metric else 35):
          var_speed = min(navi_speed, 30 if not CS.is_metric else 45)
        elif vRel >= 0:
          self.faststart_curv = False
          self.decelonstop = False
          var_speed = navi_speed
        elif vRel < (-12 if not CS.is_metric else -20): # encounter with a stopped car
          self.faststart_curv = False
          self.decelonstop = True
          var_speed = min(round(controls_state.vFuture), navi_speed)
        elif self.decelonstop:
          self.faststart_curv = False
          var_speed = min(round(controls_state.vFuture), navi_speed)
        else:
          self.faststart_curv = False
          var_speed = navi_speed
        self.faststart = False
        self.t_interval = randint(self.t_interval2+3, self.t_interval2+5) if not CS.is_metric else randint(self.t_interval2, self.t_interval2+2)
        self.cutInControl = False
      else:
        self.faststart = False
        var_speed = navi_speed
        ttime = randint(50, 70) if not CS.is_metric else randint(25, 45)
        self.t_interval = ttime if not (self.onSpeedControl or self.curvSpeedControl or self.cut_in) else randint(self.t_interval2+3, self.t_interval2+5) if not CS.is_metric else randint(self.t_interval2, self.t_interval2+2)
        self.cutInControl = False
    else:
      var_speed = navi_speed
      self.t_interval = randint(self.t_interval2+3, self.t_interval2+5) if not CS.is_metric else randint(self.t_interval2, self.t_interval2+2)
      self.cut_in_run_timer = 0
      self.cutInControl = False

    if CS.out.cruiseState.modeSel in (1,3,4) and self.curv_decel_option in (1,2):
      if CS.out.vEgo * CV.MS_TO_KPH > 40 and modelSpeed < (self.vision_curv_speed_cmph[-1] if not CS.is_metric else self.vision_curv_speed_c[-1]) and lat_plan.laneChangeState == LaneChangeState.off and \
       not (CS.out.leftBlinker or CS.out.rightBlinker):
        if not CS.is_metric:
          v_curv_speed = int(np.interp(modelSpeed, self.vision_curv_speed_cmph, self.vision_curv_speed_tmph)/2)*2
        else:
          v_curv_speed = int(np.interp(modelSpeed, self.vision_curv_speed_c, self.vision_curv_speed_t)/3)*3
        v_curv_speed = min(var_speed, v_curv_speed) # curve speed ratio
      else:
        v_curv_speed = 255
    else:
      v_curv_speed = 255

    if CS.out.cruiseState.modeSel in (1,3,4) and self.curv_decel_option in (1,3):
      if osm_data.turnSpeedLimitEndDistance > 30:
        o_curv_speed = int(np.interp(osm_data.turnSpeedLimit, self.osm_curv_speed_c, self.osm_curv_speed_t))
        self.osm_wait_timer += 1 if modelSpeed > (self.vision_curv_speed_cmph[-1] if not CS.is_metric else self.vision_curv_speed_c[-1]) else 0
        if self.osm_wait_timer > 100:
          o_curv_speed = 255
      else:
        o_curv_speed = 255
        self.osm_wait_timer = 0
    else:
      o_curv_speed = 255
      self.osm_wait_timer = 0

    # self.gasPressed_old = CS.gasPressed
    if var_speed > round(min(v_curv_speed, o_curv_speed)):
      v_ego_kph_or_mph = CS.out.vEgo * CV.MS_TO_MPH if not CS.is_metric else CS.out.vEgo * CV.MS_TO_KPH
      if round(min(v_curv_speed, o_curv_speed))+1 < v_ego_kph_or_mph and not CS.out.gasPressed:
        self.curvSpeedControl = True
      else:
        self.curvSpeedControl = False
    else:
      self.curvSpeedControl = False

    return round(min(var_speed, v_curv_speed, o_curv_speed))

  def get_live_gap(self, CS):
    self.t_interval = randint(self.t_interval2+3, self.t_interval2+5) if not CS.is_metric else randint(self.t_interval2, self.t_interval2+2)
    gap_to_set = CS.DistSet if CS.DistSet > 0 else CS.cruiseGapSet
    if 0 < CS.lead_distance <= 149 and CS.lead_objspd < -4 and CS.clu_Vanz > 30 and 0 < self.e2e_x < 120 and self.try_early_stop:
      if not self.try_early_stop_retrieve:
        self.try_early_stop_org_gap = CS.DistSet if CS.DistSet > 0 else CS.cruiseGapSet
        self.try_early_stop_retrieve = True
      if self.try_early_stop_retrieve:
        self.cruise_gap_adjusting = True
        gap_to_set = 4.0
        return gap_to_set
    elif 0 < CS.lead_distance <= 149 and self.try_early_stop_retrieve and self.try_early_stop_org_gap and CS.DistSet != self.try_early_stop_org_gap and \
      (CS.clu_Vanz <= 20 or (CS.lead_objspd >= 0 and self.e2e_x > 50 and CS.clu_Vanz > 20)) and self.try_early_stop:
      self.cruise_gap_adjusting = True
      gap_to_set = self.try_early_stop_org_gap
      return gap_to_set
    elif self.try_early_stop_retrieve and CS.DistSet == self.try_early_stop_org_gap and self.try_early_stop:
      self.try_early_stop_retrieve = False
      self.try_early_stop_org_gap = 0
      self.cruise_gap_adjusting = False
      gap_to_set = CS.DistSet if CS.DistSet > 0 else CS.cruiseGapSet
      return gap_to_set
    else:
      self.cruise_gap_adjusting = False

    return gap_to_set

  def update(self, CS, sm):
    btn_signal = None
    if not self.button_status(CS):  # 사용자가 버튼클릭하면 일정시간 기다린다.
      pass
    elif CS.acc_active:
      car_state = sm['carState']
      controls_state = sm['controlsState']
      selfdrive_state = sm['selfdriveState']
      radar_state = sm['radarState']
      lat_plan = sm['lateralPlan']
      long_plan = sm['longitudinalPlan']
      navi_data = sm['liveENaviData']
      osm_data = sm['liveMapData']

      cruiseState_speed = round(car_state.vCruise)
      if not CS.out.cruiseState.enabled and navi_data.roadLimitSpeed > 21:
        cruiseState_speed = navi_data.roadLimitSpeed + self.cruise_road_limitspd_offset
      if CS.CP.carFingerprint in CANFD_CAR:
        self.is_canfd = True
        self.ctrl_gap = self.get_live_gap(CS)
      kph_set_vEgo = self.get_navi_speed(CS, navi_data, osm_data, car_state, cruiseState_speed) # camspeed
      if self.osm_speedlimit_enabled:
        navi_speed = kph_set_vEgo
      else:
        navi_speed = min(cruiseState_speed, kph_set_vEgo)
      self.safetycam_speed = navi_speed
      if CS.out.cruiseState.modeSel == 0:
        self.ctrl_speed = cruiseState_speed
      elif CS.out.cruiseState.modeSel != 5:
        self.ctrl_speed = self.auto_speed_control(CS, controls_state, selfdrive_state, radar_state, lat_plan, long_plan, navi_speed, osm_data)
      else:
        self.ctrl_speed = navi_speed

      btn_signal = self.ascc_button_control(CS, self.ctrl_speed, self.ctrl_gap)

    return btn_signal
