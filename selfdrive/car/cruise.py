import math

from cereal import car
from openpilot.common.conversions import Conversions as CV
from openpilot.common.numpy_fast import clip, interp

import cereal.messaging as messaging
from openpilot.common.realtime import DT_MDL
from opendbc.car.hyundai.values import Buttons
from openpilot.common.params import Params

IS_METRIC = Params().get_bool("IsMetric") if Params().get_bool("IsMetric") is not None else False
LONG_ENABLED = Params().get_bool("ExperimentalLongitudinalEnabled") if Params().get_bool("ExperimentalLongitudinalEnabled") is not None else False
# WARNING: this value was determined based on the model's training distribution,
#          model predictions above this speed can be unpredictable
# V_CRUISE's are in kph
V_CRUISE_MIN = (10 if IS_METRIC else 5) if LONG_ENABLED else (30 if IS_METRIC else 20)
V_CRUISE_MAX = 160
V_CRUISE_UNSET = 255
V_CRUISE_INITIAL = V_CRUISE_MIN = (10 if IS_METRIC else 5) if LONG_ENABLED else (30 if IS_METRIC else 20)
V_CRUISE_INITIAL_EXPERIMENTAL_MODE = 105
IMPERIAL_INCREMENT = round(CV.MPH_TO_KPH, 1)  # round here to avoid rounding errors incrementing set speed

ButtonEvent = car.CarState.ButtonEvent
ButtonType = car.CarState.ButtonEvent.Type
CRUISE_LONG_PRESS = 50
CRUISE_NEAREST_FUNC = {
  ButtonType.accelCruise: math.ceil,
  ButtonType.decelCruise: math.floor,
}
CRUISE_INTERVAL_SIGN = {
  ButtonType.accelCruise: +1,
  ButtonType.decelCruise: -1,
}


class VCruiseHelper:
  def __init__(self, CP):
    self.CP = CP
    self.v_cruise_kph = V_CRUISE_UNSET
    self.v_cruise_cluster_kph = V_CRUISE_UNSET
    self.v_cruise_kph_last = 0
    self.button_timers = {ButtonType.decelCruise: 0, ButtonType.accelCruise: 0}
    self.button_change_states = {btn: {"standstill": False, "enabled": False} for btn in self.button_timers}

    self.sm = messaging.SubMaster(['controlsState', 'liveENaviData', 'liveMapData'])

    self.params = Params()
    self.is_kph = self.params.get_bool("IsMetric")
    self.variable_cruise = self.params.get_bool("KisaVariableCruise")

    self.osm_waze_spdlimit_offset = int(self.params.get("KisaSpeedLimitOffset", encoding="utf8"))
    self.osm_waze_spdlimit_offset_option = int(self.params.get("KisaSpeedLimitOffsetOption", encoding="utf8"))
    self.osm_speedlimit_enabled = self.params.get_bool("OSMSpeedLimitEnable")
    self.osm_waze_speedlimit = 255
    self.pause_spdlimit = False
    self.osm_waze_off_spdlimit_init = False

    self.navi_selection = int(self.params.get("KISANaviSelect", encoding="utf8"))

    self.osm_waze_custom_spdlimit_c = list(map(int, self.params.get("OSMCustomSpeedLimitC", encoding="utf8").split(',')))
    self.osm_waze_custom_spdlimit_t = list(map(int, self.params.get("OSMCustomSpeedLimitT", encoding="utf8").split(',')))

    self.pause_spdlimit_push = False
    self.pause_spdlimit_push_cnt = 0

    self.second2 = 0.0

    self.cruise_over_maxspeed = self.params.get_bool("CruiseOverMaxSpeed")
    self.cruise_road_limit_spd_enabled = self.params.get_bool("CruiseSetwithRoadLimitSpeedEnabled")
    self.cruise_road_limit_spd_offset = int(self.params.get("CruiseSetwithRoadLimitSpeedOffset", encoding="utf8"))

    self.cruise_road_limit_spd_switch = True
    self.cruise_road_limit_spd_switch_prev = 0

    self.first_acc = False

  @property
  def v_cruise_initialized(self):
    return self.v_cruise_kph != V_CRUISE_UNSET

  def update_v_cruise(self, CS, enabled, is_metric):
    self.v_cruise_kph_last = self.v_cruise_kph
    self.sm.update(0)
    if CS.cruiseState.available:
      m_unit = CV.MS_TO_KPH if self.is_kph else CV.MS_TO_MPH
      if not self.CP.pcmCruise:
        if self.CP.carName == "hyundai":
          self.v_cruise_kph = int(round(CS.cruiseState.speed * m_unit))
          self.v_cruise_cluster_kph = int(round(CS.cruiseState.speedCluster * m_unit))
        else:
          # if stock cruise is completely disabled, then we can use our own set speed logic
          self._update_v_cruise_non_pcm(CS, enabled, is_metric)
          self.v_cruise_cluster_kph = self.v_cruise_kph
          self.update_button_timers(CS, enabled)
      else:
        if not self.CP.carName == "hyundai":
          self.v_cruise_kph = int(round(CS.cruiseState.speed * m_unit))
          self.v_cruise_cluster_kph = int(round(CS.cruiseState.speedCluster * m_unit))
          if CS.cruiseState.speed == 0:
            self.v_cruise_kph = V_CRUISE_UNSET
            self.v_cruise_cluster_kph = V_CRUISE_UNSET
        else:
          t_speed = 30 if self.is_kph else 20
          if (self.cruise_road_limit_spd_enabled and self.sm['controlsState'].roadLimitSpeedOnTemp) and not self.cruise_road_limit_spd_switch and self.cruise_road_limit_spd_switch_prev != 0 and self.cruise_road_limit_spd_switch_prev != self.sm['liveENaviData'].roadLimitSpeed:
            self.cruise_road_limit_spd_switch = True
            self.cruise_road_limit_spd_switch_prev = 0
          if self.variable_cruise and CS.cruiseState.modeSel != 0 and self.sm['controlsState'].autoResvCruisekph > t_speed:
            self.v_cruise_kph = self.sm['controlsState'].autoResvCruisekph
            self.v_cruise_kph_last = self.v_cruise_kph
            self.v_cruise_cluster_kph = self.v_cruise_kph
          elif CS.cruiseButtons == Buttons.RES_ACCEL or CS.cruiseButtons == Buttons.SET_DECEL or (CS.cruiseState.accActive and CS.cruiseButtons == 0 and not self.first_acc):
            if CS.cruiseState.accActive and CS.cruiseButtons == 0:
              self.first_acc = True
            if (self.cruise_road_limit_spd_enabled and self.sm['controlsState'].roadLimitSpeedOnTemp) and CS.cruiseButtons == Buttons.RES_ACCEL:
              self.cruise_road_limit_spd_switch_prev = self.sm['liveENaviData'].roadLimitSpeed
              self.cruise_road_limit_spd_switch = False
            elif (self.cruise_road_limit_spd_enabled and self.sm['controlsState'].roadLimitSpeedOnTemp) and (CS.cruiseButtons == Buttons.SET_DECEL or self.first_acc):
              if 1 < int(self.sm['liveENaviData'].roadLimitSpeed) < 150:
                self.cruise_road_limit_spd_switch = True
              else:
                self.cruise_road_limit_spd_switch = False
            self.v_cruise_kph = round(CS.cruiseState.speed * m_unit)
            self.v_cruise_cluster_kph = self.v_cruise_kph
            self.v_cruise_kph_last = self.v_cruise_kph
            if self.osm_speedlimit_enabled or self.navi_selection == 2:
              self.osm_waze_off_spdlimit_init = True
              if self.navi_selection == 2:
                self.osm_waze_speedlimit = round(self.sm['liveENaviData'].wazeRoadSpeedLimit)
              elif self.osm_speedlimit_enabled:
                self.osm_waze_speedlimit = round(self.sm['liveMapData'].speedLimit)
          elif self.first_acc and not CS.cruiseState.available:
            self.first_acc = False
          elif CS.driverAcc and self.variable_cruise and (self.cruise_over_maxspeed or (self.cruise_road_limit_spd_enabled and self.sm['controlsState'].roadLimitSpeedOnTemp)) and t_speed <= self.v_cruise_kph < round(CS.vEgo*m_unit):
            self.cruise_road_limit_spd_switch_prev = self.sm['liveENaviData'].roadLimitSpeed
            self.cruise_road_limit_spd_switch = False
            self.v_cruise_kph = round(CS.vEgo*m_unit)
            self.v_cruise_cluster_kph = self.v_cruise_kph
            self.v_cruise_kph_last = self.v_cruise_kph
          elif self.variable_cruise and (self.cruise_road_limit_spd_enabled and self.sm['controlsState'].roadLimitSpeedOnTemp) and int(self.v_cruise_kph) != (int(self.sm['liveENaviData'].roadLimitSpeed) + self.cruise_road_limit_spd_offset) and \
           29 < int(self.sm['liveENaviData'].roadLimitSpeed) < 150 and self.cruise_road_limit_spd_switch:
            self.v_cruise_kph = int(self.sm['liveENaviData'].roadLimitSpeed) + self.cruise_road_limit_spd_offset
            self.v_cruise_cluster_kph = self.v_cruise_kph
            self.v_cruise_kph_last = self.v_cruise_kph
          elif self.variable_cruise and CS.cruiseState.modeSel != 0 and (self.osm_speedlimit_enabled or self.navi_selection == 2) and self.osm_waze_off_spdlimit_init:
            if self.navi_selection == 2:
              osm_waze_speedlimit_ = round(self.sm['liveENaviData'].wazeRoadSpeedLimit)
              osm_waze_speedlimitdist_ = round(self.sm['liveENaviData'].wazeAlertDistance)
            elif self.osm_speedlimit_enabled:
              osm_waze_speedlimit_ = round(self.sm['liveMapData'].speedLimit)
              osm_waze_speedlimitdist_ = 0
            else:
              osm_waze_speedlimit_ = round(self.sm['liveMapData'].speedLimit)
              osm_waze_speedlimitdist_ = 0
            if self.osm_waze_spdlimit_offset_option == 0:
              osm_waze_speedlimit = osm_waze_speedlimit_ + round(osm_waze_speedlimit_*0.01*self.osm_waze_spdlimit_offset)
            elif self.osm_waze_spdlimit_offset_option == 1:
              osm_waze_speedlimit = osm_waze_speedlimit_ + self.osm_waze_spdlimit_offset
            elif self.osm_waze_spdlimit_offset_option in (2,3):
              osm_waze_speedlimit = int(interp(osm_waze_speedlimit_, self.osm_waze_custom_spdlimit_c, self.osm_waze_custom_spdlimit_t))
            if CS.cruiseButtons == Buttons.GAP_DIST:
              self.osm_waze_speedlimit = 255
              self.pause_spdlimit = False
            elif osm_waze_speedlimitdist_ > 0:
              self.pause_spdlimit = False
            elif self.osm_waze_speedlimit == osm_waze_speedlimit_:
              self.pause_spdlimit = True
            elif osm_waze_speedlimit != self.v_cruise_kph:
              if self.navi_selection == 2 and self.sm['liveENaviData'].wazeRoadSpeedLimit > 9:
                self.v_cruise_kph = osm_waze_speedlimit
                self.v_cruise_kph_last = self.v_cruise_kph
                self.v_cruise_cluster_kph = self.v_cruise_kph
              elif self.osm_speedlimit_enabled and self.sm['liveMapData'].speedLimit > 9:
                self.v_cruise_kph = osm_waze_speedlimit
                self.v_cruise_kph_last = self.v_cruise_kph
          elif self.variable_cruise and CS.cruiseState.modeSel != 0 and not (self.osm_speedlimit_enabled or self.navi_selection == 2):
            if self.sm['liveENaviData'].safetyDistance > 600: # temporary pause to limit spd in safety section
              self.second2 += DT_MDL
              if CS.cruiseButtons == Buttons.GAP_DIST: # push gap 3 times quickly, this is toggle.
                self.pause_spdlimit_push = True
                self.second2 = 0.0
              elif self.pause_spdlimit_push:
                self.pause_spdlimit_push = False
                self.pause_spdlimit_push_cnt += 1
              elif self.pause_spdlimit_push_cnt == 3 and self.second2 > 0.5:
                self.pause_spdlimit_push_cnt = 0
                self.pause_spdlimit = not self.pause_spdlimit
              elif self.second2 > 0.5 and self.pause_spdlimit_push_cnt > 0:
                self.pause_spdlimit_push_cnt = 0
            else:
              self.second2 = 0.0
              self.pause_spdlimit_push = False
              self.pause_spdlimit_push_cnt = 0
              self.pause_spdlimit = False
    else:
      if not self.CP.carName == "hyundai":
        self.v_cruise_kph = V_CRUISE_UNSET
        self.v_cruise_cluster_kph = V_CRUISE_UNSET
      else:
        # to display maxspeed synced as roadspeedlimit on scc standby
        if self.variable_cruise and CS.cruiseState.modeSel != 0 and (self.osm_speedlimit_enabled or self.navi_selection == 2):
          if self.navi_selection == 2:
            osm_waze_speedlimit_ = round(self.sm['liveENaviData'].wazeRoadSpeedLimit)
          elif self.osm_speedlimit_enabled:
            osm_waze_speedlimit_ = round(self.sm['liveMapData'].speedLimit)
          else:
            osm_waze_speedlimit_ = round(self.sm['liveMapData'].speedLimit)
          if self.osm_waze_spdlimit_offset_option == 0:
            osm_waze_speedlimit = osm_waze_speedlimit_ + round(osm_waze_speedlimit_*0.01*self.osm_waze_spdlimit_offset)
          elif self.osm_waze_spdlimit_offset_option == 1:
            osm_waze_speedlimit = osm_waze_speedlimit_ + self.osm_waze_spdlimit_offset
          elif self.osm_waze_spdlimit_offset_option in (2,3):
            osm_waze_speedlimit = int(interp(osm_waze_speedlimit_, self.osm_waze_custom_spdlimit_c, self.osm_waze_custom_spdlimit_t))
          if osm_waze_speedlimit != self.v_cruise_kph:
            if self.navi_selection == 2 and self.sm['liveENaviData'].wazeRoadSpeedLimit > 9:
              self.v_cruise_kph = osm_waze_speedlimit
              self.v_cruise_kph_last = self.v_cruise_kph
              self.v_cruise_cluster_kph = self.v_cruise_kph
            elif self.osm_speedlimit_enabled and self.sm['liveMapData'].speedLimit > 9:
              self.v_cruise_kph = osm_waze_speedlimit
              self.v_cruise_kph_last = self.v_cruise_kph
              self.v_cruise_cluster_kph = self.v_cruise_kph

  def _update_v_cruise_non_pcm(self, CS, enabled, is_metric):
    # handle button presses. TODO: this should be in state_control, but a decelCruise press
    # would have the effect of both enabling and changing speed is checked after the state transition
    if not enabled:
      return

    long_press = False
    button_type = None

    v_cruise_delta = 1. if is_metric else IMPERIAL_INCREMENT

    for b in CS.buttonEvents:
      if b.type.raw in self.button_timers and not b.pressed:
        if self.button_timers[b.type.raw] > CRUISE_LONG_PRESS:
          return  # end long press
        button_type = b.type.raw
        break
    else:
      for k, timer in self.button_timers.items():
        if timer and timer % CRUISE_LONG_PRESS == 0:
          button_type = k
          long_press = True
          break

    if button_type is None:
      return

    # Don't adjust speed when pressing resume to exit standstill
    cruise_standstill = self.button_change_states[button_type]["standstill"] or CS.cruiseState.standstill
    if button_type == ButtonType.accelCruise and cruise_standstill:
      return

    # Don't adjust speed if we've enabled since the button was depressed (some ports enable on rising edge)
    if not self.button_change_states[button_type]["enabled"]:
      return

    v_cruise_delta = v_cruise_delta * (5 if long_press else 1)
    if long_press and self.v_cruise_kph % v_cruise_delta != 0:  # partial interval
      self.v_cruise_kph = CRUISE_NEAREST_FUNC[button_type](self.v_cruise_kph / v_cruise_delta) * v_cruise_delta
    else:
      self.v_cruise_kph += v_cruise_delta * CRUISE_INTERVAL_SIGN[button_type]

    # If set is pressed while overriding, clip cruise speed to minimum of vEgo
    if CS.gasPressed and button_type in (ButtonType.decelCruise, ButtonType.setCruise):
      self.v_cruise_kph = max(self.v_cruise_kph, CS.vEgo * CV.MS_TO_KPH)

    self.v_cruise_kph = clip(round(self.v_cruise_kph, 1), V_CRUISE_MIN, V_CRUISE_MAX)

  def update_button_timers(self, CS, enabled):
    # increment timer for buttons still pressed
    for k in self.button_timers:
      if self.button_timers[k] > 0:
        self.button_timers[k] += 1

    for b in CS.buttonEvents:
      if b.type.raw in self.button_timers:
        # Start/end timer and store current state on change of button pressed
        self.button_timers[b.type.raw] = 1 if b.pressed else 0
        self.button_change_states[b.type.raw] = {"standstill": CS.cruiseState.standstill, "enabled": enabled}

  def initialize_v_cruise(self, CS, experimental_mode: bool) -> None:
    # initializing is handled by the PCM
    if self.CP.pcmCruise:
      return

    initial = V_CRUISE_INITIAL_EXPERIMENTAL_MODE if experimental_mode else V_CRUISE_INITIAL

    if any(b.type in (ButtonType.accelCruise, ButtonType.resumeCruise) for b in CS.buttonEvents) and self.v_cruise_initialized:
      self.v_cruise_kph = self.v_cruise_kph_last
    else:
      self.v_cruise_kph = int(round(clip(CS.vEgo * CV.MS_TO_KPH, initial, V_CRUISE_MAX)))

    self.v_cruise_cluster_kph = self.v_cruise_kph
