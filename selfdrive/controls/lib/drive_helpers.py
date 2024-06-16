import math

from cereal import car, log
import cereal.messaging as messaging
from openpilot.common.conversions import Conversions as CV
from openpilot.common.numpy_fast import clip, interp
from openpilot.common.realtime import DT_CTRL, DT_MDL
from openpilot.selfdrive.modeld.constants import ModelConstants

from openpilot.selfdrive.car.hyundai.values import Buttons
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
IMPERIAL_INCREMENT = 1.6  # should be CV.MPH_TO_KPH, but this causes rounding errors

MIN_SPEED = 1.0
CONTROL_N = 17
CAR_ROTATION_RADIUS = 0.0

# EU guidelines
MAX_LATERAL_JERK = 5.0
MAX_VEL_ERR = 5.0

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

    self.setspdfive = self.params.get_bool("SetSpeedFive") 

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
        else:
          t_speed = 30 if self.is_kph else 20
          if self.cruise_road_limit_spd_enabled and not self.cruise_road_limit_spd_switch and self.cruise_road_limit_spd_switch_prev != 0 and self.cruise_road_limit_spd_switch_prev != self.sm['liveENaviData'].roadLimitSpeed:
            self.cruise_road_limit_spd_switch = True
            self.cruise_road_limit_spd_switch_prev = 0
          if self.variable_cruise and CS.cruiseState.modeSel != 0 and self.sm['controlsState'].autoResvCruisekph > t_speed:
            self.v_cruise_kph = self.sm['controlsState'].autoResvCruisekph
            self.v_cruise_kph_last = self.v_cruise_kph
            self.v_cruise_cluster_kph = self.v_cruise_kph
          elif CS.cruiseButtons == Buttons.RES_ACCEL or CS.cruiseButtons == Buttons.SET_DECEL or (CS.cruiseState.accActive and CS.cruiseButtons == 0 and not self.first_acc):
            if CS.cruiseState.accActive and CS.cruiseButtons == 0:
              self.first_acc = True
            if self.cruise_road_limit_spd_enabled and CS.cruiseButtons == Buttons.RES_ACCEL:
              self.cruise_road_limit_spd_switch_prev = self.sm['liveENaviData'].roadLimitSpeed
              self.cruise_road_limit_spd_switch = False
            elif self.cruise_road_limit_spd_enabled and (CS.cruiseButtons == Buttons.SET_DECEL or self.first_acc):
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
          elif CS.driverAcc and self.variable_cruise and (self.cruise_over_maxspeed or self.cruise_road_limit_spd_enabled) and t_speed <= self.v_cruise_kph < round(CS.vEgo*m_unit):
            self.cruise_road_limit_spd_switch_prev = self.sm['liveENaviData'].roadLimitSpeed
            self.cruise_road_limit_spd_switch = False
            self.v_cruise_kph = round(CS.vEgo*m_unit)
            self.v_cruise_cluster_kph = self.v_cruise_kph
            self.v_cruise_kph_last = self.v_cruise_kph
          elif self.variable_cruise and self.cruise_road_limit_spd_enabled and int(self.v_cruise_kph) != (int(self.sm['liveENaviData'].roadLimitSpeed) + self.cruise_road_limit_spd_offset) and \
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
      for k in self.button_timers.keys():
        if self.button_timers[k] and self.button_timers[k] % CRUISE_LONG_PRESS == 0:
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

    # 250kph or above probably means we never had a set speed
    if any(b.type in (ButtonType.accelCruise, ButtonType.resumeCruise) for b in CS.buttonEvents) and self.v_cruise_kph_last < 250:
      self.v_cruise_kph = self.v_cruise_kph_last
    else:
      self.v_cruise_kph = int(round(clip(CS.vEgo * CV.MS_TO_KPH, initial, V_CRUISE_MAX)))

    self.v_cruise_cluster_kph = self.v_cruise_kph


def apply_center_deadzone(error, deadzone):
  if (error > - deadzone) and (error < deadzone):
    error = 0.
  return error


def rate_limit(new_value, last_value, dw_step, up_step):
  return clip(new_value, last_value + dw_step, last_value + up_step)


def clip_curvature(v_ego, prev_curvature, new_curvature):
  v_ego = max(MIN_SPEED, v_ego)
  max_curvature_rate = MAX_LATERAL_JERK / (v_ego**2) # inexact calculation, check https://github.com/commaai/openpilot/pull/24755
  safe_desired_curvature = clip(new_curvature,
                                prev_curvature - max_curvature_rate * DT_CTRL,
                                prev_curvature + max_curvature_rate * DT_CTRL)

  return safe_desired_curvature

def get_lag_adjusted_curvature(CP, v_ego, psis, curvatures, curvature_rates):
  if len(psis) != CONTROL_N:
    psis = [0.0]*CONTROL_N
    curvatures = [0.0]*CONTROL_N
    curvature_rates = [0.0]*CONTROL_N
  v_ego = max(MIN_SPEED, v_ego)

  # TODO this needs more thought, use .2s extra for now to estimate other delays
  delay = max(0.01, CP.steerActuatorDelay + .2)

  # MPC can plan to turn the wheel and turn back before t_delay. This means
  # in high delay cases some corrections never even get commanded. So just use
  # psi to calculate a simple linearization of desired curvature
  current_curvature_desired = curvatures[0]
  psi = interp(delay, ModelConstants.T_IDXS[:CONTROL_N], psis)
  average_curvature_desired = psi / (v_ego * delay)
  desired_curvature = 2 * average_curvature_desired - current_curvature_desired

  # This is the "desired rate of the setpoint" not an actual desired rate
  desired_curvature_rate = curvature_rates[0]
  max_curvature_rate = MAX_LATERAL_JERK / (v_ego**2) # inexact calculation, check https://github.com/commaai/openpilot/pull/24755
  safe_desired_curvature_rate = clip(desired_curvature_rate,
                                     -max_curvature_rate,
                                     max_curvature_rate)
  safe_desired_curvature = clip(desired_curvature,
                                current_curvature_desired - max_curvature_rate * DT_MDL,
                                current_curvature_desired + max_curvature_rate * DT_MDL)

  return safe_desired_curvature, safe_desired_curvature_rate


def get_friction(lateral_accel_error: float, lateral_accel_deadzone: float, friction_threshold: float,
                 torque_params: car.CarParams.LateralTorqueTuning, friction_compensation: bool) -> float:
  friction_interp = interp(
    apply_center_deadzone(lateral_accel_error, lateral_accel_deadzone),
    [-friction_threshold, friction_threshold],
    [-torque_params.friction, torque_params.friction]
  )
  friction = float(friction_interp) if friction_compensation else 0.0
  return friction


def get_speed_error(modelV2: log.ModelDataV2, v_ego: float) -> float:
  # ToDo: Try relative error, and absolute speed
  if len(modelV2.temporalPose.trans):
    vel_err = clip(modelV2.temporalPose.trans[0] - v_ego, -MAX_VEL_ERR, MAX_VEL_ERR)
    return float(vel_err)
  return 0.0
