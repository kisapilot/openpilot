from cereal import car, log
from openpilot.common.numpy_fast import clip, interp
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N
from openpilot.selfdrive.controls.lib.pid import PIDController
from openpilot.selfdrive.modeld.constants import ModelConstants

CONTROL_N_T_IDX = ModelConstants.T_IDXS[:CONTROL_N]

LongCtrlState = car.CarControl.Actuators.LongControlState
from openpilot.common.conversions import Conversions as CV
from openpilot.common.params import Params
from decimal import Decimal

import openpilot.common.log as trace1
LongitudinalPlanSource = log.LongitudinalPlan.LongitudinalPlanSource

def long_control_state_trans(CP, active, long_control_state, v_ego,
                             should_stop, brake_pressed, cruise_standstill):
  stopping_condition = should_stop
  starting_condition = (not should_stop and
                        not cruise_standstill and
                        not brake_pressed)
  started_condition = v_ego > CP.vEgoStarting

  if not active:
    long_control_state = LongCtrlState.off

  else:
    if long_control_state in (LongCtrlState.off, LongCtrlState.pid):
      long_control_state = LongCtrlState.pid
      if stopping_condition:
        long_control_state = LongCtrlState.stopping
    elif long_control_state == LongCtrlState.stopping:
      if starting_condition and CP.startingState:
        long_control_state = LongCtrlState.starting
      elif starting_condition:
        long_control_state = LongCtrlState.pid

    elif long_control_state == LongCtrlState.starting:
      if stopping_condition:
        long_control_state = LongCtrlState.stopping
      elif started_condition:
        long_control_state = LongCtrlState.pid

  return long_control_state

class LongControl:
  def __init__(self, CP):
    self.CP = CP
    self.long_control_state = LongCtrlState.off
    self.pid = PIDController((CP.longitudinalTuning.kpBP, CP.longitudinalTuning.kpV),
                             (CP.longitudinalTuning.kiBP, CP.longitudinalTuning.kiV),
                             k_f=CP.longitudinalTuning.kf, rate=1 / DT_CTRL)
    self.last_output_accel = 0.0

    self.long_stat = ""
    self.long_plan_source = ""

    self.long_log = Params().get_bool("LongLogDisplay")
    self.stopping_dist = float(Decimal(Params().get("StoppingDist", encoding="utf8"))*Decimal('0.1'))

    self.loc_timer = 0

  def reset(self):
    self.pid.reset()

  def update(self, active, CS, a_target, should_stop, accel_limits, long_plan_source, CO, radarState):
    self.loc_timer += 1
    if self.loc_timer > 100:
      self.loc_timer = 0
      self.long_log = Params().get_bool("LongLogDisplay")

    """Update longitudinal control. This updates the state machine and runs a PID loop"""
    self.pid.neg_limit = accel_limits[0]
    self.pid.pos_limit = accel_limits[1]

    output_accel = self.last_output_accel

    if radarState is None:
      dRel = 150
      vRel = 0
    else:
      dRel = radarState.leadOne.dRel
      vRel = radarState.leadOne.vRel

    self.long_control_state = long_control_state_trans(self.CP, active, self.long_control_state, CS.vEgo,
                                                       should_stop, CS.brakePressed,
                                                       CS.cruiseState.standstill)
    if self.long_control_state == LongCtrlState.off:
      self.reset()
      output_accel = 0.

    elif self.long_control_state == LongCtrlState.stopping:
      output_accel = self.last_output_accel
      if output_accel > self.CP.stopAccel:
        output_accel = min(output_accel, 0.0)
        output_accel -= self.CP.stoppingDecelRate * DT_CTRL
      self.reset()

    elif self.long_control_state == LongCtrlState.starting:
      output_accel = self.CP.startAccel
      self.reset()

    else:  # LongCtrlState.pid
      error = a_target - CS.aEgo
      output_accel = self.pid.update(error, speed=CS.vEgo,
                                     feedforward=a_target)

    self.last_output_accel = clip(output_accel, accel_limits[0], accel_limits[1])

    if self.long_control_state == LongCtrlState.stopping:
      self.long_stat = "STP"
    elif self.long_control_state == LongCtrlState.starting:
      self.long_stat = "STR"
    elif self.long_control_state == LongCtrlState.pid:
      self.long_stat = "PID"
    elif self.long_control_state == LongCtrlState.off:
      self.long_stat = "OFF"
    else:
      self.long_stat = "---"

    if long_plan_source == LongitudinalPlanSource.lead0:
      self.long_plan_source = "lead0"
    elif long_plan_source == LongitudinalPlanSource.lead1:
      self.long_plan_source = "lead1"
    elif long_plan_source == LongitudinalPlanSource.lead2:
      self.long_plan_source = "lead2"
    elif long_plan_source == LongitudinalPlanSource.cruise:
      self.long_plan_source = "cruise"
    elif long_plan_source == LongitudinalPlanSource.e2e:
      self.long_plan_source = "e2e"
    elif long_plan_source == LongitudinalPlanSource.stop:
      self.long_plan_source = "stop"
    else:
      self.long_plan_source = "---"

    if self.long_log:
      str_log3 = 'LS={:s}  LP={:s}  AQ/AR/AT/FA={:+04.2f}/{:+04.2f}/{:+04.2f}/{:+04.2f}  GB={}  ED/RD={:04.1f}/{:04.1f}  TG={:03.0f}/{:03.0f}'.format(self.long_stat, \
       self.long_plan_source, CO.aqValue, CO.aqValueRaw, a_target, self.last_output_accel, int(CS.gasPressed or CS.brakePressed), dRel, CS.radarDRel, \
       (v_target*CV.MS_TO_MPH) if CS.isMph else (v_target*CV.MS_TO_KPH), (v_target_1sec*CV.MS_TO_MPH) if CS.isMph else (v_target_1sec*CV.MS_TO_KPH))
      trace1.printf3('{}'.format(str_log3))

    return self.last_output_accel, a_target
