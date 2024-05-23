import numpy as np
from cereal import log
from openpilot.common.conversions import Conversions as CV
from openpilot.common.realtime import DT_MDL

from openpilot.common.numpy_fast import interp
from openpilot.common.params import Params
from decimal import Decimal

from openpilot.common.params import Params

USE_LEGACY_LANE_MODEL = int(Params().get("UseLegacyLaneModel", encoding="utf8")) if Params().get("UseLegacyLaneModel", encoding="utf8") is not None else 0

if USE_LEGACY_LANE_MODEL:
  LaneChangeState = log.LateralPlan.LaneChangeState
  LaneChangeDirection = log.LateralPlan.LaneChangeDirection
else:
  LaneChangeState = log.LaneChangeState
  LaneChangeDirection = log.LaneChangeDirection

if int(Params().get("KisaLaneChangeSpeed", encoding="utf8")) < 1:
  LANE_CHANGE_SPEED_MIN = -1
elif Params().get_bool("IsMetric"):
  LANE_CHANGE_SPEED_MIN = float(int(Params().get("KisaLaneChangeSpeed", encoding="utf8")) * CV.KPH_TO_MS)
else:
  LANE_CHANGE_SPEED_MIN = float(int(Params().get("KisaLaneChangeSpeed", encoding="utf8")) * CV.MPH_TO_MS)
LANE_CHANGE_TIME_MAX = 10.

if USE_LEGACY_LANE_MODEL:
  DESIRES = {
    LaneChangeDirection.none: {
      LaneChangeState.off: log.LateralPlan.Desire.none,
      LaneChangeState.preLaneChange: log.LateralPlan.Desire.none,
      LaneChangeState.laneChangeStarting: log.LateralPlan.Desire.none,
      LaneChangeState.laneChangeFinishing: log.LateralPlan.Desire.none,
    },
    LaneChangeDirection.left: {
      LaneChangeState.off: log.LateralPlan.Desire.none,
      LaneChangeState.preLaneChange: log.LateralPlan.Desire.none,
      LaneChangeState.laneChangeStarting: log.LateralPlan.Desire.laneChangeLeft,
      LaneChangeState.laneChangeFinishing: log.LateralPlan.Desire.laneChangeLeft,
    },
    LaneChangeDirection.right: {
      LaneChangeState.off: log.LateralPlan.Desire.none,
      LaneChangeState.preLaneChange: log.LateralPlan.Desire.none,
      LaneChangeState.laneChangeStarting: log.LateralPlan.Desire.laneChangeRight,
      LaneChangeState.laneChangeFinishing: log.LateralPlan.Desire.laneChangeRight,
    },
  }
else:
  DESIRES = {
    LaneChangeDirection.none: {
      LaneChangeState.off: log.Desire.none,
      LaneChangeState.preLaneChange: log.Desire.none,
      LaneChangeState.laneChangeStarting: log.Desire.none,
      LaneChangeState.laneChangeFinishing: log.Desire.none,
    },
    LaneChangeDirection.left: {
      LaneChangeState.off: log.Desire.none,
      LaneChangeState.preLaneChange: log.Desire.none,
      LaneChangeState.laneChangeStarting: log.Desire.laneChangeLeft,
      LaneChangeState.laneChangeFinishing: log.Desire.laneChangeLeft,
    },
    LaneChangeDirection.right: {
      LaneChangeState.off: log.Desire.none,
      LaneChangeState.preLaneChange: log.Desire.none,
      LaneChangeState.laneChangeStarting: log.Desire.laneChangeRight,
      LaneChangeState.laneChangeFinishing: log.Desire.laneChangeRight,
    },
  }


class DesireHelper:
  def __init__(self):
    self.lane_change_state = LaneChangeState.off
    self.lane_change_direction = LaneChangeDirection.none
    self.lane_change_timer = 0.0
    self.lane_change_ll_prob = 1.0
    self.keep_pulse_timer = 0.0
    self.prev_one_blinker = False
    self.desire = log.LateralPlan.Desire.none if USE_LEGACY_LANE_MODEL else log.Desire.none

    self.lane_change_delay = int(Params().get("KisaAutoLaneChangeDelay", encoding="utf8"))
    self.lane_change_auto_delay = 0.0 if self.lane_change_delay == 0 else 0.2 if self.lane_change_delay == 1 else 0.5 if self.lane_change_delay == 2 \
     else 1.0 if self.lane_change_delay == 3 else 1.5 if self.lane_change_delay == 4 else 2.0

    self.lane_change_wait_timer = 0.0

    self.lane_change_adjust = [float(Decimal(Params().get("LCTimingFactor30", encoding="utf8")) * Decimal('0.01')), float(Decimal(Params().get("LCTimingFactor60", encoding="utf8")) * Decimal('0.01')),
     float(Decimal(Params().get("LCTimingFactor80", encoding="utf8")) * Decimal('0.01')), float(Decimal(Params().get("LCTimingFactor110", encoding="utf8")) * Decimal('0.01'))]
    self.lane_change_adjust_vel = [30*CV.KPH_TO_MS, 60*CV.KPH_TO_MS, 80*CV.KPH_TO_MS, 110*CV.KPH_TO_MS]
    self.lane_change_adjust_new = 2.0
    self.lane_change_adjust_enable = Params().get_bool("LCTimingFactorEnable")

    self.lane_change_keep_enable = Params().get_bool("LCTimingKeepFactorEnable")
    self.lane_change_keep_time_left = float(Decimal(Params().get("LCTimingKeepFactorLeft", encoding="utf8")) * Decimal('0.001'))
    self.lane_change_keep_time_right = float(Decimal(Params().get("LCTimingKeepFactorRight", encoding="utf8")) * Decimal('0.001'))

    self.output_scale = 0.0
    self.ready_to_change = False

  def update(self, carstate, lateral_active, lane_change_prob, controlsstate=None, md=None):
    try:
      if controlsstate is not None:
        if controlsstate.lateralControlMethod == 0:
          self.output_scale = controlsstate.lateralControlState.pidState.output
        elif controlsstate.lateralControlMethod == 1:
          self.output_scale = controlsstate.lateralControlState.indiState.output
        elif controlsstate.lateralControlMethod == 2:
          self.output_scale = controlsstate.lateralControlState.lqrState.output
        elif controlsstate.lateralControlMethod == 3:
          self.output_scale = controlsstate.lateralControlState.torqueState.output
        elif controlsstate.lateralControlMethod == 4:
          self.output_scale = controlsstate.lateralControlState.atomState.output
    except:
      pass
    v_ego = carstate.vEgo
    one_blinker = carstate.leftBlinker != carstate.rightBlinker
    below_lane_change_speed = (v_ego < LANE_CHANGE_SPEED_MIN) or (LANE_CHANGE_SPEED_MIN == -1)

    if md is not None:
      left_edge_prob = np.clip(1.0 - md.roadEdgeStds[0], 0.0, 1.0)
      left_nearside_prob = md.laneLineProbs[0]
      left_close_prob = md.laneLineProbs[1]
      right_close_prob = md.laneLineProbs[2]
      right_nearside_prob = md.laneLineProbs[3]
      right_edge_prob = np.clip(1.0 - md.roadEdgeStds[1], 0.0, 1.0)

      if right_edge_prob > 0.35 and right_nearside_prob < 0.2 and left_nearside_prob >= right_nearside_prob:
        road_edge_stat = 1
      elif left_edge_prob > 0.35 and left_nearside_prob < 0.2 and right_nearside_prob >= left_nearside_prob:
        road_edge_stat = -1
      else:
        road_edge_stat = 0
    else:
      road_edge_stat = 0


    if carstate.leftBlinker:
      self.lane_change_direction = LaneChangeDirection.left
      lane_direction = -1
    elif carstate.rightBlinker:
      self.lane_change_direction = LaneChangeDirection.right
      lane_direction = 1
    else:
      lane_direction = 2

    cancel_condition = ((abs(self.output_scale) >= 0.8 ) or (carstate.steeringTorque > 270 and controlsstate.lateralControlMethod == 5)) and self.lane_change_timer > 0.3
    if self.lane_change_state == LaneChangeState.off and road_edge_stat == lane_direction:
      self.lane_change_direction = LaneChangeDirection.none
    elif not lateral_active or (self.lane_change_timer > LANE_CHANGE_TIME_MAX) or cancel_condition:
      self.lane_change_state = LaneChangeState.off
      self.lane_change_direction = LaneChangeDirection.none
    else:
      torque_applied = carstate.steeringPressed and \
                       ((carstate.steeringTorque > 0 and self.lane_change_direction == LaneChangeDirection.left) or
                        (carstate.steeringTorque < 0 and self.lane_change_direction == LaneChangeDirection.right))

      blindspot_detected = ((carstate.leftBlindspot and self.lane_change_direction == LaneChangeDirection.left) or
                            (carstate.rightBlindspot and self.lane_change_direction == LaneChangeDirection.right))

      # LaneChangeState.off
      if self.lane_change_state == LaneChangeState.off and one_blinker and not self.prev_one_blinker and not below_lane_change_speed:
        self.lane_change_state = LaneChangeState.preLaneChange
        self.lane_change_ll_prob = 1.0
        self.lane_change_wait_timer = 0 if not self.ready_to_change else self.lane_change_auto_delay
        if self.lane_change_adjust_enable:
          if controlsstate is not None:
            if controlsstate.curvature > 0.0005 and self.lane_change_direction == LaneChangeDirection.left: # left curve
              self.lane_change_adjust_new = min(2.0, interp(v_ego, self.lane_change_adjust_vel, self.lane_change_adjust)*1.5)
            elif controlsstate.curvature < -0.0005 and self.lane_change_direction == LaneChangeDirection.right: # right curve
              self.lane_change_adjust_new = min(2.0, interp(v_ego, self.lane_change_adjust_vel, self.lane_change_adjust)*1.5)
            else:
              self.lane_change_adjust_new = interp(v_ego, self.lane_change_adjust_vel, self.lane_change_adjust)
          else:
            self.lane_change_adjust_new = interp(v_ego, self.lane_change_adjust_vel, self.lane_change_adjust)
        else:
          self.lane_change_adjust_new = 2.0
      # LaneChangeState.preLaneChange
      elif self.lane_change_state == LaneChangeState.preLaneChange:
        self.lane_change_wait_timer += DT_MDL
        if not one_blinker or below_lane_change_speed:
          self.lane_change_state = LaneChangeState.off
        elif not blindspot_detected and (torque_applied or (self.lane_change_auto_delay and self.lane_change_wait_timer > self.lane_change_auto_delay)):
          self.lane_change_state = LaneChangeState.laneChangeStarting

      # LaneChangeState.laneChangeStarting
      elif self.lane_change_state == LaneChangeState.laneChangeStarting:
        # fade out over .5s
        self.lane_change_ll_prob = max(self.lane_change_ll_prob - self.lane_change_adjust_new * DT_MDL, 0.0)

        # 98% certainty
        if lane_change_prob < 0.02 and self.lane_change_ll_prob < 0.01:
          self.lane_change_state = LaneChangeState.laneChangeFinishing

      # LaneChangeState.laneChangeFinishing
      elif self.lane_change_state == LaneChangeState.laneChangeFinishing:
        # fade in laneline over 1s
        if USE_LEGACY_LANE_MODEL and self.lane_change_keep_enable and False:
          if self.lane_change_direction == LaneChangeDirection.left:
            prob_adj_val = interp(v_ego, [16.6, 30.5], [0.05, self.lane_change_keep_time_left])
            self.lane_change_ll_prob = min(self.lane_change_ll_prob + prob_adj_val, 1.0)
          else:
            prob_adj_val = interp(v_ego, [16.6, 30.5], [0.05, self.lane_change_keep_time_right])
            self.lane_change_ll_prob = min(self.lane_change_ll_prob + prob_adj_val, 1.0)
        else:
          self.lane_change_ll_prob = min(self.lane_change_ll_prob + DT_MDL, 1.0)
        if one_blinker and self.lane_change_ll_prob > 0.99:
          self.lane_change_state = LaneChangeState.preLaneChange
        elif self.lane_change_ll_prob > 0.99:
          self.lane_change_state = LaneChangeState.off

    if self.lane_change_state in (LaneChangeState.off, LaneChangeState.preLaneChange):
      self.lane_change_timer = 0.0
    else:
      self.lane_change_timer += DT_MDL

    self.prev_one_blinker = one_blinker
    self.ready_to_change = False
    if self.lane_change_state == LaneChangeState.off and road_edge_stat == lane_direction and one_blinker:
      self.prev_one_blinker = False
      self.ready_to_change = True

    self.desire = DESIRES[self.lane_change_direction][self.lane_change_state]

    # Send keep pulse once per second during LaneChangeStart.preLaneChange
    if self.lane_change_state in (LaneChangeState.off, LaneChangeState.laneChangeStarting):
      self.keep_pulse_timer = 0.0
    elif self.lane_change_state == LaneChangeState.preLaneChange:
      self.keep_pulse_timer += DT_MDL
      if self.keep_pulse_timer > 1.0:
        self.keep_pulse_timer = 0.0
      elif USE_LEGACY_LANE_MODEL and self.desire in (log.LateralPlan.Desire.keepLeft, log.LateralPlan.Desire.keepRight):
        self.desire = log.LateralPlan.Desire.none
      elif not USE_LEGACY_LANE_MODEL and self.desire in (log.Desire.keepLeft, log.Desire.keepRight):
        self.desire = log.Desire.none