from cereal import log
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL

import numpy as np
from openpilot.common.params import Params

USE_LEGACY_LANE_MODEL = Params().get("UseLegacyLaneModel", return_default=True) if Params().get("UseLegacyLaneModel", return_default=True) is not None else 0
LOG = log.LateralPlan if USE_LEGACY_LANE_MODEL else log

LaneChangeState = LOG.LaneChangeState
LaneChangeDirection = LOG.LaneChangeDirection

speed = Params().get("KisaLaneChangeSpeed", return_default=True)
LANE_CHANGE_SPEED_MIN = -1 if speed < 1 else speed * (CV.KPH_TO_MS if Params().get_bool("IsMetric") else CV.MPH_TO_MS)
LANE_CHANGE_TIME_MAX = 10.

DESIRES = {
  LaneChangeDirection.none: {
    LaneChangeState.off: LOG.Desire.none,
    LaneChangeState.preLaneChange: LOG.Desire.none,
    LaneChangeState.laneChangeStarting: LOG.Desire.none,
    LaneChangeState.laneChangeFinishing: LOG.Desire.none,
  },
  LaneChangeDirection.left: {
    LaneChangeState.off: LOG.Desire.none,
    LaneChangeState.preLaneChange: LOG.Desire.none,
    LaneChangeState.laneChangeStarting: LOG.Desire.laneChangeLeft,
    LaneChangeState.laneChangeFinishing: LOG.Desire.laneChangeLeft,
  },
  LaneChangeDirection.right: {
    LaneChangeState.off: LOG.Desire.none,
    LaneChangeState.preLaneChange: LOG.Desire.none,
    LaneChangeState.laneChangeStarting: LOG.Desire.laneChangeRight,
    LaneChangeState.laneChangeFinishing: LOG.Desire.laneChangeRight,
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
    self.desire = LOG.Desire.none

    params = Params()

    self.lane_change_auto_delay = {0: 0.0, 1: 0.2, 2: 0.5, 3: 1.0, 4: 1.5}.get(params.get("KisaAutoLaneChangeDelay", return_default=True), 2.0)
    self.lane_change_wait_timer = 0.0
    self.lane_change_adjust = [params.get(k, return_default=True) * 0.01 for k in ("LCTimingFactor30", "LCTimingFactor60", "LCTimingFactor80", "LCTimingFactor110")]

    self.lane_change_adjust_vel = [30*CV.KPH_TO_MS, 60*CV.KPH_TO_MS, 80*CV.KPH_TO_MS, 110*CV.KPH_TO_MS]
    self.lane_change_adjust_weight = 2.0
    self.lane_change_adjust_enable = params.get_bool("LCTimingFactorEnable")

    self.output_scale = 0.0
    self.ready_to_change = False

  @staticmethod
  def get_lane_change_direction(CS):
    return LaneChangeDirection.left if CS.leftBlinker else LaneChangeDirection.right

  def update(self, carstate, lateral_active, lane_change_prob, controlsstate=None, md=None):
    try:
      if controlsstate is not None:
        states = [
          controlsstate.lateralControlState.pidState,
          controlsstate.lateralControlState.indiState,
          controlsstate.lateralControlState.lqrState,
          controlsstate.lateralControlState.torqueState,
          controlsstate.lateralControlState.atomState,
        ]
        self.output_scale = states[controlsstate.lateralControlMethod].output
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

      right_cond = right_edge_prob > 0.35 and right_nearside_prob < 0.2 and left_nearside_prob >= right_nearside_prob
      left_cond  = left_edge_prob  > 0.35 and left_nearside_prob  < 0.2 and right_nearside_prob >= left_nearside_prob
      road_edge_stat = 1 if right_cond else -1 if left_cond else 0
    else:
      road_edge_stat = 0

    lane_direction = -1 if carstate.leftBlinker else 1 if carstate.rightBlinker else 2
    colored_lc_block = (carstate.leftLaneColor == 2 and lane_direction == -1) or (carstate.rightLaneColor == 2 and lane_direction == 1)
    cancel_condition = ((abs(self.output_scale) >= 0.8 ) or (carstate.steeringTorque > 270 and controlsstate.lateralControlMethod == 5)) and self.lane_change_timer > 0.3

    if self.lane_change_state == LaneChangeState.off and (road_edge_stat == lane_direction or colored_lc_block):
      self.lane_change_direction = LaneChangeDirection.none
    elif not lateral_active or self.lane_change_timer > LANE_CHANGE_TIME_MAX or cancel_condition:
      self.lane_change_state = LaneChangeState.off
      self.lane_change_direction = LaneChangeDirection.none
    else:
      # LaneChangeState.off
      if self.lane_change_state == LaneChangeState.off and one_blinker and not self.prev_one_blinker and not below_lane_change_speed:
        self.lane_change_state = LaneChangeState.preLaneChange
        self.lane_change_ll_prob = 1.0
        # Initialize lane change direction to prevent UI alert flicker
        self.lane_change_direction = self.get_lane_change_direction(carstate)

        self.lane_change_wait_timer = 0 if not self.ready_to_change else self.lane_change_auto_delay
        if self.lane_change_adjust_enable:
          if controlsstate is not None:
            if controlsstate.curvature > 0.0005 and self.lane_change_direction == LaneChangeDirection.left: # left curve
              self.lane_change_adjust_weight = min(2.0, np.interp(v_ego, self.lane_change_adjust_vel, self.lane_change_adjust)*1.5)
            elif controlsstate.curvature < -0.0005 and self.lane_change_direction == LaneChangeDirection.right: # right curve
              self.lane_change_adjust_weight = min(2.0, np.interp(v_ego, self.lane_change_adjust_vel, self.lane_change_adjust)*1.5)
            else:
              self.lane_change_adjust_weight = np.interp(v_ego, self.lane_change_adjust_vel, self.lane_change_adjust)
          else:
            self.lane_change_adjust_weight = np.interp(v_ego, self.lane_change_adjust_vel, self.lane_change_adjust)
        else:
          self.lane_change_adjust_weight = 2.0
      # LaneChangeState.preLaneChange
      elif self.lane_change_state == LaneChangeState.preLaneChange:
        self.lane_change_wait_timer += DT_MDL

        # Update lane change direction
        self.lane_change_direction = self.get_lane_change_direction(carstate)

        torque_applied = carstate.steeringPressed and \
                         ((carstate.steeringTorque > 0 and self.lane_change_direction == LaneChangeDirection.left) or
                          (carstate.steeringTorque < 0 and self.lane_change_direction == LaneChangeDirection.right))

        blindspot_detected = ((carstate.leftBlindspot and self.lane_change_direction == LaneChangeDirection.left) or
                              (carstate.rightBlindspot and self.lane_change_direction == LaneChangeDirection.right))

        if not one_blinker or below_lane_change_speed:
          self.lane_change_state = LaneChangeState.off
          self.lane_change_direction = LaneChangeDirection.none
        elif not blindspot_detected and (torque_applied or (self.lane_change_auto_delay and self.lane_change_wait_timer > self.lane_change_auto_delay)):
          self.lane_change_state = LaneChangeState.laneChangeStarting

      # LaneChangeState.laneChangeStarting
      elif self.lane_change_state == LaneChangeState.laneChangeStarting:
        # fade out over .5s
        self.lane_change_ll_prob = max(self.lane_change_ll_prob - self.lane_change_adjust_weight * DT_MDL, 0.0)

        # 98% certainty
        if lane_change_prob < 0.02 and self.lane_change_ll_prob < 0.01:
          self.lane_change_state = LaneChangeState.laneChangeFinishing

      # LaneChangeState.laneChangeFinishing
      elif self.lane_change_state == LaneChangeState.laneChangeFinishing:
        # fade in laneline over 1s
        self.lane_change_ll_prob = min(self.lane_change_ll_prob + DT_MDL, 1.0)

        if self.lane_change_ll_prob > 0.99:
          self.lane_change_direction = LaneChangeDirection.none
          if one_blinker:
            self.lane_change_state = LaneChangeState.preLaneChange
          else:
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
      elif self.desire in (LOG.Desire.keepLeft, LOG.Desire.keepRight):
        self.desire = LOG.Desire.none