import time
import numpy as np
from openpilot.common.realtime import DT_MDL
from openpilot.common.numpy_fast import interp
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.controls.lib.lateral_mpc_lib.lat_mpc import LateralMpc
from openpilot.selfdrive.controls.lib.lateral_mpc_lib.lat_mpc import N as LAT_MPC_N
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N, MIN_SPEED, get_speed_error
from openpilot.selfdrive.controls.lib.desire_helper import DesireHelper
import cereal.messaging as messaging
from cereal import log

from openpilot.common.conversions import Conversions as CV
from openpilot.common.params import Params
from decimal import Decimal

LaneChangeState = log.LateralPlan.LaneChangeState

TRAJECTORY_SIZE = 33

CAMERA_OFFSET = (float(Decimal(Params().get("CameraOffsetAdj", encoding="utf8")) * Decimal('0.001')))
CAMERA_OFFSET_A = CAMERA_OFFSET + 0.15
PATH_OFFSET = (float(Decimal(Params().get("PathOffsetAdj", encoding="utf8")) * Decimal('0.001')))  # default 0.0

PATH_COST = 1.0
LATERAL_MOTION_COST = 0.11
LATERAL_ACCEL_COST = 0.0
LATERAL_JERK_COST = 0.04
# Extreme steering rate is unpleasant, even
# when it does not cause bad jerk.
# TODO this cost should be lowered when low
# speed lateral control is stable on all cars
STEERING_RATE_COST = 700.0

class LateralPlanner:
  def __init__(self, CP):
    self.DH = DesireHelper()

    self.params = Params()
    self.legacy_lane_mode = int(self.params.get("UseLegacyLaneModel", encoding="utf8"))

    # Vehicle model parameters used to calculate lateral movement of car
    self.factor1 = CP.wheelbase - CP.centerToFront
    self.factor2 = (CP.centerToFront * CP.mass) / (CP.wheelbase * CP.tireStiffnessRear)
    self.last_cloudlog_t = 0
    self.solution_invalid_cnt = 0

    self.path_xyz = np.zeros((TRAJECTORY_SIZE, 3))
    self.velocity_xyz = np.zeros((TRAJECTORY_SIZE, 3))
    self.v_plan = np.zeros((TRAJECTORY_SIZE,))
    if self.legacy_lane_mode:
      self.plan_yaw = np.zeros((TRAJECTORY_SIZE,))
      self.plan_yaw_rate = np.zeros((TRAJECTORY_SIZE,))
      self.t_idxs = np.arange(TRAJECTORY_SIZE)
      self.y_pts = np.zeros((TRAJECTORY_SIZE,))
      self.v_ego = 0.0

    self.l_lane_change_prob = 0.0
    self.r_lane_change_prob = 0.0

    self.lat_mpc = LateralMpc()
    self.reset_mpc(np.zeros(4))


    self.laneless_mode = int(Params().get("LanelessMode", encoding="utf8"))
    self.laneless_mode_status = False
    self.laneless_mode_status_buffer = False

    self.standstill_elapsed_time = 0.0
    self.v_cruise_kph = 0
    self.stand_still = False
    
    self.second = 0.0
    self.model_speed = 255.0


    # lane_planner
    self.ll_t = np.zeros((TRAJECTORY_SIZE,))
    self.ll_x = np.zeros((TRAJECTORY_SIZE,))
    self.lll_y = np.zeros((TRAJECTORY_SIZE,))
    self.rll_y = np.zeros((TRAJECTORY_SIZE,))
    self.lane_width_estimate = FirstOrderFilter(float(Decimal(self.params.get("LaneWidth", encoding="utf8")) * Decimal('0.1')), 9.95, DT_MDL)
    self.lane_width_certainty = FirstOrderFilter(1.0, 0.95, DT_MDL)
    self.lane_width = float(Decimal(self.params.get("LaneWidth", encoding="utf8")) * Decimal('0.1'))
    self.spd_lane_width_spd = list(map(float, self.params.get("SpdLaneWidthSpd", encoding="utf8").split(',')))
    self.spd_lane_width_set = list(map(float, self.params.get("SpdLaneWidthSet", encoding="utf8").split(',')))
    self.lll_prob = self.rll_prob = self.d_prob = self.lll_std = self.rll_std = 0.
    self.camera_offset = CAMERA_OFFSET
    self.path_offset = PATH_OFFSET
    self.path_offset2 = 0.0
    self.left_curv_offset = int(self.params.get("LeftCurvOffsetAdj", encoding="utf8"))
    self.right_curv_offset = int(self.params.get("RightCurvOffsetAdj", encoding="utf8"))
    self.drive_routine_on_co = self.params.get_bool("RoutineDriveOn")
    if self.drive_routine_on_co:
      option_list = list(self.params.get("RoutineDriveOption", encoding="utf8"))
      if '0' in option_list:
        self.drive_routine_on_co = True
      else:
        self.drive_routine_on_co = False
    self.drive_close_to_edge = self.params.get_bool("CloseToRoadEdge")
    self.left_edge_offset = float(Decimal(self.params.get("LeftEdgeOffset", encoding="utf8")) * Decimal('0.01'))
    self.right_edge_offset = float(Decimal(self.params.get("RightEdgeOffset", encoding="utf8")) * Decimal('0.01'))
    self.speed_offset = self.params.get_bool("SpeedCameraOffset")
    self.road_edge_offset = 0.0
    self.timer = 0
    self.timer2 = 0
    self.timer3 = 0
    self.sm = messaging.SubMaster(['liveMapData'])
    self.total_camera_offset = self.camera_offset
    self.is_mph = not self.params.get_bool("IsMetric")


  def parse_model(self, md, sm, v_ego):
    curvature = sm['controlsState'].curvature
    mode_select = sm['carState'].cruiseState.modeSel
    if self.drive_routine_on_co:
      self.sm.update(0)
      current_road_offset = self.sm['liveMapData'].roadCameraOffset
    else:
      current_road_offset = 0.0

    Curv = round(curvature, 4)
    # right lane is minus
    lane_differ = round(self.lll_y[0] + self.rll_y[0], 2)
    lean_offset = 0
    if int(mode_select) == 4:
      lean_offset = 0.15
    else:
      lean_offset = 0

    if (self.left_curv_offset != 0 or self.right_curv_offset != 0) and v_ego > 8 and int(mode_select) != 4:
      if curvature > 0.0008 and self.left_curv_offset < 0 and lane_differ <= 0: # left curve, if car is more close to left line
        if lane_differ > 0.6:
          lane_differ = 0.6          
        lean_offset = -round(abs(self.left_curv_offset) * abs(lane_differ * 0.05), 3) # move to left
      elif curvature > 0.0008 and self.left_curv_offset > 0 and lane_differ >= 0: # left curve, if car is more close to right line
        if lane_differ > 0.6:
          lane_differ = 0.6
        lean_offset = +round(abs(self.left_curv_offset) * abs(lane_differ * 0.05), 3) # move to right
      elif curvature < -0.0008 and self.right_curv_offset < 0 and lane_differ <= 0: # right curve, if car is more close to left line
        if lane_differ > 0.6:
          lane_differ = 0.6    
        lean_offset = -round(abs(self.right_curv_offset) * abs(lane_differ * 0.05), 3) # move to left
      elif curvature < -0.0008 and self.right_curv_offset > 0 and lane_differ >= 0: # right curve, if car is more close to right line
        if lane_differ > 0.6:
          lane_differ = 0.6    
        lean_offset = +round(abs(self.right_curv_offset) * abs(lane_differ * 0.05), 3) # move to right
      else:
        lean_offset = 0

    self.timer += DT_MDL
    if self.timer > 1.0:
      self.timer = 0.0
      self.speed_offset = self.params.get_bool("SpeedCameraOffset")
      if self.params.get_bool("KisaLiveTunePanelEnable"):
        self.camera_offset = (float(Decimal(self.params.get("CameraOffsetAdj", encoding="utf8")) * Decimal('0.001')))

    if self.drive_close_to_edge: # kisapilot
      left_edge_prob = np.clip(1.0 - md.roadEdgeStds[0], 0.0, 1.0)
      left_nearside_prob = md.laneLineProbs[0]
      left_close_prob = md.laneLineProbs[1]
      right_close_prob = md.laneLineProbs[2]
      right_nearside_prob = md.laneLineProbs[3]
      right_edge_prob = np.clip(1.0 - md.roadEdgeStds[1], 0.0, 1.0)

      self.timer3 += DT_MDL
      if self.timer3 > 1.0:
        self.timer3 = 0.0
        if right_nearside_prob < 0.2 and left_nearside_prob < 0.2:
          if self.road_edge_offset != 0.0:
            if self.road_edge_offset > 0.0:
              self.road_edge_offset -= max(0.01, round((self.road_edge_offset/5), 2))
              if self.road_edge_offset < 0.0:
                self.road_edge_offset = 0.0
            elif self.road_edge_offset < 0.0:
              self.road_edge_offset += max(0.01, round((self.road_edge_offset/5), 2))
              if self.road_edge_offset > 0.0:
                self.road_edge_offset = 0.0
        elif right_edge_prob > 0.3 and right_nearside_prob < 0.3 and right_close_prob > 0.4 and left_nearside_prob >= right_nearside_prob:
          if self.right_edge_offset != 0.0 and self.road_edge_offset != self.right_edge_offset:
            if self.road_edge_offset < self.right_edge_offset:
              self.road_edge_offset += max(0.01, round((self.right_edge_offset/5), 2))
              if self.road_edge_offset > self.right_edge_offset:
                self.road_edge_offset = self.right_edge_offset
            elif self.road_edge_offset > self.right_edge_offset:
              self.road_edge_offset -= max(0.01, round((self.right_edge_offset/5), 2))
              if self.road_edge_offset < self.right_edge_offset:
                self.road_edge_offset = self.right_edge_offset
        elif left_edge_prob > 0.3 and left_nearside_prob < 0.3 and left_close_prob > 0.4 and right_nearside_prob >= left_nearside_prob:
          if self.left_edge_offset != 0.0 and self.road_edge_offset != self.left_edge_offset:
            if self.road_edge_offset < self.left_edge_offset:
              self.road_edge_offset += max(0.01, round((self.left_edge_offset/5), 2))
              if self.road_edge_offset > self.left_edge_offset:
                self.road_edge_offset = self.left_edge_offset
            elif self.road_edge_offset > self.left_edge_offset:
              self.road_edge_offset -= max(0.01, round((self.left_edge_offset/5), 2))
              if self.road_edge_offset < self.left_edge_offset:
                self.road_edge_offset = self.left_edge_offset
        else:
          if self.road_edge_offset != 0.0:
            if self.road_edge_offset > 0.0:
              self.road_edge_offset -= max(0.01, round((self.road_edge_offset/5), 2))
              if self.road_edge_offset < 0.0:
                self.road_edge_offset = 0.0
            elif self.road_edge_offset < 0.0:
              self.road_edge_offset += max(0.01, round((self.road_edge_offset/5), 2))
              if self.road_edge_offset > 0.0:
                self.road_edge_offset = 0.0
        self.path_offset2 = self.road_edge_offset
    else:
      self.road_edge_offset = 0.0
      self.path_offset2 = self.road_edge_offset
    if self.speed_offset:
      speed_offset = -interp(v_ego, [0, 11.1, 16.6, 22.2, 31], [0.10, 0.05, 0.02, 0.01, 0.0])
    else:
      speed_offset = 0.0

    # logic is opposite compared to before
    # for self.total_camera_offset, low value to move for car left side, high value to move for car right side.
    # little confused to make sure. let me know if this is incorrect.
    self.total_camera_offset = self.camera_offset + lean_offset + current_road_offset + self.road_edge_offset + speed_offset

    lane_lines = md.laneLines
    if len(lane_lines) == 4 and len(lane_lines[0].t) == TRAJECTORY_SIZE:
      self.ll_t = (np.array(lane_lines[1].t) + np.array(lane_lines[2].t))/2
      # left and right ll x is the same
      self.ll_x = lane_lines[1].x
      self.lll_y = np.array(lane_lines[1].y) + self.total_camera_offset
      self.rll_y = np.array(lane_lines[2].y) + self.total_camera_offset
      self.lll_prob = md.laneLineProbs[1]
      self.rll_prob = md.laneLineProbs[2]
      self.lll_std = md.laneLineStds[1]
      self.rll_std = md.laneLineStds[2]

    desire_state = md.meta.desireState
    if len(desire_state):
      self.l_lane_change_prob = desire_state[log.LateralPlan.Desire.laneChangeLeft]
      self.r_lane_change_prob = desire_state[log.LateralPlan.Desire.laneChangeRight]

  def get_d_path(self, v_ego, path_t, path_xyz):
    self.timer2 += DT_MDL
    if self.timer2 > 1.0:
      self.timer2 = 0.0
      if self.params.get_bool("KisaLiveTunePanelEnable"):
        self.path_offset = (float(Decimal(self.params.get("PathOffsetAdj", encoding="utf8")) * Decimal('0.001')))
    # Reduce reliance on lanelines that are too far apart or
    # will be in a few seconds
    path_xyz[:, 1] += (self.path_offset+self.path_offset2)
    l_prob, r_prob = self.lll_prob, self.rll_prob
    width_pts = self.rll_y - self.lll_y
    prob_mods = []
    for t_check in (0.0, 1.5, 3.0):
      width_at_t = interp(t_check * (v_ego + 7), self.ll_x, width_pts)
      prob_mods.append(interp(width_at_t, [4.0, 5.0], [1.0, 0.0]))
    mod = min(prob_mods)
    l_prob *= mod
    r_prob *= mod

    # Reduce reliance on uncertain lanelines
    l_std_mod = interp(self.lll_std, [.15, .3], [1.0, 0.0])
    r_std_mod = interp(self.rll_std, [.15, .3], [1.0, 0.0])
    l_prob *= l_std_mod
    r_prob *= r_std_mod

    # Find current lanewidth
    self.lane_width_certainty.update(l_prob * r_prob)
    current_lane_width = abs(self.rll_y[0] - self.lll_y[0])
    self.lane_width_estimate.update(current_lane_width)
    speed_lane_width = interp(v_ego, self.spd_lane_width_spd, self.spd_lane_width_set)
    self.lane_width = self.lane_width_certainty.x * self.lane_width_estimate.x + \
                      (1 - self.lane_width_certainty.x) * speed_lane_width

    clipped_lane_width = min(4.0, self.lane_width)
    path_from_left_lane = self.lll_y + clipped_lane_width / 2.0
    path_from_right_lane = self.rll_y - clipped_lane_width / 2.0

    self.d_prob = l_prob + r_prob - l_prob * r_prob
    lane_path_y = (l_prob * path_from_left_lane + r_prob * path_from_right_lane) / (l_prob + r_prob + 0.0001)
    safe_idxs = np.isfinite(self.ll_t)
    if safe_idxs[0]:
      lane_path_y_interp = np.interp(path_t, self.ll_t[safe_idxs], lane_path_y[safe_idxs])
      path_xyz[:,1] = self.d_prob * lane_path_y_interp + (1.0 - self.d_prob) * path_xyz[:,1]
    else:
      cloudlog.warning("Lateral mpc - NaNs in laneline times, ignoring")
    return path_xyz


  def curve_speed(self, sm, v_ego):
    md = sm['modelV2']
    curvature = sm['controlsState'].curvature
    if md is not None and len(md.position.x) == TRAJECTORY_SIZE and len(md.position.y) == TRAJECTORY_SIZE:
      x = md.position.x
      y = md.position.y
      dy = np.gradient(y, x)
      d2y = np.gradient(dy, x)
      curv = d2y / (1 + dy ** 2) ** 1.5
      start = int(interp(v_ego, [10., 27.], [10, TRAJECTORY_SIZE-10])) # neokii's factor
      if abs(curvature) > 0.0008: # kisapilot
        curv = curv[5:TRAJECTORY_SIZE-10]
      else:
        curv = curv[start:min(start+10, TRAJECTORY_SIZE)]
      a_y_max = 2.975 - v_ego * 0.0375  # ~1.85 @ 75mph, ~2.6 @ 25mph
      v_curvature = np.sqrt(a_y_max / np.clip(np.abs(curv), 1e-4, None))
      model_speed = np.mean(v_curvature) * 0.9
      curve_speed = float(max(model_speed, 30 * CV.KPH_TO_MS))
      if np.isnan(curve_speed):
        curve_speed = 255
    else:
      curve_speed = 255
    return min(255, curve_speed * (CV.MS_TO_MPH if self.is_mph else CV.MS_TO_KPH))

  def reset_mpc(self, x0=None):
    if x0 is None:
      x0 = np.zeros(4)
    self.x0 = x0
    self.lat_mpc.reset(x0=self.x0)

  def update(self, sm):
    self.second += DT_MDL
    if self.second > 1.0:
      self.laneless_mode = int(Params().get("LanelessMode", encoding="utf8"))
      self.second = 0.0

    self.v_cruise_kph = sm['controlsState'].vCruise
    self.stand_still = sm['carState'].standStill

    v_ego = sm['carState'].vEgo
    if sm.frame % 5 == 0:
      self.model_speed = self.curve_speed(sm, v_ego)

    # clip speed , lateral planning is not possible at 0 speed
    measured_curvature = sm['controlsState'].curvature
    v_ego_car = sm['carState'].vEgo

    # Parse model predictions
    md = sm['modelV2']
    if self.legacy_lane_mode:
      self.parse_model(md, sm, v_ego)
      if len(md.position.x) == TRAJECTORY_SIZE and len(md.orientation.x) == TRAJECTORY_SIZE:
        self.path_xyz = np.column_stack([md.position.x, md.position.y, md.position.z])
        self.t_idxs = np.array(md.position.t)
        self.plan_yaw = np.array(md.orientation.z)
        self.plan_yaw_rate = np.array(md.orientationRate.z)
        self.velocity_xyz = np.column_stack([md.velocity.x, md.velocity.y, md.velocity.z])
        car_speed = np.linalg.norm(self.velocity_xyz, axis=1) - get_speed_error(md, v_ego_car)
        self.v_plan = np.clip(car_speed, MIN_SPEED, np.inf)
        self.v_ego = self.v_plan[0]

      # Lane change logic
      desire_state = md.meta.desireState
      if len(desire_state):
        self.l_lane_change_prob = desire_state[log.LateralPlan.Desire.laneChangeLeft]
        self.r_lane_change_prob = desire_state[log.LateralPlan.Desire.laneChangeRight]
      lane_change_prob = self.l_lane_change_prob + self.r_lane_change_prob
      self.DH.update(sm['carState'], sm['carControl'].latActive, lane_change_prob, sm['controlsState'], md)

      self.lat_mpc.set_weights(PATH_COST, LATERAL_MOTION_COST,
                              LATERAL_ACCEL_COST, LATERAL_JERK_COST,
                              STEERING_RATE_COST)

      if self.laneless_mode == 0:
        d_path_xyz = self.get_d_path(v_ego, self.t_idxs, self.path_xyz)
        self.laneless_mode_status = False
        y_pts = np.interp(v_ego * self.t_idxs[:LAT_MPC_N + 1], np.linalg.norm(d_path_xyz, axis=1), d_path_xyz[:, 1])
        heading_pts = np.interp(v_ego * self.t_idxs[:LAT_MPC_N + 1], np.linalg.norm(self.path_xyz, axis=1), self.plan_yaw)
        yaw_rate_pts = np.interp(v_ego * self.t_idxs[:LAT_MPC_N + 1], np.linalg.norm(self.path_xyz, axis=1), self.plan_yaw_rate)
      elif self.laneless_mode == 1:
        self.laneless_mode_status = True
        y_pts = self.path_xyz[:LAT_MPC_N+1, 1]
        heading_pts = self.plan_yaw[:LAT_MPC_N+1]
        yaw_rate_pts = self.plan_yaw_rate[:LAT_MPC_N+1]
      elif self.laneless_mode == 2 and ((self.lll_prob + self.rll_prob)/2 < 0.3) and self.DH.lane_change_state == LaneChangeState.off:
        self.laneless_mode_status = True
        self.laneless_mode_status_buffer = True
        y_pts = self.path_xyz[:LAT_MPC_N+1, 1]
        heading_pts = self.plan_yaw[:LAT_MPC_N+1]
        yaw_rate_pts = self.plan_yaw_rate[:LAT_MPC_N+1]
      elif self.laneless_mode == 2 and ((self.lll_prob + self.rll_prob)/2 > 0.5) and \
        self.laneless_mode_status_buffer and self.DH.lane_change_state == LaneChangeState.off:
        d_path_xyz = self.get_d_path(v_ego, self.t_idxs, self.path_xyz)
        self.laneless_mode_status = False
        self.laneless_mode_status_buffer = False
        y_pts = np.interp(v_ego * self.t_idxs[:LAT_MPC_N + 1], np.linalg.norm(d_path_xyz, axis=1), d_path_xyz[:, 1])
        heading_pts = np.interp(v_ego * self.t_idxs[:LAT_MPC_N + 1], np.linalg.norm(self.path_xyz, axis=1), self.plan_yaw)
        yaw_rate_pts = np.interp(v_ego * self.t_idxs[:LAT_MPC_N + 1], np.linalg.norm(self.path_xyz, axis=1), self.plan_yaw_rate)
      elif self.laneless_mode == 2 and self.laneless_mode_status_buffer == True and self.DH.lane_change_state == LaneChangeState.off:
        self.laneless_mode_status = True
        y_pts = self.path_xyz[:LAT_MPC_N+1, 1]
        heading_pts = self.plan_yaw[:LAT_MPC_N+1]
        yaw_rate_pts = self.plan_yaw_rate[:LAT_MPC_N+1]
      else:
        d_path_xyz = self.get_d_path(v_ego, self.t_idxs, self.path_xyz)
        self.laneless_mode_status = False
        self.laneless_mode_status_buffer = False
        y_pts = np.interp(v_ego * self.t_idxs[:LAT_MPC_N + 1], np.linalg.norm(d_path_xyz, axis=1), d_path_xyz[:, 1])
        heading_pts = np.interp(v_ego * self.t_idxs[:LAT_MPC_N + 1], np.linalg.norm(self.path_xyz, axis=1), self.plan_yaw)
        yaw_rate_pts = np.interp(v_ego * self.t_idxs[:LAT_MPC_N + 1], np.linalg.norm(self.path_xyz, axis=1), self.plan_yaw_rate)
      self.y_pts = y_pts

      assert len(y_pts) == LAT_MPC_N + 1
      assert len(heading_pts) == LAT_MPC_N + 1
      assert len(yaw_rate_pts) == LAT_MPC_N + 1
      lateral_factor = np.clip(self.factor1 - (self.factor2 * self.v_plan**2), 0.0, np.inf)
      p = np.column_stack([self.v_plan, lateral_factor])
      self.lat_mpc.run(self.x0,
                      p,
                      y_pts,
                      heading_pts,
                      yaw_rate_pts)
      # init state for next iteration
      # mpc.u_sol is the desired second derivative of psi given x0 curv state.
      # with x0[3] = measured_yaw_rate, this would be the actual desired yaw rate.
      # instead, interpolate x_sol so that x0[3] is the desired yaw rate for lat_control.
      self.x0[3] = interp(DT_MDL, self.t_idxs[:LAT_MPC_N + 1], self.lat_mpc.x_sol[:, 3])

      #  Check for infeasible MPC solution
      mpc_nans = np.isnan(self.lat_mpc.x_sol[:, 3]).any()
      t = time.monotonic()
      if mpc_nans or self.lat_mpc.solution_status != 0:
        self.reset_mpc()
        self.x0[3] = measured_curvature * self.v_ego
        if t > self.last_cloudlog_t + 5.0:
          self.last_cloudlog_t = t
          cloudlog.warning("Lateral mpc - nan: True")

      if self.lat_mpc.cost > 1e6 or mpc_nans:
        self.solution_invalid_cnt += 1
      else:
        self.solution_invalid_cnt = 0

  def publish(self, sm, pm):
    if self.legacy_lane_mode:
      plan_solution_valid = self.solution_invalid_cnt < 2
    plan_send = messaging.new_message('lateralPlan')
    plan_send.valid = sm.all_checks(service_list=['carState', 'controlsState', 'modelV2'])

    lateralPlan = plan_send.lateralPlan
    lateralPlan.modelMonoTime = sm.logMonoTime['modelV2']
    if self.legacy_lane_mode:
      lateralPlan.laneWidth = float(self.lane_width)
      lateralPlan.dPathPoints = self.y_pts.tolist()
      lateralPlan.psis = self.lat_mpc.x_sol[0:CONTROL_N, 2].tolist()

      lateralPlan.curvatures = (self.lat_mpc.x_sol[0:CONTROL_N, 3]/self.v_ego).tolist()
      lateralPlan.curvatureRates = [float(x.item() / self.v_ego) for x in self.lat_mpc.u_sol[0:CONTROL_N - 1]] + [0.0]
      lateralPlan.lProb = float(self.lll_prob)
      lateralPlan.rProb = float(self.rll_prob)
      lateralPlan.dProb = float(self.d_prob)

      lateralPlan.mpcSolutionValid = bool(plan_solution_valid)
      lateralPlan.solverExecutionTime = self.lat_mpc.solve_time

    lateralPlan.desire = self.DH.desire
    lateralPlan.useLaneLines = False
    lateralPlan.laneChangeState = self.DH.lane_change_state
    lateralPlan.laneChangeDirection = self.DH.lane_change_direction

    lateralPlan.modelSpeed = float(self.model_speed)
    lateralPlan.outputScale = float(self.DH.output_scale)
    lateralPlan.vCruiseSet = float(self.v_cruise_kph)
    lateralPlan.vCurvature = float(sm['controlsState'].curvature)
    lateralPlan.lanelessMode = bool(self.laneless_mode_status)
    lateralPlan.totalCameraOffset = float(self.total_camera_offset)

    if self.stand_still:
      self.standstill_elapsed_time += DT_MDL
    else:
      self.standstill_elapsed_time = 0.0
    lateralPlan.standstillElapsedTime = int(self.standstill_elapsed_time)

    pm.send('lateralPlan', plan_send)
