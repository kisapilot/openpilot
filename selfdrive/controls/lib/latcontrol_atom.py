import math
import numpy as np

from cereal import log
from openpilot.common.realtime import DT_CTRL
from openpilot.common.numpy_fast import clip, interp
from openpilot.common.filter_simple import FirstOrderFilter

from openpilot.common.params import Params
from decimal import Decimal
from openpilot.common.conversions import Conversions as CV

from openpilot.selfdrive.controls.lib.latcontrol import LatControl
from openpilot.common.pid import PIDController

from openpilot.selfdrive.controls.lib.latcontrol_torque import LatControlTorque
from openpilot.selfdrive.controls.lib.latcontrol_lqr import LatControlLQR
from openpilot.selfdrive.controls.lib.latcontrol_indi import LatControlINDI
from openpilot.selfdrive.controls.lib.latcontrol_pid import LatControlPID


class LatCtrlToqATOM(LatControlTorque):
  def __init__(self, CP, CI, TORQUE):
    self.mpc_frame = 0
    self.params = Params()
    self.sat_count_rate = 1.0 * DT_CTRL
    self.sat_limit = CP.steerLimitTimer
    self.sat_count = 0. 
    
    # we define the steer torque scale as [-1.0...1.0] 
    self.steer_max = 1.0

    self.pid = PIDController(TORQUE.kp, TORQUE.ki,
                             k_f=TORQUE.kf, pos_limit=self.steer_max, neg_limit=-self.steer_max)
    self.get_steer_feedforward = CI.get_steer_feedforward_function()
    self.use_steering_angle = TORQUE.useSteeringAngle
    self.friction = TORQUE.friction
    self.kf = TORQUE.kf
    self.steering_angle_deadzone_deg = TORQUE.steeringAngleDeadzoneDeg

    self.live_tune_enabled = False
    self.lt_timer = 0


class LatCtrlLqrATOM(LatControlLQR):
  def __init__(self, CP, CI, LQR):
    self.mpc_frame = 0
    self.live_tune_enabled = False
    self.params = Params()
    self.sat_count_rate = 1.0 * DT_CTRL 
    self.sat_limit = CP.steerLimitTimer 
    self.sat_count = 0. 
    
    # we define the steer torque scale as [-1.0...1.0]
    self.steer_max = 1.0

    self.scale = LQR.scale
    self.ki = LQR.ki

    self.A = np.array(LQR.a).reshape((2, 2))
    self.B = np.array(LQR.b).reshape((2, 1))
    self.C = np.array(LQR.c).reshape((1, 2))
    self.K = np.array(LQR.k).reshape((1, 2))
    self.L = np.array(LQR.l).reshape((2, 1))
    self.dc_gain = LQR.dcGain

    self.x_hat = np.array([[0], [0]])
    self.i_unwind_rate = 0.3 * DT_CTRL
    self.i_rate = 1.0 * DT_CTRL

    self.ll_timer = 0

    self.reset()


class LatCtrlIndATOM(LatControlINDI):
  def __init__(self, CP, CI, INDI):
    self.angle_steers_des = 0.

    A = np.array([[1.0, DT_CTRL, 0.0],
                  [0.0, 1.0, DT_CTRL],
                  [0.0, 0.0, 1.0]])
    C = np.array([[1.0, 0.0, 0.0],
                  [0.0, 1.0, 0.0]])

    # Q = np.matrix([[1e-2, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 10.0]])
    # R = np.matrix([[1e-2, 0.0], [0.0, 1e3]])

    # (x, l, K) = control.dare(np.transpose(A), np.transpose(C), Q, R)
    # K = np.transpose(K)
    K = np.array([[7.30262179e-01, 2.07003658e-04],
                  [7.29394177e+00, 1.39159419e-02],
                  [1.71022442e+01, 3.38495381e-02]])

    self.speed = 0.

    self.K = K
    self.A_K = A - np.dot(K, C)
    self.x = np.array([[0.], [0.], [0.]])

    self.mpc_frame = 0
    self.params = Params()

    self.steer_max = 1.0

    self._RC = (INDI.timeConstantBP, INDI.timeConstantV)
    self._G = (INDI.actuatorEffectivenessBP, INDI.actuatorEffectivenessV)
    self._outer_loop_gain = (INDI.outerLoopGainBP, INDI.outerLoopGainV)
    self._inner_loop_gain = (INDI.innerLoopGainBP, INDI.innerLoopGainV)

    self.RC = interp(self.speed, self._RC[0], self._RC[1])
    self.G = interp(self.speed, self._G[0], self._G[1])
    self.outer_loop_gain = interp(self.speed, self._outer_loop_gain[0], self._outer_loop_gain[1])
    self.inner_loop_gain = interp(self.speed, self._inner_loop_gain[0], self._inner_loop_gain[1])

    self.steer_filter = FirstOrderFilter(0., self.RC, DT_CTRL)

    self.sat_count_rate = 1.0 * DT_CTRL 
    self.sat_limit = CP.steerLimitTimer 
    self.sat_count = 0. 

    self.li_timer = 0
    self.live_tune_enabled = False

    self.reset()

class LatCtrlPidATOM(LatControlPID):
  def __init__(self, CP, CI, PID):

    self.steer_max = 1.0
    self.pid = PIDController((PID.kpBP, PID.kpV),
                             (PID.kiBP, PID.kiV),
                             k_f=PID.kf, k_d=PID.kd,
                             pos_limit=self.steer_max, neg_limit=-self.steer_max)

    self.get_steer_feedforward = CI.get_steer_feedforward_function()

    self.mpc_frame = 0
    self.params = Params()

    self.live_tune_enabled = False

    self.lp_timer = 0

class LaMethod:
  SPEED_LOWDT = 0
  ANGLE_LOWDT = 1
  ANGLE_INTERP = 2
  SPEED_INTERP = 3


class LatControlATOM(LatControl):
  def __init__(self, CP, CI):
    super().__init__(CP, CI)

    self.lqr = CP.lateralTuning.atom.lqr
    self.torque = CP.lateralTuning.atom.torque
    self.indi = CP.lateralTuning.atom.indi
    self.pid1 = CP.lateralTuning.atom.pid

    self.LaLqr = LatCtrlLqrATOM( CP, CI, self.lqr )
    self.LaToq = LatCtrlToqATOM( CP, CI, self.torque )
    self.LaInd = LatCtrlIndATOM( CP, CI, self.indi )
    self.LaPid = LatCtrlPidATOM( CP, CI, self.pid1 )

    self.reset()

    self.multi_lateral_method = int(Params().get("MultipleLateralUse", encoding="utf8"))
    self.multi_lat_spdMethod  = list(map(int, Params().get("MultipleLateralOpS", encoding="utf8").split(',')))
    self.multi_lat_spdBP      = list(map(int, Params().get("MultipleLateralSpd", encoding="utf8").split(',')))
    self.multi_lat_angMethod  = list(map(int, Params().get("MultipleLateralOpA", encoding="utf8").split(',')))
    self.multi_lat_angBP      = list(map(int, Params().get("MultipleLateralAng", encoding="utf8").split(',')))
    
    if self.multi_lateral_method == LaMethod.ANGLE_INTERP:
      self.lat_fun0 = self.method_func( self.multi_lat_angMethod[0] )
      self.lat_fun1 = self.method_func( self.multi_lat_angMethod[1] )
      self.lat_fun2 = self.method_func( self.multi_lat_angMethod[2] )
      self.latBP = self.multi_lat_angBP
    elif self.multi_lateral_method == LaMethod.SPEED_INTERP:
      self.lat_fun0 = self.method_func( self.multi_lat_spdMethod[0] )
      self.lat_fun1 = self.method_func( self.multi_lat_spdMethod[1] )
      self.lat_fun2 = self.method_func( self.multi_lat_spdMethod[2] )
      self.latBP = self.multi_lat_spdBP
    else:
      self.lat_fun0 = None
      self.lat_fun1 = None
      self.lat_fun2 = None
      self.latBP = None
    
    self.lat_atom_timer = 0
    self.s_value = 0

  def method_func(self, BP ):
    lat_fun = None
    if BP == 0:
      lat_fun  = self.LaPid.update
    elif BP == 1:
      lat_fun  = self.LaInd.update
    elif BP == 2:
      lat_fun  = self.LaLqr.update
    elif BP == 3:
      lat_fun  = self.LaToq.update
    return lat_fun       

  def reset(self):
    super().reset()
    self.LaLqr.reset()
    self.LaInd.steer_filter.x = 0.
    self.LaInd.speed = 0.
    self.LaPid.reset()

  def method_angle(self, active, CS, VM, params, steer_limited, desired_curvature, desired_curvature_rate, calibrated_pose):  # angle
    steer_ang = abs(CS.steeringAngleDeg)
    output_torque0, desired_angle0, log0  = self.lat_fun0( active, CS, VM, params, steer_limited, desired_curvature, desired_curvature_rate, calibrated_pose )
    output_torque1, desired_angle1, log1  = self.lat_fun1( active, CS, VM, params, steer_limited, desired_curvature, desired_curvature_rate, calibrated_pose )
    output_torque2, desired_angle2, log2  = self.lat_fun2( active, CS, VM, params, steer_limited, desired_curvature, desired_curvature_rate, calibrated_pose )

    desired_angle = interp( steer_ang, self.latBP, [desired_angle0, desired_angle1, desired_angle2] )
    output_torque = interp( steer_ang, self.latBP, [output_torque0, output_torque1, output_torque2] )

    return output_torque, desired_angle

  def method_speed(self, active, CS, VM, params, steer_limited, desired_curvature, desired_curvature_rate, calibrated_pose):  # speed
    speed = CS.vEgo * (CV.MS_TO_MPH if CS.isMph else CV.MS_TO_KPH)
    output_torque0, desired_angle0, log0  = self.lat_fun0( active, CS, VM, params, steer_limited, desired_curvature, desired_curvature_rate, calibrated_pose )
    output_torque1, desired_angle1, log1  = self.lat_fun1( active, CS, VM, params, steer_limited, desired_curvature, desired_curvature_rate, calibrated_pose )
    output_torque2, desired_angle2, log2  = self.lat_fun2( active, CS, VM, params, steer_limited, desired_curvature, desired_curvature_rate, calibrated_pose )

    desired_angle = interp( speed, self.latBP, [desired_angle0, desired_angle1, desired_angle2] )
    output_torque = interp( speed, self.latBP, [output_torque0, output_torque1, output_torque2] )
    
    return output_torque, desired_angle


  def update(self, active, CS, VM, params, steer_limited, desired_curvature, desired_curvature_rate, calibrated_pose):
    atom_log = log.ControlsState.LateralATOMState.new_message()

    selected = 3

    pid_desired_angle = 0
    ind_desired_angle = 0
    lqr_desired_angle = 0
    toq_desired_angle = 0
    pid_output_torque = 0
    ind_output_torque = 0
    lqr_output_torque = 0
    toq_output_torque = 0

    desired_angle = 0
    output_torque = 0
    if not active:
      output_torque = 0.0
      lqr_desired_angle = 0.
      atom_log.active = False
      self.reset()
    else:
      if self.multi_lateral_method == LaMethod.SPEED_LOWDT:
        if 2 in self.multi_lat_spdMethod:
          lqr_output_torque, lqr_desired_angle, lqr_log  = self.LaLqr.update( active, CS, VM, params, steer_limited, desired_curvature, desired_curvature_rate, calibrated_pose )
        if 3 in self.multi_lat_spdMethod:
          toq_output_torque, toq_desired_angle, toq_log  = self.LaToq.update( active, CS, VM, params, steer_limited, desired_curvature, desired_curvature_rate, calibrated_pose )
        if 1 in self.multi_lat_spdMethod:
          ind_output_torque, ind_desired_angle, ind_log  = self.LaInd.update( active, CS, VM, params, steer_limited, desired_curvature, desired_curvature_rate, calibrated_pose )
        if 0 in self.multi_lat_spdMethod:
          pid_output_torque, pid_desired_angle, pid_log  = self.LaPid.update( active, CS, VM, params, steer_limited, desired_curvature, desired_curvature_rate, calibrated_pose )

        speed1 = (self.multi_lat_spdBP[0] * (CV.MPH_TO_MS if CS.isMph else CV.KPH_TO_MS))
        speed2 = (self.multi_lat_spdBP[1] * (CV.MPH_TO_MS if CS.isMph else CV.KPH_TO_MS))
        self.lat_atom_timer += 1
        if self.lat_atom_timer > 300:
          self.s_value = CS.vEgo
          self.lat_atom_timer = 0
        if self.s_value < speed1:
          selected = self.multi_lat_spdMethod[0]
        elif speed1 <= self.s_value < speed2:
          selected = self.multi_lat_spdMethod[1]
        else:
          selected = self.multi_lat_spdMethod[2]
        desired_angle = interp( selected, [0, 1, 2, 3], [pid_desired_angle, ind_desired_angle, lqr_desired_angle, toq_desired_angle] )
        output_torque = interp( selected, [0, 1, 2, 3], [pid_output_torque, ind_output_torque, lqr_output_torque, toq_output_torque] )
      elif self.multi_lateral_method == LaMethod.ANGLE_LOWDT:
        if 2 in self.multi_lat_angMethod:
          lqr_output_torque, lqr_desired_angle, lqr_log  = self.LaLqr.update( active, CS, VM, params, steer_limited, desired_curvature, desired_curvature_rate, calibrated_pose )
        if 3 in self.multi_lat_angMethod:
          toq_output_torque, toq_desired_angle, toq_log  = self.LaToq.update( active, CS, VM, params, steer_limited, desired_curvature, desired_curvature_rate, calibrated_pose )
        if 1 in self.multi_lat_angMethod:
          ind_output_torque, ind_desired_angle, ind_log  = self.LaInd.update( active, CS, VM, params, steer_limited, desired_curvature, desired_curvature_rate, calibrated_pose )
        if 0 in self.multi_lat_angMethod:
          pid_output_torque, pid_desired_angle, pid_log  = self.LaPid.update( active, CS, VM, params, steer_limited, desired_curvature, desired_curvature_rate, calibrated_pose )
        ang1 = self.multi_lat_angBP[0]
        ang2 = self.multi_lat_angBP[1]
        self.lat_atom_timer += 1
        if self.lat_atom_timer > 150:
          self.s_value = abs(CS.steeringAngleDeg)
          self.lat_atom_timer = 0
        if self.s_value < ang1:
          selected = self.multi_lat_angMethod[0]
        elif ang1 <= self.s_value < ang2:
          selected = self.multi_lat_angMethod[1]
        else:
          selected = self.multi_lat_angMethod[2]
        desired_angle = interp( selected, [0, 1, 2, 3], [pid_desired_angle, ind_desired_angle, lqr_desired_angle, toq_desired_angle] )
        output_torque = interp( selected, [0, 1, 2, 3], [pid_output_torque, ind_output_torque, lqr_output_torque, toq_output_torque] )
      elif self.multi_lateral_method == LaMethod.ANGLE_INTERP:
        selected = 4
        output_torque, desired_angle  =  self.method_angle( active, CS, VM, params, steer_limited, desired_curvature, desired_curvature_rate, calibrated_pose )
      elif self.multi_lateral_method == LaMethod.SPEED_INTERP:
        selected = 4
        output_torque, desired_angle  =  self.method_speed( active, CS, VM, params, steer_limited, desired_curvature, desired_curvature_rate, calibrated_pose )


      output_torque = clip( output_torque, -self.steer_max, self.steer_max )

      # 2. log
      atom_log.active = True
      if selected == 0:
        atom_log.saturated = pid_log.saturated
        atom_log.steeringAngleDeg = pid_log.steeringAngleDeg
        atom_log.p2 = pid_log.p
        atom_log.i2 = pid_log.i
        atom_log.f2 = pid_log.f
      elif selected == 1:
        atom_log.saturated = ind_log.saturated
        atom_log.steeringAngleDeg = ind_log.steeringAngleDeg
        atom_log.rateSetPoint = ind_log.rateSetPoint
        atom_log.accelSetPoint = ind_log.accelSetPoint
        atom_log.accelError = ind_log.accelError
        atom_log.delayedOutput = ind_log.delayedOutput
        atom_log.delta = ind_log.delta
      elif selected == 2:
        atom_log.i = lqr_log.i
        atom_log.saturated = lqr_log.saturated
        atom_log.steeringAngleDeg = lqr_log.steeringAngleDeg
        atom_log.lqrOutput = lqr_log.lqrOutput
      elif selected == 3:
        atom_log.p1 = toq_log.p
        atom_log.i1 = toq_log.i
        atom_log.d1 = toq_log.d
        atom_log.f1 = toq_log.f


      atom_log.output = output_torque
      atom_log.selected = selected

    return output_torque, desired_angle, atom_log
