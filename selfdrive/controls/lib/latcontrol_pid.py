import math

from cereal import log
from openpilot.selfdrive.controls.lib.latcontrol import LatControl
from openpilot.common.pid import PIDController

from openpilot.common.params import Params
from decimal import Decimal

class LatControlPID(LatControl):
  def __init__(self, CP, CI):
    super().__init__(CP, CI)
    self.pid = PIDController((CP.lateralTuning.pid.kpBP, CP.lateralTuning.pid.kpV),
                             (CP.lateralTuning.pid.kiBP, CP.lateralTuning.pid.kiV),
                             k_f=CP.lateralTuning.pid.kf, k_d=CP.lateralTuning.pid.kd,
                             pos_limit=self.steer_max, neg_limit=-self.steer_max)
    self.get_steer_feedforward = CI.get_steer_feedforward_function()

    self.mpc_frame = 0
    self.params = Params()

    self.live_tune_enabled = False

    self.lp_timer = 0

  def reset(self):
    super().reset()
    self.pid.reset()

  # live tune referred to kegman's 
  def live_tune(self):
    self.mpc_frame += 1
    if self.mpc_frame % 300 == 0:
      self.steerKpV = float(Decimal(self.params.get("PidKp", encoding="utf8")) * Decimal('0.01'))
      self.steerKiV = float(Decimal(self.params.get("PidKi", encoding="utf8")) * Decimal('0.001'))
      self.steerKf = float(Decimal(self.params.get("PidKf", encoding="utf8")) * Decimal('0.00001'))
      self.steerKd = float(Decimal(self.params.get("PidKd", encoding="utf8")) * Decimal('0.01'))
      self.pid = PIDController(([0., 9.], [0.1, self.steerKpV]),
                          ([0., 9.], [0.01, self.steerKiV]),
                          k_f=self.steerKf, k_d=self.steerKd,
                          pos_limit=self.steer_max, neg_limit=-self.steer_max)
      self.mpc_frame = 0

  def update(self, active, CS, VM, params, steer_limited, desired_curvature, desired_curvature_rate, calibrated_pose):
    self.lp_timer += 1
    if self.lp_timer > 100:
      self.lp_timer = 0
      self.live_tune_enabled = self.params.get_bool("KisaLiveTunePanelEnable")
    if self.live_tune_enabled:
      self.live_tune()

    pid_log = log.ControlsState.LateralPIDState.new_message()
    pid_log.steeringAngleDeg = float(CS.steeringAngleDeg)
    pid_log.steeringRateDeg = float(CS.steeringRateDeg)

    angle_steers_des_no_offset = math.degrees(VM.get_steer_from_curvature(-desired_curvature, CS.vEgo, params.roll))
    angle_steers_des = angle_steers_des_no_offset + params.angleOffsetDeg
    error = angle_steers_des - CS.steeringAngleDeg

    pid_log.steeringAngleDesiredDeg = angle_steers_des
    pid_log.angleError = error
    if not active:
      output_steer = 0.0
      pid_log.active = False
      self.pid.reset()
    else:
      # offset does not contribute to resistive torque
      steer_feedforward = self.get_steer_feedforward(angle_steers_des_no_offset, CS.vEgo)

      output_steer = self.pid.update(error, override=CS.steeringPressed,
                                     feedforward=steer_feedforward, speed=CS.vEgo)
      pid_log.active = True
      pid_log.p = self.pid.p
      pid_log.i = self.pid.i
      pid_log.f = self.pid.f
      pid_log.output = output_steer
      pid_log.saturated = self._check_saturation(self.steer_max - abs(output_steer) < 1e-3, CS, steer_limited)

    return output_steer, angle_steers_des, pid_log
