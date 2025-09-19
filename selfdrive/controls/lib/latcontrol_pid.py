import math

from cereal import log
from openpilot.selfdrive.controls.lib.latcontrol import LatControl
from openpilot.common.pid import PIDController

from openpilot.common.params import Params

class LatControlPID(LatControl):
  def __init__(self, CP, CI, dt):
    super().__init__(CP, CI, dt)
    self.pid = PIDController((CP.lateralTuning.pid.kpBP, CP.lateralTuning.pid.kpV),
                             (CP.lateralTuning.pid.kiBP, CP.lateralTuning.pid.kiV),
                             k_f=CP.lateralTuning.pid.kf, k_d=CP.lateralTuning.pid.kd,
                             pos_limit=self.steer_max, neg_limit=-self.steer_max)
    self.get_steer_feedforward = CI.get_steer_feedforward_function()

    self.mpc_frame = 0
    self.params = Params()

    self.live_tune_enabled = False

    self.lp_timer = 0


  # live tune referred to kegman's 
  def live_tune(self):
    self.mpc_frame += 1
    if self.mpc_frame % 300 == 0:
      self.steerKpV = self.params.get("PidKp", return_default=True) * 0.01
      self.steerKiV = self.params.get("PidKi", return_default=True) * 0.001
      self.steerKf = self.params.get("PidKf", return_default=True) * 0.00001
      self.steerKd = self.params.get("PidKd", return_default=True) * 0.01
      self.pid = PIDController(([0., 9.], [0.1, self.steerKpV]),
                          ([0., 9.], [0.01, self.steerKiV]),
                          k_f=self.steerKf, k_d=self.steerKd,
                          pos_limit=self.steer_max, neg_limit=-self.steer_max)
      self.mpc_frame = 0


  def update(self, active, CS, VM, params, steer_limited_by_safety, desired_curvature, curvature_limited, lat_delay, desired_curvature_rate):
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
      output_torque = 0.0
      pid_log.active = False

    else:
      # offset does not contribute to resistive torque
      ff = self.get_steer_feedforward(angle_steers_des_no_offset, CS.vEgo)
      freeze_integrator = steer_limited_by_safety or CS.steeringPressed or CS.vEgo < 5

      output_torque = self.pid.update(error,
                                feedforward=ff,
                                speed=CS.vEgo,
                                freeze_integrator=freeze_integrator)

      pid_log.active = True
      pid_log.p = float(self.pid.p)
      pid_log.i = float(self.pid.i)
      pid_log.f = float(self.pid.f)
      pid_log.output = float(output_torque)
      pid_log.saturated = bool(self._check_saturation(self.steer_max - abs(output_torque) < 1e-3, CS, steer_limited_by_safety, curvature_limited))

    return output_torque, angle_steers_des, pid_log
