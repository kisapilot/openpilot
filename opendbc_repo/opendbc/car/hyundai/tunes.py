#!/usr/bin/env python3
from enum import Enum
from openpilot.common.params import Params

class LatTunes(Enum):
  INDI = 0
  LQR = 1
  PID = 2
  PID_A = 3
  PID_B = 4
  PID_C = 5
  PID_D = 6
  PID_E = 7
  PID_F = 8
  PID_G = 9
  PID_H = 10
  PID_I = 11
  PID_J = 12
  PID_K = 13
  PID_L = 14
  PID_M = 15
  TORQUE = 16


###### LAT ######
def set_lat_tune(tune, name, max_lat_accel=2.5, FRICTION=.1):
  params = Params()
  if name == LatTunes.TORQUE:
    TorqueKp = params.get("TorqueKp", return_default=True) * 0.1
    TorqueKf = params.get("TorqueKf", return_default=True) * 0.1
    TorqueKi = params.get("TorqueKi", return_default=True) * 0.1
    TorqueKd = params.get("TorqueKd", return_default=True) * 0.1
    TorqueFriction = params.get("TorqueFriction", return_default=True) * 0.001
    max_lat_accel = params.get("TorqueMaxLatAccel", return_default=True) * 0.1
    steer_ang_deadzone = params.get("TorqueAngDeadZone", return_default=True) * 0.1
    tune.init('torque')
    tune.torque.kp = TorqueKp # 1.0
    tune.torque.kf = TorqueKf # 1.0
    tune.torque.ki = TorqueKi # 0.3
    tune.torque.kd = TorqueKd # 0.0
    tune.torque.friction = TorqueFriction
    tune.torque.steeringAngleDeadzoneDeg = steer_ang_deadzone
  elif name == LatTunes.LQR:
    Scale = params.get("Scale", return_default=True) * 1.0
    LqrKi = params.get("LqrKi", return_default=True) * 0.001
    DcGain = params.get("DcGain", return_default=True) * 0.00001
    tune.init('lqr')
    tune.lqr.scale = Scale
    tune.lqr.ki = LqrKi
    tune.lqr.a = [0., 1., -0.22619643, 1.21822268]
    tune.lqr.b = [-1.92006585e-04, 3.95603032e-05]
    tune.lqr.c = [1., 0.]
    tune.lqr.k = [-110., 451.]
    tune.lqr.l = [0.33, 0.318]
    tune.lqr.dcGain = DcGain
  elif name == LatTunes.INDI:
    InnerLoopGain = params.get("InnerLoopGain", return_default=True) * 0.1
    OuterLoopGain = params.get("OuterLoopGain", return_default=True) * 0.1
    TimeConstant = params.get("TimeConstant", return_default=True) * 0.1
    ActuatorEffectiveness = params.get("ActuatorEffectiveness", return_default=True) * 0.1
    tune.init('indi')
    tune.indi.innerLoopGainBP = [0.]
    tune.indi.innerLoopGainV = [InnerLoopGain] # 4.0, third tune. Highest value that still gives smooth control. Effects turning into curves.
    tune.indi.outerLoopGainBP = [0.]
    tune.indi.outerLoopGainV = [OuterLoopGain] # 3.0, forth tune. Highest value that still gives smooth control. Effects lane centering.
    tune.indi.timeConstantBP = [0.]
    tune.indi.timeConstantV = [TimeConstant] # 1.0, second tune. Lowest value with smooth actuation. Avoid the noise of actuator gears thrashing.
    tune.indi.actuatorEffectivenessBP = [0.]
    tune.indi.actuatorEffectivenessV = [ActuatorEffectiveness] # 1.0, first tune. Lowest value without oversteering. May vary with speed.
    # actuatorEffectiveness
      # As effectiveness increases, actuation strength decreases
      # Too high: weak, sloppy lane centering, slow oscillation, can't follow high curvature, high steering error causes snappy corrections
      # Too low: overpower, saturation, jerky, fast oscillation
      # Just right: Highest still able to maintain good lane centering.
    # timeConstant
      # Extend exponential decay of prior output steer
      # Too high: sloppy lane centering
      # Too low: noisy actuation, responds to every bump, maybe unable to maintain lane center due to rapid actuation
      # Just right: above noisy actuation and lane centering instability
    # innerLoopGain
      # Steer rate error gain
      # Too high: jerky oscillation in high curvature
      # Too low: sloppy, cannot accomplish desired steer angle
      # Just right: brief snap on entering high curvature
    # outerLoopGain
      # Steer error gain
      # Too high: twitchy hyper lane centering, oversteering
      # Too low: sloppy, all over lane
      # Just right: crisp lane centering
  elif 'PID' in str(name):
    if name == LatTunes.PID:
      PidKp = params.get("PidKp", return_default=True) * 0.01
      PidKi = params.get("PidKi", return_default=True) * 0.001
      PidKd = params.get("PidKd", return_default=True) * 0.01
      PidKf = params.get("PidKf", return_default=True) * 0.00001
      tune.init('pid')
      tune.pid.kpBP = [0., 9.]
      tune.pid.kpV = [0.1, PidKp]
      tune.pid.kiBP = [0., 9.]
      tune.pid.kiV = [0.01, PidKi]
      tune.pid.kf = PidKf
      tune.pid.kd = PidKd
    elif name == LatTunes.PID_A:
      tune.init('pid')
      tune.pid.kiBP = [0.0]
      tune.pid.kpBP = [0.0]
      tune.pid.kpV = [0.2]
      tune.pid.kiV = [0.05]
      tune.pid.kf = 0.00003
    elif name == LatTunes.PID_C:
      tune.init('pid')
      tune.pid.kiBP = [0.0]
      tune.pid.kpBP = [0.0]
      tune.pid.kpV = [0.6]
      tune.pid.kiV = [0.1]
      tune.pid.kf = 0.00006
    elif name == LatTunes.PID_D:
      tune.init('pid')
      tune.pid.kiBP = [0.0]
      tune.pid.kpBP = [0.0]
      tune.pid.kpV = [0.6]
      tune.pid.kiV = [0.1]
      tune.pid.kf = 0.00007818594
    elif name == LatTunes.PID_F:
      tune.init('pid')
      tune.pid.kiBP = [0.0]
      tune.pid.kpBP = [0.0]
      tune.pid.kpV = [0.723]
      tune.pid.kiV = [0.0428]
      tune.pid.kf = 0.00006
    elif name == LatTunes.PID_G:
      tune.init('pid')
      tune.pid.kiBP = [0.0]
      tune.pid.kpBP = [0.0]
      tune.pid.kpV = [0.18]
      tune.pid.kiV = [0.015]
      tune.pid.kf = 0.00012
    elif name == LatTunes.PID_H:
      tune.init('pid')
      tune.pid.kiBP = [0.0]
      tune.pid.kpBP = [0.0]
      tune.pid.kpV = [0.17]
      tune.pid.kiV = [0.03]
      tune.pid.kf = 0.00006
    elif name == LatTunes.PID_I:
      tune.init('pid')
      tune.pid.kiBP = [0.0]
      tune.pid.kpBP = [0.0]
      tune.pid.kpV = [0.15]
      tune.pid.kiV = [0.05]
      tune.pid.kf = 0.00004
    elif name == LatTunes.PID_J:
      tune.init('pid')
      tune.pid.kiBP = [0.0]
      tune.pid.kpBP = [0.0]
      tune.pid.kpV = [0.19]
      tune.pid.kiV = [0.02]
      tune.pid.kf = 0.00007818594
    elif name == LatTunes.PID_L:
      tune.init('pid')
      tune.pid.kiBP = [0.0]
      tune.pid.kpBP = [0.0]
      tune.pid.kpV = [0.3]
      tune.pid.kiV = [0.05]
      tune.pid.kf = 0.00006
    elif name == LatTunes.PID_M:
      tune.init('pid')
      tune.pid.kiBP = [0.0]
      tune.pid.kpBP = [0.0]
      tune.pid.kpV = [0.3]
      tune.pid.kiV = [0.05]
      tune.pid.kf = 0.00007
    elif name == LatTunes.PID_N:
      tune.init('pid')
      tune.pid.kiBP = [0.0]
      tune.pid.kpBP = [0.0]
      tune.pid.kpV = [0.35]
      tune.pid.kiV = [0.15]
      tune.pid.kf = 0.00007818594
    else:
      raise NotImplementedError('This PID tune does not exist')
  else:
    raise NotImplementedError('This lateral tune does not exist')
