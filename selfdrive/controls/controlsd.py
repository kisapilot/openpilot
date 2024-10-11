#!/usr/bin/env python3
import math
from typing import SupportsFloat

from cereal import car, log
import cereal.messaging as messaging
from openpilot.common.conversions import Conversions as CV
from openpilot.common.params import Params
from openpilot.common.realtime import config_realtime_process, Priority, Ratekeeper, DT_CTRL
from openpilot.common.swaglog import cloudlog

from opendbc.car.car_helpers import get_car_interface
from openpilot.selfdrive.controls.lib.drive_helpers import clip_curvature, get_lag_adjusted_curvature
from openpilot.selfdrive.controls.lib.latcontrol import LatControl, MIN_LATERAL_CONTROL_SPEED
from openpilot.selfdrive.controls.lib.latcontrol_pid import LatControlPID
from openpilot.selfdrive.controls.lib.latcontrol_indi import LatControlINDI
from openpilot.selfdrive.controls.lib.latcontrol_lqr import LatControlLQR
from openpilot.selfdrive.controls.lib.latcontrol_angle import LatControlAngle, STEER_ANGLE_SATURATION_THRESHOLD
from openpilot.selfdrive.controls.lib.latcontrol_torque import LatControlTorque
from openpilot.selfdrive.controls.lib.latcontrol_atom import LatControlATOM
from openpilot.selfdrive.controls.lib.longcontrol import LongControl
from openpilot.selfdrive.controls.lib.vehicle_model import VehicleModel
from openpilot.selfdrive.locationd.helpers import PoseCalibrator, Pose

from openpilot.common.numpy_fast import interp

from decimal import Decimal
import openpilot.common.log as trace1

USE_LEGACY_LANE_MODEL = int(Params().get("UseLegacyLaneModel", encoding="utf8")) if Params().get("UseLegacyLaneModel", encoding="utf8") is not None else 0

State = log.SelfdriveState.OpenpilotState
if USE_LEGACY_LANE_MODEL:
  LaneChangeState = log.LateralPlan.LaneChangeState
  LaneChangeDirection = log.LateralPlan.LaneChangeDirection
else:
  LaneChangeState = log.LaneChangeState
  LaneChangeDirection = log.LaneChangeDirection

ACTUATOR_FIELDS = tuple(car.CarControl.Actuators.schema.fields.keys())

class Controls:
  def __init__(self) -> None:
    self.params = Params()
    cloudlog.info("controlsd is waiting for CarParams")
    self.CP = messaging.log_from_bytes(self.params.get("CarParams", block=True), car.CarParams)
    cloudlog.info("controlsd got CarParams")

    self.CI = get_car_interface(self.CP)

    self.sm = messaging.SubMaster(['liveParameters', 'liveTorqueParameters', 'modelV2', 'selfdriveState',
                                   'liveCalibration', 'livePose', 'longitudinalPlan', 'carState', 'carOutput',
                                   'driverMonitoringState', 'onroadEvents', 'driverAssistance', 'lateralPlan', 'radarState', 'liveENaviData'], poll='selfdriveState')
    self.pm = messaging.PubMaster(['carControl', 'controlsState'])

    self.steer_limited = False
    self.desired_curvature = 0.0

    # read params
    self.is_metric = self.params.get_bool("IsMetric")
    self.no_mdps_mods = self.params.get_bool("NoSmartMDPS")

    self.pose_calibrator = PoseCalibrator()
    self.calibrated_pose: Pose|None = None

    self.LoC = LongControl(self.CP)
    self.VM = VehicleModel(self.CP)
    self.LaC: LatControl

    self.lateral_control_method = -1
    if self.CP.steerControlType == car.CarParams.SteerControlType.angle:
      self.LaC = LatControlAngle(self.CP, self.CI)
      self.lateral_control_method = 5
    elif self.CP.lateralTuning.which() == 'pid':
      self.LaC = LatControlPID(self.CP, self.CI)
      self.lateral_control_method = 0
    elif self.CP.lateralTuning.which() == 'indi':
      self.LaC = LatControlINDI(self.CP, self.CI)
      self.lateral_control_method = 1
    elif self.CP.lateralTuning.which() == 'lqr':
      self.LaC = LatControlLQR(self.CP, self.CI)
      self.lateral_control_method = 2
    elif self.CP.lateralTuning.which() == 'torque':
      self.LaC = LatControlTorque(self.CP, self.CI)
      self.lateral_control_method = 3
    elif self.CP.lateralTuning.which() == 'atom':
      self.LaC = LatControlATOM(self.CP, self.CI)
      self.lateral_control_method = 4

    self.new_steerRatio = float(Decimal(self.params.get("SteerRatioAdj", encoding="utf8"))*Decimal('0.01'))
    self.steerRatio_to_send = 0
    self.live_sr = self.params.get_bool("KisaLiveSteerRatio")
    self.live_sr_percent = int(self.params.get("LiveSteerRatioPercent", encoding="utf8"))

    self.ready_timer = 0
    self.osm_speedlimit_enabled = self.params.get_bool("OSMSpeedLimitEnable")
    try:
      self.roadname_and_slc = self.params.get("RoadList", encoding="utf8").strip().splitlines()[1].split(',')
    except:
      self.roadname_and_slc = ""
      pass

    self.var_cruise_speed_factor = 0
    self.cruise_spamming_level = list(map(int, self.params.get("CruiseSpammingLevel", encoding="utf8").split(',')))
    self.cruise_spamming_spd = list(map(int, self.params.get("CruiseSpammingSpd", encoding="utf8").split(',')))
    self.desired_angle_deg = 0
    self.navi_selection = int(self.params.get("KISANaviSelect", encoding="utf8"))
    self.legacy_lane_mode = int(self.params.get("UseLegacyLaneModel", encoding="utf8"))
    self.standstill_elapsed_time = 0.0

  def update(self):
    self.sm.update(15)
    if self.sm.updated["liveCalibration"]:
      self.pose_calibrator.feed_live_calib(self.sm['liveCalibration'])
    if self.sm.updated["livePose"]:
      device_pose = Pose.from_live_pose(self.sm['livePose'])
      self.calibrated_pose = self.pose_calibrator.build_calibrated_pose(device_pose)

  def state_control(self):
    CS = self.sm['carState']

    # Update VehicleModel
    lp = self.sm['liveParameters']
    x = max(lp.stiffnessFactor, 0.1)

    if self.live_sr:
      sr = max(lp.steerRatio, 0.1)
      if self.live_sr_percent != 0:
        sr = sr * (1+(0.01*self.live_sr_percent))
    else:
     sr = max(self.new_steerRatio, 0.1)
    self.VM.update_params(x, sr)

    self.steerRatio_to_send = sr

    # Update Torque Params
    if self.CP.lateralTuning.which() == 'torque':
      torque_params = self.sm['liveTorqueParameters']
      if self.sm.all_checks(['liveTorqueParameters']) and torque_params.useParams:
        self.LaC.update_live_torque_params(torque_params.latAccelFactorFiltered, torque_params.latAccelOffsetFiltered,
                                           torque_params.frictionCoefficientFiltered)

    long_plan = self.sm['longitudinalPlan']
    model_v2 = self.sm['modelV2']

    CC = car.CarControl.new_message()
    CC.enabled = self.sm['selfdriveState'].enabled

    # Check which actuators can be enabled
    standstill = (abs(CS.vEgo) <= max(self.CP.minSteerSpeed, MIN_LATERAL_CONTROL_SPEED) and self.no_mdps_mods) or CS.standstill
    CC.latActive = self.sm['selfdriveState'].active and not CS.steerFaultTemporary and not CS.steerFaultPermanent and \
                   not standstill and not self.sm['carOutput'].actuatorsOutput.lkasTemporaryOff
    CC.longActive = CC.enabled and not any(e.overrideLongitudinal for e in self.sm['onroadEvents']) and self.CP.openpilotLongitudinalControl

    actuators = CC.actuators
    actuators.longControlState = self.LoC.long_control_state

    # Enable blinkers while lane changing
    lat_plan = self.sm['lateralPlan'] if self.legacy_lane_mode else model_v2.meta
    if lat_plan.laneChangeState != LaneChangeState.off:
      CC.leftBlinker = lat_plan.laneChangeDirection == LaneChangeDirection.left
      CC.rightBlinker = lat_plan.laneChangeDirection == LaneChangeDirection.right

    if not CC.latActive:
      self.LaC.reset()
    if not CC.longActive:
      self.LoC.reset()

    # accel PID loop
    pid_accel_limits = self.CI.get_pid_accel_limits(self.CP, CS.vEgo, CS.vCruise * CV.KPH_TO_MS)
    actuators.accel, actuators.oaccel = self.LoC.update(CC.longActive, CS, long_plan.aTarget, long_plan.shouldStop, pid_accel_limits, long_plan.longitudinalPlanSource, self.sm['carOutput'].actuatorsOutput, self.sm['radarState'])

    # Steering PID loop and lateral MPC
    if self.legacy_lane_mode == 2:
      model_speed = self.sm['lateralPlan'].modelSpeed
      desired_curvature1, self.desired_curvature_rate = get_lag_adjusted_curvature(self.CP, CS.vEgo, lat_plan.psis, lat_plan.curvatures, lat_plan.curvatureRates)
      desired_curvature2 = clip_curvature(CS.vEgo, self.desired_curvature, model_v2.action.desiredCurvature)
      desired_curvature3 = interp(CS.vEgo, [0.3, 1.0], [desired_curvature1, desired_curvature2])
      self.desired_curvature = interp(model_speed, [50, 100], [desired_curvature3, desired_curvature1])
      if lat_plan.laneChangeState != LaneChangeState.off:
        self.desired_curvature = desired_curvature2
    elif self.legacy_lane_mode == 1:
      self.desired_curvature, self.desired_curvature_rate = get_lag_adjusted_curvature(self.CP, CS.vEgo, lat_plan.psis, lat_plan.curvatures, lat_plan.curvatureRates)
      if lat_plan.laneChangeState != LaneChangeState.off:
        self.desired_curvature = clip_curvature(CS.vEgo, self.desired_curvature, model_v2.action.desiredCurvature)
    else:
      self.desired_curvature = clip_curvature(CS.vEgo, self.desired_curvature, model_v2.action.desiredCurvature)
    actuators.curvature = self.desired_curvature
    actuators.steer, actuators.steeringAngleDeg, lac_log = self.LaC.update(CC.latActive, CS, self.VM, lp,
                                                                            self.steer_limited, self.desired_curvature,
                                                                            self.desired_curvature_rate, self.calibrated_pose) # TODO what if not available
    self.desired_angle_deg = actuators.steeringAngleDeg

    # Ensure no NaNs/Infs
    for p in ACTUATOR_FIELDS:
      attr = getattr(actuators, p)
      if not isinstance(attr, SupportsFloat):
        continue

      if not math.isfinite(attr):
        cloudlog.error(f"actuators.{p} not finite {actuators.to_dict()}")
        setattr(actuators, p, 0.0)

    return CC, lac_log

  def publish(self, CC, lac_log):
    CS = self.sm['carState']

    # Orientation and angle rates can be useful for carcontroller
    # Only calibrated (car) frame is relevant for the carcontroller
    if self.calibrated_pose is not None:
      CC.orientationNED = self.calibrated_pose.orientation.xyz.tolist()
      CC.angularVelocity = self.calibrated_pose.angular_velocity.xyz.tolist()

    CC.cruiseControl.override = CC.enabled and not CC.longActive and self.CP.openpilotLongitudinalControl
    CC.cruiseControl.cancel = CS.cruiseState.enabled and (not CC.enabled or not self.CP.pcmCruise)

    speeds = self.sm['longitudinalPlan'].speeds
    if len(speeds):
      CC.cruiseControl.resume = CC.enabled and CS.cruiseState.standstill and speeds[-1] > 0.1

    hudControl = CC.hudControl
    hudControl.setSpeed = float(CS.vCruiseCluster * (CV.KPH_TO_MS if self.is_metric else CV.MPH_TO_MS))
    hudControl.speedVisible = CC.enabled
    hudControl.lanesVisible = CC.enabled
    hudControl.leadVisible = self.sm['longitudinalPlan'].hasLead
    hudControl.leadDistanceBars = self.sm['selfdriveState'].personality.raw + 1
    hudControl.visualAlert = self.sm['selfdriveState'].alertHudVisual

    hudControl.rightLaneVisible = True
    hudControl.leftLaneVisible = True
    if self.sm.valid['driverAssistance']:
      hudControl.leftLaneDepart = self.sm['driverAssistance'].leftLaneDeparture
      hudControl.rightLaneDepart = self.sm['driverAssistance'].rightLaneDeparture

    m_unit = CV.MS_TO_KPH if self.is_metric else CV.MS_TO_MPH
    if len(speeds):
      if CS.vEgo*m_unit < self.cruise_spamming_spd[0]:
        self.var_cruise_speed_factor = self.cruise_spamming_level[0]
      elif self.cruise_spamming_spd[0] <= CS.vEgo*m_unit < self.cruise_spamming_spd[1]:
        self.var_cruise_speed_factor = self.cruise_spamming_level[1]
      elif self.cruise_spamming_spd[1] <= CS.vEgo*m_unit < self.cruise_spamming_spd[2]:
        self.var_cruise_speed_factor = self.cruise_spamming_level[2]
      else:
        self.var_cruise_speed_factor = self.cruise_spamming_level[3]        
      v_future = speeds[self.var_cruise_speed_factor]
      v_future_a = speeds[-1]
    else:
      v_future = 100.0
      v_future_a = 100.0
    v_future_speed= float(v_future * m_unit)
    v_future_speed_a= float(v_future_a * m_unit)
    hudControl.vFuture = v_future_speed
    hudControl.vFutureA = v_future_speed_a

    CO = self.sm['carOutput']
    if self.sm['selfdriveState'].active:
      if self.CP.steerControlType == car.CarParams.SteerControlType.angle:
        self.steer_limited = abs(CC.actuators.steeringAngleDeg - CO.actuatorsOutput.steeringAngleDeg) > \
                             STEER_ANGLE_SATURATION_THRESHOLD
      else:
        self.steer_limited = abs(CC.actuators.steer - CO.actuatorsOutput.steer) > 1e-2

    # TODO: both controlsState and carControl valids should be set by
    #       sm.all_checks(), but this creates a circular dependency

    # controlsState
    dat = messaging.new_message('controlsState')
    dat.valid = CS.canValid
    cs = dat.controlsState

    lp = self.sm['liveParameters']
    steer_angle_without_offset = math.radians(CS.steeringAngleDeg - lp.angleOffsetDeg)
    cs.curvature = -self.VM.calc_curvature(steer_angle_without_offset, CS.vEgo, lp.roll)

    cs.longitudinalPlanMonoTime = self.sm.logMonoTime['longitudinalPlan']
    cs.lateralPlanMonoTime = self.sm.logMonoTime['lateralPlan'] if self.legacy_lane_mode else self.sm.logMonoTime['modelV2']
    cs.desiredCurvature = self.desired_curvature
    cs.desiredCurvatureRate = self.desired_curvature_rate
    cs.longControlState = self.LoC.long_control_state
    cs.upAccelCmd = float(self.LoC.pid.p)
    cs.uiAccelCmd = float(self.LoC.pid.i)
    cs.ufAccelCmd = float(self.LoC.pid.f)
    cs.forceDecel = bool((self.sm['driverMonitoringState'].awarenessStatus < 0.) or
                         (self.sm['selfdriveState'].state == State.softDisabling))

    cs.vFuture = float(v_future_speed)
    cs.vFutureA = float(v_future_speed_a)
    cs.alertTextMsg1 = str(CO.actuatorsOutput.kisaLog1)
    cs.alertTextMsg2 = str(CO.actuatorsOutput.kisaLog2)
    cs.alertTextMsg3 = str(trace1.global_alertTextMsg3)

    if self.osm_speedlimit_enabled or self.navi_selection == 2:
      if self.navi_selection == 2:
        cs.limitSpeedCamera = int(round(self.sm['liveENaviData'].wazeRoadSpeedLimit))
        cs.limitSpeedCameraDist = float(self.sm['liveENaviData'].wazeAlertDistance)
      elif self.osm_speedlimit_enabled:
        cs.limitSpeedCamera = int(round(self.sm['liveMapData'].speedLimit))
        cs.limitSpeedCameraDist = float(self.sm['liveMapData'].speedLimitAheadDistance)
      if self.sm['liveMapData'].currentRoadName in self.roadname_and_slc:
        try:
          r_index = self.roadname_and_slc.index(self.sm['liveMapData'].currentRoadName)
          cs.limitSpeedCamera = float(self.roadname_and_slc[r_index+1])
        except:
          pass
    elif self.navi_selection == 1 and int(self.sm['liveENaviData'].safetySign) not in (20, 21):
      cs.limitSpeedCamera = int(round(self.sm['liveENaviData'].speedLimit))
      cs.limitSpeedCameraDist = float(self.sm['liveENaviData'].safetyDistance)
      cs.mapSign = int(self.sm['liveENaviData'].safetySign)
    else:
      cs.limitSpeedCamera = 0
      cs.limitSpeedCameraDist = 0
    cs.lateralControlMethod = int(self.lateral_control_method)
    cs.steerRatio = float(self.steerRatio_to_send)
    cs.dynamicTRMode = int(self.sm['longitudinalPlan'].dynamicTRMode)
    cs.dynamicTRValue = float(self.sm['longitudinalPlan'].dynamicTRValue)
    cs.accel = float(CO.actuatorsOutput.accel)
    cs.safetySpeed = float(CO.actuatorsOutput.safetySpeed)
    cs.gapBySpeedOn = bool(CO.actuatorsOutput.gapBySpdOnTemp)
    cs.expModeTemp = bool(CO.actuatorsOutput.expModeTemp)
    cs.btnPressing = int(CO.actuatorsOutput.btnPressing)
    cs.autoResvCruisekph = float(CO.actuatorsOutput.autoResvCruisekph)
    cs.resSpeed = float(CO.actuatorsOutput.resSpeed)
    cs.roadLimitSpeedOnTemp = bool(CO.actuatorsOutput.roadLimitSpeedOnTemp)
    cs.standStill = bool(CO.actuatorsOutput.standStill)
    if cs.standStill:
      self.standstill_elapsed_time += DT_CTRL
    else:
      self.standstill_elapsed_time = 0.0
    cs.standStillTimer = int(self.standstill_elapsed_time)    

    CC.needBrake = bool(CO.actuatorsOutput.needBrake)
    CC.lkasTempDisabled = bool(CO.actuatorsOutput.lkasTempDisabled)
    CC.lanechangeManualTimer = int(CO.actuatorsOutput.lanechangeManualTimer)
    CC.emergencyManualTimer = int(CO.actuatorsOutput.emergencyManualTimer)
    CC.standstillResButton = bool(CO.actuatorsOutput.standstillResButton)
    CC.cruiseGapAdjusting = bool(CO.actuatorsOutput.cruiseGapAdjusting)
    CC.onSpeedBumpControl = bool(CO.actuatorsOutput.onSpeedBumpControl)
    CC.onSpeedControl = bool(CO.actuatorsOutput.onSpeedControl)
    CC.curvSpeedControl = bool(CO.actuatorsOutput.curvSpeedControl)
    CC.cutInControl = bool(CO.actuatorsOutput.cutInControl)
    CC.driverSccSetControl = bool(CO.actuatorsOutput.driverSccSetControl)
    CC.autoholdPopupTimer = int(CO.actuatorsOutput.autoholdPopupTimer)
    CC.autoResStarting = bool(CO.actuatorsOutput.autoResStarting)
    CC.e2eStandstill = bool(CO.actuatorsOutput.e2eStandstill)
    CC.modeChangeTimer = int(CO.actuatorsOutput.modeChangeTimer)
    CC.lkasTempDisabledTimer = int(CO.actuatorsOutput.lkasTempDisabledTimer)

    lat_tuning = self.CP.lateralTuning.which()
    if self.CP.steerControlType == car.CarParams.SteerControlType.angle:
      cs.lateralControlState.angleState = lac_log
    elif lat_tuning == 'pid':
      cs.lateralControlState.pidState = lac_log
    elif lat_tuning == 'lqr':
      cs.lateralControlState.lqrState = lac_log
    elif lat_tuning == 'indi':
      cs.lateralControlState.indiState = lac_log
    elif lat_tuning == 'torque':
      cs.lateralControlState.torqueState = lac_log
    elif lat_tuning == 'atom':
      cs.lateralControlState.atomState = lac_log

    if lat_tuning == 'torque':
      cs.steeringAngleDesiredDeg = lac_log.desiredLateralAccel
    else:
      cs.steeringAngleDesiredDeg = self.desired_angle_deg

    self.pm.send('controlsState', dat)

    # carControl
    cc_send = messaging.new_message('carControl')
    cc_send.valid = CS.canValid
    cc_send.carControl = CC
    self.pm.send('carControl', cc_send)

  def run(self):
    rk = Ratekeeper(100, print_delay_threshold=None)
    while True:
      self.update()
      CC, lac_log = self.state_control()
      self.publish(CC, lac_log)
      rk.keep_time()

def main():
  config_realtime_process(4, Priority.CTRL_HIGH)
  controls = Controls()
  controls.run()


if __name__ == "__main__":
  main()
