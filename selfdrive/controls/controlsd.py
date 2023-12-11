#!/usr/bin/env python3
import os
import math
import time
from typing import SupportsFloat

from cereal import car, log
from openpilot.common.numpy_fast import clip, interp
from openpilot.common.realtime import config_realtime_process, Priority, Ratekeeper, DT_CTRL
from openpilot.common.profiler import Profiler
from openpilot.common.params import Params, put_nonblocking, put_bool_nonblocking
import cereal.messaging as messaging
from cereal.visionipc import VisionIpcClient, VisionStreamType
from openpilot.common.conversions import Conversions as CV
from panda import ALTERNATIVE_EXPERIENCE
from openpilot.system.swaglog import cloudlog
from openpilot.system.version import is_release_branch, get_short_branch
from openpilot.selfdrive.boardd.boardd import can_list_to_can_capnp
from openpilot.selfdrive.car.car_helpers import get_car, get_startup_event, get_one_can
from openpilot.selfdrive.controls.lib.lateral_planner import CAMERA_OFFSET, CAMERA_OFFSET_A
from openpilot.selfdrive.controls.lib.drive_helpers import VCruiseHelper, get_lag_adjusted_curvature
from openpilot.selfdrive.controls.lib.latcontrol import LatControl, MIN_LATERAL_CONTROL_SPEED
from openpilot.selfdrive.controls.lib.longcontrol import LongControl
from openpilot.selfdrive.controls.lib.latcontrol_pid import LatControlPID
from openpilot.selfdrive.controls.lib.latcontrol_indi import LatControlINDI
from openpilot.selfdrive.controls.lib.latcontrol_lqr import LatControlLQR
from openpilot.selfdrive.controls.lib.latcontrol_angle import LatControlAngle, STEER_ANGLE_SATURATION_THRESHOLD
from openpilot.selfdrive.controls.lib.latcontrol_torque import LatControlTorque
from openpilot.selfdrive.controls.lib.latcontrol_atom import LatControlATOM
from openpilot.selfdrive.controls.lib.events import Events, ET
from openpilot.selfdrive.controls.lib.alertmanager import AlertManager, set_offroad_alert
from openpilot.selfdrive.controls.lib.vehicle_model import VehicleModel
from openpilot.system.hardware import HARDWARE

from decimal import Decimal

import openpilot.common.log as trace1

SOFT_DISABLE_TIME = 3  # seconds
LDW_MIN_SPEED = 50 * CV.KPH_TO_MS if Params().get_bool("IsMetric") else 31 * CV.MPH_TO_MS
LANE_DEPARTURE_THRESHOLD = 0.1

REPLAY = "REPLAY" in os.environ
SIMULATION = "SIMULATION" in os.environ
TESTING_CLOSET = "TESTING_CLOSET" in os.environ
IGNORE_PROCESSES = {"loggerd", "encoderd", "statsd", "mapd", "navd"}

ThermalStatus = log.DeviceState.ThermalStatus
State = log.ControlsState.OpenpilotState
PandaType = log.PandaState.PandaType
Desire = log.LateralPlan.Desire
LaneChangeState = log.LateralPlan.LaneChangeState
LaneChangeDirection = log.LateralPlan.LaneChangeDirection
EventName = car.CarEvent.EventName
ButtonType = car.CarState.ButtonEvent.Type
SafetyModel = car.CarParams.SafetyModel
GearShifter = car.CarState.GearShifter

IGNORED_SAFETY_MODES = (SafetyModel.silent, SafetyModel.noOutput)
CSID_MAP = {"1": EventName.roadCameraError, "2": EventName.wideRoadCameraError, "0": EventName.driverCameraError}
ACTUATOR_FIELDS = tuple(car.CarControl.Actuators.schema.fields.keys())
ACTIVE_STATES = (State.enabled, State.softDisabling, State.overriding)
ENABLED_STATES = (State.preEnabled, *ACTIVE_STATES)


class Controls:
  def __init__(self, CI=None):
    config_realtime_process(4, Priority.CTRL_HIGH)

    # Ensure the current branch is cached, otherwise the first iteration of controlsd lags
    self.branch = get_short_branch("")

    # Setup sockets
    self.pm = messaging.PubMaster(['sendcan', 'controlsState', 'carState',
                                   'carControl', 'carEvents', 'carParams'])

    self.sensor_packets = ["accelerometer", "gyroscope"]
    self.camera_packets = ["roadCameraState", "driverCameraState", "wideRoadCameraState"]

    can_timeout = None if os.environ.get('NO_CAN_TIMEOUT', False) else 20
    self.can_sock = messaging.sub_sock('can', timeout=can_timeout)

    self.log_sock = messaging.sub_sock('androidLog')

    self.params = Params()
    ignore = self.sensor_packets + ['testJoystick']
    if True:
      ignore += ['driverCameraState', 'managerState', 'driverMonitoringState']
    self.sm = messaging.SubMaster(['deviceState', 'pandaStates', 'peripheralState', 'modelV2', 'liveCalibration',
                                   'driverMonitoringState', 'longitudinalPlan', 'lateralPlan', 'liveLocationKalman',
                                   'managerState', 'liveParameters', 'radarState', 'liveTorqueParameters',
                                   'testJoystick', 'liveENaviData', 'liveMapData'] + self.camera_packets + self.sensor_packets,
                                  ignore_alive=ignore, ignore_avg_freq=['radarState', 'testJoystick', 'liveENaviData', 'liveMapData'])

    if CI is None:
      # wait for one pandaState and one CAN packet
      print("Waiting for CAN messages...")
      get_one_can(self.can_sock)

      num_pandas = len(messaging.recv_one_retry(self.sm.sock['pandaStates']).pandaStates)
      experimental_long_allowed = self.params.get_bool("ExperimentalLongitudinalEnabled")
      self.CI, self.CP = get_car(self.can_sock, self.pm.sock['sendcan'], experimental_long_allowed, num_pandas)
    else:
      self.CI, self.CP = CI, CI.CP

    self.joystick_mode = self.params.get_bool("JoystickDebugMode") or self.CP.notCar

    # set alternative experiences from parameters
    self.disengage_on_accelerator = self.params.get_bool("DisengageOnAccelerator")
    self.CP.alternativeExperience = 0
    if not self.disengage_on_accelerator:
      self.CP.alternativeExperience |= ALTERNATIVE_EXPERIENCE.DISABLE_DISENGAGE_ON_GAS

    # read params
    self.is_metric = self.params.get_bool("IsMetric")
    self.is_ldw_enabled = self.params.get_bool("IsLdwEnabled")
    openpilot_enabled_toggle = self.params.get_bool("OpenpilotEnabledToggle")
    passive = self.params.get_bool("Passive") or not openpilot_enabled_toggle

    self.auto_enabled = self.params.get_bool("AutoEnable") and self.params.get_bool("UFCModeEnabled")
    self.variable_cruise = self.params.get_bool("KisaVariableCruise")
    self.cruise_over_maxspeed = self.params.get_bool("CruiseOverMaxSpeed")
    self.cruise_road_limit_spd_enabled = self.params.get_bool("CruiseSetwithRoadLimitSpeedEnabled")
    self.cruise_road_limit_spd_offset = int(self.params.get("CruiseSetwithRoadLimitSpeedOffset", encoding="utf8"))
    self.stock_lkas_on_disengaged_status = self.params.get_bool("StockLKASEnabled")
    self.no_mdps_mods = self.params.get_bool("NoSmartMDPS")
    self.ufc_mode = self.params.get_bool("UFCModeEnabled")

    self.cruise_road_limit_spd_switch = True
    self.cruise_road_limit_spd_switch_prev = 0
    
    self.long_alt = int(self.params.get("KISALongAlt", encoding="utf8"))
    self.exp_long_enabled = self.params.get_bool("ExperimentalLongitudinalEnabled")

    # detect sound card presence and ensure successful init
    sounds_available = HARDWARE.get_sound_card_online()

    car_recognized = self.CP.carName != 'mock'

    controller_available = self.CI.CC is not None and not passive and not self.CP.dashcamOnly
    self.read_only = not car_recognized or not controller_available or self.CP.dashcamOnly
    if self.read_only:
      safety_config = car.CarParams.SafetyConfig.new_message()
      safety_config.safetyModel = car.CarParams.SafetyModel.noOutput
      self.CP.safetyConfigs = [safety_config]

    # Write previous route's CarParams
    prev_cp = self.params.get("CarParamsPersistent")
    if prev_cp is not None:
      self.params.put("CarParamsPrevRoute", prev_cp)

    # Write CarParams for radard
    cp_bytes = self.CP.to_bytes()
    self.params.put("CarParams", cp_bytes)
    put_nonblocking("CarParamsCache", cp_bytes)
    put_nonblocking("CarParamsPersistent", cp_bytes)

    # cleanup old params
    if not self.CP.experimentalLongitudinalAvailable or is_release_branch():
      self.params.remove("ExperimentalLongitudinalEnabled")
    if not self.CP.openpilotLongitudinalControl:
      self.params.remove("ExperimentalMode")

    self.CC = car.CarControl.new_message()
    self.CS_prev = car.CarState.new_message()
    self.AM = AlertManager()
    self.events = Events()

    self.LoC = LongControl(self.CP)
    self.VM = VehicleModel(self.CP)

    self.LaC: LatControl
    self.lateral_control_method = 0
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

    self.initialized = False
    self.state = State.disabled
    self.enabled = False
    self.active = False
    self.soft_disable_timer = 0
    self.mismatch_counter = 0
    self.cruise_mismatch_counter = 0
    self.can_rcv_timeout_counter = 0      # conseuctive timeout count
    self.can_rcv_cum_timeout_counter = 0  # cumulative timeout count
    self.last_blinker_frame = 0
    self.last_steering_pressed_frame = 0
    self.distance_traveled = 0
    self.last_functional_fan_frame = 0
    self.events_prev = []
    self.current_alert_types = [ET.PERMANENT]
    self.logged_comm_issue = None
    self.not_running_prev = None
    self.last_actuators = car.CarControl.Actuators.new_message()
    self.steer_limited = False
    self.desired_curvature = 0.0
    self.desired_curvature_rate = 0.0
    self.experimental_mode = False
    self.v_cruise_helper = VCruiseHelper(self.CP)
    self.recalibrating_seen = False

    # TODO: no longer necessary, aside from process replay
    self.sm['liveParameters'].valid = True
    self.can_log_mono_time = 0

    self.startup_event = get_startup_event(car_recognized, controller_available, len(self.CP.carFw) > 0)

    if not sounds_available:
      self.events.add(EventName.soundsUnavailable, static=True)
    if not car_recognized:
      self.events.add(EventName.carUnrecognized, static=True)
      if len(self.CP.carFw) > 0:
        set_offroad_alert("Offroad_CarUnrecognized", True)
      else:
        set_offroad_alert("Offroad_NoFirmware", True)
    elif self.read_only:
      self.events.add(EventName.dashcamMode, static=True)
    elif self.joystick_mode:
      self.events.add(EventName.joystickDebug, static=True)
      self.startup_event = None

    # controlsd is driven by can recv, expected at 100Hz
    self.rk = Ratekeeper(100, print_delay_threshold=None)
    self.prof = Profiler(False)  # off by default

    self.hkg_stock_lkas = True
    self.hkg_stock_lkas_timer = 0

    self.mpc_frame = 0
    self.mpc_frame_sr = 0

    self.steerRatio_Max = float(Decimal(self.params.get("SteerRatioMaxAdj", encoding="utf8")) * Decimal('0.01'))
    self.steer_angle_range = [5, 30]
    self.steerRatio_range = [self.CP.steerRatio, self.steerRatio_Max]
    self.new_steerRatio = self.CP.steerRatio
    self.new_steerRatio_prev = self.CP.steerRatio
    self.steerRatio_to_send = 0
    self.live_sr = self.params.get_bool("KisaLiveSteerRatio")
    self.live_sr_percent = int(self.params.get("LiveSteerRatioPercent", encoding="utf8"))

    self.second = 0.0
    self.second2 = 0.0
    self.map_enabled = False
    self.lane_change_delay = int(self.params.get("KisaAutoLaneChangeDelay", encoding="utf8"))
    self.auto_enable_speed = int(self.params.get("AutoEnableSpeed", encoding="utf8"))
    self.e2e_long_alert_prev = True
    self.unsleep_mode_alert_prev = True
    self.donotdisturb_mode_alert_prev = True
    self.stock_navi_info_enabled = self.params.get_bool("StockNaviSpeedEnabled")
    self.ignore_can_error_on_isg = self.params.get_bool("IgnoreCANErroronISG")
    self.ready_timer = 0
    self.osm_waze_spdlimit_offset = int(self.params.get("KisaSpeedLimitOffset", encoding="utf8"))
    self.osm_waze_spdlimit_offset_option = int(self.params.get("KisaSpeedLimitOffsetOption", encoding="utf8"))
    self.osm_speedlimit_enabled = self.params.get_bool("OSMSpeedLimitEnable")
    self.osm_waze_speedlimit = 255
    self.osm_waze_off_spdlimit_init = False
    self.safety_speed = 0
    self.lkas_temporary_off = False
    self.gap_by_spd_on_temp = True
    self.exp_mode_temp = True
    self.btn_pressing = 0
    try:
      self.roadname_and_slc = self.params.get("RoadList", encoding="utf8").strip().splitlines()[1].split(',')
    except:
      self.roadname_and_slc = ""
      pass

    #self.var_cruise_speed_factor = int(self.params.get("VarCruiseSpeedFactor", encoding="utf8"))
    self.var_cruise_speed_factor = 0
    self.cruise_spamming_level = list(map(int, self.params.get("CruiseSpammingLevel", encoding="utf8").split(',')))
    self.cruise_spamming_spd = list(map(int, self.params.get("CruiseSpammingSpd", encoding="utf8").split(',')))
    self.desired_angle_deg = 0
    self.navi_selection = int(self.params.get("KISANaviSelect", encoding="utf8"))

    self.osm_waze_custom_spdlimit_c = list(map(int, self.params.get("OSMCustomSpeedLimitC", encoding="utf8").split(',')))
    self.osm_waze_custom_spdlimit_t = list(map(int, self.params.get("OSMCustomSpeedLimitT", encoding="utf8").split(',')))

    self.pandaState_safetyModel = ""
    self.interface_safetyModel = ""
    self.rx_checks_ok = False
    self.mismatch_counter_ok = False

  def auto_enable(self, CS):
    if self.state != State.enabled:
      if CS.cruiseState.available and CS.vEgo > self.auto_enable_speed * (CV.KPH_TO_MS if self.is_metric else CV.MPH_TO_MS) and CS.gearShifter == GearShifter.drive and \
       self.sm['liveCalibration'].calStatus != log.LiveCalibrationData.Status.uncalibrated and self.initialized and self.ready_timer > 300:
        self.events.add( EventName.pcmEnable )

  def set_initial_state(self):
    if REPLAY:
      controls_state = Params().get("ReplayControlsState")
      if controls_state is not None:
        with log.ControlsState.from_bytes(controls_state) as controls_state:
          self.v_cruise_helper.v_cruise_kph = controls_state.vCruise

      if any(ps.controlsAllowed for ps in self.sm['pandaStates']):
        self.state = State.enabled

  def update_events(self, CS):
    """Compute carEvents from carState"""

    self.events.clear()

    # Add startup event
    if self.startup_event is not None:
      self.events.add(self.startup_event)
      self.startup_event = None

    # Don't add any more events if not initialized
    if not self.initialized:
      self.events.add(EventName.controlsInitializing)
      return

    # no more events while in dashcam mode
    if self.read_only:
      return

    # Block resume if cruise never previously enabled
    resume_pressed = any(be.type in (ButtonType.accelCruise, ButtonType.resumeCruise) for be in CS.buttonEvents)
    if not self.CP.pcmCruise and not self.v_cruise_helper.v_cruise_initialized and resume_pressed:
      self.events.add(EventName.resumeBlocked)

    # Disable on rising edge of accelerator or brake. Also disable on brake when speed > 0
    if not self.ufc_mode:
      if (CS.gasPressed and not self.CS_prev.gasPressed and self.disengage_on_accelerator) or \
        (CS.brakePressed and (not self.CS_prev.brakePressed or not CS.standstill)) or \
        (CS.regenBraking and (not self.CS_prev.regenBraking or not CS.standstill)):
        self.events.add(EventName.pedalPressed)

      if CS.brakePressed and CS.standstill:
        self.events.add(EventName.preEnableStandstill)

      if CS.gasPressed:
        self.events.add(EventName.gasPressedOverride)

    if not self.CP.notCar:
      self.events.add_from_msg(self.sm['driverMonitoringState'].events)

    # Add car events, ignore if CAN isn't valid
    if CS.canValid:
      self.events.add_from_msg(CS.events)

    # Create events for temperature, disk space, and memory
    if self.sm['deviceState'].thermalStatus >= ThermalStatus.red:
      self.events.add(EventName.overheat)
    if self.sm['deviceState'].freeSpacePercent < 0 and not SIMULATION:
      # under 7% of space free no enable allowed
      self.events.add(EventName.outOfSpace)
    if self.sm['deviceState'].memoryUsagePercent > 90 and not SIMULATION:
      self.events.add(EventName.lowMemory)

    # TODO: enable this once loggerd CPU usage is more reasonable
    #cpus = list(self.sm['deviceState'].cpuUsagePercent)
    #if max(cpus, default=0) > 95 and not SIMULATION:
    #  self.events.add(EventName.highCpuUsage)

    # Alert if fan isn't spinning for 5 seconds
    if self.sm['peripheralState'].pandaType != log.PandaState.PandaType.unknown:
      if self.sm['peripheralState'].fanSpeedRpm < 500 and self.sm['deviceState'].fanSpeedPercentDesired > 50:
        # allow enough time for the fan controller in the panda to recover from stalls
        if (self.sm.frame - self.last_functional_fan_frame) * DT_CTRL > 15.0:
          self.events.add(EventName.fanMalfunction)
      else:
        self.last_functional_fan_frame = self.sm.frame

    # Handle calibration status
    cal_status = self.sm['liveCalibration'].calStatus
    if cal_status != log.LiveCalibrationData.Status.calibrated:
      if cal_status == log.LiveCalibrationData.Status.uncalibrated:
        self.events.add(EventName.calibrationIncomplete)
      elif cal_status == log.LiveCalibrationData.Status.recalibrating:
        if not self.recalibrating_seen:
          set_offroad_alert("Offroad_Recalibration", True)
        self.recalibrating_seen = True
        self.events.add(EventName.calibrationRecalibrating)
      else:
        self.events.add(EventName.calibrationInvalid)

    # Handle lane change
    if not self.lkas_temporary_off:
      if self.sm['lateralPlan'].laneChangeState == LaneChangeState.preLaneChange:
        direction = self.sm['lateralPlan'].laneChangeDirection
        if (CS.leftBlindspot and direction == LaneChangeDirection.left) or \
          (CS.rightBlindspot and direction == LaneChangeDirection.right):
          self.events.add(EventName.laneChangeBlocked)
        else:
          if direction == LaneChangeDirection.left:
            if self.lane_change_delay == 0:
              self.events.add(EventName.preLaneChangeLeft)
            else:
              self.events.add(EventName.laneChange)
          else:
            if self.lane_change_delay == 0:
              self.events.add(EventName.preLaneChangeRight)
            else:
              self.events.add(EventName.laneChange)
      elif self.sm['lateralPlan'].laneChangeState in (LaneChangeState.laneChangeStarting,
                                                      LaneChangeState.laneChangeFinishing):
        self.events.add(EventName.laneChange)

    for i, pandaState in enumerate(self.sm['pandaStates']):
      # All pandas must match the list of safetyConfigs, and if outside this list, must be silent or noOutput
      if i < len(self.CP.safetyConfigs):
        #print('safety_mismatch1')
        safety_mismatch = pandaState.safetyModel != self.CP.safetyConfigs[i].safetyModel or \
                          pandaState.safetyParam != self.CP.safetyConfigs[i].safetyParam or \
                          pandaState.alternativeExperience != self.CP.alternativeExperience
        #print('safetyModel={}, safetyParam={}, alternativeExperience={}'.format(self.CP.safetyConfigs[i].safetyModel, self.CP.safetyConfigs[i].safetyParam, self.CP.alternativeExperience))
        #print('pafetyModel={}, pafetyParam={}, plternativeExperience={}'.format(pandaState.safetyModel, pandaState.safetyParam, pandaState.alternativeExperience))
        self.pandaState_safetyModel = str(pandaState.safetyModel)
        self.interface_safetyModel = str(self.CP.safetyConfigs[i].safetyModel)
      else:
        #print('safety_mismatch2')
        safety_mismatch = pandaState.safetyModel not in IGNORED_SAFETY_MODES

      self.rx_checks_ok = not pandaState.safetyRxChecksInvalid
      self.mismatch_counter_ok = self.mismatch_counter < 200
      if safety_mismatch or pandaState.safetyRxChecksInvalid or self.mismatch_counter >= 200:
        #print('safety_mismatch={} safetyRxChecksInvalid={} mismatch_counter={}'.format(safety_mismatch, pandaState.safetyRxChecksInvalid, self.mismatch_counter))
        self.events.add(EventName.controlsMismatch)

      if log.PandaState.FaultType.relayMalfunction in pandaState.faults:
        self.events.add(EventName.relayMalfunction)

    self.second += DT_CTRL
    if self.second > 1.0:
      self.live_sr = self.params.get_bool("KisaLiveSteerRatio")
      self.live_sr_percent = int(self.params.get("LiveSteerRatioPercent", encoding="utf8"))
      # E2ELongAlert
      if self.params.get_bool("E2ELong") and self.e2e_long_alert_prev:
        self.events.add(EventName.e2eLongAlert)
        self.e2e_long_alert_prev = not self.e2e_long_alert_prev
      elif not self.params.get_bool("E2ELong"):
        self.e2e_long_alert_prev = True
      # UnSleep Mode Alert
      if self.params.get_bool("KisaMonitoringMode") and self.unsleep_mode_alert_prev:
        self.events.add(EventName.unSleepMode)
        self.unsleep_mode_alert_prev = not self.unsleep_mode_alert_prev
      elif not self.params.get_bool("KisaMonitoringMode"):
        self.unsleep_mode_alert_prev = True
      # DoNotDisturb Mode Alert
      if self.params.get("CommaStockUI", encoding="utf8") == "2" and self.donotdisturb_mode_alert_prev:
        self.events.add(EventName.doNotDisturb)
        self.donotdisturb_mode_alert_prev = not self.donotdisturb_mode_alert_prev
      elif not self.params.get("CommaStockUI", encoding="utf8") == "2":
        self.donotdisturb_mode_alert_prev = True
      self.second = 0.0

    # Handle HW and system malfunctions
    # Order is very intentional here. Be careful when modifying this.
    # All events here should at least have NO_ENTRY and SOFT_DISABLE.
    num_events = len(self.events)

    not_running = {p.name for p in self.sm['managerState'].processes if not p.running and p.shouldBeRunning}
    if False: #self.sm.rcv_frame['managerState'] and (not_running - IGNORE_PROCESSES):
      self.events.add(EventName.processNotRunning)
      if not_running != self.not_running_prev:
        cloudlog.event("process_not_running", not_running=not_running, error=True)
      self.not_running_prev = not_running
    else:
      if not SIMULATION and not self.rk.lagging:
        if not self.sm.all_alive(self.camera_packets):
          self.events.add(EventName.cameraMalfunction)
        elif not self.sm.all_freq_ok(self.camera_packets):
          self.events.add(EventName.cameraFrameRate)
    if not REPLAY and self.rk.lagging:
      self.events.add(EventName.controlsdLagging)
    if len(self.sm['radarState'].radarErrors) or (not self.rk.lagging and not self.sm.all_checks(['radarState'])):
      self.events.add(EventName.radarFault)
    if not self.sm.valid['pandaStates']:
      self.events.add(EventName.usbError)
    if CS.canTimeout:
      self.events.add(EventName.canBusMissing)
    elif not CS.canValid:
      self.events.add(EventName.canError)

    # generic catch-all. ideally, a more specific event should be added above instead
    can_rcv_timeout = self.can_rcv_timeout_counter >= 5
    has_disable_events = self.events.contains(ET.NO_ENTRY) and (self.events.contains(ET.SOFT_DISABLE) or self.events.contains(ET.IMMEDIATE_DISABLE))
    no_system_errors = (not has_disable_events) or (len(self.events) == num_events)
    if (not self.sm.all_checks() or can_rcv_timeout) and no_system_errors:
      if not self.sm.all_alive():
        self.events.add(EventName.commIssue)
      elif not self.sm.all_freq_ok():
        self.events.add(EventName.commIssueAvgFreq)
      else:  # invalid or can_rcv_timeout.
        self.events.add(EventName.commIssue)

      logs = {
        'invalid': [s for s, valid in self.sm.valid.items() if not valid],
        'not_alive': [s for s, alive in self.sm.alive.items() if not alive],
        'not_freq_ok': [s for s, freq_ok in self.sm.freq_ok.items() if not freq_ok],
        'can_rcv_timeout': can_rcv_timeout,
      }
      if logs != self.logged_comm_issue:
        cloudlog.event("commIssue", error=True, **logs)
        self.logged_comm_issue = logs
    else:
      self.logged_comm_issue = None

    if not self.sm['lateralPlan'].mpcSolutionValid:
      self.events.add(EventName.plannerError)
    if not self.sm['liveLocationKalman'].posenetOK:
      self.events.add(EventName.posenetInvalid)
    if not self.sm['liveLocationKalman'].deviceStable:
      self.events.add(EventName.deviceFalling)
    if not self.sm['liveLocationKalman'].inputsOK:
      self.events.add(EventName.locationdTemporaryError)
    if not self.sm['liveParameters'].valid and not TESTING_CLOSET and (not SIMULATION or REPLAY):
      self.events.add(EventName.paramsdTemporaryError)

    # conservative HW alert. if the data or frequency are off, locationd will throw an error
    if any((self.sm.frame - self.sm.rcv_frame[s])*DT_CTRL > 10. for s in self.sensor_packets):
      self.events.add(EventName.sensorDataInvalid)

    if not REPLAY:
      # Check for mismatch between openpilot and car's PCM
      cruise_mismatch = CS.cruiseState.enabled and (not self.enabled or not self.CP.pcmCruise)
      self.cruise_mismatch_counter = self.cruise_mismatch_counter + 1 if cruise_mismatch else 0
      if self.cruise_mismatch_counter > int(6. / DT_CTRL):
        self.events.add(EventName.cruiseMismatch)

    # Check for FCW
    stock_long_is_braking = self.enabled and not self.CP.openpilotLongitudinalControl and CS.aEgo < -1.25
    model_fcw = self.sm['modelV2'].meta.hardBrakePredicted and not CS.brakePressed and not stock_long_is_braking
    planner_fcw = self.sm['longitudinalPlan'].fcw and self.enabled
    #if planner_fcw or model_fcw:
    #  self.events.add(EventName.fcw)

    for m in messaging.drain_sock(self.log_sock, wait_for_one=False):
      try:
        msg = m.androidLog.message
        if any(err in msg for err in ("ERROR_CRC", "ERROR_ECC", "ERROR_STREAM_UNDERFLOW", "APPLY FAILED")):
          csid = msg.split("CSID:")[-1].split(" ")[0]
          evt = CSID_MAP.get(csid, None)
          if evt is not None:
            self.events.add(evt)
      except UnicodeDecodeError:
        pass

    # TODO: fix simulator
    if not SIMULATION or REPLAY:
      if not self.sm['liveLocationKalman'].gpsOK and self.sm['liveLocationKalman'].inputsOK and (self.distance_traveled > 100000000000000000):
        # Not show in first 1 km to allow for driving out of garage. This event shows after 5 minutes
        self.events.add(EventName.noGps)

      if self.sm['modelV2'].frameDropPerc > 20:
        self.events.add(EventName.modeldLagging)

    # atom
    if self.auto_enabled and not self.no_mdps_mods and self.ufc_mode:
      self.ready_timer += 1 if self.ready_timer < 350 else 350
      self.auto_enable( CS )

  def data_sample(self):
    """Receive data from sockets and update carState"""

    # Update carState from CAN
    can_strs = messaging.drain_sock_raw(self.can_sock, wait_for_one=True)
    CS = self.CI.update(self.CC, can_strs)
    if len(can_strs) and REPLAY:
      self.can_log_mono_time = messaging.log_from_bytes(can_strs[0]).logMonoTime

    self.sm.update(0)

    if not self.initialized:
      all_valid = CS.canValid and self.sm.all_checks()
      timed_out = self.sm.frame * DT_CTRL > (6. if REPLAY else 3.5)
      if all_valid or timed_out or (SIMULATION and not REPLAY):
        available_streams = VisionIpcClient.available_streams("camerad", block=False)
        if VisionStreamType.VISION_STREAM_ROAD not in available_streams:
          self.sm.ignore_alive.append('roadCameraState')
        if VisionStreamType.VISION_STREAM_WIDE_ROAD not in available_streams:
          self.sm.ignore_alive.append('wideRoadCameraState')

        if not self.read_only:
          self.CI.init(self.CP, self.can_sock, self.pm.sock['sendcan'])

        self.initialized = True
        self.set_initial_state()
        put_bool_nonblocking("ControlsReady", True)

    # Check for CAN timeout
    if not can_strs:
      self.can_rcv_timeout_counter += 1
      self.can_rcv_cum_timeout_counter += 1
    else:
      self.can_rcv_timeout_counter = 0

    # When the panda and controlsd do not agree on controls_allowed
    # we want to disengage openpilot. However the status from the panda goes through
    # another socket other than the CAN messages and one can arrive earlier than the other.
    # Therefore we allow a mismatch for two samples, then we trigger the disengagement.
    if not self.enabled:
      self.mismatch_counter = 0

    # All pandas not in silent mode must have controlsAllowed when openpilot is enabled
    if self.enabled and any(not ps.controlsAllowed for ps in self.sm['pandaStates']
           if ps.safetyModel not in IGNORED_SAFETY_MODES):
      self.mismatch_counter += 1

    self.distance_traveled += CS.vEgo * DT_CTRL

    return CS

  def state_transition(self, CS):
    """Compute conditional state transitions and execute actions on state transitions"""

    self.v_cruise_helper.update_v_cruise(CS, self.enabled, self.is_metric)

    # decrement the soft disable timer at every step, as it's reset on
    # entrance in SOFT_DISABLING state
    self.soft_disable_timer = max(0, self.soft_disable_timer - 1)

    self.current_alert_types = [ET.PERMANENT]

    # ENABLED, SOFT DISABLING, PRE ENABLING, OVERRIDING
    if self.state != State.disabled:
      # user and immediate disable always have priority in a non-disabled state
      if self.events.contains(ET.USER_DISABLE):
        self.state = State.disabled
        self.current_alert_types.append(ET.USER_DISABLE)

      elif self.events.contains(ET.IMMEDIATE_DISABLE):
        self.state = State.disabled
        self.current_alert_types.append(ET.IMMEDIATE_DISABLE)

      else:
        # ENABLED
        if self.state == State.enabled:
          if self.events.contains(ET.SOFT_DISABLE):
            self.state = State.softDisabling
            self.soft_disable_timer = int(SOFT_DISABLE_TIME / DT_CTRL)
            self.current_alert_types.append(ET.SOFT_DISABLE)

          elif self.events.contains(ET.OVERRIDE_LATERAL) or self.events.contains(ET.OVERRIDE_LONGITUDINAL):
            self.state = State.overriding
            self.current_alert_types += [ET.OVERRIDE_LATERAL, ET.OVERRIDE_LONGITUDINAL]

        # SOFT DISABLING
        elif self.state == State.softDisabling:
          if not self.events.contains(ET.SOFT_DISABLE):
            # no more soft disabling condition, so go back to ENABLED
            self.state = State.enabled

          elif self.soft_disable_timer > 0:
            self.current_alert_types.append(ET.SOFT_DISABLE)

          elif self.soft_disable_timer <= 0:
            self.state = State.disabled

        # PRE ENABLING
        elif self.state == State.preEnabled:
          if not self.events.contains(ET.PRE_ENABLE):
            self.state = State.enabled
          else:
            self.current_alert_types.append(ET.PRE_ENABLE)

        # OVERRIDING
        elif self.state == State.overriding:
          if self.events.contains(ET.SOFT_DISABLE):
            self.state = State.softDisabling
            self.soft_disable_timer = int(SOFT_DISABLE_TIME / DT_CTRL)
            self.current_alert_types.append(ET.SOFT_DISABLE)
          elif not (self.events.contains(ET.OVERRIDE_LATERAL) or self.events.contains(ET.OVERRIDE_LONGITUDINAL)):
            self.state = State.enabled
          else:
            self.current_alert_types += [ET.OVERRIDE_LATERAL, ET.OVERRIDE_LONGITUDINAL]

    # DISABLED
    elif self.state == State.disabled:
      if self.events.contains(ET.ENABLE):
        if self.events.contains(ET.NO_ENTRY):
          self.current_alert_types.append(ET.NO_ENTRY)

        else:
          if self.events.contains(ET.PRE_ENABLE):
            self.state = State.preEnabled
          elif self.events.contains(ET.OVERRIDE_LATERAL) or self.events.contains(ET.OVERRIDE_LONGITUDINAL):
            self.state = State.overriding
          else:
            self.state = State.enabled
          self.current_alert_types.append(ET.ENABLE)
          self.v_cruise_helper.initialize_v_cruise(CS, self.experimental_mode)

    # Check if openpilot is engaged and actuators are enabled
    self.enabled = self.state in ENABLED_STATES
    self.active = self.state in ACTIVE_STATES
    if self.active:
      self.current_alert_types.append(ET.WARNING)

  def state_control(self, CS):
    """Given the state, this function returns a CarControl packet"""

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

    lat_plan = self.sm['lateralPlan']
    long_plan = self.sm['longitudinalPlan']

    CC = car.CarControl.new_message()
    CC.enabled = self.enabled

    # Check which actuators can be enabled
    standstill = (CS.vEgo <= max(self.CP.minSteerSpeed, MIN_LATERAL_CONTROL_SPEED) and self.no_mdps_mods) or CS.standstill
    CC.latActive = self.active and not CS.steerFaultTemporary and not CS.steerFaultPermanent and \
                   (not standstill or self.joystick_mode) and not self.lkas_temporary_off
    CC.longActive = self.enabled and not self.events.contains(ET.OVERRIDE_LONGITUDINAL) and self.CP.openpilotLongitudinalControl
    # print('active={}  steerFaultTemporary={}  steerFaultPermanent={}  standstill={}  joystick_mode={}  lkas_temporary_off={}'.format(
    # self.active, CS.steerFaultTemporary, CS.steerFaultPermanent, standstill, self.joystick_mode, self.lkas_temporary_off))
    # print('CON_latActive={}'.format(CC.latActive))

    actuators = CC.actuators
    actuators.longControlState = self.LoC.long_control_state

    # Enable blinkers while lane changing
    if self.sm['lateralPlan'].laneChangeState != LaneChangeState.off:
      CC.leftBlinker = self.sm['lateralPlan'].laneChangeDirection == LaneChangeDirection.left
      CC.rightBlinker = self.sm['lateralPlan'].laneChangeDirection == LaneChangeDirection.right

    if CS.leftBlinker or CS.rightBlinker:
      self.last_blinker_frame = self.sm.frame

    # State specific actions

    if not CC.latActive:
      self.LaC.reset()
    if not CC.longActive:
      self.LoC.reset(v_pid=CS.vEgo)

    if not self.joystick_mode:
      # accel PID loop
      pid_accel_limits = self.CI.get_pid_accel_limits(self.CP, CS.vEgo, self.v_cruise_helper.v_cruise_kph * CV.KPH_TO_MS)
      t_since_plan = (self.sm.frame - self.sm.rcv_frame['longitudinalPlan']) * DT_CTRL
      actuators.accel, actuators.oaccel = self.LoC.update(CC.longActive, CS, long_plan, pid_accel_limits, t_since_plan, self.CP, self.sm['radarState'])

      # Steering PID loop and lateral MPC
      self.desired_curvature, self.desired_curvature_rate = get_lag_adjusted_curvature(self.CP, CS.vEgo,
                                                                                       lat_plan.psis,
                                                                                       lat_plan.curvatures,
                                                                                       lat_plan.curvatureRates)
      actuators.steer, actuators.steeringAngleDeg, lac_log = self.LaC.update(CC.latActive, CS, self.VM, lp,
                                                                             self.last_actuators, self.steer_limited, self.desired_curvature,
                                                                             self.desired_curvature_rate, self.sm['liveLocationKalman'])
      actuators.curvature = self.desired_curvature
      self.desired_angle_deg = actuators.steeringAngleDeg
    else:
      lac_log = log.ControlsState.LateralDebugState.new_message()
      if self.sm.rcv_frame['testJoystick'] > 0:
        # reset joystick if it hasn't been received in a while
        should_reset_joystick = (self.sm.frame - self.sm.rcv_frame['testJoystick'])*DT_CTRL > 0.2
        if not should_reset_joystick:
          joystick_axes = self.sm['testJoystick'].axes
        else:
          joystick_axes = [0.0, 0.0]

        if CC.longActive:
          actuators.accel = 4.0*clip(joystick_axes[0], -1, 1)

        if CC.latActive:
          steer = clip(joystick_axes[1], -1, 1)
          # max angle is 45 for angle-based cars, max curvature is 0.02
          actuators.steer, actuators.steeringAngleDeg, actuators.curvature = steer, steer * 45., steer * -0.02
          self.desired_angle_deg = actuators.steeringAngleDeg

        lac_log.active = self.active
        lac_log.steeringAngleDeg = CS.steeringAngleDeg
        lac_log.output = actuators.steer
        lac_log.saturated = abs(actuators.steer) >= 0.9

    if CS.steeringPressed:
      self.last_steering_pressed_frame = self.sm.frame
    recent_steer_pressed = (self.sm.frame - self.last_steering_pressed_frame)*DT_CTRL < 2.0

    # Send a "steering required alert" if saturation count has reached the limit
    if lac_log.active and not recent_steer_pressed and not self.CP.notCar:
      if self.CP.lateralTuning.which() == 'torque' and not self.joystick_mode:
        undershooting = abs(lac_log.desiredLateralAccel) / abs(1e-3 + lac_log.actualLateralAccel) > 1.2
        turning = abs(lac_log.desiredLateralAccel) > 1.0
        good_speed = CS.vEgo > 5
        max_torque = abs(self.last_actuators.steer) > 0.99
        if undershooting and turning and good_speed and max_torque:
          lac_log.active and self.events.add(EventName.steerSaturated)
      elif lac_log.saturated:
        dpath_points = lat_plan.dPathPoints
        if len(dpath_points):
          # Check if we deviated from the path
          # TODO use desired vs actual curvature
          if self.CP.steerControlType == car.CarParams.SteerControlType.angle:
            steering_value = actuators.steeringAngleDeg
          else:
            steering_value = actuators.steer

          left_deviation = steering_value > 0 and dpath_points[0] < -0.20
          right_deviation = steering_value < 0 and dpath_points[0] > 0.20

          if left_deviation or right_deviation:
            self.events.add(EventName.steerSaturated)

    # Ensure no NaNs/Infs
    for p in ACTUATOR_FIELDS:
      attr = getattr(actuators, p)
      if not isinstance(attr, SupportsFloat):
        continue

      if not math.isfinite(attr):
        cloudlog.error(f"actuators.{p} not finite {actuators.to_dict()}")
        setattr(actuators, p, 0.0)

    return CC, lac_log

  def publish_logs(self, CS, start_time, CC, lac_log):
    """Send actuators and hud commands to the car, send controlsstate and MPC logging"""

    self.log_alertTextMsg1 = trace1.global_alertTextMsg1
    self.log_alertTextMsg2 = trace1.global_alertTextMsg2
    self.log_alertTextMsg3 = trace1.global_alertTextMsg3

    # Orientation and angle rates can be useful for carcontroller
    # Only calibrated (car) frame is relevant for the carcontroller
    orientation_value = list(self.sm['liveLocationKalman'].calibratedOrientationNED.value)
    if len(orientation_value) > 2:
      CC.orientationNED = orientation_value
    angular_rate_value = list(self.sm['liveLocationKalman'].angularVelocityCalibrated.value)
    if len(angular_rate_value) > 2:
      CC.angularVelocity = angular_rate_value

    CC.cruiseControl.override = self.enabled and not CC.longActive and self.CP.openpilotLongitudinalControl
    CC.cruiseControl.cancel = CS.cruiseState.enabled and (not self.enabled or not self.CP.pcmCruise)
    if self.joystick_mode and self.sm.rcv_frame['testJoystick'] > 0 and self.sm['testJoystick'].buttons[0]:
      CC.cruiseControl.cancel = True

    speeds = self.sm['longitudinalPlan'].speeds
    if len(speeds):
      CC.cruiseControl.resume = self.enabled and CS.cruiseState.standstill and speeds[-1] > 0.1

    hudControl = CC.hudControl
    hudControl.setSpeed = float(self.v_cruise_helper.v_cruise_kph * (CV.KPH_TO_MS if self.is_metric else CV.MPH_TO_MS))
    hudControl.speedVisible = self.enabled
    hudControl.lanesVisible = self.enabled
    hudControl.leadVisible = self.sm['longitudinalPlan'].hasLead

    hudControl.rightLaneVisible = True
    hudControl.leftLaneVisible = True

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
    recent_blinker = (self.sm.frame - self.last_blinker_frame) * DT_CTRL < 5.0  # 5s blinker cooldown
    ldw_allowed = self.is_ldw_enabled and CS.vEgo > LDW_MIN_SPEED and not recent_blinker \
                  and not CC.latActive and self.sm['liveCalibration'].calStatus == log.LiveCalibrationData.Status.calibrated

    model_v2 = self.sm['modelV2']
    desire_prediction = model_v2.meta.desirePrediction
    if len(desire_prediction) and ldw_allowed:
      right_lane_visible = model_v2.laneLineProbs[2] > 0.5
      left_lane_visible = model_v2.laneLineProbs[1] > 0.5
      l_lane_change_prob = desire_prediction[Desire.laneChangeLeft]
      r_lane_change_prob = desire_prediction[Desire.laneChangeRight]

      lane_lines = model_v2.laneLines
      if CS.cruiseState.modeSel == 4:
        l_lane_close = left_lane_visible and (lane_lines[1].y[0] > -(1.08 + CAMERA_OFFSET_A))
        r_lane_close = right_lane_visible and (lane_lines[2].y[0] < (1.08 - CAMERA_OFFSET_A))
      else:
        l_lane_close = left_lane_visible and (lane_lines[1].y[0] > -(1.08 + CAMERA_OFFSET))
        r_lane_close = right_lane_visible and (lane_lines[2].y[0] < (1.08 - CAMERA_OFFSET))

      hudControl.leftLaneDepart = bool(l_lane_change_prob > LANE_DEPARTURE_THRESHOLD and l_lane_close)
      hudControl.rightLaneDepart = bool(r_lane_change_prob > LANE_DEPARTURE_THRESHOLD and r_lane_close)

    if hudControl.rightLaneDepart or hudControl.leftLaneDepart:
      self.events.add(EventName.ldw)

    clear_event_types = set()
    if ET.WARNING not in self.current_alert_types:
      clear_event_types.add(ET.WARNING)
    if self.enabled:
      clear_event_types.add(ET.NO_ENTRY)

    alerts = self.events.create_alerts(self.current_alert_types, [self.CP, CS, self.sm, self.is_metric, self.soft_disable_timer])
    self.AM.add_many(self.sm.frame, alerts)
    current_alert = self.AM.process_alerts(self.sm.frame, clear_event_types)
    if current_alert:
      hudControl.visualAlert = current_alert.visual_alert

    if self.stock_lkas_on_disengaged_status and self.CP.carName == "hyundai" and not self.exp_long_enabled:
      if self.enabled:
        self.hkg_stock_lkas = False
        self.hkg_stock_lkas_timer = 0
      elif not self.enabled and not self.hkg_stock_lkas:
        self.hkg_stock_lkas_timer += 1
        if self.hkg_stock_lkas_timer > 300:
          self.hkg_stock_lkas = True
          self.hkg_stock_lkas_timer = 0
        elif CS.gearShifter != GearShifter.drive and self.hkg_stock_lkas_timer > 150:
          self.hkg_stock_lkas = True
          self.hkg_stock_lkas_timer = 0
      if not self.hkg_stock_lkas:
        # send car controls over can
        now_nanos = self.can_log_mono_time if REPLAY else int(time.monotonic() * 1e9)
        self.last_actuators, can_sends, self.safety_speed, self.lkas_temporary_off, self.gap_by_spd_on_temp, self.exp_mode_temp, self.btn_pressing = self.CI.apply(CC, now_nanos)
        self.pm.send('sendcan', can_list_to_can_capnp(can_sends, msgtype='sendcan', valid=CS.canValid))
        CC.actuatorsOutput = self.last_actuators
        if self.CP.steerControlType == car.CarParams.SteerControlType.angle:
          self.steer_limited = abs(CC.actuators.steeringAngleDeg - CC.actuatorsOutput.steeringAngleDeg) > \
                              STEER_ANGLE_SATURATION_THRESHOLD
        else:
          self.steer_limited = abs(CC.actuators.steer - CC.actuatorsOutput.steer) > 1e-2
    else:
      if not self.read_only and self.initialized:
        # send car controls over can
        now_nanos = self.can_log_mono_time if REPLAY else int(time.monotonic() * 1e9)
        if self.CP.carName == "hyundai":
          self.last_actuators, can_sends, self.safety_speed, self.lkas_temporary_off, self.gap_by_spd_on_temp, self.exp_mode_temp, self.btn_pressing = self.CI.apply(CC, now_nanos)
        else:
          self.last_actuators, can_sends = self.CI.apply(CC, now_nanos)
        self.pm.send('sendcan', can_list_to_can_capnp(can_sends, msgtype='sendcan', valid=CS.canValid))
        CC.actuatorsOutput = self.last_actuators
        if self.CP.steerControlType == car.CarParams.SteerControlType.angle:
          self.steer_limited = abs(CC.actuators.steeringAngleDeg - CC.actuatorsOutput.steeringAngleDeg) > \
                              STEER_ANGLE_SATURATION_THRESHOLD
        else:
          self.steer_limited = abs(CC.actuators.steer - CC.actuatorsOutput.steer) > 1e-2

    force_decel = (self.sm['driverMonitoringState'].awarenessStatus < 0.) or \
                  (self.state == State.softDisabling)

    # Curvature & Steering angle
    lp = self.sm['liveParameters']

    steer_angle_without_offset = math.radians(CS.steeringAngleDeg - lp.angleOffsetDeg)
    curvature = -self.VM.calc_curvature(steer_angle_without_offset, CS.vEgo, lp.roll)

    # controlsState
    dat = messaging.new_message('controlsState')
    dat.valid = CS.canValid
    controlsState = dat.controlsState
    if current_alert:
      controlsState.alertText1 = current_alert.alert_text_1
      controlsState.alertText2 = current_alert.alert_text_2
      controlsState.alertSize = current_alert.alert_size
      controlsState.alertStatus = current_alert.alert_status
      controlsState.alertBlinkingRate = current_alert.alert_rate
      controlsState.alertType = current_alert.alert_type
      controlsState.alertSound = current_alert.audible_alert

    controlsState.longitudinalPlanMonoTime = self.sm.logMonoTime['longitudinalPlan']
    controlsState.lateralPlanMonoTime = self.sm.logMonoTime['lateralPlan']
    controlsState.enabled = self.enabled
    controlsState.active = self.active
    controlsState.curvature = curvature
    controlsState.desiredCurvature = self.desired_curvature
    controlsState.desiredCurvatureRate = self.desired_curvature_rate
    controlsState.state = self.state
    controlsState.engageable = not self.events.contains(ET.NO_ENTRY)
    controlsState.longControlState = self.LoC.long_control_state
    controlsState.vPid = float(self.LoC.v_pid)
    controlsState.vCruise = float(self.v_cruise_helper.v_cruise_kph) # SCC setspeed number of cluster, kph or mph not m/s
    controlsState.vCruiseCluster = float(self.v_cruise_helper.v_cruise_cluster_kph) # same as vCruise
    controlsState.upAccelCmd = float(self.LoC.pid.p)
    controlsState.uiAccelCmd = float(self.LoC.pid.i)
    controlsState.ufAccelCmd = float(self.LoC.pid.f)
    controlsState.cumLagMs = -self.rk.remaining * 1000.
    controlsState.startMonoTime = int(start_time * 1e9)
    controlsState.forceDecel = bool(force_decel)
    controlsState.canErrorCounter = self.can_rcv_cum_timeout_counter
    controlsState.experimentalMode = self.experimental_mode
    controlsState.alertTextMsg1 = self.log_alertTextMsg1
    controlsState.alertTextMsg2 = self.log_alertTextMsg2
    controlsState.alertTextMsg3 = self.log_alertTextMsg3
    controlsState.pauseSpdLimit = self.v_cruise_helper.pause_spdlimit
    if self.osm_speedlimit_enabled or self.navi_selection == 2:
      if self.navi_selection == 2:
        controlsState.limitSpeedCamera = int(round(self.sm['liveENaviData'].wazeRoadSpeedLimit))
        controlsState.limitSpeedCameraDist = float(self.sm['liveENaviData'].wazeAlertDistance)
      elif self.osm_speedlimit_enabled:
        controlsState.limitSpeedCamera = int(round(self.sm['liveMapData'].speedLimit))
        controlsState.limitSpeedCameraDist = float(self.sm['liveMapData'].speedLimitAheadDistance)
      if self.sm['liveMapData'].currentRoadName in self.roadname_and_slc:
        try:
          r_index = self.roadname_and_slc.index(self.sm['liveMapData'].currentRoadName)
          controlsState.limitSpeedCamera = float(self.roadname_and_slc[r_index+1])
        except:
          pass
    elif self.navi_selection == 1 and int(self.sm['liveENaviData'].safetySign) not in (20, 21):
      controlsState.limitSpeedCamera = int(round(self.sm['liveENaviData'].speedLimit))
      controlsState.limitSpeedCameraDist = float(self.sm['liveENaviData'].safetyDistance)
      controlsState.mapSign = int(self.sm['liveENaviData'].safetySign)
    else:
      controlsState.limitSpeedCamera = 0
      controlsState.limitSpeedCameraDist = 0
    controlsState.lateralControlMethod = int(self.lateral_control_method)
    controlsState.steerRatio = float(self.steerRatio_to_send)
    controlsState.dynamicTRMode = int(self.sm['longitudinalPlan'].dynamicTRMode)
    controlsState.dynamicTRValue = float(self.sm['longitudinalPlan'].dynamicTRValue)
    controlsState.accel = float(self.last_actuators.accel)
    controlsState.safetySpeed = float(self.safety_speed)
    controlsState.gapBySpeedOn = bool(self.gap_by_spd_on_temp)
    controlsState.expModeTemp = bool(self.exp_mode_temp)
    controlsState.btnPressing = self.btn_pressing

    lat_tuning = self.CP.lateralTuning.which()
    if self.joystick_mode:
      controlsState.lateralControlState.debugState = lac_log
    elif self.CP.steerControlType == car.CarParams.SteerControlType.angle:
      controlsState.lateralControlState.angleState = lac_log
    elif lat_tuning == 'pid':
      controlsState.lateralControlState.pidState = lac_log
    elif lat_tuning == 'lqr':
      controlsState.lateralControlState.lqrState = lac_log
    elif lat_tuning == 'indi':
      controlsState.lateralControlState.indiState = lac_log
    elif lat_tuning == 'torque':
      controlsState.lateralControlState.torqueState = lac_log
    elif lat_tuning == 'atom':
      controlsState.lateralControlState.atomState = lac_log

    if lat_tuning == 'torque':
      controlsState.steeringAngleDesiredDeg = lac_log.desiredLateralAccel
    else:
      controlsState.steeringAngleDesiredDeg = self.desired_angle_deg

    controlsState.pandaSafetyModel = self.pandaState_safetyModel
    controlsState.interfaceSafetyModel = self.interface_safetyModel
    controlsState.rxChecks = self.rx_checks_ok
    controlsState.mismatchCounter = self.mismatch_counter_ok

    self.pm.send('controlsState', dat)

    # carState
    car_events = self.events.to_msg()
    cs_send = messaging.new_message('carState')
    cs_send.valid = CS.canValid
    cs_send.carState = CS
    cs_send.carState.events = car_events
    self.pm.send('carState', cs_send)

    # carEvents - logged every second or on change
    if (self.sm.frame % int(1. / DT_CTRL) == 0) or (self.events.names != self.events_prev):
      ce_send = messaging.new_message('carEvents', len(self.events))
      ce_send.carEvents = car_events
      self.pm.send('carEvents', ce_send)
    self.events_prev = self.events.names.copy()

    # carParams - logged every 50 seconds (> 1 per segment)
    if (self.sm.frame % int(50. / DT_CTRL) == 0):
      cp_send = messaging.new_message('carParams')
      cp_send.carParams = self.CP
      self.pm.send('carParams', cp_send)

    # carControl
    cc_send = messaging.new_message('carControl')
    cc_send.valid = CS.canValid
    cc_send.carControl = CC
    self.pm.send('carControl', cc_send)

    # copy CarControl to pass to CarInterface on the next iteration
    self.CC = CC

  def step(self):
    start_time = time.monotonic()
    self.prof.checkpoint("Ratekeeper", ignore=True)

    self.is_metric = self.params.get_bool("IsMetric")
    self.experimental_mode = self.params.get_bool("ExperimentalMode") and self.CP.openpilotLongitudinalControl

    # Sample data from sockets and get a carState
    CS = self.data_sample()
    cloudlog.timestamp("Data sampled")
    self.prof.checkpoint("Sample")

    self.update_events(CS)
    cloudlog.timestamp("Events updated")

    if not self.read_only and self.initialized:
      # Update control state
      self.state_transition(CS)
      self.prof.checkpoint("State transition")

    # Compute actuators (runs PID loops and lateral MPC)
    CC, lac_log = self.state_control(CS)

    self.prof.checkpoint("State Control")

    # Publish data
    self.publish_logs(CS, start_time, CC, lac_log)
    self.prof.checkpoint("Sent")

    self.CS_prev = CS

  def controlsd_thread(self):
    while True:
      self.step()
      self.rk.monitor_time()
      self.prof.display()


def main():
  controls = Controls()
  controls.controlsd_thread()


if __name__ == "__main__":
  main()
