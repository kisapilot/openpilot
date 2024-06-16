from cereal import car
from panda import Panda
from openpilot.common.conversions import Conversions as CV
from openpilot.selfdrive.car.hyundai.tunes import LatTunes, set_lat_tune
from openpilot.selfdrive.car.hyundai.hyundaicanfd import CanBus
from openpilot.selfdrive.car.hyundai.values import HyundaiFlags, CAR, DBC, CANFD_CAR, CAMERA_SCC_CAR, CANFD_RADAR_SCC_CAR, \
                                         CANFD_UNSUPPORTED_LONGITUDINAL_CAR, EV_CAR, HYBRID_CAR, LEGACY_SAFETY_MODE_CAR, \
                                         UNSUPPORTED_LONGITUDINAL_CAR, Buttons, LEGACY_SAFETY_MODE_CAR_ALT, ANGLE_CONTROL_CAR
from openpilot.selfdrive.car.hyundai.radar_interface import RADAR_START_ADDR
from openpilot.selfdrive.car import create_button_events, get_safety_config
from openpilot.selfdrive.car.interfaces import CarInterfaceBase
from openpilot.selfdrive.car.disable_ecu import disable_ecu
from openpilot.selfdrive.car.hyundai.cruise_helper import enable_radar_tracks #ajouatom

from openpilot.common.params import Params
from decimal import Decimal

Ecu = car.CarParams.Ecu
ButtonType = car.CarState.ButtonEvent.Type
EventName = car.CarEvent.EventName
SteerControlType = car.CarParams.SteerControlType
ENABLE_BUTTONS = (Buttons.RES_ACCEL, Buttons.SET_DECEL, Buttons.CANCEL)
BUTTONS_DICT = {Buttons.RES_ACCEL: ButtonType.accelCruise, Buttons.SET_DECEL: ButtonType.decelCruise,
                Buttons.GAP_DIST: ButtonType.gapAdjustCruise, Buttons.CANCEL: ButtonType.cancel}


class CarInterface(CarInterfaceBase):
  @staticmethod
  def _get_params(ret, candidate, fingerprint, car_fw, experimental_long, docs):
    ret.carName = "hyundai"
    ret.radarUnavailable = RADAR_START_ADDR not in fingerprint[1] or DBC[ret.carFingerprint]["radar"] is None

    # These cars have been put into dashcam only due to both a lack of users and test coverage.
    # These cars likely still work fine. Once a user confirms each car works and a test route is
    # added to selfdrive/car/tests/routes.py, we can remove it from this list.
    ret.dashcamOnly = False

    hda2 = Ecu.adas in [fw.ecu for fw in car_fw]
    CAN = CanBus(None, hda2, fingerprint)

    if candidate in CANFD_CAR:
      ret.isCanFD = True
      # detect if car is hybrid
      if 0x105 in fingerprint[CAN.ECAN]:
        ret.flags |= HyundaiFlags.HYBRID.value
      elif candidate in EV_CAR:
        ret.flags |= HyundaiFlags.EV.value

      # detect HDA2 with ADAS Driving ECU
      if hda2:
        ret.flags |= HyundaiFlags.CANFD_HDA2.value
        if 0x110 in fingerprint[CAN.CAM]:
          ret.flags |= HyundaiFlags.CANFD_HDA2_ALT_STEERING.value
      else:
        # non-HDA2
        if 0x1cf not in fingerprint[CAN.ECAN]:
          ret.flags |= HyundaiFlags.CANFD_ALT_BUTTONS.value
        # ICE cars do not have 0x130; GEARS message on 0x40 or 0x70 instead
        if 0x130 not in fingerprint[CAN.ECAN]:
          if 0x40 not in fingerprint[CAN.ECAN]:
            ret.flags |= HyundaiFlags.CANFD_ALT_GEARS_2.value
          else:
            ret.flags |= HyundaiFlags.CANFD_ALT_GEARS.value
        if candidate not in CANFD_RADAR_SCC_CAR:
          ret.flags |= HyundaiFlags.CANFD_CAMERA_SCC.value
    else:
      ret.isCanFD = False
      # TODO: detect EV and hybrid
      if candidate in HYBRID_CAR:
        ret.flags |= HyundaiFlags.HYBRID.value
      elif candidate in EV_CAR:
        ret.flags |= HyundaiFlags.EV.value

      # Send LFA message on cars with HDA
      if 0x485 in fingerprint[2]:
        ret.flags |= HyundaiFlags.SEND_LFA.value

      # These cars use the FCA11 message for the AEB and FCW signals, all others use SCC12
      if 0x38d in fingerprint[0] or 0x38d in fingerprint[2]:
        ret.flags |= HyundaiFlags.USE_FCA.value

    params = Params()

    ret.steerActuatorDelay = 0.1  # Default delay
    ret.steerLimitTimer = 0.4

    #ret.radarTimeStep = 0.02  # 50Hz instead of standard 20Hz

    ret.smoothSteer.method = int( Params().get("KisaSteerMethod", encoding="utf8") )   # 1
    ret.smoothSteer.maxSteeringAngle = float( Params().get("KisaMaxSteeringAngle", encoding="utf8") )   # 90
    ret.smoothSteer.maxDriverAngleWait = float( Params().get("KisaMaxDriverAngleWait", encoding="utf8") )  # 0.002
    ret.smoothSteer.maxSteerAngleWait = float( Params().get("KisaMaxSteerAngleWait", encoding="utf8") )   # 0.001  # 10 sec
    ret.smoothSteer.driverAngleWait = float( Params().get("KisaDriverAngleWait", encoding="utf8") )  #0.001

    ret.steerActuatorDelay = float(Decimal(params.get("SteerActuatorDelayAdj", encoding="utf8")) * Decimal('0.01'))
    ret.steerLimitTimer = float(Decimal(params.get("SteerLimitTimerAdj", encoding="utf8")) * Decimal('0.01'))

    ret.experimentalLong = Params().get_bool("ExperimentalLongitudinalEnabled")
    ret.experimentalLongAlt = candidate in LEGACY_SAFETY_MODE_CAR_ALT
    
    if candidate in ANGLE_CONTROL_CAR:
      ret.steerControlType = SteerControlType.angle
    else:
      lat_control_method = int(params.get("LateralControlMethod", encoding="utf8"))
      if lat_control_method == 0:
        set_lat_tune(ret.lateralTuning, LatTunes.PID)
      elif lat_control_method == 1:
        set_lat_tune(ret.lateralTuning, LatTunes.INDI)
      elif lat_control_method == 2:
        set_lat_tune(ret.lateralTuning, LatTunes.LQR)
      elif lat_control_method == 3:
        #set_lat_tune(ret.lateralTuning, LatTunes.TORQUE)
        CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)
      elif lat_control_method == 4:
        set_lat_tune(ret.lateralTuning, LatTunes.ATOM)    # Hybrid tune

    # *** longitudinal control ***
    if candidate in CANFD_CAR:
      ret.experimentalLongitudinalAvailable = candidate not in (CANFD_UNSUPPORTED_LONGITUDINAL_CAR | CANFD_RADAR_SCC_CAR)
    else:
      ret.experimentalLongitudinalAvailable = True #candidate not in (UNSUPPORTED_LONGITUDINAL_CAR | CAMERA_SCC_CAR)
    ret.openpilotLongitudinalControl = experimental_long and ret.experimentalLongitudinalAvailable #experimental_long is LongControl toggle, not Experiment Mode
    ret.pcmCruise = not ret.openpilotLongitudinalControl

    ret.stoppingControl = True
    ret.startingState = True
    ret.vEgoStarting = 0.1
    ret.startAccel = 1.0
    ret.longitudinalActuatorDelay = 0.5

    # *** feature detection ***
    if candidate in CANFD_CAR:
      ret.enableBsm = 0x1e5 in fingerprint[CAN.ECAN]
      ret.sccBus = 0
      ret.bsmAvailable = False
      ret.lfaAvailable = False
      ret.lvrAvailable = False
      ret.evgearAvailable = False
      ret.emsAvailable = False
      ret.autoHoldAvailable = False
      ret.lfaHdaAvailable = False
      ret.navAvailable = False
      ret.adrvAvailable = 0x200 in fingerprint[CAN.ECAN]
    else:
      ret.enableBsm = 0x58b in fingerprint[0]
      ret.sccBus = 2 if int(Params().get("KISALongAlt", encoding="utf8")) in (1, 2) and not Params().get_bool("ExperimentalLongitudinalEnabled") else 0
      ret.bsmAvailable = True if 1419 in fingerprint[0] else False
      ret.lfaAvailable = True if 1157 in fingerprint[2] else False
      ret.lvrAvailable = True if 871 in fingerprint[0] else False
      ret.evgearAvailable = True if 882 in fingerprint[0] else False
      ret.emsAvailable = True if 870 in fingerprint[0] else False
      ret.autoHoldAvailable = 1151 in fingerprint[0]
      ret.lfaHdaAvailable = 1157 in fingerprint[0]
      ret.navAvailable = 1348 in fingerprint[0]
      ret.adrvAvailable = False

    # *** panda safety config ***
    if candidate in CANFD_CAR:
      cfgs = [get_safety_config(car.CarParams.SafetyModel.hyundaiCanfd), ]
      if CAN.ECAN >= 4:
        cfgs.insert(0, get_safety_config(car.CarParams.SafetyModel.noOutput))
      ret.safetyConfigs = cfgs

      if ret.flags & HyundaiFlags.CANFD_HDA2:
        ret.safetyConfigs[-1].safetyParam |= Panda.FLAG_HYUNDAI_CANFD_HDA2
        if ret.flags & HyundaiFlags.CANFD_HDA2_ALT_STEERING:
          ret.safetyConfigs[-1].safetyParam |= Panda.FLAG_HYUNDAI_CANFD_HDA2_ALT_STEERING
      if ret.flags & HyundaiFlags.CANFD_ALT_BUTTONS:
        ret.safetyConfigs[-1].safetyParam |= Panda.FLAG_HYUNDAI_CANFD_ALT_BUTTONS
      if ret.flags & HyundaiFlags.CANFD_CAMERA_SCC:
        ret.safetyConfigs[-1].safetyParam |= Panda.FLAG_HYUNDAI_CAMERA_SCC
    else:
      if Params().get_bool("ExperimentalLongitudinalEnabled"):
        if candidate in LEGACY_SAFETY_MODE_CAR:
          # these cars require a special panda safety mode due to missing counters and checksums in the messages
          ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.hyundaiLegacy)]
        else:
          ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.hyundai, 0)]
      elif candidate in LEGACY_SAFETY_MODE_CAR_ALT or (candidate in LEGACY_SAFETY_MODE_CAR and Params().get_bool("UFCModeEnabled")):
        ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.hyundaiCommunity1Legacy)]
      elif Params().get_bool("UFCModeEnabled"):
        ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.hyundaiCommunity1)]
      elif candidate in LEGACY_SAFETY_MODE_CAR:
        # these cars require a special panda safety mode due to missing counters and checksums in the messages
        ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.hyundaiLegacy)]
      else:
        ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.hyundai, 0)]

      if candidate in CAMERA_SCC_CAR:
        ret.safetyConfigs[0].safetyParam |= Panda.FLAG_HYUNDAI_CAMERA_SCC

      #print('sccBus={}'.format(ret.sccBus))
      if ret.sccBus == 2:
        ret.scc13Available = 1290 in fingerprint[0] or 1290 in fingerprint[2]
        ret.scc14Available = 905 in fingerprint[0] or 905 in fingerprint[2]
        ret.openpilotLongitudinalControl = True
        ret.radarUnavailable = False
        if int(Params().get("KISALongAlt", encoding="utf8")) == 1:
          ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.hyundaiCommunity1)]
        elif int(Params().get("KISALongAlt", encoding="utf8")) == 2:
          ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.hyundaiCommunity2)]
        ret.pcmCruise = True
      else:
        ret.pcmCruise = not ret.openpilotLongitudinalControl

    if (ret.openpilotLongitudinalControl and int(Params().get("KISALongAlt", encoding="utf8")) not in (1, 2)) or Params().get_bool("ExperimentalLongitudinalEnabled"):
      ret.safetyConfigs[-1].safetyParam |= Panda.FLAG_HYUNDAI_LONG
    if ret.flags & HyundaiFlags.HYBRID:
      ret.safetyConfigs[-1].safetyParam |= Panda.FLAG_HYUNDAI_HYBRID_GAS
    elif ret.flags & HyundaiFlags.EV:
      ret.safetyConfigs[-1].safetyParam |= Panda.FLAG_HYUNDAI_EV_GAS

    if candidate in (CAR.HYUNDAI_KONA, CAR.HYUNDAI_KONA_EV, CAR.HYUNDAI_KONA_HEV, CAR.HYUNDAI_KONA_EV_2022):
      ret.flags |= HyundaiFlags.ALT_LIMITS.value
      ret.safetyConfigs[-1].safetyParam |= Panda.FLAG_HYUNDAI_ALT_LIMITS

    ret.centerToFront = ret.wheelbase * 0.4

    return ret

  @staticmethod
  def init(CP, logcan, sendcan):
    if CP.openpilotLongitudinalControl and not (CP.flags & HyundaiFlags.CANFD_CAMERA_SCC.value) and CP.experimentalLong and not CP.experimentalLongAlt:
      addr, bus = 0x7d0, 0
      if CP.flags & HyundaiFlags.CANFD_HDA2.value:
        addr, bus = 0x730, CanBus(CP).ECAN
      disable_ecu(logcan, sendcan, bus=bus, addr=addr, com_cont_req=b'\x28\x83\x01')
      enable_radar_tracks(CP, logcan, sendcan) # from ajouatom. really appreciate that.

    # for blinkers
    if CP.flags & HyundaiFlags.ENABLE_BLINKERS and CP.experimentalLong and not CP.experimentalLongAlt:
      disable_ecu(logcan, sendcan, bus=CanBus(CP).ECAN, addr=0x7B1, com_cont_req=b'\x28\x83\x01')

  def _update(self, c):
    ret = self.CS.update(self.cp, self.cp_cam)

    # most HKG cars has no long control, it is safer and easier to engage by main on
    if self.CC.ufc_mode_enabled and not self.CS.CP.experimentalLong:
      ret.cruiseState.enabled = ret.cruiseState.available

    if self.CS.CP.openpilotLongitudinalControl or self.CC.ufc_mode_enabled:
      ret.buttonEvents = create_button_events(self.CS.cruise_buttons[-1], self.CS.prev_cruise_buttons, BUTTONS_DICT)

    # On some newer model years, the CANCEL button acts as a pause/resume button based on the PCM state
    # To avoid re-engaging when openpilot cancels, check user engagement intention via buttons
    # Main button also can trigger an engagement on these cars
    allow_enable = any(btn in ENABLE_BUTTONS for btn in self.CS.cruise_buttons) or any(self.CS.main_buttons)
    events = self.create_common_events(ret, pcm_enable=self.CS.CP.pcmCruise, allow_enable=allow_enable)

    # low speed steer alert hysteresis logic (only for cars with steer cut off above 10 m/s)
    #if ret.vEgo < (self.CP.minSteerSpeed + 2.) and self.CP.minSteerSpeed > 10.:
    #  self.low_speed_alert = True
    #if ret.vEgo > (self.CP.minSteerSpeed + 4.):
    #  self.low_speed_alert = False
    if self.low_speed_alert and self.CC.no_mdps_mods:
      events.add(car.CarEvent.EventName.belowSteerSpeed)
    if self.CC.need_brake and not self.CC.longcontrol:
      events.add(EventName.needBrake)
    if not self.CC.lkas_temp_disabled:
      if self.CC.lanechange_manual_timer and ret.vEgo > 0.3:
        events.add(EventName.laneChangeManual)
      if self.CC.emergency_manual_timer:
        events.add(EventName.emgButtonManual)
      #if self.CC.driver_steering_torque_above_timer:
      #  events.add(EventName.driverSteering)
      if self.CC.standstill_res_button:
        events.add(EventName.standstillResButton)
      if self.CC.cruise_gap_adjusting:
        events.add(EventName.gapAdjusting)
      if self.CC.on_speed_bump_control and ret.vEgo > 8.3:
        events.add(EventName.speedBump)
      if self.CC.on_speed_control and ret.vEgo > 0.3:
        events.add(EventName.camSpeedDown)
      if self.CC.curv_speed_control and ret.vEgo > 8.3:
        events.add(EventName.curvSpeedDown)
      if self.CC.cut_in_control and ret.vEgo > 8.3:
        events.add(EventName.cutinDetection)
      if self.CC.driver_scc_set_control:
        events.add(EventName.sccDriverOverride)        
      if self.CC.autohold_popup_timer:
        events.add(EventName.autoHold)
      if self.CC.auto_res_starting:
        events.add(EventName.resCruise)
      if self.CC.e2e_standstill:
        events.add(EventName.chimeAtResume)
    if self.CS.cruiseState_standstill or self.CC.standstill_status == 1:
      #events.add(EventName.standStill)
      self.CP.standStill = True
    else:
      self.CP.standStill = False
    if self.CC.vFuture >= 1:
      self.CP.vFuture = self.CC.vFuture
    else:
      self.CP.vFuture = 0
    if self.CC.vFutureA >= 1:
      self.CP.vFutureA = self.CC.vFutureA
    else:
      self.CP.vFutureA = 0

    if self.CC.mode_change_timer and self.CS.out.cruiseState.modeSel == 0:
      events.add(EventName.modeChangeOpenpilot)
    elif self.CC.mode_change_timer and self.CS.out.cruiseState.modeSel == 1:
      events.add(EventName.modeChangeDistcurv)
    elif self.CC.mode_change_timer and self.CS.out.cruiseState.modeSel == 2:
      events.add(EventName.modeChangeDistance)
    elif self.CC.mode_change_timer and self.CS.out.cruiseState.modeSel == 3:
      events.add(EventName.modeChangeCurv)
    elif self.CC.mode_change_timer and self.CS.out.cruiseState.modeSel == 4:
      events.add(EventName.modeChangeOneway)
    elif self.CC.mode_change_timer and self.CS.out.cruiseState.modeSel == 5:
      events.add(EventName.modeChangeMaponly)

    if self.CC.lkas_temp_disabled:
      events.add(EventName.lkasDisabled)
    elif self.CC.lkas_temp_disabled_timer:
      events.add(EventName.lkasEnabled)

    ret.events = events.to_msg()

    return ret
