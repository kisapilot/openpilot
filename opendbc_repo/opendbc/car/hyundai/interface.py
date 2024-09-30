from panda import Panda
from opendbc.car import get_safety_config, structs
from opendbc.car.hyundai.hyundaicanfd import CanBus
from opendbc.car.hyundai.values import HyundaiFlags, CAR, DBC, CANFD_CAR, CAMERA_SCC_CAR, CANFD_RADAR_SCC_CAR, \
                                                   CANFD_UNSUPPORTED_LONGITUDINAL_CAR, EV_CAR, HYBRID_CAR, LEGACY_SAFETY_MODE_CAR, \
                                                   UNSUPPORTED_LONGITUDINAL_CAR, Buttons, LEGACY_SAFETY_MODE_CAR_ALT, ANGLE_CONTROL_CAR
from opendbc.car.hyundai.radar_interface import RADAR_START_ADDR
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.disable_ecu import disable_ecu

from opendbc.car.hyundai.cruise_helper import enable_radar_tracks #ajouatom
from opendbc.car.hyundai.tunes import LatTunes, set_lat_tune
from openpilot.common.params import Params
from decimal import Decimal

Ecu = structs.CarParams.Ecu
SteerControlType = structs.CarParams.SteerControlType
ENABLE_BUTTONS = (Buttons.RES_ACCEL, Buttons.SET_DECEL, Buttons.CANCEL)


class CarInterface(CarInterfaceBase):
  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, experimental_long, docs) -> structs.CarParams:
    ret.carName = "hyundai"
    ret.radarUnavailable = RADAR_START_ADDR not in fingerprint[1] or DBC[ret.carFingerprint]["radar"] is None

    cam_can = CanBus(None, fingerprint).CAM
    hda2 = 0x50 in fingerprint[cam_can] or 0x110 in fingerprint[cam_can]
    CAN = CanBus(None, fingerprint, hda2)

    if candidate in CANFD_CAR:
      ret.isCanFD = True
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
      cfgs = [get_safety_config(structs.CarParams.SafetyModel.hyundaiCanfd), ]
      if CAN.ECAN >= 4:
        cfgs.insert(0, get_safety_config(structs.CarParams.SafetyModel.noOutput))
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
          ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.hyundaiLegacy)]
        else:
          ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.hyundai, 0)]
      elif candidate in LEGACY_SAFETY_MODE_CAR_ALT or (candidate in LEGACY_SAFETY_MODE_CAR and Params().get_bool("UFCModeEnabled")):
        ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.hyundaiCommunity1Legacy)]
      elif Params().get_bool("UFCModeEnabled"):
        ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.hyundaiCommunity1)]
      elif candidate in LEGACY_SAFETY_MODE_CAR:
        # these cars require a special panda safety mode due to missing counters and checksums in the messages
        ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.hyundaiLegacy)]
      else:
        ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.hyundai, 0)]

      if candidate in CAMERA_SCC_CAR:
        ret.safetyConfigs[0].safetyParam |= Panda.FLAG_HYUNDAI_CAMERA_SCC

      if ret.sccBus == 2:
        ret.scc13Available = 1290 in fingerprint[0] or 1290 in fingerprint[2]
        ret.scc14Available = 905 in fingerprint[0] or 905 in fingerprint[2]
        ret.openpilotLongitudinalControl = True
        ret.radarUnavailable = False
        if int(Params().get("KISALongAlt", encoding="utf8")) == 1:
          ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.hyundaiCommunity1)]
        elif int(Params().get("KISALongAlt", encoding="utf8")) == 2:
          ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.hyundaiCommunity2)]
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
    ret.dashcamOnly = False

    return ret

  @staticmethod
  def init(CP, can_recv, can_send):
    if CP.openpilotLongitudinalControl and not (CP.flags & HyundaiFlags.CANFD_CAMERA_SCC.value) and CP.experimentalLong and not CP.experimentalLongAlt:
      addr, bus = 0x7d0, 0
      if CP.flags & HyundaiFlags.CANFD_HDA2.value:
        addr, bus = 0x730, CanBus(CP).ECAN
      disable_ecu(can_recv, can_send, bus=bus, addr=addr, com_cont_req=b'\x28\x83\x01')
      enable_radar_tracks(CP, can_recv, can_send) # from ajouatom. really appreciate that.

    # for blinkers
    if CP.flags & HyundaiFlags.ENABLE_BLINKERS:
      disable_ecu(can_recv, can_send, bus=CanBus(CP).ECAN, addr=0x7B1, com_cont_req=b'\x28\x83\x01')
