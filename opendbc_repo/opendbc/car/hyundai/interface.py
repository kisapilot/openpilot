from opendbc.car import Bus, get_safety_config, structs, uds
from opendbc.car.hyundai.hyundaicanfd import CanBus
from opendbc.car.hyundai.values import HyundaiFlags, CAR, DBC, \
                                                   CANFD_UNSUPPORTED_LONGITUDINAL_CAR, \
                                                   UNSUPPORTED_LONGITUDINAL_CAR, HyundaiSafetyFlags
from opendbc.car.hyundai.radar_interface import RADAR_START_ADDR
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.disable_ecu import disable_ecu
from opendbc.car.hyundai.carcontroller import CarController
from opendbc.car.hyundai.carstate import CarState
from opendbc.car.hyundai.radar_interface import RadarInterface

ButtonType = structs.CarState.ButtonEvent.Type

from opendbc.car.hyundai.tunes import LatTunes, set_lat_tune
from openpilot.common.params import Params

Ecu = structs.CarParams.Ecu
SteerControlType = structs.CarParams.SteerControlType
# Cancel button can sometimes be ACC pause/resume button, main button can also enable on some cars
ENABLE_BUTTONS = (ButtonType.accelCruise, ButtonType.decelCruise, ButtonType.cancel, ButtonType.mainCruise, ButtonType.lfa)


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController
  RadarInterface = RadarInterface

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = "hyundai"

    # "LKA steering" if LKAS or LKAS_ALT messages are seen coming from the camera.
    # Generally means our LKAS message is forwarded to another ECU (commonly ADAS ECU)
    # that finally retransmits our steering command in LFA or LFA_ALT to the MDPS.
    # "LFA steering" if camera directly sends LFA to the MDPS
    cam_can = CanBus(None, fingerprint).CAM
    lka_steering = 0x50 in fingerprint[cam_can] or 0x110 in fingerprint[cam_can]
    CAN = CanBus(None, fingerprint, lka_steering)

    params = Params()

    kisaLongAlt = params.get("KISALongAlt", return_default=True)

    if ret.flags & HyundaiFlags.CANFD:
      # Shared configuration for CAN-FD cars
      ret.alphaLongitudinalAvailable = candidate not in CANFD_UNSUPPORTED_LONGITUDINAL_CAR
      if lka_steering and Ecu.adas not in [fw.ecu for fw in car_fw]:
        # this needs to be figured out for cars without an ADAS ECU
        ret.alphaLongitudinalAvailable = False

      ret.isCanFD = True
      ret.enableBsm = 0x1e5 in fingerprint[CAN.ECAN]
      ret.sccBus = 2 if kisaLongAlt and not params.get_bool("AlphaLongitudinalEnabled") else 0
      ret.bsmAvailable = False
      ret.lfaAvailable = False
      ret.lvrAvailable = False
      ret.evgearAvailable = False
      ret.emsAvailable = False
      ret.autoHoldAvailable = 0x60 in fingerprint[CAN.ECAN]
      ret.lfaHdaAvailable = False
      ret.navAvailable = False
      ret.adrvAvailable = (lka_steering and 0x200 in fingerprint[CAN.ECAN]) or 0x200 in fingerprint[cam_can]
      ret.tpmsAvailable = 0x3a0 in fingerprint[CAN.ECAN]
      ret.isAngleControl = 0xcb in fingerprint[CAN.ECAN] or 0xcb in fingerprint[CAN.ACAN] or 0xcb in fingerprint[cam_can]
      ret.adrvControl = (not lka_steering) and (0x1ea in fingerprint[cam_can])
      ret.capacitiveSteeringWheel = 0x2af in fingerprint[CAN.ECAN] or 0x2af in fingerprint[CAN.ACAN] or 0x2af in fingerprint[cam_can]

      # Check if the car is hybrid. Only HEV/PHEV cars have 0xFA on E-CAN.
      if 0xFA in fingerprint[CAN.ECAN]:
        ret.flags |= HyundaiFlags.HYBRID.value

      if lka_steering:
        # detect LKA steering
        ret.flags |= HyundaiFlags.CANFD_LKA_STEERING.value
        if 0x110 in fingerprint[CAN.CAM]:
          ret.flags |= HyundaiFlags.CANFD_LKA_STEERING_ALT.value
      else:
        # no LKA steering
        if 0x1cf not in fingerprint[CAN.ECAN]:
          ret.flags |= HyundaiFlags.CANFD_ALT_BUTTONS.value
        if not ret.flags & HyundaiFlags.RADAR_SCC:
          ret.flags |= HyundaiFlags.CANFD_CAMERA_SCC.value

      # Some LKA steering cars have alternative messages for gear checks
      # ICE cars do not have 0x130; GEARS message on 0x40 or 0x70 instead
      if 0x130 not in fingerprint[CAN.ECAN]:
        if 0x40 not in fingerprint[CAN.ECAN]:
          ret.flags |= HyundaiFlags.CANFD_ALT_GEARS_2.value
        else:
          ret.flags |= HyundaiFlags.CANFD_ALT_GEARS.value

      cfgs = [get_safety_config(structs.CarParams.SafetyModel.hyundaiCanfd), ]
      if CAN.ECAN >= 4:
        cfgs.insert(0, get_safety_config(structs.CarParams.SafetyModel.noOutput))
      ret.safetyConfigs = cfgs

      if ret.flags & HyundaiFlags.CANFD_LKA_STEERING:
        ret.safetyConfigs[-1].safetyParam |= HyundaiSafetyFlags.CANFD_LKA_STEERING.value
        if ret.flags & HyundaiFlags.CANFD_LKA_STEERING_ALT:
          ret.safetyConfigs[-1].safetyParam |= HyundaiSafetyFlags.CANFD_LKA_STEERING_ALT.value
      if ret.flags & HyundaiFlags.CANFD_ALT_BUTTONS:
        ret.safetyConfigs[-1].safetyParam |= HyundaiSafetyFlags.CANFD_ALT_BUTTONS.value
      if ret.flags & HyundaiFlags.CANFD_CAMERA_SCC:
        ret.safetyConfigs[-1].safetyParam |= HyundaiSafetyFlags.CAMERA_SCC.value
      if ret.adrvControl:
        ret.safetyConfigs[-1].safetyParam |= HyundaiSafetyFlags.CANFD_ADRV_CONTROL.value
      if ret.sccBus == 2:
        ret.safetyConfigs[-1].safetyParam |= HyundaiSafetyFlags.LONG.value
      if params.get_bool("LFAButtonEngagement"):
        ret.safetyConfigs[-1].safetyParam |= HyundaiSafetyFlags.KISA_COMMUNITY.value

      if ret.sccBus == 2:
        ret.radarUnavailable = False
        ret.openpilotLongitudinalControl = True
        ret.pcmCruise = True
      else:
        ret.radarUnavailable = RADAR_START_ADDR not in fingerprint[1] or Bus.radar not in DBC[ret.carFingerprint]
        ret.openpilotLongitudinalControl = alpha_long and ret.alphaLongitudinalAvailable
        ret.pcmCruise = not ret.openpilotLongitudinalControl
      ret.startingState = True
      ret.vEgoStarting = 0.1
      ret.startAccel = 1.0
      ret.longitudinalActuatorDelay = 0.5

    else:
      ret.isCanFD = False
      # Shared configuration for non CAN-FD cars
      ret.alphaLongitudinalAvailable = candidate not in UNSUPPORTED_LONGITUDINAL_CAR
      ret.enableBsm = 0x58b in fingerprint[0]
      ret.sccBus = 2 if kisaLongAlt and not params.get_bool("AlphaLongitudinalEnabled") else 0
      ret.bsmAvailable = 1419 in fingerprint[0]
      ret.lfaAvailable = 1157 in fingerprint[2]
      ret.lvrAvailable = 872 in fingerprint[0]
      ret.evgearAvailable = 882 in fingerprint[0]
      ret.emsAvailable = 870 in fingerprint[0]
      ret.autoHoldAvailable = 1151 in fingerprint[0]
      ret.lfaHdaAvailable = 1157 in fingerprint[0]
      ret.navAvailable = 1348 in fingerprint[0]
      ret.adrvAvailable = False
      ret.tpmsAvailable = 1427 in fingerprint[0]
      ret.isAngleControl = False
      ret.evInfo = 1291 in fingerprint[0]
      ret.adrvControl = False
      ret.capacitiveSteeringWheel = False

      # Send LFA message on cars with HDA
      if 0x485 in fingerprint[2]:
        ret.flags |= HyundaiFlags.SEND_LFA.value

      # These cars use the FCA11 message for the AEB and FCW signals, all others use SCC12
      if 0x38d in fingerprint[0] or 0x38d in fingerprint[2]:
        ret.flags |= HyundaiFlags.USE_FCA.value

      if ret.flags & HyundaiFlags.LEGACY:
        # these cars require a special panda safety mode due to missing counters and checksums in the messages
        ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.hyundaiLegacy)]
      else:
        ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.hyundai, 0)]

      if ret.sccBus == 2:
        ret.scc13Available = 1290 in fingerprint[0] or 1290 in fingerprint[2]
        ret.scc14Available = 905 in fingerprint[0] or 905 in fingerprint[2]
        ret.openpilotLongitudinalControl = True
        ret.radarUnavailable = False
        ret.pcmCruise = True
      else:
        ret.radarUnavailable = RADAR_START_ADDR not in fingerprint[1] or Bus.radar not in DBC[ret.carFingerprint]
        ret.openpilotLongitudinalControl = alpha_long and ret.alphaLongitudinalAvailable
        ret.pcmCruise = not ret.openpilotLongitudinalControl
      ret.startingState = True
      ret.vEgoStarting = 0.1
      ret.startAccel = 1.0
      ret.longitudinalActuatorDelay = 0.5

      if (ret.flags & HyundaiFlags.CAMERA_SCC) or ret.sccBus == 2:
        ret.safetyConfigs[0].safetyParam |= HyundaiSafetyFlags.CAMERA_SCC.value
      if ret.sccBus == 2:
        ret.safetyConfigs[-1].safetyParam |= HyundaiSafetyFlags.LONG.value

      # These cars have the LFA button on the steering wheel
      if 0x391 in fingerprint[0]:
        ret.flags |= HyundaiFlags.HAS_LDA_BUTTON.value
      if params.get_bool("UFCModeEnabled"):
        ret.safetyConfigs[-1].safetyParam |= HyundaiSafetyFlags.KISA_COMMUNITY.value

    # Common lateral control setup

    ret.centerToFront = ret.wheelbase * 0.4
    ret.steerActuatorDelay = params.get("SteerActuatorDelayAdj", return_default=True) * 0.01   #0.1
    ret.steerLimitTimer = params.get("SteerLimitTimerAdj", return_default=True) * 0.01   #0.4

    ret.smoothSteer.method = params.get("KisaSteerMethod", return_default=True)   # 1
    ret.smoothSteer.maxSteeringAngle = params.get("KisaMaxSteeringAngle", return_default=True)   # 90
    ret.smoothSteer.maxDriverAngleWait = params.get("KisaMaxDriverAngleWait", return_default=True)  # 0.002
    ret.smoothSteer.maxSteerAngleWait = params.get("KisaMaxSteerAngleWait", return_default=True)   # 0.001  # 10 sec
    ret.smoothSteer.driverAngleWait = params.get("KisaDriverAngleWait", return_default=True)  #0.001

    ret.experimentalLong = params.get_bool("AlphaLongitudinalEnabled")
    
    if ret.isAngleControl:    
      ret.steerControlType = SteerControlType.angle
    else:
      lat_control_method = params.get("LateralControlMethod", return_default=True)
      if lat_control_method == 0:
        set_lat_tune(ret.lateralTuning, LatTunes.PID)
      elif lat_control_method == 1:
        set_lat_tune(ret.lateralTuning, LatTunes.INDI)
      elif lat_control_method == 2:
        set_lat_tune(ret.lateralTuning, LatTunes.LQR)
      elif lat_control_method == 3:
        #set_lat_tune(ret.lateralTuning, LatTunes.TORQUE)
        CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    if (ret.openpilotLongitudinalControl and not kisaLongAlt) or params.get_bool("AlphaLongitudinalEnabled"):
      ret.safetyConfigs[-1].safetyParam |= HyundaiSafetyFlags.LONG.value
    if ret.flags & HyundaiFlags.HYBRID:
      ret.safetyConfigs[-1].safetyParam |= HyundaiSafetyFlags.HYBRID_GAS.value
    elif ret.flags & HyundaiFlags.EV:
      ret.safetyConfigs[-1].safetyParam |= HyundaiSafetyFlags.EV_GAS.value
    elif ret.flags & HyundaiFlags.FCEV:
      ret.safetyConfigs[-1].safetyParam |= HyundaiSafetyFlags.FCEV_GAS.value

    # Car specific configuration overrides

    if candidate == CAR.KIA_OPTIMA_G4_FL:
      ret.steerActuatorDelay = 0.2

    ret.dashcamOnly = False

    return ret

  @staticmethod
  def init(CP, can_recv, can_send, communication_control=None):
    # 0x80 silences response
    if communication_control is None:
      communication_control = bytes([uds.SERVICE_TYPE.COMMUNICATION_CONTROL, 0x80 | uds.CONTROL_TYPE.DISABLE_RX_DISABLE_TX, uds.MESSAGE_TYPE.NORMAL])

    if CP.openpilotLongitudinalControl and not (CP.flags & (HyundaiFlags.CANFD_CAMERA_SCC | HyundaiFlags.CAMERA_SCC)):
      addr, bus = 0x7d0, CanBus(CP).ECAN if CP.flags & HyundaiFlags.CANFD else 0
      if CP.flags & HyundaiFlags.CANFD_LKA_STEERING.value:
        addr, bus = 0x730, CanBus(CP).ECAN
      disable_ecu(can_recv, can_send, bus=bus, addr=addr, com_cont_req=communication_control)

    # for blinkers
    if CP.flags & HyundaiFlags.ENABLE_BLINKERS:
      disable_ecu(can_recv, can_send, bus=CanBus(CP).ECAN, addr=0x7B1, com_cont_req=communication_control)

  @staticmethod
  def deinit(CP, can_recv, can_send):
    communication_control = bytes([uds.SERVICE_TYPE.COMMUNICATION_CONTROL, 0x80 | uds.CONTROL_TYPE.ENABLE_RX_ENABLE_TX, uds.MESSAGE_TYPE.NORMAL])
    CarInterface.init(CP, can_recv, can_send, communication_control)
