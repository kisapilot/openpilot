from cereal import car
from panda import Panda
from openpilot.common.conversions import Conversions as CV
from openpilot.selfdrive.car.hyundai.tunes import LatTunes, LongTunes, set_long_tune, set_lat_tune
from openpilot.selfdrive.car.hyundai.hyundaicanfd import CanBus
from openpilot.selfdrive.car.hyundai.values import HyundaiFlags, CAR, DBC, CANFD_CAR, CAMERA_SCC_CAR, CANFD_RADAR_SCC_CAR, \
                                         CANFD_UNSUPPORTED_LONGITUDINAL_CAR, EV_CAR, HYBRID_CAR, LEGACY_SAFETY_MODE_CAR, \
                                         UNSUPPORTED_LONGITUDINAL_CAR, Buttons, LEGACY_SAFETY_MODE_CAR_ALT
from openpilot.selfdrive.car.hyundai.radar_interface import RADAR_START_ADDR
from openpilot.selfdrive.car import create_button_events, get_safety_config
from openpilot.selfdrive.car.interfaces import CarInterfaceBase
from openpilot.selfdrive.car.disable_ecu import disable_ecu
from openpilot.selfdrive.car.hyundai.cruise_helper import enable_radar_tracks #ajouatom

from openpilot.common.params import Params
from decimal import Decimal

Ecu = car.CarParams.Ecu
SafetyModel = car.CarParams.SafetyModel
ButtonType = car.CarState.ButtonEvent.Type
EventName = car.CarEvent.EventName
ENABLE_BUTTONS = (Buttons.RES_ACCEL, Buttons.SET_DECEL, Buttons.CANCEL)
BUTTONS_DICT = {Buttons.RES_ACCEL: ButtonType.accelCruise, Buttons.SET_DECEL: ButtonType.decelCruise,
                Buttons.GAP_DIST: ButtonType.gapAdjustCruise, Buttons.CANCEL: ButtonType.cancel}


def set_safety_config_hyundai(candidate, CAN, can_fd=False):
  platform = SafetyModel.hyundaiCanfd if can_fd else \
             SafetyModel.hyundaiLegacy if candidate in LEGACY_SAFETY_MODE_CAR else \
             SafetyModel.hyundai
  cfgs = [get_safety_config(platform), ]
  if CAN.ECAN >= 4:
    cfgs.insert(0, get_safety_config(SafetyModel.noOutput))

  return cfgs


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
      # detect if car is hybrid
      if 0x105 in fingerprint[CAN.ECAN]:
        ret.flags |= HyundaiFlags.HYBRID.value
      elif candidate in EV_CAR:
        ret.flags |= HyundaiFlags.EV.value

      # detect HDA2 with ADAS Driving ECU
      if hda2:
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

    ret.minSteerSpeed = 16.67 # m/s

    ret.steerActuatorDelay = float(Decimal(params.get("SteerActuatorDelayAdj", encoding="utf8")) * Decimal('0.01'))
    ret.steerLimitTimer = float(Decimal(params.get("SteerLimitTimerAdj", encoding="utf8")) * Decimal('0.01'))

    ret.experimentalLong = Params().get_bool("ExperimentalLongitudinalEnabled")
    ret.experimentalLongAlt = candidate in LEGACY_SAFETY_MODE_CAR_ALT
    
    ret.tireStiffnessFactor = 1.

    #set_long_tune(ret.longitudinalTuning, LongTunes.KISA)
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

    if candidate in (CAR.AZERA_6TH_GEN, CAR.AZERA_HEV_6TH_GEN):
      ret.mass = 1600. if candidate == CAR.AZERA_6TH_GEN else 1675.  # ICE is ~average of 2.5L and 3.5L
      ret.wheelbase = 2.885
      ret.steerRatio = 14.5
    elif candidate in (CAR.SANTA_FE, CAR.SANTA_FE_2022, CAR.SANTA_FE_HEV_2022, CAR.SANTA_FE_PHEV_2022):
      ret.mass = 3982. * CV.LB_TO_KG
      ret.wheelbase = 2.766
      # Values from optimizer
      ret.steerRatio = 16.55  # 13.8 is spec end-to-end
      ret.tireStiffnessFactor = 0.82
    elif candidate in (CAR.SONATA, CAR.SONATA_HYBRID):
      ret.mass = 1513.
      ret.wheelbase = 2.84
      ret.steerRatio = 13.27 * 1.15   # 15% higher at the center seems reasonable
      ret.tireStiffnessFactor = 0.65
    elif candidate == CAR.SONATA_LF:
      ret.mass = 1536.
      ret.wheelbase = 2.804
      ret.steerRatio = 13.27 * 1.15   # 15% higher at the center seems reasonable
    elif candidate == CAR.PALISADE:
      ret.mass = 1999.
      ret.wheelbase = 2.90
      ret.steerRatio = 15.6 * 1.15
      ret.tireStiffnessFactor = 0.63
    elif candidate in (CAR.ELANTRA, CAR.ELANTRA_GT_I30):
      ret.mass = 1275.
      ret.wheelbase = 2.7
      ret.steerRatio = 15.4            # 14 is Stock | Settled Params Learner values are steerRatio: 15.401566348670535
      ret.tireStiffnessFactor = 0.385    # stiffnessFactor settled on 1.0081302973865127
    elif candidate == CAR.ELANTRA_2021:
      ret.mass = 2800. * CV.LB_TO_KG
      ret.wheelbase = 2.72
      ret.steerRatio = 12.9
      ret.tireStiffnessFactor = 0.65
    elif candidate == CAR.ELANTRA_HEV_2021:
      ret.mass = 3017. * CV.LB_TO_KG
      ret.wheelbase = 2.72
      ret.steerRatio = 12.9
      ret.tireStiffnessFactor = 0.65
    elif candidate == CAR.HYUNDAI_GENESIS:
      ret.mass = 2060.
      ret.wheelbase = 3.01
      ret.steerRatio = 16.5
    elif candidate in (CAR.KONA, CAR.KONA_EV, CAR.KONA_HEV, CAR.KONA_EV_2022, CAR.KONA_EV_2ND_GEN):
      ret.mass = {CAR.KONA_EV: 1685., CAR.KONA_HEV: 1425., CAR.KONA_EV_2022: 1743., CAR.KONA_EV_2ND_GEN: 1740.}.get(candidate, 1275.)
      ret.wheelbase = {CAR.KONA_EV_2ND_GEN: 2.66, }.get(candidate, 2.6)
      ret.steerRatio = {CAR.KONA_EV_2ND_GEN: 13.6, }.get(candidate, 13.42)  # Spec
      ret.tireStiffnessFactor = 0.385
    elif candidate in (CAR.IONIQ, CAR.IONIQ_EV_LTD, CAR.IONIQ_PHEV_2019, CAR.IONIQ_HEV_2022, CAR.IONIQ_EV_2020, CAR.IONIQ_PHEV):
      ret.mass = 1490.  # weight per hyundai site https://www.hyundaiusa.com/ioniq-electric/specifications.aspx
      ret.wheelbase = 2.7
      ret.steerRatio = 13.73  # Spec
      ret.tireStiffnessFactor = 0.385
    elif candidate in (CAR.IONIQ_5, CAR.IONIQ_6):
      ret.mass = 1948
      ret.wheelbase = 2.97
      ret.steerRatio = 14.26
      ret.tireStiffnessFactor = 0.65
    elif candidate == CAR.VELOSTER:
      ret.mass = 2917. * CV.LB_TO_KG
      ret.wheelbase = 2.80
      ret.steerRatio = 13.75 * 1.15
      ret.tireStiffnessFactor = 0.5
    elif candidate == CAR.TUCSON:
      ret.mass = 3520. * CV.LB_TO_KG
      ret.wheelbase = 2.67
      ret.steerRatio = 14.00 * 1.15
      ret.tireStiffnessFactor = 0.385
    elif candidate == CAR.TUCSON_4TH_GEN:
      ret.mass = 1630.  # average
      ret.wheelbase = 2.756
      ret.steerRatio = 16.
      ret.tireStiffnessFactor = 0.385
    elif candidate == CAR.SANTA_CRUZ_1ST_GEN:
      ret.mass = 1870.  # weight from Limited trim - the only supported trim
      ret.wheelbase = 3.000
      # steering ratio according to Hyundai News https://www.hyundainews.com/assets/documents/original/48035-2022SantaCruzProductGuideSpecsv2081521.pdf
      ret.steerRatio = 14.2
    elif candidate == CAR.CUSTIN_1ST_GEN:
      ret.mass = 1690.  # from https://www.hyundai-motor.com.tw/clicktobuy/custin#spec_0
      ret.wheelbase = 3.055
      ret.steerRatio = 17.0  # from learner
    elif candidate == CAR.STARIA_4TH_GEN:
      ret.mass = 2205.
      ret.wheelbase = 3.273
      ret.steerRatio = 11.94  # https://www.hyundai.com/content/dam/hyundai/au/en/models/staria-load/premium-pip-update-2023/spec-sheet/STARIA_Load_Spec-Table_March_2023_v3.1.pdf

    # Kia
    elif candidate == CAR.KIA_SORENTO:
      ret.mass = 1985.
      ret.wheelbase = 2.78
      ret.steerRatio = 14.4 * 1.1   # 10% higher at the center seems reasonable
    elif candidate in (CAR.KIA_NIRO_EV, CAR.KIA_NIRO_EV_2ND_GEN, CAR.KIA_NIRO_PHEV, CAR.KIA_NIRO_HEV_2021, CAR.KIA_NIRO_HEV_2ND_GEN, CAR.KIA_NIRO_PHEV_2022):
      ret.mass = 3543. * CV.LB_TO_KG  # average of all the cars
      ret.wheelbase = 2.7
      ret.steerRatio = 13.6  # average of all the cars
      ret.tireStiffnessFactor = 0.385
    elif candidate == CAR.KIA_SELTOS:
      ret.mass = 1337.
      ret.wheelbase = 2.63
      ret.steerRatio = 14.56
    elif candidate == CAR.KIA_SPORTAGE_5TH_GEN:
      ret.mass = 1725.  # weight from SX and above trims, average of FWD and AWD versions
      ret.wheelbase = 2.756
      ret.steerRatio = 13.6  # steering ratio according to Kia News https://www.kiamedia.com/us/en/models/sportage/2023/specifications
    elif candidate in (CAR.KIA_OPTIMA_G4, CAR.KIA_OPTIMA_G4_FL, CAR.KIA_OPTIMA_H, CAR.KIA_OPTIMA_H_G4_FL):
      ret.mass = 3558. * CV.LB_TO_KG
      ret.wheelbase = 2.80
      ret.steerRatio = 13.75
      ret.tireStiffnessFactor = 0.5
    elif candidate in (CAR.KIA_STINGER, CAR.KIA_STINGER_2022):
      ret.mass = 1825.
      ret.wheelbase = 2.78
      ret.steerRatio = 14.4 * 1.15   # 15% higher at the center seems reasonable
    elif candidate == CAR.KIA_FORTE:
      ret.mass = 2878. * CV.LB_TO_KG
      ret.wheelbase = 2.80
      ret.steerRatio = 13.75
      ret.tireStiffnessFactor = 0.5
    elif candidate == CAR.KIA_CEED:
      ret.mass = 1450.
      ret.wheelbase = 2.65
      ret.steerRatio = 13.75
      ret.tireStiffnessFactor = 0.5
    elif candidate in (CAR.KIA_K5_2021, CAR.KIA_K5_HEV_2020):
      ret.mass = 3381. * CV.LB_TO_KG
      ret.wheelbase = 2.85
      ret.steerRatio = 13.27  # 2021 Kia K5 Steering Ratio (all trims)
      ret.tireStiffnessFactor = 0.5
    elif candidate == CAR.KIA_EV6:
      ret.mass = 2055
      ret.wheelbase = 2.9
      ret.steerRatio = 16.
      ret.tireStiffnessFactor = 0.65
    elif candidate in (CAR.KIA_SORENTO_4TH_GEN, CAR.KIA_SORENTO_HEV_4TH_GEN):
      ret.wheelbase = 2.81
      ret.steerRatio = 13.5  # average of the platforms
      if candidate == CAR.KIA_SORENTO_4TH_GEN:
        ret.mass = 3957 * CV.LB_TO_KG
      else:
        ret.mass = 4396 * CV.LB_TO_KG
    elif candidate == CAR.KIA_CARNIVAL_4TH_GEN:
      ret.mass = 2087.
      ret.wheelbase = 3.09
      ret.steerRatio = 14.23
    elif candidate == CAR.KIA_K8_GL3:
      ret.mass = 1642.
      ret.wheelbase = 2.895
      ret.steerRatio = 13.27
    elif candidate == CAR.KIA_K8_HEV_1ST_GEN:
      ret.mass = 1630.  # https://carprices.ae/brands/kia/2023/k8/1.6-turbo-hybrid
      ret.wheelbase = 2.895
      ret.steerRatio = 13.27  # guesstimate from K5 platform

    # Genesis
    elif candidate == CAR.GENESIS_GV60_EV_1ST_GEN:
      ret.mass = 2205
      ret.wheelbase = 2.9
      # https://www.motor1.com/reviews/586376/2023-genesis-gv60-first-drive/#:~:text=Relative%20to%20the%20related%20Ioniq,5%2FEV6%27s%2014.3%3A1.
      ret.steerRatio = 12.6
    elif candidate == CAR.GENESIS_G70:
      ret.steerActuatorDelay = 0.1
      ret.mass = 1640.0
      ret.wheelbase = 2.84
      ret.steerRatio = 13.56
    elif candidate == CAR.GENESIS_G70_2020:
      ret.mass = 3673.0 * CV.LB_TO_KG
      ret.wheelbase = 2.83
      ret.steerRatio = 12.9
    elif candidate == CAR.GENESIS_GV70_1ST_GEN:
      ret.mass = 1950.
      ret.wheelbase = 2.87
      ret.steerRatio = 14.6
    elif candidate == CAR.GENESIS_G80:
      ret.mass = 2060.
      ret.wheelbase = 3.01
      ret.steerRatio = 16.5
    elif candidate == CAR.GENESIS_G90:
      ret.mass = 2200.
      ret.wheelbase = 3.15
      ret.steerRatio = 12.069
    elif candidate == CAR.GENESIS_GV80:
      ret.mass = 2258.
      ret.wheelbase = 2.95
      ret.steerRatio = 14.14
    else:
      # genesis
      if candidate == CAR.GENESIS_DH:
        ret.mass = 1930.
        ret.wheelbase = 3.01
      elif candidate == CAR.GENESIS_EQ900_HI:
        ret.mass = 2130.
        ret.wheelbase = 3.16
      # hyundai
      elif candidate == CAR.AVANTE_AD:
        ret.mass = 1250.
        ret.wheelbase = 2.7
      elif candidate == CAR.AVANTE_CN7:
        ret.mass = 1225.
        ret.wheelbase = 2.72
      elif candidate == CAR.AVANTE_HEV_CN7:
        ret.mass = 1335.
        ret.wheelbase = 2.72
      elif candidate == CAR.I30_PD:
        ret.mass = 1380.
        ret.wheelbase = 2.65
      elif candidate == CAR.GRANDEUR_IG:
        ret.mass = 1560.
        ret.wheelbase = 2.845
      elif candidate == CAR.GRANDEUR_HEV_IG:
        ret.mass = 1675.
        ret.wheelbase = 2.845
      elif candidate == CAR.GRANDEUR_FL_IG:
        ret.mass = 1625.
        ret.wheelbase = 2.885
      elif candidate == CAR.GRANDEUR_HEV_FL_IG:
        ret.mass = 1675.
        ret.wheelbase = 2.885
      elif candidate == CAR.NEXO_FE:
        ret.mass = 1885.
        ret.wheelbase = 2.79
        ret.steerRatio = 14.2
        ret.tireStiffnessFactor = 0.385  
      # kia
      elif candidate == CAR.K5_JF:
        ret.wheelbase = 2.805
        ret.mass = 1475.
      elif candidate == CAR.K5_HEV_JF:
        ret.wheelbase = 2.805
        ret.mass = 1600.
      elif candidate == CAR.K3_BD:
        ret.mass = 1260.
        ret.wheelbase = 2.70
      elif candidate == CAR.K7_YG:
        ret.mass = 1565.
        ret.wheelbase = 2.855
      elif candidate == CAR.K7_HEV_YG:
        ret.mass = 1680.
        ret.wheelbase = 2.855
      elif candidate == CAR.SOUL_EV_SK3:
        ret.mass = 1695.
        ret.wheelbase = 2.6
      elif candidate == CAR.MOHAVE_HM:
        ret.mass = 2285.
        ret.wheelbase = 2.895
      ret.tireStiffnessFactor = float(Decimal(params.get("TireStiffnessFactorAdj", encoding="utf8")) * Decimal('0.01'))
      ret.steerRatio = float(Decimal(params.get("SteerRatioAdj", encoding="utf8")) * Decimal('0.01'))

    # *** longitudinal control ***
    if candidate in CANFD_CAR:
      ret.longitudinalTuning.kpV = [0.1]
      ret.longitudinalTuning.kiV = [0.0]
      ret.experimentalLongitudinalAvailable = (candidate in (HYBRID_CAR | EV_CAR) and candidate not in
                                               (CANFD_UNSUPPORTED_LONGITUDINAL_CAR | CANFD_RADAR_SCC_CAR))
    else:
      ret.longitudinalTuning.kpV = [0.5]
      ret.longitudinalTuning.kiV = [0.0]
      ret.experimentalLongitudinalAvailable = True #candidate not in (LEGACY_SAFETY_MODE_CAR | CAMERA_SCC_CAR)


    ret.stoppingControl = True
    ret.startingState = True
    ret.vEgoStarting = 0.1
    ret.startAccel = 1.0
    ret.longitudinalActuatorDelayLowerBound = 0.5
    ret.longitudinalActuatorDelayUpperBound = 0.5

    ret.openpilotLongitudinalControl = experimental_long and ret.experimentalLongitudinalAvailable #experimental_long is LongControl toggle, not Experiment Mode

    # *** feature detection ***
    if candidate in CANFD_CAR:
      ret.enableBsm = 0x1e5 in fingerprint[CAN.ECAN]
      ret.mdpsBus = 0
      ret.sasBus = 0
      ret.sccBus = 0
      ret.fcaBus = 0
      ret.bsmAvailable = False
      ret.lfaAvailable = False
      ret.lvrAvailable = False
      ret.evgearAvailable = False
      ret.emsAvailable = False
      ret.autoHoldAvailable = False
      ret.lfaHdaAvailable = False
      ret.navAvailable = False
    else:
      ret.enableBsm = 0x58b in fingerprint[0]
      ret.mdpsBus = 0
      ret.sasBus = 1 if 688 in fingerprint[1] and 1296 not in fingerprint[1] else 0
      ret.sccBus = 2 if int(Params().get("KISALongAlt", encoding="utf8")) in (1, 2) and not Params().get_bool("ExperimentalLongitudinalEnabled") else 0
      #ret.sccBus = 0 if 1056 in fingerprint[0] else 1 if 1056 in fingerprint[1] and 1296 not in fingerprint[1] else 2 if 1056 in fingerprint[2] else -1
      ret.fcaBus = 0 if 909 in fingerprint[0] else 2 if 909 in fingerprint[2] else -1
      ret.bsmAvailable = True if 1419 in fingerprint[0] else False
      ret.lfaAvailable = True if 1157 in fingerprint[2] else False
      ret.lvrAvailable = True if 871 in fingerprint[0] else False
      ret.evgearAvailable = True if 882 in fingerprint[0] else False
      ret.emsAvailable = True if 870 in fingerprint[0] else False
      ret.autoHoldAvailable = 1151 in fingerprint[0]
      ret.lfaHdaAvailable = 1157 in fingerprint[0]
      ret.navAvailable = 1348 in fingerprint[0]

    # *** panda safety config ***
    ret.safetyConfigs = set_safety_config_hyundai(candidate, CAN, can_fd=(candidate in CANFD_CAR))

    if hda2:
      ret.flags |= HyundaiFlags.CANFD_HDA2.value
      ret.safetyConfigs[-1].safetyParam |= Panda.FLAG_HYUNDAI_CANFD_HDA2

    if candidate in CANFD_CAR:
      if hda2 and ret.flags & HyundaiFlags.CANFD_HDA2_ALT_STEERING:
        ret.safetyConfigs[-1].safetyParam |= Panda.FLAG_HYUNDAI_CANFD_HDA2_ALT_STEERING
      if ret.flags & HyundaiFlags.CANFD_ALT_BUTTONS:
        ret.safetyConfigs[-1].safetyParam |= Panda.FLAG_HYUNDAI_CANFD_ALT_BUTTONS

    if ret.flags & HyundaiFlags.CANFD_CAMERA_SCC or candidate in CAMERA_SCC_CAR:
      ret.safetyConfigs[-1].safetyParam |= Panda.FLAG_HYUNDAI_CAMERA_SCC
    elif candidate not in CANFD_CAR:
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

    if candidate in (CAR.KONA, CAR.KONA_EV, CAR.KONA_HEV, CAR.KONA_EV_2022):
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
    ret = self.CS.update(self.cp, self.cp2, self.cp_cam)

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
    #if self.low_speed_alert:
    #  events.add(car.CarEvent.EventName.belowSteerSpeed)

    #if abs(ret.steeringAngle) > 90. and EventName.steerTempUnavailable not in events.events:
    #  events.add(EventName.steerTempUnavailable)
    # if self.ufc_mode_enabled and EventName.pedalPressed in events.events:
    #   events.events.remove(EventName.pedalPressed)
    if ret.vEgo < self.CP.minSteerSpeed and self.CC.no_mdps_mods:
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
    if self.CC.v_cruise_kph_auto_res > (20 if self.CS.is_set_speed_in_mph else 30):
      self.CP.vCruisekph = self.CC.v_cruise_kph_auto_res
    else:
      self.CP.vCruisekph = 0
    if self.CC.res_speed != 0:
      self.CP.resSpeed = self.CC.res_speed
    else:
      self.CP.resSpeed = 0
    if self.CC.vFuture >= 1:
      self.CP.vFuture = self.CC.vFuture
    else:
      self.CP.vFuture = 0
    if self.CC.vFutureA >= 1:
      self.CP.vFutureA = self.CC.vFutureA
    else:
      self.CP.vFutureA = 0
    self.CP.aqValue = self.CC.aq_value
    self.CP.aqValueRaw = self.CC.aq_value_raw

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

  def apply(self, c, now_nanos):
    return self.CC.update(c, self.CS, now_nanos)
