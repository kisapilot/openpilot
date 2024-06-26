#!/usr/bin/env python3
import datetime
import os
import signal
import sys
import traceback

from cereal import log
import cereal.messaging as messaging
import openpilot.system.sentry as sentry
from openpilot.common.basedir import PYEXTRADIR
from openpilot.common.params import Params, ParamKeyType
from openpilot.common.text_window import TextWindow
from openpilot.system.hardware import HARDWARE, PC
from openpilot.system.manager.helpers import unblock_stdout, write_onroad_params, save_bootlog
from openpilot.system.manager.process import ensure_running
from openpilot.system.manager.process_config import managed_processes
from openpilot.system.athena.registration import register, UNREGISTERED_DONGLE_ID
from openpilot.common.swaglog import cloudlog, add_file_handler
from openpilot.system.version import get_build_metadata, terms_version, training_version


sys.path.append(os.path.join(PYEXTRADIR, "pyextra"))


def manager_init() -> None:
  #save_bootlog()

  build_metadata = get_build_metadata()

  params = Params()
  params.clear_all(ParamKeyType.CLEAR_ON_MANAGER_START)
  params.clear_all(ParamKeyType.CLEAR_ON_ONROAD_TRANSITION)
  params.clear_all(ParamKeyType.CLEAR_ON_OFFROAD_TRANSITION)
  if build_metadata.release_channel:
    params.clear_all(ParamKeyType.DEVELOPMENT_ONLY)

  default_params: list[tuple[str, str | bytes]] = [
    ("CompletedTrainingVersion", "0"),
    ("DisengageOnAccelerator", "0"),
    ("GsmMetered", "1"),
    ("HasAcceptedTerms", "0"),
    ("LanguageSetting", "main_en"),
    ("OpenpilotEnabledToggle", "1"),
    ("LongitudinalPersonality", str(log.LongitudinalPersonality.standard)),
    ("IsMetric", "1"),
    ("IsOpenpilotViewEnabled", "0"),
    ("KisaAutoShutdown", "12"),
    ("KisaForceShutdown", "5"),
    ("KisaAutoScreenOff", "-2"),
    ("KisaUIBrightness", "0"),
    ("KisaUIVolumeBoost", "0"),
    ("KisaEnableDriverMonitoring", "1"),
    ("KisaEnableLogger", "0"),
    ("KisaEnableUploader", "0"),
    ("KisaEnableGetoffAlert", "0"),
    ("KisaAutoResume", "1"),
    ("KisaVariableCruise", "1"),
    ("KisaLaneChangeSpeed", "20"),
    ("KisaAutoLaneChangeDelay", "0"),
    ("KisaSteerAngleCorrection", "0"),
    ("PutPrebuiltOn", "0"),
    ("LdwsCarFix", "0"),
    ("LateralControlMethod", "3"),
    ("CruiseStatemodeSelInit", "1"),
    ("InnerLoopGain", "35"),
    ("OuterLoopGain", "20"),
    ("TimeConstant", "14"),
    ("ActuatorEffectiveness", "20"),
    ("Scale", "1500"),
    ("LqrKi", "16"),
    ("DcGain", "265"),
    ("PidKp", "25"),
    ("PidKi", "40"),
    ("PidKd", "150"),
    ("PidKf", "7"),
    ("TorqueKp", "10"),
    ("TorqueKf", "10"),
    ("TorqueKi", "1"),
    ("TorqueFriction", "80"),
    ("TorqueUseAngle", "1"),
    ("TorqueMaxLatAccel", "30"),
    ("TorqueAngDeadZone", "10"),
    ("CameraOffsetAdj", "40"),
    ("PathOffsetAdj", "0"),
    ("SteerRatioAdj", "1375"),
    ("SteerRatioMaxAdj", "1750"),
    ("SteerActuatorDelayAdj", "15"),
    ("SteerLimitTimerAdj", "100"),
    ("TireStiffnessFactorAdj", "85"),
    ("SteerMaxBaseAdj", "384"),
    ("SteerMaxAdj", "384"),
    ("SteerDeltaUpBaseAdj", "3"),
    ("SteerDeltaUpAdj", "3"),
    ("SteerDeltaDownBaseAdj", "7"),
    ("SteerDeltaDownAdj", "7"),
    ("LeftCurvOffsetAdj", "0"),
    ("RightCurvOffsetAdj", "0"),
    ("DebugUi1", "0"),
    ("DebugUi2", "0"),
    ("DebugUi3", "0"),
    ("LongLogDisplay", "0"),
    ("KisaBlindSpotDetect", "1"),
    ("KisaMaxAngleLimit", "80"),
    ("KisaSteerMethod", "0"),
    ("KisaMaxSteeringAngle", "90"),
    ("KisaMaxDriverAngleWait", "0.002"),
    ("KisaMaxSteerAngleWait", "0.001"),
    ("KisaDriverAngleWait", "0.001"),
    ("KisaSpeedLimitOffset", "0"),
    ("KisaLiveSteerRatio", "1"),
    ("KisaVariableSteerMax", "0"),
    ("KisaVariableSteerDelta", "0"),
    ("FingerprintTwoSet", "0"),
    ("KisaDrivingRecord", "0"),
    ("KisaTurnSteeringDisable", "0"),
    ("CarModel", ""),
    ("KisaHotspotOnBoot", "0"),
    ("KisaSSHLegacy", "1"),
    ("CruiseOverMaxSpeed", "0"),
    ("JustDoGearD", "0"),
    ("LanelessMode", "2"),
    ("ComIssueGone", "1"),
    ("MaxSteer", "384"),
    ("MaxRTDelta", "112"),
    ("MaxRateUp", "3"),
    ("MaxRateDown", "7"),
    ("SteerThreshold", "150"),
    ("RecordingCount", "200"),
    ("RecordingQuality", "1"),
    ("RecordingRunning", "0"),
    ("CruiseGapAdjust", "0"),
    ("AutoEnable", "0"),
    ("CruiseAutoRes", "0"),
    ("AutoResOption", "0"),
    ("AutoResCondition", "0"),
    ("KisaMonitoringMode", "0"),
    ("KisaMonitorEyesThreshold", "45"),
    ("KisaMonitorBlinkThreshold", "75"),
    ("UFCModeEnabled", "0"),
    ("SteerWarningFix", "0"),
    ("CruiseGap1", "11"),
    ("CruiseGap2", "12"),
    ("CruiseGap3", "13"),
    ("CruiseGap4", "15"),
    ("DynamicTRGap", "1"),
    ("DynamicTRSpd", "0,20,40,60,110"),
    ("DynamicTRSet", "1.1,1.2,1.3,1.4,1.5"),
    ("LCTimingFactorUD", "1"),
    ("LCTimingFactor30", "30"),
    ("LCTimingFactor60", "60"),
    ("LCTimingFactor80", "80"),
    ("LCTimingFactor110", "100"),
    ("KisaUIBrightnessOff", "10"),
    ("LCTimingFactorEnable", "0"),
    ("AutoEnableSpeed", "5"),
    ("SafetyCamDecelDistGain", "0"),
    ("KisaLiveTunePanelEnable", "0"),
    ("RadarLongHelper", "2"),
    ("GitPullOnBoot", "0"),
    ("LiveSteerRatioPercent", "0"),
    ("StoppingDistAdj", "0"),
    ("ShowError", "1"),
    ("AutoResLimitTime", "0"),
    ("VCurvSpeedC", "30,50,70,90"),
    ("VCurvSpeedT", "43,58,73,87"),
    ("VCurvSpeedCMPH", "20,30,45,60"),
    ("VCurvSpeedTMPH", "27,36,46,57"),
    ("OCurvSpeedC", "30,40,50,60,70"),
    ("OCurvSpeedT", "35,45,60,70,80"),
    ("OSMCustomSpeedLimitC", "30,40,50,60,70,90"),
    ("OSMCustomSpeedLimitT", "30,40,65,72,80,95"),
    ("StockNaviSpeedEnabled", "0"),
    ("KISANaviSelect", "0"),
    ("E2ELong", "0"),
    ("KISAServer", "0"),
    ("IgnoreCANErroronISG", "0"),
    ("RESCountatStandstill", "20"),
    ("KisaSpeedLimitOffsetOption", "0"),
    ("KisaSpeedLimitSignType", "0"),
    ("StockLKASEnabled", "0"),
    ("SpeedLimitDecelOff", "0"),
    ("CurvDecelOption", "2"),
    ("FCA11Message", "0"),
    ("StandstillResumeAlt", "0"),
    ("AutoRESDelay", "1"),
    ("UseRadarTrack", "0"),
    ("RadarDisable", "0"),
    ("DesiredCurvatureLimit", "10"),
    ("CustomTREnabled", "1"),
    ("RoadList", "RoadName1,+0.0,RoadName2,-0.0\nRoadName3,30,RoadName4,60"),
    ("LaneWidth", "37"),
    ("SpdLaneWidthSpd", "0,31"),
    ("SpdLaneWidthSet", "2.8,3.5"),
    ("BottomTextView", "0"),
    ("CloseToRoadEdge", "0"),
    ("LeftEdgeOffset", "0"),
    ("RightEdgeOffset", "0"),
    ("AvoidLKASFaultEnabled", "1"),
    ("AvoidLKASFaultMaxAngle", "85"),
    ("AvoidLKASFaultMaxFrame", "89"),
    ("AvoidLKASFaultBeyond", "0"),
    ("UseStockDecelOnSS", "0"),
    ("AnimatedRPM", "1"),
    ("AnimatedRPMMax", "3600"),
    ("RoutineDriveOption", "KISA"),
    ("SshEnabled", "1"),
    ("UserSpecificFeature", "0"),
    ("KisaWakeUp", "0"),
    ("MultipleLateralUse", "2"),
    ("MultipleLateralOpS", "3,3,0"),
    ("MultipleLateralSpd", "60,90"),
    ("MultipleLateralOpA", "3,3,0"),
    ("MultipleLateralAng", "20,35"),
    ("StoppingDist", "35"),
    ("StopAtStopSign", "0"),
    ("VarCruiseSpeedFactor", "15"),
    ("KISASpeedBump", "0"),
    ("KISAEarlyStop", "1"),
    ("DoNotDisturbMode", "0"),
    ("DepartChimeAtResume", "0"),
    ("CommaStockUI", "0"),
    ("CruiseGapBySpdOn", "0"),
    ("CruiseGapBySpdSpd", "25,55,130"),
    ("CruiseGapBySpdGap", "1,2,3,4"),
    ("CruiseSetwithRoadLimitSpeedEnabled", "0"),
    ("CruiseSetwithRoadLimitSpeedOffset", "0"),
    ("KisaLiveTorque", "1"),
    ("ExternalDeviceIP", ""),
    ("ExternalDeviceIPAuto", "1"),
    ("ExternalDeviceIPNow", ""),
    ("SetSpeedFive", "0"),
    ("KISALongAlt", "0"),
    ("LowUIProfile", "0"),
    ("NavHome", ""),
    ("NavWork", ""),
    ("NavList", ""),
    ("RunCustomCommand", "0"),
    ("CruiseSpammingSpd", "50,80,110"),
    ("CruiseSpammingLevel", "15,10,5,0"),
    ("KisaCruiseGapSet", "4"),
    ("UseLegacyLaneModel", "2"),
    ("DrivingModel", "DrivingModel"),
    ("GitCommitRemote", "0000000000000000000000000000000000000000"),
    ("GitCommitRemoteDate", "00-00"),
    ("LCTimingKeepFactorLeft", "10"),
    ("LCTimingKeepFactorRight", "10"),
    ("LCTimingKeepFactorEnable", "1"),
    ("KISACruiseSpammingInterval", "7"),
    ("KISACruiseSpammingBtnCount", "2"),
  ]
  if not PC:
    default_params.append(("LastUpdateTime", datetime.datetime.now(datetime.UTC).replace(tzinfo=None).isoformat().encode('utf8')))

  if params.get_bool("RecordFrontLock"):
    params.put_bool("RecordFront", True)

  # set unset params
  for k, v in default_params:
    if params.get(k) is None:
      params.put(k, v)

  # Create folders needed for msgq
  try:
    os.mkdir("/dev/shm")
  except FileExistsError:
    pass
  except PermissionError:
    print("WARNING: failed to make /dev/shm")

  # set version params
  params.put("Version", build_metadata.openpilot.version)
  params.put("TermsVersion", terms_version)
  params.put("TrainingVersion", training_version)
  params.put("GitCommit", build_metadata.openpilot.git_commit)
  params.put("GitCommitDate", build_metadata.openpilot.git_commit_date)
  params.put("GitBranch", build_metadata.channel)
  params.put("GitRemote", build_metadata.openpilot.git_origin)
  params.put_bool("IsTestedBranch", build_metadata.tested_channel)
  params.put_bool("IsReleaseBranch", build_metadata.release_channel)

  # set dongle id
  reg_res = register(show_spinner=True)
  if reg_res:
    dongle_id = reg_res
  else:
    serial = params.get("HardwareSerial")
    raise Exception(f"Registration failed for device {serial}")
  os.environ['DONGLE_ID'] = dongle_id  # Needed for swaglog
  os.environ['GIT_ORIGIN'] = build_metadata.openpilot.git_normalized_origin # Needed for swaglog
  os.environ['GIT_BRANCH'] = build_metadata.channel # Needed for swaglog
  os.environ['GIT_COMMIT'] = build_metadata.openpilot.git_commit # Needed for swaglog

  if not build_metadata.openpilot.is_dirty:
    os.environ['CLEAN'] = '1'

  # init logging
  sentry.init(sentry.SentryProject.SELFDRIVE)
  cloudlog.bind_global(dongle_id=dongle_id,
                       version=build_metadata.openpilot.version,
                       origin=build_metadata.openpilot.git_normalized_origin,
                       branch=build_metadata.channel,
                       commit=build_metadata.openpilot.git_commit,
                       dirty=build_metadata.openpilot.is_dirty,
                       device=HARDWARE.get_device_type())

  # kisapilot
  if os.path.isfile('/data/log/error.txt'):
    os.remove('/data/log/error.txt')
  if os.path.isfile('/data/log/can_missing.txt'):
    os.remove('/data/log/can_missing.txt')
  if os.path.isfile('/data/log/can_timeout.txt'):
    os.remove('/data/log/can_timeout.txt')

  # preimport all processes
  for p in managed_processes.values():
    p.prepare()


def manager_cleanup() -> None:
  # send signals to kill all procs
  for p in managed_processes.values():
    p.stop(block=False)

  # ensure all are killed
  for p in managed_processes.values():
    p.stop(block=True)

  cloudlog.info("everything is dead")


def manager_thread() -> None:
  cloudlog.bind(daemon="manager")
  cloudlog.info("manager start")
  cloudlog.info({"environ": os.environ})

  params = Params()

  ignore: list[str] = []
  if params.get("DongleId", encoding='utf8') in (None, UNREGISTERED_DONGLE_ID):
    ignore += ["manage_athenad", "uploader"]
  if os.getenv("NOBOARD") is not None:
    ignore.append("pandad")
  ignore += [x for x in os.getenv("BLOCK", "").split(",") if len(x) > 0]

  sm = messaging.SubMaster(['deviceState', 'carParams'], poll='deviceState')
  pm = messaging.PubMaster(['managerState'])

  write_onroad_params(False, params)
  ensure_running(managed_processes.values(), False, params=params, CP=sm['carParams'], not_run=ignore)

  started_prev = False

  while True:
    sm.update(1000)

    started = sm['deviceState'].started

    if started and not started_prev:
      params.clear_all(ParamKeyType.CLEAR_ON_ONROAD_TRANSITION)
    elif not started and started_prev:
      params.clear_all(ParamKeyType.CLEAR_ON_OFFROAD_TRANSITION)

    # update onroad params, which drives pandad's safety setter thread
    if started != started_prev:
      write_onroad_params(started, params)

    started_prev = started

    ensure_running(managed_processes.values(), started, params=params, CP=sm['carParams'], not_run=ignore)

    running = ' '.join("{}{}\u001b[0m".format("\u001b[32m" if p.proc.is_alive() else "\u001b[31m", p.name)
                       for p in managed_processes.values() if p.proc)
    print(running)
    cloudlog.debug(running)

    # send managerState
    msg = messaging.new_message('managerState', valid=True)
    msg.managerState.processes = [p.get_process_state_msg() for p in managed_processes.values()]
    pm.send('managerState', msg)

    # Exit main loop when uninstall/shutdown/reboot is needed
    shutdown = False
    for param in ("DoUninstall", "DoShutdown", "DoReboot"):
      if params.get_bool(param):
        shutdown = True
        params.put("LastManagerExitReason", f"{param} {datetime.datetime.now()}")
        cloudlog.warning(f"Shutting down manager - {param} set")

    if shutdown:
      break


def main() -> None:
  manager_init()
  if os.getenv("PREPAREONLY") is not None:
    return

  # SystemExit on sigterm
  signal.signal(signal.SIGTERM, lambda signum, frame: sys.exit(1))

  try:
    manager_thread()
  except Exception:
    traceback.print_exc()
    sentry.capture_exception()
  finally:
    manager_cleanup()

  params = Params()
  if params.get_bool("DoUninstall"):
    cloudlog.warning("uninstalling")
    HARDWARE.uninstall()
  elif params.get_bool("DoReboot"):
    cloudlog.warning("reboot")
    HARDWARE.reboot()
  elif params.get_bool("DoShutdown"):
    cloudlog.warning("shutdown")
    HARDWARE.shutdown()


if __name__ == "__main__":
  unblock_stdout()

  try:
    main()
  except KeyboardInterrupt:
    print("got CTRL-C, exiting")
  except Exception:
    add_file_handler(cloudlog)
    cloudlog.exception("Manager failed to start")

    try:
      managed_processes['ui'].stop()
    except Exception:
      pass

    # Show last 3 lines of traceback
    error = traceback.format_exc(-3)
    error = "Manager failed to start\n\n" + error
    with TextWindow(error) as t:
      t.wait_for_exit()

    raise

  # manual exit because we are forked
  sys.exit(0)
