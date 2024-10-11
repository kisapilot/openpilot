#include "selfdrive/ui/ui.h"

#include <algorithm>
#include <cmath>

#include <QtConcurrent>

#include "common/transformations/orientation.hpp"
#include "common/swaglog.h"
#include "common/util.h"
#include "common/watchdog.h"
#include "system/hardware/hw.h"

#define BACKLIGHT_DT 0.05
#define BACKLIGHT_TS 10.00

static void update_sockets(UIState *s) {
  s->sm->update(0);
}

static void update_state(UIState *s) {
  SubMaster &sm = *(s->sm);
  UIScene &scene = s->scene;

  if (sm.updated("controlsState")) {
    scene.controls_state = sm["controlsState"].getControlsState();
    auto cons_data = sm["controlsState"].getControlsState();
    scene.lateralControlMethod = scene.controls_state.getLateralControlMethod();
    if (scene.lateralControlMethod == 0) {
      scene.output_scale = cons_data.getLateralControlState().getPidState().getOutput();
    } else if (scene.lateralControlMethod == 1) {
      scene.output_scale = cons_data.getLateralControlState().getIndiState().getOutput();
    } else if (scene.lateralControlMethod == 2) {
      scene.output_scale = cons_data.getLateralControlState().getLqrState().getOutput();
    } else if (scene.lateralControlMethod == 3) {
      scene.output_scale = cons_data.getLateralControlState().getTorqueState().getOutput();
    } else if (scene.lateralControlMethod == 4) {
      scene.output_scale = cons_data.getLateralControlState().getAtomState().getOutput();
      scene.multi_lat_selected = cons_data.getLateralControlState().getAtomState().getSelected();
    }

    scene.alertTextMsg1 = cons_data.getAlertTextMsg1(); //debug1
    scene.alertTextMsg2 = cons_data.getAlertTextMsg2(); //debug2
    scene.alertTextMsg3 = cons_data.getAlertTextMsg3(); //debug3

    scene.limitSpeedCamera = cons_data.getLimitSpeedCamera();
    scene.limitSpeedCameraDist = cons_data.getLimitSpeedCameraDist();
    scene.mapSign = cons_data.getMapSign();
    scene.mapSignCam = cons_data.getMapSignCam();
    scene.steerRatio = cons_data.getSteerRatio();
    scene.dynamic_tr_mode = cons_data.getDynamicTRMode();
    scene.dynamic_tr_value = cons_data.getDynamicTRValue();
    scene.accel = cons_data.getAccel();
    scene.ctrl_speed = cons_data.getSafetySpeed();
    scene.desired_angle_steers = cons_data.getSteeringAngleDesiredDeg();
    scene.gap_by_speed_on = cons_data.getGapBySpeedOn();
    scene.exp_mode_temp = cons_data.getExpModeTemp();
    scene.btn_pressing = cons_data.getBtnPressing();
    scene.standStill = cons_data.getStandStill();
    scene.standstillElapsedTime = cons_data.getStandStillTimer();
  }
  if (sm.updated("carState")) {
    scene.car_state = sm["carState"].getCarState();
    auto cs_data = sm["carState"].getCarState();
    auto cruiseState = scene.car_state.getCruiseState();
    scene.awake = cruiseState.getCruiseSwState();

    if (scene.leftBlinker!=cs_data.getLeftBlinker() || scene.rightBlinker!=cs_data.getRightBlinker()) {
      scene.blinker_blinkingrate = 120;
    }
    scene.brakePress = cs_data.getBrakePressed();
    scene.gasPress = cs_data.getGasPressed();
    scene.brakeLights = cs_data.getBrakeLights();
    scene.getGearShifter = cs_data.getGearShifter();
    scene.leftBlinker = cs_data.getLeftBlinker();
    scene.rightBlinker = cs_data.getRightBlinker();
    scene.leftblindspot = cs_data.getLeftBlindspot();
    scene.rightblindspot = cs_data.getRightBlindspot();
    scene.tpmsUnit = cs_data.getTpms().getUnit();
    scene.tpmsPressureFl = cs_data.getTpms().getFl();
    scene.tpmsPressureFr = cs_data.getTpms().getFr();
    scene.tpmsPressureRl = cs_data.getTpms().getRl();
    scene.tpmsPressureRr = cs_data.getTpms().getRr();
    scene.radarDRel = cs_data.getRadarDRel();
    scene.radarVRel = cs_data.getRadarVRel();
    scene.vSetDis = cs_data.getVSetDis();
    scene.cruiseAccStatus = cs_data.getCruiseAccStatus();
    scene.driverAcc = cs_data.getDriverAcc();
    scene.angleSteers = cs_data.getSteeringAngleDeg();
    scene.cruise_gap = cs_data.getCruiseGapSet();
    scene.autoHold = cs_data.getAutoHold();
    scene.steer_warning = cs_data.getSteerFaultTemporary();
    scene.a_req_value = cs_data.getAReqValue();
    scene.engine_rpm = cs_data.getEngineRpm();
    scene.gear_step = cs_data.getGearStep();
    scene.charge_meter = cs_data.getChargeMeter();
    scene.pause_spdlimit = cs_data.getPauseSpdLimit();
  }

  if (sm.updated("liveParameters")) {
    //scene.liveParams = sm["liveParameters"].getLiveParameters();
    auto live_data = sm["liveParameters"].getLiveParameters();
    scene.liveParams.angleOffset = live_data.getAngleOffsetDeg();
    scene.liveParams.angleOffsetAverage = live_data.getAngleOffsetAverageDeg();
    scene.liveParams.stiffnessFactor = live_data.getStiffnessFactor();
    scene.liveParams.steerRatio = live_data.getSteerRatio();
  }

  if (sm.updated("liveCalibration")) {
    auto list2rot = [](const capnp::List<float>::Reader &rpy_list) ->Eigen::Matrix3f {
      return euler2rot({rpy_list[0], rpy_list[1], rpy_list[2]}).cast<float>();
    };

    auto live_calib = sm["liveCalibration"].getLiveCalibration();
    if (live_calib.getCalStatus() == cereal::LiveCalibrationData::Status::CALIBRATED) {
      auto device_from_calib = list2rot(live_calib.getRpyCalib());
      auto wide_from_device = list2rot(live_calib.getWideFromDeviceEuler());
      s->scene.view_from_calib = VIEW_FROM_DEVICE * device_from_calib;
      s->scene.view_from_wide_calib = VIEW_FROM_DEVICE * wide_from_device * device_from_calib;
    } else {
      s->scene.view_from_calib = s->scene.view_from_wide_calib = VIEW_FROM_DEVICE;
    }
  }

  if (sm.updated("deviceState")) {
    scene.deviceState = sm["deviceState"].getDeviceState();
    scene.cpuPerc = scene.deviceState.getCpuUsagePercent()[0];
    scene.cpuTemp = scene.deviceState.getCpuTempC()[0];
    scene.ambientTemp = scene.deviceState.getAmbientTempC();
    scene.fanSpeed = scene.deviceState.getFanSpeedPercentDesired();
    scene.storageUsage = int(round(100. - scene.deviceState.getFreeSpacePercent()));
    scene.ipAddress = scene.deviceState.getIpAddress();
  }
  if (sm.updated("peripheralState")) {
    scene.peripheralState = sm["peripheralState"].getPeripheralState();
    scene.fanSpeedRpm = scene.peripheralState.getFanSpeedRpm();
  }
  if (sm.updated("pandaStates")) {
    auto pandaStates = sm["pandaStates"].getPandaStates();
    if (pandaStates.size() > 0) {
      scene.pandaType = pandaStates[0].getPandaType();

      if (scene.pandaType != cereal::PandaState::PandaType::UNKNOWN) {
        scene.ignition = false;
        for (const auto& pandaState : pandaStates) {
          scene.ignition |= pandaState.getIgnitionLine() || pandaState.getIgnitionCan();
          scene.controlAllowed = pandaState.getControlsAllowed();
        }
      }
    }
  } else if ((s->sm->frame - s->sm->rcv_frame("pandaStates")) > 5*UI_FREQ) {
    scene.pandaType = cereal::PandaState::PandaType::UNKNOWN;
  }
  if (scene.pandaType == cereal::PandaState::PandaType::TRES) {
    if (sm.updated("gpsLocation")) {
      auto ge_data = sm["gpsLocation"].getGpsLocation();
      scene.gpsAccuracy = ge_data.getVerticalAccuracy();
      scene.altitude = ge_data.getAltitude();
      scene.bearing = ge_data.getBearingDeg();
    }    
  } else {
    if (sm.updated("ubloxGnss")) {
      auto ub_data = sm["ubloxGnss"].getUbloxGnss();
      if (ub_data.which() == cereal::UbloxGnss::MEASUREMENT_REPORT) {
        scene.satelliteCount = ub_data.getMeasurementReport().getNumMeas();
      }
    }
    if (sm.updated("gpsLocationExternal")) {
      auto ge_data = sm["gpsLocationExternal"].getGpsLocationExternal();
      scene.gpsAccuracy = ge_data.getHorizontalAccuracy();
      scene.altitude = ge_data.getAltitude();
      scene.bearing = ge_data.getBearingDeg();
    }
  }
  if (sm.updated("carParams")) {
    auto cp_data = sm["carParams"].getCarParams();
    scene.longitudinal_control = cp_data.getOpenpilotLongitudinalControl();
    scene.steer_actuator_delay = cp_data.getSteerActuatorDelay();
    scene.car_fingerprint = cp_data.getCarFingerprint();
  }
  if (sm.updated("wideRoadCameraState")) {
    auto cam_state = sm["wideRoadCameraState"].getWideRoadCameraState();
    float scale = (cam_state.getSensor() == cereal::FrameData::ImageSensor::AR0231) ? 6.0f : 1.0f;
    scene.light_sensor = std::max(100.0f - scale * cam_state.getExposureValPercent(), 0.0f);
  } else if (!sm.allAliveAndValid({"wideRoadCameraState"})) {
    scene.light_sensor = -1;
  }

  if (s->sm->frame % (8*UI_FREQ) == 0) {
    s->is_OpenpilotViewEnabled = Params().getBool("IsOpenpilotViewEnabled");
    scene.error_occurred = Params().getBool("ErrorOccurred");
  }

  if (!s->is_OpenpilotViewEnabled) {
    scene.started = sm["deviceState"].getDeviceState().getStarted() && scene.ignition;
  } else {
    scene.started = sm["deviceState"].getDeviceState().getStarted();
  }

  if (sm.updated("lateralPlan")) {
    scene.lateral_plan = sm["lateralPlan"].getLateralPlan();
    auto lp_data = sm["lateralPlan"].getLateralPlan();
    scene.lateralPlan.laneWidth = lp_data.getLaneWidth();
    scene.lateralPlan.dProb = lp_data.getDProb();
    scene.lateralPlan.lProb = lp_data.getLProb();
    scene.lateralPlan.rProb = lp_data.getRProb();
    scene.lateralPlan.lanelessModeStatus = lp_data.getLanelessMode();
    scene.lateralPlan.totalCameraOffset = lp_data.getTotalCameraOffset();
  }
  if (sm.updated("longitudinalPlan")) {
    scene.longitudinal_plan = sm["longitudinalPlan"].getLongitudinalPlan();
    auto lop_data = sm["longitudinalPlan"].getLongitudinalPlan();
    for (int i = 0; i < std::size(scene.longitudinalPlan.e2ex); i++) {
      scene.longitudinalPlan.e2ex[i] = lop_data.getE2eX()[i];
    }
    for (int i = 0; i < std::size(scene.longitudinalPlan.lead0); i++) {
      scene.longitudinalPlan.lead0[i] = lop_data.getLead0Obstacle()[i];
    }
    for (int i = 0; i < std::size(scene.longitudinalPlan.lead1); i++) {
      scene.longitudinalPlan.lead1[i] = lop_data.getLead1Obstacle()[i];
    }
    for (int i = 0; i < std::size(scene.longitudinalPlan.cruisetg); i++) {
      scene.longitudinalPlan.cruisetg[i] = lop_data.getCruiseTarget()[i];
    }
  }
  // kisapilot
  if (sm.updated("liveENaviData")) {
    scene.live_enavi_data = sm["liveENaviData"].getLiveENaviData();
    auto lme_data = sm["liveENaviData"].getLiveENaviData();
    scene.liveENaviData.ekisaspeedlimit = lme_data.getSpeedLimit();
    scene.liveENaviData.ekisasafetydist = lme_data.getSafetyDistance();
    scene.liveENaviData.ekisasafetysign = lme_data.getSafetySign();
    scene.liveENaviData.ekisaturninfo = lme_data.getTurnInfo();
    scene.liveENaviData.ekisadisttoturn = lme_data.getDistanceToTurn();
    scene.liveENaviData.ekisaconalive = lme_data.getConnectionAlive();
    scene.liveENaviData.ekisaroadlimitspeed = lme_data.getRoadLimitSpeed();
    scene.liveENaviData.ekisalinklength = lme_data.getLinkLength();
    scene.liveENaviData.ekisacurrentlinkangle = lme_data.getCurrentLinkAngle();
    scene.liveENaviData.ekisanextlinkangle = lme_data.getNextLinkAngle();
    scene.liveENaviData.ekisaroadname = lme_data.getRoadName();
    scene.liveENaviData.ekisaishighway = lme_data.getIsHighway();
    scene.liveENaviData.ekisaistunnel = lme_data.getIsTunnel();
    if (scene.KISA_Debug) {
      scene.liveENaviData.ekisa0 = lme_data.getKisa0();
      scene.liveENaviData.ekisa1 = lme_data.getKisa1();
      scene.liveENaviData.ekisa2 = lme_data.getKisa2();
      scene.liveENaviData.ekisa3 = lme_data.getKisa3();
      scene.liveENaviData.ekisa4 = lme_data.getKisa4();
      scene.liveENaviData.ekisa5 = lme_data.getKisa5();
      scene.liveENaviData.ekisa6 = lme_data.getKisa6();
      scene.liveENaviData.ekisa7 = lme_data.getKisa7();
      scene.liveENaviData.ekisa8 = lme_data.getKisa8();
      scene.liveENaviData.ekisa9 = lme_data.getKisa9();
    }
    if (scene.navi_select == 2) {
      scene.liveENaviData.ewazealertid = lme_data.getWazeAlertId();
      scene.liveENaviData.ewazealertdistance = lme_data.getWazeAlertDistance();
      scene.liveENaviData.ewazeroadspeedlimit = lme_data.getWazeRoadSpeedLimit();
      scene.liveENaviData.ewazecurrentspeed = lme_data.getWazeCurrentSpeed();
      scene.liveENaviData.ewazeroadname = lme_data.getWazeRoadName();
      scene.liveENaviData.ewazenavsign = lme_data.getWazeNavSign();
      scene.liveENaviData.ewazenavdistance = lme_data.getWazeNavDistance();
      scene.liveENaviData.ewazealerttype = lme_data.getWazeAlertType();
      scene.liveENaviData.ewazealertextend = lme_data.getWazeAlertExtend();
    }
  }
  if (sm.updated("liveMapData")) {
    scene.live_map_data = sm["liveMapData"].getLiveMapData();
    auto lmap_data = sm["liveMapData"].getLiveMapData();
    scene.liveMapData.ospeedLimit = lmap_data.getSpeedLimit();
    scene.liveMapData.ospeedLimitAhead = lmap_data.getSpeedLimitAhead();
    scene.liveMapData.ospeedLimitAheadDistance = lmap_data.getSpeedLimitAheadDistance();
    scene.liveMapData.oturnSpeedLimit = lmap_data.getTurnSpeedLimit();
    scene.liveMapData.oturnSpeedLimitEndDistance = lmap_data.getTurnSpeedLimitEndDistance();
    scene.liveMapData.oturnSpeedLimitSign = lmap_data.getTurnSpeedLimitSign();
    scene.liveMapData.ocurrentRoadName = lmap_data.getCurrentRoadName();
    scene.liveMapData.oref = lmap_data.getRef();
  }
}

void ui_update_params(UIState *s) {
  auto params = Params();
  s->scene.is_metric = params.getBool("IsMetric");
}

void UIState::updateStatus() {
  if (scene.started && sm->updated("selfdriveState")) {
    auto ss = (*sm)["selfdriveState"].getSelfdriveState();
    auto state = ss.getState();
    if (state == cereal::SelfdriveState::OpenpilotState::PRE_ENABLED || state == cereal::SelfdriveState::OpenpilotState::OVERRIDING) {
      status = STATUS_OVERRIDE;
    } else {
      if (scene.comma_stock_ui == 2) {
        status = ss.getEnabled() ? STATUS_DND : STATUS_DISENGAGED;
      } else {
        status = ss.getEnabled() ? STATUS_ENGAGED : STATUS_DISENGAGED;
      }
    }

    scene.selfdrive_state = ss;
    scene.enabled = ss.getEnabled();
    scene.experimental_mode = ss.getExperimentalMode();
  }

  // Handle onroad/offroad transition
  if (scene.started != started_prev || sm->frame == 1) {
    if (scene.started) {
      status = STATUS_DISENGAGED;
      scene.started_frame = sm->frame;
    }
    started_prev = scene.started;
    emit offroadTransition(!scene.started);
  }

  if(scene.hotspot_on_boot) {
    if(scene.ipAddress.length() > 1 && !scene.hotspot_trigger) {
      scene.hotspot_trigger = true;
      emit hotspotSignal();
    }
  }

  // this is useful to save compiling time before depart when you use remote ignition
  if (!scene.auto_gitpull && (sm->frame - scene.started_frame > 30*UI_FREQ)) {
    if (Params().getBool("GitPullOnBoot")) {
      scene.auto_gitpull = true;
      Params().put("RunCustomCommand", "2", 1);
    } else if (sm->frame - scene.started_frame > 300*UI_FREQ) {
      scene.auto_gitpull = true;
      Params().put("RunCustomCommand", "1", 1);
    }
  }

  if (!scene.read_params_once) {
    Params params;
    // user param value init
    scene.driving_record = params.getBool("KisaDrivingRecord");
    scene.nDebugUi1 = params.getBool("DebugUi1");
    scene.nDebugUi2 = params.getBool("DebugUi2");
    scene.nDebugUi3 = params.getBool("DebugUi3");
    scene.forceGearD = params.getBool("JustDoGearD");
    scene.nKisaBlindSpotDetect = params.getBool("KisaBlindSpotDetect");
    scene.laneless_mode = std::stoi(params.get("LanelessMode"));
    scene.recording_count = std::stoi(params.get("RecordingCount"));
    scene.monitoring_mode = params.getBool("KisaMonitoringMode");
    scene.brightness = std::stoi(params.get("KisaUIBrightness"));
    scene.nVolumeBoost = std::stoi(params.get("KisaUIVolumeBoost"));
    scene.autoScreenOff = std::stoi(params.get("KisaAutoScreenOff"));
    scene.brightness_off = std::stoi(params.get("KisaUIBrightnessOff"));
    scene.cameraOffset = std::stoi(params.get("CameraOffsetAdj"));
    scene.pathOffset = std::stoi(params.get("PathOffsetAdj"));
    scene.pidKp = std::stoi(params.get("PidKp"));
    scene.pidKi = std::stoi(params.get("PidKi"));
    scene.pidKd = std::stoi(params.get("PidKd"));
    scene.pidKf = std::stoi(params.get("PidKf"));
    scene.torqueKp = std::stoi(params.get("TorqueKp"));
    scene.torqueKf = std::stoi(params.get("TorqueKf"));
    scene.torqueKi = std::stoi(params.get("TorqueKi"));
    scene.torqueFriction = std::stoi(params.get("TorqueFriction"));
    scene.torqueMaxLatAccel = std::stoi(params.get("TorqueMaxLatAccel"));
    scene.indiInnerLoopGain = std::stoi(params.get("InnerLoopGain"));
    scene.indiOuterLoopGain = std::stoi(params.get("OuterLoopGain"));
    scene.indiTimeConstant = std::stoi(params.get("TimeConstant"));
    scene.indiActuatorEffectiveness = std::stoi(params.get("ActuatorEffectiveness"));
    scene.lqrScale = std::stoi(params.get("Scale"));
    scene.lqrKi = std::stoi(params.get("LqrKi"));
    scene.lqrDcGain = std::stoi(params.get("DcGain"));
    scene.navi_select = std::stoi(params.get("KISANaviSelect"));
    scene.radar_long_helper = std::stoi(params.get("RadarLongHelper"));
    scene.live_tune_panel_enable = params.getBool("KisaLiveTunePanelEnable");
    scene.bottom_text_view = std::stoi(params.get("BottomTextView"));
    scene.max_animated_rpm = std::stoi(params.get("AnimatedRPMMax"));
    scene.show_error = params.getBool("ShowError");
    scene.speedlimit_signtype = params.getBool("KisaSpeedLimitSignType");
    scene.sl_decel_off = params.getBool("SpeedLimitDecelOff");
    scene.osm_enabled = params.getBool("OSMEnable") || params.getBool("OSMSpeedLimitEnable") || std::stoi(params.get("CurvDecelOption")) == 1 || std::stoi(params.get("CurvDecelOption")) == 3;
    scene.animated_rpm = params.getBool("AnimatedRPM");
    scene.lateralControlMethod = std::stoi(params.get("LateralControlMethod"));
    scene.do_not_disturb_mode = std::stoi(params.get("DoNotDisturbMode"));
    scene.depart_chime_at_resume = params.getBool("DepartChimeAtResume");
    scene.KISA_Debug = params.getBool("KISADebug");
    scene.low_ui_profile = params.getBool("LowUIProfile");
    scene.stock_lkas_on_disengagement = params.getBool("StockLKASEnabled");
    scene.ufc_mode = params.getBool("UFCModeEnabled");
    scene.op_long_enabled = params.getBool("ExperimentalLongitudinalEnabled");
    scene.model_name = QString::fromStdString(params.get("DrivingModel"));
    scene.hotspot_on_boot = params.getBool("KisaHotspotOnBoot");
    scene.user_specific_feature = std::stoi(params.get("UserSpecificFeature"));
    scene.use_radar_value = params.getBool("UseRadarValue");

    if (scene.autoScreenOff > 0) {
      scene.nTime = scene.autoScreenOff * 60 * UI_FREQ;
    } else if (scene.autoScreenOff == 0) {
      scene.nTime = 30 * UI_FREQ;
    } else if (scene.autoScreenOff == -1) {
      scene.nTime = 15 * UI_FREQ;
    } else if (scene.autoScreenOff == -2) {
      scene.nTime = 5 * UI_FREQ;
    } else {
      scene.nTime = -1;
    }
    scene.comma_stock_ui = std::stoi(params.get("CommaStockUI"));
    scene.kisa_livetune_ui = params.getBool("KisaLiveTunePanelEnable");
    std::system("sudo rm /data/kisa_starting");
    scene.read_params_once = true;
  }
}

UIState::UIState(QObject *parent) : QObject(parent) {
  sm = std::make_unique<SubMaster>(std::vector<const char*>{
    "modelV2", "controlsState", "liveCalibration", "radarState", "deviceState",
    "pandaStates", "carParams", "driverMonitoringState", "carState", "driverStateV2",
    "wideRoadCameraState", "managerState", "selfdriveState",
    "peripheralState", "liveParameters", "ubloxGnss", "qcomGnss", "gpsLocationExternal", "gpsLocation",
    "lateralPlan", "longitudinalPlan", "liveENaviData", "liveMapData",
  });
  prime_state = new PrimeState(this);
  language = QString::fromStdString(Params().get("LanguageSetting"));

  // update timer
  timer = new QTimer(this);
  QObject::connect(timer, &QTimer::timeout, this, &UIState::update);
  timer->start(1000 / UI_FREQ);
}

void UIState::update() {
  update_sockets(this);
  update_state(this);
  updateStatus();

  if (sm->frame % UI_FREQ == 0) {
    watchdog_kick(nanos_since_boot());
  }
  emit uiUpdate(*this);
}

Device::Device(QObject *parent) : brightness_filter(BACKLIGHT_OFFROAD, BACKLIGHT_TS, BACKLIGHT_DT), QObject(parent) {
  setAwake(true);
  resetInteractiveTimeout();

  QObject::connect(uiState(), &UIState::uiUpdate, this, &Device::update);
}

void Device::update(const UIState &s) {
  updateBrightness(s);
  updateWakefulness(s);
}

void Device::setAwake(bool on) {
  if (on != awake) {
    awake = on;
    Hardware::set_display_power(awake);
    LOGD("setting display power %d", awake);
    emit displayPowerChanged(awake);
  }
}

void Device::resetInteractiveTimeout(int timeout) {
  if (timeout == -1) {
    timeout = (ignition_on ? 10 : 30);
  }
  interactive_timeout = timeout * UI_FREQ;
}

void Device::updateBrightness(const UIState &s) {
  float clipped_brightness = offroad_brightness;
  if (s.scene.started && s.scene.light_sensor > 0) {
    clipped_brightness = s.scene.light_sensor;

    // CIE 1931 - https://www.photonstophotos.net/GeneralTopics/Exposure/Psychometric_Lightness_and_Gamma.htm
    if (clipped_brightness <= 8) {
      clipped_brightness = (clipped_brightness / 903.3);
    } else {
      clipped_brightness = std::pow((clipped_brightness + 16.0) / 116.0, 3.0);
    }

    // Scale back to 10% to 100%
    clipped_brightness = std::clamp(100.0f * clipped_brightness, 10.0f, 100.0f);
  }

  if (s.scene.comma_stock_ui == 2 && (s.scene.do_not_disturb_mode == 1 || s.scene.do_not_disturb_mode == 3)) {
    if (s.scene.touched2) {
      sleep_time = 10 * UI_FREQ;
    } else if (sleep_time > 0) {
      sleep_time--;
    } else if (s.scene.started && sleep_time == -1) {
      sleep_time = 10 * UI_FREQ;
    }
  } else if (s.scene.autoScreenOff != -3 && s.scene.touched2) {
    sleep_time = s.scene.nTime;
  } else if (s.scene.selfdrive_state.getAlertSize() != cereal::SelfdriveState::AlertSize::NONE && s.scene.autoScreenOff != -3) {
    sleep_time = s.scene.nTime;
  } else if (sleep_time > 0 && s.scene.autoScreenOff != -3) {
    sleep_time--;
  } else if (s.scene.started && sleep_time == -1 && s.scene.autoScreenOff != -3) {
    sleep_time = s.scene.nTime;
  }

  int brightness = brightness_filter.update(clipped_brightness);
  if (!awake) {
    brightness = 0;
  } else if ((s.scene.enabled && s.scene.comma_stock_ui == 2 && (s.scene.do_not_disturb_mode == 1 || s.scene.do_not_disturb_mode == 3)) && s.scene.started && sleep_time == 0) {
    brightness = 0;
  } else if (s.scene.started && sleep_time == 0 && s.scene.autoScreenOff != -3) {
    if (s.scene.brightness_off < 4) {
      brightness = 0;
    } else if (s.scene.brightness_off < 9) {
      brightness = 1;
    } else {
      brightness = s.scene.brightness_off * 0.01 * brightness;
    }
  } else if (s.scene.brightness) {
    brightness = s.scene.brightness;
  }

  if (brightness != last_brightness) {
    if (!brightness_future.isRunning()) {
      brightness_future = QtConcurrent::run(Hardware::set_brightness, brightness);
      last_brightness = brightness;
    }
  }
}

void Device::updateWakefulness(const UIState &s) {
  bool ignition_just_turned_off = !s.scene.ignition && ignition_on;
  ignition_on = s.scene.ignition;

  if (ignition_just_turned_off) {
    resetInteractiveTimeout();
  } else if (interactive_timeout > 0 && --interactive_timeout == 0 && !s.scene.error_occurred) {
    emit interactiveTimeout();
  }

  setAwake(s.scene.ignition || interactive_timeout > 0);
}

UIState *uiState() {
  static UIState ui_state;
  return &ui_state;
}

Device *device() {
  static Device _device;
  return &_device;
}
