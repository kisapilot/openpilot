#include "selfdrive/ui/ui.h"

#include <algorithm>
#include <cassert>
#include <cmath>

#include <QtConcurrent>

#include "common/transformations/orientation.hpp"
#include "common/params.h"
#include "common/swaglog.h"
#include "common/util.h"
#include "common/watchdog.h"
#include "system/hardware/hw.h"

#define BACKLIGHT_DT 0.05
#define BACKLIGHT_TS 10.00

// Projects a point in car to space to the corresponding point in full frame
// image space.
static bool calib_frame_to_full_frame(const UIState *s, float in_x, float in_y, float in_z, QPointF *out) {
  const float margin = 500.0f;
  const QRectF clip_region{-margin, -margin, s->fb_w + 2 * margin, s->fb_h + 2 * margin};

  const vec3 pt = (vec3){{in_x, in_y, in_z}};
  const vec3 Ep = matvecmul3(s->scene.wide_cam ? s->scene.view_from_wide_calib : s->scene.view_from_calib, pt);
  const vec3 KEp = matvecmul3(s->scene.wide_cam ? ECAM_INTRINSIC_MATRIX : FCAM_INTRINSIC_MATRIX, Ep);

  // Project.
  QPointF point = s->car_space_transform.map(QPointF{KEp.v[0] / KEp.v[2], KEp.v[1] / KEp.v[2]});
  if (clip_region.contains(point)) {
    *out = point;
    return true;
  }
  return false;
}

int get_path_length_idx(const cereal::XYZTData::Reader &line, const float path_height) {
  const auto line_x = line.getX();
  int max_idx = 0;
  for (int i = 1; i < line_x.size() && line_x[i] <= path_height; ++i) {
    max_idx = i;
  }
  return max_idx;
}

void update_leads(UIState *s, const cereal::RadarState::Reader &radar_state, const cereal::XYZTData::Reader &line) {
  for (int i = 0; i < 2; ++i) {
    auto lead_data = (i == 0) ? radar_state.getLeadOne() : radar_state.getLeadTwo();
    if (lead_data.getStatus()) {
      float z = line.getZ()[get_path_length_idx(line, lead_data.getDRel())];
      calib_frame_to_full_frame(s, lead_data.getDRel(), -lead_data.getYRel(), z + 1.22, &s->scene.lead_vertices[i]);
    }
  }
}

void update_line_data(const UIState *s, const cereal::XYZTData::Reader &line,
                      float y_off, float z_off, QPolygonF *pvd, int max_idx, bool allow_invert=true) {
  const auto line_x = line.getX(), line_y = line.getY(), line_z = line.getZ();
  QPointF left, right;
  pvd->clear();
  for (int i = 0; i <= max_idx; i++) {
    // highly negative x positions  are drawn above the frame and cause flickering, clip to zy plane of camera
    if (line_x[i] < 0) continue;

    bool l = calib_frame_to_full_frame(s, line_x[i], line_y[i] - y_off, line_z[i] + z_off, &left);
    bool r = calib_frame_to_full_frame(s, line_x[i], line_y[i] + y_off, line_z[i] + z_off, &right);
    if (l && r) {
      // For wider lines the drawn polygon will "invert" when going over a hill and cause artifacts
      if (!allow_invert && pvd->size() && left.y() > pvd->back().y()) {
        continue;
      }
      pvd->push_back(left);
      pvd->push_front(right);
    }
  }
}

void update_bsm_data(const UIState *s, int lr, const cereal::XYZTData::Reader &line,
                     float y_off, float z_off, QPolygonF *pvd, int max_idx, bool allow_invert=true) {

  float y_off1, y_off2;
  if (lr == 0) {
    y_off1 = y_off;
    y_off2 = -0.01;
  } else {
    y_off1 = 0.01;
    y_off2 = y_off;  
  }

  const auto line_x = line.getX(), line_y = line.getY(), line_z = line.getZ();
  QPolygonF left_points, right_points;
  left_points.reserve(max_idx + 1);
  right_points.reserve(max_idx + 1);

  for (int i = 0; i <= max_idx; i++) {
    // highly negative x positions  are drawn above the frame and cause flickering, clip to zy plane of camera
    if (line_x[i] < 0) continue;
    QPointF left, right;
    bool l = calib_frame_to_full_frame(s, line_x[i], line_y[i] - y_off1, line_z[i] + z_off, &left);
    bool r = calib_frame_to_full_frame(s, line_x[i], line_y[i] + y_off2, line_z[i] + z_off, &right);
    if (l && r) {
      // For wider lines the drawn polygon will "invert" when going over a hill and cause artifacts
      if (!allow_invert && left_points.size() && left.y() > left_points.back().y()) {
        continue;
      }
      left_points.push_back(left);
      right_points.push_front(right);
    }
  }
  *pvd = left_points + right_points;
}

void update_model(UIState *s,
                  const cereal::ModelDataV2::Reader &model) {
  UIScene &scene = s->scene;
  auto model_position = model.getPosition();
  float max_distance = std::clamp(*(model_position.getX().end() - 1),
                                  MIN_DRAW_DISTANCE, MAX_DRAW_DISTANCE);

  // update lane lines
  const auto lane_lines = model.getLaneLines();
  const auto lane_line_probs = model.getLaneLineProbs();
  int max_idx = get_path_length_idx(lane_lines[0], max_distance);
  for (int i = 0; i < std::size(scene.lane_line_vertices); i++) {
    scene.lane_line_probs[i] = lane_line_probs[i];
    update_line_data(s, lane_lines[i], 0.035 * scene.lane_line_probs[i], 0, &scene.lane_line_vertices[i], max_idx);
  }

  // update road edges
  const auto road_edges = model.getRoadEdges();
  const auto road_edge_stds = model.getRoadEdgeStds();
  for (int i = 0; i < std::size(scene.road_edge_vertices); i++) {
    scene.road_edge_stds[i] = road_edge_stds[i];
    update_line_data(s, road_edges[i], 0.035, 0, &scene.road_edge_vertices[i], max_idx);
  }

  // update bsm alert
  for (int i = 0; i < std::size(scene.bsm_vertices); i++) {
    update_bsm_data(s, i, lane_lines[i+1], 2.8, 0, &scene.bsm_vertices[i], max_idx, false);
  }

  // update path
  auto lead_one = (*s->sm)["radarState"].getRadarState().getLeadOne();
  if (lead_one.getStatus()) {
    const float lead_d = lead_one.getDRel() * 2.;
    max_distance = std::clamp((float)(lead_d - fmin(lead_d * 0.35, 10.)), 0.0f, max_distance);
  }
  max_idx = get_path_length_idx(model_position, max_distance);
  update_line_data(s, model_position, 0.9, 1.22, &scene.track_vertices, max_idx, false);
}

void update_dmonitoring(UIState *s, const cereal::DriverStateV2::Reader &driverstate, float dm_fade_state, bool is_rhd) {
  UIScene &scene = s->scene;
  const auto driver_orient = is_rhd ? driverstate.getRightDriverData().getFaceOrientation() : driverstate.getLeftDriverData().getFaceOrientation();
  const auto driver_data = is_rhd ? driverstate.getRightDriverData() : driverstate.getLeftDriverData();
  scene.dm_prob[0] = driver_data.getLeftBlinkProb();
  scene.dm_prob[1] = driver_data.getLeftEyeProb();
  scene.dm_prob[2] = driver_data.getFaceProb();
  scene.dm_prob[3] = driver_data.getRightEyeProb();
  scene.dm_prob[4] = driver_data.getRightBlinkProb();
  for (int i = 0; i < std::size(scene.driver_pose_vals); i++) {
    float v_this = (i == 0 ? (driver_orient[i] < 0 ? 0.7 : 0.9) : 0.4) * driver_orient[i];
    scene.driver_pose_diff[i] = fabs(scene.driver_pose_vals[i] - v_this);
    scene.driver_pose_vals[i] = 0.8 * v_this + (1 - 0.8) * scene.driver_pose_vals[i];
    scene.driver_pose_sins[i] = sinf(scene.driver_pose_vals[i]*(1.0-dm_fade_state));
    scene.driver_pose_coss[i] = cosf(scene.driver_pose_vals[i]*(1.0-dm_fade_state));
  }

  auto [sin_y, sin_x, sin_z] = scene.driver_pose_sins;
  auto [cos_y, cos_x, cos_z] = scene.driver_pose_coss;

  const mat3 r_xyz = (mat3){{
    cos_x * cos_z,
    cos_x * sin_z,
    -sin_x,

    -sin_y * sin_x * cos_z - cos_y * sin_z,
    -sin_y * sin_x * sin_z + cos_y * cos_z,
    -sin_y * cos_x,

    cos_y * sin_x * cos_z - sin_y * sin_z,
    cos_y * sin_x * sin_z + sin_y * cos_z,
    cos_y * cos_x,
  }};

  // transform vertices
  for (int kpi = 0; kpi < std::size(default_face_kpts_3d); kpi++) {
    vec3 kpt_this = matvecmul3(r_xyz, default_face_kpts_3d[kpi]);
    scene.face_kpts_draw[kpi] = (vec3){{kpt_this.v[0], kpt_this.v[1], (float)(kpt_this.v[2] * (1.0-dm_fade_state) + 8 * dm_fade_state)}};
  }
}

static void update_sockets(UIState *s) {
  s->sm->update(0);
}

static void update_state(UIState *s) {
  SubMaster &sm = *(s->sm);
  UIScene &scene = s->scene;

  if (sm.updated("controlsState")) {
    scene.controls_state = sm["controlsState"].getControlsState();
    scene.lateralControlMethod = scene.controls_state.getLateralControlMethod();
    if (scene.lateralControlMethod == 0) {
      scene.output_scale = scene.controls_state.getLateralControlState().getPidState().getOutput();
    } else if (scene.lateralControlMethod == 1) {
      scene.output_scale = scene.controls_state.getLateralControlState().getIndiState().getOutput();
    } else if (scene.lateralControlMethod == 2) {
      scene.output_scale = scene.controls_state.getLateralControlState().getLqrState().getOutput();
    } else if (scene.lateralControlMethod == 3) {
      scene.output_scale = scene.controls_state.getLateralControlState().getTorqueState().getOutput();
    } else if (scene.lateralControlMethod == 4) {
      scene.output_scale = scene.controls_state.getLateralControlState().getAtomState().getOutput();
      scene.multi_lat_selected = scene.controls_state.getLateralControlState().getAtomState().getSelected();
    }

    scene.alertTextMsg1 = scene.controls_state.getAlertTextMsg1(); //debug1
    scene.alertTextMsg2 = scene.controls_state.getAlertTextMsg2(); //debug2
    scene.alertTextMsg3 = scene.controls_state.getAlertTextMsg3(); //debug3

    scene.limitSpeedCamera = scene.controls_state.getLimitSpeedCamera();
    scene.limitSpeedCameraDist = scene.controls_state.getLimitSpeedCameraDist();
    scene.mapSign = scene.controls_state.getMapSign();
    scene.mapSignCam = scene.controls_state.getMapSignCam();
    scene.steerRatio = scene.controls_state.getSteerRatio();
    scene.dynamic_tr_mode = scene.controls_state.getDynamicTRMode();
    scene.dynamic_tr_value = scene.controls_state.getDynamicTRValue();
    scene.pause_spdlimit = scene.controls_state.getPauseSpdLimit();
    scene.accel = scene.controls_state.getAccel();
    scene.ctrl_speed = scene.controls_state.getSafetySpeed();
    scene.desired_angle_steers = scene.controls_state.getSteeringAngleDesiredDeg();
    scene.gap_by_speed_on = scene.controls_state.getGapBySpeedOn();
    scene.enabled = scene.controls_state.getEnabled();
    scene.experimental_mode = scene.controls_state.getExperimentalMode();
    scene.exp_mode_temp = scene.controls_state.getExpModeTemp();
    scene.btn_pressing = scene.controls_state.getBtnPressing();
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
    scene.standStill = cs_data.getStandStill();
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
    auto live_calib = sm["liveCalibration"].getLiveCalibration();
    auto rpy_list = live_calib.getRpyCalib();
    auto wfde_list = live_calib.getWideFromDeviceEuler();
    Eigen::Vector3d rpy;
    Eigen::Vector3d wfde;
    if (rpy_list.size() == 3) rpy << rpy_list[0], rpy_list[1], rpy_list[2];
    if (wfde_list.size() == 3) wfde << wfde_list[0], wfde_list[1], wfde_list[2];
    Eigen::Matrix3d device_from_calib = euler2rot(rpy);
    Eigen::Matrix3d wide_from_device = euler2rot(wfde);
    Eigen::Matrix3d view_from_device;
    view_from_device << 0, 1, 0,
                        0, 0, 1,
                        1, 0, 0;
    Eigen::Matrix3d view_from_calib = view_from_device * device_from_calib;
    Eigen::Matrix3d view_from_wide_calib = view_from_device * wide_from_device * device_from_calib;
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        scene.view_from_calib.v[i*3 + j] = view_from_calib(i, j);
        scene.view_from_wide_calib.v[i*3 + j] = view_from_wide_calib(i, j);
      }
    }
    scene.calibration_valid = live_calib.getCalStatus() == cereal::LiveCalibrationData::Status::CALIBRATED;
    scene.calibration_wide_valid = wfde_list.size() == 3;
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
  }

  if (!s->is_OpenpilotViewEnabled) {
    scene.started = sm["deviceState"].getDeviceState().getStarted() && scene.ignition;
  } else {
    scene.started = sm["deviceState"].getDeviceState().getStarted();
  }

  scene.world_objects_visible = scene.world_objects_visible ||
                                (scene.started &&
                                 sm.rcv_frame("liveCalibration") > scene.started_frame &&
                                   sm.rcv_frame("modelV2") > scene.started_frame);

  if (sm.updated("lateralPlan")) {
    scene.lateral_plan = sm["lateralPlan"].getLateralPlan();
    auto lp_data = sm["lateralPlan"].getLateralPlan();
    scene.lateralPlan.laneWidth = lp_data.getLaneWidth();
    scene.lateralPlan.dProb = lp_data.getDProb();
    scene.lateralPlan.lProb = lp_data.getLProb();
    scene.lateralPlan.rProb = lp_data.getRProb();
    scene.lateralPlan.standstillElapsedTime = lp_data.getStandstillElapsedTime();
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
  s->scene.map_on_left = params.getBool("NavSettingLeftSide");
}

void UIState::updateStatus() {
  if (scene.started && sm->updated("controlsState")) {
    auto controls_state = (*sm)["controlsState"].getControlsState();
    auto state = controls_state.getState();
    if (state == cereal::ControlsState::OpenpilotState::PRE_ENABLED || state == cereal::ControlsState::OpenpilotState::OVERRIDING) {
      status = STATUS_OVERRIDE;
    } else {
      if (scene.comma_stock_ui == 2) {
        status = controls_state.getEnabled() ? STATUS_DND : STATUS_DISENGAGED;
      } else {
        status = controls_state.getEnabled() ? STATUS_ENGAGED : STATUS_DISENGAGED;
      }
    }
  }

  // Handle onroad/offroad transition
  if (scene.started != started_prev || sm->frame == 1) {
    if (scene.started) {
      status = STATUS_DISENGAGED;
      scene.started_frame = sm->frame;
    }
    started_prev = scene.started;
    scene.world_objects_visible = false;
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
    scene.recording_quality = std::stoi(params.get("RecordingQuality"));
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
  sm = std::make_unique<SubMaster, const std::initializer_list<const char *>>({
    "modelV2", "controlsState", "liveCalibration", "radarState", "deviceState",
    "pandaStates", "carParams", "driverMonitoringState", "carState", "liveLocationKalman", "driverStateV2",
    "wideRoadCameraState", "managerState", "navInstruction", "navRoute", "clocks",
    "peripheralState", "liveParameters", "ubloxGnss", "qcomGnss", "gpsLocationExternal", "gpsLocation",
    "lateralPlan", "longitudinalPlan", "liveENaviData", "liveMapData",
  });

  Params params;
  language = QString::fromStdString(params.get("LanguageSetting"));
  auto prime_value = params.get("PrimeType");
  if (!prime_value.empty()) {
    prime_type = static_cast<PrimeType>(std::atoi(prime_value.c_str()));
  }

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

void UIState::setPrimeType(PrimeType type) {
  if (type != prime_type) {
    bool prev_prime = hasPrime();

    prime_type = type;
    Params().put("PrimeType", std::to_string(prime_type));
    emit primeTypeChanged(prime_type);

    bool prime = hasPrime();
    if (prev_prime != prime) {
      emit primeChanged(prime);
    }
  }
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
  } else if (s.scene.controls_state.getAlertSize() != cereal::ControlsState::AlertSize::NONE && s.scene.autoScreenOff != -3) {
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
  } else if ( s.scene.brightness ) {
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
  } else if (interactive_timeout > 0 && --interactive_timeout == 0) {
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
