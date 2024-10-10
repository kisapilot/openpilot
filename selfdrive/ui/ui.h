#pragma once

#include <eigen3/Eigen/Dense>
#include <memory>
#include <string>

#include <QTimer>
#include <QColor>
#include <QFuture>

#include "cereal/messaging/messaging.h"
#include "common/mat.h"
#include "common/params.h"
#include "common/util.h"
#include "system/hardware/hw.h"
#include "selfdrive/ui/qt/prime_state.h"

const int UI_BORDER_SIZE = 15;
const int UI_HEADER_HEIGHT = 420;

const int UI_FREQ = 20; // Hz
const int BACKLIGHT_OFFROAD = 50;

const Eigen::Matrix3f VIEW_FROM_DEVICE = (Eigen::Matrix3f() <<
  0.0, 1.0, 0.0,
  0.0, 0.0, 1.0,
  1.0, 0.0, 0.0).finished();

const Eigen::Matrix3f FCAM_INTRINSIC_MATRIX = (Eigen::Matrix3f() <<
  2648.0, 0.0, 1928.0 / 2,
  0.0, 2648.0, 1208.0 / 2,
  0.0, 0.0, 1.0).finished();

// tici ecam focal probably wrong? magnification is not consistent across frame
// Need to retrain model before this can be changed
const Eigen::Matrix3f ECAM_INTRINSIC_MATRIX = (Eigen::Matrix3f() <<
  567.0, 0.0, 1928.0 / 2,
  0.0, 567.0, 1208.0 / 2,
  0.0, 0.0, 1.0).finished();

typedef enum UIStatus {
  STATUS_DISENGAGED,
  STATUS_OVERRIDE,
  STATUS_ENGAGED,
  STATUS_DND,
} UIStatus;

const QColor bg_colors [] = {
  [STATUS_DISENGAGED] = QColor(0x17, 0x33, 0x49, 0xc8),
  [STATUS_OVERRIDE] = QColor(0x91, 0x9b, 0x95, 0x96),
  [STATUS_ENGAGED] = QColor(0x17, 0x86, 0x44, 0x96),
  [STATUS_DND] = QColor(0x32, 0x32, 0x32, 0x96),
};

typedef struct UIScene {
  Eigen::Matrix3f view_from_calib = VIEW_FROM_DEVICE;
  Eigen::Matrix3f view_from_wide_calib = VIEW_FROM_DEVICE;
  cereal::PandaState::PandaType pandaType;

  std::string alertTextMsg1;
  std::string alertTextMsg2;
  std::string alertTextMsg3;
  std::string car_fingerprint;
  float alert_blinking_rate;

  bool brakePress;
  bool gasPress;
  bool autoHold;

  // gps
  int satelliteCount;
  float gpsAccuracy;
  float altitude;
  float bearing;

  int cpuPerc;
  float cpuTemp;
  float ambientTemp;
  int fanSpeedRpm;
  int storageUsage;
  std::string ipAddress;
  bool rightblindspot;
  bool leftblindspot;
  bool leftBlinker;
  bool rightBlinker;
  int blinker_blinkingrate = 0;
  int tpms_blinkingrate = 120;
  int blindspot_blinkingrate = 120;
  int car_valid_status_changed = 0;
  float angleSteers;
  float desired_angle_steers;
  bool gap_by_speed_on;
  bool enabled;
  float steerRatio;
  bool brakeLights;
  bool steerOverride;
  float output_scale;
  int fanSpeed;
  int tpmsUnit;
  float tpmsPressureFl;
  float tpmsPressureFr;
  float tpmsPressureRl;
  float tpmsPressureRr;
  int lateralControlMethod;
  float radarDRel;
  float radarVRel;
  bool standStill;
  int limitSpeedCamera = 0;
  float limitSpeedCameraDist = 0;
  int mapSign;
  int mapSignCam;
  float vSetDis;
  bool cruiseAccStatus;
  bool driverAcc;
  int laneless_mode;
  int recording_count;
  bool monitoring_mode;
  bool forceGearD;
  bool kisa_livetune_ui;
  bool driving_record;
  float steer_actuator_delay;
  int cruise_gap;
  int dynamic_tr_mode;
  float dynamic_tr_value;
  bool touched2 = false;
  int brightness_off;
  int cameraOffset, pathOffset;
  int pidKp, pidKi, pidKd, pidKf;
  int indiInnerLoopGain, indiOuterLoopGain, indiTimeConstant, indiActuatorEffectiveness;
  int lqrScale, lqrKi, lqrDcGain;
  int torqueKp, torqueKf, torqueKi, torqueFriction, torqueMaxLatAccel;
  bool live_tune_panel_enable;
  int bottom_text_view;
  int live_tune_panel_list = 0;
  int list_count = 2;
  int nTime, autoScreenOff, brightness, awake;
  int nVolumeBoost = 0;
  bool read_params_once = false;
  bool nDebugUi1;
  bool nDebugUi2;
  bool nDebugUi3;
  bool nKisaBlindSpotDetect;
  bool auto_gitpull = false;
  bool is_speed_over_limit = false;
  bool controlAllowed;
  bool steer_warning;
  bool show_error;
  int display_maxspeed_time = 0;
  int navi_select;
  bool tmux_error_check = false;
  bool speedlimit_signtype;
  bool sl_decel_off;
  bool pause_spdlimit;
  float a_req_value;
  bool osm_enabled;
  int radar_long_helper;
  float engine_rpm;
  bool cal_view = false;
  float ctrl_speed;
  float accel;
  bool animated_rpm;
  int max_animated_rpm;
  int gear_step;
  float charge_meter;
  float multi_lat_selected;
  int do_not_disturb_mode;
  bool depart_chime_at_resume;
  int comma_stock_ui;
  bool KISA_Debug;
  bool rec_stat = false;
  bool rec_stat2 = false;
  bool rec_stat3 = false;
  int rec_blinker = 0;
  bool rec_blinker_stat = false;
  bool stock_lkas_on_disengagement;
  bool ufc_mode;

  bool op_long_enabled = false;
  bool experimental_mode = false;
  bool exp_mode_temp;
  int btn_pressing;
  bool low_ui_profile;
  bool multi_btn_touched = false;
  float multi_btn_slide_timer = 0;

  bool hotspot_on_boot;
  bool hotspot_trigger = false;

  int user_specific_feature = 0;
  bool use_radar_value;
  bool error_occurred = false;
  int standstillElapsedTime = 0;

  QString model_name;

  cereal::DeviceState::Reader deviceState;
  cereal::PeripheralState::Reader peripheralState;
  cereal::CarState::Reader car_state;
  cereal::ControlsState::Reader controls_state;
  cereal::SelfdriveState::Reader selfdrive_state;
  cereal::CarState::GearShifter getGearShifter;
  cereal::LateralPlan::Reader lateral_plan;
  cereal::LiveENaviData::Reader live_enavi_data;
  cereal::LiveMapData::Reader live_map_data;
  cereal::LongitudinalPlan::Reader longitudinal_plan;


  // atom
  struct _LiveParams
  {
    float angleOffset;
    float angleOffsetAverage;
    float stiffnessFactor;
    float steerRatio;
  } liveParams;

  struct _LateralPlan
  {
    float laneWidth;

    float dProb;
    float lProb;
    float rProb;

    float angleOffset;
    bool lanelessModeStatus;
    float totalCameraOffset;
  } lateralPlan;

  struct _LiveENaviData
  {
    int ekisaspeedlimit;
    float ekisasafetydist;
    int ekisasafetysign;
    int ekisaturninfo;
    float ekisadisttoturn;
    bool ekisaconalive;
    int ekisaroadlimitspeed;
    int ekisalinklength;
    int ekisacurrentlinkangle;
    int ekisanextlinkangle;
    std::string ekisaroadname;
    bool ekisaishighway;
    bool ekisaistunnel;
    std::string ekisa0;
    std::string ekisa1;
    std::string ekisa2;
    std::string ekisa3;
    std::string ekisa4;
    std::string ekisa5;
    std::string ekisa6;
    std::string ekisa7;
    std::string ekisa8;
    std::string ekisa9;
    int ewazealertid;
    int ewazealertdistance;
    int ewazeroadspeedlimit;
    int ewazecurrentspeed;
    std::string ewazeroadname;
    int ewazenavsign;
    int ewazenavdistance;
    std::string ewazealerttype;
    bool ewazealertextend;
  } liveENaviData;

  struct _LiveMapData
  {
    float ospeedLimit;
    float ospeedLimitAhead;
    float ospeedLimitAheadDistance;
    float oturnSpeedLimit;
    float oturnSpeedLimitEndDistance;
    int oturnSpeedLimitSign;
    std::string ocurrentRoadName;
    std::string oref;
    //float turnSpeedLimitsAhead[16]; // List
    //float turnSpeedLimitsAheadDistances[16]; // List
    //int turnSpeedLimitsAheadSigns[16]; // List
  } liveMapData;

  struct _LongitudinalPlan
  {
    float e2ex[13] = {0};
    float lead0[13] = {0};
    float lead1[13] = {0};
    float cruisetg[13] = {0};
  } longitudinalPlan;

  cereal::LongitudinalPersonality personality;

  float light_sensor = -1;
  bool started, ignition, is_metric, longitudinal_control;
  uint64_t started_frame;
} UIScene;

class UIState : public QObject {
  Q_OBJECT

public:
  UIState(QObject* parent = 0);
  void updateStatus();
  inline bool engaged() const {
    return scene.started && (*sm)["selfdriveState"].getSelfdriveState().getEnabled();
  }

  int fb_w = 0, fb_h = 0;
  bool is_OpenpilotViewEnabled = false;

  std::unique_ptr<SubMaster> sm;
  UIStatus status;
  UIScene scene = {};
  QString language;
  PrimeState *prime_state;

signals:
  void uiUpdate(const UIState &s);
  void offroadTransition(bool offroad);
  void hotspotSignal();

private slots:
  void update();

private:
  QTimer *timer;
  bool started_prev = false;
};

UIState *uiState();

// device management class
class Device : public QObject {
  Q_OBJECT

public:
  Device(QObject *parent = 0);
  bool isAwake() { return awake; }
  void setOffroadBrightness(int brightness) {
    offroad_brightness = std::clamp(brightness, 0, 100);
  }

private:
  bool awake = false;
  int interactive_timeout = 0;
  bool ignition_on = false;

  int offroad_brightness = BACKLIGHT_OFFROAD;
  int last_brightness = 0;
  FirstOrderFilter brightness_filter;
  QFuture<void> brightness_future;

  int sleep_time = -1;

  void updateBrightness(const UIState &s);
  void updateWakefulness(const UIState &s);
  void setAwake(bool on);

signals:
  void displayPowerChanged(bool on);
  void interactiveTimeout();

public slots:
  void resetInteractiveTimeout(int timeout = -1);
  void update(const UIState &s);
};

Device *device();
void ui_update_params(UIState *s);
