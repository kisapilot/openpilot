#pragma once

#include <memory>
#include <string>

#include <QObject>
#include <QTimer>
#include <QColor>
#include <QFuture>
#include <QPolygonF>
#include <QTransform>

#include "cereal/messaging/messaging.h"
#include "common/mat.h"
#include "common/params.h"
#include "common/timing.h"
#include "system/hardware/hw.h"

const int UI_BORDER_SIZE = 15;
const int UI_HEADER_HEIGHT = 420;

const int UI_FREQ = 20; // Hz
const int BACKLIGHT_OFFROAD = 50;

const float MIN_DRAW_DISTANCE = 10.0;
const float MAX_DRAW_DISTANCE = 100.0;
constexpr mat3 DEFAULT_CALIBRATION = {{ 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0 }};
constexpr mat3 FCAM_INTRINSIC_MATRIX = (mat3){{2648.0, 0.0, 1928.0 / 2,
                                           0.0, 2648.0, 1208.0 / 2,
                                           0.0, 0.0, 1.0}};
// tici ecam focal probably wrong? magnification is not consistent across frame
// Need to retrain model before this can be changed
constexpr mat3 ECAM_INTRINSIC_MATRIX = (mat3){{567.0, 0.0, 1928.0 / 2,
                                           0.0, 567.0, 1208.0 / 2,
                                           0.0, 0.0, 1.0}};


constexpr vec3 default_face_kpts_3d[] = {
  {-5.98, -51.20, 8.00}, {-17.64, -49.14, 8.00}, {-23.81, -46.40, 8.00}, {-29.98, -40.91, 8.00}, {-32.04, -37.49, 8.00},
  {-34.10, -32.00, 8.00}, {-36.16, -21.03, 8.00}, {-36.16, 6.40, 8.00}, {-35.47, 10.51, 8.00}, {-32.73, 19.43, 8.00},
  {-29.30, 26.29, 8.00}, {-24.50, 33.83, 8.00}, {-19.01, 41.37, 8.00}, {-14.21, 46.17, 8.00}, {-12.16, 47.54, 8.00},
  {-4.61, 49.60, 8.00}, {4.99, 49.60, 8.00}, {12.53, 47.54, 8.00}, {14.59, 46.17, 8.00}, {19.39, 41.37, 8.00},
  {24.87, 33.83, 8.00}, {29.67, 26.29, 8.00}, {33.10, 19.43, 8.00}, {35.84, 10.51, 8.00}, {36.53, 6.40, 8.00},
  {36.53, -21.03, 8.00}, {34.47, -32.00, 8.00}, {32.42, -37.49, 8.00}, {30.36, -40.91, 8.00}, {24.19, -46.40, 8.00},
  {18.02, -49.14, 8.00}, {6.36, -51.20, 8.00}, {-5.98, -51.20, 8.00},
};


typedef enum UIStatus {
  STATUS_DISENGAGED,
  STATUS_OVERRIDE,
  STATUS_ENGAGED,
  STATUS_DND,
} UIStatus;

enum PrimeType {
  UNKNOWN = -2,
  UNPAIRED = -1,
  NONE = 0,
  MAGENTA = 1,
  LITE = 2,
  BLUE = 3,
  MAGENTA_NEW = 4,
  PURPLE = 5,
};

const QColor bg_colors [] = {
  [STATUS_DISENGAGED] = QColor(0x17, 0x33, 0x49, 0xc8),
  [STATUS_OVERRIDE] = QColor(0x91, 0x9b, 0x95, 0x96),
  [STATUS_ENGAGED] = QColor(0x17, 0x86, 0x44, 0x96),
  [STATUS_DND] = QColor(0x32, 0x32, 0x32, 0x96),
};


typedef struct UIScene {
  bool calibration_valid = false;
  bool calibration_wide_valid  = false;
  bool wide_cam = true;
  mat3 view_from_calib = DEFAULT_CALIBRATION;
  mat3 view_from_wide_calib = DEFAULT_CALIBRATION;
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
  int recording_quality;
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

  QString model_name;

  cereal::DeviceState::Reader deviceState;
  cereal::PeripheralState::Reader peripheralState;
  cereal::CarState::Reader car_state;
  cereal::ControlsState::Reader controls_state;
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
    int standstillElapsedTime = 0;

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


  // modelV2
  float lane_line_probs[4];
  float road_edge_stds[2];
  QPolygonF track_vertices;
  QPolygonF lane_line_vertices[4];
  QPolygonF road_edge_vertices[2];
  QPolygonF bsm_vertices[2];

  // lead
  QPointF lead_vertices[2];

  // DMoji state
  float driver_pose_vals[3];
  float driver_pose_diff[3];
  float driver_pose_sins[3];
  float driver_pose_coss[3];
  vec3 face_kpts_draw[std::size(default_face_kpts_3d)];

  float dm_prob[5];

  cereal::LongitudinalPersonality personality;

  float light_sensor = -1;
  bool started, ignition, is_metric, longitudinal_control;
  bool world_objects_visible = false;
  uint64_t started_frame;
} UIScene;

class UIState : public QObject {
  Q_OBJECT

public:
  UIState(QObject* parent = 0);
  void updateStatus();
  inline bool engaged() const {
    return scene.started && (*sm)["controlsState"].getControlsState().getEnabled();
  }

  void setPrimeType(PrimeType type);
  inline PrimeType primeType() const { return prime_type; }
  inline bool hasPrime() const { return prime_type > PrimeType::NONE; }

  int fb_w = 0, fb_h = 0;

  std::unique_ptr<SubMaster> sm;

  UIStatus status;
  UIScene scene = {};

  QString language;

  bool is_OpenpilotViewEnabled = false;

  QTransform car_space_transform;

signals:
  void uiUpdate(const UIState &s);
  void offroadTransition(bool offroad);
  void primeChanged(bool prime);
  void primeTypeChanged(PrimeType prime_type);
  void hotspotSignal();

private slots:
  void update();

private:
  QTimer *timer;
  bool started_prev = false;
  PrimeType prime_type = PrimeType::UNKNOWN;
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
int get_path_length_idx(const cereal::XYZTData::Reader &line, const float path_height);
void update_model(UIState *s,
                  const cereal::ModelDataV2::Reader &model);
void update_dmonitoring(UIState *s, const cereal::DriverStateV2::Reader &driverstate, float dm_fade_state, bool is_rhd);
void update_leads(UIState *s, const cereal::RadarState::Reader &radar_state, const cereal::XYZTData::Reader &line);
void update_line_data(const UIState *s, const cereal::XYZTData::Reader &line,
                      float y_off, float z_off, QPolygonF *pvd, int max_idx, bool allow_invert);
