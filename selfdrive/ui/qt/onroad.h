#pragma once

#include <memory>

#include <QPushButton>
#include <QStackedLayout>
#include <QWidget>

#include "common/util.h"
#include "selfdrive/ui/ui.h"
#include "selfdrive/ui/qt/widgets/cameraview.h"


const int btn_size = 192;
const int img_size = (btn_size / 4) * 3; // 144

#include <QTimer>
#include "selfdrive/ui/qt/screenrecorder/screenrecorder.h"


// ***** onroad widgets *****
class OnroadAlerts : public QWidget {
  Q_OBJECT

public:
  OnroadAlerts(QWidget *parent = 0) : QWidget(parent) {}
  void updateState(const UIState &s);
  void clear();

protected:
  struct Alert {
    QString text1;
    QString text2;
    QString type;
    cereal::ControlsState::AlertSize size;
    cereal::ControlsState::AlertStatus status;

    bool equal(const Alert &other) const {
      return text1 == other.text1 && other.text2 == other.text2 && type == other.type;
    }
  };

  const QMap<cereal::ControlsState::AlertStatus, QColor> alert_colors = {
    {cereal::ControlsState::AlertStatus::NORMAL, QColor(0x15, 0x15, 0x15, 0xf1)},
    {cereal::ControlsState::AlertStatus::USER_PROMPT, QColor(0xDA, 0x6F, 0x25, 0xf1)},
    {cereal::ControlsState::AlertStatus::CRITICAL, QColor(0xC9, 0x22, 0x31, 0xf1)},
  };

  void paintEvent(QPaintEvent*) override;
  OnroadAlerts::Alert getAlert(const SubMaster &sm, uint64_t started_frame);

  QColor bg;
  Alert alert = {};
};

class ExperimentalButton : public QPushButton {
  Q_OBJECT

public:
  explicit ExperimentalButton(QWidget *parent = 0);
  void updateState(const UIState &s);

protected:
  inline QColor redColor(int alpha = 255) { return QColor(201, 34, 49, alpha); }
  inline QColor blackColor(int alpha = 255) { return QColor(0, 0, 0, alpha); }
  inline QColor whiteColor(int alpha = 255) { return QColor(255, 255, 255, alpha); }
  inline QColor yellowColor(int alpha = 255) { return QColor(218, 202, 37, alpha); }
  inline QColor ochreColor(int alpha = 255) { return QColor(218, 111, 37, alpha); }
  inline QColor greenColor(int alpha = 255) { return QColor(0, 255, 0, alpha); }
  inline QColor blueColor(int alpha = 255) { return QColor(0, 0, 255, alpha); }
  inline QColor orangeColor(int alpha = 255) { return QColor(255, 175, 3, alpha); }
  inline QColor greyColor(int alpha = 1) { return QColor(191, 191, 191, alpha); }

private:
  void paintEvent(QPaintEvent *event) override;
  void changeMode();

  Params params;
  QPixmap engage_img;
  QPixmap experimental_img;
  bool experimental_mode;
  bool engageable;
};


class MapSettingsButton : public QPushButton {
  Q_OBJECT

public:
  explicit MapSettingsButton(QWidget *parent = 0);
  void updateState(const UIState &s);

private:
  void paintEvent(QPaintEvent *event) override;

  QPixmap settings_img;
  QPixmap settings_img_g;
  bool navi_is_alive = false;
};

// container window for the NVG UI
class AnnotatedCameraWidget : public CameraWidget {
  Q_OBJECT

public:
  explicit AnnotatedCameraWidget(VisionStreamType type, QWidget* parent = 0);
  void updateState(const UIState &s);

  MapSettingsButton *map_settings_btn;

private:
  void drawIcon(QPainter &p, int x, int y, QPixmap &img, QBrush bg = QColor(0,0,0,0), float opacity = 1.0, bool rotation = false, float angle = 0);
  void drawText(QPainter &p, int x, int y, const QString &text, int alpha = 255);
  void uiText(QPainter &p, int x, int y, const QString &text, int alpha = 255, bool custom_color = false);
  void debugText(QPainter &p, int x, int y, const QString &text, int alpha = 255, int fontsize = 30, bool bold = false);


  QVBoxLayout *main_layout;
  ExperimentalButton *experimental_btn;
  QPixmap dm_img;
  QPixmap engage_img;
  QPixmap experimental_img;
  QPixmap gear_img_p;
  QPixmap gear_img_r;
  QPixmap gear_img_n;
  QPixmap gear_img_d;
  QPixmap kisapilot_img;
  QPixmap waze_police_img;
  QPixmap waze_cam_img;

  float speed;
  QString speedUnit;
  float setSpeed;
  float speedLimit;
  bool is_cruise_set = false;
  bool is_metric = false;
  bool dmActive = false;
  bool hideBottomIcons = false;
  bool rightHandDM = false;
  float dm_fade_state = 1.0;
  bool has_us_speed_limit = false;
  bool has_eu_speed_limit = false;
  bool v_ego_cluster_seen = false;
  int status = STATUS_DISENGAGED;
  std::unique_ptr<PubMaster> pm;

  int skip_frame_count = 0;
  bool wide_cam_requested = false;

  bool over_sl = false;
  bool lead_stat = false;
  float dist_rel = 0;
  float vel_rel = 0;

  // neokii screen recorder. thx for sharing your source. 
  ScreenRecoder* recorder;
  std::shared_ptr<QTimer> record_timer;

protected:
  void paintGL() override;
  void paintEvent(QPaintEvent *event) override;
  void initializeGL() override;
  void showEvent(QShowEvent *event) override;
  void updateFrameMat() override;
  void drawLaneLines(QPainter &painter, const UIState *s);
  void drawLead(QPainter &painter, const cereal::RadarState::LeadData::Reader &lead_data, const QPointF &vd);
  void drawHud(QPainter &p);
  void drawDriverState(QPainter &painter, const UIState *s);
  void drawWheelState(QPainter &painter, const UIState *s);
  inline QColor redColor(int alpha = 255) { return QColor(201, 34, 49, alpha); }
  inline QColor whiteColor(int alpha = 255) { return QColor(255, 255, 255, alpha); }
  inline QColor blackColor(int alpha = 255) { return QColor(0, 0, 0, alpha); }
  inline QColor yellowColor(int alpha = 255) { return QColor(218, 202, 37, alpha); }
  inline QColor ochreColor(int alpha = 255) { return QColor(218, 111, 37, alpha); }
  inline QColor greenColor(int alpha = 255) { return QColor(0, 255, 0, alpha); }
  inline QColor blueColor(int alpha = 255) { return QColor(0, 0, 255, alpha); }
  inline QColor orangeColor(int alpha = 255) { return QColor(255, 175, 3, alpha); }
  inline QColor greyColor(int alpha = 1) { return QColor(191, 191, 191, alpha); }

  double prev_draw_t = 0;
  FirstOrderFilter fps_filter;
};

// container for all onroad widgets
class OnroadWindow : public QWidget {
  Q_OBJECT

public:
  OnroadWindow(QWidget* parent = 0);
  bool isMapVisible() const { return map && map->isVisible(); }
  void showMapPanel(bool show) { if (map) map->setVisible(show); }

signals:
  void mapPanelRequested();

private:
  void createMapWidget();
  void paintEvent(QPaintEvent *event);
  void mousePressEvent(QMouseEvent* e) override;
  OnroadAlerts *alerts;
  AnnotatedCameraWidget *nvg;
  QColor bg = bg_colors[STATUS_DISENGAGED];
  QWidget *map = nullptr;
  QHBoxLayout* split;

private slots:
  void offroadTransition(bool offroad);
  void primeChanged(bool prime);
  void updateState(const UIState &s);
};
