#pragma once

#include <QVBoxLayout>
#include <memory>
#include "selfdrive/ui/qt/onroad/hud.h"
#include "selfdrive/ui/qt/onroad/buttons.h"
#include "selfdrive/ui/qt/onroad/driver_monitoring.h"
#include "selfdrive/ui/qt/widgets/cameraview.h"

#include <QTimer>
#include "selfdrive/ui/qt/screenrecorder/screenrecorder.h"

#include "common/params.h"

class AnnotatedCameraWidget : public CameraWidget {
  Q_OBJECT

public:
  explicit AnnotatedCameraWidget(VisionStreamType type, QWidget* parent = 0);
  void updateState(const UIState &s);

private:
  void drawText(QPainter &p, int x, int y, const QString &text, int alpha = 255);

  QVBoxLayout *main_layout;
  ExperimentalButton *experimental_btn;
  DriverMonitorRenderer dmon;
  HudRenderer hud;
  std::unique_ptr<PubMaster> pm;

  int skip_frame_count = 0;
  bool wide_cam_requested = false;

  // neokii screen recorder. thx for sharing your source. 
  ScreenRecoder* recorder;
  std::shared_ptr<QTimer> record_timer;

  Params params;

protected:
  void paintGL() override;
  void paintEvent(QPaintEvent *event) override;
  void initializeGL() override;
  void showEvent(QShowEvent *event) override;
  mat4 calcFrameMatrix() override;
  void drawLaneLines(QPainter &painter, const UIState *s);
  void drawLead(QPainter &painter, const cereal::RadarState::LeadData::Reader &lead_data, const QPointF &vd);
  inline QColor redColor(int alpha = 255) { return QColor(201, 34, 49, alpha); }
  inline QColor greenColor(int alpha = 255) { return QColor(0, 255, 0, alpha); }

  double prev_draw_t = 0;
  FirstOrderFilter fps_filter;
};
