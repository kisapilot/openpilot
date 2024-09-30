#pragma once

#include <QPainter>
#include "selfdrive/ui/ui.h"

class HudRenderer : public QObject {
  Q_OBJECT

public:
  HudRenderer();
  void updateState(const UIState &s);
  void draw(QPainter &p, const QRect &surface_rect);

private:
  void drawSetSpeed(QPainter &p, const QRect &surface_rect);
  void drawCurrentSpeed(QPainter &p, const QRect &surface_rect);
  void drawText(QPainter &p, int x, int y, const QString &text, int alpha = 255);

  void drawIcon(QPainter &p, int x, int y, QPixmap &img, QBrush bg = QColor(0,0,0,0), float opacity = 1.0, bool rotation = false, float angle = 0);
  void uiText(QPainter &p, int x, int y, const QString &text, int alpha = 255, bool custom_color = false);
  void debugText(QPainter &p, int x, int y, const QString &text, int alpha = 255, int fontsize = 30, bool bold = false);
  void drawWheelState(QPainter &p, const QRect &surface_rect);

  float speed = 0;
  float set_speed = 0;
  bool is_cruise_set = false;
  bool is_metric = false;
  bool v_ego_cluster_seen = false;
  int status = STATUS_DISENGAGED;

  QPixmap engage_img;
  QPixmap experimental_img;
  QPixmap gear_img_p;
  QPixmap gear_img_r;
  QPixmap gear_img_n;
  QPixmap gear_img_d;
  QPixmap kisapilot_img;
  QPixmap waze_police_img;
  QPixmap waze_cam_img;

  bool over_sl = false;
  bool lead_stat = false;
  float dist_rel = 0;
  float vel_rel = 0;

  const int UI_BORDER_SIZE = 15;

protected:
  inline QColor redColor(int alpha = 255) { return QColor(201, 34, 49, alpha); }
  inline QColor whiteColor(int alpha = 255) { return QColor(255, 255, 255, alpha); }
  inline QColor blackColor(int alpha = 255) { return QColor(0, 0, 0, alpha); }
  inline QColor yellowColor(int alpha = 255) { return QColor(218, 202, 37, alpha); }
  inline QColor ochreColor(int alpha = 255) { return QColor(218, 111, 37, alpha); }
  inline QColor greenColor(int alpha = 255) { return QColor(0, 255, 0, alpha); }
  inline QColor blueColor(int alpha = 255) { return QColor(0, 0, 255, alpha); }
  inline QColor orangeColor(int alpha = 255) { return QColor(255, 175, 3, alpha); }
};
