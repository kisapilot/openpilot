#include "selfdrive/ui/qt/onroad/buttons.h"

#include <QPainter>

#include "selfdrive/ui/qt/util.h"

void drawIcon(QPainter &p, const QPoint &center, const QPixmap &img, const QBrush &bg, float opacity) {
  p.setRenderHint(QPainter::Antialiasing);
  p.setOpacity(1.0);  // bg dictates opacity of ellipse
  p.setPen(Qt::NoPen);
  p.setBrush(bg);
  p.drawEllipse(center, btn_size / 2, btn_size / 2);
  p.setOpacity(opacity);
  p.drawPixmap(center - QPoint(img.width() / 2, img.height() / 2), img);
  p.setOpacity(1.0);
}

// ExperimentalButton
ExperimentalButton::ExperimentalButton(QWidget *parent) : experimental_mode(false), engageable(false), QPushButton(parent) {
  setFixedSize(btn_size, btn_size);

  engage_img = loadPixmap("../assets/img_chffr_wheel.png", {img_size, img_size});
  experimental_img = loadPixmap("../assets/img_experimental.svg", {img_size, img_size});
  QObject::connect(this, &QPushButton::clicked, this, &ExperimentalButton::changeMode);
}

void ExperimentalButton::changeMode() {
  const auto cp = (*uiState()->sm)["carParams"].getCarParams();
  bool can_change = hasLongitudinalControl(cp);
  if (can_change) {
    params.putBool("ExperimentalMode", !experimental_mode);
  }
}

void ExperimentalButton::updateState(const UIState &s) {
  const auto cs = (*s.sm)["controlsState"].getControlsState();
  bool eng = cs.getEngageable() || cs.getEnabled();
  if ((cs.getExperimentalMode() != experimental_mode) || (eng != engageable)) {
    engageable = eng;
    experimental_mode = cs.getExperimentalMode();
    update();
  }
}

void ExperimentalButton::paintEvent(QPaintEvent *event) {
  QPainter p(this);

  QPoint center(btn_size / 2, btn_size / 2);
  QPixmap img = experimental_mode ? experimental_img : engage_img;

  p.setOpacity(1.0);
  p.setPen(Qt::NoPen);
  p.setBrush(QColor(0, 0, 0, 166));
  p.drawEllipse(center, btn_size / 2, btn_size / 2);
  p.setOpacity((isDown() || !engageable) ? 0.6 : 1.0);
  p.drawPixmap((btn_size - img_size) / 2, (btn_size - img_size) / 2, img);
}

// MapSettingsButton
MapSettingsButton::MapSettingsButton(QWidget *parent) : QPushButton(parent) {
  setFixedSize(btn_size, btn_size);
  settings_img = loadPixmap("../assets/navigation/icon_directions_outlined.svg", {img_size, img_size});
  settings_img_g = loadPixmap("../assets/addon/img/icon_directions_outlined_green.svg", {img_size, img_size});

  // hidden by default, made visible if map is created (has prime or mapbox token)
  setVisible(false);
  setEnabled(false);
}

void MapSettingsButton::updateState(const UIState &s) {
  if (s.scene.liveENaviData.ekisaconalive && !navi_is_alive) {
    navi_is_alive = true;
    update();
  } else if (!s.scene.liveENaviData.ekisaconalive && navi_is_alive) {
    navi_is_alive = false;
    update();
  }
}

void MapSettingsButton::paintEvent(QPaintEvent *event) {
  QPainter p(this);

  QPoint center(btn_size / 2, btn_size / 2);
  QPixmap img = navi_is_alive ? settings_img_g : settings_img;

  p.setOpacity(1.0);
  p.setPen(Qt::NoPen);
  p.setBrush(QColor(0, 0, 0, 166));
  p.drawEllipse(center, btn_size / 2, btn_size / 2);
  p.setOpacity(isDown() ? 0.6 : 1.0);
  p.drawPixmap((btn_size - img_size) / 2, (btn_size - img_size) / 2, img);
}
