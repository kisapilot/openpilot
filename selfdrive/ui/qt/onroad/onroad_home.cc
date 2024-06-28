#include "selfdrive/ui/qt/onroad/onroad_home.h"

#include <QPainter>
#include <QStackedLayout>
#include <QProcess>
#include <QMouseEvent>

#include "selfdrive/ui/qt/util.h"

OnroadWindow::OnroadWindow(QWidget *parent) : QWidget(parent) {
  QVBoxLayout *main_layout  = new QVBoxLayout(this);
  main_layout->setMargin(UI_BORDER_SIZE);
  QStackedLayout *stacked_layout = new QStackedLayout;
  stacked_layout->setStackingMode(QStackedLayout::StackAll);
  main_layout->addLayout(stacked_layout);

  nvg = new AnnotatedCameraWidget(VISION_STREAM_ROAD, this);

  QWidget * split_wrapper = new QWidget;
  split = new QHBoxLayout(split_wrapper);
  split->setContentsMargins(0, 0, 0, 0);
  split->setSpacing(0);
  split->addWidget(nvg);

  if (getenv("DUAL_CAMERA_VIEW")) {
    CameraWidget *arCam = new CameraWidget("camerad", VISION_STREAM_ROAD, true, this);
    split->insertWidget(0, arCam);
  }

  stacked_layout->addWidget(split_wrapper);

  alerts = new OnroadAlerts(this);
  alerts->setAttribute(Qt::WA_TransparentForMouseEvents, true);
  stacked_layout->addWidget(alerts);

  // setup stacking order
  alerts->raise();

  setAttribute(Qt::WA_OpaquePaintEvent);
  QObject::connect(uiState(), &UIState::uiUpdate, this, &OnroadWindow::updateState);
  QObject::connect(uiState(), &UIState::offroadTransition, this, &OnroadWindow::offroadTransition);
}

void OnroadWindow::updateState(const UIState &s) {
  if (!s.scene.started) {
    return;
  }

  if (!s.is_OpenpilotViewEnabled) {
    // kisapilot
    if (s.scene.show_error) {
      if(access("/data/log/error.txt", F_OK ) != -1) {
        const std::string txt = util::read_file("/data/log/error.txt");
        if (RichTextDialog::alert(QString::fromStdString(txt), this)) {
          QProcess::execute("rm -f /data/log/error.txt");
          QTimer::singleShot(500, []() {});
        }
      }
    }
    alerts->updateState(s);
    //printf("OPVIEW: %s\n", uiState()->is_OpenpilotViewEnabled ? "true" : "false");
  }

  nvg->updateState(s);

  QColor bgColor = bg_colors[s.status];

  if (s.scene.brakePress) bgColor = bg_colors[STATUS_DISENGAGED];
  if (s.scene.comma_stock_ui == 2) bgColor = bg_colors[STATUS_DND];

  if (bg != bgColor) {
    // repaint border
    bg = bgColor;
    update();
  } else if (s.scene.rec_stat3) {
    update();
  }
}

void OnroadWindow::mousePressEvent(QMouseEvent* e) {

  QRect stockui_btn = QRect(15, uiState()->scene.low_ui_profile?693:15, 184, 202);
  QRect tuneui_btn = QRect(1420, uiState()->scene.low_ui_profile?15:895, 160, 160);
  QRect speedlimit_btn = QRect(220, uiState()->scene.low_ui_profile?700:15, 190, 190);
  QRect monitoring_btn = QRect(20, uiState()->scene.low_ui_profile?20:860, 190, 190);
  QRect multi_btn = QRect(1960, uiState()->scene.low_ui_profile?15:895, 160, 160);
  QRect rec_btn = QRect(1780, uiState()->scene.low_ui_profile?15:895, 160, 160);
  QRect laneless_btn = QRect(1600, uiState()->scene.low_ui_profile?15:895, 160, 160);

  if (uiState()->scene.multi_btn_touched && rec_btn.contains(e->pos())) {
    uiState()->scene.rec_blinker = 0;
    uiState()->scene.rec_stat = !uiState()->scene.rec_stat;
    params.putBool("RecordingRunning", uiState()->scene.rec_stat);
    return;
  }


  if ((multi_btn.contains(e->pos()) || speedlimit_btn.contains(e->pos()) || monitoring_btn.contains(e->pos()) ||
    stockui_btn.contains(e->pos()) || uiState()->scene.live_tune_panel_enable ||
    (uiState()->scene.multi_btn_touched && (rec_btn.contains(e->pos()) || laneless_btn.contains(e->pos()) || tuneui_btn.contains(e->pos()))))) {
    QWidget::mousePressEvent(e);
    return;
  }

  // propagation event to parent(HomeWindow)
  QWidget::mousePressEvent(e);
}

void OnroadWindow::offroadTransition(bool offroad) {
  alerts->clear();
}

void OnroadWindow::paintEvent(QPaintEvent *event) {
  QPainter p(this);
  p.fillRect(rect(), QColor(bg.red(), bg.green(), bg.blue(), 255));

  UIState *s = uiState();

  if (s->scene.rec_stat3) {
    const int rw = 2160;
    const int rh = 1080;
    const int rl = 80;
    QPoint topleft[] = {{0, 0}, {rl, 0}, {rl, UI_BORDER_SIZE}, {UI_BORDER_SIZE, UI_BORDER_SIZE}, {UI_BORDER_SIZE, rl}, {0, rl}};
    QPoint topright[] = {{rw, 0}, {rw-rl, 0}, {rw-rl, UI_BORDER_SIZE}, {rw-UI_BORDER_SIZE, UI_BORDER_SIZE}, {rw-UI_BORDER_SIZE, rl}, {rw, rl}};
    QPoint bottomleft[] = {{0, rh}, {rl, rh}, {rl, rh-UI_BORDER_SIZE}, {UI_BORDER_SIZE, rh-UI_BORDER_SIZE}, {UI_BORDER_SIZE, rh-rl}, {0, rh-rl}};
    QPoint bottomright[] = {{rw, rh}, {rw-rl, rh}, {rw-rl, rh-UI_BORDER_SIZE}, {rw-UI_BORDER_SIZE, rh-UI_BORDER_SIZE}, {rw-UI_BORDER_SIZE, rh-rl}, {rw, rh-rl}};

    p.setPen(Qt::NoPen);
    if(s->scene.rec_blinker >= 255) {
      s->scene.rec_blinker_stat = true;
    } else if (s->scene.rec_blinker <= 50) {
      s->scene.rec_blinker_stat = false;
    }
    if (!s->scene.rec_blinker_stat) {
      s->scene.rec_blinker += 5;
    } else {
      s->scene.rec_blinker -= 5;
    }
    p.setBrush(QColor(255, 0, 0, s->scene.rec_blinker));

    p.drawPolygon(topleft, std::size(topleft));
    p.drawPolygon(topright, std::size(topright));
    p.drawPolygon(bottomleft, std::size(bottomleft));
    p.drawPolygon(bottomright, std::size(bottomright));
  }
}
