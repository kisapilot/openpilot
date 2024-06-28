#include "selfdrive/ui/qt/home.h"

#include <QHBoxLayout>
#include <QMouseEvent>
#include <QStackedWidget>
#include <QVBoxLayout>

#include "selfdrive/ui/qt/offroad/experimental_mode.h"
#include "selfdrive/ui/qt/util.h"
#include "selfdrive/ui/qt/widgets/prime.h"

// HomeWindow: the container for the offroad and onroad UIs

HomeWindow::HomeWindow(QWidget* parent) : QWidget(parent) {
  QHBoxLayout *main_layout = new QHBoxLayout(this);
  main_layout->setMargin(0);
  main_layout->setSpacing(0);

  sidebar = new Sidebar(this);
  main_layout->addWidget(sidebar);
  QObject::connect(sidebar, &Sidebar::openSettings, this, &HomeWindow::openSettings);

  slayout = new QStackedLayout();
  main_layout->addLayout(slayout);

  home = new OffroadHome(this);
  QObject::connect(home, &OffroadHome::openSettings, this, &HomeWindow::openSettings);
  slayout->addWidget(home);

  onroad = new OnroadWindow(this);
  slayout->addWidget(onroad);

  body = new BodyWindow(this);
  slayout->addWidget(body);

  driver_view = new DriverViewWindow(this);
  connect(driver_view, &DriverViewWindow::done, [=] {
    showDriverView(false);
  });
  slayout->addWidget(driver_view);
  setAttribute(Qt::WA_NoSystemBackground);
  QObject::connect(uiState(), &UIState::uiUpdate, this, &HomeWindow::updateState);
  QObject::connect(uiState(), &UIState::offroadTransition, this, &HomeWindow::offroadTransition);
  QObject::connect(uiState(), &UIState::offroadTransition, sidebar, &Sidebar::offroadTransition);
}

void HomeWindow::showSidebar(bool show) {
  sidebar->setVisible(show);
}

void HomeWindow::updateState(const UIState &s) {
  const SubMaster &sm = *(s.sm);

  // switch to the generic robot UI
  if (onroad->isVisible() && !body->isEnabled() && sm["carParams"].getCarParams().getNotCar()) {
    body->setEnabled(true);
    slayout->setCurrentWidget(body);
  }
}

void HomeWindow::offroadTransition(bool offroad) {
  body->setEnabled(false);
  sidebar->setVisible(offroad);
  if (offroad) {
    slayout->setCurrentWidget(home);
  } else {
    slayout->setCurrentWidget(onroad);
  }
}

void HomeWindow::showDriverView(bool show) {
  if (show) {
    emit closeSettings();
    slayout->setCurrentWidget(driver_view);
  } else {
    slayout->setCurrentWidget(home);
  }
  sidebar->setVisible(show == false);
}


int HomeWindow::clip(int &x, int lo, int hi)
{
  int  nMin = hi;

  if (hi > x) nMin = x;
  if (lo > nMin) {
    x = lo;
  } else {
    x = nMin;
  }
  return x;
}
  

void HomeWindow::mousePressCommon(QMouseEvent* e, int nDir) {
  int live_tune_panel_list = uiState()->scene.live_tune_panel_list;

  if (live_tune_panel_list == 0) {
    uiState()->scene.cameraOffset += 5*nDir;
    clip(uiState()->scene.cameraOffset, -1000, 1000);
    QString value = QString::number(uiState()->scene.cameraOffset);
    Params().put("CameraOffsetAdj", value.toStdString());
  } else if (live_tune_panel_list == 1) {
    uiState()->scene.pathOffset += 5*nDir;
    clip(uiState()->scene.pathOffset, -1000, 1000);
    QString value = QString::number(uiState()->scene.pathOffset);
    Params().put("PathOffsetAdj", value.toStdString());
  }
}


void HomeWindow::mousePressPID(QMouseEvent* e, int nDir) {
  int nMenuPos = uiState()->scene.live_tune_panel_list - uiState()->scene.list_count;

  if (nMenuPos == 0) {
    uiState()->scene.pidKp += nDir;
    // 50
    clip(uiState()->scene.pidKp, 1, 50);
    QString value = QString::number(uiState()->scene.pidKp);
    Params().put("PidKp", value.toStdString());
  } else if (nMenuPos == 1) {
    uiState()->scene.pidKi += nDir;
    clip(uiState()->scene.pidKi, 1, 100);
    // 100
    QString value = QString::number(uiState()->scene.pidKi);
    Params().put("PidKi", value.toStdString());
  } else if (nMenuPos == 2) {
    uiState()->scene.pidKd += 5*nDir;
    // 300
    clip(uiState()->scene.pidKd, 0, 300);
    QString value = QString::number(uiState()->scene.pidKd);
    Params().put("PidKd", value.toStdString());
  } else if (nMenuPos == 3) {
    uiState()->scene.pidKf += nDir;
    clip(uiState()->scene.pidKf, 1, 50);
    // 50
    QString value = QString::number(uiState()->scene.pidKf);
    Params().put("PidKf", value.toStdString());
  }
}

void HomeWindow::mousePressINDI(QMouseEvent* e, int nDir) {
  int nMenuPos = uiState()->scene.live_tune_panel_list - uiState()->scene.list_count;

  if (nMenuPos == 0) {
    uiState()->scene.indiInnerLoopGain += nDir;
    clip(uiState()->scene.indiInnerLoopGain, 1, 200);
    QString value = QString::number(uiState()->scene.indiInnerLoopGain);
    Params().put("InnerLoopGain", value.toStdString());
  } else if (nMenuPos == 1) {
    uiState()->scene.indiOuterLoopGain += nDir;
    clip(uiState()->scene.indiOuterLoopGain, 1, 200);
    QString value = QString::number(uiState()->scene.indiOuterLoopGain);
    Params().put("OuterLoopGain", value.toStdString());
  } else if (nMenuPos == 2) {
    uiState()->scene.indiTimeConstant += nDir;
    clip(uiState()->scene.indiTimeConstant, 1, 200);
    QString value = QString::number(uiState()->scene.indiTimeConstant);
    Params().put("TimeConstant", value.toStdString());
  } else if (nMenuPos == 3) {
    uiState()->scene.indiActuatorEffectiveness += nDir;
    clip(uiState()->scene.indiActuatorEffectiveness, 1, 200);
    QString value = QString::number(uiState()->scene.indiActuatorEffectiveness);
    Params().put("ActuatorEffectiveness", value.toStdString());
  }
}

void HomeWindow::mousePressLQR(QMouseEvent* e, int nDir) {
  int nMenuPos = uiState()->scene.live_tune_panel_list - uiState()->scene.list_count;

  if (nMenuPos == 0) {
    uiState()->scene.lqrScale += 50*nDir;
    clip(uiState()->scene.lqrScale, 50, 5000);
    QString value = QString::number(uiState()->scene.lqrScale);
    Params().put("Scale", value.toStdString());
  } else if (nMenuPos == 1) {
    uiState()->scene.lqrKi += nDir;
    clip(uiState()->scene.lqrKi, 1, 100);
    QString value = QString::number(uiState()->scene.lqrKi);
    Params().put("LqrKi", value.toStdString());
  } else if (nMenuPos == 2) {
    uiState()->scene.lqrDcGain += 5*nDir;
    clip(uiState()->scene.lqrDcGain, 5, 500);
    QString value = QString::number(uiState()->scene.lqrDcGain);
    Params().put("DcGain", value.toStdString());
  }
}

void HomeWindow::mousePressTORQ(QMouseEvent* e, int nDir) {
  int nMenuPos = uiState()->scene.live_tune_panel_list - uiState()->scene.list_count;
  int max_lat_accel = uiState()->scene.torqueMaxLatAccel;

  if (nMenuPos == 0) {
    uiState()->scene.torqueKp += nDir;
    clip(uiState()->scene.torqueKp, 1, max_lat_accel);
    QString value = QString::number(uiState()->scene.torqueKp);
    Params().put("TorqueKp", value.toStdString());
  } else if (nMenuPos == 1) {
    uiState()->scene.torqueKf += nDir;
    clip(uiState()->scene.torqueKf, 1, max_lat_accel);
    QString value = QString::number(uiState()->scene.torqueKf);
    Params().put("TorqueKf", value.toStdString());
  } else if (nMenuPos == 2) {
    uiState()->scene.torqueKi += nDir;
    clip(uiState()->scene.torqueKi, 1, max_lat_accel);
    QString value = QString::number(uiState()->scene.torqueKi);
    Params().put("TorqueKi", value.toStdString());
  } else if (nMenuPos == 3) {
    uiState()->scene.torqueMaxLatAccel += nDir;
    clip(uiState()->scene.torqueMaxLatAccel, 1, 50);
    QString value = QString::number(uiState()->scene.torqueMaxLatAccel);
    Params().put("TorqueMaxLatAccel", value.toStdString());
  } else if (nMenuPos == 4) {
    uiState()->scene.torqueFriction += 5*nDir;
    clip(uiState()->scene.torqueFriction, 0, 300);
    QString value = QString::number(uiState()->scene.torqueFriction);
    Params().put("TorqueFriction", value.toStdString());
  }
}

void HomeWindow::mousePressMULTI(QMouseEvent* e, int nDir) {
  int nMenuPos = uiState()->scene.live_tune_panel_list - uiState()->scene.list_count;
  int max_lat_accel = uiState()->scene.torqueMaxLatAccel;
  if (nMenuPos == 0) {
    uiState()->scene.pidKp += nDir;
    // 50
    clip(uiState()->scene.pidKp, 1, 50);
    QString value = QString::number(uiState()->scene.pidKp);
    Params().put("PidKp", value.toStdString());
  } else if (nMenuPos == 1) {
    uiState()->scene.pidKi += nDir;
    clip(uiState()->scene.pidKi, 1, 100);
    // 100
    QString value = QString::number(uiState()->scene.pidKi);
    Params().put("PidKi", value.toStdString());
  } else if (nMenuPos == 2) {
    uiState()->scene.pidKd += 5*nDir;
    // 300
    clip(uiState()->scene.pidKd, 0, 300);
    QString value = QString::number(uiState()->scene.pidKd);
    Params().put("PidKd", value.toStdString());
  } else if (nMenuPos == 3) {
    uiState()->scene.pidKf += nDir;
    clip(uiState()->scene.pidKf, 1, 50);
    // 50
    QString value = QString::number(uiState()->scene.pidKf);
    Params().put("PidKf", value.toStdString());
  } else if (nMenuPos == 4) {
    uiState()->scene.indiInnerLoopGain += nDir;
    clip(uiState()->scene.indiInnerLoopGain, 1, 200);
    QString value = QString::number(uiState()->scene.indiInnerLoopGain);
    Params().put("InnerLoopGain", value.toStdString());
  } else if (nMenuPos == 5) {
    uiState()->scene.indiOuterLoopGain += nDir;
    clip(uiState()->scene.indiOuterLoopGain, 1, 200);
    QString value = QString::number(uiState()->scene.indiOuterLoopGain);
    Params().put("OuterLoopGain", value.toStdString());
  } else if (nMenuPos == 6) {
    uiState()->scene.indiTimeConstant += nDir;
    clip(uiState()->scene.indiTimeConstant, 1, 200);
    QString value = QString::number(uiState()->scene.indiTimeConstant);
    Params().put("TimeConstant", value.toStdString());
  } else if (nMenuPos == 7) {
    uiState()->scene.indiActuatorEffectiveness += nDir;
    clip(uiState()->scene.indiActuatorEffectiveness, 1, 200);
    QString value = QString::number(uiState()->scene.indiActuatorEffectiveness);
    Params().put("ActuatorEffectiveness", value.toStdString());
  } else if (nMenuPos == 8) {
    uiState()->scene.lqrScale += 50*nDir;
    clip(uiState()->scene.lqrScale, 50, 5000);
    QString value = QString::number(uiState()->scene.lqrScale);
    Params().put("Scale", value.toStdString());
  } else if (nMenuPos == 9) {
    uiState()->scene.lqrKi += nDir;
    clip(uiState()->scene.lqrKi, 1, 100);
    QString value = QString::number(uiState()->scene.lqrKi);
    Params().put("LqrKi", value.toStdString());
  } else if (nMenuPos == 10) {
    uiState()->scene.lqrDcGain += 5*nDir;
    clip(uiState()->scene.lqrDcGain, 5, 500);
    QString value = QString::number(uiState()->scene.lqrDcGain);
    Params().put("DcGain", value.toStdString());
  } else if (nMenuPos == 11) {
    uiState()->scene.torqueKp += nDir;
    clip(uiState()->scene.torqueKp, 1, max_lat_accel);
    QString value = QString::number(uiState()->scene.torqueKp);
    Params().put("TorqueKp", value.toStdString());
  } else if (nMenuPos == 12) {
    uiState()->scene.torqueKf += nDir;
    clip(uiState()->scene.torqueKf, 1, max_lat_accel);
    QString value = QString::number(uiState()->scene.torqueKf);
    Params().put("TorqueKf", value.toStdString());
  } else if (nMenuPos == 13) {
    uiState()->scene.torqueKi += nDir;
    clip(uiState()->scene.torqueKi, 1, max_lat_accel);
    QString value = QString::number(uiState()->scene.torqueKi);
    Params().put("TorqueKi", value.toStdString());
  } else if (nMenuPos == 14) {
    uiState()->scene.torqueMaxLatAccel += nDir;
    clip(uiState()->scene.torqueMaxLatAccel, 1, 50);
    QString value = QString::number(uiState()->scene.torqueMaxLatAccel);
    Params().put("TorqueMaxLatAccel", value.toStdString());
  } else if (nMenuPos == 15) {
    uiState()->scene.torqueFriction += 5*nDir;
    clip(uiState()->scene.torqueFriction, 0, 300);
    QString value = QString::number(uiState()->scene.torqueFriction);
    Params().put("TorqueFriction", value.toStdString());
  }
}

void HomeWindow::mousePressEvent(QMouseEvent* e) 
{
  QRect livetunepanel_left_above_btn = QRect(590, 570, 210, 170);
  QRect livetunepanel_right_above_btn = QRect(1360, 570, 210, 170);
  QRect livetunepanel_left_btn = QRect(590, 745, 210, 170);
  QRect livetunepanel_right_btn = QRect(1360, 745, 210, 170);

  QRect stockui_btn = QRect(15, uiState()->scene.low_ui_profile?693:15, 184, 202);
  QRect tuneui_btn = QRect(1420, uiState()->scene.low_ui_profile?15:895, 160, 160);
  QRect speedlimit_btn = QRect(220, uiState()->scene.low_ui_profile?700:15, 190, 190);
  QRect monitoring_btn = QRect(20, uiState()->scene.low_ui_profile?20:860, 190, 190);
  QRect multi_btn = QRect(1960, uiState()->scene.low_ui_profile?15:895, 160, 160);
  QRect rec_btn = QRect(1780, uiState()->scene.low_ui_profile?15:895, 160, 160);
  QRect laneless_btn = QRect(1600, uiState()->scene.low_ui_profile?15:895, 160, 160);

  printf( "mousePressEvent = (%d,%d)\n", e->x(), e->y() );

  // KISA Multi Button
  if (uiState()->scene.started && !sidebar->isVisible() && uiState()->scene.comma_stock_ui != 1 && multi_btn.contains(e->pos())) {
    uiState()->scene.multi_btn_touched = !uiState()->scene.multi_btn_touched;
    return;
  }
  // // KISA REC
  // if (uiState()->scene.started && !sidebar->isVisible() && uiState()->scene.comma_stock_ui != 1 && rec_btn.contains(e->pos()) &&
  //     uiState()->scene.multi_btn_touched) {
  //   uiState()->scene.touched = true;
  //   return;
  // }
  // Laneless mode
  if (uiState()->scene.started && !sidebar->isVisible() && uiState()->scene.comma_stock_ui != 1 && laneless_btn.contains(e->pos()) &&
      uiState()->scene.multi_btn_touched) {
    uiState()->scene.laneless_mode = uiState()->scene.laneless_mode + 1;
    if (uiState()->scene.laneless_mode > 2) {
      uiState()->scene.laneless_mode = 0;
    }
    if (uiState()->scene.laneless_mode == 0) {
      Params().put("LanelessMode", "0", 1);
    } else if (uiState()->scene.laneless_mode == 1) {
      Params().put("LanelessMode", "1", 1);
    } else if (uiState()->scene.laneless_mode == 2) {
      Params().put("LanelessMode", "2", 1);
    }
    return;
  }
  // Monitoring mode
  if (uiState()->scene.started && !sidebar->isVisible() && monitoring_btn.contains(e->pos())) {
    uiState()->scene.monitoring_mode = !uiState()->scene.monitoring_mode;
    if (uiState()->scene.monitoring_mode) {
      Params().putBool("KisaMonitoringMode", true);
    } else {
      Params().putBool("KisaMonitoringMode", false);
    }
    return;
  }
  // Stock UI Toggle
  if (uiState()->scene.started && !sidebar->isVisible() && stockui_btn.contains(e->pos())) {
    uiState()->scene.comma_stock_ui = uiState()->scene.comma_stock_ui + 1;
    if (uiState()->scene.do_not_disturb_mode > 0) {
      if (uiState()->scene.comma_stock_ui > 2) {
        uiState()->scene.comma_stock_ui = 0;
      }
    } else {
      if (uiState()->scene.comma_stock_ui > 1 ) {
        uiState()->scene.comma_stock_ui = 0;
      }
    }

    if (uiState()->scene.comma_stock_ui == 0) {
      Params().put("CommaStockUI", "0", 1);
    } else if (uiState()->scene.comma_stock_ui == 1) {
      Params().put("CommaStockUI", "1", 1);
    } else if (uiState()->scene.comma_stock_ui == 2) {
      Params().put("CommaStockUI", "2", 1);
      uiState()->scene.touched2 = true;
      QTimer::singleShot(500, []() { uiState()->scene.touched2 = false; });
    }
    return;
  }
  // LiveTune UI Toggle
  if (uiState()->scene.started && !sidebar->isVisible() && uiState()->scene.comma_stock_ui != 1 && tuneui_btn.contains(e->pos()) &&
      uiState()->scene.multi_btn_touched) {
    uiState()->scene.kisa_livetune_ui = !uiState()->scene.kisa_livetune_ui;
    if (uiState()->scene.kisa_livetune_ui) {
      Params().putBool("KisaLiveTunePanelEnable", true);
      uiState()->scene.live_tune_panel_enable = true;
    } else {
      Params().putBool("KisaLiveTunePanelEnable", false);
      uiState()->scene.live_tune_panel_enable = false;
    }
    return;
  }

  // SpeedLimit Decel on/off Toggle
  if (uiState()->scene.started && !sidebar->isVisible() && speedlimit_btn.contains(e->pos())) {
    uiState()->scene.sl_decel_off = !uiState()->scene.sl_decel_off;
    if (uiState()->scene.sl_decel_off) {
      Params().putBool("SpeedLimitDecelOff", true);
    } else {
      Params().putBool("SpeedLimitDecelOff", false);
    }
    return;
  }
  // kisa live ui tune
  if (uiState()->scene.live_tune_panel_enable && uiState()->scene.started && !sidebar->isVisible()) {
    int nBtnDir = 0;

    if (livetunepanel_left_btn.contains(e->pos())) {
      nBtnDir = -1;
    } else if (livetunepanel_right_btn.contains(e->pos())) {
      nBtnDir = 1;
    } else if (livetunepanel_left_above_btn.contains(e->pos())) {
      uiState()->scene.live_tune_panel_list -= 1;
      int nLoop = 2;

      if (uiState()->scene.live_tune_panel_list >= 0) return;
      if (uiState()->scene.lateralControlMethod == 2) {  // 2. LQR
        nLoop = 2;
      } else if (uiState()->scene.lateralControlMethod == 3) { // 3. TORQ
        nLoop = 4;
      } else if (uiState()->scene.lateralControlMethod < 2) {
        nLoop = 3;
      } else if (uiState()->scene.lateralControlMethod == 4) {
        nLoop = 15;
      }

      uiState()->scene.live_tune_panel_list = uiState()->scene.list_count + nLoop;
      //clip( uiState()->scene.live_tune_panel_list, 1, 0 );
      return;
    } else if (livetunepanel_right_above_btn.contains(e->pos())) {
      uiState()->scene.live_tune_panel_list += 1;
      int nLoop = uiState()->scene.list_count;

      if (uiState()->scene.lateralControlMethod == 2) { // 2. LQR
         nLoop = nLoop + 3;
      } else if (uiState()->scene.lateralControlMethod == 3) { // 3. TORQ
        nLoop = nLoop + 5;
      } else if (uiState()->scene.lateralControlMethod < 2) { // 0. PID,  1. INDI
        nLoop = nLoop + 4;
      } else if (uiState()->scene.lateralControlMethod == 4) { // 4. MULTI
        nLoop = nLoop + 16;
      }

      if(uiState()->scene.live_tune_panel_list < nLoop) return;
      uiState()->scene.live_tune_panel_list = 0;
      return;
    }

    if (nBtnDir) {
      mousePressCommon(e, nBtnDir);
      if (uiState()->scene.lateralControlMethod == 0) {  // 0. PID
        mousePressPID(e, nBtnDir);
      } else if (uiState()->scene.lateralControlMethod == 1) {  // 1. INDI
        mousePressINDI(e, nBtnDir);
      } else if (uiState()->scene.lateralControlMethod == 2) {  // 2. LQR
        mousePressLQR(e, nBtnDir);
      } else if (uiState()->scene.lateralControlMethod == 3) {  // 3. TORQ
        mousePressTORQ(e, nBtnDir);
      } else if (uiState()->scene.lateralControlMethod == 4) {  // 4. MULTI
        mousePressMULTI(e, nBtnDir);
      }
      return;
    }
  }

  // Handle sidebar collapsing
  if ((onroad->isVisible() || body->isVisible()) && (!sidebar->isVisible() || e->x() > sidebar->width())) {
    sidebar->setVisible(!sidebar->isVisible());
  }
  
  if (uiState()->scene.started && uiState()->scene.autoScreenOff != -3) {
    uiState()->scene.touched2 = true;
    QTimer::singleShot(500, []() { uiState()->scene.touched2 = false; });
  }

  if (uiState()->scene.monitoring_mode) {
    Params().putBool("KisaWakeUp", false);
  }
}

void HomeWindow::mouseDoubleClickEvent(QMouseEvent* e) {
  HomeWindow::mousePressEvent(e);
  const SubMaster &sm = *(uiState()->sm);
  if (sm["carParams"].getCarParams().getNotCar()) {
    if (onroad->isVisible()) {
      slayout->setCurrentWidget(body);
    } else if (body->isVisible()) {
      slayout->setCurrentWidget(onroad);
    }
    showSidebar(false);
  }
}

// OffroadHome: the offroad home page

OffroadHome::OffroadHome(QWidget* parent) : QFrame(parent) {
  QVBoxLayout* main_layout = new QVBoxLayout(this);
  main_layout->setContentsMargins(40, 40, 40, 40);

  // top header
  QHBoxLayout* header_layout = new QHBoxLayout();
  header_layout->setContentsMargins(0, 0, 0, 0);
  header_layout->setSpacing(16);

  update_notif = new QPushButton(tr("UPDATE"));
  update_notif->setVisible(false);
  update_notif->setStyleSheet("background-color: #364DEF;");
  QObject::connect(update_notif, &QPushButton::clicked, [=]() { center_layout->setCurrentIndex(1); });
  header_layout->addWidget(update_notif, 0, Qt::AlignHCenter | Qt::AlignLeft);

  alert_notif = new QPushButton();
  alert_notif->setVisible(false);
  alert_notif->setStyleSheet("background-color: #E22C2C;");
  QObject::connect(alert_notif, &QPushButton::clicked, [=] { center_layout->setCurrentIndex(2); });
  header_layout->addWidget(alert_notif, 0, Qt::AlignHCenter | Qt::AlignLeft);

  version = new ElidedLabel();
  header_layout->addWidget(version, 0, Qt::AlignHCenter | Qt::AlignRight);

  main_layout->addLayout(header_layout);

  // main content
  main_layout->addSpacing(25);
  center_layout = new QStackedLayout();

  QWidget *home_widget = new QWidget(this);
  {
    QHBoxLayout *home_layout = new QHBoxLayout(home_widget);
    home_layout->setContentsMargins(0, 0, 0, 0);
    home_layout->setSpacing(30);

    // left: PrimeAdWidget
    QStackedWidget *left_widget = new QStackedWidget(this);
    QVBoxLayout *left_prime_layout = new QVBoxLayout();
    QWidget *prime_user = new PrimeUserWidget();
    prime_user->setStyleSheet(R"(
    border-radius: 10px;
    background-color: #333333;
    )");
    left_prime_layout->addWidget(prime_user);
    left_prime_layout->addStretch();
    left_widget->addWidget(new LayoutWidget(left_prime_layout));
    left_widget->addWidget(new PrimeAdWidget);
    left_widget->setStyleSheet("border-radius: 10px;");

    left_widget->setCurrentIndex(uiState()->hasPrime() ? 0 : 1);
    connect(uiState(), &UIState::primeChanged, [=](bool prime) {
      left_widget->setCurrentIndex(prime ? 0 : 1);
    });

    home_layout->addWidget(left_widget, 1);

    // right: ExperimentalModeButton, SetupWidget
    QWidget* right_widget = new QWidget(this);
    QVBoxLayout* right_column = new QVBoxLayout(right_widget);
    right_column->setContentsMargins(0, 0, 0, 0);
    right_widget->setFixedWidth(750);
    right_column->setSpacing(30);

    ExperimentalModeButton *experimental_mode = new ExperimentalModeButton(this);
    QObject::connect(experimental_mode, &ExperimentalModeButton::openSettings, this, &OffroadHome::openSettings);
    right_column->addWidget(experimental_mode, 1);

    SetupWidget *setup_widget = new SetupWidget;
    QObject::connect(setup_widget, &SetupWidget::openSettings, this, &OffroadHome::openSettings);
    right_column->addWidget(setup_widget, 1);

    home_layout->addWidget(right_widget, 1);
  }
  center_layout->addWidget(home_widget);

  // add update & alerts widgets
  update_widget = new UpdateAlert();
  QObject::connect(update_widget, &UpdateAlert::dismiss, [=]() { center_layout->setCurrentIndex(0); });
  center_layout->addWidget(update_widget);
  alerts_widget = new OffroadAlert();
  QObject::connect(alerts_widget, &OffroadAlert::dismiss, [=]() { center_layout->setCurrentIndex(0); });
  center_layout->addWidget(alerts_widget);

  main_layout->addLayout(center_layout, 1);

  // set up refresh timer
  timer = new QTimer(this);
  timer->callOnTimeout(this, &OffroadHome::refresh);

  setStyleSheet(R"(
    * {
      color: white;
    }
    OffroadHome {
      background-color: black;
    }
    OffroadHome > QPushButton {
      padding: 15px 30px;
      border-radius: 5px;
      font-size: 40px;
      font-weight: 500;
    }
    OffroadHome > QLabel {
      font-size: 55px;
    }
  )");
}

void OffroadHome::showEvent(QShowEvent *event) {
  refresh();
  timer->start(10 * 1000);
}

void OffroadHome::hideEvent(QHideEvent *event) {
  timer->stop();
}

void OffroadHome::refresh() {
  version->setText("KisaPilot " + QString::fromStdString(params.get("KisaPilotCurrentDescription")));

  bool updateAvailable = update_widget->refresh();
  int alerts = alerts_widget->refresh();

  // pop-up new notification
  int idx = center_layout->currentIndex();
  if (!updateAvailable && !alerts) {
    idx = 0;
  } else if (updateAvailable && (!update_notif->isVisible() || (!alerts && idx == 2))) {
    idx = 1;
  } else if (alerts && (!alert_notif->isVisible() || (!updateAvailable && idx == 1))) {
    idx = 2;
  }
  center_layout->setCurrentIndex(idx);

  update_notif->setVisible(updateAvailable);
  alert_notif->setVisible(alerts);
  if (alerts) {
    alert_notif->setText(QString::number(alerts) + (alerts > 1 ? tr(" ALERTS") : tr(" ALERT")));
  }
}
