#include <cassert>
#include <cmath>
#include <string>
#include <tuple>
#include <vector>

#include <QDebug>
#include <QProcess> // kisapilot
#include <QDateTime> // kisapilot
#include <QTimer> // kisapilot
#include <QFileInfo> // kisapilot

#include "common/watchdog.h"
#include "common/util.h"
#include "selfdrive/ui/qt/network/networking.h"
#include "selfdrive/ui/qt/offroad/settings.h"
#include "selfdrive/ui/qt/qt_window.h"
#include "selfdrive/ui/qt/widgets/prime.h"
#include "selfdrive/ui/qt/widgets/scrollview.h"
#include "selfdrive/ui/qt/widgets/ssh_keys.h"

#include "selfdrive/ui/qt/widgets/kisapilot.h" // kisapilot
#include "selfdrive/ui/qt/widgets/steerWidget.h" // kisapilot

TogglesPanel::TogglesPanel(SettingsWindow *parent) : ListWidget(parent) {
  // param, title, desc, icon
  std::vector<std::tuple<QString, QString, QString, QString>> toggle_defs{
    {
      "OpenpilotEnabledToggle",
      tr("Enable openpilot"),
      tr("Use the openpilot system for adaptive cruise control and lane keep driver assistance. Your attention is required at all times to use this feature. Changing this setting takes effect when the car is powered off."),
      "../assets/offroad/icon_openpilot.png",
    },
    {
      "ExperimentalLongitudinalEnabled",
      tr("openpilot Longitudinal Control (Alpha)"),
      QString("<b>%1</b><br><br>%2")
      .arg(tr("WARNING: openpilot longitudinal control is in alpha for this car and will disable Automatic Emergency Braking (AEB)."))
      .arg(tr("On this car, openpilot defaults to the car's built-in ACC instead of openpilot's longitudinal control. "
              "Enable this to switch to openpilot longitudinal control. Enabling Experimental mode is recommended when enabling openpilot longitudinal control alpha.")),
      "../assets/offroad/icon_speed_limit.png",
    },
    {
      "ExperimentalMode",
      tr("Experimental Mode"),
      QString("%1<br>"
              "<h4>%2</h4><br>"
              "%3<br>"
              "<h4>%4</h4><br>"
              "%5<br>")
      .arg(tr("openpilot defaults to driving in <b>chill mode</b>. Experimental mode enables <b>alpha-level features</b> that aren't ready for chill mode. Experimental features are listed below:"))
      .arg(tr("End-to-End Longitudinal Control"))
      .arg(tr("Let the driving model control the gas and brakes. openpilot will drive as it thinks a human would, including stopping for red lights and stop signs. "
              "Since the driving model decides the speed to drive, the set speed will only act as an upper bound. This is an alpha quality feature; "
              "mistakes should be expected."))
      .arg(tr("New Driving Visualization"))
      .arg(tr("The driving visualization will transition to the road-facing wide-angle camera at low speeds to better show some turns. The Experimental mode logo will also be shown in the top right corner. ")),
      "../assets/img_experimental_white.svg",
    },
    {
      "DisengageOnAccelerator",
      tr("Disengage on Accelerator Pedal"),
      tr("When enabled, pressing the accelerator pedal will disengage openpilot."),
      "../assets/offroad/icon_disengage_on_accelerator.svg",
    },
    {
      "IsLdwEnabled",
      tr("Enable Lane Departure Warnings"),
      tr("Receive alerts to steer back into the lane when your vehicle drifts over a detected lane line without a turn signal activated while driving over 31 mph (50 km/h)."),
      "../assets/offroad/icon_warning.png",
    },
    {
      "AlwaysOnDM",
      tr("Always-On Driver Monitoring"),
      tr("Enable driver monitoring even when openpilot is not engaged."),
      "../assets/offroad/icon_monitoring.png",
    },
    {
      "RecordFront",
      tr("Record and Upload Driver Camera"),
      tr("Upload data from the driver facing camera and help improve the driver monitoring algorithm."),
      "../assets/offroad/icon_monitoring.png",
    },
    {
      "IsMetric",
      tr("Use Metric System"),
      tr("Display speed in km/h instead of mph."),
      "../assets/offroad/icon_metric.png",
    },
  };


  std::vector<QString> longi_button_texts{tr("Aggressive"), tr("Standard"), tr("Relaxed")};
  long_personality_setting = new ButtonParamControl("LongitudinalPersonality", tr("Driving Personality"),
                                          tr("Standard is recommended. In aggressive mode, openpilot will follow lead cars closer and be more aggressive with the gas and brake. "
                                             "In relaxed mode openpilot will stay further away from lead cars. On supported cars, you can cycle through these personalities with "
                                             "your steering wheel distance button."),
                                          "../assets/offroad/icon_speed_limit.png",
                                          longi_button_texts);

  // set up uiState update for personality setting
  QObject::connect(uiState(), &UIState::uiUpdate, this, &TogglesPanel::updateState);

  for (auto &[param, title, desc, icon] : toggle_defs) {
    auto toggle = new ParamControl(param, title, desc, icon, this);

    // bool locked = params.getBool((param + "Lock").toStdString());
    toggle->setEnabled(true);

    addItem(toggle);
    toggles[param.toStdString()] = toggle;

    // insert longitudinal personality after NDOG toggle
    if (param == "DisengageOnAccelerator") {
      addItem(long_personality_setting);
    }
  }

  // Toggles with confirmation dialogs
  toggles["ExperimentalMode"]->setActiveIcon("../assets/img_experimental.svg");
  toggles["ExperimentalMode"]->setConfirmation(true, false);
  toggles["ExperimentalLongitudinalEnabled"]->setConfirmation(true, false);
}

void TogglesPanel::updateState(const UIState &s) {
  const SubMaster &sm = *(s.sm);

  if (sm.updated("controlsState")) {
    auto personality = sm["controlsState"].getControlsState().getPersonality();
    if (personality != s.scene.personality && s.scene.started && isVisible()) {
      long_personality_setting->setCheckedButton(static_cast<int>(personality));
    }
    uiState()->scene.personality = personality;
  }
}

void TogglesPanel::expandToggleDescription(const QString &param) {
  toggles[param.toStdString()]->showDescription();
}

void TogglesPanel::showEvent(QShowEvent *event) {
}

DevicePanel::DevicePanel(SettingsWindow *parent) : ListWidget(parent) {
  setSpacing(50);
  addItem(new LabelControl(tr("Dongle ID"), getDongleId().value_or(tr("N/A"))));
  addItem(new LabelControl(tr("Serial"), params.get("HardwareSerial").c_str()));

  addItem(new OpenpilotView());

  pair_device = new ButtonControl(tr("Pair Device"), tr("PAIR"),
                                  tr("Pair your device with comma connect (connect.comma.ai) and claim your comma prime offer."));
  connect(pair_device, &ButtonControl::clicked, [=]() {
    PairingPopup popup(this);
    popup.exec();
  });
  addItem(pair_device);

  // offroad-only buttons

  auto dcamBtn = new ButtonControl(tr("Driver Camera"), tr("PREVIEW"),
                                   tr("Preview the driver facing camera to ensure that driver monitoring has good visibility. (vehicle must be off)"));
  connect(dcamBtn, &ButtonControl::clicked, [=]() { emit showDriverView(); });
  addItem(dcamBtn);

  auto resetCalibBtn = new ButtonControl(tr("Reset Calibration"), tr("RESET"), " ");
  connect(resetCalibBtn, &ButtonControl::showDescriptionEvent, this, &DevicePanel::updateCalibDescription);
  connect(resetCalibBtn, &ButtonControl::clicked, [&]() {
    if (ConfirmationDialog::confirm(tr("Are you sure you want to reset calibration?"), tr("Reset"), this)) {
      params.remove("CalibrationParams");
      //params.remove("LiveTorqueParameters");
      params.putBool("OnRoadRefresh", true);
      QTimer::singleShot(3000, [this]() {
        params.putBool("OnRoadRefresh", false);
      });
    }
  });
  addItem(resetCalibBtn);

  auto retrainingBtn = new ButtonControl(tr("Review Training Guide"), tr("REVIEW"), tr("Review the rules, features, and limitations of openpilot"));
  connect(retrainingBtn, &ButtonControl::clicked, [=]() {
    if (ConfirmationDialog::confirm(tr("Are you sure you want to review the training guide?"), tr("Review"), this)) {
      emit reviewTrainingGuide();
    }
  });
  addItem(retrainingBtn);

  if (Hardware::TICI()) {
    auto regulatoryBtn = new ButtonControl(tr("Regulatory"), tr("VIEW"), "");
    connect(regulatoryBtn, &ButtonControl::clicked, [=]() {
      const std::string txt = util::read_file("../assets/offroad/fcc.html");
      ConfirmationDialog::rich(QString::fromStdString(txt), this);
    });
    addItem(regulatoryBtn);
  }

  auto translateBtn = new ButtonControl(tr("Change Language"), tr("CHANGE"), "");
  connect(translateBtn, &ButtonControl::clicked, [=]() {
    QMap<QString, QString> langs = getSupportedLanguages();
    QString selection = MultiOptionDialog::getSelection(tr("Select a language"), langs.keys(), langs.key(uiState()->language), this);
    if (!selection.isEmpty()) {
      // put language setting, exit Qt UI, and trigger fast restart
      params.put("LanguageSetting", langs[selection].toStdString());
      qApp->exit(18);
      watchdog_kick(0);
    }
  });
  addItem(translateBtn);

  QObject::connect(uiState(), &UIState::primeTypeChanged, [this] (PrimeType type) {
    pair_device->setVisible(type == PrimeType::UNPAIRED);
  });
  QObject::connect(uiState(), &UIState::offroadTransition, [=](bool offroad) {
    for (auto btn : findChildren<ButtonControl *>()) {
      btn->setEnabled(true);
      //if (btn != pair_device) {
      //  btn->setEnabled(offroad);
      //}
    }
  });

  // power buttons
  QHBoxLayout *power_layout = new QHBoxLayout();
  power_layout->setSpacing(30);

  QPushButton *refresh_btn = new QPushButton(tr("Refresh"));
  refresh_btn->setObjectName("refresh_btn");
  power_layout->addWidget(refresh_btn);
  QObject::connect(refresh_btn, &QPushButton::clicked, this, &DevicePanel::onroadRefresh);

  QPushButton *reboot_btn = new QPushButton(tr("Reboot"));
  reboot_btn->setObjectName("reboot_btn");
  power_layout->addWidget(reboot_btn);
  QObject::connect(reboot_btn, &QPushButton::clicked, this, &DevicePanel::reboot);

  QPushButton *poweroff_btn = new QPushButton(tr("Power Off"));
  poweroff_btn->setObjectName("poweroff_btn");
  power_layout->addWidget(poweroff_btn);
  QObject::connect(poweroff_btn, &QPushButton::clicked, this, &DevicePanel::poweroff);

  if (!Hardware::PC()) {
    connect(uiState(), &UIState::offroadTransition, poweroff_btn, &QPushButton::setVisible);
  }

  setStyleSheet(R"(
    QPushButton {
      height: 120px;
      border-radius: 15px;
    }
    #refresh_btn { background-color: #83c744; }
    #refresh_btn:pressed { background-color: #c7deb1; }
    #reboot_btn { background-color: #ed8e3b; }
    #reboot_btn:pressed { background-color: #f0bf97; }
    #poweroff_btn { background-color: #E22C2C; }
    #poweroff_btn:pressed { background-color: #FF2424; }
  )");
  addItem(power_layout);
}

void DevicePanel::updateCalibDescription() {
  QString desc =
      tr("openpilot requires the device to be mounted within 4° left or right and "
         "within 5° up or 9° down. openpilot is continuously calibrating, resetting is rarely required.");
  std::string calib_bytes = params.get("CalibrationParams");
  if (!calib_bytes.empty()) {
    try {
      AlignedBuffer aligned_buf;
      capnp::FlatArrayMessageReader cmsg(aligned_buf.align(calib_bytes.data(), calib_bytes.size()));
      auto calib = cmsg.getRoot<cereal::Event>().getLiveCalibration();
      if (calib.getCalStatus() != cereal::LiveCalibrationData::Status::UNCALIBRATED) {
        double pitch = calib.getRpyCalib()[1] * (180 / M_PI);
        double yaw = calib.getRpyCalib()[2] * (180 / M_PI);
        desc += tr(" Your device is pointed %1° %2 and %3° %4.")
                    .arg(QString::number(std::abs(pitch), 'g', 1), pitch > 0 ? tr("down") : tr("up"),
                         QString::number(std::abs(yaw), 'g', 1), yaw > 0 ? tr("left") : tr("right"));
      }
    } catch (kj::Exception) {
      qInfo() << "invalid CalibrationParams";
    }
  }
  qobject_cast<ButtonControl *>(sender())->setDescription(desc);
}

void DevicePanel::onroadRefresh() {
  if (!uiState()->engaged()) {
    if (ConfirmationDialog::confirm(tr("Are you sure you want to refresh?"), tr("Refresh"), this)) {
      // Check engaged again in case it changed while the dialog was open
      if (!uiState()->engaged()) {
        params.putBool("OnRoadRefresh", true);
        QTimer::singleShot(3000, [this]() {
          params.putBool("OnRoadRefresh", false);
        });
      }
    }
  } else {
    ConfirmationDialog::alert(tr("Disengage to Refresh"), this);
  }
}

void DevicePanel::reboot() {
  if (!uiState()->engaged()) {
    if (ConfirmationDialog::confirm(tr("Are you sure you want to reboot?"), tr("Reboot"), this)) {
      // Check engaged again in case it changed while the dialog was open
      if (!uiState()->engaged()) {
        params.putBool("DoReboot", true);
      }
    }
  } else {
    ConfirmationDialog::alert(tr("Disengage to Reboot"), this);
  }
}

void DevicePanel::poweroff() {
  if (!uiState()->engaged()) {
    if (ConfirmationDialog::confirm(tr("Are you sure you want to power off?"), tr("Power Off"), this)) {
      // Check engaged again in case it changed while the dialog was open
      if (!uiState()->engaged()) {
        params.putBool("DoShutdown", true);
      }
    }
  } else {
    ConfirmationDialog::alert(tr("Disengage to Power Off"), this);
  }
}

void DevicePanel::showEvent(QShowEvent *event) {
  pair_device->setVisible(uiState()->primeType() == PrimeType::UNPAIRED);
  ListWidget::showEvent(event);
}

SoftwarePanel::SoftwarePanel(QWidget* parent) : ListWidget(parent) {
  gitRemoteLbl = new LabelControl(tr("Git Remote"));
  gitBranchLbl = new LabelControl(tr("Git Branch"));
  gitCommitLbl = new LabelControl(tr("Commit(Local/Remote)"));
  versionLbl = new LabelControl(tr("Fork"));
  lastUpdateLbl = new LabelControl(tr("Last Update Check"), "", "");
  updateBtn = new ButtonControl(tr("Check for Updates"), "");
  connect(updateBtn, &ButtonControl::clicked, [=]() {
    std::system("date '+%F %T' > /data/params/d/LastUpdateTime");
    QString last_ping = QString::fromStdString(params.get("LastAthenaPingTime"));
    QString desc = "";
    QString commit_local = QString::fromStdString(params.get("GitCommit").substr(0, 5));
    QString commit_remote = QString::fromStdString(params.get("GitCommitRemote").substr(0, 5));
    QString commit_local_date = QString::fromStdString(params.get("GitCommitLocalDate"));
    QString commit_remote_date = QString::fromStdString(params.get("GitCommitRemoteDate"));
    QString empty = "";
    desc = tr("LOCAL: %1(%2)  /  REMOTE: %3(%4)").arg(commit_local, commit_local_date, commit_remote, commit_remote_date);
    if (!last_ping.length()) {
      desc = tr("Network connection is missing or unstable. Check the connection.");
      ConfirmationDialog::alert(desc, this);
    } else if (commit_local == commit_remote) {
      params.put("RunCustomCommand", "1", 1);
      desc = tr("Checking update takes a time. If Same message, no update required.");
      ConfirmationDialog::alert(desc, this);
    } else {
      if (QFileInfo::exists("/data/KisaPilot_Updates.txt")) {
        QFileInfo fileInfo;
        fileInfo.setFile("/data/KisaPilot_Updates.txt");
        const std::string txt = util::read_file("/data/KisaPilot_Updates.txt");
        if (UpdateInfoDialog::confirm(desc + "\n" + QString::fromStdString(txt), this)) {
          if (ConfirmationDialog::confirm2(tr("Device will be updated and rebooted. Do you want to proceed?"), this)) {
            std::system("touch /data/kisa_compiling");
            params.put("RunCustomCommand", "2", 1);
          }
        }
      } else {
        QString cmd1 = "wget https://raw.githubusercontent.com/kisapilot/openpilot/"+QString::fromStdString(params.get("GitBranch"))+"/KisaPilot_Updates.txt -O /data/KisaPilot_Updates.txt";
        QProcess::execute(cmd1);
        QTimer::singleShot(2000, []() {});
        if (QFileInfo::exists("/data/KisaPilot_Updates.txt")) {
          QFileInfo fileInfo;
          fileInfo.setFile("/data/KisaPilot_Updates.txt");
          const std::string txt = util::read_file("/data/KisaPilot_Updates.txt");
          if (UpdateInfoDialog::confirm(desc + "\n" + QString::fromStdString(txt), this)) {
            if (ConfirmationDialog::confirm2(tr("Device will be updated and rebooted. Do you want to proceed?"), this)) {
              std::system("touch /data/kisa_compiling");
              params.put("RunCustomCommand", "2", 1);
            }
          }
        }
      }
    }
    updateLabels();
  });

  auto uninstallBtn = new ButtonControl(tr("Uninstall %1").arg(getBrand()), tr("UNINSTALL"));
  connect(uninstallBtn, &ButtonControl::clicked, [&]() {
    if (ConfirmationDialog::confirm2(tr("Are you sure you want to uninstall?"), this)) {
      params.putBool("DoUninstall", true);
    }
  });
  connect(parent, SIGNAL(offroadTransition(bool)), uninstallBtn, SLOT(setEnabled(true)));

  QWidget *widgets[] = {versionLbl, gitRemoteLbl, gitBranchLbl, lastUpdateLbl, updateBtn, gitCommitLbl};
  for (QWidget* w : widgets) {
    addItem(w);
  }

  //addItem(new GitHash());
  addItem(new CPresetWidget());
  addItem(new CGitGroup());
  //addItem(new CUtilWidget(this));

  addItem(uninstallBtn);
}

void SoftwarePanel::showEvent(QShowEvent *event) {
  updateLabels();
}

void SoftwarePanel::updateLabels() {
  QString lastUpdate = "";
  QString tm = QString::fromStdString(params.get("LastUpdateTime").substr(0, 19));
  if (tm != "") {
    lastUpdate = timeAgo(QDateTime::fromString(tm, "yyyy-MM-dd HH:mm:ss"));
  }
  QString lhash = QString::fromStdString(params.get("GitCommit").substr(0, 5));
  QString rhash = QString::fromStdString(params.get("GitCommitRemote").substr(0, 5));
  QString lhash_date = QString::fromStdString(params.get("GitCommitLocalDate"));
  QString rhash_date = QString::fromStdString(params.get("GitCommitRemoteDate"));

  if (lhash == rhash) {
    gitCommitLbl->setStyleSheet("color: #aaaaaa");
  } else {
    gitCommitLbl->setStyleSheet("color: #0099ff");
  }

  versionLbl->setText("KisaPilot");
  lastUpdateLbl->setText(lastUpdate);
  updateBtn->setText(tr("CHECK"));
  updateBtn->setEnabled(true);
  gitRemoteLbl->setText(QString::fromStdString(params.get("GitRemote").substr(19)));
  gitBranchLbl->setText(QString::fromStdString(params.get("GitBranch")));
  gitCommitLbl->setText(lhash + "(" + lhash_date + ")" + " / " + rhash + "(" + rhash_date + ")");
}


UIPanel::UIPanel(QWidget *parent) : QFrame(parent) {
  QVBoxLayout *layout = new QVBoxLayout(this);
  layout->setContentsMargins(50, 0, 50, 0);
  layout->setSpacing(30);

  // kisapilot
  layout->addWidget(new AutoShutdown());
  //layout->addWidget(new ForceShutdown());
  layout->addWidget(new VolumeControl());
  layout->addWidget(new BrightnessControl());
  layout->addWidget(new AutoScreenOff());
  layout->addWidget(new BrightnessOffControl());
  layout->addWidget(new DoNotDisturbMode());  
  //layout->addWidget(new GetOffAlert());
  layout->addWidget(horizontal_line());
  layout->addWidget(new DrivingRecordToggle());
  layout->addWidget(new RecordCount());
  //layout->addWidget(new RecordQuality());
  const char* record_del = "rm -f /data/media/*.mp4";
  auto recorddelbtn = new ButtonControl(tr("Delete All Recorded Files"), tr("RUN"));
  QObject::connect(recorddelbtn, &ButtonControl::clicked, [=]() {
    if (ConfirmationDialog::confirm2(tr("Delete all saved recorded files. Do you want to proceed?"), this)){
      std::system(record_del);
    }
  });
  layout->addWidget(recorddelbtn);
  layout->addWidget(horizontal_line());
  layout->addWidget(new EnableLogger());
  //layout->addWidget(new EnableUploader());
  const char* realdata_del = "rm -rf /data/media/0/realdata/*";
  auto realdatadelbtn = new ButtonControl(tr("Delete All Driving Logs"), tr("RUN"));
  QObject::connect(realdatadelbtn, &ButtonControl::clicked, [=]() {
    if (ConfirmationDialog::confirm2(tr("Delete all saved driving logs. Do you want to proceed?"), this)){
      std::system(realdata_del);
    }
  });
  layout->addWidget(realdatadelbtn);
  layout->addWidget(horizontal_line());
  layout->addWidget(new MonitoringMode());
  layout->addWidget(new MonitorEyesThreshold());
  layout->addWidget(new BlinkThreshold());
  layout->addWidget(horizontal_line());
  layout->addWidget(new KISANaviSelect());
  layout->addWidget(new ExternalDeviceIP());
  //layout->addWidget(new KISAServerSelect());
  //layout->addWidget(new KISAServerAPI());
  //layout->addWidget(new KISAMapboxStyle());
  layout->addWidget(horizontal_line());
  layout->addWidget(new KISABottomTextView());
  layout->addWidget(new RPMAnimatedToggle());
  layout->addWidget(new RPMAnimatedMaxValue());
  layout->addWidget(new LowUIProfile());
  //layout->addWidget(new OSMOfflineUseToggle());
}
DrivingPanel::DrivingPanel(QWidget *parent) : QFrame(parent) {
  QVBoxLayout *layout = new QVBoxLayout(this);
  layout->setContentsMargins(50, 0, 50, 0);
  layout->setSpacing(30);

  // kisapilot
  layout->addWidget(new CResumeGroup());
  layout->addWidget(horizontal_line());
  layout->addWidget(new CCruiseGapGroup());
  layout->addWidget(horizontal_line());
  layout->addWidget(new CVariableCruiseGroup());
  layout->addWidget(horizontal_line());
  layout->addWidget(new CLaneChangeGroup());
  layout->addWidget(horizontal_line());
  layout->addWidget(new CDrivingQuality());
  layout->addWidget(horizontal_line());
  layout->addWidget(new CSafetyandMap());
  layout->addWidget(horizontal_line());
  layout->addWidget(new CSteerWidget());
  layout->addWidget(horizontal_line());

  layout->addWidget(new UseLegacyLaneModel());
}

DeveloperPanel::DeveloperPanel(QWidget *parent) : QFrame(parent) {
  QVBoxLayout *layout = new QVBoxLayout(this);
  layout->setContentsMargins(50, 0, 50, 0);
  layout->setSpacing(30);

  // kisapilot
  layout->addWidget(new DebugUiOneToggle());
  layout->addWidget(new DebugUiTwoToggle());
  layout->addWidget(new DebugUiThreeToggle());
  layout->addWidget(new KISADebug());
  layout->addWidget(new LongLogToggle());
  layout->addWidget(horizontal_line());
  layout->addWidget(new ShowErrorToggle());
  layout->addWidget(new PrebuiltToggle());
  layout->addWidget(horizontal_line());
  layout->addWidget(new LDWSToggle());
  layout->addWidget(new GearDToggle());
  layout->addWidget(new SteerWarningFixToggle());
  layout->addWidget(new IgnoreCanErroronISGToggle());
  layout->addWidget(new NoSmartMDPSToggle());
  layout->addWidget(horizontal_line());
  layout->addWidget(new UFCModeEnabledToggle());
  layout->addWidget(new StockLKASEnabledatDisenagedStatusToggle());
  layout->addWidget(horizontal_line());
  layout->addWidget(new JoystickModeToggle());
  layout->addWidget(new UserSpecificFeature());
  //layout->addWidget(new MapboxToken());

  layout->addWidget(horizontal_line());
  layout->addWidget(new CarSelectCombo());
  layout->addWidget(horizontal_line());
  layout->addWidget(new CPandaGroup());
  layout->addWidget(horizontal_line());
  layout->addWidget(new ModelSelectCombo());
}

TuningPanel::TuningPanel(QWidget *parent) : QFrame(parent) {
  QVBoxLayout *layout = new QVBoxLayout(this);

  layout->setContentsMargins(50, 0, 50, 0);
  layout->setSpacing(30);

  // kisapilot
  //layout->addWidget(new LabelControl(tr("〓〓〓〓〓〓〓〓〓〓〓〓【 TUNING 】〓〓〓〓〓〓〓〓〓〓〓〓"), ""));
  layout->addWidget(new CameraOffset());
  layout->addWidget(new PathOffset());
  layout->addWidget(new SteerAngleCorrection());
  layout->addWidget(horizontal_line());

  layout->addWidget(new SteerActuatorDelay());

  layout->addWidget(new TireStiffnessFactor());
  layout->addWidget(new SteerThreshold());
  layout->addWidget(new SteerLimitTimer());

  layout->addWidget(new LiveSteerRatioToggle());
  layout->addWidget(new LiveSRPercent());
  layout->addWidget(new SRBaseControl());
  //layout->addWidget(new SRMaxControl());

  layout->addWidget(horizontal_line());
  //layout->addWidget(new VariableSteerMaxToggle());
  layout->addWidget(new SteerMax());
  //layout->addWidget(new VariableSteerDeltaToggle());
  layout->addWidget(new SteerDeltaUp());
  layout->addWidget(new SteerDeltaDown());

  layout->addWidget(horizontal_line());

  //layout->addWidget(new LabelControl("〓〓〓〓〓〓〓〓【 CONTROL 】〓〓〓〓〓〓〓〓", ""));
  //layout->addWidget(new LateralControl());
  //layout->addWidget(new LiveTunePanelToggle());

  layout->addWidget(new CLateralControlGroup());
  layout->addWidget(horizontal_line());
  layout->addWidget(new CLongControlGroup());

}


void SettingsWindow::showEvent(QShowEvent *event) {
  setCurrentPanel(0);
}

void SettingsWindow::setCurrentPanel(int index, const QString &param) {
  panel_widget->setCurrentIndex(index);
  nav_btns->buttons()[index]->setChecked(true);
  if (!param.isEmpty()) {
    emit expandToggleDescription(param);
  }
}

SettingsWindow::SettingsWindow(QWidget *parent) : QFrame(parent) {

  // setup two main layouts
  sidebar_widget = new QWidget;
  QVBoxLayout *sidebar_layout = new QVBoxLayout(sidebar_widget);
  sidebar_layout->setMargin(0);
  panel_widget = new QStackedWidget();
  panel_widget->setStyleSheet(R"(
    border-radius: 30px;
    background-color: #292929;
  )");

  // close button
  QPushButton *close_btn = new QPushButton(tr("×"));
  close_btn->setStyleSheet(R"(
    QPushButton {
      font-size: 140px;
      padding-bottom: 20px;
      font-weight: bold;
      border 1px grey solid;
      border-radius: 50px;
      background-color: #292929;
      font-weight: 400;
    }
    QPushButton:pressed {
      background-color: #3B3B3B;
    }
  )");
  close_btn->setFixedSize(220, 130);
  sidebar_layout->addSpacing(5);
  sidebar_layout->addWidget(close_btn, 0, Qt::AlignCenter);
  QObject::connect(close_btn, &QPushButton::clicked, this, &SettingsWindow::closeSettings);

  // setup panels
  DevicePanel *device = new DevicePanel(this);
  SoftwarePanel *software = new SoftwarePanel(this);
  QObject::connect(device, &DevicePanel::reviewTrainingGuide, this, &SettingsWindow::reviewTrainingGuide);
  QObject::connect(device, &DevicePanel::showDriverView, this, &SettingsWindow::showDriverView);

  TogglesPanel *toggles = new TogglesPanel(this);
  QObject::connect(this, &SettingsWindow::expandToggleDescription, toggles, &TogglesPanel::expandToggleDescription);

  QList<QPair<QString, QWidget *>> panels = {
    {tr("Device"), device},
    {tr("Network"), new Networking(this)},
    {tr("Toggles"), toggles},
    {tr("Software"), software},
    {tr("UIMenu"), new UIPanel(this)},
    {tr("Driving"), new DrivingPanel(this)},
    {tr("Developer"), new DeveloperPanel(this)},
    {tr("Tuning"), new TuningPanel(this)},
  };

  sidebar_layout->addSpacing(30);

  const int padding = 0;
  nav_btns = new QButtonGroup(this);
  for (auto &[name, panel] : panels) {
    QPushButton *btn = new QPushButton(name);
    btn->setCheckable(true);
    btn->setChecked(nav_btns->buttons().size() == 0);
    btn->setStyleSheet(QString(R"(
      QPushButton {
        color: grey;
        border: none;
        background: none;
        font-size: 60px;
        font-weight: 500;
        padding-top: %1px;
        padding-bottom: %1px;
      }
      QPushButton:checked {
        color: white;
      }
      QPushButton:pressed {
        color: #ADADAD;
      }
    )").arg(padding));
    btn->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
    nav_btns->addButton(btn);
    sidebar_layout->addWidget(btn, 0, Qt::AlignRight);

    const int lr_margin = name != tr("Network") ? 50 : 0;  // Network panel handles its own margins
    panel->setContentsMargins(lr_margin, 25, lr_margin, 25);

    ScrollView *panel_frame = new ScrollView(panel, this);
    panel_widget->addWidget(panel_frame);

    QObject::connect(btn, &QPushButton::clicked, [=, w = panel_frame]() {
      btn->setChecked(true);
      panel_widget->setCurrentWidget(w);
    });
  }
  sidebar_layout->setContentsMargins(50, 50, 100, 50);

  // main settings layout, sidebar + main panel
  QHBoxLayout *main_layout = new QHBoxLayout(this);

  sidebar_widget->setFixedWidth(500);
  main_layout->addWidget(sidebar_widget);
  main_layout->addWidget(panel_widget);

  setStyleSheet(R"(
    * {
      color: white;
      font-size: 50px;
    }
    SettingsWindow {
      background-color: black;
    }
    QStackedWidget, ScrollView {
      background-color: #292929;
      border-radius: 30px;
    }
  )");
}
