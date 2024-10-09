#pragma once

#include <QPushButton>
#include <QLineEdit>


#include <QComboBox>
#include <QAbstractItemView>
#include <QProcess>

#include "system/hardware/hw.h"
#include "selfdrive/ui/qt/widgets/controls.h"
#include "selfdrive/ui/qt/widgets/groupWidget.h"
#include "selfdrive/ui/ui.h"


class CLateralControlGroup : public CGroupWidget 
{
  Q_OBJECT

public:
  explicit CLateralControlGroup();

  enum TunType {
    LAT_PID = 0,
    LAT_INDI,
    LAT_LQR,
    LAT_TOROUE,
    LAT_MULTI,
    LAT_ALL,
  };  

 private:
  QPushButton  *method_label;
  int    m_nMethod;
  Params params;
  
  void  FramePID(QVBoxLayout *parent=nullptr);
  void  FrameINDI(QVBoxLayout *parent=nullptr);
  void  FrameLQR(QVBoxLayout *parent=nullptr);
  void  FrameTORQUE(QVBoxLayout *parent=nullptr);
  void  FrameMULTI(QVBoxLayout *parent=nullptr);

  

public slots:  
  virtual void refresh(int nID = 0);  
};

class CLongControlGroup : public CGroupWidget 
{
  Q_OBJECT

public:
  explicit CLongControlGroup();

};

class CGitGroup : public CGroupWidget 
{
  Q_OBJECT

public:
  explicit CGitGroup( void *parent=0);
};

class CResumeGroup : public CGroupWidget 
{
  Q_OBJECT

public:
  explicit CResumeGroup( void *parent=0);
};

class CCruiseGapGroup : public CGroupWidget 
{
  Q_OBJECT

public:
  explicit CCruiseGapGroup( void *parent=0);
};

class CVariableCruiseGroup : public CGroupWidget 
{
  Q_OBJECT

public:
  explicit CVariableCruiseGroup( void *parent=0);
};

class CLaneChangeGroup : public CGroupWidget 
{
  Q_OBJECT

public:
  explicit CLaneChangeGroup( void *parent=0);
};

class CDrivingQuality : public CGroupWidget 
{
  Q_OBJECT

public:
  explicit CDrivingQuality( void *parent=0);
};

class CSafetyandMap : public CGroupWidget 
{
  Q_OBJECT

public:
  explicit CSafetyandMap( void *parent=0);
};

class CUtilWidget : public CGroupWidget 
{
  Q_OBJECT

public:
  explicit CUtilWidget( void *parent );
};


class CPresetWidget : public CGroupWidget 
{
  Q_OBJECT

public:
  explicit CPresetWidget();

public slots:  
  virtual void refresh(int nID = 0);

  
};



class SwitchOpenpilot : public ButtonControl {
  Q_OBJECT

public:
  SwitchOpenpilot();

private slots:
  void processFinished(int exitCode, QProcess::ExitStatus exitStatus);

private:
  Params params;

  QString githubid;
  QString githubrepo;
  QString githubbranch;

  QProcess textMsgProcess;

  void refresh();
  void getUserID(const QString &userid);
  void getRepoID(const QString &repoid);
  void getBranchID(const QString &branchid);
};

class OpenpilotUserEnv : public ButtonControl {
  Q_OBJECT

public:
  OpenpilotUserEnv();

private slots:
  void processFinished(int exitCode, QProcess::ExitStatus exitStatus);

private:
  Params params;

  QString githubid;
  QString githubrepo;
  QString githubbranch;
  QString githubfile;

  QProcess textMsgProcess;

  void refresh();
  void getUserID(const QString &userid);
  void getRepoID(const QString &repoid);
  void getBranchID(const QString &branchid);
  void getFileID(const QString &fileid);
};

class AutoResumeToggle : public ToggleControl {
  Q_OBJECT

public:
  AutoResumeToggle() : ToggleControl(tr("Use Auto Resume at Stop"), tr("It uses the automatic departure function when stopping while using SCC."), "../assets/offroad/icon_shell.png", Params().getBool("KisaAutoResume")) {
    QObject::connect(this, &AutoResumeToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("KisaAutoResume", status);
    });
  }
};

class VariableCruiseToggle : public ToggleControl {
  Q_OBJECT

public:
  VariableCruiseToggle() : ToggleControl(tr("Use Cruise Button Spamming"), tr("Use the cruise button while using SCC to assist in acceleration and deceleration."), "../assets/offroad/icon_shell.png", Params().getBool("KisaVariableCruise")) {
    QObject::connect(this, &VariableCruiseToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("KisaVariableCruise", status);
    });
  }
};

class CruiseGapAdjustToggle : public ToggleControl {
  Q_OBJECT

public:
  CruiseGapAdjustToggle() : ToggleControl(tr("Change Cruise Gap at Stop"), tr("For a quick start when stopping, the cruise gap will be changed to 1 step, and after departure, it will return to the original cruise gap according to certain conditions."), "../assets/offroad/icon_shell.png", Params().getBool("CruiseGapAdjust")) {
    QObject::connect(this, &CruiseGapAdjustToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("CruiseGapAdjust", status);
    });
  }
};

class AutoEnabledToggle : public ToggleControl {
  Q_OBJECT

public:
  AutoEnabledToggle() : ToggleControl(tr("Use Auto Engagement"), tr("If the cruise button status is standby (CRUISE indication only and speed is not specified) in the Disengagement state, activate the automatic Engagement."), "../assets/offroad/icon_shell.png", Params().getBool("AutoEnable")) {
    QObject::connect(this, &AutoEnabledToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("AutoEnable", status);
    });
  }
};

class CruiseAutoResToggle : public ToggleControl {
  Q_OBJECT

public:
  CruiseAutoResToggle() : ToggleControl(tr("Use Auto RES while Driving"), tr("If the brake is applied while using the SCC and the standby mode is changed (CANCEL is not applicable), set it back to the previous speed when the brake pedal is released/accelerated pedal is operated. It operates when the cruise speed is set and the vehicle speed is more than 30 km/h or the car in front is recognized."), "../assets/offroad/icon_shell.png", Params().getBool("CruiseAutoRes")) {
    QObject::connect(this, &CruiseAutoResToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("CruiseAutoRes", status);
    });
  }
};

class BatteryChargingControlToggle : public ToggleControl {
  Q_OBJECT

public:
  BatteryChargingControlToggle() : ToggleControl(tr("Enable Battery Charging Control"), tr("It uses the battery charge control function."), "../assets/offroad/icon_shell.png", Params().getBool("KisaBatteryChargingControl")) {
    QObject::connect(this, &BatteryChargingControlToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("KisaBatteryChargingControl", status);
    });
  }
};

class BlindSpotDetectToggle : public ToggleControl {
  Q_OBJECT

public:
  BlindSpotDetectToggle() : ToggleControl(tr("Show BSM Status"), tr("If a car is detected in the rear, it will be displayed on the screen."), "../assets/offroad/icon_shell.png", Params().getBool("KisaBlindSpotDetect")) {
    QObject::connect(this, &BlindSpotDetectToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("KisaBlindSpotDetect", status);
      if (state) {
        uiState()->scene.nKisaBlindSpotDetect = true;
      } else {
        uiState()->scene.nKisaBlindSpotDetect = false;
      }
    });
  }
};

class UFCModeEnabledToggle : public ToggleControl {
  Q_OBJECT

public:
  UFCModeEnabledToggle() : ToggleControl(tr("User-Friendly Control(UFC) Mode"), tr("OP activates with Main Cruise Switch, AutoRES while driving, Seperate Lat/Long and etc"), "../assets/offroad/icon_shell.png", Params().getBool("UFCModeEnabled")) {
    QObject::connect(this, &UFCModeEnabledToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("UFCModeEnabled", status);
    });
  }
};

class WhitePandaSupportToggle : public ToggleControl {
  Q_OBJECT

public:
  WhitePandaSupportToggle() : ToggleControl(tr("Support WhitePanda"), tr("Turn on this function if you use WhitePanda."), "../assets/offroad/icon_shell.png", Params().getBool("WhitePandaSupport")) {
    QObject::connect(this, &WhitePandaSupportToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("WhitePandaSupport", status);
    });
  }
};

class SteerWarningFixToggle : public ToggleControl {
  Q_OBJECT

public:
  SteerWarningFixToggle() : ToggleControl(tr("Ignore of Steering Warning"), tr("Turn on the function when a steering error occurs in the vehicle and the open pilot cannot be executed (some vehicles only). Do not turn on the function if it occurs in a normal error environment while driving."), "../assets/offroad/icon_shell.png", Params().getBool("SteerWarningFix")) {
    QObject::connect(this, &SteerWarningFixToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("SteerWarningFix", status);
    });
  }
};

class LiveSteerRatioToggle : public ToggleControl {
  Q_OBJECT

public:
  LiveSteerRatioToggle() : ToggleControl(tr("Use Live SteerRatio"), tr("Live SteerRatio is used instead of variable/fixed SteerRatio."), "../assets/offroad/icon_shell.png", Params().getBool("KisaLiveSteerRatio")) {
    QObject::connect(this, &LiveSteerRatioToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("KisaLiveSteerRatio", status);
    });
  }
};

class DrivingRecordToggle : public ToggleControl {
  Q_OBJECT

public:
  DrivingRecordToggle() : ToggleControl(tr("Use Auto Screen Record"), tr("Automatically record/stop the screen while driving. Recording begins after departure, and recording ends when the vehicle stops."), "../assets/offroad/icon_shell.png", Params().getBool("KisaDrivingRecord")) {
    QObject::connect(this, &DrivingRecordToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("KisaDrivingRecord", status);
      if (state) {
        uiState()->scene.driving_record = true;
      } else {
        uiState()->scene.driving_record = false;
      }
    });
  }
};

class TurnSteeringDisableToggle : public ToggleControl {
  Q_OBJECT

public:
  TurnSteeringDisableToggle() : ToggleControl(tr("Stop Steer Assist on Turn Signals"), tr("When driving below the lane change speed, the automatic steering is temporarily paused while the turn signals on."), "../assets/offroad/icon_shell.png", Params().getBool("KisaTurnSteeringDisable")) {
    QObject::connect(this, &TurnSteeringDisableToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("KisaTurnSteeringDisable", status);
    });
  }
};

class HotspotOnBootToggle : public ToggleControl {
  Q_OBJECT

public:
  HotspotOnBootToggle() : ToggleControl(tr("HotSpot on Boot"), tr("It automatically runs a hotspot when booting."), "", Params().getBool("KisaHotspotOnBoot")) {
    QObject::connect(this, &HotspotOnBootToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("KisaHotspotOnBoot", status);
    });
  }
};

class CruiseOverMaxSpeedToggle : public ToggleControl {
  Q_OBJECT

public:
  CruiseOverMaxSpeedToggle() : ToggleControl(tr("Reset MaxSpeed Over CurrentSpeed"), tr("If the current speed exceeds the set speed, synchronize the set speed with the current speed."), "../assets/offroad/icon_shell.png", Params().getBool("CruiseOverMaxSpeed")) {
    QObject::connect(this, &CruiseOverMaxSpeedToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("CruiseOverMaxSpeed", status);
    });
  }
};

class DebugUiOneToggle : public ToggleControl {
  Q_OBJECT

public:
  DebugUiOneToggle() : ToggleControl(tr("DEBUG UI 1"), "", "../assets/offroad/icon_shell.png", Params().getBool("DebugUi1")) {
    QObject::connect(this, &DebugUiOneToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("DebugUi1", status);
      if (state) {
        uiState()->scene.nDebugUi1 = true;
      } else {
        uiState()->scene.nDebugUi1 = false;
      }
    });
  }
};

class DebugUiTwoToggle : public ToggleControl {
  Q_OBJECT

public:
  DebugUiTwoToggle() : ToggleControl(tr("DEBUG UI 2"), "", "../assets/offroad/icon_shell.png", Params().getBool("DebugUi2")) {
    QObject::connect(this, &DebugUiTwoToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("DebugUi2", status);
      if (state) {
        uiState()->scene.nDebugUi2 = true;
      } else {
        uiState()->scene.nDebugUi2 = false;
      }
    });
  }
};

class DebugUiThreeToggle : public ToggleControl {
  Q_OBJECT

public:
  DebugUiThreeToggle() : ToggleControl(tr("DEBUG UI 3"), "", "../assets/offroad/icon_shell.png", Params().getBool("DebugUi3")) {
    QObject::connect(this, &DebugUiThreeToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("DebugUi3", status);
      if (state) {
        uiState()->scene.nDebugUi3 = true;
      } else {
        uiState()->scene.nDebugUi3 = false;
      }
    });
  }
};

class LongLogToggle : public ToggleControl {
  Q_OBJECT

public:
  LongLogToggle() : ToggleControl(tr("Show LongControl LOG"), tr("Display logs for long tuning debugs instead of variable cruise logs on the screen."), "../assets/offroad/icon_shell.png", Params().getBool("LongLogDisplay")) {
    QObject::connect(this, &LongLogToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("LongLogDisplay", status);
    });
  }
};

class PrebuiltToggle : public ToggleControl {
  Q_OBJECT

public:
  PrebuiltToggle() : ToggleControl(tr("Use Smart Prebuilt"), tr("Create a Prebuilt file and speed up booting. When this function is turned on, the booting speed is accelerated using the cache, and if you press the update button in the menu after modifying the code, or if you rebooted with the 'gi' command in the command window, remove it automatically and compile it."), "../assets/offroad/icon_shell.png", Params().getBool("PutPrebuiltOn")) {
    QObject::connect(this, &PrebuiltToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("PutPrebuiltOn", status);
      if (state) {
        std::system("rm -f /data/openpilot/prebuilt");
      }
    });
  }
};

class LDWSToggle : public ToggleControl {
  Q_OBJECT

public:
  LDWSToggle() : ToggleControl(tr("Set LDWS Vehicles"), "", "../assets/offroad/icon_shell.png", Params().getBool("LdwsCarFix")) {
    QObject::connect(this, &LDWSToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("LdwsCarFix", status);
    });
  }
};

class GearDToggle : public ToggleControl {
  Q_OBJECT

public:
  GearDToggle() : ToggleControl(tr("Set DriverGear by Force"), tr("It is used when the gear recognition problem. Basically, CABANA data should be analyzed, but it is temporarily resolved."), "../assets/offroad/icon_shell.png", Params().getBool("JustDoGearD")) {
    QObject::connect(this, &GearDToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("JustDoGearD", status);
    });
  }
};

class RunNaviOnBootToggle : public ToggleControl {
  Q_OBJECT

public:
  RunNaviOnBootToggle() : ToggleControl(tr("Run Navigation on Boot"), tr("Automatically execute the navigation (waze) when switching to the driving screen after booting."), "../assets/offroad/icon_shell.png", Params().getBool("KisaRunNaviOnBoot")) {
    QObject::connect(this, &RunNaviOnBootToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("KisaRunNaviOnBoot", status);
    });
  }
};

class BattLessToggle : public ToggleControl {
  Q_OBJECT

public:
  BattLessToggle() : ToggleControl(tr("Set BatteryLess Device"), tr("This is a toggle for batteryless Device. Related settings apply."), "../assets/offroad/icon_shell.png", Params().getBool("KisaBattLess")) {
    QObject::connect(this, &BattLessToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("KisaBattLess", status);
    });
  }
};

class LiveTunePanelToggle : public ToggleControl {
  Q_OBJECT

public:
  LiveTunePanelToggle() : ToggleControl(tr("Use LiveTune and Show UI"), tr("Display the UI related to live tuning on the screen. Various tuning values can be adjusted live on the driving screen. It is reflected in the parameter when adjusting, and the value is maintained even after turning off the toggle and rebooting."), "../assets/offroad/icon_shell.png", Params().getBool("KisaLiveTunePanelEnable")) {
    QObject::connect(this, &LiveTunePanelToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("KisaLiveTunePanelEnable", status);
      if (state) {
        uiState()->scene.live_tune_panel_enable = true;
        uiState()->scene.kisa_livetune_ui = true;
      } else {
        uiState()->scene.live_tune_panel_enable = false;
        uiState()->scene.kisa_livetune_ui = false;
      }
    });
  }
};

class GitPullOnBootToggle : public ToggleControl {
  Q_OBJECT

public:
  GitPullOnBootToggle() : ToggleControl(tr("Git Pull On Boot"), tr("If there is an update after the boot, run Git Pull automatically and reboot."), "", Params().getBool("GitPullOnBoot")) {
    QObject::connect(this, &GitPullOnBootToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("GitPullOnBoot", status);
    });
  }
};

class StoppingDistAdjToggle : public ToggleControl {
  Q_OBJECT

public:
  StoppingDistAdjToggle() : ToggleControl(tr("Adjust Stopping Distance"), tr("Stop a little further ahead than the radar stop distance. If you approach the car in front of you at a high speed, it may sometimes be difficult to stop enough, so if you are uncomfortable, turn off the function."), "../assets/offroad/icon_shell.png", Params().getBool("StoppingDistAdj")) {
    QObject::connect(this, &StoppingDistAdjToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("StoppingDistAdj", status);
    });
  }
};

class ShowErrorToggle : public ToggleControl {
  Q_OBJECT

public:
  ShowErrorToggle() : ToggleControl(tr("Show TMUX Error"), tr("Display the error on the Device screen when a process error occurs while driving or off-road."), "../assets/offroad/icon_shell.png", Params().getBool("ShowError")) {
    QObject::connect(this, &ShowErrorToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("ShowError", status);
      if (state) {
        uiState()->scene.show_error = true;
      } else {
        uiState()->scene.show_error = false;
      }
    });
  }
};

class StockNaviSpeedToggle : public ToggleControl {
  Q_OBJECT

public:
  StockNaviSpeedToggle() : ToggleControl(tr("Use Stock SafetyCAM Speed"), tr("When decelerating the safety section, use the safety speed from the vehicle navigation system (limited to some vehicles with the corresponding data)."), "../assets/offroad/icon_shell.png", Params().getBool("StockNaviSpeedEnabled")) {
    QObject::connect(this, &StockNaviSpeedToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("StockNaviSpeedEnabled", status);
    });
  }
};

class E2ELongToggle : public ToggleControl {
  Q_OBJECT

public:
  E2ELongToggle() : ToggleControl(tr("Enable E2E Long"), tr("Activate E2E Long. It may work unexpectedly. Be careful."), "../assets/offroad/icon_shell.png", Params().getBool("E2ELong")) {
    QObject::connect(this, &E2ELongToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("E2ELong", status);
    });
  }
};

class StopAtStopSignToggle : public ToggleControl {
  Q_OBJECT

public:
  StopAtStopSignToggle() : ToggleControl(tr("Stop at Stop Sign"), tr("Openpilot tries to stop at stop sign depends on Model."), "../assets/offroad/icon_shell.png", Params().getBool("StopAtStopSign")) {
    QObject::connect(this, &StopAtStopSignToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("StopAtStopSign", status);
    });
  }
};

class GoogleMapEnabledToggle : public ToggleControl {
  Q_OBJECT

public:
  GoogleMapEnabledToggle() : ToggleControl(tr("Use GoogleMap for Mapbox"), tr("Use GoogleMap when you search a destination."), "../assets/offroad/icon_shell.png", Params().getBool("GoogleMapEnabled")) {
    QObject::connect(this, &GoogleMapEnabledToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("GoogleMapEnabled", status);
    });
  }
};

class OSMEnabledToggle : public ToggleControl {
  Q_OBJECT

public:
  OSMEnabledToggle() : ToggleControl(tr("Enable OSM"), tr("This enables OSM."), "../assets/offroad/icon_shell.png", Params().getBool("OSMEnable")) {
    QObject::connect(this, &OSMEnabledToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("OSMEnable", status);
    });
  }
};

class OSMSpeedLimitEnabledToggle : public ToggleControl {
  Q_OBJECT

public:
  OSMSpeedLimitEnabledToggle() : ToggleControl(tr("Enable OSM SpeedLimit"), tr("This enables OSM speedlimit."), "../assets/offroad/icon_shell.png", Params().getBool("OSMSpeedLimitEnable")) {
    QObject::connect(this, &OSMSpeedLimitEnabledToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("OSMSpeedLimitEnable", status);
    });
  }
};

class IgnoreCanErroronISGToggle : public ToggleControl {
  Q_OBJECT

public:
  IgnoreCanErroronISGToggle() : ToggleControl(tr("Ignore Can Error on ISG"), tr("Turn this on, if can error occurs on ISG operation."), "../assets/offroad/icon_shell.png", Params().getBool("IgnoreCANErroronISG")) {
    QObject::connect(this, &IgnoreCanErroronISGToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("IgnoreCANErroronISG", status);
    });
  }
};

class StockLKASEnabledatDisenagedStatusToggle : public ToggleControl {
  Q_OBJECT

public:
  StockLKASEnabledatDisenagedStatusToggle() : ToggleControl(tr("StockLKAS Enabled at Disengagement"), tr("Turn this on, if you want to use Stock LKAS at OP disengaged status. Seems this related to cluster error when OP active because Stock CAN messages over PANDA or not."), "../assets/offroad/icon_shell.png", Params().getBool("StockLKASEnabled")) {
    QObject::connect(this, &StockLKASEnabledatDisenagedStatusToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("StockLKASEnabled", status);
    });
  }
};

class StandstillResumeAltToggle : public ToggleControl {
  Q_OBJECT

public:
  StandstillResumeAltToggle() : ToggleControl(tr("Standstill Resume Alternative"), tr("Turn this on, if auto resume doesn't work at standstill. some cars only(ex. GENESIS). before enable, try to adjust RES message counts above.(reboot required)"), "../assets/offroad/icon_shell.png", Params().getBool("StandstillResumeAlt")) {
    QObject::connect(this, &StandstillResumeAltToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("StandstillResumeAlt", status);
    });
  }
};

class MapboxEnabledToggle : public ToggleControl {
  Q_OBJECT

public:
  MapboxEnabledToggle() : ToggleControl(tr("Enable Mapbox"), tr("If you want to use Mapbox, turn on and then connect to device using web browser http://(device ip):8082  Mapbox setting will show up and type mapbox pk and sk token(you can created this on mapbox.com website). If you want to search destinations with googlemap, first, you should create google api key and enable Enable GoogleMap for Mapbox"), "../assets/offroad/icon_shell.png", Params().getBool("MapboxEnabled")) {
    QObject::connect(this, &MapboxEnabledToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("MapboxEnabled", status);
    });
  }
};

class UseRadarTrackToggle : public ToggleControl {
  Q_OBJECT

public:
  UseRadarTrackToggle() : ToggleControl(tr("Use Radar Track"), tr("Some cars have known radar tracks(from comma) for long control. This uses radar track directly instead of scc can message. Before you go, you must need to run hyundai_enable_radar_points.py in /data/openpilot/selfdrive/debug dir to enable your radar track. (Reboot required)"), "../assets/offroad/icon_shell.png", Params().getBool("UseRadarTrack")) {
    QObject::connect(this, &UseRadarTrackToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("UseRadarTrack", status);
    });
  }
};

class UseRadarValue : public ToggleControl {
  Q_OBJECT

public:
  UseRadarValue() : ToggleControl(tr("Use Radar for lead car"), tr("If the radar detects a lead car, the device uses radar values (distance, relative velocity, etc.) because vision can sometimes be inaccurate."), "../assets/offroad/icon_shell.png", Params().getBool("UseRadarValue")) {
    QObject::connect(this, &UseRadarValue::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("UseRadarValue", status);
    });
  }
};

class RadarDisableToggle : public ToggleControl {
  Q_OBJECT

public:
  RadarDisableToggle() : ToggleControl(tr("Disable Radar"), tr("This is pre-requisite for LongControl of HKG. It seems that this affects AEB. So do not use this if you have any concern."), "../assets/offroad/icon_shell.png", Params().getBool("RadarDisable")) {
    QObject::connect(this, &RadarDisableToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("RadarDisable", status);
    });
  }
};

class C2WithCommaPowerToggle : public ToggleControl {
  Q_OBJECT

public:
  C2WithCommaPowerToggle() : ToggleControl(tr("C2 with CommaPower"), tr("This is for C2 users with Comma Power."), "../assets/offroad/icon_shell.png", Params().getBool("C2WithCommaPower")) {
    QObject::connect(this, &C2WithCommaPowerToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("C2WithCommaPower", status);
    });
  }
};

class CustomTRToggle : public ToggleControl {
  Q_OBJECT

public:
  CustomTRToggle() : ToggleControl(tr("Custom TR Enable"), tr("to use Custom TR not 1.45(comma default)."), "../assets/offroad/icon_shell.png", Params().getBool("CustomTREnabled")) {
    QObject::connect(this, &CustomTRToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("CustomTREnabled", status);
    });
  }
};

class RoutineDriveOnToggle : public ToggleControl {
  Q_OBJECT

public:
  RoutineDriveOnToggle() : ToggleControl(tr("Routine Drive by RoadName"), tr("This will adjust useful things by roadname. If you want to use, edit the file, /data/params/d/RoadList. modify like this RoadName1,offset1(ex:+0.05),RoadName2,offset2(ex:-0.05),... and the second line RoadName3,speedlimit(ex:30),RoadName4,speedlimit(ex:60),..."), "../assets/offroad/icon_shell.png", Params().getBool("RoutineDriveOn")) {
    QObject::connect(this, &RoutineDriveOnToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("RoutineDriveOn", status);
    });
  }
};

class CloseToRoadEdgeToggle : public ToggleControl {
  Q_OBJECT

public:
  CloseToRoadEdgeToggle() : ToggleControl(tr("Driving Close to RoadEdge"), tr("This will adjust the camera offset to get close to road edge if the car is on the first or last lane."), "../assets/offroad/icon_shell.png", Params().getBool("CloseToRoadEdge")) {
    QObject::connect(this, &CloseToRoadEdgeToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("CloseToRoadEdge", status);
    });
  }
};

class ToAvoidLKASFaultToggle : public ToggleControl {
  Q_OBJECT

public:
  ToAvoidLKASFaultToggle() : ToggleControl(tr("To Avoid LKAS Fault"), tr("to avoid LKAS fault above max angle limit(car specific). This is live value. Find out your maxframe while driving."), "../assets/offroad/icon_shell.png", Params().getBool("AvoidLKASFaultEnabled")) {
    QObject::connect(this, &ToAvoidLKASFaultToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("AvoidLKASFaultEnabled", status);
    });
  }
};

class ToAvoidLKASFaultBeyondToggle : public ToggleControl {
  Q_OBJECT

public:
  ToAvoidLKASFaultBeyondToggle() : ToggleControl(tr("To Avoid LKAS Fault with More Steer"), tr("This is just in case you are using other panda setting.(delta updown, maxsteer, rtdelta and etc)."), "../assets/offroad/icon_shell.png", Params().getBool("AvoidLKASFaultBeyond")) {
    QObject::connect(this, &ToAvoidLKASFaultBeyondToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("AvoidLKASFaultBeyond", status);
    });
  }
};

class StockDecelonCamToggle : public ToggleControl {
  Q_OBJECT

public:
  StockDecelonCamToggle() : ToggleControl(tr("Use Stock Decel on SaftySection"), tr("Use stock deceleration on safety section.(the vehicle equipped with Stock Navigation)"), "../assets/offroad/icon_shell.png", Params().getBool("UseStockDecelOnSS")) {
    QObject::connect(this, &StockDecelonCamToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("UseStockDecelOnSS", status);
    });
  }
};

class RPMAnimatedToggle : public ToggleControl {
  Q_OBJECT

public:
  RPMAnimatedToggle() : ToggleControl(tr("RPM Animated"), tr("Show Animated RPM"), "../assets/offroad/icon_shell.png", Params().getBool("AnimatedRPM")) {
    QObject::connect(this, &RPMAnimatedToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("AnimatedRPM", status);
    });
  }
};

class NoSmartMDPSToggle : public ToggleControl {
  Q_OBJECT

public:
  NoSmartMDPSToggle() : ToggleControl(tr("No Smart MDPS"), tr("Turn on, if you have no smartmdps or no mdps harness to avoid sending can under certain speed that is not able to use lane keeping."), "../assets/offroad/icon_shell.png", Params().getBool("NoSmartMDPS")) {
    QObject::connect(this, &NoSmartMDPSToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("NoSmartMDPS", status);
    });
  }
};

class SpeedCameraOffsetToggle : public ToggleControl {
  Q_OBJECT

public:
  SpeedCameraOffsetToggle() : ToggleControl(tr("Speed CameraOffset"), tr("This increase offset at low speed and decrease offset at low speed. If you feel car moves to right at low speed."), "../assets/offroad/icon_shell.png", Params().getBool("SpeedCameraOffset")) {
    QObject::connect(this, &SpeedCameraOffsetToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("SpeedCameraOffset", status);
    });
  }
};

class SpeedBumpDecelToggle : public ToggleControl {
  Q_OBJECT

public:
  SpeedBumpDecelToggle() : ToggleControl(tr("SpeedBump Deceleration"), tr("Use the deceleration feature on the speed bump. It's an indirect control method. It can be decelerated directly in long control control, but for versatility, indirect control for now."), "../assets/offroad/icon_shell.png", Params().getBool("KISASpeedBump")) {
    QObject::connect(this, &SpeedBumpDecelToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("KISASpeedBump", status);
    });
  }
};

class KISAEarlyStoppingToggle : public ToggleControl {
  Q_OBJECT

public:
  KISAEarlyStoppingToggle() : ToggleControl(tr("Early Slowdown with Gap"), tr("This feature may help your vehicle to stop early using Cruise Gap with value 4 when your car start to stop from model."), "../assets/offroad/icon_shell.png", Params().getBool("KISAEarlyStop")) {
    QObject::connect(this, &KISAEarlyStoppingToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("KISAEarlyStop", status);
    });
  }
};

class TorqueUseAngle : public ToggleControl {
  Q_OBJECT

public:
  TorqueUseAngle() : ToggleControl(tr("UseAngle"), tr("Use Steer Angle On/Off"), "../assets/offroad/icon_shell.png", Params().getBool("TorqueUseAngle")) {
    QObject::connect(this, &TorqueUseAngle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("TorqueUseAngle", status);
    });
  }
};

class TorqueUseLiveFriction : public ToggleControl {
  Q_OBJECT

public:
  TorqueUseLiveFriction() : ToggleControl(tr("Use LiveTorque"), tr("Use Live Torque"), "../assets/offroad/icon_shell.png", Params().getBool("KisaLiveTorque")) {
    QObject::connect(this, &TorqueUseLiveFriction::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("KisaLiveTorque", status);
    });
  }
};

class DepartChimeAtResume : public ToggleControl {
  Q_OBJECT

public:
  DepartChimeAtResume() : ToggleControl(tr("Depart Chime at Resume"), tr("Use Chime for Resume. This can notify for you to get start while not using SCC."), "../assets/offroad/icon_shell.png", Params().getBool("DepartChimeAtResume")) {
    QObject::connect(this, &DepartChimeAtResume::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("DepartChimeAtResume", status);
    });
  }
};

class CruiseGapBySpdOn : public ToggleControl {
  Q_OBJECT

public:
  CruiseGapBySpdOn() : ToggleControl(tr("Cruise Gap Change by Speed"), tr("Cruise Gap is changeable by vehicle speed."), "../assets/offroad/icon_shell.png", Params().getBool("CruiseGapBySpdOn")) {
    QObject::connect(this, &CruiseGapBySpdOn::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("CruiseGapBySpdOn", status);
    });
  }
};

class CruiseSetwithRoadLimitSpeed : public ToggleControl {
  Q_OBJECT

public:
  CruiseSetwithRoadLimitSpeed() : ToggleControl(tr("CruiseSet with RoadLimitSpeed"), tr("Cruise Set with RoadLimitSpeed(Ext Navi)"), "../assets/offroad/icon_shell.png", Params().getBool("CruiseSetwithRoadLimitSpeedEnabled")) {
    QObject::connect(this, &CruiseSetwithRoadLimitSpeed::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("CruiseSetwithRoadLimitSpeedEnabled", status);
    });
  }
};

class KISADebug : public ToggleControl {
  Q_OBJECT

public:
  KISADebug() : ToggleControl(tr("KISA Debug Mode"), tr("Run KISA Debug Mode"), "../assets/offroad/icon_shell.png", Params().getBool("KISADebug")) {
    QObject::connect(this, &KISADebug::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("KISADebug", status);
      if (state) {
        uiState()->scene.KISA_Debug = true;
      } else {
        uiState()->scene.KISA_Debug = false;
      }
    });
  }
};

class LowUIProfile : public ToggleControl {
  Q_OBJECT

public:
  LowUIProfile() : ToggleControl(tr("Low UI Profile"), tr("Low UI Profile to get UI more visible at bottom side."), "../assets/offroad/icon_shell.png", Params().getBool("LowUIProfile")) {
    QObject::connect(this, &LowUIProfile::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("LowUIProfile", status);
      if (state) {
        uiState()->scene.low_ui_profile = true;
      } else {
        uiState()->scene.low_ui_profile = false;
      }
    });
  }
};

class EnableLogger : public ToggleControl {
  Q_OBJECT

public:
  EnableLogger() : ToggleControl(tr("Enable Driving Log Record"), tr("Record the driving log locally for data analysis. Only loggers are activated and not uploaded to the server."), "../assets/offroad/icon_shell.png", Params().getBool("KisaEnableLogger")) {
    QObject::connect(this, &EnableLogger::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("KisaEnableLogger", status);
    });
  }
};

class EnableUploader : public ToggleControl {
  Q_OBJECT

public:
  EnableUploader() : ToggleControl(tr("Enable Sending Log to Server"), tr("Activate the upload process to transmit system logs and other driving data to the server. Upload it only off-road."), "../assets/offroad/icon_shell.png", Params().getBool("KisaEnableUploader")) {
    QObject::connect(this, &EnableUploader::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("KisaEnableUploader", status);
    });
  }
};

class RegenBrakeFeatureToggle : public ToggleControl {
  Q_OBJECT

public:
  RegenBrakeFeatureToggle() : ToggleControl(tr("Use RegenBrake Feature"), tr("Advanced regeneration brake features. ST: full stop, AT: deceleration level adjustment with distance, EE: E2E longitudinal assist"), "../assets/offroad/icon_shell.png", Params().getBool("RegenBrakeFeatureOn")) {
    QObject::connect(this, &RegenBrakeFeatureToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("RegenBrakeFeatureOn", status);
    });
  }
};

// openpilot preview
class OpenpilotView : public AbstractControl {
  Q_OBJECT

public:
  OpenpilotView();

private:
  QPushButton btn;
  QPushButton btnc;
  Params params;
  
  void refresh();
};

class CarSelectCombo : public AbstractControl 
{
  Q_OBJECT

public:
  CarSelectCombo();

private:
  QPushButton btn1;
  QPushButton btn2;
  Params params;

  void refresh();
};

class ModelSelectCombo : public AbstractControl 
{
  Q_OBJECT

public:
  ModelSelectCombo();

private:
  QLabel label;
  QPushButton btn1;
  QPushButton btn2;
  Params params;
  QString selection;

  void refresh();
};

class BranchSelectCombo : public AbstractControl 
{
  Q_OBJECT

public:
  BranchSelectCombo();

private:
  QPushButton btn1;
  QPushButton btn2;
  Params params;

  QString selection;
  QStringList stringList;
};

class TimeZoneSelectCombo : public AbstractControl 
{
  Q_OBJECT

public:
  TimeZoneSelectCombo();

private:
  QPushButton btn;
  QComboBox combobox;
  Params params;

  void refresh();
};


// UI
class AutoShutdown : public AbstractControl {
  Q_OBJECT

public:
  AutoShutdown();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class AutoScreenOff : public AbstractControl {
  Q_OBJECT

public:
  AutoScreenOff();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class VolumeControl : public AbstractControl {
  Q_OBJECT

public:
  VolumeControl();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class BrightnessControl : public AbstractControl {
  Q_OBJECT

public:
  BrightnessControl();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class BrightnessOffControl : public AbstractControl {
  Q_OBJECT

public:
  BrightnessOffControl();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};


class ChargingMin : public AbstractControl {
  Q_OBJECT

public:
  ChargingMin();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};
class ChargingMax : public AbstractControl {
  Q_OBJECT

public:
  ChargingMax();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};


// Driving
class CruisemodeSelInit : public AbstractControl {
  Q_OBJECT

public:
  CruisemodeSelInit();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class VariableCruiseProfile : public AbstractControl {
  Q_OBJECT

public:
  VariableCruiseProfile();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class LaneChangeSpeed : public AbstractControl {
  Q_OBJECT

public:
  LaneChangeSpeed();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class LaneChangeDelay : public AbstractControl {
  Q_OBJECT

public:
  LaneChangeDelay();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class LeftCurvOffset : public AbstractControl {
  Q_OBJECT

public:
  LeftCurvOffset();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};
class RightCurvOffset : public AbstractControl {
  Q_OBJECT

public:
  RightCurvOffset();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};



class SpeedLimitOffset : public AbstractControl {
  Q_OBJECT

public:
  SpeedLimitOffset();

private:
  QPushButton btn;
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

// 튜닝 설정
class CameraOffset : public AbstractControl {
  Q_OBJECT

public:
  CameraOffset();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class PathOffset : public AbstractControl {
  Q_OBJECT

public:
  PathOffset();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class SRBaseControl : public AbstractControl {
  Q_OBJECT

public:
  SRBaseControl();

private:
  QPushButton btndigit;
  QPushButton btnminus;
  QPushButton btnplus;
  QLabel label;
  Params params;
  float digit = 0.01;
  
  void refresh();
};

class SteerActuatorDelay : public AbstractControl {
  Q_OBJECT

public:
  SteerActuatorDelay();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

// class SteerRateCost : public AbstractControl {
//   Q_OBJECT

// public:
//   SteerRateCost();

// private:
//   QPushButton btnplus;
//   QPushButton btnminus;
//   QLabel label;
//   Params params;
  
//   void refresh();
// };

class SteerLimitTimer : public AbstractControl {
  Q_OBJECT

public:
  SteerLimitTimer();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class TireStiffnessFactor : public AbstractControl {
  Q_OBJECT

public:
  TireStiffnessFactor();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class SteerMax : public AbstractControl {
  Q_OBJECT

public:
  SteerMax();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class SteerDeltaUp : public AbstractControl {
  Q_OBJECT

public:
  SteerDeltaUp();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class SteerDeltaDown : public AbstractControl {
  Q_OBJECT

public:
  SteerDeltaDown();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

// control
class LateralControl : public AbstractControl {
  Q_OBJECT

public:
  LateralControl();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  int latcontrol;


  void refresh();
};

class PidKp : public AbstractControl {
  Q_OBJECT

public:
  PidKp();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class PidKi : public AbstractControl {
  Q_OBJECT

public:
  PidKi();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class PidKd : public AbstractControl {
  Q_OBJECT

public:
  PidKd();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class PidKf : public AbstractControl {
  Q_OBJECT

public:
  PidKf();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class IgnoreZone : public AbstractControl {
  Q_OBJECT

public:
  IgnoreZone();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class OuterLoopGain : public AbstractControl {
  Q_OBJECT

public:
  OuterLoopGain();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class InnerLoopGain : public AbstractControl {
  Q_OBJECT

public:
  InnerLoopGain();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class TimeConstant : public AbstractControl {
  Q_OBJECT

public:
  TimeConstant();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class ActuatorEffectiveness : public AbstractControl {
  Q_OBJECT

public:
  ActuatorEffectiveness();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class Scale : public AbstractControl {
  Q_OBJECT

public:
  Scale();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class LqrKi : public AbstractControl {
  Q_OBJECT

public:
  LqrKi();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class DcGain : public AbstractControl {
  Q_OBJECT

public:
  DcGain();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class TorqueKp : public AbstractControl {
  Q_OBJECT

public:
  TorqueKp();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class TorqueKf : public AbstractControl {
  Q_OBJECT

public:
  TorqueKf();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class TorqueKi : public AbstractControl {
  Q_OBJECT

public:
  TorqueKi();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class TorqueFriction : public AbstractControl {
  Q_OBJECT

public:
  TorqueFriction();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class TorqueMaxLatAccel : public AbstractControl {
  Q_OBJECT

public:
  TorqueMaxLatAccel();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class TorqueAngDeadZone : public AbstractControl {
  Q_OBJECT

public:
  TorqueAngDeadZone();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class SteerAngleCorrection : public AbstractControl {
  Q_OBJECT

public:
  SteerAngleCorrection();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class SteerThreshold : public AbstractControl {
  Q_OBJECT

public:
  SteerThreshold();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class RecordCount : public AbstractControl {
  Q_OBJECT

public:
  RecordCount();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class GitHash : public AbstractControl {
  Q_OBJECT

public:
  GitHash();

private:
  QLabel local_hash;
  QLabel remote_hash;
  Params params;
};

class RESChoice : public AbstractControl {
  Q_OBJECT

public:
  RESChoice();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class MonitoringMode : public AbstractControl {
  Q_OBJECT

public:
  MonitoringMode();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class MonitorEyesThreshold : public AbstractControl {
  Q_OBJECT

public:
  MonitorEyesThreshold();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class NormalEyesThreshold : public AbstractControl {
  Q_OBJECT

public:
  NormalEyesThreshold();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class BlinkThreshold : public AbstractControl {
  Q_OBJECT

public:
  BlinkThreshold();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class FanSpeedGain : public AbstractControl {
  Q_OBJECT

public:
  FanSpeedGain();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class CruiseGapTR : public AbstractControl {
  Q_OBJECT

public:
  CruiseGapTR();

private:
  QPushButton btn1;
  QPushButton btn2;
  QPushButton btn3;
  QPushButton btn4;
  QLabel label1;
  QLabel label2;
  QLabel label3;
  QLabel label4;
  QLabel label1a;
  QLabel label2a;
  QLabel label3a;
  QLabel label4a;
  Params params;
  
  void refresh1();
  void refresh2();
  void refresh3();
  void refresh4();
};

class DynamicTRGap : public AbstractControl {
  Q_OBJECT

public:
  DynamicTRGap();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class DynamicTRUD : public AbstractControl {
  Q_OBJECT

public:
  DynamicTRUD();
};

class LCTimingFactor : public AbstractControl {
  Q_OBJECT

public:
  LCTimingFactor();

private:
  QPushButton btn1;
  QPushButton btn2;
  QPushButton btn3;
  QPushButton btn4;
  QLabel label1;
  QLabel label2;
  QLabel label3;
  QLabel label4;
  QLabel label1a;
  QLabel label2a;
  QLabel label3a;
  QLabel label4a;
  Params params;

  void refresh1();
  void refresh2();
  void refresh3();
  void refresh4();
};

class LCTimingFactorUD : public AbstractControl {
  Q_OBJECT

public:
  LCTimingFactorUD();

private:
  QPushButton btn;
  QPushButton btn2;
  Params params;
  
  void refresh();
  void refresh2();
};

class LCTimingKeepFactor : public AbstractControl {
  Q_OBJECT

public:
  LCTimingKeepFactor();

private:
  QPushButton btn1;
  QPushButton btn2;
  QPushButton btn3;
  QPushButton btn4;
  QLabel label1;
  QLabel label2;
  QLabel label1a;
  QLabel label2a;
  Params params;

  void refresh1();
  void refresh2();
};

class LCTimingKeepFactorUD : public AbstractControl {
  Q_OBJECT

public:
  LCTimingKeepFactorUD();

private:
  QPushButton btn;
  Params params;
  
  void refresh();
};

class AutoResCondition : public AbstractControl {
  Q_OBJECT

public:
  AutoResCondition();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class AutoResLimitTime : public AbstractControl {
  Q_OBJECT

public:
  AutoResLimitTime();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class AutoEnableSpeed : public AbstractControl {
  Q_OBJECT

public:
  AutoEnableSpeed();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class CamDecelDistAdd : public AbstractControl {
  Q_OBJECT

public:
  CamDecelDistAdd();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class LiveSRPercent : public AbstractControl {
  Q_OBJECT

public:
  LiveSRPercent();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class VCurvSpeedUD : public AbstractControl {
  Q_OBJECT

public:
  VCurvSpeedUD();
};

class VCurvSpeed : public AbstractControl {
  Q_OBJECT

public:
  VCurvSpeed();

private:
  QPushButton btn;
  QLineEdit edit1;
  QLineEdit edit2;
  Params params;

  void refresh();
};

class OCurvSpeedUD : public AbstractControl {
  Q_OBJECT

public:
  OCurvSpeedUD();
};

class OCurvSpeed : public AbstractControl {
  Q_OBJECT

public:
  OCurvSpeed();

private:
  QPushButton btn;
  QLineEdit edit1;
  QLineEdit edit2;
  Params params;

  void refresh();
};

class KISANaviSelect : public AbstractControl {
  Q_OBJECT

public:
  KISANaviSelect();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class KISAMapboxStyle : public AbstractControl {
  Q_OBJECT

public:
  KISAMapboxStyle();

private:
  QPushButton btn1;
  QPushButton btn2;
  QPushButton btn3;
  QPushButton btn4;
  Params params;
  
  void refresh();
};

class RESCountatStandstill : public AbstractControl {
  Q_OBJECT

public:
  RESCountatStandstill();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class SpeedLimitSignType : public AbstractControl {
  Q_OBJECT

public:
  SpeedLimitSignType();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class RadarLongHelperOption : public AbstractControl {
  Q_OBJECT

public:
  RadarLongHelperOption();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class CurvDecelSelect : public AbstractControl {
  Q_OBJECT

public:
  CurvDecelSelect();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class AutoRESDelay : public AbstractControl {
  Q_OBJECT

public:
  AutoRESDelay();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class OSMCustomSpeedLimitUD : public AbstractControl {
  Q_OBJECT

public:
  OSMCustomSpeedLimitUD();
};

class OSMCustomSpeedLimit : public AbstractControl {
  Q_OBJECT

public:
  OSMCustomSpeedLimit();

private:
  QPushButton btn;
  QLineEdit edit1;
  QLineEdit edit2;
  Params params;

  void refresh();
};

class DesiredCurvatureLimit : public AbstractControl {
  Q_OBJECT

public:
  DesiredCurvatureLimit();

private:
  QPushButton btndigit;
  QPushButton btnminus;
  QPushButton btnplus;
  QLabel label;
  Params params;
  float digit = 0.01;
  
  void refresh();
};

class DynamicTRBySpeed : public AbstractControl {
  Q_OBJECT

public:
  DynamicTRBySpeed();

private:
  QPushButton btn;
  QLineEdit edit1;
  QLineEdit edit2;
  Params params;

  void refresh();
};

class LaneWidth : public AbstractControl {
  Q_OBJECT

public:
  LaneWidth();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class SpeedLaneWidthUD : public AbstractControl {
  Q_OBJECT

public:
  SpeedLaneWidthUD();
};

class SpeedLaneWidth : public AbstractControl {
  Q_OBJECT

public:
  SpeedLaneWidth();

private:
  QPushButton btn;
  QLineEdit edit1;
  QLineEdit edit2;
  Params params;

  void refresh();
};

class KISABottomTextView : public AbstractControl {
  Q_OBJECT

public:
  KISABottomTextView();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class KISAEdgeOffset : public AbstractControl {
  Q_OBJECT

public:
  KISAEdgeOffset();

private:
  QPushButton btnplusl;
  QPushButton btnminusl;
  QPushButton btnplusr;
  QPushButton btnminusr;
  QLabel labell1;
  QLabel labelr1;
  QLabel labell;
  QLabel labelr;
  Params params;
  
  void refreshl();
  void refreshr();
};

class ToAvoidLKASFault : public AbstractControl {
  Q_OBJECT

public:
  ToAvoidLKASFault();

private:
  QPushButton btnplusl;
  QPushButton btnminusl;
  QPushButton btnplusr;
  QPushButton btnminusr;
  QLabel labell1;
  QLabel labelr1;
  QLabel labell;
  QLabel labelr;
  Params params;
  
  void refreshl();
  void refreshr();
};

class RoutineDriveOption : public AbstractControl {
  Q_OBJECT

public:
  RoutineDriveOption();

private:
  QPushButton btn0;
  QPushButton btn1;
  Params params;
  
  void refresh();
};

class RPMAnimatedMaxValue : public AbstractControl {
  Q_OBJECT

public:
  RPMAnimatedMaxValue();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class UserSpecificFeature : public AbstractControl {
  Q_OBJECT

public:
  UserSpecificFeature();

private:
  QPushButton btn;
  QLineEdit edit;
  Params params;

  void refresh();
};

class MultipleLatSelect : public AbstractControl {
  Q_OBJECT

public:
  MultipleLatSelect();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  int    m_nMethod;


  void refresh();
};

class MultipleLateralSpeed : public AbstractControl {
  Q_OBJECT

public:
  MultipleLateralSpeed();

private:
  QLabel label1;
  QPushButton btnplusl;
  QLabel labell;
  QPushButton btnminusl;
  QPushButton btnplusr;
  QLabel labelr;
  QPushButton btnminusr;
  QPushButton btn1;
  QPushButton btn2;
  QPushButton btn3;
  Params params;
  
  void refresh1();
  void refresh2();
  void refresh3();
  void refreshl();
  void refreshr();
};

class MultipleLateralAngle : public AbstractControl {
  Q_OBJECT

public:
  MultipleLateralAngle();

private:
  QLabel label1;
  QPushButton btnplusl;
  QLabel labell;
  QPushButton btnminusl;
  QPushButton btnplusr;
  QLabel labelr;
  QPushButton btnminusr;
  QPushButton btn1;
  QPushButton btn2;
  QPushButton btn3;
  Params params;
  
  void refresh1();
  void refresh2();
  void refresh3();
  void refreshl();
  void refreshr();
};

class StoppingDist : public AbstractControl {
  Q_OBJECT

public:
  StoppingDist();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class VariableCruiseLevel : public AbstractControl {
  Q_OBJECT

public:
  VariableCruiseLevel();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class ExternalDeviceIP : public AbstractControl {
  Q_OBJECT

public:
  ExternalDeviceIP();

private:
  QPushButton btn;
  QPushButton btna;
  QLineEdit edit;
  Params params;
  
  void refresh();
};

class DoNotDisturbMode : public AbstractControl {
  Q_OBJECT

public:
  DoNotDisturbMode();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class CruiseGapBySpd : public AbstractControl {
  Q_OBJECT

public:
  CruiseGapBySpd();

private:
  QPushButton btnplus1;
  QLabel label1;
  QPushButton btnminus1;

  QPushButton btnplus2;
  QLabel label2;
  QPushButton btnminus2;

  QPushButton btnplus3;
  QLabel label3;
  QPushButton btnminus3;

  QPushButton btn1;
  QPushButton btn2;
  QPushButton btn3;
  QPushButton btn4;
  Params params;
  
  void refresh1();
  void refresh2();
  void refresh3();
  void refresh4();

  void refresh5();
  void refresh6();
  void refresh7();
};

class CruiseSetwithRoadLimitSpeedOffset : public AbstractControl {
  Q_OBJECT

public:
  CruiseSetwithRoadLimitSpeedOffset();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class LongAlternative : public AbstractControl {
  Q_OBJECT

public:
  LongAlternative();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class MapboxToken : public AbstractControl {
  Q_OBJECT

public:
  MapboxToken();

private:
  QPushButton btn;
  QLineEdit edit;
  Params params;

  void refresh();
};

class CruiseSpammingLevel : public AbstractControl {
  Q_OBJECT

public:
  CruiseSpammingLevel();

private:
  QPushButton btnplus1;
  QLabel label1;
  QPushButton btnminus1;

  QPushButton btnplus2;
  QLabel label2;
  QPushButton btnminus2;

  QPushButton btnplus3;
  QLabel label3;
  QPushButton btnminus3;

  QPushButton btn1;
  QPushButton btn2;
  QPushButton btn3;
  QPushButton btn4;
  Params params;
  
  void refresh1();
  void refresh2();
  void refresh3();
  void refresh4();

  void refresh5();
  void refresh6();
  void refresh7();
};

class KISACruiseGapSet : public AbstractControl {
  Q_OBJECT

public:
  KISACruiseGapSet();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class UseLegacyLaneModel : public AbstractControl {
  Q_OBJECT

public:
  UseLegacyLaneModel();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class KISACruiseSpammingInterval : public AbstractControl {
  Q_OBJECT

public:
  KISACruiseSpammingInterval();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class KISACruiseSpammingBtnCount : public AbstractControl {
  Q_OBJECT

public:
  KISACruiseSpammingBtnCount();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};

class RegenBrakeFeature : public AbstractControl {
  Q_OBJECT

public:
  RegenBrakeFeature();

private:
  QPushButton btn1;
  QPushButton btn2;
  QPushButton btn3;
  Params params;
  
  void refresh();
};

class SetSpeedPlus : public AbstractControl {
  Q_OBJECT

public:
  SetSpeedPlus();

private:
  QPushButton btnplus;
  QPushButton btnminus;
  QLabel label;
  Params params;
  
  void refresh();
};