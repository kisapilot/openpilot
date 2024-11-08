#include <QDebug>

#include "selfdrive/ui/qt/offroad/developer_panel.h"
#include "selfdrive/ui/qt/widgets/ssh_keys.h"
#include "selfdrive/ui/qt/widgets/controls.h"

#include "selfdrive/ui/qt/widgets/kisapilot.h" // kisapilot

DeveloperPanel::DeveloperPanel(SettingsWindow *parent) : ListWidget(parent) {
  // SSH keys
  addItem(new SshToggle());
  addItem(new SshControl());
  addItem(new SshLegacyToggle());
  addItem(new DebugUiOneToggle());
  addItem(new DebugUiTwoToggle());
  addItem(new DebugUiThreeToggle());
  addItem(new KISADebug());
  addItem(new LongLogToggle());
  addItem(new ShowErrorToggle());
  addItem(new PrebuiltToggle());
  addItem(new LDWSToggle());
  addItem(new GearDToggle());
  addItem(new SteerWarningFixToggle());
  addItem(new IgnoreCanErroronISGToggle());
  addItem(new NoSmartMDPSToggle());
  addItem(new UFCModeEnabledToggle());
  addItem(new LFAButtonEngagementToggle());
  addItem(new StockLKASEnabledatDisenagedStatusToggle());
  addItem(new UserSpecificFeature());
  //addItem(new MapboxToken());

  addItem(new CarSelectCombo());
  addItem(new ModelSelectCombo());

  joystickToggle = new ParamControl("JoystickDebugMode", tr("Joystick Debug Mode"), "", "");
  QObject::connect(joystickToggle, &ParamControl::toggleFlipped, [=](bool state) {
    params.putBool("LongitudinalManeuverMode", false);
    longManeuverToggle->refresh();
  });
  addItem(joystickToggle);

  longManeuverToggle = new ParamControl("LongitudinalManeuverMode", tr("Longitudinal Maneuver Mode"), "", "");
  QObject::connect(longManeuverToggle, &ParamControl::toggleFlipped, [=](bool state) {
    params.putBool("JoystickDebugMode", false);
    joystickToggle->refresh();
  });
  addItem(longManeuverToggle);

  // Joystick and longitudinal maneuvers should be hidden on release branches
  // also the toggles should be not available to change in onroad state
  const bool is_release = params.getBool("IsReleaseBranch");
  QObject::connect(uiState(), &UIState::offroadTransition, [=](bool offroad) {
    for (auto btn : findChildren<ParamControl *>()) {
      btn->setVisible(!is_release);
      btn->setEnabled(offroad);
    }
  });

}
