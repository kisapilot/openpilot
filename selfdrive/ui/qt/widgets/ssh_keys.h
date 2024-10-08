#pragma once

#include <QPushButton>

#include "system/hardware/hw.h"
#include "selfdrive/ui/qt/widgets/controls.h"

// SSH enable toggle
class SshToggle : public ToggleControl {
  Q_OBJECT

public:
  SshToggle() : ToggleControl(tr("Enable SSH"), "", "", Hardware::get_ssh_enabled()) {
    QObject::connect(this, &SshToggle::toggleFlipped, [=](bool state) {
      Hardware::set_ssh_enabled(state);
    });
  }
};

// SSH key management widget
class SshControl : public ButtonControl {
  Q_OBJECT

public:
  SshControl();

private:
  Params params;

  void refresh();
  void getUserKeys(const QString &username);
};

// SSH legacy toggle
class SshLegacyToggle : public ToggleControl {
  Q_OBJECT

public:
  SshLegacyToggle() : ToggleControl(tr("Use Legacy SSH Key"), tr("Public KEY (0.8.2 or less) is used when accessing SSH."), "", Params().getBool("KisaSSHLegacy")) {
    QObject::connect(this, &SshLegacyToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("KisaSSHLegacy", status);
    });
  }
};