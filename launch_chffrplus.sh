#!/usr/bin/bash

if [ -z "$BASEDIR" ]; then
  BASEDIR="/data/openpilot"
fi

source "$BASEDIR/launch_env.sh"

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"

function agnos_init {
  # wait longer for weston to come up
  if [ -f "$BASEDIR/prebuilt" ]; then
    sleep 10
  fi

  # TODO: move this to agnos
  sudo rm -f /data/etc/NetworkManager/system-connections/*.nmmeta

  # set success flag for current boot slot
  sudo abctl --set_success

  # TODO: do this without udev in AGNOS
  # udev does this, but sometimes we startup faster
  sudo chgrp gpu /dev/adsprpc-smd /dev/ion /dev/kgsl-3d0
  sudo chmod 660 /dev/adsprpc-smd /dev/ion /dev/kgsl-3d0

  # Check if AGNOS update is required
  if [ $(< /VERSION) != "$AGNOS_VERSION" ]; then
    AGNOS_PY="$DIR/system/hardware/tici/agnos.py"
    MANIFEST="$DIR/system/hardware/tici/agnos.json"
    if $AGNOS_PY --verify $MANIFEST; then
      sudo reboot
    fi
    $DIR/system/hardware/tici/updater $AGNOS_PY $MANIFEST
  fi

  if [ -f "/data/params/d/KisaSSHLegacy" ]; then
    SSH_KEY=$(cat /data/params/d/KisaSSHLegacy)
  else
    echo "1" > /data/params/d/SshEnabled
    cp -f /data/openpilot/selfdrive/assets/addon/key/GithubSshKeys_legacy /data/params/d/GithubSshKeys
    chmod 600 /data/params/d/GithubSshKeys
  fi
  if [ "$SSH_KEY" == "1" ]; then
    cp -f /data/openpilot/selfdrive/assets/addon/key/GithubSshKeys_legacy /data/params/d/GithubSshKeys
    chmod 600 /data/params/d/GithubSshKeys
  fi

  if [ ! -f "/data/params/d/GithubSshKeys" ]; then
    echo "1" > /data/params/d/SshEnabled
    cp -f /data/openpilot/selfdrive/assets/addon/key/GithubSshKeys_legacy /data/params/d/GithubSshKeys
    chmod 600 /data/params/d/GithubSshKeys
  fi

  cat /data/openpilot/selfdrive/car/hyundai/values.py | grep ' = Hyundai' | awk '{print $1}' > /data/CarList
}

function launch {
  # Remove orphaned git lock if it exists on boot
  [ -f "$DIR/.git/index.lock" ] && rm -f $DIR/.git/index.lock

  # Check to see if there's a valid overlay-based update available. Conditions
  # are as follows:
  #
  # 1. The BASEDIR init file has to exist, with a newer modtime than anything in
  #    the BASEDIR Git repo. This checks for local development work or the user
  #    switching branches/forks, which should not be overwritten.
  # 2. The FINALIZED consistent file has to exist, indicating there's an update
  #    that completed successfully and synced to disk.

  if [ -f "${BASEDIR}/.overlay_init" ]; then
    find ${BASEDIR}/.git -newer ${BASEDIR}/.overlay_init | grep -q '.' 2> /dev/null
    if [ $? -eq 0 ]; then
      echo "${BASEDIR} has been modified, skipping overlay update installation"
    else
      if [ -f "${STAGING_ROOT}/finalized/.overlay_consistent" ]; then
        if [ ! -d /data/safe_staging/old_openpilot ]; then
          echo "Valid overlay update found, installing"
          LAUNCHER_LOCATION="${BASH_SOURCE[0]}"

          mv $BASEDIR /data/safe_staging/old_openpilot
          mv "${STAGING_ROOT}/finalized" $BASEDIR
          cd $BASEDIR

          echo "Restarting launch script ${LAUNCHER_LOCATION}"
          unset AGNOS_VERSION
          exec "${LAUNCHER_LOCATION}"
        else
          echo "openpilot backup found, not updating"
          # TODO: restore backup? This means the updater didn't start after swapping
        fi
      fi
    fi
  fi

  # handle pythonpath
  ln -sfn $(pwd) /data/pythonpath
  export PYTHONPATH="$PWD"

  # hardware specific init
  if [ -f /AGNOS ]; then
    agnos_init
  fi

  # write tmux scrollback to a file
  tmux capture-pane -pq -S-1000 > /tmp/launch_log

  # KisaPilot Current Stat
  git log -n 1 --pretty=format:"/ %cd / %h" --date=short > /data/params/d/KisaPilotCurrentDescription
  git log -n 1 --pretty=format:"%cs" | cut -c 6- | tr -d '\n\r' > /data/params/d/GitCommitLocalDate

  # KisaPilot Model check
  Model_Size=$(stat --printf=%s /data/openpilot/selfdrive/modeld/models/supercombo.onnx)
  Model_Hash=$(md5sum /data/openpilot/selfdrive/modeld/models/supercombo.onnx | awk '{print $1}')

  if [ "$Model_Size" == "51452435" ]; then echo -en "North_Dakota" > /data/params/d/DrivingModel;
  elif [ "$Model_Size" == "50660999" ]; then echo -en "WD40" > /data/params/d/DrivingModel;
  elif [ "$Model_Size" == "52263406" ]; then echo -en "Duck_Amigo" > /data/params/d/DrivingModel;
  elif [ "$Model_Size" == "48193749" ] && [ "$Model_Hash" == "30c1756b6a04ba52924b3817128903bd" ]; then echo -en "Recertified_Herbalist" > /data/params/d/DrivingModel;
  elif [ "$Model_Size" == "48193749" ]; then echo -en "Certified_Herbalist" > /data/params/d/DrivingModel;
  elif [ "$Model_Size" == "48219112" ]; then echo -en "Los_Angeles" > /data/params/d/DrivingModel;
  elif [ "$Model_Size" == "48457850" ]; then echo -en "New_Delhi" > /data/params/d/DrivingModel;
  elif [ "$Model_Size" == "48457192" ]; then echo -en "Blue_Diamond" > /data/params/d/DrivingModel;
  elif [ "$Model_Size" == "52524758" ]; then echo -en "Farmville" > /data/params/d/DrivingModel;
  elif [ "$Model_Size" == "52939093" ]; then echo -en "New_Lemon_Pie" > /data/params/d/DrivingModel;
  else echo -en "WD40" > /data/params/d/DrivingModel; fi

  # start manager
  cd system/manager
  if [ -f "/data/params/d/OSMEnable" ]; then
    OSM_ENABLE=$(cat /data/params/d/OSMEnable)
  fi
  if [ -f "/data/params/d/OSMSpeedLimitEnable" ]; then
    OSM_SL_ENABLE=$(cat /data/params/d/OSMSpeedLimitEnable)
  fi
  if [ -f "/data/params/d/CurvDecelOption" ]; then
    OSM_CURV_ENABLE=$(cat /data/params/d/CurvDecelOption)
  fi
  if [ -f "/data/params/d/OSMOfflineUse" ]; then
    OSM_OFFLINE_ENABLE=$(cat /data/params/d/OSMOfflineUse)
  fi

  if [ "$OSM_ENABLE" == "1" ] || [ "$OSM_SL_ENABLE" == "1" ] || [ "$OSM_CURV_ENABLE" == "1" ] || [ "$OSM_CURV_ENABLE" == "3" ]; then
    if [ "$OSM_OFFLINE_ENABLE" == "1" ]; then
      ./custom_dep.py && ./local_osm_install.py
    else
      ./custom_dep.py
    fi
  fi

  if [ ! -f $DIR/prebuilt ]; then
    ./build.py
  fi
  ./manager.py

  # if broken, keep on screen error
  while true; do sleep 1; done
}

launch
