#!/usr/bin/env bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"

source "$DIR/launch_env.sh"

function agnos_init {
  # prebuilt recreate
  if [ -f "/data/ks" ]; then
    sudo rm /data/ks
    if [ -f "$DIR/prebuilt" ]; then
      sudo rm $DIR/prebuilt
    fi
  elif [ -f "/data/kisa_starting" ]; then
    if [ -f "$DIR/prebuilt" ]; then
      sudo rm $DIR/prebuilt
    fi
  else
    if [ -f "/data/params/d/PutPrebuiltOn" ]; then
      PREBUILT_CHECK=$(cat /data/params/d/PutPrebuiltOn)
      if [[ "$PREBUILT_CHECK" == "1" && ! -f "$DIR/prebuilt" ]]; then
        touch /data/kisa_starting
        touch $DIR/prebuilt
      fi
    fi
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

  cat /data/openpilot/opendbc_repo/opendbc/car/hyundai/values.py | grep ' = Hyundai' | awk '{print $1}' > /data/params/d/CarList
}

function launch {

  # one touch git pull
  KILINE="alias gi='git -C /data/openpilot pull && rm -f /data/openpilot/prebuilt && touch /data/ks && echo -en 1 > /data/params/d/DoReboot'"; KIFILE="$HOME/.bashrc"; grep -qxF "$KILINE" "$KIFILE" || echo "$KILINE" >> "$KIFILE"

  # Remove orphaned git lock if it exists on boot
  [ -f "$DIR/.git/index.lock" ] && rm -f $DIR/.git/index.lock

  # Check to see if there's a valid overlay-based update available. Conditions
  # are as follows:
  #
  # 1. The DIR init file has to exist, with a newer modtime than anything in
  #    the DIR Git repo. This checks for local development work or the user
  #    switching branches/forks, which should not be overwritten.
  # 2. The FINALIZED consistent file has to exist, indicating there's an update
  #    that completed successfully and synced to disk.

  if [ -f "${DIR}/.overlay_init" ]; then
    find ${DIR}/.git -newer ${DIR}/.overlay_init | grep -q '.' 2> /dev/null
    if [ $? -eq 0 ]; then
      echo "${DIR} has been modified, skipping overlay update installation"
    else
      if [ -f "${STAGING_ROOT}/finalized/.overlay_consistent" ]; then
        if [ ! -d /data/safe_staging/old_openpilot ]; then
          echo "Valid overlay update found, installing"
          
          touch /data/ks

          LAUNCHER_LOCATION="${BASH_SOURCE[0]}"

          mv $DIR /data/safe_staging/old_openpilot
          mv "${STAGING_ROOT}/finalized" $DIR
          cd $DIR

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

# KisaPilot Model check

Model_P_Hash=$(sha256sum /data/openpilot/selfdrive/modeld/models/driving_policy.onnx | awk '{print $1}')
Model_V_Hash=$(sha256sum /data/openpilot/selfdrive/modeld/models/driving_vision.onnx | awk '{print $1}')

MODEL_NAME=$(awk -v ph="$Model_P_Hash" -v vh="$Model_V_Hash" '
  $2 == ph && $3 == vh {
    print $1;
  }
' /data/openpilot/selfdrive/modeld/models/ModelList)

if [ -z "$MODEL_NAME" ]; then
  MODEL_NAME=$(head -n 1 /data/openpilot/selfdrive/modeld/models/ModelList | awk '{print $1}')
fi

echo -en "$MODEL_NAME" > /data/params/d/DrivingModel

  # kisa agent start
  python3 /data/openpilot/selfdrive/kisapilot/kisa_agent.py &

  # start manager
  cd system/manager
  if [ -f "/data/params/d/OSMSpeedLimitEnable" ]; then
    OSM_SL_ENABLE=$(cat /data/params/d/OSMSpeedLimitEnable)
  fi
  if [ -f "/data/params/d/CurvDecelOption" ]; then
    OSM_CURV_ENABLE=$(cat /data/params/d/CurvDecelOption)
  fi

  if [ "$OSM_SL_ENABLE" == "1" ] || [ "$OSM_CURV_ENABLE" == "1" ] || [ "$OSM_CURV_ENABLE" == "3" ]; then
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
