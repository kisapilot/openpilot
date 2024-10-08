#!/usr/bin/bash

cd /data/openpilot
ping -q -c 1 -w 1 google.com &> /dev/null
if [ "$?" == "0" ]; then
  REMOVED_BRANCH=$(git branch -vv | grep ': gone]' | awk '{print $1}')
  if [ "$REMOVED_BRANCH" != "" ]; then
    if [ "$REMOVED_BRANCH" == "*" ]; then
      REMOVED_BRANCH=$(git branch -vv | grep ': gone]' | awk '{print $2}')
    fi
    git remote prune origin --dry-run
    echo $REMOVED_BRANCH | xargs git branch -D
    sed -i "/$REMOVED_BRANCH/d" .git/config
  fi
  CURRENT_BRANCH=$(git rev-parse --abbrev-ref HEAD)
  git clean -d -f -f
  git fetch --all
  git reset --hard origin/$CURRENT_BRANCH
  git pull origin $CURRENT_BRANCH

  rm -f /data/params/d/DrivingModel
  rm -f /data/openpilot/selfdrive/modeld/models/supercombo.onnx
  rm -f /data/openpilot/selfdrive/modeld/models/supercombo.thneed
  rm -f /data/openpilot/selfdrive/modeld/models/supercombo_metadata.pkl
  git -C /data/openpilot/selfdrive//modeld/models checkout supercombo.onnx
  touch /data/kisa_compiling
  sleep 1

  sudo reboot
fi