#!/usr/bin/bash

# acquire git hash from remote
cd /data/openpilot
ping -q -c 1 -w 1 google.com &> /dev/null
if [ "$?" == "0" ]; then
  /data/openpilot/selfdrive/assets/addon/script/git_remove.sh
  CURRENT_BRANCH=$(git rev-parse --abbrev-ref HEAD)
  LOCAL_HASH=$(git rev-parse HEAD)
  git fetch
  REMOTE_HASH=$(git rev-parse --verify origin/$CURRENT_BRANCH)
  REMOTE_HASH_DATE=$(git log FETCH_HEAD -n 1 --pretty=format:"%cs" | cut -c 6-)
  echo -n "$REMOTE_HASH" > /data/params/d/GitCommitRemote
  echo -n "$REMOTE_HASH_DATE" > /data/params/d/GitCommitRemoteDate
  if [ "$LOCAL_HASH" != "$REMOTE_HASH" ]; then
    wget https://raw.githubusercontent.com/kisapilot/openpilot/$CURRENT_BRANCH/KisaPilot_Updates.txt -O /data/KisaPilot_Updates.txt
  else
    if [ -f "/data/KisaPilot_Updates.txt" ]; then
      rm -f /data/KisaPilot_Updates.txt
    fi
  fi
fi
