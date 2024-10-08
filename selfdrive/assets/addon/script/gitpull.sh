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
  BRANCH=$(git rev-parse --abbrev-ref HEAD)
  HASH=$(git rev-parse HEAD)
  git fetch

  REMOTE_HASH=$(git rev-parse --verify origin/$BRANCH)
  REMOTE_HASH_DATE=$(git log FETCH_HEAD -n 1 --pretty=format:"%cs" | cut -c 6-)
  echo -n "$REMOTE_HASH" > /data/params/d/GitCommitRemote
  echo -n "$REMOTE_HASH_DATE" > /data/params/d/GitCommitRemoteDate

  if [ "$HASH" != "$REMOTE_HASH" ]; then
    IS_LANGFILE_CHANGED=$(git diff @{upstream} | grep translations)
    if [ "$IS_LANGFILE_CHANGED" != "" ]; then
      git reset --hard
    fi
    git pull origin $BRANCH
    touch /data/kisa_compiling
    sleep 1

    sudo reboot
  fi
fi