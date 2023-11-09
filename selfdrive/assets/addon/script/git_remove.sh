#!/usr/bin/bash

cd /data/openpilot
REMOVED_BRANCH=$(git branch -vv | grep ': gone]' | awk '{print $1}' |  tail -n 1)
if [ "$REMOVED_BRANCH" != "" ]; then
  if [ "$REMOVED_BRANCH" == "*" ]; then
    REMOVED_BRANCH=$(git branch -vv | grep ': gone]' | awk '{print $2}' |  tail -n 1)
  fi
  git remote prune origin --dry-run
  echo $REMOVED_BRANCH | xargs git branch -D
  sed -i "/$REMOVED_BRANCH/d" .git/config
fi