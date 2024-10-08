#!/usr/bin/bash

cd /data/openpilot
ping -q -c 1 -w 1 google.com &> /dev/null
if [ "$?" == "0" ]; then
  git log --date=human --pretty=format:"%h, %ad : %s" -n 30 > /data/params/d/GitCommits
fi