#!/usr/bin/bash

cd /data/openpilot

cat /data/preset2 | while read line
do
ParamName=$(echo $line | awk -F ':' '{print $1}')
ParamValue=$(echo $line | awk -F ':' '{print $2}')
echo -n $ParamValue > /data/params/d/${ParamName}
chown comma:comma /data/params/d/${ParamName}
chmod 600 /data/params/d/${ParamName}
done