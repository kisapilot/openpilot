#!/usr/bin/bash

cd /data/openpilot

cat /data/openpilot/selfdrive/assets/addon/script/param_init_value | while read line
do
ParamName=$(echo $line | awk -F ':' '{print $1}')
ParamValue=$(echo $line | awk -F ':' '{print $2}')
echo -n $ParamValue > /data/params/d/${ParamName}
done