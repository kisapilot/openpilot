#!/usr/bin/bash

cd /data/openpilot

if [ -e /data/preset2 ]; then
rm -f /data/preset2
fi

cat /data/openpilot/selfdrive/assets/addon/script/param_init_value | while read line
do
ParamName=$(echo $line | awk -F ':' '{print $1}')
ParamValue=$(cat /data/params/d/${ParamName})
echo "$ParamName:$ParamValue" >> /data/preset2
done