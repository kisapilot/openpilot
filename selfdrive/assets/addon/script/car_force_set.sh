#!/usr/bin/bash

cd /data/openpilot
CAR_NAME=`cat /data/params/d/CarModel`
CAR_FULL_NAME=`cat /data/openpilot/opendbc_repo/opendbc/car/hyundai/values.py | grep "${CAR_NAME} =" | awk -F'"' '{print $2}'`
echo -n "${CAR_FULL_NAME}" > /data/params/d/CarModel