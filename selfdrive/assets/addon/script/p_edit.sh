#!/usr/bin/bash

cd /data/openpilot
MAX_STEER=`cat /data/params/d/MaxSteer`
MAX_RT_DELTA=`cat /data/params/d/MaxRTDelta`
MAX_RATE_UP=`cat /data/params/d/MaxRateUp`
MAX_RATE_DOWN=`cat /data/params/d/MaxRateDown`

sed -i "7s/.*/  .max_rt_delta \= ${MAX_RT_DELTA}\, \\\/g" /data/openpilot/panda/board/safety/safety_hyundai.h
sed -i "20s/.*/const SteeringLimits HYUNDAI_STEERING_LIMITS \= HYUNDAI_LIMITS(${MAX_STEER}, ${MAX_RATE_UP}, ${MAX_RATE_DOWN})\;/g" /data/openpilot/panda/board/safety/safety_hyundai.h

touch /data/kisa_compiling
sleep 1

sudo reboot