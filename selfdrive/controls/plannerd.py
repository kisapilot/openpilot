#!/usr/bin/env python3
from cereal import car
from openpilot.common.params import Params
from openpilot.common.realtime import Priority, config_realtime_process
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner
from openpilot.selfdrive.controls.lib.lateral_planner import LateralPlanner
import cereal.messaging as messaging


def plannerd_thread():
  config_realtime_process(5, Priority.CTRL_LOW)

  cloudlog.info("plannerd is waiting for CarParams")
  params = Params()
  with car.CarParams.from_bytes(params.get("CarParams", block=True)) as msg:
    CP = msg
  cloudlog.info("plannerd got CarParams: %s", CP.carName)

  longitudinal_planner = LongitudinalPlanner(CP)
  lateral_planner = LateralPlanner(CP)

  pm = messaging.PubMaster(['longitudinalPlan', 'lateralPlan'])
  sm = messaging.SubMaster(['carControl', 'carState', 'controlsState', 'radarState', 'modelV2'],
                           poll='modelV2', ignore_avg_freq=['radarState'])

  while True:
    sm.update()
    if sm.updated['modelV2']:
      lateral_planner.update(sm)
      lateral_planner.publish(sm, pm)
      longitudinal_planner.update(sm, CP)
      longitudinal_planner.publish(sm, pm)


def main():
  plannerd_thread()


if __name__ == "__main__":
  main()
