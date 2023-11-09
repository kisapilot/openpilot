#!/usr/bin/env python3
import os
import queue
import threading
import time
import subprocess
import re
import json

import cereal.messaging as messaging
from openpilot.common.params import Params
from openpilot.common.realtime import DT_TRML

import zmq

# KisaPilot, this is for getting navi data from external device.

def navid_thread(end_event, nv_queue):
  pm = messaging.PubMaster(['liveENaviData'])
  count = 0

  spd_limit = 0
  safety_distance = 0
  sign_type = 0
  turn_info = 0
  turn_distance = 0
  road_limit_speed = 0
  road_limit_speed_cnt = 0
  link_length = 0
  current_link_angle = 0
  next_link_angle = 0
  road_name = ""
  is_highway = 0
  is_tunnel = 0
  kisa_lat = 0
  kisa_lon = 0
  kisa_lat_prev = 0
  kisa_lon_prev = 0

  dest_changed = False
  dest_changed_count = 0

  KISA_Debug = Params().get_bool("KISADebug")
  if KISA_Debug:
    kisa_0 = ""
    kisa_1 = ""
    kisa_2 = ""
    kisa_3 = ""
    kisa_4 = ""
    kisa_5 = ""
    kisa_6 = ""
    kisa_7 = ""
    kisa_8 = ""
    kisa_9 = ""


  ip_add = ""
  ip_bind = False
 
  check_connection = False
  try:
    ip_count = int(len(Params().get("ExternalDeviceIP", encoding="utf8").split(',')))
  except:
    ip_count = 0
    pass
  is_metric = Params().get_bool("IsMetric")
  navi_selection = int(Params().get("KISANaviSelect", encoding="utf8"))

  mtom3 = False
  mtom2 = False
  mtom1 = False
  mtom_dist_last = 0

  if navi_selection == 2:
    waze_alert_id = 0
    waze_alert_distance = "0"
    waze_road_speed_limit = 0
    waze_current_speed = 0
    waze_road_name = ""
    waze_nav_sign = 0
    waze_nav_distance = 0
    waze_alert_type = ""
    waze_is_metric = Params().get_bool("IsMetric")
    waze_current_speed_prev = 0
    waze_lat = 0
    waze_lon = 0
    waze_lat_prev = 0
    waze_lon_prev = 0

  while not end_event.is_set():
    if not ip_bind:
      if (count % int(max(60., ip_count) / DT_TRML)) == 0:
        os.system("/data/openpilot/selfdrive/assets/addon/script/find_ip.sh &")
      if (count % int((63+ip_count) / DT_TRML)) == 0:
        ip_add = Params().get("ExternalDeviceIPNow", encoding="utf8")
        if ip_add is not None:
          ip_bind = True
          check_connection = True
          context = zmq.Context()
          socket = context.socket(zmq.SUB)
          socket.connect("tcp://" + str(ip_add) + ":5555")

    if ip_bind:
      spd_limit = 0
      safety_distance = 0
      sign_type = 0
      turn_info = 0
      turn_distance = 0
      #road_limit_speed = 0
      link_length = 0
      current_link_angle = 0
      next_link_angle = 0
      road_name = ""
      is_highway = 0
      is_tunnel = 0

      if navi_selection == 2:
        if (count % int(7. / DT_TRML)) == 0 and int(waze_current_speed) > 2:
          waze_alert_id = 0
          waze_alert_distance = "0"
          waze_alert_type = ""
        if (count % int(10. / DT_TRML)) == 0 and int(waze_current_speed) > 2 and int(waze_nav_distance) < 30:
          waze_nav_sign = 0
          waze_nav_distance = 0
        waze_current_speed = 0

      if KISA_Debug:
        kisa_0 = ""
        kisa_1 = ""
        kisa_2 = ""
        kisa_3 = ""
        kisa_4 = ""
        kisa_5 = ""
        kisa_6 = ""
        kisa_7 = ""
        kisa_8 = ""
        kisa_9 = ""

      socket.subscribe("")
      message = str(socket.recv(), 'utf-8')

      if (count % int(30. / DT_TRML)) == 0:
        try:
          rtext = subprocess.check_output(["netstat", "-n"])
          check_connection = True if str(rtext).find('5555      ESTABLISHED') != -1 else False
        except:
          pass
      
      for line in message.split('\n'):
        if "kisaspdlimit" in line:
          arr = line.split('kisaspdlimit: ')
          spd_limit = arr[1]
        if "kisaspddist" in line:
          arr = line.split('kisaspddist: ')
          safety_distance = arr[1]
        if "kisasigntype" in line:
          arr = line.split('kisasigntype: ')
          sign_type = arr[1]
        if "kisaturninfo" in line:
          arr = line.split('kisaturninfo: ')
          turn_info = arr[1]
        if "kisadistancetoturn" in line:
          arr = line.split('kisadistancetoturn: ')
          turn_distance = arr[1]
        if "kisaroadlimitspd" in line:
          arr = line.split('kisaroadlimitspd: ')
          road_limit_speed = arr[1]
          road_limit_speed_cnt = 0
        elif (count % int(2. / DT_TRML)) == 0:
          road_limit_speed_cnt += 1
          if road_limit_speed_cnt > 30:
            road_limit_speed = 0
            road_limit_speed_cnt = 0
        if "kisalinklength" in line:
          arr = line.split('kisalinklength: ')
          link_length = arr[1]
        if "kisacurrentlinkangle" in line:
          arr = line.split('kisacurrentlinkangle: ')
          current_link_angle = arr[1]
        if "kisanextlinkangle" in line:
          arr = line.split('kisanextlinkangle: ')
          next_link_angle = arr[1]
        if "kisaroadname" in line:
          arr = line.split('kisaroadname: ')
          road_name = arr[1]
        if "kisaishighway" in line:
          arr = line.split('kisaishighway: ')
          is_highway = arr[1]
        if "kisaistunnel" in line:
          arr = line.split('kisaistunnel: ')
          is_tunnel = arr[1]
        if "kisadestlat" in line:
          arr = line.split('kisadestlat: ')
          kisa_lat = arr[1]
        if "kisadestlon" in line:
          arr = line.split('kisadestlon: ')
          kisa_lon = arr[1]

        if kisa_lat and kisa_lon and kisa_lat != kisa_lat_prev and kisa_lon != kisa_lon_prev:
          dest_changed = True
          dest_changed_count = 0
          try:
            Params().remove("NavDestination")
            Params().remove("NavDestinationWaypoints")
          except:
            pass
          kisa_lat_prev = kisa_lat
          kisa_lon_prev = kisa_lon
          kisa_lat_ = float(kisa_lat)
          kisa_lon_ = float(kisa_lon)
          dest = {"latitude": kisa_lat_, "longitude": kisa_lon_,}
          waypoints = [(kisa_lon_, kisa_lat_),]
        elif dest_changed:
          dest_changed_count += 1
          if dest_changed_count > 2:
            dest_changed = False
            Params().put("NavDestination", json.dumps(dest))
            Params().put("NavDestinationWaypoints", json.dumps(waypoints))

        if navi_selection == 2: # NAV unit should be metric. Do not use miles unit.(Distance factor is not detailed.)
          if "kisawazereportid" in line:
            arr = line.split('kisawazereportid: ')
            try:
              waze_alert_type = arr[1]
              if "icon_report_speedlimit" in arr[1]:
                waze_alert_id = 1
              elif "icon_report_camera" in arr[1]:
                waze_alert_id = 1
              elif "icon_report_speedcam" in arr[1]:
                waze_alert_id = 1
              elif "icon_report_police" in arr[1]:
                waze_alert_id = 2
              elif "icon_report_hazard" in arr[1]:
                waze_alert_id = 3
              elif "icon_report_traffic" in arr[1]:
                waze_alert_id = 4
            except:
              pass
          if "kisawazealertdist" in line:
            arr = line.split('kisawazealertdist: ')
            try:
              if arr[1] is None or arr[1] == "":
                waze_alert_distance = "0"
              else:
                waze_alert_distance = str(re.sub(r'[^0-9]', '', arr[1]))
            except:
              pass
          if "kisawazeroadspdlimit" in line:
            arr = line.split('kisawazeroadspdlimit: ')
            try:
              if arr[1] == "-1":
                waze_road_speed_limit = 0
              elif arr[1] is None or arr[1] == "":
                waze_road_speed_limit = 0
              else:
                waze_road_speed_limit = arr[1]
            except:
              waze_road_speed_limit = 0
              pass
          if "kisawazecurrentspd" in line:
            arr = line.split('kisawazecurrentspd: ')
            try:
              waze_current_speed = arr[1]
            except:
              pass
          if "kisawazeroadname" in line: # route should be set.
            arr = line.split('kisawazeroadname: ')
            try:
              waze_road_name = arr[1]
            except:
              pass
          if "kisawazenavsign" in line: # route should be set.
            arr = line.split('kisawazenavsign: ')
            try:
              waze_nav_sign = arr[1]
            except:
              pass
          if "kisawazenavdist" in line: # route should be set.
            arr = line.split('kisawazenavdist: ')
            try:
              waze_nav_distance = arr[1]
            except:
              pass
          if "kisawazedestlat" in line: # route should be set.
            arr = line.split('kisawazedestlat: ')
            try:
              waze_lat_temp = arr[1]
              waze_lat_temp_back = waze_lat_temp[-6:]
              waze_lat_temp_front_temp = waze_lat_temp.split(waze_lat_temp_back)
              waze_lat_temp_front = waze_lat_temp_front_temp[0]
              waze_lat = waze_lat_temp_front + "." + waze_lat_temp_back
            except:
              pass
          if "kisawazedestlon" in line: # route should be set.
            arr = line.split('kisawazedestlon: ')
            try:
              waze_lon_temp = arr[1]
              waze_lon_temp_back = waze_lon_temp[-6:]
              waze_lon_temp_front_temp = waze_lon_temp.split(waze_lon_temp_back)
              waze_lon_temp_front = waze_lon_temp_front_temp[0]
              waze_lon = waze_lon_temp_front + "." + waze_lon_temp_back
            except:
              pass

          if waze_lat and waze_lon and waze_lat != waze_lat_prev and waze_lon != waze_lon_prev:
            dest_changed = True
            dest_changed_count = 0
            try:
              Params().remove("NavDestination")
              Params().remove("NavDestinationWaypoints")
            except:
              pass
            waze_lat_prev = waze_lat
            waze_lon_prev = waze_lon
            waze_lat_ = float(waze_lat)
            waze_lon_ = float(waze_lon)
            dest = {"latitude": waze_lat_, "longitude": waze_lon_,}
            waypoints = [(waze_lon_, waze_lat_),]
          elif dest_changed:
            dest_changed_count += 1
            if dest_changed_count > 2:
              dest_changed = False
              Params().put("NavDestination", json.dumps(dest))
              Params().put("NavDestinationWaypoints", json.dumps(waypoints))

        if KISA_Debug:
          try:
            if "kisa0" in line:
              arr = line.split('kisa0   : ')
              kisa_0 = arr[1]
          except:
            pass
          try:
            if "kisa1" in line:
              arr = line.split('kisa1   : ')
              kisa_1 = arr[1]
          except:
            pass
          try:
            if "kisa2" in line:
              arr = line.split('kisa2   : ')
              kisa_2 = arr[1]
          except:
            pass
          try:
            if "kisa3" in line:
              arr = line.split('kisa3   : ')
              kisa_3 = arr[1]
          except:
            pass
          try:
            if "kisa4" in line:
              arr = line.split('kisa4   : ')
              kisa_4 = arr[1]
          except:
            pass
          try:
            if "kisa5" in line:
              arr = line.split('kisa5   : ')
              kisa_5 = arr[1]
          except:
            pass
          try:
            if "kisa6" in line:
              arr = line.split('kisa6   : ')
              kisa_6 = arr[1]
          except:
            pass
          try:
            if "kisa7" in line:
              arr = line.split('kisa7   : ')
              kisa_7 = arr[1]
          except:
            pass
          try:
            if "kisa8" in line:
              arr = line.split('kisa8   : ')
              kisa_8 = arr[1]
          except:
            pass
          try:
            if "kisa9" in line:
              arr = line.split('kisa9   : ')
              kisa_9 = arr[1]
          except:
            pass

      navi_msg = messaging.new_message('liveENaviData')
      navi_msg.liveENaviData.speedLimit = int(spd_limit)
      navi_msg.liveENaviData.safetyDistance = float(safety_distance)
      navi_msg.liveENaviData.safetySign = int(sign_type)
      navi_msg.liveENaviData.turnInfo = int(turn_info)
      navi_msg.liveENaviData.distanceToTurn = float(turn_distance)
      navi_msg.liveENaviData.connectionAlive = bool(check_connection)
      navi_msg.liveENaviData.roadLimitSpeed = int(road_limit_speed)
      navi_msg.liveENaviData.linkLength = int(link_length)
      navi_msg.liveENaviData.currentLinkAngle = int(current_link_angle)
      navi_msg.liveENaviData.nextLinkAngle = int(next_link_angle)
      navi_msg.liveENaviData.roadName = str(road_name)
      navi_msg.liveENaviData.isHighway = bool(int(is_highway))
      navi_msg.liveENaviData.isTunnel = bool(int(is_tunnel))
      navi_msg.liveENaviData.kisaLatitude = float(kisa_lat)
      navi_msg.liveENaviData.kisaLongitude = float(kisa_lon)

      if KISA_Debug:
        navi_msg.liveENaviData.kisa0 = str(kisa_0)
        navi_msg.liveENaviData.kisa1 = str(kisa_1)
        navi_msg.liveENaviData.kisa2 = str(kisa_2)
        navi_msg.liveENaviData.kisa3 = str(kisa_3)
        navi_msg.liveENaviData.kisa4 = str(kisa_4)
        navi_msg.liveENaviData.kisa5 = str(kisa_5)
        navi_msg.liveENaviData.kisa6 = str(kisa_6)
        navi_msg.liveENaviData.kisa7 = str(kisa_7)
        navi_msg.liveENaviData.kisa8 = str(kisa_8)
        navi_msg.liveENaviData.kisa9 = str(kisa_9)

      if navi_selection == 2:
        navi_msg.liveENaviData.wazeAlertId = int(waze_alert_id)

        if waze_is_metric:
          navi_msg.liveENaviData.wazeAlertDistance = int(waze_alert_distance)
        else:
          if waze_alert_distance == "0":
            mtom1 = False
            mtom2 = False
            mtom3 = False
            navi_msg.liveENaviData.wazeAlertDistance = 0
            mtom_dist_last = 0
            waze_current_speed_prev = 0
          elif len(waze_alert_distance) in (1,2,3) and waze_alert_distance[0] != '0':
            navi_msg.liveENaviData.wazeAlertDistance = round(int(waze_alert_distance) / 3.281)
          elif int(waze_current_speed) == 0:
            navi_msg.liveENaviData.wazeAlertDistance = mtom_dist_last
          elif mtom1 and (count % int(1. / DT_TRML)) == 0:
            navi_msg.liveENaviData.wazeAlertDistance = max(152, round(mtom_dist_last - (((int(waze_current_speed) + waze_current_speed_prev)/2) / 2.237)))
            mtom_dist_last = max(152, round(mtom_dist_last - (((int(waze_current_speed) + waze_current_speed_prev)/2) / 2.237)))
            waze_current_speed_prev = int(waze_current_speed)
          elif waze_alert_distance == "01" and not mtom1:
            waze_current_speed_prev = int(waze_current_speed)
            mtom1 = True
            mtom2 = False
            mtom3 = False
            count = 0
            navi_msg.liveENaviData.wazeAlertDistance = 225
            mtom_dist_last = 225
          elif mtom2 and (count % int(1. / DT_TRML)) == 0:
            navi_msg.liveENaviData.wazeAlertDistance = max(225, round(mtom_dist_last - (((int(waze_current_speed) + waze_current_speed_prev)/2) / 2.237)))
            mtom_dist_last = max(225, round(mtom_dist_last - (((int(waze_current_speed) + waze_current_speed_prev)/2) / 2.237)))
            waze_current_speed_prev = int(waze_current_speed)
          elif waze_alert_distance == "02" and not mtom2:
            waze_current_speed_prev = int(waze_current_speed)
            mtom1 = False
            mtom2 = True
            mtom3 = False
            count = 0
            navi_msg.liveENaviData.wazeAlertDistance = 386
            mtom_dist_last = 386
          elif mtom3 and (count % int(1. / DT_TRML)) == 0:
            navi_msg.liveENaviData.wazeAlertDistance = max(386, round(mtom_dist_last - (((int(waze_current_speed) + waze_current_speed_prev)/2) / 2.237)))
            mtom_dist_last = max(386, round(mtom_dist_last - (((int(waze_current_speed) + waze_current_speed_prev)/2) / 2.237)))
            waze_current_speed_prev = int(waze_current_speed)
          elif waze_alert_distance == "03" and not mtom3:
            waze_current_speed_prev = int(waze_current_speed)
            mtom1 = False
            mtom2 = False
            mtom3 = True
            count = 0
            navi_msg.liveENaviData.wazeAlertDistance = 550
            mtom_dist_last = 550
          else:
            navi_msg.liveENaviData.wazeAlertDistance = mtom_dist_last
        navi_msg.liveENaviData.wazeRoadSpeedLimit = int(waze_road_speed_limit)
        navi_msg.liveENaviData.wazeCurrentSpeed = int(waze_current_speed)
        navi_msg.liveENaviData.wazeRoadName = str(waze_road_name)
        navi_msg.liveENaviData.wazeNavSign = int(waze_nav_sign)
        navi_msg.liveENaviData.wazeNavDistance = int(waze_nav_distance)
        navi_msg.liveENaviData.wazeAlertType = str(waze_alert_type)
        navi_msg.liveENaviData.wazeLatitude = float(waze_lat)
        navi_msg.liveENaviData.wazeLongitude = float(waze_lon)

      pm.send('liveENaviData', navi_msg)

    count += 1
    time.sleep(DT_TRML)


def main():
  nv_queue = queue.Queue(maxsize=1)
  end_event = threading.Event()

  t = threading.Thread(target=navid_thread, args=(end_event, nv_queue))

  t.start()

  try:
    while True:
      time.sleep(1)
      if not t.is_alive():
        break
  finally:
    end_event.set()

  t.join()


if __name__ == "__main__":
  main()