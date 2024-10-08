#!/usr/bin/env python3
import os
import queue
import threading
import time
import subprocess
import re
import json
import random

import cereal.messaging as messaging
from openpilot.common.params import Params

from openpilot.common.realtime import Ratekeeper

import zmq

# KisaPilot, this is for getting navi data from external device.

class ENavi:
  def __init__(self, pm):
    self.pm = pm
    self.count = 1

    self.params = Params()

    self.spd_limit = 0
    self.safety_distance = 0
    self.sign_type = 0
    self.turn_info = 0
    self.turn_distance = 0
    self.road_limit_speed = 0
    self.road_limit_speed_cnt = 0
    self.link_length = 0
    self.current_link_angle = 0
    self.next_link_angle = 0
    self.road_name = ""
    self.is_highway = 0
    self.is_tunnel = 0
    self.kisa_lat = 0
    self.kisa_lon = 0
    self.kisa_lat_prev = 0
    self.kisa_lon_prev = 0

    self.dest_changed = False
    self.dest_changed_count = 0
    self.dest = {"latitude": 0.0, "longitude": 0.0,}
    self.waypoints = [(0.0, 0.0),]

    self.Auto_IP = int(self.params.get("ExternalDeviceIPAuto", encoding="utf8"))

    self.KISA_Debug = self.params.get_bool("KISADebug")
    if self.KISA_Debug:
      self.kisa_0 = ""
      self.kisa_1 = ""
      self.kisa_2 = ""
      self.kisa_3 = ""
      self.kisa_4 = ""
      self.kisa_5 = ""
      self.kisa_6 = ""
      self.kisa_7 = ""
      self.kisa_8 = ""
      self.kisa_9 = ""

    self.ip_bind = False
    self.count2 = 70

    self.ip_list_out = []
    self.result = []
  
    self.check_connection = False
    self.check_connection_count = 0

    self.message = ""

    self.ip_count = 0

    self.is_ip_add = False

    if not self.Auto_IP:
      try:
        self.ip_count = int(len(self.params.get("ExternalDeviceIP", encoding="utf8").split(',')))
        if self.ip_count > 0:
          ip_list = self.params.get("ExternalDeviceIP", encoding="utf8").split(',')
          for input_list in ip_list:
            if '-' in input_list:
              t_out = input_list.split('-')
              left1 = t_out[0].split('.')
              leftp = left1[-1]
              right1 = t_out[-1].split('.')
              rightp = right1[0]
              for x in range(int(leftp), int(rightp)+1):
                self.ip_list_out.append(input_list.replace(leftp+"-"+rightp, str(x)))
            else:
              self.ip_list_out.append(input_list)
          random.shuffle(self.ip_list_out)
      except:
        self.ip_count = 0
        pass
    self.is_metric = self.params.get_bool("IsMetric")
    self.navi_selection = int(self.params.get("KISANaviSelect", encoding="utf8"))

    self.mtom4 = False
    self.mtom3 = False
    self.mtom2 = False
    self.mtom1 = False
    self.mtom_dist_last = 0
    self.waze_dist_longer_meter = 0
    self.waze_road_speed_limit_keep = 0

    if self.navi_selection == 2:
      self.waze_alert_id = 0
      self.waze_alert_distance = "0"
      self.waze_road_speed_limit = 0
      self.waze_current_speed = 0
      self.waze_road_name = ""
      self.waze_nav_sign = 0
      self.waze_nav_distance = 0
      self.waze_alert_type = ""
      self.waze_is_metric = self.params.get_bool("IsMetric")
      self.waze_current_speed_prev = 0
      self.waze_lat = 0
      self.waze_lon = 0
      self.waze_lat_prev = 0
      self.waze_lon_prev = 0

  def int_to_degree(self, int_value):
    int_value = (int_value/ 100) / 3600
    degree = int_value
    int_value = 60 * (int_value - degree)
    minute = int_value
    int_value = 60 * (int_value - minute)
    second = int_value
    return(float(degree) + float(minute)/60 + float(second)/(60*60))

  def update(self):
    self.count += 1
    if not self.ip_bind:
      if self.Auto_IP and self.count in (30, 90, 150, 210, 270, 330, 390, 450, 510, 570, 630, 690, 750, 810, 870, 930, 990):
        out = subprocess.check_output("ip route", shell=True)
        try:
          ip_via = str(out.strip().decode()).split('via ')[1].split(' ')[0]
          if self.Auto_IP == 1:
            ip_src = str(out.strip().decode()).split('src ')[1].split(' ')[0]
            compare1 = ip_src.split('.')
            c_num = 0
            for compare2 in ip_via.split('.'):
              if compare2 != compare1[c_num]:
                break
              else:
                c_num += 1
            compare1[c_num] = '*'
            ip_s = '.'.join(compare1)
            for x in range(1, 255):
              self.ip_list_out.append(ip_s.replace('*', str(x)))
            self.ip_list_out.remove(ip_src)
            random.shuffle(self.ip_list_out)
          elif self.Auto_IP == 2:
            self.ip_list_out.append(ip_via)
          self.is_ip_add = True
        except:
          self.is_ip_add = False
          pass
      if not self.Auto_IP or (self.Auto_IP and self.is_ip_add):
        if (self.count % 60) == 0:
          self.count2 = self.count + 10
          self.result = []
          for address in self.ip_list_out:
              p = subprocess.Popen(['ping', '-c', '1', '-W', '1', '-q', address])
              self.result.append(p)
        elif (self.count % self.count2) == 0:
          for ip, p in zip(self.ip_list_out, self.result):
              if p.wait() == 0:
                  res = subprocess.call(['nc', '-vz', ip, '5555'])
                  if res == 0:
                    self.params.put_nonblocking("ExternalDeviceIPNow", ip)
                    self.ip_bind = True
                    self.check_connection = True
                    self.context = zmq.Context()
                    self.socket = self.context.socket(zmq.SUB)
                    self.socket.connect("tcp://" + str(ip) + ":5555")
                    break

    navi_msg = messaging.new_message('liveENaviData')
    if self.ip_bind:
      self.spd_limit = 0
      self.safety_distance = 0
      self.sign_type = 0
      self.turn_info = 0
      self.turn_distance = 0
      #self.road_limit_speed = 0
      self.link_length = 0
      self.current_link_angle = 0
      self.next_link_angle = 0
      self.road_name = ""
      self.is_highway = 0
      self.is_tunnel = 0

      if self.navi_selection == 2:
        if (self.count % 3) == 0 and int(self.waze_current_speed) > 2:
          self.waze_alert_id = 0
          self.waze_alert_distance = "0"
          self.waze_alert_type = ""
        if (self.count % 5) == 0 and int(self.waze_current_speed) > 2 and int(self.waze_nav_distance) < 30:
          self.waze_nav_sign = 0
          self.waze_nav_distance = 0
        self.waze_current_speed = 0

      if self.KISA_Debug:
        self.kisa_0 = ""
        self.kisa_1 = ""
        self.kisa_2 = ""
        self.kisa_3 = ""
        self.kisa_4 = ""
        self.kisa_5 = ""
        self.kisa_6 = ""
        self.kisa_7 = ""
        self.kisa_8 = ""
        self.kisa_9 = ""

      self.socket.subscribe("")
      try:
        if self.check_connection_count > 5:
          self.message = ""
          self.check_connection = False
        else:
          self.check_connection = True
        self.message = str(self.socket.recv(flags=zmq.NOBLOCK), 'utf-8')
        self.check_connection_count -= 1 if self.check_connection_count > 0 else 0
      except zmq.ZMQError:
        self.check_connection_count = min(8, self.check_connection_count+2)
        pass
      
      for line in self.message.split('\n'):
        if self.navi_selection == 1:
          if "kisaspdlimit" in line:
            arr = line.split('kisaspdlimit: ')
            self.spd_limit = arr[1]
          if "kisaspddist" in line:
            arr = line.split('kisaspddist: ')
            self.safety_distance = arr[1]
          if "kisasigntype" in line:
            arr = line.split('kisasigntype: ')
            self.sign_type = arr[1]
          if "kisaturninfo" in line:
            arr = line.split('kisaturninfo: ')
            self.turn_info = arr[1]
          if "kisadistancetoturn" in line:
            arr = line.split('kisadistancetoturn: ')
            self.turn_distance = arr[1]
          if "kisaroadlimitspd" in line:
            arr = line.split('kisaroadlimitspd: ')
            self.road_limit_speed = arr[1]
            self.road_limit_speed_cnt = 0
          elif (self.count % 2) == 0:
            self.road_limit_speed_cnt += 1
            if self.road_limit_speed_cnt > 30:
              self.road_limit_speed = 0
              self.road_limit_speed_cnt = 0
          if "kisalinklength" in line:
            arr = line.split('kisalinklength: ')
            self.link_length = arr[1]
          if "kisacurrentlinkangle" in line:
            arr = line.split('kisacurrentlinkangle: ')
            self.current_link_angle = arr[1]
          if "kisanextlinkangle" in line:
            arr = line.split('kisanextlinkangle: ')
            self.next_link_angle = arr[1]
          if "kisaroadname" in line:
            arr = line.split('kisaroadname: ')
            self.road_name = arr[1]
          if "kisaishighway" in line:
            arr = line.split('kisaishighway: ')
            self.is_highway = arr[1]
          if "kisaistunnel" in line:
            arr = line.split('kisaistunnel: ')
            self.is_tunnel = arr[1]
          if "kisadestlat" in line:
            arr = line.split('kisadestlat: ')
            self.kisa_lat = arr[1]
            if int(float(self.kisa_lat)) > 180:
              self.kisa_lat = self.int_to_degree(int(arr[1]))
          if "kisadestlon" in line:
            arr = line.split('kisadestlon: ')
            self.kisa_lon = arr[1]
            if int(float(self.kisa_lon)) > 360:
              self.kisa_lon = self.int_to_degree(int(arr[1]))

          if self.kisa_lat and self.kisa_lon and self.kisa_lat != self.kisa_lat_prev and self.kisa_lon != self.kisa_lon_prev:
            self.dest_changed = True
            self.dest_changed_count = 0
            self.kisa_lat_prev = self.kisa_lat
            self.kisa_lon_prev = self.kisa_lon
            kisa_lat_ = float(self.kisa_lat)
            kisa_lon_ = float(self.kisa_lon)
            self.dest = {"latitude": kisa_lat_, "longitude": kisa_lon_,}
            self.waypoints = [(kisa_lon_, kisa_lat_),]
          elif self.dest_changed:
            self.dest_changed_count += 1
            if self.dest_changed_count > 2:
              self.dest_changed = False
        elif self.navi_selection == 2:
          if "kisawazereportid" in line:
            arr = line.split('kisawazereportid: ')
            try:
              self.waze_alert_type = arr[1]
              if "icon_report_speedlimit" in arr[1]:
                self.waze_alert_id = 1
              elif "icon_report_camera" in arr[1]:
                self.waze_alert_id = 1
              elif "icon_report_speedcam" in arr[1]:
                self.waze_alert_id = 1
              elif "icon_report_police" in arr[1]:
                self.waze_alert_id = 2
              elif "icon_report_hazard" in arr[1]:
                self.waze_alert_id = 3
              elif "icon_report_traffic" in arr[1]:
                self.waze_alert_id = 4
            except:
              pass
          if "kisawazealertdist" in line:
            arr = line.split('kisawazealertdist: ')
            try:
              if arr[1] is None or arr[1] == "":
                self.waze_alert_distance = "0"
              else:
                self.waze_alert_distance = str(re.sub(r'[^0-9]', '', arr[1]))
            except:
              pass
          if "kisawazeroadspdlimit" in line:
            arr = line.split('kisawazeroadspdlimit: ')
            try:
              if arr[1] == "-1":
                self.waze_road_speed_limit = 0
              elif arr[1] is None or arr[1] == "":
                self.waze_road_speed_limit = 0
              else:
                self.waze_road_speed_limit = arr[1]
            except:
              self.waze_road_speed_limit = 0
              pass
          if "kisawazecurrentspd" in line:
            arr = line.split('kisawazecurrentspd: ')
            try:
              self.waze_current_speed = arr[1]
            except:
              pass
          if "kisawazeroadname" in line: # route should be set.
            arr = line.split('kisawazeroadname: ')
            try:
              self.waze_road_name = arr[1]
            except:
              pass
          if "kisawazenavsign" in line: # route should be set.
            arr = line.split('kisawazenavsign: ')
            try:
              self.waze_nav_sign = arr[1]
            except:
              pass
          if "kisawazenavdist" in line: # route should be set.
            arr = line.split('kisawazenavdist: ')
            try:
              self.waze_nav_distance = arr[1]
            except:
              pass
          if "kisawazedestlat" in line: # route should be set.
            arr = line.split('kisawazedestlat: ')
            try:
              waze_lat_temp = arr[1]
              waze_lat_temp_back = waze_lat_temp[-6:]
              waze_lat_temp_front_temp = waze_lat_temp.split(waze_lat_temp_back)
              waze_lat_temp_front = waze_lat_temp_front_temp[0]
              self.waze_lat = waze_lat_temp_front + "." + waze_lat_temp_back
            except:
              pass
          if "kisawazedestlon" in line: # route should be set.
            arr = line.split('kisawazedestlon: ')
            try:
              waze_lon_temp = arr[1]
              waze_lon_temp_back = waze_lon_temp[-6:]
              waze_lon_temp_front_temp = waze_lon_temp.split(waze_lon_temp_back)
              waze_lon_temp_front = waze_lon_temp_front_temp[0]
              self.waze_lon = waze_lon_temp_front + "." + waze_lon_temp_back
            except:
              pass

          if self.waze_lat and self.waze_lon and self.waze_lat != self.waze_lat_prev and self.waze_lon != self.waze_lon_prev:
            self.dest_changed = True
            self.dest_changed_count = 0
            self.waze_lat_prev = self.waze_lat
            self.waze_lon_prev = self.waze_lon
            waze_lat_ = float(self.waze_lat)
            waze_lon_ = float(self.waze_lon)
            self.dest = {"latitude": waze_lat_, "longitude": waze_lon_,}
            self.waypoints = [(waze_lon_, waze_lat_),]
          elif self.dest_changed:
            self.dest_changed_count += 1
            if self.dest_changed_count > 2:
              self.dest_changed = False

        if self.KISA_Debug:
          try:
            if "kisa0" in line:
              arr = line.split('kisa0   : ')
              self.kisa_0 = arr[1]
          except:
            pass
          try:
            if "kisa1" in line:
              arr = line.split('kisa1   : ')
              self.kisa_1 = arr[1]
          except:
            pass
          try:
            if "kisa2" in line:
              arr = line.split('kisa2   : ')
              self.kisa_2 = arr[1]
          except:
            pass
          try:
            if "kisa3" in line:
              arr = line.split('kisa3   : ')
              self.kisa_3 = arr[1]
          except:
            pass
          try:
            if "kisa4" in line:
              arr = line.split('kisa4   : ')
              self.kisa_4 = arr[1]
          except:
            pass
          try:
            if "kisa5" in line:
              arr = line.split('kisa5   : ')
              self.kisa_5 = arr[1]
          except:
            pass
          try:
            if "kisa6" in line:
              arr = line.split('kisa6   : ')
              self.kisa_6 = arr[1]
          except:
            pass
          try:
            if "kisa7" in line:
              arr = line.split('kisa7   : ')
              self.kisa_7 = arr[1]
          except:
            pass
          try:
            if "kisa8" in line:
              arr = line.split('kisa8   : ')
              self.kisa_8 = arr[1]
          except:
            pass
          try:
            if "kisa9" in line:
              arr = line.split('kisa9   : ')
              self.kisa_9 = arr[1]
          except:
            pass

      if self.navi_selection == 1:
        navi_msg.liveENaviData.speedLimit = int(self.spd_limit)
        navi_msg.liveENaviData.safetyDistance = float(self.safety_distance)
        navi_msg.liveENaviData.safetySign = int(self.sign_type)
        navi_msg.liveENaviData.turnInfo = int(self.turn_info)
        navi_msg.liveENaviData.distanceToTurn = float(self.turn_distance)
        navi_msg.liveENaviData.connectionAlive = bool(self.check_connection)
        navi_msg.liveENaviData.roadLimitSpeed = int(self.road_limit_speed)
        navi_msg.liveENaviData.linkLength = int(self.link_length)
        navi_msg.liveENaviData.currentLinkAngle = int(self.current_link_angle)
        navi_msg.liveENaviData.nextLinkAngle = int(self.next_link_angle)
        navi_msg.liveENaviData.roadName = str(self.road_name)
        navi_msg.liveENaviData.isHighway = bool(int(self.is_highway))
        navi_msg.liveENaviData.isTunnel = bool(int(self.is_tunnel))
        navi_msg.liveENaviData.kisaLatitude = float(self.kisa_lat)
        navi_msg.liveENaviData.kisaLongitude = float(self.kisa_lon)
      elif self.navi_selection == 2:
        navi_msg.liveENaviData.connectionAlive = bool(self.check_connection)
        navi_msg.liveENaviData.wazeAlertId = int(self.waze_alert_id)
        if self.waze_is_metric:
          navi_msg.liveENaviData.wazeAlertDistance = int(self.waze_alert_distance)
        else:
          if self.waze_alert_distance == "0":
            if self.waze_dist_longer_meter > 0:
              self.waze_dist_longer_meter = round(self.waze_dist_longer_meter - (((int(self.waze_current_speed) + self.waze_current_speed_prev)/2) / 2.237))
              navi_msg.liveENaviData.wazeAlertDistance = self.waze_dist_longer_meter
              self.waze_road_speed_limit = self.waze_road_speed_limit_keep
              navi_msg.liveENaviData.wazeAlertExtend = True
            else:
              self.mtom1 = False
              self.mtom2 = False
              self.mtom3 = False
              self.mtom4 = False
              navi_msg.liveENaviData.wazeAlertDistance = 0
              self.mtom_dist_last = 0
              self.waze_current_speed_prev = 0
              self.waze_dist_longer_meter = 0
              self.waze_road_speed_limit_keep = 0
              navi_msg.liveENaviData.wazeAlertExtend = False
          elif len(self.waze_alert_distance) in (1,2,3) and self.waze_alert_distance[0] != '0':
            self.waze_dist_longer_meter = 402
            self.waze_road_speed_limit_keep = int(self.waze_road_speed_limit)
            navi_msg.liveENaviData.wazeAlertDistance = round(int(self.waze_alert_distance) / 3.281)
          elif int(self.waze_current_speed) == 0:
            navi_msg.liveENaviData.wazeAlertDistance = self.mtom_dist_last
          elif self.mtom1:
            self.mtom_dist_last = max(152, round(self.mtom_dist_last - (((int(self.waze_current_speed) + self.waze_current_speed_prev)/2) / 2.237)))
            navi_msg.liveENaviData.wazeAlertDistance = self.mtom_dist_last
            self.waze_current_speed_prev = int(self.waze_current_speed)
          elif self.waze_alert_distance == "01" and not self.mtom1:
            self.waze_current_speed_prev = int(self.waze_current_speed)
            self.mtom1 = True
            self.mtom2 = False
            self.mtom3 = False
            self.mtom4 = False
            navi_msg.liveENaviData.wazeAlertDistance = 305
            self.mtom_dist_last = 305
          elif self.mtom2:
            self.mtom_dist_last = max(305, round(self.mtom_dist_last - (((int(self.waze_current_speed) + self.waze_current_speed_prev)/2) / 2.237)))
            navi_msg.liveENaviData.wazeAlertDistance = self.mtom_dist_last
            self.waze_current_speed_prev = int(self.waze_current_speed)
          elif self.waze_alert_distance == "02" and not self.mtom2:
            self.waze_current_speed_prev = int(self.waze_current_speed)
            self.mtom1 = False
            self.mtom2 = True
            self.mtom3 = False
            self.mtom4 = False
            navi_msg.liveENaviData.wazeAlertDistance = 466
            self.mtom_dist_last = 466
          elif self.mtom3:
            self.mtom_dist_last = max(466, round(self.mtom_dist_last - (((int(self.waze_current_speed) + self.waze_current_speed_prev)/2) / 2.237)))
            navi_msg.liveENaviData.wazeAlertDistance = self.mtom_dist_last
            self.waze_current_speed_prev = int(self.waze_current_speed)
          elif self.waze_alert_distance == "03" and not self.mtom3:
            self.waze_current_speed_prev = int(self.waze_current_speed)
            self.mtom1 = False
            self.mtom2 = False
            self.mtom3 = True
            self.mtom4 = False
            navi_msg.liveENaviData.wazeAlertDistance = 579
            self.mtom_dist_last = 579
          elif self.mtom4:
            self.mtom_dist_last = max(466, round(self.mtom_dist_last - (((int(self.waze_current_speed) + self.waze_current_speed_prev)/2) / 2.237)))
            navi_msg.liveENaviData.wazeAlertDistance = self.mtom_dist_last
            self.waze_current_speed_prev = int(self.waze_current_speed)
          elif self.waze_alert_distance == "04" and not self.mtom4:
            self.waze_current_speed_prev = int(self.waze_current_speed)
            self.mtom1 = False
            self.mtom2 = False
            self.mtom3 = False
            self.mtom4 = True
            navi_msg.liveENaviData.wazeAlertDistance = 740
            self.mtom_dist_last = 740
          else:
            navi_msg.liveENaviData.wazeAlertDistance = self.mtom_dist_last
        navi_msg.liveENaviData.wazeRoadSpeedLimit = int(self.waze_road_speed_limit)
        navi_msg.liveENaviData.wazeCurrentSpeed = int(self.waze_current_speed)
        navi_msg.liveENaviData.wazeRoadName = str(self.waze_road_name)
        navi_msg.liveENaviData.wazeNavSign = int(self.waze_nav_sign)
        navi_msg.liveENaviData.wazeNavDistance = int(self.waze_nav_distance)
        navi_msg.liveENaviData.wazeAlertType = str(self.waze_alert_type)
        navi_msg.liveENaviData.wazeLatitude = float(self.waze_lat)
        navi_msg.liveENaviData.wazeLongitude = float(self.waze_lon)

      if self.KISA_Debug:
        navi_msg.liveENaviData.kisa0 = str(self.kisa_0)
        navi_msg.liveENaviData.kisa1 = str(self.kisa_1)
        navi_msg.liveENaviData.kisa2 = str(self.kisa_2)
        navi_msg.liveENaviData.kisa3 = str(self.kisa_3)
        navi_msg.liveENaviData.kisa4 = str(self.kisa_4)
        navi_msg.liveENaviData.kisa5 = str(self.kisa_5)
        navi_msg.liveENaviData.kisa6 = str(self.kisa_6)
        navi_msg.liveENaviData.kisa7 = str(self.kisa_7)
        navi_msg.liveENaviData.kisa8 = str(self.kisa_8)
        navi_msg.liveENaviData.kisa9 = str(self.kisa_9)
    else:
      navi_msg.liveENaviData.connectionAlive = bool(self.check_connection)
    self.pm.send('liveENaviData', navi_msg)


def main():
  pm = messaging.PubMaster(['liveENaviData'])

  rk = Ratekeeper(1.0, print_delay_threshold=None)
  e_navi = ENavi(pm)
  while True:
    e_navi.update()
    rk.keep_time()


if __name__ == "__main__":
  main()
