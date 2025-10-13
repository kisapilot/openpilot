#!/usr/bin/env python3
import threading
import socket
import re
import time

import cereal.messaging as messaging
from openpilot.common.params import Params
from openpilot.common.realtime import Ratekeeper

# KisaPilot, this is for getting navi data from external device using UDP broadcast.

class ENaviUDP:
  def __init__(self, pm):
    self.pm = pm
    self.spd_limit = 0
    self.safety_distance = 0
    self.safety_bl_distance = 0
    self.sign_type = ''
    self.road_limit_speed = 0
    self.road_name = ''

    self.waze_current_speed = 0
    self.waze_current_speed_prev = 0
    self.waze_road_speed_limit = 0
    self.waze_road_speed_limit_keep = 0
    self.waze_alert_id = 0
    self.waze_alert_distance = ""
    self.waze_alert_distance_raw = ""
    self.waze_alert_str = ""
    self.waze_road_name = ""
    self.mtom1 = self.mtom2 = self.mtom3 = self.mtom4 = False
    self.mtom_dist_last = 0
    self.check_connection = False
    self.waze_dist_longer_meter = 0
    self.waze_alert_trigger_start = False

    self.cnt1 = self.cnt2 = 0
    self.cnt_threshold = 1

    self.navi_selection = Params().get("KISANaviSelect", return_default=True)
    self.is_metric = Params().get_bool("IsMetric")

    self.lock = threading.Lock()


  def udp_broadcast_listener(self):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.bind(('', 12345))
    sock.settimeout(3)

    while True:
      try:
        data, addr = sock.recvfrom(1024)
        decoded_data = data.decode('utf-8')

        if self.navi_selection == 1:  # Tmap
          self.handle_tmap_data(decoded_data)
        elif self.navi_selection == 2:  # Waze
          self.handle_waze_data(decoded_data)
      except socket.timeout:
        self.reset_data()
    
    sock.close()


  def handle_tmap_data(self, data):
    if "kisasigntype" in data:
      data_dict = {pair.split(':')[0]: pair.split(':')[1] for pair in data.split('/') if pair}
      with self.lock:
        self.cnt1 = 0
        self.check_connection = True
        self.spd_limit = int(data_dict.get('kisaspdlimit'))
        self.safety_distance = float(data_dict.get('kisaspddist'))
        self.safety_bl_distance = float(data_dict.get('kisaspdbldist'))
        self.sign_type = str(data_dict.get('kisasigntype'))
    elif "Kisa_Tmap_Alive" in data:
      with self.lock:
        self.cnt1 += 1
        self.check_connection = True
        if self.cnt1 > self.cnt_threshold:
          self.cnt1 = 0
          self.spd_limit = 0
          self.safety_distance = 0.0
          self.safety_bl_distance = 0.0
          self.sign_type = ''
    if "kisaroadlimitspd" in data:
      data_dict = {pair.split(':')[0]: pair.split(':')[1] for pair in data.split('/') if pair}
      with self.lock:
        self.road_limit_speed = int(data_dict.get('kisaroadlimitspd'))
        self.road_name = str(data_dict.get('kisaroadname'))
        self.cnt2 = 0
        self.check_connection = True
    elif "Kisa_Tmap_Alive" in data:
      with self.lock:
        self.cnt2 += 1
        self.check_connection = True
        if self.cnt2 > self.cnt_threshold+2:
          self.cnt2 = 0
          self.road_limit_speed = 0
          self.road_name = ''


  def handle_waze_data(self, data):
    if "kisawazealert" in data:
      data_dict = {pair.split(':')[0]: pair.split(':')[1] for pair in data.split('/') if pair}
      with self.lock:
        self.waze_alert_trigger_start = True
        self.waze_alert_str = str(data_dict.get('kisawazealert'))
        if "camera" in self.waze_alert_str or "Camera" in self.waze_alert_str:
          self.waze_alert_id = 1
          self.waze_alert_str = "camera"
        elif "police" in self.waze_alert_str or "Police" in self.waze_alert_str:
          self.waze_alert_id = 2
          self.waze_alert_str = "police"
        else:
          self.waze_alert_id = 0
        waze_alert_distance_str = data_dict.get('kisawazealert')
        if waze_alert_distance_str not in (None, ""):
          self.waze_alert_distance_raw = str(waze_alert_distance_str)
          self.waze_alert_distance = str(re.sub(r'[^0-9]', '', self.waze_alert_distance_raw))
        else:
          self.waze_alert_distance = ""
          self.waze_alert_distance_raw = ""
    if "kisawazeendalert" in data:
      data_dict = {pair.split(':')[0]: pair.split(':')[1] for pair in data.split('/') if pair}
      with self.lock:
        self.waze_alert_distance = ""
    if "kisawazeroadname" in data:
      data_dict = {pair.split(':')[0]: pair.split(':')[1] for pair in data.split('/') if pair}
      with self.lock:
        self.waze_road_name = str(data_dict.get('kisawazeroadname'))
    if "kisawazecurrentspd" in data:
      data_dict = {pair.split(':')[0]: pair.split(':')[1] for pair in data.split('/') if pair}
      with self.lock:
        self.waze_current_speed = int(data_dict.get('kisawazecurrentspd'))
        self.waze_road_speed_limit = int(data_dict.get('kisawazeroadspdlimit'))
        self.check_connection = True


  def reset_data(self):
    with self.lock:
      if self.navi_selection == 1:
        self.spd_limit = 0
        self.safety_distance = 0.0
        self.safety_bl_distance = 0.0
        self.sign_type = ''
        self.road_limit_speed = 0
        self.road_name = ''
        self.check_connection = False
      elif self.navi_selection == 2:
        self.waze_current_speed = 0
        self.waze_road_speed_limit = 0
        self.waze_alert_id = 0
        self.waze_alert_distance = ""
        self.waze_alert_distance_raw = ""
        self.waze_road_name = ""
        self.check_connection = False
        self.cnt1 = 0
        self.waze_alert_trigger_start = False


  def update(self):
    navi_msg = messaging.new_message('liveENaviData')
    with self.lock:
      if self.navi_selection == 1:
        navi_msg.liveENaviData.speedLimit = int(self.spd_limit)
        if self.safety_distance != 0.0:
          navi_msg.liveENaviData.safetyDistance = float(self.safety_distance)
        else:
          navi_msg.liveENaviData.safetyDistance = float(self.safety_bl_distance)
        navi_msg.liveENaviData.safetySign = str(self.sign_type)
        navi_msg.liveENaviData.roadLimitSpeed = int(self.road_limit_speed)  
        navi_msg.liveENaviData.roadName = str(self.road_name)
        navi_msg.liveENaviData.connectionAlive = bool(self.check_connection)
      elif self.navi_selection == 2:
        navi_msg.liveENaviData.wazeAlertId = int(self.waze_alert_id)
        if self.is_metric:
          navi_msg.liveENaviData.wazeAlertDistance = int(self.waze_alert_distance) if self.waze_alert_distance not in ("", None) else 0
          navi_msg.liveENaviData.wazeAlertDistanceRaw = str(self.waze_alert_distance_raw)
        else:
          if self.waze_alert_trigger_start:
            if self.waze_alert_distance == "" or self.waze_alert_distance == "0":
              if self.waze_dist_longer_meter > 0:
                self.waze_dist_longer_meter = max(0, round(self.waze_dist_longer_meter - (((int(self.waze_current_speed) + self.waze_current_speed_prev)/2) / 2.237)))
                navi_msg.liveENaviData.wazeAlertDistance = int(self.waze_dist_longer_meter)
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
                self.waze_alert_trigger_start = False
                self.waze_alert_str = ""
                self.waze_alert_id = 0
                self.waze_alert_distance = ""
                self.waze_alert_distance_raw = ""
            elif len(self.waze_alert_distance) in (1,2,3) and self.waze_alert_distance[0] != '0':
              self.waze_dist_longer_meter = 402
              self.waze_road_speed_limit_keep = int(self.waze_road_speed_limit)
              navi_msg.liveENaviData.wazeAlertDistance = round(int(self.waze_alert_distance) / 3.281)
            elif int(self.waze_current_speed) == 0:
              navi_msg.liveENaviData.wazeAlertDistance = int(self.mtom_dist_last)
            elif self.waze_alert_distance == "01" and self.mtom1:
              self.mtom_dist_last = max(152, round(self.mtom_dist_last - (((int(self.waze_current_speed) + self.waze_current_speed_prev)/2) / 2.237)))
              navi_msg.liveENaviData.wazeAlertDistance = int(self.mtom_dist_last)
              self.waze_current_speed_prev = int(self.waze_current_speed)
            elif self.waze_alert_distance == "01" and not self.mtom1:
              self.waze_current_speed_prev = int(self.waze_current_speed)
              self.mtom1 = True
              self.mtom2 = False
              self.mtom3 = False
              self.mtom4 = False
              navi_msg.liveENaviData.wazeAlertDistance = 305
              self.mtom_dist_last = 305
            elif self.waze_alert_distance == "02" and self.mtom2:
              self.mtom_dist_last = max(305, round(self.mtom_dist_last - (((int(self.waze_current_speed) + self.waze_current_speed_prev)/2) / 2.237)))
              navi_msg.liveENaviData.wazeAlertDistance = int(self.mtom_dist_last)
              self.waze_current_speed_prev = int(self.waze_current_speed)
            elif self.waze_alert_distance == "02" and not self.mtom2:
              self.waze_current_speed_prev = int(self.waze_current_speed)
              self.mtom1 = False
              self.mtom2 = True
              self.mtom3 = False
              self.mtom4 = False
              navi_msg.liveENaviData.wazeAlertDistance = 466
              self.mtom_dist_last = 466
            elif self.waze_alert_distance == "03" and self.mtom3:
              self.mtom_dist_last = max(466, round(self.mtom_dist_last - (((int(self.waze_current_speed) + self.waze_current_speed_prev)/2) / 2.237)))
              navi_msg.liveENaviData.wazeAlertDistance = int(self.mtom_dist_last)
              self.waze_current_speed_prev = int(self.waze_current_speed)
            elif self.waze_alert_distance == "03" and not self.mtom3:
              self.waze_current_speed_prev = int(self.waze_current_speed)
              self.mtom1 = False
              self.mtom2 = False
              self.mtom3 = True
              self.mtom4 = False
              navi_msg.liveENaviData.wazeAlertDistance = 579
              self.mtom_dist_last = 579
            elif self.waze_alert_distance == "04" and self.mtom4:
              self.mtom_dist_last = max(579, round(self.mtom_dist_last - (((int(self.waze_current_speed) + self.waze_current_speed_prev)/2) / 2.237)))
              navi_msg.liveENaviData.wazeAlertDistance = int(self.mtom_dist_last)
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
              navi_msg.liveENaviData.wazeAlertDistance = int(self.mtom_dist_last)
        navi_msg.liveENaviData.wazeAlertDistanceRaw = str(self.waze_alert_distance_raw)
        navi_msg.liveENaviData.wazeRoadSpeedLimit = int(self.waze_road_speed_limit)
        navi_msg.liveENaviData.wazeCurrentSpeed = int(self.waze_current_speed)
        navi_msg.liveENaviData.wazeAlertType = str(self.waze_alert_str)
        navi_msg.liveENaviData.wazeRoadName = str(self.waze_road_name)
        navi_msg.liveENaviData.connectionAlive = bool(self.check_connection)

    self.pm.send('liveENaviData', navi_msg)


def main():
  pm = messaging.PubMaster(['liveENaviData'])

  rk = Ratekeeper(1.0, print_delay_threshold=None)
  e_navi_udp = ENaviUDP(pm)

  udp_thread = threading.Thread(target=e_navi_udp.udp_broadcast_listener, daemon=False)
  udp_thread.start()

  while True:
    e_navi_udp.update()
    rk.keep_time()


if __name__ == "__main__":
  main()
