#!/usr/bin/env python3
import os
import queue
import threading
import time
import subprocess

from openpilot.common.params import Params
from openpilot.common.realtime import DT_HW

# KisaPilot SW Updater

def sw_update_thread(end_event, nv_queue):
  scount = 0
  lcount = 0
  params = Params()
  p_order = 0
  command1 = command2 = command3 = command4 = command5 = command6 = ""

  while not end_event.is_set():
    # Custom commands
    if (scount % int(1. / DT_HW)) == 0:
      if params.get("RunCustomCommand") is not None and params.get("RunCustomCommand") != "0":
        if len(params.get("RunCustomCommand")) > 2:
          if p_order == 0:
            selection = params.get("RunCustomCommand").decode()
            command1 = "git -C /data/openpilot clean -d -f -f"
            command2 = "git -C /data/openpilot remote set-branches --add origin " + selection
            command3 = "/data/openpilot/selfdrive/assets/addon/script/git_remove.sh"
            command31 = "git -C /data/openpilot branch -D " + selection
            command4 = "git -C /data/openpilot fetch --progress origin"
            command41 = "git -C /data/openpilot stash"
            command42 = "git -C /data/openpilot stash clear"
            command5 = "git -C /data/openpilot checkout --track origin/" + selection
            command6 = "git -C /data/openpilot checkout " + selection
            command7 = "/data/openpilot/selfdrive/assets/addon/script/git_reset.sh"
            p_order = 1
            lcount = 0
            result=subprocess.Popen(command1, shell=True)
          elif p_order == 1:
            rvalue=result.poll()
            if rvalue == 0:
              p_order = 2
              result=subprocess.Popen(command2, shell=True)
            else:
              lcount += 1
              if lcount > 30: # killing in 30sec if proc is abnormal or not completed.
                params.put("RunCustomCommand", "0")
                p_order = 0
                lcount = 0
                result.kill()
          elif p_order == 2:
            rvalue=result.poll()
            if rvalue == 0:
              p_order = 21
              result=subprocess.Popen(command3, shell=True)
            else:
              lcount += 1
              if lcount > 30: # killing in 30sec if proc is abnormal or not completed.
                params.put("RunCustomCommand", "0")
                p_order = 0
                lcount = 0
                result.kill()
          elif p_order == 21:
            rvalue=result.poll()
            if rvalue == 0:
              p_order = 3
              result=subprocess.Popen(command31, shell=True)
            else:
              lcount += 1
              if lcount > 30: # killing in 30sec if proc is abnormal or not completed.
                params.put("RunCustomCommand", "0")
                p_order = 0
                lcount = 0
                result.kill()
          elif p_order == 3:
            rvalue=result.poll()
            if rvalue == 0 or lcount > 5:
              p_order = 31
              result=subprocess.Popen(command4, shell=True)
            else:
              lcount += 1
              if lcount > 30: # killing in 30sec if proc is abnormal or not completed.
                params.put("RunCustomCommand", "0")
                p_order = 0
                lcount = 0
                result.kill()
          elif p_order == 31:
            rvalue=result.poll()
            if rvalue == 0:
              p_order = 32
              result=subprocess.Popen(command41, shell=True)
            else:
              lcount += 1
              if lcount > 30: # killing in 30sec if proc is abnormal or not completed.
                params.put("RunCustomCommand", "0")
                p_order = 0
                lcount = 0
                result.kill()
          elif p_order == 32:
            rvalue=result.poll()
            if rvalue == 0:
              p_order = 4
              result=subprocess.Popen(command42, shell=True)
            else:
              lcount += 1
              if lcount > 30: # killing in 30sec if proc is abnormal or not completed.
                params.put("RunCustomCommand", "0")
                p_order = 0
                lcount = 0
                result.kill()
          elif p_order == 4:
            rvalue=result.poll()
            if rvalue == 0:
              p_order = 5
              result=subprocess.Popen(command5, shell=True)
            else:
              lcount += 1
              if lcount > 30: # killing in 30sec if proc is abnormal or not completed.
                params.put("RunCustomCommand", "0")
                p_order = 0
                lcount = 0
                result.kill()
          elif p_order == 5:
            rvalue=result.poll()
            if rvalue in (0,128):
              p_order = 6
              result=subprocess.Popen(command6, shell=True)
            else:
              lcount += 1
              if lcount > 30: # killing in 30sec if proc is abnormal or not completed.
                params.put("RunCustomCommand", "0")
                p_order = 0
                lcount = 0
                result.kill()
          elif p_order == 6:
            rvalue=result.poll()
            if rvalue == 0:
              p_order = 7
              result=subprocess.Popen(command7, shell=True)
            else:
              lcount += 1
              if lcount > 30: # killing in 30sec if proc is abnormal or not completed.
                params.put("RunCustomCommand", "0")
                p_order = 0
                lcount = 0
                result.kill()
          elif p_order == 7:
            rvalue=result.poll()
            if rvalue == 0:
              p_order = 0
              params.put("RunCustomCommand", "0")
            else:
              lcount += 1
              if lcount > 30: # killing in 30sec if proc is abnormal or not completed.
                params.put("RunCustomCommand", "0")
                p_order = 0
                lcount = 0
                result.kill()
        elif int(params.get("RunCustomCommand", encoding="utf8")) == 1:
          if p_order == 0:
            command1 = "/data/openpilot/selfdrive/assets/addon/script/gitcommit.sh"
            p_order = 1
            lcount = 0
            result=subprocess.Popen(command1, shell=True)
          elif p_order == 1:
            rvalue=result.poll()
            if rvalue == 0:
              p_order = 0
              params.put("RunCustomCommand", "0")
            else:
              lcount += 1
              if lcount > 30: # killing in 30sec if proc is abnormal or not completed.
                params.put("RunCustomCommand", "0")
                p_order = 0
                lcount = 0
                result.kill()
        elif int(params.get("RunCustomCommand", encoding="utf8")) == 2:
          if p_order == 0:
            command1 = "/data/openpilot/selfdrive/assets/addon/script/gitpull.sh"
            p_order = 1
            lcount = 0
            result=subprocess.Popen(command1, shell=True)
          elif p_order == 1:
            rvalue=result.poll()
            if rvalue == 0:
              p_order = 0
              params.put("RunCustomCommand", "0")
            else:
              lcount += 1
              if lcount > 30: # killing in 30sec if proc is abnormal or not completed.
                params.put("RunCustomCommand", "0")
                p_order = 0
                lcount = 0
                result.kill()
        elif int(params.get("RunCustomCommand", encoding="utf8")) == 3:
          if p_order == 0:
            command1 = "rm -f /data/branches"
            command2 = "git -C /data/openpilot remote prune origin"
            command3 = "git -C /data/openpilot fetch origin"
            command4 = "git -C /data/openpilot ls-remote --refs | grep refs/heads | awk -F '/' '{print $3}' > /data/branches"
            p_order = 1
            lcount = 0
            result=subprocess.Popen(command1, shell=True)
          elif p_order == 1:
            rvalue=result.poll()
            if rvalue == 0:
              p_order = 2
              result=subprocess.Popen(command2, shell=True)
            else:
              lcount += 1
              if lcount > 30: # killing in 30sec if proc is abnormal or not completed.
                params.put("RunCustomCommand", "0")
                p_order = 0
                lcount = 0
                result.kill()
          elif p_order == 2:
            rvalue=result.poll()
            if rvalue == 0:
              p_order = 3
              result=subprocess.Popen(command3, shell=True)
            else:
              lcount += 1
              if lcount > 30: # killing in 30sec if proc is abnormal or not completed.
                params.put("RunCustomCommand", "0")
                p_order = 0
                lcount = 0
                result.kill()
          elif p_order == 3:
            rvalue=result.poll()
            if rvalue == 0:
              p_order = 4
              result=subprocess.Popen(command4, shell=True)
            else:
              lcount += 1
              if lcount > 30: # killing in 30sec if proc is abnormal or not completed.
                params.put("RunCustomCommand", "0")
                p_order = 0
                lcount = 0
                result.kill()
          elif p_order == 4:
            rvalue=result.poll()
            if rvalue == 0:
              p_order = 0
              params.put("RunCustomCommand", "0")
            else:
              lcount += 1
              if lcount > 30: # killing in 30sec if proc is abnormal or not completed.
                params.put("RunCustomCommand", "0")
                p_order = 0
                lcount = 0
                result.kill()
        elif int(params.get("RunCustomCommand", encoding="utf8")) == 4:
          if p_order == 0:
            model_name = params.get("DrivingModel", encoding="utf8")
            command1 = "wget -P /data/model https://raw.githubusercontent.com/kisapilot/model/main/models/" + model_name
            command2 = "rm -f /data/openpilot/selfdrive/modeld/models/supercombo.onnx"
            command3 = "rm -f /data/openpilot/selfdrive/modeld/models/supercombo.thneed"
            command4 = "rm -f /data/openpilot/selfdrive/modeld/models/supercombo_metadata.pkl"
            command5 = "cp -f /data/model/" + model_name + " /data/openpilot/selfdrive/modeld/models/supercombo.onnx"
            command6 = "sudo reboot"
            p_order = 1
            lcount = 0
            if not os.path.isfile("/data/model/" + model_name):
              result=subprocess.Popen(command1, shell=True)
            else:
              result=subprocess.Popen("ls", shell=True)
          elif p_order == 1:
            rvalue=result.poll()
            if rvalue == 0:
              p_order = 2
              result=subprocess.Popen(command2, shell=True)
            else:
              lcount += 1
              if lcount > 300: # killing in 180sec if proc is abnormal or not completed.
                params.put("RunCustomCommand", "0")
                p_order = 0
                lcount = 0
                result.kill()
          elif p_order == 2:
            rvalue=result.poll()
            if rvalue == 0:
              p_order = 3
              result=subprocess.Popen(command3, shell=True)
            else:
              lcount += 1
              if lcount > 30: # killing in 30sec if proc is abnormal or not completed.
                params.put("RunCustomCommand", "0")
                p_order = 0
                lcount = 0
                result.kill()
          elif p_order == 3:
            rvalue=result.poll()
            if rvalue == 0:
              p_order = 4
              result=subprocess.Popen(command4, shell=True)
            else:
              lcount += 1
              if lcount > 30: # killing in 30sec if proc is abnormal or not completed.
                params.put("RunCustomCommand", "0")
                p_order = 0
                lcount = 0
                result.kill()
          elif p_order == 4:
            rvalue=result.poll()
            if rvalue == 0:
              p_order = 5
              result=subprocess.Popen(command5, shell=True)
            else:
              lcount += 1
              if lcount > 30: # killing in 30sec if proc is abnormal or not completed.
                params.put("RunCustomCommand", "0")
                p_order = 0
                lcount = 0
                result.kill()
          elif p_order == 5:
            rvalue=result.poll()
            if rvalue == 0:
              p_order = 6
              result=subprocess.Popen(command6, shell=True)
            else:
              lcount += 1
              if lcount > 30: # killing in 30sec if proc is abnormal or not completed.
                params.put("RunCustomCommand", "0")
                p_order = 0
                lcount = 0
                result.kill()
          elif p_order == 6:
            rvalue=result.poll()
            if rvalue == 0:
              p_order = 0
              params.put("RunCustomCommand", "0")
            else:
              lcount += 1
              if lcount > 30: # killing in 30sec if proc is abnormal or not completed.
                params.put("RunCustomCommand", "0")
                p_order = 0
                lcount = 0
                result.kill()
        elif int(params.get("RunCustomCommand", encoding="utf8")) == 5:
          if p_order == 0:
            command1 = "rm -f /data/openpilot/selfdrive/modeld/models/supercombo.onnx"
            command2 = "rm -f /data/openpilot/selfdrive/modeld/models/supercombo.thneed"
            command3 = "rm -f /data/openpilot/selfdrive/modeld/models/supercombo_metadata.pkl"
            command4 = "git -C /data/openpilot/selfdrive//modeld/models checkout supercombo.onnx"
            command5 = "sudo reboot"
            p_order = 1
            lcount = 0
            result=subprocess.Popen(command1, shell=True)
          elif p_order == 1:
            rvalue=result.poll()
            if rvalue == 0:
              p_order = 2
              result=subprocess.Popen(command2, shell=True)
            else:
              lcount += 1
              if lcount > 30: # killing in 30sec if proc is abnormal or not completed.
                params.put("RunCustomCommand", "0")
                p_order = 0
                lcount = 0
                result.kill()
          elif p_order == 2:
            rvalue=result.poll()
            if rvalue == 0:
              p_order = 3
              result=subprocess.Popen(command3, shell=True)
            else:
              lcount += 1
              if lcount > 30: # killing in 30sec if proc is abnormal or not completed.
                params.put("RunCustomCommand", "0")
                p_order = 0
                lcount = 0
                result.kill()
          elif p_order == 3:
            rvalue=result.poll()
            if rvalue == 0:
              p_order = 4
              result=subprocess.Popen(command4, shell=True)
            else:
              lcount += 1
              if lcount > 30: # killing in 30sec if proc is abnormal or not completed.
                params.put("RunCustomCommand", "0")
                p_order = 0
                lcount = 0
                result.kill()
          elif p_order == 4:
            rvalue=result.poll()
            if rvalue == 0:
              p_order = 5
              result=subprocess.Popen(command5, shell=True)
            else:
              lcount += 1
              if lcount > 30: # killing in 30sec if proc is abnormal or not completed.
                params.put("RunCustomCommand", "0")
                p_order = 0
                lcount = 0
                result.kill()
          elif p_order == 5:
            rvalue=result.poll()
            if rvalue == 0:
              p_order = 0
              params.put("RunCustomCommand", "0")
            else:
              lcount += 1
              if lcount > 30: # killing in 30sec if proc is abnormal or not completed.
                params.put("RunCustomCommand", "0")
                p_order = 0
                lcount = 0
                result.kill()
    scount += 1
    time.sleep(DT_HW)


def main():
  nv_queue = queue.Queue(maxsize=1)
  end_event = threading.Event()

  t = threading.Thread(target=sw_update_thread, args=(end_event, nv_queue))

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
