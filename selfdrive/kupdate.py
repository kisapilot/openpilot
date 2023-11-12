#!/usr/bin/env python3
import os
import queue
import threading
import time
import subprocess

from openpilot.common.params import Params
from openpilot.common.realtime import DT_TRML

# KisaPilot SW Updater

def sw_update_thread(end_event, nv_queue):
  scount = 0
  lcount = 0
  params = Params()
  p_order = 0
  command1 = command2 = command3 = command4 = command5 = command6 = ""

  while not end_event.is_set():
    # Custom commands
    if (scount % int(1. / DT_TRML)) == 0:
      if params.get("RunCustomCommand") is not None and params.get("RunCustomCommand") != "0":
        if len(params.get("RunCustomCommand")) > 2:
          if p_order == 0:
            selection = params.get("RunCustomCommand").decode()
            command1 = "git -C /data/openpilot clean -d -f -f"
            command2 = "git -C /data/openpilot remote set-branches --add origin " + selection
            command3 = "/data/openpilot/selfdrive/assets/addon/script/git_remove.sh"
            command4 = "git -C /data/openpilot fetch --progress origin"
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
    scount += 1
    time.sleep(DT_TRML)


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