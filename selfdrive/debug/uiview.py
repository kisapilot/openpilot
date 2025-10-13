#!/usr/bin/env python3
import time
import signal
from cereal import log, messaging
from openpilot.system.manager.process_config import managed_processes

procs = ['camerad']
pm = messaging.PubMaster(['deviceState', 'pandaStates'])

msgs_on = messaging.new_message('deviceState')
msgs_on.deviceState.started = True

msgs_panda_on = messaging.new_message('pandaStates', 1)
msgs_panda_on.pandaStates[0].ignitionLine = True
msgs_panda_on.pandaStates[0].pandaType = log.PandaState.PandaType.tres

msgs_off = messaging.new_message('deviceState')
msgs_off.deviceState.started = False

msgs_panda_off = messaging.new_message('pandaStates', 1)
msgs_panda_off.pandaStates[0].ignitionLine = False
msgs_panda_off.pandaStates[0].pandaType = log.PandaState.PandaType.unknown

def cleanup():
  for p in procs:
    managed_processes[p].stop()
  pm.send('deviceState', msgs_off)
  pm.send('pandaStates', msgs_panda_off)

def handle_sigterm(signum, frame):
  cleanup()
  exit(0)

signal.signal(signal.SIGTERM, handle_sigterm)

try:
  while True:
    for p in procs:
      managed_processes[p].start()
    pm.send('deviceState', msgs_on)
    pm.send('pandaStates', msgs_panda_on)
    time.sleep(1)
except KeyboardInterrupt:
  cleanup()
