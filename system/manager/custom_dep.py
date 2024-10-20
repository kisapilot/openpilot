#!/usr/bin/env python3
import time
import subprocess
import importlib.util
from urllib.request import urlopen


# NOTE: Do NOT import anything here that needs be built (e.g. params)
from openpilot.common.spinner import Spinner

OPSPLINE_SPEC = importlib.util.find_spec('scipy')
OVERPY_SPEC = importlib.util.find_spec('overpy')
MAX_BUILD_PROGRESS = 100

def wait_for_internet_connection(return_on_failure=False):
  retries = 0
  while True:
    try:
      _ = urlopen('https://www.google.com/', timeout=10)
      return True
    except Exception as e:
      print(f'Wait for internet failed: {e}')
      if return_on_failure and retries == 15:
        return False
      retries += 1
      time.sleep(2)  # Wait for 2 seconds before retrying

def install_dep(spinner):
  wait_for_internet_connection()

  TOTAL_PIP_STEPS = 34

  packages = []
  if OPSPLINE_SPEC is None:
    packages.append('scipy')
  if OVERPY_SPEC is None:
    packages.append('overpy')

  pip = subprocess.Popen(['pip', 'install', '-v', '--prefer-binary'] + packages, stdout=subprocess.PIPE)

  # Read progress from pip and update spinner
  steps = 0
  while True:
    output = pip.stdout.readline()
    if pip.poll() is not None:
      break
    if output:
      steps += 1
      spinner.update(f"Installing... {round(min(100, MAX_BUILD_PROGRESS * (steps / TOTAL_PIP_STEPS)))}%")
      print(output.decode('utf8', 'replace'))


if __name__ == "__main__" and not OPSPLINE_SPEC:
  spinner = Spinner()
  spinner.update_progress(0, 100)
  install_dep(spinner)