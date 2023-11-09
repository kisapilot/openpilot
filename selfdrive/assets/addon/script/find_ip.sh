#!/usr/bin/bash

cd /data/openpilot
IP_FILE=$(cat /data/params/d/ExternalDeviceIP)

OIFS=$IFS
IFS=',' read -r -a array <<< "$IP_FILE"

for NUM in "${!array[@]}"; do
  ping -c 1 -W 1 ${array[NUM]}
  if [ $(echo $?) == "0" ]; then
    nc -vz ${array[NUM]} 5555
    if [ $(echo $?) == "0" ]; then
      echo -n ${array[NUM]} > /data/params/d/ExternalDeviceIPNow
    fi
  fi
done;
