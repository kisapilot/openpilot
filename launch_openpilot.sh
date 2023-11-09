#!/usr/bin/bash

# prebuilt recreate
if [ -f "/data/kisa_compiling" ]; then
    sudo rm /data/kisa_compiling
    sudo rm /data/openpilot/prebuilt
else
    if [ -f "/data/params/d/PutPrebuiltOn" ]; then
        PREBUILT_CHECK=$(cat /data/params/d/PutPrebuiltOn)
        if [[ "$PREBUILT_CHECK" == "1" && ! -f "/data/openpilot/prebuilt" ]]; then
            touch /data/openpilot/prebuilt
        fi
    fi
fi

export PASSIVE="0"
exec ./launch_chffrplus.sh

