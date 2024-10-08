#!/usr/bin/env bash

# prebuilt recreate
if [ -f "/data/kisa_compiling" ]; then
    sudo rm /data/kisa_compiling
    if [ -f "/data/openpilot/prebuilt" ]; then
        sudo rm /data/openpilot/prebuilt
    fi
elif [ -f "/data/kisa_starting" ]; then
    if [ -f "/data/openpilot/prebuilt" ]; then
        sudo rm /data/openpilot/prebuilt
    fi
else
    if [ -f "/data/params/d/PutPrebuiltOn" ]; then
        PREBUILT_CHECK=$(cat /data/params/d/PutPrebuiltOn)
        if [[ "$PREBUILT_CHECK" == "1" && ! -f "/data/openpilot/prebuilt" ]]; then
            touch /data/kisa_starting
            touch /data/openpilot/prebuilt
        fi
    fi
fi

exec ./launch_chffrplus.sh
