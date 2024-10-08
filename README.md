<h1 style="text-align:center">Welcome to KisaPilot</h1>

![](https://raw.githubusercontent.com/kisapilot/model/main/KisaPilot.png)


Table of Contents
==================

* [What is openpilot?](#what-is-openpilot-🤖🚘) 🤖🚘
* [What is KisaPilot](#about-this-fork-🔎🍴) 🔎🍴
* [Join Our Discord](#join-our-discord-🎟️💬) 🎟️💬
* [Main Features](#main-features-🌟🪄) 🌟🪄
* [Branch Definitions](#branch-definitions-🌳📄) 🌳📄
* [How to Install](#how-to-install-❓💾) ❓💾
* [Settings Menu](#setting-menu-⚙️📌) ⚙️📌
* [Special Thanks](#special-thanks-🎖️👏) 🎖️👏
* [Donate](#donate-🤝💵) 🤝💵
* [Licensing](#licensing-🖋️📑) 🖋️📑

What Is openpilot 🤖🚘
------

[openpilot](http://github.com/commaai/openpilot) is an open source driver assistance system. Currently, openpilot performs the functions of Adaptive Cruise Control (ACC), Automated Lane Centering (ALC), Forward Collision Warning (FCW) and Lane Departure Warning (LDW) for a growing variety of [supported car makes, models and model years](docs/CARS.md). In addition, while openpilot is engaged, a camera based Driver Monitoring (DM) feature alerts distracted and asleep drivers. See more about [the vehicle integration](docs/INTEGRATION.md) and [limitations](docs/LIMITATIONS.md).

<table>
  <tr>
    <td><a href="https://youtu.be/NmBfgOanCyk" title="Video By Greer Viau"><img src="https://i.imgur.com/1w8c6d2.jpg"></a></td>
    <td><a href="https://youtu.be/VHKyqZ7t8Gw" title="Video By Logan LeGrand"><img src="https://i.imgur.com/LnBucik.jpg"></a></td>
    <td><a href="https://youtu.be/VxiR4iyBruo" title="Video By Charlie Kim"><img src="https://i.imgur.com/4Qoy48c.jpg"></a></td>
    <td><a href="https://youtu.be/-IkImTe1NYE" title="Video By Aragon"><img src="https://i.imgur.com/04VNzPf.jpg"></a></td>
  </tr>
  <tr>
    <td><a href="https://youtu.be/iIUICQkdwFQ" title="Video By Logan LeGrand"><img src="https://i.imgur.com/b1LHQTy.jpg"></a></td>
    <td><a href="https://youtu.be/XOsa0FsVIsg" title="Video By PinoyDrives"><img src="https://i.imgur.com/6FG0Bd8.jpg"></a></td>
    <td><a href="https://youtu.be/bCwcJ98R_Xw" title="Video By JS"><img src="https://i.imgur.com/zO18CbW.jpg"></a></td>
    <td><a href="https://youtu.be/BQ0tF3MTyyc" title="Video By Tsai-Fi"><img src="https://i.imgur.com/eZzelq3.jpg"></a></td>
  </tr>
</table><br />

What is KisaPilot 🔎🍴
================

This fork is specifically focused on Hyundai, Kia, and Genesis Vehicles, and is the ideal choice for enthusiasts seeking extreme customization and comprehensive control over the openpilot environment.


Main Features 🌟🪄
==============

 - 🖥️ Advanced UI with on-screen settings for most openpilot parameters.
 - 🎚️ Live Tune feature accessible via the settings/UI.
 - 👆 Ability to change cruise mode by pushing the GAP Button at Cruise Standby status.(OpenpilotStock, Dist+Curv, Dist only, Curv only, OneWay mode, Speedlimit decelation mode only)
 - 🗺️ MapBox support (special thanks to Dragonpilot).
 - 📡 Display IP Address, SSID name, cell carrier, and signal strength.
 - ↕️ Cruise Button Spamming feature to adjust car set speed without longitudinal control, using OP target speed to maintain a certain distance to the lead car. Includes variable SCC speed control for smooth deceleration (better than stock) when approaching the lead car, and smooth acceleration.
 - 🚗 SmartMDPS support for steering down to 0 mph (certain vehicles that are normally unsupported to 0 mph)
 - 🚌 Auto Recognition of SCC bus on CAN 2 for long control.
 - 🎨 Variable color speed display (braking is represented by shades of red, acceleration by shades of green, and coasting (no brake, no gas) by white).
 - 🏞️ OSM integration for automatic SCC speed adjustment via button spamming feature and slow down on curve using custom UI settings.
 - 👤 2 Users presets.
 - 🛣️ Automatic lane mode selection (laneless or lanefull).
 - 🔀 Multiple lateral control options.
 - 🤝 User-Friendly Control Mode (UFC) with:
   - ↔️ Full-time lateral control.
   - 🚦 Auto Resume while driving.
   - 🔜 Separate Lat/StockLong (coming soon).
 - 🔓 No SSH knowledge required, as most parameters can be adjusted via the UI.
 - ⏫ Always updated and current.

Branch Definitions 🌳📄
====================

  - `KisaPilot:` Stable production branch for Comma 3&3X device, not latest features. This will normally be updated once testing branch features are complete.
  - `KisaPilot_test:` Test branch for Comma 3&3X device, not stable, latest developments, for testing new functions, codes, or the other things. <br />
  ** old branches are in openpilot_bak repository.

How To Install ❓💾
===============

- 📥 `Use the KisaPilot fork installer:`
  - Stable production branchs 
    - Comma 3: https://smiskol.com/fork/kisapilot/KisaPilot

  - Test branchs
    - Comma 3: https://smiskol.com/fork/kisapilot/KisaPilot_test

- ⌨️ `SSH:`
  - cd /data && mv openpilot openpilot_bak && git clone https://github.com/kisapilot/openpilot.git -b KisaPilot && reboot <br />

Setting Menu ⚙️📌
=============

 - `Device` (**Function Name:** Description)
   - **Driving Camera:** Ability to preview the openpilot Driving Camera.

 - `Network` (**Function Name:** Description)
   - **HotSpot on Boot:** Automatically turns on the hotspot during boot.  (reboot required)
   - **Use Legacy SSH Key:** Uses the old ssh key access. (below 0.8.2) (no reboot required)

 - `Toggles` (**Function Name:** Description)
   - **Enable Lane selector Mode:** Displays a lane mode button on the on-road driving screen, with Laneline/LaneLess/AUTO options. AUTO mode switches based on the presence of lane markers. (no reboot required)
   - **Enable Driver Monitoring:** Toggles the driver monitoring feature. For devices without IR camera, a filterless IR camera, or for anyone unable to use the front cam for other reasons. (reboot required)
   - **Enable Driving Log Record:** Records driving logs locally instead of on an online server. (reboot required)
   - **Enable Sending Log to Server:** Enables log uploading to online server. (reboot required)
   - **Use Comma Stock UI:** Enables the use of the original Comma UI. This can be applied on the on-road driving screen in real-time by clicking the MaxSpeed box at the top-left corner. (no reboot required)

 - `Software` (**Function Name:** Description)
   - **Check for Updates:** Check for new commits of current fork. Pressing OK will initiate an update and reboot, similar to the 'git pull' command.
   - **Commit(Local/Remote):** Commit name of local(device) and Remote.(run once when boot in manager.py, search gitcommit.sh at the file, internet connection required)
   - **Git Pull On Boot:** Executes 'git pull' command during boot.
   - **Load Preset/Save Preset:** Allows you to load or save your KisaPilot settings into two presets. The settings are stored in the files /data/preset1 and /data/preset2.
   - **Parameter/Settings Reset :** Remove all user settings and restore default values.
   - **Git Reset:** Discards local changes and reverts to the original status of the branch.
   - **Cancel Git Pull:** Reverts to the previous version of the fork if the last update is not desired.
   - **Panda Flashing:** Run Panda flashing command manually. (This is not necessary on normal operation.)
   - **Change Repo/Branch:** Ability to install others forks and/or branches. Three prompts: Github Username, Github Repository, Github Branch.

 - `UI Menu` (**Function Name:** Description)
   - **Device Auto-Shutdown Time:** The device will be shutdown after the set time once car ignition is turned off.
   - **Device Force-Shutdown Time:** The device will be shutdown by force at offroad status after the set time.
   - **Deivce Volume Control(%):** Manually set device volume.
   - **Device Brightness Control(%):** Manually set the device brightness to a specific percentage, or set it to adjust automatically.
   - **Device Screen Off Timer:** The **Brightness at Screen Off(%)** setting will be in effect after this set time when driving.
   - **Brightness at Screen Off(%):** Work with **Device Screen Off Timer setting** to set screen brightness level for timer.
   - **Device Detach Reminder:** Will play alert sound when your car ignition is turned off. (Can be used as a reminder to remove device from mount to protect from sun, theft, etc.)
      - None
      - Korean
      - English
   - **Enable Battery Charging Control:** Ability to configure min and max battery settings (not applicable for units without a battery).
   - **Auto Screen Record:** Automatically begin screen recording upon departure and stops recording once the vehicle has been put in park.
   - **Number of Recorded Files:** Set the maximum number of mp4 files to record before overwriting. (to prevent device storage filling up).
   - **Recording Quality:** Adjust video quality of recordings. 
      - Low
      - Mid
      - High
      - U-High
   - **Delete All Recorded Files:** Erase all recorded files from `/sdcard/videos` on the device.
   - **Delete All Driving Logs:** Erase all driving logs from `/sdcard/realdata` on the device.
   - **Driver Monitoring Mode:** Select Default or Unsleep. (reboot not required).
      - Default: Uses the standard Comma's DM mode.
      - Unsleep: A **MORE** sensitive DM mode than default. <br />
        (Switch modes in real-time by tapping the DM face at the bottom-left corner of the on-road driving screen).  <br />
        - No background is Default Mode. <br />
        - Light green background is Unsleep Mode. 
   - **E2E EYE Threshold:** Experimental setting.
   - **Normal EYE Threshold:** Set the value below threshold of your face recognition.
   - **Blink Threshold:** For Driver Monitoring. Set the value below the threshold of your eyes blink recognition. The Driver Monitoring camera shows the values of your face recognition, eyes, and other metrics. (Preview 'Driver Camera' and check the recognition value of your eye blink to modify the value on menu.)
   - **Navigation Select:** Choose navigation software:
      - iNavi (Korea)
      - Mappy (Korea)
      - Waze (Global)
   - **RUN Navigation on Boot:** Launch your selected navigation software on boot. (If loads correctly, it will automatically set to background task after a few seconds.)
   - **Display Date on Screen:** Enable the device to display the current date.
   - **Display Time on Screen:** Enable the device to display the current time.
   - **API Server:** Choose the server to upload your driver logs:
      - Kisa
      - Comma
      - User Defined
   - **User's API Address:** Enter the URL for your driver log server.
   - **Mapbox Style:** Choose a map style:
      - Mapbox
      - Comma
      - Kisa (locallized in Korea)
      - Customize your own map style
        - Create a Mapbox style at https://studio.mapbox.com/ and publish it to use.
        - Edit `/data/params/d/MapboxStyleCustom` with new map style.
   - **Top Text View:** Display Date/Time/Roadname at the top of the driving screen.
   - **RPM Animation:** Display RPMs on UI with a customizable limit.
   - **Show Stop Line:** Enable visualization of stop lines on screen.
   - **Enable RTShield Process:** Good for C2 openpilot, gives op higher cpu priority by ensuring CPU 3 always remains available for RT processes (runs as SCHED_FIFO with minimum priority to ensure kthreads don't get scheduled onto CPU 3, but it's always preemptible by realtime openpilot processes.) 
   - **Offline OSM:** Enable offline use of OpenStreetMap. (Korea only) (64G storage only) 

 - `Driving Menu` (**Function Name:** Description)
   - **Use Auto Resume at Stop:** After standstill, op will auto-resume when lead car start moving.
   - **RES count at standstill:** Some model need RES count to be adjusted so car moves when lead car starts moving. (reboot required)
   - **Accelerated Departure by Cruise Gap:** Cruise gap automatically changed to step 1 for faster departure, sets back to orignal gap selection after few second.
   - **Alternative Standstill Resume:** Some model need this for Auto Resume at Stop.
   - **Use Cruise Button Spamming:** For use in conjunction with stock SCC. The set speed is automatically changed up and down to facilitate many functions related to Kisa's auto speed control features.
   - **Cruise Start Mode:** Set your custom Cruise Mode when boot. There are 6 modes. (OpenpilotStock, Dist+Curv, Dist, Curv, Oneway, CamSpeed) 
      - OpenpilotStock: SCC button will set SCC speed, and then will work like stock button to set op. 
      - Dist+Curv: Changed set speed by distance to lead car and curvature. 
      - Dist: Changed set speed by Distance only. 
      - Curv: Changed set speed by Curvature only. 
      - Oneway: Changed set speed by camera offset to approach the edge of a road. 
      - CamSpeed: Changing set speed only by value of speed sign (OSM, iNavi, Mappy).
   - **LaneChange Speed:** Minimum speed op will assist making lane changes.
   - **LaneChange Delay:** Adjust time before lane change initiates.
      - Nudge
      - Nudgeless
      - Set seconds
   - **LaneChange Time(km/h: value):** Adjust amount of time for lane change to be completed. For faster lane change increase the value, for slower decrease.
   - **LeftCurv Offset:** If you are not satisfy with Left Curve Section, this can move your car to left or right side.(no reboot required)
   - **RightCurv Offset:** if you are not satisfy with Right Curve Section, this can move your car to left or right side.(no reboot required)
   - **Show BSM Status:** Shows when a car is in blindspot on UI. (Requires BSM feature on vehicle)
   - **Steer Control Method** Choose between normal and smooth.
   - **Max Steering Angle:** Default is 90. If you want more, increase this. Some car will not accept value above 90.
   - **Str Angle Adjust:** To keep car on a straight road, If the value of steering angle is not 0.0, adjust this to be 0.0
   - **Stop Steer Assist on Turn Signals:** Openpilot doesn't steer your car when turn signal is active.
   - **Reset MaxSpeed Over CurrentSpeed:** Sync SCC speed with car current speed, the OP MaxSpeed synchronize to your car speed.
   - **Enable OSM:** use OSM feature
   - **Enable OSM SpeedLimit:** Use OSM SpeedLimit, active internet required. (reboot required).
   - **Use Stock SafetyCAM Speed:** Some cars have the signal in CAN message. not for all HKG cars.
   - **SpeedLimit Offset (%, +- or C):** Use to set SCC speed above or below the OSM or Stock CAN reported speed. This can be % Speed amount + / -  
   - **OSMCustomSpeedlimit([SL],[Target Speed]):** (Set SpeedLimit Offset to C) set custom OSM speed offset SL & Target Speeds.
   - **SafetyCam SignType:** You can select 2 options to show on the screen
      - Circular: (EU) type of speedlimit sign.
      - Retangular: (US) type of speedlimit sign.
   - **SafetyCamDist Adj(%):** Change the target distance if you are in the decel situation of speed limit or safetycam.
   - **Curve Decel Option :** Which curve decel you want to use,  Vision / OSM.
   - **VisionCurvDecel([CV],[Target Speed]):** set speed is changed by Curve Vector and Target Speed.
   - **OSMCurvDecel([TSL],[Target Speed]):** If OSM has the value of curv, set your target speed.
   - **Use Auto Engagement:** When OP is in disengagement status, Auto engagement is enabled when your car is moving. Cruise Standby status is needed at least.
   - **Auto Engage Speed(km/h):** Auto Engagement is enabled at this speed.
   - **Use Auto RES while Driving:** SCC speed automatically resume when brake is release or gas is applied.(reboot required)
   - **AutoRES Option:** CruiseSet/MaxSpeedSet, MaxSpeedSet: Your OP MAX Speed set with RES Set speed. CruiseSet:only set to current set speed, not changed OP MAX Speed.
   - **AutoRES Condition:** RelBrake/OnGas, RelBrake: SCC speed set again when you release from brake pedal. OnGas: SCC speed set again when you step gas pedal.
   - **AutoRES Allow(sec):** If AutoRES does not occur before set time, then auto resume is cancelled.
   - **AutoRESDelay (sec):** AutoRes will not reume until set time elapse, to prevent premature resume.
   - **Set LaneWidth:** Adjust if road lane is narrow
   - **Speed LaneWidth:** [Spd(m/s)], [Lanewidth] Adjust speed based on lane width.
   - **Routine Drive by Roadname:** (WIP) will change drive characteristics based on road, eg if local or highway will handle curve differently.
   - **Driving Close to RoadEdge**
   - **Avoid LKAS Fault**
   - **Speed CameraOffset**

 - `Developer Menu` (**Function Name:** Description)
   - **DEBUG UI 1:** Show debug UI on screen. 2 lines bottom of screen.(no reboot required)
   - **DEBUG UI 2:** Show debug UI on screen. other lines except 2 lines bottom.(no reboot required)
   - **DEBUG UI 3:** Show debug UI on screen. more debug info.(no reboot required)
   - **Show TMUX Error:** Turn this on, if you want to show tmux error on screen related to process such as controlsd, plannerd and so on.(reboot required)
   - **Show LongControl LOG:** show long control log at DEBUG UI 1.(no reboot required)
   - **Use Smart Prebuilt:** Your device can be booted quickly. The file, Prebuilt is removed when you do push 'CHECK' button on the Menu or type 'gi' command on command line, after then it will be created again when boot&compile is completed.(reboot required)
   - **Use FingerPrint 2.0:** (reboot required)
   - **Support WhitePanda:** Turn this on if you use WhitePanda. this is related to issue stopping processes when you turn device off.(reboot required)
   - **Set BatteryLess Eon:** Screen doesn't show information of battery status.
   - **Turn Off Communication Issue Alarm:** Turn this on if you do not want alert of communication error. Sometimes you could get an commuication issue alert while using navi app or your custom UI is busy or rarely due to locationd(regarding GPS) process. I could use OP with the alert before without any issues. This is quite old issue. Currently I'm not sure if the error is still exist.
   - **Set LDWS Vehicles**
   - **Set DriveGear by Force:** for cars don't have dbc info of gear(reboot required)
   - **Ignore of Steering Warning:** Some cars have Steerwarning, so that not engaged.
   - **Ignore Can Error on ISG:** for ISG cars. In order to ignore can error, if you want to prevent disengagement.
   - **Enable FCA11 Message:** For some newer vechicle
   - **Steer Wind Down :**
   - **MainSwitch Openpilot On/Off:**
   - **StockLKAS Enable at disengagement:**
   - **C2 with CommaPower:**
   - **Use GoogleMap for Mapbox:** Use google map when you search your destination at a web browser.
   - **Timezone setting:** (reboot required)
   - **Enable Calibration by Force:** developer for engagment test
   - **Open Android Settings**
   - **SoftKey RUN/SET:** softkey application
   - **RUN Mixplorer:** file manager application
   - **CAR Force Recognition:** If your car is not recognized, choose your car at this. (reboot required)

 - `Tuning Menu` (**Function Name:** Description)
   - **CameraOffset:** set your camera offset
   - **PathOffset:** i'm not sure this. but i recommend if you move your camera offset, adjust this as the value.
   - **Use Live SteerRatio:** Use Live Parameter value.
   - **LiveSR Adjsut(%):** in some cases, live parameter is higher than original steeratio, i set this to minus value to not steer aggressively.
   - **SteerRatio:** Your default SteerRatio
   - **SteerRatioMax:** Max SteerRatio if you use Varaible SteerRatio not Live SR.
   - **SteerActuatorDelay:** level how your car reacts to upcoming road curvature.
   - **SteerRateCost:** How your car make steer strong to turn with the road curvature. you want make it strong lower the value. too much low values, it will make the steering little unstable.
   - **SteerLimitTimer:** timer how long op hold the steer. and timer for alert.
   - **TireStiffnessFactor:** lower value makes your steer more aggressive.
   - **SteerMaxDefault:** SteerMax Default value
   - **SteerMaxMax:** SteerMax Max value if you use variable SteerMax.
   - **SteerMaxV:** multiply to the output scale. it effects steer saturation or others.
   - **Use variable SteerMax:** use variable steermax by road curvature. it works above 30km/h.
   - **SteerDeltaUpDefault:** how fast steer inside in certain timing scope.
   - **SteerDeltaUpMax:** max value if you use variable steerdelta
   - **SteerDeltaDownDefault:** how fast steer outside in certain timing scope.
   - **SteerDeltaDownMax:** max value if you use variable steerdelta
   - **Use variable SteerDelta:** use variable steerdelta by road curvature. it works above 30km/h.
   - **SteerThreshold:** driver steering torque threshold
   - **LatControl:** (reboot required)
      - PID
      - INDI
      - LQR
      - Torque
   - **Use LiveTune and Show UI:** this will show button on the screen, you can up/down the values of tune. it will be applied in realtime. you can also touch top-right corner(comma wheel icon) to show tune panel on the screen.
   - **Tune Values:** change and find appropriate values.
   - `LONG CONTROL MENU` **(RadarHareness required)**
      - **Use DynamicTR:** TR changed by car speed.
      - **CruiseGap:** set TR of other Cruise Gaps
      - **Use Radar Long Assist:** when your car approaches to lead car, use combined value both OP gas/brake and Radar one.
      - **Adjust Stopping Distance:** help stopping distance more close to lead car(not recommended)
      - **Enable E2E Long:** Use Comma E2E long, sometimes it is not comfortable. think it's earlier to release.

Special Thanks 🎖️👏
================

`Special Thanks:` ku7, xx979xx, tk211x, xps-genesis, atom(respect you), hoya, moksatang, mamul, neokii, oricialworks, dragonpilot, shane, kegman, dnv26, move-fast, D.Fyffe, Swish865 and everyone helping me or contributing for HKGs.

Donate 🤝💵
================

If you have enjoyed any features of this fork, and would like to show your support, feel free to [donate via PayPal](https://paypal.me/multiKYD). Any support is greatly appreciated! 🙏

Thank you!

Licensing 🖋️📑
================

KisaPilot is released under the MIT license. Some parts of the software are released under other licenses as specified.

Any user of this software shall indemnify and hold harmless Comma.ai, Inc. and its directors, officers, employees, agents, stockholders, affiliates, subcontractors and customers from and against all allegations, claims, actions, suits, demands, damages, liabilities, obligations, losses, settlements, judgments, costs and expenses (including without limitation attorneys’ fees and costs) which arise out of, relate to or result from any use of this software by user.

**THIS IS ALPHA QUALITY SOFTWARE FOR RESEARCH PURPOSES ONLY. THIS IS NOT A PRODUCT.
YOU ARE RESPONSIBLE FOR COMPLYING WITH LOCAL LAWS AND REGULATIONS.
NO WARRANTY EXPRESSED OR IMPLIED.**

---

<img src="https://cdn-images-1.medium.com/max/1600/1*C87EjxGeMPrkTuVRVWVg4w.png" width="225"></img>
