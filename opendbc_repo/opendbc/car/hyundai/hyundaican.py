import crcmod
from opendbc.car.hyundai.values import CAR, HyundaiFlags, LEGACY_SAFETY_MODE_CAR_ALT, LEGACY_SAFETY_MODE_CAR_ALT2

hyundai_checksum = crcmod.mkCrcFun(0x11D, initCrc=0xFD, rev=False, xorOut=0xdf)

def create_lkas11(packer, frame, CP, apply_steer, steer_req,
                  torque_fault, lkas11, sys_warning, sys_state, enabled,
                  left_lane, right_lane,
                  left_lane_depart, right_lane_depart, bus, ldws):
  values = {s: lkas11[s] for s in [
    "CF_Lkas_LdwsActivemode",
    "CF_Lkas_LdwsSysState",
    "CF_Lkas_SysWarning",
    "CF_Lkas_LdwsLHWarning",
    "CF_Lkas_LdwsRHWarning",
    "CF_Lkas_HbaLamp",
    "CF_Lkas_FcwBasReq",
    "CF_Lkas_HbaSysState",
    "CF_Lkas_FcwOpt",
    "CF_Lkas_HbaOpt",
    "CF_Lkas_FcwSysState",
    "CF_Lkas_FcwCollisionWarning",
    "CF_Lkas_FusionState",
    "CF_Lkas_FcwOpt_USM",
    "CF_Lkas_LdwsOpt_USM",
  ]}
  values["CF_Lkas_LdwsSysState"] = sys_state
  values["CF_Lkas_SysWarning"] = 3 if sys_warning else 0
  values["CF_Lkas_LdwsLHWarning"] = left_lane_depart
  values["CF_Lkas_LdwsRHWarning"] = right_lane_depart
  values["CR_Lkas_StrToqReq"] = apply_steer
  values["CF_Lkas_ActToi"] = steer_req and not (torque_fault and (True if CP.carFingerprint in LEGACY_SAFETY_MODE_CAR_ALT2 else False))
  values["CF_Lkas_ToiFlt"] = torque_fault  # seems to allow actuation on CR_Lkas_StrToqReq
  values["CF_Lkas_MsgCount"] = frame % 0x10

  if CP.carFingerprint == CAR.HYUNDAI_GRANDEUR_HEV_IG:
    nSysWarnVal = 9
    if steer_req:
      nSysWarnVal = 4
    values["CF_Lkas_SysWarning"] = nSysWarnVal if sys_warning else 0

  if CP.carFingerprint in (CAR.HYUNDAI_SONATA, CAR.HYUNDAI_PALISADE, CAR.KIA_NIRO_EV, CAR.KIA_NIRO_HEV_2021, CAR.HYUNDAI_SANTA_FE,
                           CAR.HYUNDAI_IONIQ_EV_2020, CAR.HYUNDAI_IONIQ_PHEV, CAR.KIA_SELTOS, CAR.HYUNDAI_ELANTRA_2021, CAR.GENESIS_G70_2020,
                           CAR.HYUNDAI_ELANTRA_HEV_2021, CAR.HYUNDAI_SONATA_HYBRID, CAR.HYUNDAI_KONA_EV, CAR.HYUNDAI_KONA_HEV, CAR.HYUNDAI_KONA_EV_2022,
                           CAR.HYUNDAI_SANTA_FE_2022, CAR.KIA_K5_2021, CAR.HYUNDAI_IONIQ_HEV_2022, CAR.HYUNDAI_SANTA_FE_HEV_2022,
                           CAR.HYUNDAI_SANTA_FE_PHEV_2022, CAR.KIA_STINGER_2022, CAR.KIA_K5_HEV_2020, CAR.KIA_CEED,
                           CAR.HYUNDAI_AZERA_6TH_GEN, CAR.HYUNDAI_AZERA_HEV_6TH_GEN, CAR.HYUNDAI_CUSTIN_1ST_GEN, CAR.KIA_NIRO_PHEV_2022):
    values["CF_Lkas_LdwsActivemode"] = int(left_lane) + (int(right_lane) << 1)
    values["CF_Lkas_LdwsOpt_USM"] = 2

    # FcwOpt_USM 5 = Orange blinking car + lanes
    # FcwOpt_USM 4 = Orange car + lanes
    # FcwOpt_USM 3 = Green blinking car + lanes
    # FcwOpt_USM 2 = Green car + lanes
    # FcwOpt_USM 1 = White car + lanes
    # FcwOpt_USM 0 = No car + lanes
    values["CF_Lkas_FcwOpt_USM"] = 2 if enabled else 1

    # SysWarning 4 = keep hands on wheel
    # SysWarning 5 = keep hands on wheel (red)
    # SysWarning 6 = keep hands on wheel (red) + beep
    # Note: the warning is hidden while the blinkers are on
    values["CF_Lkas_SysWarning"] = 4 if sys_warning else 0

  # Likely cars lacking the ability to show individual lane lines in the dash
  elif CP.carFingerprint in (CAR.KIA_OPTIMA_G4, CAR.KIA_OPTIMA_G4_FL, CAR.HYUNDAI_TUCSON):
    # SysWarning 4 = keep hands on wheel + beep
    values["CF_Lkas_SysWarning"] = 4 if sys_warning else 0

    # SysState 0 = no icons
    # SysState 1-2 = white car + lanes
    # SysState 3 = green car + lanes, green steering wheel
    # SysState 4 = green car + lanes
    values["CF_Lkas_LdwsSysState"] = 3 if enabled else 1
    values["CF_Lkas_LdwsOpt_USM"] = 2  # non-2 changes above SysState definition

    # these have no effect
    values["CF_Lkas_LdwsActivemode"] = 0
    values["CF_Lkas_FcwOpt_USM"] = 0

  elif CP.carFingerprint in (CAR.HYUNDAI_GENESIS, CAR.GENESIS_DH):
    # This field is actually LdwsActivemode
    # Genesis and Optima fault when forwarding while engaged
    values["CF_Lkas_LdwsActivemode"] = 2

  if ldws:
  	values["CF_Lkas_LdwsOpt_USM"] = 3

  dat = packer.make_can_msg("LKAS11", 0, values)[1]

  if CP.flags & HyundaiFlags.CHECKSUM_CRC8:
    # CRC Checksum as seen on 2019 Hyundai Santa Fe
    dat = dat[:6] + dat[7:8]
    checksum = hyundai_checksum(dat)
  elif CP.flags & HyundaiFlags.CHECKSUM_6B:
    # Checksum of first 6 Bytes, as seen on 2018 Kia Sorento
    checksum = sum(dat[:6]) % 256
  else:
    # Checksum of first 6 Bytes and last Byte as seen on 2018 Kia Stinger
    checksum = (sum(dat[:6]) + dat[7]) % 256

  values["CF_Lkas_Chksum"] = checksum

  return packer.make_can_msg("LKAS11", bus, values)


def create_clu11(packer, frame, clu11, button, speed = None, bus = 0):
  values = {s: clu11[s] for s in [
    "CF_Clu_CruiseSwState",
    "CF_Clu_CruiseSwMain",
    "CF_Clu_SldMainSW",
    "CF_Clu_ParityBit1",
    "CF_Clu_VanzDecimal",
    "CF_Clu_Vanz",
    "CF_Clu_SPEED_UNIT",
    "CF_Clu_DetentOut",
    "CF_Clu_RheostatLevel",
    "CF_Clu_CluInfo",
    "CF_Clu_AmpInfo",
    "CF_Clu_AliveCnt1",
  ]}

  if speed != None:
    values["CF_Clu_Vanz"] = speed
  values["CF_Clu_CruiseSwState"] = button
  #values["CF_Clu_AliveCnt1"] = frame % 0x10
  values["CF_Clu_AliveCnt1"] = (values["CF_Clu_AliveCnt1"] + 1) % 0x10
  # send buttons to camera on camera-scc based cars
  #bus = 2 if CP.flags & HyundaiFlags.CAMERA_SCC else 0
  return packer.make_can_msg("CLU11", bus, values)


def create_lfahda_mfc(packer, enabled, hda_set_speed=0):
  values = {
    "LFA_Icon_State": 2 if enabled else 0,
    "HDA_Active": 1 if hda_set_speed else 0,
    "HDA_Icon_State": 2 if hda_set_speed else 0,
    "HDA_VSetReq": hda_set_speed,
  }
  return packer.make_can_msg("LFAHDA_MFC", 0, values)

def create_acc_commands(packer, enabled, accel, upper_jerk, idx, hud_control, set_speed, stopping, long_override, use_fca, gap_setting):
  commands = []

  scc11_values = {
    "MainMode_ACC": 1,
    "TauGapSet": gap_setting,
    "VSetDis": set_speed if enabled else 0,
    "AliveCounterACC": idx % 0x10,
    "ObjValid": 1, # close lead makes controls tighter
    "ACC_ObjStatus": 1, # close lead makes controls tighter
    "ACC_ObjLatPos": 0,
    "ACC_ObjRelSpd": 0,
    "ACC_ObjDist": 1, # close lead makes controls tighter
    "SCCInfoDisplay": 4 if stopping else 0,
    }
  commands.append(packer.make_can_msg("SCC11", 0, scc11_values))

  scc12_values = {
    "ACCMode": 2 if enabled and long_override else 1 if enabled else 0,
    "StopReq": 1 if stopping else 0,
    "aReqRaw": accel,
    "aReqValue": accel,  # stock ramps up and down respecting jerk limit until it reaches aReqRaw
    "CR_VSM_Alive": idx % 0xF,
  }

  # show AEB disabled indicator on dash with SCC12 if not sending FCA messages.
  # these signals also prevent a TCS fault on non-FCA cars with alpha longitudinal
  if not use_fca:
    scc12_values["CF_VSM_ConfMode"] = 1
    scc12_values["AEB_Status"] = 1  # AEB disabled

  scc12_dat = packer.make_can_msg("SCC12", 0, scc12_values)[1]
  scc12_values["CR_VSM_ChkSum"] = 0x10 - sum(sum(divmod(i, 16)) for i in scc12_dat) % 0x10

  commands.append(packer.make_can_msg("SCC12", 0, scc12_values))

  scc14_values = {
    "ComfortBandUpper": 0.0, # stock usually is 0 but sometimes uses higher values
    "ComfortBandLower": 0.0, # stock usually is 0 but sometimes uses higher values
    "JerkUpperLimit": upper_jerk, # stock usually is 1.0 but sometimes uses higher values
    "JerkLowerLimit": 5.0, # stock usually is 0.5 but sometimes uses higher values
    "ACCMode": 2 if enabled and long_override else 1 if enabled else 4, # stock will always be 4 instead of 0 after first disengage
    "ObjGap": 2 if hud_control.leadVisible else 0, # 5: >30, m, 4: 25-30 m, 3: 20-25 m, 2: < 20 m, 0: no lead
  }
  commands.append(packer.make_can_msg("SCC14", 0, scc14_values))

  # Only send FCA11 on cars where it exists on the bus
  if use_fca:
    # note that some vehicles most likely have an alternate checksum/counter definition
    # https://github.com/commaai/opendbc/commit/9ddcdb22c4929baf310295e832668e6e7fcfa602
    fca11_values = {
      "CR_FCA_Alive": idx % 0xF,
      "PAINT1_Status": 1,
      "FCA_DrvSetStatus": 1,
      "FCA_Status": 1,  # AEB disabled
    }
    fca11_dat = packer.make_can_msg("FCA11", 0, fca11_values)[1]
    fca11_values["CR_FCA_ChkSum"] = hyundai_checksum(fca11_dat[:7])
    commands.append(packer.make_can_msg("FCA11", 0, fca11_values))

  return commands

def create_acc_opt(packer):
  commands = []

  scc13_values = {
    "SCCDrvModeRValue": 2,
    "SCC_Equip": 1,
    "Lead_Veh_Dep_Alert_USM": 2,
  }
  commands.append(packer.make_can_msg("SCC13", 0, scc13_values))

  # TODO: this needs to be detected and conditionally sent on unsupported long cars
  fca12_values = {
    "FCA_DrvSetState": 2,
    "FCA_USM": 1, # AEB disabled
  }
  commands.append(packer.make_can_msg("FCA12", 0, fca12_values))

  return commands

def create_frt_radar_opt(packer):
  frt_radar11_values = {
    "CF_FCA_Equip_Front_Radar": 1,
  }
  return packer.make_can_msg("FRT_RADAR11", 0, frt_radar11_values)
  
def create_scc11(packer, frame, set_speed, lead_visible, scc_live, lead_dist, lead_vrel, lead_yrel, car_fingerprint, speed, standstill, gap_setting, stopping, radar_recognition, scc11):
  values = scc11
  values["AliveCounterACC"] = frame // 2 % 0x10
  if not radar_recognition:
    if stopping:
      values["SCCInfoDisplay"] = 4
    else:
      values["SCCInfoDisplay"] = 0
  if not scc_live:
    if standstill:
      values["SCCInfoDisplay"] = 4
    else:
      values["SCCInfoDisplay"] = 0
    values["DriverAlertDisplay"] = 0
    values["MainMode_ACC"] = 1
    values["VSetDis"] = set_speed
    values["TauGapSet"] = gap_setting
    values["ObjValid"] = lead_visible
    values["ACC_ObjStatus"] = lead_visible
    values["ACC_ObjRelSpd"] = clip(lead_vrel if lead_visible else 0, -20., 20.)
    values["ACC_ObjDist"] = clip(lead_dist if lead_visible else 204.6, 0., 204.6)
    values["ACC_ObjLatPos"] = clip(-lead_yrel if lead_visible else 0, -170., 170.)

  return packer.make_can_msg("SCC11", 0, values)

def create_scc12(packer, apply_accel, enabled, scc_live, gaspressed, brakepressed, aebcmdact, car_fingerprint, speed, stopping, standstill, radar_recognition, cnt, scc12):
  values = scc12
  if not aebcmdact:
    if enabled and car_fingerprint == CAR.KIA_NIRO_EV:
      values["ACCMode"] = 2 if gaspressed and (apply_accel > -0.2) else 1
      values["aReqRaw"] = apply_accel
      values["aReqValue"] = apply_accel
      if not radar_recognition and standstill and stopping:
        if stopping:
          values["StopReq"] = 1
        else:
          values["StopReq"] = 0
    elif enabled and not brakepressed:
      values["ACCMode"] = 2 if gaspressed and (apply_accel > -0.2) else 1
      values["aReqRaw"] = apply_accel
      values["aReqValue"] = apply_accel
      if not radar_recognition and standstill and stopping:
        values["aReqRaw"] = 0
        values["aReqValue"] = 0
        if stopping:
          values["StopReq"] = 1
        else:
          values["StopReq"] = 0
    else:
      values["ACCMode"] = 0
      values["aReqRaw"] = 0
      values["aReqValue"] = 0
  if not scc_live:
    if apply_accel < 0.0 and standstill:
      values["StopReq"] = 1
    else:
      values["StopReq"] = 0
    values["ACCMode"] = 1 if enabled else 0 # 2 if gas padel pressed
  values["CR_VSM_Alive"] = cnt
  values["CR_VSM_ChkSum"] = 0
  dat = packer.make_can_msg("SCC12", 0, values)[1]
  values["CR_VSM_ChkSum"] = 16 - sum([sum(divmod(i, 16)) for i in dat]) % 16

  return packer.make_can_msg("SCC12", 0, values)

def create_scc13(packer, scc13):
  values = scc13
  return packer.make_can_msg("SCC13", 0, values)

def create_scc14(packer, enabled, scc14, aebcmdact, lead_visible, lead_dist, v_ego, standstill, car_fingerprint):
  values = scc14
  if enabled and not aebcmdact and car_fingerprint == CAR.KIA_NIRO_EV:
    if standstill:
      values["JerkUpperLimit"] = 0.5
      values["JerkLowerLimit"] = 10.
      values["ComfortBandUpper"] = 0.
      values["ComfortBandLower"] = 0.
      if v_ego > 0.27:
        values["ComfortBandUpper"] = 2.
        values["ComfortBandLower"] = 0.
    else:
      values["JerkUpperLimit"] = 50.
      values["JerkLowerLimit"] = 50.
      values["ComfortBandUpper"] = 50.
      values["ComfortBandLower"] = 50.
  elif enabled and not aebcmdact:
    values["JerkUpperLimit"] = 12.7
    values["JerkLowerLimit"] = 12.7
    values["ComfortBandUpper"] = 0
    values["ComfortBandLower"] = 0
    values["ACCMode"] = 1 # stock will always be 4 instead of 0 after first disengage
    values["ObjGap"] = int(min(lead_dist+2, 10)/2) if lead_visible else 0 # 1-5 based on distance to lead vehicle
  else:
    values["JerkUpperLimit"] = 0
    values["JerkLowerLimit"] = 0
    values["ComfortBandUpper"] = 0
    values["ComfortBandLower"] = 0
    values["ACCMode"] = 4 # stock will always be 4 instead of 0 after first disengage
    values["ObjGap"] = 0

  return packer.make_can_msg("SCC14", 0, values)

def create_scc42a(packer):
  values = {
    "CF_FCA_Equip_Front_Radar": 1
  }
  return packer.make_can_msg("FRT_RADAR11", 0, values)