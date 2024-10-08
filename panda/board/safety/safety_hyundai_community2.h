#pragma once

#include "safety_hyundai_common.h"

int OP_LKAS_live = 0;
int OP_MDPS_live = 0;
int OP_CLU_live = 0;
int OP_SCC_live = 0;
int car_SCC_live = 0;
int OP_EMS_live = 0;
int HKG_mdps_bus = -1;
int HKG_scc_bus = -1;

bool HKG_LCAN_on_bus1 = false;
bool HKG_forward_bus1 = false;
bool HKG_forward_obd = false;
bool HKG_forward_bus2 = true;
int HKG_LKAS_bus0_cnt = 0;
int HKG_Lcan_bus1_cnt = 0;
uint32_t ts_last2 = 0;

static bool msg_allowed2(const CANPacket_t *to_send, const CanMsg msg_list[], int len) {
  int addr = GET_ADDR(to_send);
  int bus = GET_BUS(to_send);
  int length = GET_LEN(to_send);

  bool allowed = false;
  for (int i = 0; i < len; i++) {
    if ((addr == msg_list[i].addr) && (bus == msg_list[i].bus) && (length == msg_list[i].len)) {
      allowed = true;
      break;
    }
  }
  return allowed;
}

static bool max_limit_check2(int val, const int MAX_VAL, const int MIN_VAL) {
  return (val > MAX_VAL) || (val < MIN_VAL);
}

static bool driver_limit_check2(int val, int val_last, const struct sample_t *val_driver,
                        const int MAX_VAL, const int MAX_RATE_UP, const int MAX_RATE_DOWN,
                        const int MAX_ALLOWANCE, const int DRIVER_FACTOR) {

  // torque delta/rate limits
  int highest_allowed_rl = MAX(val_last, 0) + MAX_RATE_UP;
  int lowest_allowed_rl = MIN(val_last, 0) - MAX_RATE_UP;

  // driver
  int driver_max_limit = MAX_VAL + (MAX_ALLOWANCE + val_driver->max) * DRIVER_FACTOR;
  int driver_min_limit = -MAX_VAL + (-MAX_ALLOWANCE + val_driver->min) * DRIVER_FACTOR;

  // if we've exceeded the applied torque, we must start moving toward 0
  int highest_allowed = MIN(highest_allowed_rl, MAX(val_last - MAX_RATE_DOWN,
                                             MAX(driver_max_limit, 0)));
  int lowest_allowed = MAX(lowest_allowed_rl, MIN(val_last + MAX_RATE_DOWN,
                                           MIN(driver_min_limit, 0)));

  // check for violation
  return max_limit_check2(val, highest_allowed, lowest_allowed);
}

static bool rt_rate_limit_check2(int val, int val_last, const int MAX_RT_DELTA) {

  // *** torque real time rate limit check ***
  int highest_val = MAX(val_last, 0) + MAX_RT_DELTA;
  int lowest_val = MIN(val_last, 0) - MAX_RT_DELTA;

  // check for violation
  return max_limit_check2(val, highest_val, lowest_val);
}

static void relay_malfunction_reset2(void) {
  relay_malfunction = false;
  fault_recovered(FAULT_RELAY_MALFUNCTION);
}

static const CanMsg HYUNDAI_COMMUNITY2_TX_MSGS[] = {
  {0x340, 0, 8}, {0x340, 1, 8}, // LKAS11 Bus 0, 1
  {0x4F1, 0, 4}, {0x4F1, 1, 4}, {0x4F1, 2, 4}, // CLU11 Bus 0, 1, 2
  {0x485, 0, 4}, // LFAHDA_MFC Bus 0
  {0x251, 2, 8},  // MDPS12, Bus 2
  {0x420, 0, 8}, //   SCC11,  Bus 0
  {0x421, 0, 8}, //   SCC12,  Bus 0
  {0x50A, 0, 8}, //   SCC13,  Bus 0
  {0x389, 0, 8},  //   SCC14,  Bus 0
  {0x4A2, 0, 8},  //   4a2SCC, Bus 0
  {0x316, 1, 8}, // EMS11, Bus 1
  {0x483, 0, 8}, //   FCA12,  Bus 0
  {0x38D, 0, 8},  //   FCA11,  Bus 0
  {0x7D0, 0, 8},  // SCC_DIAG, Bus 0
};

static void hyundai_community2_rx_hook(const CANPacket_t *to_push) {

  int addr = GET_ADDR(to_push);
  int bus = GET_BUS(to_push);

  // check if we have a LCAN on Bus1
  if (bus == 1 && (addr == 0x510 || addr == 0x20C)) {
    HKG_Lcan_bus1_cnt = 500;
    if (HKG_forward_bus1 || !HKG_LCAN_on_bus1) {
      HKG_LCAN_on_bus1 = true;
      HKG_forward_bus1 = false;
    }
  }
  // check if LKAS on Bus0
  if (addr == 0x340) {
    if (bus == 0 && HKG_forward_bus2) {HKG_forward_bus2 = false; HKG_LKAS_bus0_cnt = 20;}
    if (bus == 2) {
      if (HKG_LKAS_bus0_cnt > 0) {HKG_LKAS_bus0_cnt--;} else if (!HKG_forward_bus2) {HKG_forward_bus2 = true;}
      if (HKG_Lcan_bus1_cnt > 0) {HKG_Lcan_bus1_cnt--;} else if (HKG_LCAN_on_bus1) {HKG_LCAN_on_bus1 = false;}
    }
  }
  // check MDPS on Bus
  if ((addr == 0x251 || addr == 0x381) && HKG_mdps_bus != bus) {
    if (bus != 1 || (!HKG_LCAN_on_bus1 || HKG_forward_obd)) {
      HKG_mdps_bus = bus;
      if (bus == 1 && !HKG_forward_obd) {if (!HKG_forward_bus1 && !HKG_LCAN_on_bus1) {HKG_forward_bus1 = true;}}
    }
  }
  // check SCC on Bus
  if ((addr == 0x420 || addr == 0x421) && HKG_scc_bus != bus) {
    if (bus != 1 || !HKG_LCAN_on_bus1) {
      HKG_scc_bus = bus;
      if (bus == 1) {if (!HKG_forward_bus1) {HKG_forward_bus1 = true;}}
    }
  }

  if (addr == 0x251 && bus == HKG_mdps_bus) {
    int torque_driver_new = ((GET_BYTES(to_push, 0, 4) & 0x7ffU) * 0.79) - 808; // scale down new driver torque signal to match previous one
    // update array of samples
    update_sample(&torque_driver, torque_driver_new);
  }

  if (addr == 0x420 && !OP_SCC_live) {
    // 1 bits: 0
    bool cruise_available = GET_BIT(to_push, 0U);
    hyundai_common_cruise_state_check_alt(cruise_available);
  }

  // cruise control for car without SCC
  if (addr == 0x4F1 && bus == 0 && HKG_scc_bus == -1 && !OP_SCC_live) {
    int cruise_button = GET_BYTE(to_push, 0) & 0x7U;
    // enable on res+ or set- buttons press
    if (!controls_allowed && (cruise_button == 1 || cruise_button == 2)) {
      hyundai_common_cruise_state_check_alt(true);
    }
    // disable on cancel press
    if (cruise_button == 4) {
      controls_allowed = 0;
    }
  }

  // sample wheel speed, averaging opposite corners
  if (addr == 0x386 && bus == 0) {
    uint32_t front_left_speed = GET_BYTES(to_push, 0, 2) & 0x3FFFU;
    uint32_t rear_right_speed = GET_BYTES(to_push, 6, 2) & 0x3FFFU;
    vehicle_moving = (front_left_speed > HYUNDAI_STANDSTILL_THRSLD) || (rear_right_speed > HYUNDAI_STANDSTILL_THRSLD);
  }

  gas_pressed = brake_pressed = false;

  generic_rx_checks((addr == 0x340 && bus == 0));
}

static bool hyundai_community2_tx_hook(const CANPacket_t *to_send) {

  bool tx = true;
  int addr = GET_ADDR(to_send);
  int bus = GET_BUS(to_send);

  tx = msg_allowed2(to_send, HYUNDAI_COMMUNITY2_TX_MSGS, sizeof(HYUNDAI_COMMUNITY2_TX_MSGS)/sizeof(HYUNDAI_COMMUNITY2_TX_MSGS[0]));

  // LKA STEER: safety check
  if (addr == 0x340) {
    OP_LKAS_live = 20;
    int desired_torque = ((GET_BYTES(to_send, 0, 4) >> 16) & 0x7ffU) - 1024U;
    uint32_t ts = microsecond_timer_get();
    bool violation = false;

    if (controls_allowed) {

      // *** global torque limit check ***
      bool torque_check = 0;
      violation |= torque_check = max_limit_check2(desired_torque, HYUNDAI_MAX_STEER, -HYUNDAI_MAX_STEER);

      // *** torque rate limit check ***
      bool torque_rate_check = 0;
      violation |= torque_rate_check = driver_limit_check2(desired_torque, desired_torque_last, &torque_driver,
        HYUNDAI_MAX_STEER, HYUNDAI_MAX_RATE_UP, HYUNDAI_MAX_RATE_DOWN,
        HYUNDAI_DRIVER_TORQUE_ALLOWANCE, HYUNDAI_DRIVER_TORQUE_FACTOR);

      // used next time
      desired_torque_last = desired_torque;

      // *** torque real time rate limit check ***
      bool torque_rt_check = 0;
      violation |= torque_rt_check = rt_rate_limit_check2(desired_torque, rt_torque_last, HYUNDAI_MAX_RT_DELTA);

      // every RT_INTERVAL set the new limits
      uint32_t ts_elapsed = get_ts_elapsed(ts, ts_last2);
      if (ts_elapsed > HYUNDAI_RT_INTERVAL) {
        rt_torque_last = desired_torque;
        ts_last2 = ts;
      }
    }

    // no torque if controls is not allowed
    if (!controls_allowed && (desired_torque != 0)) {
      violation = true;
    }

    // reset to 0 if either controls is not allowed or there's a violation
    if (!controls_allowed) { // a reset worsen the issue of Panda blocking some valid LKAS messages
      desired_torque_last = 0;
      rt_torque_last = 0;
      ts_last2 = ts;
    }

    if (violation) {
      tx = false;
    }
  }

  // FORCE CANCEL: safety check only relevant when spamming the cancel button.
  // ensuring that only the cancel button press is sent (VAL 4) when controls are off.
  // This avoids unintended engagements while still allowing resume spam
  //allow clu11 to be sent to MDPS if MDPS is not on bus0
  if (addr == 0x4F1 && !controls_allowed && (bus != HKG_mdps_bus && HKG_mdps_bus == 1)) {
    if ((GET_BYTES(to_send, 0, 4) & 0x7U) != 4U) {
      tx = false;
    }
  }

  if (addr == 0x251) {OP_MDPS_live = 20;}
  if (addr == 0x4F1 && bus == 1) {OP_CLU_live = 20;} // only count mesage created for MDPS
  if (addr == 0x421) {OP_SCC_live = 20; if (car_SCC_live > 0) {car_SCC_live -= 1;}}
  if (addr == 0x316) {OP_EMS_live = 20;}

  // true allows the message through
  return tx;
}

static int hyundai_community2_fwd_hook(int bus_num, int addr) {

  int bus_fwd = -1;
  int fwd_to_bus1 = -1;
  if (HKG_forward_bus1 || HKG_forward_obd){fwd_to_bus1 = 1;}

  // forward cam to ccan and viceversa, except lkas cmd
  if (HKG_forward_bus2) {
    if (bus_num == 0) {
      if (!OP_CLU_live || addr != 0x4F1 || HKG_mdps_bus == 0) {
        if (!OP_MDPS_live || addr != 0x251) {
          if (!OP_EMS_live || addr != 0x316) {
            bus_fwd = fwd_to_bus1 == 1 ? 12 : 2;
          } else {
            bus_fwd = 2;  // EON create EMS11 for MDPS
            OP_EMS_live -= 1;
          }
        } else {
          bus_fwd = fwd_to_bus1;  // EON create MDPS for LKAS
          OP_MDPS_live -= 1;
        }
      } else {
        bus_fwd = 2; // EON create CLU12 for MDPS
        OP_CLU_live -= 1;
      }
    }
    if (bus_num == 1 && (HKG_forward_bus1 || HKG_forward_obd)) {
      if (!OP_MDPS_live || addr != 0x251) {
        if (!OP_SCC_live || (addr != 0x420 && addr != 0x421 && addr != 0x50A && addr != 0x389)) {
          bus_fwd = 20;
        } else {
          bus_fwd = 2;  // EON create SCC11 SCC12 SCC13 SCC14 for Car
          OP_SCC_live -= 1;
        }
      } else {
        bus_fwd = 0;  // EON create MDPS for LKAS
        OP_MDPS_live -= 1;
      }
    }
    if (bus_num == 2) {
      if (!OP_LKAS_live || (addr != 0x340 && addr != 0x485)) {
        if (!OP_SCC_live || (addr != 0x420 && addr != 0x421 && addr != 0x50A && addr != 0x389)) {
          bus_fwd = fwd_to_bus1 == 1 ? 10 : 0;
        } else {
          bus_fwd = fwd_to_bus1;  // EON create SCC12 for Car
          OP_SCC_live -= 1;
        }
      } else if (HKG_mdps_bus == 0) {
        bus_fwd = fwd_to_bus1; // EON create LKAS and LFA for Car
        OP_LKAS_live -= 1;
      } else {
        OP_LKAS_live -= 1; // EON create LKAS and LFA for Car and MDPS
      }
    }
  } else {
    if (bus_num == 0) {
      bus_fwd = fwd_to_bus1;
    }
    if (bus_num == 1 && (HKG_forward_bus1 || HKG_forward_obd)) {
      bus_fwd = 0;
    }
  }

  return bus_fwd;
}

static safety_config hyundai_community2_init(uint16_t param) {
  // older hyundai models have less checks due to missing counters and checksums
  static RxCheck hyundai_community2_rx_checks[] = {
    {.msg = {{0x260, 0, 8, .check_checksum = true, .max_counter = 3U, .frequency = 100U},
            {0x371, 0, 8, .frequency = 100U}, { 0 }}},
    {.msg = {{0x386, 0, 8, .frequency = 50U}, { 0 }, { 0 }}},
    // {.msg = {{916, 0, 8, .frequency = 50U}}}, some Santa Fe does not have this msg, need to find alternative
  };
  hyundai_common_init(param);
  controls_allowed = false;
  relay_malfunction_reset2();

  // if (current_board->has_obd && HKG_forward_obd) {
  //   current_board->set_can_mode(CAN_MODE_OBD_CAN2);
  // }

  return BUILD_SAFETY_CFG(hyundai_community2_rx_checks, HYUNDAI_COMMUNITY2_TX_MSGS);
}

const safety_hooks hyundai_community2_hooks = {
  .init = hyundai_community2_init,
  .rx = hyundai_community2_rx_hook,
  .tx = hyundai_community2_tx_hook,
  .fwd = hyundai_community2_fwd_hook,
};
