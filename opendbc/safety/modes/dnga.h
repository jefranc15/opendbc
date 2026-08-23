#pragma once

#include "opendbc/safety/declarations.h"

static uint32_t dnga_get_wheelspeed_raw(const CANPacket_t *msg) {
  uint32_t wheelspeed_raw;

  // WHEEL_SPEED.WHEELSPEED_F: 7|24@0+
  wheelspeed_raw =
    ((uint32_t)((msg->data[0] >> 7U) & 0x1U)) |
    ((uint32_t)(msg->data[1]) << 1U) |
    ((uint32_t)(msg->data[2]) << 9U) |
    ((uint32_t)(msg->data[3] & 0x7FU) << 17U);

  return wheelspeed_raw;
}

static uint32_t dnga_get_acc_cmd_raw(const CANPacket_t *msg) {
  uint32_t acc_cmd_raw;

  acc_cmd_raw =
    ((uint32_t)((msg->data[2] >> 7U) & 0x1U)) |
    ((uint32_t)(msg->data[3]) << 1U) |
    ((uint32_t)(msg->data[4] & 0x7FU) << 9U);

  return acc_cmd_raw;
}

static bool dnga_get_bit_bool(const CANPacket_t *msg, uint32_t bit_index) {
  return GET_BIT(msg, bit_index) != 0U;
}

static void dnga_rx_hook(const CANPacket_t *msg) {
  // Preserve BukaPilot DNGA engagement behavior for the initial port.
  // The Python cruise latch is the current engagement authority.
  controls_allowed = true;

  if (msg->addr == 416U) {
    const uint32_t wheelspeed_raw = dnga_get_wheelspeed_raw(msg);
    vehicle_moving = (wheelspeed_raw != 0U);
  }

  if (msg->addr == 399U) {
    // carstate: gasPressed = not GAS_PEDAL_2.GAS_PEDAL_STEP
    gas_pressed = !dnga_get_bit_bool(msg, 1U);
  }

  if (msg->addr == 161U) {
    // BRAKE.BRAKE_ENGAGED: bit 5
    brake_pressed = dnga_get_bit_bool(msg, 5U);
  }
}

static bool dnga_tx_hook(const CANPacket_t *msg) {
  static const uint8_t DNGA_ACC_BRAKE_MAGNITUDE_RAW_MIN = 44U;
  static const uint8_t DNGA_ACC_BRAKE_MAGNITUDE_RAW_MAX = 200U;
  static const uint8_t DNGA_ACC_BRAKE_PUMP_RAW_MAX = 10U;
  bool tx = true;

  // ACC_CMD_HUD: if no engagement/request bit is asserted, ACC_CMD must be 0.
  if (msg->addr == 627U) {
    const bool set_me_1_2 = dnga_get_bit_bool(msg, 9U);
    const bool set_1_when_engage = dnga_get_bit_bool(msg, 13U);
    const bool bit_37_set = dnga_get_bit_bool(msg, 37U);
    const bool bit_38_set = dnga_get_bit_bool(msg, 38U);
    const bool engage_requested = set_me_1_2 || set_1_when_engage || bit_37_set || bit_38_set;

    if (!engage_requested) {
      const uint32_t acc_cmd_raw = dnga_get_acc_cmd_raw(msg);
      if (acc_cmd_raw != 0U) {
        tx = false;
      }
    }
  }

  // ACC_BRAKE: preserve BukaPilot's initial payload envelope.
  if (msg->addr == 625U) {
    const bool set_me_1_when_engage = dnga_get_bit_bool(msg, 8U);

    if (!set_me_1_when_engage) {
      const bool brake_req = dnga_get_bit_bool(msg, 13U);
      if (brake_req) {
        tx = false;
      }
    }

    const uint8_t magnitude_raw = msg->data[5];
    if ((magnitude_raw < DNGA_ACC_BRAKE_MAGNITUDE_RAW_MIN) ||
        (magnitude_raw > DNGA_ACC_BRAKE_MAGNITUDE_RAW_MAX)) {
      tx = false;
    }

    const uint8_t pump_raw = msg->data[4];
    if (pump_raw > DNGA_ACC_BRAKE_PUMP_RAW_MAX) {
      tx = false;
    }

    const uint8_t pump_inverse_raw = (uint8_t)(0U - (uint32_t)pump_raw);
    if (msg->data[3] != pump_inverse_raw) {
      tx = false;
    }
  }

  return tx;
}

static safety_config dnga_init(uint16_t param) {
  static const CanMsg DNGA_TX_MSGS[] = {
    {464, 0, 8, .check_relay = true},   // STEERING_LKAS
    {628, 0, 8, .check_relay = true},   // LKAS_HUD
    {625, 0, 8, .check_relay = true},   // ACC_BRAKE
    {627, 0, 8, .check_relay = true},   // ACC_CMD_HUD
    {519, 0, 6, .check_relay = false},  // PCM_BUTTONS_HYBRID
    {520, 0, 6, .check_relay = false},  // PCM_BUTTONS
    {2015, 0, 8, .check_relay = false}, // DTC clear
  };

  // BukaPilot uses 1 Hz placeholders here. Current sunnypilot/opendbc rejects
  // safety-relevant RX checks below 10 Hz, so use the actual DNGA parser rates.
  static RxCheck dnga_rx_checks[] = {
    {.msg = {{416, 0, 8, 50U,  .ignore_checksum = true, .ignore_counter = true, .max_counter = 0U, .ignore_quality_flag = true}, {0}, {0}}},
    {.msg = {{399, 0, 8, 60U,  .ignore_checksum = true, .ignore_counter = true, .max_counter = 0U, .ignore_quality_flag = true}, {0}, {0}}},
    {.msg = {{161, 0, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .max_counter = 0U, .ignore_quality_flag = true}, {0}, {0}}},
    {.msg = {{627, 2, 8, 20U,  .ignore_checksum = true, .ignore_counter = true, .max_counter = 0U, .ignore_quality_flag = true}, {0}, {0}}},
    {.msg = {{625, 2, 8, 20U,  .ignore_checksum = true, .ignore_counter = true, .max_counter = 0U, .ignore_quality_flag = true}, {0}, {0}}},
  };

  SAFETY_UNUSED(param);
  controls_allowed = true;

  return BUILD_SAFETY_CFG(dnga_rx_checks, DNGA_TX_MSGS);
}

const safety_hooks dnga_hooks = {
  .init = dnga_init,
  .rx = dnga_rx_hook,
  .tx = dnga_tx_hook,
};
