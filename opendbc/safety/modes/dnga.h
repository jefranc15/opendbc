#pragma once

#include "opendbc/safety/declarations.h"

// Road-Ready DNGA Safety Model ported from the user's DragonPilot 0.8.13
// safety_toyota.h. This intentionally preserves the permissive Python-latch
// engagement model and the four-message DNGA TX whitelist used on the Yaris
// Cross HEV.

static void dnga_rx_hook(const CANPacket_t *msg) {
  // Dynamic vehicle moving check from WHEEL_SPEED (416 / 0x1A0).
  // Preserve the original DragonPilot byte interpretation exactly.
  if (msg->addr == 416U) {
    const uint32_t speed_front =
      ((uint32_t)msg->data[0] << 16U) |
      ((uint32_t)msg->data[1] << 8U) |
      (uint32_t)msg->data[2];
    vehicle_moving = speed_front > 0U;
  }

  // Cruise engagement is reconstructed by the DNGA Python cruise latch.
  // Keep C safety permissive, matching the proven DragonPilot model.
  controls_allowed = true;
}

static bool dnga_tx_hook(const CANPacket_t *msg) {
  // Preserve the original DragonPilot whitelist exactly:
  //   464 / 0x1D0 STEERING_LKAS
  //   625 / 0x271 ACC_BRAKE
  //   627 / 0x273 ACC_CMD_HUD
  //   628 / 0x274 LKAS_HUD
  return (msg->addr == 464U) ||
         (msg->addr == 625U) ||
         (msg->addr == 627U) ||
         (msg->addr == 628U);
}

static bool dnga_fwd_hook(int bus_num, int addr) {
  // Current opendbc maps bus 0 <-> bus 2 globally. The hook now returns
  // whether a frame should be blocked rather than the destination bus.
  // Match the original DragonPilot behavior by blocking the stock camera's
  // four LKAS/ACC commands on bus 2 while forwarding everything else.
  return (bus_num == 2) &&
         ((addr == 464) || (addr == 625) || (addr == 627) || (addr == 628));
}

static safety_config dnga_init(uint16_t param) {
  // Current opendbc only calls a mode's RX hook for registered RX messages.
  // Register only WHEEL_SPEED so the original vehicle_moving logic executes;
  // ignore checksum/counter/quality exactly as the old permissive model did.
  static RxCheck dnga_rx_checks[] = {
    {.msg = {{416, 0, 8, 50U,
              .ignore_checksum = true,
              .ignore_counter = true,
              .max_counter = 0U,
              .ignore_quality_flag = true}, {0}, {0}}},
  };

  static const CanMsg DNGA_TX_MSGS[] = {
    {464, 0, 8, .check_relay = true},  // STEERING_LKAS
    {625, 0, 8, .check_relay = true},  // ACC_BRAKE
    {627, 0, 8, .check_relay = true},  // ACC_CMD_HUD
    {628, 0, 8, .check_relay = true},  // LKAS_HUD
  };

  SAFETY_UNUSED(param);
  controls_allowed = true;

  return BUILD_SAFETY_CFG(dnga_rx_checks, DNGA_TX_MSGS);
}

const safety_hooks dnga_hooks = {
  .init = dnga_init,
  .rx = dnga_rx_hook,
  .tx = dnga_tx_hook,
  .fwd = dnga_fwd_hook,
};
