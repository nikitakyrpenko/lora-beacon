#pragma once

#include <cstdint>

#include "AckPacket.hpp"
#include "ByteOrder.hpp"
#include "SX1280Constants.hpp"

struct AckPacketIn {
  uint8_t wake_word[LORA_BEACON_PROTOCOL::WAKE_WORD_LEN];
  uint16_t ranging_window_ms;

  static AckPacketIn parse(const uint8_t* buf)
  {
    AckPacketIn ack{};
    ack.wake_word[0] = buf[0];
    ack.wake_word[1] = buf[1];

    ack.ranging_window_ms = ByteOrder::get_u16(buf + LORA_BEACON_PROTOCOL::WAKE_WORD_LEN);
    return ack;
  }
};