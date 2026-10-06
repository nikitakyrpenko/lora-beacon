#pragma once

#include <array>
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

  inline bool serialize(uint8_t* buf, uint8_t len) const
  {
    if (len < LORA_BEACON_PROTOCOL::WAKE_PAYLOAD_LEN) {
      return false;
    }

    buf[0] = wake_word[0];
    buf[1] = wake_word[1];
    ByteOrder::put_u16(buf + 2, ranging_window_ms);
    return true;
  }
};