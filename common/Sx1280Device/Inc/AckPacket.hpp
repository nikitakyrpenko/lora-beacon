#pragma once

#include <cstddef>
#include <cstdint>

#include "ByteOrder.hpp"
#include "SX1280Constants.hpp"

struct AckPacket {
  static constexpr size_t SIZE = sizeof(uint32_t) + 3 * sizeof(int16_t);

  uint32_t anchor_id;
  int16_t x_cm;
  int16_t y_cm;
  int16_t z_cm;

  void serialize(uint8_t* out) const
  {
    ByteOrder::put_u32(out, anchor_id);
    ByteOrder::put_i16(out + 4, x_cm);
    ByteOrder::put_i16(out + 6, y_cm);
    ByteOrder::put_i16(out + 8, z_cm);
  }

  static AckPacket parse(const uint8_t* in)
  {
    AckPacket ack{};
    ack.anchor_id = ByteOrder::get_u32(in);
    ack.x_cm = ByteOrder::get_i16(in + 4);
    ack.y_cm = ByteOrder::get_i16(in + 6);
    ack.z_cm = ByteOrder::get_i16(in + 8);
    return ack;
  }
};

static_assert(sizeof(LORA_BEACON_PROTOCOL::WAKE_ACK) + AckPacket::SIZE == LORA_BEACON_PROTOCOL::WAKE_ACK_PAYLOAD_LEN,
              "WAKE_ACK_PAYLOAD_LEN doesn't match AckPacket's serialized size");
