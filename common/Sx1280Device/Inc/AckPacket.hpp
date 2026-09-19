#pragma once

#include <cstddef>
#include <cstdint>

#include "SX1280Constants.hpp"

struct AckPacket {
  static constexpr size_t SIZE = sizeof(uint32_t) + 3 * sizeof(int16_t);

  uint32_t anchor_id;
  int16_t x_cm;
  int16_t y_cm;
  int16_t z_cm;

  void serialize(uint8_t* out) const
  {
    out[0] = static_cast<uint8_t>(anchor_id >> 24);
    out[1] = static_cast<uint8_t>(anchor_id >> 16);
    out[2] = static_cast<uint8_t>(anchor_id >> 8);
    out[3] = static_cast<uint8_t>(anchor_id & 0xFF);
    put_i16(out + 4, x_cm);
    put_i16(out + 6, y_cm);
    put_i16(out + 8, z_cm);
  }

  static AckPacket parse(const uint8_t* in)
  {
    AckPacket ack{};
    ack.anchor_id = (static_cast<uint32_t>(in[0]) << 24) | (static_cast<uint32_t>(in[1]) << 16) | (static_cast<uint32_t>(in[2]) << 8) |
                    static_cast<uint32_t>(in[3]);
    ack.x_cm = get_i16(in + 4);
    ack.y_cm = get_i16(in + 6);
    ack.z_cm = get_i16(in + 8);
    return ack;
  }

private:
  static void put_i16(uint8_t* out, int16_t value)
  {
    const uint16_t raw = static_cast<uint16_t>(value);  // two's complement bits, sign handled by the cast
    out[0] = static_cast<uint8_t>(raw >> 8);
    out[1] = static_cast<uint8_t>(raw & 0xFF);
  }

  static int16_t get_i16(const uint8_t* in)
  {
    return static_cast<int16_t>(static_cast<uint16_t>((static_cast<uint16_t>(in[0]) << 8) | in[1]));
  }
};

static_assert(sizeof(LORA_BEACON_PROTOCOL::WAKE_ACK) + AckPacket::SIZE == LORA_BEACON_PROTOCOL::WAKE_ACK_PAYLOAD_LEN,
              "WAKE_ACK_PAYLOAD_LEN doesn't match AckPacket's serialized size");
