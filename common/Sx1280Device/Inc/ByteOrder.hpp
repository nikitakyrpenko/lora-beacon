#pragma once

#include <array>
#include <cstdint>

// MSB-first ("big-endian") byte packing, as used everywhere on the wire in this project: the SX1280 SPI commands, the wake/ack
// payloads (AckPacket) and the beacon's UART frame (CycleFrame). Header-only and free of HAL types, so it can also be built
// on a PC. `put_*` writes to out[0..n), `get_*` reads from in[0..n); the caller guarantees the buffer is long enough.
namespace ByteOrder {

inline void put_u16(uint8_t* out, uint16_t value)
{
  out[0] = static_cast<uint8_t>(value >> 8);
  out[1] = static_cast<uint8_t>(value & 0xFF);
}

inline void put_u32(uint8_t* out, uint32_t value)
{
  out[0] = static_cast<uint8_t>(value >> 24);
  out[1] = static_cast<uint8_t>(value >> 16);
  out[2] = static_cast<uint8_t>(value >> 8);
  out[3] = static_cast<uint8_t>(value & 0xFF);
}

// signed values go through their unsigned two's-complement bits, so negative numbers round-trip
inline void put_i16(uint8_t* out, int16_t value)
{
  put_u16(out, static_cast<uint16_t>(value));
}

inline void put_i32(uint8_t* out, int32_t value)
{
  put_u32(out, static_cast<uint32_t>(value));
}

inline uint16_t get_u16(const uint8_t* in)
{
  return static_cast<uint16_t>((static_cast<uint16_t>(in[0]) << 8) | in[1]);
}

inline uint32_t get_u24(const uint8_t* in)
{
  return (static_cast<uint32_t>(in[0]) << 16) | (static_cast<uint32_t>(in[1]) << 8) | static_cast<uint32_t>(in[2]);
}

inline uint32_t get_u32(const uint8_t* in)
{
  return (static_cast<uint32_t>(in[0]) << 24) | (static_cast<uint32_t>(in[1]) << 16) | (static_cast<uint32_t>(in[2]) << 8) |
         static_cast<uint32_t>(in[3]);
}

inline int16_t get_i16(const uint8_t* in)
{
  return static_cast<int16_t>(get_u16(in));
}

inline int32_t get_i32(const uint8_t* in)
{
  return static_cast<int32_t>(get_u32(in));
}

// SX1280 WriteRegister payload for a 32-bit value: [register address MSB][register address LSB][value, MSB-first, 4 bytes].
// Used for the ranging address registers (own address on the anchor, target address on the beacon).
inline std::array<uint8_t, 6> register_write_u32(uint16_t reg, uint32_t value)
{
  std::array<uint8_t, 6> frame{};
  put_u16(frame.data(), reg);
  put_u32(frame.data() + 2, value);
  return frame;
}

}  // namespace ByteOrder
