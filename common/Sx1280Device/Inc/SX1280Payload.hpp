#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "ByteOrder.hpp"

// Builders for the byte payloads that follow an SX1280 SPI opcode. Every value comes in as a parameter (the register
// addresses, IRQ bits, field values live in SX1280Constants.hpp), so this header depends on nothing but ByteOrder and the
// byte layout of each command is written down exactly once, here. All constexpr: SX1280Constants.hpp defines the ready-made
// payloads from them at compile time, and the bridges pass `.data()` to SX1280Device::SPI_write().
namespace SX1280Payload {

// WriteRegister: [register address, 2 bytes][value, 1 byte]
constexpr std::array<uint8_t, 3> register_write_u8(uint16_t reg, uint8_t value)
{
  std::array<uint8_t, 3> payload{};
  ByteOrder::put_u16(payload.data(), reg);
  payload[2] = value;
  return payload;
}

// WriteRegister: [register address, 2 bytes][value, 2 bytes MSB-first]
constexpr std::array<uint8_t, 4> register_write_u16(uint16_t reg, uint16_t value)
{
  std::array<uint8_t, 4> payload{};
  ByteOrder::put_u16(payload.data(), reg);
  ByteOrder::put_u16(payload.data() + 2, value);
  return payload;
}

// WriteRegister: [register address, 2 bytes][value, 4 bytes MSB-first] -- the ranging address registers (own address on the
// anchor, target address on the beacon)
constexpr std::array<uint8_t, 6> register_write_u32(uint16_t reg, uint32_t value)
{
  std::array<uint8_t, 6> payload{};
  ByteOrder::put_u16(payload.data(), reg);
  ByteOrder::put_u32(payload.data() + 2, value);
  return payload;
}

// SetDioIrqParams (Table 11-71): [irqMask][dio1Mask][dio2Mask][dio3Mask], each 16 bits MSB-first. irqMask says which IRQs
// are latched at all, dio1Mask which of those are routed to DIO1, and so on.
constexpr std::array<uint8_t, 8> irq_params(uint16_t irq_mask, uint16_t dio1_mask, uint16_t dio2_mask = 0, uint16_t dio3_mask = 0)
{
  std::array<uint8_t, 8> payload{};
  ByteOrder::put_u16(payload.data(), irq_mask);
  ByteOrder::put_u16(payload.data() + 2, dio1_mask);
  ByteOrder::put_u16(payload.data() + 4, dio2_mask);
  ByteOrder::put_u16(payload.data() + 6, dio3_mask);
  return payload;
}

// SetTx / SetRx (Table 11-24): [periodBase][periodBaseCount, 16 bits MSB-first]. count 0 = no timeout (SetTx: single shot,
// SetRx: listen indefinitely).
constexpr std::array<uint8_t, 3> period_params(uint8_t period_base, uint16_t period_base_count)
{
  std::array<uint8_t, 3> payload{};
  payload[0] = period_base;
  ByteOrder::put_u16(payload.data() + 1, period_base_count);
  return payload;
}

// SetRfFrequency: the 24-bit PLL word, MSB-first
constexpr std::array<uint8_t, 3> rf_frequency_params(uint32_t frequency_reg)
{
  std::array<uint8_t, 3> payload{};
  ByteOrder::put_u24(payload.data(), frequency_reg);
  return payload;
}

// SetPacketParams, LoRa/Ranging layout (Table 11-58): [preamble][headerType][payloadLength][CRC][invertIQ][unused][unused]
constexpr std::array<uint8_t, 7> lora_packet_params(
  uint8_t preamble, uint8_t header_type, uint8_t payload_length, uint8_t crc, uint8_t invert_iq)
{
  return {preamble, header_type, payload_length, crc, invert_iq, 0x00, 0x00};
}

// constexpr element-wise comparison (std::array's operator== is only constexpr from C++20); for static_asserts
template <size_t N>
constexpr bool array_equal(const std::array<uint8_t, N>& a, const std::array<uint8_t, N>& b)
{
  for (size_t i = 0; i < N; ++i) {
    if (a[i] != b[i]) {
      return false;
    }
  }
  return true;
}

}  // namespace SX1280Payload
