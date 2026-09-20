#pragma once

#include <cstddef>
#include <cstdint>

#include "ByteOrder.hpp"
#include "RangeEntry.hpp"
#include "SX1280Constants.hpp"

// Binary frame sent over the UART for one finished cycle: a plain uint8_t array, all multi-byte values MSB-first (same as
// the ack, see AckPacket).
//
//   [0xA5][0x5A]              sync
//   [length]                  bytes from `cycle` up to and including the last entry (everything before the checksum)
//   [cycle: 4]                running counter, lets the receiver notice lost cycles
//   [tick_ms: 4]              beacon HAL tick when the cycle finished
//   [count: 1]                anchors that acked = number of entries; 0 is valid (nobody answered the wake-up)
//   count x entry (15 bytes): [anchor id: 4][x_cm: 2][y_cm: 2][z_cm: 2]   (AckPacket, int16 cm from the site origin)
//                             [distance_cm: 4, int32, -1 unless status == 0][status: 1: 0 OK, 1 TIMEOUT, 2 FAILED]
//   [checksum: 1]             XOR of every byte from [length] through the last entry
//
// Debug logs share the UART, so a receiver hunts for the sync bytes, reads `length`, and accepts the frame only if the
// checksum matches.
namespace CycleFrame {

static constexpr uint8_t SYNC_0 = 0xA5;
static constexpr uint8_t SYNC_1 = 0x5A;
static constexpr size_t ENTRY_BYTES = AckPacket::SIZE + sizeof(int32_t) + sizeof(uint8_t);     // 15
static constexpr size_t HEADER_BYTES = sizeof(uint32_t) + sizeof(uint32_t) + sizeof(uint8_t);  // cycle + tick + count
static constexpr size_t MAX_BYTES =
  2 /*sync*/ + 1 /*length*/ + HEADER_BYTES + LORA_BEACON_PROTOCOL::EXPECTED_ANCHOR_COUNT * ENTRY_BYTES + 1 /*checksum*/;
static_assert(HEADER_BYTES + LORA_BEACON_PROTOCOL::EXPECTED_ANCHOR_COUNT * ENTRY_BYTES <= 255,
              "frame length doesn't fit the 1-byte length field");

// Writes the frame for `count` entries into out; returns the number of bytes written, or 0 if `capacity` is too small.
inline size_t Serialize(uint32_t cycle, uint32_t tick_ms, const RangeEntry* entries, uint8_t count, uint8_t* out, size_t capacity)
{
  if (count > LORA_BEACON_PROTOCOL::EXPECTED_ANCHOR_COUNT) {
    count = LORA_BEACON_PROTOCOL::EXPECTED_ANCHOR_COUNT;
  }
  const size_t payload_bytes = HEADER_BYTES + count * ENTRY_BYTES;
  const size_t total_bytes = 2 + 1 + payload_bytes + 1;
  if (capacity < total_bytes) {
    return 0;
  }

  size_t pos = 0;
  out[pos++] = SYNC_0;
  out[pos++] = SYNC_1;
  out[pos++] = static_cast<uint8_t>(payload_bytes);
  ByteOrder::put_u32(&out[pos], cycle);
  pos += 4;
  ByteOrder::put_u32(&out[pos], tick_ms);
  pos += 4;
  out[pos++] = count;
  for (uint8_t i = 0; i < count; ++i) {
    entries[i].anchor.serialize(&out[pos]);
    pos += AckPacket::SIZE;
    ByteOrder::put_i32(&out[pos], entries[i].distance_cm);
    pos += 4;
    out[pos++] = static_cast<uint8_t>(entries[i].status);
  }

  uint8_t checksum = 0;
  for (size_t i = 2; i < pos; ++i) {  // from [length], sync bytes excluded
    checksum ^= out[i];
  }
  out[pos++] = checksum;
  return pos;
}

}  // namespace CycleFrame
