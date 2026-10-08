#pragma once

#include <cstdint>

#include "ByteOrder.hpp"
#include "SX1280Constants.hpp"

// Third step of the handshake: SynPacket (beacon wake) -> SynAckPacket (anchor answer) -> AckPacket (beacon: start ranging). Broadcast; an
// anchor starts ranging only if the low byte of its own address is in the list, the others stay in LISTENING.
struct AckPacket {
  uint8_t ack_word[LORA_BEACON_PROTOCOL::ACK_WORD_LEN];
  uint16_t ranging_window_ms;
  uint8_t anchor_ids[LORA_BEACON_PROTOCOL::ACK_MAX_ANCHORS];  // low byte of each selected anchor's address, 0x00 = unused

  static AckPacket parse(const uint8_t* buf)
  {
    AckPacket ack{};
    for (uint8_t i = 0; i < LORA_BEACON_PROTOCOL::ACK_WORD_LEN; i++) {
      ack.ack_word[i] = buf[i];
    }
    ack.ranging_window_ms = ByteOrder::get_u16(buf + LORA_BEACON_PROTOCOL::ACK_WORD_LEN);
    for (uint8_t i = 0; i < LORA_BEACON_PROTOCOL::ACK_MAX_ANCHORS; i++) {
      ack.anchor_ids[i] = buf[LORA_BEACON_PROTOCOL::ACK_WORD_LEN + sizeof(uint16_t) + i];
    }
    return ack;
  }

  inline bool serialize(uint8_t* buf, uint8_t len) const
  {
    if (len < LORA_BEACON_PROTOCOL::ACK_PAYLOAD_LEN) {
      return false;
    }

    for (uint8_t i = 0; i < LORA_BEACON_PROTOCOL::ACK_WORD_LEN; i++) {
      buf[i] = ack_word[i];
    }
    ByteOrder::put_u16(buf + LORA_BEACON_PROTOCOL::ACK_WORD_LEN, ranging_window_ms);
    for (uint8_t i = 0; i < LORA_BEACON_PROTOCOL::ACK_MAX_ANCHORS; i++) {
      buf[LORA_BEACON_PROTOCOL::ACK_WORD_LEN + sizeof(uint16_t) + i] = anchor_ids[i];
    }
    return true;
  }

  inline bool word_matches() const
  {
    for (uint8_t i = 0; i < LORA_BEACON_PROTOCOL::ACK_WORD_LEN; i++) {
      if (ack_word[i] != LORA_BEACON_PROTOCOL::ACK_WORD[i]) {
        return false;
      }
    }
    return true;
  }

  // true if the anchor whose address has this low byte is selected (0x00 is the "unused" marker, never a real address)
  inline bool selects(uint8_t anchor_low_byte) const
  {
    if (anchor_low_byte == 0x00) {
      return false;
    }
    for (uint8_t id : anchor_ids) {
      if (id == anchor_low_byte) {
        return true;
      }
    }
    return false;
  }
};
