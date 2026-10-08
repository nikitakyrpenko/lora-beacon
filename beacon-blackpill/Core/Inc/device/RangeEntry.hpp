#pragma once

#include <cstdint>

#include "SynAckPacket.hpp"

// Outcome of ranging one anchor, recorded by BeaconBridge and written into the UART frame (see CycleFrame.hpp).
// A plain struct with no HAL types, so the frame serializer can be built and tested on a PC.

enum class RangeStatus : uint8_t {
  DEFAULT,
  OK,       // distance_cm is valid
  TIMEOUT,  // the anchor acked the wake-up but did not answer the ranging request
  FAILED
};

struct RangeEntry {
  SynAckPacket anchor;     // id + x/y/z cm, as reported in the anchor's own ack
  int32_t distance_cm;  // -1 unless status == OK
  RangeStatus status;
  uint8_t timeout;
};
