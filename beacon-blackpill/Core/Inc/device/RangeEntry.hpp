#pragma once

#include <cstdint>

#include "AckPacket.hpp"

// Outcome of ranging one anchor, recorded by BeaconBridge and written into the UART frame (see CycleFrame.hpp).
// A plain struct with no HAL types, so the frame serializer can be built and tested on a PC.

enum class RangeStatus : uint8_t {
  OK,       // distance_cm is valid
  TIMEOUT,  // the anchor acked the wake-up but did not answer the ranging request
  FAILED,   // configuration or result readback failed, or the anchor was never ranged (an earlier configuration failed)
};

struct RangeEntry {
  AckPacket anchor;     // id + x/y/z cm, as reported in the anchor's own ack
  int32_t distance_cm;  // -1 unless status == OK
  RangeStatus status;
};
