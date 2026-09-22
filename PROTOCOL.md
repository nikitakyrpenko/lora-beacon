## Radio
2.45 GHz, SF7, BW 1600 kHz, CR 4/5, 12-symbol preamble. Wake/ack: LoRa, CRC on. Ranging: CRC off. Beacon TX power -5 dBm.

## Cycle (every 5000 ms)
1. Beacon broadcasts the wake word: `BE AC` + ranging window in ms (u16, 2000).
2. Anchor matches it (window clamped to 500..10000 ms) and waits its ack slot: (ANCHOR_ADDRESS - 0xA19) * 20 ms.
3. Anchor acks (12 bytes, MSB-first): `AC 4B` | id (4) | its own position x, y, z (int16 each); then arms as ranging slave.
4. Beacon collects acks: 100 ms RX, re-armed after each ack (RX returns to standby after every packet); the IRQ status tells RX_DONE
   from RX timeout. Ends at EXPECTED_ANCHOR_COUNT (2), on RX timeout, or after 150 ms.
5. Beacon ranges each collected anchor in turn (1000 ms timeout). The ranging result is the distance: distance_cm = raw * 20;
   status OK / TIMEOUT / FAILED.
6. Beacon sends one UART frame; anchors return to radio on response-done or window expiry.

Addresses start at 0xA19, differ in the low byte (8-bit ranging address check) and must stay <= ~0xA1D so the ack fits the 100 ms window.

## UART frame (per cycle, MSB-first)
`A5 5A` | length (1) | cycle (4) | tick_ms (4) | count (1) | count x [id (4), x, y, z (2 each), distance_cm (4), status (1: 0 OK, 1 TIMEOUT, 2 FAILED)] | XOR (1, over length..last entry)

## Known issues
- Ranging requests time out with two anchors. Suspect: SetRx count 0x0000 is Rx single (ends after any frame); the datasheet advises 0xFFFF for
  the slave. Continuous RX gave distances, but one anchor then stopped answering; reverted.
- Ranging calibration is not written (chip default, raw results); calibrate at known distances.
- Idle duty cycling is off, the ranging-window sizing formula is open, and an anchor is not retried when its to_radio() fails.
