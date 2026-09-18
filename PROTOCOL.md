# Multi-anchor protocol

Status: anchor flow implemented. Beacon collect/range: implemented, not yet hardware-verified.

## Anchor flow (agreed)

1. Idle, listening for wake word (`SX1280_Radio_mode()`).
2. On match: extract + clamp ranging-window duration from payload (`RANGING_WINDOW_MIN_MS..MAX_MS`).
3. Compute `slot_delay_ms` from own address (deterministic, no beacon coordination):
   ```
   slot_delay_ms = (ANCHOR_RANGING_ADDRESS - RANGING_ADDRESS_BLOCK_BASE) * ANCHOR_ACK_SLOT_WIDTH_MS
   ```
   - `RANGING_ADDRESS_BLOCK_BASE = 0x00000A19`
   - `ANCHOR_ACK_SLOT_WIDTH_MS` ~20ms (> ack airtime + SPI jitter)
   - Why needed: without staggering, multiple anchors ACK at the same instant -> RF collision -> beacon never
     cleanly receives any of them -> never learns those addresses -> never queries them for ranging at all. Not
     about ranging-exchange safety (already safe, see below) -- about the beacon receiving the ACK in the first
     place.
4. Wait (non-blocking, tick-deadline, no `HAL_Delay`) until `slot_delay_ms` elapses.
5. Send ACK: `WAKE_ACK` (2B) + own 4B address.
6. Enter ranging-slave mode, arm local `ranging_window_ms` timer (clamped value from step 2).
7. **Early exit**: `IRQ_BIT_RANGING_SLAVE_RESPONSE_DONE` already routed to DIO1 in `SX1280_Ranging_Slave_Mode()`,
   just unused today (`main.c` logs and ignores DIO1 fires while `ranging_active`). Fix: when DIO1 fires during
   `ranging_active`, read IRQ status; if `RangingSlaveResponseDone` set, exit to idle immediately. Self-directed by
   the anchor's own IRQ, not a beacon-sent message -- doesn't reintroduce the "stuck forever if message lost" risk
   PLAN.md already rejected.
8. **Fallback**: if the window timer expires first (anchor never reached/ranged), exit to idle anyway. Not a
   replacement for step 7, a safety net alongside it.

## Beacon collect phase (designed)
Can't range + listen at once — radio is half-duplex, a late ACK during ranging is lost. So: collect all ACKs first, range after.

`SX1280_Listen_For_Ack()`: continuous RX (`rx[3] = {PERIOD_BASE_1_MS, 0xFF, 0xFF}`), IRQ mask = `RX_DONE` only (no timeout in continuous mode, no re-arm needed between catches — chip keeps listening on its own). New `SX1280_Stop_Ack_Listen()` (`SetStandby(RC)`) before leaving, since `SX1280_Ranging_Master_Mode()` expects `STDBY_RC` already.

3-state machine (`main.c`): `BEACON_IDLE -> BEACON_COLLECTING -> BEACON_RANGING -> BEACON_IDLE`.
- COLLECTING: each DIO1 catch -> clear IRQ (every catch, not just once) -> validate+dedup+store in `collected_addresses[EXPECTED_ANCHOR_COUNT]`. Stop when `count==3` or `collect_phase_deadline_tick` elapses (fallback).
- 0 collected -> straight back to idle, no wasted ranging attempts.

## Beacon range phase (designed)
After collect closes: sequentially range each collected address (existing single-anchor logic, looped via `ranging_index`). Hardware address-filtering already makes this collision-safe regardless of anchor count.

Wake payload's ranging-window duration must cover the worst case since range phase is sequential — last-ranged anchor's window must still be open when the beacon finally reaches it. Not yet finalized; partially relieved by the anchor's own early-exit (step 7).

## Collision inventory
1. Wake broadcast — safe (1 beacon)
2. ACK — fixed by slot delay above
3. Ranging exchange — safe (addressed, hw filtered)
4. Beacon state races — already guarded
5. DIO1 latch — fixed for single-ack; must extend to collect loop and to the anchor's early-exit check
6. External RF interference — not protocol-fixable, antenna is separate known weak point

## Open params
- `EXPECTED_ANCHOR_COUNT = 3`
- collect ceiling >= `3 * ANCHOR_ACK_SLOT_WIDTH_MS` + margin
- beacon: fixed array[3] + count for collected addresses
- wake payload ranging-window duration sizing (worst-case formula, not finalized)
