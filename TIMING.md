# Timing target: 5-10 Hz position fixes

## Goal
Beacon's ranging output is meant to substitute for a GPS position provider on a flight
controller. That means position fixes at 5-10 Hz (100-200 ms per fix), not the current
on-demand cycle.

## Budget
Single radio per beacon, anchors ranged sequentially. With N anchors in range at once
(expected deployment: 3-4), the per-anchor budget within one fix period is:

| Target rate | Anchors | Per-anchor budget |
|---|---|---|
| 10 Hz | 4 | 25 ms |
| 5 Hz | 3 | 66 ms |

## Current numbers (see PROTOCOL.md)
- Fixed 5000 ms cycle interval
- Wake + ack collect: up to 150 ms (100 ms RX window per anchor, re-armed per ack)
- Per-anchor ranging: 1000 ms timeout
- Worst case per cycle already exceeds 2 s for 2 anchors, and that only runs once per 5 s

These are safety-margin timeouts, not real radio time. Real SF7/BW1600 over-air exchange
(preamble ~1.3 ms + short payload + SX1280 internal turnaround) is expected in the low
single-digit ms, so there are roughly 2-3 orders of magnitude of margin baked into the
current numbers.

## What has to change
1. **Decouple discovery from the hot loop.** Wake + collect (anchor discovery, position
   exchange) runs rarely, not every fix; the fast loop only re-ranges already-known
   anchors by address, skipping the ack/collect phase entirely.
2. **Shrink per-anchor ranging timeout** from 1000 ms toward tens of ms, sized against
   measured real exchange time (needs a hardware measurement, not a datasheet estimate).
3. **Replace the fixed 5000 ms cycle interval** with the target fix period (100-200 ms).
4. **Anchor side needs a change: see below.** The window field's meaning changes from
   "ceiling for one exchange" to "inactivity timeout for the whole session," and
   `on_ranging()`'s exit condition has to split accordingly.

## Anchor side: window becomes an inactivity timeout, not a session ceiling
Reuses the existing wake-word window field instead of adding a new session concept.

- Wake arrives with window size (unchanged wire format). Anchor acks, arms as ranging
  slave, starts the TIM6 deadline = window — same as today.
- **Change**: on `RESPONSE_DONE`, instead of exiting to `LISTENING`, re-arm as ranging
  slave again, stay in `RANGING`, and **refresh the TIM6 deadline back to the full
  window** (same pattern as the IWDG feed already in this codebase — proof of a
  successful exchange resets the clock). So the window behaves as an inactivity timeout:
  it never expires while the beacon keeps successfully ranging this anchor, and only
  counts down for real once responses actually stop.
- Window expires for real (no successful response refreshed it in time) -> back to
  `LISTENING`. If the beacon is still around, it just sends another wake to open a fresh
  window; no separate renewal message.
- Genuine failures (header/CRC error, etc.) still go to `RECOVER` same as now — that edge
  doesn't change, only the `RESPONSE_DONE` exit does.
- No wake at all during some idle stretch of `LISTENING` -> drop to RxDutyCycle + deep
  sleep (see README's recovery/power section).

Sleep depth falls out of this for free: README's existing rule already puts any
TIM6-armed state (`RANGING` included) in shallow sleep only. Since the anchor now stays
in `RANGING` for the whole active dwell instead of one exchange, it is automatically
shallow-sleep the entire time, and automatically eligible for deep sleep the moment it's
back in plain `LISTENING` with nothing armed — no separate "active session" grace-period
variable needed.

The window field now serves two different jobs sharing one number: max allowed gap since
the last successful exchange before giving up (the inactivity timeout above) vs.
per-anchor exchange budget (25-66 ms, a beacon-side pacing concern the anchor never
times). Current default 2000 ms / clamp 500-10000 ms was sized for the old
single-exchange meaning — likely still a reasonable inactivity timeout, but keep these
two meanings distinct going forward.

## Beacon side: discovery retries until prerequisites met, then holds the session
Ties the adaptive re-discovery idea above to a concrete two-phase beacon behavior:

- **Wake/discovery phase**: keep retrying discovery (short backoff) as long as collected
  anchor count < `EXPECTED_ANCHOR_COUNT`. A lost ack just means another short-backoff
  retry, not a long wait.
- **Prerequisites met** (collected count == expected count): stop re-running discovery,
  move into the active-session fast ranging loop against the now-complete cached anchor
  list.
- Drop back to the wake/discovery phase only when the active session actually needs
  re-acquiring an anchor (repeated ranging failures suggest it left range), not on a
  fixed timer while everything is still responding.

## Discovery failure modes: bad wake vs. bad ack
Two different ways a discovery round can lose an anchor, needing two different fixes.

**Bad wake** — anchor never hears the broadcast at all. No side effect on the anchor
(it never armed anything); fixed entirely on the beacon side by the retry-until-
`EXPECTED_ANCHOR_COUNT` behavior above. No anchor-side change needed.

**Bad ack** — anchor hears the wake, sends its ack, gets local `TX_DONE`, and arms as
ranging slave — but the ack packet itself is lost in the air, so the beacon never learns
this anchor exists. The anchor has no way to know its ack failed. Without a fix it sits
armed in `RANGING` for the *entire* window duration waiting for a master request that
will never arrive (the beacon doesn't know to send one), and while armed it can't hear a
repeat wake either (`RANGING`'s radio config isn't listening for wake-word LoRa packets,
and wake matching only runs from `Mode::LISTENING`). That both wastes a full window of
dead time and works against the beacon's short-backoff retry design, since the missing
anchor may still be armed-and-deaf for most of that backoff gap.

Fix: split the timer armed at `ACK_SENT` -> `RANGING` into two different deadlines
instead of always using `latch.ranging`:
- **First arm** (right after `to_ranging()` from `ACK_SENT`): a short deadline, not the
  full window. If no `RESPONSE_DONE` arrives in time, the ack almost certainly never
  reached the beacon -> fall back to `LISTENING` via the existing "window closed" edge,
  fast, so the anchor is catchable again by the next wake almost immediately instead of
  after a multi-second stall.
- **Every re-arm after a successful `RESPONSE_DONE`** (inside `on_ranging()`): keep using
  `latch.ranging`, the long refresh/inactivity window from the section above — this is
  the legitimate ongoing-session case.

No new state or flag needed — just which constant gets passed to `arm_timer()` at the two
different call sites (`on_ack_done()`'s first arm vs. `on_ranging()`'s success re-arm).

The short first-deadline can't be too short: the beacon might not issue a request to a
newly-acked anchor immediately if it's mid-round-robin through other cached anchors
first, so the deadline needs to comfortably exceed that worst-case beacon-side delay, not
just the raw radio exchange time — depends on the beacon's loop structure, not yet nailed
down (see open questions).

## Open questions
- Real measured per-anchor ranging exchange time (timestamp request TX vs
  `RESPONSE_DONE` on hardware).
- MCU-side turnaround per anchor (SPI transactions, state machine passes) against the
  25-66 ms per-anchor budget above.
- Ceiling on discovery-retry backoff: a genuinely absent anchor (not just a lost ack)
  will also fail every short-backoff retry, so there needs to be a give-up point after M
  retries that falls back to the long interval, or the beacon pins itself in a tight
  discovery loop forever waiting for an anchor that isn't coming back.
- Whether a single missed/late ranging request inside an active window should be
  tolerated (anchor just keeps waiting) or treated as a signal the beacon left early.
