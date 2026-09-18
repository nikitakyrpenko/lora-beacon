# Permanent Hardware Bring-Up Test for SX1280 Boards

## Context

Every new beacon/anchor board is hand-assembled (STM32 Blackpill + a hand-soldered SX1280 module on
pitch-adapter carrier boards), and this session's debugging found two real fault classes the hard way:
a `NRESET` pulse that can permanently wedge `BUSY` HIGH, and a hand-soldered `MISO` joint that produced
a plausible-looking-but-garbage SPI response (`hal=HAL_OK`, but the status byte was a constant,
timing-independent `0xFF`/`0x00` — nothing real was being driven back). Diagnosing that took an entire
session of throwaway, hand-rolled probes (`BUSY_ONLY_PROBE` modes 1-8 in `beacon-blackpill/main.c`),
including a real race-condition bug in the ad-hoc test code itself (clocking SPI before waiting for
`BUSY`, unlike the real driver).

The goal now is to turn that hard-won knowledge into a **permanent, reusable bring-up test** so the next
newly-assembled board can be verified in minutes, not a multi-day debugging session — and to remove the
throwaway ladder once it's superseded.

Per discussion with the user, this is **two distinct, separately-triggered layers**:

1. **Board-level** — verify the STM32 board's own GPIO/UART/SPI wiring, independent of whether the SX1280
   module is even attached or known-good. Output pins are checked by toggling + printing, with the user
   probing each physical pin with a multimeter/LED at their own pace (no extra jumper wiring required for
   this part). SPI3 is checked via a MOSI↔MISO loopback jumper (same technique used ad hoc this session).
2. **Module-level** — once the SX1280 is attached, verify the chip itself: reset sequence, a real
   `GetStatus` response (rejecting the exact garbage patterns this session found), a register
   write/readback round-trip on a different opcode (independent corroboration), and the DIO1/EXTI
   interrupt path — all using the real gated driver path (`SPI_write`), so the mode-6 race-condition bug
   class is structurally impossible here.

Both replace the old `BUSY_ONLY_PROBE` ladder with one permanent, well-documented flag.

## Layer 1: Board-level bring-up (`BRINGUP_MODE == 1`)

New code lives directly in each board's `main.c` (mirrors how `MX_GPIO_Init()`/etc. are already
per-board duplicated CubeMX-adjacent code — this is MCU/peripheral-level, not SX1280-protocol-level, so
it doesn't belong in `common/Sx1280Device/`). Roughly identical between beacon and anchor since the
pinout is shared; write once, copy to the other board's `main.c`.

Runs forever (interactive/manual — there's no automatic pass/fail for a pin the firmware can't read back
on its own), until reset or reflashed:

- **UART sanity**: print a clear `"[tick] BOARD-BRINGUP: UART TX OK if you can read this"` banner at
  start. Seeing any output at all *is* the TX test. (Optional stretch, not required for v1: a short
  `HAL_UART_Receive` poll-and-echo loop to also confirm RX, since `huart1` is already configured
  `UART_MODE_TX_RX`.)
- **GPIO output toggle test**: cycle through `NSS` (PA7), `NRESET` (PB5), `TCXOEN` (PB6), `LED` (PC13,
  active-low) — drive each HIGH, print `"[tick] BOARD-BRINGUP: <PIN> = HIGH, probe now"`,
  `HAL_Delay(2000)`, drive LOW, print, `HAL_Delay(2000)`, repeat forever. Long enough dwell time for a
  multimeter or LED to confirm each pin at its physical header.
- **GPIO input read test**: poll and print raw `BUSY` (PB7) level every ~300ms so the user can watch it
  change while manually touching a jumper to 3V3/GND on that pin. Do the same for `DIO1` (PB8) raw level,
  *and* print whenever the existing `DIO1_Callback_detected` EXTI flag transitions — this confirms both
  the raw wire and the interrupt/NVIC path when the user manually pulses DIO1 high.
- **SPI3 loopback test**: requires the user to jumper `MOSI` to `MISO` on the SPI3 header (documented in
  the startup banner print). Firmware sends a repeating known byte pattern via
  `HAL_SPI_TransmitReceive` on `hspi3`, compares the echoed byte, and prints a running
  `"[tick] BOARD-BRINGUP: SPI loopback byte 0x%02X -> 0x%02X %s"` (`MATCH`/`MISMATCH`) — this is the one
  sub-check with a real automatic pass/fail, since it's fully internal to the STM32 (no module needed).

## Layer 2: Module-level bring-up (`BRINGUP_MODE == 2`)

### New shared module: `common/Sx1280Device/{Inc,Src}/SX1280BringUp.hpp/.cpp`

Add `Src/SX1280BringUp.cpp` to `common/Sx1280Device/CMakeLists.txt`'s `add_library(Sx1280Device STATIC ...)` sources.

Role-agnostic free functions (matches the existing `execute_step`/`step_ok` free-function style, not a
class — no state carried between calls). Each does its own `printf("[%lu] ...")` diagnostics and returns
`bool`. Each calls only `SX1280Device`'s already-public API (`NRESET_reset`, `SPI_write`, and the new
`GetStatus` below) — **never** hand-rolled SPI like the old mode-6 probe did, which is what makes this
immune to that race-condition bug by construction (confirmed: `SPI_write()` and `NSS_begin()` both call
`BUSY_wait()` *before* touching NSS — see `common/Sx1280Device/Src/SX1280Device.cpp` lines 119 and 55).

```cpp
namespace SX1280BringUp {
bool CheckIdleGpioState(GPIO_TypeDef* NSS_port, uint16_t NSS_pin,
                         GPIO_TypeDef* NRESET_port, uint16_t NRESET_pin,
                         GPIO_TypeDef* TCXOEN_port, uint16_t TCXOEN_pin,
                         GPIO_TypeDef* BUSY_port, uint16_t BUSY_pin);
bool CheckResetSequence(SX1280Device& device);
bool CheckChipAlive(SX1280Device& device, SX1280Device::SX1280_Status* status_out, uint8_t* raw_status_out);
bool CheckRegisterReadback(SX1280Device& device);
bool CheckDio1Irq(SX1280Device& device, volatile uint8_t* dio1_flag);
}
```

### New `SX1280Device::GetStatus()` (closes an existing gap)

Add to `SX1280Device.hpp` public section, right after `SPI_write` (~line 69):
`HAL_StatusTypeDef GetStatus(SX1280_Status* out);`

Implement in `SX1280Device.cpp` via the existing `SPI_write(&SX1280_OPERATIONS::GET_STATUS_OP_CODE, tx, nullptr, 1, out)`
— this is exactly what the beacon bridge's `SX1280_Get_Status_Raw` (`SX1280Bridge.cpp:191-206`) already
hand-rolls; promote it to the driver class. Requires adding `#include "SX1280Constants.hpp"` to
`SX1280Device.cpp` (not currently included there). After this exists, reimplement
`SX1280_Get_Status_Raw` in terms of it (zero behavior change, closes the duplication).

### The five checks, in order, run unconditionally (not fail-fast like `execute_step` — the whole point is maximum diagnostic info from one flash-and-read cycle)

1. **GPIO idle-state** (`CheckIdleGpioState`) — before `SX1280_Create()` touches anything. Hard
   pass/fail against known idle levels from `MX_GPIO_Init()`: NSS=HIGH, NRESET=HIGH, TCXOEN=LOW; BUSY
   printed but not gated here (chip-driven, resolved authoritatively by check 2).
2. **Reset sequence** (`CheckResetSequence`) — calls `device.NRESET_reset()`, asserts `true` (BUSY
   dropped within `BUSY_TIMEOUT`). This is the check that would have caught the "NRESET wedges BUSY"
   fault directly instead of needing hand-built probe mode 8.
3. **Chip-alive / GetStatus** (`CheckChipAlive`) — *the check that would have caught this session's
   actual fault*. Pass requires `hal==HAL_OK` **and** `circuit_mode==STDBY_RC (2)`. Explicitly reject
   `RESERVED_0`/`RESERVED_1` with an actionable message (e.g. pointing at a possible MISO solder-joint
   issue). Print the **raw status byte in hex** always, not just the enum name — `0xFF` decodes to
   circuit_mode bit-pattern `7`, which has no named enum value at all, so an enum-only print would show
   nothing useful for the exact garbage byte this session actually observed. Note: unlike `circuit_mode`,
   `CommandStatus::RESERVED (0x0)` *is* legitimate (both bridges already whitelist it) — don't reject it.
4. **Register write/readback** (`CheckRegisterReadback`) — non-destructive read→write→verify→restore→
   verify on `SX1280_VALUES::REG_LORA_MEM_CLOCK_ENABLE` (0x097F), modeled directly on the already-proven
   RMW dance in `SX1280_Read_Ranging_Result_Cm` (`beacon SX1280Bridge.cpp:404-476`). Write pattern
   `original ^ 0xFF` (guaranteed different), verify, restore, verify again. Independent corroboration via
   a completely different opcode than check 3, in case a MISO fault happens to produce a plausible
   `GetStatus` byte by coincidence.
5. **DIO1/EXTI** (`CheckDio1Irq`) — fully self-contained, no RF partner: `SetStandby`→`SetPacketType(LoRa)`
   →`SetDioIrqParams` routing `IRQ_BIT_RX_TX_TIMEOUT` to DIO1→`SetRx` with a short (~50ms) timeout. The
   chip legitimately times out with nothing on air, firing a real DIO1 rising edge through the actual
   `HAL_GPIO_EXTI_Rising_Callback()`/NVIC path (not a software fake). Poll `*dio1_flag` up to ~200ms,
   clear it, `SetStandby` again. Validates the physical DIO1 wire + EXTI + NVIC end-to-end.

### Per-board glue

Add to each board's `SX1280Bridge.h`/`.cpp` (needs the file-scope `LoRa_SX1280` singleton and, for
anchor, `ANCHOR_RANGING_ADDRESS`):

```c
uint16_t SX1280_BringUp_Run(SPI_HandleTypeDef* SPI_port,
                             GPIO_TypeDef* BUSY_GPIO_port, GPIO_TypeDef* NSS_GPIO_port,
                             GPIO_TypeDef* NRESET_GPIO_port, GPIO_TypeDef* TCXOEN_GPIO_port,
                             uint16_t BUSY_pin, uint16_t NSS_pin, uint16_t NRESET_pin, uint16_t TCXOEN_pin,
                             volatile uint8_t* dio1_flag);
```

Mirrors `SX1280_Create()`'s exact param list plus one `dio1_flag` — `main.c` already has every value on
hand. Internally: run check 1 → `SX1280_Create(...)` → `LoRa_SX1280->NRESET_reset()` again (capturing the
bool this time; `SX1280_Create` discards it) for check 2 → checks 3-5 → (anchor only) print
`ANCHOR_RANGING_ADDRESS` in hex so it can be cross-checked against the board's physical label, per
`PLAN.md`'s own noted concern about mismatched addresses → print the mask summary → return mask.

Bit layout: bit0=GpioIdle, bit1=ResetSequence, bit2=ChipAlive, bit3=RegisterReadback, bit4=Dio1Irq.
`SX1280_BRINGUP_FULL_MASK = 0x1F`.

## Wiring into `main.c` (both boards) — single unified flag

Replace the old `BUSY_ONLY_PROBE` (delete beacon's lines 131-567 entirely — comment block + all 8 modes +
the `#else`/`#endif` wrapping) with one permanent, documented three-way flag, added to **both**
`beacon-blackpill/main.c` and `anchor-blackpill/main.c`:

```c
// Permanent hardware bring-up test. Reflash with the desired mode, run, read UART @ 115200 8N1, reflash
// back to 0 for deployment.
// 0 = normal production <role> logic (default)
// 1 = board-level bring-up: GPIO/UART/SPI3 checks only, module need not be attached. See
//     "Board-level bring-up" section for what to probe/jumper.
// 2 = module-level bring-up: SX1280 chip checks (reset, GetStatus, register RW, DIO1 IRQ). Module must
//     be attached.
#define BRINGUP_MODE 0

#if BRINGUP_MODE == 1
  // board-level checks (Layer 1, described above) -- interactive, runs forever
#elif BRINGUP_MODE == 2
  uint16_t bringup_mask = SX1280_BringUp_Run(&hspi3, BUSY_GPIO_Port, NSS_GPIO_Port, NRESET_GPIO_Port,
                                              TCXOEN_GPIO_Port, BUSY_Pin, NSS_Pin, NRESET_Pin, TCXOEN_Pin,
                                              &DIO1_Callback_detected);
  uint8_t bringup_pass = (bringup_mask == SX1280_BRINGUP_FULL_MASK);
  printf("[%lu] BRING-UP SELF-TEST mask=0x%02X (full=0x%02X) %s\r\n",
         (unsigned long)HAL_GetTick(), bringup_mask, SX1280_BRINGUP_FULL_MASK, bringup_pass ? "PASS" : "FAIL");
  while (1) {
    // Bench-visible pass/fail even with no terminal attached: solid LED = pass, ~5Hz blink = fail.
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin,
                       bringup_pass ? GPIO_PIN_RESET : (((HAL_GetTick() / 200) % 2) ? GPIO_PIN_RESET : GPIO_PIN_SET));
    HAL_Delay(20);
  }
#else
  /* existing production body, unchanged (beacon: SX1280_Create + state machine;
     anchor: SX1280_Create + SX1280_Radio_mode + state machine) */
#endif
```

Anchor currently has no ladder to remove — just wrap its existing production body (today's
lines 148-155-ish) as the new `#else` branch.

## Small consistency fixes

- **`anchor-blackpill/CMakeLists.txt`**: add `DEBUG_PINS` and `DEBUG_PIN_TRACE` to its (currently empty)
  `target_compile_definitions` block, matching beacon, so anchor bring-up runs get the same per-step/
  per-pin tracing when a check fails and finer-grained localization is needed.
- **`SX1280_Get_Status_Raw`**: reimplement in terms of the new `SX1280Device::GetStatus()` (see above).

## Verification

No host-side test runner exists for embedded firmware, so this is bench/UART-transcript verification:

1. **Board-level, known-good board**: `BRINGUP_MODE=1`, flash, open serial @ 115200 8N1, confirm the
   startup banner and toggle/read prints appear; manually confirm each GPIO pin with a multimeter/LED per
   the printed dwell-time prompts; jumper MOSI↔MISO and confirm the loopback prints `MATCH` continuously.
2. **Module-level, known-good board**: `BRINGUP_MODE=2` on a board already proven to work. Confirm all 5
   checks print `OK` and the final line reads `mask=0x1F (full=0x1F) PASS`, LED solid on.
3. **Gold-standard negative test**: if the actual faulty module from this session (cold MISO joint) is
   still available pre-rework, flash `BRINGUP_MODE=2` onto it. Expect check 3 (chip-alive) to fail with
   the implausible-status-byte message, check 4 (register readback) to also fail, check 5 (DIO1) to fail
   too — while checks 1-2 (pure GPIO/reset) may still pass. Re-run after resoldering; confirm it flips to
   `PASS`. This is the most direct confirmation the tool catches the fault it was built for.
4. **BUSY-wedge simulation**: briefly jumper BUSY (PB7) to 3V3 on a known-good board during
   `BRINGUP_MODE=2`; confirm check 2 fails with an explicit BUSY-never-released message while check 1
   still passes. Remove jumper, re-run, confirm clean `PASS`.
5. Run both layers on at least one physical beacon and one physical anchor before treating this as a
   reliable per-role factory gate (anchor additionally exercises the compiled-address printout).

## Critical files

- `common/Sx1280Device/Inc/SX1280Device.hpp`, `Src/SX1280Device.cpp` — add `GetStatus()`
- `common/Sx1280Device/CMakeLists.txt` — add new source file
- `common/Sx1280Device/Inc/SX1280BringUp.hpp` (new), `Src/SX1280BringUp.cpp` (new)
- `beacon-blackpill/Core/Src/main.c` — remove `BUSY_ONLY_PROBE` ladder (current lines 131-567), add
  `BRINGUP_MODE` (both layers)
- `anchor-blackpill/Core/Src/main.c` — add `BRINGUP_MODE` (both layers; no existing ladder to remove)
- `beacon-blackpill/Core/Src/device/SX1280Bridge.{h,cpp}` — add `SX1280_BringUp_Run`, reimplement
  `SX1280_Get_Status_Raw`
- `anchor-blackpill/Core/Src/device/SX1280Bridge.{h,cpp}` — add `SX1280_BringUp_Run`, add
  `SX1280_Get_Status_Raw` (symmetry)
- `anchor-blackpill/CMakeLists.txt` — add `DEBUG_PINS`/`DEBUG_PIN_TRACE`
