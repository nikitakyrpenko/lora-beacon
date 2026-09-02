# LoRa Trilateration with Anchor Auto-Discovery — SX1280 Ranging Engine

## Status

- `.devcontainer/` done — minimal bootstrap from `homework_18`'s proven config (bare-metal ARM toolchain + Python for `host/`), `.clang-format`/`.clangd` kept, comments stripped.
- `anchor/` CubeMX project scaffolded — STM32L476, `CMake` toolchain (not `EWARM`), leaned down to the ~96 files the build actually references (dropped CubeMX's default CMSIS-DSP/NN/RTOS vendoring via the "Copy only the necessary library files" Code Generator setting).
- `.vscode/tasks.json` (build/clean/flash/kill-stray-debug, scoped to `anchor/`) and `.vscode/launch.json` (`st-util`+`gdb-multiarch` remote debug) done.
- Not yet started: `rover/` project, `Sx1280Device` ranging API additions, `host/` script.

## Context

`homework_18` (in `miltech-cpp`) has a working single-link setup: an STM32 Nucleo-L476RG + SX1280 reads an AS5600 angle sensor and streams `Measure` packets over plain LoRa TX, with a UART command interface (`FREQ <hz>`) built on DMA+IDLE-line RX. That project's `Sx1280Device` driver (SPI/opcode plumbing: `send_command`, `BUSY_wait`, `SPI_NSS_begin/end`, `read_register`, `write_buffer`, etc.) is solid, hardware-verified low-level code.

The trilateration idea — using the SX1280's hardware **Ranging Engine** (true time-of-flight) against 3 fixed anchors to compute a rover's 2D position — is its own standalone project (this repo), not a `homework_18` feature. It reuses `homework_18`'s `Sx1280Device` driver as a starting reference/copy-in — not `homework_18` itself, and not its AS5600/angle-telemetry code, which is unrelated to positioning.

Decided scope:
- Trilateration math runs **on a host/PC**, not the STM32 — firmware just performs ranging exchanges and reports raw distances over UART.
- Output is **local X/Y in meters** relative to the anchors' own frame — no GPS lat/lon conversion.
- **2D only** (3+ anchors sufficient; no altitude).
- No AS5600/angle-telemetry carried over — this project is purely positioning, so there's no LoRa-mode-switching conflict to design around.
- **Anchors aren't hardcoded by index** — each anchor is flashed with a distinct Ranging address inside a chosen contiguous block (e.g. `0xA19-0xA30`), and the rover **auto-discovers** which addresses in that block actually have a live anchor, rather than the firmware needing to know "there are exactly 3, at these addresses" ahead of time.

This plan follows the same methodology `homework_18/SX1280_DRIVER_PLAN.md` already established: never commit an opcode/register value to code without cross-checking the real datasheet, and verify each SPI step against real hardware before layering the next one on top.

## Project layout

```
lora-beacon/
├── .devcontainer/     # done
├── .vscode/           # done (anchor/ build+flash+debug tasks)
├── anchor/            # scaffolded: STM32 CubeMX project, ranging Slave, minimal firmware
├── rover/             # not yet created: STM32 CubeMX project, ranging Master + UART reporting
├── host/              # not yet created: Python serial reader + trilateration solver + live display
└── PLAN.md
```

`anchor/` is a much smaller firmware image than `rover/`: no AS5600, no command parser (a bare UART debug print is still useful for bring-up) — just ranging-slave init + loop. `rover/` should bootstrap the same way `anchor/` did: CubeMX generated with `Toolchain/IDE = CMake` (not `EWARM`) and Code Generator set to "Copy only the necessary library files", `cmake/gcc-arm-none-eabi.cmake` toolchain file copied over (`-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard`, `--specs=nano.specs`, `-fno-rtti -fno-exceptions` once C++ is enabled), and `Sx1280Device.hpp`/`.cpp` copied in as the starting SPI/opcode driver.

## SX1280 ↔ STM32 pinout (reference, from `homework_18`'s hardware-verified wiring)

`anchor.ioc` doesn't have any of this configured yet — it's currently just a base Nucleo-L476RG project (USART2 VCP, user button, `LD2`). This table is the known-working reference pinout from `homework_18` to wire `anchor/`'s (and later `rover/`'s) SX1280 module against, before the planned pinout rework to consolidate pins onto fewer Morpho connector rows.

| STM32 pin | SX1280 signal | CubeMX mode/signal | Default/idle value |
|---|---|---|---|
| `PB10` | SCK | `SPI2_SCK`, Full-Duplex Master | driven only during a transfer |
| `PC2` | MISO | `SPI2_MISO`, Full-Duplex Master | driven by the SX1280 |
| `PC3` | MOSI | `SPI2_MOSI`, Full-Duplex Master | driven by the STM32 |
| `PB14` | NSS | `GPIO_Output` (software NSS) | idle **HIGH** (`GPIO_PIN_SET`) — deselected |
| `PB0` | NReset | `GPIO_Output` | idle **HIGH** (`GPIO_PIN_SET`) — not held in reset |
| `PB13` | TCXOEN | `GPIO_Output` | idle **LOW** (`GPIO_PIN_RESET`, no `PinState` override) — TCXO disabled until explicitly driven high |
| `PB1` | Busy | `GPIO_Input`, no pull | floating, polled (no EXTI) |
| `PA1` | DIO1 | `GPXTI1`, EXTI1 rising-edge, pull-down | idle **LOW** via pull-down; pulses high on `TxDone`/ranging IRQ |

`SPI2` config: Master, Full-Duplex, 8-bit, MSB-first, Mode 0 (`CPOL=Low`/`CPHA=1Edge`), software NSS, prescaler `/8` → **10MHz** (APB1=80MHz).

## Hardware prerequisite (blocks Phase 1)

At least 3 physical anchor boards, each with an SX1280 module, each flashed from `anchor/` with a **distinct Ranging address drawn from the chosen discovery block** (e.g. `0xA19-0xA30`) and a known, tape-measured physical position recorded against that specific address (not a fixed "anchor 1/2/3" index — the host config maps `address → (x,y)`, since discovery order isn't guaranteed). Anchor placement should be roughly triangular and well-spread (not near-collinear) — trilateration accuracy is bounded by how well anchor positions are actually known, independent of code quality.

The discovery block should be sized a bit larger than the number of anchors actually deployed (e.g. a ~24-address block for 3-4 physical anchors) so boards can be added/swapped later without picking new addresses outside the block, but not so large that a full discovery sweep (one ranging attempt + timeout per address) takes unreasonably long — see the sequencing section below.

One option to evaluate in week 1, not decide now: the existing ESP32-S3 (`~/miltech/lora_rx`, RadioLib-based) supports SX128x ranging natively and could serve as one of the anchors, reducing new STM32 hardware needed. Verify STM32-to-STM32 ranging works first before betting on RadioLib interop — that's a second, separate risk.

## Datasheet-confirmed facts to build from (`/home/mickaborscha/sx1280.txt`, Semtech Rev 1.1 extraction)

- `PACKET_TYPE_RANGING = 0x02` (vs `0x01` for plain LoRa), set via the existing `SET_PACKET_TYPE_OP_CODE`.
- Ranging modulation params are restricted vs plain LoRa: SF5-SF10 only (no SF11/12), BW ∈ {406.25, 812.5, 1625} kHz — **203.125kHz (used in `homework_18`'s plain LoRa) is not legal for ranging**. SF/BW/CR are freely combinable (Table 13-55 lists them as independent columns).
- Addressing (how ranging avoids cross-talk on a shared frequency — not RSSI/broadcast): a 32-bit Ranging ID in the packet header.
  - **Slave** (`anchor/`): `WriteRegister` its own address into `0x916-0x919` (`RangingRangingAddress[31:24..7:0]`), plus how many LSBs to check via `0x931[7:6]` (0x0=8bit … 0x3=32bit).
  - **Master** (`rover/`): `WriteRegister` the *target* slave's address into `0x912-0x915` (`RangingRequestAddress[...]`) — rewritten between exchanges to cycle through discovered anchors.
- Role: `SetRangingRole(role)`, opcode **`0xA3`**, `0x01`=Master / `0x00`=Slave — explicit per device, not dynamic.
- `SetRx` opcode **`0x82`**, params `[periodBase, periodBaseCount[15:8], periodBaseCount[7:0]]` — same shape as the already-implemented `set_tx()` (`0x83`). Not yet implemented anywhere in the existing `Sx1280Device` (RX so far only exists on the sibling ESP32/RadioLib project).
- Calibration: `WriteRegister` a 16-bit value into `0x92C`/`0x92D` (RxTx delay offset). **Open research item**: this datasheet revision gives only the register addresses, not per-SF/BW numeric values. Plan: check RadioLib's SX128x source (already used by the sibling ESP32 project) for hardcoded calibration constants as a cross-reference (verify before trusting, don't copy blind); if none found, use **empirical calibration** — range against one anchor at a precisely known distance (e.g. 1.000m), compute the offset from the raw reading, write it back. Do this in Phase 0, before Phase 1 firmware work.
- IRQ bits (`SetDioIrqParams`, same opcode as `homework_18`'s existing `TxDone` wiring): bit7=`RangingSlaveResponseDone`, bit8=`RangingSlaveRequestDiscard`, bit9=`RangingMasterResultValid`, bit10=`RangingMasterTimeout`, bit11=`RangingMasterRequestValid` (slave-side). Master routes bit9+bit10 onto `DIO1` — same EXTI-flag pattern `homework_18` already established for `TxDone`.
- Reading a result (master-only, after `RangingMasterResultValid`) is a 4-step dance, not a plain register read: `SetStandby(STDBY_XOSC)` → `WriteRegister(0x97F, ReadRegister(0x97F)|(1<<1))` (enable LoRa memory clock) → set `RangingResMux` bits `0x924[5:4]` to select result type (00=raw, 01=average, 10=debiased, 11=filtered — **use debiased or filtered, both non-negative**) → `ReadRegister` the 3 bytes at `0x961/0x962/0x963` → `SetStandby(STDBY_RC)`. Distance: debiased/filtered → `meters = RangingResult / 5.0`; raw → `meters = RangingResult * 150 / (2^12 * BW_MHz)`.

## `Sx1280Device` additions (both `rover/` and `anchor/` copies, reuse existing `send_command`/pointer-based style)

New opcode: `SET_RANGING_ROLE_OP_CODE = 0xA3` in the anonymous `Sx1280_OPCODE` namespace.

New public methods (mirror existing `init()`/`set_frequency()`/`set_tx()` shape):
```cpp
bool init_ranging();                                     // Standby(RC) -> PacketType(RANGING) ->
                                                           // SetModulationParams(ranging-legal SF/BW/CR) ->
                                                           // SetPacketParams -> SetRfFrequency -> SetTxParams
bool set_ranging_role(bool is_master);                    // opcode 0xA3
bool set_ranging_calibration(uint16_t cal_value);         // WriteRegister 0x92C/0x92D
bool set_ranging_master_target(uint32_t slave_addr);      // WriteRegister 0x912-0x915 (rover only)
bool set_ranging_slave_address(uint32_t own_addr,
                                uint8_t check_len_bits);  // WriteRegister 0x916-0x919 + 0x931 (anchor only)
bool set_rx(bool (*verify)(Status) = &check_processed);   // opcode 0x82, mirrors existing set_tx()
bool read_ranging_result(uint8_t result_type,
                          int32_t* out_cm);                // the 4-step Standby/register dance above (rover only)
```

`homework_18`'s existing `set_tx()` hardcodes `periodBaseCount = 0x0000` (continuous/no-timeout, fine for a single always-on plain-LoRa link). Ranging — and discovery sweeps especially — need a **real, finite timeout** so a `SetTx` against an address with no listening anchor actually completes (via `RangingMasterTimeout`) instead of hanging: add a `periodBaseCount` parameter to `set_tx()` (or a ranging-specific overload), short during discovery (fail fast through empty addresses) and longer during steady-state ranging against known-good addresses (reduce spurious timeouts on a real anchor that's just slow to respond).

Keep `init_ranging()` as a **separate** ordered `send_command` chain, not a parametrized merge with a shared `init()` — matches how `homework_18` already treated BLE vs LoRa as parallel, not merged, init sequences.

`read_ranging_result` returns fixed-point **centimeters as `int32_t`**, not `float` — sidesteps embedded float-formatting fragility over UART; convert to meters only on the host.

## `rover/` sequencing (new bridge, e.g. `Sx1280RangingBridge.cpp`) — discover once, then range fast

Two states, both built from the same flag-in-ISR/work-in-main-loop pattern `homework_18` already established (`TIM6_callback_triggered`/`DIO1_callback_triggered`):

**1. Discovery pass** — runs at startup, and re-runs on demand (see the UART command below) or on a slow periodic cadence (e.g. once every N steady-state cycles) to notice anchors being added/removed/power-cycled:
- Iterate every address in the configured block (e.g. `0xA19..0xA30`): `set_ranging_master_target(addr)` → `set_tx(short_timeout)` → wait for `DIO1_callback_triggered`, check whether the IRQ was `RangingMasterResultValid` (anchor present at this address — record it) or `RangingMasterTimeout` (nothing there — skip, move to next address).
- Result: a small in-memory list of *discovered* addresses (out of the whole block), built once per discovery pass rather than assumed at compile time.
- Short per-address timeout here matters — sweeping ~24 addresses at a generous timeout would make discovery itself slow; a short timeout is fine since a real anchor should respond quickly, and failing fast through empty addresses keeps the sweep bounded.

**2. Steady-state ranging loop** — cycles only through the addresses the last discovery pass actually found (like a fixed list, until the next re-discovery):
- Ranging state (`IDLE` / `WAIT_DISCOVERED[i]`) tracked in the bridge, driven off a periodic timer tick (reuse `TIM6`'s flag pattern): each tick, `set_ranging_master_target()` the next discovered address, `set_tx(normal_timeout)`, and the `DIO1_callback_triggered` handler checks which IRQ fired before reading+converting the result and advancing to the next discovered address.
- This is the fast path — no wasted cycles on addresses known not to have an anchor, unlike sweeping the full block every time.

No AS5600/telemetry-mode-switching concern here — this rover is purpose-built for ranging only, so there's no `PACKET_TYPE_LORA`↔`PACKET_TYPE_RANGING` switching cost to design around (that complexity only existed if this were bolted onto `homework_18`).

## UART reporting protocol (new, small `UartBridge` port from `homework_18`'s pattern)

Reports are keyed by the **actual 32-bit Ranging address**, not a fixed 1-3 index, since discovery order/count isn't guaranteed:

- `Uart_SendRange(uint32_t addr, int32_t distance_cm)`, format `RANGE <addr_hex> <distance_m>\r\n` (print the fixed-point cm value as `%d.%02d`), reusing `homework_18`'s proven `snprintf`-into-stack-buffer + blocking `HAL_UART_Transmit` pattern.
- `Uart_SendDiscovered(...)`, format `ANCHORS <addr1_hex> <addr2_hex> ...\r\n` announced once after each discovery pass completes — lets the host know the current discovered set (and notice if it changed) without having to infer it from which `RANGE` addresses show up over time.
- A `sscanf`-based command parser (DMA+IDLE-line RX, same as `homework_18`'s `Uart_PollCommand`) for a `DISCOVER` command to trigger an on-demand re-discovery pass from the host (e.g. after physically adding/moving an anchor), in addition to whatever periodic auto-re-discovery cadence the firmware runs on its own.

## `host/` trilateration script (Python)

- `pyserial`, reading `RANGE <addr_hex> <dist>` and `ANCHORS <addr_hex>...` lines off the rover's VCP (`/dev/ttyACM0`, 115200 8N1, same as `screen /dev/ttyACM0 115200`).
- Anchor config is a `dict[address_hex] -> (x, y)`, set up once when anchors are physically placed and labeled with their flashed address (e.g. a sticker on each board). Incoming `RANGE` lines are looked up by address against this config; an address that shows up in `ANCHORS`/`RANGE` but isn't in the config is a bring-up mismatch worth surfacing (e.g. an anchor flashed with the wrong address, or a config entry not yet added) rather than silently ignoring it.
- Trilateration: linearized least-squares (subtract circle equation N from equations 1..N-1 to cancel the quadratic term, solve via `numpy.linalg.lstsq`) — robust to range noise, unlike an exact 3-equation solve, and **naturally generalizes to however many anchors the current discovery pass actually found** (3 minimum, more if available — an overdetermined least-squares fit from 4+ anchors should be more accurate than exactly 3, worth using if the deployment has spare anchors).
- Live display: a periodically-redrawn matplotlib scatter (anchors fixed, rover moving) is enough for a course-project demo.

## Phasing (1-5 week budget)

- **Phase 0 (~1 day)**: bootstrap repo + devcontainer from `homework_18`'s proven config *(done)*; resolve calibration research (RadioLib cross-check or plan the empirical procedure); prototype the least-squares solver standalone against synthetic data.
- **Week 1**: copy `Sx1280Device` into `rover/`+`anchor/`, add ranging API; verify one rover↔one-anchor exchange on real hardware — uncalibrated first (does the exchange complete?), then calibrated, checked against 2-3 tape-measured known distances (expect ~±1m-class accuracy from this hardware/SF/BW class — set that expectation early).
- **Week 2**: implement the discovery pass (address-block sweep, short timeout) + steady-state ranging loop over the discovered set; verify with 3 physical anchors that discovery finds exactly the right addresses and steady-state ranging reports distinct, plausible distances per cycle.
- **Week 3**: `Uart_SendRange`/`Uart_SendDiscovered` + host script skeleton (parse-and-print only, no math yet) — confirms the UART link carries the discovered set and all ranges correctly over a full cycle.
- **Week 4**: trilateration math + live position display; acceptance test — place rover at 2-3 known ground-truth positions, compare computed vs actual X/Y, quantify error.
- **Week 5 (buffer)**: accuracy tuning if needed (try `filtered` result type, tune SF/preamble, re-derive calibration, host-side outlier rejection). Cut this phase first if earlier ones slip — a working, roughly-accurate demo is the real bar, not centimeter precision.

## Critical files

- `rover/Core/Inc/Sx1280Device.hpp`, `Core/Src/Sx1280Device.cpp` — copied from `homework_18`, extended with ranging API
- `rover/` — new `Sx1280RangingBridge.cpp`, `UartBridge.cpp` (ported from `homework_18`), `main.c`
- `anchor/` — minimal CubeMX project reusing the same `Sx1280Device`
- `host/` — Python trilateration script
- `/home/mickaborscha/miltech/miltech-cpp/homework_18/` — source for bootstrapped devcontainer/toolchain/driver files (read-only reference, not modified)
- `/home/mickaborscha/sx1280.txt` — required reference for calibration research before Phase 1 code

## Verification

- Each phase has its own hardware-verification gate (above) — confirm each SPI/driver step against real hardware (UART/LED debug output, `GetIrqStatus` polling before trusting interrupts) before building the next layer on top, not write-the-whole-stack-then-debug.
- Final acceptance test (end of Phase 4): rover placed at known ground-truth positions in the anchor frame, host script's computed X/Y compared against tape-measured actual position, error quantified in meters.
