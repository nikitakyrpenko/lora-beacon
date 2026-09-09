# LoRa Trilateration with Anchor Auto-Discovery — SX1280 Ranging Engine

## Status

- `.devcontainer/` done — minimal bootstrap from `homework_18`'s proven config (bare-metal ARM toolchain + Python for `host/`), `.clang-format`/`.clangd` kept, comments stripped.
- `anchor/` CubeMX project scaffolded — STM32L476, `CMake` toolchain (not `EWARM`), leaned down to the ~96 files the build actually references (dropped CubeMX's default CMSIS-DSP/NN/RTOS vendoring via the "Copy only the necessary library files" Code Generator setting). Pinout finalized and wired into `anchor.ioc` — see the updated pin table below (moved off `homework_18`'s original SPI2 pins onto SPI3 + a consolidated Morpho set).
- `.vscode/tasks.json` (build/clean/flash/kill-stray-debug, scoped to `anchor/`) and `.vscode/launch.json` (`st-util`+`gdb-multiarch` remote debug) done.
- **Architecture done**: the low-level `SX1280Device` protocol layer (SPI/opcode plumbing, generic across roles) moved into a shared `common/Sx1280Device/` static library, `add_subdirectory`'d from `anchor/CMakeLists.txt` (linked against `stm32cubemx` for HAL includes; `DEBUG_PINS` propagated separately so tracing still works). `SX1280Bridge` (role-specific sequencing) stays per-project. See "`common/Sx1280Device/` — shared library, confirmed scope" below.
- **`SX1280Constants.hpp` fully expanded and cross-checked against the datasheet** — see that section below for the full list. `SPREADING_FACTOR_SF_7` bug fixed, `SF_7_REGISTER_FIXUP` completed, `SetLongPreamble` opcode ambiguity resolved (`0x9B`).
- **Anchor `IDLE`-state init sequence + wake-word validation designed** — see "Power management: idle/wake-up protocol" below for the numbered `enter_idle_full_init()` / `resume_idle_after_rejected_wake()` steps.
- `SX1280_Init()` in `SX1280Bridge.cpp` now implements the full 10-step `enter_idle_full_init()` sequence (`SetStandby` → `SetPacketType` → `SetRfFrequency` → `SetBufferBaseAddress` → `SetModulationParams` → SF7 fixup → `SetPacketParams` → `SetDioIrqParams` → `SetLongPreamble` → `SetRxDutyCycle`), including the chosen first-pass duty-cycle values.
- Still not building / not implemented: `SPI_read` has no body in `SX1280Device.cpp` (declared only), the `RxDone` handler / wake-word check / `IDLE`↔`ARMED` transition logic isn't written yet (only the idle-entry init exists so far), `GET_RX_BUFFER_STATUS_OP_CODE`/`LORA_BEACON_PROTOCOL::WAKE_WORD` not yet added to `SX1280Constants.hpp`, `PB9`/`DIO1` still plain `GPIO_Input` (needs EXTI9). Not yet started: `rover/` project, `host/` script.

## Context

`homework_18` (in `miltech-cpp`) has a working single-link setup: an STM32 Nucleo-L476RG + SX1280 reads an AS5600 angle sensor and streams `Measure` packets over plain LoRa TX, with a UART command interface (`FREQ <hz>`) built on DMA+IDLE-line RX. That project's `Sx1280Device` driver (SPI/opcode plumbing: `send_command`, `BUSY_wait`, `SPI_NSS_begin/end`, `read_register`, `write_buffer`, etc.) is solid, hardware-verified low-level code.

The trilateration idea — using the SX1280's hardware **Ranging Engine** (true time-of-flight) against 3 fixed anchors to compute a rover's 2D position — is its own standalone project (this repo), not a `homework_18` feature. It reuses `homework_18`'s `Sx1280Device` driver as a starting reference/copy-in — not `homework_18` itself, and not its AS5600/angle-telemetry code, which is unrelated to positioning.

Decided scope:
- Trilateration math runs **on a host/PC**, not the STM32 — firmware just performs ranging exchanges and reports raw distances over UART.
- Output is **local X/Y in meters** relative to the anchors' own frame — no GPS lat/lon conversion.
- **2D only** (3+ anchors sufficient; no altitude).
- No AS5600/angle-telemetry carried over — this project is purely positioning, so there's no LoRa-mode-switching conflict to design around.
- **Anchors aren't hardcoded by index** — each anchor is flashed with a distinct Ranging address inside a chosen contiguous block (e.g. `0xA19-0xA30`), and the rover **auto-discovers** which addresses in that block actually have a live anchor, rather than the firmware needing to know "there are exactly 3, at these addresses" ahead of time.
  - This same per-address discovery/addressing also solves radio-channel collision between multiple armed anchors, without needing any anti-collision delay/staggering scheme: ranging exchanges are unicast at the hardware level (each anchor only responds to a `RangingRequestAddress` matching its own flashed `RangingSlaveAddress`, per the SX1280's own address-check logic — `REG_RANGING_ADDR_CHECK_LEN`), and the rover always targets exactly one address at a time, sequentially, whether during a discovery sweep or a steady-state ranging cycle (see "`rover/` sequencing" below). Multiple anchors can sit `ARMED` simultaneously with zero risk of them replying on top of each other, since a non-addressed anchor simply stays silent for a request that isn't its own. The wake-up broadcast itself is the one genuinely simultaneous/unaddressed radio event, but it's rover→anchors one-way (no anchor replies to it), so it doesn't have this problem either.
- **Anchors default to a low-power idle state, not continuous ranging-slave listen** — the rover is mobile (flies/drives through the anchor field) and doesn't need anchors range-ready except while it's nearby. An anchor only arms its ranging engine for a bounded window after hearing a wake-up broadcast from the rover, then reverts to idle on its own. See "Power management: idle/wake-up protocol" below.

This plan follows the same methodology `homework_18/SX1280_DRIVER_PLAN.md` already established: never commit an opcode/register value to code without cross-checking the real datasheet, and verify each SPI step against real hardware before layering the next one on top.

## Project layout

```
lora-beacon/
├── .devcontainer/     # done
├── .vscode/           # done (anchor/ build+flash+debug tasks)
├── common/
│   └── Sx1280Device/  # not yet created: shared static lib (SX1280Device + SX1280Constants),
│                      # add_subdirectory'd from anchor/ and rover/
├── anchor/            # scaffolded: STM32 CubeMX project, ranging Slave, minimal firmware
├── rover/             # not yet created: STM32 CubeMX project, ranging Master + UART reporting
├── host/              # not yet created: Python serial reader + trilateration solver + live display
└── PLAN.md
```

`anchor/` is a much smaller firmware image than `rover/`: no AS5600, no command parser (a bare UART debug print is still useful for bring-up) — just ranging-slave init + loop. `rover/` should bootstrap the same way `anchor/` did: CubeMX generated with `Toolchain/IDE = CMake` (not `EWARM`) and Code Generator set to "Copy only the necessary library files", `cmake/gcc-arm-none-eabi.cmake` toolchain file copied over (`-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard`, `--specs=nano.specs`, `-fno-rtti -fno-exceptions` once C++ is enabled). Instead of copying `SX1280Device.hpp`/`.cpp` into `rover/` too, it `add_subdirectory`s `common/Sx1280Device/` the same way `anchor/` does — one shared source location for the protocol layer, each project still builds it from source into its own firmware image. Toolchain flags already match between the two projects, and `anchor/cmake/stm32cubemx/CMakeLists.txt` (CubeMX-regenerated, never hand-edited) has no references to `device/` sources — all custom wiring lives in the hand-editable top-level `CMakeLists.txt`'s reserved "Add user sources/include paths/linked libraries" sections, which survive `.ioc` regeneration.

## SX1280 ↔ STM32 pinout

Finalized and wired into `anchor.ioc` — consolidated onto fewer Morpho connector rows, moved off `homework_18`'s original SPI2/PB-heavy layout onto SPI3 + a PC/PD-based GPIO set:

| STM32 pin | SX1280 signal | CubeMX mode/signal | Default/idle value |
|---|---|---|---|
| `PC10` | SCK | `SPI3_SCK`, Full-Duplex Master | driven only during a transfer |
| `PC11` | MISO | `SPI3_MISO`, Full-Duplex Master | driven by the SX1280 |
| `PC12` | MOSI | `SPI3_MOSI`, Full-Duplex Master | driven by the STM32 |
| `PD2` | NSS | `GPIO_Output` (software NSS) | idle **HIGH** — deselected |
| `PC6` | NReset | `GPIO_Output` | idle **HIGH** — not held in reset |
| `PB8` | TCXOEN | `GPIO_Output` | idle **LOW** — TCXO disabled until explicitly driven high |
| `PC5` | Busy | `GPIO_Input`, no pull | floating, polled (no EXTI) |
| `PB9` | DIO1 | `GPIO_Input`, no pull | **not yet EXTI-configured** — currently plain input, needs switching to `GPXTI9`/EXTI9 rising-edge (matching `homework_18`'s `PA1`/`GPXTI1` pattern) before interrupt-driven IRQ handling (`TxDone`, ranging results, `RxDone` for the wake-up protocol) can work |

`SPI3` config: Master, Full-Duplex, 8-bit, MSB-first, Mode 0 (`CPOL=Low`/`CPHA=1Edge`), software NSS, prescaler `/8` → **10MHz** (APB1=80MHz).

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

## Power management: idle/wake-up protocol

Motivation: an always-listening ranging-slave anchor keeps its radio (and MCU) fully awake indefinitely, which is wasteful for anchors that only need to respond while the rover happens to be nearby. Instead, anchors spend most of their time in a low-power periodic-RX idle state and only fully arm for a bounded window after a wake-up call.

- **Idle state (anchor default)**: packet type `PACKET_TYPE_LORA` (plain LoRa, not ranging), radio in the SX1280's "sniff mode" periodic duty-cycled RX — `SetRxDutyCycle`, opcode `0x94` (`SET_RX_DUTY_CYCLE_OP_CODE` in `SX1280Constants.hpp`). Params are `[periodBase, rxPeriodBaseCount[15:8], rxPeriodBaseCount[7:0], sleepPeriodBaseCount[15:8], sleepPeriodBaseCount[7:0]]` — **one shared `periodBase`** governs both durations (Rx Duration = `periodBase × rxPeriodBaseCount`, Sleep Duration = `periodBase × sleepPeriodBaseCount`), not a separate rx/sleep period-base pair as earlier drafts of this section assumed. `periodBase` is the same 4-value time-unit enum as `SetTx`/`SetRx` (`PERIOD_BASE_15_625_US`=`0x00` … `PERIOD_BASE_4_MS`=`0x03`). Loop: RX watching for a preamble → if none, Sleep, then repeat; a detected preamble/packet fires `RxDone` and **stops** the loop (drops to `STDBY_RC` — resuming idle sniffing needs `SetLongPreamble`+`SetRxDutyCycle` reissued, see the numbered sequences below). `rxPeriodBaseCount = 0x0000` means "wait indefinitely for a packet," not zero duration — a footgun if misread.
  - **Prerequisite, datasheet-confirmed**: `SetLongPreamble` must be issued *before* `SetRxDutyCycle`. With it enabled, a detected preamble auto-extends the RX window by `SleepPeriod + 2×RxPeriod` — this is what gives the anchor enough time to actually receive the wake-up packet's sync word + payload, not just notice a preamble edge and time out. **Resolved**: opcode is `0x9B` (`SET_LONG_PREAMBLE_OP_CODE`). Both the command's own data-transfer table (11-30) and the master opcode summary table agree on `0x9B` — the `0x98` that appears in §11.5.7's prose is a datasheet typo; `0x98` is actually `SetAutoTx`'s opcode (Table 11-34), confirmed by three separate `SetAutoTx` byte-diagrams elsewhere in the datasheet all showing `Opcode = 0x98`.
- **Wake-up packet**: a short, separate plain-LoRa broadcast from the rover — not a ranging packet, not addressed to any specific anchor. Its only job is "something is listening nearby, arm up" — it does **not** carry the discovery address block; discovery/ranging stays a separate pass that runs after wake-up, using the existing per-address sweep.
- **Wake-word validation (required, not optional)**: `SetRxDutyCycle`'s loop stops on *any* detected packet, not just the rover's — without a payload check, other LoRa traffic sharing 2.45GHz would wake the anchor into a full `RANGING_WINDOW_MS` window for nothing. On `RxDone`, the anchor reads the payload and compares it against a fixed 2-byte magic value (`LORA_BEACON_PROTOCOL::WAKE_WORD`, not yet added to `SX1280Constants.hpp`) — identical compile-time constant in `anchor` and `rover`, pure wake-word discrimination (not authentication, not a per-deployment ID). Combined with LoRa's own CRC (kept enabled for this packet type), 2 bytes gives a comfortably low false-positive rate without adding meaningful airtime to the rover's repeated wake broadcasts. Exact byte value and whether it's wake-word-only vs. wake-word+version-byte are still open.
- **On wake (wake-word matched)**: anchor switches `PACKET_TYPE` to `RANGING`, calls the existing `set_ranging_slave_address()`/ranging init, and opens a full ranging-slave listen for a fixed `RANGING_WINDOW_MS`. This window is armed by a local timer (reuse the `TIM6`-flag pattern already used elsewhere), **not** by waiting for a second "go to sleep" radio message from the rover — a message-based sleep signal would leave the anchor stuck fully awake if the rover flies out of range or the sleep message is lost, whereas a local timeout degrades safely back to idle regardless of what the rover does next.
- **On wake-word mismatch**: discard, resume idle sniffing (`resume_idle_after_rejected_wake()` below) without a full reconfiguration — packet type/frequency/modulation weren't touched by a bare `RxDone`.
- **On window expiry**: anchor switches back to `PACKET_TYPE_LORA` + `SetRxDutyCycle` (full re-init, since `ARMED` changed packet-type-dependent config). No ranging exchanges are attempted after expiry until the next wake-up.
- **Rover side**: before starting a discovery pass or a steady-state ranging cycle, the rover transmits the wake-up broadcast, then proceeds with its normal address sweep. The wake-up's preamble/on-air time needs to be long enough to guarantee at least one anchor duty-cycle wake window overlaps it (standard long-preamble wake-on-radio approach) — exact preamble length is derived from whatever idle `rxPeriodBase`/`sleepPeriodBase` values get chosen, so pick those first, then size the wake-up preamble against them, not the other way around.

### Anchor `IDLE`-state initialization — ordered steps

Two distinct sequences, not one — they differ because `ARMED` changes packet-type-dependent radio configuration that a plain "resume" doesn't need to redo. All names below are the confirmed constants already in `SX1280Constants.hpp` unless noted.

**`enter_idle_full_init()`** — boot, and every return from `ARMED` after a window expiry:
1. `SetStandby(STDBY_RC)` — re-issue for robustness even though `SX1280_Init` already does this once at boot.
2. `SetPacketType(PACKET_TYPE_LORA)`
3. `SetRfFrequency(FREQUENCY_MSB, FREQUENCY_MID, FREQUENCY_LSB)`
4. `SetBufferBaseAddress(txBaseAddress=0x00, rxBaseAddress=0x00)` — needed so `GetRxBufferStatus`'s `rxStartBufferPointer` is a known offset.
5. `SetModulationParams(SPREADING_FACTOR_SF_7, BANDWITH_BW_1600, CHIP_RATE_CR_4_5)`
6. `WriteRegister(REG_SF_MODULATION_FIXUP, SF_7_REGISTER_FIXUP)` — SF7 fixup, must immediately follow step 5. Applies regardless of packet type (LoRa or Ranging) — it's a modulation-block hardware quirk, not packet-type-specific; the datasheet places it in the general LoRa modulation section (§13.4), which the ranging section explicitly says it reuses.
7. `SetPacketParams(LORA_PREAMBLE_12_SYMBOLS, EXPLICIT_HEADER, wakePayloadLength, LORA_CRC_ENABLE, LORA_IQ_STD, 0x00, 0x00)` — CRC **enabled** here (unlike the ranging profile) since this is a plain data packet, not the ranging engine — cheaply rejects bit-corrupted packets before the wake-word comparison even runs.
8. `SetDioIrqParams(irqMask=IRQ_BIT_RX_DONE, dio1Mask=IRQ_BIT_RX_DONE, dio2Mask=0, dio3Mask=0)` — only `RxDone` routed to `DIO1` while idle.
9. `SetLongPreamble(enable=0x01)` — required immediately before step 10 per the datasheet's own "Notice!" (§11.5.6).
10. `SetRxDutyCycle(PERIOD_BASE_1_MS, ANCHOR_IDLE_RX_PERIOD_BASE_COUNT, ANCHOR_IDLE_SLEEP_PERIOD_BASE_COUNT)` — first-pass values chosen (10ms RX / 490ms sleep), not yet hardware-validated, see Open Items.

**On `RxDone`** (before deciding full-init vs. resume):
1. `ClearIrqStatus(all)`
2. `GetRxBufferStatus` — opcode `0x17`, **not yet added** to `SX1280Constants.hpp` → `rxPayloadLength`, `rxStartBufferPointer`
3. `ReadBuffer(rxStartBufferPointer, rxPayloadLength)` → payload bytes
4. Compare against `LORA_BEACON_PROTOCOL::WAKE_WORD` → branch to the `ARMED` entry sequence, or to `resume_idle_after_rejected_wake()`

**`resume_idle_after_rejected_wake()`** — `RxDone` fired but the wake-word didn't match:
1. `SetLongPreamble(enable=0x01)`
2. `SetRxDutyCycle(...)` (same params as step 10 above)

- **Open items to resolve before implementation**:
  - `SetRxDutyCycle` first-pass values **chosen, not yet hardware-validated**: `PERIOD_BASE_1_MS`, `ANCHOR_IDLE_RX_PERIOD_BASE_COUNT = 10` (10ms RX), `ANCHOR_IDLE_SLEEP_PERIOD_BASE_COUNT = 490` (490ms sleep) — 500ms cycle, ~2% RX duty cycle. Consequence: the rover's wake broadcast needs to stay on-air ≥500ms to guarantee landing in some anchor's RX window regardless of phase. Tune once wake latency / power draw can actually be measured on hardware.
  - **`RANGING_WINDOW_MS`**: starts as a **fixed, shared compile-time constant** on both anchor and rover for initial bring-up (no value chosen yet). Once anchor+rover have been tested together and basic wake/range exchanges work on real hardware, plan is to switch this to a **runtime value the rover sends inside the wake-up payload** instead (anchor arms its `ARMED`-window timer from the received value rather than its own constant) — deliberately deferred past initial bring-up rather than built first, to avoid taking on the extra payload-parsing/clamping complexity before the basic link is even proven. Since that value would arrive over radio (untrusted -- `RxDone` doesn't guarantee a valid packet), the anchor must clamp it to a sane `[min, max]` range before arming any timer with it once this lands.
  - Wake-word value **resolved**: `LORA_BEACON_PROTOCOL::WAKE_WORD = {0xBE, 0xAC}`, wake-word-only payload (2 bytes) for now -- see `RANGING_WINDOW_MS` item above for the planned later extension to also carry a duration field.
  - Decide whether the rover repeats the wake-up broadcast more than once per approach (e.g. to cover the case where its first transmission lands between an anchor's duty-cycle wake windows) or relies on a single long-preamble send.
  - `PB9`/`DIO1` in `anchor.ioc` is still plain `GPIO_Input` — needs switching to `GPXTI9`/EXTI9 rising-edge before any of this IRQ-driven flow can run.

## `common/Sx1280Device/` — shared library, confirmed scope

Moves out of `anchor/Core/{Inc,Src}/device/` into `common/Sx1280Device/{Inc,Src}/`, `add_subdirectory`'d from both `anchor/` and (once created) `rover/`. `SX1280Bridge` (role-specific sequencing) is **not** shared — anchor and rover have genuinely different control flow (idle-duty-cycle+wake-detect vs. discovery-sweep+ranging-master), so a shared bridge would just mean `#ifdef` branching.

**No new `SX1280Device` methods needed beyond finishing what's already declared.** `SPI_write`/`SPI_read` are opcode-agnostic — `WriteRegister`/`WriteBuffer`/`ReadRegister`/`ReadBuffer`/`GetIrqStatus`/`ClearIrqStatus` are just specific opcodes (`0x18`/`0x1A`/`0x19`/`0x1B`/`0x15`/`0x97`) with a caller-assembled params buffer, not separate operations — each `SX1280Bridge` builds that buffer from `SX1280Constants.hpp` values and calls `SPI_write`/`SPI_read` directly, the same way `SX1280_Init` already does for `SetStandby`. Concretely:

- **Implement `SPI_read`** — declared in `SX1280Device.hpp`, no body in the `.cpp` yet. Blocks `GetStatus`, `ReadRegister`, `ReadBuffer`, `GetIrqStatus`, and the ranging-result readback dance below.
- Optional/deferred: a small `constexpr` free-function helper in `SX1280Constants.hpp` (not a class method) for packing the register-address/buffer-offset prefix, so `anchor/`'s and `rover/`'s bridges don't each re-derive that packing independently.

### `SX1280Constants.hpp` expansion — done

**Bug fixed**: `SPREADING_FACTOR_SF_7` was `0x7A`, now `0x70` (Table 13-47). `BANDWITH_BW_1600 = 0x0A` and `CHIP_RATE_CR_4_5 = 0x01` were already correct. `SF_7_REGISTER_FIXUP = 0x37` completed.

Opcodes added: `SET_RX_OP_CODE = 0x82`, `SET_RANGING_ROLE_OP_CODE = 0xA3`, `SET_RX_DUTY_CYCLE_OP_CODE = 0x94`, `SET_LONG_PREAMBLE_OP_CODE = 0x9B` (**resolved** — datasheet prose's "`0x98`" was a typo; that opcode is actually `SetAutoTx`'s, confirmed via three separate `SetAutoTx` byte-diagrams; both `SetLongPreamble`'s own data-transfer table and the command summary table agree on `0x9B`).

Registers added: `REG_SF_MODULATION_FIXUP = 0x925`, `REG_RANGING_MASTER_TARGET_ADDR = 0x912` (4B, rover), `REG_RANGING_SLAVE_OWN_ADDR = 0x916` (4B, anchor), `REG_RANGING_ADDR_CHECK_LEN = 0x931`, `REG_RANGING_CALIBRATION = 0x92C` (2B), `REG_RANGING_RESULT_MUX = 0x924`, `REG_RANGING_RESULT_MSB = 0x961` (3B, rover), `REG_LORA_MEM_CLOCK_ENABLE = 0x97F`.

Param values added: `PACKET_TYPE_LORA = 0x01` (alongside renamed `PACKET_TYPE_RANGING = 0x02`), `SetTxParams` power/ramp (`TX_OUTPUT_POWER = 13`, `RADIO_RAMP_04_US = 0x20`), `SetPacketParams` field values (preamble/header/CRC/IQ), shared `PERIOD_BASE_*` enum, IRQ bit constants, `RANGING_RESULT_*` mux values + `RANGING_RESULT_TO_CM_MULTIPLIER = 20`.

**Still missing** (needed for the anchor `IDLE` sequence above, not added yet): `GET_RX_BUFFER_STATUS_OP_CODE = 0x17`, `LORA_BEACON_PROTOCOL::WAKE_WORD` (new namespace, not an SX1280 hardware fact so doesn't belong alongside the datasheet-sourced constants — 2 bytes recommended, exact value not chosen).

Ranging results are read back and reported as fixed-point **centimeters (`int32_t`)**, not `float` — sidesteps embedded float-formatting fragility over UART; convert to meters only on the host. The debiased/average/filtered conversion reduces to an exact integer: `distance_cm = RangingResult * 20` (no rounding loss).

A **finite `SetTx`/`SetRx` timeout** (not `homework_18`'s hardcoded continuous `periodBaseCount = 0x0000`) is needed throughout — discovery sweeps especially rely on a short timeout so an address with no listening anchor actually completes via `RangingMasterTimeout` instead of hanging; steady-state ranging against known-good addresses can use a longer one.

## `anchor/` main loop — idle by default, ranging only inside a wake window

Same flag-in-ISR/work-in-main-loop pattern as `rover/`, driven off `DIO1_callback_triggered` (radio IRQ) and a `TIM6`-style local timer for the ranging-window timeout:

- **`IDLE`** (default, on boot and after every window expiry): `enter_idle_full_init()` (10-step sequence, "Power management" section above). On `RxDone`, the payload is read and checked against the wake-word before trusting it — a match switches to `RANGING`, re-arms `set_ranging_slave_address()`, starts the `RANGING_WINDOW_MS` timer, transitions to `ARMED`; a mismatch calls `resume_idle_after_rejected_wake()` instead of the full 10-step init.
- **`ARMED`**: normal ranging-slave listen (as already planned, unchanged) — responds to the rover's per-address ranging requests. On timer expiry → back to `IDLE` via `enter_idle_full_init()`, regardless of how many (if any) ranging exchanges happened during the window.

## `rover/` sequencing (new bridge, e.g. `Sx1280RangingBridge.cpp`) — wake, then discover once, then range fast

Every pass (discovery or steady-state) is prefixed by a wake-up broadcast so anchors are actually listening before the address sweep starts:

**0. Wake-up** — `send_wakeup_broadcast()`, then a short fixed delay before starting the sweep below (budget derived from the anchors' idle duty-cycle settings, see "Power management" above — not yet chosen).

Two sweep states, both built from the same flag-in-ISR/work-in-main-loop pattern `homework_18` already established (`TIM6_callback_triggered`/`DIO1_callback_triggered`):

**1. Discovery pass** — runs at startup, and re-runs on demand (see the UART command below) or on a slow periodic cadence (e.g. once every N steady-state cycles) to notice anchors being added/removed/power-cycled:
- Iterate every address in the configured block (e.g. `0xA19..0xA30`): `set_ranging_master_target(addr)` → `set_tx(short_timeout)` → wait for `DIO1_callback_triggered`, check whether the IRQ was `RangingMasterResultValid` (anchor present at this address — record it) or `RangingMasterTimeout` (nothing there — skip, move to next address).
- Result: a small in-memory list of *discovered* addresses (out of the whole block), built once per discovery pass rather than assumed at compile time.
- Short per-address timeout here matters — sweeping ~24 addresses at a generous timeout would make discovery itself slow; a short timeout is fine since a real anchor should respond quickly, and failing fast through empty addresses keeps the sweep bounded.

**2. Steady-state ranging loop** — cycles only through the addresses the last discovery pass actually found (like a fixed list, until the next re-discovery):
- Driven off a periodic timer tick (reuse `TIM6`'s flag pattern), but the unit of work per tick is now a **whole cycle through the discovered list**, not one address — since anchors drop back to `IDLE` after their own `RANGING_WINDOW_MS`, every cycle needs its own wake-up (step 0 above) before ranging any address, not just once at startup.
- Within a cycle: `set_ranging_master_target()` the next discovered address, `set_tx(normal_timeout)`, `DIO1_callback_triggered` checks which IRQ fired before reading+converting the result and advancing to the next discovered address — same as before, just bounded to fit inside one anchor-side ranging window (`sum(per-address timeout) < RANGING_WINDOW_MS`, another reason to pick those two numbers together, see "Power management" above).
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
- **Week 1**: move `Sx1280Device` into `common/Sx1280Device/`, wire `anchor/` (and newly-bootstrapped `rover/`) to it via `add_subdirectory`, expand `SX1280Constants.hpp`, implement `SPI_read`; verify one rover↔one-anchor exchange on real hardware — uncalibrated first (does the exchange complete?), then calibrated, checked against 2-3 tape-measured known distances (expect ~±1m-class accuracy from this hardware/SF/BW class — set that expectation early).
- **Week 2**: implement the discovery pass (address-block sweep, short timeout) + steady-state ranging loop over the discovered set; verify with 3 physical anchors that discovery finds exactly the right addresses and steady-state ranging reports distinct, plausible distances per cycle. Also implement the idle/wake-up protocol (`set_rx_duty_cycle`, wake-up broadcast, `IDLE`/`ARMED` anchor states) once basic ranging works — verify an anchor left idle actually wakes and completes a ranging exchange within the window, and drops back to idle on timeout.
- **Week 3**: `Uart_SendRange`/`Uart_SendDiscovered` + host script skeleton (parse-and-print only, no math yet) — confirms the UART link carries the discovered set and all ranges correctly over a full cycle.
- **Week 4**: trilateration math + live position display; acceptance test — place rover at 2-3 known ground-truth positions, compare computed vs actual X/Y, quantify error.
- **Week 5 (buffer)**: accuracy tuning if needed (try `filtered` result type, tune SF/preamble, re-derive calibration, host-side outlier rejection), plus idle/wake timing tuning (duty-cycle period, `RANGING_WINDOW_MS`) if power draw or wake latency isn't acceptable. Cut this phase first if earlier ones slip — a working, roughly-accurate demo is the real bar, not centimeter precision.

## Critical files

- `common/Sx1280Device/CMakeLists.txt`, `Inc/SX1280Device.hpp`, `Inc/SX1280Constants.hpp`, `Src/SX1280Device.cpp` — shared protocol layer, moved out of `anchor/`, `SPI_read` implemented, constants expanded per above
- `anchor/CMakeLists.txt` — `add_subdirectory`s `common/Sx1280Device`, links against it
- `anchor/Core/Inc/device/SX1280Bridge.h`, `Core/Src/device/SX1280Bridge.cpp` — anchor-only, grows into the IDLE/ARMED state machine
- `rover/` (not yet created) — bootstraps the same way `anchor/` did, `add_subdirectory`s `common/Sx1280Device` the same way, plus its own `Sx1280RangingBridge.cpp`/`UartBridge.cpp` (ported from `homework_18`), `main.c`
- `host/` — Python trilateration script
- `/home/mickaborscha/miltech/miltech-cpp/homework_18/` — source for bootstrapped devcontainer/toolchain/driver files (read-only reference, not modified)
- `/home/mickaborscha/sx1280.txt` — required reference for calibration research and every opcode/register value before committing to code

## Verification

- Each phase has its own hardware-verification gate (above) — confirm each SPI/driver step against real hardware (UART/LED debug output, `GetIrqStatus` polling before trusting interrupts) before building the next layer on top, not write-the-whole-stack-then-debug.
- Final acceptance test (end of Phase 4): rover placed at known ground-truth positions in the anchor frame, host script's computed X/Y compared against tape-measured actual position, error quantified in meters.
