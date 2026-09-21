# lora-beacon

SX1280 LoRa ranging: a beacon wakes anchors, collects their positions and ranges each one; STM32H523 firmware.

- [PROTOCOL.md](PROTOCOL.md) - radio cycle and UART frame
- [BRING_UP.md](BRING_UP.md) - bring-up modes

## Hardware
| Part | Used |
|---|---|
| MCU | STM32H523CETx, Blackpill-style board |
| Radio | SX1280 module with TCXO (enabled through TCXOEN) |

## Interface parameters
| | SPI3 | USART1 |
|---|---|---|
| Role | master, full duplex | TX + RX, no flow control |
| Format | 8-bit, MSB first, mode 0 (CPOL low, CPHA 1st edge), software NSS | 8 data bits, no parity, 1 stop bit (8N1) |
| Speed | prescaler 8 | 115200 baud |
| Levels | 3.3 V | 3.3 V TTL, idle high, not inverted (start bit low) |

> **Important: chip select is software.** NSS (PA7) is a plain GPIO driven by firmware, not the SPI peripheral's hardware NSS. Wait for BUSY low, pull NSS low, transfer, then NSS high.

## Pin configuration
| Signal | GPIO mode | Idle | Active |
|---|---|---|---|
| NSS (software CS) | output push-pull | high | low (selected) |
| NRESET | output push-pull | high | low (reset) |
| TCXOEN | output push-pull | low | high (TCXO on) |
| BUSY | input, no pull | low | high (chip busy) |
| DIO1 | input, EXTI rising edge, no pull | low | high (interrupt) |
| LED | output push-pull | high (off) | low (on) |
| SPI3 MOSI / MISO / SCK | alternate function (AF4 / AF5 / AF4), no pull | - | - |
| USART1 TX / RX | alternate function (AF4) | high | low (start bit) |

## Initialization
Runs once in the bridge constructor (`NRESET_reset()`), before any SPI traffic.

| Step | Action | Wait |
|---|---|---|
| 0 | Pins at idle: NSS high, NRESET high, TCXOEN low | - |
| 1 | TCXOEN high (TCXO on), skipped if already high | 3 ms warm-up |
| 2 | NRESET low | 2 ms hold |
| 3 | NRESET high | BUSY must go low, 10 ms timeout |
| 4 | `to_radio()`: standby RC, LoRa packet type, frequency, buffer base, SF7 / BW1600 / CR 4/5, SF7 register fixup, packet params, RxDone IRQ on DIO1, SetRx | each SPI command waits for BUSY low first |

The TCXO is started before the reset so the clock is stable when the chip boots. After a good reset BUSY is low and the chip is in STDBY_RC.

## Radio parameters
| Parameter | Value |
|---|---|
| Frequency | 2.45 GHz |
| Packet type | LoRa (wake and ack), Ranging (ranging exchange) |
| Modulation | SF7, BW 1600 kHz, CR 4/5 (SF7 register fixup 0x925 = 0x37) |
| Preamble / header | 12 symbols, explicit header |
| CRC / IQ | CRC on for wake and ack, standard IQ |
| Payload | wake 4 bytes, ack 12 bytes |
| TX power | -5 dBm (register value 13), ramp 20 us |
| Buffer base | TX 0x00, RX 0x00 |
| IRQ routing | DIO1 only (RxDone, TxDone, ranging bits) |

## Ranging parameters
| Parameter | Value |
|---|---|
| Roles | beacon = master, anchor = slave |
| Packet params | same as radio, but CRC off |
| Address check | 8-bit (register 0x931 = 0x00) |
| Anchor address | `ANCHOR_ADDRESS`, block starts at `0x00000A19` |
| Calibration | 13610 (register 0x92C), per module pair, not verified |
| Result | debiased, `distance_cm = raw x 20` |
| Master IRQ | RESULT_VALID (bit 9), TIMEOUT (bit 10) |
| Slave IRQ | RESPONSE_DONE (bit 7), MASTER_REQUEST_VALID (bit 11) |
| Request timeout | 1000 ms |
| Ranging window | 2000 ms (limits 500 to 10000) |

## Wiring (beacon and anchor)
| Signal | Pin |
|---|---|
| MOSI / MISO / SCK | PA4 / PB0 / PB1 (SPI3) |
| NSS (software CS, GPIO) | PA7 |
| NRESET | PB5 |
| TCXOEN | PB6 |
| BUSY | PB7 |
| DIO1 | PB8 |
| LED | PC13 |
| UART TX / RX | PB14 / PB15 (USART1, 115200) |

## CMake macros (`-D` at configure)
| Macro | Board | Meaning |
|---|---|---|
| `BRINGUP_MODE` | both | 0 production, 1 board test, 2 module test; always pass it |
| `ANCHOR_ADDRESS` | anchor | required, e.g. `0x00000A19`, unique per anchor |
| `ANCHOR_X_CM` `ANCHOR_Y_CM` `ANCHOR_Z_CM` | anchor | required, position in cm |
| `DEBUG_ANCHOR` | anchor | `ON` enables log output |
| `DEBUG_BEACON` | beacon | `ON` enables log output |
| `DEBUG_PIN_TRACE` | beacon | `ON` traces GPIO edges |

Anchor (`anchor-blackpill/`):
```
cmake --preset Release -DBRINGUP_MODE=0 -DDEBUG_ANCHOR=ON -DANCHOR_ADDRESS=0x00000A19 -DANCHOR_X_CM=100 -DANCHOR_Y_CM=-50 -DANCHOR_Z_CM=150 && cmake --build --preset Release
```
Beacon (`beacon-blackpill/`):
```
cmake --preset Release -DBRINGUP_MODE=0 -DDEBUG_BEACON=ON && cmake --build --preset Release
```

## Anchor state machine
`AnchorBridge::step()` is called every main-loop pass (then `WFI`). It reads and clears the radio IRQ once and runs the handler of the current state.

```text
                    to_radio ok               RxDone + wake word
 boot --> RECOVER ---------------> LISTENING -------------------> ACK_REQUESTED
             ^                        ^                                |
             |                        | RESPONSE_DONE or               | slot reached,
             |                        | window closed                  | ack sent, TX_DONE
             |                        |                                v
             |                     RANGING <------------------- ACK_SENT
             |                                  slave armed
             |
             +-- from LISTENING, ACK_REQUESTED, ACK_SENT, RANGING on any failure:
                 header/CRC error, not a wake word, slot missed (> 10 ms), ack send failed,
                 no TX_DONE (50 ms), arming failed, request discarded, to_radio failed
```

| State | Chip | Waits for | Leaves when |
|---|---|---|---|
| `RECOVER` | unknown | retry timer | `to_radio()` ok -> `LISTENING`; first try is immediate, then every 1 s, chip reset (NRESET) after 3 fails |
| `LISTENING` | RX | DIO1: RxDone, header error, CRC error | wake word matches -> `ACK_REQUESTED`; anything else -> `RECOVER` |
| `ACK_REQUESTED` | STDBY | this anchor's ack slot (`(address - 0xA19) x 20 ms`) | sends the ack and polls TX_DONE (50 ms); more than 10 ms late or any failure -> `RECOVER` |
| `ACK_SENT` | STDBY | nothing, next pass | `to_ranging_slave()` ok -> `RANGING`, else `RECOVER` |
| `RANGING` | ranging slave | DIO1: RESPONSE_DONE, or the window from the wake packet | back to `LISTENING` via `to_radio()` |

## Example log
Prerequisite: build with `-DDEBUG_BEACON=ON` (beacon) / `-DDEBUG_ANCHOR=ON` (anchor), otherwise nothing is printed. Open the UART terminal at 115200 8N1 before reset.

Beacon:
```
[3715010] send_wake_broadcast mask=0x3 (full=0x3)
[3715015] entering ACK_LISTEN
[3715023] wake_ack_matched: payload = AC 4B 00 00 0A 19 00 01 FF FE 00 02
[3715031] wake ack matched (anchor 0x00000A19 at 1, -2, 2 cm)
[3715037] collected anchor 0x00000A19 (1/2)
[3715042] wake_ack_matched: payload = AC 4B 00 00 0A 1A 00 01 FF FE 00 02
[3715050] wake ack matched (anchor 0x00000A1A at 1, -2, 2 cm)
[3715056] collected anchor 0x00000A1A (2/2)
[3715060] ACK_LISTEN stopped
[3715063] collect phase done, 2 anchor(s) collected
[3715068] entering RANGING (target 0x00000A19)
[3715078] anchor 0x00000A19 ranging request timed out
[3715084] entering RANGING (target 0x00000A1A)
[3715094] anchor 0x00000A1A ranging request timed out
[3715100] entering RADIO
```
Anchor:
```
[83789] entering LISTENING
[86767] wake word matched (ranging window 2000ms, ack delay 0ms)
[86773] send_ranging_slave_ack mask=0xF (full=0xF)
[86779] send_ranging_slave_ack: TX_DONE confirmed (irq_mask=0x1)
[86786] entering RANGING
[88789] entering LISTENING
```
Timestamps are ms since boot. The beacon example shows ranging timeouts (open issue, see PROTOCOL.md).

## Further development
- **Payload encryption.** Wake and ack payloads are plain today, so anchor positions can be read and the wake can be replayed. Encrypt and authenticate them in firmware (the SX1280 has no AES engine), with a per-cycle counter or nonce against replay and a key stored per device.
- **Ranging calibration.** The RxTx-delay register (13610) is an offset tuned by hand at two distances, and close range returns 0 cm. Measure at known distances (1 m, 2.5 m, 5 m), fit the offset per anchor and beacon pair, keep it per anchor instead of one constant, and consider averaging several results per anchor.
