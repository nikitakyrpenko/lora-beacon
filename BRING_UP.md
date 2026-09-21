# Bring-up modes

Set with `-DBRINGUP_MODE=n` at configure time (the VS Code build tasks pass it; always pass it, the anchor's CMake default is not 0).
Output goes to USART1, 115200 8N1. Reflash with mode 0 for deployment.

| Mode | What runs |
|---|---|
| 0 | production firmware |
| 1 | board-level test, SX1280 module not needed |
| 2 | module-level self-test, module attached |

## Mode 1: board level
Runs forever. Prints a `BOARD-BRINGUP: UART TX OK` banner, then:
- cycles NSS, NRESET, TCXOEN and the LED, each HIGH for 2 s then LOW for 2 s (probe the header with a meter; the LED is active-low);
- every 300 ms prints the raw BUSY and DIO1 levels, and `DIO1 EXTI fired!` when the interrupt flag sets (pulse DIO1 by hand);
- SPI3 loopback: jumper MOSI to MISO; it prints `MATCH` / `MISMATCH` for a running byte. This is the only automatic pass/fail here.

## Mode 2: module level
Runs once at boot; the checks all run, none stops the others. Each prints `BRING-UP n/5 <name>: OK|FAIL`, then
`BRING-UP SELF-TEST mask=0x.. (full=0x1F) PASS|FAIL` (bit 0..4 = check 1..5). LED: solid = pass, ~5 Hz blink = fail. The anchor also prints its compiled address and position first.

| # | Check | Passes when |
|---|---|---|
| 1 | GPIO idle | NSS high, NRESET high, TCXOEN low (read before the chip is reset) |
| 2 | Reset | BUSY reads LOW 20 ms after the reset |
| 3 | Chip alive | GetStatus reports circuit mode STDBY_RC (2); modes 0, 1 or 7 point at MISO (undriven line) |
| 4 | Command round-trip | anchor `to_radio()`, beacon `listen_for_ack()`: GetStatus then reads RX (5) |
| 5 | DIO1 / EXTI | anchor: the ack's TX_DONE edge reaches the interrupt flag (one ack goes on air); beacon: the 100 ms RX-timeout edge does (transmits nothing) |

Notes: everything prints once, so open the terminal first, then reset. GetStatus is shown decoded, not as the raw byte. A DIO1 that is
already stuck high gives no edge, so check 5 cannot see it.

Code: board-level test, checks 1-3 and the summary are shared in `common/Sx1280Device/Inc/SX1280BringUp.hpp`; checks 4-5 are in each board's `main.cpp`.
