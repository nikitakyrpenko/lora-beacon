#pragma once

#include <cstddef>
#include <cstdint>
#include <cstdio>

#include "SX1280Device.hpp"

// Permanent hardware bring-up tests shared by the anchor and the beacon (see test.md). Header-only so the boards just
// include it. Everything here is role-agnostic; the role-specific checks (commands only one board has) live in each
// board's main.cpp. BRINGUP_MODE: 0 = production, 1 = board-level (RunBoardLevel), 2 = module-level (checks below).
namespace SX1280BringUp {

// Pins the tests touch -- the pinout is identical on the anchor and the beacon Blackpill boards.
struct BoardPins {
  GPIO_TypeDef* nss_port;
  uint16_t nss_pin;
  GPIO_TypeDef* nreset_port;
  uint16_t nreset_pin;
  GPIO_TypeDef* tcxoen_port;
  uint16_t tcxoen_pin;
  GPIO_TypeDef* led_port;
  uint16_t led_pin;
  GPIO_TypeDef* busy_port;
  uint16_t busy_pin;
  GPIO_TypeDef* dio1_port;
  uint16_t dio1_pin;
};

// Bit layout of the module-level result mask
static constexpr uint16_t BIT_GPIO_IDLE = 1u << 0;
static constexpr uint16_t BIT_RESET = 1u << 1;
static constexpr uint16_t BIT_CHIP_ALIVE = 1u << 2;
static constexpr uint16_t BIT_COMMAND_ROUNDTRIP = 1u << 3;
static constexpr uint16_t BIT_DIO1 = 1u << 4;
static constexpr uint16_t FULL_MASK = 0x1F;

inline const char* LevelName(GPIO_PinState level)
{
  return level == GPIO_PIN_SET ? "HIGH" : "LOW";
}

inline void Report(int number, const char* name, bool ok)
{
  printf("[%lu] BRING-UP %d/5 %s: %s\r\n", (unsigned long)HAL_GetTick(), number, name, ok ? "OK" : "FAIL");
}

// Busy-waits up to timeout_ms for *flag (set from an EXTI callback) to become non-zero; returns whether it was set and
// clears it either way.
inline bool WaitForFlag(volatile uint8_t* flag, uint32_t timeout_ms)
{
  const uint32_t deadline = HAL_GetTick() + timeout_ms;
  while (!*flag && HAL_GetTick() < deadline) {
  }
  const bool set = (*flag != 0);
  *flag = 0;
  return set;
}

// ------------------------------------------------------------------------------------------------------------------
// Layer 1: board-level bring-up (BRINGUP_MODE == 1). Interactive, never returns. The SX1280 module need not be attached.
// ------------------------------------------------------------------------------------------------------------------
[[noreturn]] inline void RunBoardLevel(const BoardPins& pins, SPI_HandleTypeDef* spi, volatile uint8_t* dio1_flag)
{
  printf("[%lu] BOARD-BRINGUP: UART TX OK if you can read this\r\n", (unsigned long)HAL_GetTick());
  printf(
    "[%lu] BOARD-BRINGUP: probe each output pin at its header while it is driven; jumper SPI3 MOSI to MISO for the "
    "loopback check\r\n",
    (unsigned long)HAL_GetTick());

  struct Output {
    GPIO_TypeDef* port;
    uint16_t pin;
    const char* name;
  };
  const Output outputs[] = {
    {pins.nss_port, pins.nss_pin, "NSS"},
    {pins.nreset_port, pins.nreset_pin, "NRESET"},
    {pins.tcxoen_port, pins.tcxoen_pin, "TCXOEN"},
    {pins.led_port, pins.led_pin, "LED (active-low: LOW = on)"},
  };
  constexpr size_t OUTPUT_COUNT = sizeof(outputs) / sizeof(outputs[0]);
  constexpr uint32_t OUTPUT_DWELL_MS = 2000;  // long enough for a multimeter / LED at the physical header
  constexpr uint32_t INPUT_POLL_MS = 300;

  const uint32_t start_tick = HAL_GetTick();
  uint32_t last_phase = UINT32_MAX;
  uint32_t input_last_tick = start_tick;
  uint8_t loopback_byte = 0xA5;

  while (true) {
    // outputs: each pin HIGH for the dwell time, then LOW for the dwell time, then the next pin, forever
    const uint32_t phase = (HAL_GetTick() - start_tick) / OUTPUT_DWELL_MS;
    if (phase != last_phase) {
      last_phase = phase;
      const Output& output = outputs[(phase / 2) % OUTPUT_COUNT];
      const GPIO_PinState level = (phase % 2 == 0) ? GPIO_PIN_SET : GPIO_PIN_RESET;
      HAL_GPIO_WritePin(output.port, output.pin, level);
      printf("[%lu] BOARD-BRINGUP: <%s> = %s, probe now\r\n", (unsigned long)HAL_GetTick(), output.name, LevelName(level));
    }

    // inputs + SPI3 loopback, polled
    if (HAL_GetTick() - input_last_tick >= INPUT_POLL_MS) {
      input_last_tick = HAL_GetTick();

      printf("[%lu] BOARD-BRINGUP: <BUSY> = %s, <DIO1> = %s\r\n",
             (unsigned long)HAL_GetTick(),
             LevelName(HAL_GPIO_ReadPin(pins.busy_port, pins.busy_pin)),
             LevelName(HAL_GPIO_ReadPin(pins.dio1_port, pins.dio1_pin)));

      // confirms the interrupt/NVIC path when DIO1 is pulsed high by hand
      if (*dio1_flag) {
        *dio1_flag = 0;
        printf("[%lu] BOARD-BRINGUP: DIO1 EXTI fired!\r\n", (unsigned long)HAL_GetTick());
      }

      // fully internal to the STM32 (MOSI jumpered to MISO): the one check with an automatic pass/fail
      uint8_t tx = loopback_byte++;
      uint8_t rx = 0;
      const HAL_StatusTypeDef spi_status = HAL_SPI_TransmitReceive(spi, &tx, &rx, 1, 10);
      printf("[%lu] BOARD-BRINGUP: SPI loopback byte 0x%02X -> 0x%02X (hal=%d) %s\r\n",
             (unsigned long)HAL_GetTick(),
             tx,
             rx,
             static_cast<int>(spi_status),
             (spi_status == HAL_OK && tx == rx) ? "MATCH" : "MISMATCH");
    }
  }
}

// ------------------------------------------------------------------------------------------------------------------
// Layer 2: module-level bring-up (BRINGUP_MODE == 2). Shared checks 1-3; checks 4-5 are per board. The checks run
// unconditionally, not fail-fast, to get as much diagnostic information as possible from one flash-and-read cycle.
// ------------------------------------------------------------------------------------------------------------------

// 1/5: idle levels straight after MX_GPIO_Init(): NSS=HIGH, NRESET=HIGH, TCXOEN=LOW. BUSY is printed only -- it is driven
// by the chip and check 2 decides on it. Must run BEFORE the bridge exists: its constructor resets the chip.
inline bool CheckIdleGpioState(const BoardPins& pins)
{
  const GPIO_PinState nss = HAL_GPIO_ReadPin(pins.nss_port, pins.nss_pin);
  const GPIO_PinState nreset = HAL_GPIO_ReadPin(pins.nreset_port, pins.nreset_pin);
  const GPIO_PinState tcxoen = HAL_GPIO_ReadPin(pins.tcxoen_port, pins.tcxoen_pin);
  const GPIO_PinState busy = HAL_GPIO_ReadPin(pins.busy_port, pins.busy_pin);

  const bool nss_ok = (nss == GPIO_PIN_SET);
  const bool nreset_ok = (nreset == GPIO_PIN_SET);
  const bool tcxoen_ok = (tcxoen == GPIO_PIN_RESET);

  printf("[%lu]   NSS    = %s (expect HIGH) %s\r\n", (unsigned long)HAL_GetTick(), LevelName(nss), nss_ok ? "ok" : "WRONG");
  printf("[%lu]   NRESET = %s (expect HIGH) %s\r\n", (unsigned long)HAL_GetTick(), LevelName(nreset), nreset_ok ? "ok" : "WRONG");
  printf("[%lu]   TCXOEN = %s (expect LOW)  %s\r\n", (unsigned long)HAL_GetTick(), LevelName(tcxoen), tcxoen_ok ? "ok" : "WRONG");
  printf("[%lu]   BUSY   = %s (chip-driven, not gated here)\r\n", (unsigned long)HAL_GetTick(), LevelName(busy));

  const bool ok = nss_ok && nreset_ok && tcxoen_ok;
  Report(1, "GPIO idle state", ok);
  return ok;
}

// 2/5: the bridge constructor has already pulsed NRESET and waited for BUSY (its result isn't exposed), so judge the
// outcome from the pin: after another 20 ms BUSY must be LOW. The extra wait also avoids trusting a BUSY that simply
// hasn't been raised yet right after the release -- the chip raises it for its start-up calibration.
inline bool CheckResetBusyLow(const BoardPins& pins)
{
  HAL_Delay(20);
  const GPIO_PinState busy = HAL_GPIO_ReadPin(pins.busy_port, pins.busy_pin);
  const bool ok = (busy == GPIO_PIN_RESET);
  printf("[%lu]   BUSY = %s 20 ms after the reset (expect LOW)\r\n", (unsigned long)HAL_GetTick(), LevelName(busy));
  if (!ok) {
    printf("[%lu]   BUSY did not fall: the chip is not finishing start-up (clock or supply problem) or BUSY is stuck high\r\n",
           (unsigned long)HAL_GetTick());
  }
  Report(2, "reset sequence", ok);
  return ok;
}

// 3/5: get_status() must complete and report circuit_mode == STDBY_RC (2). Works with any bridge that has
// `HAL_StatusTypeDef get_status(SX1280Device::SX1280_Status*)`. Prints the decoded fields; the bridges don't expose the raw
// status byte, but circuit_mode 7 / command_status 7 is the all-ones pattern of a MISO that isn't driven.
template <typename Bridge>
bool CheckChipAlive(Bridge& bridge)
{
  SX1280Device::SX1280_Status sta{};
  const HAL_StatusTypeDef hal = bridge.get_status(&sta);
  if (hal != HAL_OK) {
    printf("[%lu]   GetStatus transfer failed: hal=%d (3 = BUSY never released, 1 = NSS/transfer error)\r\n",
           (unsigned long)HAL_GetTick(),
           static_cast<int>(hal));
    Report(3, "chip alive (GetStatus)", false);
    return false;
  }

  const uint8_t mode = static_cast<uint8_t>(sta.circuit_mode);
  const uint8_t cmd = static_cast<uint8_t>(sta.command_status);
  printf("[%lu]   GetStatus: circuit_mode=%u command_status=%u busy=%d\r\n",
         (unsigned long)HAL_GetTick(),
         mode,
         cmd,
         static_cast<int>(sta.busy));

  // after a reset the chip must be in STDBY_RC (2); command_status RESERVED (0) is legitimate and not rejected
  const bool ok = (sta.circuit_mode == SX1280Device::CircuitMode::STDBY_RC);
  if (!ok) {
    if (mode > static_cast<uint8_t>(SX1280Device::CircuitMode::TX)) {
      printf(
        "[%lu]   implausible circuit_mode %u (no such mode): no valid data on MISO -- undriven/floating line, check the "
        "MISO joint\r\n",
        (unsigned long)HAL_GetTick(),
        mode);
    }
    else if (mode < static_cast<uint8_t>(SX1280Device::CircuitMode::STDBY_RC)) {
      printf(
        "[%lu]   circuit_mode %u is RESERVED: the chip is not answering with a valid status -- MISO joint / wiring, or "
        "the chip has not finished start-up\r\n",
        (unsigned long)HAL_GetTick(),
        mode);
    }
    else {
      printf("[%lu]   the chip answers but circuit_mode %u is not STDBY_RC (2) after a reset\r\n", (unsigned long)HAL_GetTick(), mode);
    }
  }
  Report(3, "chip alive (GetStatus)", ok);
  return ok;
}

// Prints the final mask and shows the result on the LED forever: solid = pass, ~5 Hz blink = fail (LED is active-low),
// so the result is visible even with no terminal attached.
[[noreturn]] inline void Finish(const BoardPins& pins, uint16_t mask)
{
  const bool pass = (mask == FULL_MASK);
  printf(
    "[%lu] BRING-UP SELF-TEST mask=0x%02X (full=0x%02X) %s\r\n", (unsigned long)HAL_GetTick(), mask, FULL_MASK, pass ? "PASS" : "FAIL");
  while (true) {
    HAL_GPIO_WritePin(pins.led_port, pins.led_pin, pass ? GPIO_PIN_RESET : (((HAL_GetTick() / 200) % 2) ? GPIO_PIN_RESET : GPIO_PIN_SET));
    HAL_Delay(20);
  }
}

}  // namespace SX1280BringUp
