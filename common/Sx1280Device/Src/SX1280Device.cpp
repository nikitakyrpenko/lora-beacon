#include <cstdint>

#include "SX1280Device.hpp"
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_def.h"
#include "stm32l4xx_hal_spi.h"

#ifdef DEBUG_PINS
#include <cstdio>
static void debug_print_pin(const SX1280Device::GPIO_Pin& pin)
{
  printf("[%lu] %s: %s\r\n", (unsigned long)HAL_GetTick(), pin.name, pin.current == GPIO_PIN_SET ? "HIGH" : "LOW");
}
#define DEBUG_PIN(pin) debug_print_pin(pin)
#else
#define DEBUG_PIN(pin)
#endif

SX1280Device::SX1280Device(SPI_HandleTypeDef* SPI_port_,
                           GPIO_TypeDef* BUSY_GPIO_port,
                           GPIO_TypeDef* NSS_GPIO_port,
                           GPIO_TypeDef* NRESET_GPIO_port,
                           GPIO_TypeDef* TCXOEN_GPIO_port,
                           uint16_t BUSY_pin,
                           uint16_t NSS_pin,
                           uint16_t NRESET_pin,
                           uint16_t TCXOEN_pin)
  : SPI_port(SPI_port_)
  , BUSY{BUSY_GPIO_port, BUSY_pin, BUSY_PIN_NAME, HAL_GPIO_ReadPin(BUSY_GPIO_port, BUSY_pin)}
  , NSS{NSS_GPIO_port, NSS_pin, NSS_PIN_NAME, HAL_GPIO_ReadPin(NSS_GPIO_port, NSS_pin)}
  , NRESET{NRESET_GPIO_port, NRESET_pin, NRESET_PIN_NAME, HAL_GPIO_ReadPin(NRESET_GPIO_port, NRESET_pin)}
  , TCXOEN{TCXOEN_GPIO_port, TCXOEN_pin, TCXOEN_PIN_NAME, HAL_GPIO_ReadPin(TCXOEN_GPIO_port, TCXOEN_pin)}
{
}

SX1280Device::SX1280_Status SX1280Device::to_status(uint8_t r)
{
  return {static_cast<CircuitMode>((r >> 5) & 0x7), static_cast<CommandStatus>((r >> 2) & 0x7), static_cast<bool>(r & 0x1)};
}

bool SX1280Device::BUSY_wait(uint8_t timeout)
{
  uint32_t start = HAL_GetTick();
  BUSY.current = HAL_GPIO_ReadPin(BUSY.port, BUSY.pin);
  DEBUG_PIN(BUSY);
  while (BUSY.current == GPIO_PIN_SET) {
    if (HAL_GetTick() - start >= timeout) {
      return false;
    }
    BUSY.current = HAL_GPIO_ReadPin(BUSY.port, BUSY.pin);
    DEBUG_PIN(BUSY);
  }
  return true;
}

bool SX1280Device::NSS_begin()
{
  if (!BUSY_wait(BUSY_TIMEOUT)) {
    return false;
  }
  if (HAL_GPIO_ReadPin(NSS.port, NSS.pin) == GPIO_PIN_RESET) {
    return false;
  }

  HAL_GPIO_TogglePin(NSS.port, NSS.pin);
  NSS.current = (NSS.current == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET;
  DEBUG_PIN(NSS);

  return true;
}

bool SX1280Device::NSS_end()
{
  if (HAL_GPIO_ReadPin(NSS.port, NSS.pin) == GPIO_PIN_SET) {
    return false;
  }
  HAL_GPIO_TogglePin(NSS.port, NSS.pin);
  NSS.current = (NSS.current == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET;
  DEBUG_PIN(NSS);
  return true;
}

bool SX1280Device::TCXOEN_warmup()
{
  // already high
  if (HAL_GPIO_ReadPin(TCXOEN.port, TCXOEN.pin) == GPIO_PIN_SET) {
    TCXOEN.current = GPIO_PIN_SET;
    DEBUG_PIN(TCXOEN);
    return true;
  }
  HAL_GPIO_WritePin(TCXOEN.port, TCXOEN.pin, GPIO_PIN_SET);
  TCXOEN.current = GPIO_PIN_SET;
  DEBUG_PIN(TCXOEN);
  HAL_Delay(TCXOEN_HOLD);

  return true;
}

bool SX1280Device::NRESET_reset()
{
  TCXOEN_warmup();

  HAL_GPIO_WritePin(NRESET.port, NRESET.pin, GPIO_PIN_RESET);
  NRESET.current = GPIO_PIN_RESET;
  DEBUG_PIN(NRESET);

  HAL_Delay(NRESET_HOLD);

  HAL_GPIO_WritePin(NRESET.port, NRESET.pin, GPIO_PIN_SET);
  NRESET.current = GPIO_PIN_SET;
  DEBUG_PIN(NRESET);

  return BUSY_wait(BUSY_TIMEOUT);
}

HAL_StatusTypeDef SX1280Device::SPI_write(const uint8_t* reg, const uint8_t* buf, const uint16_t len, SX1280_Status* out)
{
  if (len > SPI_PACKET_SIZE) {
    return HAL_ERROR;
  }

  if (!BUSY_wait(BUSY_TIMEOUT)) {
    return HAL_TIMEOUT;
  }

  if (!NSS_begin()) {
    return HAL_ERROR;
  }

  uint8_t tx[SPI_PACKET_SIZE + 1] = {*reg};
  uint8_t rx[SPI_PACKET_SIZE + 1] = {};

  for (uint16_t i = 0; i < len; ++i) {
    tx[i + 1] = buf[i];
  }

  HAL_StatusTypeDef result = HAL_SPI_TransmitReceive(SPI_port, tx, rx, len + 1, SPI_TIMEOUT);
  if (result == HAL_OK) {
    *out = to_status(rx[1]);
  }

  if (!NSS_end()) {
    return HAL_ERROR;
  }

  return result;
}

HAL_StatusTypeDef SX1280Device::SPI_read(
  const uint8_t* reg, const uint16_t* addr, uint8_t* buf, uint16_t len, SX1280Device::SX1280_Status* out)
{
  if (len > SPI_PACKET_SIZE) {
    return HAL_ERROR;
  }

  if (!BUSY_wait(BUSY_TIMEOUT)) {
    return HAL_TIMEOUT;
  }

  if (!NSS_begin()) {
    return HAL_ERROR;
  }

  uint8_t addr_msb = static_cast<uint8_t>(*addr >> 8);
  uint8_t addr_lsb = static_cast<uint8_t>(*addr & 0xFF);

  // opcode, addrMSB, addrLSB, NOP (status returned here), then len data bytes
  uint8_t tx[SPI_PACKET_SIZE + 4] = {*reg, addr_msb, addr_lsb};
  uint8_t rx[SPI_PACKET_SIZE + 4] = {};

  HAL_StatusTypeDef result = HAL_SPI_TransmitReceive(SPI_port, tx, rx, len + 4, SPI_TIMEOUT);
  if (result == HAL_OK) {
    *out = to_status(rx[3]);
    for (uint16_t i = 0; i < len; ++i) {
      buf[i] = rx[i + 4];
    }
  }

  if (!NSS_end()) {
    return HAL_ERROR;
  }

  return result;
}
