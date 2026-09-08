#include <stdint.h>

#include "stm32l4xx_hal.h"

#include "SX1280Bridge.h"
#include "SX1280Constants.hpp"
#include "SX1280Device.hpp"

static SX1280Device* LoRa_SX1280 = nullptr;

extern "C" void SX1280_Create(SPI_HandleTypeDef* SPI_port,
                              GPIO_TypeDef* BUSY_GPIO_port,
                              GPIO_TypeDef* NSS_GPIO_port,
                              GPIO_TypeDef* NRESET_GPIO_port,
                              GPIO_TypeDef* TCXOEN_GPIO_port,
                              uint16_t BUSY_pin,
                              uint16_t NSS_pin,
                              uint16_t NRESET_pin,
                              uint16_t TCXOEN_pin)
{
  if (LoRa_SX1280 == nullptr) {
    static SX1280Device device(
      SPI_port, BUSY_GPIO_port, NSS_GPIO_port, NRESET_GPIO_port, TCXOEN_GPIO_port, BUSY_pin, NSS_pin, NRESET_pin, TCXOEN_pin);
    LoRa_SX1280 = &device;
  }

  LoRa_SX1280->NRESET_reset();
}

extern "C" void SX1280_Init()
{
  if (LoRa_SX1280 == nullptr) {
    return;
  }

  SX1280Device::SX1280_Status sta{};

  if (LoRa_SX1280->SPI_write(&SX1280_OPERATIONS::SET_STANDBY_OP_CODE, &SX1280_VALUES::STDBY_RC_STAND_BY, static_cast<uint16_t>(1), &sta) !=
      HAL_OK) {
  }
}
