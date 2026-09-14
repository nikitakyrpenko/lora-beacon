#include <cstddef>
#include <stdint.h>

#include "stm32l4xx_hal.h"

#include "SX1280Bridge.h"
#include "SX1280Constants.hpp"
#include "SX1280Device.hpp"
#include "stm32l4xx_hal_def.h"

#ifdef DEBUG_PINS
#include <cstdio>
static void debug_print_step(const char* name, HAL_StatusTypeDef hal, const SX1280Device::SX1280_Status& sta)
{
  printf("[%lu] %s: hal=%d cmd_status=%d busy=%d\r\n",
         (unsigned long)HAL_GetTick(),
         name,
         static_cast<int>(hal),
         static_cast<int>(sta.command_status),
         static_cast<int>(sta.busy));
}
#define DEBUG_STEP(name, hal, sta) debug_print_step(name, hal, sta)
#else
#define DEBUG_STEP(name, hal, sta)
#endif

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

static bool step_ok(HAL_StatusTypeDef hal, const SX1280Device::SX1280_Status& sta)
{
  return hal == HAL_OK && (sta.command_status == SX1280Device::CommandStatus::COMMAND_SUCCESS ||
                           sta.command_status == SX1280Device::CommandStatus::RESERVED);
}

struct Sx1280Step {
  const char* name;
  const uint8_t* op_code;
  const uint8_t* tx;
  uint16_t len;
};

static uint16_t execute_step(const Sx1280Step* steps, size_t count)
{
  if (LoRa_SX1280 == nullptr) {
    return 0;
  }

  uint16_t mask = 0;
  SX1280Device::SX1280_Status sta{};

  for (size_t i = 0; i < count; ++i) {
    HAL_StatusTypeDef hal = LoRa_SX1280->SPI_write(steps[i].op_code, steps[i].tx, nullptr, steps[i].len, &sta);
    DEBUG_STEP(steps[i].name, hal, sta);
    if (!step_ok(hal, sta)) {
      return mask;
    }
    mask |= static_cast<uint16_t>(1u << i);
  }

  return mask;
}

extern "C" uint16_t SX1280_Beacon_Radio()
{
  constexpr uint8_t wake_word_len = static_cast<uint8_t>(sizeof(LORA_BEACON_PROTOCOL::WAKE_WORD));

  uint8_t packet[7] = {SX1280_VALUES::LORA_PREAMBLE_12_SYMBOLS,
                       SX1280_VALUES::EXPLICIT_HEADER,
                       wake_word_len,
                       SX1280_VALUES::LORA_CRC_ENABLE,
                       SX1280_VALUES::LORA_IQ_STD,
                       0x00,
                       0x00};

  uint8_t tx_params[2] = {SX1280_VALUES::TX_OUTPUT_POWER, SX1280_VALUES::RADIO_RAMP_04_US};

  const Sx1280Step radio_steps[] = {
    {"SetStandby", &SX1280_OPERATIONS::SET_STANDBY_OP_CODE, &SX1280_VALUES::STDBY_RC_STAND_BY, 1},
    {"SetPacketType", &SX1280_OPERATIONS::SET_PACKET_TYPE_OP_CODE, &SX1280_VALUES::PACKET_TYPE_LORA, 1},
    {"SetRfFrequency", &SX1280_OPERATIONS::SET_FREQUENCY_OP_CODE, SX1280_VALUES::RF_FREQUENCY_BYTES, 3},
    {"SetBufferBaseAddress", &SX1280_OPERATIONS::SET_BUFFER_BASE_ADDRESS_OP_CODE, SX1280_VALUES::BUFFER_BASE_ADDRESS, 2},
    {"SetModulationParams", &SX1280_OPERATIONS::SET_MODULATION_OP_CODE, SX1280_VALUES::MODULATION_PARAMS_SF7, 3},
    {"SF7RegisterFixup", &SX1280_OPERATIONS::WRITE_REGISTER_OP_CODE, SX1280_VALUES::SF_7_FIXUP_WRITE, 3},
    {"SetPacketParams", &SX1280_OPERATIONS::SET_PACKET_PARAMS_OP_CODE, packet, 7},
    {"SetTxParams", &SX1280_OPERATIONS::SET_TX_PARAMS_OP_CODE, tx_params, 2},
  };

  return execute_step(radio_steps, sizeof(radio_steps) / sizeof(radio_steps[0]));
}

extern "C" uint16_t SX1280_Send_Wake_Broadcast()
{
  uint8_t write_buffer[3] = {0x00, LORA_BEACON_PROTOCOL::WAKE_WORD[0], LORA_BEACON_PROTOCOL::WAKE_WORD[1]};
  uint8_t tx[3] = {SX1280_VALUES::PERIOD_BASE_1_MS, 0x00, 0x00};  // timeoutCount=0x0000 -> single-shot TX, auto-standby on TxDone

  const Sx1280Step send_steps[] = {
    {"WriteBuffer", &SX1280_OPERATIONS::WRITE_BUFFER_OP_CODE, write_buffer, 3},
    {"SetTx", &SX1280_OPERATIONS::SET_TX_OP_CODE, tx, 3},
  };

  return execute_step(send_steps, sizeof(send_steps) / sizeof(send_steps[0]));
}