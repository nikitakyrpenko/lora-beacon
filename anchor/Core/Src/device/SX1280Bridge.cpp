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

extern "C" uint16_t SX1280_Radio_mode()
{
  uint16_t mask = 0x0;

  if (LoRa_SX1280 == nullptr) {
    return mask;
  }

  SX1280Device::SX1280_Status sta{};

  HAL_StatusTypeDef hal = LoRa_SX1280->SPI_write(
    &SX1280_OPERATIONS::SET_STANDBY_OP_CODE, &SX1280_VALUES::STDBY_RC_STAND_BY, nullptr, static_cast<uint16_t>(1), &sta);
  DEBUG_STEP("SetStandby", hal, sta);
  if (!step_ok(hal, sta)) {
    return mask;
  }
  mask |= 0b1;

  hal = LoRa_SX1280->SPI_write(
    &SX1280_OPERATIONS::SET_PACKET_TYPE_OP_CODE, &SX1280_VALUES::PACKET_TYPE_LORA, nullptr, static_cast<uint16_t>(1), &sta);
  DEBUG_STEP("SetPacketType", hal, sta);
  if (!step_ok(hal, sta)) {
    return mask;
  }
  mask |= 0b10;

  uint8_t FREQ[3] = {SX1280_VALUES::FREQUENCY_MSB, SX1280_VALUES::FREQUENCY_MID, SX1280_VALUES::FREQUENCY_LSB};
  hal = LoRa_SX1280->SPI_write(&SX1280_OPERATIONS::SET_FREQUENCY_OP_CODE, FREQ, nullptr, static_cast<uint16_t>(3), &sta);
  DEBUG_STEP("SetRfFrequency", hal, sta);
  if (!step_ok(hal, sta)) {
    return mask;
  }
  mask |= 0b100;

  uint8_t BASE_ADDRESS[2] = {0x00, 0x00};
  hal = LoRa_SX1280->SPI_write(&SX1280_OPERATIONS::SET_BUFFER_BASE_ADDRESS_OP_CODE, BASE_ADDRESS, nullptr, static_cast<uint16_t>(2), &sta);
  DEBUG_STEP("SetBufferBaseAddress", hal, sta);
  if (!step_ok(hal, sta)) {
    return mask;
  }
  mask |= 0b1000;

  uint8_t MODULATION_PARAMS[3] = {SX1280_VALUES::SPREADING_FACTOR_SF_7, SX1280_VALUES::BANDWITH_BW_1600, SX1280_VALUES::CHIP_RATE_CR_4_5};
  hal = LoRa_SX1280->SPI_write(&SX1280_OPERATIONS::SET_MODULATION_OP_CODE, MODULATION_PARAMS, nullptr, static_cast<uint16_t>(3), &sta);
  DEBUG_STEP("SetModulationParams", hal, sta);
  if (!step_ok(hal, sta)) {
    return mask;
  }
  mask |= 0b10000;

  uint8_t SF_7_FIXUP[3] = {static_cast<uint8_t>(SX1280_VALUES::REG_SF_MODULATION_FIXUP >> 8),  // 0x09
                           static_cast<uint8_t>(SX1280_VALUES::REG_SF_MODULATION_FIXUP & 0xFF),
                           SX1280_VALUES::SF_7_REGISTER_FIXUP};
  hal = LoRa_SX1280->SPI_write(&SX1280_OPERATIONS::WRITE_REGISTER_OP_CODE, SF_7_FIXUP, nullptr, static_cast<uint16_t>(3), &sta);
  DEBUG_STEP("SF7RegisterFixup", hal, sta);
  if (!step_ok(hal, sta)) {
    return mask;
  }
  mask |= 0b100000;

  uint8_t PACKET_PARAMS[7] = {SX1280_VALUES::LORA_PREAMBLE_12_SYMBOLS,
                              SX1280_VALUES::EXPLICIT_HEADER,
                              0x02,
                              SX1280_VALUES::LORA_CRC_ENABLE,
                              SX1280_VALUES::LORA_IQ_STD,
                              0x00,
                              0x00};
  hal = LoRa_SX1280->SPI_write(&SX1280_OPERATIONS::SET_PACKET_PARAMS_OP_CODE, PACKET_PARAMS, nullptr, static_cast<uint16_t>(7), &sta);
  DEBUG_STEP("SetPacketParams", hal, sta);
  if (!step_ok(hal, sta)) {
    return mask;
  }
  mask |= 0b1000000;

  uint8_t IRQ_PARAMS[8] = {static_cast<uint8_t>(SX1280_VALUES::IRQ_BIT_RX_DONE >> 8),
                           static_cast<uint8_t>(SX1280_VALUES::IRQ_BIT_RX_DONE),
                           static_cast<uint8_t>(SX1280_VALUES::IRQ_BIT_RX_DONE >> 8),
                           static_cast<uint8_t>(SX1280_VALUES::IRQ_BIT_RX_DONE),
                           0x0,
                           0x0,
                           0x0,
                           0x0};
  hal = LoRa_SX1280->SPI_write(&SX1280_OPERATIONS::SET_DIO_IRQ_PARAMS_OP_CODE, IRQ_PARAMS, nullptr, static_cast<uint16_t>(8), &sta);
  DEBUG_STEP("SetDioIrqParams", hal, sta);
  if (!step_ok(hal, sta)) {
    return mask;
  }
  mask |= 0b10000000;

  uint8_t PREAMBLE[1] = {0x1};
  hal = LoRa_SX1280->SPI_write(&SX1280_OPERATIONS::SET_LONG_PREAMBLE_OP_CODE, PREAMBLE, nullptr, static_cast<uint16_t>(1), &sta);
  DEBUG_STEP("SetLongPreamble", hal, sta);
  if (!step_ok(hal, sta)) {
    return mask;
  }
  mask |= 0b100000000;

  uint8_t DUTY_CYCLE_PARAMS[5] = {SX1280_VALUES::PERIOD_BASE_1_MS,
                                  static_cast<uint8_t>(SX1280_VALUES::ANCHOR_IDLE_RX_PERIOD_BASE_COUNT >> 8),
                                  static_cast<uint8_t>(SX1280_VALUES::ANCHOR_IDLE_RX_PERIOD_BASE_COUNT),
                                  static_cast<uint8_t>(SX1280_VALUES::ANCHOR_IDLE_SLEEP_PERIOD_BASE_COUNT >> 8),
                                  static_cast<uint8_t>(SX1280_VALUES::ANCHOR_IDLE_SLEEP_PERIOD_BASE_COUNT)};
  hal = LoRa_SX1280->SPI_write(&SX1280_OPERATIONS::SET_RX_DUTY_CYCLE_OP_CODE, DUTY_CYCLE_PARAMS, nullptr, static_cast<uint16_t>(5), &sta);
  DEBUG_STEP("SetRxDutyCycle", hal, sta);
  if (!step_ok(hal, sta)) {
    return mask;
  }
  mask |= 0b1000000000;

  return mask;
}

extern "C" uint16_t SX1280_Ranging_Slave_Mode() {}

extern "C" uint16_t SX1280_Check_Wake_Word_Matches()
{
  SX1280Device::SX1280_Status sta{};

  uint8_t tx_clear_irq[2] = {0xFF, 0xFF};

  auto hal = LoRa_SX1280->SPI_write(&SX1280_OPERATIONS::CLEAR_IRQ_STATUS_OP_CODE, tx_clear_irq, nullptr, 2, &sta);
  if (hal != HAL_OK ||
      (sta.command_status != SX1280Device::CommandStatus::RESERVED && sta.command_status != SX1280Device::CommandStatus::COMMAND_SUCCESS)) {
    return 0;
  }
  sta = {};

  uint8_t tx_buffer_status[3] = {};
  uint8_t rx_buffer_status[3] = {};

  hal = LoRa_SX1280->SPI_write(&SX1280_OPERATIONS::READ_BUFFER_STATUS_OP_CODE, tx_buffer_status, rx_buffer_status, 3, &sta);

  if (hal != HAL_OK ||
      (sta.command_status != SX1280Device::CommandStatus::RESERVED && sta.command_status != SX1280Device::CommandStatus::COMMAND_SUCCESS)) {
    return 0;
  }

  uint8_t rx_len = rx_buffer_status[1];
  uint8_t rx_start = rx_buffer_status[2];

  constexpr uint8_t WAKE_WORD_LEN = sizeof(LORA_BEACON_PROTOCOL::WAKE_WORD);
  if (rx_len != WAKE_WORD_LEN) {
    return 0;
  }

  uint8_t tx_read_buffer[2 + WAKE_WORD_LEN] = {rx_start};
  uint8_t rx_read_buffer[2 + WAKE_WORD_LEN] = {};
  hal = LoRa_SX1280->SPI_write(
    &SX1280_OPERATIONS::READ_BUFFER_OP_CODE, tx_read_buffer, rx_read_buffer, static_cast<uint16_t>(sizeof(tx_read_buffer)), &sta);

  if (hal != HAL_OK ||
      (sta.command_status != SX1280Device::CommandStatus::RESERVED && sta.command_status != SX1280Device::CommandStatus::COMMAND_SUCCESS)) {
    return 0;
  }

  const uint8_t* payload = &rx_read_buffer[2];
  for (uint8_t i = 0; i < WAKE_WORD_LEN; ++i) {
    if (payload[i] != LORA_BEACON_PROTOCOL::WAKE_WORD[i]) {
      return 0;
    }
  }

  return 1;
}