#pragma once

#include <cstdint>
#include "SX1280Constants.hpp"
#include "SX1280Device.hpp"

enum class MODE { RADIO, RANGING, NONE };
enum class ACK_STATE { PENDING, IDLE };

class AnchorBridge {
  SX1280Device device;
  MODE mode;
  ACK_STATE ack_state;
  uint32_t ranging_window_duration_ms;

public:
  AnchorBridge(SPI_HandleTypeDef* SPI_port_,
               GPIO_TypeDef* BUSY_GPIO_port_,
               GPIO_TypeDef* NSS_GPIO_port_,
               GPIO_TypeDef* NRESET_GPIO_port_,
               GPIO_TypeDef* TCXOEN_GPIO_port_,
               uint16_t BUSY_pin_,
               uint16_t NSS_pin_,
               uint16_t NRESET_pin_,
               uint16_t TCXOEN_pin_)
    : device(
        SPI_port_, BUSY_GPIO_port_, NSS_GPIO_port_, NRESET_GPIO_port_, TCXOEN_GPIO_port_, BUSY_pin_, NSS_pin_, NRESET_pin_, TCXOEN_pin_)
    , mode(MODE::NONE)
    , ack_state(ACK_STATE::IDLE)
    , ranging_window_duration_ms(LORA_BEACON_PROTOCOL::RANGING_WINDOW_MS)
  {
    device.NRESET_reset();
  }

  uint16_t to_radio();
  uint16_t to_ranging_slave();
  uint16_t send_ranging_slave_ack();

  //check is wake word matches and fetch duration of ranging mode (see get_ranging_duration_ms());
  //sets ack_state to PENDING on match
  bool wake_word_matched();

  HAL_StatusTypeDef get_status(SX1280Device::SX1280_Status* sta_out);
  HAL_StatusTypeDef get_irq_mask(uint16_t* mask_out);

  HAL_StatusTypeDef clear_irq_mask(SX1280Device::SX1280_Status* sta_out);

  // DIO1 fired during RANGING but irq_mask didn't have the response-done bit -- caller had nothing else to
  // do with this case, so log it here instead of silently dropping it.
  void log_ranging_irq_unmatched(uint16_t irq_mask);

  uint32_t get_ack_delay_ms();
  uint32_t get_ranging_duration_ms();

  inline MODE get_mode() { return mode; }
  inline ACK_STATE get_ack_state() { return ack_state; }
};