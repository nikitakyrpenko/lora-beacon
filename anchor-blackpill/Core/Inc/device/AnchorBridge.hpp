#pragma once

#include <cstdint>
#include "SX1280Constants.hpp"
#include "SX1280Device.hpp"

enum class MODE { LISTENING, RANGING, ACK_REQUESTED, ACK_SENT, RECOVER };

class AnchorBridge {
  SX1280Device device;
  MODE mode;

  uint32_t ranging_window_duration_ms;
  uint32_t ack_slot_deadline_tick;
  uint32_t ranging_window_deadline_tick;
  uint32_t recover_last_fail_tick{0};
  uint8_t recover_fail_count{0};


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
    , mode(MODE::RECOVER)
    , ranging_window_duration_ms(LORA_BEACON_PROTOCOL::RANGING_WINDOW_MS)
    , ack_slot_deadline_tick(0)
    , ranging_window_deadline_tick(0)
  {
    device.NRESET_reset();
  }

  uint16_t to_radio();
  uint16_t to_ranging_slave();
  uint16_t send_ranging_slave_ack();

  //check is wake word matches and fetch duration of ranging mode (see get_ranging_duration_ms())
  bool wake_word_matched();

  HAL_StatusTypeDef get_status(SX1280Device::SX1280_Status* sta_out);
  HAL_StatusTypeDef get_irq_mask(uint16_t* mask_out);

  HAL_StatusTypeDef clear_irq_mask(SX1280Device::SX1280_Status* sta_out);

  // DIO1 fired during RANGING but irq_mask didn't have the response-done bit -- caller had nothing else to
  // do with this case, so log it here instead of silently dropping it.
  void log_ranging_irq_unmatched(uint16_t irq_mask);

  uint32_t get_ack_delay_ms();
  uint32_t get_ranging_duration_ms();

  // Advances the state machine by one pass; call it every main-loop iteration. `dio1_flag` is the flag the DIO1 EXTI callback
  // sets -- consumed and cleared here, and the IRQ status is read and cleared once here and handed to the handlers.
  void step(volatile uint8_t& dio1_flag);

  inline MODE get_mode() { return mode; }

  // state handlers, one per MODE; `irq` is the IRQ status step() read for this pass (0 when DIO1 did not fire)
  void on_listen(uint16_t irq, uint32_t hal_tick);
  void on_ack_requested(uint16_t irq, uint32_t hal_tick);
  void on_ack_sent(uint16_t irq, uint32_t hal_tick);
  void on_ranging(uint16_t irq, uint32_t hal_tick);
  void on_recover(uint16_t irq, uint32_t hal_tick);
};
