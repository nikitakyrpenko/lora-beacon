#pragma once

#include <cstdint>
#include "SX1280Constants.hpp"
#include "AckPacket.hpp"
#include "SX1280Device.hpp"

enum class MODE { RADIO, ACK_LISTEN, RANGING, NONE };

class BeaconBridge {
  SX1280Device device;
  MODE mode;
  uint32_t ranging_target_address;  // anchor the last to_ranging_master() call targeted, only used to tag log lines

public:
  BeaconBridge(SPI_HandleTypeDef* SPI_port_,
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
    , ranging_target_address(0)
  {
    device.NRESET_reset();
  }

  uint16_t to_radio();
  uint16_t send_wake_broadcast();
  bool was_tx_done();

  uint16_t listen_for_ack();
  uint16_t stop_ack_listen();
  bool wake_ack_matched(AckPacket* ack_out);

  uint16_t to_ranging_master(uint32_t target_anchor_address);
  uint16_t send_ranging_request();

  bool read_ranging_result_cm(int32_t* distance_cm_out);
  // no RESULT_VALID bit in irq_mask (timeout, or nothing at all) -- logs which
  void log_ranging_no_result(uint16_t irq_mask);

  HAL_StatusTypeDef get_status(SX1280Device::SX1280_Status* sta_out);

  HAL_StatusTypeDef get_irq_mask(uint16_t* mask_out);
  HAL_StatusTypeDef clear_irq_mask(SX1280Device::SX1280_Status* sta_out);

  inline MODE get_mode() { return mode; }
};
