#pragma once

#include <cstdint>
#include <optional>
#include "AckPacketIn.hpp"
#include "SX1280Constants.hpp"
#include "AckPacket.hpp"
#include "CycleFrame.hpp"
#include "RangeEntry.hpp"
#include "SX1280Device.hpp"
#include "stm32h5xx_hal_tim.h"

enum class Mode { RADIO, ACK_IN_PROGRESS, ACK_REQUESTED, ACK_LISTENING, ACK_RECIEVED, RANGING, RANGING_REQUESTED, RANGING_WAIT, RECOVER };

class BeaconBridge {
  struct AckBag {
    AckPacket anchors[LORA_BEACON_PROTOCOL::EXPECTED_ANCHOR_COUNT] = {};
    uint8_t count = 0;
  };

  struct RangeBag {
    RangeEntry measured[LORA_BEACON_PROTOCOL::EXPECTED_ANCHOR_COUNT] = {};
    uint8_t count = 0;
  };

  struct Recover {
    bool active = false;  // an attempt was already made, so the next one waits RECOVERY_DELAY_MS
    uint8_t count = 0;    // consecutive failed attempts
    uint32_t last_tick = 0;
  };

  SX1280Device device;
  TIM_HandleTypeDef* timer;
  Mode mode;
  uint32_t ranging_target_address;  // anchor the last to_ranging_master() call targeted, only used to tag log lines

  AckBag acks = {};
  RangeBag measures = {};
  Recover recover = {};

  uint8_t cursor = 0;  // global index currently being ranged
  uint8_t misses = 0;  // consecutive ranging timeouts, reset by any result and on entering RANGING

  // outcome of the cycle in progress, and the frame of the last finished one (see take_frame())
  uint32_t cycle_counter;
  uint8_t frame[CycleFrame::MAX_BYTES];
  size_t frame_length;  // 0 = no finished cycle waiting to be taken

public:
  static constexpr uint32_t WAKE_BROADCAST_INTERVAL_MS = 5000;
  // Ack-collect inactivity timeout: armed once on the wake's TX_DONE and restarted by every NEW anchor's ack. Chip RX
  // timeouts (one listen window) don't touch it, so it only expires when no new ack arrived for this long.
  static constexpr uint32_t ACK_COLLECT_TIMEOUT_MS = 250;
  // RECOVER: delay between two attempts, and how many plain to_radio() retries come before an NRESET of the chip
  // Consecutive ranging timeouts after which the beacon gives up and goes back to RADIO to send a new wake. Without it a deaf anchor
  // keeps receiving ranging requests it cannot decode (header errors) and no wake ever reaches it.
  static constexpr uint8_t RANGING_MAX_CONSECUTIVE_MISSES = 20;
  // Idle gap between two passes over all collected anchors (RANGING_WAIT, ended by TIM6). The rate is 1 / (gap + pass duration), so with
  // one anchor (about 6 ms per pass) 100 ms is about 9.4 Hz and 200 ms about 4.9 Hz; it drops slightly as anchors are added.
  static constexpr uint32_t RANGING_PASS_GAP_MS = 200;
  static constexpr uint32_t RECOVERY_DELAY_MS = 1000;
  static constexpr uint8_t RADIO_RECOVER_MAX_RETRIES = 1;
  // Fallback ceiling for one ranging exchange (a bit more than the RANGING_REQUEST_TIMEOUT_MS the request's SetTx
  // arms) in case DIO1 is somehow missed -- a normal exchange resolves via the interrupt within low single-digit ms of
  // the chip deciding RangingMasterResultValid or RangingMasterTimeout, not after this full window.
  static constexpr uint32_t RANGING_RESULT_CEILING_MS = LORA_BEACON_PROTOCOL::RANGING_REQUEST_TIMEOUT_MS + 100;

  BeaconBridge(SPI_HandleTypeDef* SPI_port_,
               TIM_HandleTypeDef* TIM_timer_,
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
    , timer(TIM_timer_)
    , mode(Mode::RECOVER)
    , ranging_target_address(0)
    , cycle_counter(0)
    , frame{}
    , frame_length(0)
  {
    device.NRESET_reset();
  }

  uint16_t to_radio();
  uint16_t to_ranging();

  uint8_t send_ack_request(const AckPacketIn* pack);
  uint8_t ack_listen();
  std::optional<AckPacket> get_ack_response();

  uint8_t start_ranging(uint32_t anchor_id);
  uint8_t get_ranging_result(int32_t* out);

  void arm_timer(uint32_t ms);
  void disarm_timer();

  bool rearm_rx();

  bool register_anchor(const AckPacket* ack);

  // helpers kept from the old flow for the cycle/frame logic and bring-up
  HAL_StatusTypeDef get_status(SX1280Device::SX1280_Status* sta_out);
  void clear_irq();
  void finish_cycle();
  void return_to_idle();

  HAL_StatusTypeDef get_irq_mask(uint16_t* mask_out);
  HAL_StatusTypeDef clear_irq_mask(SX1280Device::SX1280_Status* sta_out);

  void step(volatile uint8_t& dio1_flag, volatile uint8_t& tim_flag);

  size_t take_frame(uint8_t* out, size_t capacity);

  inline Mode get_mode() { return mode; }

  void on_radio(uint16_t irq, bool timer_event, uint32_t tick);
  void on_ack_in_progress(uint16_t irq, bool timer_event, uint32_t tick);
  void on_ack_requested(uint16_t irq, bool timer_event, uint32_t tick);
  void on_ack_listen(uint16_t irq, bool timer_event, uint32_t tick);
  void on_ack_recieved(uint16_t irq, bool timer_event, uint32_t tick);
  void on_ranging(uint16_t irq, bool timer_event, uint32_t tick);
  void on_ranging_requested(uint16_t irq, bool timer_event, uint32_t tick);
  void on_ranging_wait(uint16_t irq, bool timer_event, uint32_t tick);
  void on_recover(uint16_t irq, bool timer_event, uint32_t tick);
};
