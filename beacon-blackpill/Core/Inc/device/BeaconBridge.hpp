#pragma once

#include <cstdint>
#include "SX1280Constants.hpp"
#include "AckPacket.hpp"
#include "SX1280Device.hpp"

enum class MODE { RADIO, ACK_LISTEN, RANGING, NONE };

// Protocol phase of the wake -> collect acks -> range cycle driven by BeaconBridge::step(). Separate from MODE, which
// only records what the chip is currently configured for: the state stays RANGING for the whole loop over the collected
// anchors, while the mode drops to NONE between two of them.
enum class BeaconState {
  IDLE,        // radio mode, broadcasting a wake request every WAKE_BROADCAST_INTERVAL_MS
  COLLECTING,  // listening for anchor acks after a broadcast
  RANGING,     // ranging the collected anchors one at a time
};

class BeaconBridge {
  SX1280Device device;
  MODE mode;
  uint32_t ranging_target_address;  // anchor the last to_ranging_master() call targeted, only used to tag log lines

  // wake -> collect -> range cycle state, advanced only by step()
  BeaconState state;
  AckPacket collected_anchors[LORA_BEACON_PROTOCOL::EXPECTED_ANCHOR_COUNT];
  uint8_t collected_count;
  uint8_t ranging_index;  // index into collected_anchors of the anchor currently being ranged
  uint32_t wake_broadcast_last_tick;
  uint32_t collect_phase_deadline_tick;
  uint32_t ranging_request_sent_tick;

  void clear_irq();
  // back to wake-broadcast radio config, ready for the next cycle
  void return_to_idle();
  // configure as ranging master for one anchor and fire the request; false if the configuration didn't complete
  bool start_ranging(uint32_t anchor_address, volatile uint8_t& dio1_flag);

public:
  static constexpr uint32_t WAKE_BROADCAST_INTERVAL_MS = 5000;
  // Fallback ceiling for one ranging exchange (a bit more than the RANGING_REQUEST_TIMEOUT_MS the request's SetTx
  // arms) in case DIO1 is somehow missed -- a normal exchange resolves via the interrupt within low single-digit ms of
  // the chip deciding RangingMasterResultValid or RangingMasterTimeout, not after this full window.
  static constexpr uint32_t RANGING_RESULT_CEILING_MS = LORA_BEACON_PROTOCOL::RANGING_REQUEST_TIMEOUT_MS + 100;

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
    , state(BeaconState::IDLE)
    , collected_anchors{}
    , collected_count(0)
    , ranging_index(0)
    // in the past (not HAL_GetTick()) so the first step() fires the first wake broadcast immediately, instead of
    // special-casing an extra send outside the loop
    , wake_broadcast_last_tick(HAL_GetTick() - WAKE_BROADCAST_INTERVAL_MS)
    , collect_phase_deadline_tick(0)
    , ranging_request_sent_tick(0)
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

  // Advances the wake -> collect acks -> range cycle by one pass; call it every main-loop iteration. `dio1_flag` is the
  // flag the DIO1 EXTI callback sets -- consumed and cleared here.
  void step(volatile uint8_t& dio1_flag);

  inline MODE get_mode() { return mode; }
  inline BeaconState get_state() { return state; }
};
