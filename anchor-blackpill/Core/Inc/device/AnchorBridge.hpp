#pragma once

#include <cstdint>
#include <optional>

#include "AckPacketIn.hpp"
#include "SX1280Constants.hpp"
#include "SX1280Device.hpp"
#include "stm32h5xx_hal_tim.h"

#if !defined(ANCHOR_ADDRESS) || !defined(ANCHOR_X_CM) || !defined(ANCHOR_Y_CM) || !defined(ANCHOR_Z_CM)
#error "ANCHOR_ADDRESS / ANCHOR_X_CM / ANCHOR_Y_CM / ANCHOR_Z_CM must be defined (see anchor-blackpill/CMakeLists.txt)"
#endif

static_assert(ANCHOR_ADDRESS >= LORA_BEACON_PROTOCOL::RANGING_ADDRESS_BLOCK_BASE,
              "ANCHOR_ADDRESS must be within the discovery block (>= RANGING_ADDRESS_BLOCK_BASE)");
static_assert(ANCHOR_X_CM >= INT16_MIN && ANCHOR_X_CM <= INT16_MAX, "ANCHOR_X_CM must fit int16_t (+-327 m)");
static_assert(ANCHOR_Y_CM >= INT16_MIN && ANCHOR_Y_CM <= INT16_MAX, "ANCHOR_Y_CM must fit int16_t (+-327 m)");
static_assert(ANCHOR_Z_CM >= INT16_MIN && ANCHOR_Z_CM <= INT16_MAX, "ANCHOR_Z_CM must fit int16_t (+-327 m)");

static constexpr uint32_t ACK_SLOT_DELAY_MS =
  (ANCHOR_ADDRESS - LORA_BEACON_PROTOCOL::RANGING_ADDRESS_BLOCK_BASE) * LORA_BEACON_PROTOCOL::ANCHOR_ACK_SLOT_WIDTH_MS;

static constexpr uint32_t RECOVERY_DELAY_MS = 1000;

static constexpr uint8_t RADIO_RECOVER_MAX_RETRIES = 10;
static constexpr uint8_t NRESET_RECOVER_MAX_RETRIES = 5;

static constexpr uint16_t RADIO_SUCCESS = 0x1FF;
static constexpr uint16_t RANGING_SUCCESS = 0x7FF;
static constexpr uint16_t ACK_SUCCESS = 0xF;

enum class Mode { LISTENING, RANGING, ACK_REQUESTED, ACK_IN_PROGRESS, RECOVER };

class AnchorBridge {
  //represents how much time can anchor spend in mode i.e. window durations
  struct Latch {
    uint32_t ack{ACK_SLOT_DELAY_MS};
    uint32_t ranging{LORA_BEACON_PROTOCOL::DEFAULT_RANGING_WINDOW_MS};
  };

  struct Recover {
    uint8_t count{0};
    uint32_t last_fail_tick{0};
  };

public:
  AnchorBridge(SPI_HandleTypeDef* SPI_port_,
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
  {
    device.NRESET_reset();
  }

  inline Mode get_mode() { return mode; }
  inline uint8_t failed_recovery_count() { return recover.count; }

  void step(volatile uint8_t& dio1_flag, volatile uint8_t& tim_flag);

  uint16_t to_radio();
  uint16_t send_ack();

  HAL_StatusTypeDef get_status(SX1280Device::SX1280_Status* sta_out);

private:
  SX1280Device device;
  TIM_HandleTypeDef* timer;

  Mode mode{Mode::RECOVER};
  Latch latch{};
  Recover recover{};

  void arm_timer(uint32_t ms);
  void disarm_timer();

  // sets mode = RECOVER and arms a near-immediate first attempt -- every RECOVER transition must go through
  // this, not a raw assignment, or on_recover() never gets its first timer-gated dispatch
  void try_recover(uint32_t tick);

  // shared tails for on_recover()'s success/failure paths
  void recover_success(uint32_t tick);
  void recover_retry(uint32_t tick, const char* reason);

  // state handlers
  void on_listen(uint16_t irq, uint32_t hal_tick);
  void on_ack_requested(uint16_t irq, uint32_t hal_tick);
  void on_ack_in_progress(uint16_t irq, uint32_t hal_tick);
  void on_ack_done(uint16_t irq, uint32_t hal_tick);
  void on_ranging(uint16_t irq, bool timer_event, uint32_t hal_tick);
  void on_recover(bool timer_event, uint32_t hal_tick);

  uint16_t to_ranging();

  std::optional<AckPacketIn> get_ack_packet();

  HAL_StatusTypeDef get_irq_mask(uint16_t* mask_out);

  HAL_StatusTypeDef clear_irq_mask(SX1280Device::SX1280_Status* sta_out);
};
