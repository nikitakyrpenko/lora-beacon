#include <array>
#include <cstdint>
#include <optional>

#include "SynAckPacket.hpp"
#include "AnchorBridge.hpp"
#include "SynPacket.hpp"
#include "AckPacket.hpp"
#include "ByteOrder.hpp"
#include "SX1280Constants.hpp"
#include "SX1280Device.hpp"
#include "stm32h5xx_hal.h"
#include "stm32h5xx_hal_def.h"
#include "stm32h5xx_hal_tim.h"

#ifdef DEBUG_ANCHOR
#include <cstdio>
#define ANCHOR_LOG(...) printf(__VA_ARGS__)
#else
#define ANCHOR_LOG(...)
#endif

namespace {

bool step_ok(HAL_StatusTypeDef hal, const SX1280Device::SX1280_Status& sta)
{
  return hal == HAL_OK &&
         (sta.command_status == SX1280Device::CommandStatus::COMMAND_SUCCESS ||
          sta.command_status == SX1280Device::CommandStatus::RESERVED || sta.command_status == SX1280Device::CommandStatus::DATA_AVAILABLE);
}

void log_step_failure(const char* func, uint16_t mask, HAL_StatusTypeDef hal, const SX1280Device::SX1280_Status& sta)
{
  ANCHOR_LOG("[%lu] %s failed: hal=%d circuit_mode=%d cmd_status=%d busy=%d mask=0x%X\r\n",
             (unsigned long)HAL_GetTick(),
             func,
             static_cast<int>(hal),
             static_cast<int>(sta.circuit_mode),
             static_cast<int>(sta.command_status),
             static_cast<int>(sta.busy),
             mask);
}

bool is_wake_word_matches(const SynPacket& ack)
{
  for (uint8_t i = 0; i < LORA_BEACON_PROTOCOL::WAKE_WORD_LEN; ++i) {
    if (ack.wake_word[i] != LORA_BEACON_PROTOCOL::WAKE_WORD[i]) {
      return false;
    }
  }
  return true;
}

uint16_t clamp_ranging_window(uint16_t window_ms)
{
  uint16_t dur = window_ms;

  if (window_ms < LORA_BEACON_PROTOCOL::RANGING_WINDOW_MIN_MS) {
    dur = LORA_BEACON_PROTOCOL::RANGING_WINDOW_MIN_MS;
  }
  else if (window_ms > LORA_BEACON_PROTOCOL::RANGING_WINDOW_MAX_MS) {
    dur = LORA_BEACON_PROTOCOL::RANGING_WINDOW_MAX_MS;
  }
  return dur;
}

}  // namespace

AnchorBridge::AnchorBridge(SPI_HandleTypeDef* SPI_port_,
                           TIM_HandleTypeDef* TIM_timer_,
                           GPIO_TypeDef* BUSY_GPIO_port_,
                           GPIO_TypeDef* NSS_GPIO_port_,
                           GPIO_TypeDef* NRESET_GPIO_port_,
                           GPIO_TypeDef* TCXOEN_GPIO_port_,
                           uint16_t BUSY_pin_,
                           uint16_t NSS_pin_,
                           uint16_t NRESET_pin_,
                           uint16_t TCXOEN_pin_)
  : device(SPI_port_, BUSY_GPIO_port_, NSS_GPIO_port_, NRESET_GPIO_port_, TCXOEN_GPIO_port_, BUSY_pin_, NSS_pin_, NRESET_pin_, TCXOEN_pin_)
  , timer(TIM_timer_)
{
  if (device.NRESET_reset()) {
    ANCHOR_LOG("[%lu] NRESET compeleted\r\n", (unsigned long)HAL_GetTick());
  }

  //initial setup need to manually trigget recover
  try_recover(HAL_GetTick());
}

HAL_StatusTypeDef AnchorBridge::clear_irq_mask(SX1280Device::SX1280_Status* sta_out)
{
  uint8_t CLEAR_IRQ_MASK[2] = {0xFF, 0xFF};
  return device.SPI_write(&SX1280_OPERATIONS::CLEAR_IRQ_STATUS_OP_CODE, CLEAR_IRQ_MASK, nullptr, 2, sta_out);
}

uint16_t AnchorBridge::to_radio(bool duty_cycle)
{
  uint16_t mask = 0;
  SX1280Device::SX1280_Status sta{};
  HAL_StatusTypeDef hal;

  // drop to standby before reconfiguring
  hal = device.SPI_write(&SX1280_OPERATIONS::SET_STANDBY_OP_CODE, &SX1280_VALUES::STDBY_RC_STAND_BY, nullptr, 1, &sta);
  if (!step_ok(hal, sta)) {
    log_step_failure(__func__, mask, hal, sta);
    return mask;
  }
  mask |= (1u << 0);

  // select LoRa packet type
  hal = device.SPI_write(&SX1280_OPERATIONS::SET_PACKET_TYPE_OP_CODE, &SX1280_VALUES::PACKET_TYPE_LORA, nullptr, 1, &sta);
  if (!step_ok(hal, sta)) {
    log_step_failure(__func__, mask, hal, sta);
    return mask;
  }
  mask |= (1u << 1);

  // set carrier frequency
  hal = device.SPI_write(&SX1280_OPERATIONS::SET_FREQUENCY_OP_CODE, SX1280_VALUES::RF_FREQUENCY_BYTES, nullptr, 3, &sta);
  if (!step_ok(hal, sta)) {
    log_step_failure(__func__, mask, hal, sta);
    return mask;
  }
  mask |= (1u << 2);

  // set TX/RX buffer base addresses
  hal = device.SPI_write(&SX1280_OPERATIONS::SET_BUFFER_BASE_ADDRESS_OP_CODE, SX1280_VALUES::BUFFER_BASE_ADDRESS, nullptr, 2, &sta);
  if (!step_ok(hal, sta)) {
    log_step_failure(__func__, mask, hal, sta);
    return mask;
  }
  mask |= (1u << 3);

  // set SF7/BW1600/CR4·5 modulation
  hal = device.SPI_write(&SX1280_OPERATIONS::SET_MODULATION_OP_CODE, SX1280_VALUES::MODULATION_PARAMS_SF7, nullptr, 3, &sta);
  if (!step_ok(hal, sta)) {
    log_step_failure(__func__, mask, hal, sta);
    return mask;
  }
  mask |= (1u << 4);

  // required register fixup for SF7/SF8
  hal = device.SPI_write(&SX1280_OPERATIONS::WRITE_REGISTER_OP_CODE, SX1280_VALUES::SF_7_FIXUP_WRITE, nullptr, 3, &sta);
  if (!step_ok(hal, sta)) {
    log_step_failure(__func__, mask, hal, sta);
    return mask;
  }
  mask |= (1u << 5);

  // configure packet params to expect a wake-payload-sized packet
  hal = device.SPI_write(&SX1280_OPERATIONS::SET_PACKET_PARAMS_OP_CODE,
                         LORA_BEACON_PROTOCOL::WAKE_PACKET_PARAM_PAY,
                         nullptr,
                         LORA_BEACON_PROTOCOL::WAKE_PACKET_PARAM_LEN,
                         &sta);
  if (!step_ok(hal, sta)) {
    log_step_failure(__func__, mask, hal, sta);
    return mask;
  }
  mask |= (1u << 6);

  // route RxDone interrupt to DIO1
  hal = device.SPI_write(&SX1280_OPERATIONS::SET_DIO_IRQ_PARAMS_OP_CODE, LORA_BEACON_PROTOCOL::LISTEN_IRQ_MASK, nullptr, 8, &sta);
  if (!step_ok(hal, sta)) {
    log_step_failure(__func__, mask, hal, sta);
    return mask;
  }
  mask |= (1u << 7);

  hal = device.SPI_write(&SX1280_OPERATIONS::SET_LONG_PREAMBLE_OP_CODE, &SX1280_VALUES::LONG_PREAMBLE_DISABLE, nullptr, 1, &sta);
  if (!step_ok(hal, sta)) {
    log_step_failure(__func__, mask, hal, sta);
    return mask;
  }
  mask |= (1u << 8);

  // put Sx1280 into sniff mode with rx window and sleep
  if (duty_cycle) {
    hal =
      device.SPI_write(&SX1280_OPERATIONS::SET_RX_DUTY_CYCLE_OP_CODE, SX1280_VALUES::ANCHOR_IDLE_RX_DUTY_CYCLE_PARAMS, nullptr, 5, &sta);
  }
  else {
    hal = device.SPI_write(&SX1280_OPERATIONS::SET_RX_OP_CODE, LORA_BEACON_PROTOCOL::RX_CONTINUOUS_PARAMS, nullptr, 3, &sta);
  }
  if (!step_ok(hal, sta)) {
    log_step_failure(__func__, mask, hal, sta);
    return mask;
  }
  mask |= (1u << 9);

  return mask;
}

uint16_t AnchorBridge::to_ranging()
{
  uint16_t mask = 0;
  SX1280Device::SX1280_Status sta{};
  HAL_StatusTypeDef hal;

  // select ranging packet type
  hal = device.SPI_write(&SX1280_OPERATIONS::SET_PACKET_TYPE_OP_CODE, &SX1280_VALUES::PACKET_TYPE_RANGING, nullptr, 1, &sta);
  if (!step_ok(hal, sta)) {
    log_step_failure(__func__, mask, hal, sta);
    return mask;
  }
  mask |= (1u << 0);

  // set carrier frequency
  hal = device.SPI_write(&SX1280_OPERATIONS::SET_FREQUENCY_OP_CODE, SX1280_VALUES::RF_FREQUENCY_BYTES, nullptr, 3, &sta);
  if (!step_ok(hal, sta)) {
    log_step_failure(__func__, mask, hal, sta);
    return mask;
  }
  mask |= (1u << 1);

  // set SF7/BW1600/CR4·5 modulation
  hal = device.SPI_write(&SX1280_OPERATIONS::SET_MODULATION_OP_CODE, SX1280_VALUES::MODULATION_PARAMS_SF7, nullptr, 3, &sta);
  if (!step_ok(hal, sta)) {
    log_step_failure(__func__, mask, hal, sta);
    return mask;
  }
  mask |= (1u << 2);

  // required register fixup for SF7/SF8
  hal = device.SPI_write(&SX1280_OPERATIONS::WRITE_REGISTER_OP_CODE, SX1280_VALUES::SF_7_FIXUP_WRITE, nullptr, 3, &sta);
  if (!step_ok(hal, sta)) {
    log_step_failure(__func__, mask, hal, sta);
    return mask;
  }
  mask |= (1u << 3);

  // configure packet params for the ranging exchange
  hal = device.SPI_write(&SX1280_OPERATIONS::SET_PACKET_PARAMS_OP_CODE, SX1280_VALUES::RANGING_PACKET_PARAMS, nullptr, 7, &sta);
  if (!step_ok(hal, sta)) {
    log_step_failure(__func__, mask, hal, sta);
    return mask;
  }
  mask |= (1u << 4);

  // set this anchor's own ranging address
  hal = device.SPI_write(&SX1280_OPERATIONS::WRITE_REGISTER_OP_CODE,
                         ByteOrder::register_write_u32(SX1280_VALUES::REG_RANGING_SLAVE_OWN_ADDR, ANCHOR_ADDRESS).data(),
                         nullptr,
                         6,
                         &sta);
  if (!step_ok(hal, sta)) {
    log_step_failure(__func__, mask, hal, sta);
    return mask;
  }
  mask |= (1u << 5);

  // set ranging address check length
  hal = device.SPI_write(&SX1280_OPERATIONS::WRITE_REGISTER_OP_CODE, SX1280_VALUES::RANGING_ADDR_CHECK_LEN_8BIT, nullptr, 3, &sta);
  if (!step_ok(hal, sta)) {
    log_step_failure(__func__, mask, hal, sta);
    return mask;
  }
  mask |= (1u << 6);

  // write RxTx-delay calibration offset
  hal = device.SPI_write(&SX1280_OPERATIONS::WRITE_REGISTER_OP_CODE, SX1280_VALUES::RANGING_CALIBRATION_WRITE, nullptr, 4, &sta);
  if (!step_ok(hal, sta)) {
    log_step_failure(__func__, mask, hal, sta);
    return mask;
  }
  mask |= (1u << 7);

  // set ranging role to slave
  hal = device.SPI_write(&SX1280_OPERATIONS::SET_RANGING_ROLE_OP_CODE, &SX1280_VALUES::RANGING_ROLE_SLAVE, nullptr, 1, &sta);
  if (!step_ok(hal, sta)) {
    log_step_failure(__func__, mask, hal, sta);
    return mask;
  }
  mask |= (1u << 8);

  // route ranging interrupts to DIO1
  hal = device.SPI_write(&SX1280_OPERATIONS::SET_DIO_IRQ_PARAMS_OP_CODE, SX1280_VALUES::RANGING_SLAVE_IRQ_MASK, nullptr, 8, &sta);
  if (!step_ok(hal, sta)) {
    log_step_failure(__func__, mask, hal, sta);
    return mask;
  }
  mask |= (1u << 9);

  // continuous RX for the ranging slave
  hal = device.SPI_write(&SX1280_OPERATIONS::SET_RX_OP_CODE, LORA_BEACON_PROTOCOL::RANGING_SLAVE_RX_PARAMS, nullptr, 3, &sta);
  if (!step_ok(hal, sta)) {
    log_step_failure(__func__, mask, hal, sta);
    return mask;
  }
  mask |= (1u << 10);

  return mask;
}

bool AnchorBridge::rearm_rx()
{
  SX1280Device::SX1280_Status sta{};

  HAL_StatusTypeDef hal =
    device.SPI_write(&SX1280_OPERATIONS::SET_RX_OP_CODE, LORA_BEACON_PROTOCOL::RX_CONTINUOUS_PARAMS, nullptr, 3, &sta);
  if (!step_ok(hal, sta)) {
    log_step_failure(__func__, 0, hal, sta);
    return false;
  }

  return true;
}

uint16_t AnchorBridge::send_ack()
{
  const SynAckPacket ack{ANCHOR_ADDRESS, ANCHOR_X_CM, ANCHOR_Y_CM, ANCHOR_Z_CM};

  std::array<uint8_t, 1 + LORA_BEACON_PROTOCOL::WAKE_ACK_PAYLOAD_LEN> payload = {
    0x00, LORA_BEACON_PROTOCOL::WAKE_ACK[0], LORA_BEACON_PROTOCOL::WAKE_ACK[1]};

  ack.serialize(payload.data() + 1 + sizeof(LORA_BEACON_PROTOCOL::WAKE_ACK));

  uint16_t mask = 0;
  SX1280Device::SX1280_Status sta{};
  HAL_StatusTypeDef hal;

  // configure packet params for the ack payload length
  hal = device.SPI_write(&SX1280_OPERATIONS::SET_PACKET_PARAMS_OP_CODE, LORA_BEACON_PROTOCOL::WAKE_ACK_PACKET_PARAMS, nullptr, 7, &sta);
  if (!step_ok(hal, sta)) {
    log_step_failure(__func__, mask, hal, sta);
    return mask;
  }
  mask |= (1u << 0);

  // write the ack magic + this anchor's own address + position into the TX buffer
  hal = device.SPI_write(&SX1280_OPERATIONS::WRITE_BUFFER_OP_CODE, payload.data(), nullptr, static_cast<uint16_t>(sizeof(payload)), &sta);
  if (!step_ok(hal, sta)) {
    log_step_failure(__func__, mask, hal, sta);
    return mask;
  }
  mask |= (1u << 1);

  // no IRQ step: the LISTENING routing set by to_radio() (LISTEN_IRQ_MASK) already includes TX_DONE

  // transmit the ack
  hal = device.SPI_write(&SX1280_OPERATIONS::SET_TX_OP_CODE, LORA_BEACON_PROTOCOL::TX_SINGLE_SHOT_PARAMS, nullptr, 3, &sta);
  if (!step_ok(hal, sta)) {
    log_step_failure(__func__, mask, hal, sta);
    return mask;
  }
  mask |= (1u << 2);

  ANCHOR_LOG("[%lu] send_ranging_slave_ack mask=0x%X (full=0x%X)\r\n", (unsigned long)HAL_GetTick(), mask, ACK_SUCCESS);

  return mask;
}

HAL_StatusTypeDef AnchorBridge::read_rx_length(uint8_t* len_out)
{
  SX1280Device::SX1280_Status sta{};

  constexpr uint8_t BUFF_STA_SIZE = 3;
  uint8_t tx_buff_sta[BUFF_STA_SIZE] = {};
  uint8_t rx_buff_sta[BUFF_STA_SIZE] = {};

  // GetRxBufferStatus -> [status][payload length][start pointer]
  HAL_StatusTypeDef hal = device.SPI_write(&SX1280_OPERATIONS::READ_BUFFER_STATUS_OP_CODE, tx_buff_sta, rx_buff_sta, BUFF_STA_SIZE, &sta);
  if (!step_ok(hal, sta)) {
    ANCHOR_LOG("[%lu] rx length read failed, hal=%d\r\n", (unsigned long)HAL_GetTick(), static_cast<int>(hal));
    return hal == HAL_OK ? HAL_ERROR : hal;
  }

  *len_out = rx_buff_sta[1];
  return HAL_OK;
}

std::optional<SynPacket> AnchorBridge::get_syn_packet()
{
  SX1280Device::SX1280_Status sta{};

  constexpr uint8_t BUFF_STA_SIZE = 3;
  uint8_t tx_buff_sta[BUFF_STA_SIZE] = {};
  uint8_t rx_buff_sta[BUFF_STA_SIZE] = {};

  // read RX buffer status -> length + start
  HAL_StatusTypeDef hal = device.SPI_write(&SX1280_OPERATIONS::READ_BUFFER_STATUS_OP_CODE, tx_buff_sta, rx_buff_sta, BUFF_STA_SIZE, &sta);
  if (!step_ok(hal, sta)) {
    ANCHOR_LOG("[%lu] buffer read failed, hal=%d\r\n", (unsigned long)HAL_GetTick(), static_cast<int>(hal));
    return std::nullopt;
  }

  uint8_t len = rx_buff_sta[1];
  uint8_t beg = rx_buff_sta[2];

  // if value of buffer shorter or longer then WAKE_PAYLOAD_LEN -> return early
  if (len != LORA_BEACON_PROTOCOL::WAKE_PAYLOAD_LEN) {
    ANCHOR_LOG(
      "[%lu] unexpected payload len=%u (expected %u)\r\n", (unsigned long)HAL_GetTick(), len, LORA_BEACON_PROTOCOL::WAKE_PAYLOAD_LEN);
    return std::nullopt;
  }

  constexpr uint8_t BUFFER_OFFSET_SIZE = 2;  // the buffer offset to start reading from + a mandatory dummy value
  constexpr uint8_t BUFF_READ_SIZE = BUFFER_OFFSET_SIZE + LORA_BEACON_PROTOCOL::WAKE_PAYLOAD_LEN;

  uint8_t tx_buff_read[BUFF_READ_SIZE] = {beg};
  uint8_t rx_buff_read[BUFF_READ_SIZE] = {};

  // read the received payload
  hal = device.SPI_write(&SX1280_OPERATIONS::READ_BUFFER_OP_CODE, tx_buff_read, rx_buff_read, static_cast<uint16_t>(BUFF_READ_SIZE), &sta);
  if (!step_ok(hal, sta)) {
    ANCHOR_LOG("[%lu] payload read failed, hal=%d\r\n", (unsigned long)HAL_GetTick(), static_cast<int>(hal));
    return std::nullopt;
  }

  SynPacket ack = SynPacket::parse(rx_buff_read + BUFFER_OFFSET_SIZE);
  return ack;
};

std::optional<AckPacket> AnchorBridge::get_ack_packet()
{
  SX1280Device::SX1280_Status sta{};

  constexpr uint8_t BUFF_STA_SIZE = 3;
  uint8_t tx_buff_sta[BUFF_STA_SIZE] = {};
  uint8_t rx_buff_sta[BUFF_STA_SIZE] = {};

  // read RX buffer status -> length + start
  HAL_StatusTypeDef hal = device.SPI_write(&SX1280_OPERATIONS::READ_BUFFER_STATUS_OP_CODE, tx_buff_sta, rx_buff_sta, BUFF_STA_SIZE, &sta);
  if (!step_ok(hal, sta)) {
    ANCHOR_LOG("[%lu] buffer read failed, hal=%d\r\n", (unsigned long)HAL_GetTick(), static_cast<int>(hal));
    return std::nullopt;
  }

  uint8_t len = rx_buff_sta[1];
  uint8_t beg = rx_buff_sta[2];

  // if value of buffer shorter or longer then ACK_PAYLOAD_LEN -> return early
  if (len != LORA_BEACON_PROTOCOL::ACK_PAYLOAD_LEN) {
    ANCHOR_LOG(
      "[%lu] unexpected payload len=%u (expected %u)\r\n", (unsigned long)HAL_GetTick(), len, LORA_BEACON_PROTOCOL::ACK_PAYLOAD_LEN);
    return std::nullopt;
  }

  constexpr uint8_t BUFFER_OFFSET_SIZE = 2;  // the buffer offset to start reading from + a mandatory dummy value
  constexpr uint8_t BUFF_READ_SIZE = BUFFER_OFFSET_SIZE + LORA_BEACON_PROTOCOL::ACK_PAYLOAD_LEN;

  uint8_t tx_buff_read[BUFF_READ_SIZE] = {beg};
  uint8_t rx_buff_read[BUFF_READ_SIZE] = {};

  // read the received payload
  hal = device.SPI_write(&SX1280_OPERATIONS::READ_BUFFER_OP_CODE, tx_buff_read, rx_buff_read, static_cast<uint16_t>(BUFF_READ_SIZE), &sta);
  if (!step_ok(hal, sta)) {
    ANCHOR_LOG("[%lu] payload read failed, hal=%d\r\n", (unsigned long)HAL_GetTick(), static_cast<int>(hal));
    return std::nullopt;
  }

  // the caller checks word_matches() and selects(own low byte)
  return AckPacket::parse(rx_buff_read + BUFFER_OFFSET_SIZE);
}

HAL_StatusTypeDef AnchorBridge::get_status(SX1280Device::SX1280_Status* sta_out)
{
  uint8_t tx_status[1] = {0x00};
  return device.SPI_write(&SX1280_OPERATIONS::GET_STATUS_OP_CODE, tx_status, nullptr, 1, sta_out);
}

HAL_StatusTypeDef AnchorBridge::get_irq_mask(uint16_t* mask_out)
{
  SX1280Device::SX1280_Status sta{};

  constexpr uint8_t IRQ_STA_SIZE = 3;
  uint8_t tx_irq[IRQ_STA_SIZE] = {};
  uint8_t rx_irq[IRQ_STA_SIZE] = {};

  HAL_StatusTypeDef hal = device.SPI_write(&SX1280_OPERATIONS::GET_IRQ_STATUS_OP_CODE, tx_irq, rx_irq, IRQ_STA_SIZE, &sta);

  if (hal != HAL_OK) {
    *mask_out = 0x0000;
    return hal;
  }

  *mask_out = ByteOrder::get_u16(&rx_irq[1]);
  return hal;
}

void AnchorBridge::arm_timer(uint32_t ms)
{
  HAL_TIM_Base_Stop_IT(timer);

  __HAL_TIM_CLEAR_FLAG(timer, TIM_FLAG_UPDATE);
  __HAL_TIM_SET_COUNTER(timer, 0);
  __HAL_TIM_SET_AUTORELOAD(timer, ms - 1);
  HAL_TIM_Base_Start_IT(timer);
}

void AnchorBridge::disarm_timer()
{
  HAL_TIM_Base_Stop_IT(timer);
}

void AnchorBridge::try_recover(uint32_t tick)
{
  mode = Mode::RECOVER;
  ANCHOR_LOG("[%lu] entering RECOVER\r\n", (unsigned long)tick);
  arm_timer(10);  // trigger timer
}

void AnchorBridge::on_idle(bool dio1_event, uint32_t tick)
{
  if (!dio1_event) {
    return;
  }

  // hack to call reset on the SX1280 chip after DIO1 trigger done
  if (device.NRESET_reset() && to_radio() == RADIO_SUCCESS) {
    mode = Mode::LISTENING;
    arm_timer(ACK_LISTENING_WINDOW_MS);
    ANCHOR_LOG("[%lu] on_idle: duty-cycle wake, reset radio, LISTENING\r\n", (unsigned long)tick);
    return;
  }
  try_recover(tick);
}

void AnchorBridge::on_listen(uint16_t irq, bool timer_event, uint32_t tick)
{
  // listening window expired -> fallback to IDLE's power-saving duty-cycle RX
  if (timer_event) {
    disarm_timer();
    if (to_radio(/*duty_cycle=*/true) != RADIO_SUCCESS) {
      ANCHOR_LOG("[%lu] on_listen: to_radio(duty_cycle) failed\r\n", (unsigned long)tick);
      try_recover(tick);
      return;
    }
    mode = Mode::IDLE;
    ANCHOR_LOG("[%lu] on_listen: LISTEN window expired\r\n", (unsigned long)tick);
    return;
  }

  // a corrupted packet still ends the single-shot RX, so each error is its own way to RECOVER (which re-arms RX)
  if (irq & SX1280_VALUES::IRQ_BIT_HEADER_ERROR) {
    ANCHOR_LOG("[%lu] on_listen: header error (irq=0x%X)\r\n", (unsigned long)tick, irq);
    rearm_rx();
    return;
  }

  if (irq & SX1280_VALUES::IRQ_BIT_CRC_ERROR) {
    ANCHOR_LOG("[%lu] on_listen: CRC error (irq=0x%X)\r\n", (unsigned long)tick, irq);
    rearm_rx();
    return;
  }

  if (irq & SX1280_VALUES::IRQ_BIT_RX_DONE) {
    uint8_t len = 0;
    if (read_rx_length(&len) != HAL_OK) {
      try_recover(tick);
      return;
    }

    // SynPacket
    if (len == LORA_BEACON_PROTOCOL::WAKE_PAYLOAD_LEN) {
      const std::optional<SynPacket> syn = get_syn_packet();
      if (!syn) {
        try_recover(tick);  // the length matched, so this is an SPI failure
        return;
      }

      if (is_wake_word_matches(*syn)) {
        mode = Mode::ACK_REQUESTED;
        ANCHOR_LOG("[%lu] wake word matched (ack delay %lums)\r\n", (unsigned long)tick, (unsigned long)latch.ack);
        arm_timer(latch.ack);
        return;
      }

      ANCHOR_LOG("[%lu] on_listen: wake_word mismatch (irq=0x%X)\r\n", (unsigned long)tick, irq);
      rearm_rx();
      return;
    }

    // AckPacket
    if (len == LORA_BEACON_PROTOCOL::ACK_PAYLOAD_LEN) {
      const std::optional<AckPacket> ack = get_ack_packet();
      if (!ack) {
        try_recover(tick);
        return;
      }

      if (ack->word_matches() && ack->selects(static_cast<uint8_t>(ANCHOR_ADDRESS & 0xFF))) {
        latch.ranging = clamp_ranging_window(ack->ranging_window_ms);
        ANCHOR_LOG("[%lu] ack matched (ranging window %lums)\r\n", (unsigned long)tick, (unsigned long)latch.ranging);

        const uint16_t r = to_ranging();
        if (r != RANGING_SUCCESS) {
          ANCHOR_LOG("[%lu] on_listen: to_ranging_slave failed mask=0x%X (full=0x%X)\r\n", (unsigned long)tick, r, RANGING_SUCCESS);
          try_recover(tick);
          return;
        }

        // the ranging window starts once the slave is armed
        mode = Mode::RANGING;
        arm_timer(RANGING_REQUEST_TIMEOUT_MS);
        ANCHOR_LOG("[%lu] entering RANGING\r\n", (unsigned long)tick);
        return;
      }

      // if beacon selected other anchors but not this to start ranging
      ANCHOR_LOG("[%lu] on_listen: ack not for this anchor (irq=0x%X)\r\n", (unsigned long)tick, irq);
      rearm_rx();
      return;
    }

    // foreign traffic on the same BW -> ignore
    ANCHOR_LOG("[%lu] on_listen: ignored frame, len=%u\r\n", (unsigned long)tick, static_cast<unsigned>(len));
    rearm_rx();
    return;
  }

  //any other case rollback to recover
  try_recover(tick);
}

void AnchorBridge::on_ack_requested(uint16_t irq, uint32_t tick)
{
  static constexpr uint32_t ACK_TX_DONE_TIMEOUT_MS = 50;

  const uint16_t r = send_ack();
  if (r != ACK_SUCCESS) {
    ANCHOR_LOG("[%lu] on_ack_requested: ack send failed mask=0x%X (full=0xF)\r\n", (unsigned long)tick, r);
    try_recover(tick);
    return;
  }
  mode = Mode::ACK_IN_PROGRESS;
  arm_timer(ACK_TX_DONE_TIMEOUT_MS);
}

void AnchorBridge::on_ack_in_progress(uint16_t irq, uint32_t hal_tick)
{
  // SynAck tx done
  if (irq & SX1280_VALUES::IRQ_BIT_TX_DONE) {
    disarm_timer();
    ANCHOR_LOG("[%lu] send_ranging_slave_ack: TX_DONE confirmed (irq_mask=0x%X)\r\n", (unsigned long)hal_tick, irq);
    on_ack_done(irq, hal_tick);
    return;
  }

  //ACK_TX_DONE_TIMEOUT_MS expired (chip can be in a bad state without TX_DONE callback routed to DIO1) -> fallback to Recover
  try_recover(hal_tick);
}

void AnchorBridge::on_ack_done(uint16_t irq, uint32_t tick)
{
  //SynPacket out -> rearm tx for listen Syn/Ack
  if (!rearm_rx()) {
    ANCHOR_LOG("[%lu] on_ack_done: rearm_rx failed\r\n", (unsigned long)tick);
    try_recover(tick);
    return;
  }

  mode = Mode::LISTENING;
  arm_timer(ACK_LISTENING_WINDOW_MS);
  ANCHOR_LOG("[%lu] SynAck sent, LISTENING for the start command\r\n", (unsigned long)tick);
}

void AnchorBridge::on_ranging(uint16_t irq, bool timer_event, uint32_t tick)
{
  // this block triggered by timer, no ranging request done in RANGING_REQUEST_TIMEOUT_MS period or latch.ranging -> fallback to radio
  if (timer_event) {
    const uint16_t r = to_radio();
    if (r != RADIO_SUCCESS) {
      ANCHOR_LOG(
        "[%lu] on_ranging: fallback to_radio on ACK_RESPONSE_MS failed mask=0x%X (full=0x%X)\r\n", (unsigned long)tick, r, RANGING_SUCCESS);
      try_recover(tick);
      return;
    }
    mode = Mode::LISTENING;
    arm_timer(ACK_LISTENING_WINDOW_MS);
    ANCHOR_LOG("[%lu] RANGING window expired, fallback to LISTENING\r\n", (unsigned long)tick);
    return;
  }

  // the slave ranging response has left the antenna or was discarded
  if (irq & SX1280_VALUES::IRQ_BIT_RANGING_SLAVE_RESPONSE_DONE || irq & SX1280_VALUES::IRQ_BIT_RANGING_SLAVE_REQUEST_DISCARD) {
    disarm_timer();
    arm_timer(latch.ranging);  // RX is continuous, so the chip is still listening
    mode = Mode::RANGING;
    ANCHOR_LOG("[%lu] refresh RANGING window\r\n", (unsigned long)tick);
    return;
  }

  if (irq & SX1280_VALUES::IRQ_BIT_HEADER_ERROR) {
    ANCHOR_LOG("[%lu] on_ranging: header error, still listening (irq=0x%X)\r\n", (unsigned long)tick, irq);
    return;
  }

  // nothing to do; anything else is unexpected but not fatal
  ANCHOR_LOG("[%lu] on_ranging: unexpected irq bits set (irq=0x%X)\r\n", (unsigned long)tick, irq);
}

void AnchorBridge::recover_success(uint32_t tick)
{
  recover.count = 0;
  mode = Mode::LISTENING;
  arm_timer(ACK_LISTENING_WINDOW_MS);
  ANCHOR_LOG("[%lu] recovered LISTENING\r\n", (unsigned long)tick);
}

void AnchorBridge::recover_retry(uint32_t tick, const char* reason)
{
  recover.count++;
  arm_timer(RECOVERY_DELAY_MS);
  ANCHOR_LOG("[%lu] on_recover: %s failed (recover.count=%u)\r\n", (unsigned long)tick, reason, static_cast<unsigned>(recover.count));
}

void AnchorBridge::on_recover(bool tim_event, uint32_t tick)
{
  if (!tim_event) {
    return;
  }

  if (recover.count < RADIO_RECOVER_MAX_RETRIES) {
    if (to_radio() == RADIO_SUCCESS) {
      recover_success(tick);
      return;
    }
    recover_retry(tick, "switching to radio");
    return;
  }

  if (device.NRESET_reset()) {
    ANCHOR_LOG("[%lu] on_recover: NRESET success\r\n", (unsigned long)tick);
    if (to_radio() == RADIO_SUCCESS) {
      recover_success(tick);
      return;
    }
    recover_retry(tick, "post-NRESET radio retry");
    return;
  }
  recover_retry(tick, "NRESET");
}

Mode AnchorBridge::step(volatile uint8_t& dio1_flag, volatile uint8_t& tim_flag)
{
  // one place reads and clears the IRQ status; the handlers get the result as a parameter
  uint16_t irq = 0;
  const bool dio1_event = dio1_flag;
  if (dio1_event) {
    dio1_flag = 0;

    get_irq_mask(&irq);

    SX1280Device::SX1280_Status sta{};
    clear_irq_mask(&sta);
  }

  const bool timer_event = tim_flag;
  if (timer_event) {
    tim_flag = 0;
  }

  const uint32_t now = HAL_GetTick();

  switch (mode) {
    case Mode::LISTENING:
      if (dio1_event || timer_event) {
        on_listen(irq, timer_event, now);
      }
      break;
    case Mode::ACK_REQUESTED:
      if (timer_event) {
        on_ack_requested(irq, now);
      }
      break;
    case Mode::ACK_IN_PROGRESS:
      if (dio1_event || timer_event) {
        on_ack_in_progress(irq, now);
      }
      break;
    case Mode::RANGING:
      if (dio1_event || timer_event) {
        on_ranging(irq, timer_event, now);
      }
      break;
    case Mode::RECOVER:
      on_recover(timer_event, now);
      break;
    case Mode::IDLE:
      if (dio1_event || timer_event) {
        on_idle(dio1_event, now);
      }
      break;
  }

  return mode;
}
