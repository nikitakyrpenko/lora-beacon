#include <array>
#include <cstdint>
#include <optional>

#include "BeaconBridge.hpp"
#include "AckPacket.hpp"
#include "AckPacketIn.hpp"
#include "ByteOrder.hpp"
#include "RangeEntry.hpp"
#include "SX1280Constants.hpp"
#include "stm32h5xx_hal_def.h"

// Set at build time (-DDEBUG_BEACON or #define above this include) to turn all BeaconBridge logging on/off in one
// place.
#ifdef DEBUG_BEACON
#include <cstdio>
#define BEACON_LOG(...) printf(__VA_ARGS__)
#else
#define BEACON_LOG(...)
#endif

// Bitmask a multi-step method returns success results
static constexpr uint16_t TO_RADIO_FULL_MASK = (1u << 7) - 1;
static constexpr uint16_t TO_RANGING_FULL_MASK = (1u << 9) - 1;
static constexpr uint16_t SEND_ACK_REQUEST_FULL_MASK = (1u << 4) - 1;
static constexpr uint16_t ACK_LISTEN_FULL_MASK = (1u << 3) - 1;
static constexpr uint16_t START_RANGING_FULL_MASK = (1u << 2) - 1;
static constexpr uint16_t GET_RANGING_RESULT_FULL_MASK = (1u << 7) - 1;

namespace {

bool step_ok(HAL_StatusTypeDef hal, const SX1280Device::SX1280_Status& sta)
{
  return hal == HAL_OK && (sta.command_status == SX1280Device::CommandStatus::COMMAND_SUCCESS ||
                           sta.command_status == SX1280Device::CommandStatus::RESERVED ||
                           sta.command_status == SX1280Device::CommandStatus::DATA_AVAILABLE ||
                           sta.command_status == SX1280Device::CommandStatus::COMMAND_TX_DONE ||
                           sta.command_status == SX1280Device::CommandStatus::COMMAND_TIMEOUT);
}

void log_step_failure(const char* func, uint16_t mask, HAL_StatusTypeDef hal, const SX1280Device::SX1280_Status& sta)
{
  BEACON_LOG("[%lu] %s failed: hal=%d circuit_mode=%d cmd_status=%d busy=%d mask=0x%X\r\n",
             (unsigned long)HAL_GetTick(),
             func,
             static_cast<int>(hal),
             static_cast<int>(sta.circuit_mode),
             static_cast<int>(sta.command_status),
             static_cast<int>(sta.busy),
             mask);
}
}  // namespace

HAL_StatusTypeDef BeaconBridge::get_status(SX1280Device::SX1280_Status* sta_out)
{
  uint8_t tx_status[1] = {0x00};
  return device.SPI_write(&SX1280_OPERATIONS::GET_STATUS_OP_CODE, tx_status, nullptr, 1, sta_out);
}

HAL_StatusTypeDef BeaconBridge::get_irq_mask(uint16_t* mask_out)
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

HAL_StatusTypeDef BeaconBridge::clear_irq_mask(SX1280Device::SX1280_Status* sta_out)
{
  uint8_t CLEAR_IRQ_MASK[2] = {0xFF, 0xFF};
  return device.SPI_write(&SX1280_OPERATIONS::CLEAR_IRQ_STATUS_OP_CODE, CLEAR_IRQ_MASK, nullptr, 2, sta_out);
}

uint16_t BeaconBridge::to_radio()
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

  // set TX output power + ramp time
  hal = device.SPI_write(&SX1280_OPERATIONS::SET_TX_PARAMS_OP_CODE, SX1280_VALUES::TX_PARAMS, nullptr, 2, &sta);
  if (!step_ok(hal, sta)) {
    log_step_failure(__func__, mask, hal, sta);
    return mask;
  }
  mask |= (1u << 6);

  mode = Mode::RADIO;
  BEACON_LOG("[%lu] entering RADIO\r\n", (unsigned long)HAL_GetTick());

  return mask;
}

uint16_t BeaconBridge::to_ranging()
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

  // set ranging address check length (must match the anchor's)
  hal = device.SPI_write(&SX1280_OPERATIONS::WRITE_REGISTER_OP_CODE, SX1280_VALUES::RANGING_ADDR_CHECK_LEN_8BIT, nullptr, 3, &sta);
  if (!step_ok(hal, sta)) {
    log_step_failure(__func__, mask, hal, sta);
    return mask;
  }
  mask |= (1u << 5);

  // write RxTx-delay calibration offset
  hal = device.SPI_write(&SX1280_OPERATIONS::WRITE_REGISTER_OP_CODE, SX1280_VALUES::RANGING_CALIBRATION_WRITE, nullptr, 4, &sta);
  if (!step_ok(hal, sta)) {
    log_step_failure(__func__, mask, hal, sta);
    return mask;
  }
  mask |= (1u << 6);

  // set ranging role to master
  hal = device.SPI_write(&SX1280_OPERATIONS::SET_RANGING_ROLE_OP_CODE, &SX1280_VALUES::RANGING_ROLE_MASTER, nullptr, 1, &sta);
  if (!step_ok(hal, sta)) {
    log_step_failure(__func__, mask, hal, sta);
    return mask;
  }
  mask |= (1u << 7);

  // route ranging interrupts to DIO1 (RANGING_MASTER_RESULT_VALID + RANGING_MASTER_TIMEOUT); the same for every anchor, so set once
  hal = device.SPI_write(&SX1280_OPERATIONS::SET_DIO_IRQ_PARAMS_OP_CODE, SX1280_VALUES::RANGING_MASTER_IRQ_MASK, nullptr, 8, &sta);
  if (!step_ok(hal, sta)) {
    log_step_failure(__func__, mask, hal, sta);
    return mask;
  }
  mask |= (1u << 8);

  BEACON_LOG("[%lu] entering RANGING\r\n", (unsigned long)HAL_GetTick());

  return mask;
}

uint8_t BeaconBridge::send_ack_request(const AckPacketIn* ack_request)
{
  uint16_t mask = 0;
  SX1280Device::SX1280_Status sta{};
  HAL_StatusTypeDef hal;

  // set packet parameters for ack tx
  hal = device.SPI_write(&SX1280_OPERATIONS::SET_PACKET_PARAMS_OP_CODE,
                         LORA_BEACON_PROTOCOL::WAKE_PACKET_PARAM_PAY,
                         nullptr,
                         LORA_BEACON_PROTOCOL::WAKE_PACKET_PARAM_LEN,
                         &sta);
  if (!step_ok(hal, sta)) {
    log_step_failure(__func__, mask, hal, sta);
    return mask;
  }
  mask |= (1u << 0);

  // write the wake word + ranging-window duration into the TX buffer
  constexpr uint8_t WRITE_BUFFER_SIZE = 1 + LORA_BEACON_PROTOCOL::WAKE_PAYLOAD_LEN;
  uint8_t buf[WRITE_BUFFER_SIZE] = {0x00};

  if (!ack_request->serialize(buf + 1, LORA_BEACON_PROTOCOL::WAKE_PAYLOAD_LEN)) {
    BEACON_LOG("[%lu] %s: wake payload serialize failed, mask=0x%X\r\n", (unsigned long)HAL_GetTick(), __func__, mask);
    return mask;
  }
  hal = device.SPI_write(&SX1280_OPERATIONS::WRITE_BUFFER_OP_CODE, buf, nullptr, static_cast<uint16_t>(WRITE_BUFFER_SIZE), &sta);
  if (!step_ok(hal, sta)) {
    log_step_failure(__func__, mask, hal, sta);
    return mask;
  }
  mask |= (1u << 1);

  // set irq callback for TX_DONE or TX_TIMEOUT
  uint16_t irq_mask = SX1280_VALUES::IRQ_BIT_TX_DONE | SX1280_VALUES::IRQ_BIT_RX_TX_TIMEOUT;
  uint8_t irq[LORA_BEACON_PROTOCOL::DIO_IRQ_PARAMS_LEN] = {};
  ByteOrder::put_u16(irq, irq_mask);
  ByteOrder::put_u16(irq + 2, irq_mask);

  hal = device.SPI_write(&SX1280_OPERATIONS::SET_DIO_IRQ_PARAMS_OP_CODE, irq, nullptr, LORA_BEACON_PROTOCOL::DIO_IRQ_PARAMS_LEN, &sta);
  if (!step_ok(hal, sta)) {
    log_step_failure(__func__, mask, hal, sta);
    return mask;
  }
  mask |= (1u << 2);

  // transmit (single-shot, auto-standby on TxDone)
  uint8_t tx_param[3] = {SX1280_VALUES::PERIOD_BASE_1_MS};
  ByteOrder::put_u16(tx_param + 1, BEACON_CONSTANTS::BEACON_ACK_TX_TIMEOUT_MS);

  hal = device.SPI_write(&SX1280_OPERATIONS::SET_TX_OP_CODE, tx_param, nullptr, 3, &sta);
  if (!step_ok(hal, sta)) {
    log_step_failure(__func__, mask, hal, sta);
    return mask;
  }
  mask |= (1u << 3);

  BEACON_LOG("[%lu] ack request sent\r\n", (unsigned long)HAL_GetTick());

  return mask;
}

uint8_t BeaconBridge::ack_listen()
{
  uint16_t mask = 0;
  SX1280Device::SX1280_Status sta{};
  HAL_StatusTypeDef hal;

  // Reuses the frequency/modulation with different ack len
  hal = device.SPI_write(&SX1280_OPERATIONS::SET_PACKET_PARAMS_OP_CODE,
                         LORA_BEACON_PROTOCOL::WAKE_ACK_PACKET_PARAMS,
                         nullptr,
                         LORA_BEACON_PROTOCOL::WAKE_PACKET_PARAM_LEN,
                         &sta);
  if (!step_ok(hal, sta)) {
    log_step_failure(__func__, mask, hal, sta);
    return mask;
  }
  mask |= (1u << 0);

  // route RxDone + RxTimeout interrupts to DIO1
  hal = device.SPI_write(&SX1280_OPERATIONS::SET_DIO_IRQ_PARAMS_OP_CODE, LORA_BEACON_PROTOCOL::ACK_LISTEN_IRQ_MASK, nullptr, 8, &sta);
  if (!step_ok(hal, sta)) {
    log_step_failure(__func__, mask, hal, sta);
    return mask;
  }
  mask |= (1u << 1);

  // start timeout-active RX
  hal = device.SPI_write(&SX1280_OPERATIONS::SET_RX_OP_CODE, LORA_BEACON_PROTOCOL::ACK_LISTEN_RX_PARAMS, nullptr, 3, &sta);
  if (!step_ok(hal, sta)) {
    log_step_failure(__func__, mask, hal, sta);
    return mask;
  }
  mask |= (1u << 2);

  BEACON_LOG("[%lu] entering ACK_LISTEN\r\n", (unsigned long)HAL_GetTick());

  return mask;
}

std::optional<AckPacket> BeaconBridge::get_ack_response()
{
  SX1280Device::SX1280_Status sta{};

  constexpr uint8_t BUFF_STA_SIZE = 3;
  uint8_t tx_buff_sta[BUFF_STA_SIZE] = {};
  uint8_t rx_buff_sta[BUFF_STA_SIZE] = {};

  // read RX buffer status -> length + start
  HAL_StatusTypeDef hal = device.SPI_write(&SX1280_OPERATIONS::READ_BUFFER_STATUS_OP_CODE, tx_buff_sta, rx_buff_sta, BUFF_STA_SIZE, &sta);
  if (!step_ok(hal, sta)) {
    BEACON_LOG("[%lu] buffer read failed, hal=%d\r\n", (unsigned long)HAL_GetTick(), static_cast<int>(hal));
    return std::nullopt;
  }

  uint8_t len = rx_buff_sta[1];
  uint8_t beg = rx_buff_sta[2];

  // if value of buffer shorter or longer then WAKE_PAYLOAD_LEN -> return early
  if (len != LORA_BEACON_PROTOCOL::WAKE_ACK_PAYLOAD_LEN) {
    BEACON_LOG(
      "[%lu] unexpected payload len=%u (expected %u)\r\n", (unsigned long)HAL_GetTick(), len, LORA_BEACON_PROTOCOL::WAKE_ACK_PAYLOAD_LEN);
    return std::nullopt;
  }

  constexpr uint8_t BUFFER_OFFSET_SIZE = 2;  // the buffer offset to start reading from + a mandatory dummy value
  constexpr uint8_t BUF_READ_SIZE = BUFFER_OFFSET_SIZE + LORA_BEACON_PROTOCOL::WAKE_ACK_PAYLOAD_LEN;

  uint8_t tx_buff_read[BUF_READ_SIZE] = {beg, 0x00};
  uint8_t rx_buff_read[BUF_READ_SIZE] = {};

  // read the received payload
  hal = device.SPI_write(&SX1280_OPERATIONS::READ_BUFFER_OP_CODE, tx_buff_read, rx_buff_read, static_cast<uint16_t>(BUF_READ_SIZE), &sta);
  if (!step_ok(hal, sta)) {
    BEACON_LOG("[%lu] payload read failed, hal=%d\r\n", (unsigned long)HAL_GetTick(), static_cast<int>(hal));
    return std::nullopt;
  }

  // check magic if no return early
  const uint8_t* payload = rx_buff_read + BUFFER_OFFSET_SIZE;
  if (payload[0] != LORA_BEACON_PROTOCOL::WAKE_ACK[0] || payload[1] != LORA_BEACON_PROTOCOL::WAKE_ACK[1]) {
    BEACON_LOG("[%lu] ack magic mismatch\r\n", (unsigned long)HAL_GetTick());
    return std::nullopt;
  }

  return AckPacket::parse(payload + sizeof(LORA_BEACON_PROTOCOL::WAKE_ACK));
}

void BeaconBridge::arm_timer(uint32_t ms)
{
  HAL_TIM_Base_Stop_IT(timer);

  __HAL_TIM_CLEAR_FLAG(timer, TIM_FLAG_UPDATE);
  __HAL_TIM_SET_COUNTER(timer, 0);
  __HAL_TIM_SET_AUTORELOAD(timer, ms - 1);
  HAL_TIM_Base_Start_IT(timer);
}

void BeaconBridge::disarm_timer()
{
  HAL_TIM_Base_Stop_IT(timer);
}

bool BeaconBridge::rearm_rx()
{
  SX1280Device::SX1280_Status sta{};

  HAL_StatusTypeDef hal =
    device.SPI_write(&SX1280_OPERATIONS::SET_RX_OP_CODE, LORA_BEACON_PROTOCOL::ACK_LISTEN_RX_PARAMS, nullptr, 3, &sta);
  if (!step_ok(hal, sta)) {
    log_step_failure(__func__, 0, hal, sta);
    return false;
  }

  return true;
}

bool BeaconBridge::register_anchor(const AckPacket* ack)
{
  for (int i = 0; i < acks.count; i++) {
    if (acks.anchors[i].anchor_id == ack->anchor_id) {
      return false;
    }
  }
  acks.anchors[acks.count] = *ack;
  acks.count++;
  return true;
}

uint8_t BeaconBridge::start_ranging(uint32_t anchor_id)
{
  uint8_t mask = 0;
  SX1280Device::SX1280_Status sta{};
  HAL_StatusTypeDef hal;

  ranging_target_address = anchor_id;  // only used to tag log lines

  // set the target anchor's ranging address
  hal = device.SPI_write(&SX1280_OPERATIONS::WRITE_REGISTER_OP_CODE,
                         ByteOrder::register_write_u32(SX1280_VALUES::REG_RANGING_MASTER_TARGET_ADDR, anchor_id).data(),
                         nullptr,
                         6,
                         &sta);
  if (!step_ok(hal, sta)) {
    log_step_failure(__func__, mask, hal, sta);
    return mask;
  }
  mask |= (1u << 0);

  // transmit the ranging request (the DIO1 routing was set once in to_ranging()); the chip's own timeout ends the exchange if the anchor never answers
  hal = device.SPI_write(&SX1280_OPERATIONS::SET_TX_OP_CODE, LORA_BEACON_PROTOCOL::RANGING_REQUEST_TX_PARAMS, nullptr, 3, &sta);
  if (!step_ok(hal, sta)) {
    log_step_failure(__func__, mask, hal, sta);
    return mask;
  }
  mask |= (1u << 1);

  BEACON_LOG("[%lu] anchor 0x%08lX ranging request sent\r\n", (unsigned long)HAL_GetTick(), (unsigned long)anchor_id);
  return mask;
}

uint8_t BeaconBridge::get_ranging_result(int32_t* out)
{
  uint8_t mask = 0;
  SX1280Device::SX1280_Status sta{};
  HAL_StatusTypeDef hal;

  // XOSC standby, required before touching the ranging result registers
  hal = device.SPI_write(&SX1280_OPERATIONS::SET_STANDBY_OP_CODE, &SX1280_VALUES::STDBY_XOSC_STAND_BY, nullptr, 1, &sta);
  if (!step_ok(hal, sta)) {
    log_step_failure(__func__, mask, hal, sta);
    return mask;
  }
  mask |= (1u << 0);

  // read the LoRa memory clock register -- read-modify-write, so its original value is kept for the restore step
  uint8_t read_clock_tx[4] = {static_cast<uint8_t>(SX1280_VALUES::REG_LORA_MEM_CLOCK_ENABLE >> 8),
                              static_cast<uint8_t>(SX1280_VALUES::REG_LORA_MEM_CLOCK_ENABLE & 0xFF),
                              0x00,
                              0x00};
  uint8_t read_clock_rx[4] = {};

  hal = device.SPI_write(&SX1280_OPERATIONS::READ_REGISTER_OP_CODE, read_clock_tx, read_clock_rx, 4, &sta);
  if (!step_ok(hal, sta)) {
    log_step_failure(__func__, mask, hal, sta);
    return mask;
  }
  mask |= (1u << 1);
  uint8_t mem_clock_reg = read_clock_rx[3];

  // enable the LoRa memory clock (bit 1)
  uint8_t write_clock_tx[3] = {static_cast<uint8_t>(SX1280_VALUES::REG_LORA_MEM_CLOCK_ENABLE >> 8),
                               static_cast<uint8_t>(SX1280_VALUES::REG_LORA_MEM_CLOCK_ENABLE & 0xFF),
                               static_cast<uint8_t>(mem_clock_reg | (1u << 1))};
  hal = device.SPI_write(&SX1280_OPERATIONS::WRITE_REGISTER_OP_CODE, write_clock_tx, nullptr, 3, &sta);
  if (!step_ok(hal, sta)) {
    log_step_failure(__func__, mask, hal, sta);
    return mask;
  }
  mask |= (1u << 2);

  // select the debiased result type
  hal = device.SPI_write(&SX1280_OPERATIONS::WRITE_REGISTER_OP_CODE, SX1280_VALUES::RANGING_RESULT_MUX_DEBIASED_WRITE, nullptr, 3, &sta);
  if (!step_ok(hal, sta)) {
    log_step_failure(__func__, mask, hal, sta);
    return mask;
  }
  mask |= (1u << 3);

  // read the 3-byte result
  uint8_t result_tx[6] = {static_cast<uint8_t>(SX1280_VALUES::REG_RANGING_RESULT_MSB >> 8),
                          static_cast<uint8_t>(SX1280_VALUES::REG_RANGING_RESULT_MSB & 0xFF),
                          0x00,
                          0x00,
                          0x00,
                          0x00};
  uint8_t result_rx[6] = {};
  hal = device.SPI_write(&SX1280_OPERATIONS::READ_REGISTER_OP_CODE, result_tx, result_rx, 6, &sta);
  if (!step_ok(hal, sta)) {
    log_step_failure(__func__, mask, hal, sta);
    return mask;
  }
  mask |= (1u << 4);

  int32_t raw_result = static_cast<int32_t>(ByteOrder::get_u24(&result_rx[3]));

  // Restore the LoRa memory clock enable bit to its original
  uint8_t restore_clock_tx[3] = {static_cast<uint8_t>(SX1280_VALUES::REG_LORA_MEM_CLOCK_ENABLE >> 8),
                                 static_cast<uint8_t>(SX1280_VALUES::REG_LORA_MEM_CLOCK_ENABLE & 0xFF),
                                 mem_clock_reg};
  hal = device.SPI_write(&SX1280_OPERATIONS::WRITE_REGISTER_OP_CODE, restore_clock_tx, nullptr, 3, &sta);
  if (!step_ok(hal, sta)) {
    log_step_failure(__func__, mask, hal, sta);
    return mask;
  }
  mask |= (1u << 5);

  // back to RC standby
  hal = device.SPI_write(&SX1280_OPERATIONS::SET_STANDBY_OP_CODE, &SX1280_VALUES::STDBY_RC_STAND_BY, nullptr, 1, &sta);
  if (!step_ok(hal, sta)) {
    log_step_failure(__func__, mask, hal, sta);
    return mask;
  }
  mask |= (1u << 6);

  *out = raw_result * SX1280_VALUES::RANGING_RESULT_TO_CM_MULTIPLIER;

  BEACON_LOG("[%lu] anchor 0x%08lX ranging result: raw=%ld -> %ld cm (mem_clock=0x%02X, mask=0x%X)\r\n",
             (unsigned long)HAL_GetTick(),
             (unsigned long)ranging_target_address,
             (long)raw_result,
             (long)*out,
             mem_clock_reg,
             mask);

  return mask;
}

void BeaconBridge::clear_irq()
{
  SX1280Device::SX1280_Status sta{};
  clear_irq_mask(&sta);
}

void BeaconBridge::on_radio(uint16_t irq, bool timer_event, uint32_t tick)
{
  //can be static consexper
  AckPacketIn ack = {{LORA_BEACON_PROTOCOL::WAKE_WORD[0], LORA_BEACON_PROTOCOL::WAKE_WORD[1]},
                     LORA_BEACON_PROTOCOL::DEFAULT_RANGING_WINDOW_MS};

  uint8_t r = send_ack_request(&ack);
  if (r != SEND_ACK_REQUEST_FULL_MASK) {
    BEACON_LOG("[%lu] on_radio: send_ack_request failed\r\n", (unsigned long)tick);
    mode = Mode::RECOVER;
    return;
  }

  mode = Mode::ACK_IN_PROGRESS;
}

void BeaconBridge::on_ack_in_progress(uint16_t irq, bool timer_event, uint32_t tick)
{
  if (irq & SX1280_VALUES::IRQ_BIT_RX_TX_TIMEOUT) {
    mode = Mode::RADIO;
    BEACON_LOG("[%lu] on_ack_in_progress: TX timeout (irq=0x%X)\r\n", (unsigned long)tick, irq);
    return;
  }
  if (irq & SX1280_VALUES::IRQ_BIT_TX_DONE) {
    arm_timer(ACK_COLLECT_TIMEOUT_MS);  // armed once per cycle, see ACK_COLLECT_TIMEOUT_MS
    mode = Mode::ACK_REQUESTED;
    BEACON_LOG("[%lu] on_ack_in_progress: TX Done (irq=0x%X)\r\n", (unsigned long)tick, irq);
    return;
  }
  mode = Mode::RECOVER;
}

void BeaconBridge::on_ack_requested(uint16_t irq, bool timer_event, uint32_t tick)
{
  // no new acks for ACK_COLLECT_TIMEOUT_MS and the set is incomplete
  if (timer_event) {
    mode = Mode::RADIO;
    BEACON_LOG("[%lu] on_ack_requested: collect timeout with %u/%u anchors, fallback to radio\r\n",
               (unsigned long)tick,
               static_cast<unsigned>(acks.count),
               static_cast<unsigned>(LORA_BEACON_PROTOCOL::EXPECTED_ANCHOR_COUNT));
    return;
  }

  uint8_t r = ack_listen();
  if (r != ACK_LISTEN_FULL_MASK) {
    mode = Mode::RECOVER;
    BEACON_LOG("[%lu] on_ack_requested: failed execute ack_listen (irq=0x%X)\r\n", (unsigned long)tick, r);
    return;
  }
  mode = Mode::ACK_LISTENING;
  return;
}

void BeaconBridge::on_ack_listen(uint16_t irq, bool timer_event, uint32_t tick)
{
  if (timer_event) {
    mode = Mode::RADIO;
    BEACON_LOG("[%lu] on_ack_listen: collect timeout with %u/%u anchors, fallback to radio\r\n",
               (unsigned long)tick,
               static_cast<unsigned>(acks.count),
               static_cast<unsigned>(LORA_BEACON_PROTOCOL::EXPECTED_ANCHOR_COUNT));
    return;
  }

  if (irq & SX1280_VALUES::IRQ_BIT_RX_DONE) {
    mode = Mode::ACK_RECIEVED;
    BEACON_LOG("[%lu] on_ack_listen: ACK recieved (irq=0x%X)\r\n", (unsigned long)tick, irq);
    return;
  }

  if (irq & SX1280_VALUES::IRQ_BIT_RX_TX_TIMEOUT) {
    mode = Mode::ACK_REQUESTED;
    BEACON_LOG("[%lu] on_ack_listen: ACK RX timeout (irq=0x%X)\r\n", (unsigned long)tick, irq);
    return;
  }
  // maybe handle generic exception if crc or else to rollback into ACK_RECIEVED
  mode = Mode::RECOVER;
}

void BeaconBridge::on_ack_recieved(uint16_t irq, bool timer_event, uint32_t tick)
{
  std::optional<AckPacket> ack = get_ack_response();

  // ack dispatching failed or wake word mismatch
  if (!ack) {
    mode = Mode::ACK_REQUESTED;
    BEACON_LOG("[%lu] on_ack_recieved: ACK is failed or empty or wake mismatch\r\n", (unsigned long)tick);
    return;
  }

  //if deduped anchor registered and enought anchors -> RANGING
  if (register_anchor(&*ack)) {
    if (acks.count >= LORA_BEACON_PROTOCOL::EXPECTED_ANCHOR_COUNT) {
      disarm_timer();  // leaving the collect phase
      if (to_ranging() == TO_RANGING_FULL_MASK) {
        cursor = 0;
        measures.count = 0;

        mode = Mode::RANGING;
        return;
      }
      mode = Mode::RECOVER;
      return;
    }
    arm_timer(ACK_COLLECT_TIMEOUT_MS);  // a new anchor answered: restart the inactivity window
    BEACON_LOG("[%lu] on_ack_recieved: ACK is confirmed (%u/%u anchors)\r\n",
               (unsigned long)tick,
               static_cast<unsigned>(acks.count),
               static_cast<unsigned>(LORA_BEACON_PROTOCOL::EXPECTED_ANCHOR_COUNT));
  }
  else {
    BEACON_LOG("[%lu] on_ack_recieved: duplicate ack ignored (%u/%u anchors)\r\n",
               (unsigned long)tick,
               static_cast<unsigned>(acks.count),
               static_cast<unsigned>(LORA_BEACON_PROTOCOL::EXPECTED_ANCHOR_COUNT));
  }

  // not enough anchors collected, or a duplicate: back to listening
  mode = Mode::ACK_REQUESTED;
}

void BeaconBridge::on_ranging(uint16_t irq, bool timer_event, uint32_t tick)
{
  if (acks.count == 0) {
    BEACON_LOG("[%lu] on_ranging: no collected anchors to range\r\n", (unsigned long)tick);
    mode = Mode::RECOVER;
    return;
  }

  //end of cycle
  if (cursor >= acks.count) {
    cursor = 0;
    measures.count = 0;
  }

  const AckPacket ack = acks.anchors[cursor];
  const uint8_t r = start_ranging(ack.anchor_id);
  if (r != START_RANGING_FULL_MASK) {
    BEACON_LOG("[%lu] on_ranging: request for anchor 0x%08lX failed, mask=0x%X (full=0x%X)\r\n",
               (unsigned long)tick,
               (unsigned long)ack.anchor_id,
               r,
               START_RANGING_FULL_MASK);
    mode = Mode::RECOVER;
    return;
  }

  mode = Mode::RANGING_REQUESTED;
}

void BeaconBridge::on_ranging_requested(uint16_t irq, bool timer_event, uint32_t tick)
{
  const AckPacket ack = acks.anchors[cursor];

  if (irq & SX1280_VALUES::IRQ_BIT_RANGING_MASTER_RESULT_VALID) {
    int32_t result = 0;
    const uint8_t r = get_ranging_result(&result);
    if (r != GET_RANGING_RESULT_FULL_MASK) {
      BEACON_LOG("[%lu] on_ranging_requested: result read for anchor 0x%08lX failed, mask=0x%X (full=0x%X)\r\n",
                 (unsigned long)tick,
                 (unsigned long)ack.anchor_id,
                 r,
                 GET_RANGING_RESULT_FULL_MASK);
      mode = Mode::RECOVER;
      return;
    }

    measures.measured[cursor] = {ack, result, RangeStatus::OK};
    measures.count = static_cast<uint8_t>(cursor + 1);
    BEACON_LOG("[%lu] on_ranging_requested: anchor 0x%08lX -> %ld cm\r\n", (unsigned long)tick, (unsigned long)ack.anchor_id, (long)result);

    cursor++;
    mode = Mode::RANGING;
    on_ranging(0, false, tick);  // next anchor right away, no wait for another step()
    return;
  }

  if (irq & SX1280_VALUES::IRQ_BIT_RANGING_MASTER_TIMEOUT) {
    measures.measured[cursor] = {ack, -1, RangeStatus::TIMEOUT};
    measures.count = static_cast<uint8_t>(cursor + 1);
    BEACON_LOG("[%lu] on_ranging_requested: anchor 0x%08lX timed out\r\n", (unsigned long)tick, (unsigned long)ack.anchor_id);

    cursor++;
    mode = Mode::RANGING;
    on_ranging(0, false, tick);
    return;
  }

  BEACON_LOG("[%lu] on_ranging_requested: unexpected irq=0x%X, still waiting for anchor 0x%08lX\r\n",
             (unsigned long)tick,
             irq,
             (unsigned long)ack.anchor_id);
}

void BeaconBridge::finish_cycle()
{
  // anchors past measures.count were never ranged: an earlier anchor's configuration failed and ended the pass
  for (uint8_t i = measures.count; i < acks.count; i++) {
    measures.measured[i] = RangeEntry{acks.anchors[i], -1, RangeStatus::FAILED};
  }
  frame_length = CycleFrame::Serialize(++cycle_counter, HAL_GetTick(), measures.measured, acks.count, frame, sizeof(frame));
}

size_t BeaconBridge::take_frame(uint8_t* out, size_t capacity)
{
  if (frame_length == 0 || capacity < frame_length) {
    return 0;
  }
  const size_t length = frame_length;
  for (size_t i = 0; i < length; i++) {
    out[i] = frame[i];
  }
  frame_length = 0;
  return length;
}

void BeaconBridge::return_to_idle()
{
  finish_cycle();  // every path that ends a cycle comes through here
  to_radio();
  mode = Mode::RADIO;
}

void BeaconBridge::step(volatile uint8_t& dio1_flag, volatile uint8_t& tim_flag)
{
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
    case Mode::RADIO:
      on_radio(irq, timer_event, now);
      break;
    case Mode::ACK_IN_PROGRESS:
      if (dio1_event) {
        on_ack_in_progress(irq, timer_event, now);
      }
      break;
    case Mode::ACK_REQUESTED:
      on_ack_requested(irq, timer_event, now);
      break;
    case Mode::ACK_LISTENING:
      if (dio1_event || timer_event) {
        on_ack_listen(irq, timer_event, now);
      }
      break;
    case Mode::ACK_RECIEVED:
      on_ack_recieved(irq, timer_event, now);
      break;
    case Mode::RANGING:
      on_ranging(irq, timer_event, now);
      break;
    case Mode::RANGING_REQUESTED:
      if (dio1_event) {
        on_ranging_requested(irq, timer_event, now);
      }
      break;
    case Mode::RECOVER:
      // no recovery handler yet: the beacon stays here until one is written
      break;
  }
}
