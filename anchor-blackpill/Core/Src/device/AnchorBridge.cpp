#include <array>
#include <cstdint>
#include <optional>

#include "AckPacket.hpp"
#include "AnchorBridge.hpp"
#include "AckPacketIn.hpp"
#include "ByteOrder.hpp"
#include "SX1280Constants.hpp"
#include "SX1280Device.hpp"
#include "stm32h5xx_hal.h"
#include "stm32h5xx_hal_def.h"

#ifdef DEBUG_ANCHOR
#include <cstdio>
#define ANCHOR_LOG(...) printf(__VA_ARGS__)
#else
#define ANCHOR_LOG(...)
#endif

// how long to poll GetIrqStatus for TX_DONE after issuing the ack's SetTx before giving up -- generous margin
// over the ack payload's actual SF7 airtime (single-digit ms)
static constexpr uint32_t ACK_TX_DONE_TIMEOUT_MS = 50;

namespace {

bool step_ok(HAL_StatusTypeDef hal, const SX1280Device::SX1280_Status& sta)
{
  return hal == HAL_OK &&
         (sta.command_status == SX1280Device::CommandStatus::COMMAND_SUCCESS ||
          sta.command_status == SX1280Device::CommandStatus::RESERVED || sta.command_status == SX1280Device::CommandStatus::DATA_AVAILABLE);
}

bool is_wake_word_matches(const AckPacketIn& ack)
{
  for (uint8_t i = 0; i < LORA_BEACON_PROTOCOL::WAKE_WORD_LEN; ++i) {
    if (ack.wake_word[i] != LORA_BEACON_PROTOCOL::WAKE_WORD[i]) {
      return false;
    }
  }
  return true;
}

uint16_t clamp_ranging_window(const AckPacketIn& ack)
{
  uint16_t dur = ack.ranging_window_ms;

  if (ack.ranging_window_ms < LORA_BEACON_PROTOCOL::RANGING_WINDOW_MIN_MS) {
    dur = LORA_BEACON_PROTOCOL::RANGING_WINDOW_MIN_MS;
  }
  else if (ack.ranging_window_ms > LORA_BEACON_PROTOCOL::RANGING_WINDOW_MAX_MS) {
    dur = LORA_BEACON_PROTOCOL::RANGING_WINDOW_MAX_MS;
  }
  return dur;
}

}  // namespace

HAL_StatusTypeDef AnchorBridge::clear_irq_mask(SX1280Device::SX1280_Status* sta_out)
{
  uint8_t CLEAR_IRQ_MASK[2] = {0xFF, 0xFF};
  return device.SPI_write(&SX1280_OPERATIONS::CLEAR_IRQ_STATUS_OP_CODE, CLEAR_IRQ_MASK, nullptr, 2, sta_out);
}

uint16_t AnchorBridge::to_radio()
{
  uint16_t mask = 0;
  SX1280Device::SX1280_Status sta{};
  HAL_StatusTypeDef hal;

  // drop to standby before reconfiguring
  hal = device.SPI_write(&SX1280_OPERATIONS::SET_STANDBY_OP_CODE, &SX1280_VALUES::STDBY_RC_STAND_BY, nullptr, 1, &sta);
  if (!step_ok(hal, sta)) {
    ANCHOR_LOG("[%lu] %s failed: hal=%d circuit_mode=%d cmd_status=%d busy=%d mask=0x%X\r\n",
               (unsigned long)HAL_GetTick(),
               __func__,
               static_cast<int>(hal),
               static_cast<int>(sta.circuit_mode),
               static_cast<int>(sta.command_status),
               static_cast<int>(sta.busy),
               mask);
    return mask;
  }
  mask |= (1u << 0);

  // select LoRa packet type
  hal = device.SPI_write(&SX1280_OPERATIONS::SET_PACKET_TYPE_OP_CODE, &SX1280_VALUES::PACKET_TYPE_LORA, nullptr, 1, &sta);
  if (!step_ok(hal, sta)) {
    ANCHOR_LOG("[%lu] %s failed: hal=%d circuit_mode=%d cmd_status=%d busy=%d mask=0x%X\r\n",
               (unsigned long)HAL_GetTick(),
               __func__,
               static_cast<int>(hal),
               static_cast<int>(sta.circuit_mode),
               static_cast<int>(sta.command_status),
               static_cast<int>(sta.busy),
               mask);
    return mask;
  }
  mask |= (1u << 1);

  // set carrier frequency
  hal = device.SPI_write(&SX1280_OPERATIONS::SET_FREQUENCY_OP_CODE, SX1280_VALUES::RF_FREQUENCY_BYTES, nullptr, 3, &sta);
  if (!step_ok(hal, sta)) {
    ANCHOR_LOG("[%lu] %s failed: hal=%d circuit_mode=%d cmd_status=%d busy=%d mask=0x%X\r\n",
               (unsigned long)HAL_GetTick(),
               __func__,
               static_cast<int>(hal),
               static_cast<int>(sta.circuit_mode),
               static_cast<int>(sta.command_status),
               static_cast<int>(sta.busy),
               mask);
    return mask;
  }
  mask |= (1u << 2);

  // set TX/RX buffer base addresses
  hal = device.SPI_write(&SX1280_OPERATIONS::SET_BUFFER_BASE_ADDRESS_OP_CODE, SX1280_VALUES::BUFFER_BASE_ADDRESS, nullptr, 2, &sta);
  if (!step_ok(hal, sta)) {
    ANCHOR_LOG("[%lu] %s failed: hal=%d circuit_mode=%d cmd_status=%d busy=%d mask=0x%X\r\n",
               (unsigned long)HAL_GetTick(),
               __func__,
               static_cast<int>(hal),
               static_cast<int>(sta.circuit_mode),
               static_cast<int>(sta.command_status),
               static_cast<int>(sta.busy),
               mask);
    return mask;
  }
  mask |= (1u << 3);

  // set SF7/BW1600/CR4·5 modulation
  hal = device.SPI_write(&SX1280_OPERATIONS::SET_MODULATION_OP_CODE, SX1280_VALUES::MODULATION_PARAMS_SF7, nullptr, 3, &sta);
  if (!step_ok(hal, sta)) {
    ANCHOR_LOG("[%lu] %s failed: hal=%d circuit_mode=%d cmd_status=%d busy=%d mask=0x%X\r\n",
               (unsigned long)HAL_GetTick(),
               __func__,
               static_cast<int>(hal),
               static_cast<int>(sta.circuit_mode),
               static_cast<int>(sta.command_status),
               static_cast<int>(sta.busy),
               mask);
    return mask;
  }
  mask |= (1u << 4);

  // required register fixup for SF7/SF8
  hal = device.SPI_write(&SX1280_OPERATIONS::WRITE_REGISTER_OP_CODE, SX1280_VALUES::SF_7_FIXUP_WRITE, nullptr, 3, &sta);
  if (!step_ok(hal, sta)) {
    ANCHOR_LOG("[%lu] %s failed: hal=%d circuit_mode=%d cmd_status=%d busy=%d mask=0x%X\r\n",
               (unsigned long)HAL_GetTick(),
               __func__,
               static_cast<int>(hal),
               static_cast<int>(sta.circuit_mode),
               static_cast<int>(sta.command_status),
               static_cast<int>(sta.busy),
               mask);
    return mask;
  }
  mask |= (1u << 5);

  // configure packet params to expect a wake-payload-sized packet
  hal = device.SPI_write(&SX1280_OPERATIONS::SET_PACKET_PARAMS_OP_CODE, LORA_BEACON_PROTOCOL::WAKE_PACKET_PARAMS, nullptr, 7, &sta);
  if (!step_ok(hal, sta)) {
    ANCHOR_LOG("[%lu] %s failed: hal=%d circuit_mode=%d cmd_status=%d busy=%d mask=0x%X\r\n",
               (unsigned long)HAL_GetTick(),
               __func__,
               static_cast<int>(hal),
               static_cast<int>(sta.circuit_mode),
               static_cast<int>(sta.command_status),
               static_cast<int>(sta.busy),
               mask);
    return mask;
  }
  mask |= (1u << 6);

  // route RxDone interrupt to DIO1
  hal = device.SPI_write(&SX1280_OPERATIONS::SET_DIO_IRQ_PARAMS_OP_CODE, LORA_BEACON_PROTOCOL::IDLE_RX_IRQ_MASK, nullptr, 8, &sta);
  if (!step_ok(hal, sta)) {
    ANCHOR_LOG("[%lu] %s failed: hal=%d circuit_mode=%d cmd_status=%d busy=%d mask=0x%X\r\n",
               (unsigned long)HAL_GetTick(),
               __func__,
               static_cast<int>(hal),
               static_cast<int>(sta.circuit_mode),
               static_cast<int>(sta.command_status),
               static_cast<int>(sta.busy),
               mask);
    return mask;
  }
  mask |= (1u << 7);

  // start listening indefinitely
  hal = device.SPI_write(&SX1280_OPERATIONS::SET_RX_OP_CODE, LORA_BEACON_PROTOCOL::RX_CONTINUOUS_PARAMS, nullptr, 3, &sta);
  if (!step_ok(hal, sta)) {
    ANCHOR_LOG("[%lu] %s failed: hal=%d circuit_mode=%d cmd_status=%d busy=%d mask=0x%X\r\n",
               (unsigned long)HAL_GetTick(),
               __func__,
               static_cast<int>(hal),
               static_cast<int>(sta.circuit_mode),
               static_cast<int>(sta.command_status),
               static_cast<int>(sta.busy),
               mask);
    return mask;
  }
  mask |= (1u << 8);

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
    ANCHOR_LOG("[%lu] %s failed: hal=%d circuit_mode=%d cmd_status=%d busy=%d mask=0x%X\r\n",
               (unsigned long)HAL_GetTick(),
               __func__,
               static_cast<int>(hal),
               static_cast<int>(sta.circuit_mode),
               static_cast<int>(sta.command_status),
               static_cast<int>(sta.busy),
               mask);
    return mask;
  }
  mask |= (1u << 0);

  // set carrier frequency
  hal = device.SPI_write(&SX1280_OPERATIONS::SET_FREQUENCY_OP_CODE, SX1280_VALUES::RF_FREQUENCY_BYTES, nullptr, 3, &sta);
  if (!step_ok(hal, sta)) {
    ANCHOR_LOG("[%lu] %s failed: hal=%d circuit_mode=%d cmd_status=%d busy=%d mask=0x%X\r\n",
               (unsigned long)HAL_GetTick(),
               __func__,
               static_cast<int>(hal),
               static_cast<int>(sta.circuit_mode),
               static_cast<int>(sta.command_status),
               static_cast<int>(sta.busy),
               mask);
    return mask;
  }
  mask |= (1u << 1);

  // set SF7/BW1600/CR4·5 modulation
  hal = device.SPI_write(&SX1280_OPERATIONS::SET_MODULATION_OP_CODE, SX1280_VALUES::MODULATION_PARAMS_SF7, nullptr, 3, &sta);
  if (!step_ok(hal, sta)) {
    ANCHOR_LOG("[%lu] %s failed: hal=%d circuit_mode=%d cmd_status=%d busy=%d mask=0x%X\r\n",
               (unsigned long)HAL_GetTick(),
               __func__,
               static_cast<int>(hal),
               static_cast<int>(sta.circuit_mode),
               static_cast<int>(sta.command_status),
               static_cast<int>(sta.busy),
               mask);
    return mask;
  }
  mask |= (1u << 2);

  // required register fixup for SF7/SF8
  hal = device.SPI_write(&SX1280_OPERATIONS::WRITE_REGISTER_OP_CODE, SX1280_VALUES::SF_7_FIXUP_WRITE, nullptr, 3, &sta);
  if (!step_ok(hal, sta)) {
    ANCHOR_LOG("[%lu] %s failed: hal=%d circuit_mode=%d cmd_status=%d busy=%d mask=0x%X\r\n",
               (unsigned long)HAL_GetTick(),
               __func__,
               static_cast<int>(hal),
               static_cast<int>(sta.circuit_mode),
               static_cast<int>(sta.command_status),
               static_cast<int>(sta.busy),
               mask);
    return mask;
  }
  mask |= (1u << 3);

  // configure packet params for the ranging exchange
  hal = device.SPI_write(&SX1280_OPERATIONS::SET_PACKET_PARAMS_OP_CODE, SX1280_VALUES::RANGING_PACKET_PARAMS, nullptr, 7, &sta);
  if (!step_ok(hal, sta)) {
    ANCHOR_LOG("[%lu] %s failed: hal=%d circuit_mode=%d cmd_status=%d busy=%d mask=0x%X\r\n",
               (unsigned long)HAL_GetTick(),
               __func__,
               static_cast<int>(hal),
               static_cast<int>(sta.circuit_mode),
               static_cast<int>(sta.command_status),
               static_cast<int>(sta.busy),
               mask);
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
    ANCHOR_LOG("[%lu] %s failed: hal=%d circuit_mode=%d cmd_status=%d busy=%d mask=0x%X\r\n",
               (unsigned long)HAL_GetTick(),
               __func__,
               static_cast<int>(hal),
               static_cast<int>(sta.circuit_mode),
               static_cast<int>(sta.command_status),
               static_cast<int>(sta.busy),
               mask);
    return mask;
  }
  mask |= (1u << 5);

  // set ranging address check length
  hal = device.SPI_write(&SX1280_OPERATIONS::WRITE_REGISTER_OP_CODE, SX1280_VALUES::RANGING_ADDR_CHECK_LEN_8BIT, nullptr, 3, &sta);
  if (!step_ok(hal, sta)) {
    ANCHOR_LOG("[%lu] %s failed: hal=%d circuit_mode=%d cmd_status=%d busy=%d mask=0x%X\r\n",
               (unsigned long)HAL_GetTick(),
               __func__,
               static_cast<int>(hal),
               static_cast<int>(sta.circuit_mode),
               static_cast<int>(sta.command_status),
               static_cast<int>(sta.busy),
               mask);
    return mask;
  }
  mask |= (1u << 6);

  // write RxTx-delay calibration offset
  hal = device.SPI_write(&SX1280_OPERATIONS::WRITE_REGISTER_OP_CODE, SX1280_VALUES::RANGING_CALIBRATION_WRITE, nullptr, 4, &sta);
  if (!step_ok(hal, sta)) {
    ANCHOR_LOG("[%lu] %s failed: hal=%d circuit_mode=%d cmd_status=%d busy=%d mask=0x%X\r\n",
               (unsigned long)HAL_GetTick(),
               __func__,
               static_cast<int>(hal),
               static_cast<int>(sta.circuit_mode),
               static_cast<int>(sta.command_status),
               static_cast<int>(sta.busy),
               mask);
    return mask;
  }
  mask |= (1u << 7);

  // set ranging role to slave
  hal = device.SPI_write(&SX1280_OPERATIONS::SET_RANGING_ROLE_OP_CODE, &SX1280_VALUES::RANGING_ROLE_SLAVE, nullptr, 1, &sta);
  if (!step_ok(hal, sta)) {
    ANCHOR_LOG("[%lu] %s failed: hal=%d circuit_mode=%d cmd_status=%d busy=%d mask=0x%X\r\n",
               (unsigned long)HAL_GetTick(),
               __func__,
               static_cast<int>(hal),
               static_cast<int>(sta.circuit_mode),
               static_cast<int>(sta.command_status),
               static_cast<int>(sta.busy),
               mask);
    return mask;
  }
  mask |= (1u << 8);

  // route ranging interrupts to DIO1
  hal = device.SPI_write(&SX1280_OPERATIONS::SET_DIO_IRQ_PARAMS_OP_CODE, SX1280_VALUES::RANGING_SLAVE_IRQ_MASK, nullptr, 8, &sta);
  if (!step_ok(hal, sta)) {
    ANCHOR_LOG("[%lu] %s failed: hal=%d circuit_mode=%d cmd_status=%d busy=%d mask=0x%X\r\n",
               (unsigned long)HAL_GetTick(),
               __func__,
               static_cast<int>(hal),
               static_cast<int>(sta.circuit_mode),
               static_cast<int>(sta.command_status),
               static_cast<int>(sta.busy),
               mask);
    return mask;
  }
  mask |= (1u << 9);

  // start listening indefinitely
  hal = device.SPI_write(&SX1280_OPERATIONS::SET_RX_OP_CODE, LORA_BEACON_PROTOCOL::RX_CONTINUOUS_PARAMS, nullptr, 3, &sta);
  if (!step_ok(hal, sta)) {
    ANCHOR_LOG("[%lu] %s failed: hal=%d circuit_mode=%d cmd_status=%d busy=%d mask=0x%X\r\n",
               (unsigned long)HAL_GetTick(),
               __func__,
               static_cast<int>(hal),
               static_cast<int>(sta.circuit_mode),
               static_cast<int>(sta.command_status),
               static_cast<int>(sta.busy),
               mask);
    return mask;
  }
  mask |= (1u << 10);

  return mask;
}

uint16_t AnchorBridge::send_ack()
{
  const AckPacket ack{ANCHOR_ADDRESS, ANCHOR_X_CM, ANCHOR_Y_CM, ANCHOR_Z_CM};

  std::array<uint8_t, 1 + LORA_BEACON_PROTOCOL::WAKE_ACK_PAYLOAD_LEN> payload = {
    0x00, LORA_BEACON_PROTOCOL::WAKE_ACK[0], LORA_BEACON_PROTOCOL::WAKE_ACK[1]};

  ack.serialize(payload.data() + 1 + sizeof(LORA_BEACON_PROTOCOL::WAKE_ACK));

  uint16_t mask = 0;
  SX1280Device::SX1280_Status sta{};
  HAL_StatusTypeDef hal;

  // configure packet params for the ack payload length
  hal = device.SPI_write(&SX1280_OPERATIONS::SET_PACKET_PARAMS_OP_CODE, LORA_BEACON_PROTOCOL::WAKE_ACK_PACKET_PARAMS, nullptr, 7, &sta);
  if (!step_ok(hal, sta)) {
    ANCHOR_LOG("[%lu] %s failed: hal=%d circuit_mode=%d cmd_status=%d busy=%d mask=0x%X\r\n",
               (unsigned long)HAL_GetTick(),
               __func__,
               static_cast<int>(hal),
               static_cast<int>(sta.circuit_mode),
               static_cast<int>(sta.command_status),
               static_cast<int>(sta.busy),
               mask);
    return mask;
  }
  mask |= (1u << 0);

  // write the ack magic + this anchor's own address + position into the TX buffer
  hal = device.SPI_write(&SX1280_OPERATIONS::WRITE_BUFFER_OP_CODE, payload.data(), nullptr, static_cast<uint16_t>(sizeof(payload)), &sta);
  if (!step_ok(hal, sta)) {
    ANCHOR_LOG("[%lu] %s failed: hal=%d circuit_mode=%d cmd_status=%d busy=%d mask=0x%X\r\n",
               (unsigned long)HAL_GetTick(),
               __func__,
               static_cast<int>(hal),
               static_cast<int>(sta.circuit_mode),
               static_cast<int>(sta.command_status),
               static_cast<int>(sta.busy),
               mask);
    return mask;
  }
  mask |= (1u << 1);

  // route TxDone interrupt to DIO1 -- whatever IrqMask to_radio() left active (RX_DONE only) would never latch
  // TX_DONE into the IrqStatus register at all, so this must be reprogrammed before SetTx for the poll below to work
  hal = device.SPI_write(&SX1280_OPERATIONS::SET_DIO_IRQ_PARAMS_OP_CODE, LORA_BEACON_PROTOCOL::TX_DONE_IRQ_MASK, nullptr, 8, &sta);
  if (!step_ok(hal, sta)) {
    ANCHOR_LOG("[%lu] %s failed: hal=%d circuit_mode=%d cmd_status=%d busy=%d mask=0x%X\r\n",
               (unsigned long)HAL_GetTick(),
               __func__,
               static_cast<int>(hal),
               static_cast<int>(sta.circuit_mode),
               static_cast<int>(sta.command_status),
               static_cast<int>(sta.busy),
               mask);
    return mask;
  }
  mask |= (1u << 2);

  // transmit the ack
  hal = device.SPI_write(&SX1280_OPERATIONS::SET_TX_OP_CODE, LORA_BEACON_PROTOCOL::TX_SINGLE_SHOT_PARAMS, nullptr, 3, &sta);
  if (!step_ok(hal, sta)) {
    ANCHOR_LOG("[%lu] %s failed: hal=%d circuit_mode=%d cmd_status=%d busy=%d mask=0x%X\r\n",
               (unsigned long)HAL_GetTick(),
               __func__,
               static_cast<int>(hal),
               static_cast<int>(sta.circuit_mode),
               static_cast<int>(sta.command_status),
               static_cast<int>(sta.busy),
               mask);
    return mask;
  }
  mask |= (1u << 3);

  ANCHOR_LOG("[%lu] send_ranging_slave_ack mask=0x%X (full=0x%X)\r\n", (unsigned long)HAL_GetTick(), mask, 0b1111);

  return mask;
}

std::optional<AckPacketIn> AnchorBridge::get_ack_packet()
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

  AckPacketIn ack = AckPacketIn::parse(rx_buff_read + BUFFER_OFFSET_SIZE);
  return ack;
};

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

void AnchorBridge::on_listen(uint16_t irq, uint32_t tick)
{
  // a corrupted packet still ends the single-shot RX, so each error is its own way to RECOVER (which re-arms RX)
  if (irq & SX1280_VALUES::IRQ_BIT_HEADER_ERROR) {
    ANCHOR_LOG("[%lu] on_listen: header error (irq=0x%X)\r\n", (unsigned long)tick, irq);
    mode = Mode::RECOVER;
    return;
  }

  if (irq & SX1280_VALUES::IRQ_BIT_CRC_ERROR) {
    ANCHOR_LOG("[%lu] on_listen: CRC error (irq=0x%X)\r\n", (unsigned long)tick, irq);
    mode = Mode::RECOVER;
    return;
  }

  if (irq & SX1280_VALUES::IRQ_BIT_RX_DONE) {
    std::optional<AckPacketIn> ack = get_ack_packet();

    if (ack) {
      if (is_wake_word_matches(*ack)) {
        latch.ranging = clamp_ranging_window(*ack);
        deadline.ack = tick + latch.ack;
        mode = Mode::ACK_REQUESTED;
        return;
      }
      else {
        ANCHOR_LOG("[%lu] on_listen: wake_word mismatch (irq=0x%X)\r\n", (unsigned long)tick, irq);
        mode = Mode::RECOVER;
        return;
      }
    }
  }

  //any other case rollback to recover
  mode = Mode::RECOVER;
}

void AnchorBridge::on_ack_requested(uint16_t irq, uint32_t tick)
{
  constexpr uint32_t MAX_ACK_DELAY_MS = 10;

  //ack deadline not reached
  if (static_cast<int32_t>(tick - deadline.ack) < 0) {
    return;
  }

  //if delayed too much -> fallback to recover
  if (tick - deadline.ack > MAX_ACK_DELAY_MS) {
    ANCHOR_LOG("[%lu] on_ack_requested: ack slot missed by %lu ms (limit %lu ms), dropping the ack\r\n",
               (unsigned long)tick,
               (unsigned long)(tick - deadline.ack),
               (unsigned long)MAX_ACK_DELAY_MS);
    mode = Mode::RECOVER;
    return;
  }
  const uint16_t r = send_ack();
  if (r != ACK_SUCCESS) {
    ANCHOR_LOG("[%lu] on_ack_requested: ack send failed mask=0x%X (full=0xF)\r\n", (unsigned long)tick, r);
    mode = Mode::RECOVER;
    return;
  }

  // the ack is only done once it has left the antenna: poll TX_DONE, the chip does not leave TX on its own after a fault
  bool tx_done = false;
  const uint32_t tx_start_tick = HAL_GetTick();
  while (HAL_GetTick() - tx_start_tick < ACK_TX_DONE_TIMEOUT_MS) {
    uint16_t tx_irq = 0;
    if (get_irq_mask(&tx_irq) == HAL_OK && (tx_irq & SX1280_VALUES::IRQ_BIT_TX_DONE)) {
      tx_done = true;
      break;
    }
  }

  SX1280Device::SX1280_Status sta{};
  clear_irq_mask(&sta);

  if (!tx_done) {
    ANCHOR_LOG("[%lu] on_ack_requested: TX_DONE not seen within %lu ms\r\n", (unsigned long)tick, (unsigned long)ACK_TX_DONE_TIMEOUT_MS);
    mode = Mode::RECOVER;
    return;
  }

  mode = Mode::ACK_SENT;
}

void AnchorBridge::on_ack_sent(uint16_t irq, uint32_t tick)
{
  // the ack has left the antenna (confirmed in on_ack_requested): arm the ranging slave
  const uint16_t r = to_ranging();
  if (r != RANGING_SUCCESS) {
    ANCHOR_LOG("[%lu] on_ack_sent: to_ranging_slave failed mask=0x%X (full=0x%X)\r\n", (unsigned long)tick, r, RANGING_SUCCESS);
    mode = Mode::RECOVER;
    return;
  }

  // the window starts once the slave is armed
  deadline.ranging = HAL_GetTick() + latch.ranging;
  mode = Mode::RANGING;
}

void AnchorBridge::on_ranging(uint16_t irq, uint32_t tick)
{
  // the slave response has left the antenna
  if (irq & SX1280_VALUES::IRQ_BIT_RANGING_SLAVE_RESPONSE_DONE) {
    if (to_radio() != RADIO_SUCCESS) {
      mode = Mode::RECOVER;
      return;
    }
    mode = Mode::LISTENING;
    return;
  }

  // the request was discarded
  if (irq & SX1280_VALUES::IRQ_BIT_RANGING_SLAVE_REQUEST_DISCARD) {
    ANCHOR_LOG("[%lu] on_ranging: request discarded (irq=0x%X)\r\n", (unsigned long)tick, irq);
    mode = Mode::RECOVER;
    return;
  }

  // MASTER_REQUEST_VALID is the normal first event of an exchange, nothing to do; anything else is unexpected but not fatal
  if (irq & static_cast<uint16_t>(~SX1280_VALUES::IRQ_BIT_RANGING_MASTER_REQUEST_VALID)) {
    ANCHOR_LOG("[%lu] on_ranging: unexpected irq bits set (irq=0x%X)\r\n", (unsigned long)tick, irq);
  }

  // window closed without a finished exchang
  if (static_cast<int32_t>(tick - deadline.ranging) >= 0) {
    ANCHOR_LOG("[%lu] on_ranging: window closed without a response\r\n", (unsigned long)tick);
    if (to_radio() != RADIO_SUCCESS) {
      mode = Mode::RECOVER;
      return;
    }
    mode = Mode::LISTENING;
  }
}

void AnchorBridge::on_recover(uint16_t irq, uint32_t tick)
{
  constexpr uint32_t RECOVER_RETRY_MS = 1000;  // pause between attempts once one has failed
  constexpr uint8_t RECOVER_RESET_AFTER = 3;   // failed attempts in a row before the chip gets a hardware reset

  // the first attempt is immediate, after a failure wait RECOVER_RETRY_MS (elapsed form, wrap-safe)
  if (recover_fail_count > 0 && tick - recover_last_fail_tick < RECOVER_RETRY_MS) {
    return;
  }

  // to_radio() keeps failing -> the chip may be stuck (BUSY held high, lost config): reset it first
  if (recover_fail_count >= RECOVER_RESET_AFTER) {
    const bool busy_released = device.NRESET_reset();
    ANCHOR_LOG("[%lu] on_recover: NRESET_reset after %u failed attempts, busy_released=%d\r\n",
               (unsigned long)tick,
               static_cast<unsigned>(recover_fail_count),
               static_cast<int>(busy_released));
    recover_fail_count = 0;
  }

  const uint16_t r = to_radio();
  if (r == RADIO_SUCCESS) {
    recover_fail_count = 0;
    mode = Mode::LISTENING;
    return;
  }

  ++recover_fail_count;
  recover_last_fail_tick = tick;
  ANCHOR_LOG(
    "[%lu] on_recover: to_radio failed mask=0x%03X (attempt %u)\r\n", (unsigned long)tick, r, static_cast<unsigned>(recover_fail_count));
}

void AnchorBridge::step(volatile uint8_t& dio1_flag)
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

  const uint32_t now = HAL_GetTick();

  // handlers that react to a radio event only run when DIO1 fired, the others run on every pass
  switch (mode) {
    case Mode::LISTENING:
      if (dio1_event) {
        on_listen(irq, now);
      }
      break;
    case Mode::ACK_REQUESTED:
      on_ack_requested(irq, now);
      break;
    case Mode::ACK_SENT:
      on_ack_sent(irq, now);
      break;
    case Mode::RANGING:
      on_ranging(irq, now);
      break;
    case Mode::RECOVER:
      on_recover(irq, now);
      break;
  }
}
