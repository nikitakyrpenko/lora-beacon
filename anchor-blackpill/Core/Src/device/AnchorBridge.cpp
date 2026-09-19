#include <array>
#include <cstdint>

#include "AckPacket.hpp"
#include "AnchorBridge.hpp"
#include "SX1280Constants.hpp"
#include "stm32h5xx_hal_def.h"

// Set at build time (-DDEBUG_ANCHOR or #define above this include) to turn all AnchorBridge logging on/off in
// one place -- main.cpp calls these methods with no printf of its own, so this is the only switch needed.
#ifdef DEBUG_ANCHOR
#include <cstdio>
#define ANCHOR_LOG(...) printf(__VA_ARGS__)
#else
#define ANCHOR_LOG(...)
#endif

// Per-board identity comes from CMake (ANCHOR_ADDRESS / ANCHOR_X_CM / ANCHOR_Y_CM / ANCHOR_Z_CM, see
// anchor-blackpill/CMakeLists.txt) -- fail the build if a non-CMake build forgot them, rather than silently
// falling back to some default identity.
#if !defined(ANCHOR_ADDRESS) || !defined(ANCHOR_X_CM) || !defined(ANCHOR_Y_CM) || !defined(ANCHOR_Z_CM)
#error "ANCHOR_ADDRESS / ANCHOR_X_CM / ANCHOR_Y_CM / ANCHOR_Z_CM must be defined (see anchor-blackpill/CMakeLists.txt)"
#endif

// unique anchor address id
static constexpr uint32_t ANCHOR_RANGING_ADDRESS = ANCHOR_ADDRESS;
// a lower address would underflow ACK_SLOT_DELAY_MS below
static_assert(ANCHOR_RANGING_ADDRESS >= LORA_BEACON_PROTOCOL::RANGING_ADDRESS_BLOCK_BASE,
              "ANCHOR_ADDRESS must be within the discovery block (>= RANGING_ADDRESS_BLOCK_BASE)");

// this anchor's position in cm from the shared site origin, sent to the beacon in the wake ack (int16 each, MSB-first)
static_assert(ANCHOR_X_CM >= INT16_MIN && ANCHOR_X_CM <= INT16_MAX, "ANCHOR_X_CM must fit int16_t (+-327 m)");
static_assert(ANCHOR_Y_CM >= INT16_MIN && ANCHOR_Y_CM <= INT16_MAX, "ANCHOR_Y_CM must fit int16_t (+-327 m)");
static_assert(ANCHOR_Z_CM >= INT16_MIN && ANCHOR_Z_CM <= INT16_MAX, "ANCHOR_Z_CM must fit int16_t (+-327 m)");

static constexpr int16_t ANCHOR_POSITION_CM[3] = {
  static_cast<int16_t>(ANCHOR_X_CM), static_cast<int16_t>(ANCHOR_Y_CM), static_cast<int16_t>(ANCHOR_Z_CM)};

// delay before sending ACK for beacon wake request to avoid RF collisions
static constexpr uint32_t ACK_SLOT_DELAY_MS =
  (ANCHOR_RANGING_ADDRESS - LORA_BEACON_PROTOCOL::RANGING_ADDRESS_BLOCK_BASE) * LORA_BEACON_PROTOCOL::ANCHOR_ACK_SLOT_WIDTH_MS;
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

std::array<uint8_t, 6> make_address_register_write(uint16_t reg, uint32_t address)
{
  return {static_cast<uint8_t>(reg >> 8),
          static_cast<uint8_t>(reg & 0xFF),
          static_cast<uint8_t>(address >> 24),
          static_cast<uint8_t>(address >> 16),
          static_cast<uint8_t>(address >> 8),
          static_cast<uint8_t>(address & 0xFF)};
}

}  // namespace

HAL_StatusTypeDef AnchorBridge::clear_irq_mask(SX1280Device::SX1280_Status* sta_out)
{
  uint8_t CLEAR_IRQ_MASK[2] = {0xFF, 0xFF};
  return device.SPI_write(&SX1280_OPERATIONS::CLEAR_IRQ_STATUS_OP_CODE, CLEAR_IRQ_MASK, nullptr, 2, sta_out);
}

uint16_t AnchorBridge::to_radio()
{
  mode = MODE::NONE;  // reset up front so any early-return failure leaves get_mode() accurate, not stale
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
  hal = device.SPI_write(&SX1280_OPERATIONS::SET_DIO_IRQ_PARAMS_OP_CODE, LORA_BEACON_PROTOCOL::RX_DONE_IRQ_MASK, nullptr, 8, &sta);
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

  if (mask == 0b0000000111111111) {
    this->mode = MODE::RADIO;
    ANCHOR_LOG("[%lu] entering RADIO\r\n", (unsigned long)HAL_GetTick());
  }

  return mask;
}

uint16_t AnchorBridge::to_ranging_slave()
{
  mode = MODE::NONE;            // reset up front so any early-return failure leaves get_mode() accurate, not stale
  ack_state = ACK_STATE::IDLE;  // wake-word-matched ack has now been consumed (sent), whatever the outcome below
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
                         make_address_register_write(SX1280_VALUES::REG_RANGING_SLAVE_OWN_ADDR, ANCHOR_RANGING_ADDRESS).data(),
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

  if (mask == 0b0000011111111111) {
    this->mode = MODE::RANGING;
    ANCHOR_LOG("[%lu] entering RANGING\r\n", (unsigned long)HAL_GetTick());
  }

  return mask;
}

uint16_t AnchorBridge::send_ranging_slave_ack()
{
  const AckPacket ack{ANCHOR_RANGING_ADDRESS, ANCHOR_POSITION_CM[0], ANCHOR_POSITION_CM[1], ANCHOR_POSITION_CM[2]};

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

  // poll for TX_DONE to confirm the ack actually left the antenna -- SetTx's SPI cmd_status above only means the
  // command was accepted, not that the packet finished transmitting
  uint16_t irq_mask = 0;
  bool tx_done = false;
  uint32_t tx_done_deadline_tick = HAL_GetTick() + ACK_TX_DONE_TIMEOUT_MS;
  while (HAL_GetTick() < tx_done_deadline_tick) {
    if (this->get_irq_mask(&irq_mask) == HAL_OK && (irq_mask & SX1280_VALUES::IRQ_BIT_TX_DONE)) {
      tx_done = true;
      break;
    }
  }
  SX1280Device::SX1280_Status clear_sta{};
  this->clear_irq_mask(&clear_sta);

  ANCHOR_LOG("[%lu] send_ranging_slave_ack: TX_DONE %s (irq_mask=0x%X)\r\n",
             (unsigned long)HAL_GetTick(),
             tx_done ? "confirmed" : "NOT confirmed within timeout",
             irq_mask);

  return mask;
}

bool AnchorBridge::wake_word_matched()
{
  SX1280Device::SX1280_Status sta{};

  // clear IRQ status
  HAL_StatusTypeDef hal = this->clear_irq_mask(&sta);
  if (!step_ok(hal, sta)) {
    ANCHOR_LOG("[%lu] wake_word_matched: clear_irq_mask failed, hal=%d\r\n", (unsigned long)HAL_GetTick(), static_cast<int>(hal));
    return false;
  }
  sta = {};

  constexpr uint8_t BUFF_STA_SIZE = 3;
  uint8_t tx_buff_sta[BUFF_STA_SIZE] = {};
  uint8_t rx_buff_sta[BUFF_STA_SIZE] = {};

  // read RX buffer status -> length + start
  hal = device.SPI_write(&SX1280_OPERATIONS::READ_BUFFER_STATUS_OP_CODE, tx_buff_sta, rx_buff_sta, BUFF_STA_SIZE, &sta);
  if (!step_ok(hal, sta)) {
    ANCHOR_LOG("[%lu] wake_word_matched: buffer status read failed, hal=%d\r\n", (unsigned long)HAL_GetTick(), static_cast<int>(hal));
    return false;
  }

  uint8_t len = rx_buff_sta[1];
  uint8_t beg = rx_buff_sta[2];

  // if value of buffer shorter or longer then WAKE_PAYLOAD_LEN -> return early
  if (len != LORA_BEACON_PROTOCOL::WAKE_PAYLOAD_LEN) {
    ANCHOR_LOG("[%lu] wake_word_matched: unexpected payload len=%u (expected %u)\r\n",
               (unsigned long)HAL_GetTick(),
               len,
               LORA_BEACON_PROTOCOL::WAKE_PAYLOAD_LEN);
    return false;
  }

  constexpr uint8_t BUFFER_OFFSET_SIZE = 2;  // the buffer offset to start reading from + a mandatory dummy/turnaround clock cycle
  constexpr uint8_t BUFF_READ_SIZE = BUFFER_OFFSET_SIZE + LORA_BEACON_PROTOCOL::WAKE_PAYLOAD_LEN;

  uint8_t tx_buff_read[BUFF_READ_SIZE] = {beg};
  uint8_t rx_buff_read[BUFF_READ_SIZE] = {};

  // read the received payload
  hal = device.SPI_write(&SX1280_OPERATIONS::READ_BUFFER_OP_CODE, tx_buff_read, rx_buff_read, static_cast<uint16_t>(BUFF_READ_SIZE), &sta);
  if (!step_ok(hal, sta)) {
    ANCHOR_LOG("[%lu] wake_word_matched: payload read failed, hal=%d\r\n", (unsigned long)HAL_GetTick(), static_cast<int>(hal));
    return false;
  }

  const uint8_t* payload_start = &rx_buff_read[BUFFER_OFFSET_SIZE];
  constexpr uint8_t WAKE_WORD_LEN = sizeof(LORA_BEACON_PROTOCOL::WAKE_WORD);

  //check does wake word matches
  for (uint8_t i = 0; i < WAKE_WORD_LEN; ++i) {
    if (payload_start[i] != LORA_BEACON_PROTOCOL::WAKE_WORD[i]) {
      return false;
    }
  }

  ack_state = ACK_STATE::PENDING;  // matched -- caller now owes an ack (see get_ack_delay_ms())

  //fetch duration of ranging mode
  const uint8_t* duration_bytes = &payload_start[WAKE_WORD_LEN];
  uint32_t duration_ms = (static_cast<uint32_t>(duration_bytes[0]) << 8) | static_cast<uint32_t>(duration_bytes[1]);

  // untrusted (arrived over radio) -- clamp into a safe range before it's ever used to arm a timer, rather
  // than rejecting it and leaving ranging_window_duration_ms stale
  if (duration_ms < LORA_BEACON_PROTOCOL::RANGING_WINDOW_MIN_MS) {
    duration_ms = LORA_BEACON_PROTOCOL::RANGING_WINDOW_MIN_MS;
  }
  else if (duration_ms > LORA_BEACON_PROTOCOL::RANGING_WINDOW_MAX_MS) {
    duration_ms = LORA_BEACON_PROTOCOL::RANGING_WINDOW_MAX_MS;
  }
  ranging_window_duration_ms = duration_ms;

  ANCHOR_LOG("[%lu] wake word matched (ranging window %lums, ack delay %lums)\r\n",
             (unsigned long)HAL_GetTick(),
             (unsigned long)ranging_window_duration_ms,
             (unsigned long)get_ack_delay_ms());

  return true;
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

  *mask_out = (static_cast<uint16_t>(rx_irq[1]) << 8) | rx_irq[2];
  return hal;
}

void AnchorBridge::log_ranging_irq_unmatched(uint16_t irq_mask)
{
  ANCHOR_LOG(
    "[%lu] %s: DIO1 fired during RANGING but response-done bit not set (mask=0x%X)\r\n", (unsigned long)HAL_GetTick(), __func__, irq_mask);
}

uint32_t AnchorBridge::get_ack_delay_ms()
{
  return ACK_SLOT_DELAY_MS;
}

uint32_t AnchorBridge::get_ranging_duration_ms()
{
  return ranging_window_duration_ms;
}