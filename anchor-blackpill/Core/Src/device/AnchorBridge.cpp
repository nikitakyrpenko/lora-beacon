#include <array>
#include <cstdint>

#include "AckPacket.hpp"
#include "AnchorBridge.hpp"
#include "ByteOrder.hpp"
#include "SX1280Constants.hpp"
#include "SX1280Device.hpp"
#include "stm32h5xx_hal.h"
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

uint16_t AnchorBridge::to_ranging_slave()
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
                         ByteOrder::register_write_u32(SX1280_VALUES::REG_RANGING_SLAVE_OWN_ADDR, ANCHOR_RANGING_ADDRESS).data(),
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

  //fetch duration of ranging mode
  const uint8_t* duration_bytes = &payload_start[WAKE_WORD_LEN];
  uint32_t duration_ms = ByteOrder::get_u16(duration_bytes);

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

  *mask_out = ByteOrder::get_u16(&rx_irq[1]);
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

void AnchorBridge::on_listen(uint16_t irq, uint32_t tick)
{
  // a corrupted packet still ends the single-shot RX, so each error is its own way to RECOVER (which re-arms RX)
  if (irq & SX1280_VALUES::IRQ_BIT_HEADER_ERROR) {
    ANCHOR_LOG("[%lu] on_listen: header error (irq=0x%X)\r\n", (unsigned long)tick, irq);
    mode = MODE::RECOVER;
    return;
  }

  if (irq & SX1280_VALUES::IRQ_BIT_CRC_ERROR) {
    ANCHOR_LOG("[%lu] on_listen: CRC error (irq=0x%X)\r\n", (unsigned long)tick, irq);
    mode = MODE::RECOVER;
    return;
  }

  if (irq & SX1280_VALUES::IRQ_BIT_RX_DONE) {
    if (wake_word_matched()) {
      ack_slot_deadline_tick = tick + get_ack_delay_ms();
      mode = MODE::ACK_REQUESTED;
      return;
    }
  }

  //any other case rollback to recover
  mode = MODE::RECOVER;
}

void AnchorBridge::on_ack_requested(uint16_t irq, uint32_t tick)
{
  constexpr uint32_t MAX_ACK_DELAY_MS = 10;

  //ack deadline not reached
  if (static_cast<int32_t>(tick - ack_slot_deadline_tick) < 0) {
    return;
  }

  //if delayed too much -> fallback to recover
  if (tick - ack_slot_deadline_tick > MAX_ACK_DELAY_MS) {
    ANCHOR_LOG("[%lu] on_ack_requested: ack slot missed by %lu ms (limit %lu ms), dropping the ack\r\n",
               (unsigned long)tick,
               (unsigned long)(tick - ack_slot_deadline_tick),
               (unsigned long)MAX_ACK_DELAY_MS);
    mode = MODE::RECOVER;
    return;
  }
  const uint16_t r = send_ranging_slave_ack();
  if (r != 0xF) {
    ANCHOR_LOG("[%lu] on_ack_requested: ack send failed mask=0x%X (full=0xF)\r\n", (unsigned long)tick, r);
    mode = MODE::RECOVER;
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
    mode = MODE::RECOVER;
    return;
  }

  mode = MODE::ACK_SENT;
}

void AnchorBridge::on_ack_sent(uint16_t irq, uint32_t tick)
{
  // the ack has left the antenna (confirmed in on_ack_requested): arm the ranging slave
  const uint16_t r = to_ranging_slave();
  if (r != 0x7FF) {
    ANCHOR_LOG("[%lu] on_ack_sent: to_ranging_slave failed mask=0x%X (full=0x7FF)\r\n", (unsigned long)tick, r);
    mode = MODE::RECOVER;
    return;
  }

  // the window starts once the slave is armed
  ranging_window_deadline_tick = HAL_GetTick() + ranging_window_duration_ms;
  mode = MODE::RANGING;
}

void AnchorBridge::on_ranging(uint16_t irq, uint32_t tick)
{
  // the slave response has left the antenna: the exchange is over, back to listening
  if (irq & SX1280_VALUES::IRQ_BIT_RANGING_SLAVE_RESPONSE_DONE) {
    if (to_radio() != 0x1FF) {
      mode = MODE::RECOVER;
      return;
    }
    mode = MODE::LISTENING;
    return;
  }

  // the request was discarded: the ranging slave can not be trusted any more
  if (irq & SX1280_VALUES::IRQ_BIT_RANGING_SLAVE_REQUEST_DISCARD) {
    ANCHOR_LOG("[%lu] on_ranging: request discarded (irq=0x%X)\r\n", (unsigned long)tick, irq);
    mode = MODE::RECOVER;
    return;
  }

  // MASTER_REQUEST_VALID is the normal first event of an exchange, nothing to do; anything else is unexpected but not fatal
  if (irq & static_cast<uint16_t>(~SX1280_VALUES::IRQ_BIT_RANGING_MASTER_REQUEST_VALID)) {
    log_ranging_irq_unmatched(irq);
  }

  // window closed without a finished exchange (no request came for this anchor): back to listening
  if (static_cast<int32_t>(tick - ranging_window_deadline_tick) >= 0) {
    ANCHOR_LOG("[%lu] on_ranging: window closed without a response\r\n", (unsigned long)tick);
    if (to_radio() != 0x1FF) {
      mode = MODE::RECOVER;
      return;
    }
    mode = MODE::LISTENING;
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
  if (r == 0x1FF) {
    recover_fail_count = 0;
    mode = MODE::LISTENING;
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
    case MODE::LISTENING:
      if (dio1_event) {
        on_listen(irq, now);
      }
      break;
    case MODE::ACK_REQUESTED:
      on_ack_requested(irq, now);
      break;
    case MODE::ACK_SENT:
      on_ack_sent(irq, now);
      break;
    case MODE::RANGING:
      on_ranging(irq, now);
      break;
    case MODE::RECOVER:
      on_recover(irq, now);
      break;
  }
}
