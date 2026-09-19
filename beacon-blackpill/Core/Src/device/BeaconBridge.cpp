#include <array>
#include <cstdint>

#include "BeaconBridge.hpp"
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

// hal: HAL_OK=0, HAL_ERROR=1, HAL_BUSY=2, HAL_TIMEOUT=3 (stm32*_hal_def.h). SPI_write() returns HAL_TIMEOUT only from
// its leading BUSY_wait() (chip never released BUSY -- not powered/responding at all), and HAL_ERROR only from
// NSS_begin()/NSS_end() finding NSS already in the wrong state before it toggles. sta stays zero-initialized in both of
// those early-return cases (SPI_write only writes *out on HAL_OK). Uses __func__, so expand it inside the failing method.
#define BEACON_LOG_STEP_FAIL(hal, sta, mask)                                                \
  BEACON_LOG("[%lu] %s failed: hal=%d circuit_mode=%d cmd_status=%d busy=%d mask=0x%X\r\n", \
             (unsigned long)HAL_GetTick(),                                                  \
             __func__,                                                                      \
             static_cast<int>(hal),                                                         \
             static_cast<int>((sta).circuit_mode),                                          \
             static_cast<int>((sta).command_status),                                        \
             static_cast<int>((sta).busy),                                                  \
             mask)

// Bitmask a multi-step method returns when every one of its SPI steps succeeded (bit i == step i)
static constexpr uint16_t TO_RADIO_FULL_MASK = (1u << 9) - 1;
static constexpr uint16_t SEND_WAKE_BROADCAST_FULL_MASK = (1u << 2) - 1;
static constexpr uint16_t LISTEN_FOR_ACK_FULL_MASK = (1u << 3) - 1;
static constexpr uint16_t TO_RANGING_MASTER_FULL_MASK = (1u << 10) - 1;

namespace {

bool step_ok(HAL_StatusTypeDef hal, const SX1280Device::SX1280_Status& sta)
{
  return hal == HAL_OK && (sta.command_status == SX1280Device::CommandStatus::COMMAND_SUCCESS ||
                           sta.command_status == SX1280Device::CommandStatus::RESERVED ||
                           sta.command_status == SX1280Device::CommandStatus::DATA_AVAILABLE ||
                           sta.command_status == SX1280Device::CommandStatus::COMMAND_TX_DONE ||
                           sta.command_status == SX1280Device::CommandStatus::COMMAND_TIMEOUT);
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

  *mask_out = (static_cast<uint16_t>(rx_irq[1]) << 8) | rx_irq[2];
  return hal;
}

HAL_StatusTypeDef BeaconBridge::clear_irq_mask(SX1280Device::SX1280_Status* sta_out)
{
  uint8_t CLEAR_IRQ_MASK[2] = {0xFF, 0xFF};
  return device.SPI_write(&SX1280_OPERATIONS::CLEAR_IRQ_STATUS_OP_CODE, CLEAR_IRQ_MASK, nullptr, 2, sta_out);
}

uint16_t BeaconBridge::to_radio()
{
  mode = MODE::NONE;

  uint16_t mask = 0;
  SX1280Device::SX1280_Status sta{};
  HAL_StatusTypeDef hal;

  // drop to standby before reconfiguring
  hal = device.SPI_write(&SX1280_OPERATIONS::SET_STANDBY_OP_CODE, &SX1280_VALUES::STDBY_RC_STAND_BY, nullptr, 1, &sta);
  if (!step_ok(hal, sta)) {
    BEACON_LOG_STEP_FAIL(hal, sta, mask);
    return mask;
  }
  mask |= (1u << 0);

  // select LoRa packet type
  hal = device.SPI_write(&SX1280_OPERATIONS::SET_PACKET_TYPE_OP_CODE, &SX1280_VALUES::PACKET_TYPE_LORA, nullptr, 1, &sta);
  if (!step_ok(hal, sta)) {
    BEACON_LOG_STEP_FAIL(hal, sta, mask);
    return mask;
  }
  mask |= (1u << 1);

  // set carrier frequency
  hal = device.SPI_write(&SX1280_OPERATIONS::SET_FREQUENCY_OP_CODE, SX1280_VALUES::RF_FREQUENCY_BYTES, nullptr, 3, &sta);
  if (!step_ok(hal, sta)) {
    BEACON_LOG_STEP_FAIL(hal, sta, mask);
    return mask;
  }
  mask |= (1u << 2);

  // set TX/RX buffer base addresses
  hal = device.SPI_write(&SX1280_OPERATIONS::SET_BUFFER_BASE_ADDRESS_OP_CODE, SX1280_VALUES::BUFFER_BASE_ADDRESS, nullptr, 2, &sta);
  if (!step_ok(hal, sta)) {
    BEACON_LOG_STEP_FAIL(hal, sta, mask);
    return mask;
  }
  mask |= (1u << 3);

  // set SF7/BW1600/CR4·5 modulation
  hal = device.SPI_write(&SX1280_OPERATIONS::SET_MODULATION_OP_CODE, SX1280_VALUES::MODULATION_PARAMS_SF7, nullptr, 3, &sta);
  if (!step_ok(hal, sta)) {
    BEACON_LOG_STEP_FAIL(hal, sta, mask);
    return mask;
  }
  mask |= (1u << 4);

  // required register fixup for SF7/SF8
  hal = device.SPI_write(&SX1280_OPERATIONS::WRITE_REGISTER_OP_CODE, SX1280_VALUES::SF_7_FIXUP_WRITE, nullptr, 3, &sta);
  if (!step_ok(hal, sta)) {
    BEACON_LOG_STEP_FAIL(hal, sta, mask);
    return mask;
  }
  mask |= (1u << 5);

  // configure packet params for a wake-payload-sized packet
  hal = device.SPI_write(&SX1280_OPERATIONS::SET_PACKET_PARAMS_OP_CODE, LORA_BEACON_PROTOCOL::WAKE_PACKET_PARAMS, nullptr, 7, &sta);
  if (!step_ok(hal, sta)) {
    BEACON_LOG_STEP_FAIL(hal, sta, mask);
    return mask;
  }
  mask |= (1u << 6);

  // route TxDone interrupt to DIO1
  hal = device.SPI_write(&SX1280_OPERATIONS::SET_DIO_IRQ_PARAMS_OP_CODE, LORA_BEACON_PROTOCOL::TX_DONE_IRQ_MASK, nullptr, 8, &sta);
  if (!step_ok(hal, sta)) {
    BEACON_LOG_STEP_FAIL(hal, sta, mask);
    return mask;
  }
  mask |= (1u << 7);

  // set TX output power + ramp time
  hal = device.SPI_write(&SX1280_OPERATIONS::SET_TX_PARAMS_OP_CODE, SX1280_VALUES::TX_PARAMS, nullptr, 2, &sta);
  if (!step_ok(hal, sta)) {
    BEACON_LOG_STEP_FAIL(hal, sta, mask);
    return mask;
  }
  mask |= (1u << 8);

  if (mask == TO_RADIO_FULL_MASK) {
    this->mode = MODE::RADIO;
    BEACON_LOG("[%lu] entering RADIO\r\n", (unsigned long)HAL_GetTick());
  }

  return mask;
}

uint16_t BeaconBridge::send_wake_broadcast()
{
  constexpr uint16_t ranging_window_ms = static_cast<uint16_t>(LORA_BEACON_PROTOCOL::RANGING_WINDOW_MS);
  uint8_t write_buffer[1 + LORA_BEACON_PROTOCOL::WAKE_PAYLOAD_LEN] = {0x00,  // buffer offset
                                                                      LORA_BEACON_PROTOCOL::WAKE_WORD[0],
                                                                      LORA_BEACON_PROTOCOL::WAKE_WORD[1],
                                                                      static_cast<uint8_t>(ranging_window_ms >> 8),
                                                                      static_cast<uint8_t>(ranging_window_ms & 0xFF)};

  uint16_t mask = 0;
  SX1280Device::SX1280_Status sta{};
  HAL_StatusTypeDef hal;

  // write the wake magic + ranging-window duration into the TX buffer
  hal =
    device.SPI_write(&SX1280_OPERATIONS::WRITE_BUFFER_OP_CODE, write_buffer, nullptr, static_cast<uint16_t>(sizeof(write_buffer)), &sta);
  if (!step_ok(hal, sta)) {
    BEACON_LOG_STEP_FAIL(hal, sta, mask);
    return mask;
  }
  mask |= (1u << 0);

  // transmit (single-shot, auto-standby on TxDone)
  hal = device.SPI_write(&SX1280_OPERATIONS::SET_TX_OP_CODE, LORA_BEACON_PROTOCOL::TX_SINGLE_SHOT_PARAMS, nullptr, 3, &sta);
  if (!step_ok(hal, sta)) {
    BEACON_LOG_STEP_FAIL(hal, sta, mask);
    return mask;
  }
  mask |= (1u << 1);

  BEACON_LOG("[%lu] send_wake_broadcast mask=0x%X (full=0x%X)\r\n", (unsigned long)HAL_GetTick(), mask, SEND_WAKE_BROADCAST_FULL_MASK);

  return mask;
}

bool BeaconBridge::was_tx_done()
{
  uint16_t irq_mask = 0;
  if (this->get_irq_mask(&irq_mask) != HAL_OK) {
    return false;
  }

  SX1280Device::SX1280_Status clear_sta{};
  this->clear_irq_mask(&clear_sta);

  return (irq_mask & SX1280_VALUES::IRQ_BIT_TX_DONE) != 0;
}

uint16_t BeaconBridge::listen_for_ack()
{
  mode = MODE::NONE;

  uint16_t mask = 0;
  SX1280Device::SX1280_Status sta{};
  HAL_StatusTypeDef hal;

  // Reuses the frequency/modulation
  hal = device.SPI_write(&SX1280_OPERATIONS::SET_PACKET_PARAMS_OP_CODE, LORA_BEACON_PROTOCOL::WAKE_ACK_PACKET_PARAMS, nullptr, 7, &sta);
  if (!step_ok(hal, sta)) {
    BEACON_LOG_STEP_FAIL(hal, sta, mask);
    return mask;
  }
  mask |= (1u << 0);

  // route RxDone + RxTxTimeout interrupts to DIO1
  hal = device.SPI_write(&SX1280_OPERATIONS::SET_DIO_IRQ_PARAMS_OP_CODE, LORA_BEACON_PROTOCOL::ACK_LISTEN_IRQ_MASK, nullptr, 8, &sta);
  if (!step_ok(hal, sta)) {
    BEACON_LOG_STEP_FAIL(hal, sta, mask);
    return mask;
  }
  mask |= (1u << 1);

  // start timeout-active RX
  hal = device.SPI_write(&SX1280_OPERATIONS::SET_RX_OP_CODE, LORA_BEACON_PROTOCOL::ACK_LISTEN_RX_PARAMS, nullptr, 3, &sta);
  if (!step_ok(hal, sta)) {
    BEACON_LOG_STEP_FAIL(hal, sta, mask);
    return mask;
  }
  mask |= (1u << 2);

  if (mask == LISTEN_FOR_ACK_FULL_MASK) {
    this->mode = MODE::ACK_LISTEN;
    BEACON_LOG("[%lu] entering ACK_LISTEN\r\n", (unsigned long)HAL_GetTick());
  }

  return mask;
}

uint16_t BeaconBridge::stop_ack_listen()
{
  mode = MODE::NONE;
  uint16_t mask = 0;
  SX1280Device::SX1280_Status sta{};

  // RX doesn't necessarily stop on its own -- explicitly SetStandby before reconfiguring for ranging, since
  // to_ranging_master() doesn't start with its own SetStandby (assumes the caller left the chip in STDBY_RC)
  HAL_StatusTypeDef hal = device.SPI_write(&SX1280_OPERATIONS::SET_STANDBY_OP_CODE, &SX1280_VALUES::STDBY_RC_STAND_BY, nullptr, 1, &sta);
  if (!step_ok(hal, sta)) {
    BEACON_LOG_STEP_FAIL(hal, sta, mask);
    return mask;
  }
  mask |= (1u << 0);

  BEACON_LOG("[%lu] ACK_LISTEN stopped\r\n", (unsigned long)HAL_GetTick());

  return mask;
}

bool BeaconBridge::wake_ack_matched(AckPacket* ack_out)
{
  SX1280Device::SX1280_Status sta{};

  constexpr uint8_t BUFF_STA_SIZE = 3;
  uint8_t tx_buff_sta[BUFF_STA_SIZE] = {};
  uint8_t rx_buff_sta[BUFF_STA_SIZE] = {};

  // read RX buffer status -> length + start
  HAL_StatusTypeDef hal = device.SPI_write(&SX1280_OPERATIONS::READ_BUFFER_STATUS_OP_CODE, tx_buff_sta, rx_buff_sta, BUFF_STA_SIZE, &sta);
  if (!step_ok(hal, sta)) {
    BEACON_LOG("[%lu] wake_ack_matched: buffer status read failed, hal=%d cmd_status=%d busy=%d\r\n",
               (unsigned long)HAL_GetTick(),
               static_cast<int>(hal),
               static_cast<int>(sta.command_status),
               static_cast<int>(sta.busy));
    return false;
  }

  uint8_t len = rx_buff_sta[1];
  uint8_t beg = rx_buff_sta[2];

  // shorter or longer than an ack -> not ours
  constexpr uint8_t PAYLOAD_LEN = LORA_BEACON_PROTOCOL::WAKE_ACK_PAYLOAD_LEN;
  if (len != PAYLOAD_LEN) {
    BEACON_LOG("[%lu] wake_ack_matched: unexpected payload len=%u (expected %u)\r\n", (unsigned long)HAL_GetTick(), len, PAYLOAD_LEN);
    return false;
  }

  constexpr uint8_t BUFFER_OFFSET_SIZE = 2;  // the buffer offset to start reading from + a mandatory dummy/turnaround clock cycle
  constexpr uint8_t BUFF_READ_SIZE = BUFFER_OFFSET_SIZE + PAYLOAD_LEN;

  uint8_t tx_buff_read[BUFF_READ_SIZE] = {beg};
  uint8_t rx_buff_read[BUFF_READ_SIZE] = {};
  sta = {};

  // read the received payload
  hal = device.SPI_write(&SX1280_OPERATIONS::READ_BUFFER_OP_CODE, tx_buff_read, rx_buff_read, static_cast<uint16_t>(BUFF_READ_SIZE), &sta);
  if (!step_ok(hal, sta)) {
    BEACON_LOG("[%lu] wake_ack_matched: payload read failed, hal=%d cmd_status=%d busy=%d\r\n",
               (unsigned long)HAL_GetTick(),
               static_cast<int>(hal),
               static_cast<int>(sta.command_status),
               static_cast<int>(sta.busy));
    return false;
  }

  const uint8_t* payload = &rx_buff_read[BUFFER_OFFSET_SIZE];
  constexpr uint8_t MAGIC_LEN = sizeof(LORA_BEACON_PROTOCOL::WAKE_ACK);

  // check the ack magic matches
  for (uint8_t i = 0; i < MAGIC_LEN; ++i) {
    if (payload[i] != LORA_BEACON_PROTOCOL::WAKE_ACK[i]) {
      BEACON_LOG("[%lu] wake_ack_matched: magic mismatch at byte %u got=0x%02X want=0x%02X\r\n",
                 (unsigned long)HAL_GetTick(),
                 i,
                 payload[i],
                 LORA_BEACON_PROTOCOL::WAKE_ACK[i]);
      return false;
    }
  }

  // raw payload hex dump (magic + anchor id + x/y/z) for testing, only reached when the magic matched
  BEACON_LOG("[%lu] wake_ack_matched: payload =", (unsigned long)HAL_GetTick());
  for (uint8_t i = 0; i < PAYLOAD_LEN; ++i) {
    BEACON_LOG(" %02X", payload[i]);
  }
  BEACON_LOG("\r\n");

  const AckPacket ack = AckPacket::parse(&payload[MAGIC_LEN]);
  if (ack_out != nullptr) {
    *ack_out = ack;
  }

  BEACON_LOG("[%lu] wake ack matched (anchor 0x%08lX at %d, %d, %d cm)\r\n",
             (unsigned long)HAL_GetTick(),
             (unsigned long)ack.anchor_id,
             static_cast<int>(ack.x_cm),
             static_cast<int>(ack.y_cm),
             static_cast<int>(ack.z_cm));

  return true;
}

uint16_t BeaconBridge::to_ranging_master(uint32_t target_anchor_address)
{
  mode = MODE::NONE;
  ranging_target_address = target_anchor_address;
  uint16_t mask = 0;
  SX1280Device::SX1280_Status sta{};
  HAL_StatusTypeDef hal;

  // select ranging packet type
  hal = device.SPI_write(&SX1280_OPERATIONS::SET_PACKET_TYPE_OP_CODE, &SX1280_VALUES::PACKET_TYPE_RANGING, nullptr, 1, &sta);
  if (!step_ok(hal, sta)) {
    BEACON_LOG_STEP_FAIL(hal, sta, mask);
    return mask;
  }
  mask |= (1u << 0);

  // set carrier frequency
  hal = device.SPI_write(&SX1280_OPERATIONS::SET_FREQUENCY_OP_CODE, SX1280_VALUES::RF_FREQUENCY_BYTES, nullptr, 3, &sta);
  if (!step_ok(hal, sta)) {
    BEACON_LOG_STEP_FAIL(hal, sta, mask);
    return mask;
  }
  mask |= (1u << 1);

  // set SF7/BW1600/CR4·5 modulation
  hal = device.SPI_write(&SX1280_OPERATIONS::SET_MODULATION_OP_CODE, SX1280_VALUES::MODULATION_PARAMS_SF7, nullptr, 3, &sta);
  if (!step_ok(hal, sta)) {
    BEACON_LOG_STEP_FAIL(hal, sta, mask);
    return mask;
  }
  mask |= (1u << 2);

  // required register fixup for SF7/SF8
  hal = device.SPI_write(&SX1280_OPERATIONS::WRITE_REGISTER_OP_CODE, SX1280_VALUES::SF_7_FIXUP_WRITE, nullptr, 3, &sta);
  if (!step_ok(hal, sta)) {
    BEACON_LOG_STEP_FAIL(hal, sta, mask);
    return mask;
  }
  mask |= (1u << 3);

  // configure packet params for the ranging exchange
  hal = device.SPI_write(&SX1280_OPERATIONS::SET_PACKET_PARAMS_OP_CODE, SX1280_VALUES::RANGING_PACKET_PARAMS, nullptr, 7, &sta);
  if (!step_ok(hal, sta)) {
    BEACON_LOG_STEP_FAIL(hal, sta, mask);
    return mask;
  }
  mask |= (1u << 4);

  // set the target anchor's ranging address
  hal = device.SPI_write(&SX1280_OPERATIONS::WRITE_REGISTER_OP_CODE,
                         make_address_register_write(SX1280_VALUES::REG_RANGING_MASTER_TARGET_ADDR, target_anchor_address).data(),
                         nullptr,
                         6,
                         &sta);
  if (!step_ok(hal, sta)) {
    BEACON_LOG_STEP_FAIL(hal, sta, mask);
    return mask;
  }
  mask |= (1u << 5);

  // set ranging address check length (must match the anchor's)
  hal = device.SPI_write(&SX1280_OPERATIONS::WRITE_REGISTER_OP_CODE, SX1280_VALUES::RANGING_ADDR_CHECK_LEN_8BIT, nullptr, 3, &sta);
  if (!step_ok(hal, sta)) {
    BEACON_LOG_STEP_FAIL(hal, sta, mask);
    return mask;
  }
  mask |= (1u << 6);

  // write RxTx-delay calibration offset
  hal = device.SPI_write(&SX1280_OPERATIONS::WRITE_REGISTER_OP_CODE, SX1280_VALUES::RANGING_CALIBRATION_WRITE, nullptr, 4, &sta);
  if (!step_ok(hal, sta)) {
    BEACON_LOG_STEP_FAIL(hal, sta, mask);
    return mask;
  }
  mask |= (1u << 7);

  // set ranging role to master
  hal = device.SPI_write(&SX1280_OPERATIONS::SET_RANGING_ROLE_OP_CODE, &SX1280_VALUES::RANGING_ROLE_MASTER, nullptr, 1, &sta);
  if (!step_ok(hal, sta)) {
    BEACON_LOG_STEP_FAIL(hal, sta, mask);
    return mask;
  }
  mask |= (1u << 8);

  // route ranging interrupts to DIO1
  hal = device.SPI_write(&SX1280_OPERATIONS::SET_DIO_IRQ_PARAMS_OP_CODE, SX1280_VALUES::RANGING_MASTER_IRQ_MASK, nullptr, 8, &sta);
  if (!step_ok(hal, sta)) {
    BEACON_LOG_STEP_FAIL(hal, sta, mask);
    return mask;
  }
  mask |= (1u << 9);

  if (mask == TO_RANGING_MASTER_FULL_MASK) {
    this->mode = MODE::RANGING;
    BEACON_LOG("[%lu] entering RANGING (target 0x%08lX)\r\n", (unsigned long)HAL_GetTick(), (unsigned long)target_anchor_address);
  }

  return mask;
}

uint16_t BeaconBridge::send_ranging_request()
{
  uint16_t mask = 0;
  SX1280Device::SX1280_Status sta{};

  // transmit the ranging request; the chip's own timeout ends the exchange if the anchor never answers
  HAL_StatusTypeDef hal =
    device.SPI_write(&SX1280_OPERATIONS::SET_TX_OP_CODE, LORA_BEACON_PROTOCOL::RANGING_REQUEST_TX_PARAMS, nullptr, 3, &sta);
  if (!step_ok(hal, sta)) {
    BEACON_LOG_STEP_FAIL(hal, sta, mask);
    return mask;
  }
  mask |= (1u << 0);

  return mask;
}

bool BeaconBridge::read_ranging_result_cm(int32_t* distance_out)
{
  mode = MODE::NONE;

  uint16_t mask = 0;
  SX1280Device::SX1280_Status sta{};
  HAL_StatusTypeDef hal;

  // XOSC standby, required before touching the ranging result registers
  hal = device.SPI_write(&SX1280_OPERATIONS::SET_STANDBY_OP_CODE, &SX1280_VALUES::STDBY_XOSC_STAND_BY, nullptr, 1, &sta);
  if (!step_ok(hal, sta)) {
    BEACON_LOG_STEP_FAIL(hal, sta, mask);
    return false;
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
    BEACON_LOG_STEP_FAIL(hal, sta, mask);
    return false;
  }
  mask |= (1u << 1);
  uint8_t mem_clock_reg = read_clock_rx[3];

  // enable the LoRa memory clock (bit 1)
  uint8_t write_clock_tx[3] = {static_cast<uint8_t>(SX1280_VALUES::REG_LORA_MEM_CLOCK_ENABLE >> 8),
                               static_cast<uint8_t>(SX1280_VALUES::REG_LORA_MEM_CLOCK_ENABLE & 0xFF),
                               static_cast<uint8_t>(mem_clock_reg | (1u << 1))};
  hal = device.SPI_write(&SX1280_OPERATIONS::WRITE_REGISTER_OP_CODE, write_clock_tx, nullptr, 3, &sta);
  if (!step_ok(hal, sta)) {
    BEACON_LOG_STEP_FAIL(hal, sta, mask);
    return false;
  }
  mask |= (1u << 2);

  // select the debiased result type
  hal = device.SPI_write(&SX1280_OPERATIONS::WRITE_REGISTER_OP_CODE, SX1280_VALUES::RANGING_RESULT_MUX_DEBIASED_WRITE, nullptr, 3, &sta);
  if (!step_ok(hal, sta)) {
    BEACON_LOG_STEP_FAIL(hal, sta, mask);
    return false;
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
    BEACON_LOG_STEP_FAIL(hal, sta, mask);
    return false;
  }
  mask |= (1u << 4);

  int32_t raw_result =
    (static_cast<int32_t>(result_rx[3]) << 16) | (static_cast<int32_t>(result_rx[4]) << 8) | static_cast<int32_t>(result_rx[5]);

  // Restore the LoRa memory clock enable bit to its original
  uint8_t restore_clock_tx[3] = {static_cast<uint8_t>(SX1280_VALUES::REG_LORA_MEM_CLOCK_ENABLE >> 8),
                                 static_cast<uint8_t>(SX1280_VALUES::REG_LORA_MEM_CLOCK_ENABLE & 0xFF),
                                 mem_clock_reg};
  hal = device.SPI_write(&SX1280_OPERATIONS::WRITE_REGISTER_OP_CODE, restore_clock_tx, nullptr, 3, &sta);
  if (!step_ok(hal, sta)) {
    BEACON_LOG_STEP_FAIL(hal, sta, mask);
    return false;
  }
  mask |= (1u << 5);

  // back to RC standby
  hal = device.SPI_write(&SX1280_OPERATIONS::SET_STANDBY_OP_CODE, &SX1280_VALUES::STDBY_RC_STAND_BY, nullptr, 1, &sta);
  if (!step_ok(hal, sta)) {
    BEACON_LOG_STEP_FAIL(hal, sta, mask);
    return false;
  }
  mask |= (1u << 6);

  *distance_out = raw_result * SX1280_VALUES::RANGING_RESULT_TO_CM_MULTIPLIER;

  BEACON_LOG("[%lu] anchor 0x%08lX ranging result: %ld cm\r\n",
             (unsigned long)HAL_GetTick(),
             (unsigned long)ranging_target_address,
             (long)*distance_out);

  return true;
}

void BeaconBridge::log_ranging_no_result(uint16_t irq_mask)
{
  if (irq_mask & SX1280_VALUES::IRQ_BIT_RANGING_MASTER_TIMEOUT) {
    BEACON_LOG("[%lu] anchor 0x%08lX ranging request timed out\r\n", (unsigned long)HAL_GetTick(), (unsigned long)ranging_target_address);
  }
  else {
    BEACON_LOG("[%lu] anchor 0x%08lX ranging request: no result (irq=0x%04X)\r\n",
               (unsigned long)HAL_GetTick(),
               (unsigned long)ranging_target_address,
               irq_mask);
  }
}
