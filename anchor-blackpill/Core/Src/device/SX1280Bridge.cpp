#include <cstddef>
#include <cstdio>
#include <stdint.h>

#include "stm32h5xx_hal.h"

#include "SX1280Bridge.h"
#include "SX1280Constants.hpp"
#include "SX1280Device.hpp"

#ifdef DEBUG_PINS
#include <cstdio>
static void debug_print_step(const char* name, HAL_StatusTypeDef hal, const SX1280Device::SX1280_Status& sta)
{
  printf("[%lu] %s: hal=%d cmd_status=%d busy=%d\r\n",
         (unsigned long)HAL_GetTick(),
         name,
         static_cast<int>(hal),
         static_cast<int>(sta.command_status),
         static_cast<int>(sta.busy));
}
#define DEBUG_STEP(name, hal, sta) debug_print_step(name, hal, sta)
#else
#define DEBUG_STEP(name, hal, sta)
#endif

static SX1280Device* LoRa_SX1280 = nullptr;

// TODO: must be unique per physical anchor board, drawn from the shared discovery block (e.g. 0xA19-0xA30) --
// change this before flashing each board. 0xA19 here is just the block's first address, not a real deployment
// value. File-scope (not local to SX1280_Ranging_Slave_Mode()) so SX1280_Send_Wake_Ack() can also embed it in the
// ack payload -- the beacon learns this address from the ack instead of only trusting its own hardcoded copy.
static constexpr uint32_t ANCHOR_RANGING_ADDRESS = 0x00000A19;
// Multi-anchor ACK collision avoidance (see PROTOCOL.md) -- derived purely from this board's own address, no
// runtime coordination with the beacon needed.
static constexpr uint32_t ACK_SLOT_DELAY_MS =
  (ANCHOR_RANGING_ADDRESS - LORA_BEACON_PROTOCOL::RANGING_ADDRESS_BLOCK_BASE) * LORA_BEACON_PROTOCOL::ANCHOR_ACK_SLOT_WIDTH_MS;

extern "C" void SX1280_Create(SPI_HandleTypeDef* SPI_port,
                              GPIO_TypeDef* BUSY_GPIO_port,
                              GPIO_TypeDef* NSS_GPIO_port,
                              GPIO_TypeDef* NRESET_GPIO_port,
                              GPIO_TypeDef* TCXOEN_GPIO_port,
                              uint16_t BUSY_pin,
                              uint16_t NSS_pin,
                              uint16_t NRESET_pin,
                              uint16_t TCXOEN_pin)
{
  if (LoRa_SX1280 == nullptr) {
    static SX1280Device device(
      SPI_port, BUSY_GPIO_port, NSS_GPIO_port, NRESET_GPIO_port, TCXOEN_GPIO_port, BUSY_pin, NSS_pin, NRESET_pin, TCXOEN_pin);
    LoRa_SX1280 = &device;
  }

  LoRa_SX1280->NRESET_reset();
}

static bool step_ok(HAL_StatusTypeDef hal, const SX1280Device::SX1280_Status& sta)
{
  return hal == HAL_OK &&
         (sta.command_status == SX1280Device::CommandStatus::COMMAND_SUCCESS ||
          sta.command_status == SX1280Device::CommandStatus::RESERVED || sta.command_status == SX1280Device::CommandStatus::DATA_AVAILABLE);
}

struct Sx1280Step {
  const char* name;
  const uint8_t* op_code;
  const uint8_t* tx;
  uint16_t len;
};

static uint16_t execute_step(const Sx1280Step* steps, size_t count)
{
  if (LoRa_SX1280 == nullptr) {
    return 0;
  }

  uint16_t mask = 0;
  SX1280Device::SX1280_Status sta{};

  for (size_t i = 0; i < count; ++i) {
    HAL_StatusTypeDef hal = LoRa_SX1280->SPI_write(steps[i].op_code, steps[i].tx, nullptr, steps[i].len, &sta);
    DEBUG_STEP(steps[i].name, hal, sta);
    if (!step_ok(hal, sta)) {
      // hal: HAL_OK=0, HAL_ERROR=1, HAL_BUSY=2, HAL_TIMEOUT=3 (stm32*_hal_def.h). SPI_write() returns HAL_TIMEOUT
      // only from its leading BUSY_wait() (chip never released BUSY), and HAL_ERROR only from NSS_begin()/NSS_end()
      // finding NSS already in the wrong state before it toggles. sta stays zero-initialized in both early-return
      // cases (SPI_write only writes *out on HAL_OK).
      printf("[%lu] step %s failed: hal=%d circuit_mode=%d cmd_status=%d busy=%d\r\n",
             (unsigned long)HAL_GetTick(),
             steps[i].name,
             static_cast<int>(hal),
             static_cast<int>(sta.circuit_mode),
             static_cast<int>(sta.command_status),
             static_cast<int>(sta.busy));
      return mask;
    }
    mask |= static_cast<uint16_t>(1u << i);
  }

  return mask;
}

extern "C" uint16_t SX1280_Radio_mode()
{
  uint8_t packet[7] = {SX1280_VALUES::LORA_PREAMBLE_12_SYMBOLS,
                       SX1280_VALUES::EXPLICIT_HEADER,
                       LORA_BEACON_PROTOCOL::WAKE_PAYLOAD_LEN,
                       SX1280_VALUES::LORA_CRC_ENABLE,
                       SX1280_VALUES::LORA_IQ_STD,
                       0x00,
                       0x00};
  uint8_t irq_mask[8] = {static_cast<uint8_t>(SX1280_VALUES::IRQ_BIT_RX_DONE >> 8),
                         static_cast<uint8_t>(SX1280_VALUES::IRQ_BIT_RX_DONE),
                         static_cast<uint8_t>(SX1280_VALUES::IRQ_BIT_RX_DONE >> 8),
                         static_cast<uint8_t>(SX1280_VALUES::IRQ_BIT_RX_DONE),
                         0x0,
                         0x0,
                         0x0,
                         0x0};
  // Duty cycling disabled for now (bring-up: testing ranging without the wake-catch-probability confound) --
  // continuous SetRx below instead. Restore SetLongPreamble+SetRxDutyCycle (using rx_duty) once ranging itself
  // is proven and it's time to bring idle power draw back down.
  // uint8_t rx_duty[5] = {SX1280_VALUES::PERIOD_BASE_1_MS,
  //                       static_cast<uint8_t>(SX1280_VALUES::ANCHOR_IDLE_RX_PERIOD_BASE_COUNT >> 8),
  //                       static_cast<uint8_t>(SX1280_VALUES::ANCHOR_IDLE_RX_PERIOD_BASE_COUNT),
  //                       static_cast<uint8_t>(SX1280_VALUES::ANCHOR_IDLE_SLEEP_PERIOD_BASE_COUNT >> 8),
  //                       static_cast<uint8_t>(SX1280_VALUES::ANCHOR_IDLE_SLEEP_PERIOD_BASE_COUNT)};
  uint8_t rx_continuous[3] = {SX1280_VALUES::PERIOD_BASE_1_MS, 0x00, 0x00};  // count=0x0000 -> listen indefinitely

  const Sx1280Step radio_steps[] = {
    {"SetStandby", &SX1280_OPERATIONS::SET_STANDBY_OP_CODE, &SX1280_VALUES::STDBY_RC_STAND_BY, 1},
    {"SetPacketType", &SX1280_OPERATIONS::SET_PACKET_TYPE_OP_CODE, &SX1280_VALUES::PACKET_TYPE_LORA, 1},
    {"SetRfFrequency", &SX1280_OPERATIONS::SET_FREQUENCY_OP_CODE, SX1280_VALUES::RF_FREQUENCY_BYTES, 3},
    {"SetBufferBaseAddress", &SX1280_OPERATIONS::SET_BUFFER_BASE_ADDRESS_OP_CODE, SX1280_VALUES::BUFFER_BASE_ADDRESS, 2},
    {"SetModulationParams", &SX1280_OPERATIONS::SET_MODULATION_OP_CODE, SX1280_VALUES::MODULATION_PARAMS_SF7, 3},
    {"SF7RegisterFixup", &SX1280_OPERATIONS::WRITE_REGISTER_OP_CODE, SX1280_VALUES::SF_7_FIXUP_WRITE, 3},
    {"SetPacketParams", &SX1280_OPERATIONS::SET_PACKET_PARAMS_OP_CODE, packet, 7},
    {"SetDioIrqParams", &SX1280_OPERATIONS::SET_DIO_IRQ_PARAMS_OP_CODE, irq_mask, 8},
    // {"SetLongPreamble", &SX1280_OPERATIONS::SET_LONG_PREAMBLE_OP_CODE, &SX1280_VALUES::LONG_PREAMBLE_ENABLE, 1},
    // {"SetRxDutyCycle", &SX1280_OPERATIONS::SET_RX_DUTY_CYCLE_OP_CODE, rx_duty, 5},
    {"SetRx", &SX1280_OPERATIONS::SET_RX_OP_CODE, rx_continuous, 3},
  };

  return execute_step(radio_steps, sizeof(radio_steps) / sizeof(radio_steps[0]));
}

extern "C" uint16_t SX1280_Ranging_Slave_Mode()
{
  uint8_t packet[7] = {SX1280_VALUES::LORA_PREAMBLE_12_SYMBOLS,
                       SX1280_VALUES::EXPLICIT_HEADER,
                       0x02,
                       SX1280_VALUES::LORA_CRC_DISABLE,
                       SX1280_VALUES::LORA_IQ_STD,
                       0x00,
                       0x00};

  uint8_t own_address[6] = {static_cast<uint8_t>(SX1280_VALUES::REG_RANGING_SLAVE_OWN_ADDR >> 8),
                            static_cast<uint8_t>(SX1280_VALUES::REG_RANGING_SLAVE_OWN_ADDR & 0xFF),
                            static_cast<uint8_t>(ANCHOR_RANGING_ADDRESS >> 24),
                            static_cast<uint8_t>(ANCHOR_RANGING_ADDRESS >> 16),
                            static_cast<uint8_t>(ANCHOR_RANGING_ADDRESS >> 8),
                            static_cast<uint8_t>(ANCHOR_RANGING_ADDRESS & 0xFF)};

  uint8_t addr_check_len[3] = {static_cast<uint8_t>(SX1280_VALUES::REG_RANGING_ADDR_CHECK_LEN >> 8),
                               static_cast<uint8_t>(SX1280_VALUES::REG_RANGING_ADDR_CHECK_LEN & 0xFF),
                               0x00};  // bits[7:6] = 0x0 -> 8-bit check, sufficient since the discovery block only varies in the low byte

  uint8_t role[1] = {0x00};  // 0x00 = Slave (SET_RANGING_ROLE_OP_CODE)

  // RxTx-delay calibration (datasheet Table 13-59) -- see RANGING_CALIBRATION_VALUE's own comment for the
  // empirical derivation (two known-distance samples on real hardware, calibration=0).
  uint8_t calibration[4] = {static_cast<uint8_t>(SX1280_VALUES::REG_RANGING_CALIBRATION >> 8),
                            static_cast<uint8_t>(SX1280_VALUES::REG_RANGING_CALIBRATION & 0xFF),
                            static_cast<uint8_t>(SX1280_VALUES::RANGING_CALIBRATION_VALUE >> 8),
                            static_cast<uint8_t>(SX1280_VALUES::RANGING_CALIBRATION_VALUE & 0xFF)};

  constexpr uint16_t ranging_irq_bits =
    SX1280_VALUES::IRQ_BIT_RANGING_SLAVE_RESPONSE_DONE | SX1280_VALUES::IRQ_BIT_RANGING_MASTER_REQUEST_VALID;
  uint8_t irq_mask[8] = {static_cast<uint8_t>(ranging_irq_bits >> 8),
                         static_cast<uint8_t>(ranging_irq_bits),
                         static_cast<uint8_t>(ranging_irq_bits >> 8),
                         static_cast<uint8_t>(ranging_irq_bits),
                         0x0,
                         0x0,
                         0x0,
                         0x0};

  uint8_t rx[3] = {SX1280_VALUES::PERIOD_BASE_1_MS, 0x00, 0x00};  // count=0x0000 -> listen indefinitely

  const Sx1280Step ranging_steps[] = {
    {"SetPacketType", &SX1280_OPERATIONS::SET_PACKET_TYPE_OP_CODE, &SX1280_VALUES::PACKET_TYPE_RANGING, 1},
    {"SetRfFrequency", &SX1280_OPERATIONS::SET_FREQUENCY_OP_CODE, SX1280_VALUES::RF_FREQUENCY_BYTES, 3},
    {"SetModulationParams", &SX1280_OPERATIONS::SET_MODULATION_OP_CODE, SX1280_VALUES::MODULATION_PARAMS_SF7, 3},
    {"SF7RegisterFixup", &SX1280_OPERATIONS::WRITE_REGISTER_OP_CODE, SX1280_VALUES::SF_7_FIXUP_WRITE, 3},
    {"SetPacketParams", &SX1280_OPERATIONS::SET_PACKET_PARAMS_OP_CODE, packet, 7},
    {"OwnRangingAddress", &SX1280_OPERATIONS::WRITE_REGISTER_OP_CODE, own_address, 6},
    {"RangingAddrCheckLen", &SX1280_OPERATIONS::WRITE_REGISTER_OP_CODE, addr_check_len, 3},
    {"RangingCalibration", &SX1280_OPERATIONS::WRITE_REGISTER_OP_CODE, calibration, 4},
    {"SetRangingRole", &SX1280_OPERATIONS::SET_RANGING_ROLE_OP_CODE, role, 1},
    {"SetDioIrqParams", &SX1280_OPERATIONS::SET_DIO_IRQ_PARAMS_OP_CODE, irq_mask, 8},
    {"SetRx", &SX1280_OPERATIONS::SET_RX_OP_CODE, rx, 3},
  };

  return execute_step(ranging_steps, sizeof(ranging_steps) / sizeof(ranging_steps[0]));
}

extern "C" uint32_t SX1280_Ranging_Window_Ms()
{
  return LORA_BEACON_PROTOCOL::RANGING_WINDOW_MS;
}

extern "C" uint16_t SX1280_Check_Wake_Word_Matches(uint32_t* ranging_window_ms_out)
{
  SX1280Device::SX1280_Status sta{};

  uint8_t tx_clear_irq[2] = {0xFF, 0xFF};

  auto hal = LoRa_SX1280->SPI_write(&SX1280_OPERATIONS::CLEAR_IRQ_STATUS_OP_CODE, tx_clear_irq, nullptr, 2, &sta);
  if (!step_ok(hal, sta)) {
    return 0;
  }
  sta = {};

  uint8_t tx_buffer_status[3] = {};
  uint8_t rx_buffer_status[3] = {};

  hal = LoRa_SX1280->SPI_write(&SX1280_OPERATIONS::READ_BUFFER_STATUS_OP_CODE, tx_buffer_status, rx_buffer_status, 3, &sta);

  if (!step_ok(hal, sta)) {
    return 0;
  }

  uint8_t rx_len = rx_buffer_status[1];
  uint8_t rx_start = rx_buffer_status[2];

  constexpr uint8_t PAYLOAD_LEN = LORA_BEACON_PROTOCOL::WAKE_PAYLOAD_LEN;
  if (rx_len != PAYLOAD_LEN) {
    return 0;
  }

  uint8_t tx_read_buffer[2 + PAYLOAD_LEN] = {rx_start};
  uint8_t rx_read_buffer[2 + PAYLOAD_LEN] = {};
  hal = LoRa_SX1280->SPI_write(
    &SX1280_OPERATIONS::READ_BUFFER_OP_CODE, tx_read_buffer, rx_read_buffer, static_cast<uint16_t>(sizeof(tx_read_buffer)), &sta);

  if (!step_ok(hal, sta)) {
    return 0;
  }

  const uint8_t* payload = &rx_read_buffer[2];
  constexpr uint8_t WAKE_WORD_LEN = sizeof(LORA_BEACON_PROTOCOL::WAKE_WORD);
  for (uint8_t i = 0; i < WAKE_WORD_LEN; ++i) {
    if (payload[i] != LORA_BEACON_PROTOCOL::WAKE_WORD[i]) {
      return 0;
    }
  }

  if (ranging_window_ms_out != nullptr) {
    const uint8_t* duration_bytes = &payload[WAKE_WORD_LEN];
    uint32_t received_ms = (static_cast<uint32_t>(duration_bytes[0]) << 8) | static_cast<uint32_t>(duration_bytes[1]);

    // Untrusted (arrived over radio) -- clamp before it's ever used to arm a timer, per PLAN.md's own stated
    // requirement for this feature.
    if (received_ms < LORA_BEACON_PROTOCOL::RANGING_WINDOW_MIN_MS) {
      received_ms = LORA_BEACON_PROTOCOL::RANGING_WINDOW_MIN_MS;
    } else if (received_ms > LORA_BEACON_PROTOCOL::RANGING_WINDOW_MAX_MS) {
      received_ms = LORA_BEACON_PROTOCOL::RANGING_WINDOW_MAX_MS;
    }
    *ranging_window_ms_out = received_ms;
  }

  return 1;
}

extern "C" uint16_t SX1280_Send_Wake_Ack()
{
  // No SetStandby/SetPacketType/SetModulationParams needed here: SX1280_Radio_mode()'s periodBaseCount=0x0000
  // SetRx is single-shot (per the datasheet, it auto-returns to STDBY_RC the moment RxDone fires), so by the time
  // a wake-word match is being checked the chip is already in STDBY_RC with the same LoRa/frequency/modulation
  // still active. Packet params DO need reissuing though -- the ack payload (magic + this anchor's own 4-byte
  // ranging address) is longer than SX1280_Radio_mode()'s 2-byte payloadLength, and TX length is governed by
  // SetPacketParams, not by however many bytes WriteBuffer wrote.
  uint8_t packet[7] = {SX1280_VALUES::LORA_PREAMBLE_12_SYMBOLS,
                       SX1280_VALUES::EXPLICIT_HEADER,
                       LORA_BEACON_PROTOCOL::WAKE_ACK_PAYLOAD_LEN,
                       SX1280_VALUES::LORA_CRC_ENABLE,
                       SX1280_VALUES::LORA_IQ_STD,
                       0x00,
                       0x00};

  uint8_t write_buffer[1 + LORA_BEACON_PROTOCOL::WAKE_ACK_PAYLOAD_LEN] = {
    0x00,
    LORA_BEACON_PROTOCOL::WAKE_ACK[0],
    LORA_BEACON_PROTOCOL::WAKE_ACK[1],
    static_cast<uint8_t>(ANCHOR_RANGING_ADDRESS >> 24),
    static_cast<uint8_t>(ANCHOR_RANGING_ADDRESS >> 16),
    static_cast<uint8_t>(ANCHOR_RANGING_ADDRESS >> 8),
    static_cast<uint8_t>(ANCHOR_RANGING_ADDRESS & 0xFF)};
  uint8_t tx[3] = {SX1280_VALUES::PERIOD_BASE_1_MS, 0x00, 0x00};  // timeoutCount=0x0000 -> single-shot TX

  const Sx1280Step send_steps[] = {
    {"SetPacketParams", &SX1280_OPERATIONS::SET_PACKET_PARAMS_OP_CODE, packet, 7},
    {"WriteBuffer", &SX1280_OPERATIONS::WRITE_BUFFER_OP_CODE, write_buffer, static_cast<uint16_t>(sizeof(write_buffer))},
    {"SetTx", &SX1280_OPERATIONS::SET_TX_OP_CODE, tx, 3},
  };

  return execute_step(send_steps, sizeof(send_steps) / sizeof(send_steps[0]));
}

extern "C" uint32_t SX1280_Ack_Slot_Delay_Ms()
{
  return ACK_SLOT_DELAY_MS;
}

extern "C" uint16_t SX1280_Get_Irq_Status_Raw(uint8_t* hal_status_out, uint8_t* command_status_out)
{
  SX1280Device::SX1280_Status sta{};

  uint8_t tx_irq_status[3] = {};
  uint8_t rx_irq_status[3] = {};
  HAL_StatusTypeDef hal = LoRa_SX1280->SPI_write(&SX1280_OPERATIONS::GET_IRQ_STATUS_OP_CODE, tx_irq_status, rx_irq_status, 3, &sta);

  if (hal_status_out != nullptr) {
    *hal_status_out = static_cast<uint8_t>(hal);
  }
  if (command_status_out != nullptr) {
    *command_status_out = static_cast<uint8_t>(sta.command_status);
  }

  // command_status reflects whatever command ran *before* this read, not whether this read itself succeeded --
  // see the identical fix on the beacon side (SX1280_Get_Irq_Status_Raw()) for why this must gate on hal alone.
  if (hal != HAL_OK) {
    return 0x0000;
  }

  return (static_cast<uint16_t>(rx_irq_status[1]) << 8) | rx_irq_status[2];
}

extern "C" void SX1280_Clear_Irq_Status_Raw()
{
  SX1280Device::SX1280_Status sta{};
  uint8_t tx_clear_irq[2] = {0xFF, 0xFF};
  LoRa_SX1280->SPI_write(&SX1280_OPERATIONS::CLEAR_IRQ_STATUS_OP_CODE, tx_clear_irq, nullptr, 2, &sta);
}
