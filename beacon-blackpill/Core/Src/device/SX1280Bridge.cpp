#include <cstddef>
#include <cstdio>
#include <cstdint>

#include "stm32h5xx_hal.h"

#include "SX1280Bridge.h"
#include "SX1280Constants.hpp"
#include "SX1280Device.hpp"

#ifdef DEBUG_PINS
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

void SX1280_Create(SPI_HandleTypeDef* SPI_port,
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
  // COMMAND_TIMEOUT is included because it's the legitimate status GetIrqStatus reports right after a ranging
  // request that genuinely times out (no anchor response) -- without it, step_ok() rejected that exact status
  // and SX1280_Get_Irq_Mask() (then named SX1280_Get_Irq_Status_Raw()) returned its 0xFFFF failure sentinel
  // instead of the real IRQ bits, which has RANGING_MASTER_RESULT_VALID_BIT set and got misread as a valid
  // (stale) result every single timeout.
  return hal == HAL_OK && (sta.command_status == SX1280Device::CommandStatus::COMMAND_SUCCESS ||
                           sta.command_status == SX1280Device::CommandStatus::RESERVED ||
                           sta.command_status == SX1280Device::CommandStatus::COMMAND_TX_DONE ||
                           sta.command_status == SX1280Device::CommandStatus::COMMAND_TIMEOUT);
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
      // only from its leading BUSY_wait() (chip never released BUSY -- not powered/responding at all), and
      // HAL_ERROR only from NSS_begin()/NSS_end() finding NSS already in the wrong state before it toggles.
      // sta stays zero-initialized in both of those early-return cases (SPI_write only writes *out on HAL_OK).
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

HAL_StatusTypeDef SX1280_Get_Irq_Mask(uint16_t* mask_out)
{
  SX1280Device::SX1280_Status sta{};

  uint8_t tx_irq_status[3] = {};
  uint8_t rx_irq_status[3] = {};
  HAL_StatusTypeDef hal = LoRa_SX1280->SPI_write(&SX1280_OPERATIONS::GET_IRQ_STATUS_OP_CODE, tx_irq_status, rx_irq_status, 3, &sta);

  // sta.command_status reflects whatever command ran *before* this GetIrqStatus read (TX_DONE, TIMEOUT, a
  // ranging ResultValid, etc.) -- it says nothing about whether this read itself succeeded, so gating on a
  // command_status whitelist here would be wrong. Only the HAL transfer result (hal) is meaningful here; the
  // caller now gets that directly via the return value instead of it being conflated with the mask itself.
  if (hal != HAL_OK) {
    // 0x0000 (no bits set), not left untouched -- a caller checking RANGING_MASTER_RESULT_VALID_BIT first must
    // not misread a failed read as a valid ranging result.
    *mask_out = 0x0000;
    return hal;
  }

  *mask_out = (static_cast<uint16_t>(rx_irq_status[1]) << 8) | rx_irq_status[2];
  return hal;
}

void SX1280_Clear_Irq_Status_Raw()
{
  SX1280Device::SX1280_Status sta{};
  uint8_t tx_clear_irq[2] = {0xFF, 0xFF};
  LoRa_SX1280->SPI_write(&SX1280_OPERATIONS::CLEAR_IRQ_STATUS_OP_CODE, tx_clear_irq, nullptr, 2, &sta);
}

HAL_StatusTypeDef SX1280_Get_Status(SX1280Device::SX1280_Status* sta)
{
  uint8_t tx_status[1] = {0x00};
  return LoRa_SX1280->SPI_write(&SX1280_OPERATIONS::GET_STATUS_OP_CODE, tx_status, nullptr, 1, sta);
}

uint16_t SX1280_Beacon_Radio()
{
  uint8_t packet[7] = {SX1280_VALUES::LORA_PREAMBLE_12_SYMBOLS,
                       SX1280_VALUES::EXPLICIT_HEADER,
                       LORA_BEACON_PROTOCOL::WAKE_PAYLOAD_LEN,
                       SX1280_VALUES::LORA_CRC_ENABLE,
                       SX1280_VALUES::LORA_IQ_STD,
                       0x00,
                       0x00};

  uint8_t tx_params[2] = {SX1280_VALUES::TX_OUTPUT_POWER, SX1280_VALUES::RADIO_RAMP_04_US};

  constexpr uint16_t tx_done_irq = SX1280_VALUES::IRQ_BIT_TX_DONE;
  uint8_t irq_mask[8] = {static_cast<uint8_t>(tx_done_irq >> 8),
                         static_cast<uint8_t>(tx_done_irq),
                         static_cast<uint8_t>(tx_done_irq >> 8),
                         static_cast<uint8_t>(tx_done_irq),
                         0x0,
                         0x0,
                         0x0,
                         0x0};

  const Sx1280Step radio_steps[] = {
    {"SetStandby", &SX1280_OPERATIONS::SET_STANDBY_OP_CODE, &SX1280_VALUES::STDBY_RC_STAND_BY, 1},
    {"SetPacketType", &SX1280_OPERATIONS::SET_PACKET_TYPE_OP_CODE, &SX1280_VALUES::PACKET_TYPE_LORA, 1},
    {"SetRfFrequency", &SX1280_OPERATIONS::SET_FREQUENCY_OP_CODE, SX1280_VALUES::RF_FREQUENCY_BYTES, 3},
    {"SetBufferBaseAddress", &SX1280_OPERATIONS::SET_BUFFER_BASE_ADDRESS_OP_CODE, SX1280_VALUES::BUFFER_BASE_ADDRESS, 2},
    {"SetModulationParams", &SX1280_OPERATIONS::SET_MODULATION_OP_CODE, SX1280_VALUES::MODULATION_PARAMS_SF7, 3},
    {"SF7RegisterFixup", &SX1280_OPERATIONS::WRITE_REGISTER_OP_CODE, SX1280_VALUES::SF_7_FIXUP_WRITE, 3},
    {"SetPacketParams", &SX1280_OPERATIONS::SET_PACKET_PARAMS_OP_CODE, packet, 7},
    {"SetDioIrqParams", &SX1280_OPERATIONS::SET_DIO_IRQ_PARAMS_OP_CODE, irq_mask, 8},
    {"SetTxParams", &SX1280_OPERATIONS::SET_TX_PARAMS_OP_CODE, tx_params, 2},
  };

  return execute_step(radio_steps, sizeof(radio_steps) / sizeof(radio_steps[0]));
}

uint8_t SX1280_Was_Tx_Done()
{
  SX1280Device::SX1280_Status sta{};

  uint8_t tx_irq_status[3] = {};
  uint8_t rx_irq_status[3] = {};
  HAL_StatusTypeDef hal = LoRa_SX1280->SPI_write(&SX1280_OPERATIONS::GET_IRQ_STATUS_OP_CODE, tx_irq_status, rx_irq_status, 3, &sta);

  if (!step_ok(hal, sta)) {
    return 0;
  }

  uint16_t irq_status = (static_cast<uint16_t>(rx_irq_status[1]) << 8) | rx_irq_status[2];

  uint8_t tx_clear_irq[2] = {0xFF, 0xFF};
  LoRa_SX1280->SPI_write(&SX1280_OPERATIONS::CLEAR_IRQ_STATUS_OP_CODE, tx_clear_irq, nullptr, 2, &sta);

  return (irq_status & SX1280_VALUES::IRQ_BIT_TX_DONE) ? 1 : 0;
}

uint16_t SX1280_Send_Wake_Broadcast()
{
  // Wake payload now carries the ranging-window duration (ms, MSB-first) alongside the magic, so the anchor can
  // arm its ARMED-window timer from a rover-supplied value instead of its own compile-time constant. Sends
  // RANGING_WINDOW_MS for now -- nothing varies this per-cycle yet, but the anchor no longer needs to keep a
  // separately-flashed constant in sync with the beacon's.
  constexpr uint16_t ranging_window_ms = static_cast<uint16_t>(LORA_BEACON_PROTOCOL::RANGING_WINDOW_MS);
  uint8_t write_buffer[1 + LORA_BEACON_PROTOCOL::WAKE_PAYLOAD_LEN] = {0x00,
                                                                      LORA_BEACON_PROTOCOL::WAKE_WORD[0],
                                                                      LORA_BEACON_PROTOCOL::WAKE_WORD[1],
                                                                      static_cast<uint8_t>(ranging_window_ms >> 8),
                                                                      static_cast<uint8_t>(ranging_window_ms & 0xFF)};
  uint8_t tx[3] = {SX1280_VALUES::PERIOD_BASE_1_MS, 0x00, 0x00};  // timeoutCount=0x0000 -> single-shot TX, auto-standby on TxDone

  const Sx1280Step send_steps[] = {
    {"WriteBuffer", &SX1280_OPERATIONS::WRITE_BUFFER_OP_CODE, write_buffer, static_cast<uint16_t>(sizeof(write_buffer))},
    {"SetTx", &SX1280_OPERATIONS::SET_TX_OP_CODE, tx, 3},
  };

  return execute_step(send_steps, sizeof(send_steps) / sizeof(send_steps[0]));
}

uint16_t SX1280_Listen_For_Ack()
{
  // Reuses the frequency/modulation already active from SX1280_Beacon_Radio() (the wake broadcast just went out
  // in that same config) -- but packet params DO need reissuing: the ack payload (magic + anchor's 4-byte
  // address) is longer than the wake exchange's 2-byte payloadLength, and RX buffer-status reporting is governed
  // by SetPacketParams, not by whatever the sender actually transmitted.
  uint8_t packet[7] = {SX1280_VALUES::LORA_PREAMBLE_12_SYMBOLS,
                       SX1280_VALUES::EXPLICIT_HEADER,
                       LORA_BEACON_PROTOCOL::WAKE_ACK_PAYLOAD_LEN,
                       SX1280_VALUES::LORA_CRC_ENABLE,
                       SX1280_VALUES::LORA_IQ_STD,
                       0x00,
                       0x00};

  // Continuous RX (periodBaseCount=0xFFFF, datasheet SetRx behavior table): the chip keeps listening and reports
  // an RxDone indication for EACH incoming packet without needing to be manually re-armed between catches -- what
  // the collect phase in main.c needs to gather multiple anchors' ACKs. RxTxTimeout doesn't apply in this mode
  // (no per-listen timeout), so only RX_DONE is masked; the overall collect-phase ceiling is software-driven
  // (main.c), not chip-driven.
  constexpr uint16_t ack_irq_bits = SX1280_VALUES::IRQ_BIT_RX_DONE;
  uint8_t irq_mask[8] = {static_cast<uint8_t>(ack_irq_bits >> 8),
                         static_cast<uint8_t>(ack_irq_bits),
                         static_cast<uint8_t>(ack_irq_bits >> 8),
                         static_cast<uint8_t>(ack_irq_bits),
                         0x0,
                         0x0,
                         0x0,
                         0x0};
  uint8_t rx[3] = {SX1280_VALUES::PERIOD_BASE_1_MS, 0xFF, 0xFF};

  const Sx1280Step steps[] = {
    {"SetPacketParams", &SX1280_OPERATIONS::SET_PACKET_PARAMS_OP_CODE, packet, 7},
    {"SetDioIrqParams", &SX1280_OPERATIONS::SET_DIO_IRQ_PARAMS_OP_CODE, irq_mask, 8},
    {"SetRx", &SX1280_OPERATIONS::SET_RX_OP_CODE, rx, 3},
  };

  return execute_step(steps, sizeof(steps) / sizeof(steps[0]));
}

uint16_t SX1280_Stop_Ack_Listen()
{
  // Continuous RX doesn't stop on its own -- must explicitly SetStandby before reconfiguring for ranging, since
  // SX1280_Ranging_Master_Mode() doesn't start with its own SetStandby (assumes the caller already left the chip
  // in STDBY_RC, same assumption the rest of this codebase already relies on).
  const Sx1280Step steps[] = {
    {"SetStandby", &SX1280_OPERATIONS::SET_STANDBY_OP_CODE, &SX1280_VALUES::STDBY_RC_STAND_BY, 1},
  };
  return execute_step(steps, sizeof(steps) / sizeof(steps[0]));
}

uint16_t SX1280_Check_Wake_Ack_Matches(uint32_t* anchor_address_out)
{
  // No leading ClearIrqStatus here (unlike the anchor's SX1280_Check_Wake_Word_Matches()) -- the caller in
  // main.c already clears IRQ status when it picks up DIO1_Callback_detected for this phase.
  SX1280Device::SX1280_Status sta{};

  uint8_t tx_buffer_status[3] = {};
  uint8_t rx_buffer_status[3] = {};

  HAL_StatusTypeDef hal =
    LoRa_SX1280->SPI_write(&SX1280_OPERATIONS::READ_BUFFER_STATUS_OP_CODE, tx_buffer_status, rx_buffer_status, 3, &sta);

  if (!step_ok(hal, sta)) {
#ifdef DEBUG_PINS
    printf("[%lu] SX1280_Check_Wake_Ack_Matches: ReadBufferStatus failed hal=%d cmd_status=%d busy=%d\r\n",
           (unsigned long)HAL_GetTick(), static_cast<int>(hal), static_cast<int>(sta.command_status), static_cast<int>(sta.busy));
#endif
    return 0;
  }

  uint8_t rx_len = rx_buffer_status[1];
  uint8_t rx_start = rx_buffer_status[2];

  constexpr uint8_t PAYLOAD_LEN = LORA_BEACON_PROTOCOL::WAKE_ACK_PAYLOAD_LEN;
  if (rx_len != PAYLOAD_LEN) {
#ifdef DEBUG_PINS
    printf("[%lu] SX1280_Check_Wake_Ack_Matches: length mismatch rx_len=%u expected=%u\r\n",
           (unsigned long)HAL_GetTick(), rx_len, PAYLOAD_LEN);
#endif
    return 0;
  }

  uint8_t tx_read_buffer[2 + PAYLOAD_LEN] = {rx_start};
  uint8_t rx_read_buffer[2 + PAYLOAD_LEN] = {};
  sta = {};
  hal = LoRa_SX1280->SPI_write(
    &SX1280_OPERATIONS::READ_BUFFER_OP_CODE, tx_read_buffer, rx_read_buffer, static_cast<uint16_t>(sizeof(tx_read_buffer)), &sta);

  if (!step_ok(hal, sta)) {
#ifdef DEBUG_PINS
    printf("[%lu] SX1280_Check_Wake_Ack_Matches: ReadBuffer failed hal=%d cmd_status=%d busy=%d\r\n",
           (unsigned long)HAL_GetTick(), static_cast<int>(hal), static_cast<int>(sta.command_status), static_cast<int>(sta.busy));
#endif
    return 0;
  }

  const uint8_t* payload = &rx_read_buffer[2];
  constexpr uint8_t MAGIC_LEN = sizeof(LORA_BEACON_PROTOCOL::WAKE_ACK);
  for (uint8_t i = 0; i < MAGIC_LEN; ++i) {
    if (payload[i] != LORA_BEACON_PROTOCOL::WAKE_ACK[i]) {
#ifdef DEBUG_PINS
      printf("[%lu] SX1280_Check_Wake_Ack_Matches: magic mismatch at byte %u got=0x%02X want=0x%02X\r\n",
             (unsigned long)HAL_GetTick(), i, payload[i], LORA_BEACON_PROTOCOL::WAKE_ACK[i]);
#endif
      return 0;
    }
  }

  if (anchor_address_out != nullptr) {
    const uint8_t* addr_bytes = &payload[MAGIC_LEN];
    *anchor_address_out = (static_cast<uint32_t>(addr_bytes[0]) << 24) | (static_cast<uint32_t>(addr_bytes[1]) << 16) |
                          (static_cast<uint32_t>(addr_bytes[2]) << 8) | static_cast<uint32_t>(addr_bytes[3]);
  }

  return 1;
}

uint16_t SX1280_Ranging_Master_Mode(uint32_t target_anchor_address)
{
  uint8_t packet[7] = {SX1280_VALUES::LORA_PREAMBLE_12_SYMBOLS,
                       SX1280_VALUES::EXPLICIT_HEADER,
                       0x02,
                       SX1280_VALUES::LORA_CRC_DISABLE,
                       SX1280_VALUES::LORA_IQ_STD,
                       0x00,
                       0x00};

  uint8_t target_address[6] = {static_cast<uint8_t>(SX1280_VALUES::REG_RANGING_MASTER_TARGET_ADDR >> 8),
                               static_cast<uint8_t>(SX1280_VALUES::REG_RANGING_MASTER_TARGET_ADDR & 0xFF),
                               static_cast<uint8_t>(target_anchor_address >> 24),
                               static_cast<uint8_t>(target_anchor_address >> 16),
                               static_cast<uint8_t>(target_anchor_address >> 8),
                               static_cast<uint8_t>(target_anchor_address & 0xFF)};

  uint8_t addr_check_len[3] = {static_cast<uint8_t>(SX1280_VALUES::REG_RANGING_ADDR_CHECK_LEN >> 8),
                               static_cast<uint8_t>(SX1280_VALUES::REG_RANGING_ADDR_CHECK_LEN & 0xFF),
                               0x00};  // bits[7:6] = 0x0 -> 8-bit check, must match anchor's RangingAddrCheckLen

  uint8_t role[1] = {SX1280_VALUES::RANGING_ROLE_MASTER};

  // RxTx-delay calibration (datasheet Table 13-59) -- see RANGING_CALIBRATION_VALUE's own comment for the
  // empirical derivation (two known-distance samples on real hardware, calibration=0).
  uint8_t calibration[4] = {static_cast<uint8_t>(SX1280_VALUES::REG_RANGING_CALIBRATION >> 8),
                            static_cast<uint8_t>(SX1280_VALUES::REG_RANGING_CALIBRATION & 0xFF),
                            static_cast<uint8_t>(SX1280_VALUES::RANGING_CALIBRATION_VALUE >> 8),
                            static_cast<uint8_t>(SX1280_VALUES::RANGING_CALIBRATION_VALUE & 0xFF)};

  constexpr uint16_t ranging_irq_bits = SX1280_VALUES::IRQ_BIT_RANGING_MASTER_RESULT_VALID | SX1280_VALUES::IRQ_BIT_RANGING_MASTER_TIMEOUT;
  uint8_t irq_mask[8] = {static_cast<uint8_t>(ranging_irq_bits >> 8),
                         static_cast<uint8_t>(ranging_irq_bits),
                         static_cast<uint8_t>(ranging_irq_bits >> 8),
                         static_cast<uint8_t>(ranging_irq_bits),
                         0x0,
                         0x0,
                         0x0,
                         0x0};

  const Sx1280Step ranging_steps[] = {
    {"SetPacketType", &SX1280_OPERATIONS::SET_PACKET_TYPE_OP_CODE, &SX1280_VALUES::PACKET_TYPE_RANGING, 1},
    {"SetRfFrequency", &SX1280_OPERATIONS::SET_FREQUENCY_OP_CODE, SX1280_VALUES::RF_FREQUENCY_BYTES, 3},
    {"SetModulationParams", &SX1280_OPERATIONS::SET_MODULATION_OP_CODE, SX1280_VALUES::MODULATION_PARAMS_SF7, 3},
    {"SF7RegisterFixup", &SX1280_OPERATIONS::WRITE_REGISTER_OP_CODE, SX1280_VALUES::SF_7_FIXUP_WRITE, 3},
    {"SetPacketParams", &SX1280_OPERATIONS::SET_PACKET_PARAMS_OP_CODE, packet, 7},
    {"TargetRangingAddress", &SX1280_OPERATIONS::WRITE_REGISTER_OP_CODE, target_address, 6},
    {"RangingAddrCheckLen", &SX1280_OPERATIONS::WRITE_REGISTER_OP_CODE, addr_check_len, 3},
    {"RangingCalibration", &SX1280_OPERATIONS::WRITE_REGISTER_OP_CODE, calibration, 4},
    {"SetRangingRole", &SX1280_OPERATIONS::SET_RANGING_ROLE_OP_CODE, role, 1},
    {"SetDioIrqParams", &SX1280_OPERATIONS::SET_DIO_IRQ_PARAMS_OP_CODE, irq_mask, 8},
  };

  return execute_step(ranging_steps, sizeof(ranging_steps) / sizeof(ranging_steps[0]));
}

uint16_t SX1280_Send_Ranging_Request()
{
  uint8_t tx[3] = {SX1280_VALUES::PERIOD_BASE_1_MS, 0x03, 0xE8};  // 1000ms timeout, comfortably inside anchor's 2000ms RANGING_WINDOW_MS

  const Sx1280Step send_steps[] = {
    {"SetTx", &SX1280_OPERATIONS::SET_TX_OP_CODE, tx, 3},
  };

  return execute_step(send_steps, sizeof(send_steps) / sizeof(send_steps[0]));
}

// Readback dance per SX1280 datasheet (ranging result is only valid on IRQ_BIT_RANGING_MASTER_RESULT_VALID), plus
// one step beyond the datasheet's literal 4: SetStandby(XOSC) -> enable LoRa memory clock (0x97F bit1) -> select
// debiased result mux (0x924[5:4]) -> read the 3-byte result at 0x961-0x963 -> restore the memory clock bit to its
// original state (not in the datasheet's own procedure, but leaving it permanently forced open broke later
// exchanges -- see the comment at that step) -> SetStandby(RC). Returns 0 (and leaves *distance_cm_out untouched)
// on any SPI step failing partway through.
uint8_t SX1280_Read_Ranging_Result_Cm(int32_t* distance_cm_out)
{
  SX1280Device::SX1280_Status sta{};
  HAL_StatusTypeDef hal;

  hal = LoRa_SX1280->SPI_write(&SX1280_OPERATIONS::SET_STANDBY_OP_CODE, &SX1280_VALUES::STDBY_XOSC_STAND_BY, nullptr, 1, &sta);
  if (!step_ok(hal, sta)) {
    return 0;
  }

  uint8_t read_clock_tx[4] = {static_cast<uint8_t>(SX1280_VALUES::REG_LORA_MEM_CLOCK_ENABLE >> 8),
                              static_cast<uint8_t>(SX1280_VALUES::REG_LORA_MEM_CLOCK_ENABLE & 0xFF),
                              0x00,
                              0x00};
  uint8_t read_clock_rx[4] = {};
  hal = LoRa_SX1280->SPI_write(&SX1280_OPERATIONS::READ_REGISTER_OP_CODE, read_clock_tx, read_clock_rx, 4, &sta);
  if (!step_ok(hal, sta)) {
    return 0;
  }
  uint8_t mem_clock_reg = read_clock_rx[3];

  uint8_t write_clock_tx[3] = {static_cast<uint8_t>(SX1280_VALUES::REG_LORA_MEM_CLOCK_ENABLE >> 8),
                               static_cast<uint8_t>(SX1280_VALUES::REG_LORA_MEM_CLOCK_ENABLE & 0xFF),
                               static_cast<uint8_t>(mem_clock_reg | (1u << 1))};
  hal = LoRa_SX1280->SPI_write(&SX1280_OPERATIONS::WRITE_REGISTER_OP_CODE, write_clock_tx, nullptr, 3, &sta);
  if (!step_ok(hal, sta)) {
    return 0;
  }

  uint8_t mux_tx[3] = {static_cast<uint8_t>(SX1280_VALUES::REG_RANGING_RESULT_MUX >> 8),
                       static_cast<uint8_t>(SX1280_VALUES::REG_RANGING_RESULT_MUX & 0xFF),
                       static_cast<uint8_t>(SX1280_VALUES::RANGING_RESULT_DEBIASED << 4)};
  hal = LoRa_SX1280->SPI_write(&SX1280_OPERATIONS::WRITE_REGISTER_OP_CODE, mux_tx, nullptr, 3, &sta);
  if (!step_ok(hal, sta)) {
    return 0;
  }

  uint8_t result_tx[6] = {static_cast<uint8_t>(SX1280_VALUES::REG_RANGING_RESULT_MSB >> 8),
                          static_cast<uint8_t>(SX1280_VALUES::REG_RANGING_RESULT_MSB & 0xFF),
                          0x00,
                          0x00,
                          0x00,
                          0x00};
  uint8_t result_rx[6] = {};
  hal = LoRa_SX1280->SPI_write(&SX1280_OPERATIONS::READ_REGISTER_OP_CODE, result_tx, result_rx, 6, &sta);
  if (!step_ok(hal, sta)) {
    return 0;
  }

  int32_t raw_result =
    (static_cast<int32_t>(result_rx[3]) << 16) | (static_cast<int32_t>(result_rx[4]) << 8) | static_cast<int32_t>(result_rx[5]);

  // Restore the LoRa memory clock enable bit to its original (pre-readback) state -- the datasheet's own 4-step
  // procedure never says to undo the bit it has you set, but leaving that clock domain permanently forced open
  // after the first successful read left the ranging engine unable to commit fresh results into these registers
  // on later exchanges, producing the same stale distance every cycle until a full chip reset (NRESET, e.g. via
  // reflash) happened to clear it.
  uint8_t restore_clock_tx[3] = {static_cast<uint8_t>(SX1280_VALUES::REG_LORA_MEM_CLOCK_ENABLE >> 8),
                                 static_cast<uint8_t>(SX1280_VALUES::REG_LORA_MEM_CLOCK_ENABLE & 0xFF),
                                 mem_clock_reg};
  hal = LoRa_SX1280->SPI_write(&SX1280_OPERATIONS::WRITE_REGISTER_OP_CODE, restore_clock_tx, nullptr, 3, &sta);
  if (!step_ok(hal, sta)) {
    return 0;
  }

  hal = LoRa_SX1280->SPI_write(&SX1280_OPERATIONS::SET_STANDBY_OP_CODE, &SX1280_VALUES::STDBY_RC_STAND_BY, nullptr, 1, &sta);
  if (!step_ok(hal, sta)) {
    return 0;
  }

  *distance_cm_out = raw_result * SX1280_VALUES::RANGING_RESULT_TO_CM_MULTIPLIER;
  return 1;
}
