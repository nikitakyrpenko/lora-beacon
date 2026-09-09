#include <cstdint>

namespace SX1280_OPERATIONS {
static constexpr uint8_t GET_STATUS_OP_CODE = 0xC0;

static constexpr uint8_t SET_STANDBY_OP_CODE = 0x80;
static constexpr uint8_t SET_PACKET_TYPE_OP_CODE = 0x8A;
static constexpr uint8_t SET_FREQUENCY_OP_CODE = 0x86;
static constexpr uint8_t SET_BUFFER_BASE_ADDRESS_OP_CODE = 0x8F;
static constexpr uint8_t SET_MODULATION_OP_CODE = 0x8B;
static constexpr uint8_t SET_PACKET_PARAMS_OP_CODE = 0x8C;

static constexpr uint8_t WRITE_REGISTER_OP_CODE = 0x18;
static constexpr uint8_t READ_REGISTER_OP_CODE = 0x19;

static constexpr uint8_t WRITE_BUFFER_OP_CODE = 0x1A;
static constexpr uint8_t READ_BUFFER_OP_CODE = 0x1B;

static constexpr uint8_t READ_BUFFER_STATUS_OP_CODE = 0x17;

static constexpr uint8_t CLEAR_IRQ_STATUS_OP_CODE = 0x97;
static constexpr uint8_t GET_IRQ_STATUS_OP_CODE = 0x15;

static constexpr uint8_t SET_TX_OP_CODE = 0x83;
static constexpr uint8_t SET_TX_PARAMS_OP_CODE = 0x8E;
// Same [periodBase, count[15:8], count[7:0]] param shape as SET_TX_OP_CODE (Table 11-24)
static constexpr uint8_t SET_RX_OP_CODE = 0x82;

static constexpr uint8_t SET_DIO_IRQ_PARAMS_OP_CODE = 0x8D;

// Idle low-power periodic RX (Table 11-27)
static constexpr uint8_t SET_RX_DUTY_CYCLE_OP_CODE = 0x94;
// Table 11-30 / command summary table (p.68, p.83) both give 0x9B. Section 11.5.7's own prose says "opcode 0x98",
// but 0x98 is SetAutoTx's opcode (Table 11-34) -- that sentence is a datasheet typo, not a real ambiguity.
static constexpr uint8_t SET_LONG_PREAMBLE_OP_CODE = 0x9B;
// 0x00 = Slave, 0x01 = Master
static constexpr uint8_t SET_RANGING_ROLE_OP_CODE = 0xA3;

}  // namespace SX1280_OPERATIONS

namespace SX1280_VALUES {
static constexpr uint8_t STDBY_RC_STAND_BY = static_cast<uint8_t>(0x00);

// PacketType (Table 11-40)
static constexpr uint8_t PACKET_TYPE_LORA = 0x01;
static constexpr uint8_t PACKET_TYPE_RANGING = 0x02;

static constexpr double RF_FREQUENCY_HZ = 2450000000.0;  // 2.45 GHz
static constexpr uint32_t RF_FREQUENCY_REG = static_cast<uint32_t>((RF_FREQUENCY_HZ / (52000000.0 / 262144.0) + 0.5));

//frequency
static constexpr uint8_t FREQUENCY_MSB = static_cast<uint8_t>((RF_FREQUENCY_REG >> 16) & 0xFF);
static constexpr uint8_t FREQUENCY_MID = static_cast<uint8_t>((RF_FREQUENCY_REG >> 8) & 0xFF);
static constexpr uint8_t FREQUENCY_LSB = static_cast<uint8_t>(RF_FREQUENCY_REG & 0xFF);

// modulation -- Table 13-55: for Ranging any SF x any of the 3 ranging-legal BWs x any CR is valid, not fixed pairs
static constexpr uint8_t SPREADING_FACTOR_SF_7 = 0x70;  // Table 13-47
static constexpr uint8_t BANDWITH_BW_1600 = 0x0A;       // Table 13-48
static constexpr uint8_t CHIP_RATE_CR_4_5 = 0x01;       // Table 13-49

// If the Spreading Factor is SF7 or SF-8 then the command WriteRegister( 0x925, 0x37 ) must be used
static constexpr uint8_t SF_7_REGISTER_FIXUP = 0x37;

// SetPacketParams, LoRa/Ranging layout (Table 11-58): preamble, headerType, payloadLength, CRC, invertIQ, [unused x2]
static constexpr uint8_t LORA_PREAMBLE_12_SYMBOLS =
  0x23;                                           // Table 13-50: mantissa=3, exponent=2 -> 3*2^2 = 12 symbols (datasheet-recommended)
static constexpr uint8_t EXPLICIT_HEADER = 0x00;  // Table 13-51
static constexpr uint8_t IMPLICIT_HEADER = 0x80;
static constexpr uint8_t LORA_CRC_ENABLE = 0x20;  // Table 13-53
static constexpr uint8_t LORA_CRC_DISABLE = 0x00;
static constexpr uint8_t LORA_IQ_STD = 0x40;  // Table 13-54
static constexpr uint8_t LORA_IQ_INVERTED = 0x00;
// PayloadLength (packetParam3) isn't a shared constant -- each consumer fills it from its own payload struct size
// at compile time. Ranging still requires a value in that field even though the ranging engine itself ignores it.

// SetTxParams (Table 11-45/11-47): Pout[dBm] = -18 + power
static constexpr uint8_t TX_OUTPUT_POWER = 13;  // Pout = -5 dBm, modest/safe first-bring-up power
static constexpr uint8_t RADIO_RAMP_04_US = 0x20;

// Shared periodBase time-unit enum (Table 11-22), used by SetTx / SetRx / SetRxDutyCycle alike
static constexpr uint8_t PERIOD_BASE_15_625_US = 0x00;
static constexpr uint8_t PERIOD_BASE_62_5_US = 0x01;
static constexpr uint8_t PERIOD_BASE_1_MS = 0x02;
static constexpr uint8_t PERIOD_BASE_4_MS = 0x03;
// SetRxDutyCycle params (Table 11-27) are [periodBase, rxPeriodBaseCount[15:8], rxPeriodBaseCount[7:0],
// sleepPeriodBaseCount[15:8], sleepPeriodBaseCount[7:0]] -- ONE shared periodBase governs both the Rx and Sleep
// durations (Rx Duration = periodBase * rxPeriodBaseCount, Sleep Duration = periodBase * sleepPeriodBaseCount),
// not a separate rx/sleep periodBase pair. rxPeriodBaseCount = 0x0000 means "wait until a packet is found", not
// zero duration.

// Anchor idle-state duty cycle only (rover never calls SetRxDutyCycle) -- first-pass values, NOT validated on
// hardware yet: with PERIOD_BASE_1_MS this is 10ms RX / 490ms sleep (500ms cycle, ~2% RX duty cycle). Tune once
// wake latency / power draw can actually be measured -- see PLAN.md Open Items.
static constexpr uint16_t ANCHOR_IDLE_RX_PERIOD_BASE_COUNT = 10;
static constexpr uint16_t ANCHOR_IDLE_SLEEP_PERIOD_BASE_COUNT = 490;

// IRQ bit positions (Table 11-71/13-6x), combined into SetDioIrqParams'/GetIrqStatus'/ClearIrqStatus' 16-bit masks
static constexpr uint16_t IRQ_BIT_TX_DONE = static_cast<uint16_t>(1u << 0);
static constexpr uint16_t IRQ_BIT_RX_DONE = static_cast<uint16_t>(1u << 1);
static constexpr uint16_t IRQ_BIT_RANGING_SLAVE_RESPONSE_DONE = static_cast<uint16_t>(1u << 7);
static constexpr uint16_t IRQ_BIT_RANGING_SLAVE_REQUEST_DISCARD = static_cast<uint16_t>(1u << 8);
static constexpr uint16_t IRQ_BIT_RANGING_MASTER_RESULT_VALID = static_cast<uint16_t>(1u << 9);
static constexpr uint16_t IRQ_BIT_RANGING_MASTER_TIMEOUT = static_cast<uint16_t>(1u << 10);
static constexpr uint16_t IRQ_BIT_RANGING_MASTER_REQUEST_VALID = static_cast<uint16_t>(1u << 11);

// Ranging registers (Section 13.5)
static constexpr uint16_t REG_SF_MODULATION_FIXUP = 0x0925;         // see SF_7_REGISTER_FIXUP above
static constexpr uint16_t REG_RANGING_MASTER_TARGET_ADDR = 0x0912;  // 4 bytes MSB-first through 0x0915, Table 13-58 (rover/master only)
static constexpr uint16_t REG_RANGING_SLAVE_OWN_ADDR = 0x0916;      // 4 bytes MSB-first through 0x0919, Table 13-56 (anchor/slave only)
static constexpr uint16_t REG_RANGING_ADDR_CHECK_LEN = 0x0931;      // bits[7:6]: 0x0=8bit, 0x1=16bit, 0x2=24bit, 0x3=32bit (Table 13-57)
static constexpr uint16_t REG_RANGING_CALIBRATION = 0x092C;         // 2 bytes MSB-first through 0x092D, RxTx delay offset
static constexpr uint16_t REG_RANGING_RESULT_MUX = 0x0924;          // bits[5:4] select result type, see RANGING_RESULT_* below
static constexpr uint16_t REG_RANGING_RESULT_MSB = 0x0961;          // 3 bytes MSB-first through 0x0963
static constexpr uint16_t REG_LORA_MEM_CLOCK_ENABLE = 0x097F;       // read-modify-write bit 1, required before reading a ranging result

// RangingResMux (0x924 bits[5:4], Table 13-62) -- use DEBIASED or FILTERED, both non-negative; RAW/AVERAGE can be negative
static constexpr uint8_t RANGING_RESULT_RAW = 0x00;
static constexpr uint8_t RANGING_RESULT_AVERAGE = 0x01;
static constexpr uint8_t RANGING_RESULT_DEBIASED = 0x02;
static constexpr uint8_t RANGING_RESULT_FILTERED = 0x03;
// Distance[m] = RangingResult * 20 / 100 for Average/Debiased/Filtered (Table 13-62) -- exact in centimeters with
// no rounding loss: distance_cm = RangingResult * RANGING_RESULT_TO_CM_MULTIPLIER. Raw instead needs
// Distance[m] = RangingResult * 150 / (2^12 * BW_MHz); not covered here since Raw isn't used by this project.
static constexpr int32_t RANGING_RESULT_TO_CM_MULTIPLIER = 20;

}  // namespace SX1280_VALUES

// Application-level protocol constants -- not SX1280 hardware facts, kept separate from the datasheet-sourced
// namespaces above. Shared verbatim between anchor and rover (once rover exists).
namespace LORA_BEACON_PROTOCOL {
static constexpr uint8_t WAKE_WORD[2] = {0xBE, 0xAC};
}  // namespace LORA_BEACON_PROTOCOL
