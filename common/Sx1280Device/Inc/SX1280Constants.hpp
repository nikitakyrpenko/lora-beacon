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

static constexpr uint8_t CLEAR_IRQ_STATUS_OP_CODE = 0x97;
static constexpr uint8_t GET_IRQ_STATUS_OP_CODE = 0x15;

static constexpr uint8_t SET_TX_OP_CODE = 0x83;
static constexpr uint8_t SET_TX_PARAMS_OP_CODE = 0x8E;

static constexpr uint8_t SET_DIO_IRQ_PARAMS_OP_CODE = 0x8D;

}  // namespace SX1280_OPERATIONS

namespace SX1280_VALUES {
static constexpr uint8_t STDBY_RC_STAND_BY = static_cast<uint8_t>(0x00);
static constexpr uint8_t RANGING_PACKET_TYPE = 0x02;

static constexpr double RF_FREQUENCY_HZ = 2450000000.0;  // 2.45 GHz
static constexpr uint32_t RF_FREQUENCY_REG = static_cast<uint32_t>((RF_FREQUENCY_HZ / (52000000.0 / 262144.0) + 0.5));

//frequency
static constexpr uint8_t FREQUENCY_MSB = static_cast<uint8_t>((RF_FREQUENCY_REG >> 16) & 0xFF);
static constexpr uint8_t FREQUENCY_MID = static_cast<uint8_t>((RF_FREQUENCY_REG >> 8) & 0xFF);
static constexpr uint8_t FREQUENCY_LSB = static_cast<uint8_t>(RF_FREQUENCY_REG & 0xFF);

//modulation
static constexpr uint8_t SPREADING_FACTOR_SF_7 = 0x7A;
static constexpr uint8_t BANDWITH_BW_1600 = 0x0A;
static constexpr uint8_t CHIP_RATE_CR_4_5 = 0x01;

//Spreading Factor is SF7 or SF-8 then the command WriteRegister( 0x925, 0x37 ) must be used
static constexpr uint8_t SF_7_REGISTER_FIXUP = 0x37;
}  // namespace SX1280_VALUES