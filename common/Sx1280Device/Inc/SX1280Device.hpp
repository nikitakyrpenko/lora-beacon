
#include "stm32l4xx_hal.h"

static constexpr uint8_t BUSY_TIMEOUT = 10;  // 10 ms BUSY timeout
static constexpr uint8_t SPI_TIMEOUT = 10;   // 10 ms SPI timeout
static constexpr uint8_t NRESET_HOLD = 2;    // 2 ms NRESET hold
static constexpr uint8_t TCXOEN_HOLD = 3;    // 3 ms TCXOEN hold

static constexpr const char* BUSY_PIN_NAME = "BUSY";
static constexpr const char* NSS_PIN_NAME = "NSS";
static constexpr const char* NRESET_PIN_NAME = "NRESET";
static constexpr const char* TCXOEN_PIN_NAME = "TCXOE1N";

static constexpr uint8_t SPI_PACKET_SIZE = 64;

class SX1280Device {
public:
  struct GPIO_Pin {
    GPIO_TypeDef* port;
    uint16_t pin;
    const char* name;
    GPIO_PinState current;
  };
  enum class CircuitMode : uint8_t {
    RESERVED_0 = 0x0,
    RESERVED_1 = 0x1,
    STDBY_RC = 0x2,
    STDBY_XOSC = 0x3,
    FS = 0x4,
    RX = 0x5,
    TX = 0x6,
  };

  enum class CommandStatus : uint8_t {
    RESERVED = 0x0,
    COMMAND_SUCCESS = 0x1,
    DATA_AVAILABLE = 0x2,
    COMMAND_TIMEOUT = 0x3,
    COMMAND_PROCESS_ERROR = 0x4,
    COMMAND_EXECUTION_FAILURE = 0x5,
    COMMAND_TX_DONE = 0x6,
  };

  struct SX1280_Status {
    CircuitMode circuit_mode = SX1280Device::CircuitMode::RESERVED_0;
    CommandStatus command_status = SX1280Device::CommandStatus::RESERVED;
    bool busy = 1;
  };

  SX1280Device(SPI_HandleTypeDef* SPI_port,
               GPIO_TypeDef* BUSY_GPIO_port,
               GPIO_TypeDef* NSS_GPIO_port,
               GPIO_TypeDef* NRESET_GPIO_port,
               GPIO_TypeDef* TCXOEN_GPIO_port,
               uint16_t BUSY_pin,
               uint16_t NSS_pin,
               uint16_t NRESET_pin,
               uint16_t TCXOEN_pin);

  bool NRESET_reset();
  HAL_StatusTypeDef SPI_write(const uint8_t* reg, const uint8_t* buf, const uint16_t len, SX1280_Status* out);
  HAL_StatusTypeDef SPI_read(const uint8_t* reg, uint8_t* buf, uint16_t len, SX1280_Status* out);

private:
  SPI_HandleTypeDef* SPI_port;
  GPIO_Pin BUSY;
  GPIO_Pin NSS;
  GPIO_Pin NRESET;
  GPIO_Pin TCXOEN;

  bool BUSY_wait(uint8_t timeout);
  bool NSS_begin();
  bool NSS_end();
  bool TCXOEN_warmup();

  SX1280_Status to_status(uint8_t r);
};