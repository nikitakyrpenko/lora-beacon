#ifndef SX1280_BRIDGE_H
#define SX1280_BRIDGE_H

#include <stdint.h>
#include "stm32h5xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

void SX1280_Create(SPI_HandleTypeDef* SPI_port,
                   GPIO_TypeDef* BUSY_GPIO_port,
                   GPIO_TypeDef* NSS_GPIO_port,
                   GPIO_TypeDef* NRESET_GPIO_port,
                   GPIO_TypeDef* TCXOEN_GPIO_port,
                   uint16_t BUSY_pin,
                   uint16_t NSS_pin,
                   uint16_t NRESET_pin,
                   uint16_t TCXOEN_pin);

uint16_t SX1280_Radio_mode();
uint16_t SX1280_Check_Wake_Word_Matches(uint32_t* ranging_window_ms_out);
uint16_t SX1280_Send_Wake_Ack();
uint32_t SX1280_Ack_Slot_Delay_Ms();
uint16_t SX1280_Ranging_Slave_Mode();
uint32_t SX1280_Ranging_Window_Ms();
uint16_t SX1280_Get_Irq_Status_Raw(uint8_t* hal_status_out, uint8_t* command_status_out);
void SX1280_Clear_Irq_Status_Raw();

#ifdef __cplusplus
}
#endif

#endif
