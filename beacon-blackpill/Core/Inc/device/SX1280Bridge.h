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

uint16_t SX1280_Beacon_Radio();
uint16_t SX1280_Send_Wake_Broadcast();
uint8_t SX1280_Was_Tx_Done();
uint16_t SX1280_Get_Irq_Status_Raw(uint8_t* hal_status_out, uint8_t* command_status_out);
void SX1280_Clear_Irq_Status_Raw();
void SX1280_Get_Status_Raw(uint8_t* circuit_mode_out, uint8_t* command_status_out);
uint16_t SX1280_Listen_For_Ack();
uint16_t SX1280_Check_Wake_Ack_Matches(uint32_t* anchor_address_out);
uint16_t SX1280_Ranging_Master_Mode(uint32_t target_anchor_address);
uint16_t SX1280_Send_Ranging_Request();
uint8_t SX1280_Read_Ranging_Result_Cm(int32_t* distance_cm_out);

#ifdef __cplusplus
}
#endif

#endif
