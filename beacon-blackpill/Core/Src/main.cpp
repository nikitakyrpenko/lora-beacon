/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <cstdint>
#include "BeaconBridge.hpp"
#include "SX1280BringUp.hpp"
#include "cmsis_gcc.h"
#include "stm32h5xx_hal_gpio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// Permanent hardware bring-up test (see test.md). Reflash with the desired mode, run, read UART @ 115200 8N1, reflash
// back to 0 for deployment. Set via CMake: -DBRINGUP_MODE=n.
// 0 = normal production beacon logic
// 1 = board-level bring-up: GPIO/UART/SPI3 checks only, the SX1280 module need not be attached
// 2 = module-level bring-up: SX1280 chip checks driven through BeaconBridge only; the module must be attached
#ifndef BRINGUP_MODE
#define BRINGUP_MODE 0
#endif

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

SPI_HandleTypeDef hspi3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

volatile uint8_t DIO1_Callback_detected = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ICACHE_Init(void);
static void MX_SPI3_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#if BRINGUP_MODE == 2
// Beacon-specific module-level checks (4/5 and 5/5). Checks 1-3, the board-level test and the final summary are shared with
// the anchor: common/Sx1280Device/Inc/SX1280BringUp.hpp. Everything here goes through BeaconBridge's public API, so every
// SPI transfer takes the real BUSY-gated path.

// 4/5: independent corroboration through different opcodes than check 3. listen_for_ack() sends SetPacketParams,
// SetDioIrqParams and a 100 ms SetRx; right after it the chip must report circuit_mode RX (5) via GetStatus, which needs both
// directions of the SPI link to work. Don't trust listen_for_ack()'s own mask alone: it accepts a status of 0, so an
// all-zero MISO can look like success there.
static bool bringup_check_command_roundtrip(BeaconBridge& bridge)
{
  DIO1_Callback_detected = 0;  // check 5 waits for the edge of the RX timeout that starts here
  const uint16_t mask = bridge.listen_for_ack();
  printf("[%lu]   listen_for_ack mask=0x%X (full=0x7) get_mode()=%s\r\n",
         (unsigned long)HAL_GetTick(),
         mask,
         bridge.get_mode() == MODE::ACK_LISTEN ? "ACK_LISTEN" : "NONE");

  SX1280Device::SX1280_Status sta{};
  const HAL_StatusTypeDef hal = bridge.get_status(&sta);
  const bool ok = (hal == HAL_OK) && (sta.circuit_mode == SX1280Device::CircuitMode::RX);
  if (hal != HAL_OK) {
    printf("[%lu]   GetStatus after listen_for_ack failed: hal=%d\r\n", (unsigned long)HAL_GetTick(), static_cast<int>(hal));
  }
  else {
    printf(
      "[%lu]   after SetRx: circuit_mode=%u (expect 5 = RX)\r\n", (unsigned long)HAL_GetTick(), static_cast<unsigned>(sta.circuit_mode));
  }
  if (!ok) {
    printf("[%lu]   the chip did not take the configuration (MOSI/SCK/NSS path) or its status is unreadable (MISO)\r\n",
           (unsigned long)HAL_GetTick());
  }
  SX1280BringUp::Report(4, "command round-trip (SetRx -> RX)", ok);
  return ok;
}

// 5/5: DIO1 / EXTI, self-contained and silent on air: the ack-listen RX started in check 4 has a 100 ms timeout and routes
// RX_TX_TIMEOUT to DIO1, so with nothing transmitting the chip raises a real DIO1 rising edge that must reach
// DIO1_Callback_detected through the actual EXTI callback / NVIC path. (Nothing is transmitted -- the beacon's wake
// broadcast would wake real anchors.) It can't detect a DIO1 that is already stuck HIGH (no edge). Leaves the chip in STDBY_RC.
static bool bringup_check_dio1(BeaconBridge& bridge)
{
  const bool ok = SX1280BringUp::WaitForFlag(&DIO1_Callback_detected, 3 * LORA_BEACON_PROTOCOL::ACK_LISTEN_TIMEOUT_MS);
  if (!ok) {
    printf("[%lu]   no DIO1 edge within %u ms of the RX timeout: chip not raising DIO1, the DIO1 wire, or the EXTI/NVIC path\r\n",
           (unsigned long)HAL_GetTick(),
           static_cast<unsigned>(3 * LORA_BEACON_PROTOCOL::ACK_LISTEN_TIMEOUT_MS));
  }
  bridge.stop_ack_listen();
  SX1280BringUp::Report(5, "DIO1 / EXTI (RX timeout)", ok);
  return ok;
}
#endif  // BRINGUP_MODE == 2

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ICACHE_Init();
  MX_SPI3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

#if BRINGUP_MODE != 0
  // pinout shared by the anchor and the beacon Blackpill boards
  static const SX1280BringUp::BoardPins bringup_pins{NSS_GPIO_Port,
                                                     NSS_Pin,
                                                     NRESET_GPIO_Port,
                                                     NRESET_Pin,
                                                     TCXOEN_GPIO_Port,
                                                     TCXOEN_Pin,
                                                     LED_GPIO_Port,
                                                     LED_Pin,
                                                     BUSY_GPIO_Port,
                                                     BUSY_Pin,
                                                     DIO1_GPIO_Port,
                                                     DIO1_Pin};
#endif

#if BRINGUP_MODE == 1
  // Board-level bring-up (test.md layer 1): shared with the anchor, never returns. The SX1280 module need not be attached.
  SX1280BringUp::RunBoardLevel(bringup_pins, &hspi3, &DIO1_Callback_detected);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */
  }
#elif BRINGUP_MODE == 2
  // Module-level bring-up (test.md layer 2): SX1280 chip checks through BeaconBridge, the module must be attached.
  printf("[%lu] BRING-UP: beacon module-level self-test\r\n", (unsigned long)HAL_GetTick());

  uint16_t bringup_mask = 0;
  if (SX1280BringUp::CheckIdleGpioState(bringup_pins)) {  // before the bridge exists: its constructor resets the chip
    bringup_mask |= SX1280BringUp::BIT_GPIO_IDLE;
  }

  static BeaconBridge beacon_bridge(
    &hspi3, BUSY_GPIO_Port, NSS_GPIO_Port, NRESET_GPIO_Port, TCXOEN_GPIO_Port, BUSY_Pin, NSS_Pin, NRESET_Pin, TCXOEN_Pin);

  if (SX1280BringUp::CheckResetBusyLow(bringup_pins)) {
    bringup_mask |= SX1280BringUp::BIT_RESET;
  }
  if (SX1280BringUp::CheckChipAlive(beacon_bridge)) {
    bringup_mask |= SX1280BringUp::BIT_CHIP_ALIVE;
  }
  if (bringup_check_command_roundtrip(beacon_bridge)) {
    bringup_mask |= SX1280BringUp::BIT_COMMAND_ROUNDTRIP;
  }
  if (bringup_check_dio1(beacon_bridge)) {
    bringup_mask |= SX1280BringUp::BIT_DIO1;
  }

  SX1280BringUp::Finish(bringup_pins, bringup_mask);  // prints the summary, LED pass/fail, never returns
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */
  }
#else
  static BeaconBridge beacon_bridge(
    &hspi3, BUSY_GPIO_Port, NSS_GPIO_Port, NRESET_GPIO_Port, TCXOEN_GPIO_Port, BUSY_Pin, NSS_Pin, NRESET_Pin, TCXOEN_Pin);

  beacon_bridge.to_radio();

  //debug led for traking mode
  if (beacon_bridge.get_mode() == MODE::RADIO) {
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    beacon_bridge.step(DIO1_Callback_detected);
    __WFI();
  }
#endif  // BRINGUP_MODE
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_CSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV2;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.CSIState = RCC_CSI_ON;
  RCC_OscInitStruct.CSICalibrationValue = RCC_CSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLL1_SOURCE_CSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 32;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1_VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1_VCORANGE_WIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
    Error_Handler();
  }

  /** Configure the programming delay
  */
  __HAL_FLASH_SET_PROGRAM_DELAY(FLASH_PROGRAMMING_DELAY_0);
}

/**
  * @brief ICACHE Initialization Function
  * @param None
  * @retval None
  */
static void MX_ICACHE_Init(void)
{
  /* USER CODE BEGIN ICACHE_Init 0 */

  /* USER CODE END ICACHE_Init 0 */

  /* USER CODE BEGIN ICACHE_Init 1 */

  /* USER CODE END ICACHE_Init 1 */

  /** Enable instruction cache in 1-way (direct mapped cache)
  */
  if (HAL_ICACHE_ConfigAssociativityMode(ICACHE_1WAY) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_ICACHE_Enable() != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN ICACHE_Init 2 */

  /* USER CODE END ICACHE_Init 2 */
}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{
  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 0x7;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi3.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi3.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi3.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi3.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi3.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi3.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi3.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  hspi3.Init.ReadyMasterManagement = SPI_RDY_MASTER_MANAGEMENT_INTERNALLY;
  hspi3.Init.ReadyPolarity = SPI_RDY_POLARITY_HIGH;
  if (HAL_SPI_Init(&hspi3) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{
  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(NRESET_GPIO_Port, NRESET_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TCXOEN_GPIO_Port, TCXOEN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : NSS_Pin */
  GPIO_InitStruct.Pin = NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(NSS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : NRESET_Pin TCXOEN_Pin */
  GPIO_InitStruct.Pin = NRESET_Pin | TCXOEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : BUSY_Pin */
  GPIO_InitStruct.Pin = BUSY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUSY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DIO1_Pin */
  GPIO_InitStruct.Pin = DIO1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DIO1_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI8_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI8_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

extern "C" int __io_putchar(int ch)
{
  uint8_t c = (uint8_t)ch;
  HAL_UART_Transmit(&huart1, &c, 1, HAL_MAX_DELAY);
  return ch;
}

void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == DIO1_Pin) {
    DIO1_Callback_detected = 1;
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
