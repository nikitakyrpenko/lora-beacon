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
#include <cstdio>
#include "SX1280Bridge.h"
#include "cmsis_gcc.h"
#include "stm32h5xx_hal_gpio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

SPI_HandleTypeDef hspi3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

volatile uint8_t DIO1_Callback_detected = 0;

#define SX1280_BEACON_RADIO_STEP_COUNT 9U
#define SX1280_BEACON_RADIO_FULL_MASK ((uint16_t)((1u << SX1280_BEACON_RADIO_STEP_COUNT) - 1))

#define SX1280_RANGING_MASTER_STEP_COUNT 10U
#define SX1280_RANGING_MASTER_FULL_MASK ((uint16_t)((1u << SX1280_RANGING_MASTER_STEP_COUNT) - 1))

#define SX1280_WAKE_BROADCAST_STEP_COUNT 2U  // WriteBuffer, SetTx -- see SX1280_Send_Wake_Broadcast() in SX1280Bridge.cpp
#define SX1280_WAKE_BROADCAST_FULL_MASK ((uint16_t)((1u << SX1280_WAKE_BROADCAST_STEP_COUNT) - 1))

/* Mirrors SX1280_VALUES::IRQ_BIT_RANGING_MASTER_{RESULT_VALID,TIMEOUT} in SX1280Constants.hpp (Table 11-71) --
 * duplicated here as plain hex since this is a C file and that header is C++-namespaced. */
#define RANGING_MASTER_RESULT_VALID_BIT ((uint16_t)(1u << 9))
#define RANGING_MASTER_TIMEOUT_BIT ((uint16_t)(1u << 10))
/* Mirrors SX1280_VALUES::LORA_BEACON_PROTOCOL::COLLECT_PHASE_CEILING_MS/EXPECTED_ANCHOR_COUNT, same reason as
 * above. During BEACON_COLLECTING, SX1280_Listen_For_Ack()'s irq mask only routes RX_DONE, so a DIO1 fire there
 * can only mean a packet arrived -- no separate bit check needed. */
#define COLLECT_PHASE_CEILING_MS 150U
#define EXPECTED_ANCHOR_COUNT 3U

typedef enum {
  BEACON_IDLE,
  BEACON_COLLECTING,
  BEACON_RANGING,
} BeaconState;

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

  SX1280_Create(&hspi3, BUSY_GPIO_Port, NSS_GPIO_Port, NRESET_GPIO_Port, TCXOEN_GPIO_Port, BUSY_Pin, NSS_Pin, NRESET_Pin, TCXOEN_Pin);
  uint16_t radio_result = SX1280_Beacon_Radio();

  printf("[%lu] SX1280_Beacon_Radio mask=0x%03X (full=0x%03X)%s\r\n",
         (unsigned long)HAL_GetTick(),
         radio_result,
         SX1280_BEACON_RADIO_FULL_MASK,
         (radio_result == SX1280_BEACON_RADIO_FULL_MASK) ? " OK" : " INCOMPLETE");

  if (radio_result == SX1280_BEACON_RADIO_FULL_MASK) {
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
  }
  // Initialized 5000ms in the past (not HAL_GetTick()) so the loop's own cadence check fires the first wake
  // broadcast immediately on entry, instead of special-casing an extra send here outside the loop.
  uint32_t wake_broadcast_last_tick = HAL_GetTick() - 5000;
  BeaconState beacon_state = BEACON_IDLE;
  uint32_t collected_addresses[EXPECTED_ANCHOR_COUNT];
  uint8_t collected_count = 0;
  uint32_t collect_phase_deadline_tick = 0;
  uint8_t ranging_index = 0;
  uint32_t ranging_request_sent_tick = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (beacon_state == BEACON_IDLE && (HAL_GetTick() - wake_broadcast_last_tick >= 5000)) {
      wake_broadcast_last_tick = HAL_GetTick();
      uint16_t wake_broadcast_mask = SX1280_Send_Wake_Broadcast();
      printf("[%lu] sending wake broadcast mask=0x%X (full=0x%X)%s\r\n",
             (unsigned long)wake_broadcast_last_tick,
             wake_broadcast_mask,
             SX1280_WAKE_BROADCAST_FULL_MASK,
             (wake_broadcast_mask == SX1280_WAKE_BROADCAST_FULL_MASK) ? " OK" : " INCOMPLETE");

      DIO1_Callback_detected = 0;
      SX1280_Clear_Irq_Status_Raw();

      SX1280_Listen_For_Ack();
      printf("[%lu] collecting acks\r\n", (unsigned long)wake_broadcast_last_tick);

      collected_count = 0;
      collect_phase_deadline_tick = HAL_GetTick() + COLLECT_PHASE_CEILING_MS;
      beacon_state = BEACON_COLLECTING;
    }

    if (beacon_state == BEACON_COLLECTING) {
      if (DIO1_Callback_detected) {
        DIO1_Callback_detected = 0;
        SX1280_Clear_Irq_Status_Raw();  // every catch, not just once -- continuous RX keeps listening on its own

        uint32_t learned_anchor_address = 0;
        if (SX1280_Check_Wake_Ack_Matches(&learned_anchor_address) && collected_count < EXPECTED_ANCHOR_COUNT) {
          uint8_t already_have = 0;
          for (uint8_t i = 0; i < collected_count; i++) {
            if (collected_addresses[i] == learned_anchor_address) {
              already_have = 1;
              break;
            }
          }
          if (!already_have) {
            collected_addresses[collected_count] = learned_anchor_address;
            collected_count++;
            printf("[%lu] collected ack from anchor 0x%08lX (%u/%u)\r\n",
                   (unsigned long)HAL_GetTick(),
                   (unsigned long)learned_anchor_address,
                   collected_count,
                   EXPECTED_ANCHOR_COUNT);
          }
        }
      }

      if (collected_count >= EXPECTED_ANCHOR_COUNT || HAL_GetTick() >= collect_phase_deadline_tick) {
        SX1280_Stop_Ack_Listen();
        printf("[%lu] collect phase done, %u anchor(s) collected\r\n", (unsigned long)HAL_GetTick(), collected_count);

        if (collected_count == 0) {
          SX1280_Beacon_Radio();
          beacon_state = BEACON_IDLE;
        }
        else {
          ranging_index = 0;
          uint16_t ranging_master_mask = SX1280_Ranging_Master_Mode(collected_addresses[ranging_index]);
          if (ranging_master_mask == SX1280_RANGING_MASTER_FULL_MASK) {
            DIO1_Callback_detected = 0;
            SX1280_Clear_Irq_Status_Raw();
            SX1280_Send_Ranging_Request();
            ranging_request_sent_tick = HAL_GetTick();
            beacon_state = BEACON_RANGING;
          }
          else {
            printf("[%lu] SX1280_Ranging_Master_Mode INCOMPLETE (mask=0x%03X full=0x%03X)\r\n",
                   (unsigned long)HAL_GetTick(),
                   ranging_master_mask,
                   SX1280_RANGING_MASTER_FULL_MASK);
            SX1280_Beacon_Radio();
            beacon_state = BEACON_IDLE;
          }
        }
      }
    }

    if (beacon_state == BEACON_RANGING) {
      // 1100ms is a fallback ceiling only (a bit more than the 1000ms SetTx timeout) in case DIO1 is somehow
      // missed -- a normal exchange resolves via the interrupt within low single-digit ms of the chip deciding
      // RangingMasterResultValid or RangingMasterTimeout, not after this full window.
      uint8_t ceiling_elapsed = (HAL_GetTick() - ranging_request_sent_tick) >= 1100;
      if (DIO1_Callback_detected || ceiling_elapsed) {
        DIO1_Callback_detected = 0;

        uint16_t ranging_irq = 0;
        SX1280_Get_Irq_Mask(&ranging_irq);
        SX1280_Clear_Irq_Status_Raw();

        uint32_t ranged_address = collected_addresses[ranging_index];
        if (ranging_irq & RANGING_MASTER_RESULT_VALID_BIT) {
          int32_t distance_cm = 0;
          if (SX1280_Read_Ranging_Result_Cm(&distance_cm)) {
            printf("[%lu] anchor 0x%08lX ranging result: %ld cm\r\n",
                   (unsigned long)HAL_GetTick(),
                   (unsigned long)ranged_address,
                   (long)distance_cm);
          }
          else {
            printf("[%lu] anchor 0x%08lX ranging result: readback failed\r\n", (unsigned long)HAL_GetTick(), (unsigned long)ranged_address);
          }
        }
        else if (ranging_irq & RANGING_MASTER_TIMEOUT_BIT) {
          printf("[%lu] anchor 0x%08lX ranging request timed out\r\n", (unsigned long)HAL_GetTick(), (unsigned long)ranged_address);
        }
        else {
          printf("[%lu] anchor 0x%08lX ranging request: no result (irq=0x%04X)\r\n",
                 (unsigned long)HAL_GetTick(),
                 (unsigned long)ranged_address,
                 ranging_irq);
        }

        ranging_index++;
        if (ranging_index < collected_count) {
          uint16_t ranging_master_mask = SX1280_Ranging_Master_Mode(collected_addresses[ranging_index]);
          if (ranging_master_mask == SX1280_RANGING_MASTER_FULL_MASK) {
            DIO1_Callback_detected = 0;
            SX1280_Clear_Irq_Status_Raw();
            SX1280_Send_Ranging_Request();
            ranging_request_sent_tick = HAL_GetTick();
            // stay in BEACON_RANGING for the next collected address
          }
          else {
            printf("[%lu] SX1280_Ranging_Master_Mode INCOMPLETE (mask=0x%03X full=0x%03X)\r\n",
                   (unsigned long)HAL_GetTick(),
                   ranging_master_mask,
                   SX1280_RANGING_MASTER_FULL_MASK);
            SX1280_Beacon_Radio();
            beacon_state = BEACON_IDLE;
          }
        }
        else {
          SX1280_Beacon_Radio();
          beacon_state = BEACON_IDLE;
        }
      }
    }

    {
      static uint32_t busy_poll_last_tick = 0;
      if (HAL_GetTick() - busy_poll_last_tick >= 1000) {
        busy_poll_last_tick = HAL_GetTick();
        GPIO_PinState busy_raw_level = HAL_GPIO_ReadPin(BUSY_GPIO_Port, BUSY_Pin);
        printf("[%lu] raw BUSY pin level (GPIO read only, no SPI) = %s\r\n",
               (unsigned long)HAL_GetTick(),
               busy_raw_level == GPIO_PIN_SET ? "HIGH" : "LOW");
      }
    }

    // Sleep until the next interrupt (SysTick @1ms, or DIO1's EXTI) instead of busy-spinning -- safe against the
    // check-then-sleep race because NVIC latches a pending interrupt regardless of core sleep state, so WFI
    // returns immediately if one arrived since the checks above.
    __WFI();
  }
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
