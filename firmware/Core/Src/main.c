/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "can_definitions.h"
#include "crc.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define ENABLE_CAN_TRANSCEIVER() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET)
#define LED_OFF() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET)
#define LED_ON() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET)
#define MAX(x, y) ((x > y) ? (x) : (y))
#define ABS(x) ((x) > 0 ? (x) : -(x))
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

CAN_HandleTypeDef hcan;

DAC_HandleTypeDef hdac;

/* USER CODE BEGIN PV */
CAN_TxHeaderTypeDef txHeader;
uint32_t  txMailbox;
CAN_FilterTypeDef canfil;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_CAN_Init(void);
static void MX_DAC_Init(void);
/* USER CODE BEGIN PFP */
//void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
//void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan);
void readCAN();
void CAN_RX_Interrupt(CAN_HandleTypeDef *hcan, int mailbox);
uint32_t sendCAN(uint16_t id, uint8_t buffer[], uint8_t len);
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define CAN_GAS_INPUT  0x200
#define CAN_GAS_OUTPUT 0x201U
#define CAN_GAS_SIZE 6
#define COUNTER_CYCLE 0xFU

// two independent values
uint16_t gas_set_0 = 0;
uint16_t gas_set_1 = 0;

#define MAX_TIMEOUT 1000U
uint32_t timeout = 0;
uint32_t current_index = 0;

#define NO_FAULT 0U
#define FAULT_BAD_CHECKSUM 1U
#define FAULT_SEND 2U
#define FAULT_SCE 3U
#define FAULT_STARTUP 4U
#define FAULT_TIMEOUT 5U
#define FAULT_INVALID 6U
uint8_t state = FAULT_STARTUP;
const uint8_t crc_poly = 0xD5U;  // standard crc8

uint32_t pdl0 = 0;
uint32_t pdl1 = 0;
unsigned int pkt_idx = 0;

int last_tick = 0;


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
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_CAN_Init();
  MX_DAC_Init();
  /* USER CODE BEGIN 2 */
  ENABLE_CAN_TRANSCEIVER();
  LED_OFF();
  if (HAL_CAN_Start(&hcan) != HAL_OK) {
	  LED_ON();
  }
  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO1_MSG_PENDING);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
	  readCAN();
	  int tick = HAL_GetTick();
	  if (timeout == MAX_TIMEOUT) {
	    state = FAULT_TIMEOUT;
	  } else {
	    timeout += 1U;
	  }

	  if (tick - last_tick >= 20) {
		  uint8_t dat[6];
		  dat[0] = (pdl0 >> 8) & 0xFFU;
		  dat[1] = (pdl0 >> 0) & 0xFFU;
		  dat[2] = (pdl1 >> 8) & 0xFFU;
		  dat[3] = (pdl1 >> 0) & 0xFFU;
		  dat[4] = ((state & 0xFU) << 4) | pkt_idx;
		  dat[5] = crc_checksum(dat, CAN_GAS_SIZE - 1, crc_poly);
		  sendCAN(CAN_GAS_OUTPUT, dat, 6);
		  ++pkt_idx;
		  pkt_idx &= COUNTER_CYCLE;
		  last_tick = tick;
	  }

	  HAL_ADC_Start(&hadc1);
	  HAL_ADC_PollForConversion(&hadc1, 1);
	  pdl0 = HAL_ADC_GetValue(&hadc1);

	  HAL_ADC_Start(&hadc2);
	  HAL_ADC_PollForConversion(&hadc2, 1);
	  pdl1 = HAL_ADC_GetValue(&hadc2);

	  // write the pedal to the DAC
	  if (state == NO_FAULT) {
		HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, MAX(gas_set_0, pdl0));
		HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
		HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, MAX(gas_set_1, pdl1));
		HAL_DAC_Start(&hdac, DAC_CHANNEL_2);
	  } else {
		HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, pdl0);
		HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
		HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, pdl1);
		HAL_DAC_Start(&hdac, DAC_CHANNEL_2);
	  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV1;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN;
  hcan.Init.Prescaler = 4;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_15TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */
  canfil.FilterMode = CAN_FILTERMODE_IDMASK;
  canfil.FilterScale = CAN_FILTERSCALE_32BIT;
  canfil.FilterIdHigh = 0x0000;
  canfil.FilterIdLow = 0x0000;
  canfil.FilterMaskIdHigh = 0x0000;
  canfil.FilterMaskIdLow = 0x0000;
  canfil.FilterFIFOAssignment = 0;
  canfil.FilterActivation = ENABLE;
  HAL_CAN_ConfigFilter(&hcan, &canfil);

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */

  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT2 config
  */
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB11 PB3 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */


uint32_t sendCAN(uint16_t id, uint8_t buffer[], uint8_t len) {
	CAN_TxHeaderTypeDef header;

	header.IDE = CAN_ID_STD;
	header.RTR = CAN_RTR_DATA;
	header.TransmitGlobalTime = DISABLE;
	header.DLC = len;
	header.StdId = id;

	uint32_t mailbox;
	if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) > 0) {
		uint8_t res = HAL_CAN_AddTxMessage(&hcan, &header, buffer, &mailbox);
		if (res != HAL_OK) {
			state = FAULT_SEND;
		} else {
			state = NO_FAULT;
		}
	} else {
		if (HAL_CAN_IsTxMessagePending(&hcan, txMailbox)) {
			state = FAULT_SEND;
		}
	}

	return mailbox;
}

void CAN_RX_Interrupt(CAN_HandleTypeDef *hcan, int mailbox) {
	CAN_RxHeaderTypeDef tmp;
	uint8_t dat[8];
	HAL_CAN_GetRxMessage(hcan, mailbox, &tmp, dat);
	int address = tmp.StdId;
	if (address == CAN_GAS_INPUT) {
		uint16_t value_0 = (dat[0] << 8) | dat[1];
		uint16_t value_1 = (dat[2] << 8) | dat[3];
		uint8_t enable = ((dat[4] >> 7) & 1U) != 0U;
		uint8_t index = dat[4] & COUNTER_CYCLE;
		if (crc_checksum(dat, CAN_GAS_SIZE - 1, crc_poly) == dat[5]) {
			if (((current_index + 1U) & COUNTER_CYCLE) == index) {
				LED_ON();
				if (enable) {
					gas_set_0 = value_0;
					gas_set_1 = value_1;
				} else {
					// clear the fault state if values are 0
					if ((value_0 == 0U) && (value_1 == 0U)) {
						state = NO_FAULT;
					} else {
						state = FAULT_INVALID;
					}
					gas_set_0 = 0;
					gas_set_1 = 0;
				}
				timeout = 0;
			} else {
				LED_OFF();
			}
			current_index = index;
		} else {
			state = FAULT_BAD_CHECKSUM;
		}
	}
}

void readCAN() {
	if (HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0) > 0) {
		CAN_RX_Interrupt(&hcan, CAN_RX_FIFO0);
	}
	if (HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO1) > 0) {
		CAN_RX_Interrupt(&hcan, CAN_RX_FIFO0);
	}
}

//void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
//	CAN_RX_Interrupt(hcan, CAN_RX_FIFO0);
//}
//
//void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) {
//	CAN_RX_Interrupt(hcan, CAN_RX_FIFO1);
//}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
