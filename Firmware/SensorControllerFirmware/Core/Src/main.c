/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "ADS1118.h"
#include <memory.h>
#include "config/config.h"
#include "config/pt_config.h"
#include "utils/board_utils.h"
#include "utils/can_utils.h"
#include "stdbool.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TX_ID 0x110

#define ENDPOINT_TEMPERATURE 15
#define ENDPOINT_LED 23
#define ENDPOINT_CALIBRATION 97
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim14;

/* USER CODE BEGIN PV */
PtConfig* PT_A = NULL;
PtConfig* PT_B = NULL;
Ads1118TypeDef adc;
volatile uint8_t start_read_adc = 0;
volatile uint8_t adc_read_cplt = 0;
uint32_t board_id[3];
static uint8_t short_board_id;
bool flash_signal_cmd = 0;
bool adc_reading = false;
bool port_a_read_cplt = false;
bool port_b_read_cplt = false;
bool send_port_a = false;
bool send_port_b = false;
float port_a_val = 0.0f;
float port_b_val = 0.0f;
uint16_t port_a_config = (ADS1118_CONFIG_DEFAULT | (0b111 << ADS1118_CONFIG_BIT_MUX) | (1 << ADS1118_CONFIG_BIT_SS) | (0b000 << 9)) & 0xFBFF;
uint16_t port_b_config = (ADS1118_CONFIG_DEFAULT | (0b101 << ADS1118_CONFIG_BIT_MUX) | (1 << ADS1118_CONFIG_BIT_SS) | (0b000 << 9)) & 0xFBFF;
uint16_t port_a_rx_buf[] = {0, 0};
uint16_t port_b_rx_buf[] = {0, 0};
float v_fs = 6.144f;
uint8_t tx_data[8];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM14_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);

static void GET_PORT_A_READING(uint16_t config);
static void GET_PORT_B_READING(uint16_t config);
static float CONVERT_ADC_READING(PtConfig *pt, int16_t raw_val);
static void INSERT_FLOAT_TO_TX_DATA(float val);
/* USER CODE BEGIN PFP */
uint32_t frequency_to_period_ms(uint32_t frequency);
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
  MX_DMA_Init();
  MX_CAN_Init();
  MX_SPI1_Init();
  MX_TIM14_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  GET_BOARD_UID(board_id);
  short_board_id = GET_SHORT_BOARD_ID(board_id);
  // Configure filter for extended ID
    CAN_FilterTypeDef filter;
    filter.FilterActivation = CAN_FILTER_ENABLE;
    filter.FilterBank = 0;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;

    // Page 1091 in STM32F405 Reference Manual
    // first 29 are the identifier, then IDE, then RTR, then 0

    // we only care about the 8 bits to make sure that its talking to the right board, so we use that
    uint32_t canIdFilter = (short_board_id << 16);
    uint32_t canIdMask = 0x00FF0000;

    // Set IDE bit in both filter and mask
    canIdFilter |= CAN_ID_EXT;
    canIdFilter |= CAN_RTR_DATA;
    canIdMask |= CAN_ID_EXT;
    canIdMask |= CAN_RTR_DATA;

    filter.FilterIdHigh = (canIdFilter >> 16) & 0xFFFF;
    filter.FilterIdLow = canIdFilter & 0xFFFF;
    filter.FilterMaskIdHigh = (canIdMask >> 16) & 0xFFFF;
    filter.FilterMaskIdLow = canIdMask & 0xFFFF;

    if (HAL_CAN_ConfigFilter(&hcan, &filter) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING)) {
      Error_Handler();
    };

    if (HAL_CAN_Start(&hcan) != HAL_OK) {
        Error_Handler();
    }

    STATUS_IND_Toggle();
    HAL_Delay(500);
    STATUS_IND_Toggle();

  // Configure ADC
  adc.hspi = &hspi1;
  adc.cs_gpio_port = ADC_CS_GPIO_Port;
  adc.cs_pin = ADC_CS_Pin;
  adc.config = port_a_config;
  Ads1118_Configure(&adc);

//  get both pt can ids
  uint32_t pt_1_can_id = GET_PT_CAN_ID(board_id, 1);
  uint32_t pt_2_can_id = GET_PT_CAN_ID(board_id, 2);

  htim2.Init.Period = 0;
  htim3.Init.Period = 0;

  if (pt_1_can_id != -1) {
//	  set pt_1 to either A or B based on its config
	  PtConfig* tempPtConfig = GET_PT_CONFIG(pt_1_can_id);
	  if (tempPtConfig->port == 'A') {
		  PT_A = tempPtConfig;
		  htim2.Init.Period = frequency_to_period_ms(PT_A->frequency);
	  }
	  else if (tempPtConfig->port == 'B') {
		  PT_B = tempPtConfig;
		  htim3.Init.Period = frequency_to_period_ms(PT_B->frequency);
	  }
  }

  if (pt_2_can_id != -1) {
//	  set pt_2 to either A or B based on its config
	  PtConfig* tempPtConfig = GET_PT_CONFIG(pt_2_can_id);
	  if (tempPtConfig->port == 'A') {
		  PT_A = tempPtConfig;
		  htim2.Init.Period = frequency_to_period_ms(PT_A->frequency);
	  }
	  else if (tempPtConfig->port == 'B') {
		  PT_B = tempPtConfig;
		  htim3.Init.Period = frequency_to_period_ms(PT_B->frequency);
	  }
  }
  // Start peripherals
//  HAL_ADC_Start(&hadc);
  HAL_TIM_Base_Start_IT(&htim14);

//  set timer 2 to interrupt at PT_A frequency
  if (htim2.Init.Period != 0) {
	  HAL_TIM_Base_Init(&htim2);
	  HAL_TIM_Base_Start_IT(&htim2);
  }
//  set timer 3 to interrupt at PT_B frequency
  if (htim3.Init.Period != 0) {
  	  HAL_TIM_Base_Init(&htim3);
  	  HAL_TIM_Base_Start_IT(&htim3);
    }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
//  int16_t buf[] = { 0, 0 };
//  float res = 0.0f;

 while (1)
  {
//      if (adc_read_cplt && (!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4))) {
//    	  // Update ADC config readback
//    	  adc.config_readback = buf[1];
//
//    	   Send measurement over CAN
//          res = (v_fs/(0x7FFF))*(float)buf[0]; // Convert from ADC output to voltage
//          res = 251.493315f * res - 122.744008;
//          send_can_msg((uint8_t*)(&res), sizeof(res));
//
//          adc_read_cplt = 0;
//      }
//
//      if (start_read_adc) {
//    	  // Start new single shot
//        if (Ads1118_Transmit(&adc, (uint32_t*)&buf) != HAL_OK) {
//          Error_Handler();
//        }
//
//         start_read_adc = 0;
//      }

	 GET_PORT_A_READING(port_a_config);
	 GET_PORT_B_READING(port_b_config);
	  if (flash_signal_cmd) {
		  flash_signal_cmd = Tick_SIGNAL(flash_signal_cmd);
	  }
	  if (send_port_a) {
		  float data = CONVERT_ADC_READING(PT_A, port_a_val);
		  INSERT_FLOAT_TO_TX_DATA(data);
		  send_can_msg(pt_1_can_id, tx_data, sizeof(data), &hcan);
		  send_port_a = false;
	  }
	  if (send_port_b) {
		  float data = CONVERT_ADC_READING(PT_B, port_b_val);
		  INSERT_FLOAT_TO_TX_DATA(data);
		  send_can_msg(pt_1_can_id, tx_data, sizeof(data), &hcan);
		  send_port_b = false;
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
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
  hcan.Init.Prescaler = 6;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
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

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 63999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 63999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 99;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 47999;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ADC_CS_GPIO_Port, ADC_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, STATUS_IND_Pin|WARN_IND_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : ADC_CS_Pin */
  GPIO_InitStruct.Pin = ADC_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(ADC_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : STATUS_IND_Pin WARN_IND_Pin */
  GPIO_InitStruct.Pin = STATUS_IND_Pin|WARN_IND_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
float temperature_code_to_temperature(int16_t temperature_code) {
    if (temperature_code & (1 << 14)) {
        temperature_code -= 1;
        temperature_code = ~temperature_code;
    }
    return temperature_code * 0.03125f;
}

//HAL_StatusTypeDef send_can_msg(const uint8_t *data, size_t len) {
//    CAN_TxHeaderTypeDef header;
//    header.IDE = CAN_ID_STD;
//    header.StdId = TX_ID;
//    header.RTR = CAN_RTR_DATA;
//    header.TransmitGlobalTime = DISABLE;
//    header.DLC = len;
//
//    uint32_t mailbox;
//
//    HAL_StatusTypeDef status = HAL_CAN_AddTxMessage(&hcan, &header, data, &mailbox);
//    if (status != HAL_OK) {
//
//    }
//
//    return status;
//}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
	if (adc.config == port_a_config) {
		port_a_val = port_a_rx_buf[0];
		adc_reading = false;
		port_a_read_cplt = true;
	}
	if (adc.config == port_b_config) {
		port_b_val = port_b_rx_buf[0];
		adc_reading = false;
		port_b_read_cplt = true;
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

//	if (htim->Instance == TIM14) {
//		start_read_adc = 1;
//		    should_read_adc = 1;
//
//		    float out[2];
//		    uint32_t buf;
//
//		    buf = HAL_ADC_GetValue(&hadc);
//		    out[TS_MCU] = __LL_ADC_CALC_TEMPERATURE(3300, buf, LL_ADC_RESOLUTION_12B);
//
//		  uint16_t adc_config = ADS1118_CONFIG_DEFAULT | (0b100 << ADS1118_CONFIG_BIT_MUX) | (1 << ADS1118_CONFIG_BIT_SS);
//		    if (Ads1118_Transmit(&adc_config, &hspi1, &buf, 1000) != HAL_OK) {
//		        Error_Handler();
//		    }
//		    out[TS_ADC] = temperature_code_to_temperature(buf);
//
//		    send_can_msg((uint8_t*)(&buf), 4);
//	}
	if (htim->Instance == TIM2) {
//		Handle PT_A timer interrupt
		if (port_a_read_cplt) {
			send_port_a = true;
			port_a_read_cplt =  false;
		}
	}
	if (htim->Instance == TIM3) {
//		Handle PT_B timer interrupt
		if (port_b_read_cplt) {
			send_port_b = true;
			port_b_read_cplt =  false;
		}
	}

}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
//	STATUS_IND_Toggle();
    CAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8];  // Max CAN data length = 8 bytes

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
    {
        Error_Handler();
    }

    // Parse extended ID to extract fields
    uint8_t sender, board_id, msg_type, instance;
    parse_can_extended_id(RxHeader.ExtId, &sender, &board_id, &msg_type, &instance);

    // Check if message is intended for this board
    if (board_id == short_board_id) {
        // Set command based on component type
        switch (msg_type) {

            case MSG_TYPE_LED:
                // Toggle status LED for feedback
                STATUS_IND_Toggle();
                break;

            case MSG_TYPE_FLASH_SIGNAL:
            	flash_signal_cmd = 1;
            	break;
            default:
                // Unknown component type
                break;
        }
    }
}

uint32_t frequency_to_period_ms(uint32_t frequency) {
	return (uint32_t)((1.0f / frequency) * 1000);
}

static void GET_PORT_A_READING(uint16_t config) {
	if (!adc_reading) {
		adc_reading = true;
		adc.config = config;
		Ads1118_Configure(&adc);
		if (Ads1118_Transmit(&adc, port_a_rx_buf) != HAL_OK) {
			Error_Handler();
		}
	}
}
static void GET_PORT_B_READING(uint16_t config) {
	if (!adc_reading) {
		adc_reading = true;
		adc.config = config;
		Ads1118_Configure(&adc);
		if (Ads1118_Transmit(&adc, port_b_rx_buf) != HAL_OK) {
			Error_Handler();
		}
	}
}

static float CONVERT_ADC_READING(PtConfig *pt, int16_t raw_val) {
	float voltage = (v_fs/(0x7FFF))*(float)raw_val;
	float unscaled_val = ((voltage / v_fs) * (pt->max_val - pt->min_val)) + pt->min_val;
	return (unscaled_val * pt->gain) + pt->offset;
}

static void INSERT_FLOAT_TO_TX_DATA(float val) {
	memcpy(&tx_data, &val, sizeof(val));
}
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
	  HAL_GPIO_TogglePin(WARN_IND_GPIO_Port, WARN_IND_Pin);
	  HAL_GPIO_TogglePin(STATUS_IND_GPIO_Port, STATUS_IND_Pin);
	  HAL_Delay(100);
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
