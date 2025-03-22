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
#include "stdbool.h"
#include "servo_utils.h"
#include "config/config.h"
#include "utils/board_utils.h"
#include "utils/can_utils.h"
//#include "heater_utils.h"

#include "servo_state_machine.h"
#include "thermocouple_state_machine.h"
#include "heater_state_machine.h"
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
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

CAN_HandleTypeDef hcan;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim14;
TIM_HandleTypeDef htim16;

// This allows multiple commands to be sent in a row without any potential issues with interrupts occurring over each other.
volatile SERVO_CMD servo_cmd = CLOSE_SERVO;
volatile uint8_t servo_instance = -1;
volatile THERMO_CMD thermo_cmd = TEMP_WAIT;
volatile uint8_t thermo_instance = -1;
volatile HEATER_CMD heater_cmd = H_OFF;
volatile uint8_t heater_instance = -1;
volatile bool new_command_received = 0;
volatile bool flash_signal_cmd = 0;

/* USER CODE BEGIN PV */
static volatile uint32_t adc_val = 0;

uint32_t board_uid[3];
static uint8_t short_board_id;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN_Init(void);
static void MX_TIM14_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//enum COMMANDS {
//	SERVO_OPEN = 0,
//	SERVO_CLOSE = 1,
//	HEATER_ON = 2,
//	HEATER_OFF = 3
//};

void TestServo (Servo* servo) {
  while (1) {
	// LED TURNS ON, SERVO OPENS
	Tick_SERVO(OPEN_SERVO, servo);
	HAL_Delay(1000); // wait 15 seconds

	// LED STAYS ON, SERVO CLOSES
	Tick_SERVO(CLOSE_SERVO, servo);
	HAL_Delay(1000);

	// LED TURNS ON, SERVO GOES TO OPEN ANGLE
	Tick_SERVO(OPEN_SERVO, servo);
	HAL_Delay(1000); // wait 15 seconds

	// LED STAYS ON, SERVO CLOSES
	Tick_SERVO(CLOSE_SERVO, servo);
	HAL_Delay(1000);
  }
}

void TestHeater(Heater* heater) {

	while (1) {
	// TURNS HEATER ON
		Tick_HEATER (H_ON, heater);
		HAL_Delay(5000);


		// TURNS HEATER OFF
		Tick_HEATER (H_OFF, heater);
		HAL_Delay(5000);
	}
}

void TestThermocouple (Thermocouple* thermo){
	while (1) {
		Tick_THERMO(-1, thermo);
		double temp = thermo->temperature;
		HAL_Delay(200);
	}
}

void TestHeaterAuto (Heater* heater, Thermocouple* thermo) {

	while (1) {
	// TURNS HEATER ON
		Tick_THERMO (-1, thermo);
		Tick_HEATER (H_AUTO, heater);
	}
}

void TestServoAndHeater() {

}

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
  MX_TIM14_Init();
  MX_TIM16_Init();
  MX_TIM2_Init();
  MX_ADC_Init();

/* USER CODE BEGIN 2 */
  GET_BOARD_UID (board_uid);
  short_board_id = GET_SHORT_BOARD_ID (board_uid);

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

//  CAN_FilterTypeDef filter;
//
//  // Configure filter to accept all extended IDs
//  filter.FilterActivation = CAN_FILTER_ENABLE;
//  filter.FilterBank = 0;
//  filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
//  filter.FilterMode = CAN_FILTERMODE_IDMASK;
//  filter.FilterScale = CAN_FILTERSCALE_32BIT;
//
//  // Set IDE bit in filter ID (to match extended IDs)
//  filter.FilterIdHigh = 0x0000;
//  filter.FilterIdLow = CAN_ID_EXT & 0xFFFF;
//  // Only require IDE bit match, mask everything else to 0
//  filter.FilterMaskIdHigh = 0x0000;
//  filter.FilterMaskIdLow = CAN_ID_EXT & 0xFFFF;



  if (HAL_CAN_ConfigFilter(&hcan, &filter) != HAL_OK) {
      Error_Handler();
  }
  HAL_ADCEx_Calibration_Start(&hadc);
  HAL_ADC_Start_DMA(&hadc,(uint32_t*)&adc_val,1);

  if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING)) {
    Error_Handler();
  };

  if (HAL_CAN_Start(&hcan) != HAL_OK) {
      Error_Handler();
  }

  STATUS_IND_Toggle();
  HAL_Delay(500);
  STATUS_IND_Toggle();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);

  /* configure board */
  // uint32_t uid[3] = GET_BOARD_UUID();

  /* initialize servos */
  uint32_t servo_can_id = GET_SERVO_CAN_ID(board_uid, 0);
  uint32_t thermo_can_id = GET_THERMO_CAN_ID(board_uid, 0);
  uint32_t heater_can_id = GET_HEATER_CAN_ID(board_uid, 0);
  if (servo_can_id == -1 || thermo_can_id == -1 || heater_can_id == -1) {
	  Error_Handler();
  }

  Servo* servo = construct_servo (servo_can_id, &htim2);
  Thermocouple* thermo = construct_thermo(thermo_can_id, &hadc, &adc_val);
  Heater* heater = construct_heater (heater_can_id, thermo);

  if (!servo) {
	  CRITIAL_ERROR_GENERIC_On();
  }

  if (!thermo) {
	  CRITIAL_ERROR_GENERIC_On();
  }

  if (!heater) {
	  CRITIAL_ERROR_GENERIC_On();
  }
  // struct Heater* heater = construct_header (uid);
  // struct Thermocouple* thermo = construct_thermo (uid);
  // ** COMMENT OUT IF NOT TESTING COMPONENTS ** //



  // ** TESTING FUNCTIONS AT TOP OF FILE ** //
//  TestServo (servo);
//  TestHeater (heater);

//  TestThermocouple (thermo);
//  TestHeaterAuto (heater, thermo);
  // ** TESTING


  // will run through a set of test commands to see if everything works then repeat
  while (1)
  {
    // Execute the current commands on each loop iteration
    Tick_SERVO(servo_cmd, servo);
    Tick_THERMO(thermo_cmd, thermo);
    Tick_HEATER(heater_cmd, heater);

    // Visual feedback when new command is received (optional)
    if (new_command_received) {
        STATUS_IND_Toggle();
        new_command_received = 0;
    }

    // more visual feedback

    if (flash_signal_cmd) {
    	flash_signal_cmd = Tick_SIGNAL (flash_signal_cmd);
    }
  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI14|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
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
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = ENABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 5;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 159999;
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
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 0;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 65535;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  HAL_GPIO_WritePin(GPIOA, SERVO_EN_Pin|HEATER_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, STATUS_IND_Pin|WARN_IND_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SERVO_CONT_Pin HEATER_CONT_Pin */
  GPIO_InitStruct.Pin = SERVO_CONT_Pin|HEATER_CONT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SERVO_EN_Pin HEATER_EN_Pin */
  GPIO_InitStruct.Pin = SERVO_EN_Pin|HEATER_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
/* USER CODE BEGIN 4 */
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
    parseCanExtendedId(RxHeader.ExtId, &sender, &board_id, &msg_type, &instance);

    // Check if message is intended for this board
    if (board_id == short_board_id) {
        // Set command based on component type
        switch (msg_type) {
            case MSG_TYPE_SERVO:
                // First byte contains the servo command
                servo_cmd = RxData[0];
                servo_instance = instance;
                new_command_received = 1;
                break;

            case MSG_TYPE_THERMOCOUPLE:
                // First byte contains the thermocouple command
                thermo_cmd = RxData[0];
                thermo_instance = instance;
                new_command_received = 1;
                break;

            case MSG_TYPE_HEATER:
                // First byte contains the heater command
                heater_cmd = RxData[0];
                heater_instance = instance;
                new_command_received = 1;
                break;

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
