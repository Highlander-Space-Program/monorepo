/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "config/config.h"
#include "utils/board_utils.h"
#include "utils/can_utils.h"
#include "utils/radio_utils.h"
#include "config/servo_config.h"
#include "config/heater_config.h"
#include "config/thermo_config.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LENGTH 8
#define NUM_BOARDS 5
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
uint32_t board_uid[3];
uint8_t data[LENGTH];
bool servos_activated = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_USART6_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//void TOGGLE_SERVO_TEST() {
//	uint32_t servo_board_ext_id = 0x00010108;
//
//	if (servo_cmd == 1) {
//		servo_cmd = 0;
//	} else {
//		servo_cmd = 1;
//	}
//
//	data [0] = servo_cmd;
//	HAL_StatusTypeDef status = send_can_msg(servo_board_ext_id, data, LENGTH, &hcan1);
//	if (status != HAL_OK) {
//	    HAL_GPIO_TogglePin(STATUS_IND_GPIO_Port, STATUS_IND_Pin); // Indicate error
//	}
//}

// 0 is open 1 is close for servo_cmd. see SERVO_CMD in servo_state_machine.h
void ACTUATE_SERVO(uint32_t ext_id, uint8_t servo_cmd) {
	data [0] = servo_cmd;
	HAL_StatusTypeDef status = send_can_msg(ext_id, data, LENGTH, &hcan1);
	if (status != HAL_OK) {
	    HAL_GPIO_TogglePin(STATUS_IND_GPIO_Port, STATUS_IND_Pin); // Indicate error
	}
}

void PAD_CONTROLLER_SETUP_ROUTINE (uint32_t* board_can_ids, uint8_t numBoards){
  STATUS_IND_Toggle();
  HAL_Delay(500);
  STATUS_IND_Toggle();

  FLASH_ALL (board_can_ids, numBoards);
}

// send a flash signal throguh all the boards
void FLASH_ALL (uint32_t* board_can_ids, uint8_t numBoards) {
  uint8_t short_board_id;
  uint32_t* board_uid;

  //0x01020600
  for (int i = 0; i < numBoards; i++) {
	board_uid = GET_BOARD_UID_FROM_CAN_ID (board_can_ids[i]);
	short_board_id = GET_SHORT_BOARD_ID (board_uid);
	uint32_t ext_id = build_can_extended_id (SENDER_PAD_CONTROLLER, short_board_id, MSG_TYPE_FLASH_SIGNAL, 0x00);
	HAL_StatusTypeDef status = send_can_msg(ext_id, data, LENGTH, &hcan1);
	if (status != HAL_OK) {
		HAL_GPIO_TogglePin(STATUS_IND_GPIO_Port, STATUS_IND_Pin); // Indicate error
	}
  }
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
  MX_CAN1_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */

  HAL_UART_Receive_IT(&huart6, rx_buff, 1);
  GET_BOARD_UID (board_uid);
//  short_board_id = GET_SHORT_BOARD_ID (board_uid);

  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_TX_MAILBOX_EMPTY) != HAL_OK)
  {
      Error_Handler();
  }

  // allow anything for now
  CAN_FilterTypeDef canfilterconfig;
  canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
  canfilterconfig.FilterBank = 0;
  canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
  canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
//  canfilterconfig.FilterIdHigh = 0x0000;
//  canfilterconfig.FilterIdLow = CAN_ID_EXT & 0xFFFF;
//  canfilterconfig.FilterMaskIdHigh = 0x0000;
//  canfilterconfig.FilterMaskIdLow = CAN_ID_EXT & 0xFFFF;
  canfilterconfig.FilterIdHigh = 0x0000;
  canfilterconfig.FilterIdLow = 0x0000;
  canfilterconfig.FilterMaskIdHigh = 0x0000;
  canfilterconfig.FilterMaskIdLow = 0x0000;
  HAL_CAN_ConfigFilter(&hcan1, &canfilterconfig);
  HAL_CAN_Start(&hcan1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  uint32_t* no2_board_uid = GET_BOARD_ID_FROM_PNID ("FV-N02");
  uint32_t* no3_board_uid = GET_BOARD_ID_FROM_PNID ("FV-N03");
  uint32_t* no4_board_uid = GET_BOARD_ID_FROM_PNID ("FV-N04");
  uint32_t* pyro_board_uid = GET_BOARD_ID_FROM_PNID ("FV-PYRO");
  uint32_t* pt01_board_uid = GET_BOARD_ID_FROM_PNID ("PT-01");

  uint32_t no2_can_id = GET_CAN_ID_FROM_BOARD_UID (no2_board_uid);
  uint32_t no3_can_id = GET_CAN_ID_FROM_BOARD_UID (no3_board_uid);
  uint32_t no4_can_id = GET_CAN_ID_FROM_BOARD_UID (no4_board_uid);
  uint32_t pyro_can_id = GET_CAN_ID_FROM_BOARD_UID (pyro_board_uid);
  uint32_t pt01_can_id = GET_CAN_ID_FROM_BOARD_UID (pt01_board_uid);

  uint32_t board_can_ids[NUM_BOARDS] = {no2_can_id,
		  no3_can_id,
		  no4_can_id,
		  pyro_can_id,
		  pt01_can_id};
  PAD_CONTROLLER_SETUP_ROUTINE (board_can_ids, NUM_BOARDS);

  uint32_t current = HAL_GetTick();
  uint32_t prev = current;
  uint32_t servo_can_id;
  uint32_t heater_can_id;
  uint32_t thermo_can_id;

  ack = Create_Ack();

  while (1)
  {
//	  enum COMMANDS {
//	    SIGNAL_ALL = 0,  // previously OPEN_EO1 = 0,
//	    REPORT_ALL = 1,  // previously CLOSE_EO1 = 1,
//	  //  OPEN_NO6 = 2,
//	  //  CLOSE_NO6 = 3,
//	    OPEN_NO4 = 4,
//	    CLOSE_NO4 = 5,
//	    OPEN_NO3 = 6,
//	    CLOSE_NO3 = 7,
//	    START_1 = 8,
//	    OPEN_NO2 = 9,
//	    CLOSE_NO2 = 10,
//	    CLOSE_ALL = 12,
//	    DECLOSE_ALL = 13,
//	    ACTIVATE_IGNITER = 14,
//	    DEACTIVATE_IGNITER = 15,
//	    ABORT = 16,
//	    ACTIVATE_SERVOS = 17,
//	    DEACTIVATE_SERVOS = 18,
//	    DEABORT = 19,
//	    CHECK_STATE = 20,
//	    DESTART = 21,
//	  };
	// determine
	switch (rx_buff[0]){
		case SIGNAL_ALL:
			FLASH_ALL (board_can_ids, NUM_BOARDS);
			break;
		case REPORT_ALL:
			// debugging option
			break;
		case ACTIVATE_SERVOS:
			servos_activated = 1;
			break;
		case DEACTIVATE_SERVOS:
			servos_activated = 0;
			break;
		case OPEN_NO2:
			if (servos_activated) {
				Update_Ack(&ack, 5, 0);
				servo_can_id = GET_SERVO_CAN_ID (no2_board_uid, 0);
				ACTUATE_SERVO(servo_can_id, OPEN_SERVO);
			}
			break;
		case CLOSE_NO2:
			if (servos_activated) {
				Update_Ack(&ack, 5, 1);
				servo_can_id = GET_SERVO_CAN_ID (no2_board_uid, 0);
				ACTUATE_SERVO(servo_can_id, CLOSE_SERVO);
			}
			break;
		case OPEN_NO3:
			if (servos_activated) {
				Update_Ack(&ack, 4, 0);
				servo_can_id = GET_SERVO_CAN_ID (no3_board_uid, 0);
				ACTUATE_SERVO(servo_can_id, OPEN_SERVO);
			}
			break;
		case CLOSE_NO3:
			if (servos_activated) {
				Update_Ack(&ack, 4, 1);
				servo_can_id = GET_SERVO_CAN_ID (no3_board_uid, 0);
				ACTUATE_SERVO(servo_can_id, CLOSE_SERVO);
			}
			break;
		case OPEN_NO4:
			if (servos_activated) {
				Update_Ack(&ack, 3, 0);
				servo_can_id = GET_SERVO_CAN_ID (no4_board_uid, 0);
				ACTUATE_SERVO(servo_can_id, OPEN_SERVO);
			}
			break;
		case CLOSE_NO4:
			if (servos_activated) {
				Update_Ack(&ack, 3, 1);
				servo_can_id = GET_SERVO_CAN_ID (no4_board_uid, 0);
				ACTUATE_SERVO(servo_can_id, CLOSE_SERVO);
			}
			break;
		default:
			break;
	}

	current = HAL_GetTick();
	if (current - prev >= 150) {
		tx_buff[0] = ack;
		HAL_UART_Transmit_IT(&huart6, tx_buff, 1);
		prev = current;
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 8;
  hcan1.Init.Mode = CAN_MODE_LOOPBACK;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 9600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

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
  HAL_GPIO_WritePin(STATUS_IND_GPIO_Port, STATUS_IND_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : STATUS_IND_Pin */
  GPIO_InitStruct.Pin = STATUS_IND_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(STATUS_IND_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
   HAL_UART_Receive_IT(&huart6, rx_buff, 1);
   STATUS_IND_Toggle();
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
//   STATUS_IND_Toggle();
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef rxHeader;
	uint8_t rxData[8];

	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK)
	{
		// Process received message
		HAL_GPIO_TogglePin(STATUS_IND_GPIO_Port, STATUS_IND_Pin);
		// Maybe compare the data with what was sent to verify loopback
	}
}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
{
	STATUS_IND_Toggle();
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
