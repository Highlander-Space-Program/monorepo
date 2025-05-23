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
#include "W25Qxx.h"
#include "MS5607SPI.h"
#include "math.h"

#include <stdio.h>  // For printf, if used for debugging
#include <string.h> // For strlen, etc.
#include "stdbool.h"
#include "config/config.h"
//#include "utils/board_utils.h"
#include "utils/can_utils.h"

// XBee radio includes
#include "stm32/platform_config.h"    // Defines xbee_serial_t, XBEE_FRAME_HANDLERS, etc.
#include "stm32/xbee_platform_uart.h" // For xbee_platform_init, xbee_platform_serial, xbee_uart_isr
#include "stm32/xbee_actions.h"       // For xbee_frame_handlers, xbee_rx_packet_available/dequeue
#include "stm32/xbee_handler.h"       // For XBeeRxFrame_t, xbee_handler_*, etc.
#include "xbee/platform.h"            // Core XBee library platform types
#include "xbee/device.h"              // For xbee_dev_t, xbee_device_init, xbee_dev_tick etc.
#include "xbee/atcmd.h"               // For AT command interface, xbee_cmd_tick, xbee_next_frame_id
#include "xbee/byteorder.h"           // For htobe16(), etc.
#include "xbee/wpan.h"                // For WPAN_IEEE_ADDR_BROADCAST, etc.
#include <errno.h>                    // For error codes like ENOSPC, ETIMEDOUT
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NUM_BOARDS 7
#define XBEE_DEVICE_INIT_TIMEOUT_MS 10000 // Timeout for XBee library initial handshake with module
#define XBEE_AT_COMMAND_TIMEOUT_MS  5000  // Default timeout for AT commands sent via handler

#define MAX_CONSECUTIVE_XBEE_TX_FAILS 10
#define XBEE_RETRY_PERIOD_MS 10000
#define PERIODIC_ACK_INTERVAL_MS 1000
#define ACK_PENDING_TIMEOUT_MS 5000 // Timeout for an ACK waiting for TX status

//0013A2004238A3E3
#define TARGET_XBEE_ADDR_64_B0 0x00 // Example byte 0 (MSB)
#define TARGET_XBEE_ADDR_64_B1 0x13 // Example byte 1
#define TARGET_XBEE_ADDR_64_B2 0xA2 // Example byte 2
#define TARGET_XBEE_ADDR_64_B3 0x00 // Example byte 3
#define TARGET_XBEE_ADDR_64_B4 0x42 // Example byte 4
#define TARGET_XBEE_ADDR_64_B5 0x38 // Example byte 5
#define TARGET_XBEE_ADDR_64_B6 0xA3 // Example byte 6
#define TARGET_XBEE_ADDR_64_B7 0xE3 // Example byte 7 (LSB)

#define TARGET_XBEE_ADDR_16 WPAN_NET_ADDR_UNDEFINED // 0xFFFE, usually means use 64-bit address for routing
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart5;

/* USER CODE BEGIN PV */
volatile bool g_xbee_target_unreachable = false;
volatile uint32_t g_xbee_tx_fail_count = 0;
volatile uint8_t g_periodic_ack_frame_id = 0; // Stores the frame_id of the last periodic ACK sent. 0 means no ACK is pending TX status.
volatile uint32_t g_ack_pending_timestamp = 0; // Timestamp for when an ACK started waiting for TX status

static xbee_dev_t   xbee;

uint32_t board_uid[3];
bool servos_activated = 0;

uint32_t* no2_board_uid;
uint32_t* no3_board_uid;
uint32_t* no4_board_uid;
uint32_t* pyro_board_uid;
uint32_t* pt01_board_uid;
uint32_t* pt03_board_uid;
uint32_t* pc01_board_uid;

uint32_t no2_can_id;
uint32_t no3_can_id;
uint32_t no4_can_id;
uint32_t pyro_can_id;
uint32_t pt01_can_id;
uint32_t pt03_can_id;
uint32_t pc01_can_id;

uint32_t board_can_ids[NUM_BOARDS];

uint32_t current_time_ms = 0;
uint32_t prev_ack_send_time_ms = 0;
uint32_t servo_can_id;

XBeeRxFrame_t current_received_xbee_frame;
static uint8_t short_board_id;

static const addr64 g_target_xbee_ieee_addr = {
    .b = {TARGET_XBEE_ADDR_64_B0, TARGET_XBEE_ADDR_64_B1, TARGET_XBEE_ADDR_64_B2, TARGET_XBEE_ADDR_64_B3,
          TARGET_XBEE_ADDR_64_B4, TARGET_XBEE_ADDR_64_B5, TARGET_XBEE_ADDR_64_B6, TARGET_XBEE_ADDR_64_B7}
};
static const uint16_t g_target_xbee_network_addr = TARGET_XBEE_ADDR_16;

uint8_t ack_byte_value;
uint8_t tx_ack_payload[1];
uint8_t last_command_received = 0xFF; // Stores the last command byte received via XBee
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_CAN1_Init(void);
static void MX_UART5_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t TxData[30] = "Hello World";
uint8_t RxData[30];

float indx = 0;
float temp = 0;
float press = 0;
double alt = 0;
bool reset = false;
float position = 0;
uint32_t pageNum;
float tNum1 = 100.58;
float rNum1 = 0;

void FLIGHT_COMPUTER_SETUP_ROUTINE (uint32_t* board_can_ids, uint8_t numBoards){
  FLASH_ALL (board_can_ids, numBoards);
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
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_CAN1_Init();
  MX_UART5_Init();
  /* USER CODE BEGIN 2 */
//  HAL_GPIO_TogglePin(STATUS_IND_GPIO_Port, STATUS_IND_Pin);
//  HAL_Delay(3000); // Initial status indication
//  HAL_GPIO_TogglePin(STATUS_IND_GPIO_Port, STATUS_IND_Pin);


  GET_BOARD_UID (board_uid);
  short_board_id = GET_SHORT_BOARD_ID(board_uid);

  // --- CAN Initialization ---
  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_TX_MAILBOX_EMPTY) != HAL_OK)
  {
      Error_Handler();
  }

  CAN_FilterTypeDef canfilterconfig;
  canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
  canfilterconfig.FilterBank = 0;
  canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
  canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
  canfilterconfig.FilterIdHigh = 0x0000;
  canfilterconfig.FilterIdLow = 0x0000;
  canfilterconfig.FilterMaskIdHigh = 0x0000;
  canfilterconfig.FilterMaskIdLow = 0x0000;
  if (HAL_CAN_ConfigFilter(&hcan1, &canfilterconfig) != HAL_OK)
  {
      Error_Handler();
  }
  if (HAL_CAN_Start(&hcan1) != HAL_OK)
  {
      Error_Handler();
  }

  FLIGHT_COMPUTER_SETUP_ROUTINE();

  //MS5607_Init(&hspi1, GPIOB, 12);
  W25Q_Reset();

	//W25Q_Write_Page(1, 10,strlen(TxData), TxData);

  //position = W25Q_Read_NUM(1,2);
  //pageNum = (position / 255) + 2
  //W25Q_Write_NUM(1, 15, tNum1);

  W25Q_Write_NUM(1, 10, tNum1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	//MS5607Update();
	//temp = MS5607GetTemperatureC();
	//press = MS5607GetPressurePa();
	//alt = (44330 * (1-pow((press/101325),(1/5.255)))) * 3.28;

	//W25Q_Write_NUM(pageNum, position - (pageNum * 255), alt);

	position += 2;
	pageNum = (position / 255) + 2;

	rNum1 = W25Q_Read_NUM(1, 15);


	HAL_Delay (100);
	//W25Q_Read(0, 250, 20, RxData);
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
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
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
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_15, GPIO_PIN_SET);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef RxHeader;
	uint8_t RxData[8]; // Max CAN data length = 8 bytes

	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK) {
		// Log error or use Error_Handler();
		return;
	}

	// --- Process only Extended ID messages ---
	if (RxHeader.IDE != CAN_ID_EXT) {
		return; // Ignore standard ID messages
	}

	// The 29-bit ID received from the header
	uint32_t received_id_29bit = RxHeader.ExtId;

	// --- Check if the message is an ACK (using the MSB = bit 28) ---
	bool is_incoming_ack = (received_id_29bit & CAN_ID_ACK_FLAG_29BIT) != 0;

	// --- Parse the ID fields ---
	// (parse_can_extended_id ignores the ACK bit itself if it uses masks correctly)
	uint8_t sender, board_id, msg_type, instance;
	parse_can_extended_id(received_id_29bit, &sender, &board_id, &msg_type, &instance);

    if (is_incoming_ack){
		// Construct the payload for XBee: 4-byte CAN ID (shifted) + CAN Data
		uint8_t xbee_payload[4 + 8]; // Max size: 4 bytes ID + 8 bytes data
		size_t xbee_payload_len = 4 + RxHeader.DLC; // Total length

		// Ensure DLC is valid before proceeding
		if (RxHeader.DLC > 8) {
			 // Invalid CAN DLC, handle error or ignore
			 return;
		}

		// 1. Get the 32-bit shifted CAN ID representation (as expected by Python side)
		//    This includes the ACK flag already set in received_id_29bit
		uint32_t can_id_for_xbee_32bit = received_id_29bit << 3;

		// 2. Copy CAN ID into XBee payload buffer (Big Endian)
		xbee_payload[0] = (uint8_t)((can_id_for_xbee_32bit >> 24) & 0xFF);
		xbee_payload[1] = (uint8_t)((can_id_for_xbee_32bit >> 16) & 0xFF);
		xbee_payload[2] = (uint8_t)((can_id_for_xbee_32bit >> 8) & 0xFF);
		xbee_payload[3] = (uint8_t)(can_id_for_xbee_32bit & 0xFF);

		// 3. Copy CAN Data payload (if any)
		if (RxHeader.DLC > 0) {
			memcpy(&xbee_payload[4], RxData, RxHeader.DLC);
		}

		// 4. Send the combined payload via XBee to the Pad Controller
		//    Using the assumed higher-level function for simplicity here.
		//    Using Frame ID 0 often means no TX status is requested or needed for this forward.
		//    Adjust parameters based on your actual XBee handler function.
		int send_status = xbee_handler_send_byte_array(&xbee, // XBee device instance
													   &g_target_xbee_ieee_addr, // Target 64-bit Addr
													   g_target_xbee_network_addr,     // Target 16-bit Addr (0xFFFE)
													   xbee_payload,                     // Data to send
													   xbee_payload_len,                 // Length of data
													   0,                                // Frame ID (0 = No TX Status)
													   0);                               // Options (e.g., XBEE_HANDLER_TX_OPT_NONE)

		if (send_status < 0) {
		// Handle XBee transmission failure (e.g., log error, queue full?)
		// printf("Failed to forward CAN ACK (ID: 0x%08lX) via XBee, status: %d\r\n", received_id_29bit, send_status);
		} else {
			// Message successfully queued for XBee transmission
			// printf("Forwarded CAN ACK (ID: 0x%08lX) via XBee.\r\n", received_id_29bit);
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
