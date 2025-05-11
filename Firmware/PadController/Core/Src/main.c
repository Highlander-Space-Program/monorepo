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
#include <stdio.h>  // For printf, if used for debugging
#include <string.h> // For strlen, etc.
#include "stdbool.h"
#include "config/config.h"
#include "utils/board_utils.h"
#include "utils/can_utils.h"
// #include "utils/radio_utils.h" // XBee specific includes are used instead
#include "config/servo_config.h"
#include "config/heater_config.h"
#include "config/thermo_config.h"
#include "PC_state.h"
#include "igniter.h"

// XBee radio includes
#include "stm32/platform_config.h"    // Defines xbee_serial_t, XBEE_FRAME_HANDLERS, etc.
#include "stm32/xbee_platform_uart.h" // For xbee_platform_init, xbee_platform_serial, xbee_uart_isr
#include "stm32/xbee_actions.h"       // For xbee_frame_handlers, xbee_rx_packet_available/dequeue
#include "stm32/xbee_handler.h"       // For XBeeRxFrame_t, xbee_handler_*, etc.
#include "xbee/platform.h"            // Core XBee library platform types
#include "xbee/device.h"              // For xbee_dev_t, xbee_device_init, xbee_dev_tick etc.
#include "xbee/atcmd.h"               // For AT command interface, xbee_cmd_tick
#include "xbee/byteorder.h"           // For htobe16(), etc.
#include "xbee/wpan.h"                // For WPAN_IEEE_ADDR_BROADCAST, etc.
// #include "wpan/types.h"            // This seems redundant if xbee/wpan.h and xbee/device.h are included.
                                    // addr64 is in xbee/device.h
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NUM_BOARDS 5
#define XBEE_DEVICE_INIT_TIMEOUT_MS 10000 // Timeout for XBee library initial handshake with module
#define XBEE_AT_COMMAND_TIMEOUT_MS  5000  // Default timeout for AT commands sent via handler

// --- Define Target XBee Address ---
// Replace with your actual target XBee's 64-bit IEEE address (MSB first)
#define TARGET_XBEE_ADDR_64_B0 0x00 // Example byte 0 (MSB)
#define TARGET_XBEE_ADDR_64_B1 0x13 // Example byte 1
#define TARGET_XBEE_ADDR_64_B2 0xA2 // Example byte 2
#define TARGET_XBEE_ADDR_64_B3 0x00 // Example byte 3
#define TARGET_XBEE_ADDR_64_B4 0x41 // Example byte 4
#define TARGET_XBEE_ADDR_64_B5 0xB3 // Example byte 5
#define TARGET_XBEE_ADDR_64_B6 0xF9 // Example byte 6
#define TARGET_XBEE_ADDR_64_B7 0xC6 // Example byte 7 (LSB)

#define TARGET_XBEE_ADDR_16 WPAN_NET_ADDR_UNDEFINED // 0xFFFE, usually means use 64-bit address for routing
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
// typedef struct { // This typedef is not used in main.c; a similar one is internal to xbee_handler.c
//     volatile bool done;
//     uint32_t      value;
//     uint16_t      flags;
// } at_sync_t;

// Debug variables (user-defined)
// static volatile uint32_t dbg_cr1;
// volatile uint32_t dbg_iser;
// volatile bool     dbg_rxneie;
// volatile bool     dbg_irqn;
// volatile uint32_t dbg_lib_AP;
// volatile uint16_t dbg_xbee_flags;

static xbee_dev_t   xbee; // XBee device instance

uint32_t board_uid[3];
bool servos_activated = 0;

// Assuming GET_BOARD_ID_FROM_PNID and GET_CAN_ID_FROM_BOARD_UID are defined in your config/utils
uint32_t* no2_board_uid;
uint32_t* no3_board_uid;
uint32_t* no4_board_uid;
uint32_t* pyro_board_uid;
uint32_t* pt01_board_uid;

uint32_t no2_can_id;
uint32_t no3_can_id;
uint32_t no4_can_id;
uint32_t pyro_can_id;
uint32_t pt01_can_id;

uint32_t board_can_ids[NUM_BOARDS];

uint32_t current_time_ms = 0;
uint32_t prev_ack_send_time_ms = 0;
uint32_t servo_can_id;
// uint32_t heater_can_id; // Declared but not used in provided snippet
// uint32_t thermo_can_id; // Declared but not used in provided snippet

XBeeRxFrame_t current_received_xbee_frame; // Buffer for the latest dequeued XBee frame

// Target XBee address for sending data (e.g., periodic ACKs)
static const addr64 g_target_xbee_ieee_addr = { // Using designated initializer for clarity
    .b = {TARGET_XBEE_ADDR_64_B0, TARGET_XBEE_ADDR_64_B1, TARGET_XBEE_ADDR_64_B2, TARGET_XBEE_ADDR_64_B3,
          TARGET_XBEE_ADDR_64_B4, TARGET_XBEE_ADDR_64_B5, TARGET_XBEE_ADDR_64_B6, TARGET_XBEE_ADDR_64_B7}
};
static const uint16_t g_target_xbee_network_addr = TARGET_XBEE_ADDR_16;

uint8_t ack_byte_value;     // Holds the current ACK status byte to be sent periodically
uint8_t tx_ack_payload[1];  // Buffer for sending the ACK byte via XBee

// uint8_t rx_buff[1]; // This buffer was for HAL_UART_Receive_IT, which is not used for XBee RX.
                     // XBee commands come via current_received_xbee_frame.
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

/**
  * @brief Sends a status response back to the originator of an XBee message.
  * @param originating_frame The received XBee frame that triggered this response.
  * @param status_code A byte representing the status to send.
  */
static void send_pad_status_response(const XBeeRxFrame_t* originating_frame, uint8_t status_code) {
    if (!originating_frame) return;

    uint8_t response_payload[1];
    response_payload[0] = status_code;

    // Send back to the source of the original message
    // Use frame_id_request = 0 for no TX status needed for this simple response
    int send_status = xbee_handler_send_data_frame(&xbee, // Use the global xbee instance
                                       &originating_frame->source_addr_64,
                                       originating_frame->source_addr_16,
                                       response_payload,
                                       sizeof(response_payload),
                                       0, // No TX status requested for this response
                                       XBEE_HANDLER_TX_OPT_NONE);
    if (send_status < 0) {
        // Failed to send response, perhaps log or blink an error LED
        // printf("Error sending pad status response: %d\r\n", send_status);
        // HAL_GPIO_WritePin(STATUS_IND_GPIO_Port, STATUS_IND_Pin, GPIO_PIN_SET); // Example: Turn on LED on error
    }
}

/**
  * @brief Setup routine for the Pad Controller.
  * @param board_can_ids Array of CAN IDs for connected boards.
  * @param numBoards Number of boards in the array.
  */
void PAD_CONTROLLER_SETUP_ROUTINE (uint32_t* board_can_ids, uint8_t numBoards){
  HAL_GPIO_TogglePin(STATUS_IND_GPIO_Port, STATUS_IND_Pin); // Toggle LED
  HAL_Delay(500);
  HAL_GPIO_TogglePin(STATUS_IND_GPIO_Port, STATUS_IND_Pin); // Toggle LED back

  FLASH_ALL (board_can_ids, numBoards); // Assuming FLASH_ALL is defined elsewhere
}

/**
  * @brief Processes commands received via XBee for the Pad Controller.
  * @param received_xbee_frame Pointer to the dequeued XBee frame containing the command.
  */
static void process_pad_controller_command(XBeeRxFrame_t* received_xbee_frame) {
    if (received_xbee_frame == NULL || received_xbee_frame->length == 0) {
        // No payload or invalid frame pointer.
        return;
    }

    uint8_t command = received_xbee_frame->payload[0]; // First byte of payload is the command

    // Optional: Log received command for debugging
    // printf("Processing Pad Controller Command: %u from 0x%08lX%08lX (16:0x%04X), Opts: 0x%02X, Len: %u\r\n",
    //        command,
    //        (uint32_t)(received_xbee_frame->source_addr_64.l >> 32), (uint32_t)received_xbee_frame->source_addr_64.l,
    //        received_xbee_frame->source_addr_16,
    //        received_xbee_frame->receive_options,
    //        received_xbee_frame->length);

    switch (command) {
		case SIGNAL_ALL:
			FLASH_ALL (board_can_ids, NUM_BOARDS);
            //send_pad_status_response(received_xbee_frame, ACK_SIGNAL_ALL_OK);
			break;
		case REPORT_ALL:
			// Implement reporting logic here
            // //send_pad_status_response(received_xbee_frame, ACK_REPORT_ALL_OK); // Example
			break;
		case ACTIVATE_SERVOS:
			servos_activated = 1;
            //send_pad_status_response(received_xbee_frame, ACK_SERVOS_ACTIVATED);
			break;
		case DEACTIVATE_SERVOS:
			servos_activated = 0;
            //send_pad_status_response(received_xbee_frame, ACK_SERVOS_DEACTIVATED);
			break;
		case OPEN_NO2:
			if (servos_activated) {
				Update_Ack(&ack_byte_value, 5, 0);
				servo_can_id = GET_SERVO_CAN_ID (no2_board_uid, 1);
				ACTUATE_SERVO(servo_can_id, OPEN_SERVO);
                //send_pad_status_response(received_xbee_frame, ACK_OPEN_NO2_OK);
			} else {
                //send_pad_status_response(received_xbee_frame, ERR_SERVOS_NOT_ACTIVE);
            }
			break;
		case CLOSE_NO2:
			if (servos_activated) {
				Update_Ack(&ack_byte_value, 5, 1);
				servo_can_id = GET_SERVO_CAN_ID (no2_board_uid, 1);
				ACTUATE_SERVO(servo_can_id, CLOSE_SERVO);
                //send_pad_status_response(received_xbee_frame, ACK_CLOSE_NO2_OK);
			} else {
                //send_pad_status_response(received_xbee_frame, ERR_SERVOS_NOT_ACTIVE);
            }
			break;
		case OPEN_NO3:
			if (servos_activated) {
				Update_Ack(&ack_byte_value, 4, 0);
				servo_can_id = GET_SERVO_CAN_ID (no3_board_uid, 1);
				ACTUATE_SERVO(servo_can_id, OPEN_SERVO);
                //send_pad_status_response(received_xbee_frame, ACK_OPEN_NO3_OK);
			} else {
                //send_pad_status_response(received_xbee_frame, ERR_SERVOS_NOT_ACTIVE);
            }
			break;
		case CLOSE_NO3:
			if (servos_activated) {
				Update_Ack(&ack_byte_value, 4, 1);
				servo_can_id = GET_SERVO_CAN_ID (no3_board_uid, 1);
				ACTUATE_SERVO(servo_can_id, CLOSE_SERVO);
                //send_pad_status_response(received_xbee_frame, ACK_CLOSE_NO3_OK);
			} else {
                //send_pad_status_response(received_xbee_frame, ERR_SERVOS_NOT_ACTIVE);
            }
			break;
		case OPEN_NO4:
			if (servos_activated) {
				Update_Ack(&ack_byte_value, 3, 0);
				servo_can_id = GET_SERVO_CAN_ID (no4_board_uid, 1);
				ACTUATE_SERVO(servo_can_id, OPEN_SERVO);
                //send_pad_status_response(received_xbee_frame, ACK_OPEN_NO4_OK);
			} else {
                //send_pad_status_response(received_xbee_frame, ERR_SERVOS_NOT_ACTIVE);
            }
			break;
		case CLOSE_NO4:
			if (servos_activated) {
				Update_Ack(&ack_byte_value, 3, 1);
				servo_can_id = GET_SERVO_CAN_ID (no4_board_uid, 1);
				ACTUATE_SERVO(servo_can_id, CLOSE_SERVO);
                //send_pad_status_response(received_xbee_frame, ACK_CLOSE_NO4_OK);
			} else {
                //send_pad_status_response(received_xbee_frame, ERR_SERVOS_NOT_ACTIVE);
            }
			break;
		case OPEN_PYRO:
			if (servos_activated) {
				Update_Ack(&ack_byte_value, 1, 0);
				servo_can_id = GET_SERVO_CAN_ID (pyro_board_uid, 1);
				ACTUATE_SERVO(servo_can_id, OPEN_SERVO);
                //send_pad_status_response(received_xbee_frame, ACK_OPEN_PYRO_OK);
			} else {
                //send_pad_status_response(received_xbee_frame, ERR_SERVOS_NOT_ACTIVE);
            }
			break;
		case CLOSE_PYRO:
			if (servos_activated) {
				Update_Ack(&ack_byte_value, 1, 1);
				servo_can_id = GET_SERVO_CAN_ID (pyro_board_uid, 1);
				ACTUATE_SERVO(servo_can_id, CLOSE_SERVO);
                //send_pad_status_response(received_xbee_frame, ACK_CLOSE_PYRO_OK);
			} else {
                //send_pad_status_response(received_xbee_frame, ERR_SERVOS_NOT_ACTIVE);
            }
			break;
		default:
            //send_pad_status_response(received_xbee_frame, ERR_UNKNOWN_COMMAND);
			break;
	}
    // Toggle LED on any processed command for visual feedback (can be removed for production)
    HAL_GPIO_TogglePin(STATUS_IND_GPIO_Port, STATUS_IND_Pin);
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

  // Initialize board UID (assuming GET_BOARD_UID is defined)
  GET_BOARD_UID (board_uid);

  // Initialize CAN
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

  // --- XBee Initialization ---
  // 1. Initialize the platform serial port for XBee (configures UART, enables RX interrupt via xbee_uart_isr)
  xbee_platform_init();

  // 2. Initialize the XBee device structure
  xbee_dev_init(&xbee, xbee_platform_serial(), always_awake, NULL);
  xbee_dev_flowcontrol(&xbee, 0);
  xbee_cmd_init_device(&xbee);

  // one tick to *send* the first "HV" frame
  xbee_dev_tick(&xbee);

  xbee_cmd_tick();
  // put a breakpoint here and scope your TX pin — you should see:
  //   7E 00 04 08 01 48 56 58
  int status = 0;
  int xbee_init_status = 0;
  uint8_t last_command_received = 0xFF; // Initialize to a non-command value
  // now spin until that handshake comes back

  do {
  	  xbee_dev_tick(&xbee);
  	  xbee_cmd_tick();
	  status = xbee_cmd_query_status(&xbee);
  } while (status == -EBUSY);

  // check result
  if (status != 0) {
	  // handshake failed — set a breakpoint here to inspect 'status'
	  return status;
  }


  // 4. Initialize the application-level XBee handler's received frame queue
  xbee_handler_init_rx_queue();

  // Initialize the ACK byte value
  ack_byte_value = Create_Ack(); // Assuming Create_Ack() is defined

  // Initial Pad Controller Setup Routine
  // NOTE: HAL_UART_Receive_IT(&huart6, rx_buff, 1); is NOT called here.
  // XBee library handles UART RX via xbee_uart_isr() called from USART6_IRQHandler.

  prev_ack_send_time_ms = HAL_GetTick(); // Initialize for periodic ACK sending

  no2_board_uid = GET_BOARD_ID_FROM_PNID ("FV-N02");
  no3_board_uid = GET_BOARD_ID_FROM_PNID ("FV-N03");
  no4_board_uid = GET_BOARD_ID_FROM_PNID ("FV-N04");
  pyro_board_uid = GET_BOARD_ID_FROM_PNID ("FV-PYRO");
  pt01_board_uid = GET_BOARD_ID_FROM_PNID ("PT-01");

  no2_can_id = GET_CAN_ID_FROM_BOARD_UID (no2_board_uid);
  no3_can_id = GET_CAN_ID_FROM_BOARD_UID (no3_board_uid);
  no4_can_id = GET_CAN_ID_FROM_BOARD_UID (no4_board_uid);
  pyro_can_id = GET_CAN_ID_FROM_BOARD_UID (pyro_board_uid);
  pt01_can_id = GET_CAN_ID_FROM_BOARD_UID (pt01_board_uid);

  board_can_ids[0] = no2_can_id;
  board_can_ids[1] = no3_can_id;
  board_can_ids[2] = no4_can_id;
  board_can_ids[3] = pyro_can_id;
  board_can_ids[4] = pt01_can_id;

  PAD_CONTROLLER_SETUP_ROUTINE (board_can_ids, NUM_BOARDS);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // --- XBee Processing ---
    xbee_dev_tick(&xbee);
    xbee_cmd_tick();
    xbee_handler_service_rx_from_library();

    if (xbee_handler_is_rx_frame_available()) {
        if (xbee_handler_rx_frame_dequeue(&current_received_xbee_frame)) {
            if (current_received_xbee_frame.length > 0) {
                last_command_received = current_received_xbee_frame.payload[0];
            } else {
                last_command_received = 0xFF; // Or some other "no command" indicator
            }
            process_pad_controller_command(&current_received_xbee_frame);
        }
    }

    // --- Application Specific Ticks ---
    // Pass the last command received via XBee, or a default if no command yet.
    // Ensure Tick_Igniter and PadController_Tick can handle a "no command" value if appropriate.
	Tick_Igniter(last_command_received, &ack_byte_value);
	PadController_Tick(last_command_received, pyro_board_uid, &ack_byte_value);
	Tick_Breakwire_LED();

	// --- Periodic ACK Sending ---
	current_time_ms = HAL_GetTick();
	if (current_time_ms - prev_ack_send_time_ms >= 300) {
		FLASH_ALL (board_can_ids, NUM_BOARDS);

		tx_ack_payload[0] = ack_byte_value; // Load the current ACK value
		int send_status = xbee_handler_send_byte_array(&xbee,
                                         &g_target_xbee_ieee_addr,
                                         g_target_xbee_network_addr,
                                         tx_ack_payload,
                                         sizeof(tx_ack_payload),
                                         0, // Frame ID 0 for no TX status response
                                         XBEE_HANDLER_TX_OPT_NONE);
        if (send_status < 0) {
            // Failed to send periodic ACK
            // printf("Error sending periodic ACK: %d\r\n", send_status);
             HAL_GPIO_TogglePin(STATUS_IND_GPIO_Port, STATUS_IND_Pin); // Example error
        } else {
        	HAL_GPIO_TogglePin(STATUS_IND_GPIO_Port, STATUS_IND_Pin); // Example success
        }
		prev_ack_send_time_ms = current_time_ms;
	}
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
  // Ensure USART6_IRQn is enabled and its handler calls xbee_uart_isr().
  // This is typically done in HAL_UART_MspInit (called by HAL_UART_Init) or here.
  // Example:
  // HAL_NVIC_SetPriority(USART6_IRQn, 0, 0); // Set appropriate priority
  // HAL_NVIC_EnableIRQ(USART6_IRQn);
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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, BRK_CONT_LED_SINK_Pin|BRK_CONT_LED_Pin|IGNITER_Pin|BRK_CONT_SINK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : STATUS_IND_Pin */
  GPIO_InitStruct.Pin = STATUS_IND_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(STATUS_IND_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : IGNITER_CONT_Pin */
  GPIO_InitStruct.Pin = IGNITER_CONT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IGNITER_CONT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BRK_CONT_LED_SINK_Pin BRK_CONT_LED_Pin IGNITER_Pin BRK_CONT_SINK_Pin */
  GPIO_InitStruct.Pin = BRK_CONT_LED_SINK_Pin|BRK_CONT_LED_Pin|IGNITER_Pin|BRK_CONT_SINK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : BRK_CONT_Pin */
  GPIO_InitStruct.Pin = BRK_CONT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BRK_CONT_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef rxHeader;
	uint8_t rxData[8];

	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK)
	{
		// Process received CAN message
		// HAL_GPIO_TogglePin(STATUS_IND_GPIO_Port, STATUS_IND_Pin); // Example
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
    HAL_GPIO_TogglePin(STATUS_IND_GPIO_Port, STATUS_IND_Pin); // Fast blink for error
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
  // printf("Wrong parameters value: file %s on line %lu\r\n", (char *)file, line);
  Error_Handler();
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
