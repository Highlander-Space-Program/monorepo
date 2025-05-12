/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file            : main.c
 * @brief           : Main program body
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
#define NUM_BOARDS 5
#define XBEE_DEVICE_INIT_TIMEOUT_MS 10000 // Timeout for XBee library initial handshake with module
#define XBEE_AT_COMMAND_TIMEOUT_MS  5000  // Default timeout for AT commands sent via handler

#define MAX_CONSECUTIVE_XBEE_TX_FAILS 10
#define XBEE_RETRY_PERIOD_MS 10000
#define PERIODIC_ACK_INTERVAL_MS 1000
#define ACK_PENDING_TIMEOUT_MS 5000 // Timeout for an ACK waiting for TX status

// --- Define Target XBee Address ---
// Replace with your actual target XBee's 64-bit IEEE address (MSB first)

// Target XBee 1 (Commented out)
//0013A20041B3F9C6
#define TARGET_XBEE_ADDR_64_B0 0x00 // Example byte 0 (MSB)
#define TARGET_XBEE_ADDR_64_B1 0x13 // Example byte 1
#define TARGET_XBEE_ADDR_64_B2 0xA2 // Example byte 2
#define TARGET_XBEE_ADDR_64_B3 0x00 // Example byte 3
#define TARGET_XBEE_ADDR_64_B4 0x41 // Example byte 4
#define TARGET_XBEE_ADDR_64_B5 0xB3 // Example byte 5
#define TARGET_XBEE_ADDR_64_B6 0xF9 // Example byte 6
#define TARGET_XBEE_ADDR_64_B7 0xC6 // Example byte 7 (LSB)

// Target XBee 2 (Active)
//0013A2004238A3E3
//#define TARGET_XBEE_ADDR_64_B0 0x00 // Example byte 0 (MSB)
//#define TARGET_XBEE_ADDR_64_B1 0x13 // Example byte 1
//#define TARGET_XBEE_ADDR_64_B2 0xA2 // Example byte 2
//#define TARGET_XBEE_ADDR_64_B3 0x00 // Example byte 3
//#define TARGET_XBEE_ADDR_64_B4 0x42 // Example byte 4
//#define TARGET_XBEE_ADDR_64_B5 0x38 // Example byte 5
//#define TARGET_XBEE_ADDR_64_B6 0xA3 // Example byte 6
//#define TARGET_XBEE_ADDR_64_B7 0xE3 // Example byte 7 (LSB)

#define TARGET_XBEE_ADDR_16 WPAN_NET_ADDR_UNDEFINED // 0xFFFE, usually means use 64-bit address for routing
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
// Debug variables (user-defined)
// static volatile uint32_t dbg_cr1;
// volatile uint32_t dbg_iser;
// volatile bool     dbg_rxneie;
// volatile bool     dbg_irqn;
// volatile uint32_t dbg_lib_AP;
// volatile uint16_t dbg_xbee_flags;

// Global XBee communication state variables
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

uint32_t no2_can_id;
uint32_t no3_can_id;
uint32_t no4_can_id;
uint32_t pyro_can_id;
uint32_t pt01_can_id;

uint32_t board_can_ids[NUM_BOARDS];

uint32_t current_time_ms = 0;
uint32_t prev_ack_send_time_ms = 0;
uint32_t servo_can_id;

XBeeRxFrame_t current_received_xbee_frame;

static const addr64 g_target_xbee_ieee_addr = {
    .b = {TARGET_XBEE_ADDR_64_B0, TARGET_XBEE_ADDR_64_B1, TARGET_XBEE_ADDR_64_B2, TARGET_XBEE_ADDR_64_B3,
          TARGET_XBEE_ADDR_64_B4, TARGET_XBEE_ADDR_64_B5, TARGET_XBEE_ADDR_64_B6, TARGET_XBEE_ADDR_64_B7}
};
static const uint16_t g_target_xbee_network_addr = TARGET_XBEE_ADDR_16;

uint8_t ack_byte_value;
uint8_t tx_ack_payload[1];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_USART6_UART_Init(void);
/* USER CODE BEGIN PFP */
// Recommendation: Implement an Independent Watchdog (IWDG) and pet it in the main loop
// void IWDG_Init(void); // Example
// void IWDG_Refresh(void); // Example
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void send_pad_status_response(const XBeeRxFrame_t* originating_frame, uint8_t status_code) {
    if (!originating_frame) return;

    // Before sending any response, check if the target (original sender) is considered reachable.
    // This is a simplified check. For a multi-device system, you'd need per-device reachability.
    // For now, we assume responses are critical and attempt them, but this could be expanded.
    // if (g_xbee_target_unreachable) {
    //     // printf("Target for response is currently marked unreachable. Skipping response.\r\n");
    //     return;
    // }

    uint8_t response_payload[1];
    response_payload[0] = status_code;

    int send_status = xbee_handler_send_data_frame(&xbee,
                                       &originating_frame->source_addr_64,
                                       originating_frame->source_addr_16,
                                       response_payload,
                                       sizeof(response_payload),
                                       0,
                                       XBEE_HANDLER_TX_OPT_NONE);
    if (send_status < 0) {
        // Failed to queue the response.
        // This is a non-critical response, so we might not increment global failure counters
        // unless this becomes a persistent issue.
        // printf("Error sending pad status response: %d\r\n", send_status);
    }
}

void PAD_CONTROLLER_SETUP_ROUTINE (uint32_t* board_can_ids, uint8_t numBoards){
  HAL_GPIO_TogglePin(STATUS_IND_GPIO_Port, STATUS_IND_Pin);
  HAL_Delay(500);
  HAL_GPIO_TogglePin(STATUS_IND_GPIO_Port, STATUS_IND_Pin);

  FLASH_ALL (board_can_ids, numBoards);
}

static void process_pad_controller_command(XBeeRxFrame_t* received_xbee_frame) {
    if (received_xbee_frame == NULL || received_xbee_frame->length == 0) {
        return;
    }
    uint8_t command = received_xbee_frame->payload[0];
    HAL_GPIO_TogglePin(STATUS_IND_GPIO_Port, STATUS_IND_Pin);
    switch (command) {
        case SIGNAL_ALL:
            FLASH_ALL (board_can_ids, NUM_BOARDS);
            break;
        case REPORT_ALL:
            break;
        case ACTIVATE_SERVOS:
            servos_activated = 1;
            break;
        case DEACTIVATE_SERVOS:
            servos_activated = 0;
            break;
        case OPEN_NO2:
            if (servos_activated) {
                Update_Ack(&ack_byte_value, 5, 0);
                servo_can_id = GET_SERVO_CAN_ID (no2_board_uid, 1);
                ACTUATE_SERVO(servo_can_id, OPEN_SERVO);
            }
            break;
        case CLOSE_NO2:
            if (servos_activated) {
                Update_Ack(&ack_byte_value, 5, 1);
                servo_can_id = GET_SERVO_CAN_ID (no2_board_uid, 1);
                ACTUATE_SERVO(servo_can_id, CLOSE_SERVO);
            }
            break;
        case OPEN_NO3:
            if (servos_activated) {
                Update_Ack(&ack_byte_value, 4, 0);
                servo_can_id = GET_SERVO_CAN_ID (no3_board_uid, 1);
                ACTUATE_SERVO(servo_can_id, OPEN_SERVO);
            }
            break;
        case CLOSE_NO3:
            if (servos_activated) {
                Update_Ack(&ack_byte_value, 4, 1);
                servo_can_id = GET_SERVO_CAN_ID (no3_board_uid, 1);
                ACTUATE_SERVO(servo_can_id, CLOSE_SERVO);
            }
            break;
        case OPEN_NO4:
            if (servos_activated) {
                Update_Ack(&ack_byte_value, 3, 0);
                servo_can_id = GET_SERVO_CAN_ID (no4_board_uid, 1);
                ACTUATE_SERVO(servo_can_id, OPEN_SERVO);
            }
            break;
        case CLOSE_NO4:
            if (servos_activated) {
                Update_Ack(&ack_byte_value, 3, 1);
                servo_can_id = GET_SERVO_CAN_ID (no4_board_uid, 1);
                ACTUATE_SERVO(servo_can_id, CLOSE_SERVO);
            }
            break;
        case OPEN_PYRO:
            if (servos_activated) {
                Update_Ack(&ack_byte_value, 1, 0);
                servo_can_id = GET_SERVO_CAN_ID (pyro_board_uid, 1);
                ACTUATE_SERVO(servo_can_id, OPEN_SERVO);
            }
            break;
        case CLOSE_PYRO:
            if (servos_activated) {
                Update_Ack(&ack_byte_value, 1, 1);
                servo_can_id = GET_SERVO_CAN_ID (pyro_board_uid, 1);
                ACTUATE_SERVO(servo_can_id, CLOSE_SERVO);
            }
            break;
        default:
            break;
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
  HAL_Init();
  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  SystemClock_Config();
  /* USER CODE BEGIN SysInit */
  // Recommendation: Initialize IWDG here if used
  // IWDG_Init();
  /* USER CODE END SysInit */

  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_TogglePin(STATUS_IND_GPIO_Port, STATUS_IND_Pin);
  HAL_Delay(3000);
  HAL_GPIO_TogglePin(STATUS_IND_GPIO_Port, STATUS_IND_Pin);

  SCB->SHCSR |= SCB_SHCSR_USGFAULTENA_Msk
              |  SCB_SHCSR_BUSFAULTENA_Msk
              |  SCB_SHCSR_MEMFAULTENA_Msk;

  GET_BOARD_UID (board_uid);

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
  HAL_CAN_ConfigFilter(&hcan1, &canfilterconfig);
  HAL_CAN_Start(&hcan1);

  HAL_NVIC_SetPriority(USART6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(USART6_IRQn);

  xbee_platform_init();
  xbee_dev_init(&xbee, xbee_platform_serial(), always_awake, NULL); // 'always_awake' needs to be defined or from xbee_actions.h
  xbee_dev_flowcontrol(&xbee, 0);
  xbee_cmd_init_device(&xbee);
  xbee_dev_tick(&xbee);
  xbee_cmd_tick();

  int status = 0;
//  int xbee_init_status = 0; // Not used
  uint8_t last_command_received = 0xFF;
  do {
      xbee_dev_tick(&xbee);
      xbee_cmd_tick();
      status = xbee_cmd_query_status(&xbee);
  } while (status == -EBUSY); // Note: Digi XBee library uses negative errno for errors. -EBUSY is typical.

  if (status != 0) {
      return status; // Or call Error_Handler()
  }

  xbee_handler_init_rx_queue();
  ack_byte_value = Create_Ack();
  prev_ack_send_time_ms = HAL_GetTick();
  g_periodic_ack_frame_id = 0; // Ensure it's initialized

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
      // Recommendation: Pet the IWDG here if used
      // IWDG_Refresh();

      xbee_dev_tick(&xbee); // Processes XBee library events, including receiving data and TX status
	  xbee_cmd_tick();      // Processes AT command responses
      xbee_handler_service_rx_from_library(); // Moves received data to application queue

      if (xbee_handler_is_rx_frame_available()) {
          if (xbee_handler_rx_frame_dequeue(&current_received_xbee_frame)) {
              if (current_received_xbee_frame.length > 0) {
                  last_command_received = current_received_xbee_frame.payload[0];
              } else {
                  last_command_received = 0xFF;
              }
              process_pad_controller_command(&current_received_xbee_frame);
          }
      }

      Tick_Igniter(last_command_received, &ack_byte_value);
      PadController_Tick(last_command_received, pyro_board_uid, &ack_byte_value);
      Tick_Breakwire_LED();
      FLASH_ALL(board_can_ids, NUM_BOARDS); // This seems to be called very frequently. Ensure it's not blocking.

      current_time_ms = HAL_GetTick();

      // --- Periodic ACK Sending Logic ---
      if (!g_xbee_target_unreachable && (current_time_ms - prev_ack_send_time_ms >= PERIODIC_ACK_INTERVAL_MS)) {
    	  if (g_periodic_ack_frame_id == 0) { // Only send if no previous ACK is awaiting TX status
              tx_ack_payload[0] = ack_byte_value;
              g_periodic_ack_frame_id = xbee_next_frame_id(&xbee); // Get ID before sending

              if (g_periodic_ack_frame_id == 0) { // Frame ID 0 is not used for TX status requests
                  // This could happen if xbee_next_frame_id wraps around and lands on 0.
                  // The library might prevent this, but as a fallback, try getting another one next time.
                  // Or, simply try to send with frame_id = 1 (auto-assign by library) if this is an issue.
                  // For now, we'll skip this attempt if ID is 0, and it will retry next interval.
                  // printf("Failed to get a valid XBee frame ID for ACK. Will retry.\r\n");
              } else {
                  // // printf("Attempting to send periodic ACK with Frame ID: %u\r\n", g_periodic_ack_frame_id);
            	  int send_status = xbee_handler_send_byte_array(&xbee,
            	                                               &g_target_xbee_ieee_addr,
            	                                               g_target_xbee_network_addr,
            	                                               tx_ack_payload,
            	                                               sizeof(tx_ack_payload),
            	                                               g_periodic_ack_frame_id,
            	                                               XBEE_HANDLER_TX_OPT_NONE);
            	  if (send_status < 0) {
            	      // Check if the error is due to local TX buffer being full
            	      // xbee_ser_write now returns -ENOSPC or 0 if buffer full.
            	      // xbee_frame_write might propagate this or return its own error.
            	      // Let's assume -ENOSPC is a specific error for "buffer full from driver".
            	      // The XBee library might also return -EAGAIN.
            	      if (send_status == -ENOSPC || send_status == -EAGAIN) { // Or if xbee_ser_write returned 0 and that was propagated
            	          // Local TX buffer full or XBee library is temporarily busy.
            	          // The XBee library should retry on a subsequent xbee_dev_tick().
            	          // Do not increment g_xbee_tx_fail_count for this.
            	          // g_periodic_ack_frame_id remains set for this pending attempt.
            	          // printf("Periodic ACK: UART TX buffer full or XBee lib busy (Frame ID: %u). Will retry.\r\n", g_periodic_ack_frame_id);
            	      } else {
            	          // Other error from XBee library (e.g., bad parameters, internal issue).
            	          // This attempt to queue has failed more definitively.
            	          // printf("Error queuing periodic ACK: %d (Frame ID: %u).\r\n", send_status, g_periodic_ack_frame_id);
            	          g_xbee_tx_fail_count++;
            	          g_periodic_ack_frame_id = 0; // Reset to allow a new ACK attempt next interval.
            	      }
            	  } else {
            	      // Successfully queued with XBee library (and should be making its way to the non-blocking UART driver).
            	      g_ack_pending_timestamp = current_time_ms;
            	      // printf("Periodic ACK successfully queued with Frame ID: %u.\r\n", g_periodic_ack_frame_id);
            	  }
            	  prev_ack_send_time_ms = current_time_ms; // Update time for the next interval check
              }
          } else {
              // A previous ACK (g_periodic_ack_frame_id != 0) is still awaiting TX status.
              // Do not send another one yet. Check for timeout on this pending ACK.
              if (current_time_ms - g_ack_pending_timestamp > ACK_PENDING_TIMEOUT_MS) {
                  // printf("Timeout waiting for TX status for ACK Frame ID: %u. Counting as failure.\r\n", g_periodic_ack_frame_id);
                  g_periodic_ack_frame_id = 0; // Give up on this ACK
                  g_xbee_tx_fail_count++;
                  prev_ack_send_time_ms = current_time_ms; // Allow new ACK attempt in next interval
              }
          }
      }

      // --- Check for Max Consecutive Failures ---
      // This check is now driven by both queuing failures and TX status failures (via g_xbee_tx_fail_count)
      if (g_xbee_tx_fail_count >= MAX_CONSECUTIVE_XBEE_TX_FAILS && !g_xbee_target_unreachable) {
          // // printf("Max TX fail count (%lu) reached. Marking target unreachable for %u ms.\r\n", g_xbee_tx_fail_count, XBEE_RETRY_PERIOD_MS);
          g_xbee_target_unreachable = true;
          g_periodic_ack_frame_id = 0; // Clear any pending ACK ID as we are stopping transmissions
          // The unreachable_timestamp logic below will handle the start of the backoff period.
      }

      // --- Target Unreachable / Retry Logic ---
      if (g_xbee_target_unreachable) {
          static uint32_t unreachable_timestamp = 0;
          if (unreachable_timestamp == 0) {
              unreachable_timestamp = HAL_GetTick();
              // // printf("Target marked unreachable. Starting backoff period.\r\n");
          }
          if (HAL_GetTick() - unreachable_timestamp > XBEE_RETRY_PERIOD_MS) {
              // printf("XBee retry period elapsed. Attempting to resend to target.\r\n");
              g_xbee_target_unreachable = false;
              g_xbee_tx_fail_count = 0;
              unreachable_timestamp = 0;
              prev_ack_send_time_ms = HAL_GetTick() - PERIODIC_ACK_INTERVAL_MS; // Force immediate ACK attempt
          }
      }
      /* USER CODE END 3 */
    }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

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
}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{
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
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(STATUS_IND_GPIO_Port, STATUS_IND_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, BRK_CONT_LED_SINK_Pin|BRK_CONT_LED_Pin|IGNITER_Pin|BRK_CONT_SINK_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = STATUS_IND_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(STATUS_IND_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = IGNITER_CONT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IGNITER_CONT_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = BRK_CONT_LED_SINK_Pin|BRK_CONT_LED_Pin|IGNITER_Pin|BRK_CONT_SINK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = BRK_CONT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BRK_CONT_GPIO_Port, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rxHeader;
    uint8_t rxData[8];
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK)
    {
    }
}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
{
//    STATUS_IND_Toggle();
}
/* USER CODE END 4 */

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
    HAL_GPIO_TogglePin(STATUS_IND_GPIO_Port, STATUS_IND_Pin);
    HAL_Delay(100); // Fast blink for error
  }
}

#ifdef  USE_FULL_ASSERT
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

