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
#define NUM_BOARDS 6
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
//#define TARGET_XBEE_ADDR_64_B0 0x00 // Example byte 0 (MSB)
//#define TARGET_XBEE_ADDR_64_B1 0x13 // Example byte 1
//#define TARGET_XBEE_ADDR_64_B2 0xA2 // Example byte 2
//#define TARGET_XBEE_ADDR_64_B3 0x00 // Example byte 3
//#define TARGET_XBEE_ADDR_64_B4 0x41 // Example byte 4
//#define TARGET_XBEE_ADDR_64_B5 0xB3 // Example byte 5
//#define TARGET_XBEE_ADDR_64_B6 0xF9 // Example byte 6
//#define TARGET_XBEE_ADDR_64_B7 0xC6 // Example byte 7 (LSB)

// Target XBee 2 (Active)
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
volatile bool_t g_request_uart6_reinit = false;

static xbee_dev_t   xbee;

uint32_t board_uid[3];
bool servos_activated = 0;

uint32_t* no2_board_uid;
uint32_t* no3_board_uid;
uint32_t* no4_board_uid;
uint32_t* pyro_board_uid;
uint32_t* pt01_board_uid;
uint32_t* pt03_board_uid;

uint32_t no2_can_id;
uint32_t no3_can_id;
uint32_t no4_can_id;
uint32_t pyro_can_id;
uint32_t pt01_can_id;
uint32_t pt03_can_id;

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
        case CMD_BOARD_STATUS_REQUEST:
        	Report_All_PadController_States();
        	STATUS_ALL_Start();
        	break;
        case MSG_TYPE_RESET:
        	NVIC_SystemReset();
        default:
            break;
    }
}

void send_pad_controller_status_can(uint8_t component_type, uint8_t status_byte) {
    uint8_t instance = 0; // Usually instance 0 for these statuses
    uint32_t can_id_29bit;
    uint8_t xbee_payload[4 + 1]; // 4 bytes ID + 1 byte data
    size_t xbee_payload_len = 5;
    uint32_t can_id_for_xbee_32bit;

    // 1. Build the 32-bit CAN ID
    // Sender=PadCtrl(1), Board=ControlPanel(5), CompType=variable, Inst=0
    can_id_for_xbee_32bit = build_can_extended_id(
		short_board_id,
		SENDER_CONTROL_PANEL,
        component_type,
        instance
    );

//    // 2. Convert to 32-bit shifted format for XBee payload
//    can_id_for_xbee_32bit = can_id_29bit << 3;

    // 3. Populate XBee payload buffer (Big Endian)
    xbee_payload[0] = (uint8_t)((can_id_for_xbee_32bit >> 24) & 0xFF);
    xbee_payload[1] = (uint8_t)((can_id_for_xbee_32bit >> 16) & 0xFF);
    xbee_payload[2] = (uint8_t)((can_id_for_xbee_32bit >> 8) & 0xFF);
    xbee_payload[3] = (uint8_t)(can_id_for_xbee_32bit & 0xFF);
    xbee_payload[4] = status_byte; // Data payload

    // 4. Send via XBee
    // Use Frame ID 0 for no specific TX status needed for broadcasts
    int send_status = xbee_handler_send_byte_array(
        &xbee,
        &g_target_xbee_ieee_addr,
        g_target_xbee_network_addr,
        xbee_payload,
        xbee_payload_len,
        0, // Frame ID 0
        0  // Options
    );

    if (send_status < 0) {
        // Log XBee send error (implement your logging)
        // printf("Error sending CAN status type %d via XBee: %d\n", component_type, send_status);
    } else {
        // Log success if needed
        // printf("Sent CAN status type %d via XBee.\n", component_type);
    }
}

void send_servos_power_status_can(bool is_on) {
    uint8_t status_byte = is_on ? 1 : 0;
    send_pad_controller_status_can(MSG_TYPE_SERVOS_POWER_STATUS, status_byte);
}

void Report_All_PadController_States(void) {
    // Call each individual sender function
    send_breakwire_status_can(isAutoArmed);
    send_igniter_status_can(igniterState);
    // Determine 'auto mode on' based on pcState
    send_auto_mode_status_can(pcState == PC_AUTO_ON || pcState == PC_DELAY || pcState == PC_FIRE || pcState == PC_OPEN);
    send_servos_power_status_can(servos_activated);
    send_pc_state_can(pcState);

    // TODO: Add calls to report status for other components managed by Pad Controller if any
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
  // Recommendation: Initialize Independent Watchdog (IWDG) here if used
  // MX_IWDG_Init(); // Assuming you have this function if IWDG is enabled in CubeMX
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_USART6_UART_Init(); /* This initializes huart6 */
  /* USER CODE BEGIN 2 */
  HAL_GPIO_TogglePin(STATUS_IND_GPIO_Port, STATUS_IND_Pin);
  HAL_Delay(3000); // Initial status indication
  HAL_GPIO_TogglePin(STATUS_IND_GPIO_Port, STATUS_IND_Pin);

  // Enable UsageFault, BusFault, and MemManage Faults
  SCB->SHCSR |= SCB_SHCSR_USGFAULTENA_Msk
              |  SCB_SHCSR_BUSFAULTENA_Msk
              |  SCB_SHCSR_MEMFAULTENA_Msk;

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

  // --- UART Interrupt Priority for XBee ---
  // Ensure USART6 interrupt is enabled with appropriate priority
  // This is crucial for timely processing of RX data and errors.
  HAL_NVIC_SetPriority(USART6_IRQn, 5, 0); // Priority 5, Subpriority 0
  HAL_NVIC_EnableIRQ(USART6_IRQn);

  HAL_Delay(10); // Small delay before XBee initialization

  // --- Initial XBee Platform and Device Initialization ---
  xbee_platform_init(); // This calls xbee_ser_open for the first time for huart6

  // Initialize the XBee device structure
  // The 'always_awake' variable is defined in USER CODE BEGIN PV
  xbee_dev_init(&xbee, xbee_platform_serial(), (bool_t)always_awake, NULL);
  xbee_dev_flowcontrol(&xbee, 0); // Assuming no hardware flow control (0 = disabled)
  xbee_cmd_init_device(&xbee);    // Initialize AT command processor for this device

  // Initial check for XBee module readiness
  int status = 0;
  uint32_t init_start_time = HAL_GetTick();
  do {
      xbee_dev_tick(&xbee); // Allow XBee library to process incoming/outgoing data
      xbee_cmd_tick();      // Process AT command responses
      status = xbee_cmd_query_status(&xbee); // Query basic XBee status
      if ((HAL_GetTick() - init_start_time) > 5000) { // 5-second timeout
          // printf("Timeout waiting for initial XBee query status.\r\n"); // Requires printf retargeting
          Error_Handler(); // Or handle appropriately (e.g., log error, retry)
          break;
      }
  } while (status == -EBUSY); // -EBUSY is a typical "busy" response from XBee lib

  if (status != 0) {
      // printf("Initial XBee query status failed: %d\r\n", status);
      Error_Handler(); // Or handle appropriately
  }

  xbee_handler_init_rx_queue(); // Initialize your application's RX queue for XBee frames
  ack_byte_value = Create_Ack();
  prev_ack_send_time_ms = HAL_GetTick();
  g_periodic_ack_frame_id = 0; // Ensure it's initialized

  // --- Board and CAN ID Setup ---
  no2_board_uid = GET_BOARD_ID_FROM_PNID ("FV-N02");
  no3_board_uid = GET_BOARD_ID_FROM_PNID ("FV-N03");
  no4_board_uid = GET_BOARD_ID_FROM_PNID ("FV-N04");
  pyro_board_uid = GET_BOARD_ID_FROM_PNID ("FV-PYRO");
  pt01_board_uid = GET_BOARD_ID_FROM_PNID ("PT-01");
  pt03_board_uid = GET_BOARD_ID_FROM_PNID ("PT-03");

  no2_can_id = GET_CAN_ID_FROM_BOARD_UID (no2_board_uid);
  no3_can_id = GET_CAN_ID_FROM_BOARD_UID (no3_board_uid);
  no4_can_id = GET_CAN_ID_FROM_BOARD_UID (no4_board_uid);
  pyro_can_id = GET_CAN_ID_FROM_BOARD_UID (pyro_board_uid);
  pt01_can_id = GET_CAN_ID_FROM_BOARD_UID (pt01_board_uid);
  pt03_can_id = GET_CAN_ID_FROM_BOARD_UID (pt03_board_uid);

  board_can_ids[0] = no2_can_id;
  board_can_ids[1] = no3_can_id;
  board_can_ids[2] = no4_can_id;
  board_can_ids[3] = pyro_can_id;
  board_can_ids[4] = pt01_can_id;
  board_can_ids[5] = pt03_can_id;

  PAD_CONTROLLER_SETUP_ROUTINE (board_can_ids, NUM_BOARDS);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
    // Recommendation: Pet the Independent Watchdog here if used
    // HAL_IWDG_Refresh(&hiwdg); // Assuming hiwdg is your IWDG handle

    // --- UART/XBee Re-initialization Check (Robust Recovery) ---
//    if (g_request_uart6_reinit)
//    {
//      // Atomically clear the flag to prevent re-entry if another interrupt occurs during this block
//      uint32_t primask_status = __get_PRIMASK(); // Store current global interrupt state
//      __disable_irq();                          // Disable global interrupts
//      g_request_uart6_reinit = false;           // Clear the flag
//      __set_PRIMASK(primask_status);            // Restore global interrupt state
//
//      // Optional: Log that a re-initialization is occurring
//      // printf("Attempting UART6 and XBee re-initialization due to error...\r\n");
//
//      xbee_serial_t *xbee_uart_port = xbee_platform_serial();
//      if (xbee_uart_port)
//      {
//        uint32_t current_baudrate = xbee_uart_port->baudrate; // Preserve current baudrate
//
//        // 1. Close the low-level serial port (calls HAL_UART_DeInit, flushes platform buffers)
//        xbee_ser_close(xbee_uart_port);
//        HAL_Delay(50); // Brief delay for peripheral to settle if necessary
//
//        // 2. Re-open the low-level serial port
//        // (calls HAL_UART_Init, resets platform buffers, starts HAL_UART_Receive_IT)
//        if (xbee_ser_open(xbee_uart_port, current_baudrate) != 0)
//        {
//          // printf("FATAL: UART6 re-open (xbee_ser_open) failed during recovery!\r\n");
//          Error_Handler(); // This is a critical failure if re-open fails
//        }
//
//        // 3. Re-initialize the XBee device context with the re-opened serial port
//        xbee_dev_init(&xbee, xbee_uart_port, (bool_t)always_awake, NULL);
//
//        // 4. Re-run essential XBee post-init steps
//        xbee_dev_flowcontrol(&xbee, 0); // Re-apply flow control setting
//        xbee_cmd_init_device(&xbee);    // Re-initialize AT command processor
//
//        // 5. Verify XBee module readiness again
//        int status_reinit = 0;
//        uint32_t reinit_loop_start_time = HAL_GetTick();
//        do {
//            xbee_dev_tick(&xbee); // Allow XBee library to process
//            xbee_cmd_tick();      // Allow AT command processor to work
//            status_reinit = xbee_cmd_query_status(&xbee);
//            if ((HAL_GetTick() - reinit_loop_start_time) > 5000) { // 5-second timeout
//                // printf("Timeout waiting for XBee query status after re-init.\r\n");
//                // Error_Handler(); // Or decide on less drastic action for repeated failures
//                break;
//            }
//        } while (status_reinit == -EBUSY);
//
//        if (status_reinit != 0) {
//            // printf("XBee query status failed after re-init: %d\r\n", status_reinit);
//            // The g_request_uart6_reinit flag is false, so it won't loop here indefinitely.
//            // The next UART error might trigger this recovery again.
//            // Consider more robust error counting or alternative recovery if this happens frequently.
//        } else {
//            // printf("UART6 and XBee re-initialized successfully after error.\r\n");
//        }
//
//        // 6. Re-initialize application-level XBee handlers and state
//        xbee_handler_init_rx_queue(); // Reset your application's RX queue
//
//        // Reset any other relevant application state related to XBee communication
//        uint8_t last_command_received_after_reinit = 0xFF; // Local var for clarity
//        last_command_received = last_command_received_after_reinit; // Reset last command state
//        ack_byte_value = Create_Ack(); // Recreate ACK if needed
//        prev_ack_send_time_ms = HAL_GetTick(); // Reset ACK timing
//        // g_periodic_ack_frame_id = 0; // Reset if this is part of your XBee state
//
//      } // end if (xbee_uart_port)
//      else
//      {
//        // This should ideally never happen if xbee_platform_serial() is robust
//        // printf("FATAL: xbee_platform_serial() returned NULL during recovery!\r\n");
//        Error_Handler();
//      }
//    } // end if (g_request_uart6_reinit)

    // --- Regular XBee Processing and Application Logic ---
    xbee_dev_tick(&xbee); // Processes XBee library events, RX data, TX status
	xbee_cmd_tick();      // Processes AT command responses
    xbee_handler_service_rx_from_library(); // Moves received data to application queue

    if (xbee_handler_is_rx_frame_available()) {
        if (xbee_handler_rx_frame_dequeue(&current_received_xbee_frame)) {
            if (current_received_xbee_frame.length > 0) {
                // Assuming payload[0] is the command byte
                last_command_received = current_received_xbee_frame.payload[0];
            } else {
                last_command_received = 0xFF; // Indicate no valid command or empty frame
            }
            process_pad_controller_command(&current_received_xbee_frame);
        }
    }

    Tick_Igniter(last_command_received, &ack_byte_value);
    PadController_Tick(last_command_received, pyro_board_uid, &ack_byte_value);
    Tick_Breakwire_LED();

    current_time_ms = HAL_GetTick(); // Update current time
    FLASH_ALL (board_can_ids, NUM_BOARDS);
    STATUS_ALL_Process(board_can_ids); // Corrected function name based on your snippet
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
  huart6.Init.BaudRate = 38400;
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

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
{
//    STATUS_IND_Toggle();
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
  // printf("Wrong parameters value: file %s on line %lu\r\n", (char *)file, line);
  Error_Handler();
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
