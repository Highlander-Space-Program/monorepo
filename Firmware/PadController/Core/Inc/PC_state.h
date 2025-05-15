#ifndef INC_PC_STATE_H_
#define INC_PC_STATE_H_

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"

#include "main.h"
#include "PC_state.h"
#include "breakwire.h"
#include "config/config.h"
#include "config/igniter_config.h"
#include "utils/can_utils.h"
#include "utils/servo_utils.h"
#include "utils/radio_utils.h"

//testing for debug
//#define AUTO_OFF   0x00
//#define AUTO_ON    0x01
// how long to wait after the wire breaks before opening the valve
#define VALVE_DELAY_MS 2000
#define VALVE_FIRE_MS 3000
#define FLASH_FREQUENCY_MS 1000
#define CAN_SEND_INTERVAL_MS 50

// pad-controller states
typedef enum
{
    PC_STARTUP = 0, // immediately becomes AUTO_OFF
    PC_AUTO_OFF = 1,    // waiting for AUTO_ON + BW intact
    PC_AUTO_ON = 2,     // AUTO switch on, wire intact
    PC_DELAY = 3,       // wire broke, delaying before valve
	PC_FIRE = 4,		// spam CAN command
    PC_OPEN = 5         // valve has been opened
} PadControllerState_t;

bool            isCloseAll   = false;
bool            isAborted    = false;
bool            isStarted    = false;
IgniterState_t  igniterState = IGNITER_INIT;

//break-wire armed flag
bool isAutoArmed = false;

//state-machine globals
static uint32_t last_can_message_dispatch_time = 0;
static uint32_t prev_flash_trigger_time = 0;
static int8_t g_status_all_current_board_idx = -1; // -1: inactive; 0 to N-1: current board index
static uint32_t g_status_all_last_internal_send_time_ms = 0; // Tracks STATUS_ALL's own pacing

PadControllerState_t pcState   = PC_STARTUP;
uint32_t             delayStart = 0;
uint32_t             fireStart = 0;
uint32_t 			 prev_flash = 0;

#define LENGTH 8
uint8_t data[LENGTH];

extern CAN_HandleTypeDef hcan1;
extern UART_HandleTypeDef huart6;
extern bool servos_activated;
// send a flash signal throguh all the boards
void FLASH_ALL (uint32_t* board_can_ids, uint8_t numBoards) {
  uint8_t short_board_id;
  uint32_t* board_uid;
  uint32_t current_tick; // To store current time from HAL_GetTick()

  // Get current time for the outer 1000ms check
  current_tick = HAL_GetTick();

  // Check if it's time to run the main flashing sequence (every FLASH_FREQUENCY_MS)
  if (current_tick - prev_flash_trigger_time >= FLASH_FREQUENCY_MS) {
    prev_flash_trigger_time = current_tick; // Update timestamp for this flash sequence activation

    // Determine the number of boards to flash, based on original logic
    // minus one because we dont want the pad controller itself.
    int num_boards_to_process = GET_NUM_BOARD_CONFIGS() - 1;

    for (int i = 0; i < num_boards_to_process; i++) {
      // Get current time before checking the 50ms CAN send interval
      current_tick = HAL_GetTick();

      // Ensure 50ms has passed since the last CAN message was dispatched by this function.
      // The 'last_can_message_dispatch_time != 0' check ensures that the very first message
      // (or the first after a long pause/reset) isn't unnecessarily delayed.
      if (last_can_message_dispatch_time != 0) {
        uint32_t time_since_last_dispatch = current_tick - last_can_message_dispatch_time;
        if (time_since_last_dispatch < CAN_SEND_INTERVAL_MS) {
          // Not enough time has passed, so wait for the remainder of the interval
          HAL_Delay(CAN_SEND_INTERVAL_MS - time_since_last_dispatch);
        }
      }

      // Update the dispatch time for the current message *before* actually sending it.
      // This marks the beginning of the 50ms slot for this message.
      last_can_message_dispatch_time = HAL_GetTick();

      // Prepare CAN message details

//      board_uid = GET_BOARD_UID_FROM_CAN_ID (board_can_ids[i]);
//      short_board_id = GET_SHORT_BOARD_ID (board_uid);
      short_board_id = GET_SHORT_BOARD_ID_FROM_CAN_ID(board_can_ids[i]);
      uint32_t ext_id = build_can_extended_id (SENDER_PAD_CONTROLLER, short_board_id, MSG_TYPE_FLASH_SIGNAL, 0x00);

      // Send the CAN message
      // Ensure 'data' and 'LENGTH' are defined and accessible in this scope.
      // These would be your actual CAN payload and its length.
      HAL_StatusTypeDef status = send_can_msg(ext_id, data, 0, &hcan1);

      if (status != HAL_OK) {
        // Indicate error, e.g., toggle an LED
        HAL_GPIO_TogglePin(STATUS_IND_GPIO_Port, STATUS_IND_Pin);
      }
    }
  }
}

bool STATUS_ALL_Start(void) {
    // Critical section for checking/setting g_status_all_current_board_idx might be needed
    // if STATUS_ALL_Start could be called from an ISR and main loop, but here it seems
    // it's called from process_pad_controller_command which is in main context.
    // HAL_NVIC_DisableIRQ(relevant_irq); // Example if critical section needed
    if (g_status_all_current_board_idx != -1) {
        // A sequence is already in progress. Ignore this new request.
        // Optionally log this event: printf("STATUS_ALL_Start: Sequence busy, new request ignored.\n");
        // HAL_NVIC_EnableIRQ(relevant_irq); // End critical section
        return false; // Indicate busy / request ignored
    }

    int num_boards_to_target = GET_NUM_BOARD_CONFIGS() - 1;
    if (num_boards_to_target <= 0) {
        // No boards to process, or configuration is such that only the controller itself exists.
        // Mark as inactive. Not strictly "busy", but nothing to "start".
        g_status_all_current_board_idx = -1;
        // HAL_NVIC_EnableIRQ(relevant_irq); // End critical section
        return true; // System is idle, though no sequence will run.
    }

    g_status_all_current_board_idx = 0; // Start with the first board
    // Allow the first message to be sent promptly by STATUS_ALL_Process's internal timer,
    // the shared 'last_can_message_dispatch_time' will still enforce overall bus coordination.
    g_status_all_last_internal_send_time_ms = HAL_GetTick() - CAN_SEND_INTERVAL_MS;
    // HAL_NVIC_EnableIRQ(relevant_irq); // End critical section
    return true; // Indicate sequence successfully initiated
}

/**
 * @brief Processes the sending of status requests non-blockingly.
 * Should be called periodically from the main loop.
 * @param board_can_ids_array Pointer to the array of board CAN IDs.
 */
void STATUS_ALL_Process(uint32_t* board_can_ids_array) {
    if (g_status_all_current_board_idx < 0) {
        return; // Sequence is not active
    }

    int num_boards_to_target = GET_NUM_BOARD_CONFIGS() - 1; // As per FLASH_ALL logic

    // Basic check: if somehow index is out of bounds or no boards, deactivate.
    if (num_boards_to_target <= 0 || g_status_all_current_board_idx >= num_boards_to_target) {
        g_status_all_current_board_idx = -1; // All boards processed or no boards
        return;
    }

    uint32_t current_tick = HAL_GetTick();

    // Check 1: STATUS_ALL's own internal pacing (50ms since its last send)
    if (current_tick - g_status_all_last_internal_send_time_ms >= CAN_SEND_INTERVAL_MS) {

        // Check 2: Shared CAN bus coordination (50ms since ANY relevant CAN send)
        // The 'last_can_message_dispatch_time != 0' check is mostly for the very first send ever.
        // If last_can_message_dispatch_time is 0, it means no CAN message has been sent via this mechanism yet.
        if (last_can_message_dispatch_time != 0) { // last_can_message_dispatch_time is the global static from PC_state.h
            uint32_t time_since_global_last_dispatch = current_tick - last_can_message_dispatch_time;
            if (time_since_global_last_dispatch < CAN_SEND_INTERVAL_MS) {
                // Bus busy due to another function (e.g., FLASH_ALL) using it recently.
                // Defer send; STATUS_ALL_Process will try again on its next call from main loop.
                return;
            }
        }

        // If both checks pass, it's time to send.
        uint8_t short_board_id = GET_SHORT_BOARD_ID_FROM_CAN_ID(board_can_ids_array[g_status_all_current_board_idx]);
        uint32_t ext_id = build_can_extended_id(SENDER_PAD_CONTROLLER, short_board_id, MSG_TYPE_BOARD_STATUS_RESPONSE, 0x00);

        // Assuming 'data' and 'LENGTH' are accessible (e.g., global or passed in)
        // HAL_StatusTypeDef status = send_can_msg(ext_id, data, LENGTH, &hcan1); // Original uses global `data` and `LENGTH`

        // Using the global `data` and `LENGTH` defined in your PC_state.h
        extern uint8_t data[]; // If data is global
        extern CAN_HandleTypeDef hcan1; // from main.c
        #define LENGTH 8 // Ensure this is defined or accessible

        HAL_StatusTypeDef status = send_can_msg(ext_id, data, LENGTH, &hcan1);


        if (status != HAL_OK) {
            // Consider a more robust error indication if HAL_GPIO_TogglePin is too simple or used by other things
            HAL_GPIO_TogglePin(STATUS_IND_GPIO_Port, STATUS_IND_Pin);
        }

        // Update timestamps
        g_status_all_last_internal_send_time_ms = current_tick; // Mark this send time for STATUS_ALL's pacing
        last_can_message_dispatch_time = current_tick;          // Update the shared global timer

        g_status_all_current_board_idx++; // Move to the next board

        if (g_status_all_current_board_idx >= num_boards_to_target) {
            g_status_all_current_board_idx = -1; // Sequence finished
        }
    }
}

void send_pc_state_can(PadControllerState_t state) {
    uint8_t status_byte = (uint8_t)state; // Send the raw enum value
    send_pad_controller_status_can(MSG_TYPE_PC_STATE_STATUS, status_byte);
}

//PadController_Tick
void PadController_Tick(uint8_t  cmd,
                        uint32_t *pyroBoardUid,
                        uint8_t  *ackPtr)
{
//    uint32_t canId = build_can_extended_id(1 /*SENDER_PAD_CONTROLLER*/,
//                                           GET_SHORT_BOARD_ID(pyroBoardUid),
//                                           1 /*MSG_TYPE_SERVO*/,
//                                           pyroInstance);
    uint32_t canId = GET_SERVO_CAN_ID (pyroBoardUid, 1);


    //fix ack, with igniter code.
    switch (pcState)
    {
    case PC_STARTUP:
        pcState = PC_AUTO_OFF;
    	HAL_GPIO_WritePin(BRK_CONT_LED_SINK_GPIO_Port, BRK_CONT_LED_SINK_Pin, GPIO_PIN_RESET);
    	HAL_GPIO_WritePin(BRK_CONT_SINK_GPIO_Port, BRK_CONT_SINK_Pin, GPIO_PIN_RESET);
        break;

    case PC_AUTO_OFF:
        if (cmd == AUTO_ON && Check_Breakwire() == GPIO_PIN_RESET)
        {
            pcState     = PC_AUTO_ON;
            isAutoArmed = true;
        }
        break;

    case PC_AUTO_ON:
        if (cmd == AUTO_OFF)
        {
            pcState     = PC_AUTO_OFF;
            isAutoArmed = false;
        }
        else if (Check_Breakwire() == GPIO_PIN_SET)
        {
            pcState    = PC_DELAY;
            delayStart = HAL_GetTick();
        }
        break;

    case PC_DELAY:
        if (HAL_GetTick() - delayStart >= VALVE_DELAY_MS)
        {
            if (servos_activated){
            	pcState = PC_FIRE;
                Update_Ack(&ack, 1, 0);
                fireStart = HAL_GetTick();
            }
        }
        break;

    case PC_FIRE:
		 ACTUATE_SERVO(canId, OPEN_SERVO);
		 pcState = PC_STARTUP;

    case PC_OPEN:
        /* stay here until power-cycle */
    	//blink control box led
        break;

    default:
        pcState = PC_AUTO_OFF;
        break;
    }
}


#endif // PAD_CONTROLLER_H
