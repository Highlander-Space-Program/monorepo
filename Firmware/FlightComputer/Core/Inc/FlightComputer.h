/*
 * FlightComputer.h
 *
 *  Created on: May 24, 2025
 *      Author: brandonmarcus
 */

#ifndef INC_FLIGHTCOMPUTER_H_
#define INC_FLIGHTCOMPUTER_H_

#define VALVE_DELAY_MS 2000
#define VALVE_FIRE_MS 3000
#define FLASH_FREQUENCY_MS 1000
#define CAN_SEND_INTERVAL_MS 50

static uint32_t last_can_message_dispatch_time = 0;
static uint32_t prev_flash_trigger_time = 0;
static int8_t g_status_all_current_board_idx = -1; // -1: inactive; 0 to N-1: current board index
static uint32_t g_status_all_last_internal_send_time_ms = 0; // Tracks STATUS_ALL's own pacing

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



#endif /* INC_FLIGHTCOMPUTER_H_ */
