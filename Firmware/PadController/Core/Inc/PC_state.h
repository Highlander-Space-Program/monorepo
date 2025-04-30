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
PadControllerState_t pcState   = PC_STARTUP;
uint32_t             delayStart = 0;
uint32_t             fireStart = 0;

#define LENGTH 8
uint8_t data[LENGTH];

extern CAN_HandleTypeDef hcan1;
extern UART_HandleTypeDef huart6;
extern bool servos_activated;
// send a flash signal throguh all the boards
void FLASH_ALL (uint32_t* board_can_ids, uint8_t numBoards) {
  uint8_t short_board_id;
  uint32_t* board_uid;

  //0x01020600
  for (int i = 0; i < GET_NUM_BOARD_CONFIGS(); i++) {
	board_uid = GET_BOARD_UID_FROM_CAN_ID (board_can_ids[i]);
	short_board_id = GET_SHORT_BOARD_ID (board_uid);
	uint32_t ext_id = build_can_extended_id (SENDER_PAD_CONTROLLER, short_board_id, MSG_TYPE_FLASH_SIGNAL, 0x00);
	HAL_StatusTypeDef status = send_can_msg(ext_id, data, LENGTH, &hcan1);
	if (status != HAL_OK) {
		HAL_GPIO_TogglePin(STATUS_IND_GPIO_Port, STATUS_IND_Pin); // Indicate error
	}
  }
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
    	if (HAL_GetTick() - fireStart >= VALVE_FIRE_MS){
    		// pcState = PC_OPEN;

			// ** DEBUG ** //
			 pcState = PC_STARTUP;
			// ** DEBUG ** //
		}

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
