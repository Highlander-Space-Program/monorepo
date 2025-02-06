/*
 * can_utils.h
 *
 *  Created on: Nov 20, 2024
 *      Author: zande
 */

#ifndef INC_CAN_UTILS_H_
#define INC_CAN_UTILS_H_

#include "stm32f0xx_hal.h"

#define TX_ID 0x440


HAL_StatusTypeDef send_can_msg(const uint8_t *data, size_t len, CAN_HandleTypeDef hcan) {
    CAN_TxHeaderTypeDef header;
    header.IDE = CAN_ID_STD;
    header.StdId = TX_ID;
    header.RTR = CAN_RTR_DATA;
    header.TransmitGlobalTime = DISABLE;
    header.DLC = len;

    uint32_t mailbox;

    HAL_StatusTypeDef status = HAL_CAN_AddTxMessage(&hcan, &header, data, &mailbox);
    if (status != HAL_OK) {

    }

    return status;
}

#endif /* INC_CAN_UTILS_H_ */
