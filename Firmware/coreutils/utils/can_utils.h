/*
 * can_utils.h
 *
 *  Created on: Nov 20, 2024
 *      Author: zande
 */

#ifndef INC_CAN_UTILS_H_
#define INC_CAN_UTILS_H_

#if defined(STM32F0)
    #include "stm32f0xx_hal.h"
#elif defined(STM32F405) || defined(STM32F4)
    #include "stm32f4xx_hal.h"
#else
    #error "STM32 family not defined or not supported!"
#endif

// CAN Extended ID Shift
#define EXT_ID_SENDER_SHIFT      21
#define EXT_ID_BOARD_ID_SHIFT    13
#define EXT_ID_COMP_TYPE_SHIFT   5
#define EXT_ID_INSTANCE_MASK     0x1F

#define CAN_ID_ACK_FLAG_29BIT (1UL << 28) // 0x10000000 (Bit 28)

#define LENGTH 8

uint8_t data[LENGTH];

HAL_StatusTypeDef send_can_msg(const uint32_t extId, const uint8_t *data, const size_t len, CAN_HandleTypeDef *hcan) {
    CAN_TxHeaderTypeDef header;
    header.IDE = CAN_ID_EXT;
    header.ExtId = extId >> 3;
    header.RTR = CAN_RTR_DATA;
    header.TransmitGlobalTime = DISABLE;
    header.DLC = len;
    uint32_t mailbox;
    return HAL_CAN_AddTxMessage(hcan, &header, data, &mailbox);
}

// returns the shifted extended ID
uint32_t build_can_extended_id(uint8_t sender, uint8_t boardId, uint8_t msgType, uint8_t instance) {
    uint32_t extId = 0;
    extId |= ((uint32_t)sender << EXT_ID_SENDER_SHIFT);
    extId |= ((uint32_t)boardId << EXT_ID_BOARD_ID_SHIFT);
    extId |= ((uint32_t)msgType << EXT_ID_COMP_TYPE_SHIFT);
    extId |= ((uint32_t)instance & EXT_ID_INSTANCE_MASK);
    extId = extId << 3;
    return extId;
}

void parse_can_extended_id(const uint32_t extId, uint8_t *sender, uint8_t *boardId,
                        uint8_t *msgType, uint8_t *instance) {
    *sender = (extId >> EXT_ID_SENDER_SHIFT) & 0xFF;
    *boardId = (extId >> EXT_ID_BOARD_ID_SHIFT) & 0xFF;
    *msgType = (extId >> EXT_ID_COMP_TYPE_SHIFT) & 0xFF;
    *instance = extId & EXT_ID_INSTANCE_MASK;
}

HAL_StatusTypeDef send_can_ack(const uint32_t received_ext_id_29bit, const uint8_t *ack_data, const size_t ack_len, CAN_HandleTypeDef *hcan)
{
    // 1. Create the 29-bit ACK ID by setting the ACK flag (MSB) on the received ID
    uint32_t ack_ext_id_29bit = received_ext_id_29bit | CAN_ID_ACK_FLAG_29BIT;

    // 2. Prepare the 32-bit shifted ID required by send_can_msg
    //    (Shift the 29-bit ACK ID left by 3)
    uint32_t ack_ext_id_32bit_shifted = ack_ext_id_29bit << 3;

    // 3. Call the existing send function with the 32-bit shifted ACK ID
    return send_can_msg(ack_ext_id_32bit_shifted, ack_data, ack_len, hcan);
}

#endif /* INC_CAN_UTILS_H_ */
