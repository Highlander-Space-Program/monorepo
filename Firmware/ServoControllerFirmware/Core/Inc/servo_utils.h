/*
 * servo_utils.h
 *
 *  Created on: Nov 20, 2024
 *      Author: zande
 */

#ifndef INC_SERVO_UTILS_H_
#define INC_SERVO_UTILS_H_

#include "stm32f0xx_hal.h"
#include "main.h"

#define HSP_SERVO_MIN_PULSE_WIDTH 500
#define HSP_SERVO_MAX_PULSE_WIDTH 2500
#define HSP_SERVO_PWM_PERIOD 20000
#define HSP_SERVO_MAX_DEG 270

void Turn_On_Servo(void);
void Turn_Off_Servo(void);
uint32_t Deg_To_CCR(uint8_t deg, const TIM_HandleTypeDef *timer);
void Actuate_Servo(uint8_t deg, const TIM_HandleTypeDef *htim);
void Test_Actuate_Servo(const TIM_HandleTypeDef *htim);
bool Check_Servo_Cont(void);

void Turn_On_Servo() {
	HAL_GPIO_WritePin(SERVO_EN_GPIO_Port, SERVO_EN_Pin, GPIO_PIN_SET);
}

void Turn_Off_Servo() {
	HAL_GPIO_WritePin(SERVO_EN_GPIO_Port, SERVO_EN_Pin, GPIO_PIN_RESET);
}


uint32_t Deg_To_CCR(uint8_t deg, const TIM_HandleTypeDef *timer) {
	// Converts deg to the required CCR for timer to achieve that degree
    uint32_t arr = timer->Init.Period;
    double pulse_width = ((double)HSP_SERVO_MAX_PULSE_WIDTH-HSP_SERVO_MIN_PULSE_WIDTH)/HSP_SERVO_MAX_DEG * deg + HSP_SERVO_MIN_PULSE_WIDTH;
    return pulse_width*arr/HSP_SERVO_PWM_PERIOD;
}

void Actuate_Servo(uint8_t deg, const TIM_HandleTypeDef *htim) {
	// Sets servo position to deg
	TIM2->CCR3 = Deg_To_CCR(deg, htim);
}

void Test_Actuate_Servo(const TIM_HandleTypeDef *htim) {
	// Moves servo 90 degrees then back
	Actuate_Servo(90, htim);
	HAL_Delay(2000);
	Actuate_Servo(175, htim);
	HAL_Delay(2000);
	Actuate_Servo(90, htim);
}

bool Check_Servo_Cont() {
	/* Checks continuity of servo pin
	Delay needed bc electrical engineering is hard
	Turns off servo, must turn on servo again after calling this
	if desired */
	Turn_Off_Servo();
	HAL_Delay(50);
	GPIO_PinState continuity = HAL_GPIO_ReadPin(SERVO_CONT_GPIO_Port, SERVO_CONT_Pin);
	return continuity;
}


#endif /* INC_SERVO_UTILS_H_ */
