/*
 * servo.h
 *
 *  Created on: Jan 7, 2025
 *      Author: zande
 */

#include "tim.h"
#include "config.h"

#ifndef INC_SERVO_H_
#define INC_SERVO_H_

//disables PWM signal to all servos
void Servo_Disable() {
	  HAL_TIM_PWM_Stop(&htim16, TIM_CHANNEL_1);
	  HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
	  HAL_TIMEx_PWMN_Stop(&htim17, TIM_CHANNEL_1);
	  //HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4);
	  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
	  HAL_GPIO_WritePin(SERVO_EN_GPIO_Port, SERVO_EN_Pin, GPIO_PIN_RESET);
}

void Servo_Enable() {
	  HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
	  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
	  HAL_TIMEx_PWMN_Start(&htim17, TIM_CHANNEL_1);
	  //HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4);
	  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);  // <- this makes me want to cry
	  HAL_GPIO_WritePin(SERVO_EN_GPIO_Port, SERVO_EN_Pin, GPIO_PIN_SET);
}
#endif /* INC_SERVO_H_ */
