/*
 * deg_to_ccr.h
 *
 *  Created on: Jun 1, 2024
 *      Author: zan
 */

#ifndef INC_DEG_TO_CCR_H_
#define INC_DEG_TO_CCR_H_
#include "config.h"

//returns ccr needed to move servo to the degree passed in in deg variable
uint32_t Deg_To_CCR(uint8_t deg, const struct Servo *servo, const int max_deg) {
  uint32_t arr = servo->timer->Init.Period;
  double pulse_width = ((double)HSP_SERVO_MAX_PULSE_WIDTH-HSP_SERVO_MIN_PULSE_WIDTH)/max_deg * deg + HSP_SERVO_MIN_PULSE_WIDTH;
  return pulse_width*arr/HSP_SERVO_PWM_PERIOD;
}


#endif /* INC_DEG_TO_CCR_H_ */
