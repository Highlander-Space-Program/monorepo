/*
 * temperature_utils.h
 *
 *  Created on: Jan 27, 2025
 *      Author: brandonmarcus
 */

#ifndef INC_THERMOCOUPLE_UTILS_H_
#define INC_THERMOCOUPLE_UTILS_H_

double Get_Temperature(uint32_t adc_val) {
	double voltage = adc_val * (3.3 / 4096);
	return (voltage - 1.24) / 0.005;
}

#endif /* INC_THERMOCOUPLE_UTILS_H_ */
