#ifndef SENSORBOARDFIRMWARE_ADS1118_H
#define SENSORBOARDFIRMWARE_ADS1118_H

#include <stdint-gcc.h>
#include "stm32f0xx_hal_spi.h"

#define ADS1118_SPI_SIZE 4 // Length of SPI transmission in bytes

#define ADS1118_CONFIG_BIT_SS 15
#define ADS1118_CONFIG_BIT_MUX 12

#define ADS1118_CONFIG_DEFAULT 0x058b
#define ADS1118_MODE_CONTINUOUS 0b0
#define ADS1118_MODE_SINGLE_SHOT 0b1
#define ADS1118_TS_MODE_ADC 0b0
#define ADS1118_TS_MODE_TEMPERATURE 0b1
#define ADS1118_NOP 0b01

typedef uint16_t Ads1118Config;

typedef struct {
	uint16_t config;
	uint16_t config_readback; // Config values read back from ADS1118 during SPI transmissions

	SPI_HandleTypeDef *hspi;

	GPIO_TypeDef *cs_gpio_port;
	uint16_t cs_pin;
} Ads1118TypeDef;

HAL_StatusTypeDef Ads1118_Configure(Ads1118TypeDef *adc) {
  HAL_GPIO_WritePin(adc->cs_gpio_port, adc->cs_pin, GPIO_PIN_RESET);
  HAL_Delay(1);

  uint32_t out = adc->config | (((uint32_t)adc->config)<<16); // Transmit 16 bit config twice for a 32 bit transmission sequence
  HAL_StatusTypeDef status = HAL_SPI_Transmit_DMA(adc->hspi, (uint8_t*)&out, 4);
  while(1) {
	  if (status != HAL_OK) {
		  HAL_GPIO_WritePin(adc->cs_gpio_port, adc->cs_pin, GPIO_PIN_SET);
		  return status;
	  }
    if (adc->hspi->State != HAL_SPI_STATE_BUSY) {
      break;
    }
  }
  // Reset ADC SPI
	HAL_Delay(1);
    HAL_GPIO_WritePin(adc->cs_gpio_port, adc->cs_pin, GPIO_PIN_SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(adc->cs_gpio_port, adc->cs_pin, GPIO_PIN_RESET);
	HAL_Delay(1); // 100ns CS rising edge propagation delay per ADS1118 datasheet

	return status;
}

HAL_StatusTypeDef Ads1118_Transmit(Ads1118TypeDef *adc, uint32_t *data) {
    uint32_t out = adc->config | (((uint32_t)adc->config)<<16); // Transmit 16 bit config twice for a 32 bit transmission sequence
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive_DMA(adc->hspi, (uint8_t*)&out, (uint8_t*)data, ADS1118_SPI_SIZE);
    if (status != HAL_OK) {
    }
	return status;
}

#endif //SENSORBOARDFIRMWARE_ADS1118_H
