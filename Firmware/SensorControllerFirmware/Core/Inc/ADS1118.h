#ifndef SENSORBOARDFIRMWARE_ADS1118_H
#define SENSORBOARDFIRMWARE_ADS1118_H

#include <stdint-gcc.h>
#include "stm32f0xx_hal_spi.h" // Ensure this matches your MCU family, e.g., stm32f4xx_hal_spi.h

#define ADS1118_SPI_RX_BUFFER_SIZE_WORDS 2 // To receive 16-bit data + 16-bit config readback = 2 words (4 bytes)

#define ADS1118_CONFIG_BIT_SS 15
#define ADS1118_CONFIG_BIT_MUX 12

#define ADS1118_CONFIG_DEFAULT 0x058b // Example: PGA=FS, DR=128SPS. Adjust as needed.
// Define other constants as before

typedef uint16_t Ads1118Config;

typedef struct {
	uint16_t config;
	uint16_t config_readback;

	SPI_HandleTypeDef *hspi;

	GPIO_TypeDef *cs_gpio_port;
	uint16_t cs_pin;
} Ads1118TypeDef;

/**
 * @brief Configures the ADS1118 for a specific channel and settings.
 * @param adc Pointer to Ads1118TypeDef structure.
 * @retval HAL_StatusTypeDef HAL status.
 * @note This function uses blocking SPI communication.
 */
HAL_StatusTypeDef Ads1118_Configure(Ads1118TypeDef *adc) {
  HAL_GPIO_WritePin(adc->cs_gpio_port, adc->cs_pin, GPIO_PIN_RESET); // CS Low

  // For ADS1118, writing the config register is a single 16-bit operation.
  // Ensure your SPI peripheral is configured for 16-bit data size (hspi1.Init.DataSize = SPI_DATASIZE_16BIT).
  // The original code sent config twice. If this is required by your specific setup or interpretation,
  // you can send two 16-bit words. However, typically one is enough for simple config.
  // uint16_t config_payload[2] = {adc->config, adc->config}; // If sending twice
  // HAL_StatusTypeDef status = HAL_SPI_Transmit(adc->hspi, (uint8_t*)config_payload, 2, HAL_MAX_DELAY);

  // Sending config once:
  HAL_StatusTypeDef status = HAL_SPI_Transmit(adc->hspi, (uint8_t*)&(adc->config), 1, HAL_MAX_DELAY); // 1 unit of 16-bit data

  HAL_GPIO_WritePin(adc->cs_gpio_port, adc->cs_pin, GPIO_PIN_SET); // CS High to complete transaction

  // A very short delay might be needed here if the ADC requires it before CS can go low again.
  // HAL_Delay(1); // Or a microsecond delay if available and necessary.

  return status;
}

/**
 * @brief Transmits configuration to ADS1118 to start a conversion and prepares to receive data using DMA.
 * @param adc Pointer to Ads1118TypeDef structure.
 * @param data_rx_buffer Pointer to the buffer where 2 words (16-bit each: data + config_readback) will be stored.
 * @retval HAL_StatusTypeDef HAL status.
 */
HAL_StatusTypeDef Ads1118_Transmit(Ads1118TypeDef *adc, uint16_t *data_rx_buffer) {
    HAL_GPIO_WritePin(adc->cs_gpio_port, adc->cs_pin, GPIO_PIN_RESET); // CS Low for transaction

    // To initiate a conversion and read data, we send the configuration (with SS=1).
    // The ADS1118 will then output the conversion result on subsequent clocks, followed by its config register.
    // We send the config again as dummy data to clock out both the result and the config readback.
    uint16_t tx_payload[ADS1118_SPI_RX_BUFFER_SIZE_WORDS];
    tx_payload[0] = adc->config; // This should contain the desired MUX, PGA, DR, and SS=1
    tx_payload[1] = adc->config; // Dummy write, or another config/NOP to clock out config readback

    // SPI is 16-bit data units. We are transmitting/receiving 2 such units (4 bytes total).
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive_DMA(adc->hspi, (uint8_t*)tx_payload, (uint8_t*)data_rx_buffer, ADS1118_SPI_RX_BUFFER_SIZE_WORDS);

    if (status != HAL_OK) {
      // If DMA initiation fails, bring CS high to terminate the SPI transaction.
      HAL_GPIO_WritePin(adc->cs_gpio_port, adc->cs_pin, GPIO_PIN_SET);
    }
    // CS will be brought high in the HAL_SPI_TxRxCpltCallback upon DMA completion.
	return status;
}

#endif //SENSORBOARDFIRMWARE_ADS1118_H
