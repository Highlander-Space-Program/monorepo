/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
#include "stdbool.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
// Make these global so they can be inspected in the debugger
volatile uint32_t stacked_r0;
volatile uint32_t stacked_r1;
volatile uint32_t stacked_r2;
volatile uint32_t stacked_r3;
volatile uint32_t stacked_r12;
volatile uint32_t stacked_lr; // Link Register from stack
volatile uint32_t stacked_pc; // Program Counter from stack
volatile uint32_t stacked_psr;// Program Status Register from stack

volatile uint32_t actual_lr_value; // Actual LR at time of fault entry

volatile uint32_t _cfsr;
volatile uint32_t _hfsr;
volatile uint32_t _dfsr;
volatile uint32_t _afsr;
volatile uint32_t _mmfar;
volatile uint32_t _bfar;
volatile uint16_t _ufsr_flags;
volatile uint8_t  _mfsr_flags;
volatile uint8_t  _bfsr_flags;

volatile uint8_t test_var;

extern uint8_t g_hal_rx_byte_buffer;
extern volatile bool_t g_request_uart6_reinit;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
extern void xbee_platform_uart_tx_cplt_callback(UART_HandleTypeDef *huart_that_completed);
extern void xbee_platform_hal_enqueue_rx_byte(uint8_t byte_val);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern CAN_HandleTypeDef hcan1;
extern UART_HandleTypeDef huart6;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */
//
  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles CAN1 TX interrupts.
  */
void CAN1_TX_IRQHandler(void)
{
  /* USER CODE BEGIN CAN1_TX_IRQn 0 */

  /* USER CODE END CAN1_TX_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan1);
  /* USER CODE BEGIN CAN1_TX_IRQn 1 */

  /* USER CODE END CAN1_TX_IRQn 1 */
}

/**
  * @brief This function handles CAN1 RX0 interrupts.
  */
void CAN1_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN CAN1_RX0_IRQn 0 */

  /* USER CODE END CAN1_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan1);
  /* USER CODE BEGIN CAN1_RX0_IRQn 1 */

  /* USER CODE END CAN1_RX0_IRQn 1 */
}

/**
  * @brief This function handles USART6 global interrupt.
  */
void USART6_IRQHandler(void)
{
  /* USER CODE BEGIN USART6_IRQn 0 */

  /* USER CODE END USART6_IRQn 0 */
  HAL_UART_IRQHandler(&huart6);
  /* USER CODE BEGIN USART6_IRQn 1 */

  /* USER CODE END USART6_IRQn 1 */
}

/* USER CODE BEGIN 1 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART6) // Or your specific XBee UART instance
  {
    // A byte has been received into g_hal_rx_byte_buffer (defined in xbee_platform_uart.c)
    xbee_platform_hal_enqueue_rx_byte(g_hal_rx_byte_buffer);

    // Re-arm the reception for the next byte
    if (HAL_UART_Receive_IT(huart, &g_hal_rx_byte_buffer, 1) != HAL_OK)
    {
      // Error re-arming reception. This is a critical issue.
      // You might want to set an application error flag or attempt recovery.
      // For example, you could try to re-initialize the UART receive path.
      // Error_Handler(); // Or a less drastic error handling
    }
  }
  // Add else if for other UARTs if you use them with Receive_IT
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART6) // Or your specific XBee UART instance
  {
    xbee_platform_uart_tx_cplt_callback(huart);
  }
  // Add else if for other UARTs if you use them with Transmit_IT
}

/**
  * @brief  UART error callbacks.
  * @param  huart Pointer to a UART_HandleTypeDef structure that contains
  * the configuration information for the specified UART module.
  * @retval None
  */

/**
  * @brief  UART error callbacks.
  * @param  huart Pointer to a UART_HandleTypeDef structure that contains
  * the configuration information for the specified UART module.
  * @retval None
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART6) // Check if the error is from your XBee UART
  {
    // You can log huart->ErrorCode to see what specific error occurred
    // For example: printf("UART6 Error: 0x%lx\r\n", huart->ErrorCode);

    // Handle specific errors and attempt to recover
    // It's important to clear error flags. The HAL_UART_IRQHandler might clear some,
    // but explicitly clearing them or taking recovery actions here is robust.

    if (huart->ErrorCode & HAL_UART_ERROR_ORE)
    {
      // Overrun error
      // For many STM32s, the HAL_UART_IRQHandler (by reading DR) might clear ORE.
      // However, ensuring the UART is ready for new data is key.
      __HAL_UART_CLEAR_OREFLAG(huart); // Good practice to explicitly clear if available/needed
    }
    if (huart->ErrorCode & HAL_UART_ERROR_NE)
    {
      // Noise error
      __HAL_UART_CLEAR_NEFLAG(huart); // Check if specific clear flag macros exist for your HAL version/MCU
                                      // Or often, reading DR (done by IRQHandler for RXNE) clears these.
    }
    if (huart->ErrorCode & HAL_UART_ERROR_FE)
    {
      // Framing error
      __HAL_UART_CLEAR_FEFLAG(huart); // Similar to NE.
    }

    // After an error, the HAL might have stopped the interrupt-driven reception.
    // It's crucial to re-start it to continue receiving data.
    // The UART error flags should be cleared before attempting to re-arm.
    // The HAL_UART_IRQHandler usually handles reading DR to clear RXNE, which also clears ORE, FE, NE.
    // The main goal here is to ensure HAL_UART_Receive_IT is called again.

    // Attempt to re-arm the reception.
    // It's possible that HAL_UART_AbortReceive_IT(huart) might be needed first if the
    // HAL's internal state is stuck, but try re-arming directly first.
    if (HAL_UART_Receive_IT(huart, &g_hal_rx_byte_buffer, 1) != HAL_OK)
    {
      // Failed to re-arm reception after an error.
      // This is a serious situation. You might need to:
      // 1. Log this critical failure.
      // 2. Attempt a full UART peripheral re-initialization:
      //    (Be cautious with calling your xbee_ser_close/open from an ISR directly
      //     if they are not designed for it. Setting a flag for the main loop
      //     to handle re-initialization might be safer.)
      //    Error_Handler(); // Or a less drastic, specific error handling routine
    	test_var = test_var + 1;
    }
  }
  // Add else if for other UARTs if they have errors
}

/* USER CODE END 1 */
/* USER CODE END 1 */
