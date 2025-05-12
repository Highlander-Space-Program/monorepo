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

extern uint8_t g_hal_rx_byte_buffer;
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
//void HardFault_Handler(void)
//{
//  uint32_t hfsr = SCB->HFSR;
//  uint32_t cfsr = SCB->CFSR;
//  uint32_t mmfar = SCB->MMFAR;
//  uint32_t bfar  = SCB->BFAR;
//  // now send these out over UART/semihost/SWO or inspect them in the debugger
//  for(;;);
//}

__attribute__((naked))
void HardFault_Handler(void)
{
  __asm volatile(
    " mov r2, lr \n"          // Save actual LR (EXC_RETURN value)
    " tst lr, #4            \n"
    " ite eq                \n"
    " mrseq r0, msp         \n"
    " mrsne r0, psp         \n"
    " mov r1, r0            \n" // Pass stacked frame pointer in R0 (as per AAPCS)
                              // R1 will also have stack pointer for C handler
    " b hard_fault_handler_c\n"
  );
}

void hard_fault_handler_c(uint32_t *p_stacked_frame_arg)
{
  // p_stacked_frame_arg points to the stacked registers (R0, R1, R2, R3, R12, LR, PC, PSR)
  stacked_r0  = p_stacked_frame_arg[0];
  stacked_r1  = p_stacked_frame_arg[1];
  stacked_r2  = p_stacked_frame_arg[2];
  stacked_r3  = p_stacked_frame_arg[3];
  stacked_r12 = p_stacked_frame_arg[4];
  stacked_lr  = p_stacked_frame_arg[5]; // LR (return address from subroutines)
  stacked_pc  = p_stacked_frame_arg[6]; // PC (faulting instruction)
  stacked_psr = p_stacked_frame_arg[7]; // xPSR

  // Capture fault status registers
  _cfsr  = SCB->CFSR;
  _hfsr  = SCB->HFSR;
  _dfsr  = SCB->DFSR;
  _afsr  = SCB->AFSR; // Note: AFSR might not be present or useful on all M4s for this
  _mmfar = SCB->MMFAR;
  _bfar  = SCB->BFAR;

  _ufsr_flags = (_cfsr >> 16) & 0xFFFF; // UsageFault Status Register
  _mfsr_flags = (_cfsr >> 0)  & 0xFF;   // MemManage Fault Status Register
  _bfsr_flags = (_cfsr >> 8)  & 0xFF;   // BusFault Status Register

  // actual_lr_value = ??; // Need to get this from r2 if you used the asm above.
  // A simple way is to pass it as another argument or store in global from asm
  // For now, we focus on the stacked LR/PC.

  // If you have a safe way to print (e.g., a simple polled UART not involved in the fault):
  // printf("HardFault!\nPC: 0x%08lx LR: 0x%08lx CFSR: 0x%08lx HFSR: 0x%08lx\n",
  //        stacked_pc, stacked_lr, _cfsr, _hfsr);
  // if (_ufsr_flags & (1 << 0)) { printf(" UFSR: UNALIGNED\n"); }
  // if (_ufsr_flags & (1 << 1)) { printf(" UFSR: INVSTATE\n"); }
  // ... and so on for other flags

  __BKPT(0); // Breakpoint for debugger
  while(1);
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
// at top of stm32f4xx_it.c
/**
  * @brief This function handles Usage fault interrupt.
  */
__attribute__((naked))
void UsageFault_Handler(void)
{
  __asm volatile(
    " tst lr, #4            \n"
    " ite eq                \n"
    " mrseq r0, msp         \n"
    " mrsne r0, psp         \n"
    " b usage_fault_handler_c\n"
  );
}

void usage_fault_handler_c(uint32_t *p_stacked_frame_arg)
{
  stacked_r0  = p_stacked_frame_arg[0];
  stacked_r1  = p_stacked_frame_arg[1];
  stacked_r2  = p_stacked_frame_arg[2];
  stacked_r3  = p_stacked_frame_arg[3];
  stacked_r12 = p_stacked_frame_arg[4];
  stacked_lr  = p_stacked_frame_arg[5];
  stacked_pc  = p_stacked_frame_arg[6];
  stacked_psr = p_stacked_frame_arg[7];

  _cfsr  = SCB->CFSR;
  _hfsr  = SCB->HFSR; // Check HFSR too, as UsageFault can escalate
  _ufsr_flags = (_cfsr >> 16) & 0xFFFF;

  // printf("UsageFault!\nPC: 0x%08lx LR: 0x%08lx CFSR: 0x%08lx (UFSR: 0x%04x)\n",
  //        stacked_pc, stacked_lr, _cfsr, _ufsr_flags);
  // if (_ufsr_flags & SCB_CFSR_UNALIGNED_Msk) { printf(" UFSR: UNALIGNED\n"); }
  // if (_ufsr_flags & SCB_CFSR_INVSTATE_Msk) { printf(" UFSR: INVSTATE\n"); }
  // if (_ufsr_flags & SCB_CFSR_INVPC_Msk) { printf(" UFSR: INVPC\n"); }
  // if (_ufsr_flags & SCB_CFSR_NOCP_Msk) { printf(" UFSR: NOCP\n"); }
  // if (_ufsr_flags & SCB_CFSR_UNDEFINSTR_Msk) { printf(" UFSR: UNDEFINSTR\n"); }
  // if (_ufsr_flags & SCB_CFSR_DIVBYZERO_Msk) { printf(" UFSR: DIVBYZERO\n"); }


  __BKPT(0);
  while(1);
}


//void UsageFault_Handler(void)
//{
//  /* USER CODE BEGIN UsageFault_IRQn 0 */
//
//  /* USER CODE END UsageFault_IRQn 0 */
//  while (1)
//  {
//    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
//    /* USER CODE END W1_UsageFault_IRQn 0 */
//  }
//}

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

/* USER CODE END 1 */
