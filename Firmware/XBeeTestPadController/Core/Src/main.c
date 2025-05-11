/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "stm32/platform_config.h"
#include "stm32/xbee_platform_uart.h"
#include "stm32/xbee_actions.h"
#include "xbee/platform.h"
#include "xbee/device.h"
#include "xbee/atcmd.h"
#include "xbee/byteorder.h"   // for htobe16()
#include "xbee/wpan.h"
#include "wpan/types.h"       // for addr64, WPAN_IEEE_ADDR_BROADCAST, WPAN_NET_ADDR_UNDEFINED

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_USART6_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
typedef struct {
    volatile bool done;
    uint32_t      value;
    uint16_t      flags;
} at_sync_t;


static volatile uint32_t dbg_cr1;
volatile uint32_t dbg_iser;
volatile bool     dbg_rxneie;
volatile bool     dbg_irqn;
volatile uint32_t dbg_lib_AP;
volatile uint16_t dbg_xbee_flags;

static xbee_dev_t   xbee;

//#define TEST_HTOBE16(x) (uint16_t)( (((x)&0xFF)<<8) | (((x)&0xFF00)>>8) )
//
//#if htobe16(0x1234) != TEST_HTOBE16(0x1234)
//  #error "htobe16() is not swapping correctly — fix your BYTE_ORDER macros"
//#endif

static int at_sync_cb(const xbee_cmd_response_t *resp)
{
    at_sync_t *sync = (at_sync_t *)resp->context;

    /* pull up to 4 bytes into host‑endian uint32_t */
    uint32_t v = 0;
    switch (resp->value_length) {
        case 1: v = resp->value_bytes[0];                               break;
        case 2: v = be16toh(*(uint16_t *)resp->value_bytes);            break;
        case 4: v = be32toh(*(uint32_t *)resp->value_bytes);            break;
        default: /* >4 bytes ignored for this helper */                 break;
    }

    sync->value = v;
    sync->flags = resp->flags;
    sync->done  = true;
    return XBEE_ATCMD_DONE;               /* auto‑release handle      */
}

/* call:  xbee_at_read_uint32(&xbee, "VR", &ver);                         */
static int xbee_at_read_uint32(xbee_dev_t *xbee,
                               const char *at,
                               uint32_t   *out)
{
    int16_t handle = xbee_cmd_create(xbee, at);
    if (handle < 0)                           return handle;

    at_sync_t sync = { .done = false };
    xbee_cmd_set_callback(handle, at_sync_cb, &sync);

    int rc = xbee_cmd_send(handle);           /* fire the request      */
    if (rc)                                   return rc;

    while (!sync.done) {                      /* busy‑wait for reply   */
        xbee_dev_tick(xbee);
        xbee_cmd_tick();
    }

    if (XBEE_AT_RESP_STATUS(sync.flags) != XBEE_AT_RESP_SUCCESS)
        return -EIO;                          /* modem returned error  */

    *out = sync.value;
    return 0;
}

static void send_broadcast(void)
{
    const addr64 ieee_broadcast = *WPAN_IEEE_ADDR_BROADCAST;
    static const char msg[] = "Hello from F405";

    uint8_t frame_id = xbee_next_frame_id(&xbee);

    xbee_header_transmit_t hdr = {
        .frame_type         = XBEE_FRAME_TRANSMIT,      /* 0x10 */
        .frame_id           = frame_id,
        .ieee_address       = ieee_broadcast,
        .network_address_be = htobe16(WPAN_NET_ADDR_UNDEFINED), /* 0xFFFE */
        .broadcast_radius   = 0,        /* use NH parameter (default 10)  */
        .options            = 0
    };

    int rc = xbee_frame_write(&xbee,
                              &hdr, sizeof hdr,
                              msg, sizeof msg,
                              XBEE_WRITE_FLAG_NONE);
    if (rc < 0) {
        //printf("xbee_frame_write error %d\r\n", rc);
        HAL_GPIO_TogglePin(STATUS_IND_GPIO_Port, STATUS_IND_Pin);
    }
}

// call this once at startup, before any xbee_dev_init()
// returns true if we actually changed from AT→API
// returns false if already in API and untouched
// max frame size = 256 payload + overhead
#define FRAME_BUF_SIZE   (256 + 4)

/// Blocking flush of any bytes in the UART RX FIFO (up to timeout_ms)
static void uart_flush_rx(uint32_t timeout_ms) {
    uint8_t tmp;
    uint32_t t0 = HAL_GetTick();
    while (HAL_UART_Receive(&huart6, &tmp, 1, 1) == HAL_OK) {
        if ((HAL_GetTick() - t0) > timeout_ms) break;
    }
}

/// Send a raw API frame to read the “AP” parameter
static void send_at_read_AP(void) {
    // length = 0x0004 (frame type + frame id + AT cmd[2])
    // checksum = 0xFF - sum(08 + 01 + 'A' + 'P') = 0x65
    const uint8_t frame[] = {
        0x7E,       // start
        0x00, 0x04, // length = 4
        0x08,       // frame type: local AT
        0x01,       // frame ID = 1
        'A','P',    // AT command = "AP"
        0x65        // checksum
    };
    HAL_UART_Transmit(&huart6, (uint8_t*)frame, sizeof frame, HAL_MAX_DELAY);
}

/// Wait for one full API frame, up to timeout_ms ms.
/// On success copies the entire frame (including checksum) into buf,
/// and returns the total byte count (>=4). On failure returns -1.
static int uart_read_api_frame(uint8_t *buf, uint32_t timeout_ms) {
    uint8_t b;
    uint32_t t0 = HAL_GetTick();

    // 1) wait for 0x7E
    do {
        if (HAL_UART_Receive(&huart6, &b, 1, 1) == HAL_OK && b == 0x7E) {
            buf[0] = 0x7E;
            break;
        }
    } while ((HAL_GetTick() - t0) < timeout_ms);
    if (buf[0] != 0x7E) return -1;

    // 2) read length MSB/LSB
    if (HAL_UART_Receive(&huart6, buf+1, 2, timeout_ms) != HAL_OK) return -1;
    uint16_t length = (buf[1] << 8) | buf[2];
    if (length + 4 > FRAME_BUF_SIZE) return -1;  // too big!

    // 3) read payload + checksum
    if (HAL_UART_Receive(&huart6, buf+3, length + 1, timeout_ms) != HAL_OK)
        return -1;

    return length + 4;
}

/// Master routine: flush old bytes, send AT "AP", read response, print value.
int read_AP_over_api(void) {
    uint8_t frame[FRAME_BUF_SIZE];
    int n;

    // 1) clear any old cruft
    uart_flush_rx(50);

    // 2) send the AT-read-AP request
    send_at_read_AP();

    // 3) read back one API frame
    n = uart_read_api_frame(frame, 200);
    if (n < 0) {
        //printf("No API frame received\n");
        return;
    }

    // 4) parse it: expect a Local AT response (0x88), ID=1, cmd='A','P', status=0
    if (n >= 9 &&
        frame[0] == 0x7E &&
        frame[3] == 0x88 &&
        frame[4] == 0x01 &&
        frame[5] == 'A' &&
        frame[6] == 'P' &&
        frame[7] == 0x00  /* status OK */)
    {
        uint8_t ap = frame[8];
        //printf(">>> AP = %u\n", ap);
        return ap;
    }
//    else {
        //printf("Unexpected frame (len=%d):", n);
    	//for (int i = 0; i < n; i++) printf(" %02X", frame[i]);
        //printf("\n");
//    }
    return -1;
}

static void dbg_dump_xbee_flags(xbee_dev_t *xbee)
{
    dbg_xbee_flags = xbee->flags;
}

static void uart_debug_dump(void)
{
    dbg_cr1    = USART6->CR1;
    uint32_t idx = USART6_IRQn >> 5;
    uint32_t bit = USART6_IRQn & 0x1F;
    dbg_iser   = NVIC->ISER[idx];
    dbg_rxneie = (dbg_cr1 & USART_CR1_RXNEIE) != 0;
    dbg_irqn   = (dbg_iser & (1u << bit)) != 0;
}

/**
 * Send an arbitrary byte sequence over USART6, then block for up to
 * timeout_ms to receive up to rx_len bytes into rx_buf.
 *
 * @param tx_buf      Pointer to bytes to send
 * @param tx_len      Number of bytes to send
 * @param rx_buf      Buffer to store received bytes
 * @param rx_len      Max bytes to receive
 * @param timeout_ms  How long to wait (ms) for the first byte (then reads as many as arrive)
 * @return            Number of bytes actually received, or -1 on TX error
 */
int uart_send_receive(
    const uint8_t *tx_buf, size_t tx_len,
    uint8_t *rx_buf,       size_t rx_len,
    uint32_t timeout_ms
) {
    // 1) clear hardware FIFO
    {
        uint8_t tmp;
        uint32_t t0 = HAL_GetTick();
        while (HAL_UART_Receive(&huart6, &tmp, 1, 1) == HAL_OK) {
            if ((HAL_GetTick() - t0) > timeout_ms) break;
        }
    }
    // 2) clear software ring
    xbee_ser_rx_flush( xbee_platform_serial() );

    // 3) send your bytes
    if (HAL_UART_Transmit(&huart6, (uint8_t*)tx_buf, (uint16_t)tx_len, HAL_MAX_DELAY) != HAL_OK) {
        return -1;
    }

    // 4) now block for reply
    int n = HAL_UART_Receive(&huart6, rx_buf, (uint16_t)rx_len, timeout_ms);
    return n < 0 ? 0 : n;
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(STATUS_IND_GPIO_Port, STATUS_IND_Pin, GPIO_PIN_SET);
//  int ap = read_AP_over_api();
//
//  uint8_t tx_frame[] = {
//      0x7E, 0x00, 0x04,   // correct length = 4
//      0x08,               // Local AT
//      0x01,               // frame ID
//      'H','V',            // AT command = "HV"
//      0x58
//  };
//  uint8_t rx_buf[32];
//  int rx_n = uart_send_receive(tx_frame, sizeof tx_frame,
//                                 rx_buf,   sizeof rx_buf,
//                                 1000);
//  rx_n = uart_send_receive(tx_frame, sizeof tx_frame,
//                                 rx_buf,   sizeof rx_buf,
//                                 1000);

  // --- 1) init the XBee device in the library ---

  // THIS DELAY ALLOWS XBEE TO WAKE UP
  HAL_Delay(1000);

  xbee_dev_init(&xbee, xbee_platform_serial(), always_awake, NULL);
  xbee_dev_flowcontrol(&xbee, 0);

  // --- 2) verify interrupts are really enabled ---
  uart_debug_dump();
  // now set a breakpoint on the next line and check:
  //  dbg_rxneie == true
  //  dbg_irqn   == true
  dbg_dump_xbee_flags(&xbee);

  // --- 3) kick off the built-in handshake/query ---
  xbee_ser_rx_flush( xbee_platform_serial() );
  xbee_cmd_init_device(&xbee);

  // one tick to *send* the first "HV" frame
  xbee_dev_tick(&xbee);

  xbee_cmd_tick();
  // put a breakpoint here and scope your TX pin — you should see:
  //   7E 00 04 08 01 48 56 58
  int status = 0;
  // now spin until that handshake comes back

  do {
      xbee_dev_tick(&xbee);
      xbee_cmd_tick();
      dbg_dump_xbee_flags(&xbee);
      status = xbee_cmd_query_status(&xbee);
  } while (status == -EBUSY);

  // check result
  if (status != 0) {
      // handshake failed — set a breakpoint here to inspect 'status'
      return status;
  }

  // --- 4) now use the library to read "AP" ---
  dbg_lib_AP = 0xFFFFFFFF;
  xbee_at_read_uint32(&xbee, "AP", &dbg_lib_AP);
  // read failed — dbg_lib_AP stays 0xFFFFFFFF
  // if all is well, dbg_lib_AP == 2

  // 4. Now it’s safe to issue your own commands
  uint32_t sh;
  if (xbee_at_read_uint32(&xbee, "SH", &sh) == 0) {
      //printf("Serial‑Low (SH) = %08lX\r\n", (unsigned long)sh);
	  HAL_GPIO_TogglePin(STATUS_IND_GPIO_Port, STATUS_IND_Pin);
  }

  uint32_t sl;
  if (xbee_at_read_uint32(&xbee, "SL", &sl) == 0) {
      //printf("Serial‑Low (SL) = %08lX\r\n", (unsigned long)sl);
	  HAL_GPIO_TogglePin(STATUS_IND_GPIO_Port, STATUS_IND_Pin);
  }

  /* forever */
  uint32_t tick = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  xbee_dev_tick(&xbee);   /* UART RX/TX, frame dispatcher          */
	  xbee_cmd_tick();        /* keeps AT‑command table moving         */

	  if (HAL_GetTick() - tick >= 1000) {      /* every 1 s */
//		  send_broadcast();
		  tick = HAL_GetTick();
		  HAL_GPIO_TogglePin(STATUS_IND_GPIO_Port, STATUS_IND_Pin);
	  }
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 8;
  hcan1.Init.Mode = CAN_MODE_LOOPBACK;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 9600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(STATUS_IND_GPIO_Port, STATUS_IND_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, BRK_CONT_LED_SINK_Pin|BRK_CONT_LED_Pin|IGNITER_Pin|BRK_CONT_SINK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : STATUS_IND_Pin */
  GPIO_InitStruct.Pin = STATUS_IND_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(STATUS_IND_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : IGNITER_CONT_Pin */
  GPIO_InitStruct.Pin = IGNITER_CONT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IGNITER_CONT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BRK_CONT_LED_SINK_Pin BRK_CONT_LED_Pin IGNITER_Pin BRK_CONT_SINK_Pin */
  GPIO_InitStruct.Pin = BRK_CONT_LED_SINK_Pin|BRK_CONT_LED_Pin|IGNITER_Pin|BRK_CONT_SINK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : BRK_CONT_Pin */
  GPIO_InitStruct.Pin = BRK_CONT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BRK_CONT_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: //printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
