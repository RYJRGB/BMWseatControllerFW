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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "board.h"
#include "usbd_cdc_if.h"
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern UART_HandleTypeDef huart2;   // USART2 from CubeMX (LIN mode)
static uint8_t make_lin_id_with_parity(uint8_t id6);
static HAL_StatusTypeDef lin_send_header(UART_HandleTypeDef *huart, uint8_t id6);
static int lin_receive_response(UART_HandleTypeDef *huart, uint8_t *buf, int max_len, uint32_t timeout_ms);
static void cdc_print_hex(const char *prefix, const uint8_t *data, int len);
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t TxData[20];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void uart_flush_errors(UART_HandleTypeDef* hu);

/* USER CODE BEGIN PFP */
void SetHalfBridge(HalfBridge_t hb, Direction_t dir);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void LIN_Ping_And_Print(void)
{
    uint8_t tx[2] = { 0x55, 0xC1 };   // SYNC + PID for ID 0x01

    // Optional but recommended: keep RX off during header to avoid echo/junk
    CLEAR_BIT(huart2.Instance->CR1, USART_CR1_RE);

    HAL_LIN_SendBreak(&huart2);
    HAL_UART_Transmit(&huart2, tx, 2, 20);   // <-- length = 2 (not 3)

    // Flush BREAK artifacts so first slave byte isn't lost
    __HAL_UART_CLEAR_FEFLAG(&huart2);
    __HAL_UART_CLEAR_NEFLAG(&huart2);
    __HAL_UART_CLEAR_OREFLAG(&huart2);
#if defined(USART_RDR_RDR) || defined(USART_ISR_RXNE)
    (void)huart2.Instance->ISR; (void)huart2.Instance->RDR;
#else
    (void)huart2.Instance->SR;  (void)huart2.Instance->DR;
#endif
    huart2.ErrorCode = HAL_UART_ERROR_NONE;

    // Re-enable RX to catch the response
    SET_BIT(huart2.Instance->CR1, USART_CR1_RE);

    uint8_t rx[9];
    int n = lin_receive_response(&huart2, rx, sizeof(rx), 80); // try 50–100 ms first
    if (n <= 0) {
        CDC_Transmit_FS((uint8_t*)"LIN: no response\r\n", 18);
    } else {
        cdc_print_hex("LIN ID 0x01 -> ", rx, n);
    }
}


// --- Helpers ---

// Parity per LIN 2.x: ID = b5..b0, P0 on b6, P1 on b7
static uint8_t make_lin_id_with_parity(uint8_t id6)
{
    id6 &= 0x3F;
    uint8_t b0 = (id6 >> 0) & 1;
    uint8_t b1 = (id6 >> 1) & 1;
    uint8_t b2 = (id6 >> 2) & 1;
    uint8_t b3 = (id6 >> 3) & 1;
    uint8_t b4 = (id6 >> 4) & 1;
    uint8_t b5 = (id6 >> 5) & 1;

    uint8_t p0 = (b0 ^ b1 ^ b2 ^ b4) & 1;
    uint8_t p1 = (~(b1 ^ b3 ^ b4 ^ b5)) & 1;

    return (uint8_t)(id6 | (p0 << 6) | (p1 << 7));
}

// Send BREAK + SYNC(0x55) + ID(with parity)
static HAL_StatusTypeDef lin_send_header(UART_HandleTypeDef *huart, uint8_t id6)
{
    // 1) Keep RX off while we transmit BREAK+SYNC+PID
    CLEAR_BIT(huart->Instance->CR1, USART_CR1_RE);

    if (HAL_LIN_SendBreak(huart) != HAL_OK) goto err;

    uint8_t sync = 0x55;
    if (HAL_UART_Transmit(huart, &sync, 1, 5) != HAL_OK) goto err;

    uint8_t pid = make_lin_id_with_parity(id6);   // 0x01 -> 0xC1
    if (HAL_UART_Transmit(huart, &pid, 1, 5) != HAL_OK) goto err;

    // 2) Flush errors/junk from our own header
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_OREFLAG(huart);
#if defined(USART_RDR_RDR) || defined(USART_ISR_RXNE)
    (void)huart->Instance->ISR; (void)huart->Instance->RDR;
#else
    (void)huart->Instance->SR;  (void)huart->Instance->DR;
#endif
    huart->ErrorCode = HAL_UART_ERROR_NONE;

    // 3) Turn RX back on right before the slave talks
    SET_BIT(huart->Instance->CR1, USART_CR1_RE);
    return HAL_OK;

err:
    SET_BIT(huart->Instance->CR1, USART_CR1_RE);
    return HAL_ERROR;
}


// Read up to max_len bytes until timeout (slave data + checksum)
static int lin_receive_response(UART_HandleTypeDef *huart, uint8_t *buf, int max_len, uint32_t timeout_ms)
{
    // Simple blocking read; for production, use DMA + IDLE detection
    int n = 0;
    uint32_t t0 = HAL_GetTick();
    while (n < max_len) {
        if (HAL_UART_Receive(huart, &buf[n], 1, 1) == HAL_OK) {
            n++;
            t0 = HAL_GetTick(); // got something; extend window
        }
        if ((HAL_GetTick() - t0) > timeout_ms) break;
    }
    return n; // 0 means no data
}

static void cdc_print_hex(const char *prefix, const uint8_t *data, int len)
{
    char line[128];
    int off = snprintf(line, sizeof(line), "%s", prefix);
    for (int i = 0; i < len && off < (int)sizeof(line) - 4; ++i) {
        off += snprintf(line + off, sizeof(line) - off, "%02X ", data[i]);
    }
    if (off < (int)sizeof(line) - 2) line[off++] = '\r', line[off++] = '\n';
    CDC_Transmit_FS((uint8_t*)line, off);
}

static void uart_flush_errors(UART_HandleTypeDef* hu)
{
  __HAL_UART_CLEAR_FEFLAG(hu);
  __HAL_UART_CLEAR_NEFLAG(hu);
  __HAL_UART_CLEAR_OREFLAG(hu);
#if defined(USART_RDR_RDR) || defined(USART_ISR_RXNE) // F0/L4/G4 style
  (void)hu->Instance->ISR; (void)hu->Instance->RDR;
#else // F1/F2/F3/F4 style
  (void)hu->Instance->SR;  (void)hu->Instance->DR;
#endif
  hu->ErrorCode = HAL_UART_ERROR_NONE;
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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  HAL_Delay(1000);
  HAL_GPIO_WritePin(LIN_CS_GPIO_Port, LIN_CS_Pin, GPIO_PIN_SET); //set LIN txcvr CS pin high
  const char *msg = "Hello BMW\r\n";

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  LIN_Ping_And_Print();
	  HAL_Delay(250);
//	  for (HalfBridge_t hb = HB1; hb <= HB5; hb++) {
//	      SetHalfBridge(hb, DIR_FORWARD);
//	      HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
//	      HAL_Delay(3000);  // 3 seconds forward
//	      HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
//
//	      SetHalfBridge(hb, DIR_REVERSE);
//	      HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
//	      HAL_Delay(3000);  // 3 seconds reverse
//	      HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
//	      SetHalfBridge(hb, DIR_IDLE);  // Optional step before idle all
//	      HAL_Delay(1000);  // Pause 1 second
//	      HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
//
//	      SetHalfBridge(IDLE_MOTOR, DIR_IDLE);  // All off
//	      HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
//	    }
    /* USER CODE END WHILE */

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 15;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 19200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_LIN_Init(&huart2, UART_LINBREAKDETECTLENGTH_11B) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, BR1B_Pin|BR1A_Pin|BR2B_Pin|BR2A_Pin
                          |BR3B_Pin|BR3A_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, BR4B_Pin|BR4A_Pin|BR5B_Pin|BR5A_Pin
                          |LS3_EN_Pin|LS2_EN_Pin|LS1_EN_Pin|LIN_CS_Pin
                          |RS485DE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BR1B_Pin BR1A_Pin BR2B_Pin BR2A_Pin
                           BR3B_Pin BR3A_Pin */
  GPIO_InitStruct.Pin = BR1B_Pin|BR1A_Pin|BR2B_Pin|BR2A_Pin
                          |BR3B_Pin|BR3A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : BR4B_Pin BR4A_Pin BR5B_Pin BR5A_Pin
                           LS3_EN_Pin LS2_EN_Pin LS1_EN_Pin LIN_CS_Pin
                           RS485DE_Pin */
  GPIO_InitStruct.Pin = BR4B_Pin|BR4A_Pin|BR5B_Pin|BR5A_Pin
                          |LS3_EN_Pin|LS2_EN_Pin|LS1_EN_Pin|LIN_CS_Pin
                          |RS485DE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : INA_ALERT_Pin */
  GPIO_InitStruct.Pin = INA_ALERT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(INA_ALERT_GPIO_Port, &GPIO_InitStruct);

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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
