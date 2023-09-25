/* USER CODE BEGIN Header */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "debug.h"
#include "rfm69/include/rfm69.h"
#include "config.h"

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
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
RFM69 RFM;
/**
 * Buffer used by the UART1 interrupt
 * Everytime the interrupt is triggered,
 * the received character is put here
 */
uint8_t UART1_rxBuffer[1] = {0};

/**
 * Accumulator for bytes received over UART1.
 * Only bytes received after the START bytes have been received
 * are put here.
 */
uint8_t UART1_msgAccumulator[32] = {0};
/**
 * The index at which the next character should be.
 * If this value is 2, the last valid character is at index 1.
 *
 * Should never be >= the size of the msgAccumulator.
 */
uint8_t UART1_msgBufferIdx = 0;

/**
 * Buffer containing a parsed message.
 * The contents of the msgAccumulator are put here after the STOP bytes
 * have been received.
 */
uint8_t UART1_msgReadyBuffer[32] = {0};
/**
 * The length of the msgReadyBuffer.
 *
 * Should not be >= 32
 */
uint8_t UART1_msgReadyLength = 0;
/**
 * Whether the UART1 interrupt handler
 * is currently in the middle of a message.
 * I.e. when the START bytes have been seen
 */
bool UART1_inMessage = false;
/**
 * Whether there's a message ready in UART1_msgReadyBuffer.
 * I.e. UART1_inMessage was true and the STOP bytes have been seen
 */
bool UART1_msgReady = false;
/**
 * The previous byte received by the UART1 interrupt handler.
 */
uint8_t UART1_previousValue = 0;

/**
 * Buffer for transmitting over UART1
 */
uint8_t UART1_txBuffer[UartTxBufSize] = { 0 };
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
 * Interrupt handler for UART1.
 * This handler is called for every character received via UART1.
 *
 * It'll detect the START and STOP bytes set by the controller and extract the message in between.
 * If a full message has been found, `UART1_msgReady` is set.
 * @param huart
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  // Safe as this interrupt gets called when
  // a character is received.
  uint8_t byte = UART1_rxBuffer[0];

  if (byte == 0xFE && UART1_previousValue == 0xFF) {
    // We've detected the START bytes
    UART1_inMessage = true;
  } else if (UART1_previousValue == 0xFF && byte == 0xFD && UART1_inMessage) {
    // We've detected the STOP byte,
    // and we were in the middle of a message

    // As the current character is the second byte of the STOP bytes,
    // we must remove the first byte of the STOP bytes from our message buffer
    --UART1_msgBufferIdx;

    // We're no longer in a message now, we're at the end
    UART1_inMessage = false;

    // Copy the contents of the message accumulator
    // to the message ready buffer
    for (int i = 0; i < UART1_msgBufferIdx && i < 32; i++) {
      UART1_msgReadyBuffer[i] = UART1_msgAccumulator[i];
    }

    // Set the length and inform that a message is ready
    UART1_msgReadyLength = UART1_msgBufferIdx;
    UART1_msgReady = true;

    // Reset state
    UART1_msgBufferIdx = 0;
    UART1_previousValue = 0;
  } else if (UART1_inMessage) {
    // We're in the middle of a message,
    // so add the received character to the buffer
    UART1_msgAccumulator[UART1_msgBufferIdx++] = byte;

    UART1_previousValue = byte;
  } else {
    // We're not in a message.
    // Just set the previous_value for the next time
    // the interrupt is triggered.

    UART1_previousValue = byte;
  }

  // Start receiving in interrupt mode again
  HAL_UART_Receive_IT(huart, UART1_rxBuffer, 1);
}

/**
 * Handle any data received by the RFM, if there is any.
 * It'll format a packet and transmit it over UART1
 */
void handleRfmRx() {
  if (RFM.datalen > 0) {
    debug_print("Received message from radio");

    // Clear the buffer
    for(int i = 0; i < UartTxBufSize; i++) {
      UART1_txBuffer[i] = 0x0;
    }

    // Set the START bytes for the UART payload
    UART1_txBuffer[0] = 0xFF;
    UART1_txBuffer[1] = 0xFE;

    // Copy the data from the RFM's RX buffer to the UART TX buffer,
    // after the START bytes.
    for (int i = 0; i < RFM.datalen && i < UartTxBufSize - 4; i++) {
      UART1_txBuffer[i + 2] = RFM.data[i];
    }

    // Set the STOP bytes after the data.
    UART1_txBuffer[RFM.datalen + 2] = 0xFF;
    UART1_txBuffer[RFM.datalen + 3] = 0xFD;

    // Transmit over UART1
    int length = RFM.datalen + 4 > UartTxBufSize ? UartTxBufSize : RFM.datalen + 4;
    HAL_UART_Transmit(&huart1, UART1_txBuffer, length, HAL_MAX_DELAY);

    // Reset the datalen for the next reception
    RFM.datalen = 0;
  }
}

/**
 * Handle data received from UART1, if there is any.
 * It'll send the data over the RFM.
 */
void handleUart1Rx() {
  // Transmit a payload over the radio
  // received from the controller
  if (UART1_msgReady) {
    debug_print("Received message from controller");
    debug_print("Will send: ");
    for(int i = 0; i < UART1_msgReadyLength; i++) {
      char buf[5] = { 0 };
      sprintf(buf, "0x%X", UART1_msgReadyBuffer[i] );
      debug_print(buf);
    }


    send(&RFM, RfmOwnId, RfmDeviceID, UART1_msgReadyBuffer, UART1_msgReadyLength);

    // Reset UART1 state
    UART1_msgReady = false;

    // Start receiving again with the RFM
    receive_begin(&RFM);
  }
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
  MX_USART3_UART_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  RFM = get_rfm(&hspi1, RFM69_NSS_GPIO_Port, RFM69_NSS_Pin, RfmOwnId);
  HAL_StatusTypeDef result = init(&RFM, RfmNetworkId);
  if(result != HAL_OK) {
    debug_print("Failed to init RFM");
    Error_Handler();
    return 1;
  }

  HAL_UART_Receive_IT(&huart1, UART1_rxBuffer, 1);
  receive_begin(&RFM);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    handleRfmRx();
    handleUart1Rx();
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL3;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  huart1.Init.BaudRate = 9600;
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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(External_LED_GPIO_Port, External_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Debug_LED_GPIO_Port, Debug_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PH0 PH1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA2 PA3
                           PA4 PA8 PA12 RFM69_NSS_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_8|GPIO_PIN_12|RFM69_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB12 PB13
                           PB14 PB15 PB3 PB4
                           PB5 PB6 PB7 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_12|GPIO_PIN_13
                          |GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : External_LED_Pin */
  GPIO_InitStruct.Pin = External_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(External_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Debug_LED_Pin */
  GPIO_InitStruct.Pin = Debug_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Debug_LED_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

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
  while (1) {
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
