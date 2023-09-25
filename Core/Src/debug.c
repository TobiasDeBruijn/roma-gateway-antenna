#include <stdint.h>
#include "debug.h"
#include "stm32l1xx_hal.h"
#include <string.h>

extern UART_HandleTypeDef huart3;

void debug_print(char* text) {
	uint8_t len = strlen(text) + 1;
  HAL_UART_Transmit(&huart3, text, len, HAL_TIMEOUT);
}