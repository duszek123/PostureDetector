/*
 * uart.c
 *
 *  Created on: Feb 7, 2021
 *      Author: Pawel
 */
#include "uart.h"

extern UART_HandleTypeDef huart2;

void uart_put_string(char *info)
{
	int16_t len = strlen(info);
	HAL_UART_Transmit(&huart2, (uint8_t *)info,len, 100);
}
