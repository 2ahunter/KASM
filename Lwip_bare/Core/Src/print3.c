/*
 * print3.c
 *
 *  Created on: Mar 11, 2026
 *      Author: aaronhunter
 */

#include <stdint.h>
#include<string.h>
#include "main.h"

extern UART_HandleTypeDef huart3; // default handle for print_uart3()
uint16_t size = { 0 };
uint32_t timeout = HAL_MAX_DELAY;
HAL_StatusTypeDef status;

HAL_StatusTypeDef print_uart3(const char *pData) {
	status = HAL_UART_Transmit(&huart3, (uint8_t*) pData, strlen(pData),
			timeout);
	return (status);
}
