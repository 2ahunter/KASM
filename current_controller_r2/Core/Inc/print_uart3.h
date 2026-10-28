/*
 * print_uart3.h
 *
 *  Created on: Mar 11, 2026
 *      Author: aaronhunter
 */

#ifndef INC_PRINT_UART3_H_
#define INC_PRINT_UART3_H_
/* wrapper for HAL_UART_Transmit */
#include "stm32h7xx_hal.h"

HAL_StatusTypeDef print_uart3(const char *pData);


#endif /* INC_PRINT_UART3_H_ */