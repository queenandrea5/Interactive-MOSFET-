/*
 * logger_lpuart.h
 *
 *  Created on: Nov 6, 2025
 *      Author: PC
 */

#ifndef INC_LOGGER_LPUART_H_
#define INC_LOGGER_LPUART_H_

#include "main.h"

#define LPUART_LOG_BUFFER_SIZE 512

void lpuart_log_init(void* huart);
void lpuart_log_printf(const char* format, ...);

#endif /* INC_LOGGER_LPUART_H_ */
