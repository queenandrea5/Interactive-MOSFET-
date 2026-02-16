/*
 * logger_lpuart.c
 *
 *  Created on: Nov 6, 2025
 *      Author: PC
 */


#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "main.h"
#include "logger_lpuart.h"

#define LPUART_LOG_TIMEOUT 100

UART_HandleTypeDef *lpuart_log_huart;
static char lpuart_log_buffer[LPUART_LOG_BUFFER_SIZE];

// 🔧 Initialise le handle de l’UART utilisé (ici : LPUART1)
void lpuart_log_init(void* huart)
{
    lpuart_log_huart = (UART_HandleTypeDef*)huart;
}

// 🔧 Fonction d’envoi brute
static void lpuart_log_transmit(uint8_t *data, uint16_t data_len)
{
    HAL_UART_Transmit(lpuart_log_huart, data, data_len, LPUART_LOG_TIMEOUT);
}

// ✏️ Fonction printf-like
void lpuart_log_printf(const char* format, ...)
{
    va_list argptr;
    va_start(argptr, format);
    vsnprintf(lpuart_log_buffer, LPUART_LOG_BUFFER_SIZE, format, argptr);
    va_end(argptr);
    lpuart_log_transmit((uint8_t*)lpuart_log_buffer, strlen(lpuart_log_buffer));
}
