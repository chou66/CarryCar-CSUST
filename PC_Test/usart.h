#ifndef __PC_TEST_USART_H
#define __PC_TEST_USART_H

/* 主机单元测试只需要UART句柄声明；真实初始化由STM32工程的Core/Inc/usart.h提供。 */
#include "main.h"

extern UART_HandleTypeDef hlpuart1;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;

#endif
