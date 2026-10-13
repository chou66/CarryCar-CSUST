#ifndef __APP_UART_H
#define __APP_UART_H

#include "hwt101_uart.h"

/*
 * 应用层 UART 接收统一入口。
 *
 * 当前正式/测试接线：
 *   USART1 -> HWT101
 *   USART2 -> HMI（PA3 RX / PA2 TX，当前链路验证专用）
 *   UART5  -> VOFA（保留调试入口）
 *
 * 重要：
 *   当前测试版 USART2 暂时由 HMI 独占。
 *   如果后续启用 GM67/MJ6000，必须重新分配串口，不能同时占用 USART2。
 *
 * HAL_UART_RxCpltCallback() 只允许在 app_uart.c 中维护一份。
 */
void AppUart_Init(void);
Hwt101 *AppUart_GetImu(void);


/*
 * HMI / USART2 接收诊断变量。
 *
 * 这些变量只用于 Keil Watch，不参与控制逻辑。
 * 用途：
 *   - 判断 USART2 的 1-byte IT 接收是否成功挂起；
 *   - 判断 PA3 / USART2_RX 是否真的收到来自串口屏 TX 的字节；
 *   - 判断是否发生 UART ORE/FE/NE/PE 等错误。
 */
extern volatile uint32_t hmi_usart2_rx_count;
extern volatile uint32_t hmi_usart2_rearm_ok_count;
extern volatile uint32_t hmi_usart2_rearm_fail_count;
extern volatile uint32_t hmi_usart2_error_count;
extern volatile uint32_t hmi_usart2_last_error;
extern volatile uint8_t  hmi_usart2_last_byte;

#endif
