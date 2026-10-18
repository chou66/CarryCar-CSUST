#ifndef __APP_UART_H
#define __APP_UART_H

#include "hwt101_uart.h"

/*
 * 应用层 UART 接收统一入口。
 *
 * 当前接线：
 *   USART1 -> HWT101
 *   USART3 -> HMI 串口屏（PE15 RX / PB10 TX）
 *   UART4  -> K230
 *   UART5  -> VOFA（保留调试入口）
 *
 * 重要：
 *   HMI 已从 USART2 迁到 USART3。USART2 现在没有接收方，不再挂
 *   1-byte IT 接收；它对应用层是空闲口，可以留给 GM67/MJ6000 之类的
 *   新外设（LPUART1 也是空的）。
 *
 * HAL_UART_RxCpltCallback() 只允许在 app_uart.c 中维护一份。
 */
void AppUart_Init(void);
Hwt101 *AppUart_GetImu(void);

/* HWT101 停车抗漂移层的 Keil Watch 镜像。 */
extern volatile float imu_debug_yaw_raw_deg;
extern volatile float imu_debug_yaw_filtered_deg;
extern volatile float imu_debug_gyro_z_dps;
extern volatile uint8_t imu_debug_yaw_hold_active;
extern volatile uint8_t imu_debug_platform_moving;
extern volatile uint32_t imu_debug_hold_enter_count;
extern volatile uint32_t imu_debug_hold_exit_count;


/*
 * HMI / USART3 接收诊断变量。
 *
 * 这些变量只用于 Keil Watch，不参与控制逻辑。
 * 用途：
 *   - 判断 USART3 的 1-byte IT 接收是否成功挂起；
 *   - 判断 PE15 / USART3_RX 是否真的收到来自串口屏 TX 的字节；
 *   - 判断是否发生 UART ORE/FE/NE/PE 等错误。
 */
extern volatile uint32_t hmi_usart3_rx_count;
extern volatile uint32_t hmi_usart3_rearm_ok_count;
extern volatile uint32_t hmi_usart3_rearm_fail_count;
extern volatile uint32_t hmi_usart3_error_count;
extern volatile uint32_t hmi_usart3_last_error;
extern volatile uint8_t  hmi_usart3_last_byte;

/* K230 / UART4 原始链路诊断；用于区分协议状态机问题与串口收字节问题。 */
extern volatile uint32_t k230_uart4_rx_count;
extern volatile uint32_t k230_uart4_rearm_ok_count;
extern volatile uint32_t k230_uart4_rearm_fail_count;
extern volatile uint32_t k230_uart4_error_count;
extern volatile uint32_t k230_uart4_last_error;
extern volatile uint8_t  k230_uart4_last_byte;

#endif
