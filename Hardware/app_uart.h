#ifndef __APP_UART_H
#define __APP_UART_H

#include "hwt101_uart.h"

/*
 * 应用层 UART 接收统一入口。
 *
 * 当前接线：
 *   USART1 -> HWT101
 *   USART2 -> GM67/MJ6000 扫码模块（PA3 RX / PA2 TX）
 *   USART3 -> HMI 串口屏（PE15 RX / PB10 TX）
 *   UART4  -> K230
 *   UART5  -> VOFA（保留调试入口）
 *
 * 重要：
 *   HMI 已从 USART2 迁到 USART3。USART2 以 1-byte IT 方式持续接收
 *   无包头包尾的二维码 ASCII 数据。
 *
 * HAL_UART_RxCpltCallback() 只允许在 app_uart.c 中维护一份。
 */
void AppUart_Init(void);
void AppUart_Update(uint32_t now_ms);
Hwt101 *AppUart_GetImu(void);

/*
 * HWT101 零偏自校准（VOFA 命令 0x03）。
 *
 * 发送 FF AA 48 01 00 后模块进入“自动获取零偏”模式，整车必须保持静止
 * 约 20 s；到时自动发送 FF AA 48 00 00 退出。返回值 0 表示拒绝
 * （正在校准 / 底盘仍在运动 / 串口发送失败）。
 *
 * 注意：
 *   - 校准会改变模块后续的零偏估计，应在建立比赛世界零点之前做；
 *   - duration_ms 传 0 使用默认 20000 ms，内部限制在 5 s ~ 120 s；
 *   - 校准过程中一旦检测到底盘运动命令，会立即退出并置为中止状态。
 */
uint8_t AppUart_ImuZeroBiasCalStart(uint32_t now_ms, uint32_t duration_ms);
void AppUart_ImuZeroBiasCalAbort(void);

/* 零偏校准状态镜像：0=空闲, 1=校准中, 2=完成, 3=因底盘运动中止, 4=串口失败。 */
extern volatile uint8_t imu_debug_zero_bias_cal_state;
extern volatile uint32_t imu_debug_zero_bias_cal_elapsed_ms;
extern volatile uint32_t imu_debug_zero_bias_cal_total_ms;

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

/* GM67/MJ6000 / USART2 原始链路诊断。 */
extern volatile uint32_t qr_usart2_rx_count;
extern volatile uint32_t qr_usart2_rearm_ok_count;
extern volatile uint32_t qr_usart2_rearm_fail_count;
extern volatile uint32_t qr_usart2_error_count;
extern volatile uint32_t qr_usart2_last_error;
extern volatile uint8_t  qr_usart2_last_byte;

/* K230 / UART4 原始链路诊断；用于区分协议状态机问题与串口收字节问题。 */
extern volatile uint32_t k230_uart4_rx_count;
extern volatile uint32_t k230_uart4_rearm_ok_count;
extern volatile uint32_t k230_uart4_rearm_fail_count;
extern volatile uint32_t k230_uart4_error_count;
extern volatile uint32_t k230_uart4_last_error;
extern volatile uint8_t  k230_uart4_last_byte;

#endif
