#include "app_uart.h"
#include "usart.h"
#include "hmi_protocol.h"
#include "vofa_debug.h"
#include "k230.h"

/*
 * 统一 UART 单字节中断分发。
 *
 * 当前已接入：
 *   USART1 -> HWT101
 *   USART2 -> HMI 协议解析（PA3 RX / PA2 TX，当前链路验证专用）
 *   UART5  -> VOFA 调试接收（10.7.2 需先 0x70 人工接管后才允许运动命令）
 *
 * 注意：
 *   本测试版暂时把 USART2 独占给 HMI，用于验证整条串口屏链路。
 *   Reference_Original_F4_Modules 中的 MJ6000/GM67 代码未加入当前 Keil 编译；
 *   后续若重新启用 GM67，必须先重新分配串口资源，不能与 HMI 共用 USART2。
 *
 * 后续 UART4(K230) 等模块接入时继续在这里统一分发，
 * 不要在其它模块再次定义 HAL_UART_RxCpltCallback()，避免回调互相覆盖。
 */
static uint8_t s_rx1;
static uint8_t s_rx4;
static uint8_t s_rx_hmi;
static uint8_t s_rx5;
static Hwt101 s_imu;

/*
 * USART2 / HMI 原始接收诊断。
 *
 * 当前链路验证接线：
 *   HMI TX -> PA3 / USART2_RX
 *   HMI RX <- PA2 / USART2_TX
 *
 * 注意：
 * 这里只统计“UART有没有收到字节”，不判断协议是否合法。
 * 协议是否合法由 hmi_protocol.c 的 hmi_proto_* 变量继续判断。
 */
volatile uint32_t hmi_usart2_rx_count = 0U;
volatile uint32_t hmi_usart2_rearm_ok_count = 0U;
volatile uint32_t hmi_usart2_rearm_fail_count = 0U;
volatile uint32_t hmi_usart2_error_count = 0U;
volatile uint32_t hmi_usart2_last_error = 0U;
volatile uint8_t  hmi_usart2_last_byte = 0U;

static void arm_rx(UART_HandleTypeDef *huart, uint8_t *byte)
{
    HAL_StatusTypeDef st = HAL_UART_Receive_IT(huart, byte, 1U);

    /*
     * 只对 HMI / USART2 记录挂接结果。
     * 正常情况下初始化和每次收完 1 字节以后都应该重新得到 HAL_OK。
     */
    if (huart == &huart2)
    {
        if (st == HAL_OK)
            hmi_usart2_rearm_ok_count++;
        else
            hmi_usart2_rearm_fail_count++;
    }
}

void AppUart_Init(void)
{
    hwt101_init(&s_imu);

    /*
     * 各外设的 MX_xxx_Init() 必须先在 main() 中完成，再启动 1 byte IT 接收。
     * HMI 使用 USART2：PA3 接收的字节直接喂给 hmi_protocol.c，
     * 主循环只消费协议层已经解析好的事件。
     */
    arm_rx(&huart1, &s_rx1);
		arm_rx(&huart2, &s_rx_hmi);
		arm_rx(&huart4, &s_rx4);
		arm_rx(&huart5, &s_rx5);
}

Hwt101 *AppUart_GetImu(void)
{
    return &s_imu;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uint32_t now = HAL_GetTick();

    if (huart == &huart1)
    {
        hwt101_feed_byte(&s_imu, s_rx1, now);
        arm_rx(&huart1, &s_rx1);
    }
    else if (huart == &huart2)
    {
        /*
         * HMI 原始字节诊断：
         * 先记录 PA3 / USART2_RX 实际收到的字节，再交给协议层。
         * 如果 hmi_usart2_rx_count 始终为 0，问题仍在协议层以前。
         */
        hmi_usart2_rx_count++;
        hmi_usart2_last_byte = s_rx_hmi;

        /* HMI: AA 55 ... 字节流由协议层自行组帧、校验并生成地图/控制事件。 */
        HMI_Protocol_RxByte(s_rx_hmi);
        arm_rx(&huart2, &s_rx_hmi);
    }
		else if (huart == &huart4)
		{
				K230_FeedByte(s_rx4);
				arm_rx(&huart4, &s_rx4);
		}
    else if (huart == &huart5)
    {
        /*
         * 10.7.2 同时运行 VofaDebug_Update()，但运动命令由 0x70 人工接管门控。
         * 未接管时 VOFA 不会与 RobotApp 争用底盘。
         */
        VofaDebug_FeedByte(s_rx5);
        arm_rx(&huart5, &s_rx5);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (!huart) return;

    /*
     * HAL 进入 ErrorCallback 时 ErrorCode 仍保留本次错误原因。
     * USART2 单独记录，便于区分“根本没收到”和“收到但持续串口错误”。
     */
    if (huart == &huart2)
    {
        hmi_usart2_error_count++;
        hmi_usart2_last_error = huart->ErrorCode;
    }

    __HAL_UART_CLEAR_PEFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_OREFLAG(huart);
    (void)HAL_UART_AbortReceive(huart);

    if (huart == &huart1)
        arm_rx(&huart1, &s_rx1);
    else if (huart == &huart2)
        arm_rx(&huart2, &s_rx_hmi);
		else if (huart == &huart4)
				arm_rx(&huart4, &s_rx4);
    else if (huart == &huart5)
        arm_rx(&huart5, &s_rx5);
}
