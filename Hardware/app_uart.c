#include "app_uart.h"
#include "usart.h"
#include "hmi_protocol.h"
#include "vofa_debug.h"
#include "k230.h"
#include "qr_scanner.h"

/*
 * 统一 UART 单字节中断分发。
 *
 * 当前已接入：
 *   USART1 -> HWT101
 *   USART2 -> GM67/MJ6000 二维码模块（PA3 RX / PA2 TX）
 *   USART3 -> HMI 协议解析（PE15 RX / PB10 TX）
 *   UART4  -> K230
 *   UART5  -> VOFA 调试接收（10.7.2 需先 0x70 人工接管后才允许运动命令）
 *
 * 注意：
 *   HMI 已从 USART2 迁到 USART3，USART2 现在专供扫码模块。
 *
 * 后续新模块接入时继续在这里统一分发，
 * 不要在其它模块再次定义 HAL_UART_RxCpltCallback()，避免回调互相覆盖。
 */
static uint8_t s_rx1;
static uint8_t s_rx2;
static uint8_t s_rx4;
static uint8_t s_rx_hmi;
static uint8_t s_rx5;
static Hwt101 s_imu;

/*
 * HWT101 零偏自校准。
 *
 * HWT101 是纯积分单轴陀螺，零偏会直接积分成航向漂移。模块支持
 * “自动获取零偏”模式：静止约 20 s 让固件重新估计零偏，再退出。
 * 这里只负责发指令和计时，静止由调用者保证；一旦底盘收到运动命令，
 * 说明校准条件已被破坏，立即退出并标记中止，避免写入一个错误的零偏。
 */
#define IMU_ZERO_BIAS_CAL_DEFAULT_MS     20000U
#define IMU_ZERO_BIAS_CAL_MIN_MS          5000U
#define IMU_ZERO_BIAS_CAL_MAX_MS        120000U
#define IMU_ZERO_BIAS_CAL_TX_TIMEOUT_MS     20U

#define IMU_ZERO_BIAS_CAL_IDLE              0U
#define IMU_ZERO_BIAS_CAL_RUNNING           1U
#define IMU_ZERO_BIAS_CAL_DONE              2U
#define IMU_ZERO_BIAS_CAL_ABORTED_MOTION    3U
#define IMU_ZERO_BIAS_CAL_UART_ERROR        4U

static const uint8_t s_imu_cal_enter_cmd[5] = {0xFFU, 0xAAU, 0x48U, 0x01U, 0x00U};
static const uint8_t s_imu_cal_exit_cmd[5]  = {0xFFU, 0xAAU, 0x48U, 0x00U, 0x00U};

volatile uint8_t imu_debug_zero_bias_cal_state = IMU_ZERO_BIAS_CAL_IDLE;
volatile uint32_t imu_debug_zero_bias_cal_elapsed_ms = 0U;
volatile uint32_t imu_debug_zero_bias_cal_total_ms = 0U;

static uint32_t s_imu_cal_start_ms;
static uint32_t s_imu_cal_total_ms;

/* HWT101 滤波诊断镜像：避免 Keil Watch 依赖本文件 static 符号解析。 */
volatile float imu_debug_yaw_raw_deg = 0.0f;
volatile float imu_debug_yaw_filtered_deg = 0.0f;
volatile float imu_debug_gyro_z_dps = 0.0f;
volatile uint8_t imu_debug_yaw_hold_active = 0U;
volatile uint8_t imu_debug_platform_moving = 0U;
volatile uint32_t imu_debug_hold_enter_count = 0U;
volatile uint32_t imu_debug_hold_exit_count = 0U;

static void update_imu_debug_mirror(void)
{
    imu_debug_yaw_raw_deg = s_imu.yaw_raw_deg;
    imu_debug_yaw_filtered_deg = s_imu.yaw_deg;
    imu_debug_gyro_z_dps = s_imu.gyro_z_dps;
    imu_debug_yaw_hold_active = s_imu.yaw_hold_active;
    imu_debug_platform_moving = s_imu.platform_moving;
    imu_debug_hold_enter_count = s_imu.yaw_hold_enter_count;
    imu_debug_hold_exit_count = s_imu.yaw_hold_exit_count;
}

/*
 * USART3 / HMI 原始接收诊断。
 *
 * 当前接线：
 *   HMI TX -> PE15 / USART3_RX
 *   HMI RX <- PB10 / USART3_TX
 *
 * 注意：
 * 这里只统计“UART有没有收到字节”，不判断协议是否合法。
 * 协议是否合法由 hmi_protocol.c 的 hmi_proto_* 变量继续判断。
 */
volatile uint32_t hmi_usart3_rx_count = 0U;
volatile uint32_t hmi_usart3_rearm_ok_count = 0U;
volatile uint32_t hmi_usart3_rearm_fail_count = 0U;
volatile uint32_t hmi_usart3_error_count = 0U;
volatile uint32_t hmi_usart3_last_error = 0U;
volatile uint8_t  hmi_usart3_last_byte = 0U;

/* USART2 / QR 原始接收诊断。 */
volatile uint32_t qr_usart2_rx_count = 0U;
volatile uint32_t qr_usart2_rearm_ok_count = 0U;
volatile uint32_t qr_usart2_rearm_fail_count = 0U;
volatile uint32_t qr_usart2_error_count = 0U;
volatile uint32_t qr_usart2_last_error = 0U;
volatile uint8_t  qr_usart2_last_byte = 0U;

/* UART4 / K230 原始接收诊断：只统计链路，不参与协议和任务判断。 */
volatile uint32_t k230_uart4_rx_count = 0U;
volatile uint32_t k230_uart4_rearm_ok_count = 0U;
volatile uint32_t k230_uart4_rearm_fail_count = 0U;
volatile uint32_t k230_uart4_error_count = 0U;
volatile uint32_t k230_uart4_last_error = 0U;
volatile uint8_t  k230_uart4_last_byte = 0U;

static void arm_rx(UART_HandleTypeDef *huart, uint8_t *byte)
{
    HAL_StatusTypeDef st = HAL_UART_Receive_IT(huart, byte, 1U);

    /*
     * 对扫码、HMI、K230记录挂接结果。
     * 正常情况下初始化和每次收完 1 字节以后都应该重新得到 HAL_OK。
     */
    if (huart == &huart2)
    {
        if (st == HAL_OK)
            qr_usart2_rearm_ok_count++;
        else
            qr_usart2_rearm_fail_count++;
    }
    else if (huart == &huart3)
    {
        if (st == HAL_OK)
            hmi_usart3_rearm_ok_count++;
        else
            hmi_usart3_rearm_fail_count++;
    }
    else if (huart == &huart4)
    {
        if (st == HAL_OK)
            k230_uart4_rearm_ok_count++;
        else
            k230_uart4_rearm_fail_count++;
    }
}

void AppUart_Init(void)
{
    hwt101_init(&s_imu);
    QRScanner_Init();

    /*
     * 各外设的 MX_xxx_Init() 必须先在 main() 中完成，再启动 1 byte IT 接收。
     * HMI 使用 USART3：PE15 接收的字节直接喂给 hmi_protocol.c，
     * 主循环只消费协议层已经解析好的事件。
     */
    arm_rx(&huart1, &s_rx1);
    arm_rx(&huart2, &s_rx2);
    arm_rx(&huart3, &s_rx_hmi);
    arm_rx(&huart4, &s_rx4);
    arm_rx(&huart5, &s_rx5);
}

Hwt101 *AppUart_GetImu(void)
{
    return &s_imu;
}

static uint8_t imu_command_send(const uint8_t *cmd)
{
    return (HAL_UART_Transmit(&huart1, (uint8_t *)cmd, 5U,
                              IMU_ZERO_BIAS_CAL_TX_TIMEOUT_MS) == HAL_OK) ? 1U : 0U;
}

static void imu_zero_bias_cal_finish(uint8_t final_state)
{
    /* 只有真正处于校准模式时才需要退出指令，避免干扰模块正常输出。 */
    if (!imu_command_send(s_imu_cal_exit_cmd))
    {
        final_state = IMU_ZERO_BIAS_CAL_UART_ERROR;
    }

    imu_debug_zero_bias_cal_state = final_state;
}

uint8_t AppUart_ImuZeroBiasCalStart(uint32_t now_ms, uint32_t duration_ms)
{
    if (imu_debug_zero_bias_cal_state == IMU_ZERO_BIAS_CAL_RUNNING)
        return 0U;

    /* 底盘仍有运动命令时不允许开始，否则零偏会被运动污染。 */
    if (s_imu.platform_moving)
        return 0U;

    if (duration_ms == 0U)
        duration_ms = IMU_ZERO_BIAS_CAL_DEFAULT_MS;
    if (duration_ms < IMU_ZERO_BIAS_CAL_MIN_MS)
        duration_ms = IMU_ZERO_BIAS_CAL_MIN_MS;
    if (duration_ms > IMU_ZERO_BIAS_CAL_MAX_MS)
        duration_ms = IMU_ZERO_BIAS_CAL_MAX_MS;

    if (!imu_command_send(s_imu_cal_enter_cmd))
    {
        imu_debug_zero_bias_cal_state = IMU_ZERO_BIAS_CAL_UART_ERROR;
        return 0U;
    }

    s_imu_cal_start_ms = now_ms;
    s_imu_cal_total_ms = duration_ms;
    imu_debug_zero_bias_cal_elapsed_ms = 0U;
    imu_debug_zero_bias_cal_total_ms = duration_ms;
    imu_debug_zero_bias_cal_state = IMU_ZERO_BIAS_CAL_RUNNING;
    return 1U;
}

void AppUart_ImuZeroBiasCalAbort(void)
{
    if (imu_debug_zero_bias_cal_state != IMU_ZERO_BIAS_CAL_RUNNING)
        return;

    imu_zero_bias_cal_finish(IMU_ZERO_BIAS_CAL_IDLE);
}

void AppUart_Update(uint32_t now_ms)
{
    uint32_t elapsed_ms;

    if (imu_debug_zero_bias_cal_state != IMU_ZERO_BIAS_CAL_RUNNING)
        return;

    elapsed_ms = (uint32_t)(now_ms - s_imu_cal_start_ms);
    imu_debug_zero_bias_cal_elapsed_ms = elapsed_ms;

    /*
     * 校准要求整车静止。底盘一旦收到运动命令就立即中止：此时固件可能
     * 已经把运动样本当成零偏，继续等待只会写入更差的结果。
     */
    if (s_imu.platform_moving)
    {
        imu_zero_bias_cal_finish(IMU_ZERO_BIAS_CAL_ABORTED_MOTION);
        return;
    }

    if (elapsed_ms >= s_imu_cal_total_ms)
    {
        imu_zero_bias_cal_finish(IMU_ZERO_BIAS_CAL_DONE);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uint32_t now = HAL_GetTick();

    if (huart == &huart1)
    {
        hwt101_feed_byte(&s_imu, s_rx1, now);
        update_imu_debug_mirror();
        arm_rx(&huart1, &s_rx1);
    }
    else if (huart == &huart2)
    {
        /*
         * 扫码模块始终开启，因此USART2始终保持1字节中断接收。
         * 这里只记录原始链路并把字节交给QRScanner；任务码是否合法、是否
         * 应当开始任务由主循环中的RobotApp判断，中断里不推进机械状态机。
         */
        qr_usart2_rx_count++;
        qr_usart2_last_byte = s_rx2;
        QRScanner_FeedByte(s_rx2, now);
        arm_rx(&huart2, &s_rx2);
    }
    else if (huart == &huart3)
    {
        /*
         * HMI 原始字节诊断：
         * 先记录 PE15 / USART3_RX 实际收到的字节，再交给协议层。
         * 如果 hmi_usart3_rx_count 始终为 0，问题仍在协议层以前。
         */
        hmi_usart3_rx_count++;
        hmi_usart3_last_byte = s_rx_hmi;

        /* HMI: AA 55 ... 字节流由协议层自行组帧、校验并生成地图/控制事件。 */
        HMI_Protocol_RxByte(s_rx_hmi);
        arm_rx(&huart3, &s_rx_hmi);
    }
    else if (huart == &huart4)
    {
        k230_uart4_rx_count++;
        k230_uart4_last_byte = s_rx4;
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
     * USART3 单独记录，便于区分“根本没收到”和“收到但持续串口错误”。
     */
    if (huart == &huart2)
    {
        qr_usart2_error_count++;
        qr_usart2_last_error = huart->ErrorCode;
    }
    else if (huart == &huart3)
    {
        hmi_usart3_error_count++;
        hmi_usart3_last_error = huart->ErrorCode;
    }
    else if (huart == &huart4)
    {
        k230_uart4_error_count++;
        k230_uart4_last_error = huart->ErrorCode;
    }

    __HAL_UART_CLEAR_PEFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_OREFLAG(huart);
    (void)HAL_UART_AbortReceive(huart);

    if (huart == &huart1)
        arm_rx(&huart1, &s_rx1);
    else if (huart == &huart2)
        arm_rx(&huart2, &s_rx2);
    else if (huart == &huart3)
        arm_rx(&huart3, &s_rx_hmi);
    else if (huart == &huart4)
        arm_rx(&huart4, &s_rx4);
    else if (huart == &huart5)
        arm_rx(&huart5, &s_rx5);
}
