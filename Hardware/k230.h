#ifndef __K230_H
#define __K230_H

#include "main.h"
#include <stdint.h>

#define K230_DATA_TIMEOUT_MS          3000U
#define K230_ERROR_TEXT_MAX             20U
#define K230_TASK_TIMEOUT_MS          60000U

/* K230产生的事件。 */
#define K230_EVENT_NONE       0x00000000UL
#define K230_EVENT_COLOR      0x00000001UL
#define K230_EVENT_OFFSET     0x00000002UL
#define K230_EVENT_GRAB       0x00000004UL
#define K230_EVENT_PLACE      0x00000008UL
#define K230_EVENT_ERROR      0x00000010UL
#define K230_EVENT_HEARTBEAT  0x00000020UL
#define K230_EVENT_TARGET     0x00000040UL
#define K230_EVENT_STOPPED    0x00000080UL

typedef struct
{
    uint8_t valid;

    /* 最近一次识别到的颜色，范围1～6。 */
    uint8_t color_id;

    /* 第二轮叠放时K230确认锁定的位置号，范围1～3。 */
    uint8_t target_id;

    /* a、d、w、s。只保留最近解析到的那一行。 */
    char direction;

    /* 当前协议规定偏差单位为像素。 */
    int32_t offset_px;

    /*
     * 正交双轴偏差（2026-10-12 起的新报文）。
     *
     * K230 现在一帧发两行，例如：
     *     d,136.00
     *     w,102.50
     * 两行分属车体的两个正交轴：
     *     w/s -> ±X（车体右 / 左）
     *     a/d -> ±Y（车体前 / 后）
     * 这两行会在同一个主循环周期里被先后解析，上面那两个字段只能留下
     * 最后一行（上面例子里是 w），d 那一行会被直接覆盖丢掉。所以这里
     * 再按轴各存一份，并分别记下到达时刻，上层才配得成一对。
     *
     * axis_*_dir 为 0 表示该轴至今没收到过偏差行；
     * axis_*_px 只存像素大小，方向由 axis_*_dir 表达（与 offset_px 一致）。
     */
    char     axis_x_dir;    /* 'w' 或 's' */
    int32_t  axis_x_px;
    uint32_t axis_x_ms;     /* 该行到达时的 HAL_GetTick() */
    char     axis_y_dir;    /* 'a' 或 'd' */
    int32_t  axis_y_px;
    uint32_t axis_y_ms;

    /* 最近一次error,xxx中的xxx。 */
    char error_text[K230_ERROR_TEXT_MAX];

    /* Most recently parsed K230_EVENT_xxx value, for non-destructive diagnostics. */
    uint32_t last_event;
    uint32_t timestamp_ms;
    uint32_t message_count;
} K230_Status_t;

/*
 * 一次K230视觉任务的运行模式。
 * DETECT等待grab完成，PLACE等待place完成。
 */
typedef enum
{
    K230_TASK_MODE_NONE = 0,
    K230_TASK_MODE_DETECT,
    K230_TASK_MODE_PLACE
} K230_TaskMode_t;

typedef enum
{
    K230_TASK_IDLE = 0,
    K230_TASK_WAIT_RESULT,
    K230_TASK_DONE,
    K230_TASK_ERROR
} K230_TaskState_t;

#define K230_TASK_ERROR_NONE       0U
#define K230_TASK_ERROR_SEND       1U
#define K230_TASK_ERROR_REMOTE     2U
#define K230_TASK_ERROR_TIMEOUT    3U

/*
 * 当前一次视觉任务的结果快照。
 * requested_id是detect的颜色编号或place的位置编号；
 * offset_pending表示有一条尚未被底盘微调逻辑取走的新偏差。
 */
typedef struct
{
    K230_TaskMode_t mode;
    uint8_t requested_id;

    uint8_t color_valid;
    uint8_t color_id;

    uint8_t offset_pending;
    char direction;
    int32_t offset_px;

    uint8_t error_code;
    char error_text[K230_ERROR_TEXT_MAX];

    uint32_t events_seen;
    uint32_t start_ms;
    uint32_t start_message_count;
    uint32_t last_message_count;
} K230_TaskResult_t;

/*
 * 诊断变量，可直接放进Keil Watch。
 * 应用程序只能读取，不要在外部修改。
 */
extern K230_Status_t g_k230_status;
extern volatile K230_TaskState_t g_k230_task_state;
extern K230_TaskResult_t g_k230_task_result;

extern volatile uint32_t k230_rx_byte_count;
extern volatile uint32_t k230_ring_overflow_count;
extern volatile uint32_t k230_valid_line_count;
extern volatile uint32_t k230_invalid_line_count;

void K230_Init(void);
void K230_FeedByte(uint8_t byte);
void K230_Process(void);

/* 取出并清除尚未处理的事件。 */
uint32_t K230_TakeEvents(void);

uint8_t K230_GetStatus(K230_Status_t *out);
uint8_t K230_IsOnline(uint32_t now_ms);

/* 通用字符串发送。 */
HAL_StatusTypeDef K230_SendString(const char *text);

/* 正式协议命令。 */
HAL_StatusTypeDef K230_StartDetect(uint8_t color_id);
HAL_StatusTypeDef K230_StartPlace(uint8_t position_id);
/* 第二轮暂存区：用第一层anchor_color_id物料确认position_id身份，再找外圆。 */
HAL_StatusTypeDef K230_StartStack(uint8_t position_id, uint8_t anchor_color_id);
HAL_StatusTypeDef K230_Stop(void);

/* 一次非阻塞视觉任务：Start只调用一次，Update需要在主循环持续调用。 */
uint8_t K230_TaskStartDetect(uint8_t color_id);
uint8_t K230_TaskStartPlace(uint8_t position_id);
void K230_TaskUpdate(uint32_t now_ms);
void K230_TaskAbort(void);

uint8_t K230_TaskIsBusy(void);
uint8_t K230_TaskIsDone(void);
uint8_t K230_TaskHasError(void);

/* 取出一条新偏差并清除offset_pending；不会清除最终任务结果。 */
uint8_t K230_TaskTakeOffset(char *direction, int32_t *offset_px);
uint8_t K230_TaskGetResult(K230_TaskResult_t *out);

#endif
