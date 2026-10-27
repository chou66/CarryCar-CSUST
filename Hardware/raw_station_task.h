#ifndef __RAW_STATION_TASK_H
#define __RAW_STATION_TASK_H

#include "chassis.h"
#include "main.h"

#include <stdbool.h>
#include <stdint.h>

/**
 * @file raw_station_task.h
 * @brief 原料站完整事务：临时停靠、三次视觉抓取、撤回导航节点。
 *
 * RobotApp 只启动并等待本任务，不再展开原料区的底盘与机械臂子状态。
 * 本任务完成时车体已经撤回 N8，但不宣称航向仍精确；RobotApp 随后必须
 * 重新执行任务航向校正，校正成功后才能进入下一段导航。
 */
typedef enum
{
    RAW_STATION_IDLE = 0,
    RAW_STATION_DOCKING,       /* 车体 +X 靠近圆盘，同时开始滑台前伸 */
    RAW_STATION_DOCK_SETTLE,   /* 底盘报告到位后的结构稳定时间 */
    RAW_STATION_START_ZONE,    /* 启动本批次的三步原料 ZoneTask */
    RAW_STATION_RUN_ZONE,      /* 等待三次视觉抓取与放入料盘 */
    RAW_STATION_UNDOCKING,     /* 机械臂回 HOME 后车体 -X 返回 N8 */
    RAW_STATION_UNDOCK_SETTLE, /* 返回到位后的稳定时间 */
    RAW_STATION_DONE,          /* 已回 N8，等待 RobotApp 重新校正航向 */
    RAW_STATION_ERROR,         /* 错误码见 RAW_STATION_ERROR_* */
    /* 追加在末尾，保持前面所有枚举值不变：等停靠窗口里并行的滑台前伸到位。 */
    RAW_STATION_WAIT_PRESLIDE
} RawStationTaskState_t;

#define RAW_STATION_ERROR_NONE       0U
#define RAW_STATION_ERROR_PARAM      1U
#define RAW_STATION_ERROR_HOME       2U
#define RAW_STATION_ERROR_DOCK       3U
#define RAW_STATION_ERROR_ZONE_START 4U
#define RAW_STATION_ERROR_ZONE       5U
#define RAW_STATION_ERROR_UNDOCK     6U
#define RAW_STATION_ERROR_ABORTED    7U
#define RAW_STATION_ERROR_PRESLIDE   8U  /* 停靠窗口并行的滑台前伸失败 */

void RawStationTask_Init(Chassis *chassis);
bool RawStationTask_Start(uint8_t batch, uint32_t now_ms);
void RawStationTask_Update(uint32_t now_ms);
void RawStationTask_Abort(void);

bool RawStationTask_IsBusy(void);
bool RawStationTask_IsDone(void);
bool RawStationTask_HasError(void);
uint8_t RawStationTask_GetError(void);

extern volatile uint8_t robot_raw_station_state;
extern volatile uint8_t robot_raw_station_error;
extern volatile uint8_t robot_raw_station_batch;
extern volatile uint8_t robot_raw_station_docked;

#endif
