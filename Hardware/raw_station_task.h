#ifndef __RAW_STATION_TASK_H
#define __RAW_STATION_TASK_H

#include "chassis.h"
#include "main.h"

#include <stdbool.h>
#include <stdint.h>

/**
 * @file raw_station_task.h
 * @brief 原料站完整事务：三次视觉抓取（滑台在到站时预伸）。
 *
 * RobotApp 只启动并等待本任务，不再展开原料区的机械臂子状态。
 * 原料区不做底盘停靠平移；本任务完成时车体仍停在 N8，RobotApp 随后
 * 重新执行任务航向校正作为离开前的安全检查，再进入下一段导航。
 */
typedef enum
{
    RAW_STATION_IDLE = 0,
    /* 1/2 原 DOCKING / DOCK_SETTLE 已删除；保留编号，避免 Keil Watch
     * 与既有调试脚本的数值错位。 */
    RAW_STATION_START_ZONE = 3,    /* 启动本批次的三步原料 ZoneTask */
    RAW_STATION_RUN_ZONE = 4,      /* 等待三次视觉抓取与放入料盘 */
    /* 5/6 原 UNDOCKING / UNDOCK_SETTLE 已删除。 */
    RAW_STATION_DONE = 7,          /* 批次完成，等待 RobotApp 重新校正航向 */
    RAW_STATION_ERROR = 8,         /* 错误码见 RAW_STATION_ERROR_* */
    RAW_STATION_WAIT_PRESLIDE = 9  /* 等到站时启动的滑台预伸到位 */
} RawStationTaskState_t;

#define RAW_STATION_ERROR_NONE       0U
#define RAW_STATION_ERROR_PARAM      1U
#define RAW_STATION_ERROR_HOME       2U
/* 3 原 DOCK 错误已删除。 */
#define RAW_STATION_ERROR_ZONE_START 4U
#define RAW_STATION_ERROR_ZONE       5U
/* 6 原 UNDOCK 错误已删除。 */
#define RAW_STATION_ERROR_ABORTED    7U
#define RAW_STATION_ERROR_PRESLIDE   8U  /* 到站预伸的滑台前伸失败 */

void RawStationTask_Init(Chassis *chassis);
bool RawStationTask_Start(uint8_t batch, uint32_t now_ms);
void RawStationTask_Update(uint32_t now_ms);
void RawStationTask_Abort(void);

bool RawStationTask_IsBusy(void);
bool RawStationTask_IsDone(void);
bool RawStationTask_HasError(void);

extern volatile uint8_t robot_raw_station_state;
extern volatile uint8_t robot_raw_station_error;
extern volatile uint8_t robot_raw_station_batch;
extern volatile uint8_t robot_raw_station_docked;

#endif
