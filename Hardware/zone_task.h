#ifndef __ZONE_TASK_H
#define __ZONE_TASK_H

#include "main.h"

#include <stdbool.h>
#include <stdint.h>

/**
 * @file zone_task.h
 * @brief 区域机械动作调度：把“某个区域、某一批”展开成一串机械臂步骤。
 *
 * 顶层 mission 到达作业节点后只需调用：
 * @code
 * ZoneTask_Start(ZONE_RAW, 0);
 * @endcode
 * 随后由主循环持续调用 ZoneTask_Update()。
 *
 * 每个区域、每个批次对应一张步骤表（见 zone_task.c 的 s_zone_program）：
 *   原料区   = 3 步  原料台 -> 料盘
 *   粗加工区 = 6 步  料盘 -> 三个工位（放料），再三个工位 -> 料盘（取回）
 *   暂存区   = 3 步  料盘 -> 三个暂存位（第二批 layer=1，叠放到第一层上）
 *
 * 原料区三步的 align_color 非 0：每步在夹取之前先做一轮 K230 视觉对位
 * （转夹爪到统一识别位 POS2 + 张开 -> 滑台前伸到识别位 -> 发 detect,<颜色号>
 * -> 云台与滑台二维位置闭环 -> 滑台退回原点），识别到对应颜色才夹取。
 * 注意识别前那两件事的顺序是固定的「云台先转、滑台后伸」：夹爪在缩回
 * 状态下回转更不容易和外围干涉；两件都就位后才发 detect。
 * 原料区三步的表内 pose 以 ZONE_RAW_POS2_POSE 为初值；对准完成后会把
 * 实际云台角度与滑台位置一起写入本步夹取 pose，从原点重放同一位姿。
 * 底盘在三块料之间保持不动。
 * 对位失败则整个区域任务失败停车；此时滑台可能停在半路，位置基准作废，
 * 需人工把滑台推回原点并调用 Manipulator_Slide_SetHome() 才能重新启动。
 * （云台预转阶段失败也同样作废基准——急停语义下不假设滑台没被碰过。）
 * 粗加工区/暂存区的对位由 mission 在整区域动作之前做一次，步骤里为 0。
 * 其中粗加工区固定只识别2号圆环来修正底盘；完成后1号、3号依靠已标定的
 * 固定云台角和齿条长度放置/取回，不会在六个机械步骤中再次启动视觉。
 *
 * 步骤表与所有位姿标定都集中在 zone_task.c / robot_config.h，
 * 本模块不含任何机械尺寸数字。
 */

typedef enum
{
    ZONE_RAW = 0,       /* 原料区：原料台取料放进料盘 */
    ZONE_ROUGH,         /* 粗加工区：料盘放料到工位，加工完再取回 */
    ZONE_STORAGE,       /* 暂存区：料盘放料到暂存位（第二批叠放） */
    ZONE_ID_COUNT
} ZoneId_t;

typedef enum
{
    ZONE_TASK_IDLE = 0,
    ZONE_TASK_MOVE_TRAY,     /* 料盘选择舵机转到本步槽位 */
    ZONE_TASK_WAIT_TRAY,     /* 等料盘稳定 */
    ZONE_TASK_ALIGN,         /* 视觉对位（仅 align_color != 0 的步骤） */
    ZONE_TASK_WAIT_ALIGN,    /* 等对位结束才允许夹 */
    ZONE_TASK_START_ACTION,  /* 启动一步机械臂动作 */
    ZONE_TASK_WAIT_ACTION,   /* 等这一步动作结束 */
    ZONE_TASK_DONE,
    ZONE_TASK_ERROR,
    /* 追加在末尾，保持前面所有枚举值不变。 */
    ZONE_TASK_SLIDE_OUT,       /* 识别前：云台先转到识别位并张开，滑台再前伸 */
    ZONE_TASK_WAIT_SLIDE_OUT,
    ZONE_TASK_SLIDE_HOME,      /* 识别后：滑台退回原点，夹取仍从原点出发 */
    ZONE_TASK_WAIT_SLIDE_HOME
} ZoneTaskState_t;

void ZoneTask_Init(void);

/**
 * @brief 启动某个区域某一批次的机械动作序列。
 * @param zone  区域（ZONE_RAW / ZONE_ROUGH / ZONE_STORAGE）
 * @param batch 批次：0=第一批，1=第二批（暂存区第二批为叠放）
 * @return false 表示参数非法或当前有动作未结束；此时不改变任何状态。
 */
bool ZoneTask_Start(ZoneId_t zone, uint8_t batch);

void ZoneTask_Update(uint32_t now_ms);
void ZoneTask_Abort(void);

bool ZoneTask_IsBusy(void);
bool ZoneTask_IsDone(void);
bool ZoneTask_HasError(void);

/* 诊断：当前步骤序号（0-based）与本次序列的总步数。 */
uint8_t ZoneTask_GetStep(void);
uint8_t ZoneTask_GetStepCount(void);

/*
 * 状态镜像（非 static 全局，Keil Watch 直接输名字即可，无需限定名）：
 *   robot_zone_task_state       ZoneTaskState_t 当前值
 *   robot_zone_task_step        当前步骤序号（0-based）
 *   robot_zone_task_step_count  本次序列总步数
 *   robot_zone_task_error_step  失败时停在第几步
 *   robot_zone_task_zone/batch  本次任务的区域与批次
 * 由 zone_task.c 的 sync_task_state() 在状态转换与每帧刷新。
 */
extern volatile uint8_t robot_zone_task_state;
extern volatile uint8_t robot_zone_task_step;
extern volatile uint8_t robot_zone_task_step_count;
extern volatile uint8_t robot_zone_task_error_step;
extern volatile uint8_t robot_zone_task_zone;
extern volatile uint8_t robot_zone_task_batch;

#endif
