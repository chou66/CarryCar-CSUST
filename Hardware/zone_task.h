#ifndef __ZONE_TASK_H
#define __ZONE_TASK_H

#include "main.h"
#include "task_code.h"

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
 * -> 云台与滑台二维位置闭环），识别到对应颜色才夹取。
 * 注意识别前那两件事的顺序是固定的「云台先转、滑台后伸」：夹爪在缩回
 * 状态下回转更不容易和外围干涉；两件都就位后才发 detect。
 * 第一步的滑台前伸已经由 RawStationTask 在车体 +X 停靠平移的同一窗口里
 * 并行完成，到站后直接发 detect，不再重复前伸。
 * 原料区三步的表内 pose 以 ZONE_RAW_POS2_POSE 为初值；对准完成后会把
 * 实际云台角度与滑台位置一起写入本步抓取 pose。抓取不再先退回原点、
 * 再重新伸出，而是直接从当前对准位下降电机5，见
 * ManipulatorAction_StartToTrayFromAlignedPose()。
 * 底盘在三块料之间保持不动。
 * 对位失败则整个区域任务失败停车；此时滑台可能停在半路，位置基准作废，
 * 需人工把滑台推回原点并调用 Manipulator_Slide_SetHome() 才能重新启动。
 * （云台预转阶段失败也同样作废基准——急停语义下不假设滑台没被碰过。）
 * 粗加工区/暂存区的对位由 mission 在整区域动作之前做一次，步骤里为 0。
 * 其中粗加工区固定只识别2号圆环来修正底盘；完成后1/2/3号均依靠已标定
 * 的夹爪云台角和齿条长度放置/取回，不会在六个机械步骤中再次启动视觉。
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
    ZONE_TASK_MOVE_TRAY,     /* 料盘与云台/夹爪同时转向本步目标 */
    ZONE_TASK_WAIT_TRAY,     /* 等料盘和舵机都稳定 */
    ZONE_TASK_ALIGN,         /* 视觉对位（仅 align_color != 0 的步骤） */
    ZONE_TASK_WAIT_ALIGN,    /* 等对位结束才允许夹 */
    ZONE_TASK_START_ACTION,  /* 启动一步机械臂动作 */
    ZONE_TASK_WAIT_ACTION,   /* 等这一步动作结束 */
    ZONE_TASK_DONE,
    ZONE_TASK_ERROR,
    /* 追加在末尾，保持前面所有枚举值不变。 */
    ZONE_TASK_SLIDE_OUT,       /* 识别前：云台先转到识别位并张开，滑台再前伸 */
    ZONE_TASK_WAIT_SLIDE_OUT,
    ZONE_TASK_SLIDE_HOME,      /* 保留状态值；原料视觉对准后已改为直接抓取 */
    ZONE_TASK_WAIT_SLIDE_HOME
} ZoneTaskState_t;

void ZoneTask_Init(void);

/*
 * 在启动原料区、粗加工区、暂存区动作前装载已严格校验的二维码任务。
 * ZoneTask内部会复制整份配置，后续连续扫码不会改变正在执行的搬运顺序。
 */
bool ZoneTask_SetTaskCode(const MissionTaskCode_t *task);
void ZoneTask_ClearTaskCode(void);

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
uint8_t ZoneTask_GetCurrentColor(void);

/**
 * @brief 查询某区域某批次第一步要用的料盘槽位，供行驶途中提前转料盘。
 *
 * 只能在 ZoneTask_SetTaskCode() 之后查询：步骤表的槽位在装载任务码时被
 * 二维码覆盖，之后不再改变，因此这里查到的就是到站后真实使用的槽位。
 * 本函数只读表，不改变任何运行状态，可以在 ZoneTask 未启动时调用。
 *
 * @param slot 输出槽位 0..2（三张表的首步槽位当前都是 0，但这里不写死，
 *             改步骤表后自动跟随）。
 * @return false 表示区域/批次非法或 table 为空。
 */
bool ZoneTask_GetFirstTraySlot(ZoneId_t zone, uint8_t batch, uint8_t *slot);

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
