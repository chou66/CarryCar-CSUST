#ifndef __K230_ALIGN_H
#define __K230_ALIGN_H

#include "main.h"
#include "chassis.h"

#include <stdbool.h>
#include <stdint.h>

/**
 * @file k230_align.h
 * @brief K230 视觉对位：把底盘微调到夹爪正对目标。
 *
 * 调用方式（每个作业点对一次，或由 zone_task 在每一步之前各对一次）：
 * @code
 * K230Align_Start(K230_ALIGN_MODE_DETECT, color_id, carousel_us, now_ms);
 * @endcode
 * 随后由主循环持续调用 K230Align_Update()。
 *
 * 这是一个闭环，不是一次性动作：
 *   转夹爪到指定姿态并张开 -> 发 detect/place -> 等一帧偏差 a,18
 *   -> 换算成车体位移 -> 平移 -> 停稳 -> 再等偏差
 *   -> ... 直到偏差进入容差或达到最大次数。
 *
 * 摄像头装在夹爪上，图像朝向随夹爪旋转，所以必须先转到调用方给的
 * carousel_us 姿态，之后的 w/a/s/d 才有确定含义。当前约定识别阶段
 * 一律转取料位2（K230_ALIGN_CAROUSEL_POS_US），夹取才转各步自己的朝向。
 * 同一步还要张开夹爪（合拢的夹爪会挡住摄像头），张开度取
 * GripperServo_Open_Large()，与取料姿态一致。
 * 姿势不到位之前不发检测命令，对位过程中也不再动夹爪。
 *
 * 收工条件按模式分：
 *   DETECT（原料区）：先等 K230 报回目标颜色（整行一个字符 1~6），颜色对了
 *     才开始用偏差帧纠偏；颜色确认之前进来的偏差帧不消费、grab 也不算数。
 *     收工两条路——偏差进入容差，或收到 grab；达成其一即发 stop 并 DONE。
 *   PLACE（粗加工区/暂存区）：K230 不报颜色，直接用偏差帧纠偏。
 *     收工两条路——偏差进入容差，或收到 place；达成其一即发 stop 并 DONE。
 * 只等偏差帧的话，遇到"识别成功只回收工信号"的固件会一直空等下去，
 * 表现为"K230 已经识别到了，但机械臂不夹"。
 * WAIT_OFFSET 不设失败超时：颜色与偏差帧可以无限等，到点只补发命令催促。
 * 颜色校验由 K230_ALIGN_REQUIRE_COLOR 控制（原料区需要，见 robot_config.h）。
 *
 * 偏差报文与方向映射的说明见 robot_config.h 的 K230_ALIGN_* 宏；
 * 本模块不含任何标定数字，只负责流程与换算。
 */

/*
 * 对位启动命令，按区域选择：
 *   原料区          = DETECT，发 detect,<颜色号>，让 K230 去原料台找指定颜色的物料
 *   粗加工区/暂存区 = PLACE， 发 place,<位置号>，让 K230 对准放置位
 */
typedef enum
{
    K230_ALIGN_MODE_DETECT = 0,
    K230_ALIGN_MODE_PLACE
} K230AlignMode_t;

typedef enum
{
    K230_ALIGN_IDLE = 0,
    K230_ALIGN_ROTATE,        /* 先把夹爪转到基准取料姿态并张开 */
    K230_ALIGN_WAIT_OFFSET,   /* 等一帧新偏差 */
    K230_ALIGN_MOVING,        /* 底盘微调中 */
    K230_ALIGN_SETTLE,        /* 移动后稳定等待 */
    K230_ALIGN_DONE,
    K230_ALIGN_ERROR
} K230AlignState_t;

#define K230_ALIGN_ERROR_NONE       0U
#define K230_ALIGN_ERROR_PARAM      1U  /* 方向字符非法 / 参数非法 */
#define K230_ALIGN_ERROR_SEND       2U  /* detect 命令发送失败 */
#define K230_ALIGN_ERROR_NO_OFFSET  3U  /* 超时仍没有偏差帧。当前不可达：WAIT_OFFSET 不设失败超时 */
#define K230_ALIGN_ERROR_MOTION     4U  /* 底盘微调失败 */
#define K230_ALIGN_ERROR_ITER       5U  /* 用满次数仍不收敛 */
#define K230_ALIGN_ERROR_REMOTE     6U  /* K230 回报 error,xxx */
#define K230_ALIGN_ERROR_ABORTED    7U
#define K230_ALIGN_ERROR_COLOR      8U  /* DETECT 超时仍没识别到目标颜色。当前不可达，同上 */

/*
 * 本次对位是怎么结束的。Watch 只看 robot_k230_align_exit 就能定位 stop 的来源，
 * 不必再从 state/error/grab 几个量反推。由 K230Align_Start() 清成 NONE。
 */
#define K230_ALIGN_EXIT_NONE       0U  /* 尚未结束（对位进行中或刚启动） */
#define K230_ALIGN_EXIT_TOLERANCE  1U  /* 偏差 <= 容差，正常收工 */
#define K230_ALIGN_EXIT_GRAB       2U  /* DETECT 收到 grab，正常收工（要求颜色已确认） */
#define K230_ALIGN_EXIT_PLACE      3U  /* PLACE 收到 place，正常收工 */
#define K230_ALIGN_EXIT_FAIL       4U  /* align_fail()：原因看 robot_k230_align_error */
#define K230_ALIGN_EXIT_ABORT      5U  /* K230Align_Abort()：被上层任务停掉 */

/* 诊断量：可直接放进 Keil Watch，应用程序只读。 */
extern volatile uint8_t  robot_k230_align_mode;
extern volatile uint8_t  robot_k230_align_state;
extern volatile uint8_t  robot_k230_align_iteration;
extern volatile uint8_t  robot_k230_align_error;
extern volatile uint8_t  robot_k230_align_exit;       /* 本次结束方式，见 K230_ALIGN_EXIT_* */
extern volatile uint8_t  robot_k230_align_color_ok;  /* DETECT：本轮已确认识别到目标色 */
extern volatile uint8_t  robot_k230_align_grab;      /* DETECT：本轮已收到 K230 的 grab */
extern volatile uint8_t  robot_k230_align_place;     /* PLACE：本轮已收到 K230 的 place */
extern volatile char     robot_k230_align_last_dir;
extern volatile int32_t  robot_k230_align_last_px;
extern volatile int32_t  robot_k230_align_px_x;   /* 本轮合成用的 ±X 像素偏差，带符号 */
extern volatile int32_t  robot_k230_align_px_y;   /* 本轮合成用的 ±Y 像素偏差，带符号 */

void K230Align_Init(Chassis *chassis);

/**
 * @brief 开始一次视觉对位。
 * @param mode        对位模式：原料区用 K230_ALIGN_MODE_DETECT，
 *                    粗加工区/暂存区用 K230_ALIGN_MODE_PLACE
 * @param target_id   模式为 DETECT 时是颜色号 1~6；
 *                    模式为 PLACE  时是位置号 1~3
 * @param carousel_us 对位时夹爪旋转舵机的目标脉宽。摄像头装在夹爪上，
 *                    这个值决定图像朝向；识别阶段一律传
 *                    K230_ALIGN_CAROUSEL_POS_US（取料位2），换掉必须重推
 *                    k230_align.c 的 offset_to_body()。
 * @return false 表示参数非法或当前正在对位；此时不改变任何状态。
 */
bool K230Align_Start(K230AlignMode_t mode, uint8_t target_id,
                     uint16_t carousel_us, uint32_t now_ms);

void K230Align_Update(uint32_t now_ms);
void K230Align_Abort(void);

bool K230Align_IsBusy(void);
bool K230Align_IsDone(void);
bool K230Align_HasError(void);

/* 诊断：已执行的微调次数与错误码。 */
uint8_t K230Align_GetIteration(void);
uint8_t K230Align_GetError(void);

#endif
