#ifndef __K230_ALIGN_H
#define __K230_ALIGN_H

#include "main.h"
#include "chassis.h"

#include <stdbool.h>
#include <stdint.h>

/**
 * @file k230_align.h
 * @brief K230 通用视觉对位；当前任务流程用于粗加工区/暂存区的底盘微调。
 *
 * 调用方式（每个作业点对一次，或由 zone_task 在每一步之前各对一次）：
 * @code
 * K230Align_Start(K230_ALIGN_MODE_DETECT, color_id, carousel_deg, now_ms);
 * @endcode
 * 随后由主循环持续调用 K230Align_Update()。
 *
 * 常规入口按「云台姿态由谁摆」二选一，不要混用：
 *   K230Align_Start()       本模块自己转云台并张开夹爪，再去取像。
 *   K230Align_StartPreset() 上层已经用 ManipulatorAction_SlideJogToAlignStart()
 *                           把云台转到识别姿态并张开（识别前那一段是
 *                           「云台先转、滑台后伸」，见 manipulator_action.h），
 *                           这里不再动舵机，只清残帧、发命令、开始纠偏。
 *   K230Align_StartStackPreset() 是第二轮暂存区的预置入口，机械前提与
 *                           StartPreset相同，另外携带第一层物料颜色做身份锚点。
 * 当前只有 robot_app 的粗加工区/暂存区放置对位走 StartPreset；原料区
 * zone_task 使用独立的 RawMaterialAlign。
 *
 * 这是一个底盘平移闭环。原料区不再走本模块的 DETECT 分支，而由
 * raw_material_align.c 用云台 + 滑台做二维位置控制；PLACE（粗加工区/
 * 暂存区）仍把偏差换算成车体位移，平移后重新取像。
 *
 * 摄像头装在夹爪上，图像朝向随夹爪旋转，所以必须先转到调用方给的
 * carousel_deg 姿态，之后的 w/a/s/d 才有确定含义。当前约定识别阶段
 * 一律先转取料位2（K230_ALIGN_CAROUSEL_ANGLE_DEG），本模块对位期间保持
 * 该姿态；原料区动态云台跟踪在 raw_material_align.c 中实现。
 * 同一步还要张开夹爪（合拢的夹爪会挡住摄像头），张开度取
 * GripperServo_Open_Large()，与取料姿态一致。
 * 起始姿势不到位之前不发检测命令。
 *
 * 收工条件按模式分：
 *   DETECT（原料区）：先等 K230 报回目标颜色（整行一个字符 1~6），颜色对了
 *     才开始用偏差帧纠偏；颜色确认之前进来的偏差帧不消费、grab 也不算数。
 *     收工两条路——偏差进入容差，或收到 grab；达成其一即发 stop，收到
 *     stopped 确认后才 DONE。
 *   PLACE（粗加工区/暂存区）：K230 不报颜色，直接用偏差帧纠偏。
 *     收工两条路——偏差进入容差，或收到 place；同样等 stopped 后才 DONE。
 *   STACK（第二轮暂存区）：先等匹配的 target,<位置号>，才接受偏差；K230用
 *     第一层物料颜色选择目标外圆，但偏差仍由外圆产生。最终只接受带匹配
 *     位置号的 place,<位置号>，不会因为邻近圆环误检而提前收工。
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
 *   粗加工区/第一轮暂存区 = PLACE，发 place,<位置号>，让 K230 对准放置位
 *   第二轮暂存区          = STACK，发 stack,<位置号>,<第一层颜色号>
 */
typedef enum
{
    K230_ALIGN_MODE_DETECT = 0,
    K230_ALIGN_MODE_PLACE,
    /* 第二轮暂存区：先用第一层物料颜色确认位置身份，再以外圆做几何对位。 */
    K230_ALIGN_MODE_STACK
} K230AlignMode_t;

typedef enum
{
    K230_ALIGN_IDLE = 0,
    K230_ALIGN_ROTATE,        /* 等夹爪姿态就位：Start 自己转，StartPreset 时已就位 */
    K230_ALIGN_WAIT_OFFSET,   /* 等一帧新偏差 */
    K230_ALIGN_MOVING,        /* 底盘微调中 */
    K230_ALIGN_SETTLE,        /* 移动后稳定等待 */
    K230_ALIGN_DONE,
    K230_ALIGN_ERROR,
    /* 追加在末尾，保留原有0~6状态编号，方便既有Keil Watch继续使用。 */
    K230_ALIGN_STOPPING       /* 已判定对准，等待K230回stopped */
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
extern volatile uint8_t  robot_k230_align_target_ok; /* STACK：K230回报的位置号与请求一致 */
extern volatile char     robot_k230_align_last_dir;
extern volatile int32_t  robot_k230_align_last_px;
extern volatile int32_t  robot_k230_align_px_x;   /* 本轮合成用的 ±X 像素偏差，带符号 */
extern volatile int32_t  robot_k230_align_px_y;   /* 本轮合成用的 ±Y 像素偏差，带符号 */
extern volatile uint16_t robot_k230_align_stop_requests; /* stop发送次数；>1表示发生重试 */
extern volatile uint16_t robot_k230_align_stop_tx_failures; /* K230_Stop返回非HAL_OK的次数 */
extern volatile uint32_t robot_k230_align_stop_wait_ms; /* 当前/最终等待stopped的毫秒数 */
extern volatile uint32_t robot_k230_align_stop_ack_ms; /* 收到stopped时的握手耗时 */
extern volatile uint8_t  robot_k230_align_stop_ack_received; /* 1=本次握手已收到stopped */

void K230Align_Init(Chassis *chassis);

/**
 * @brief 开始一次视觉对位。
 * @param mode        对位模式：原料区用 K230_ALIGN_MODE_DETECT，
 *                    粗加工区/第一轮暂存区用 K230_ALIGN_MODE_PLACE；
 *                    第二轮暂存区请使用K230Align_StartStackPreset
 * @param target_id   模式为 DETECT 时是颜色号 1~6；
 *                    模式为 PLACE/STACK 时是位置号 1~3
 * @param carousel_deg 对位时夹爪旋转舵机的目标角度。摄像头装在夹爪上，
 *                    这个值决定图像朝向；识别阶段一律传
 *                    K230_ALIGN_CAROUSEL_ANGLE_DEG（取料位2），换掉必须重推
 *                    k230_align.c 的 offset_to_body()。
 * @return false 表示参数非法或当前正在对位；此时不改变任何状态。
 */
bool K230Align_Start(K230AlignMode_t mode, uint8_t target_id,
                     float carousel_deg, uint32_t now_ms);

/**
 * @brief 云台已在识别姿态（夹爪已转好并张开）时开始一次视觉对位。
 *
 * 与 K230Align_Start() 的唯一区别是不再动舵机，直接从「清残帧 -> 发
 * detect/place」开始，省掉重复设定同一舵机目标带来的一个稳定期。
 * 姿态必须由调用方在此之前摆好（ManipulatorAction_SlideJogToAlignStart()），
 * 否则相机朝向不对，w/a/s/d 的含义全是错的。
 *
 * @param mode       同 K230Align_Start
 * @param target_id  同 K230Align_Start
 * @return false 表示参数非法或当前正在对位；此时不改变任何状态。
 */
bool K230Align_StartPreset(K230AlignMode_t mode, uint8_t target_id,
                           uint32_t now_ms);

/**
 * @brief 启动第二轮暂存区的“身份锚定 + 外圆定位”。
 *
 * 上层必须已经把云台和滑台摆到暂存区识别位。本函数向K230发送
 * stack,<position_id>,<anchor_color_id>：颜色只用于从多个圆环中选定目标位，
 * 底盘偏差仍由目标位外圆计算。STM32在收到匹配target,n之前不会移动。
 */
bool K230Align_StartStackPreset(uint8_t position_id, uint8_t anchor_color_id,
                                uint32_t now_ms);

void K230Align_Update(uint32_t now_ms);
void K230Align_Abort(void);

bool K230Align_IsBusy(void);
bool K230Align_IsDone(void);
bool K230Align_HasError(void);

/* 诊断：已执行的微调次数与错误码。 */
uint8_t K230Align_GetIteration(void);
uint8_t K230Align_GetError(void);

#endif
