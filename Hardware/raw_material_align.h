#ifndef __RAW_MATERIAL_ALIGN_H
#define __RAW_MATERIAL_ALIGN_H

#include "main.h"

#include <stdbool.h>
#include <stdint.h>

/**
 * @file raw_material_align.h
 * @brief 二维视觉位置闭环：原料区（detect）与粗加工/暂存区机械臂校准（place）。
 *
 * 上层先完成机械预置，然后调用 Start / StartPlace。本模块把 K230 的两轴
 * 像素误差转换成两个位置执行器的增量目标：
 *   w/s (px_x) -> 齿轮齿条绝对伸出量
 *   a/d (px_y) -> 云台舵机绝对角度
 * 每次执行器到位并稳定后再取下一帧，不驱动底盘。
 *
 * 两种目标：
 *   RAW_ALIGN_MODE_DETECT：原料区发 detect,<颜色号>，识别到该颜色且居中
 *     后结束；Start() 是它的兼容入口。
 *   RAW_ALIGN_MODE_PLACE：粗加工区/第一轮暂存区的1、3号位发 place,<位号>。
 *     上层必须先按该位现有固定位姿伸出（云台角+齿条长度），本模块只做
 *     机械臂二维微调，不移动底盘；收到 place 或连续居中同样可结束。
 *
 * 完成后 GetCarouselDeg()/GetSlideMm() 返回相机真正对准时的机械位姿。
 * 调用方必须保存两者：仅保存云台角度而把滑台恢复成固定值，会再次改变
 * 相机/夹爪与目标的径向关系。当前位置环不限制迭代次数，只能由居中、
 * K230 的 grab/place、远端 error 或上层 Abort 结束。
 */

typedef enum
{
    RAW_ALIGN_IDLE = 0,
    RAW_ALIGN_WAIT_OFFSET, /* 等停稳后产生的新颜色/两轴偏差帧 */
    RAW_ALIGN_MOVING,      /* 云台与滑台向本轮位置目标运动 */
    RAW_ALIGN_SETTLE,      /* 执行器到位后的机械稳定时间 */
    RAW_ALIGN_DONE,        /* 已居中或 K230 明确返回 grab */
    RAW_ALIGN_ERROR,       /* 发送、远端、滑台或参数错误 */
    /* 追加在末尾，保留原有 0~5 状态编号，方便既有 Keil Watch 继续使用。 */
    RAW_ALIGN_STOPPING     /* 等待 K230 的 stopped；也用于 busy 后清旧任务 */
} RawMaterialAlignState_t;

#define RAW_ALIGN_ERROR_NONE       0U
#define RAW_ALIGN_ERROR_PARAM      1U
#define RAW_ALIGN_ERROR_SEND       2U
#define RAW_ALIGN_ERROR_REMOTE     3U
#define RAW_ALIGN_ERROR_SLIDE      4U
#define RAW_ALIGN_ERROR_ABORTED    5U

#define RAW_ALIGN_EXIT_NONE        0U
#define RAW_ALIGN_EXIT_TOLERANCE   1U
#define RAW_ALIGN_EXIT_GRAB        2U
#define RAW_ALIGN_EXIT_FAIL        3U
#define RAW_ALIGN_EXIT_ABORT       4U
/* PLACE 模式收到 K230 的 place 收工；追加在末尾，保留原有编号。 */
#define RAW_ALIGN_EXIT_PLACE       5U

/* 本模块当前跟踪哪类目标。 */
typedef enum
{
    RAW_ALIGN_MODE_DETECT = 0, /* detect,<颜色1..6>：原料区 */
    RAW_ALIGN_MODE_PLACE       /* place,<位号1..3>：粗加工/第一轮暂存区机械臂 */
} RawAlignMode_t;

/* 位置目标被哪个软件边界钳住；NONE 表示本轮没有撞限位。 */
#define RAW_ALIGN_LIMIT_NONE       0U
#define RAW_ALIGN_LIMIT_MIN        1U
#define RAW_ALIGN_LIMIT_MAX        2U

void RawMaterialAlign_Init(void);
/* 原料区 DETECT 入口：发 detect,<color_id>。 */
bool RawMaterialAlign_Start(uint8_t color_id, uint32_t now_ms);

/*
 * 粗加工区/第一轮暂存区 1、3 号位机械臂校准入口：发 place,<position_id>。
 *
 * 调用前必须已经用 SlideJogToRingAlignStart() 把云台转到 carousel_deg、
 * 齿条伸到该位现有长度（可带视觉预下降）。本函数不重复驱动云台/滑台，
 * 以当前滑台位置作为径向环起点，以 carousel_deg 作为切向环起点；
 * 完成后 GetCarouselDeg()/GetSlideMm() 就是校准后的放置/取回位姿。
 *
 * @param position_id  K230 放置位号 1..3
 * @param carousel_deg 预置位云台角（该位写死位姿的 carousel_deg）
 * @return false 表示参数非法、滑台位置不可信或当前仍在忙。
 */
bool RawMaterialAlign_StartPlace(uint8_t position_id, float carousel_deg,
                                 uint32_t now_ms);

void RawMaterialAlign_Update(uint32_t now_ms);
void RawMaterialAlign_Abort(void);

bool RawMaterialAlign_IsBusy(void);
bool RawMaterialAlign_IsDone(void);
bool RawMaterialAlign_HasError(void);

float RawMaterialAlign_GetCarouselDeg(void);
float RawMaterialAlign_GetSlideMm(void);
uint16_t RawMaterialAlign_GetIteration(void);
uint8_t RawMaterialAlign_GetError(void);

/*
 * Keil Watch 诊断量：
 *   px_x/px_y       最近一次被控制器消费的带符号像素误差；
 *   *_step_*        本轮经过 PID 和安全钳位后的实际位置增量；
 *   carousel/slide  当前已下发的绝对目标，也是成功后重放的抓取位姿；
 *   stable_frames   连续进入像素容差的帧数；
 *   iteration       实际发生机械位置调整的次数（边界等待不计数）；
 *   *_limit         0=未限位，1=最小限位，2=最大限位；
 *   have_x/have_y   本轮配对窗口内是否收到对应轴的新像素；
 *   detect_requests 本轮向 K230 发送 detect 的累计次数；
 *   stop_requests   本轮 stop 的发送次数（大于1表示发生过确认丢包/重试）；
 *   stop_tx_failures K230_Stop 在STM32端返回非HAL_OK的次数；
 *   stop_wait/ack_ms 当前等待时间 / 收到stopped时的最终耗时；
 *   stop_ack_received 1=本次握手已收到stopped；
 *   busy_recoveries 收到 error,busy 后完成自动清旧任务的累计次数。
 */
extern volatile uint8_t robot_raw_align_state;
extern volatile uint8_t robot_raw_align_error;
extern volatile uint8_t robot_raw_align_exit;
extern volatile uint8_t robot_raw_align_color_ok;
extern volatile uint8_t robot_raw_align_grab;
extern volatile uint8_t robot_raw_align_stable_frames;
extern volatile uint16_t robot_raw_align_iteration;
extern volatile int32_t robot_raw_align_px_x;
extern volatile int32_t robot_raw_align_px_y;
extern volatile float robot_raw_align_carousel_deg;
extern volatile float robot_raw_align_slide_mm;
extern volatile float robot_raw_align_angle_step_deg;
extern volatile float robot_raw_align_slide_step_mm;
extern volatile uint8_t robot_raw_align_angle_limit;
extern volatile uint8_t robot_raw_align_slide_limit;
extern volatile uint8_t robot_raw_align_have_x;
extern volatile uint8_t robot_raw_align_have_y;
extern volatile uint16_t robot_raw_align_detect_requests;
extern volatile uint16_t robot_raw_align_stop_requests;
extern volatile uint16_t robot_raw_align_stop_tx_failures;
extern volatile uint32_t robot_raw_align_stop_wait_ms;
extern volatile uint32_t robot_raw_align_stop_ack_ms;
extern volatile uint8_t robot_raw_align_stop_ack_received;
extern volatile uint16_t robot_raw_align_busy_recoveries;

#endif
