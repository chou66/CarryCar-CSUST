#ifndef __ROUGH_CALIBRATION_H
#define __ROUGH_CALIBRATION_H

#include "chassis.h"
#include <stdint.h>

/*
 * 粗加工区实车标定状态机。
 *
 * 它只在 VOFA 已经接管 RobotApp 后使用，目的不是替代正式任务，而是让每次
 * 测滑台长度时，小车都通过正式 EdgeMotion 走到同一个 N2 物理停靠位，并
 * 用与正式任务相同的航向 PD 对准 +90 deg。这样测出的长度不会混入人工摆车
 * 的位置/角度误差。
 */
typedef enum
{
    ROUGH_CAL_IDLE = 0,
    ROUGH_CAL_NAVIGATING,
    ROUGH_CAL_ALIGNING,
    ROUGH_CAL_ALIGN_SETTLE,
    ROUGH_CAL_WAIT_HOME_CONFIRM,
    ROUGH_CAL_HOME_READY,
    ROUGH_CAL_ARM_MOVING,
    ROUGH_CAL_ARM_READY,
    ROUGH_CAL_SLIDE_MOVING,
    ROUGH_CAL_SLIDE_READY,
    ROUGH_CAL_ERROR,

    /* 追加在末尾，保留前面 0~10 的 VOFA 状态编号不变。 */
    ROUGH_CAL_EXIT_PARK
} RoughCalibrationState_t;

/* 错误码镜像，供 Keil Watch/VOFA 通道直接观察。 */
#define ROUGH_CAL_ERROR_NONE          0U
#define ROUGH_CAL_ERROR_PARAM         1U
#define ROUGH_CAL_ERROR_BUSY          2U
#define ROUGH_CAL_ERROR_IMU           3U
#define ROUGH_CAL_ERROR_NAVIGATION    4U
#define ROUGH_CAL_ERROR_EDGE_MOTION   5U
#define ROUGH_CAL_ERROR_HOME          6U
#define ROUGH_CAL_ERROR_MANIPULATOR   7U
#define ROUGH_CAL_ERROR_PARK          8U

/* 0x80 的特殊起点码：蓝色启停区不是 1~9 路网节点，单独编码避免与 N1/N2 混淆。 */
#define ROUGH_CAL_START_BAY1          0xA1U
#define ROUGH_CAL_START_BAY2          0xA2U

void RoughCalibration_Init(Chassis *chassis);

/*
 * 从操作者声明的当前节点/启停区出发，使用正式前行优先 EdgeMotion 到 N2。
 * 启动瞬间的车头方向会被定义为地图 +Y/世界 0 deg，所以发命令前必须把车头
 * 朝向地图上方：
 *   start_node=1..9                  当前就在对应路网节点；
 *   start_node=ROUGH_CAL_START_BAY1  当前在启停区1中心，先退出到 N9；
 *   start_node=ROUGH_CAL_START_BAY2  当前在启停区2中心，先退出到 N3；
 *   start_node=2                     不平移，只执行 N2 的 +90 deg 任务航向校正。
 */
uint8_t RoughCalibration_Start(uint8_t start_node, uint32_t now_ms);

/*
 * 确认滑台已经由人工缩到机械原点，然后把此位置立为软件 0 mm。
 * confirm_key 必须是 0xA5，防止误触；本函数本身不会驱动滑台回零。
 */
uint8_t RoughCalibration_ConfirmSlideHome(uint8_t confirm_key);

/* 选择 1/2/3 号圆环：只在滑台 0 mm 时转云台并张开夹爪。 */
uint8_t RoughCalibration_SelectRing(uint8_t ring, uint32_t now_ms);

/*
 * 手工微调粗加工区云台绝对角，单位 0.1 deg，例如 2325 表示 232.5 deg。
 * 必须先用 SelectRing 选中待标定的1/2/3号并等云台到位；允许在齿条伸出后
 * 小幅调角，方便同时找固定圆环的角度/半径。角度受 robot_config.h 中
 * ROUGH_CAROUSEL_CAL_MIN/MAX_DEG 限制，防止误命令让夹爪扫过整车。
 */
uint8_t RoughCalibration_MoveCarouselDeciDeg(uint16_t target_deci_deg,
                                             uint32_t now_ms);

/*
 * 滑台绝对位置命令，单位 0.1 mm。例如 1250 表示从机械原点伸到 125.0 mm。
 * 允许从一个目标直接微调到另一个目标，状态机内部换算相对运动量。
 */
uint8_t RoughCalibration_MoveSlideDeciMm(uint16_t target_deci_mm);

/* 从当前已确认位置退回软件 0 mm。 */
uint8_t RoughCalibration_RetractSlide(void);

/* 急停并退出标定；若滑台曾运动，位置基准作废，下一次必须重新人工回零确认。 */
void RoughCalibration_Abort(void);

/* 必须在主循环持续调用；内部推进 EdgeMotion、航向 PD 和滑台非阻塞状态机。 */
void RoughCalibration_Update(uint32_t now_ms);

uint8_t RoughCalibration_IsActive(void);
RoughCalibrationState_t RoughCalibration_GetState(void);
uint8_t RoughCalibration_GetError(void);
uint8_t RoughCalibration_GetCurrentNode(void);
uint8_t RoughCalibration_GetSelectedRing(void);
uint8_t RoughCalibration_IsHomeConfirmed(void);
float RoughCalibration_GetSlideTargetMm(void);
float RoughCalibration_GetCarouselTargetDeg(void);

/* 非 static 调试镜像，Keil Watch 可直接查看。 */
extern volatile uint8_t rough_cal_state;
extern volatile uint8_t rough_cal_error;
extern volatile uint8_t rough_cal_current_node;
extern volatile uint8_t rough_cal_selected_ring;
extern volatile uint8_t rough_cal_home_confirmed;
extern volatile float rough_cal_target_slide_mm;
extern volatile float rough_cal_target_carousel_deg;
extern volatile float rough_cal_world_yaw_deg;

#endif
