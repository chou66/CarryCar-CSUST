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
    ROUGH_CAL_EXIT_PARK,

    /* 保持2号环低位相机姿态，由K230实际平移底盘对准圆心。 */
    ROUGH_CAL_VISION_ALIGN
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
#define ROUGH_CAL_ERROR_K230          9U

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
/*
 * ==================== 高位识别（保留旧标定） ====================
 * 电机5保持最高原点，0x83只移动滑台。该入口保留旧参数复核能力，不代表
 * 当前正式比赛的取像高度。
 */
uint8_t RoughCalibration_Start(uint8_t start_node, uint32_t now_ms);

/*
 * ==================== 低位识别（匹配正式逻辑） ====================
 * 路线、航向和手动调参方法与高位版相同；区别是第一次0x83从原点伸出后，
 * 自动下降电机5 ROUGH_VISION_PRE_DROP_MM，再开放K230观察和长度微调。
 * 0x84回零时会先把电机5抬回最高原点，再收回滑台。
 */
uint8_t RoughCalibration_StartLow(uint8_t start_node, uint32_t now_ms);

/*
 * 确认滑台已经由人工缩到机械原点，然后把此位置立为软件 0 mm。
 * confirm_key 必须是 0xA5，防止误触；本函数本身不会驱动滑台回零。
 */
uint8_t RoughCalibration_ConfirmSlideHome(uint8_t confirm_key);

/* 选择 1/2/3 号圆环：只在滑台 0 mm 时转云台并张开夹爪。 */
uint8_t RoughCalibration_SelectRing(uint8_t ring, uint32_t now_ms);

/*
 * 手工微调粗加工区云台绝对角，单位 0.1 deg，例如 2325 表示 232.5 deg。
 * 必须先用 SelectRing 选中待标定的1/2/3号并等云台到位；允许在齿条伸出、
 * 丝杆下降到放置高度后直接调角，方便观察夹爪与圆环中心。低位调角不再
 * 限制单次角度差，但整体目标仍受ROUGH_CAROUSEL_CAL_MIN/MAX_DEG限制。
 */
uint8_t RoughCalibration_MoveCarouselDeciDeg(uint16_t target_deci_deg,
                                             uint32_t now_ms);

/*
 * 滑台绝对位置命令，单位 0.1 mm。例如 1250 表示从机械原点伸到 125.0 mm。
 * 允许从一个目标直接微调到另一个目标，状态机内部换算相对运动量；丝杆
 * 位于放置低位时单次最多调整ROUGH_LOW_CAL_MAX_SLIDE_STEP_MM（当前10 mm）。
 */
uint8_t RoughCalibration_MoveSlideDeciMm(uint16_t target_deci_mm);

/*
 * ==================== 低位识别后的实际落料点验证 ====================
 * 仅在0x89低位模式、已选圆环且滑台到位后有效。target_deci_mm表示电机5
 * 从最高机械原点累计下降量，单位0.1 mm；例如1850表示下降185.0 mm。
 * 可在ROUGH_VISION_PRE_DROP_MM与-MANIP_PICKUP_GROUND_MM之间反复调整。
 * 185 mm是此前正式跑图已验证的绝对高度，因此粗加工区允许从最高原点或
 * 100 mm观察位直接到185 mm；状态机内部始终按“目标绝对高度-当前绝对
 * 高度”计算相对行程，不会把185叠加在100之上。
 */
uint8_t RoughCalibration_MoveGimbalDropDeciMm(uint16_t target_deci_mm);

/*
 * 分轴回收：RaiseGimbalToTop只抬丝杆，RetractSlideOnly只收齿条。
 * 后者要求丝杆已经位于最高原点。原RetractSlide仍是“先抬升、再收回”的
 * 组合安全动作，保留给一键回收和旧命令使用。
 */
uint8_t RoughCalibration_RaiseGimbalToTop(void);
uint8_t RoughCalibration_RetractSlideOnly(void);

/* 从当前已确认位置组合退回：先抬丝杆，再收齿条到软件0 mm。 */
uint8_t RoughCalibration_RetractSlide(void);

/*
 * 粗加工区2号环底盘视觉对准。只在2号环、识别角、观察高度都已经到位时
 * 启动K230 PLACE-2；不抓放、不改变云台/齿条/丝杆，完成后仍可继续标定。
 */
uint8_t RoughCalibration_StartVisionAlign(uint32_t now_ms);

/*
 * 急停并退出标定；若滑台曾运动，位置基准作废，下一次必须重新人工回零确认。
 * 本接口遵守急停语义，不会主动抬升电机5；低位标定正常退出应先RetractSlide。
 */
void RoughCalibration_Abort(void);

/* 必须在主循环持续调用；内部推进 EdgeMotion、航向 PD 和滑台非阻塞状态机。 */
void RoughCalibration_Update(uint32_t now_ms);

uint8_t RoughCalibration_IsActive(void);
RoughCalibrationState_t RoughCalibration_GetState(void);
uint8_t RoughCalibration_GetError(void);
uint8_t RoughCalibration_GetCurrentNode(void);
uint8_t RoughCalibration_GetSelectedRing(void);
uint8_t RoughCalibration_IsHomeConfirmed(void);
uint8_t RoughCalibration_IsLowVision(void);
float RoughCalibration_GetSlideTargetMm(void);
float RoughCalibration_GetCarouselTargetDeg(void);
float RoughCalibration_GetGimbalDropTargetMm(void);

/* 非 static 调试镜像，Keil Watch 可直接查看。 */
extern volatile uint8_t rough_cal_state;
extern volatile uint8_t rough_cal_error;
extern volatile uint8_t rough_cal_current_node;
extern volatile uint8_t rough_cal_selected_ring;
extern volatile uint8_t rough_cal_home_confirmed;
/* 0=旧高位识别，1=正式低位识别。 */
extern volatile uint8_t rough_cal_low_vision;
extern volatile float rough_cal_target_slide_mm;
extern volatile float rough_cal_target_carousel_deg;
/* 正数=从电机5最高机械原点向下的目标距离。 */
extern volatile float rough_cal_target_gimbal_drop_mm;
extern volatile float rough_cal_world_yaw_deg;

#endif
