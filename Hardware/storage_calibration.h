#ifndef __STORAGE_CALIBRATION_H
#define __STORAGE_CALIBRATION_H

#include "chassis.h"
#include <stdint.h>

/*
 * 暂存区实车标定状态机。
 *
 * 固定流程（只允许从启停区2开始）：
 *   启停区2 -> N3 -> N2 -> +90 deg任务航向
 *   -> 复现粗加工2号环的205 deg + 78 mm相机位姿
 *   -> K230平移底盘对准2号环（不抓、不放）
 *   -> 齿条回零 -> N2 -> N5 -> N4 -> 180 deg任务航向
 *   -> 停车，开放暂存区1/2/3号位的云台/齿条手动标定。
 *
 * 这条路线把已标定的粗加工2号圆环当作途中定位基准：K230修正之后，
 * Navigation仍把车辆认作准确的N2，再从N2行驶到N4。这样暂存区参数中
 * 不会混入“从启停区2一路开到N4”的全部累计误差。
 */
typedef enum
{
    STORAGE_CAL_IDLE = 0,
    STORAGE_CAL_EXIT_PARK,
    STORAGE_CAL_NAV_TO_ROUGH,
    STORAGE_CAL_ROUGH_ALIGNING,
    STORAGE_CAL_ROUGH_ALIGN_SETTLE,
    STORAGE_CAL_ROUGH_PRESET,
    STORAGE_CAL_ROUGH_VISION_ALIGN,
    STORAGE_CAL_ROUGH_RETRACT,
    STORAGE_CAL_NAV_TO_STORAGE,
    STORAGE_CAL_STORAGE_ALIGNING,
    STORAGE_CAL_STORAGE_ALIGN_SETTLE,
    STORAGE_CAL_HOME_READY,
    STORAGE_CAL_ARM_MOVING,
    STORAGE_CAL_ARM_READY,
    STORAGE_CAL_SLIDE_MOVING,
    STORAGE_CAL_SLIDE_READY,
    STORAGE_CAL_ERROR,

    /*
     * 追加在末尾，保持0~16的既有VOFA状态编号不变。
     * 当前状态表示：在N4保持2号环机械位姿，用K230平移底盘锁定圆心。
     */
    STORAGE_CAL_STORAGE_VISION_ALIGN
} StorageCalibrationState_t;

/* 错误码镜像，供VOFA ch1与Keil Watch直接观察。 */
#define STORAGE_CAL_ERROR_NONE          0U
#define STORAGE_CAL_ERROR_PARAM         1U
#define STORAGE_CAL_ERROR_BUSY          2U
#define STORAGE_CAL_ERROR_IMU           3U
#define STORAGE_CAL_ERROR_NAVIGATION    4U
#define STORAGE_CAL_ERROR_EDGE_MOTION   5U
#define STORAGE_CAL_ERROR_HOME          6U
#define STORAGE_CAL_ERROR_MANIPULATOR   7U
#define STORAGE_CAL_ERROR_PARK          8U
#define STORAGE_CAL_ERROR_K230          9U

/* 与粗加工标定相同的二次确认口令：表示滑台确实位于机械原点。 */
#define STORAGE_CAL_HOME_CONFIRM_KEY    0xA5U

void StorageCalibration_Init(Chassis *chassis);

/*
 * 从启停区2启动完整模拟流程。
 * 发命令前必须满足：车头朝地图+Y、车体停在启停区2标定位置、滑台已物理
 * 缩到机械原点。confirm_key必须为A5；函数只重建软件0 mm，不会自动找原点。
 */
uint8_t StorageCalibration_StartFromBay2(uint8_t confirm_key,
                                         uint32_t now_ms);

/* 到N4并停稳后使用：先选1/2/3号位，再调绝对角度和齿条绝对长度。 */
uint8_t StorageCalibration_SelectRing(uint8_t ring, uint32_t now_ms);
uint8_t StorageCalibration_MoveCarouselDeciDeg(uint16_t target_deci_deg,
                                               uint32_t now_ms);
uint8_t StorageCalibration_MoveSlideDeciMm(uint16_t target_deci_mm);
uint8_t StorageCalibration_RetractSlide(void);

/*
 * 在N4用当前2号环相机位姿锁定底盘位置。
 *
 * 启动条件：已经选中2号、云台保持K230固定识别角205 deg、齿条已到达本轮
 * 测试长度（例如90 mm），状态为SLIDE_READY。函数不改变云台/齿条，只启动
 * K230 PLACE-2底盘平移。图像对准后会重新用IMU验证180 deg航向；若发生
 * 航向修正，则再次运行K230，直到“圆心像素对准 + 航向对准”同时成立。
 * 完成后仍停在SLIDE_READY，不抓放、不自动收齿条，便于立即观察或继续调参。
 */
uint8_t StorageCalibration_StartStorageVisionAlign(uint32_t now_ms);

/* 全局急停/人工退出；按急停语义使滑台软件坐标失效。 */
void StorageCalibration_Abort(void);

/* 必须在主循环持续调用；推进路线、航向PD、K230纠偏和滑台非阻塞动作。 */
void StorageCalibration_Update(uint32_t now_ms);

uint8_t StorageCalibration_IsActive(void);
uint8_t StorageCalibration_AllowsManualVision(void);
StorageCalibrationState_t StorageCalibration_GetState(void);
uint8_t StorageCalibration_GetError(void);
uint8_t StorageCalibration_GetCurrentNode(void);
uint8_t StorageCalibration_GetSelectedRing(void);
float StorageCalibration_GetSlideTargetMm(void);
float StorageCalibration_GetCarouselTargetDeg(void);

/* 非static调试镜像，Keil Watch可直接输入变量名。 */
extern volatile uint8_t storage_cal_state;
extern volatile uint8_t storage_cal_error;
extern volatile uint8_t storage_cal_current_node;
extern volatile uint8_t storage_cal_selected_ring;
extern volatile uint8_t storage_cal_home_confirmed;
extern volatile float storage_cal_target_slide_mm;
extern volatile float storage_cal_target_carousel_deg;
extern volatile float storage_cal_world_yaw_deg;

#endif
