#ifndef __STORAGE_CALIBRATION_H
#define __STORAGE_CALIBRATION_H

#include "chassis.h"
#include <stdint.h>

/*
 * 暂存区实车标定状态机。
 *
 * 固定流程（只允许从启停区2开始）：
 *   启停区2 -> N3 -> N2 -> +90 deg任务航向
 *   -> 复现粗加工2号环相机位姿（旧高位78 mm；正式低位95 mm）
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
/*
 * ==================== 高位识别（保留旧标定） ====================
 * N2途中定位和N4手工观察都让电机5保持最高原点，用于复核旧参数。
 */
uint8_t StorageCalibration_StartFromBay2(uint8_t confirm_key,
                                         uint32_t now_ms);

/*
 * ==================== 低位识别（匹配正式逻辑） ====================
 * N2途中定位使用ROUGH_VISION_PRE_DROP_MM；到N4后，第一次0x83伸出会使用
 * STORAGE_VISION_PRE_DROP_MM下降识圆。0x84会先抬升电机5，再收回滑台。
 */
uint8_t StorageCalibration_StartFromBay2Low(uint8_t confirm_key,
                                            uint32_t now_ms);

/*
 * 到N4并停稳后使用：先选1/2/3号位，再调绝对角度和齿条绝对长度。
 * 在185 mm放置高度仍可直接调云台；齿条单次最多改变10 mm，调参方法与
 * 粗加工区完全一致。
 */
uint8_t StorageCalibration_SelectRing(uint8_t ring, uint32_t now_ms);
uint8_t StorageCalibration_MoveCarouselDeciDeg(uint16_t target_deci_deg,
                                               uint32_t now_ms);
uint8_t StorageCalibration_MoveSlideDeciMm(uint16_t target_deci_mm);

/*
 * ==================== 低位识别后的实际落料点验证 ====================
 * 仅在0x8A低位流程已经到N4、选好圆环且滑台到位后有效。
 * target_deci_mm为电机5从最高原点累计向下的距离（0.1 mm）：1000回取像
 * 高度，1250验证第二层，1850验证第一层。185 mm是已经过正式跑图验证的
 * 绝对高度，可从最高点或100 mm观察位直接到达，不会把185叠加在当前位置。
 */
uint8_t StorageCalibration_MoveGimbalDropDeciMm(uint16_t target_deci_mm);

/*
 * 分轴回收：先用RaiseGimbalToTop只把丝杆抬到最高原点，再用
 * RetractSlideOnly只把齿条收回0 mm。原RetractSlide仍保留为自动执行
 * “先抬丝杆、再收齿条”的组合安全动作。
 */
uint8_t StorageCalibration_RaiseGimbalToTop(void);
uint8_t StorageCalibration_RetractSlideOnly(void);
uint8_t StorageCalibration_RetractSlide(void);

/*
 * 在N4用当前2号环相机位姿锁定底盘位置。
 *
 * 启动条件：已经选中2号、云台保持K230固定识别角205 deg、齿条已到达本轮
 * 测试长度（低位基础值95 mm），状态为SLIDE_READY。函数不改变云台/齿条，只启动
 * K230 PLACE-2底盘平移。图像对准后会重新用IMU验证180 deg航向；若发生
 * 航向修正，则再次运行K230，直到“圆心像素对准 + 航向对准”同时成立。
 * 完成后仍停在SLIDE_READY，不抓放、不自动收齿条，便于立即观察或继续调参。
 */
uint8_t StorageCalibration_StartStorageVisionAlign(uint32_t now_ms);

/*
 * 全局急停/人工退出；按急停语义使滑台软件坐标失效，不主动抬升电机5。
 * 低位标定的正常退出顺序应为RetractSlide完成后再Abort。
 */
void StorageCalibration_Abort(void);

/* 必须在主循环持续调用；推进路线、航向PD、K230纠偏和滑台非阻塞动作。 */
void StorageCalibration_Update(uint32_t now_ms);

uint8_t StorageCalibration_IsActive(void);
uint8_t StorageCalibration_AllowsManualVision(void);
StorageCalibrationState_t StorageCalibration_GetState(void);
uint8_t StorageCalibration_GetError(void);
uint8_t StorageCalibration_GetCurrentNode(void);
uint8_t StorageCalibration_GetSelectedRing(void);
uint8_t StorageCalibration_IsLowVision(void);
float StorageCalibration_GetSlideTargetMm(void);
float StorageCalibration_GetCarouselTargetDeg(void);
float StorageCalibration_GetGimbalDropTargetMm(void);

/* 非static调试镜像，Keil Watch可直接输入变量名。 */
extern volatile uint8_t storage_cal_state;
extern volatile uint8_t storage_cal_error;
extern volatile uint8_t storage_cal_current_node;
extern volatile uint8_t storage_cal_selected_ring;
extern volatile uint8_t storage_cal_home_confirmed;
/* 0=旧高位识别，1=正式低位识别。 */
extern volatile uint8_t storage_cal_low_vision;
extern volatile float storage_cal_target_slide_mm;
extern volatile float storage_cal_target_carousel_deg;
/* 正数=从电机5最高机械原点向下的目标距离。 */
extern volatile float storage_cal_target_gimbal_drop_mm;
extern volatile float storage_cal_world_yaw_deg;

#endif
