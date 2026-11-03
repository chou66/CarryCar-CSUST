#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "mecanum_kinematics.h"
#include "hwt101_uart.h"
#include "heading_control.h"

/*
 * 底盘运动模式。
 *
 * CHASSIS_MODE_VELOCITY：直接速度命令。收到后立即按目标轮速运行，
 *   由调用方自己周期性刷新（航向 PD、视觉微调等）。EMM 使用 chassis->acceleration。
 *
 * CHASSIS_MODE_PROFILE：速度模式 + 软件正弦加减速剖面。一次位移被拆成
 *   “正弦加速 -> 匀速 -> 正弦减速 -> STOP”，内部按 CHASSIS_PROFILE_UPDATE_MS
 *   刷新速度；完成与否由时间积分判断，不再依赖 0xFD 位置模式和 0x3A 状态轮询。
 *
 * CHASSIS_MODE_POSITION：EMM 位置模式（0xFD）+ 0x3A 到位轮询。目标脉冲一次
 *   交给驱动器，加减速由驱动器完成。专供 K230 视觉微调这类 mm 级小位移：
 *   速度剖面的总时长短于刷新周期时会被时间量化截断，导致车轮反复点动。
 *   导航大位移仍走 PROFILE，不经过本模式。
 */
typedef enum {
    CHASSIS_MODE_VELOCITY,
    CHASSIS_MODE_PROFILE,
    CHASSIS_MODE_POSITION
} ChassisMode;

typedef enum {
    CHASSIS_TX_IDLE,
    CHASSIS_TX_WHEEL_1,
    CHASSIS_TX_WHEEL_2,
    CHASSIS_TX_WHEEL_3,
    CHASSIS_TX_WHEEL_4,
    CHASSIS_TX_SYNC,
    CHASSIS_TX_WAIT_POSITION,  /* 位置模式：等 0x3A 到位 */
    CHASSIS_TX_STOP_1,
    CHASSIS_TX_STOP_2,
    CHASSIS_TX_STOP_3,
    CHASSIS_TX_STOP_4
} ChassisTxState;

typedef struct {
    MecanumConfig config;
    /* 可选：绑定后由底盘命令自动门控 HWT101 停车抗漂移层，漂移路径也用它读航向。 */
    Hwt101 *imu_filter;
    ChassisMode mode;
    float target_x;       /* 速度模式：vx_mps；剖面模式：当前峰值 vx_mps */
    float target_y;       /* 速度模式：vy_mps；剖面模式：当前峰值 vy_mps */
    float target_z;       /* 速度模式：wz_radps；剖面模式：当前峰值 wz_radps；位置模式：dtheta_rad */
    uint8_t acceleration;

    uint32_t last_command_ms;
    uint32_t watchdog_ms;

    /* ---- 位置模式（0xFD + 0x3A 到位轮询，仅视觉微调）---- */
    float speed_limit;                 /* 位置模式的最大轮缘等效线速度 */
    uint32_t position_timeout_ms;      /* 0=按行程动态估算 */
    uint32_t active_position_timeout_ms; /* 本次动作最终采用的超时 */
    uint32_t position_start_ms;        /* 同步启动成功的时刻 */
    uint32_t position_feedback_not_before_ms;
    uint32_t position_deadline_ms;
    uint32_t next_status_query_ms;
    uint8_t status_query_id;
    uint8_t motor_mask;                /* 本次位置动作参与的电机位 */

    /* ---- 速度剖面（正弦加减速）---- */
    float profile_vx_mps;          /* 车体系巡航峰值速度 */
    float profile_vy_mps;
    float profile_wz_radps;
    float profile_world_vx_mps;    /* 世界系巡航峰值速度（漂移模式） */
    float profile_world_vy_mps;
    float profile_target_yaw_deg;  /* 漂移模式的目标世界航向 */
    float profile_last_yaw_deg;    /* IMU 断帧时沿用上一帧航向 */
    HeadingController profile_heading;
    uint32_t profile_ramp_ms;      /* 加/减速各段时长 */
    uint32_t profile_cruise_ms;    /* 匀速段时长 */
    uint32_t profile_total_ms;     /* 剖面总时长 */
    uint32_t profile_start_ms;     /* 剖面计时起点 */
    uint32_t profile_next_update_ms;
    uint8_t profile_heading_enabled; /* 1=漂移模式，按 IMU 航向旋转速度矢量 */
    uint8_t profile_running;         /* 1=剖面正在执行 */
    uint8_t profile_finishing;       /* 1=剖面已到时间，正在下发 STOP */

    /* ---- 拐角圆弧：世界速度方向与航向目标同步进给 ---- */
    float profile_arc_speed_mps;      /* 圆弧线速度（恒速） */
    float profile_arc_start_yaw_deg;  /* 圆弧起始切线方向（=车体中心行进方向） */
    float profile_arc_yaw_rate_dps;   /* 圆弧进给角速度（世界系，顺时针为正） */
    uint8_t profile_arc_enabled;      /* 1=弧线剖面，0=直线漂移剖面 */
    uint8_t profile_arc_reverse;      /* 1=倒车走弧：车头航向 = 切线 + 180° */

    /*
     * 相位衔接：直线→圆弧→直线时前后两段不能各自起停。
     * ramp_in=0 表示以满速直接进入（前一段末尾保持满速）；
     * ramp_out=0 表示结束时不再减速，把满速交给下一段。
     */
    uint8_t profile_ramp_in;
    uint8_t profile_ramp_out;

    uint32_t next_tx_ms;
    uint32_t tx_deadline_ms;          /* CAN 派发/停车序列最晚完成时间 */

    bool dirty;
    bool emergency_stop;
    bool velocity_watchdog_armed;     /* 收到速度命令后才启用软件速度看门狗 */
    bool tx_failed;                   /* CAN 连续发送失败并超过超时 */
    bool position_done;               /* 剖面/位置运动完成（旧名字，上层通用） */
    bool position_failed;             /* 剖面/位置运动失败或被中止 */

    ChassisMode active_mode;
    ChassisTxState tx_state;
    MecanumWheelCommand active_wheels;
} Chassis;

void chassis_init(Chassis *chassis, const MecanumConfig *config);
void chassis_bind_imu_filter(Chassis *chassis, Hwt101 *imu);

/*
 * 直接速度命令：vx 向右为正，vy 向前为正，wz 逆时针为正。
 * 会取消正在执行的剖面/位置运动（调用方应避免中途混用）。
 */
void chassis_set_velocity(Chassis *chassis, float vx_mps, float vy_mps,
                          float wz_radps, uint32_t now_ms);

/*
 * 位置模式（0xFD）：一次下发各轮目标脉冲，由 EMM 内部完成加减速并回 0x3A
 * 到位。用于 K230 视觉对位的 mm 级微调——速度剖面在几毫米行程下总时长
 * 会短于刷新周期，被时间量化截断后只会让轮子一顿一挫。
 * dx/dy/dtheta 为车体系位移，speed_mps 是正的速度上限。
 */
void chassis_set_position(Chassis *chassis, float dx_m, float dy_m,
                          float dtheta_rad, float speed_mps, uint32_t now_ms);

/*
 * 车体系速度剖面。
 * vx/vy/wz 是匀速段的峰值速度；ramp_ms 是加速段、减速段各自的时长，
 * cruise_ms 是匀速段时长。总时长 = 2*ramp + cruise。
 * 速度按 v(t)=峰值*(0.5-0.5*cos(pi*t/ramp)) 变化，结束时自然回到 0 再 STOP。
 */
void chassis_set_profile(Chassis *chassis, float vx_mps, float vy_mps,
                         float wz_radps, uint32_t ramp_ms, uint32_t cruise_ms,
                         uint32_t now_ms);

/*
 * 漂移路径剖面：vx/vy 是世界系速度峰值，运动期间用 IMU 航向
 * 把速度矢量旋转到车体系，并做航向 PD 锁到 target_world_yaw_deg。
 * 效果是“沿世界直线边走边纠/边转”，而不是先转到位再直行。
 */
void chassis_set_profile_world(Chassis *chassis,
                               float vx_world_mps, float vy_world_mps,
                               float target_world_yaw_deg,
                               uint32_t ramp_ms, uint32_t cruise_ms,
                               uint32_t now_ms);

/*
 * 漂移剖面的“相位衔接”版本：ramp_in/ramp_out 为 0 时首/尾不做正弦加减速，
 * 保持满速直接接上下一段（直线→圆弧→直线连续通过拐角）。
 * 总时长 = ramp_in*ramp_ms + cruise_ms + ramp_out*ramp_ms。
 */
void chassis_set_profile_world_ex(Chassis *chassis,
                                  float vx_world_mps, float vy_world_mps,
                                  float target_world_yaw_deg,
                                  uint8_t ramp_in, uint8_t ramp_out,
                                  uint32_t ramp_ms, uint32_t cruise_ms,
                                  uint32_t now_ms);

/*
 * 世界系圆弧剖面：世界速度方向与航向目标以 yaw_rate_dps 同步旋转，
 * 形成半径 R = speed / |yaw_rate| 的圆弧；航向 PD 只修残差。
 * 圆弧两端默认不加减速（要与前后直线段无缝衔接）。
 */
void chassis_set_profile_world_arc(Chassis *chassis,
                                   float start_world_yaw_deg,
                                   float yaw_rate_dps,
                                   float speed_mps,
                                   uint32_t total_ms,
                                   uint32_t now_ms);

/*
 * 倒车/前进通用版本：start_world_yaw_deg 始终是车体中心的切线方向
 * （=世界速度方向），reverse=1 时车头航向锁到切线+180°，车体沿切线
 * 倒着走。用于“倒车直线 → 倒车圆弧 → 倒车直线”的连续过弯。
 */
void chassis_set_profile_world_arc_ex(Chassis *chassis,
                                      float start_world_yaw_deg,
                                      float yaw_rate_dps,
                                      float speed_mps,
                                      uint8_t reverse,
                                      uint32_t total_ms,
                                      uint32_t now_ms);

void chassis_stop(Chassis *chassis);
bool chassis_position_done(const Chassis *chassis);
bool chassis_position_failed(const Chassis *chassis);
bool chassis_busy(const Chassis *chassis);

/* Call from a fast periodic scheduler (1 ms recommended). No delay and no busy wait. */
void chassis_update(Chassis *chassis, uint32_t now_ms);
