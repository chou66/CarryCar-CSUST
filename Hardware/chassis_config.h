#pragma once

#include <stdint.h>
#include "mecanum_kinematics.h"

/* ============================================================
 * 1. 已确定的机械参数
 * ============================================================ */

#define CHASSIS_WHEEL_DIAMETER_M           0.07700f
#define CHASSIS_WHEEL_RADIUS_M             (CHASSIS_WHEEL_DIAMETER_M * 0.5f)

/* 前后轮中心距 219.29 mm */
#define CHASSIS_WHEELBASE_M                0.21929f
#define CHASSIS_HALF_LENGTH_M              (CHASSIS_WHEELBASE_M * 0.5f)

/* 左右轮中心距 161.91 mm */
#define CHASSIS_TRACK_WIDTH_M              0.16191f
#define CHASSIS_HALF_WIDTH_M               (CHASSIS_TRACK_WIDTH_M * 0.5f)

#define CHASSIS_GEAR_RATIO                 1.0f

/* 默认 1.8° 步进电机 + 16 细分 -> 200 * 16 = 3200 pulse/rev */
#define CHASSIS_MOTOR_STEP_ANGLE_DEG       1.8f
#define CHASSIS_MOTOR_MICROSTEP            16U
#define CHASSIS_MOTOR_PULSES_PER_REV       3200U

/* 第一阶段保守软件上限；如果你已有实测安全上限，可在这里替换。 */
#define CHASSIS_MAX_MOTOR_RPM              1000U

/* ============================================================
 * 2. 四轮安装方向（2026-09-29 实车悬空测试确认）
 *
 * 0/1 只表示 EMM 指令里的方向位，不代表统一物理 CW/CCW。
 * ID1=FL(left-front), ID2=RL(left-rear), ID3=RR(right-rear), ID4=FR(right-front)
 *
 * 实测：
 *   ID1 DIR=0 -> 车体前进方向
 *   ID2 DIR=0 -> 车体前进方向
 *   ID3 DIR=1 -> 车体前进方向
 *   ID4 DIR=1 -> 车体前进方向
 * ============================================================ */
#define CHASSIS_MOTOR_1_FORWARD_DIR        0U
#define CHASSIS_MOTOR_2_FORWARD_DIR        0U
#define CHASSIS_MOTOR_3_FORWARD_DIR        1U
#define CHASSIS_MOTOR_4_FORWARD_DIR        1U

/* ============================================================
 * 3. 实车脉冲标定（已完成）
 *
 * 实测标定：
 *   前进 450 mm -> 5849 pulse
 *   横移 450 mm -> 6030 pulse
 *   原地旋转 90 deg -> about 3850 pulse
 *
 * 左移 45.2 cm、右移 44.9 cm，左右差异很小，因此当前不拆分左右标定。
 * ============================================================ */
#define CHASSIS_FORWARD_PULSES_PER_M       12998.0f  /* 5849 pulse / 0.45 m */
#define CHASSIS_STRAFE_PULSES_PER_M        13437.0f  /* 6030 pulse / 0.45 m */
#define CHASSIS_ROTATE_PULSES_PER_RAD      2451.0f   /* about 3850 pulse / 90 deg */

/* ============================================================
 * 4. 有符号运动校准
 *
 * 核心坐标约定：+X 右，-X 左；+Y 前，-Y 后；+theta 逆时针。
 * 这些系数在 chassis_motion_move() 的核心入口统一应用，
 * 因此人工快捷函数和未来节点寻路 translate(dx,dy) 都不会绕过标定。
 * ============================================================ */
#define MOTION_CAL_X_POS_SCALE             1.0000f
#define MOTION_CAL_X_NEG_SCALE             1.0000f
#define MOTION_CAL_Y_POS_SCALE             1.0000f
#define MOTION_CAL_Y_NEG_SCALE             1.0000f
#define MOTION_CAL_THETA_POS_SCALE         1.0000f
#define MOTION_CAL_THETA_NEG_SCALE         1.0000f

/*
 * 横移 -> 前后方向串扰补偿（2026-09-29 实车标定）
 *
 * 第一轮补偿后再次实测：
 *   +X 右移 0.85 m：左后 -1.5 cm，右后 0 cm
 *      -> 中心约 -0.75 cm，因此把原来的后退补偿减小。
 *
 *   -X 左移 0.85 m：左后 +1.0 cm，右后 0 cm
 *      -> 中心约 +0.50 cm，因此把后退补偿略微增大。
 *
 * 2026-10-01 场地复测（新场地，节点坐标同步更新）：
 *   -X 左移 0.45 m 实测净后退约 3.0 cm。其中命令的补偿分量
 *   为 0.45*0.06118 = 2.75 cm，说明当前地面下左移原始前漂约为 0，
 *   原系数变成纯过补偿，因此左移系数清零，待 0.85 m 边长复测微调。
 *
 * 2026-10-02 复测：
 *   +X 右移 0.45 m 净后退约 1.0 cm。当前 -0.0165 在 0.45 m 已含
 *   -0.74 cm 后退命令，反推原始后漂仅约 0.26 cm/0.45 m，原系数过补偿。
 *   新系数 = 旧系数 - 净漂移/距离 = -0.0165 + 0.010/0.45 = +0.0057
 *   （改为小幅前进命令抵消轻微后漂）。
 *
 * 系数单位：m(Y correction) / m(strafe command)。
 * 这里修正的是车体中心的前后漂移；左右差值对应的偏航由下面
 * MOTION_CAL_X_*_TO_THETA 单独补偿。
 */
//#define MOTION_CAL_X_POS_TO_Y      (0.0057f)
//#define MOTION_CAL_X_NEG_TO_Y              (0.0f)

//////////////////////////////////////////////
#define MOTION_CAL_X_POS_TO_Y      (-0.029f)			//10.15改
#define MOTION_CAL_X_NEG_TO_Y      (-0.0230f)
///////////////////////////////////////////////
/*
 * 前后 -> 横向串扰补偿，单位 m(X correction) / m(move command)。
 *
 * 2026-10-01 实车观察：后退（-Y）会向左（-X）偏，故新增本组系数。
 * 填法（对应 signed core 的正方向约定）：
 *   MOTION_CAL_Y_NEG_TO_X = 左偏量(m) / 后退距离(m)
 * 实测标定：后退 0.45 m 左偏 2.0 cm -> +0.0444（命令 +X 向右抵消左偏）。
 * 若复测向右偏则改负值。前进（+Y）的对称系数暂无数据，保持 0。
 * 填数前先用 VOFA ch2 确认后退过程 ch2 偏航没有明显漂移，
 * 否则偏左可能是偏航造成的，先修航向再补位置。
 */
//#define MOTION_CAL_Y_POS_TO_X      (0.0f)
//#define MOTION_CAL_Y_NEG_TO_X      (0.0222f)
#define MOTION_CAL_Y_POS_TO_X      (0.0f)
#define MOTION_CAL_Y_NEG_TO_X      (0.0f)
/*
 * 前后 -> 偏航串扰补偿，单位 rad(theta correction) / m(move command)。
 *
 * 2026-10-02 实车观察：前进（+Y）0.45 m 航向约 +0.9 deg（CCW 左偏）。
 *   MOTION_CAL_Y_POS_TO_THETA = -(0.9*pi/180)/0.45 = -0.0349
 *   （命令负 theta 即 CW 自转抵消 CCW 漂移；theta 分量由
 *   mecanum_inverse_pos() 与平移合成同一次位置运动，边走边转）。
 * 2026-10-02 复测：后退（-Y）0.45 m 航向约 -0.5 deg（CW 右偏）。
 *   MOTION_CAL_Y_NEG_TO_THETA = -(-0.5*pi/180)/0.45 = +0.0194
 *   （命令正 theta 即 CCW 自转抵消 CW 漂移）。
 *
 * 2026-10-27 复测（VOFA 0x40/0x41 各 0.9 m，2.20 m/s / acc150，3 次平均）：
 *   前进：+2.0 deg（CCW 左偏），即 2.22 deg/m
 *     MOTION_CAL_Y_POS_TO_THETA = -(2.0*pi/180)/0.9 = -0.0388
 *   后退：-1.6 deg（CW 右偏），即 1.78 deg/m
 *     MOTION_CAL_Y_NEG_TO_THETA = +(1.6*pi/180)/0.9 = +0.0310
 *   两者方向相反、量级接近，符合“左右轮有效行程差”的几何失配特征
 *   （0.9 m 上等效左右差约 4.5~5.4 mm，即 0.5~0.6%）。
 *   10-15 曾把这组前馈清零，清零后裸漂移就是这个量级，因此恢复前馈。
 *   注意：本次标定工况是 VOFA 的 2.20 m/s / acc150（0.9 m 内实际峰值约
 *   0.83 m/s）；导航边是 0.8 m/s / acc100，峰值约 0.66 m/s。若任务中仍残差
 *   明显，按导航工况复测后再微调。
 *
 * 车体坐标约定：+theta = CCW，-theta = CW。
 */
//#define MOTION_CAL_Y_POS_TO_THETA          (-0.0349f)
//#define MOTION_CAL_Y_NEG_TO_THETA          (0.0194f)
#define MOTION_CAL_Y_POS_TO_THETA  (-0.0388f) /* 2026-10-27：前进 0.9 m 左偏 2° */
#define MOTION_CAL_Y_NEG_TO_THETA  (0.0310f)  /* 2026-10-27：后退 0.9 m 右偏 1.6° */
/*
 * 横移 -> 偏航串扰补偿，单位 rad(theta correction) / m(strafe command)。
 *
 * 2026-10-02 实车观察：右移（+X）0.45 m 航向约 +0.9 deg（CCW 左偏）。
 *   MOTION_CAL_X_POS_TO_THETA = -(0.9*pi/180)/0.45 = -0.0349
 *   （命令负 theta 即 CW 自转抵消 CCW 漂移；与 Y->theta 同机制，
 *   theta 分量由 mecanum_inverse_pos() 合成进同一次位置运动）。
 *   本组此前只是定义未接线（死代码），2026-10-02 起在
 *   chassis_motion_move() 中生效；旧场地 0.85 m 标定值已废弃。
 *
 * 左移（-X）系数暂无新场地数据，保持 0（旧值 +0.0727 已废弃）；
 * 复测后按 系数 = -(测得偏航 rad) / 距离(m) 填入。
 *
 * 车体坐标约定：+theta = CCW，-theta = CW。
 */
//#define MOTION_CAL_X_POS_TO_THETA          (-0.0349f)
//#define MOTION_CAL_X_NEG_TO_THETA          (0.0f)
#define MOTION_CAL_X_POS_TO_THETA  (-0.0f)
#define MOTION_CAL_X_NEG_TO_THETA  (-0.0602f)
/*
 * 把本文件真正接入 MecanumConfig。
 * 方向数组预留到 ID6，未使用位置保持 0。
 */
static inline void chassis_config_init_mecanum(MecanumConfig *cfg)
{
    uint8_t i;

    if (cfg == 0) return;

    cfg->wheel_radius_m = CHASSIS_WHEEL_RADIUS_M;
    cfg->half_length_m = CHASSIS_HALF_LENGTH_M;
    cfg->half_width_m = CHASSIS_HALF_WIDTH_M;
    cfg->gear_ratio = CHASSIS_GEAR_RATIO;
    cfg->max_motor_rpm = (uint16_t)CHASSIS_MAX_MOTOR_RPM;
    cfg->motor_pulses_per_rev = CHASSIS_MOTOR_PULSES_PER_REV;
    cfg->forward_pulses_per_m = CHASSIS_FORWARD_PULSES_PER_M;
    cfg->strafe_pulses_per_m = CHASSIS_STRAFE_PULSES_PER_M;
    cfg->rotate_pulses_per_rad = CHASSIS_ROTATE_PULSES_PER_RAD;

    for (i = 0U; i < 7U; ++i) cfg->forward_dir[i] = 0U;
    cfg->forward_dir[1] = CHASSIS_MOTOR_1_FORWARD_DIR;
    cfg->forward_dir[2] = CHASSIS_MOTOR_2_FORWARD_DIR;
    cfg->forward_dir[3] = CHASSIS_MOTOR_3_FORWARD_DIR;
    cfg->forward_dir[4] = CHASSIS_MOTOR_4_FORWARD_DIR;
}
