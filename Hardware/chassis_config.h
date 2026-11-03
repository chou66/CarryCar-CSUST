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
 * 速度模式正弦加减速剖面
 *
 * 导航/大位移走速度模式的一段正弦加减速曲线（0 -> 巡航 -> 0）。加速段/
 * 减速段各占 RAMP_MS，中间匀速段由距离决定；短行程会自动把速度曲线压成
 * “两个半段”，峰值仍然不超过调用方给的 speed_mps。
 *
 * 例外：K230 视觉对位的 mm 级微调仍走 EMM 位置模式（0xFD + 0x3A 到位），
 * 见 chassis_motion_translate_precise()。几毫米行程的剖面总时长会被压到
 * 4~8 ms，短于下面的 PROFILE_UPDATE_MS，整段位移经常一帧速度都发不出去，
 * 表现为车轮原地抖动；位置模式由驱动器按目标脉冲闭环，没有时间量化。
 *
 * PROFILE_UPDATE_MS 是速度刷新周期：EMM 在 acc=0 时按目标转速立即
 * 运行，刷新太快会让 CAN 发送队列长期占满，太慢则正弦曲线阶梯明显。
 * 一次四轮 + 同步帧约 12 ms 发完，10 ms 刷新会被状态机自然合并。
 *
 * RAMP_MS 同时决定“每拍台阶有多大”，也就是抖动的主因：
 *   峰值转速 rpm_peak 时，曲线最陡处斜率 = rpm_peak × (π/2) / RAMP，
 *   每个 10 ms 刷新周期跳变 ≈ rpm_peak × 15.7 / RAMP_MS (rpm)。
 *   248 rpm / 300 ms -> 每拍约 13 rpm，EMM 又会瞬时跟随，等于 80 Hz
 *   的转速台阶，实车表现为明显抖动；降到 174 rpm / 500 ms 后每拍约
 *   5.5 rpm，台阶小一半以上，起停也柔和。
 * 短行程（转角很小的转向）会自动压缩加速段，不会因为本值变大而变慢。
 * ============================================================ */
#define CHASSIS_PROFILE_RAMP_MS             500U
#define CHASSIS_PROFILE_UPDATE_MS            10U
#define CHASSIS_PROFILE_IMU_TIMEOUT_MS      300U

/* ============================================================
 * 漂移路径（边走边纠航向）的航向环
 *
 * 平移过程中把世界系速度矢量按当前航向旋到车体系，同时用 HWT101
 * 世界航向做 PD，把车头锁在目标航向上。参数含义与 EdgeMotion 的
 * 航向纠偏一致：远处最高 DRIFT_MAX_RPM，近处降到 MIN_RPM。
 * ============================================================ */
#define CHASSIS_DRIFT_YAW_KP                 4.4f
#define CHASSIS_DRIFT_YAW_KD                 0.4f
#define CHASSIS_DRIFT_YAW_DEADBAND_DEG      0.50f
#define CHASSIS_DRIFT_YAW_MAX_RPM            100U
#define CHASSIS_DRIFT_YAW_MIN_RPM              3U

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
 * 2026-10-31 复测（VOFA 0x41 后退 0.9 m，0.8 m/s / acc130，即导航工况；
 *   航向读 VOFA ch2 原始偏航，+ = CCW）：
 *   +0.0310 前馈生效时，后退 0.9 m 仍留有 +1.8 deg（CCW）残差。
 *   残差 = 裸漂移 + 前馈 -> 裸漂移仅 +0.2 deg（CCW），与 10-27 在 2.20 m/s
 *   下测到的 -1.6 deg（CW）方向相反：该项与车速强相关，且可能受云台/电机5
 *   姿态带来的重心横向变化影响，不能用高速工况的值外推到导航速度。
 *   新系数 = 0.0310 - (1.8*pi/180)/0.9 = -0.0039，即只需约 0.2 deg/0.9 m
 *   的 CW 前馈。该残差已接近 IMU 读数噪声（±0.2 deg），下次复测若在
 *   ±0.5 deg 内正负跳变，直接清零本系数，不要拿噪声当标定值。
 *
 *   同工况前进（0x40，0.9 m）：-0.0388 前馈（命令 -2.0 deg）下残差 -1.5 deg
 *   （CW），反推裸漂移 +0.5 deg（CCW），仍与 2.20 m/s 的 +2.0 deg 同向但小 4 倍。
 *   新系数 = -(0.5*pi/180)/0.9 = -0.0097。
 *   注意 0.8 m/s 下前进 +0.5 deg、后退 +0.2 deg 已同号（高速工况是一正一负），
 *   说明导航速度下主导项不再是左右轮行程差，更像重心横向偏置带来的同向蠕偏；
 *   这类漂移随云台/电机5 姿态变化，若换姿态后仍看到 ±0.5 deg 以上残差，
 *   说明该前馈需要按任务姿态分档，而不是继续微调这两个常数。
 *
 * 车体坐标约定：+theta = CCW，-theta = CW。
 */
//#define MOTION_CAL_Y_POS_TO_THETA          (-0.0349f)
//#define MOTION_CAL_Y_NEG_TO_THETA          (0.0194f)
#define MOTION_CAL_Y_POS_TO_THETA  (-0.0097f) /* 2026-10-31：0.8 m/s 工况，前馈生效后仍 -1.5°，裸漂移约 +0.5° */
#define MOTION_CAL_Y_NEG_TO_THETA  (-0.0039f) /* 2026-10-31：0.8 m/s 工况，前馈生效后仍 +1.8°，裸漂移约 +0.2° */
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
