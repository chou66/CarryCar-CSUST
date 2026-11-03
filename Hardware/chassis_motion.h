#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "chassis.h"

/*
 * 比赛任务层的基础动作接口。
 * 坐标约定和 mecanum_kinematics 保持一致：
 *   +X = 右，-X = 左
 *   +Y = 前，-Y = 后
 *   +theta = 逆时针，-theta = 顺时针
 *
 * 未来节点寻路请优先调用 signed core 接口：
 *   chassis_motion_translate(dx, dy, ...)
 * 而不是先判断正负再选择 forward/backward/left/right。
 */

void chassis_motion_move(Chassis *chassis,
                         float dx_m, float dy_m, float dtheta_rad,
                         float speed_mps, uint32_t now_ms);

void chassis_motion_translate(Chassis *chassis,
                              float dx_m, float dy_m,
                              float speed_mps, uint32_t now_ms);

/*
 * 精确小位移入口：EMM 位置模式（0xFD + 0x3A 到位），mm 级位移由驱动器
 * 按目标脉冲闭环执行。专供 K230 视觉微调；导航大位移请继续用
 * chassis_motion_move / chassis_motion_translate（速度剖面）。
 *
 * 速度剖面的总时长在几毫米行程下会被压缩到短于刷新周期，导致车轮只收到
 * 一帧速度就被 STOP 打断，出现原地抖动；位置模式没有这个时间量化。
 */
void chassis_motion_move_precise(Chassis *chassis,
                                 float dx_m, float dy_m, float dtheta_rad,
                                 float speed_mps, uint32_t now_ms);

void chassis_motion_translate_precise(Chassis *chassis,
                                      float dx_m, float dy_m,
                                      float speed_mps, uint32_t now_ms);

/*
 * 漂移路径（边走边纠航向）。
 *
 * world_dx/world_dy  : 拓扑边的世界坐标向量，只取方向
 * distance_m         : 车身 Y 方向的有符号距离（负值=后退）
 * target_world_yaw_deg : 该边对应的目标世界航向（后退时已由 edge_turn 换成反方向）
 *
 * 与 chassis_motion_translate 的区别：平移过程中底盘按当前 IMU 航向把
 * 世界速度旋到车体系，并用航向 PD 锁住目标航向，所以车头有偏差时仍沿
 * 世界直线行驶（参考工程里的“漂移”）。完成判据仍是速度剖面时间到。
 */
void chassis_motion_drift_translate(Chassis *chassis,
                                    float world_dx_m, float world_dy_m,
                                    float distance_m,
                                    float target_world_yaw_deg,
                                    float speed_mps, uint32_t now_ms);

/*
 * 漂移直线段的“相位衔接”版本（拐角圆弧前/后使用）。
 *
 * ramp_in/ramp_out 为 0 时该端不做正弦加减速，保持满速与相邻的圆弧段
 * 无缝衔接，避免在拐角处先停车再起步。其余含义与
 * chassis_motion_drift_translate 完全一致。
 */
void chassis_motion_drift_segment(Chassis *chassis,
                                  float world_dx_m, float world_dy_m,
                                  float distance_m,
                                  float target_world_yaw_deg,
                                  float speed_mps,
                                  uint8_t ramp_in, uint8_t ramp_out,
                                  uint32_t now_ms);

/*
 * 世界系恒速圆弧（像汽车一样通过拐角）。
 *
 * start_world_yaw_deg : 圆弧起点切线方向（=进入前的直线航向）
 * turn_deg            : 有符号转角，世界系顺时针为正（0=+Y，顺时针+）
 * radius_m            : 圆弧半径
 * 时间 = 弧长 / speed，期间世界速度方向与车头目标同步旋转，
 * 因此车体中心沿半径 radius_m 的圆弧走到新方向。
 */
void chassis_motion_arc_world(Chassis *chassis,
                              float start_world_yaw_deg,
                              float turn_deg,
                              float radius_m,
                              float speed_mps,
                              uint32_t now_ms);

/*
 * 前进/倒车通用圆弧：start_tangent_world_yaw_deg 始终是车体中心的行进
 * 切线方向；reverse=1 时车头锁在切线+180°，沿弧倒车（倒车直线→倒车
 * 圆弧→倒车直线可以连续衔接）。
 */
void chassis_motion_arc_world_ex(Chassis *chassis,
                                 float start_tangent_world_yaw_deg,
                                 float turn_deg,
                                 float radius_m,
                                 float speed_mps,
                                 uint8_t reverse,
                                 uint32_t now_ms);

void chassis_motion_move_x(Chassis *chassis,
                           float dx_m, float speed_mps, uint32_t now_ms);

void chassis_motion_move_y(Chassis *chassis,
                           float dy_m, float speed_mps, uint32_t now_ms);

/*
 * 原地旋转走 EMM 位置模式（0xFD），不做速度剖面的时间量化；
 * 导航的节点原地转（如 90°）和 VOFA 人工旋转都用这组入口。
 */
void chassis_motion_rotate(Chassis *chassis,
                           float dtheta_rad, float speed_mps, uint32_t now_ms);

void chassis_motion_rotate_deg(Chassis *chassis,
                               float angle_deg, float speed_mps, uint32_t now_ms);

/* 人工调车快捷函数：内部仍回到同一个 signed core，不会绕开校准。 */
void chassis_motion_forward(Chassis *chassis, float distance_m,
                            float speed_mps, uint32_t now_ms);
void chassis_motion_backward(Chassis *chassis, float distance_m,
                             float speed_mps, uint32_t now_ms);
void chassis_motion_strafe_right(Chassis *chassis, float distance_m,
                                 float speed_mps, uint32_t now_ms);
void chassis_motion_strafe_left(Chassis *chassis, float distance_m,
                                float speed_mps, uint32_t now_ms);
void chassis_motion_rotate_ccw_deg(Chassis *chassis, float angle_deg,
                                   float speed_mps, uint32_t now_ms);
void chassis_motion_rotate_cw_deg(Chassis *chassis, float angle_deg,
                                  float speed_mps, uint32_t now_ms);

bool chassis_motion_busy(const Chassis *chassis);
bool chassis_motion_done(const Chassis *chassis);
bool chassis_motion_failed(const Chassis *chassis);
void chassis_motion_stop(Chassis *chassis);
