#ifndef __SERVO_H
#define __SERVO_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/*
 * 舵机运行时标定参数
 * ------------------
 * 这几个变量故意不声明为 const，并增加 volatile：
 * 1. 可在 Keil Debug 的 Watch 窗口中直接修改角度，无需反复编译、下载；
 * 2. volatile 可确保正式动作每次执行时都从 RAM 重新读取调试后的数值；
 * 3. 变量只在本次上电期间有效，复位后会恢复 robot_config.h 中的默认宏值。
 *
 * 调试时请在机械机构停止后修改，避免舵机运动过程中改变目标角度造成碰撞。
 */
extern volatile float g_carousel_place_deg;       /* 云台抓取后转向物料盘的放料角度 */
extern volatile float g_tray_slot_deg[];          /* [0]/[1]/[2] 分别对应物料盘1/2/3号位 */

void CarouselServo_Update(uint32_t now_ms);
bool CarouselServo_IsDone(void);

void Servo_Init(void);

/* Convert a mechanical angle to PWM pulse width using a servo's travel. */
uint16_t Servo_AngleToUs(float angle_deg, float travel_deg);

/*GripperServo*/
void GripperServo_SetUs(uint16_t us);
void GripperServo_SetAngle(float angle_deg);
void GripperServo_Open_Small(void);
void GripperServo_Open_Large(void);
void GripperServo_Close(void);
void GripperServo_Mid(void);

void GripperServo_Update(uint32_t now_ms);
bool GripperServo_IsDone(void);
/*CarouselServo*/
void CarouselServo_SetUs(uint16_t us);
void CarouselServo_SetAngle(float angle_deg);
void CarouselServo_ToPickup(void);       /* 原函数继续代表夹取位2 */
void CarouselServo_ToPlace(void);
void CarouselServo_ToMid(void);

/*TraySelectorServo —— 板载PWM1 / TIM3_CH1*/
void TraySelectorServo_SetUs(uint16_t us);
void TraySelectorServo_SetAngle(float angle_deg);
void TraySelectorServo_GotoSlot(uint8_t slot);

void Pwm1Servo_SetUs(uint16_t us);

/*QrServo —— 板载PWM3 / TIM3_CH4（扫码模块转向，SG90）*/
void QrServo_SetUs(uint16_t us);
void QrServo_SetAngle(float angle_deg);
void QrServo_ScanStart(uint32_t now_ms);  /* 从下边界开始往返扫描 */
void QrServo_ScanStop(void);              /* 停止扫描并回 QR_SERVO_CENTER_DEG */
void QrServo_Update(uint32_t now_ms);     /* 扫描节拍，主循环周期调用 */

#endif
