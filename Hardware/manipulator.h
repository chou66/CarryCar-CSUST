#ifndef __MANIPULATOR_H
#define __MANIPULATOR_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

HAL_StatusTypeDef Manipulator_Gimbal_Velocity(uint8_t dir, uint16_t rpm, uint8_t acc);
HAL_StatusTypeDef Manipulator_Slide_Velocity(uint8_t dir, uint16_t rpm, uint8_t acc);

HAL_StatusTypeDef Manipulator_Gimbal_MovePulses(uint8_t dir, uint16_t rpm,
                                                uint8_t acc, uint32_t pulses);
HAL_StatusTypeDef Manipulator_Slide_MovePulses(uint8_t dir, uint16_t rpm,
                                               uint8_t acc, uint32_t pulses);

bool Manipulator_Gimbal_MoveMm(float mm, uint16_t rpm, uint8_t acc);
bool Manipulator_Slide_MoveMm(float mm, uint16_t rpm, uint8_t acc);

/*
 * 电机6（滑台）位置基准。
 *
 * 电机6 是相对位移控制，而 manipulator_action 的动作引擎假定「每段都从
 * 机械原点出发」。视觉识别前后各加一段滑台点动（前伸到识别位、退回原点）
 * 之后，中间那一段（识别期间）滑台停在非原点处，所以必须知道「现在到底
 * 停在哪」，否则下一段按原点算的位移会跑偏。
 *
 * 本工程只在两个已知落点之间来回，所以不跟踪毫米数，只跟踪基准：
 *   REF_HOME    —— 确知停在机械原点
 *   REF_ALIGN   —— 确知停在识别位（前伸点动成功后）
 *   REF_UNKNOWN —— 中途截停 / 失败，位置不可知
 *
 * REF_UNKNOWN 由 Manipulator_StopAll() 置入，而 StopAll 在急停、动作失败、
 * Abort 都会走到，所以「任何失败之后都必须人工归零」——恢复方式是人工把
 * 滑台推回机械原点后调用 Manipulator_Slide_SetHome()，或重新上电
 * （上电时基准默认为 REF_HOME，前提是上电时滑台确实停在原点）。
 *
 * 这里不发任何「自动回原点」的指令：急停路径不允许再动电机。
 */
bool Manipulator_Slide_AtHome(void);
bool Manipulator_Slide_AtAlign(void);
void Manipulator_Slide_SetHome(void);
void Manipulator_Slide_SetAlign(void);

/*
 * 位置基准镜像（非 static 全局，Keil Watch 直接输名字即可）：
 * 0=UNKNOWN，1=HOME，2=ALIGN。
 */
extern volatile uint8_t robot_slide_ref;

void Manipulator_StopAll(void);

#endif
