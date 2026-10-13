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

void Manipulator_StopAll(void);

#endif
