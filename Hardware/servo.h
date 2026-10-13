#ifndef __SERVO_H
#define __SERVO_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

void CarouselServo_Update(uint32_t now_ms);
bool CarouselServo_IsBusy(void);
bool CarouselServo_IsDone(void);

void Servo_Init(void);

/*GripperServo*/
void GripperServo_SetUs(uint16_t us);
void GripperServo_Open_Small(void);
void GripperServo_Open_Large(void);
void GripperServo_Close(void);
void GripperServo_Mid(void);

void GripperServo_Update(uint32_t now_ms);
bool GripperServo_IsBusy(void);
bool GripperServo_IsDone(void);
/*CarouselServo*/
void CarouselServo_SetUs(uint16_t us);
void CarouselServo_ToPickupPos1(void);
void CarouselServo_ToPickup(void);       /* 原函数继续代表夹取位2 */
void CarouselServo_ToPickupPos3(void);
void CarouselServo_ToPlace(void);
void CarouselServo_ToMid(void);

/*TraySelectorServo*/
void TraySelectorServo_SetUs(uint16_t us);
void TraySelectorServo_GotoSlot(uint8_t slot);

void Pwm1Servo_SetUs(uint16_t us);

#endif
