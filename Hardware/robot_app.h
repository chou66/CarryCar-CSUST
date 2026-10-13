#ifndef __ROBOT_APP_H
#define __ROBOT_APP_H

#include <stdint.h>
#include "chassis.h"
#include "hmi_navigation.h"
#include "edge_motion.h"

void RobotApp_Init(void);
void RobotApp_Update(uint32_t now_ms);

uint8_t RobotApp_GotoNode(uint8_t target_node);
uint8_t RobotApp_GotoReturnNode(void);
void RobotApp_EStop(void);

/*
 * 10.7.2 VOFA diagnostic takeover.
 * Once entered, automatic RobotApp motion stays suspended until MCU reset.
 */
void RobotApp_EnterVofaOverride(void);
uint8_t RobotApp_IsVofaOverride(void);
uint8_t RobotApp_IsVofaOverrideReady(void);
extern volatile uint8_t robot_app_vofa_override;
extern volatile uint8_t robot_app_vofa_override_ready;

Chassis *RobotApp_GetChassis(void);
HMI_Navigation_t *RobotApp_GetNavigation(void);

/*
 * 10.7.1 full-route test debug variables.
 * Keil Watch can observe them directly:
 *   stage 0=QR, 1=RAW, 2=ROUGH, 3=STORAGE, 4=RETURN.
 */
extern volatile uint8_t robot_route_test_state;
extern volatile uint8_t robot_route_test_stage;
extern volatile uint8_t robot_route_test_target;
extern volatile uint8_t robot_route_test_error;

#endif
