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
/* 1=任务尚未启动或已结束/失败，允许做不影响执行器的维护操作。 */
uint8_t RobotApp_IsMissionIdle(void);
extern volatile uint8_t robot_app_vofa_override;
extern volatile uint8_t robot_app_vofa_override_ready;

/* 正式任务诊断：供Keil Watch观察航向确认和“圆心+航向”联合锁定。 */
extern volatile uint8_t robot_mission_state;
extern volatile uint8_t robot_mission_stage;
extern volatile uint8_t robot_mission_target;
extern volatile uint8_t robot_mission_error;
extern volatile float robot_world_yaw_deg;
extern volatile float robot_mission_target_yaw_deg;
extern volatile float robot_mission_yaw_error_deg;
extern volatile float robot_mission_align_wz_radps;
extern volatile uint8_t robot_mission_align_attempt;
extern volatile uint8_t robot_mission_align_stable_samples;
extern volatile uint8_t robot_mission_pose_lock_active;
extern volatile uint8_t robot_mission_pose_lock_yaw_changed;
extern volatile uint8_t robot_mission_task_step;
extern volatile uint8_t robot_mission_task_steps;
extern volatile uint8_t robot_mission_vision_iteration;

/* 二维码任务解析结果：批次下标0/1，物料下标0..2。 */
extern volatile uint8_t robot_task_code_valid;
extern volatile uint8_t robot_task_color[2][3];
extern volatile uint8_t robot_task_rough_slot[2][3];
extern volatile uint8_t robot_task_storage_slot[2][3];

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
