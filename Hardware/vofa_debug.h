#ifndef __VOFA_DEBUG_H
#define __VOFA_DEBUG_H
#include "main.h"
#include <stdint.h>

#define VOFA_DEBUG_PERIOD_MS 40U
#define VOFA_DEBUG_MAX_RPM       500U
#define VOFA_CHASSIS_MAX_RPM     100U

/* Formal chassis-motion validation.
 * 0.20 m/s is approximately 50 wheel RPM for the current 77 mm wheel.
 */
#define VOFA_FORMAL_MOVE_SPEED_MPS  0.50f
#define VOFA_FORMAL_MOVE_ACC        10U
#define VOFA_FORMAL_DISTANCE_M      0.45f
#define VOFA_FORMAL_ANGLE_DEG       90.0f

/*
 * HWT101 automatic heading correction after every navigation edge.
 *
 * Operation:
 *   1) Before an 85 cm edge starts, current yaw is stored as software zero.
 *   2) The chassis executes the edge.
 *   3) After a short settling time, relative yaw is checked.
 *   4) If |yaw| > deadband, ONE small opposite rotation is executed.
 *   5) Only then is Navigation_EdgeReached() called.
 *
 * This is deliberately one-shot for the first real-car test to avoid
 * oscillating back and forth around 0 degrees.
 */
#define VOFA_NAV_YAW_DEADBAND_DEG        0.50f
#define VOFA_NAV_YAW_MAX_START_DEG        8.00f
#define VOFA_NAV_YAW_CORRECT_RPM          20U
#define VOFA_NAV_YAW_CORRECT_ACC          10U
#define VOFA_NAV_YAW_SETTLE_MS            150U
#define VOFA_NAV_IMU_TIMEOUT_MS           300U
#define VOFA_NAV_YAW_CORRECT_TIMEOUT_MS  1800U

/* RX frame from VOFA, HEX mode:
 * AA 55 CMD P1 P2 P3 P4
 *
 * 01 stop all motors
 * 02 zero HWT101 yaw
 * 10 motor enable: P1=id, P2=0/1
 * 11 motor velocity: P1=id, P2=dir, P3=rpm/10, P4=acc
 * 12 stop one motor: P1=id
 * 13 ID5/ID6 relative position test: P1=id, P2=dir, P3:P4=pulses
 *    Test motion uses 30 RPM and acceleration 20.
 * 20 gripper: P1=0 open,1 mid,2 close
 * 21 carousel: P1=slot 0..2
 * 22 raw servo us: P1=0 gripper/1 carousel, P2:P3=us
 *
 * Chassis bench commands:
 * 30 chassis enable: P1=1 enable / 0 disable
 * 31 forward:       P1=rpm/10, P2=acc
 * 32 backward:      P1=rpm/10, P2=acc
 * 33 strafe left:   P1=rpm/10, P2=acc
 * 34 strafe right:  P1=rpm/10, P2=acc
 * 35 rotate CCW:    P1=rpm/10, P2=acc
 * 36 rotate CW:     P1=rpm/10, P2=acc
 * 37 chassis stop
 *
 * Formal chassis_motion/chassis_set_position validation:
 * 40 forward 450 mm
 * 41 backward 450 mm
 * 42 strafe left 450 mm
 * 43 strafe right 450 mm
 * 44 rotate CCW 90 deg
 * 45 rotate CW 90 deg
 * 46 stop formal position motion
 *
 * Navigation + Dijkstra bench:
 * 50 init/reset logical start node: P1=start node 1..9
 * 51 plan + physically drive to target: P1=target node 1..9
 * 52 abort navigation and stop chassis
 * 53 set static obstacle mask:
 *    mask = P1 | (P2<<8) | (P3<<16) | (P4<<24)
 *
 * Direct CAN smoke test (bypasses chassis/kinematics entirely):
 * 60 motor 1 rotates one revolution; ch9: 1=commanded+bus alive,
 *    13=no CAN reply from the driver (bus still broken)
 * 61 motor 5 (gimbal) reply probe, read-only S_FLAG query, no actuation
 * 62 motor 6 (slide) reply probe, same contract as 61
 *    ch9: 1=answered, 13=no reply in 60 ms
 *
 * Combined RobotApp diagnostic build:
 * 70 enter VOFA manual override. This E-stops/suspends RobotApp permanently
 *    until MCU reset. Motion commands are rejected before 0x70.
 *
 * Map orientation:
 *   7 -- 8 -- 9
 *   |    |    |
 *   4 -- 5 -- 6
 *   |    |    |
 *   1 -- 2 -- 3
 *
 * Robot body coordinate alignment for this test:
 *   vehicle FRONT points from node 1 toward node 4 (+Y)
 *   vehicle RIGHT points from node 1 toward node 2 (+X)
 *
 * Ch9 / last_result:
 * 0=no command, 1=accepted/running, 2=bad parameter or busy,
 * 3=unknown command, 4=formal position done, 5=formal position failed,
 * 6=navigation target reached, 7=navigation/no-path logic error,
 * 8=navigation physical edge motion failed,
 * 9=waiting for fresh HWT101 data,
 * 10=automatic edge-end yaw correction is running,
 * 11=yaw error too large to auto-correct safely,
 * 12=yaw correction timeout,
 * 13=motor1 one-rev test failed: no CAN reply,
 * 14=VOFA motion command rejected: send 0x70 first,
 * 15=VOFA takeover is stopping RobotApp; wait briefly and retry.
 */
void VofaDebug_Init(void);
void VofaDebug_FeedByte(uint8_t byte);
void VofaDebug_Update(uint32_t now_ms);

/* 1 = motor-1 one-rev test auto-runs 3 s after power-up. */
#define MOTOR1_REV_TEST_ON_BOOT   1

/*
 * Direct CAN smoke test: motor 1 turns exactly one revolution
 * (3200 pulses @ 16 microstep). Returns 1 if the driver answered the
 * pre-move status query (bus round trip OK), 0 if it did not.
 */
uint8_t Motor1_RevTest(void);

/*
 * Direct CAN smoke test: slide-rail motor (id 6) turns two revolutions.
 * Same probe-first contract; see vofa_debug.c for the pulses-per-rev note.
 */
uint8_t Slide_RevTest(void);
#endif
