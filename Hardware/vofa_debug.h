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
#define VOFA_FORMAL_MOVE_SPEED_MPS  2.20f
#define VOFA_FORMAL_MOVE_ACC        150U
#define VOFA_FORMAL_DISTANCE_M      1.00f
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
 * 40 forward by VOFA_FORMAL_DISTANCE_M (currently 1.00 m)
 * 41 backward by VOFA_FORMAL_DISTANCE_M (currently 1.00 m)
 * 42 strafe left by VOFA_FORMAL_DISTANCE_M (currently 1.00 m)
 * 43 strafe right by VOFA_FORMAL_DISTANCE_M (currently 1.00 m)
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
 * 粗加工区固定圆环/滑台绝对距离标定（必须先 70 接管）：
 * 80 自动走到粗加工 N2：P1=当前实际节点 1..9，A1=启停区1，A2=启停区2。
 *    从启停区会先按正式坐标退出到 N9/N3；发命令前车头必须朝地图 +Y；
 *    本状态机复用正式 EdgeMotion（先转向、只前行、边末 PD），到 N2 后继续
 *    用正式任务航向 PD 对准 +90 deg。P1=2 时不平移，只校正任务航向。
 * 81 确认滑台机械原点：P1=A5。此命令不会自动找原点，必须先人工缩到底；
 *    口令正确后才把当前位置立为软件 0 mm。
 * 82 选择圆环：P1=1/2/3。只允许在滑台 0 mm 时转云台并张开夹爪。
 * 83 滑台绝对位置：P1:P2=0.1 mm，大端在前。例如 125.0 mm -> 04 E2；
 *    P3/P4 填 00。可以连续发送不同绝对值做细调，软件上限 160.0 mm。
 * 84 滑台退回软件 0 mm。
 * 85 急停并退出标定；退出后软件位置作废，再次标定必须重新执行 81。
 * 86 粗加工云台绝对角：P1:P2=0.1 deg，大端在前。例如 232.5 deg -> 09 15；
 *    必须先用82选中待标定圆环。齿条伸出后也允许小幅调角，但齿条正在运动
 *    时拒绝；安全范围由 ROUGH_CAROUSEL_CAL_MIN/MAX_DEG 限制。
 * 标定阶段禁止用K230移动底盘：启停区2直达N2的物理停靠位被当作准确基准，
 * 先用72观察2号圆环，再只调云台/齿条让相机中心对准它。正式任务经过多轮
 * 行驶后才反过来固定这组2号机械位姿、用K230偏差平移底盘恢复基准位置。
 *
 * 暂存区固定圆环标定（必须先70接管，齿条物理缩回原点）：
 * 87 启动完整模拟流程，P1=A5（原点确认口令）：启停区2 -> N3 -> N2，
 *    在N2校正+90 deg并复现粗加工2号环的205 deg + 78 mm相机位姿，随后
 *    用K230真实平移底盘对准2号环，但不抓圆、不放圆；齿条回零后继续按
 *    N2 -> N5 -> N4前行，在N4校正到180 deg并停车，进入暂存区调参态。
 *    发87前车头必须朝地图+Y、车体必须在启停区2标定位置；A5只重建软件
 *    0 mm，不会自动找机械原点。自动流程中可随时发01或85急停。
 * 到N4且ch0=11(HOME_READY)后，复用82~86：82选暂存位，83调齿条绝对长度，
 * 84退回0 mm，86调云台绝对角，85退出。72可用于观察所选1/2/3号圆环；
 * 自动到N4之前拒绝72/73，避免与粗加工途中自动K230对准争用串口任务。
 * 88 锁定N4二号环基准：先82选2号、83伸到本轮候选长度并等ch0=15，
 *    保持云台205 deg，再发88（参数全00）。状态机用K230 PLACE-2平移底盘，
 *    随后复核180 deg航向；如果PD转过车，就再做一次K230，直到图像和航向
 *    同时对准。期间不抓放、不收齿条；完成后回到ch0=15，可继续观察、
 *    回零或调1/3号。ch0=17表示K230底盘对准正在运行。
 *
 * 遥测选择 74：P1=0 常规 IMU/CAN，P1=1 K230，P1=2 粗加工标定，
 * P1=3 暂存区标定。
 * P1=2 时：ch0=标定状态，ch1=标定错误，ch2=当前节点，ch3=世界航向角，
 * ch4=滑台当前位置(mm，-1 表示未知)，ch5=目标位置(mm)，ch6=云台目标角(deg)，
 * ch7=K230 齿条径向偏差(w为正/s为负)，ch8=上一命令，ch9=命令结果。
 * P1=3使用完全相同的ch0~ch9定义；状态11表示已到N4、可以开始调参。
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
 * 15=VOFA takeover is stopping RobotApp; wait briefly and retry,
 * 17=calibration owns actuators; conflicting command rejected,
 * 18=calibration rejected command; see ch1 error code.
 * 粗加工 ch1：1参数、2忙、3 IMU、4寻路、5边运动、6未回零、7机械臂、
 * 8启停区出库。暂存区相同，另加9=K230对准失败。
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
