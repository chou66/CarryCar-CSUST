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
#define VOFA_FORMAL_MOVE_SPEED_MPS  0.80f
#define VOFA_FORMAL_MOVE_ACC        130U
#define VOFA_FORMAL_DISTANCE_M      0.90f
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
 * 03 HWT101 zero-bias auto calibration:
 *    P1=1 start (send FF AA 48 01 00, keep the whole chassis still),
 *    P1=0 abort (send FF AA 48 00 00 immediately).
 *    P2=calibration seconds, 0 -> default 20 s (clamped to 5..120 s).
 *    Only accepted when the mission is idle or VOFA override is active.
 *    The exit command FF AA 48 00 00 is sent automatically when time is up,
 *    when P1=0, or when a chassis motion command arrives during calibration.
 * 10 motor enable: P1=id, P2=0/1
 * 11 motor velocity: P1=id, P2=dir, P3=rpm/10, P4=acc
 * 12 stop one motor: P1=id
 * 13 ID5/ID6 relative position test: P1=id, P2=dir, P3:P4=pulses
 *    Test motion uses 30 RPM and acceleration 20.
 * 20 gripper: P1=0 small-open(release), 1 mid, 2 close(grab),
 *             3 large-open(before descending to grab)
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
 * 粗加工区固定圆环/滑台绝对距离标定（必须先70接管）：
 *
 * ==================== 高位识别：旧流程，完整保留 ====================
 * 80 自动走到粗加工N2，电机5保持最高原点识圆。
 *    P1=当前实际节点1..9，A1=启停区1，A2=启停区2。
 *    从启停区会先按正式坐标退出到 N9/N3；发命令前车头必须朝地图 +Y；
 *    本状态机复用正式 EdgeMotion（先转向、只前行、边末 PD），到 N2 后继续
 *    用正式任务航向 PD 对准 +90 deg。P1=2 时不平移，只校正任务航向。
 *
 * ==================== 低位识别：新流程，匹配正式逻辑 ====================
 * 89 路线和P1含义与80完全相同；区别在0x83第一次从原点伸到候选长度后，
 *    自动把电机5下降ROUGH_VISION_PRE_DROP_MM，再观察圆心和微调长度。
 *    后续连续0x83只微调滑台，不会重复下降；0x84会先抬升电机5再收滑台。
 *
 * 以下81~86由高、低两套粗加工标定共用：
 * 81 确认滑台机械原点：P1=A5。此命令不会自动找原点，必须先人工缩到底；
 *    口令正确后才把当前位置立为软件 0 mm。
 * 82 选择圆环：P1=1/2/3。只允许在滑台 0 mm 时转云台并张开夹爪。
 * 83 滑台绝对位置：P1:P2=0.1 mm，大端在前。例如 125.0 mm -> 04 E2；
 *    P3/P4 填 00。可以连续发送不同绝对值做细调，软件上限 160.0 mm。
 * 84 滑台退回软件0 mm；低位模式会先抬电机5，再收回滑台。
 * 85 急停并退出标定；急停不会自动抬升/回零。低位模式正常结束必须先84，
 *    再85；若直接85，需人工确认机械姿态并复位后再开始下一轮标定。
 * 86 粗加工云台绝对角：P1:P2=0.1 deg，大端在前。例如 232.5 deg -> 09 15；
 *    必须先用82选中待标定圆环。齿条伸出后也允许小幅调角，但齿条正在运动
 *    时拒绝；安全范围由 ROUGH_CAROUSEL_CAL_MIN/MAX_DEG 限制。
 * 8B 旧版低位实际落料点验证：P1:P2=电机5从最高原点累计下降0.1 mm，
 *    P3/P4=00。保留兼容；粗加工区与暂存区均使用绝对高度和相同下探方式。
 *    仅0x89/0x8A低位模式、圆环已选且0x83到位后有效。当前配置下：
 *      03 E8 = 100.0 mm，回到低位取像高度；
 *      04 E2 = 125.0 mm，验证暂存区第二层高度；
 *      07 3A = 185.0 mm。粗加工区和暂存区均允许从最高原点或100 mm
 *                观察位直接到达；数值始终是相对最高原点的绝对下降量。
 *    粗加工区下探到放置高度后仍允许用0x83/0x86（或Watch版8D/8C）微调
 *    1/2/3号圆环；云台不限制单次角度差，齿条单次最多10 mm，注意观察干涉；
 *    暂存区采用相同方法：云台不限制单次角度差，齿条单次最多10 mm。
 *    也可直接发84，由状态机从当前高度完整抬到最高原点后收回滑台。
 *    2号环必须分两次记数：K230居中时的滑台值写入*_VISION_LOW_SLIDE_POS2_MM；
 *    之后按“8B下探 -> 8B抬回 -> 83/86微调”得到的夹爪落点，写入普通
 *    *_SLIDE_POS2_MM和*_CAROUSEL_POS2_DEG。两组值不得互相覆盖。
 *
 * ==================== VOFA执行 + Keil Watch填参数 ====================
 * 下列命令的P1必须填写操作者期望的当前圆环号1/2/3，P2~P4填00。
 * 状态机内部圆环号与P1不一致时会拒绝动作，防止选1号失败后误在2号执行。
 * 它们只负责“执行动作”，具体数值直接
 * 读取Keil Watch中的十进制浮点变量，不用再换算0.1单位的十六进制：
 *   8C P1=环号：应用cal_dbg_carousel_deg，转到Watch中的云台角度；
 *   8D P1=环号：应用cal_dbg_slide_mm，齿条进入Watch中的绝对伸长位置；
 *   8E P1=环号：应用cal_dbg_observe_drop_mm，丝杆回到观察绝对高度；
 *   8F P1=环号：应用cal_dbg_place_drop_mm，丝杆到放置候选绝对高度；
 *   90 P1=环号：只把丝杆抬回最高原点，齿条保持当前伸长量；
 *   91 P1=环号：只把齿条收回0 mm，要求丝杆已经位于最高原点。
 * 其中8C/8D分别等价于原86/83；8E/8F最终仍走8B的高度状态机，但把观察
 * 和放置两个参数明确分开。粗加工区、暂存区的8F均允许直达已验证的
 * 185 mm绝对高度；向上回观察位、或90回最高原点也可以一次完成。
 * 原8B和组合回收84仍完整保留。
 * 例：已选1号，Watch改cal_dbg_slide_mm=121.5，再发8D 01 00 00 00执行。
 * 上述命令仍经过标定状态和高度安全互锁，不是绕过检查的裸电机指令。
 * 88 在当前标定区执行2号环底盘对准：粗加工N2或暂存N4都必须先选2号、
 * 云台保持K230固定识别角、齿条和丝杆停在观察位。它会真正根据K230结果
 * 平移底盘；不是只开摄像头观察。72只启动K230观察输出，不移动底盘。
 *
 * 暂存区固定圆环标定（必须先70接管，齿条物理缩回原点）：
 *
 * ==================== 高位识别：旧流程，完整保留 ====================
 * 87 启动高位完整模拟流程，P1=A5（原点确认口令）：启停区2 -> N3 -> N2，
 *    在N2校正+90 deg并复现粗加工2号环的205 deg + 78 mm相机位姿，随后
 *    用K230真实平移底盘对准2号环，但不抓圆、不放圆；齿条回零后继续按
 *    N2 -> N5 -> N4前行，在N4校正到180 deg并停车，进入暂存区调参态。
 *    发87前车头必须朝地图+Y、车体必须在启停区2标定位置；A5只重建软件
 *    0 mm，不会自动找机械原点。自动流程中可随时发01或85急停，但急停只
 *    停止执行器，不会自动抬升电机5或收回滑台。
 *
 * ==================== 低位识别：新流程，匹配正式逻辑 ====================
 * 8A 路线和A5口令与87相同，但途经N2时按正式逻辑下降
 *    ROUGH_VISION_PRE_DROP_MM识别；到N4后，第一次0x83伸出还会下降
 *    STORAGE_VISION_PRE_DROP_MM。后续0x83只微调，0x84先抬升再收回。
 *
 * 到N4且ch0=11(HOME_READY)后，复用82~86：82选暂存位，83调齿条绝对长度，
 * 84退回0 mm，86调云台绝对角，85退出。72可用于观察所选1/2/3号圆环；
 * 自动到N4之前拒绝72/73，避免与粗加工途中自动K230对准争用串口任务。
 * 88 锁定当前区二号环基准：粗加工区等ch0=9；暂存区先82选2号，再把
 *    cal_dbg_slide_mm改为95并发8D 02伸到相机基础长度，等ch0=15，
 *    保持云台205 deg，再发88（参数全00）。状态机用K230 PLACE-2平移底盘，
 *    随后复核180 deg航向；如果PD转过车，就再做一次K230，直到图像和航向
 *    同时对准。期间不抓放、不收齿条；完成后回到ch0=15，可继续观察、
 *    回零或调1/2/3号。ch0=17表示K230底盘对准正在运行。高位87只允许在
 *    电机5最高点执行88；低位8A只允许在完成100 mm预下降后执行88。
 *
 * 遥测选择 74：P1=0 常规 IMU/CAN，P1=1 K230，P1=2 粗加工标定，
 * P1=3 暂存区标定。
 * P1=2 时：ch0=标定状态，ch1=标定错误，ch2=当前节点，ch3=世界航向角，
 * ch4=滑台当前位置(mm，-1 表示未知)，ch5=目标位置(mm)，ch6=云台目标角(deg)，
 * ch7=K230 齿条径向偏差(w为正/s为负)，ch8=上一命令，ch9=命令结果。
 * P1=3使用完全相同的ch0~ch9定义；状态11表示已到N4、可以开始调参。
 * 高度调试时Keil Watch优先观察cal_dbg_current_drop_mm（当前绝对下降量，
 * 正数）和cal_dbg_last_gimbal_delta_mm（本次相对位移，正=下、负=上）；底层
 * robot_manip_vision_drop_mm仍是负数方向，仅供核对内部账目。
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
 * 0=no command, 1=accepted/running, 2=bad parameter or busy
 *   (0x03 also returns 2 when the mission is running, the chassis is still
 *    moving, calibration is already running, or the USART1 TX failed),
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

/*
 * ==================== Keil Watch 直接标定面板 ====================
 *
 * 不需要手工组合 VOFA 二进制帧。使用方法是：
 *   1. 将目标参数（cal_dbg_ring / *_deg / *_mm）直接改成十进制数；
 *   2. 把需要执行的 *_req 写成1；
 *   3. 程序执行一次后自动把该 *_req 清零。
 *
 * 只改参数不会立即转动，必须再触发对应 *_req，避免在 Watch
 * 里输入到一半时电机就动。Watch请求完全复用原VOFA命令处理，
 * 所以接管、状态、范围、高度安全互锁仍然有效。
 *
 * 标准低位粗加工顺序：
 *   cal_dbg_takeover_req=1              RobotApp接管，等ready后再继续；
 *   cal_dbg_rough_start_req=1           固定从启停区2跑到N2；
 *   cal_dbg_home_confirm_req=1          人工确认齿条在机械原点；
 *   cal_dbg_ring=2, cal_dbg_select_ring_req=1   选中2号环；
 *   填角度/滑台/下降值，再分别触发对应apply按钮。
 *
 * 标准低位暂存顺序：
 *   齿条物理缩回后 takeover_req=1，再 storage_start_req=1；
 *   状态机自动复现启停区2 -> N2 -> N4，到N4后从选环开始调参。
 *
 * cal_dbg_last_result 与 VOFA ch9 一致：1=已接受，2=参数/忙，
 * 14=未接管，15=接管停车尚未完成，18=标定状态机拒绝。
 * 拒绝时再配合 rough_cal_error / storage_cal_error 判断具体原因。
 */

/* cal_dbg_last_action的动作编号，仅供回读，操作时不需要记这些数字。 */
#define CAL_DBG_ACTION_NONE              0U
#define CAL_DBG_ACTION_TAKEOVER          1U
#define CAL_DBG_ACTION_ROUGH_START       2U
#define CAL_DBG_ACTION_STORAGE_START     3U
#define CAL_DBG_ACTION_HOME_CONFIRM      4U
#define CAL_DBG_ACTION_SELECT_RING       5U
#define CAL_DBG_ACTION_APPLY_CAROUSEL    6U
#define CAL_DBG_ACTION_APPLY_SLIDE       7U
#define CAL_DBG_ACTION_APPLY_DROP        8U
#define CAL_DBG_ACTION_RETRACT           9U
#define CAL_DBG_ACTION_K230_START       10U
#define CAL_DBG_ACTION_K230_STOP        11U
#define CAL_DBG_ACTION_STORAGE_ALIGN    12U
#define CAL_DBG_ACTION_ABORT            13U
#define CAL_DBG_ACTION_ESTOP            14U
#define CAL_DBG_ACTION_APPLY_OBSERVE_DROP 15U
#define CAL_DBG_ACTION_APPLY_PLACE_DROP 16U
#define CAL_DBG_ACTION_RAISE_ONLY       17U
#define CAL_DBG_ACTION_SLIDE_RETRACT_ONLY 18U
#define CAL_DBG_ACTION_CHASSIS_ALIGN    19U

extern volatile uint8_t cal_dbg_takeover_req;
extern volatile uint8_t cal_dbg_rough_start_req;
extern volatile uint8_t cal_dbg_storage_start_req;
extern volatile uint8_t cal_dbg_home_confirm_req;

extern volatile uint8_t cal_dbg_ring;
extern volatile uint8_t cal_dbg_select_ring_req;
extern volatile float cal_dbg_carousel_deg;
extern volatile uint8_t cal_dbg_apply_carousel_req;
extern volatile float cal_dbg_slide_mm;
extern volatile uint8_t cal_dbg_apply_slide_req;
extern volatile float cal_dbg_drop_mm;
extern volatile uint8_t cal_dbg_apply_drop_req;
/* 观察/放置均为相对丝杆最高原点的正累计下降量，绝不是相对当前位置。 */
extern volatile float cal_dbg_observe_drop_mm;
extern volatile float cal_dbg_place_drop_mm;
extern volatile uint8_t cal_dbg_apply_observe_drop_req;
extern volatile uint8_t cal_dbg_apply_place_drop_req;
extern volatile uint8_t cal_dbg_raise_only_req;
extern volatile uint8_t cal_dbg_slide_retract_only_req;
extern volatile float cal_dbg_current_drop_mm;
extern volatile float cal_dbg_last_gimbal_delta_mm;

extern volatile uint8_t cal_dbg_retract_req;
extern volatile uint8_t cal_dbg_k230_start_req;
extern volatile uint8_t cal_dbg_k230_stop_req;
extern volatile uint8_t cal_dbg_storage_align_req;
extern volatile uint8_t cal_dbg_chassis_align_req;
extern volatile uint8_t cal_dbg_abort_req;
extern volatile uint8_t cal_dbg_estop_req;

extern volatile uint8_t cal_dbg_last_action;
extern volatile uint8_t cal_dbg_last_result;
extern volatile uint32_t cal_dbg_execute_count;

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
