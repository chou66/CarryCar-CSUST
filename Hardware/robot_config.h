#ifndef __ROBOT_CONFIG_H
#define __ROBOT_CONFIG_H

/* ---------- CAN motor IDs ---------- */
#define MOTOR_FL_ID                 1U
#define MOTOR_RL_ID                 2U
#define MOTOR_RR_ID                 3U
#define MOTOR_FR_ID                 4U
#define MOTOR_GIMBAL_ID             5U
#define MOTOR_SLIDE_ID              6U

/* ---------- Route geometry ---------- */
/* Real field measurement: adjacent navigation-node centers are 850 mm apart.
 * The yellow forbidden square is 450 x 450 mm; that is NOT the node spacing.
 */
#define ROUTE_GRID_STEP_M           0.850f
#define ROUTE_EDGE_SPEED_MPS        2.200f

/* ---------- Servo channel mapping ----------
 * PWM1: PB4 / TIM3_CH1 = reserved
 * PWM2: PE3 / TIM3_CH2 = gripper rotating servo
 * PWM3: PB7 / TIM3_CH4 = tray selector servo
 * PWM4: PE4 / TIM3_CH3 = gripper open/close servo
 */
 
 /* ---------- Servo pulse limits ---------- */
#define SERVO_MIN_US                500U
#define SERVO_MAX_US                2500U
#define SERVO_SAFE_US               1500U

/* ---------- Gripper open/close servo: PWM4 ---------- */
#define GRIPPER_OPEN_US_Small     1648U  /* 张开（小） */
#define GRIPPER_OPEN_US_Large     1870U  /* 张开（大） */
#define GRIPPER_CLOSE_US    1500U  /* 闭合 */
#define GRIPPER_MID_US      1500U  /* 暂时保留 */

/* The old tested code supports 270/360-degree positional servo mapping.
 * This project defaults to 270 degrees because three 120-degree positions
 * fit at 0/120/240 degrees. Change this to the actual servo travel.
 */
#define CAROUSEL_TRAVEL_DEG         270U
#define CAROUSEL_SLOT0_DEG          0U
#define CAROUSEL_SLOT_STEP_DEG      120U
#define CAROUSEL_REVERSE            0U

/* ---------- Gripper rotating servo: PWM2 ---------- */
#define CAROUSEL_PICKUP_POS1_US         2278U  /* 夹取位1 */
#define CAROUSEL_PICKUP_POS2_US         2093U  /* 夹取位2，保留原名称 */
#define CAROUSEL_PICKUP_POS3_US         1797U  /* 夹取位3 */

#define CAROUSEL_PLACE_US     1382U  /* 放置位 */
#define CAROUSEL_MID_US       1500U  /* 安全中位 */

/* ---------- Material tray selector servo: PWM3 ---------- */
#define TRAY_SELECTOR_SLOT_COUNT        3U

#define TRAY_SELECTOR_SLOT0_US          1241U
#define TRAY_SELECTOR_SLOT1_US          1907U
#define TRAY_SELECTOR_SLOT2_US           574U

#define TRAY_SELECTOR_MID_US            1500U


/* ---------- Motor 5/6 mechanical calibration ----------
 * Leave 0.0 until measured. Raw pulse APIs remain usable.
 */
#define GIMBAL_DIR_DOWN              0U
#define GIMBAL_DIR_UP                1U
#define GIMBAL_PULSES_PER_MM         400.0f

#define SLIDE_DIR_EXTEND            1U  /* 前伸 */
#define SLIDE_DIR_RETRACT           0U  /* 后缩 */
#define SLIDE_PULSES_PER_MM         20.0f

/* 电机5（云台升降）与电机6（滑台前伸）各自独立的转速与加速度。
 *
 * 为什么要拆：两轴机械特性差得远——电机5 是 400 脉冲/mm 的竖直升降轴，
 * 带着整套夹爪机构和物料，惯量大；电机6 是 20 脉冲/mm 的水平前伸轴，
 * 轻载。原来两轴共用 MANIP_DEFAULT_RPM/ACC 一个值，想单独调其中一个
 * 就必须连另一个一起动，所以按轴拆开。
 *
 * 当前两项都沿用拆分前的原值 3000/0，行为与拆分前完全一致；
 * 标定后只改这里的数字即可，manipulator_action.c 的调用点不用再动。
 *
 * rpm：电机输出轴转速。与底盘 CHASSIS_MAX_MOTOR_RPM(1000) 无关，
 *      底盘那套钳位只作用于底盘运动学，管不到机械臂这两轴。
 * acc：Emm 固件加速度档位。0 = 不做加减速，直接跳到设定转速；
 *      acc!=0 时每 (256-acc)*50us 增/减 1 RPM，数字越大加速越猛。
 *      与底盘 chassis.c 用的是同一套语义（底盘当前取 100）。
 *      电机5 惯量大，建议给非 0 的 acc 并适当降 rpm，
 *      否则 0 档下 3000 rpm 的起停冲击全落在升降轴上。
 */
#define MANIP_GIMBAL_RPM            2000U  /* 电机5 升降转速 */
#define MANIP_GIMBAL_ACC            255U     /* 电机5 加速度档位 */
#define MANIP_SLIDE_RPM             2000U  /* 电机6 前伸转速 */
#define MANIP_SLIDE_ACC             200U     /* 电机6 加速度档位 */

/* Manipulator placement timing. */
#define MANIP_PLACE_TRAY_WAIT_MS         800U
/* 调试期临时放宽（原 15000U）：电机5全程接近15s，避免超时早停掩盖真实到位时间。 */
#define MANIP_PLACE_MOTOR_TIMEOUT_MS     40000U
#define MANIP_PLACE_STATUS_PERIOD_MS     50U
/* 到位确认守卫窗：段启动后经过该时间的新鲜 0x3A 应答才采信（同底盘 CHASSIS_STATUS_QUERY_GUARD_MS）。 */
#define MANIP_MOTOR_FEEDBACK_GUARD_MS    80U

/* 0x37 位置误差回读阈值（单位：电机脉冲）。到位确认已改回底盘式只看 0x02 标志，
 * 这组参数仅供必要时重新启用 perr 诊断/兜底。 */
#define MANIP_GIMBAL_PERR_TOL            40  /* 电机5：400脉冲/mm，40脉冲约0.1mm */
#define MANIP_SLIDE_PERR_TOL             10  /* 电机6：20脉冲/mm，10脉冲=0.5mm */

/* “运动确实展开”的证据门槛：段内位置误差必须先超过该值，小误差才可信。 */
#define MANIP_GIMBAL_PERR_LARGE          8000  /* 电机5：约20mm，小于最短段(42mm=16800)的一半 */
#define MANIP_SLIDE_PERR_LARGE            300  /* 电机6：15mm，小于40mm段(800脉冲)的一半 */

/* ---------- Manipulator relative movement ---------- */

/* Pickup movement*/
#define MANIP_PICKUP_GROUND_MM           -185.0f//从最高点下降到地面取料高度
#define MANIP_PICKUP_TABLE_MM            -105.0f//从最高点下降到原料台取料高度
#define MANIP_PICKUP_TRAY_MM             -42.0f //从最高点下降到物料盘取料高度
#define MANIP_PICKUP_SLIDE_MM            125.0f

/* ---------- Zone action poses ----------
 * 每个区域的“外部目标位姿”：
 *   MANIP_POSE(夹爪旋转脉宽, 电机6前伸mm, 电机5下降mm, 叠放层号)
 * 只描述去程参数；回程位移由 manipulator_action 内部按 -有效值现算。
 * 标定前全部沿用现有实测值，日后实测只改本节数字，不必动状态机。
 */

/* 位姿构造宏：与 manipulator_action.h 的 ManipPose_t 同构。 */
#define MANIP_POSE(us_, slide_mm_, gimbal_mm_, layer_) \
    { (uint16_t)(us_), (float)(slide_mm_), (float)(gimbal_mm_), (uint8_t)(layer_) }

/* 叠放层高：第二层相对第一层抬高的毫米数（正数=抬高）。
 * 物理标定前保持 0，此时 layer=1 与 layer=0 行为完全一致。 */
#define MANIP_LAYER_STEP_MM              0.0f

/* 原料区：原料台取料 -> 料盘 */
#define ZONE_RAW_POS1_POSE   MANIP_POSE(CAROUSEL_PICKUP_POS1_US, MANIP_PICKUP_SLIDE_MM, MANIP_PICKUP_TABLE_MM, 0U)
#define ZONE_RAW_POS2_POSE   MANIP_POSE(CAROUSEL_PICKUP_POS2_US, MANIP_PICKUP_SLIDE_MM, MANIP_PICKUP_TABLE_MM, 0U)
#define ZONE_RAW_POS3_POSE   MANIP_POSE(CAROUSEL_PICKUP_POS3_US, MANIP_PICKUP_SLIDE_MM, MANIP_PICKUP_TABLE_MM, 0U)

/* 粗加工区：料盘 <-> 三个工位（放料与取回共用同一组位姿） */
#define ZONE_ROUGH_POS1_POSE   MANIP_POSE(CAROUSEL_PICKUP_POS1_US, MANIP_PICKUP_SLIDE_MM, MANIP_PICKUP_GROUND_MM, 0U)
#define ZONE_ROUGH_POS2_POSE   MANIP_POSE(CAROUSEL_PICKUP_POS2_US, MANIP_PICKUP_SLIDE_MM, MANIP_PICKUP_GROUND_MM, 0U)
#define ZONE_ROUGH_POS3_POSE   MANIP_POSE(CAROUSEL_PICKUP_POS3_US, MANIP_PICKUP_SLIDE_MM, MANIP_PICKUP_GROUND_MM, 0U)

/* 暂存区：料盘 -> 三个暂存位。第一层 layer=0，第二层叠放用 _L2_ 版本（layer=1）。 */
#define ZONE_STORAGE_POS1_POSE      MANIP_POSE(CAROUSEL_PICKUP_POS1_US, MANIP_PICKUP_SLIDE_MM, MANIP_PICKUP_GROUND_MM, 0U)
#define ZONE_STORAGE_POS2_POSE      MANIP_POSE(CAROUSEL_PICKUP_POS2_US, MANIP_PICKUP_SLIDE_MM, MANIP_PICKUP_GROUND_MM, 0U)
#define ZONE_STORAGE_POS3_POSE      MANIP_POSE(CAROUSEL_PICKUP_POS3_US, MANIP_PICKUP_SLIDE_MM, MANIP_PICKUP_GROUND_MM, 0U)
#define ZONE_STORAGE_POS1_L2_POSE   MANIP_POSE(CAROUSEL_PICKUP_POS1_US, MANIP_PICKUP_SLIDE_MM, MANIP_PICKUP_GROUND_MM, 1U)
#define ZONE_STORAGE_POS2_L2_POSE   MANIP_POSE(CAROUSEL_PICKUP_POS2_US, MANIP_PICKUP_SLIDE_MM, MANIP_PICKUP_GROUND_MM, 1U)
#define ZONE_STORAGE_POS3_L2_POSE   MANIP_POSE(CAROUSEL_PICKUP_POS3_US, MANIP_PICKUP_SLIDE_MM, MANIP_PICKUP_GROUND_MM, 1U)

/* Motor status masks. */
#define MOTOR_GIMBAL_MASK \
    ((uint8_t)(1U << (MOTOR_GIMBAL_ID - 1U)))

#define MOTOR_SLIDE_MASK \
    ((uint8_t)(1U << (MOTOR_SLIDE_ID - 1U)))


/* ---------- Full Mission + world-frame motion ----------
 *
 * Mission order:
 *   HMI start bay -> road corner -> N6(QR) -> N8(RAW-1) -> N2(ROUGH-1)
 *   -> N4(STORAGE-1) -> N8(RAW-2) -> N2(ROUGH-2) -> N4(STORAGE-2)
 *   -> return road corner -> return bay (without task-yaw alignment).
 *
 * Task actions are now connected at N8/N2/N4 through zone_task.c
 * (ZONE_RAW / ZONE_ROUGH / ZONE_STORAGE step programs, see ZONE_*_POSE above).
 * N6 (QR) still uses the 2 s placeholder because the scanner link is not wired.
 */
#define MISSION_ENABLE                       1U
#define MISSION_QR_NODE                      6U
#define MISSION_RAW_NODE                     8U
#define MISSION_ROUGH_NODE                   2U
#define MISSION_STORAGE_NODE                 4U
#define MISSION_START_DELAY_MS            1500U
#define MISSION_TASK_HOLD_MS              2000U

/* Task-yaw alignment. World yaw is clockwise-positive; Chassis rotation is CCW-positive. */
#define MISSION_ALIGN_TOLERANCE_DEG         1.50f
#define MISSION_ALIGN_SETTLE_MS              200U
#define MISSION_ALIGN_MAX_ATTEMPTS             3U
#define MISSION_ALIGN_SPEED_MPS             0.20f

/* ---------- K230 视觉对位（每个作业点对一次） ----------
 * 上行偏差报文：<方向字符>,<无符号像素数>，例如 a,18（见 k230.c 的 process_line）。
 * 数值不带符号，往哪边偏完全由前面的字符表达。
 *
 * 方向字符是“夹取姿态下夹爪正向”的前/左/后/右，换算到车体系为：
 *   w 夹爪前 -> 车体右(+X)      s 夹爪后 -> 车体左(-X)
 *   a 夹爪左 -> 车体前(+Y)      d 夹爪右 -> 车体后(-Y)
 * 车朝偏差所指方向平移，即把目标移回夹爪正前方。
 *
 * 摄像头装在夹爪上：图像朝向随夹爪旋转，所以 w/a/s/d 只有在夹爪处于
 * 约定姿态时才有确定含义。对位的第一步就是把夹爪转到
 * K230_ALIGN_CAROUSEL_POS_US 指定的固定姿态（POS2），转到位之前不发检测
 * 命令，对位过程中也不再转动夹爪；夹取时才转到该步自己的 POSn。
 * 换一个基准姿态必须重推 offset_to_body()。
 *
 * 启动命令按区域分派：
 *   原料区          -> detect,<颜色号>   每个作业点发三轮，每轮一个颜色，
 *                                         识别到该颜色 -> 对位 -> 夹一块
 *                                         （颜色见 K230_RAW_ALIGN_B*_COLOR_*）
 *   粗加工区/暂存区 -> place,<位置号>    对准放置位
 */
#define K230_ALIGN_ENABLE                    1U    /* 置 0 可整段跳过，退回纯 IMU 对位 */
#define K230_ALIGN_REQUIRE_COLOR             1U    /* 1=原料区 DETECT 必须先收到目标颜色的独立报文
                                                    *   （整行只有一个字符 1~6）才开始纠偏：颜色确认
                                                    *   之前，偏差帧不消费、grab 也不算数。收不到颜色
                                                    *   就一直在 WAIT_OFFSET 等下去（该状态没有失败
                                                    *   超时，只按周期补发命令催），不会自行退出识别。
                                                    *   实测 K230 返回流是 "1 / w,215.25 ... / grab"，
                                                    *   颜色行排在最前，这条路走得通。
                                                    * 0=不校验颜色，收到偏差帧或 grab 就继续（旧行为：
                                                    *   没找到料时 K230 的偏差帧/grab 也会被当成
                                                    *   "已对准"而退出识别）。
                                                    * 粗加工区/暂存区走 PLACE 模式，K230 不报颜色，
                                                    * 本宏对它们无影响。 */
#define K230_ALIGN_PLACE_ID                  2U    /* 粗加工区/暂存区 place 的位置号（1~3） */
//#define K230_ALIGN_MM_PER_PX                0.5f   
#define K230_ALIGN_TOLERANCE_PX              5U    /* 偏差 <= 此值即认为对准 */
#define K230_ALIGN_MAX_ITER                  255U  /* 微调次数上限：255 是 uint8_t 上限，标定阶段等同于取消次数限制，
                                                    * 不再因为“调了几次”就失败；代价是失去次数兜底。 */
#define K230_ALIGN_STEP_MAX_MM             30.0f   /* 单次位移钳位，防 mm_per_px 标错时冲过头 */
#define K230_ALIGN_OFFSET_TIMEOUT_MS        1000U  /* 补发 detect/place 的周期。这里不设“等多久
                                                    * 就失败”的上限：颜色识别和偏差帧想等多久
                                                    * 就等多久，到点只补发一次命令催一催。
                                                    * 调大=催得慢，调小=催得勤。 */
#define K230_ALIGN_PAIR_WINDOW_MS            150U  /* 等另一正交轴偏差行的窗口。K230 一帧发两行
                                                    * （如 d,136.00 / w,102.50），两行到齐就合成为
                                                    * 一次斜向平移；若某轴偏差恰为 0 而 K230 不发
                                                    * 那一行，等满这个窗口就按单轴纠偏继续走，免得
                                                    * 每轮都空等。调大=更愿意等齐（一步到位但每轮慢），
                                                    * 调小=更容易退化成单轴处理（可能多走一轮）。 */
#define K230_ALIGN_SETTLE_MS                 300U  /* 移动后稳定等待，再取新偏差 */
#define K230_ALIGN_SPEED_MPS               0.12f   /* 视觉微调速度，慢于导航 */

/* ---------- 原料区三轮识别颜色 ----------
 * 原料区要把三块料依次抓进料盘 1/2/3，每块料之前各做一轮
 * “转夹爪到该步朝向 + 张开 + 发 detect,<颜色号> + 对位”，识别到该颜色才夹。
 * 三轮识别都用统一的识别位 POS2（见 K230_ALIGN_CAROUSEL_POS_US），
 * 夹取那一下才分别转 POS1/POS2/POS3——物料靠夹爪转身分别够到。
 * 颜色号本该由任务码给出：第一批取第一组、第二批取第三组三位数
 * （见设计文档《智能搬运机器人总体状态机流程设计.md》）；
 * 扫码链未接，先写死。颜色编号：1红 2黄 3蓝 4绿 5黑 6浅蓝。
 * 接入任务码后，把这两组宏换成解析结果即可，zone_task 不改。 */
#define K230_RAW_ALIGN_B1_COLOR_1            1U    /* 第一批第1块（POS1）要找的颜色 */
#define K230_RAW_ALIGN_B1_COLOR_2            2U    /* 第一批第2块（POS2） */
#define K230_RAW_ALIGN_B1_COLOR_3            3U    /* 第一批第3块（POS3） */
#define K230_RAW_ALIGN_B2_COLOR_1            1U    /* 第二批第1块（POS1） */
#define K230_RAW_ALIGN_B2_COLOR_2            2U    /* 第二批第2块（POS2） */
#define K230_RAW_ALIGN_B2_COLOR_3            3U    /* 第二批第3块（POS3） */

/* 对位时夹爪的统一朝向：识别前一律先转到取料位2（POS2）。
 * 原料区、粗加工区、暂存区都用这一个姿态——识别阶段不靠转夹爪区分目标，
 * K230 只按颜色号/位置号去找；夹取那一下才各自转 ZONE_*_POSn_POSE 里的
 * carousel_us。三个取料位约差 65°，POS2 居中，识别位选它视野覆盖最均。
 * 注意：换掉这个姿态必须重推 k230_align.c 的 offset_to_body()，
 * 因为 w/a/s/d 是在这个姿态下定义的。 */
#define K230_ALIGN_CAROUSEL_POS_US          CAROUSEL_PICKUP_POS2_US
#define K230_ALIGN_ROTATE_TIMEOUT_MS        2500U  /* 等夹爪转到位并张开；舵机无位置反馈，到时即继续 */


#endif
