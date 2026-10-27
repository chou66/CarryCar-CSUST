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

/*
 * 导航直线速度。按 120 rpm 反算：
 *   v = rpm × 2π × 轮半径 / 60 = 120 × 2π × 0.0385 / 60 ≈ 0.484 m/s
 * （换算系数见 mecanum_kinematics.c：rpm = v × 60 / (2π × 轮半径)，当前约
 * 248 rpm 每 m/s。改轮径后本值必须重算。）
 */
#define ROUTE_EDGE_SPEED_MPS        0.8f

/*
 * 拓扑边只允许车身纵向运动，不再用大距离横移：
 *   - 车头更接近边方向时，用 +Y 前进；
 *   - 车尾更接近边方向时，允许用 -Y 后退，避免无意义地原地转 180 deg。
 *
 * 前进/后退的选择由 edge_turn 统一计算。只有后退方案至少少转下面设定的
 * 角度时才选后退；接近 90 deg 的临界位置默认前进，防止很小的 IMU 波动
 * 改变方案。若当前边终点是本次导航的任务节点，还会把随后必做的任务航向
 * 转角一并计入，避免“路上少转了一点，到任务点又多转 180 deg”。
 */
#define ROUTE_TURN_TOLERANCE_DEG                 0.80f
#define ROUTE_ALLOW_REVERSE                         1U
#define ROUTE_REVERSE_MIN_TURN_SAVING_DEG         5.00f
#define ROUTE_TURN_SETTLE_MS           150U
#define ROUTE_TURN_MAX_ATTEMPTS          3U
#define ROUTE_TURN_SPEED_MPS           0.50f
#define ROUTE_TURN_ACC                  130U

/* Route preference: N5 is the central four-way intersection.  The planner
 * uses this only as a secondary criterion after total distance, so a longer
 * detour is never selected merely to pass through the center. */
#define ROUTE_CENTER_NODE            5U
#define ROUTE_CENTER_TIE_BREAK       1U

/* ---------- Servo channel mapping ----------
 * PWM1: PB4 / TIM3_CH1 = reserved
 * PWM2: PE3 / TIM3_CH2 = gripper rotating servo
 * PWM3: PB7 / TIM3_CH4 = tray selector servo
 * PWM4: PE4 / TIM3_CH3 = gripper open/close servo
 */
 
/* ---------- Servo angle/pulse mapping ----------
 * All mechanical positions below are calibrated in degrees.  The driver
 * converts them to PWM pulse width only when a command is issued:
 *
 *   pulse_us = SERVO_MIN_US
 *            + angle_deg / travel_deg * (SERVO_MAX_US - SERVO_MIN_US)
 *
 * PWM4 is a 270-degree positional servo; PWM2 and PWM3 are both 360-degree
 * positional servos, so their calibrated angles are true mechanical degrees
 * (0..360 maps to 500..2500 us).  SERVO_MIN_US/SERVO_MAX_US are electrical
 * limits, not mechanical-position calibration values, so they stay in us.
 *
 * 2026-10-25: PWM2 (carousel) changed from a 270-degree to a 360-degree
 * servo.  Every carousel calibration value below was rescaled by 360/270
 * (= 4/3) when this changed, so the emitted pulse widths — and therefore the
 * real mechanical poses — are identical to the previous build.  When copying
 * old bench notes that say "205 deg", multiply by 4/3 to get this file's
 * value.  Vision PID output is in carousel degrees too, so RAW_VISION_ANGLE_*
 * was rescaled alongside the angles.
 */
#define SERVO_MIN_US                500U
#define SERVO_MAX_US                2500U
#define SERVO_CENTER_US             1500U

#define GRIPPER_TRAVEL_DEG          270.0f
#define CAROUSEL_TRAVEL_DEG         360.0f
#define TRAY_SELECTOR_TRAVEL_DEG    360.0f

/* ---------- Gripper open/close servo: PWM4 ---------- */
#define GRIPPER_OPEN_SMALL_DEG      155.0f /* VOFA PWM4: AA 55 22 03 06 70 00 (1648 us) */
#define GRIPPER_OPEN_LARGE_DEG      185.0f /* VOFA PWM4: AA 55 22 03 07 4E 00 (1870 us) */
#define GRIPPER_CLOSE_DEG           130.0f /* VOFA PWM4: AA 55 22 03 05 DC 00 (1500 us) */
#define GRIPPER_MID_DEG             135.0f /* VOFA PWM4: AA 55 22 03 05 DC 00 (1500 us) */

/* ---------- Gripper rotating servo: PWM2 ---------- *///云台参数
/*
 * CAROUSEL_PLACE_DEG 是云台放料角度的上电默认值。
 * 运行中真正使用的是 servo.c 的 g_carousel_place_deg，可在 Debug Watch 调节；
 * 调好后再把最终值写回本宏，确保重新上电后仍保留标定结果。
 */
/* 360° 舵机刻度：0..360 对应 500..2500 us（每度 5.556 us）。
 * 数值 = 旧 270° 刻度值 × 4/3，脉宽与实车姿态保持不变。 */
#define CAROUSEL_PICKUP_POS1_DEG  316.67f /* 2259 us（旧刻度 237.5；2204 us 是更早 POS1=230° 时的注释） */
#define CAROUSEL_PICKUP_POS2_DEG  273.33f /* 2019 us */
#define CAROUSEL_PICKUP_POS3_DEG  232.67f /* 1793 us */
#define CAROUSEL_PLACE_DEG        145.33f /* 1307 us；云台偏转到物料盘上方的放料角度 */
#define CAROUSEL_MID_DEG          180.00f /* VOFA PWM2: AA 55 22 01 05 DC 00 (1500 us) */
#define CAROUSEL_POWERON_DEG        CAROUSEL_PICKUP_POS3_DEG /* 上电初始：停在夹取位3 */

/* ---------- Material tray selector servo: PWM3 ---------- *///料盘参数
/*
 * 下面三个宏是物料盘1/2/3号位的上电默认值。
 * 运行时分别对应 g_tray_slot_deg[0]/[1]/[2]，可在 Debug Watch 直接调节；
 * 调好后再把最终值写回对应宏，避免复位后丢失本次标定结果。
 */
#define TRAY_SELECTOR_SLOT_COUNT        3U
#define TRAY_SELECTOR_SLOT0_DEG  78.0f /* 1号位理论初值：以2号位为基准减120度 */
#define TRAY_SELECTOR_SLOT1_DEG  198.0f /* 2号位当前实车标定基准 */
#define TRAY_SELECTOR_SLOT2_DEG  318.4f /* 3号位理论初值：以2号位为基准加120度 */
#define TRAY_SELECTOR_MID_DEG        180.0f /* VOFA PWM3: AA 55 22 02 05 DC 00 (1500 us) */


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
#define MANIP_GIMBAL_RPM            3000U  /* 电机5 升降转速 */
#define MANIP_GIMBAL_ACC               0U     /* 电机5 加速度档位 */
#define MANIP_SLIDE_RPM              120U  /* 电机6 前伸转速 */
#define MANIP_SLIDE_ACC              220U     /* 电机6 加速度档位 */

/* Manipulator placement timing. */
#define MANIP_PLACE_TRAY_WAIT_MS         800U
/* 调试期临时放宽（原 15000U）：电机5全程接近15s，避免超时早停掩盖真实到位时间。 */
#define MANIP_PLACE_MOTOR_TIMEOUT_MS     40000U
#define MANIP_PLACE_STATUS_PERIOD_MS     20U
/* 到位确认守卫窗：段启动后经过该时间的新鲜 0x3A 应答才采信（同底盘 CHASSIS_STATUS_QUERY_GUARD_MS）。 */
#define MANIP_MOTOR_FEEDBACK_GUARD_MS    50U

/* 0x37 位置误差回读阈值（单位：电机脉冲）。到位确认已改回底盘式只看 0x02 标志，
 * 这组参数仅供必要时重新启用 perr 诊断/兜底。 */
#define MANIP_GIMBAL_PERR_TOL            40  /* 电机5：400脉冲/mm，40脉冲约0.1mm */
#define MANIP_SLIDE_PERR_TOL             10  /* 电机6：20脉冲/mm，10脉冲=0.5mm */

/* “运动确实展开”的证据门槛：段内位置误差必须先超过该值，小误差才可信。 */
#define MANIP_GIMBAL_PERR_LARGE          8000  /* 电机5：约20mm，小于最短段(42mm=16800)的一半 */
#define MANIP_SLIDE_PERR_LARGE            300  /* 电机6：15mm，小于40mm段(800脉冲)的一半 */

/* ---------- Manipulator relative movement ---------- */

/* Pickup movement*/
#define MANIP_PICKUP_GROUND_MM           -183.0f//从最高点下降到地面取料高度
#define MANIP_PICKUP_TABLE_MM            -103.0f//从最高点下降到原料台取料高度
#define MANIP_PICKUP_TRAY_MM             -50.0f //从最高点下降到物料盘取料高度

/*
 * 暂存区识别前的垂直预下降量（正数=下降）。
 *
 * 相机装在夹爪上，暂存区识别固定在“云台转到识别角 + 滑台伸到2号环识别位”
 * 之后进行。原流程在电机5最高原点取像；现在先在识别位把电机5下降 100 mm，
 * 让相机更靠近圆环再发 place，以提高像素分辨率和识别稳定性。
 *
 *    识别前：转云台 -> 伸滑台到识别位 -> 下降 100 mm -> K230 识别
 *    识别后：先抬回最高原点 -> 回缩滑台到 0 mm -> 正常固定位姿任务
 *
 * 原料区不适用本宏，仍然保持在最高原点识别：原料区每块料都要在夹爪
 * 靠近物料的位置做多轮旋转微调，提前下降会缩小夹爪与物料的间隙。
 *
 * 只改本宏即可调整暂存区预下降量；0 表示恢复原来的最高点识别。
 */
#define STORAGE_VISION_PRE_DROP_MM       100.0f

/*
 * 粗加工区识别前的垂直预下降量（正数=下降），与暂存区使用相同的 100 mm。
 *
 * SlideJogToRoughAlignStart() 先把滑台伸到ROUGH_VISION_LOW_SLIDE_POS2_MM，再下降
 * 电机5到本高度，然后发 K230 place 做2号环底盘对位；对位和航向复核结束后
 * SlideJogHomeStart() 会先抬回最高原点，再回缩滑台，之后 1/2/3 号圆环
 * 仍按 ROUGH_* 固定位姿执行。
 *
 * 原料区不适用，仍保持最高原点识别。设为 0 可恢复粗加工区原来的最高点识别。
 */
#define ROUGH_VISION_PRE_DROP_MM         100.0f

/*
 * 低位标定高度均以丝杆最高机械原点为0 mm，向下用正的累计下降量表示。
 * 例如185 mm表示“从最高点累计下降185 mm”，绝不能理解为从100 mm观察位
 * 再下降185 mm。该绝对高度已经过正式跑图验证，粗加工区和暂存区调试都
 * 可以从最高点或100 mm观察位直接到达，底层会自动换算本次相对行程。
 */

/* 电机6前伸距离：这里只保留原料区的原有值。
 * 数值是「从机械原点量起的绝对伸出量」，不是增量。原料区视觉识别前会
 * 先伸到识别位；识别/对准完成后直接从当前伸出量下降抓取，不再退回原点。
 * 粗加工区和暂存区仍在视觉对位后退回原点，再从原点执行固定位姿动作。
 * 两个区域都在下方拥有各自的独立参数，后续实车调其中一个区域不会再连带
 * 改变另外两个区域。 */
#define MANIP_PICKUP_SLIDE_POS1_MM       130.0f
#define MANIP_PICKUP_SLIDE_POS2_MM        80.0f
#define MANIP_PICKUP_SLIDE_POS3_MM       130.0f

/*
 * 粗加工区固定圆环的独立标定位姿。
 *
 * 标定与正式运行的因果顺序不能颠倒：
 *   标定：信任“启停区2直达N2”的底盘位置，绝不做视觉底盘修正；固定底盘，
 *         先调ROUGH_VISION_LOW_SLIDE_POS2_MM直到K230中心对准2号圆环；再用
 *         VOFA 0x8B把夹爪下降到实际放置高度，分别标定1/2/3号夹爪中心的
 *         ROUGH_CAROUSEL_POSn_DEG和ROUGH_SLIDE_POSn_MM。
 *   正式：经过多轮任务到达N2后，复现已标定的2号相机位姿，用K230偏差
 *         平移底盘回到上述准确位置；之后底盘不动，1/2/3号全部按写死
 *         位姿放置和取回，1/3号不再视觉对位。
 *
 * 因此这些值必须是粗加工区专用值，不能借用原料区/暂存区参数。2号识别
 * 角固定为273.33 deg（旧270°刻度记作205 deg；K230方向映射据此定义），但相机取像滑台长度与夹爪实际
 * 放置滑台长度必须分开保存：相机光心和夹爪中心不保证机械同轴。
 */
/*
 * 注意：以下历史记录中的角度都是 270° 旧刻度下的实测值；下面的宏已按
 * 360° 新刻度换算（×4/3），脉宽与实测姿态一致。阅读老记录请自行换算。
 *
 * 2026-10-17 高位识别实车标定结果（电机5位于最高原点，启停区2直达N2、
 * 航向+90 deg，底盘固定不动）：
 *   1号圆环：云台 228 deg，齿条绝对伸出 120 mm；
 *   2号圆环：云台保持 K230 方向映射所用的 205 deg，旧高位齿条伸出78 mm；
 *   3号圆环：云台 178 deg，齿条绝对伸出 120 mm。
 *
 * 重要：正式逻辑现已改成电机5下降ROUGH_VISION_PRE_DROP_MM后识圆，透视
 * 和圆心像素可能与这组高位结果不同。旧高位复核仍可使用0x80，正式任务
 * 只使用下面单独保存的低位相机参数和夹爪落点参数。
 *
 * 2026-10-23低位实车复测结果（丝杆下降到185 mm观察夹爪中心）：
 *   K230对准2号圆环：云台205 deg，齿条95 mm；
 *   夹爪对准1号圆环：云台232 deg，齿条119 mm；
 *   夹爪对准2号圆环：云台205 deg，齿条 90 mm；
 *   夹爪对准3号圆环：云台180 deg，齿条110 mm。
 * 机械设计上相机光心与夹爪中心追求同轴，当前实测也只剩小量误差；但
 * “相机取像基准”和“夹爪实际落点”仍分别保存。这样以后继续精调其中一项
 * 时不会连带改变另一项，不能只因二者数值接近就共用同一个宏。
 *
 * 三组参数的用途必须严格区分：
 *   ROUGH_VISION_HIGH_SLIDE_POS2_MM：旧0x80高位识别，仅作历史对照；
 *   ROUGH_VISION_LOW_SLIDE_POS2_MM ：0x89和正式任务的低位K230取像基准，
 *                                    当前实车值为95 mm；
 *   ROUGH_SLIDE_POS1/2/3_MM        ：夹爪下到-185 mm后的真实放置/取回位姿；
 *                                    2号当前为90 mm，不是相机的95 mm。
 * 正式任务经过多轮行驶到达N2后，只复现LOW取像参数做底盘纠偏；收回齿条后
 * 再使用ROUGH_SLIDE_POSn_MM抓放，因此即使仍有小量安装偏差也不会互相覆盖。
 */
#define ROUGH_VISION_HIGH_SLIDE_POS2_MM    90.0f//旧的高识别伸长量
#define ROUGH_VISION_LOW_SLIDE_POS2_MM     90.0f

#define ROUGH_CAROUSEL_POS1_DEG          308.0f /* 旧 228.5 → 2193 us */
#define ROUGH_CAROUSEL_POS2_DEG          273.7f /* 旧 205.0 → 2019 us，K230 方向映射基准 */
#define ROUGH_CAROUSEL_POS3_DEG          239.5f /* 旧 181.0 → 1841 us */

#define ROUGH_SLIDE_POS1_MM              132.5f
#define ROUGH_SLIDE_POS2_MM               87.5f//10.23
#define ROUGH_SLIDE_POS3_MM              128.0f

/*
 * 仅用于 VOFA 手工标定命令 0x86 的安全角度窗口。
 * 正式任务仍只会使用上面的三个固定角；这里在两端各留约 20 deg 调整余量，
 * 既便于实车找中心，也避免误发 0/360 deg 让伸出的夹爪扫过整车。
 * 窗口随刻度换算（旧 160..250 → ×4/3）。
 */
#define ROUGH_CAROUSEL_CAL_MIN_DEG       213.33f
#define ROUGH_CAROUSEL_CAL_MAX_DEG       333.33f

/*
 * 粗加工区低位落点微调保护。
 * 丝杆已经下降到放置高度时，允许继续用8C/8D调整1/2/3号圆环。云台舵机
 * 对很小角度变化不灵敏，因此不限制单次角度差；角度仍受上面的绝对范围
 * 约束。齿条单条命令最多改变20 mm，更大的改变量应分次执行并观察干涉。
 */
#define ROUGH_LOW_CAL_MAX_SLIDE_STEP_MM   20.0f

/*
 * 暂存区固定圆环的独立位姿。
 *
 * 标定路径为“启停区2 -> 粗加工区2号环视觉校准 -> 暂存区”，先借助固定的
 * 粗加工2号环消除长距离行驶累积误差，再在暂存区确定下列三组水平位姿：
 * 2026-10-23暂时以粗加工区最新低位结果作为N4重新标定的基础值
 *（以下角度同样是 270° 旧刻度记录，宏已 ×4/3 换算）：
 *   1号：云台232 deg，齿条从机械原点绝对伸出119 mm；
 *   2号：云台205 deg，夹爪齿条绝对伸出90 mm；
 *   3号：云台180 deg，齿条从机械原点绝对伸出110 mm。
 *
 * 正式逻辑会下降STORAGE_VISION_PRE_DROP_MM后识圆。暂存区也把2号环
 * “高位相机基准、低位相机基准、夹爪实际落料位姿”拆开保存，原因同粗加工：
 * 机械设计目标是同轴，但仍给实车的小量安装误差保留独立标定能力。低位相机
 * 基准先用95 mm，2号夹爪基础值为90 mm；0x87只复核旧高位，0x8A标定正式低位。
 *
 * 第一批与第二批物料落在相同的三个圆环中心，因此两批共用这里的云台角和
 * 齿条伸长量；第二批仅通过位姿中的layer=1叠加竖直高度补偿，不能再为第二批
 * 另改水平参数。第二层高度由MANIP_LAYER_STEP_MM单独标定。
 */
#define STORAGE_VISION_HIGH_SLIDE_POS2_MM  90.0f
#define STORAGE_VISION_LOW_SLIDE_POS2_MM   90.0f

#define STORAGE_CAROUSEL_POS1_DEG        308.0f /* 旧 228.5 → 2193 us */
#define STORAGE_CAROUSEL_POS2_DEG        273.7f /* 旧 205.0 → 2019 us */
#define STORAGE_CAROUSEL_POS3_DEG        239.5f /* 旧 181.0 → 1841 us */

#define STORAGE_SLIDE_POS1_MM            132.5f
#define STORAGE_SLIDE_POS2_MM             87.5f//10.23
#define STORAGE_SLIDE_POS3_MM            128.0f//10.23

/* VOFA暂存区手工角度命令的安全窗口；正式任务只使用上面的三个固定值。
 * 窗口随刻度换算（旧 160..250 → ×4/3）。 */
#define STORAGE_CAROUSEL_CAL_MIN_DEG     213.33f
#define STORAGE_CAROUSEL_CAL_MAX_DEG     333.33f

/*
 * 暂存区在185 mm放置高度调齿条时的单次改变量上限。调参方式与粗加工区
 * 一致：云台不限制单次角度差，齿条每条命令最多改变20 mm。
 */
#define STORAGE_LOW_CAL_MAX_SLIDE_STEP_MM  20.0f

/* 原料区旧的通用识别预伸量：在取料位2(POS2)姿态下使用
 * （直接使用 CAROUSEL_PICKUP_POS2_DEG），相机装在夹爪上，
 * 所以识别前先把滑台伸到 POS2 的伸出量，让取像距离与识别姿态自洽。
 * 识别前的机械预置分两种节奏：通用/原料区是「云台先转、滑台后伸」，先把
 * 夹爪转到 POS2 并张开，再把滑台伸到这里给的值；粗加工区/暂存区则把转向
 * 与前伸并行下发，两者都完成才算就位。不论哪种，两件事都完成后才发
 * detect/place。
 * 原料区识别结束后不再退回原点：视觉闭环已经把滑台停在相机验证过的
 * 伸出量上，抓取动作直接从该位置下降。粗加工区/暂存区的底盘对位仍会
 * 在视觉完成后由 SlideJogHomeStart() 退回原点。
 * 粗加工区由SlideJogToRoughAlignStart()读取ROUGH_VISION_LOW_SLIDE_POS2_MM；
 * 暂存区由SlideJogToStorageAlignStart()读取STORAGE_VISION_LOW_SLIDE_POS2_MM；
 * 原料区使用 RAW_VISION_SLIDE_START_MM。 */
#define MANIP_PICKUP_SLIDE_ALIGN_MM      MANIP_PICKUP_SLIDE_POS2_MM

/* ---------- Zone action poses ----------
 * 每个区域的“外部目标位姿”：
 *   MANIP_POSE(夹爪旋转角度, 电机6前伸mm, 电机5下降mm, 叠放层号)
 * 只描述去程参数；回程位移由 manipulator_action 内部按 -有效值现算。
 * 第二项是从滑台机械原点量起的绝对伸出量。粗加工区使用独立的
 * ROUGH_SLIDE_POS1/2/3_MM；暂存区使用 STORAGE_SLIDE_POS1/2/3_MM。
 * 标定时只改对应区域的三项数字，不必改状态机。
 */

/* 位姿构造宏：与 manipulator_action.h 的 ManipPose_t 同构。 */
#define MANIP_POSE(angle_deg_, slide_mm_, gimbal_mm_, layer_) \
    { (float)(angle_deg_), (float)(slide_mm_), (float)(gimbal_mm_), (uint8_t)(layer_) }

/*
 * 暂存区叠放层高：第二层相对第一层少下降的距离（正数=抬高）。
 *
 * 旧值0 mm会让layer=1与第一层完全相同，第二批因此仍下降到地面；这里恢复
 * 先前工程使用的60 mm实物参数。地面放置下降量为-185 mm时，第二层实际
 * 下降量为 -185 + 60 = -125 mm。正式动作表已把第二批标成layer=1，因而
 * 只有第二批暂存放置会使用这60 mm，原料区、粗加工区和第一批均不受影响。
 *
 * 赛题只给出物料最大尺寸、没有规定固定高度；若更换现场物料形状，只调
 * 本宏即可，不要修改ZONE_STORAGE_POSn_L2_POSE或动作状态机。
 */
#define MANIP_LAYER_STEP_MM             60.0f

/* 原料区：原料台取料 -> 料盘。
 * 注意：原料区三步实际只用 POS2（夹取朝向必须与识别朝向一致，理由见
 * 本文件末尾「原料区三轮识别颜色」一节和 zone_task.c 的 s_raw_batch0）。
 * POS1/POS3 两条保留定义以保持与粗加工区/暂存区同构，当前无引用。 */
#define ZONE_RAW_POS1_POSE   MANIP_POSE(CAROUSEL_PICKUP_POS1_DEG, MANIP_PICKUP_SLIDE_POS1_MM, MANIP_PICKUP_TABLE_MM, 0U)
#define ZONE_RAW_POS2_POSE   MANIP_POSE(CAROUSEL_PICKUP_POS2_DEG, MANIP_PICKUP_SLIDE_POS2_MM, MANIP_PICKUP_TABLE_MM, 0U)
#define ZONE_RAW_POS3_POSE   MANIP_POSE(CAROUSEL_PICKUP_POS3_DEG, MANIP_PICKUP_SLIDE_POS3_MM, MANIP_PICKUP_TABLE_MM, 0U)

/* 粗加工区：料盘 <-> 三个固定圆环（放料与取回共用同一组独立位姿）。 */
#define ZONE_ROUGH_POS1_POSE   MANIP_POSE(ROUGH_CAROUSEL_POS1_DEG, ROUGH_SLIDE_POS1_MM, MANIP_PICKUP_GROUND_MM, 0U)
#define ZONE_ROUGH_POS2_POSE   MANIP_POSE(ROUGH_CAROUSEL_POS2_DEG, ROUGH_SLIDE_POS2_MM, MANIP_PICKUP_GROUND_MM, 0U)
#define ZONE_ROUGH_POS3_POSE   MANIP_POSE(ROUGH_CAROUSEL_POS3_DEG, ROUGH_SLIDE_POS3_MM, MANIP_PICKUP_GROUND_MM, 0U)

/* 暂存区：料盘 -> 三个暂存位。第一层 layer=0，第二层叠放用 _L2_ 版本（layer=1）。 */
#define ZONE_STORAGE_POS1_POSE      MANIP_POSE(STORAGE_CAROUSEL_POS1_DEG, STORAGE_SLIDE_POS1_MM, MANIP_PICKUP_GROUND_MM, 0U)
#define ZONE_STORAGE_POS2_POSE      MANIP_POSE(STORAGE_CAROUSEL_POS2_DEG, STORAGE_SLIDE_POS2_MM, MANIP_PICKUP_GROUND_MM, 0U)
#define ZONE_STORAGE_POS3_POSE      MANIP_POSE(STORAGE_CAROUSEL_POS3_DEG, STORAGE_SLIDE_POS3_MM, MANIP_PICKUP_GROUND_MM, 0U)
#define ZONE_STORAGE_POS1_L2_POSE   MANIP_POSE(STORAGE_CAROUSEL_POS1_DEG, STORAGE_SLIDE_POS1_MM, MANIP_PICKUP_GROUND_MM, 1U)
#define ZONE_STORAGE_POS2_L2_POSE   MANIP_POSE(STORAGE_CAROUSEL_POS2_DEG, STORAGE_SLIDE_POS2_MM, MANIP_PICKUP_GROUND_MM, 1U)
#define ZONE_STORAGE_POS3_L2_POSE   MANIP_POSE(STORAGE_CAROUSEL_POS3_DEG, STORAGE_SLIDE_POS3_MM, MANIP_PICKUP_GROUND_MM, 1U)

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
 * N6停车等待USART2扫码并通过task_code.c校验后才继续；区域机械位姿仍以
 * 固定2号圆环做视觉基准，1/2/3目标位由任务码动态选择。
 */
#define MISSION_ENABLE                       1U
#define MISSION_CHASSIS_ONLY                 0U
#define MISSION_QR_NODE                      6U
#define MISSION_RAW_NODE                     8U
#define MISSION_ROUGH_NODE                   2U
#define MISSION_STORAGE_NODE                 4U
#define MISSION_START_DELAY_MS            1500U
#define MISSION_TASK_HOLD_MS              2000U

/* Task-yaw alignment. World yaw is clockwise-positive; Chassis rotation is CCW-positive. */
#define MISSION_ALIGN_TOLERANCE_DEG         0.50f  /* 粗加工/暂存等任务点的最终航向容差。
                                                    * 相机距车体中心约300 mm时，0.5°对应约
                                                    * 2.6 mm横向变化。STOP后仍会稳定、重新
                                                    * 验角，未达标就继续PD，不按次数放行。 */
#define MISSION_ALIGN_SETTLE_MS              200U
#define MISSION_ALIGN_IMU_TIMEOUT_MS          300U
#define MISSION_ALIGN_STABLE_SAMPLES            3U  /* STOP后需要3个不同IMU帧都在容差内 */

/*
 * 任务点航向改为速度模式 PD：远处最高 80 RPM，接近目标降到 4 RPM，
 * STOP 后稳定并重新验角。没有“最多尝试次数”和角度超时，未对准就继续纠正。
 */
#define MISSION_ALIGN_MAX_RPM                  80U
#define MISSION_ALIGN_MIN_RPM                   4U
#define MISSION_ALIGN_ACC                      10U
#define MISSION_ALIGN_KP                     1.80f
#define MISSION_ALIGN_KD                     0.35f
#define MISSION_ALIGN_REFRESH_MS               40U

/* ---------- 原料区临时停靠位 ----------
 *
 * N8 仍是路网节点，不修改 g_nodes[]。小车在 N8 完成 -90 deg 任务航向后，
 * 车体 +X（右侧）正对原料区；机械臂完全缩回时先向 +X 靠近一次，三块料
 * 全部取完后再沿 -X 退回同样距离，随后重新做任务航向校正。
 *
 * 实车约束（2026-10-17）：
 *   N8 车体中心到场地边线约 300 mm
 *   旋转后朝圆盘方向的车体半宽约 100 mm
 *   圆盘边缘最大伸入场地 85 mm
 *   要求车体与圆盘至少留 50 mm
 * 因而理论靠近上限约 300-100-85-50 = 65 mm。尺寸目前都是手量约数，
 * 首版只走 40 mm，理论剩余间隙约 75 mm；实测确认前不要直接调到 65 mm。
 */
#define RAW_DOCK_ENABLE                         1U
#define RAW_DOCK_BODY_X_M                    0.040f
#define RAW_DOCK_SPEED_MPS                   0.080f
#define RAW_DOCK_SETTLE_MS                     200U

/*
 * 底盘停靠后，相机随车体已经向圆盘靠近 RAW_DOCK_BODY_X_M。原料识别位
 * 因而不能仍沿用 115 mm 的通用预伸，否则相机比旧标定多推进约 40 mm，
 * 容易越过可用视场。这里把车体靠近量从滑台预伸量中扣除，使第一次取像
 * 时的相机世界位置与已跑通的旧版基本一致：115 - 40 = 75 mm。
 */
#if RAW_DOCK_ENABLE
#define RAW_VISION_SLIDE_START_MM \
    (MANIP_PICKUP_SLIDE_ALIGN_MM - (RAW_DOCK_BODY_X_M * 1000.0f))
#else
#define RAW_VISION_SLIDE_START_MM       MANIP_PICKUP_SLIDE_ALIGN_MM
#endif

/*
 * 原料视觉闭环允许滑台在安全范围内调节。机械行程约 250 mm；实车已经
 * 验证 125 mm 会限制远端物料的对准，因此把软件上限放宽到 160 mm，仍给
 * 机械末端保留约 90 mm 余量。继续增大前要重新检查线束、夹爪和圆盘干涉。
 */
#define RAW_VISION_SLIDE_MIN_MM                 0.0f
/* 滑台统一软件绝对位置上限。视觉追踪、VOFA 标定和通用绝对点动都必须遵守；
 * 这是软件保护，不代替机械限位/原点开关。 */
#define MANIP_SLIDE_SOFT_MAX_MM                210.0f

/* 保留旧名字供原料区视觉位置环使用，避免同一个物理上限出现两份数值。 */
#define RAW_VISION_SLIDE_MAX_MM                MANIP_SLIDE_SOFT_MAX_MM
#define RAW_VISION_CAROUSEL_MIN_DEG   CAROUSEL_PICKUP_POS3_DEG
#define RAW_VISION_CAROUSEL_MAX_DEG   CAROUSEL_PICKUP_POS1_DEG

/* ---------- K230 视觉对位（每个作业点对一次） ----------
 * 上行偏差报文：<方向字符>,<无符号像素数>，例如 a,18（见 k230.c 的 process_line）。
 * 数值不带符号，往哪边偏完全由前面的字符表达。
 *
 * 方向字符是“夹取姿态下夹爪正向”的前/左/后/右。在固定 POS2 下换算到
 * 车体系为：
 *   w 夹爪前 -> 车体右(+X)      s 夹爪后 -> 车体左(-X)
 *   a 夹爪左 -> 车体前(+Y)      d 夹爪右 -> 车体后(-Y)
 * PLACE 模式让车朝偏差所指方向平移；DETECT 模式见下面的原料区专用策略。
 *
 * 摄像头装在夹爪上：图像朝向随夹爪旋转，所以 w/a/s/d 只有在夹爪处于
 * 约定姿态时才有确定含义。对位的第一步就是把夹爪转到
 * CAROUSEL_PICKUP_POS2_DEG 指定的起始姿态（POS2），转到位之前不发检测
 * 命令。PLACE 保持该角度，DETECT 允许在安全范围内继续转舵机跟踪物料。
 * 这一步现在由上层的视觉预置做掉（ManipulatorAction_SlideJogToXXXAlignStart；
 * 粗加工区/暂存区是转向与前伸并行，两者都完成才算就位），本模块从
 * K230Align_StartPreset 进去后不再重复发初始舵机目标；用同一个宏，两处必须一致。
 * 换一个基准姿态必须重推 offset_to_body()。
 *
 * 启动命令按区域分派：
 *   原料区          -> detect,<颜色号>   每个作业点发三轮，每轮一个颜色，
 *                                         识别到该颜色 -> 对位 -> 夹一块
 *                                         （颜色见 K230_RAW_ALIGN_B*_COLOR_*）
 *   粗加工区/第一轮暂存区 -> place,<位置号>    对准放置位
 *   第二轮暂存区         -> stack,<位置号>,<第一层颜色号>
 *                            先用物料颜色确认位置身份，再用外层圆环定位
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
                                                    * 粗加工区/第一轮暂存区走 PLACE，第二轮暂存区
                                                    * 走独立STACK身份闸门；本宏对它们均无影响。 */
#define K230_ALIGN_PLACE_ID                  2U    /* 粗加工区位置基准固定为2号圆环；只对它做一次底盘视觉对位。
                                                    * 1/3号随后完全使用 ZONE_ROUGH_POS1/3_POSE，
                                                    * 不再次启动视觉。暂存区目前也沿用位置号2。 */
//#define K230_ALIGN_MM_PER_PX                0.5f   
#define K230_ALIGN_TOLERANCE_PX              1U    /* STM32几何兜底使用的圆形半径容差（px）。
                                                    * K230的place/grab仍是首选完成信号；STM32只有
                                                    * 完整收到同一轮X/Y偏差并连续稳定后才会使用本值。 */
#define K230_ALIGN_STABLE_FRAMES             5U    /* STM32兜底至少连续收到5组完整双轴中心偏差。
                                                    * 这里数的是不同的新偏差组，不会在主循环里把
                                                    * 同一组缓存重复累计多次。 */
#define K230_ALIGN_STABLE_MIN_MS           300U    /* 从第一组合格偏差到兜底完成至少持续300 ms，
                                                    * 防止高帧率下5组数据在很短时间内瞬间放行。 */
#define K230_ALIGN_STABLE_MAX_GAP_MS       200U    /* 连续合格组之间允许的最大间隔；超过后从1重新计数，
                                                    * 防止中途长时间丢圆后沿用旧的稳定次数。 */
#define K230_ALIGN_MAX_ITER                  255U  /* 微调次数上限：255 是 uint8_t 上限，标定阶段等同于取消次数限制，
                                                    * 不再因为“调了几次”就失败；代价是失去次数兜底。 */
#define K230_ALIGN_STEP_MAX_MM             100.0f   /* 单次位移钳位，防 mm_per_px 标错时冲过头 */
#define K230_ALIGN_OFFSET_TIMEOUT_MS        1000U  /* 补发 detect/place 的周期。这里不设“等多久
                                                    * 就失败”的上限：颜色识别和偏差帧想等多久
                                                    * 就等多久，到点只补发一次命令催一催。
                                                    * 调大=催得慢，调小=催得勤。 */
#define K230_ALIGN_PAIR_WINDOW_MS            150U  /* 等同一轮另一正交轴偏差行的窗口。当前K230每轮都会
                                                    * 发送X/Y两行（0偏差也发送），因此缺一轴不能再按0
                                                    * 使用；超过窗口就丢弃孤立轴并清稳定计数，等待下一组
                                                    * 完整双轴数据，避免“只收到近中心轴就误判到位”。 */
#define K230_ALIGN_SETTLE_MS                 0U  /* 底盘/舵机自身报告停稳后，再取新偏差 */
#define K230_ALIGN_SPEED_MPS               0.12f   /* 视觉微调速度，慢于导航 */

/* 通用圆环对位结束时等待 K230 确认 stopped，丢包则持续重发。 */
#define K230_ALIGN_STOP_RETRY_MS             100U

/* ---------- 原料区二维视觉位置闭环 ----------
 *
 * raw_material_align.c 独占原料区 DETECT：底盘固定在临时停靠位，使用
 *   px_y(a/d) -> 云台角度（切向）
 *   px_x(w/s) -> 滑台伸出量（径向）
 * 两个位置环同时算下一目标，执行器到位后重新取像。这里的 PID 输出不是
 * RPM，而是每轮的“位置修正量”（deg / mm）；舵机没有角度反馈、滑台只有
 * 到位反馈，所以不能把它当连续速度环使用。
 *
 * Kp 是主要项；Kd 只抑制目标跳变，Ki 默认 0，避免旋转物料/限位时积分
 * 饱和。若实车发现某一轴越调越偏，只反转对应的 *_SIGN，不要改 K230 协议。
 */
/*
 * STM32机械夹取容差故意放宽到4 px；K230仍保持2 px并继续独立执行“精确居中
 * + 静止确认 -> grab”。K230现在连中心附近的0~2 px也会上报，因此STM32可
 * 依靠连续3帧自行提前结束，不会因K230进入静止确认而无偏差可收、空等数秒。
 */
#define RAW_VISION_TOLERANCE_PX                  4U
#define RAW_VISION_STABLE_FRAMES                 3U
#define RAW_VISION_PAIR_WINDOW_MS                50U
#define RAW_VISION_RESEND_MS                   1000U
#define RAW_VISION_SETTLE_MS                    100U

/*
 * 原料对准结束后的 K230 stop 采用“命令-确认”握手：K230 回 stopped 才允许
 * ZoneTask 进入机械夹取。确认丢失时每 100 ms 重发一次，不再因固定
 * 时间超时而退出；迟到的 stopped 仍能继续任务，也避免上一颜色任务
 * 残留并把下一次 detect 变成 error,busy。
 */
#define RAW_VISION_STOP_RETRY_MS                100U

/* 输出单位是“云台角度单位”。2026-10-25 云台刻度由 270° 改为 360°，
 * 同样的物理转角对应的数值变大 4/3，故 Kp/Kd/限幅一起 ×4/3，单像素对应的
 * 实际机械修正量与改刻度前完全一致。 */
#define RAW_VISION_ANGLE_SIGN                    1.0f
#define RAW_VISION_ANGLE_KP                      0.1333f /* 旧 0.10 */
#define RAW_VISION_ANGLE_KI                      0.00f
#define RAW_VISION_ANGLE_KD                      0.0067f /* 旧 0.005 */
#define RAW_VISION_ANGLE_I_LIMIT_PX_S          200.0f
#define RAW_VISION_ANGLE_STEP_MAX_DEG           10.667f /* 旧 8.0 */

#define RAW_VISION_SLIDE_SIGN                    1.0f
#define RAW_VISION_SLIDE_KP                      0.50f
#define RAW_VISION_SLIDE_KI                      0.00f
#define RAW_VISION_SLIDE_KD                      0.010f
#define RAW_VISION_SLIDE_I_LIMIT_PX_S          200.0f
/* 单次20 mm限幅必须保留：它防止方向符号、像素比例或识别跳变出错时，
 * 滑台一次冲到总行程上限。闭环速度主要靠“停稳后立即请求下一帧”提高，
 * 不靠取消这道机械保护。 */
#define RAW_VISION_SLIDE_STEP_MAX_MM            20.0f

/* ---------- 原料区三轮识别颜色 ----------
 * 原料区要把三块料依次抓进料盘 1/2/3，每块料之前各做一轮
 * “转夹爪到 POS2 + 张开 -> 滑台前伸到识别位 -> 发 detect,<颜色号> + 舵机跟踪”，
 * 识别到该颜色才夹。
 *
 * 识别从 POS2 + RAW_VISION_SLIDE_START_MM 起步；K230 的切向误差改变云台
 * 角度，径向误差改变滑台位置。对准完成的两个动态目标都由 zone_task 保存，
 * 识别后滑台先退回原点，正式夹取再重放同一位姿。三块料不再靠底盘逐块
 * 横移区分。
 *
 * 正式任务颜色由task_code.c解析：第一批取第一组、第二批取第三组，并校验
 * 两批包含同一组三种颜色。下面故意用1~6覆盖六色的宏，只供不经过
 * RobotApp扫码流程的独立识别测试使用，不能当作合法比赛任务码示例。
 * 颜色编号：1红 2黄 3蓝 4绿 5黑 6浅蓝。 */
#define K230_RAW_ALIGN_B1_COLOR_1            1U    /* 第一批第1块要找的颜色 */
#define K230_RAW_ALIGN_B1_COLOR_2            2U    /* 第一批第2块 */
#define K230_RAW_ALIGN_B1_COLOR_3            3U    /* 第一批第3块 */
#define K230_RAW_ALIGN_B2_COLOR_1            4U    /* 独立六色识别测试第4块：绿色 */
#define K230_RAW_ALIGN_B2_COLOR_2            5U    /* 独立六色识别测试第5块：黑色 */
#define K230_RAW_ALIGN_B2_COLOR_3            6U    /* 独立六色识别测试第6块：浅蓝 */

/*
 * 第二轮暂存区在2号位做底盘联合定位时，数字2已经被第一层物料遮住，不能再
 * 依赖数字识别。正式任务会按二维码第二组查出2号位第一层的实际颜色；K230
 * 用它只确认“哪个框是2号位”，
 * 随后仍以该位外层圆环的圆心计算偏差；物料自身有少量放偏不会成为定位基准。
 * 下面的宏同样只保留给未装载任务码的独立调试。
 */
#define K230_STORAGE_STACK_ANCHOR_COLOR      K230_RAW_ALIGN_B1_COLOR_2

/* 对位起始时夹爪的统一朝向直接使用 CAROUSEL_PICKUP_POS2_DEG。
 * 原料区、粗加工区、暂存区都用这一个姿态——识别阶段不靠转夹爪区分目标，
 * K230 只按颜色号/位置号去找。原料区随后可在安全范围内跟踪并沿用动态角度；
 * 粗加工区/暂存区夹取时才各自转 ZONE_ROUGH_/ZONE_STORAGE_POSn_POSE 里的
 * carousel_deg。三个取料位跨度约 84°，POS2 居中，识别位选它视野覆盖最均。
 * 注意：修改 CAROUSEL_PICKUP_POS2_DEG 必须重推 k230_align.c 的
 * offset_to_body()，因为 w/a/s/d 是在这个姿态下定义的。 */
/* 等夹爪转到位并张开；舵机无位置反馈，到时即继续。
 * 两处用同一个值、同一个动作：manipulator_action.c 的视觉预置（等云台转到位；
 * 粗加工区/暂存区并行预置里它还是「滑台已到、舵机还没 DONE」的兜底超时）
 * 和 k230_align.c 的 ROTATE（K230Align_Start 自己转云台时）。 */
#define K230_ALIGN_ROTATE_TIMEOUT_MS        2500U


#endif
