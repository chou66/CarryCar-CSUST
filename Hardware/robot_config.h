#ifndef __ROBOT_CONFIG_H
#define __ROBOT_CONFIG_H

/* ---------- CAN 电机 ID ---------- */
#define MOTOR_FL_ID                 1U  /* 左前轮电机 */
#define MOTOR_RL_ID                 2U  /* 左后轮电机 */
#define MOTOR_RR_ID                 3U  /* 右后轮电机 */
#define MOTOR_FR_ID                 4U  /* 右前轮电机 */
#define MOTOR_GIMBAL_ID             5U  /* 云台升降轴 */
#define MOTOR_SLIDE_ID              6U  /* 滑台前伸轴 */

/* ---------- 路网几何 ---------- */
#define ROUTE_GRID_STEP_M           0.850f  /* 相邻导航节点中心间距（m），实测值 */

/*
 * 导航直线速度（m/s）。轮径 77 mm、1:1 直连：v = rpm × 2π × 0.0385 / 60，
 * 即 1 rpm ≈ 0.00403 m/s，所以 0.70 m/s ≈ 174 rpm、1.00 m/s ≈ 248 rpm。
 */
#define ROUTE_EDGE_SPEED_MPS        0.70f

/* 拓扑边行驶与航向控制。边只走纵向运动，前后方向由 edge_turn 按“少转”原则选择。 */
#define ROUTE_TURN_TOLERANCE_DEG                 0.80f  /* 航向到位容差（deg） */
#define ROUTE_ALLOW_REVERSE                         1U  /* 1=允许倒车走边，减少无谓掉头 */
#define ROUTE_REVERSE_MIN_TURN_SAVING_DEG         5.00f  /* 倒车至少少转该角度才采用（deg） */
#define ROUTE_TURN_SETTLE_MS           100U  /* 原地转到位后的稳停时间（ms） */
#define ROUTE_TURN_MAX_ATTEMPTS        100U  /* 单次转向最多重试次数 */
#define ROUTE_TURN_SPEED_MPS           0.40f  /* 转向时线速度（m/s） */
/* 原地转向走 EMM 位置模式，本值是驱动器 acc 档位：
 *   acc=0   直接按设定转速启动（无斜坡）；
 *   acc!=0  每 (256-acc)*50 us 增/减 1 RPM，130 约 6.3 ms/RPM。
 * 位置模式不再有软件正弦剖面，所以必须给非 0 值，否则 90° 原地转起步/停车很硬。 */
#define ROUTE_TURN_ACC                  130U  /* 转向加速度档位（Emm 语义） */

/* ---------- 拐弯方式：圆滑拐角（直线 → 圆弧 → 直线） ----------
 * 置 1 时，路径上需要拐弯的中间节点不再停车原地转，而是提前 R 米开始
 * 走圆弧：本段直线末尾保持满速，按半径 ROUTE_CORNER_ARC_RADIUS_M 走一段
 * 圆弧转过拐角，再以满速接着走下一条边的直线，像汽车过弯一样连续。
 *
 * 圆弧的切线方向就是拓扑边方向，车体中心始终贴着圆角切线走；航向由底盘
 * 航向 PD 在弧线中跟随切线。ROUTE_ALLOW_REVERSE=1 时拐角最大 90°。
 *
 * 不能圆滑通过的情况仍回退到“停车 + 原地转向”：
 *   - 转角超过 ROUTE_CORNER_ARC_MAX_DEG（例如关闭倒车后的 180° 掉头）；
 *   - 当前边是倒车边，或圆弧前后的直线不足以容纳半径；
 *   - 拐角节点就是本次导航的目标节点（必须在节点停车执行任务）。
 *
 * 半径越小转得越急（角速度 = 车速 / 半径）：0.35 m、0.7 m/s 约 115°/s。 */
#define ROUTE_CORNER_ARC_ENABLE         1U
#define ROUTE_CORNER_ARC_RADIUS_M   0.40f
#define ROUTE_CORNER_ARC_MAX_DEG    95.0f
#define ROUTE_CORNER_ARC_MIN_MARGIN_M 0.10f  /* 圆弧前后直线至少要比半径多出的余量 */

/* ---------- 区段专用转弯参数：N2（粗加工）-> N4（暂存） ----------
 * 该区段固定走 N2->N1->N4，两段都是倒车（车头保持 +X 不变），到 N1 时
 * 一边倒车一边走圆弧。倒车入弯的曲率、最大转角、倒车阈值与普通前进
 * 拐角差别较大，这里单独给一组参数，改这里不会影响其他区段。
 *
 * 识别规则（运行期自动判断，无需调用方传参）：
 *   本次导航路径的起点是 MISSION_ROUGH_NODE，且目标节点是
 *   MISSION_STORAGE_NODE。封路回退到 N2->N5->N4 时同样生效。
 * 置 0 关闭覆盖，全部回退到上面的全局 ROUTE_* 参数。
 * ------------------------------------------------------------------ */
#define ROUTE_SEG_TURN_N2_N4_ENABLE                 1U
#define ROUTE_SEG_N2_N4_CORNER_ARC_ENABLE           1U  /* 1=该区段仍走圆滑拐角 */
#define ROUTE_SEG_N2_N4_ALLOW_REVERSE               1U  /* 1=允许倒车走该区段（N2->N4 必须为 1） */
#define ROUTE_SEG_N2_N4_TURN_TOLERANCE_DEG       0.80f  /* 航向到位容差（deg） */
#define ROUTE_SEG_N2_N4_REVERSE_MIN_TURN_SAVING_DEG 5.00f /* 倒车至少少转该角度才采用（deg） */
#define ROUTE_SEG_N2_N4_TURN_SPEED_MPS           0.40f  /* 原地转向线速度（m/s） */
#define ROUTE_SEG_N2_N4_CORNER_ARC_RADIUS_M      0.35f  /* 倒车圆弧半径（m） */
#define ROUTE_SEG_N2_N4_CORNER_ARC_MAX_DEG      95.0f   /* 超过该转角仍停车原地转（deg） */
#define ROUTE_SEG_N2_N4_CORNER_ARC_MIN_MARGIN_M  0.10f  /* 圆弧前后直线至少比半径多出的余量（m） */

/* 路径偏好：优先经过中央十字路口，仅作为总距离之后的次级判据。 */
#define ROUTE_CENTER_NODE            5U  /* 中央路口节点号 */
#define ROUTE_CENTER_TIE_BREAK       1U  /* 1=距离相同时优先走中央节点 */

/* ---------- 路径偏好：固定区段绕行节点 ----------
 * 这些起终点对各有两条等长路线（6-5-8 与 6-9-8、2-5-4 与 2-1-4、
 * 4-5-8 与 4-7-8、4-5-6-9 与 4-7-8-9），单纯比距离和转弯数分不出来，
 * 按现场动线强制走指定绕行节点。
 * 规划器先尝试 起点->绕行节点->终点：任何一段被障碍切断（节点或边封路）
 * 就自动回退到普通最短路，所以 HMI 封路后不会卡死在偏好的路线上。
 * 置 0 表示该区段不设偏好。节点号含义见 MISSION_*_NODE。
 */
#define ROUTE_VIA_QR_TO_RAW_NODE          9U  /* 扫码区 N6->原料区 N8：优先 N6->N9->N8 */
#define ROUTE_VIA_ROUGH_TO_STORAGE_NODE   1U  /* 粗加工 N2->暂存区 N4：优先 N2->N1->N4 */
#define ROUTE_VIA_STORAGE_TO_RAW_NODE     7U  /* 暂存区 N4->原料区 N8：优先 N4->N7->N8 */
/* 全部搬运任务结束后的返回段（暂存区 N4 -> 启停区 N3/N9）。
 * 优先复用左侧车道 N4->N7->N8->N9：与经中央路口的 4-5-6-9 等长，
 * 但在 N7 用圆弧直接过弯，不需要在中央路口 N5 停车转向。
 * 返回节点是 N3 时该路线多绕一整圈，必然超过下面的余量门限而回退最短路，
 * 因此这里不需要按返回节点分别配置。 */
#define ROUTE_VIA_STORAGE_TO_RETURN_NODE  8U  /* 返回段 N4->N3/N9：优先经 N8 */

/* 偏好绕行路线最多允许比普通最短路长多少（mm）。
 * 各条偏好路线与最短路等长，正常情况差值恒为 0；一旦封路迫使绕行，
 * 绕行代价会多出一整条边（≥850），必然超过此门限而回退到普通最短路。 */
#define ROUTE_VIA_MAX_EXTRA_MM          100U

/* ---------- 经过中央路口时改走原地直角转弯 ----------
 * 任意一段导航只要封路后绕到中央路口 N5，就说明这一段必须在十字路口
 * 换向：中央路口附近余量紧，拉圆弧容易切到场地边界，而且封路本身说明
 * 现场动线和标定时不同，所以这一段所有拐角都关闭圆滑拐角，恢复
 * “到节点停车 + 原地直角转弯”。
 * 判定方式：本次导航的规划路径里包含中央节点 ROUTE_CENTER_NODE。
 * 不含 N5 的拐角（如返回段的 N7）仍按 ROUTE_CORNER_ARC_* 走圆弧。
 *
 * 例外：N4->N8 走廊（原料往返）不受本策略影响，始终保留圆滑拐弯 ——
 * 偏好路线 N4->N7->N8 走 N7 圆弧，封路退化到 N4->N5->N8 时同样走圆弧。
 *
 * 置 0 关闭该策略，所有区段一律按 ROUTE_CORNER_ARC_* 执行。 */
#define ROUTE_CENTER_INPLACE_TURN           1U

/* ---------- 路径偏好：直线优先 ---------- */
/* 规划层把一次拐弯折算为额外路程，参与 Dijkstra 比较。值越大路线越直、越少拐弯。 */
#define ROUTE_TURN_PENALTY_MM       900U  /* 一次 90° 拐弯的折算代价（mm） */
#define ROUTE_UTURN_PENALTY_MM     1800U  /* 一次掉头的折算代价（mm） */

/* 边运动层把共线同向的连续拓扑边合并成一次直线平移；置 0 恢复逐边停车校准。 */
#define ROUTE_MERGE_STRAIGHT_EDGES     1U

#define ROUTE_COORD_EPS_M           0.001f  /* 判定两节点同排/同列的坐标容差（m） */
#define ROUTE_COLLINEAR_EPS_M2      0.001f  /* 判定两相邻边共线时的叉积容差（m²） */

/* ---------- 舵机通道映射 ----------
 * PWM1: PB4 / TIM3_CH1 = 料盘选择
 * PWM2: PE3 / TIM3_CH2 = 云台旋转
 * PWM3: PB7 / TIM3_CH4 = 预留
 * PWM4: PE4 / TIM3_CH3 = 夹爪开合
 */

/* ---------- 舵机角度/脉宽换算 ----------
 *   pulse_us = SERVO_MIN_US + angle_deg / travel_deg * (SERVO_MAX_US - SERVO_MIN_US)
 * 角度为机械角度，仅在发命令时换算成脉宽。
 */
#define SERVO_MIN_US                500U  /* 0° 对应脉宽（us） */
#define SERVO_MAX_US               2500U  /* 满行程对应脉宽（us） */
#define SERVO_CENTER_US            1500U  /* 中点脉宽（us），也是上电初始输出 */

#define GRIPPER_TRAVEL_DEG          270.0f  /* 夹爪开合舵机行程（deg） */
#define CAROUSEL_TRAVEL_DEG         360.0f  /* 云台旋转舵机行程（deg） */
#define TRAY_SELECTOR_TRAVEL_DEG    360.0f  /* 料盘选择舵机行程（deg） */

/* ---------- 夹爪开合舵机 PWM4 ---------- */
#define GRIPPER_OPEN_SMALL_DEG      147.0f  /* 小开口*/
#define GRIPPER_OPEN_LARGE_DEG      180.0f  /* 大开口*/
#define GRIPPER_CLOSE_DEG           114.0f  /* 夹紧*/
#define GRIPPER_MID_DEG             135.0f  /* 中间位*/

/* ---------- 云台旋转舵机 PWM2 ---------- */
/* 0..360° 对应 500..2500 us。CAROUSEL_PLACE_DEG 是放料角的上电默认值，
 * 运行时以 servo.c 的 g_carousel_place_deg 为准，标定后写回本宏。 */
#define CAROUSEL_PICKUP_POS1_DEG  306.67f  /* 夹取位1（2259 us） */
#define CAROUSEL_PICKUP_POS2_DEG  260.0f   /* 夹取位2，也是视觉对位基准姿态（2019 us） */
#define CAROUSEL_PICKUP_POS3_DEG  222.67f  /* 夹取位3（1793 us） */
#define CAROUSEL_PLACE_DEG        133.0f   /* 放料角：云台转向物料盘上方（1307 us） */
#define CAROUSEL_MID_DEG          180.00f  /* 回中位（1500 us） */
#define CAROUSEL_POWERON_DEG        170.0f  /* 上电初始角（独立值，不再跟随夹取位3） */

/* ---------- 料盘选择舵机 PWM1 ---------- */
/* 三个宏为料盘 1/2/3 号位的上电默认值，运行时对应 g_tray_slot_deg[0..2]。 */
#define TRAY_SELECTOR_SLOT_COUNT        3U
#define TRAY_SELECTOR_SLOT0_DEG  107.0f  /* 1号位（以2号位为基准 -120°） */
#define TRAY_SELECTOR_SLOT1_DEG  227.0f  /* 2号位，实车标定基准 */
#define TRAY_SELECTOR_SLOT2_DEG  347.0f  /* 3号位（以2号位为基准 +120°） */
#define TRAY_SELECTOR_MID_DEG        180.0f  /* 回中位（1500 us） */
#define TRAY_SELECTOR_POWERON_DEG    209.0f  /* 上电初始角，默认停中位 */

/* ---------- 电机5/6 机械标定 ---------- */
#define GIMBAL_DIR_DOWN              0U  /* 电机5 下降方向 */
#define GIMBAL_DIR_UP                1U  /* 电机5 上升方向 */
#define GIMBAL_PULSES_PER_MM         400.0f  /* 电机5 脉冲/mm */

#define SLIDE_DIR_EXTEND            1U  /* 电机6 前伸方向 */
#define SLIDE_DIR_RETRACT           0U  /* 电机6 后缩方向 */
#define SLIDE_PULSES_PER_MM         20.0f  /* 电机6 脉冲/mm */

/* 电机5（升降）与电机6（前伸）各自独立的转速与加速度，便于按轴单独标定。
 * rpm：电机输出轴转速；acc：Emm 加速度档位，0=不做加减速。 */
#define MANIP_GIMBAL_RPM            2500U  /* 电机5 升降转速 */
#define MANIP_GIMBAL_ACC               0U  /* 电机5 加速度档位 */
#define MANIP_SLIDE_RPM              120U  /* 电机6 前伸转速 */
#define MANIP_SLIDE_ACC              230U  /* 电机6 加速度档位 */

/* 机械臂放置时序参数 */
#define MANIP_PLACE_TRAY_WAIT_MS         800U  /* 放料前托盘等待时间（ms） */
/* 地面/原料区 -> 料盘：云台舵机到位（含软件平滑稳定）后再停该时间，
 * 之后才让电机5 下降，给舵机机械停稳留余量（ms）。 */
#define MANIP_PLACE_SERVO_SETTLE_MS      200U
#define MANIP_PLACE_MOTOR_TIMEOUT_MS     40000U  /* 电机5 单段运动超时（ms），已放宽以容纳全程升降 */
#define MANIP_PLACE_STATUS_PERIOD_MS     10U  /* 电机状态查询周期（ms） */
#define MANIP_MOTOR_FEEDBACK_GUARD_MS    20U  /* 段启动后该时间内的应答不采信（ms） */

/* 夹取/放置任务中，5、6 号电机超过该时间仍未回读到“到位”标志，
 * 就按到位继续下一状态，不再把整段动作拖到 MANIP_PLACE_MOTOR_TIMEOUT_MS。
 * 正常全程升降约 0.55 s（3 ms/mm），2 s 已留足余量；电机主动上报
 * 堵转保护仍优先判失败。 */
#define MANIP_MOTOR_IGNORE_TIMEOUT_MS   2000U

/* 收尾上升阶段电机6 的缩回时机（仅地面取料/地面放料生效）：
 * 电机5 升到回程的该比例后，才开始回缩滑台，避免齿条带着夹爪在低位
 * 横向扫过物料。用时间戳延时实现，电机5 仍是一条完整位移指令。
 * 调大更保守（更晚回缩），1.0 = 完全升到顶再回缩。 */
#define MANIP_SLIDE_RETRACT_HOLD_RATIO      0.9f

/* 只有下降深度达到该值（mm，取绝对值）的动作才延后回缩滑台：
 * 地面/粗加工/暂存区 183mm 延后；原料台 103mm、物料盘 50mm 保持同帧边升边收。 */
#define MANIP_SLIDE_RETRACT_HOLD_MIN_MM     150.0f

/* 视觉识别结束后（SlideJogHomeStart）的回收时序：
 *   1 = 电机5抬回最高原点与电机6回缩同一帧下发，两者并行执行，都到位才结束；
 *   0 = 回退到旧的「先抬到最高点，再回缩滑台」串行流程。
 * 并行省掉一整段上升时间；前提是识别位高度回缩时齿条不会扫到圆环/工装，
 * 首次上机请低速确认一遍。原料区本来就没有视觉预下降，两种设置等价。 */
#define MANIP_HOME_PARALLEL_ENABLE          1U

/* 电机5 匀速上升耗时换算：rpm × 3200 脉冲/圈 ÷ 60 → 脉冲/秒，再折算每毫米。
 * 3000rpm、400 脉冲/mm 时为 2.5 ms/mm。加减速造成的偏差由比例宏留余量。 */
#define MANIP_GIMBAL_MS_PER_MM \
    (60000.0f * GIMBAL_PULSES_PER_MM / ((float)MANIP_GIMBAL_RPM * 3200.0f))

/* 去程“先伸出、再下降”的分步时序（与上面的延时回缩互为镜像）：
 * 电机6 先伸出，伸到该比例行程后电机5 才开始下降，避免齿条带着夹爪在
 * 低位横向推进物料。同样用时间戳延时实现，电机5 仍是一条完整位移指令。
 * 换算按电机6 的匀速段，MANIP_SLIDE_ACC 的加减速会让实际伸出量略小于
 * 该比例（偏保守）；要更晚下降就把比例调大，1.0 ≈ 完全伸出后再下降。 */
#define MANIP_SLIDE_EXTEND_HOLD_RATIO       0.5f

/* 电机6 匀速伸出耗时换算：rpm × 3200 脉冲/圈 ÷ 60 → 脉冲/秒，再折算每毫米。
 * 120rpm、20 脉冲/mm 时为 3.125 ms/mm。 */
#define MANIP_SLIDE_MS_PER_MM \
    (60000.0f * SLIDE_PULSES_PER_MM / ((float)MANIP_SLIDE_RPM * 3200.0f))

/* 0x37 位置误差回读阈值（电机脉冲），仅供诊断/兜底使用 */
#define MANIP_GIMBAL_PERR_TOL            40  /* 电机5：约 0.1 mm */
#define MANIP_SLIDE_PERR_TOL             10  /* 电机6：0.5 mm */

/* “运动确实展开”的误差门槛，位置误差需先超过该值才认小误差 */
#define MANIP_GIMBAL_PERR_LARGE          8000  /* 电机5：约 20 mm */
#define MANIP_SLIDE_PERR_LARGE            300  /* 电机6：15 mm */

/* ---------- 机械臂相对运动量 ---------- */
/* 取料高度：从丝杆最高机械原点 0 mm 向下为负 */
#define MANIP_PICKUP_GROUND_MM           -183.0f  /* 地面取料下降量 */
#define MANIP_PICKUP_TABLE_MM            -103.0f  /* 原料台取料下降量 */
#define MANIP_PICKUP_TRAY_MM             -50.0f   /* 物料盘取料下降量 */

/* 暂存区识别前电机5 的预下降量（正数=下降），让相机更靠近圆环取像。
 * 正式任务在行驶预置阶段完成该下降；0 表示回到最高点识别。 */
#define STORAGE_VISION_PRE_DROP_MM       100.0f

/* 暂存区第二轮（叠放 STACK-2）的识别高度：与原料区一致，电机5 保持在
 * 最高机械原点识别，不做视觉预下降。0 表示不降；要恢复低位取像就填
 * 与 STORAGE_VISION_PRE_DROP_MM 相同的值。
 * 必须与 robot_app.c 的行驶预置、SlideJogToStorageStackAlignStart() 用
 * 同一个宏：两边数值不一致会被 slide_jog_start_target() 直接拒绝。 */
#define STORAGE_STACK_VISION_PRE_DROP_MM    0.0f

/* 粗加工区识别前的预下降量（正数=下降），语义同暂存区；0 表示最高点识别。 */
#define ROUGH_VISION_PRE_DROP_MM         100.0f

/* 电机6 前伸距离（从机械原点量起的绝对伸出量，非增量）——原料区 */
#define MANIP_PICKUP_SLIDE_POS1_MM       130.0f  /* 原料区1号位伸出量 */
#define MANIP_PICKUP_SLIDE_POS2_MM        90.0f  /* 原料区2号位伸出量，也是识别预伸基准 */
#define MANIP_PICKUP_SLIDE_POS3_MM       130.0f  /* 原料区3号位伸出量 */

/* ---------- 粗加工区固定圆环位姿 ---------- */
#define ROUGH_VISION_HIGH_SLIDE_POS2_MM    90.0f  /* 旧高位识别的2号环滑台伸出量，仅历史对照 */
#define ROUGH_VISION_LOW_SLIDE_POS2_MM     90.0f  /* 低位 K230 取像基准（2号环），用于底盘视觉纠偏 */

#define ROUGH_CAROUSEL_POS1_DEG          295.0f  /* 1号环云台角（2193 us） */
#define ROUGH_CAROUSEL_POS2_DEG         262.00f  /* 2号环云台角，K230 方向映射基准（2019 us） */
#define ROUGH_CAROUSEL_POS3_DEG          228.0f  /* 3号环云台角（1841 us） */

#define ROUGH_SLIDE_POS1_MM              130.0f  /* 1号环夹爪落点伸出量（绝对） */
#define ROUGH_SLIDE_POS2_MM               83.0f  /* 2号环夹爪落点伸出量（绝对） */
#define ROUGH_SLIDE_POS3_MM              117.0f  /* 3号环夹爪落点伸出量（绝对） */

/* VOFA 手工标定命令 0x86 的云台安全角度窗口，两端预留调整余量 */
#define ROUGH_CAROUSEL_CAL_MIN_DEG       213.33f  /* 窗口下限（deg） */
#define ROUGH_CAROUSEL_CAL_MAX_DEG       333.33f  /* 窗口上限（deg） */

/* 粗加工区低位调齿条时单条命令的最大改变量，超出应分次执行 */
#define ROUGH_LOW_CAL_MAX_SLIDE_STEP_MM   20.0f

/* ---------- 暂存区固定圆环位姿 ---------- */
#define STORAGE_VISION_HIGH_SLIDE_POS2_MM  90.0f  /* 旧高位识别的2号环伸出量，仅历史对照 */
#define STORAGE_VISION_LOW_SLIDE_POS2_MM   90.0f  /* 低位 K230 取像基准（2号环） */

#define STORAGE_CAROUSEL_POS1_DEG        295.0f  /* 1号位云台角（2193 us） */
#define STORAGE_CAROUSEL_POS2_DEG        262.0f  /* 2号位云台角（2019 us） */
#define STORAGE_CAROUSEL_POS3_DEG        228.0f  /* 3号位云台角（1841 us） */

#define STORAGE_SLIDE_POS1_MM            130.0f  /* 1号位夹爪落点伸出量（绝对） */
#define STORAGE_SLIDE_POS2_MM             83.0f  /* 2号位夹爪落点伸出量（绝对） */
#define STORAGE_SLIDE_POS3_MM            120.0f  /* 3号位夹爪落点伸出量（绝对） */

/* VOFA 暂存区手工角度命令的安全窗口 */
#define STORAGE_CAROUSEL_CAL_MIN_DEG     213.33f  /* 窗口下限（deg） */
#define STORAGE_CAROUSEL_CAL_MAX_DEG     333.33f  /* 窗口上限（deg） */

/* 暂存区低位调齿条时单条命令的最大改变量 */
#define STORAGE_LOW_CAL_MAX_SLIDE_STEP_MM  20.0f

/* 原料区识别前的滑台预伸量：取 POS2 姿态，使取像距离与识别姿态一致 */
#define MANIP_PICKUP_SLIDE_ALIGN_MM      MANIP_PICKUP_SLIDE_POS2_MM

/* ---------- 区域动作位姿 ----------
 * MANIP_POSE(夹爪旋转角度, 电机6前伸mm, 电机5下降mm, 叠放层号)
 * 只描述去程参数，回程由 manipulator_action 内部现算。
 */
#define MANIP_POSE(angle_deg_, slide_mm_, gimbal_mm_, layer_) \
    { (float)(angle_deg_), (float)(slide_mm_), (float)(gimbal_mm_), (uint8_t)(layer_) }

/* 暂存区叠放层高：第二层相对第一层抬高的距离（正数=抬高） */
#define MANIP_LAYER_STEP_MM             60.0f

/* 原料区：原料台取料 -> 料盘。当前只用 POS2，POS1/POS3 保留以保持三区同构。 */
#define ZONE_RAW_POS1_POSE   MANIP_POSE(CAROUSEL_PICKUP_POS1_DEG, MANIP_PICKUP_SLIDE_POS1_MM, MANIP_PICKUP_TABLE_MM, 0U)
#define ZONE_RAW_POS2_POSE   MANIP_POSE(CAROUSEL_PICKUP_POS2_DEG, MANIP_PICKUP_SLIDE_POS2_MM, MANIP_PICKUP_TABLE_MM, 0U)
#define ZONE_RAW_POS3_POSE   MANIP_POSE(CAROUSEL_PICKUP_POS3_DEG, MANIP_PICKUP_SLIDE_POS3_MM, MANIP_PICKUP_TABLE_MM, 0U)

/* 粗加工区：料盘 <-> 三个固定圆环（放料与取回共用同一组位姿） */
#define ZONE_ROUGH_POS1_POSE   MANIP_POSE(ROUGH_CAROUSEL_POS1_DEG, ROUGH_SLIDE_POS1_MM, MANIP_PICKUP_GROUND_MM, 0U)
#define ZONE_ROUGH_POS2_POSE   MANIP_POSE(ROUGH_CAROUSEL_POS2_DEG, ROUGH_SLIDE_POS2_MM, MANIP_PICKUP_GROUND_MM, 0U)
#define ZONE_ROUGH_POS3_POSE   MANIP_POSE(ROUGH_CAROUSEL_POS3_DEG, ROUGH_SLIDE_POS3_MM, MANIP_PICKUP_GROUND_MM, 0U)

/* 暂存区：料盘 -> 三个暂存位。第一层 layer=0，第二层用 _L2_ 版本（layer=1）。 */
#define ZONE_STORAGE_POS1_POSE      MANIP_POSE(STORAGE_CAROUSEL_POS1_DEG, STORAGE_SLIDE_POS1_MM, MANIP_PICKUP_GROUND_MM, 0U)
#define ZONE_STORAGE_POS2_POSE      MANIP_POSE(STORAGE_CAROUSEL_POS2_DEG, STORAGE_SLIDE_POS2_MM, MANIP_PICKUP_GROUND_MM, 0U)
#define ZONE_STORAGE_POS3_POSE      MANIP_POSE(STORAGE_CAROUSEL_POS3_DEG, STORAGE_SLIDE_POS3_MM, MANIP_PICKUP_GROUND_MM, 0U)
#define ZONE_STORAGE_POS1_L2_POSE   MANIP_POSE(STORAGE_CAROUSEL_POS1_DEG, STORAGE_SLIDE_POS1_MM, MANIP_PICKUP_GROUND_MM, 1U)
#define ZONE_STORAGE_POS2_L2_POSE   MANIP_POSE(STORAGE_CAROUSEL_POS2_DEG, STORAGE_SLIDE_POS2_MM, MANIP_PICKUP_GROUND_MM, 1U)
#define ZONE_STORAGE_POS3_L2_POSE   MANIP_POSE(STORAGE_CAROUSEL_POS3_DEG, STORAGE_SLIDE_POS3_MM, MANIP_PICKUP_GROUND_MM, 1U)

/* 电机状态位掩码 */
#define MOTOR_GIMBAL_MASK \
    ((uint8_t)(1U << (MOTOR_GIMBAL_ID - 1U)))

#define MOTOR_SLIDE_MASK \
    ((uint8_t)(1U << (MOTOR_SLIDE_ID - 1U)))


/* ---------- 完整任务与场地坐标运动 ----------
 * 任务顺序：启停区 -> 道路拐角 -> N6(扫码) -> N8(原料1) -> N2(粗加工1)
 *           -> N4(暂存1) -> N8(原料2) -> N2(粗加工2) -> N4(暂存2)
 *           -> 返回拐角 -> 返回启停区
 */
#define MISSION_ENABLE                       1U  /* 1=启用完整任务状态机 */

/*
 * 运行模式总开关：
 *   0 = 全流程模式：底盘导航 + 视觉对位 + 机械臂抓放全部执行；
 *   1 = 只跑底盘模式：正常完成整条导航路径（扫码、原料/粗加工/暂存
 *       作业点都照常到站），IMU 任务航向对准、启停区进出也照常执行，
 *       但不做机械臂抓放。到站后的处理分两种：
 *         - 粗加工区(N2)/暂存区(N4)：仍然做完整视觉校准（机械预置 ->
 *           K230 对环 -> 航向复核 -> 齿条回缩），校准结束直接进入下一段
 *           底盘运动，不执行放置/夹取；
 *         - 其它作业点（原料区等）：原地等待 MISSION_CHASSIS_HOLD_MS
 *           （默认 2 s）占位，然后继续下一段。
 *
 * 可在 Keil 的 C/C++ -> Define 里直接写 MISSION_CHASSIS_ONLY=1 覆盖，
 * 不必改本文件。
 */
#ifndef MISSION_CHASSIS_ONLY
#define MISSION_CHASSIS_ONLY                 1U  /* 0=全流程，1=只跑底盘（机械臂任务延时2s占位） */
#endif
/* 只跑底盘模式：每个作业点替代机械臂任务的占位延时。 */
#define MISSION_CHASSIS_HOLD_MS           2000U
#define MISSION_QR_NODE                      6U  /* 二维码停车节点 */
#define MISSION_RAW_NODE                     8U  /* 原料区作业节点 */
#define MISSION_ROUGH_NODE                   2U  /* 粗加工区作业节点 */
#define MISSION_STORAGE_NODE                 4U  /* 暂存区作业节点 */
/* 启停区返回节点：HMI 地图帧的 return_node 只允许这两个值（见 hmi_protocol.h），
 * 分别对应右下角 N3 和右上角 N9，两者都有 MapGraph_GetParkingPose() 的车位。 */
#define MISSION_IS_RETURN_NODE(node) \
    (((node) == 3U) || ((node) == 9U))
#define MISSION_START_DELAY_MS            1500U  /* 启动后等待时间（ms） */
#define MISSION_TASK_HOLD_MS              2000U  /* 任务码保持/显示时间（ms） */

/* 任务点航向对准参数。世界 yaw 顺时针为正，底盘旋转逆时针为正。 */
#define MISSION_ALIGN_TOLERANCE_DEG         0.50f  /* 最终航向容差（deg） */
#define MISSION_ALIGN_SETTLE_MS              100U  /* STOP 后稳定时间（ms） */
#define MISSION_ALIGN_IMU_TIMEOUT_MS          300U  /* IMU 数据超时（ms） */
#define MISSION_ALIGN_STABLE_SAMPLES            3U  /* STOP 后需连续合格的 IMU 帧数 */

/* 航向速度模式 PD：远处最高速，接近目标降速，未对准则持续纠正 */
#define MISSION_ALIGN_MAX_RPM                  160U  /* 最大转速（rpm） */
#define MISSION_ALIGN_MIN_RPM                   8U  /* 最小转速（rpm） */
#define MISSION_ALIGN_ACC                      10U  /* 加速度档位 */
#define MISSION_ALIGN_KP                     4.40f  /* 比例增益 */
#define MISSION_ALIGN_KD                     0.35f  /* 微分增益 */
#define MISSION_ALIGN_REFRESH_MS               40U  /* 控制刷新周期（ms） */

/* ---------- 粗加工区/暂存区任务航向：位置模式 ----------
 * N2（粗加工）和 N4（暂存）到站后的任务航向不再用速度环 PD 逐帧逼近，
 * 而是把当前角度差一次性换算成轮脉冲，交给 EMM 位置模式（0xFD）闭环到位。
 * 停稳后仍用同样的 IMU 验角流程：残余误差超过容差就按新角度差再下发一次。
 *
 * 位置模式下 acc 是驱动器档位（0=直接按设定转速启动），和速度环的
 * MISSION_ALIGN_ACC 语义不同，所以单独给值。
 * 其它任务点（扫码 N6、原料 N8）继续走上面的航向 PD。 */
#define MISSION_ALIGN_POS_ENABLE                1U  /* 0=所有任务点都回退到 PD */
#define MISSION_ALIGN_POS_SPEED_MPS          0.50f  /* 位置模式转速对应的轮线速度（m/s） */
#define MISSION_ALIGN_POS_ACC                 180U  /* EMM 位置模式 acc 档位 */

/* ---------- 原料区识别预伸 ----------
 * N8 不再做车体停靠平移：原先 40 mm 的车体靠近量并入滑台伸出量，
 * 相机相对圆盘的世界位置与旧标定保持一致。
 */
#define RAW_VISION_SLIDE_START_MM       MANIP_PICKUP_SLIDE_ALIGN_MM

/* 原料视觉闭环允许的滑台位置范围 */
#define RAW_VISION_SLIDE_MIN_MM                 0.0f  /* 视觉调节下限（mm） */

/* 滑台软件绝对位置上限：视觉追踪、VOFA 标定、绝对点动都必须遵守（软件保护，非机械限位） */
#define MANIP_SLIDE_SOFT_MAX_MM                210.0f

#define RAW_VISION_SLIDE_MAX_MM                MANIP_SLIDE_SOFT_MAX_MM  /* 原料区视觉上限，与统一上限同值 */
#define RAW_VISION_CAROUSEL_MIN_DEG   CAROUSEL_PICKUP_POS3_DEG  /* 原料视觉云台角下限 */
#define RAW_VISION_CAROUSEL_MAX_DEG   CAROUSEL_PICKUP_POS1_DEG  /* 原料视觉云台角上限 */

/* ---------- K230 视觉对位（每个作业点对一次）----------
 * 上行偏差报文：<方向字符>,<像素数>，方向字符为夹取姿态下夹爪的前/左/后/右，
 * 对应车体系为：w=+X s=-X a=+Y d=-Y。摄像头随夹爪旋转，故必须先转到
 * CAROUSEL_PICKUP_POS2_DEG 才开始发检测命令。
 *   原料区            -> detect,<颜色号>，每轮一个颜色
 *   粗加工区/第一轮暂存 -> place,<位置号>
 *   第二轮暂存         -> stack,<位置号>,<第一层颜色号>
 */
#define K230_ALIGN_ENABLE                    1U    /* 1=启用视觉对位，0=退回纯 IMU 对位 */
#define K230_ALIGN_REQUIRE_COLOR             1U    /* 1=原料区必须先收到目标颜色报文才开始纠偏 */
#define K230_ALIGN_PLACE_ID                  2U    /* 粗加工区/暂存区的视觉基准位置号（2号圆环） */
//#define K230_ALIGN_MM_PER_PX                0.5f
#define K230_ALIGN_TOLERANCE_PX              1U    /* STM32 几何兜底的像素容差 */
#define K230_ALIGN_STABLE_FRAMES             5U    /* 兜底完成所需的连续合格双轴偏差组数 */
#define K230_ALIGN_STABLE_MIN_MS           300U    /* 兜底计数的持续时长下限（ms） */
#define K230_ALIGN_STABLE_MAX_GAP_MS       200U    /* 相邻合格组最大间隔（ms），超时重新计数 */
#define K230_ALIGN_MAX_ITER                  255U  /* 视觉微调次数上限（255 等同取消次数兜底） */
#define K230_ALIGN_STEP_MAX_MM             100.0f  /* 单次位移钳位（mm），防比例参数标错冲过头 */
#define K230_ALIGN_OFFSET_TIMEOUT_MS        1000U  /* 补发 detect/place 的周期（ms），不设失败超时 */
#define K230_ALIGN_PAIR_WINDOW_MS            150U  /* 等待同一轮另一正交轴偏差的窗口（ms） */
#define K230_ALIGN_SETTLE_MS                 0U    /* 底盘/舵机停稳后到取新偏差的等待（ms） */
#define K230_ALIGN_SPEED_MPS               0.12f  /* 视觉微调平移速度（m/s） */

/* 对位结束时等待 K230 回 stopped，丢包则重发 */
#define K230_ALIGN_STOP_RETRY_MS             100U

/* ---------- 原料区二维视觉位置闭环 ----------
 * 底盘固定在停靠位：px_y(a/d) -> 云台角度（切向），px_x(w/s) -> 滑台伸出量（径向）。
 * 每轮输出的是位置修正量（deg/mm），不是速度；Kp 为主项，Kd 抑制跳变，Ki 默认 0。
 * 若某轴越调越偏，只反转对应 *_SIGN。
 */
#define RAW_VISION_TOLERANCE_PX                  4U  /* STM32 夹取判定像素容差 */
#define RAW_VISION_STABLE_FRAMES                 3U  /* 连续合格帧数即提前结束 */
#define RAW_VISION_PAIR_WINDOW_MS                50U  /* 双轴偏差配对窗口（ms） */
#define RAW_VISION_RESEND_MS                   1000U  /* 重发 detect 的周期（ms） */
#define RAW_VISION_SETTLE_MS                    100U  /* 执行器到位后的稳定时间（ms） */

/* 对准结束后与 K230 的 stop 命令-确认握手重发周期（ms） */
#define RAW_VISION_STOP_RETRY_MS                100U

/* 云台角度环 PID，输出单位为云台角度（deg） */
#define RAW_VISION_ANGLE_SIGN                    1.0f  /* 云台修正方向，1 或 -1 */
#define RAW_VISION_ANGLE_KP                      0.1333f  /* 比例增益 */
#define RAW_VISION_ANGLE_KI                      0.00f  /* 积分增益 */
#define RAW_VISION_ANGLE_KD                      0.0067f  /* 微分增益 */
#define RAW_VISION_ANGLE_I_LIMIT_PX_S          200.0f  /* 积分限幅（px·s） */
#define RAW_VISION_ANGLE_STEP_MAX_DEG           10.667f  /* 单次云台修正量上限（deg） */

/* 滑台径向环 PID，输出单位为伸出量（mm） */
#define RAW_VISION_SLIDE_SIGN                    1.0f  /* 滑台修正方向，1 或 -1 */
#define RAW_VISION_SLIDE_KP                      0.50f  /* 比例增益 */
#define RAW_VISION_SLIDE_KI                      0.00f  /* 积分增益 */
#define RAW_VISION_SLIDE_KD                      0.010f  /* 微分增益 */
#define RAW_VISION_SLIDE_I_LIMIT_PX_S          200.0f  /* 积分限幅（px·s） */
#define RAW_VISION_SLIDE_STEP_MAX_MM            20.0f  /* 单次滑台修正量上限（mm），必须保留 */

/* ---------- 原料区三轮识别颜色 ----------
 * 每块料先“转夹爪到 POS2 + 张开 -> 滑台伸到识别位 -> detect,<颜色号> + 舵机跟踪”，
 * 识别到该颜色才夹取。正式任务的颜色由 task_code.c 解析，下面只供独立识别测试用。
 * 颜色编号：1红 2黄 3蓝 4绿 5黑 6浅蓝。
 */
#define K230_RAW_ALIGN_B1_COLOR_1            1U    /* 第一批第1块要找的颜色 */
#define K230_RAW_ALIGN_B1_COLOR_2            2U    /* 第一批第2块 */
#define K230_RAW_ALIGN_B1_COLOR_3            3U    /* 第一批第3块 */
#define K230_RAW_ALIGN_B2_COLOR_1            4U    /* 测试用第4块：绿色 */
#define K230_RAW_ALIGN_B2_COLOR_2            5U    /* 测试用第5块：黑色 */
#define K230_RAW_ALIGN_B2_COLOR_3            6U    /* 测试用第6块：浅蓝 */

/* 第二轮暂存区靠第一层物料颜色确认2号位身份，定位仍用外层圆环；仅限独立调试 */
#define K230_STORAGE_STACK_ANCHOR_COLOR      K230_RAW_ALIGN_B1_COLOR_2

/* 视觉对位起始时夹爪统一朝向取 POS2（三区共用），定位靠颜色号/位置号而非转向。
 * 修改 CAROUSEL_PICKUP_POS2_DEG 必须同步重推 k230_align.c 的 offset_to_body()。 */
#define K230_ALIGN_ROTATE_TIMEOUT_MS        2500U  /* 等夹爪转到位并张开的超时（ms） */


#endif
