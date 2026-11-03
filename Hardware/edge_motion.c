#include "edge_motion.h"
#include "app_uart.h"
#include "chassis_motion.h"
#include "edge_turn.h"
#include "hwt101_uart.h"
#include "navigation.h"
#include "robot_config.h"
#include <math.h>

/*
 * 平移加速度、死区和稳停时间继承 VOFA 导航实车调试值；
 * 航向纠偏已升级为独立 PD 控制层，不再使用固定 20 RPM。
 *
 * 直线平动和圆弧走速度模式 + 软件正弦加减速（见 chassis.c 的 profile_*），
 * 位移按实测脉冲换算成“加速-匀速-减速”速度曲线，因此 EDGE_MOTION_MOVE_ACC
 * 只对直接速度命令（航向 PD 等）有效。
 *
 * 原地旋转（EDGE_MOTION_TURNING_TO_EDGE）走 EMM 位置模式，没有软件剖面，
 * ROUTE_TURN_ACC 作为驱动器 acc 档位重新生效。
 */
#define EDGE_MOTION_MOVE_ACC                   130U
#define EDGE_MOTION_YAW_DEADBAND_DEG          0.50f
#define EDGE_MOTION_YAW_CORRECT_MAX_RPM         80U
#define EDGE_MOTION_YAW_CORRECT_MIN_RPM          2U
#define EDGE_MOTION_YAW_CORRECT_ACC             10U
#define EDGE_MOTION_YAW_SETTLE_MS              150U
#define EDGE_MOTION_IMU_TIMEOUT_MS             300U

/*
 * 世界角误差 -> 角速度的 PD 参数。
 * kp 单位约为 (deg/s)/deg；kd 用世界角速度抑制接近目标时的惯性超调。
 *
 * 增益上限由“死区 / 刷新周期”决定，不能只按收敛速度往上调：
 *   80 rpm 满速 ≈ 97 deg/s，40 ms 刷新 → 每帧约 3.9 deg；
 *   死区只有 0.5 deg，所以近目标处必须靠 kp 把输出压下来，
 *   否则输出直接饱和成 bang-bang，越过死区→反向→再越过，形成左右震荡。
 * kp = 1.8 时 0.61 deg 误差才到满速、1 deg 误差约 1.8 deg/s（被 8 rpm
 * 最小输出兜到约 9.7 deg/s = 0.39 deg/帧 < 死区），闭环才稳定收敛。
 * 2026-10-31：曾误把本值写成 160（≈ 90 倍），导致回启停区边末纠偏持续震荡。
 */
#define EDGE_MOTION_YAW_KP                     1.80f
#define EDGE_MOTION_YAW_KD                     0.35f

/* 大角度允许更长纠偏时间，但仍保留失控/机械卡死保护。 */
#define EDGE_MOTION_YAW_TIMEOUT_BASE_MS       1500U
#define EDGE_MOTION_YAW_TIMEOUT_PER_DEG_MS      35U
#define EDGE_MOTION_YAW_TIMEOUT_MAX_MS        9000U

/*
 * Chassis 速度模式带 150 ms 软件看门狗。
 * 航向修正期间每 40 ms 刷新一次速度命令并重新计算 PD 输出：
 *   - 正常运行时不会触发看门狗；
 *   - 如果 EdgeMotion_Update() 因程序异常停止执行，底盘仍会在约 150 ms 后自行停车。
 */
#define EDGE_MOTION_YAW_REFRESH_MS               40U

#define EDGE_MOTION_TWO_PI_F 6.28318530717958647692f
#define EDGE_MOTION_RAD_TO_DEG_F 57.29577951308232f

static float edge_absf(float x)
{
    return (x >= 0.0f) ? x : -x;
}

static int8_t edge_signf(float x)
{
    if (x > 0.0f) return 1;
    if (x < 0.0f) return -1;
    return 0;
}

/*========================================================================
 * 区段专用转弯参数
 *
 * 默认全部取全局 ROUTE_*；只有 N2（粗加工）-> N4（暂存）这一段用
 * ROUTE_SEG_N2_N4_* 覆盖，使倒车入弯可以单独调半径/转角/倒车阈值，
 * 不影响其他区段。
 *
 * 识别不看调用点，只看本次导航：路径起点是 MISSION_ROUGH_NODE、目标
 * 节点是 MISSION_STORAGE_NODE。整段都成立，因此 N2->N1 和 N1->N4
 * 两段直线拿到的是同一组参数，封路绕行 N2->N5->N4 也一样。
 *
 * 返回段（N4 -> 启停区 N3/N9）另有 ROUTE_RETURN_CENTER_INPLACE_TURN：
 * 规划结果经过中央路口 N5 时关闭圆滑拐角，改走原地直角转弯。
 *========================================================================*/
typedef struct
{
    uint8_t arc_enable;
    uint8_t allow_reverse;
    float turn_tolerance_deg;
    float reverse_min_turn_saving_deg;
    float turn_speed_mps;
    float arc_radius_m;
    float arc_max_deg;
    float arc_min_margin_m;
} EdgeTurnTuning_t;

/* 本次导航路径里是否经过某个拓扑节点（返回段用来判定是否经中央路口）。 */
#if ROUTE_RETURN_CENTER_INPLACE_TURN
static uint8_t edge_path_contains_node(const PathResult_t *path, uint8_t node)
{
    uint8_t i;

    if (!path || !path->valid || node == 0U)
        return 0U;

    for (i = 0U; (i < path->length) && (i < PATH_MAX_LEN); i++)
    {
        if (path->node[i] == node)
            return 1U;
    }

    return 0U;
}
#endif /* ROUTE_RETURN_CENTER_INPLACE_TURN */

static void edge_turn_tuning(const EdgeMotion_t *em, EdgeTurnTuning_t *tuning)
{
    if (!tuning)
        return;

    tuning->arc_enable = (ROUTE_CORNER_ARC_ENABLE != 0U) ? 1U : 0U;
    tuning->allow_reverse = (ROUTE_ALLOW_REVERSE != 0U) ? 1U : 0U;
    tuning->turn_tolerance_deg = ROUTE_TURN_TOLERANCE_DEG;
    tuning->reverse_min_turn_saving_deg = ROUTE_REVERSE_MIN_TURN_SAVING_DEG;
    tuning->turn_speed_mps = ROUTE_TURN_SPEED_MPS;
    tuning->arc_radius_m = ROUTE_CORNER_ARC_RADIUS_M;
    tuning->arc_max_deg = ROUTE_CORNER_ARC_MAX_DEG;
    tuning->arc_min_margin_m = ROUTE_CORNER_ARC_MIN_MARGIN_M;

#if ROUTE_SEG_TURN_N2_N4_ENABLE
    if (em && em->hmi_nav)
    {
        const PathResult_t *path;
        uint8_t target;

        path = Navigation_GetPath(&em->hmi_nav->nav);
        target = Navigation_GetTargetNode(&em->hmi_nav->nav);

        if (path && path->valid && path->length > 0U &&
            path->node[0] == (uint8_t)MISSION_ROUGH_NODE &&
            target == (uint8_t)MISSION_STORAGE_NODE)
        {
            tuning->arc_enable =
                (ROUTE_SEG_N2_N4_CORNER_ARC_ENABLE != 0U) ? 1U : 0U;
            tuning->allow_reverse =
                (ROUTE_SEG_N2_N4_ALLOW_REVERSE != 0U) ? 1U : 0U;
            tuning->turn_tolerance_deg =
                ROUTE_SEG_N2_N4_TURN_TOLERANCE_DEG;
            tuning->reverse_min_turn_saving_deg =
                ROUTE_SEG_N2_N4_REVERSE_MIN_TURN_SAVING_DEG;
            tuning->turn_speed_mps =
                ROUTE_SEG_N2_N4_TURN_SPEED_MPS;
            tuning->arc_radius_m =
                ROUTE_SEG_N2_N4_CORNER_ARC_RADIUS_M;
            tuning->arc_max_deg =
                ROUTE_SEG_N2_N4_CORNER_ARC_MAX_DEG;
            tuning->arc_min_margin_m =
                ROUTE_SEG_N2_N4_CORNER_ARC_MIN_MARGIN_M;
        }
    }
#elif !(ROUTE_RETURN_CENTER_INPLACE_TURN)
    (void)em;
#endif

#if ROUTE_RETURN_CENTER_INPLACE_TURN
    /*
     * 返回段（暂存区 N4 -> 启停区 N3/N9）：
     * 偏好路线 N4->N7->N8->N9 被封路切断时，规划器回退到经中央路口 N5 的
     * 最短路（N4->N5->N6->N9 / N4->N5->N6->N3）。中央路口余量紧，且封路
     * 说明现场动线与标定不同，所以本段关掉圆滑拐角，恢复“到节点停车 +
     * 原地直角转弯”。偏好路线本身不含 N5，仍按圆弧通过。
     */
    if (em && em->hmi_nav)
    {
        const PathResult_t *path;
        uint8_t target;

        path = Navigation_GetPath(&em->hmi_nav->nav);
        target = Navigation_GetTargetNode(&em->hmi_nav->nav);

        if (path && path->valid && path->length > 0U &&
            path->node[0] == (uint8_t)MISSION_STORAGE_NODE &&
            MISSION_IS_RETURN_NODE(target) &&
            edge_path_contains_node(path, (uint8_t)ROUTE_CENTER_NODE))
        {
            tuning->arc_enable = 0U;
        }
    }
#endif
}

static void edge_save_acceleration(EdgeMotion_t *em)
{
    if (!em || !em->chassis || em->acceleration_saved)
        return;

    em->saved_acceleration = em->chassis->acceleration;
    em->acceleration_saved = 1U;
}

static void edge_restore_acceleration(EdgeMotion_t *em)
{
    if (!em || !em->chassis || !em->acceleration_saved)
        return;

    em->chassis->acceleration = em->saved_acceleration;
    em->acceleration_saved = 0U;
}

/*
 * 把 VOFA 已验证的“20 RPM 原地纠偏”换算成 Chassis 速度模式的 wz(rad/s)。
 *
 * mecanum_inverse() 纯旋转时每个轮子的转速幅值为：
 *   rpm = (half_length + half_width) * |wz|
 *         * 60 * gear_ratio / (2*pi*wheel_radius)
 *
 * 因此这里反算 wz，保证正式 EdgeMotion 仍走：
 *   chassis_set_velocity -> mecanum -> emm_vel -> CAN
 * 而不是在运动层重新直接拼 EMM 指令。
 */
static float edge_yaw_wz_for_rpm(const Chassis *chassis, uint16_t rpm)
{
    float k;
    float factor;

    if (!chassis ||
        chassis->config.wheel_radius_m <= 0.0f ||
        chassis->config.gear_ratio <= 0.0f)
        return 0.0f;

    k = chassis->config.half_length_m + chassis->config.half_width_m;
    if (k <= 0.0f)
        return 0.0f;

    factor = 60.0f * chassis->config.gear_ratio /
             (EDGE_MOTION_TWO_PI_F * chassis->config.wheel_radius_m);

    if (factor <= 0.0f)
        return 0.0f;

    return (float)rpm / (k * factor);
}

static void edge_clear_runtime(EdgeMotion_t *em)
{
    em->from_node = 0U;
    em->to_node = 0U;
    em->run_steps = 1U;
    em->settle_until_ms = 0U;
    em->yaw_correct_start_ms = 0U;
    em->yaw_correct_deadline_ms = 0U;
    em->next_yaw_refresh_ms = 0U;
    em->yaw_start_sign = 0;
    em->yaw_velocity_active = 0U;
    em->yaw_command_wz_radps = 0.0f;
    em->turn_attempt = 0U;
    em->corner_valid = 0U;
    em->corner_next_to_node = 0U;
    em->corner_next_steps = 0U;
    em->corner_reverse = 0U;
    em->corner_turn_deg = 0.0f;
    em->corner_next_world_dx_m = 0.0f;
    em->corner_next_world_dy_m = 0.0f;
    em->corner_next_length_m = 0.0f;
    em->corner_next_yaw_deg = 0.0f;
    em->corner_arc_radius_m = 0.0f;
    em->arc_entry_radius_m = 0.0f;
}

/*
 * 结束航向速度修正。
 * chassis_motion_stop() 通过正式 Chassis 状态机下发四轮 STOP；
 * 不直接调用 VOFA 专用 Emm_V5_* API。
 */
static void edge_stop_yaw_velocity(EdgeMotion_t *em)
{
    if (!em || !em->chassis || !em->yaw_velocity_active)
        return;

    chassis_motion_stop(em->chassis);
    em->yaw_velocity_active = 0U;
    em->yaw_command_wz_radps = 0.0f;
}

/*
 * 当前边执行失败后的统一收口。
 *
 * Navigation 的 current_node 只允许由 EdgeReached() 推进，所以失败时不伪造位置；
 * 这里仅停止真实底盘并锁定 EDGE_MOTION_ERROR，等待上层人工/任务层恢复。
 */
static void edge_fail(EdgeMotion_t *em)
{
    if (!em || !em->chassis)
        return;

    edge_stop_yaw_velocity(em);
    chassis_motion_stop(em->chassis);
    edge_restore_acceleration(em);
    em->state = EDGE_MOTION_ERROR;
}

/*========================================================================
 * 直线段合并：把共线同向的连续拓扑边走成一次平移
 *
 * 规划层给出的是逐节点的拓扑路径，但相邻两段方向完全一样时（同一条车道
 * 上的 1->2->3），中间那个节点既没有任务要执行，也没有理由停车。原来每段
 * 边都要停车 + 边末航向校准，白白多花两轮起停和两次稳停的时间。
 *
 * 这里在领取一条边时向前预读路径，把共线同向的连续边合并成一次平移：
 *   - 中间节点不提交、不停车、不再做航向校准；
 *   - 只在需要拐弯的节点，或本次导航的目标节点停车；
 *   - 任务动作仍然只在 mission 当前 stage 的目标节点执行。
 *========================================================================*/

/* 在本次规划的路径中查找节点下标；找不到返回 0xFF。 */
static uint8_t edge_path_index(const PathResult_t *path, uint8_t node)
{
    uint8_t i;

    if (!path || !path->valid)
        return 0xFFU;

    for (i = 0; i < path->length; i++)
    {
        if (path->node[i] == node)
            return i;
    }

    return 0xFFU;
}

#if ROUTE_MERGE_STRAIGHT_EDGES
/* a->b 与 b->c 是否严格共线同向（叉积为 0 且点积为正）。 */
static uint8_t edge_collinear(uint8_t a, uint8_t b, uint8_t c)
{
    float dx1 = g_nodes[b].x - g_nodes[a].x;
    float dy1 = g_nodes[b].y - g_nodes[a].y;
    float dx2 = g_nodes[c].x - g_nodes[b].x;
    float dy2 = g_nodes[c].y - g_nodes[b].y;
    float cross = dx1 * dy2 - dy1 * dx2;
    float dot   = dx1 * dx2 + dy1 * dy2;

    if (dot <= 0.0f)
        return 0U;  /* 反向或坐标退化 */

    if (cross > ROUTE_COLLINEAR_EPS_M2 || cross < -ROUTE_COLLINEAR_EPS_M2)
        return 0U;  /* 不在同一条直线上 */

    return 1U;
}
#endif

/*
 * 从 from_node 出发，计算这次直线平移实际能走到哪个节点。
 *
 * @param to_node 输出合并后的终点；无法合并时保持 Navigation 给出的下一节点。
 * @return 这次平移覆盖的拓扑边条数，至少为 1。
 */
static uint8_t edge_merge_straight_run(const EdgeMotion_t *em,
                                       uint8_t from_node,
                                       uint8_t *to_node)
{
#if ROUTE_MERGE_STRAIGHT_EDGES
    const PathResult_t *path;
    uint8_t target;
    uint8_t index;
    uint8_t end;

    if (!em || !em->hmi_nav || !to_node)
        return 1U;

    path = Navigation_GetPath(&em->hmi_nav->nav);
    target = Navigation_GetTargetNode(&em->hmi_nav->nav);
    index = edge_path_index(path, from_node);

    if (index == 0xFFU ||
        ((uint16_t)index + 1U) >= (uint16_t)path->length)
        return 1U;

    end = (uint8_t)(index + 1U);

    while (((uint16_t)end + 1U) < (uint16_t)path->length)
    {
        uint8_t mid = path->node[end];

        /* 本次导航的目标节点必须停车，绝不跨过去。 */
        if (mid == target)
            break;

        if (!edge_collinear(path->node[end - 1U], mid,
                            path->node[end + 1U]))
            break;

        end++;
    }

    *to_node = path->node[end];
    return (uint8_t)(end - index);
#else
    (void)em;
    (void)from_node;
    (void)to_node;
    return 1U;
#endif
}

/*
 * 合并段跑完、真正停在 run_to_node 之后，紧接着要执行的那条边。
 * 供“边末是否还要转向”的预测使用，保证预测对象和下一次实际领取的边一致。
 */
static uint8_t edge_next_run_edge(const EdgeMotion_t *em,
                                  uint8_t run_to_node,
                                  uint8_t *from_node,
                                  uint8_t *to_node)
{
    const PathResult_t *path;
    uint8_t index;

    if (!em || !em->hmi_nav || !from_node || !to_node)
        return 0U;

    if (Navigation_GetState(&em->hmi_nav->nav) != NAV_STATE_RUNNING)
        return 0U;

    path = Navigation_GetPath(&em->hmi_nav->nav);
    index = edge_path_index(path, run_to_node);

    if (index == 0xFFU ||
        ((uint16_t)index + 1U) >= (uint16_t)path->length)
        return 0U;

    *from_node = run_to_node;
    *to_node   = path->node[index + 1U];
    return 1U;
}

/*
 * 一条真实物理边只有在：
 *   位置运动完成 + HWT101 航向确认完成
 * 之后才能提交给 Navigation。
 *
 * 若这次平移合并了多条共线同向的拓扑边，就按 run_steps 连续推进节点下标；
 * 中间节点既不停车，也不做边末航向校准。
 */
static uint8_t edge_commit(EdgeMotion_t *em)
{
    uint8_t steps;
    uint8_t i;

    if (!em || !em->hmi_nav)
        return 0U;

    steps = (em->run_steps > 0U) ? em->run_steps : 1U;

    for (i = 0; i < steps; i++)
    {
        if (!HMI_Navigation_EdgeReached(em->hmi_nav))
        {
            edge_fail(em);
            return 0U;
        }
    }

    HMI_Navigation_UpdateDebugDisplay(em->hmi_nav);
    edge_restore_acceleration(em);
    edge_clear_runtime(em);
    em->state = EDGE_MOTION_IDLE;
    return 1U;
}

/*
 * 判断“当前边提交节点后，是否紧接着一定会做一次能覆盖当前航向的转向”。
 *
 * 两种来源：
 *   1) 当前边终点就是本次导航最终任务点，且调用方声明到达后做任务航向
 *      PD —— 此时不需要先纠到当前边航向，任务 PD 会直接转到 task_yaw；
 *   2) 当前边终点是中间节点，但由 edge_turn 预测下一条边需要原地转向
 *      —— 此时直接提交，让下一段一次转到下一条边的目标角度。
 *
 * 只有预测到“确实要转”才返回 1；若下一条边是直行通过、当前误差又落在
 * 转向容差内，仍返回 0，让原有边末纠偏把航向收进死区，避免小误差沿路径
 * 一直累积。
 */
static uint8_t edge_following_turn_covers_heading(EdgeMotion_t *em,
                                                  float current_world_yaw_deg)
{
    uint8_t from_node;
    uint8_t to_node;
    uint8_t target_node;
    EdgeTurnOptions_t options;
    EdgeTurnPlan_t plan;
    EdgeTurnTuning_t tuning;

    if (!em || !em->hmi_nav)
        return 0U;

    target_node = Navigation_GetTargetNode(&em->hmi_nav->nav);

    /* 最终任务点：上层随后会直接做任务航向 PD。 */
    if (em->arrival_align_valid && em->to_node == target_node)
        return 1U;

    /*
     * 中间节点：用与 edge_apply_turn_plan() 完全相同的决策参数预测下一段。
     * 合并段的终点可能已经在路径里前进了好几格，因此这里按合并终点取边，
     * 而不是按 executor 的当前位置。
     */
    if (!edge_next_run_edge(em, em->to_node, &from_node, &to_node))
        return 0U;

    edge_turn_tuning(em, &tuning);

    options.allow_reverse = tuning.allow_reverse;
    options.reverse_min_turn_saving_deg =
        tuning.reverse_min_turn_saving_deg;
    options.arrival_yaw_valid = 0U;
    options.arrival_world_yaw_deg = 0.0f;
    options.allow_merged_edge = 0U; /* 预测对象是紧邻的下一段拓扑边。 */

    if (to_node == target_node &&
        target_node >= 1U && target_node <= NODE_NUM &&
        g_nodes[target_node].task_yaw_valid)
    {
        options.arrival_yaw_valid = 1U;
        options.arrival_world_yaw_deg = g_nodes[target_node].task_yaw_deg;
    }

    if (!EdgeTurn_Plan(from_node,
                       to_node,
                       current_world_yaw_deg,
                       tuning.turn_tolerance_deg,
                       &options,
                       &plan))
        return 0U;

    return plan.turn_required ? 1U : 0U;
}

/*========================================================================
 * 圆滑拐角：直线 → 圆弧 → 直线
 *
 * 领取一段直线时向前预读：如果这段直线终点处要拐弯，就不在节点停车原地
 * 转，而是让直线末段保持满速，提前 R 米开始走一段半径 R 的圆弧转过拐角，
 * 再以满速接着走下一条边的直线（ROUTE_CORNER_ARC_ENABLE）。
 *
 * 圆弧的切线始终等于拓扑边方向，因此车体中心贴着“圆角”走，既不切角也
 * 不偏出边线；车头由底盘航向 PD 跟随切线。
 *
 * 倒车边同样可以圆滑通过：车头朝向 = 行进切线 + 180°，圆弧按倒车走，
 * 弧后直线继续倒车；切线方向连续、车头角速度与切线角速度一致，因此
 * N2->N1->N4 这类“先倒车再拐弯”的路径也不用停车。
 *
 * 不能圆滑通过时返回 0，调用方保持原来的“到节点停车 + 原地转向”：
 *   - 转角超过该区段配置的圆弧最大转角；
 *   - 圆弧前后的直线长度不足以容纳半径；
 *   - 拐角节点就是本次导航的目标节点（必须在节点精确停车）。
 *
 * 半径/最大转角/前后余量由 edge_turn_tuning() 按区段给出：普通区段取
 * ROUTE_CORNER_ARC_*，N2->N4 区段取 ROUTE_SEG_N2_N4_CORNER_ARC_*。
 *========================================================================*/

/*
 * 预读本段直线终点处的拐角。
 *
 * @param corner_node   本段直线的终点（拐角节点）
 * @param run_dir_deg   本段直线的车头世界航向（前进=切线，倒车=切线+180°）
 * @param run_length_m  本段直线的长度
 * @param run_backward  本段是否倒车
 * @return 1=可以用圆弧通过，结果写入 em->corner_*。
 */
static uint8_t edge_plan_corner(EdgeMotion_t *em,
                                uint8_t corner_node,
                                float run_dir_deg,
                                float run_length_m,
                                uint8_t run_backward)
{
    uint8_t end_node = corner_node;
    uint8_t steps;
    uint8_t target;
    float dx;
    float dy;
    float length;
    float dir_deg;
    float travel_dir_deg;
    float turn_deg;
    float min_run_m;
    EdgeTurnTuning_t tuning;

    if (!em || !em->hmi_nav)
        return 0U;

    em->corner_valid = 0U;
    em->corner_reverse = 0U;
    em->corner_arc_radius_m = 0.0f;

    /* N2->N4 区段的转弯参数与其他区段分开，见 edge_turn_tuning()。 */
    edge_turn_tuning(em, &tuning);
    if (!tuning.arc_enable)
        return 0U;

    target = Navigation_GetTargetNode(&em->hmi_nav->nav);
    if (corner_node == target)
        return 0U;

    /* 圆弧之后实际要走的直线（同样把共线边合并掉）。 */
    steps = edge_merge_straight_run(em, corner_node, &end_node);
    if (steps == 0U || end_node == corner_node)
        return 0U;

    dx = g_nodes[end_node].x - g_nodes[corner_node].x;
    dy = g_nodes[end_node].y - g_nodes[corner_node].y;
    length = sqrtf(dx * dx + dy * dy);
    if (length <= 0.0f)
        return 0U;

    dir_deg = atan2f(dx, dy) * EDGE_MOTION_RAD_TO_DEG_F;
    /* 倒车时车头与行进方向差 180°，转角必须按切线（行进方向）计算。 */
    travel_dir_deg = run_backward ?
                     EdgeTurn_WrapDeg(run_dir_deg + 180.0f) : run_dir_deg;
    turn_deg = EdgeTurn_WrapDeg(dir_deg - travel_dir_deg);

    if (edge_absf(turn_deg) <= tuning.turn_tolerance_deg)
        return 0U;  /* 直行通过，不是拐角 */
    if (edge_absf(turn_deg) > tuning.arc_max_deg)
        return 0U;  /* 掉头等大角度仍停车原地转 */

    min_run_m = tuning.arc_radius_m + tuning.arc_min_margin_m;
    if (run_length_m < min_run_m || length < min_run_m)
        return 0U;

    em->corner_valid = 1U;
    em->corner_reverse = run_backward ? 1U : 0U;
    em->corner_turn_deg = turn_deg;
    em->corner_next_to_node = end_node;
    em->corner_next_steps = steps;
    em->corner_next_world_dx_m = dx;
    em->corner_next_world_dy_m = dy;
    em->corner_next_length_m = length;
    em->corner_arc_radius_m = tuning.arc_radius_m;
    /* 弧后直线的车头目标航向：倒车时同样是切线+180°。 */
    em->corner_next_yaw_deg = run_backward ?
                              EdgeTurn_WrapDeg(dir_deg + 180.0f) : dir_deg;
    return 1U;
}

/*
 * 启动一段世界系直线。
 *
 * ramp_in=0 表示从圆弧出来，ramp_out 由 em->corner_valid 决定：
 * 有拐角时保持满速交给圆弧，没有拐角时正常减速停车。
 * 圆弧前后各要占用 R 米直线来过渡，因此实际平直段长度要扣掉对应半径。
 */
static void edge_start_straight(EdgeMotion_t *em,
                                float world_dx_m, float world_dy_m,
                                float body_dy_m,
                                float target_world_yaw_deg,
                                float run_length_m,
                                uint8_t ramp_in,
                                uint32_t now_ms)
{
    float trim_m = run_length_m;
    float scale = 1.0f;
    float entry_radius_m = em->arc_entry_radius_m;

    em->arc_entry_radius_m = 0.0f;
    if (!ramp_in)
        trim_m -= entry_radius_m;              /* 圆弧出口：用刚走过的圆弧半径 */
    if (em->corner_valid)
        trim_m -= em->corner_arc_radius_m;     /* 圆弧入口：用本段拐角的半径 */

    if (trim_m < 0.05f)
        trim_m = 0.05f;

    if (run_length_m > 0.0001f)
        scale = trim_m / run_length_m;

    chassis_motion_drift_segment(em->chassis,
                                 world_dx_m, world_dy_m,
                                 body_dy_m * scale,
                                 target_world_yaw_deg,
                                 ROUTE_EDGE_SPEED_MPS,
                                 ramp_in,
                                 em->corner_valid ? 0U : 1U,
                                 now_ms);
}

/*
 * 直线末段满速进入圆弧：切线方向 = 当前边行进方向，车头目标随之旋转。
 * 倒车边传入的切线是“车头航向 + 180°”，并把 reverse 交给底盘层，
 * 由底盘把车头锁在切线+180°。
 */
static void edge_start_corner_arc(EdgeMotion_t *em, uint32_t now_ms)
{
    float start_tangent_deg = em->edge_heading_ref_deg;

    if (em->corner_reverse)
        start_tangent_deg = EdgeTurn_WrapDeg(start_tangent_deg + 180.0f);

    em->chassis->acceleration = EDGE_MOTION_MOVE_ACC;
    em->arc_entry_radius_m = em->corner_arc_radius_m;
    chassis_motion_arc_world_ex(em->chassis,
                                start_tangent_deg,
                                em->corner_turn_deg,
                                em->corner_arc_radius_m,
                                ROUTE_EDGE_SPEED_MPS,
                                em->corner_reverse,
                                now_ms);
    em->state = EDGE_MOTION_CORNER_ARC;
}

static uint32_t edge_yaw_timeout_ms(float yaw_error_deg)
{
    uint32_t timeout_ms = EDGE_MOTION_YAW_TIMEOUT_BASE_MS +
        (uint32_t)(edge_absf(yaw_error_deg) *
                   (float)EDGE_MOTION_YAW_TIMEOUT_PER_DEG_MS);

    if (timeout_ms > EDGE_MOTION_YAW_TIMEOUT_MAX_MS)
        timeout_ms = EDGE_MOTION_YAW_TIMEOUT_MAX_MS;
    return timeout_ms;
}

/*
 * 启动/重新启动边末航向修正。
 *
 * 不再用固定 RPM。heading_control 根据角度误差和世界角速度生成本周期 wz：
 * 大误差时最高 80 RPM，接近目标后逐步降到 8 RPM，并用 gyro 项提前制动。
 */
static uint8_t edge_start_yaw_correction(EdgeMotion_t *em,
                                         const Hwt101 *imu,
                                         uint32_t now_ms)
{
    float current_yaw_deg;
    float yaw_error_deg;
    float world_gyro_dps;
    float wz_cmd;

    if (!em || !em->chassis || !imu)
        return 0U;

    current_yaw_deg = hwt101_world_yaw_deg(imu);
    yaw_error_deg = EdgeTurn_WrapDeg(current_yaw_deg -
                                     em->edge_heading_ref_deg);
    world_gyro_dps = HWT101_WORLD_YAW_SIGN * imu->gyro_z_dps;

    heading_set_target(&em->yaw_controller, em->edge_heading_ref_deg);
    wz_cmd = heading_update_world_cw(&em->yaw_controller,
                                     current_yaw_deg,
                                     world_gyro_dps,
                                     (float)EDGE_MOTION_YAW_REFRESH_MS / 1000.0f);
    if (wz_cmd == 0.0f)
        return 0U;

    em->chassis->acceleration = EDGE_MOTION_YAW_CORRECT_ACC;
    chassis_set_velocity(em->chassis, 0.0f, 0.0f, wz_cmd, now_ms);

    em->yaw_start_sign = edge_signf(yaw_error_deg);
    if (em->yaw_correct_deadline_ms == 0U)
    {
        em->yaw_correct_start_ms = now_ms;
        em->yaw_correct_deadline_ms =
            now_ms + edge_yaw_timeout_ms(yaw_error_deg);
    }
    em->next_yaw_refresh_ms = now_ms + EDGE_MOTION_YAW_REFRESH_MS;
    em->yaw_velocity_active = 1U;
    em->yaw_command_wz_radps = wz_cmd;
    em->state = EDGE_MOTION_YAW_CORRECTING;
    return 1U;
}

/*
 * 根据 edge_turn 的纯计算结果启动下一动作。
 *
 * 需要转向：下发最短转角的速度剖面，等待状态机复查 IMU。
 * 已对准：不再做世界->车身横移换算，固定 dx=0；dy由决策层选择
 *         +边长（前进）或-边长（后退），从执行层保证整条拓扑边只沿
 *         车身Y轴行驶，既杜绝大距离横移，也避免不必要的180°转向。
 *         直线段由 chassis_motion_drift_translate() 边走边锁航向。
 *
 * 启动直线段之前会向前预读一个拐角：终点要拐弯时直线末段保持满速，
 * 接着由 EDGE_MOTION_CORNER_ARC 走圆弧过去（见 edge_plan_corner）。
 */
static uint8_t edge_apply_turn_plan(EdgeMotion_t *em,
                                    float current_world_yaw_deg,
                                    uint32_t now_ms)
{
    EdgeTurnPlan_t plan;
    EdgeTurnOptions_t options;
    EdgeTurnTuning_t tuning;
    uint8_t navigation_target;

    if (!em || !em->chassis || !em->hmi_nav)
        return 0U;

    /* N2->N4 区段使用专用转弯参数，其他区段取全局值。 */
    edge_turn_tuning(em, &tuning);

    options.allow_reverse = tuning.allow_reverse;
    options.reverse_min_turn_saving_deg =
        tuning.reverse_min_turn_saving_deg;
    options.arrival_yaw_valid = 0U;
    options.arrival_world_yaw_deg = 0.0f;
    /* 合并段的起终点在 g_graph 中没有直接边，需要显式放行连通性检查。 */
    options.allow_merged_edge = (em->run_steps > 1U) ? 1U : 0U;

    /*
     * 只有to_node恰好是本次导航最终目标时，task_yaw才属于这一次到站动作。
     * 路径只是经过某个功能节点时不能套用其任务航向，否则会平白多转一次。
     */
    navigation_target = Navigation_GetTargetNode(&em->hmi_nav->nav);
    if (em->to_node == navigation_target &&
        navigation_target >= 1U && navigation_target <= NODE_NUM &&
        g_nodes[navigation_target].task_yaw_valid)
    {
        options.arrival_yaw_valid = 1U;
        options.arrival_world_yaw_deg = g_nodes[navigation_target].task_yaw_deg;
    }

    if (!EdgeTurn_Plan(em->from_node,
                       em->to_node,
                       current_world_yaw_deg,
                       tuning.turn_tolerance_deg,
                       &options,
                       &plan))
        return 0U;

    em->world_dx_m = plan.world_dx_m;
    em->world_dy_m = plan.world_dy_m;
    em->body_dx_m = 0.0f;
    em->body_dy_m = plan.body_dy_m;
    em->edge_heading_ref_deg = plan.target_world_yaw_deg;
    em->turn_world_error_deg = plan.world_yaw_error_deg;
    em->turn_chassis_command_deg = plan.chassis_turn_deg;
    em->drive_backward = plan.drive_backward;

    if (plan.turn_required)
    {
        if (em->turn_attempt >= ROUTE_TURN_MAX_ATTEMPTS)
            return 0U;

        em->chassis->acceleration = ROUTE_TURN_ACC;
        chassis_motion_rotate_deg(em->chassis,
                                  plan.chassis_turn_deg,
                                  tuning.turn_speed_mps,
                                  now_ms);
        em->turn_attempt++;
        em->state = EDGE_MOTION_TURNING_TO_EDGE;
        return 1U;
    }

    /*
     * 直线段改用漂移路径：世界方向由节点向量给出，车体中心沿世界直线
     * 前进/后退；底盘层在行驶过程中用 HWT101 世界航向做 PD，把车头锁在
     * 本边目标航向上。
     *
     * 同时预读本段终点处的拐角：能圆滑通过时，直线末段保持满速进入圆弧，
     * 由 EDGE_MOTION_CORNER_ARC 接着把车头转到下一条边的方向。
     */
    (void)edge_plan_corner(em,
                           em->to_node,
                           plan.target_world_yaw_deg,
                           plan.distance_m,
                           plan.drive_backward);

    em->chassis->acceleration = EDGE_MOTION_MOVE_ACC;
    edge_start_straight(em,
                        plan.world_dx_m,
                        plan.world_dy_m,
                        plan.body_dy_m,
                        plan.target_world_yaw_deg,
                        plan.distance_m,
                        1U,
                        now_ms);
    em->state = EDGE_MOTION_TRANSLATING;
    return 1U;
}

void EdgeMotion_Init(EdgeMotion_t *em, Chassis *chassis, HMI_Navigation_t *hmi_nav)
{
    if (!em) return;

    em->chassis = chassis;
    em->hmi_nav = hmi_nav;
    em->state = EDGE_MOTION_IDLE;
    em->saved_acceleration = 0U;
    em->acceleration_saved = 0U;
    em->world_dx_m = 0.0f;
    em->world_dy_m = 0.0f;
    em->body_dx_m = 0.0f;
    em->body_dy_m = 0.0f;
    em->edge_heading_ref_deg = 0.0f;
    em->turn_world_error_deg = 0.0f;
    em->turn_chassis_command_deg = 0.0f;
    em->drive_backward = 0U;
    em->turn_attempt = 0U;
    em->arrival_align_valid = 0U;
    em->corner_valid = 0U;
    em->corner_next_to_node = 0U;
    em->corner_next_steps = 0U;
    em->corner_reverse = 0U;
    em->corner_turn_deg = 0.0f;
    em->corner_next_world_dx_m = 0.0f;
    em->corner_next_world_dy_m = 0.0f;
    em->corner_next_length_m = 0.0f;
    em->corner_next_yaw_deg = 0.0f;
    em->corner_arc_radius_m = 0.0f;
    em->arc_entry_radius_m = 0.0f;
    heading_init(&em->yaw_controller,
                 EDGE_MOTION_YAW_KP,
                 EDGE_MOTION_YAW_KD,
                 edge_yaw_wz_for_rpm(chassis,
                                     EDGE_MOTION_YAW_CORRECT_MAX_RPM),
                 EDGE_MOTION_YAW_DEADBAND_DEG);
    heading_set_min_output(&em->yaw_controller,
                           edge_yaw_wz_for_rpm(chassis,
                                               EDGE_MOTION_YAW_CORRECT_MIN_RPM));
    edge_clear_runtime(em);
}

void EdgeMotion_BeginRoute(EdgeMotion_t *em)
{
    if (!em)
        return;

    /* 每条边领取时都会重新计算；这里只清理上一路线的观察值。 */
    em->edge_heading_ref_deg = 0.0f;
    em->turn_world_error_deg = 0.0f;
    em->turn_chassis_command_deg = 0.0f;
    em->drive_backward = 0U;
    em->turn_attempt = 0U;
    /* 新路线默认不假设上层会做任务航向，调用方需显式重新声明。 */
    em->arrival_align_valid = 0U;
}

void EdgeMotion_SetArrivalAlign(EdgeMotion_t *em, uint8_t valid)
{
    if (!em) return;
    em->arrival_align_valid = valid ? 1U : 0U;
}

void EdgeMotion_Update(EdgeMotion_t *em, uint32_t now_ms)
{
    NavigationState_t nav_state;
    Hwt101 *imu;
    float yaw_error_deg;
    int8_t yaw_sign;

    if (!em || !em->chassis || !em->hmi_nav ||
        !HMI_Navigation_IsInitialized(em->hmi_nav))
        return;

    nav_state = Navigation_GetState(&em->hmi_nav->nav);
    imu = AppUart_GetImu();

    /*
     * 真暂停必须停止物理运动。
     * 速度剖面被异步打断后剩余距离无法可靠还原，所以保持 10.4 原设计：
     * 不盲目 resume 当前边，而是进入 PAUSE_ABORT，等待上层重新定位/规划。
     */
    if (nav_state == NAV_STATE_PAUSED)
    {
        if (EdgeMotion_IsActive(em))
        {
            edge_stop_yaw_velocity(em);
            chassis_motion_stop(em->chassis);
            edge_restore_acceleration(em);
            em->state = EDGE_MOTION_PAUSE_ABORT;
        }
        return;
    }

    if (em->state == EDGE_MOTION_PAUSE_ABORT ||
        em->state == EDGE_MOTION_ERROR)
        return;

    /* 1) 正在把车头转到当前拓扑边方向。 */
    if (em->state == EDGE_MOTION_TURNING_TO_EDGE)
    {
        if (chassis_motion_failed(em->chassis))
        {
            edge_fail(em);
            return;
        }

        if (chassis_motion_done(em->chassis))
        {
            em->state = EDGE_MOTION_SETTLE_AFTER_TURN;
            em->settle_until_ms = now_ms + ROUTE_TURN_SETTLE_MS;
        }
        return;
    }

    /* 2) 转向后稳停并复查；有残差则最多补转 ROUTE_TURN_MAX_ATTEMPTS 次。 */
    if (em->state == EDGE_MOTION_SETTLE_AFTER_TURN)
    {
        if ((int32_t)(now_ms - em->settle_until_ms) < 0)
            return;

        if (!hwt101_is_fresh(imu, now_ms, EDGE_MOTION_IMU_TIMEOUT_MS) ||
            !hwt101_world_zero_ready(imu))
            return;

        if (!edge_apply_turn_plan(em, hwt101_world_yaw_deg(imu), now_ms))
            edge_fail(em);
        return;
    }

    /* 3) 正在执行当前拓扑边的纯纵向位置运动（可能前进，也可能后退）。 */
    if (em->state == EDGE_MOTION_TRANSLATING)
    {
        if (chassis_motion_failed(em->chassis))
        {
            edge_fail(em);
            return;
        }

        if (chassis_motion_done(em->chassis))
        {
            /* 终点处需要拐弯：不停车，直线末段保持满速直接进圆弧。 */
            if (em->corner_valid)
            {
                edge_start_corner_arc(em, now_ms);
                return;
            }

            em->state = EDGE_MOTION_SETTLE_AFTER_TRANSLATE;
            em->settle_until_ms = now_ms + EDGE_MOTION_YAW_SETTLE_MS;
        }
        return;
    }

    /* 3b) 圆弧通过拐角：结束后无缝接上下一条边的直线。 */
    if (em->state == EDGE_MOTION_CORNER_ARC)
    {
        float next_dx;
        float next_dy;
        float next_length;
        float next_yaw;
        uint8_t next_to_node;
        uint8_t next_steps;
        uint8_t next_reverse;

        if (chassis_motion_failed(em->chassis))
        {
            edge_fail(em);
            return;
        }

        if (!chassis_motion_done(em->chassis))
            return;

        next_dx = em->corner_next_world_dx_m;
        next_dy = em->corner_next_world_dy_m;
        next_length = em->corner_next_length_m;
        next_yaw = em->corner_next_yaw_deg;
        next_to_node = em->corner_next_to_node;
        next_steps = em->corner_next_steps;
        next_reverse = em->corner_reverse;

        /*
         * 圆弧已经越过拐角节点，但 Navigation 只在整段直线（含圆弧）
         * 真正结束并完成航向确认后才提交；这里先累计拓扑边条数。
         */
        em->from_node = em->to_node;
        em->to_node = next_to_node;
        em->run_steps = (uint8_t)(em->run_steps + next_steps);

        em->world_dx_m = next_dx;
        em->world_dy_m = next_dy;
        em->body_dx_m = 0.0f;
        em->body_dy_m = next_reverse ? -next_length : next_length;
        em->edge_heading_ref_deg = next_yaw;
        em->drive_backward = next_reverse;

        /* 继续预读下一个拐角，支持连续拐弯不停车（倒车段继续按倒车预读）。 */
        (void)edge_plan_corner(em, em->to_node, next_yaw,
                               next_length, next_reverse);

        em->chassis->acceleration = EDGE_MOTION_MOVE_ACC;
        edge_start_straight(em, next_dx, next_dy,
                            next_reverse ? -next_length : next_length,
                            next_yaw, next_length, 0U, now_ms);
        em->state = EDGE_MOTION_TRANSLATING;
        return;
    }

    /* 4) 纵向平移完成后先等车体/IMU稳定，再读取边末yaw。 */
    if (em->state == EDGE_MOTION_SETTLE_AFTER_TRANSLATE)
    {
        if ((int32_t)(now_ms - em->settle_until_ms) < 0)
            return;

        /*
         * IMU 暂时没有新帧时保持停车等待，不推进节点。
         * 这和 10.4 VOFA 实车逻辑一致，避免“没有航向数据也当作到边”。
         */
        if (!hwt101_is_fresh(imu, now_ms, EDGE_MOTION_IMU_TIMEOUT_MS))
            return;

        yaw_error_deg = EdgeTurn_WrapDeg(hwt101_world_yaw_deg(imu) -
                                         em->edge_heading_ref_deg);

        if (edge_absf(yaw_error_deg) <= EDGE_MOTION_YAW_DEADBAND_DEG)
        {
            (void)edge_commit(em);
            return;
        }

        /*
         * 后面本来就要转向时，不要“先纠到当前边航向、紧接着再转走”。
         * 直接提交节点，让任务航向 PD 或下一段拓扑边转向一次到位。
         */
        if (edge_following_turn_covers_heading(em,
                                               hwt101_world_yaw_deg(imu)))
        {
            (void)edge_commit(em);
            return;
        }

        /* 任意角度都进入闭环纠偏；机械卡死仍由动态超时保护。 */
        if (!edge_start_yaw_correction(em, imu, now_ms))
            edge_fail(em);
        return;
    }

    /* 5) 自适应闭环纠偏：远处高速、近处降速，达到死区/穿零立即停车。 */
    if (em->state == EDGE_MOTION_YAW_CORRECTING)
    {
        if (!hwt101_is_fresh(imu, now_ms, EDGE_MOTION_IMU_TIMEOUT_MS))
        {
            edge_fail(em);
            return;
        }

        yaw_error_deg = EdgeTurn_WrapDeg(hwt101_world_yaw_deg(imu) -
                                         em->edge_heading_ref_deg);
        yaw_sign = edge_signf(yaw_error_deg);

        if (edge_absf(yaw_error_deg) <= EDGE_MOTION_YAW_DEADBAND_DEG ||
            (yaw_sign != 0 && yaw_sign != em->yaw_start_sign))
        {
            edge_stop_yaw_velocity(em);
            em->state = EDGE_MOTION_SETTLE_AFTER_YAW;
            em->settle_until_ms = 0U; /* 等 STOP 真正完成后再开始 150 ms 稳定计时。 */
            return;
        }

        if ((int32_t)(now_ms - em->yaw_correct_deadline_ms) >= 0)
        {
            edge_fail(em);
            return;
        }

        /*
         * 刷新 Chassis 速度看门狗；若主循环/EdgeMotion 卡死，不刷新后底盘会自行 STOP。
         */
        if ((int32_t)(now_ms - em->next_yaw_refresh_ms) >= 0)
        {
            float world_gyro_dps = HWT101_WORLD_YAW_SIGN * imu->gyro_z_dps;
            float wz_cmd = heading_update_world_cw(
                &em->yaw_controller,
                hwt101_world_yaw_deg(imu),
                world_gyro_dps,
                (float)EDGE_MOTION_YAW_REFRESH_MS / 1000.0f);

            chassis_set_velocity(em->chassis, 0.0f, 0.0f, wz_cmd, now_ms);
            em->yaw_command_wz_radps = wz_cmd;
            em->next_yaw_refresh_ms = now_ms + EDGE_MOTION_YAW_REFRESH_MS;
        }
        return;
    }

    /* 6) 纠偏停车完成后再稳定一次，随后才真正提交当前节点。 */
    if (em->state == EDGE_MOTION_SETTLE_AFTER_YAW)
    {
        if (chassis_motion_busy(em->chassis))
            return;

        if (em->settle_until_ms == 0U)
        {
            em->settle_until_ms = now_ms + EDGE_MOTION_YAW_SETTLE_MS;
            return;
        }

        if ((int32_t)(now_ms - em->settle_until_ms) < 0)
            return;

        if (!hwt101_is_fresh(imu, now_ms, EDGE_MOTION_IMU_TIMEOUT_MS))
            return;

        /*
         * STOP 后必须重新验角。若惯性造成超调，不提交节点，而是反向小速补偿；
         * 只有真正落入死区才允许 Navigation 前进。
         */
        yaw_error_deg = EdgeTurn_WrapDeg(hwt101_world_yaw_deg(imu) -
                                         em->edge_heading_ref_deg);
        if (edge_absf(yaw_error_deg) <= EDGE_MOTION_YAW_DEADBAND_DEG)
        {
            (void)edge_commit(em);
            return;
        }

        if ((int32_t)(now_ms - em->yaw_correct_deadline_ms) >= 0 ||
            !edge_start_yaw_correction(em, imu, now_ms))
            edge_fail(em);
        return;
    }

    /* 7) IDLE：Navigation 处于 RUNNING 时领取下一条待执行拓扑边。 */
    if (em->state != EDGE_MOTION_IDLE || nav_state != NAV_STATE_RUNNING)
        return;

    if (HMI_Navigation_GetCurrentEdge(em->hmi_nav,
                                      &em->from_node, &em->to_node))
    {
        float current_yaw_deg;

        /* 正式 Mission 使用整场唯一世界零点，绝不在每条边重新清零 HWT101。 */
        if (!hwt101_is_fresh(imu, now_ms, EDGE_MOTION_IMU_TIMEOUT_MS) ||
            !hwt101_world_zero_ready(imu))
            return;

        current_yaw_deg = hwt101_world_yaw_deg(imu);
        edge_save_acceleration(em);
        em->turn_attempt = 0U;

        /*
         * 先向前预读路径：共线同向的连续边合并成一次直线平移，
         * 只在需要拐弯的节点或本次导航目标节点停车。
         */
        em->run_steps = edge_merge_straight_run(em,
                                                em->from_node,
                                                &em->to_node);

        if (!edge_apply_turn_plan(em, current_yaw_deg, now_ms))
            edge_fail(em);
    }
}

void EdgeMotion_EStop(EdgeMotion_t *em)
{
    if (!em || !em->chassis) return;

    edge_stop_yaw_velocity(em);
    chassis_motion_stop(em->chassis);
    edge_restore_acceleration(em);
    em->state = EDGE_MOTION_ERROR;
}

EdgeMotionState_t EdgeMotion_GetState(const EdgeMotion_t *em)
{
    return em ? em->state : EDGE_MOTION_ERROR;
}

uint8_t EdgeMotion_IsActive(const EdgeMotion_t *em)
{
    if (!em) return 0U;

    return (em->state == EDGE_MOTION_TURNING_TO_EDGE ||
            em->state == EDGE_MOTION_SETTLE_AFTER_TURN ||
            em->state == EDGE_MOTION_CORNER_ARC ||
            em->state == EDGE_MOTION_TRANSLATING ||
            em->state == EDGE_MOTION_SETTLE_AFTER_TRANSLATE ||
            em->state == EDGE_MOTION_YAW_CORRECTING ||
            em->state == EDGE_MOTION_SETTLE_AFTER_YAW) ? 1U : 0U;
}
