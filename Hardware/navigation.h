#ifndef __NAVIGATION_H
#define __NAVIGATION_H
#include <stdint.h>
#include "map_graph.h"
#include "path_planner.h"
#include "path_executor.h"

/* Navigation只负责节点级导航逻辑，不直接控制底盘/电机。 */
typedef enum {
    NAV_STATE_IDLE = 0,
    NAV_STATE_RUNNING,
    NAV_STATE_PAUSED,
    NAV_STATE_DONE,
    NAV_STATE_NO_PATH,
    NAV_STATE_ERROR
} NavigationState_t;

typedef struct {
    NavigationState_t state;
    uint32_t obstacle_mask;
    uint8_t current_node; /* 最后确认实际到达的节点 */
    uint8_t target_node;  /* 当前导航目标 */
    PathResult_t path;    /* 最近一次Dijkstra结果，供调试/HMI读取 */
    PathExecutor_t executor;
} Navigation_t;

/**
 * @brief 初始化导航模块并按障碍掩码建立地图。
 * @param nav Navigation对象。
 * @param start_node 当前实际节点，范围1~NODE_NUM。
 * @param obstacle_mask 障碍掩码。
 * @return 1成功，0参数非法。成功后状态为IDLE。
 */
uint8_t Navigation_Init(Navigation_t *nav, uint8_t start_node,
                        uint32_t obstacle_mask);

/**
 * @brief 更新障碍地图。
 * @return 1成功，0失败。
 * @warning 只重建地图，不自动改道；需要改道时随后调用Navigation_Replan()。
 */
uint8_t Navigation_SetObstacleMask(Navigation_t *nav, uint32_t obstacle_mask);

/**
 * @brief 人工/定位模块校正当前节点。
 * @return 1成功，0节点非法。
 * @warning 会清除旧导航任务；正常行驶不要用它代替Navigation_EdgeReached()。
 */
uint8_t Navigation_SetCurrentNode(Navigation_t *nav, uint8_t node);

/**
 * @brief 从当前确认节点规划并开始前往target_node。
 * @return 1规划并装载成功，0无路或参数非法。
 * @note current==target时成功并立即进入DONE。
 */
uint8_t Navigation_PlanTo(Navigation_t *nav, uint8_t target_node);

/**
 * @brief 从当前确认节点重新规划到原target_node。
 * @return 1成功，0失败/无路。
 * @note 典型用法：SetObstacleMask()后调用本函数进行动态改道。
 */
uint8_t Navigation_Replan(Navigation_t *nav);

/**
 * @brief 获取当前需要真实车辆执行的地图边。
 * @param from_node 输出当前边起点。
 * @param to_node 输出当前边终点。
 * @return 1有待执行边，0当前没有可执行边。
 * @note 后续edge_motion层主要消费此接口。
 */
uint8_t Navigation_GetCurrentEdge(const Navigation_t *nav,
                                  uint8_t *from_node, uint8_t *to_node);

/**
 * @brief 预看当前边之后的下一条边，不推进路径进度。
 *
 * @param from_node 输出下一条边的起点（即当前边的终点）。
 * @param to_node   输出下一条边的终点。
 * @return 1 存在下一条边；0 当前边已是最后一条或导航未在运行。
 *
 * @note 边运动层在提交当前节点前用它预测“到站后是否还要转向”，
 *       从而避免先纠到当前边航向、紧接着又转走一次。
 */
uint8_t Navigation_GetFollowingEdge(const Navigation_t *nav,
                                    uint8_t *from_node, uint8_t *to_node);

/**
 * @brief 通知导航层：当前from->to边已经真实执行完成。
 * @return 1成功推进，0当前状态不能推进。
 * @warning 实车必须确认到达to_node后才能调用；PC测试可模拟调用。
 */
uint8_t Navigation_EdgeReached(Navigation_t *nav);

/** @brief 暂停导航逻辑进度。实际底盘停车以后由运动层负责。 */
uint8_t Navigation_Pause(Navigation_t *nav);

/** @brief 恢复PAUSED导航。 */
uint8_t Navigation_Resume(Navigation_t *nav);

/**
 * @brief 中止当前任务并回到IDLE。
 * @note 保留最后确认到达的current_node，清除目标和旧路径。
 */
void Navigation_Abort(Navigation_t *nav);

/** @brief 查询导航状态；nav为空返回NAV_STATE_ERROR。 */
NavigationState_t Navigation_GetState(const Navigation_t *nav);

/** @brief 获取最后确认到达节点；失败返回0。 */
uint8_t Navigation_GetCurrentNode(const Navigation_t *nav);

/** @brief 获取当前目标节点；无目标/失败返回0。 */
uint8_t Navigation_GetTargetNode(const Navigation_t *nav);

/**
 * @brief 获取最近一次规划结果的只读指针。
 * @note 返回Navigation内部对象，不要由调用者修改。
 */
const PathResult_t *Navigation_GetPath(const Navigation_t *nav);

/** @brief 是否已到达本次目标。 */
uint8_t Navigation_IsDone(const Navigation_t *nav);

/** @brief 是否处于NO_PATH或ERROR。 */
uint8_t Navigation_HasError(const Navigation_t *nav);
#endif
