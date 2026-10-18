#include "navigation.h"
#include <string.h>

static uint8_t node_valid(uint8_t n) { return n >= 1U && n <= NODE_NUM; }
static void clear_path(PathResult_t *p) { if (p) memset(p, 0, sizeof(*p)); }

uint8_t Navigation_Init(Navigation_t *nav, uint8_t start_node,
                        uint32_t obstacle_mask)
{
    if (!nav || !node_valid(start_node)) return 0U;
    memset(nav, 0, sizeof(*nav));
    nav->obstacle_mask = obstacle_mask;
    nav->current_node = start_node;
    nav->state = NAV_STATE_IDLE;
    MapGraph_Build(obstacle_mask);
    PathExecutor_Init(&nav->executor);
    return 1U;
}

uint8_t Navigation_SetObstacleMask(Navigation_t *nav, uint32_t obstacle_mask)
{
    if (!nav) return 0U;
    nav->obstacle_mask = obstacle_mask;
    MapGraph_Build(obstacle_mask);
    return 1U;
}

uint8_t Navigation_SetCurrentNode(Navigation_t *nav, uint8_t node)
{
    if (!nav || !node_valid(node)) return 0U;
    nav->current_node = node;
    nav->target_node = 0U;
    nav->state = NAV_STATE_IDLE;
    clear_path(&nav->path);
    PathExecutor_Init(&nav->executor);
    return 1U;
}

uint8_t Navigation_PlanTo(Navigation_t *nav, uint8_t target_node)
{
    if (!nav || !node_valid(nav->current_node) || !node_valid(target_node)) {
        if (nav) nav->state = NAV_STATE_ERROR;
        return 0U;
    }
    nav->target_node = target_node;
    nav->path = PathPlanner_Dijkstra(nav->current_node, target_node);
    if (!nav->path.valid) {
        PathExecutor_Init(&nav->executor);
        nav->state = NAV_STATE_NO_PATH;
        return 0U;
    }
    PathExecutor_Init(&nav->executor);
    if (!PathExecutor_Start(&nav->executor, &nav->path)) {
        nav->state = NAV_STATE_ERROR;
        return 0U;
    }
    if (PathExecutor_IsDone(&nav->executor)) {
        nav->current_node = PathExecutor_GetCurrentNode(&nav->executor);
        nav->state = NAV_STATE_DONE;
    } else {
        nav->state = NAV_STATE_RUNNING;
    }
    return 1U;
}

uint8_t Navigation_Replan(Navigation_t *nav)
{
    uint8_t target;
    if (!nav || !node_valid(nav->current_node) || !node_valid(nav->target_node)) {
        if (nav) nav->state = NAV_STATE_ERROR;
        return 0U;
    }
    target = nav->target_node;
    return Navigation_PlanTo(nav, target);
}

uint8_t Navigation_GetCurrentEdge(const Navigation_t *nav,
                                  uint8_t *from_node, uint8_t *to_node)
{
    uint8_t cur, next;
    if (!nav || !from_node || !to_node || nav->state != NAV_STATE_RUNNING)
        return 0U;
    cur = PathExecutor_GetCurrentNode(&nav->executor);
    next = PathExecutor_GetNextNode(&nav->executor);
    if (!node_valid(cur) || !node_valid(next)) return 0U;
    *from_node = cur;
    *to_node = next;
    return 1U;
}

uint8_t Navigation_GetFollowingEdge(const Navigation_t *nav,
                                    uint8_t *from_node, uint8_t *to_node)
{
    const PathExecutor_t *exec;

    if (!nav || !from_node || !to_node || nav->state != NAV_STATE_RUNNING)
        return 0U;

    exec = &nav->executor;
    if (exec->state != PATH_EXEC_RUNNING)
        return 0U;

    /* 当前边 = path[index] -> path[index+1]，下一条边从 index+1 开始。 */
    if ((uint16_t)exec->index + 2U >= (uint16_t)exec->path.length)
        return 0U;

    *from_node = exec->path.node[exec->index + 1U];
    *to_node   = exec->path.node[exec->index + 2U];
    return 1U;
}

uint8_t Navigation_EdgeReached(Navigation_t *nav)
{
    if (!nav || nav->state != NAV_STATE_RUNNING) return 0U;
    if (!PathExecutor_EdgeReached(&nav->executor)) {
        nav->state = NAV_STATE_ERROR;
        return 0U;
    }
    nav->current_node = PathExecutor_GetCurrentNode(&nav->executor);
    if (!node_valid(nav->current_node)) {
        nav->state = NAV_STATE_ERROR;
        return 0U;
    }
    if (PathExecutor_IsDone(&nav->executor)) {
        if (nav->current_node != nav->target_node) {
            nav->state = NAV_STATE_ERROR;
            return 0U;
        }
        nav->state = NAV_STATE_DONE;
    }
    return 1U;
}

uint8_t Navigation_Pause(Navigation_t *nav)
{
    if (!nav) return 0U;
    if (nav->state == NAV_STATE_PAUSED) return 1U;
    if (nav->state != NAV_STATE_RUNNING) return 0U;
    PathExecutor_Pause(&nav->executor);
    if (PathExecutor_GetState(&nav->executor) != PATH_EXEC_PAUSED) {
        nav->state = NAV_STATE_ERROR; return 0U;
    }
    nav->state = NAV_STATE_PAUSED;
    return 1U;
}

uint8_t Navigation_Resume(Navigation_t *nav)
{
    if (!nav || nav->state != NAV_STATE_PAUSED) return 0U;
    PathExecutor_Resume(&nav->executor);
    if (PathExecutor_GetState(&nav->executor) != PATH_EXEC_RUNNING) {
        nav->state = NAV_STATE_ERROR; return 0U;
    }
    nav->state = NAV_STATE_RUNNING;
    return 1U;
}

void Navigation_Abort(Navigation_t *nav)
{
    if (!nav) return;
    nav->target_node = 0U;
    nav->state = NAV_STATE_IDLE;
    clear_path(&nav->path);
    PathExecutor_Init(&nav->executor);
}

NavigationState_t Navigation_GetState(const Navigation_t *nav)
{ return nav ? nav->state : NAV_STATE_ERROR; }

uint8_t Navigation_GetCurrentNode(const Navigation_t *nav)
{ return nav ? nav->current_node : 0U; }

uint8_t Navigation_GetTargetNode(const Navigation_t *nav)
{ return nav ? nav->target_node : 0U; }

const PathResult_t *Navigation_GetPath(const Navigation_t *nav)
{ return nav ? &nav->path : 0; }

uint8_t Navigation_IsDone(const Navigation_t *nav)
{ return nav && nav->state == NAV_STATE_DONE; }

uint8_t Navigation_HasError(const Navigation_t *nav)
{ return !nav || nav->state == NAV_STATE_NO_PATH || nav->state == NAV_STATE_ERROR; }
