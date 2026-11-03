#include "path_planner.h"
#include "map_graph.h"
#include "robot_config.h"


//====================================================
// 直线优先的最短路
//
// 普通 Dijkstra 只看“走了多远”，在等距路线上会随便挑一条，
// 于是经常选到拐弯更多的路。这里把状态扩展成 (节点, 进入方向)，
// 松弛时给“方向发生变化”额外计一段代价：
//
//   直行       +0
//   转 90 deg  +ROUTE_TURN_PENALTY_MM
//   掉头       +ROUTE_UTURN_PENALTY_MM
//
// 代价单位与边长一致（mm），所以“多绕一条边但一路直线”的路线会
// 赢过“距离略短但拐三次弯”的路线。真实边长仍然是主项，转弯只是
// 次级偏好，不会为了直行绕出离谱的大圈。
//====================================================

/* 进入某个节点时的行驶方向。世界坐标：+X 向右、+Y 向上。 */
#define PP_DIR_POS_Y     0U
#define PP_DIR_POS_X     1U
#define PP_DIR_NEG_Y     2U
#define PP_DIR_NEG_X     3U

/* 起点没有“进入方向”，因此不产生任何转弯惩罚。 */
#define PP_DIR_NONE      4U
#define PP_DIR_COUNT     5U

/* 内部用 32 位无穷大，避免“路程 + 多次掉头惩罚”超过 INF_COST 后被误判为不可达。 */
#define PP_INF           0xFFFFFFFFUL


//====================================================
// 由相邻节点坐标得到边的几何行驶方向
//====================================================
static uint8_t pp_edge_dir(uint8_t from_node, uint8_t to_node)
{
    float dx = g_nodes[to_node].x - g_nodes[from_node].x;
    float dy = g_nodes[to_node].y - g_nodes[from_node].y;

    if (dx > ROUTE_COORD_EPS_M)
    {
        return PP_DIR_POS_X;
    }

    if (dx < -ROUTE_COORD_EPS_M)
    {
        return PP_DIR_NEG_X;
    }

    if (dy > ROUTE_COORD_EPS_M)
    {
        return PP_DIR_POS_Y;
    }

    if (dy < -ROUTE_COORD_EPS_M)
    {
        return PP_DIR_NEG_Y;
    }

    /* 两个节点坐标重合，拓扑异常；按不可走处理。 */
    return PP_DIR_NONE;
}


//====================================================
// 从 in_dir 转入 out_dir 需要付出的额外代价
//====================================================
static uint32_t pp_turn_penalty(uint8_t in_dir, uint8_t out_dir)
{
    if (in_dir == PP_DIR_NONE || out_dir == PP_DIR_NONE)
    {
        return 0U;
    }

    if (in_dir == out_dir)
    {
        return 0U;
    }

    /* 相差 180 deg 即掉头。 */
    if ((uint8_t)((in_dir + 2U) & 3U) == out_dir)
    {
        return ROUTE_UTURN_PENALTY_MM;
    }

    return ROUTE_TURN_PENALTY_MM;
}


//====================================================
// Dijkstra 最短路（直线优先）
//
// 只做纯粹的“起点 -> 终点”最短路；固定区段绕行偏好由下面的
// PathPlanner_Dijkstra() 包装层处理，避免偏好逻辑污染搜索核心。
//====================================================
static PathResult_t pp_dijkstra_core(uint8_t start_node,
                                     uint8_t target_node)
{
    uint32_t dist[NODE_NUM + 1][PP_DIR_COUNT];
    uint8_t visited[NODE_NUM + 1][PP_DIR_COUNT];
    uint8_t prev_node[NODE_NUM + 1][PP_DIR_COUNT];
    uint8_t prev_dir[NODE_NUM + 1][PP_DIR_COUNT];
    uint8_t center_count[NODE_NUM + 1][PP_DIR_COUNT];

    uint8_t reverse_path[PATH_MAX_LEN];

    uint8_t i;
    uint8_t j;
    uint8_t d;
    uint8_t u;
    uint8_t u_dir;
    uint8_t target_dir = PP_DIR_NONE;

    uint8_t reverse_len = 0;
    uint32_t min_dist;
    uint32_t new_dist;
    uint32_t penalty;
    uint8_t min_center_count;
    uint8_t new_center_count;
    uint16_t total_cost;

    PathResult_t result;


    //------------------------------------------------
    // 初始化返回值
    //------------------------------------------------
    result.length = 0;
    result.valid = 0;
    result.total_cost = 0;

    for(i = 0; i < PATH_MAX_LEN; i++)
    {
        result.node[i] = 0;
    }


    //------------------------------------------------
    // 检查节点编号
    //------------------------------------------------
    if(start_node < 1 || start_node > NODE_NUM)
    {
        return result;
    }

    if(target_node < 1 || target_node > NODE_NUM)
    {
        return result;
    }


    //------------------------------------------------
    // 起点就是终点
    //------------------------------------------------
    if(start_node == target_node)
    {
        result.node[0] = start_node;
        result.length = 1;
        result.valid = 1;
        result.total_cost = 0;

        return result;
    }


    //------------------------------------------------
    // 初始化状态空间
    //------------------------------------------------
    for(i = 1; i <= NODE_NUM; i++)
    {
        for(d = 0; d < PP_DIR_COUNT; d++)
        {
            dist[i][d] = PP_INF;
            visited[i][d] = 0;
            prev_node[i][d] = 0;
            prev_dir[i][d] = PP_DIR_NONE;
            center_count[i][d] = 0U;
        }
    }

    /* 起点只放出“无进入方向”这一个状态，出边不受转弯惩罚。 */
    dist[start_node][PP_DIR_NONE] = 0U;


    //------------------------------------------------
    // 主循环
    //------------------------------------------------
    for(i = 0; i <= (uint8_t)(NODE_NUM * PP_DIR_COUNT); i++)
    {
        min_dist = PP_INF;
        min_center_count = 0U;
        u = 0;
        u_dir = PP_DIR_NONE;


        //------------------------------------------------
        // 找当前代价最小且未访问的状态
        //------------------------------------------------
        for(j = 1; j <= NODE_NUM; j++)
        {
            for(d = 0; d < PP_DIR_COUNT; d++)
            {
                if(visited[j][d] || dist[j][d] == PP_INF)
                {
                    continue;
                }

                if((dist[j][d] < min_dist) ||
                   ((dist[j][d] == min_dist) &&
                    (center_count[j][d] > min_center_count)))
                {
                    min_dist = dist[j][d];
                    min_center_count = center_count[j][d];
                    u = j;
                    u_dir = d;
                }
            }
        }


        //------------------------------------------------
        // 已经没有可达状态
        //------------------------------------------------
        if(u == 0)
        {
            break;
        }


        //------------------------------------------------
        // 目标首次出队即为最优（含转弯代价）
        //------------------------------------------------
        if(u == target_node)
        {
            target_dir = u_dir;
            break;
        }


        visited[u][u_dir] = 1;


        //------------------------------------------------
        // 松弛所有邻居
        //------------------------------------------------
        for(j = 1; j <= NODE_NUM; j++)
        {
            uint8_t out_dir;

            if(g_graph[u][j] >= INF_COST)
            {
                continue;
            }

            out_dir = pp_edge_dir(u, j);
            if(out_dir == PP_DIR_NONE)
            {
                continue;
            }

            penalty = pp_turn_penalty(u_dir, out_dir);
            new_dist = dist[u][u_dir] +
                       (uint32_t)g_graph[u][j] +
                       penalty;

#if ROUTE_CENTER_TIE_BREAK
            new_center_count = (uint8_t)(center_count[u][u_dir] +
                ((j == ROUTE_CENTER_NODE) ? 1U : 0U));
#else
            new_center_count = center_count[u][u_dir];
#endif

            if(visited[j][out_dir])
            {
                continue;
            }

            if((new_dist < dist[j][out_dir]) ||
               ((new_dist == dist[j][out_dir]) &&
                (new_center_count > center_count[j][out_dir])))
            {
                dist[j][out_dir] = new_dist;
                center_count[j][out_dir] = new_center_count;
                prev_node[j][out_dir] = u;
                prev_dir[j][out_dir] = u_dir;
            }
        }
    }


    //------------------------------------------------
    // 目标不可达
    //------------------------------------------------
    if(target_dir == PP_DIR_NONE ||
       dist[target_node][target_dir] == PP_INF)
    {
        return result;
    }


    //------------------------------------------------
    // 从终点逆向恢复路径
    //------------------------------------------------
    u = target_node;
    d = target_dir;

    while(u != 0)
    {
        if(reverse_len >= PATH_MAX_LEN)
        {
            return result;
        }

        reverse_path[reverse_len] = u;
        reverse_len++;

        if(u == start_node)
        {
            break;
        }

        j = prev_node[u][d];
        d = prev_dir[u][d];
        u = j;
    }


    //------------------------------------------------
    // 如果最终没回到起点，说明路径异常
    //------------------------------------------------
    if(reverse_path[reverse_len - 1] != start_node)
    {
        return result;
    }


    //------------------------------------------------
    // 反转为起点 -> 终点
    //------------------------------------------------
    result.length = reverse_len;

    for(i = 0; i < reverse_len; i++)
    {
        result.node[i] =
            reverse_path[reverse_len - 1 - i];
    }

    /*
     * total_cost 仍只累计真实边长，保持“路程”语义：HMI/测试拿到的
     * 是实际要走的距离，转弯偏好只影响选路，不污染这个数值。
     */
    total_cost = 0U;
    for(i = 0; i + 1U < reverse_len; i++)
    {
        uint16_t edge_cost = g_graph[result.node[i]][result.node[i + 1U]];

        if(edge_cost >= INF_COST)
        {
            return result;
        }

        if((uint32_t)total_cost + (uint32_t)edge_cost >= INF_COST)
        {
            total_cost = INF_COST - 1U;
            break;
        }

        total_cost = (uint16_t)(total_cost + edge_cost);
    }

    result.total_cost = total_cost;
    result.valid = 1;

    return result;
}


//====================================================
// 固定区段绕行偏好
//
// 有些起终点对的两条候选路线距离和转弯数完全相同，Dijkstra 只能靠
// ROUTE_CENTER_TIE_BREAK 随便挑一条，但现场动线希望固定走某一条：
//   N6(扫码) -> N8(原料)   优先 N6->N9->N8（右侧车道先北上）
//   N2(粗加工) -> N4(暂存) 优先 N2->N1->N4（左侧车道先西行）
//   N4(暂存) -> N8(原料)   优先 N4->N7->N8（左侧车道先北上，
//                          在 N7 用圆弧拐角通过，不在拐角节点停车）
//   N4(暂存) -> N3/N9(启停) 优先经 N8：返回段沿用左侧车道 N4->N7->N8->N9，
//                          该路线被封路切断时回退最短路（一般经中央路口）。
// 具体节点号由 robot_config.h 的 ROUTE_VIA_* 配置，0 表示不设偏好。
//====================================================
static uint8_t pp_preferred_via(uint8_t start_node, uint8_t target_node)
{
#if (ROUTE_VIA_QR_TO_RAW_NODE != 0)
    if((start_node == MISSION_QR_NODE) &&
       (target_node == MISSION_RAW_NODE))
    {
        return (uint8_t)ROUTE_VIA_QR_TO_RAW_NODE;
    }
#endif

#if (ROUTE_VIA_ROUGH_TO_STORAGE_NODE != 0)
    if((start_node == MISSION_ROUGH_NODE) &&
       (target_node == MISSION_STORAGE_NODE))
    {
        return (uint8_t)ROUTE_VIA_ROUGH_TO_STORAGE_NODE;
    }
#endif

#if (ROUTE_VIA_STORAGE_TO_RAW_NODE != 0)
    if((start_node == MISSION_STORAGE_NODE) &&
       (target_node == MISSION_RAW_NODE))
    {
        return (uint8_t)ROUTE_VIA_STORAGE_TO_RAW_NODE;
    }
#endif

#if (ROUTE_VIA_STORAGE_TO_RETURN_NODE != 0)
    /*
     * 返回启停区：N4->N3/N9 优先走 N7->N8 这条左侧车道。
     * 返回节点是 N9 时与经中央路口的 4-5-6-9 等长；是 N3 时这条路线
     * 会多绕一整圈，由下面的 ROUTE_VIA_MAX_EXTRA_MM 门限自动回退最短路。
     */
    if((start_node == MISSION_STORAGE_NODE) &&
       MISSION_IS_RETURN_NODE(target_node))
    {
        return (uint8_t)ROUTE_VIA_STORAGE_TO_RETURN_NODE;
    }
#endif

    return 0U;
}


//====================================================
// 拼接“起点->绕行节点”与“绕行节点->终点”两段路径
//
// 第二段的首节点就是绕行节点，与第一段尾节点重复，只取其后半段。
//====================================================
static PathResult_t pp_concat_path(const PathResult_t *head,
                                   const PathResult_t *tail)
{
    PathResult_t out = *head;
    uint8_t i;

    out.valid = 0U;

    for(i = 1U; i < tail->length; i++)
    {
        if(out.length >= PATH_MAX_LEN)
        {
            return out;
        }

        out.node[out.length] = tail->node[i];
        out.length++;
    }

    if((uint32_t)out.total_cost + (uint32_t)tail->total_cost >= INF_COST)
    {
        out.total_cost = INF_COST - 1U;
    }
    else
    {
        out.total_cost = (uint16_t)(out.total_cost + tail->total_cost);
    }

    out.valid = 1U;
    return out;
}


//====================================================
// 对外入口：先按现场动线尝试偏好绕行，封路则回退最短路
//====================================================
PathResult_t PathPlanner_Dijkstra(uint8_t start_node,
                                  uint8_t target_node)
{
    PathResult_t direct;
    PathResult_t via_path;
    uint8_t via;

    direct = pp_dijkstra_core(start_node, target_node);

    via = pp_preferred_via(start_node, target_node);

    if((via == 0U) || (via == start_node) || (via == target_node))
    {
        return direct;
    }

    via_path = pp_dijkstra_core(start_node, via);

    if(via_path.valid)
    {
        PathResult_t tail = pp_dijkstra_core(via, target_node);

        if(tail.valid)
        {
            via_path = pp_concat_path(&via_path, &tail);
        }
        else
        {
            via_path.valid = 0U;
        }
    }

    if(!via_path.valid)
    {
        return direct;
    }

    /*
     * 绕行节点本身被封：搜索仍可能绕一大圈从别的方向摸到 via，拼出来的
     * 路径会多走整条边再折回来。只有“不比最短路长太多”才认为偏好路线
     * 可用，否则视为该区段封路，回退普通最短路。
     */
    if(direct.valid &&
       ((uint32_t)via_path.total_cost >
        (uint32_t)direct.total_cost + (uint32_t)ROUTE_VIA_MAX_EXTRA_MM))
    {
        return direct;
    }

    return via_path;
}
