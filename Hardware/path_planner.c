#include "path_planner.h"
#include "map_graph.h"
#include "robot_config.h"


//====================================================
// Dijkstra 最短路
//====================================================
PathResult_t PathPlanner_Dijkstra(uint8_t start_node,
                                  uint8_t target_node)
{
    uint16_t dist[NODE_NUM + 1];
    uint8_t visited[NODE_NUM + 1];
    uint8_t prev[NODE_NUM + 1];
    uint8_t center_count[NODE_NUM + 1];

    uint8_t reverse_path[PATH_MAX_LEN];

    uint8_t i;
    uint8_t j;
    uint8_t u;

    uint8_t reverse_len = 0;

    uint16_t min_dist;
    uint16_t new_dist;
    uint8_t min_center_count;
    uint8_t new_center_count;

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
    // 初始化Dijkstra数组
    //------------------------------------------------
    for(i = 1; i <= NODE_NUM; i++)
    {
        dist[i] = INF_COST;
        visited[i] = 0;
        prev[i] = 0;
        center_count[i] = 0U;
    }

    dist[start_node] = 0;


    //------------------------------------------------
    // 主循环
    //------------------------------------------------
    for(i = 1; i <= NODE_NUM; i++)
    {
        min_dist = INF_COST;
        min_center_count = 0U;
        u = 0;


        //------------------------------------------------
        // 找当前距离最小且未访问节点
        //------------------------------------------------
        for(j = 1; j <= NODE_NUM; j++)
        {
            if(!visited[j] &&
               ((dist[j] < min_dist) ||
                ((dist[j] == min_dist) &&
                 (center_count[j] > min_center_count))))
            {
                min_dist = dist[j];
                min_center_count = center_count[j];
                u = j;
            }
        }


        //------------------------------------------------
        // 已经没有可达节点
        //------------------------------------------------
        if(u == 0)
        {
            break;
        }


        //------------------------------------------------
        // 已找到目标
        //------------------------------------------------
        if(u == target_node)
        {
            break;
        }


        visited[u] = 1;


        //------------------------------------------------
        // 松弛所有邻居
        //------------------------------------------------
        for(j = 1; j <= NODE_NUM; j++)
        {
            if(!visited[j] &&
               g_graph[u][j] < INF_COST)
            {
                new_dist = dist[u] + g_graph[u][j];

#if ROUTE_CENTER_TIE_BREAK
                new_center_count = (uint8_t)(center_count[u] +
                    ((j == ROUTE_CENTER_NODE) ? 1U : 0U));
#else
                new_center_count = center_count[u];
#endif

                if((new_dist < dist[j]) ||
                   ((new_dist == dist[j]) &&
                    (new_center_count > center_count[j])))
                {
                    dist[j] = new_dist;
                    center_count[j] = new_center_count;
                    prev[j] = u;
                }
            }
        }
    }


    //------------------------------------------------
    // 目标不可达
    //------------------------------------------------
    if(dist[target_node] >= INF_COST)
    {
        return result;
    }


    //------------------------------------------------
    // 从终点逆向恢复路径
    //------------------------------------------------
    u = target_node;

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

        u = prev[u];
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

    result.total_cost = dist[target_node];
    result.valid = 1;

    return result;
}
