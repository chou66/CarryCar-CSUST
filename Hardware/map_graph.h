#ifndef __MAP_GRAPH_H
#define __MAP_GRAPH_H

#include <stdint.h>

#define NODE_NUM        9
#define INF_COST        0x3FFF

//====================================================
// 节点定义
// 节点编号使用 1~9
//====================================================
typedef enum
{
    MAP_TASK_NONE = 0,
    MAP_TASK_PARK,
    MAP_TASK_QR,
    MAP_TASK_RAW,
    MAP_TASK_ROUGH,
    MAP_TASK_STORAGE
} MapTaskId_t;

typedef struct
{
    float x;
    float y;

    /* 固定功能区的标准作业属性。
     * task_yaw_deg 使用世界坐标航向：0=上，+90=右，-90=左，180=下。
     * 注意：仅当该节点是 Mission 当前真正目标时才执行此作业航向。
     * 普通行驶航向不存放在节点中，而由 edge_turn 根据当前拓扑边的
     * from/to 坐标实时计算；普通节点因此无需配置航向参数。
     */
    uint8_t task_id;
    uint8_t task_yaw_valid;
    float task_yaw_deg;

} MapNode_t;


//====================================================
// 地图数据
//====================================================

// 节点数组
extern MapNode_t g_nodes[NODE_NUM + 1];

/* N3/N9 are road intersections. Parking bays have separate physical poses;
 * HMI and Dijkstra still use only the original 1..9 road-node protocol. */
uint8_t MapGraph_GetParkingPose(uint8_t corner_node, float *x, float *y);

// 邻接矩阵
// 下标仍然使用 1~9
extern uint16_t g_graph[NODE_NUM + 1][NODE_NUM + 1];


//====================================================
// 对外函数
//====================================================

// 根据障碍掩码重新建立地图
void MapGraph_Build(uint32_t obstacle_mask);

// 判断某条边是否连通
uint8_t MapGraph_IsConnected(uint8_t node_a, uint8_t node_b);

// 获取边代价
uint16_t MapGraph_GetCost(uint8_t node_a, uint8_t node_b);

#endif
