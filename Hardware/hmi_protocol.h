#ifndef __HMI_PROTOCOL_H
#define __HMI_PROTOCOL_H

#include <stdint.h>


//====================================================
// 通用协议定义
//====================================================

#define HMI_HEAD_1              0xAA
#define HMI_HEAD_2              0x55

#define HMI_TAIL_1              0x55
#define HMI_TAIL_2              0xAA


//====================================================
// 主命令类型
//====================================================

// 地图配置
#define HMI_CMD_MAP_CONFIG      0x01

// 暂停 / 继续
#define HMI_CMD_RUN_CONTROL     0x02

// 显示模式选择
#define HMI_CMD_DISPLAY_MODE    0x03


//====================================================
// 各类数据帧长度
//====================================================

// AA 55 01 SS RR O1...O21 55 AA
#define HMI_MAP_FRAME_LEN       28

// AA 55 02 CC 55 AA
#define HMI_CTRL_FRAME_LEN      6

// AA 55 03 MODE 55 AA
#define HMI_MODE_FRAME_LEN      6

// 当前最大帧长度
#define HMI_MAX_FRAME_LEN       28


//====================================================
// CMD = 0x02：运行控制子命令
//====================================================

#define HMI_CTRL_PAUSE          0x01
#define HMI_CTRL_RESUME         0x02


//====================================================
// CMD = 0x03：显示模式子命令
//====================================================

#define HMI_DISPLAY_RACE        0x01
#define HMI_DISPLAY_DEBUG       0x02


//====================================================
// 障碍区域数量
//====================================================

#define HMI_OBSTACLE_NUM        21


//====================================================
// 地图配置结构体
//====================================================
typedef struct
{
    // 出发节点
    // 当前合法值：3 或 9
    uint8_t start_node;

    // 返回节点
    // 当前合法值：3 或 9
    uint8_t return_node;

    // 21个障碍状态
    //
    // obstacle[0]  = z01
    // ...
    // obstacle[20] = z21
    //
    // 0 = 无障碍
    // 1 = 有障碍
    uint8_t obstacle[HMI_OBSTACLE_NUM];

    // 21个障碍压缩后的bit mask
    //
    // bit0  = z01
    // ...
    // bit20 = z21
    uint32_t obstacle_mask;

    // 是否至少收到过一次合法地图配置
    uint8_t valid;

    // 是否有一份新的地图配置尚未被主程序处理
    uint8_t updated;

} HMI_MapConfig_t;


//====================================================
// 暂停 / 继续请求
//====================================================
typedef struct
{
    // 1 = 收到暂停请求
    uint8_t pause_request;

    // 1 = 收到继续请求
    uint8_t resume_request;

} HMI_Control_t;


//====================================================
// HMI显示模式
//====================================================
typedef enum
{
    // 尚未选择模式
    HMI_MODE_NONE = 0,

    // 比赛显示模式
    HMI_MODE_RACE,

    // 调试显示模式
    HMI_MODE_DEBUG

} HMI_DisplayMode_t;


//====================================================
// HMI显示模式状态
//====================================================
typedef struct
{
    // 当前显示模式
    HMI_DisplayMode_t mode;

    // 1 = 显示模式刚刚发生变化
    uint8_t updated;

} HMI_ModeState_t;


//====================================================
// 全局状态
//
// 注意：
// 地图配置推荐通过
// HMI_MapConfig_TakeUpdatedSnapshot()
// 安全读取。
//====================================================

extern HMI_MapConfig_t g_hmi_map;
extern HMI_Control_t g_hmi_ctrl;
extern HMI_ModeState_t g_hmi_mode;


//====================================================
// 协议初始化
//====================================================

/**
 * @brief HMI协议层初始化
 *
 * 系统启动时调用一次。
 */
void HMI_Protocol_Init(void);


//====================================================
// 串口字节输入
//====================================================

/**
 * @brief 将一个收到的串口字节送入HMI协议解析器
 *
 * @param data 收到的1字节数据
 *
 * UART每收到1个字节调用一次。
 */
void HMI_Protocol_RxByte(uint8_t data);


//====================================================
// 地图配置接口
//====================================================

/**
 * @brief 查询是否曾经收到合法地图配置
 *
 * @return
 * 0 = 尚未收到
 * 1 = 已经收到
 */
uint8_t HMI_MapConfig_IsValid(void);


/**
 * @brief 查询是否有新的地图配置尚未处理
 */
uint8_t HMI_MapConfig_IsUpdated(void);


/**
 * @brief 清除地图配置更新标志
 *
 * 如果使用 HMI_MapConfig_TakeUpdatedSnapshot()，
 * 一般不需要单独调用本函数。
 */
void HMI_MapConfig_ClearUpdated(void);


/**
 * @brief 原子取得最新地图配置并清除updated标志
 *
 * @param out
 * 用于保存地图配置副本的结构体地址
 *
 * @return
 * 0 = 当前没有新配置
 * 1 = 已成功复制一份新配置
 *
 * 推荐主循环使用这个函数读取地图配置。
 *
 * 示例：
 *
 * HMI_MapConfig_t map;
 *
 * if(HMI_MapConfig_TakeUpdatedSnapshot(&map))
 * {
 *     MapGraph_Build(map.obstacle_mask);
 * }
 */
uint8_t HMI_MapConfig_TakeUpdatedSnapshot(HMI_MapConfig_t *out);


//====================================================
// 暂停 / 继续接口
//====================================================

/**
 * @brief 查询是否收到暂停请求
 */
uint8_t HMI_Control_GetPauseRequest(void);


/**
 * @brief 查询是否收到继续请求
 */
uint8_t HMI_Control_GetResumeRequest(void);


/**
 * @brief 清除暂停请求
 */
void HMI_Control_ClearPauseRequest(void);


/**
 * @brief 清除继续请求
 */
void HMI_Control_ClearResumeRequest(void);


//====================================================
// 显示模式接口
//====================================================

/**
 * @brief 获取当前HMI显示模式
 */
HMI_DisplayMode_t HMI_Mode_Get(void);


/**
 * @brief 查询显示模式是否刚刚改变
 */
uint8_t HMI_Mode_IsUpdated(void);


/**
 * @brief 清除显示模式更新标志
 */
void HMI_Mode_ClearUpdated(void);



//====================================================
// 10.7.3 HMI协议诊断变量
//
// 只用于 Keil Watch，不参与控制逻辑。
//====================================================

extern volatile uint32_t hmi_proto_byte_count;
extern volatile uint32_t hmi_proto_head1_count;
extern volatile uint32_t hmi_proto_head2_count;
extern volatile uint32_t hmi_proto_complete_count;
extern volatile uint32_t hmi_proto_map_ok_count;
extern volatile uint32_t hmi_proto_map_reject_count;
extern volatile uint32_t hmi_proto_unknown_cmd_count;

extern volatile uint8_t hmi_proto_last_byte;
extern volatile uint8_t hmi_proto_last_cmd;
extern volatile uint8_t hmi_proto_last_reject_reason;
extern volatile uint8_t hmi_proto_debug_rx_index;
extern volatile uint8_t hmi_proto_debug_expected_len;

/*
 * hmi_proto_last_reject_reason:
 *   0 = 无拒绝 / 最近地图帧合法
 *   1 = 帧头错误
 *   2 = CMD错误
 *   3 = 帧尾错误
 *   4 = start_node 非 3/9
 *   5 = return_node 非 3/9
 *   6 = 某个 obstacle 字节不是 0/1
 *   7 = 接收缓存越界
 *   8 = 未知 CMD
 */

#endif
