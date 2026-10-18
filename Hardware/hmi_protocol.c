#include "hmi_protocol.h"
#include "main.h"
#include <string.h>


//====================================================
// HMI协议层
//
// 负责解析：
//
// 1. 地图配置
//    AA 55 01 SS RR O1 ... O21 55 AA
//
// 2. 暂停 / 继续
//    AA 55 02 CC 55 AA
//
// 3. 显示模式
//    AA 55 03 MODE 55 AA
//====================================================


//====================================================
// 全局状态
//====================================================

// 当前最新地图配置
HMI_MapConfig_t g_hmi_map;

// 暂停 / 继续请求
HMI_Control_t g_hmi_ctrl;

// 当前显示模式
HMI_ModeState_t g_hmi_mode;


//====================================================
// 10.7.3 HMI协议层诊断
//
// 与 g_hmi_map 分开：
// - app_uart.c 的 hmi_usart3_* 判断“PE15 / USART3是否收到原始字节”；
// - 这里的 hmi_proto_* 判断“原始字节是否组成了合法协议帧”。
//====================================================

volatile uint32_t hmi_proto_byte_count = 0U;
volatile uint32_t hmi_proto_head1_count = 0U;
volatile uint32_t hmi_proto_head2_count = 0U;
volatile uint32_t hmi_proto_complete_count = 0U;
volatile uint32_t hmi_proto_map_ok_count = 0U;
volatile uint32_t hmi_proto_map_reject_count = 0U;
volatile uint32_t hmi_proto_unknown_cmd_count = 0U;

volatile uint8_t hmi_proto_last_byte = 0U;
volatile uint8_t hmi_proto_last_cmd = 0U;
volatile uint8_t hmi_proto_last_reject_reason = 0U;
volatile uint8_t hmi_proto_debug_rx_index = 0U;
volatile uint8_t hmi_proto_debug_expected_len = 0U;



//====================================================
// 协议接收缓冲区
//====================================================

// 临时存放当前正在接收的数据帧
//
// 这里只是“接收缓存”，
// 一帧解析完成后会被下一帧重复使用。
static uint8_t hmi_frame[HMI_MAX_FRAME_LEN];

// 当前已经接收到的字节数量
static uint8_t hmi_rx_index = 0;

// 当前帧预计总长度
static uint8_t hmi_expected_len = 0;


//====================================================
// 内部函数声明
//====================================================

// 重置协议接收状态
static void HMI_ResetReceiver(void);


// 地图配置
static uint8_t HMI_CheckMapFrame(const uint8_t *frame);
static void HMI_ParseMapFrame(const uint8_t *frame);


// 暂停 / 继续
static uint8_t HMI_CheckControlFrame(const uint8_t *frame);
static void HMI_ParseControlFrame(const uint8_t *frame);


// 显示模式
static uint8_t HMI_CheckModeFrame(const uint8_t *frame);
static void HMI_ParseModeFrame(const uint8_t *frame);


//====================================================
// HMI_Protocol_Init
// HMI协议初始化
//====================================================
void HMI_Protocol_Init(void)
{
    /* 协议诊断计数器随协议初始化一起清零。 */
    hmi_proto_byte_count = 0U;
    hmi_proto_head1_count = 0U;
    hmi_proto_head2_count = 0U;
    hmi_proto_complete_count = 0U;
    hmi_proto_map_ok_count = 0U;
    hmi_proto_map_reject_count = 0U;
    hmi_proto_unknown_cmd_count = 0U;
    hmi_proto_last_byte = 0U;
    hmi_proto_last_cmd = 0U;
    hmi_proto_last_reject_reason = 0U;
    hmi_proto_debug_rx_index = 0U;
    hmi_proto_debug_expected_len = 0U;

    // 清空地图状态
    memset(&g_hmi_map, 0, sizeof(g_hmi_map));

    // 清空暂停 / 继续状态
    memset(&g_hmi_ctrl, 0, sizeof(g_hmi_ctrl));

    // 清空显示模式状态
    memset(&g_hmi_mode, 0, sizeof(g_hmi_mode));

    // 清空接收缓冲区
    memset(hmi_frame, 0, sizeof(hmi_frame));

    // 复位协议解析状态机
    HMI_ResetReceiver();
}


//====================================================
// HMI_ResetReceiver
// 重置接收状态机
//====================================================
static void HMI_ResetReceiver(void)
{
    hmi_rx_index = 0;
    hmi_expected_len = 0;

    hmi_proto_debug_rx_index = 0U;
    hmi_proto_debug_expected_len = 0U;
}


//====================================================
// HMI_Protocol_RxByte
//
// 每收到1个来自HMI的串口字节，就调用一次。
//====================================================
void HMI_Protocol_RxByte(uint8_t data)
{
    hmi_proto_byte_count++;
    hmi_proto_last_byte = data;

    //------------------------------------------------
    // 状态0：
    // 等待第一个帧头 0xAA
    //------------------------------------------------
    if(hmi_rx_index == 0)
    {
        if(data == HMI_HEAD_1)
        {
            hmi_proto_head1_count++;
            hmi_frame[0] = data;
            hmi_rx_index = 1;
            hmi_proto_debug_rx_index = hmi_rx_index;
        }

        return;
    }


    //------------------------------------------------
    // 状态1：
    // 等待第二个帧头 0x55
    //------------------------------------------------
    if(hmi_rx_index == 1)
    {
        if(data == HMI_HEAD_2)
        {
            hmi_proto_head2_count++;
            hmi_frame[1] = data;
            hmi_rx_index = 2;
            hmi_proto_debug_rx_index = hmi_rx_index;
        }
        else if(data == HMI_HEAD_1)
        {
            /*
             * 如果收到：
             *
             * AA AA 55 ...
             *
             * 那么第二个AA仍然可能是新帧的起点。
             */
            hmi_frame[0] = HMI_HEAD_1;
            hmi_rx_index = 1;
        }
        else
        {
            // 帧头错误
            HMI_ResetReceiver();
        }

        return;
    }


    //------------------------------------------------
    // 状态2：
    // 第三个字节为CMD
    //------------------------------------------------
    if(hmi_rx_index == 2)
    {
        hmi_frame[2] = data;
        hmi_rx_index = 3;
        hmi_proto_last_cmd = data;
        hmi_proto_debug_rx_index = hmi_rx_index;


        //------------------------------------------------
        // 地图配置
        //------------------------------------------------
        if(data == HMI_CMD_MAP_CONFIG)
        {
            hmi_expected_len = HMI_MAP_FRAME_LEN;
        }

        //------------------------------------------------
        // 暂停 / 继续
        //------------------------------------------------
        else if(data == HMI_CMD_RUN_CONTROL)
        {
            hmi_expected_len = HMI_CTRL_FRAME_LEN;
        }

        //------------------------------------------------
        // 显示模式
        //------------------------------------------------
        else if(data == HMI_CMD_DISPLAY_MODE)
        {
            hmi_expected_len = HMI_MODE_FRAME_LEN;
        }

        //------------------------------------------------
        // 未知命令
        //------------------------------------------------
        else
        {
            hmi_proto_unknown_cmd_count++;
            hmi_proto_last_reject_reason = 8U;
            HMI_ResetReceiver();
        }

        hmi_proto_debug_expected_len = hmi_expected_len;
        return;
    }


    //------------------------------------------------
    // 接收剩余数据
    //------------------------------------------------

    // 防止数组越界
    if(hmi_rx_index >= HMI_MAX_FRAME_LEN)
    {
        hmi_proto_last_reject_reason = 7U;
        HMI_ResetReceiver();
        return;
    }

    hmi_frame[hmi_rx_index] = data;
    hmi_rx_index++;
    hmi_proto_debug_rx_index = hmi_rx_index;
    hmi_proto_debug_expected_len = hmi_expected_len;


    //------------------------------------------------
    // 判断当前帧是否收满
    //------------------------------------------------
    if((hmi_expected_len != 0) &&
       (hmi_rx_index >= hmi_expected_len))
    {
        //--------------------------------------------
        // CMD 0x01：地图配置
        //--------------------------------------------
        if(hmi_frame[2] == HMI_CMD_MAP_CONFIG)
        {
            hmi_proto_complete_count++;

            if(HMI_CheckMapFrame(hmi_frame))
            {
                hmi_proto_map_ok_count++;
                hmi_proto_last_reject_reason = 0U;
                HMI_ParseMapFrame(hmi_frame);
            }
            else
            {
                hmi_proto_map_reject_count++;
            }
        }

        //--------------------------------------------
        // CMD 0x02：暂停 / 继续
        //--------------------------------------------
        else if(hmi_frame[2] == HMI_CMD_RUN_CONTROL)
        {
            if(HMI_CheckControlFrame(hmi_frame))
            {
                HMI_ParseControlFrame(hmi_frame);
            }
        }

        //--------------------------------------------
        // CMD 0x03：显示模式
        //--------------------------------------------
        else if(hmi_frame[2] == HMI_CMD_DISPLAY_MODE)
        {
            if(HMI_CheckModeFrame(hmi_frame))
            {
                HMI_ParseModeFrame(hmi_frame);
            }
        }


        //--------------------------------------------
        // 当前数据帧处理结束
        //--------------------------------------------
        HMI_ResetReceiver();
    }
}


//====================================================
// HMI_CheckMapFrame
//
// 检查地图配置帧是否合法。
//====================================================
static uint8_t HMI_CheckMapFrame(const uint8_t *frame)
{
    uint8_t i;


    //------------------------------------------------
    // 帧头
    //------------------------------------------------
    if(frame[0] != HMI_HEAD_1 ||
       frame[1] != HMI_HEAD_2)
    {
        hmi_proto_last_reject_reason = 1U;
        return 0;
    }


    //------------------------------------------------
    // CMD
    //------------------------------------------------
    if(frame[2] != HMI_CMD_MAP_CONFIG)
    {
        hmi_proto_last_reject_reason = 2U;
        return 0;
    }


    //------------------------------------------------
    // 帧尾
    //------------------------------------------------
    if(frame[26] != HMI_TAIL_1 ||
       frame[27] != HMI_TAIL_2)
    {
        hmi_proto_last_reject_reason = 3U;
        return 0;
    }


    //------------------------------------------------
    // 出发节点合法性
    //
    // 当前：
    // Node9 = 启停区1
    // Node3 = 启停区2
    //------------------------------------------------
    if((frame[3] != 3) &&
       (frame[3] != 9))
    {
        hmi_proto_last_reject_reason = 4U;
        return 0;
    }


    //------------------------------------------------
    // 返回节点合法性
    //------------------------------------------------
    if((frame[4] != 3) &&
       (frame[4] != 9))
    {
        hmi_proto_last_reject_reason = 5U;
        return 0;
    }


    //------------------------------------------------
    // 障碍状态只能是0或1
    //------------------------------------------------
    for(i = 0; i < HMI_OBSTACLE_NUM; i++)
    {
        if(frame[5 + i] > 1)
        {
            hmi_proto_last_reject_reason = 6U;
            return 0;
        }
    }


    return 1;
}


//====================================================
// HMI_ParseMapFrame
//
// 解析地图配置。
//
// 重要：
// 不直接一项一项修改 g_hmi_map。
//
// 而是：
//
// 原始帧
//   ↓
// new_map 临时配置
//   ↓
// 完整解析
//   ↓
// 一次性替换 g_hmi_map
//
// 这样避免正式配置处于“更新了一半”的状态。
//====================================================
static void HMI_ParseMapFrame(const uint8_t *frame)
{
    uint8_t i;

    uint32_t mask = 0;

    //------------------------------------------------
    // 临时地图配置
    //------------------------------------------------
    HMI_MapConfig_t new_map;


    //------------------------------------------------
    // 先把临时结构体清零
    //------------------------------------------------
    memset(&new_map, 0, sizeof(new_map));


    //------------------------------------------------
    // 解析起点和返回点
    //------------------------------------------------
    new_map.start_node = frame[3];
    new_map.return_node = frame[4];


    //------------------------------------------------
    // 解析21个障碍状态
    //------------------------------------------------
    for(i = 0; i < HMI_OBSTACLE_NUM; i++)
    {
        new_map.obstacle[i] = frame[5 + i];


        //------------------------------------------------
        // 同时生成 obstacle_mask
        //
        // obstacle[0]  -> bit0
        // ...
        // obstacle[20] -> bit20
        //------------------------------------------------
        if(frame[5 + i] != 0)
        {
            mask |= (1UL << i);
        }
    }


    //------------------------------------------------
    // 保存障碍掩码
    //------------------------------------------------
    new_map.obstacle_mask = mask;


    //------------------------------------------------
    // 这是一份合法配置
    //------------------------------------------------
    new_map.valid = 1;


    //------------------------------------------------
    // 告诉主程序有新配置
    //------------------------------------------------
    new_map.updated = 1;


    //------------------------------------------------
    // 最后一步：
    // 一次性替换正式地图配置
    //
    // 旧配置被完整覆盖，
    // 不会保留上一帧的旧障碍状态。
    //------------------------------------------------
    g_hmi_map = new_map;
}


//====================================================
// HMI_CheckControlFrame
//
// AA 55 02 CC 55 AA
//
// CC：
// 01 = 暂停
// 02 = 继续
//====================================================
static uint8_t HMI_CheckControlFrame(const uint8_t *frame)
{
    //------------------------------------------------
    // 帧头
    //------------------------------------------------
    if(frame[0] != HMI_HEAD_1 ||
       frame[1] != HMI_HEAD_2)
    {
        return 0;
    }


    //------------------------------------------------
    // CMD
    //------------------------------------------------
    if(frame[2] != HMI_CMD_RUN_CONTROL)
    {
        return 0;
    }


    //------------------------------------------------
    // 帧尾
    //------------------------------------------------
    if(frame[4] != HMI_TAIL_1 ||
       frame[5] != HMI_TAIL_2)
    {
        return 0;
    }


    //------------------------------------------------
    // 控制命令
    //------------------------------------------------
    if((frame[3] != HMI_CTRL_PAUSE) &&
       (frame[3] != HMI_CTRL_RESUME))
    {
        return 0;
    }


    return 1;
}


//====================================================
// HMI_ParseControlFrame
//
// 解析暂停 / 继续请求。
//====================================================
static void HMI_ParseControlFrame(const uint8_t *frame)
{
    //------------------------------------------------
    // 暂停
    //------------------------------------------------
    if(frame[3] == HMI_CTRL_PAUSE)
    {
        g_hmi_ctrl.pause_request = 1;

        // 暂停与继续互斥
        g_hmi_ctrl.resume_request = 0;
    }

    //------------------------------------------------
    // 继续
    //------------------------------------------------
    else if(frame[3] == HMI_CTRL_RESUME)
    {
        g_hmi_ctrl.resume_request = 1;

        // 暂停与继续互斥
        g_hmi_ctrl.pause_request = 0;
    }
}


//====================================================
// HMI_CheckModeFrame
//
// AA 55 03 MODE 55 AA
//
// MODE：
// 01 = 比赛模式
// 02 = 调试模式
//====================================================
static uint8_t HMI_CheckModeFrame(const uint8_t *frame)
{
    //------------------------------------------------
    // 帧头
    //------------------------------------------------
    if(frame[0] != HMI_HEAD_1 ||
       frame[1] != HMI_HEAD_2)
    {
        return 0;
    }


    //------------------------------------------------
    // CMD
    //------------------------------------------------
    if(frame[2] != HMI_CMD_DISPLAY_MODE)
    {
        return 0;
    }


    //------------------------------------------------
    // 帧尾
    //------------------------------------------------
    if(frame[4] != HMI_TAIL_1 ||
       frame[5] != HMI_TAIL_2)
    {
        return 0;
    }


    //------------------------------------------------
    // 模式合法性
    //------------------------------------------------
    if((frame[3] != HMI_DISPLAY_RACE) &&
       (frame[3] != HMI_DISPLAY_DEBUG))
    {
        return 0;
    }


    return 1;
}


//====================================================
// HMI_ParseModeFrame
//
// 解析显示模式。
//====================================================
static void HMI_ParseModeFrame(const uint8_t *frame)
{
    //------------------------------------------------
    // 比赛显示模式
    //------------------------------------------------
    if(frame[3] == HMI_DISPLAY_RACE)
    {
        g_hmi_mode.mode = HMI_MODE_RACE;
    }

    //------------------------------------------------
    // 调试显示模式
    //------------------------------------------------
    else if(frame[3] == HMI_DISPLAY_DEBUG)
    {
        g_hmi_mode.mode = HMI_MODE_DEBUG;
    }


    //------------------------------------------------
    // 标记模式发生改变
    //------------------------------------------------
    g_hmi_mode.updated = 1;
}


//====================================================
// HMI_MapConfig_IsValid
//====================================================
uint8_t HMI_MapConfig_IsValid(void)
{
    return g_hmi_map.valid;
}


//====================================================
// HMI_MapConfig_IsUpdated
//====================================================
uint8_t HMI_MapConfig_IsUpdated(void)
{
    return g_hmi_map.updated;
}


//====================================================
// HMI_MapConfig_ClearUpdated
//====================================================
void HMI_MapConfig_ClearUpdated(void)
{
    g_hmi_map.updated = 0;
}


//====================================================
// HMI_MapConfig_TakeUpdatedSnapshot
//
// 原子取得地图配置副本。
// 同时清除 updated 标志。
//
// 推荐主循环以后使用这个函数。
//
// 返回：
// 0 = 没有新地图
// 1 = 成功取得新地图
//====================================================
uint8_t HMI_MapConfig_TakeUpdatedSnapshot(HMI_MapConfig_t *out)
{
    uint8_t has_update = 0;

    uint32_t primask;


    //------------------------------------------------
    // 空指针保护
    //------------------------------------------------
    if(out == NULL)
    {
        return 0;
    }


    //------------------------------------------------
    // 保存进入临界区前的中断状态
    //
    // PRIMASK = 0
    // 表示进入前中断是开启的。
    //
    // PRIMASK = 1
    // 表示进入前中断本身已经关闭。
    //------------------------------------------------
    primask = __get_PRIMASK();


    //------------------------------------------------
    // 暂时禁止可屏蔽中断
    //
    // 防止HMI UART中断刚好在结构体复制过程中
    // 更新 g_hmi_map。
    //------------------------------------------------
    __disable_irq();


    //------------------------------------------------
    // 只在有新配置时复制
    //------------------------------------------------
    if(g_hmi_map.updated)
    {
        //--------------------------------------------
        // 快速复制当前完整配置
        //--------------------------------------------
        *out = g_hmi_map;

        //--------------------------------------------
        // 当前这份新配置已经被主程序取走
        //--------------------------------------------
        g_hmi_map.updated = 0;

        has_update = 1;
    }


    //------------------------------------------------
    // 恢复进入临界区之前的中断状态
    //
    // 如果进入前中断本来就是开启的，
    // 才重新开启。
    //------------------------------------------------
    if(primask == 0U)
    {
        __enable_irq();
    }


    return has_update;
}


//====================================================
// HMI_Control_GetPauseRequest
//====================================================
uint8_t HMI_Control_GetPauseRequest(void)
{
    return g_hmi_ctrl.pause_request;
}


//====================================================
// HMI_Control_GetResumeRequest
//====================================================
uint8_t HMI_Control_GetResumeRequest(void)
{
    return g_hmi_ctrl.resume_request;
}


//====================================================
// HMI_Control_ClearPauseRequest
//====================================================
void HMI_Control_ClearPauseRequest(void)
{
    g_hmi_ctrl.pause_request = 0;
}


//====================================================
// HMI_Control_ClearResumeRequest
//====================================================
void HMI_Control_ClearResumeRequest(void)
{
    g_hmi_ctrl.resume_request = 0;
}


//====================================================
// HMI_Mode_Get
//====================================================
HMI_DisplayMode_t HMI_Mode_Get(void)
{
    return g_hmi_mode.mode;
}


//====================================================
// HMI_Mode_IsUpdated
//====================================================
uint8_t HMI_Mode_IsUpdated(void)
{
    return g_hmi_mode.updated;
}


//====================================================
// HMI_Mode_ClearUpdated
//====================================================
void HMI_Mode_ClearUpdated(void)
{
    g_hmi_mode.updated = 0;
}
