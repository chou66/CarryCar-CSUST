#ifndef __HMI_DISPLAY_H
#define __HMI_DISPLAY_H

#include <stdint.h>


//====================================================
// HMI Display 驱动
//
// 功能：
// 负责 G491 -> 淘晶驰串口屏 的显示更新。
//
// 本文件不负责解析屏幕发来的数据。
// HMI -> G491 数据解析由 hmi_protocol.c 负责。
//====================================================


//====================================================
// 初始化类
//====================================================

/**
 * @brief HMI_Display_Init
 *        HMI显示驱动初始化
 *
 * 功能：
 * 初始化显示驱动内部的定时器、缓存等状态。
 *
 * 建议：
 * 系统启动时调用一次。
 */
void HMI_Display_Init(void);


//====================================================
// 页面切换类
//====================================================

/**
 * @brief HMI_Page_Config
 *        切换到赛前配置页面 page0
 *
 * 用途：
 * 回到启停区 / 障碍物配置总览页面。
 */
void HMI_Page_Config(void);


/**
 * @brief HMI_Page_ModeSelect
 *        切换到显示模式选择页面 page3
 *
 * 用途：
 * 让用户选择“比赛显示”或“调试显示”。
 */
void HMI_Page_ModeSelect(void);


/**
 * @brief HMI_Page_Race
 *        切换到比赛显示页面 page4
 *
 * 用途：
 * 显示大字任务码以及比赛任务信息。
 */
void HMI_Page_Race(void);


/**
 * @brief HMI_Page_Debug
 *        切换到调试显示页面 page5
 *
 * 用途：
 * 显示Yaw、节点、路径、速度等调试数据。
 */
void HMI_Page_Debug(void);


//====================================================
// page4：比赛显示接口
//====================================================

/**
 * @brief HMI_Race_SetTaskCode
 *        设置比赛页面的大字任务码
 *
 * @param code
 * 扫码模块收到的任务码字符串，例如：
 * "156+123+516+231"
 *
 * RobotApp在N6扫码区收到一个新的候选码后会立即调用本接口，因此格式
 * 错误的原始内容也能显示出来，方便判断串口错位或二维码印刷问题。
 * 只有TaskCode_Parse()校验通过后，该任务码才会真正锁定并驱动搬运顺序。
 */
void HMI_Race_SetTaskCode(const char *code);


/**
 * @brief HMI_Race_SetStage
 *        设置当前比赛阶段
 *
 * @param stage
 * 例如：
 * "二维码区"
 * "原料区"
 * "粗加工区"
 * "暂存区"
 * "返回"
 */
void HMI_Race_SetStage(const char *stage);


/**
 * @brief HMI_Race_SetAction
 *        设置当前执行动作
 *
 * @param action
 * 例如：
 * "前往目标"
 * "夹取"
 * "放置"
 * "识别"
 * "等待"
 */
void HMI_Race_SetAction(const char *action);


/**
 * @brief HMI_Race_SetColor
 *        设置当前操作物料颜色
 *
 * @param color
 * 例如：
 * "红色"
 * "绿色"
 * "蓝色"
 */
void HMI_Race_SetColor(const char *color);


/**
 * @brief HMI_Race_SetResult
 *        设置当前任务执行结果
 *
 * @param result
 * 例如：
 * "执行中"
 * "成功"
 * "失败"
 */
void HMI_Race_SetResult(const char *result);


//====================================================
// page5：调试显示接口
//====================================================

/**
 * @brief HMI_Debug_SetTaskCode
 *        设置调试页面任务码
 *
 * @param code
 * 当前任务码字符串，格式与比赛页面完全相同，固定为15个ASCII字符。
 *
 * 与比赛页不同：
 * 此处使用普通小字体，仅用于调试观察；不参与任何状态机判断。
 */
void HMI_Debug_SetTaskCode(const char *code);


/**
 * @brief HMI_Debug_SetTargetYaw
 *        设置导航目标Yaw角
 *
 * @param yaw_deg
 * 地图坐标系下的目标航向角，单位：度。
 */
void HMI_Debug_SetTargetYaw(float yaw_deg);


/**
 * @brief HMI_Debug_SetCurrentYaw
 *        设置小车当前Yaw角
 *
 * @param yaw_deg
 * 经过IMU安装方向、零点偏置校正后的
 * 地图坐标系Yaw角，单位：度。
 */
void HMI_Debug_SetCurrentYaw(float yaw_deg);


/**
 * @brief HMI_Debug_SetCurrentNode
 *        设置当前节点编号
 *
 * @param node
 * 节点编号，范围 1~9。
 */
void HMI_Debug_SetCurrentNode(uint8_t node);


/**
 * @brief HMI_Debug_SetTargetNode
 *        设置当前导航目标节点
 *
 * @param node
 * 节点编号，范围 1~9。
 */
void HMI_Debug_SetTargetNode(uint8_t node);


/**
 * @brief HMI_Debug_SetPath
 *        显示当前路径规划结果
 *
 * @param path
 * 路径节点数组，例如：
 * {3, 6, 9, 8}
 *
 * @param length
 * 路径数组有效长度。
 *
 * 屏幕显示效果：
 * 3>6>9>8
 *
 * 建议：
 * 仅在首次规划或重新规划时更新一次，
 * 不需要周期重复发送。
 */
void HMI_Debug_SetPath(const uint8_t *path,
                       uint8_t length);


/**
 * @brief HMI_Debug_SetSpeed
 *        显示小车当前运动速度
 *
 * @param speed
 * 当前速度。
 *
 * 推荐最终统一单位：
 * mm/s
 */
void HMI_Debug_SetSpeed(float speed);


//====================================================
// 周期刷新接口
//====================================================

/**
 * @brief HMI_Display_Periodic
 *        周期刷新调试页实时数据
 *
 * @param now_ms
 * 当前系统运行时间，单位ms。
 * 一般传入 HAL_GetTick()。
 *
 * @param current_yaw
 * 当前地图坐标系Yaw角。
 *
 * @param target_yaw
 * 当前导航目标Yaw角。
 *
 * @param current_speed
 * 当前小车速度。
 *
 * 功能：
 * 内部按固定周期刷新：
 *   - 当前Yaw
 *   - 目标Yaw
 *   - 当前速度
 *
 * 默认不需要每次主循环都真正发送串口。
 * 函数内部自行限频。
 */
void HMI_Display_Periodic(uint32_t now_ms,
                          float current_yaw,
                          float target_yaw,
                          float current_speed);


//====================================================
// 底层发送接口
//====================================================

/**
 * @brief HMI_SendCommand
 *        向淘晶驰屏发送一条HMI指令
 *
 * @param cmd
 * 不带 FF FF FF 结束符的指令字符串。
 *
 * 示例：
 * HMI_SendCommand(
 *     "page5.tCurrentYaw.txt=\"90.0\""
 * );
 *
 * 本函数会自动在命令尾部追加：
 * FF FF FF
 *
 * 一般业务代码不要直接频繁调用，
 * 优先使用上面封装好的显示接口。
 */
void HMI_SendCommand(const char *cmd);


#endif
