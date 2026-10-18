# CarryCar G491 Full Integrated V1

这是基于当前 STM32G491 工程整合的第一版完整模块工程。目标不是“上电直接跑比赛”，
而是先把所有已知模块按 PCB 引脚正确接入，并且避免任何上电自动运动。

## 已集成

- FDCAN2 / Emm CAN 电机 1~6
  - ID1 左前
  - ID2 左后
  - ID3 右后
  - ID4 右前
  - ID5 云台旋转
  - ID6 抓夹前后移动
- 麦克纳姆底盘位置/速度控制
- HWT101 陀螺仪
- K230 视觉
- 扫码模块
- 淘晶驰/Nextion 类串口触摸屏
- 9 节点地图、21 个障碍位、Dijkstra 寻路
- 路径边 -> 0.45 m 实际底盘移动
- PB4 抓夹舵机
- PE3 三工位圆盘舵机（0/120/240 度逻辑）
- PE0 蜂鸣器
- UART5 保留给无刷/扩展

## 关键修正

1. FDCAN2 从错误的约 1.328 Mbit/s 改成精确 500 kbit/s：
   170 MHz / 17 / (1+15+4) = 500 kbit/s。
2. main 中补上 fdcan2_UserInit()，真正配置 filter/start/interrupt。
3. FDCAN PB12/PB13 GPIO speed 改为 VERY_HIGH。
4. 串口屏已从 USART2 迁到 USART3：PE15 RX / PB10 TX；HMI 应用层统一使用 huart3。
   （此前"链路验证临时切到 USART2"那一步已经作废，USART2 与应用层解耦，空闲可留给扫码模块。）
5. 新增 UART4 PC10/PC11、UART5 PC12/PD2。
6. 舵机从旧 TIM4/PD12~15 改为 PCB：
   PB4 TIM3_CH1 抓夹、PE3 TIM3_CH2 圆盘。
7. 修复 emm_adapter.c：
   原版会无条件返回 HAL_OK，而且 `emm_all_position_reached_after()` 永远 true，
   会让底盘错误地认为位置运动已经完成。现在使用 CAN 0x3A 状态缓存真实判断。
8. 去掉 can.c / HMI 显示中的 printf/snprintf 依赖，避免再次引入 semihosting。
9. 所有 UART 接收由 app_uart.c 的唯一 HAL 回调统一分发，避免重复 callback。

## 启动行为

上电后：
- 初始化 CAN/UART/PWM/导航；
- 舵机只置于 1500 us 安全中位；
- 不自动启动任何电机；
- 不自动执行路线。

只有调用 `RobotApp_GotoNode(n)` 才会让导航层规划并驱动底盘沿路径移动。

## 必须标定

见 `PINMAP_AND_CALIBRATION.md`。尤其是：
- 四个底盘电机 forward_dir；
- 1 m 前进、横移、1 rad 旋转的脉冲标定；
- 抓夹开/合 PWM；
- 圆盘 slot0 机械零位和实际舵机总行程；
- 电机 5 pulses/deg；
- 电机 6 pulses/mm。

在这些量未标定前，不要直接跑全程比赛任务。

## 动态障碍更新的安全行为

如果车辆正在两个节点之间运动，而屏幕此时改变障碍导致重新规划或无路，
本版会立即停车，不会把“最后确认节点”当成车辆此刻真实位置继续盲走。
重新定位到最近节点后再重新下发导航目标。
