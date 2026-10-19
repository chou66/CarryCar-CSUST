# 10.7.2 RobotApp + VOFA 双通道诊断版

## 目的

完整路线不动作时，可以在同一个固件里用 VOFA 验证电机/EMM/CAN，快速判断故障是在正式导航链还是底层驱动链。

## 为什么不能简单地同时开放两个控制器

`RobotApp` 和 `VofaDebug` 各自维护一个 `Chassis` 状态对象，但最终都会向同一组 EMM/CAN 驱动器发命令。
即使肉眼看到小车“没动”，RobotApp 仍可能处于等待 IMU、等待边启动或位置状态机未完全结束的阶段。
因此本版虽然两个 Update 都运行，但默认只允许 RobotApp 控制运动。

## 进入 VOFA 人工接管

确认车辆静止并留出安全空间后，先发送：

```text
AA 55 70 00 00 00 00
```

该命令会：

```text
RobotApp_EStop()
-> robot_app_vofa_override = 1
-> 正式 Chassis 把四轮 STOP 序列发送完成
-> robot_app_vofa_override_ready = 1
-> RobotApp 不再执行自动运动
-> route_test_error = 6
```

发送 `0x70` 后建议等待约 100 ms，再发送运动命令。
如果太快发送，`last_result=15`，等待后重试即可。

### 四轮测试

```text
AA 55 30 01 00 00 00    // 四轮使能
AA 55 31 0A 14 00 00    // 前进 100 RPM, ACC20
AA 55 37 00 00 00 00    // 同步停止
```

### 单电机测试

```text
AA 55 10 01 01 00 00
AA 55 11 01 00 0A 14
AA 55 12 01 00 00 00
```

如果未发送 0x70 就发送运动命令，VOFA `last_result` 返回：

```text
14 = manual override required
```

## HWT101 零偏自校准（命令 0x03）

HWT101 是纯积分单轴陀螺，静止零偏会直接积分成航向漂移。VOFA 0x03
让模块进入“自动获取零偏”模式，静止计时结束后自动退出：

```text
AA 55 03 01 00 00 00    // 开始，默认 20 s
AA 55 03 01 14 00 00    // 开始，显式指定 20 s（P2=0x14）
AA 55 03 00 00 00 00    // 中止并立即退出校准模式
```

执行要求：

- 只允许在任务未启动/已结束，或已发送 0x70 接管后执行；
- 校准期间整车必须静止，不要发任何运动命令；
- 一旦底盘收到运动命令，固件会立即发送 `FF AA 48 00 00` 退出并置
  `imu_debug_zero_bias_cal_state = 3`（因运动中止）；
- 校准完成后状态为 2；串口发送失败为 4；
- 校准会改变模块的零偏估计，**应在 HMI 下发地图、建立比赛世界零点之前完成**。

Watch 变量：

```text
imu_debug_zero_bias_cal_state       // 0空闲 1校准中 2完成 3运动中止 4串口失败
imu_debug_zero_bias_cal_elapsed_ms
imu_debug_zero_bias_cal_total_ms
```

## 如何解释结果

如果接管后 VOFA 能正常驱动四轮：

```text
UART5 / VOFA parser
EMM API
CAN
驱动器
电机/四轮方向
```

这些底层基本可以排除，继续查：

```text
HMI -> HMI_Navigation -> RouteTest -> Navigation -> EdgeMotion -> 正式 Chassis
```

如果接管后 VOFA 也不能运动，则故障已经落到底层。

## 为什么不能在线退出人工接管

VOFA 人工移动后，真实位置可能已不等于 `Navigation.current_node`。继续原路径会造成逻辑位置与物理位置不一致。
因此本版故意不提供恢复命令：要回到完整路线测试，复位 MCU、重新摆车并重新下发 HMI 地图。

## Keil Watch

```c
robot_app_vofa_override
robot_app_vofa_override_ready
robot_route_test_state
robot_route_test_error
```

接管后预期：

```text
robot_app_vofa_override       = 1
robot_app_vofa_override_ready = 1
robot_route_test_state          = 5
robot_route_test_error          = 6
```
