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
