# 10.4 -> 正式 EdgeMotion 合并说明

## 1. 本次合并边界

基线：`CarryCar-VoFa_debug_10.4`

本次只处理正式导航运动层，不改已经验证过的底层：

- 不改 `can.c/.h`
- 不改 `emm_v5.c/.h`
- 不改 `emm_adapter.c/.h`
- 不改两套 EMM API 的现状
- 不改 `chassis.c/.h`
- 不改 `chassis_motion.c/.h` 中已有实车标定/补偿
- 不改 `chassis_config.h` 的 12998 / 13400 / 2451 等参数
- 不改 `vofa_debug.c/.h`，保留原 VOFA 导航实车逻辑作为 A/B 回归基线

实际修改：

- `Hardware/edge_motion.c`
- `Hardware/edge_motion.h`
- `Hardware/robot_app.c`

---

## 2. 迁入正式 EdgeMotion 的实车逻辑

10.4 VOFA 导航已经验证的边执行流程：

```text
取当前 Navigation 边
    -> 记录本边起点 HWT101 软件零点
    -> chassis_motion_translate(dx, dy)
    -> 等位置运动完成
    -> 停稳 150 ms
    -> 读取相对 yaw
    -> |yaw| <= 0.5 deg：直接提交 EdgeReached
    -> 否则以约 20 RPM 低速原地反向纠偏
    -> 进入 0.5 deg 死区或穿过 0 deg 后停车
    -> 再停稳 150 ms
    -> EdgeReached
```

正式 `edge_motion.c` 现在实现为：

```text
EDGE_MOTION_IDLE
    -> EDGE_MOTION_TRANSLATING
    -> EDGE_MOTION_SETTLE_AFTER_TRANSLATE
    -> [需要纠偏] EDGE_MOTION_YAW_CORRECTING
    -> EDGE_MOTION_SETTLE_AFTER_YAW
    -> EDGE_MOTION_IDLE + HMI_Navigation_EdgeReached()
```

---

## 3. 航向修正没有依赖 VOFA 专用 EMM API

VOFA 原调试逻辑通过四个 `Emm_V5_Vel_Control()` 直接下发 20 RPM 原地旋转。

正式 EdgeMotion 没有复制这种依赖，而是走正式底盘链：

```text
EdgeMotion
    -> chassis_set_velocity(vx=0, vy=0, wz)
    -> mecanum_inverse()
    -> chassis
    -> emm_vel()
    -> can_SendCmd()
```

根据当前机械参数反算，20 RPM 约等于：

```text
wz = 0.423 rad/s ~= 24.24 deg/s
```

四轮方向与 VOFA 已验证方向一致：

```text
CCW：ID1~ID4 DIR 都为 1
CW ：ID1~ID4 DIR 都为 0
```

航向修正速度命令每 80 ms 刷新一次，原因是 `Chassis` 速度模式有 150 ms 软件看门狗。这样主循环若异常停止刷新，底盘仍会自动停车。

---

## 4. 沿用的 VOFA 实车参数

正式 EdgeMotion 当前直接继承 10.4 VOFA 已验证值：

```text
边运动 acceleration      = 10
Yaw deadband              = 0.50 deg
自动纠偏最大起始误差      = 8.00 deg
Yaw correct wheel RPM     = 20 RPM
Yaw correct acceleration  = 10
平移后/纠偏后停稳         = 150 ms
IMU fresh timeout         = 300 ms
Yaw correction timeout    = 1800 ms
```

本轮不重新调这些数。

---

## 5. 安全行为

- 每条边开始前必须有 300 ms 内的新鲜 HWT101 数据，否则保持停车等待。
- 平移结束后没有新鲜 IMU 数据，不会调用 `EdgeReached()`。
- 纠偏过程中 IMU 掉线：立即停车并进入 `EDGE_MOTION_ERROR`。
- 边末 yaw 初始误差大于 8 deg：拒绝自动纠偏并进入 `EDGE_MOTION_ERROR`。
- 纠偏超过 1800 ms：停车并进入 `EDGE_MOTION_ERROR`。
- 暂停发生在物理边执行中：停车并进入 `EDGE_MOTION_PAUSE_ABORT`，不盲目继续剩余位置。

---

## 6. RobotApp 的同步修改

10.4 原版只在：

```text
EDGE_MOTION_MOVING
```

期间收到地图重规划/无路事件时停车。

正式 EdgeMotion 现在还有：

```text
平移后停稳
Yaw 修正
Yaw 后停稳
```

因此新增 `EdgeMotion_IsActive()`，只要一条真实边尚未 commit，地图重规划/无路都会 `EdgeMotion_EStop()`。

注意：当前设计仍然是“中途障碍 -> 停车锁定”，没有假装车辆仍在上一个节点直接继续新路线。真正的中途定位/退回节点恢复策略应单独设计。

---

## 7. 当前还没有做的事情

本包没有把 `main.c` 从 VOFA 调试入口切换到 `RobotApp_Update()`。

原因是正式 HMI UART 链还需要单独收口，例如：

```text
USART2 RX (PA3)
    -> HMI_Protocol_RxByte()
    -> HMI_Navigation_Process()
```

> 当前链路验证版临时使用 USART2（PA3 RX / PA2 TX）作为 HMI 端口；协议与导航逻辑不变。

当前 `app_uart.c` 主要仍服务 HWT101 + VOFA。

因此本次结果是：

> 正式 EdgeMotion 已具备 VOFA 已验证的运动/航向逻辑；
> 但“显示屏 -> HMI -> Navigation -> EdgeMotion”整条链还没有切到主程序实际运行。

下一阶段应先接通 HMI UART 和 RobotApp 主循环，再做显示屏地图/障碍端到端测试。

---

## 8. 构建验证

本次已用 GCC 对修改后的 `edge_motion.c`、`robot_app.c` 做 C11 语法检查，工程源码本身无新增语法错误。

未在当前环境下运行 Keil ARMCC，也没有生成新的 `.axf/.hex`。

打开 Keil 后必须先 **Rebuild** 再烧录测试。
