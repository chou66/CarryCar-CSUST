# 10.7.1 Full Route Navigation Test

## 1. 本版目的

本版是“纯导航完整路线”实车测试，不执行二维码识别、K230 对准或机械臂动作。

HMI 下发第一份合法地图配置后，测试流程固定为：

```text
HMI 选择的 start_node
  -> N6 扫码/二维码区
  -> N8 原料区
  -> N2 粗加工区
  -> N4 暂存区
  -> HMI 选择的 return_node
```

这里只固定五个阶段的“必经目标”。每一段都会重新调用 `RobotApp_GotoNode()`，
因此两个必经目标之间经过哪些中间节点仍由当前 `obstacle_mask + Dijkstra`
决定，测试代码没有写死中间路线。

---

## 2. 已确认的场地区域与节点

```text
N7 ---- N8 ---- N9
|       |       |
N4 ---- N5 ---- N6
|       |       |
N1 ---- N2 ---- N3
```

当前比赛场地图对应：

| 区域 | 节点 |
|---|---:|
| 扫码 / 二维码板 | N6 |
| 原料区 | N8 |
| 粗加工区 | N2 |
| 暂存区 | N4 |
| 启停区1 | N9 |
| 启停区2 | N3 |

因此 `Hardware/robot_config.h` 已直接配置：

```c
#define ROUTE_TEST_QR_NODE                  6U
#define ROUTE_TEST_RAW_NODE                 8U
#define ROUTE_TEST_ROUGH_NODE               2U
#define ROUTE_TEST_STORAGE_NODE             4U
```

启停区不写死。起点和最终返回点仍来自 HMI 地图帧中的：

```text
start_node  = N3 或 N9
return_node = N3 或 N9
```

---

## 3. 实际导航流程

每一段都是：

```text
最后确认到达的节点
  -> RobotApp_GotoNode(当前必经目标)
  -> Navigation_PlanTo()
  -> MapGraph + Dijkstra
  -> EdgeMotion 逐边执行
  -> 每条边：平移 -> 停稳 -> yaw 修正 -> EdgeReached()
  -> 到达本阶段目标
  -> 短暂停留
  -> 重新规划下一阶段
```

因此障碍物只改变“这一段具体经过哪些中间节点”，不会改变：

```text
扫码 -> 原料 -> 粗加工 -> 暂存 -> 返回
```

这个测试顺序。

---

## 4. HMI 障碍

HMI 的 `O1~O21` 继续按现有协议映射：

```text
O1~O9   = N1~N9 节点障碍
O10     = N1-N2
O11     = N2-N3
O12     = N4-N5
O13     = N5-N6
O14     = N7-N8
O15     = N8-N9
O16     = N1-N4
O17     = N4-N7
O18     = N2-N5
O19     = N5-N8
O20     = N3-N6
O21     = N6-N9
```

注意：如果把本测试的必经目标节点 N6/N8/N2/N4 本身设置成障碍，
Dijkstra 无法到达该阶段是正常现象，测试会进入规划错误。

本轮建议在车辆出发前把最终障碍地图一次性配置好。
“车辆正处在某条边中间时突然修改障碍”的恢复策略目前仍未实现。

---

## 5. 自动启动

第一份合法 HMI 地图配置完成后：

```text
ROUTE_TEST_START_DELAY_MS = 1500 ms
```

随后自动开始前往 N6。

因此发送最终地图帧以前必须确认：

1. 小车实际位于 HMI 选择的 start_node；
2. 当前测试仍采用“车头朝地图 +Y、车右朝地图 +X”的基线假设；
3. 运动区域已经清空；
4. 障碍配置不会把某个必经目标完全封死。

---

## 6. 功能区停留

每次到达 N6 / N8 / N2 / N4 后只短暂停留：

```text
ROUTE_TEST_STAGE_HOLD_MS = 300 ms
```

不会：

- 真正扫码；
- 夹取原料；
- 执行粗加工动作；
- 执行暂存动作。

这个版本只验证完整导航闭环。

---

## 7. Keil Watch

```c
robot_route_test_state
robot_route_test_stage
robot_route_test_target
robot_route_test_error
```

`robot_route_test_state`：

```text
0 = WAIT_MAP
1 = START_DELAY
2 = NAVIGATING
3 = STAGE_HOLD
4 = DONE
5 = ERROR
```

`robot_route_test_stage`：

```text
0 = QR       -> N6
1 = RAW      -> N8
2 = ROUGH    -> N2
3 = STORAGE  -> N4
4 = RETURN   -> HMI return_node
```

`robot_route_test_error`：

```text
0 = 正常
1 = 功能区节点配置非法
2 = 无路 / 规划失败
3 = Navigation 进入 NO_PATH/ERROR
4 = EdgeMotion ERROR
5 = HMI return_node 非法
6 = VOFA 人工接管（诊断模式，不是导航故障）
```

---

## 8. 本版相对 10.7 的修改范围

只修正完整路线测试的功能区语义与节点配置：

```text
Hardware/robot_config.h
Hardware/robot_app.c
Hardware/robot_app.h
Hardware/hmi_display.h   （仅注释示例：细加工区 -> 暂存区）
FULL_ROUTE_TEST_NOTE.md
```

没有修改：

```text
CAN
EMM
Chassis
ChassisMotion
EdgeMotion
地图节点物理坐标
Dijkstra
HMI 障碍协议
VOFA API
底盘标定参数
```

测试路线现在正式为：

```text
start -> N6 -> N8 -> N2 -> N4 -> return
```
