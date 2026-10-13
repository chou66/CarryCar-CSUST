# HMI / USART2 PA2-PA3 链路诊断

> 当前目标：只验证整条串口屏链路是否通畅。
> 当前测试端口：USART2，PA3 RX / PA2 TX，115200 8N1。

## 1. 物理接线

```text
HMI TX -> STM32 PA3 / USART2_RX
HMI RX <- STM32 PA2 / USART2_TX
HMI GND <-> STM32 GND
```

串口必须 TX/RX 交叉连接。

## 2. CubeMX / HAL 基础链

当前工程已经包含：

```text
MX_USART2_UART_Init()
PA2 -> USART2_TX
PA3 -> USART2_RX
115200 8N1
USART2_IRQn enabled
USART2_IRQHandler()
    -> HAL_UART_IRQHandler(&huart2)
```

应用层在 `AppUart_Init()` 中执行：

```c
HAL_UART_Receive_IT(&huart2, ..., 1);
```

## 3. 第一层 Watch：UART 是否收到原始字节

```c
hmi_usart2_rearm_ok_count
hmi_usart2_rearm_fail_count
hmi_usart2_rx_count
hmi_usart2_last_byte
hmi_usart2_error_count
hmi_usart2_last_error
huart2.RxState
huart2.RxXferCount
```

正常启动后，应首先满足：

```text
hmi_usart2_rearm_ok_count >= 1
hmi_usart2_rearm_fail_count == 0
huart2.RxXferCount == 1
```

点击“比赛显示模式”后屏幕发送：

```text
AA 55 03 01 55 AA
```

若 6 字节全部收到：

```text
hmi_usart2_rx_count       += 6
hmi_usart2_last_byte       = 0xAA
hmi_usart2_rearm_ok_count += 6
```

## 4. 第二层 Watch：协议是否完成解析

只有 `hmi_usart2_rx_count > 0` 后才继续看：

```c
hmi_proto_byte_count
hmi_proto_head1_count
hmi_proto_head2_count
hmi_proto_complete_count
hmi_proto_map_ok_count
hmi_proto_map_reject_count
hmi_proto_last_byte
hmi_proto_last_cmd
hmi_proto_last_reject_reason
```

地图帧格式仍然不变：

```text
AA 55 01 SS RR O1 ... O21 55 AA
```

显示模式帧：

```text
AA 55 03 01 55 AA
AA 55 03 02 55 AA
```

## 5. 判断方法

### `rearm_ok >= 1`，但 `rx_count = 0`

HAL 已经等待数据，但 PA3 / USART2_RX 没有形成完整接收。查：

- HMI TX 是否真的接 PA3
- 是否共地
- 115200 / 8N1 是否一致
- PA3 上是否有 UART 波形

### `rx_count > 0`，但 `complete_count = 0`

USART2 硬件链已经通过，继续查协议组帧。

### `complete_count > 0`，地图仍不启动

UART 与协议均已通过，再看：

```c
g_hmi_map.valid
robot_route_test_state
robot_route_test_target
robot_route_test_error
```

此时才进入 RobotApp / Navigation 层排查。

## 6. USART2 资源提醒

当前 Keil 正式工程中没有编译 `Reference_Original_F4_Modules/mj6000.c`，所以 USART2 可以暂时给 HMI 做测试。

以后若重新接入 GM67/MJ6000，不要让它与 HMI 同时使用 USART2。
