# HMI -> RobotApp 正式链路（USART2 PA2/PA3 测试版）

## 当前测试目的

本版本优先验证“串口屏 -> STM32 -> HMI协议 -> RobotApp”整条链路是否通畅。

串口屏当前临时接到 **USART2**：

```text
HMI TX -> PA3 / USART2_RX
HMI RX <- PA2 / USART2_TX
HMI GND <-> STM32 GND
115200 8N1
```

> USART2 在本测试版中由 HMI 独占。`Reference_Original_F4_Modules` 中 MJ6000/GM67 代码未加入当前 Keil 编译；以后若启用 GM67，需要重新分配串口资源。

## 当前完整链路

```text
HMI TX
  -> PA3 / USART2_RX
  -> USART2_IRQHandler()
  -> HAL_UART_IRQHandler(&huart2)
  -> HAL_UART_RxCpltCallback()
  -> HMI_Protocol_RxByte()
  -> HMI_Navigation / RobotApp
```

反向显示链路：

```text
RobotApp / HMI_Display
  -> HMI_SendCommand()
  -> HAL_UART_Transmit(&huart2, ...)
  -> PA2 / USART2_TX
  -> HMI RX
```

本次迁移不修改：

- HMI 二进制协议格式
- `hmi_protocol.c` 组帧/校验逻辑
- Navigation / Dijkstra
- EdgeMotion
- chassis / chassis_motion
- EMM / CAN
- 底盘标定参数

## Watch 变量

```c
hmi_usart2_rearm_ok_count
hmi_usart2_rearm_fail_count
hmi_usart2_rx_count
hmi_usart2_last_byte
hmi_usart2_error_count
hmi_usart2_last_error
```

如果上电运行后：

```text
hmi_usart2_rearm_ok_count >= 1
hmi_usart2_rearm_fail_count == 0
```

说明 `HAL_UART_Receive_IT(&huart2, ..., 1)` 已成功挂起。

点击“比赛显示模式”，屏幕发送：

```text
AA 55 03 01 55 AA
```

理想结果：

```text
hmi_usart2_rx_count       += 6
hmi_usart2_last_byte       = 0xAA
hmi_usart2_rearm_ok_count += 6
```

如果这一步通过，再继续观察 `hmi_proto_complete_count`、`g_hmi_map.valid` 和 RobotApp 状态。
