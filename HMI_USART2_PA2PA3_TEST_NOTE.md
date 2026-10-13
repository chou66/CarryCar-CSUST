# HMI 临时迁移到 USART2（PA2/PA3）测试说明

## 目的

只验证串口屏完整收发链是否通畅，尽量不引入其它模块改动。

## 端口

```text
USART2
PA2 = TX -> HMI RX
PA3 = RX <- HMI TX
115200 8N1
```

## 修改边界

只修改：

- `Hardware/app_uart.c/.h`
- `Hardware/hmi_display.c`
- HMI 串口相关注释和诊断文档

不修改：

- HMI 帧协议
- Navigation / Dijkstra
- EdgeMotion
- Chassis / CAN / EMM
- VOFA
- 地图与标定参数

## 关键链路

```text
HMI TX
 -> PA3
 -> USART2
 -> USART2_IRQHandler
 -> HAL_UART_RxCpltCallback
 -> HMI_Protocol_RxByte
 -> RobotApp
```

```text
RobotApp/HMI_Display
 -> HAL_UART_Transmit(&huart2)
 -> PA2
 -> HMI RX
```

## 诊断变量

```c
hmi_usart2_rearm_ok_count
hmi_usart2_rearm_fail_count
hmi_usart2_rx_count
hmi_usart2_last_byte
hmi_usart2_error_count
hmi_usart2_last_error
```

## 资源冲突说明

`Reference_Original_F4_Modules/mj6000.c` 中历史代码使用 USART2，但该文件不在当前 Keil 工程编译列表中。

当前测试版 USART2 由 HMI 独占；后续若启用 GM67/MJ6000，需要重新分配串口。
