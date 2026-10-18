# PCB 引脚与模块核对

| 模块 | 外设 | 引脚 |
|---|---|---|
| HWT101 | USART1 | PC4 TX / PC5 RX |
| 扫码模块（未启用） | USART2 | PA2 TX / PA3 RX |
| 触摸屏 / HMI | USART3 | PB10 TX / PE15 RX |
| K230 | UART4 | PC10 TX / PC11 RX |
| 无刷/扩展 | UART5 | PC12 TX / PD2 RX |
| CAN 电机 | FDCAN2 | PB13 TX / PB12 RX |
| 抓夹舵机 | TIM3_CH1 | PB4 |
| 三工位圆盘舵机 | TIM3_CH2 | PE3 |
| 蜂鸣器 | GPIO | PE0 |
| SWD | SYS | PA13 / PA14 |

CAN 收发器 MCU 侧必须保持：
PB13/FDCAN2_TX -> CAN_TX -> SN65HVD230 D(pin1)
SN65HVD230 R(pin4) -> CAN_RX -> PB12/FDCAN2_RX

## 机械标定入口

`Hardware/robot_config.h`
- `GRIPPER_OPEN_US`
- `GRIPPER_CLOSE_US`
- `CAROUSEL_TRAVEL_DEG`
- `CAROUSEL_SLOT0_DEG`
- `CAROUSEL_REVERSE`
- `GIMBAL_PULSES_PER_DEG`
- `SLIDE_PULSES_PER_MM`
- `ROUTE_GRID_STEP_M`
- `ROUTE_EDGE_SPEED_MPS`

`Hardware/chassis_config.h`
- `CHASSIS_MOTOR_1_FORWARD_DIR ... 4`
- `CHASSIS_FORWARD_PULSES_PER_M`
- `CHASSIS_STRAFE_PULSES_PER_M`
- `CHASSIS_ROTATE_PULSES_PER_RAD`

## 电机编号

1 左前 FL
2 左后 RL
3 右后 RR
4 右前 FR
5 云台旋转
6 抓夹前后移动
