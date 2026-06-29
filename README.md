# NUEDC 2025 E Smart Car Line Tracker

本仓库是 2025 电赛 E 题“小车部分”的 STM32 裸机工程。当前版本只实现小车沿正方形黑线自动循迹、角点计数、圈数统计、达到目标圈数自动停车和 OLED 调试显示；不包含瞄准、云台、激光、视觉识别或电源管理代码。

## 项目目标

- MCU：STM32F103C8T6
- 开发环境：Keil MDK / uVision
- 编译器：ARM Compiler 6.24 / ARMCLANG
- 固件库：STM32F10x Standard Peripheral Library V3.5.0
- 架构：bare-metal，无 RTOS
- 工程文件：`Project.uvprojx`

工程继续使用 STM32F10x SPL，不迁移 HAL、LL、STM32CubeMX、FreeRTOS、CMake 或 Makefile。

## 当前功能

- 小车沿 100 cm x 100 cm 正方形黑色轨迹线逆时针循迹。
- 目标圈数 N 可通过 K1 设置为 1~5。
- K2 启动当前 N 圈任务。
- K3 停止并复位到 READY。
- K4 在 FAULT 状态下清故障，否则强制停车回 IDLE。
- 使用 8 路灰度误差 PD 控制循迹转向。
- 使用灰度多路压黑 + 编码器距离保护检测 90 度角点。
- 每 4 个有效角点累计 1 圈，达到目标圈数后停车。
- 丢线短时间内按最后误差方向找线，超时进入 FAULT 并停车。
- OLED 显示目标圈数、已完成圈数、角点数、状态码、故障码、进度和灰度误差。
- `ECar_GetLapProgress()` 提供 0.0~0.999 的当前圈进度接口，供后续模块同步使用。
- HC-04 通过 USART1 作为透明串口，用网页调参工具远程修改速度、Kp、Kd 和目标圈数。
- 每 100 ms 通过串口回传 `[plot,target,current,error,pwm]` 曲线数据。

## 已移除的 H 题内容

以下旧模块已从工程文件中移除，并从本地文件夹删除：

- `App/app_protocol.c`
- `App/app_protocol.h`
- `App/app_state.c`
- `App/app_state.h`
- `Hardware/BT_proto.c`
- `Hardware/BT_proto.h`
- `Hardware/RP.c`
- `Hardware/RP.h`
- `Hardware/MPU6050.c`
- `Hardware/MPU6050.h`
- `Hardware/LED.c`
- `Hardware/LED.h`

工程不再包含 H 题 task1/task2/task3/task4 状态机、A/B/C/D 路径逻辑、半圆弧逻辑、MPU yaw 对准逻辑、蓝牙遥控运动逻辑或旧复杂串口协议。

## 目录结构

```text
User/       主入口、主循环、TIM1 中断调度
App/        E 题状态机、速度闭环封装、灰度循迹
Hardware/   PWM、电机、编码器、灰度、按键、OLED、串口、声光提示
Control/    PID 控制模块
System/     Delay 和 Timer 配置
Start/      CMSIS Core、启动文件、system_stm32f10x
Library/    STM32F10x 标准外设库源文件
```

关键应用文件：

- `User/main.c`：初始化和主循环调度。
- `App/app_e_car.c`：E 题小车状态机、参数、全局控制变量、按键和 OLED 显示。
- `App/app_e_car.h`：E 题小车公共接口。
- `App/app_e_serial.c`：HC-04 / USART1 远程调参协议解析和 plot 回传。
- `App/app_e_serial.h`：远程调参模块公共接口。
- `App/app_control.c`：编码器速度更新、速度/转向 PID、电机 PWM 输出。
- `App/app_line.c`：8 路灰度读取、线误差计算、灰度 PD 转向计算。

## 主程序流程

`main()` 初始化顺序：

```c
OLED_Init();
Key_Init();
Grayscale_Init();
Motor_Init();
Encoder_Init();
App_Line_GPIOForceInit();
BeepLed_Init();
Serial_Init();
App_Control_Init();
ECar_Init();
ECar_Serial_Init();
Timer_Init();
```

主循环：

1. `g_flag_10ms` 置位后调用 `ECar_Control10ms()`。
2. 同一个 10 ms 调度中调用 `ECar_SerialPlot10ms()`，内部每 100 ms 回传一次 plot。
3. 持续调用 `ECar_KeyProcess()`。
4. 持续调用 `ECar_SerialProcess()` 解析串口包。
5. `g_oledRefreshFlag` 置位后调用 `ECar_ShowStatus()`。

TIM1 1ms 中断只做轻量工作：

1. 清 TIM1 更新中断标志。
2. `Key_Tick()`。
3. `ECar_PromptTick1ms()`。
4. 每 10ms 置 `g_flag_10ms`。
5. 每 200ms 置 `g_oledRefreshFlag`。

## E 题状态机

状态定义在 `App/app_e_car.h`：

```c
typedef enum
{
    E_CAR_IDLE = 0,
    E_CAR_READY,
    E_CAR_LINE_RUN,
    E_CAR_CORNER_ENTER,
    E_CAR_CORNER_TURN,
    E_CAR_LINE_RECOVER,
    E_CAR_FINISH,
    E_CAR_FAULT
} ECarState_t;
```

运行逻辑：

- `E_CAR_IDLE` / `E_CAR_READY`：停车等待。
- `E_CAR_LINE_RUN`：正常循迹、更新编码器、计算进度、检测角点和丢线。
- `E_CAR_CORNER_ENTER`：角点计数，四个角点累计一圈。
- `E_CAR_CORNER_TURN`：低速固定差速转弯，超过最小转角时间后等待重新捕线。
- `E_CAR_LINE_RECOVER`：低速循迹恢复，连续稳定后回到正常循迹。
- `E_CAR_FINISH`：达到目标圈数后停车。
- `E_CAR_FAULT`：丢线或转角恢复异常后停车，等待 K4 清除。

## 主要参数

E 题参数在 `App/app_e_car.c` 的 `g_eCarParam` 中修改：

```c
ECarParam_t g_eCarParam =
{
    30.0f,   // base_speed
    18.0f,   // recover_speed
    12.0f,   // corner_forward_speed
    35.0f,   // corner_turn_speed
    0.25f,   // line_kp
    0.50f,   // line_kd
    80.0f,   // turn_limit
    300U,    // lost_timeout_ms
    150U,    // recover_stable_ms
    250U,    // corner_min_turn_ms
    1200U,   // corner_max_turn_ms
    4500,    // min_corner_interval_pulse
    28120,   // lap_pulse_default
    5U       // corner_black_count_th
};
```

优先实车调试：

- `base_speed`
- `recover_speed`
- `corner_forward_speed`
- `corner_turn_speed`
- `line_kp`
- `line_kd`
- `turn_limit`
- `corner_min_turn_ms`
- `corner_max_turn_ms`
- `min_corner_interval_pulse`
- `lap_pulse_default`
- `corner_black_count_th`
- `g_lineTurnSign`

## IO 分配

### 电机 TB6612

```text
PWMA = PA0 / TIM2_CH1
PWMB = PA1 / TIM2_CH2
AIN1 = PB12
AIN2 = PB13
BIN1 = PB14
BIN2 = PB15
LEFT_MOTOR_DIR_SIGN  = -1
RIGHT_MOTOR_DIR_SIGN = +1
```

### 编码器

```text
Left encoder E1:  PA6 / PA7 -> TIM3_CH1 / TIM3_CH2
Right encoder E2: PB6 / PB7 -> TIM4_CH1 / TIM4_CH2
LEFT_ENCODER_SIGN  = -1
RIGHT_ENCODER_SIGN = +1
```

### 8 路灰度传感器

```text
AD0 = PA8
AD1 = PB3
AD2 = PB4
OUT = PB0
```

PB3/PB4 默认是 JTAG 相关引脚，`Grayscale_Init()` 和 `App_Line_GPIOForceInit()` 会关闭 JTAG，只保留 SWD。

### OLED

```text
SCL = PB8
SDA = PB9
```

MPU6050 已从第一版工程删除，PB8/PB9 当前只给 OLED 软件 I2C 使用。

### 串口

```text
USART1_TX = PA9
USART1_RX = PA10
Baudrate = 9600
```

HC-04 作为普通透明串口模块使用：

```text
HC-04 TXD -> PA10
HC-04 RXD -> PA9
GND 共地
```

USART1 保持 9600 bps。当前保留基础发送、接收环形缓冲和 `Serial_Printf()` 能力，并新增 E 题远程调参协议；不保留旧 H 题串口协议和蓝牙遥控运动。

网页默认调参协议映射：

```text
[slider,target,value] -> g_eCarParam.base_speed, range 0~60
[slider,Kp,value]     -> g_eCarParam.line_kp, range 0~3
[slider,Kd,value]     -> g_eCarParam.line_kd, range 0~8
[slider,Ki,value]     -> target lap N, range 1~5, running state returns busy
[key,emergency,down]  -> ECar_Stop()
[joystick,...]        -> ignored, never drives motors
```

兼容查询和控制包：

```text
[get]
[status]
[start]
[stop]
[ecar,get]
[ecar,status]
[ecar,start]
[ecar,stop]
[ecar,set,n,value]
```

回包示例：

```text
[status,ok,set,target,30]
[status,ok,set,Kp,0.25]
[status,err,busy]
[ecar,val,n,3,base,30,Kp,0.25,Kd,0.50,turnLimit,80]
[ecar,state,s,2,n,3,lap,1,corner,5,err,-120,pwm,350,prog,30,fault,0,time,12340]
[plot,30,28,-120,350]
```

### 按键与声光提示

```text
K1 = PB10
K2 = PB11
K3 = PA11
K4 = PA12
BEEP = PB1, low active
LED_EXT = PB5, high active
```

## 安全行为

任何 STOP、FINISH、FAULT 都会执行：

1. `g_targetForwardSpeed = 0.0f`
2. `g_targetTurnSpeed = 0.0f`
3. `g_carEnable = 0`
4. `Motor_StopAll()`
5. `App_Control_ResetPID()`

上电后默认进入 E 题待机界面，不启动电机。`[joystick,...]` 等遥控运动包不会让小车运动。

## Keil 配置

默认目标芯片按 STM32F103C8T6 处理：

- Device：`STM32F103C8` 或 `STM32F103C8Tx`
- Define：`USE_STDPERIPH_DRIVER,STM32F10X_MD`
- Startup：`Start/startup_stm32f10x_md.s`
- IROM1：Start `0x08000000`，Size `0x00010000`
- IRAM1：Start `0x20000000`，Size `0x00005000`

`Project.uvprojx` 已包含当前 E 题工程需要编译的 `.c` 文件，并已移除旧 H 题文件引用。

## 构建步骤

在 Keil 中执行：

1. Pack Installer 确认安装 `Keil.STM32F1xx_DFP`。
2. Options for Target -> Device 选择 `STM32F103C8` 或 `STM32F103C8Tx`。
3. Options for Target -> Target 检查 IROM/IRAM。
4. Options for Target -> C/C++ (AC6) 检查 Define 和 Include Path。
5. Options for Target -> Utilities -> Flash Download 添加 STM32F10x Medium-density Flash。
6. Clean Target。
7. Rebuild all target files。
8. Download。

不要声称工程已通过 Keil 编译，除非有真实 Keil build log。不要声称硬件行为已验证，除非有实车测试结果。

## 调试顺序

建议按以下顺序实车调试：

1. Keil 编译通过。
2. 烧录固件。
3. 架空小车，确认上电后轮子不转。
4. 检查 OLED 是否显示 E CAR 页面。
5. K1 检查 N=1~5 循环。
6. K2 低速启动循迹。
7. 调 `line_kp`、`line_kd`、`turn_limit`，先保证不脱线。
8. 调角点检测阈值和距离保护。
9. 检查每 4 个角点是否累计 1 圈。
10. 检查达到目标 N 圈后是否自动停车。
11. 再逐步提高 `base_speed`。

## Git 跟踪规则

仓库使用白名单 `.gitignore`，默认忽略所有文件。当前源码目录已放行：

```gitignore
!Project.uvprojx
!Start/**
!Library/**
!Hardware/**
!System/**
!Control/**
!App/**
!User/main.c
!User/stm32f10x_conf.h
!User/stm32f10x_it.c
!User/stm32f10x_it.h
!.gitignore
!AGENTS.md
```

不要提交 `Objects/`、`Listings/`、`DebugConfig/`、`*.o`、`*.d`、`*.crf`、`*.htm`、`*.map`、`*.axf`、`*.hex` 等生成文件。
