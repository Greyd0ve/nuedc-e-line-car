# NUEDC H Smart Car

本仓库是 NUEDC H 题自动行驶小车的 STM32 裸机固件工程。

## 项目目标

- MCU：STM32F103C8T6，Medium Density
- 开发环境：Keil MDK / uVision
- 编译器：ARM Compiler 6.24，ARMCLANG
- 固件库：STM32F10x Standard Peripheral Library V3.5.0
- 架构：裸机，无 RTOS
- 工程文件：Project.uvprojx

本项目继续使用 STM32F10x 标准外设库，不迁移 HAL、LL、STM32CubeMX、FreeRTOS、CMake 或 Makefile。

## 功能范围

- task1：A -> B
- task2：A -> B -> C -> D -> A
- task3：A -> C -> B -> D -> A
- task4：按 task3 路径多圈运行
- MPU yaw 闭环直线控制
- 入弯 yaw 硬切入 + 灰度找线
- 半圆八路灰度循迹
- 出弯最小距离 + 丢线确认 + yaw 窗口
- 速度 PID 与左右轮差速输出
- OLED 状态显示
- 串口在线调参、任务控制、plot/telemetry 回传
- MPU 校准、yaw 清零、急停和安全锁定

蓝牙串口只用于参数调试和任务控制，蓝牙遥控运动已禁用。`joystick`、`Bluetooth` 手动驾驶、`forward/backward/left/right`、`speedUp/speedDown` 等远程运动命令不得重新启用。

## 目录结构

```text
User/       主入口、全局状态、任务状态机、主循环
Hardware/   电机、编码器、OLED、MPU6050、灰度、PWM、串口、按键等底层驱动
Control/    PID 控制模块
App/        协议解析、控制封装、循迹和状态接口
System/     延时和定时器配置
Start/      CMSIS Core、启动文件、system_stm32f10x
Library/    STM32F10x 标准外设库源码
```

当前工程使用扁平化 `Library/` 目录保存 SPL 源文件，而不是标准的 `Library/STM32F10x_StdPeriph_Driver/` 子目录。

## Keil 配置

默认目标芯片按 STM32F103C8T6 处理：

- Device：STM32F103C8 或 STM32F103C8Tx
- Define：`USE_STDPERIPH_DRIVER,STM32F10X_MD`
- Startup：`Start/startup_stm32f10x_md.s`
- IROM1：Start `0x08000000`，Size `0x00010000`
- IRAM1：Start `0x20000000`，Size `0x00005000`

`Project.uvprojx` 已切换到 ARM Compiler 6.24 / ARMCLANG，并包含 `Start/system_stm32f10x.c`、`Library/misc.c` 和当前工程使用的 SPL 源文件。

## 构建步骤

在 Keil 中执行：

1. Pack Installer 确认安装 `Keil.STM32F1xx_DFP`
2. Options for Target -> Device 选择 `STM32F103C8` 或 `STM32F103C8Tx`
3. Options for Target -> Target 检查 IROM/IRAM
4. Options for Target -> C/C++ (AC6) 检查 Define 和 Include Path
5. Options for Target -> Utilities -> Flash Download 添加 STM32F10x Medium-density Flash
6. Clean Target
7. Rebuild all target files
8. Download

不要声称工程已通过 Keil 编译，除非有真实 Keil build log。不要声称硬件行为已验证，除非有实车测试结果。

## Git 跟踪规则

仓库使用白名单 `.gitignore`，默认忽略所有文件。需要追踪的工程文件已显式放行：

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
```

不要提交 `Objects/`、`Listings/`、`DebugConfig/`、`*.o`、`*.d`、`*.crf`、`*.htm`、`*.map`、`*.axf`、`*.hex` 等生成文件。
