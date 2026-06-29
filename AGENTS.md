# AGENTS.md - NUEDC 2025 E Smart Car

## Project Identity

This repository contains bare-metal firmware for the 2025 NUEDC E-problem smart car line-tracking part.

Current hardware/software target:

- MCU: **STM32F103C8T6**
- Toolchain: **Keil MDK / uVision**
- Compiler: **ARM Compiler 6.24 / ARMCLANG**
- Firmware library: **STM32F10x Standard Peripheral Library V3.5.0**
- Architecture: **bare-metal, no RTOS**

Do not migrate this project to HAL, LL, STM32CubeMX, FreeRTOS, CMake, Makefile, another MCU, or another firmware architecture unless explicitly requested.

The old H-problem code has been removed. The default project is now the E-problem car firmware. Do not add H-problem compatibility modes.

## File Operation Rules

Do not bulk-delete files or directories.

Do not use:

- `del /s`
- `rd /s`
- `rmdir /s`
- `Remove-Item -Recurse`
- `rm -rf`

When deleting files, delete only one explicit file path at a time, for example:

```powershell
Remove-Item "C:\path\to\file.txt"
```

If a task requires bulk deletion, stop and ask the user to delete those files manually.

## Hard Constraints

- Work only in the current local workspace.
- Do not clone, pull, fetch, push, or create remote branches.
- Do not commit or publish changes unless the user explicitly asks.
- Keep patches small and reviewable.
- Do not introduce dynamic allocation.
- Do not add large libraries.
- Do not add blocking long delays in the control path.
- Do not assume command-line builds work; the project is built through Keil.
- Do not claim the project builds unless a real Keil build log is available.
- If adding a compiled source file, update `Project.uvprojx`.
- This repository uses an allow-list `.gitignore`; make sure new source files are not accidentally ignored.

## Current Project Scope

The current firmware only implements the E-problem car line-tracking part:

- Follow a 100 cm x 100 cm square black line.
- Black line width is about 1.8 cm.
- Drive counter-clockwise.
- Target laps N can be set from 1 to 5.
- Stop automatically after N laps.
- First target is stable single-lap time `t <= 20s`.
- Prioritize stability before speed.
- No remote driving during motion.
- No turret, laser, camera, visual recognition, aiming module, power-management module, MSPM0 migration, or H-problem compatibility.

## Current Architecture

Main entry:

- `User/main.c`

E-problem application:

- `App/app_e_car.c`
- `App/app_e_car.h`

Reusable base modules:

- `Hardware/PWM.c`, `Hardware/PWM.h`
- `Hardware/Motor.c`, `Hardware/Motor.h`
- `Hardware/Encoder.c`, `Hardware/Encoder.h`
- `Hardware/Grayscale.c`, `Hardware/Grayscale.h`
- `Hardware/Key.c`, `Hardware/Key.h`
- `Hardware/OLED.c`, `Hardware/OLED.h`
- `Hardware/Serial.c`, `Hardware/Serial.h`
- `Hardware/BeepLed.c`, `Hardware/BeepLed.h`
- `System/Timer.c`, `System/Timer.h`
- `Control/pid.c`, `Control/pid.h`
- `App/app_control.c`, `App/app_control.h`
- `App/app_line.c`, `App/app_line.h`

Removed H-problem modules:

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

Do not reintroduce task1/task2/task3/task4, A/B/C/D path logic, half-circle arc logic, MPU yaw alignment logic, Bluetooth remote driving, or the old complex serial protocol unless the user explicitly requests it.

## Directory Ownership

| Directory | Purpose |
| --- | --- |
| `User/` | Firmware entry point, initialization, main loop, TIM1 ISR scheduling |
| `App/` | E-car state machine, line tracking glue, motor control glue |
| `Hardware/` | Low-level drivers: Motor, Encoder, OLED, Grayscale, PWM, Serial, Key, BeepLed |
| `Control/` | Generic PID code |
| `System/` | Delay and timer setup |
| `Start/` | CMSIS Core, startup file, system file |
| `Library/` | STM32F10x SPL vendor code; avoid editing |

## Runtime Flow

`main()` initializes:

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
Timer_Init();
```

Main loop responsibilities:

1. If `g_flag_10ms` is set, clear it and call `ECar_Control10ms()`.
2. Call `ECar_KeyProcess()`.
3. If `g_oledRefreshFlag` is set, clear it and call `ECar_ShowStatus()`.

TIM1 1ms interrupt responsibilities:

1. Clear TIM1 update interrupt flag.
2. Call `Key_Tick()`.
3. Call `ECar_PromptTick1ms()`.
4. Set `g_flag_10ms` every 10 ms.
5. Set `g_oledRefreshFlag` every 200 ms.

Do not run the state machine, OLED refresh, serial printing, or long delays inside the interrupt.

## E-Car State Machine

State enum in `App/app_e_car.h`:

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

State behavior:

- `E_CAR_IDLE`: stopped, waiting for lap selection or start.
- `E_CAR_READY`: target lap selected, motor stopped.
- `E_CAR_LINE_RUN`: normal line tracking, corner detection, lost-line protection.
- `E_CAR_CORNER_ENTER`: count one valid corner; every four corners count one lap.
- `E_CAR_CORNER_TURN`: fixed low-speed differential turn.
- `E_CAR_LINE_RECOVER`: low-speed line tracking until line is stable.
- `E_CAR_FINISH`: target laps reached, stopped.
- `E_CAR_FAULT`: abnormal stop, waiting for K4 to clear.

Public interfaces:

```c
void ECar_Init(void);
void ECar_Reset(void);
void ECar_Start(void);
void ECar_Stop(void);
void ECar_Control10ms(void);
void ECar_KeyProcess(void);
void ECar_ShowStatus(void);
float ECar_GetLapProgress(void);
uint8_t ECar_GetLapCount(void);
uint8_t ECar_GetTargetLap(void);
ECarState_t ECar_GetState(void);
void ECar_PromptTick1ms(void);
```

## Main Globals

The shared control and line globals are defined in `App/app_e_car.c`, not in `User/main.c`.

Control globals:

- `g_targetForwardSpeed`
- `g_targetTurnSpeed`
- `g_carEnable`
- `g_pwmLimit`
- `g_leftSpeed`
- `g_rightSpeed`
- `g_forwardSpeed`
- `g_turnSpeed`
- `g_leftPwm`
- `g_rightPwm`
- `g_leftEncoderTotal`
- `g_rightEncoderTotal`
- `g_forwardEncoderTotal`
- `g_turnEncoderTotal`

Line globals:

- `g_lineBlackLevelF`
- `g_lineReverseOrderF`
- `g_lineTurnSign`
- `g_lineKp`
- `g_lineKd`
- `g_lineTurnLimit`
- `g_lineFilterAlpha`
- `g_lineError`
- `g_lineValid`
- `g_lineMask`
- `g_lineRawMask`
- `g_lastLineDir`
- `g_lineLostMs`

Do not add H-problem globals such as `g_straightActive`, `g_arcActive`, `g_taskState`, or MPU yaw globals.

## E-Car Parameters

Main parameters are in `g_eCarParam` in `App/app_e_car.c`:

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

Tune these on the real car:

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

## Hardware Mapping

### Motor Driver TB6612

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

### Encoders

```text
Left encoder E1:  PA6 / PA7 -> TIM3_CH1 / TIM3_CH2
Right encoder E2: PB6 / PB7 -> TIM4_CH1 / TIM4_CH2
LEFT_ENCODER_SIGN  = -1
RIGHT_ENCODER_SIGN = +1
```

### 8-Channel Grayscale Sensor Through CD4051

```text
AD0 = PA8
AD1 = PB3
AD2 = PB4
OUT = PB0
```

PB3/PB4 require JTAG to be disabled while SWD remains enabled. `Grayscale_Init()` and `App_Line_GPIOForceInit()` do this.

### OLED Software I2C

```text
PB8 = SCL
PB9 = SDA
```

MPU6050 is not part of the first E-car version and has been removed from the project.

### Serial

```text
USART1_TX = PA9
USART1_RX = PA10
Baudrate = 9600
```

Serial basic send/receive support remains. The old H-problem serial task protocol is removed. Serial input must not make the car move unless a new explicit, reviewed E-car command parser is added.

### Keys

```text
K1 / SW1 = PB10
K2 / SW2 = PB11
K3 / SW3 = PA11
K4 / SW4 = PA12
```

Current key behavior:

| Key | Behavior |
| --- | --- |
| K1 | Increase target lap N, cycling 1 -> 5 |
| K2 | Start current N-lap line-tracking task |
| K3 | Stop and reset to READY |
| K4 | Clear FAULT; if not in FAULT, force stop and return to IDLE |

### Prompt IO

```text
PB1 = BEEP, low-level active
PB5 = LED_EXT, high-level active
```

Do not use PC13 for prompt output in this project.

## Line Tracking Rules

- Call `App_Line_Update()` once every 10 ms from `ECar_Control10ms()`.
- Use `g_lineValid`, `g_lineError`, `g_lineMask`, and `g_lineRawMask`.
- Normal line tracking uses PD:

```c
turn_cmd = line_kp * error + line_kd * (error - last_error);
```

- Clamp turn command to `+-turn_limit`.
- Use `g_lineTurnSign` if real-car steering direction is reversed.
- Do not rewrite the whole tracking algorithm just to invert direction.

## Corner and Lap Counting

Corner detection uses:

- `g_lineValid != 0`
- black channel count >= `corner_black_count_th`
- current average encoder pulse minus `last_corner_forward_pulse` >= `min_corner_interval_pulse`

Lap counting:

- `corner_count++` for each valid corner.
- `lap_count++` for every four valid corners.
- Stop when `lap_count >= target_lap`.
- `lap_progress` is based on current lap encoder pulse divided by `lap_pulse_default`, clamped to `0.0~0.999`.

Do not count laps using total distance only.

## Safety Rules

Any STOP, FINISH, or FAULT must force:

```c
g_targetForwardSpeed = 0.0f;
g_targetTurnSpeed = 0.0f;
g_carEnable = 0U;
Motor_StopAll();
App_Control_ResetPID();
```

Startup must leave PWM at zero and the car stopped.

Lost-line handling:

- In `E_CAR_LINE_RUN`, short lost-line periods may search using the last error direction.
- If lost-line time exceeds `lost_timeout_ms`, enter `E_CAR_FAULT`.
- In `E_CAR_CORNER_TURN`, short lost-line periods are normal and must not immediately fault.
- In `E_CAR_LINE_RECOVER`, prolonged failure to recover enters `E_CAR_FAULT`.

## OLED Status

`ECar_ShowStatus()` displays:

```text
Line 1: E CAR
Line 2: N:<target> L:<lap_count>
Line 3: C:<corner_count> S:<state> F:<fault>
Line 4: P:<progress>% E:<line_error>
```

## Keil Project Rules

- Keil project: `Project.uvprojx`
- Objects: `Objects/`, not tracked
- Listings: `Listings/`, not tracked
- Vendor library: `Library/`, read-only unless explicitly requested

The project file must not reference deleted `.c` files. After adding or removing compiled files, update `Project.uvprojx`.

Current source groups should include:

- `User/main.c`
- `App/app_e_car.c`
- `App/app_control.c`
- `App/app_line.c`
- `Control/pid.c`
- required `Hardware/`, `System/`, `Start/`, and `Library/` files

## Git Ignore Rules

This repository uses an allow-list `.gitignore`. Source directories are explicitly unignored:

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

Do not track generated files such as `Objects/`, `Listings/`, `DebugConfig/`, `*.o`, `*.d`, `*.crf`, `*.htm`, `*.map`, `*.axf`, or `*.hex`.

## Coding Style

- Use existing naming style.
- Public functions generally use `Module_FunctionName()`.
- Keep internal helpers `static`.
- Prefer fixed-width integer types where appropriate.
- Keep RAM and code size low.
- Do not use `malloc` or `free`.
- Avoid expensive float formatting in frequent telemetry.
- Use `Serial_Printf()` for UART output.
- Use `OLED_Printf()` or OLED primitives for display.
- Avoid direct `printf()` unless the file already intentionally uses it.
- Header guards follow the existing project style.

## Bring-Up Sequence

Recommended bring-up:

1. Confirm Keil compile result.
2. Flash firmware.
3. Put the car on a stand and confirm wheels do not move on power-up.
4. Confirm OLED shows `E CAR`.
5. Use K1 to check N cycles from 1 to 5.
6. Use K2 to start at low speed.
7. Tune line PD until the car tracks reliably.
8. Tune corner detection and minimum corner interval.
9. Confirm each four corners increments one lap.
10. Confirm target N laps stop automatically.
11. Increase speed only after stability is good.

Do not claim hardware behavior is verified unless the user provides test results.
Do not claim Keil build success unless a real Keil build log is available.
