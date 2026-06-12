# AGENTS.md — NUEDC H Smart Car

## Project overview
- STM32F103C8T6 (Cortex-M3, 64KB/20KB) firmware for a line-following smart car.
- Keil MDK 5 (uVision) project — **ARM Compiler 5 (V5.06)**. Builds only through Keil IDE.
- Uses **STM32F10x Standard Peripheral Library V3.5.0** (not HAL/LL). Library sources live in `Library/`.
- No Makefile, no CMake, no CLI build, no CI, no test framework, no linter/formatter.

## Build output
- Objects: `Objects/` (not tracked by git)
- Listings: `Listings/` (not tracked by git)
- Keil project file: `Project.uvprojx` — the single source of truth for toolchain and compile/link settings.

## .gitignore is an allow-list
```
# Ignores everything by default:
*

# Only these directories are tracked:
!Hardware/**
!System/**
!Control/**
!App/**
!User/main.c
!.gitignore
```
**Any new source file or directory must be explicitly added to `.gitignore`** or it won't be tracked.

## Directory structure and ownership
| Dir | Purpose |
|-----|---------|
| `User/` | Entrypoint (`main.c`), interrupts (`stm32f10x_it.c`), library config (`stm32f10x_conf.h`) |
| `Hardware/` | Low-level peripheral drivers (Motor, Encoder, OLED, MPU6050, Grayscale, PWM, Serial, etc.) |
| `Control/` | PID controller (`pid.c/h`) — generic, reusable position PID |
| `App/` | Higher-level logic: protocol parsing (`app_protocol.c`), control glue (`app_control.c`), line tracking (`app_line.c`), mode switching (`app_state.c`) |
| `System/` | Software delay (`Delay.c`) and timer setup (`Timer.c`) |
| `Library/` | STM32F10x StdPeriphLib — read-only vendor code, do not edit |

## Architecture
- **Main loop** (no RTOS): polls `App_Protocol_Process()` (serial packet dispatch), `Main_KeyProcess()` (physical buttons), then conditionally updates MPU/OLED/plot serial output at fixed intervals.
- **TIM1 1ms ISR**: debounces keys, toggles prompts, increments timing counters, and runs `Control_Run10ms()` every 10ms (100Hz control loop).
- **All global state is `volatile`** — parameters can change mid-loop from protocol slider/key packets.
- The protocol parser (`app_protocol.c`) directly writes to `extern volatile` globals from `main.c`. App layer and main.c are tightly coupled through shared state, not cleanly separated.

## Control flow in `Control_Run10ms()`
Priority dispatch: task2 states → straight active → arc active → tracing mode → BT/standby timeout.
Each mode sets `g_targetForwardSpeed`, `g_targetTurnSpeed`, `g_carEnable`; `App_Control_ApplyMotorOutput()` translates those to motor PWM via PID.

## Protocol (serial UART)
Packet format: `[type,field1,field2,...]\r\n`

| Packet type | Example |
|------------|---------|
| `key` | `[key,Bluetooth,down]`, `[key,emergency,down]`, `[key,task1,down]` |
| `slider` | `[slider,Kp,14.0]`, `[slider,task2SearchPulse,6500]` |
| `joystick` | `[joystick,50,60]` (turn, forward, range -100..100) |
| `pid` | `[pid,set,kp,12.5]`, `[pid,start]`, `[pid,stop]`, `[pid,get]` |

Keys are case-insensitive. `key` commands only activate on `down` (press, not release).
Slider aliases are extensive — see `app_protocol.c` for all accepted names.

## Physical buttons (PCB mapping)
| Button | Pin | Function |
|--------|-----|----------|
| K1 | PB10 | Cycle mode: Standby → Encoder Debug → MPU Debug → Standby |
| K2 | PB11 | Cycle task: task1 → task2 → task3 → task4 (select only, doesn't run) |
| K3 | PA11 | Execute selected task / clear encoder pulses / zero yaw |
| K4 | PA12 | Unlock (clear `g_safetyLocked`, PWM zero, return to standby) |

## Safety / emergency
- `g_safetyLocked` blocks all motion. Set by `[key,emergency,down]` or `App_EmergencyStop()`.
- Only cleared by K4 (physical) or `[key,unlock,down]`.
- BT timeout: if `g_lastCmdTickMs > 600ms` without a valid joystick/protocol packet, speeds zero out.

## Debug/plot modes (`g_plotMode`)
| Value | Display on OLED | Serial output |
|-------|----------------|---------------|
| 0 | Default status | `[p,...]` basic telemetry |
| 1 | Encoder debug | `[p,...]` encoder/encoder totals |
| 2 | MPU debug | `[p,...]` yaw, gyro, calibration status |
| 3 | Straight/yaw debug | `[p,...]` yaw, distance, yaw error |
| 4 | Arc/task2 debug | `[p,...]` arc delta, line error, wheel diff |
| 5 | Web PID tuning | `[plot,...]` target vs actual speed + PID error |

MPU6050 gyro Z calibration blocks the main loop for ~900ms (300 samples at 3ms each). During calibration, `g_mpuCalibrating = 1`.

## Writing new code
- Add new `.c/.h` to `Hardware/`, `App/`, `Control/`, or `System/`.
- **Update `.gitignore`** to track the new file.
- Avoid malloc — this is a bare-metal embedded system with 20KB RAM.
- Follow existing naming conventions: `Module_FunctionName()` for public APIs.
- Don't use printf directly; use `Serial_Printf()` for UART output or `OLED_Printf()` for display.
- Headers use `#ifndef __MODULE_H` / `#define __MODULE_H` guards (double underscores).
