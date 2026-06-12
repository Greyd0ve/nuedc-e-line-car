#ifndef __APP_PROTOCOL_H
#define __APP_PROTOCOL_H

#include <stdint.h>

#define PLOT_MODE_WEB_PID       5U

void App_Protocol_Process(void);
void App_Protocol_ApplySpeedLimitPercent(float percent);

/* Hooks implemented by User/main.c to keep legacy task logic in place. */
void App_ProtocolTaskReset(void);
void App_ProtocolForcePWMZero(void);
void App_ProtocolPromptStart(uint16_t ms);
void App_ProtocolEncoderDebugClearTotals(void);
void App_ProtocolEnterEncoderDebug(void);
void App_ProtocolEnterMpuDebug(void);
void App_ProtocolMpuCalibrateGyroZ(void);
void App_ProtocolMpuResetYaw(void);
void App_ProtocolSelectTask1(void);
void App_ProtocolSelectTask2(void);
void App_ProtocolSelectOnly(uint8_t task);
void App_ProtocolStartSelectedTask(void);
void App_ProtocolTaskStop(void);
void App_ProtocolArcStart(void);
uint8_t App_ProtocolTask2IsSpecialState(void);

#endif
