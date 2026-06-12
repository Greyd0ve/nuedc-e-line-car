#ifndef __APP_CONTROL_H
#define __APP_CONTROL_H

void App_Control_Init(void);
void App_Control_UpdatePIDParam(void);
void App_Control_ResetPID(void);
void App_Control_ForcePWMZero(void);
void App_Control_UpdateEncoderSpeed(void);
void App_Control_ApplyMotorOutput(void);

#endif
