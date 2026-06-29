#include "app_control.h"
#include "Encoder.h"
#include "Motor.h"
#include "PWM.h"
#include "pid.h"
#include <stdint.h>

#define APP_PWM_LIMIT_MIN       0.0f
#define APP_FORWARD_I_LIMIT     260.0f
#define APP_TURN_I_LIMIT        220.0f

extern volatile float g_forwardKp;
extern volatile float g_forwardKi;
extern volatile float g_forwardKd;
extern volatile float g_turnKp;
extern volatile float g_turnKi;
extern volatile float g_turnKd;

extern volatile float g_pwmLimit;
extern volatile float g_targetForwardSpeed;
extern volatile float g_targetTurnSpeed;
extern volatile float g_leftSpeed;
extern volatile float g_rightSpeed;
extern volatile float g_forwardSpeed;
extern volatile float g_turnSpeed;
extern volatile float g_speedPwm;
extern volatile float g_diffPwm;
extern volatile float g_forwardSpeedError;

extern volatile int16_t g_leftPwm;
extern volatile int16_t g_rightPwm;
extern volatile uint8_t g_carEnable;

extern volatile int32_t g_leftEncoderTotal;
extern volatile int32_t g_rightEncoderTotal;
extern volatile int32_t g_forwardEncoderTotal;
extern volatile int32_t g_turnEncoderTotal;

static PID_TypeDef ForwardPID;
static PID_TypeDef TurnPID;

static float App_Control_LimitFloat(float value, float minVal, float maxVal)
{
    if (value < minVal) return minVal;
    if (value > maxVal) return maxVal;
    return value;
}

static int16_t App_Control_LimitI16(int32_t value, int16_t minVal, int16_t maxVal)
{
    if (value < minVal) return minVal;
    if (value > maxVal) return maxVal;
    return (int16_t)value;
}

void App_Control_Init(void)
{
    PID_Init(&ForwardPID, g_forwardKp, g_forwardKi, g_forwardKd, (float)PWM_MAX_DUTY, APP_FORWARD_I_LIMIT);
    PID_Init(&TurnPID, g_turnKp, g_turnKi, g_turnKd, (float)PWM_MAX_DUTY * 0.85f, APP_TURN_I_LIMIT);
}

void App_Control_UpdatePIDParam(void)
{
    PID_SetTunings(&ForwardPID, g_forwardKp, g_forwardKi, g_forwardKd);
    PID_SetTunings(&TurnPID, g_turnKp, g_turnKi, g_turnKd);
}

void App_Control_ResetPID(void)
{
    PID_Reset(&ForwardPID);
    PID_Reset(&TurnPID);
}

void App_Control_ForcePWMZero(void)
{
    g_targetForwardSpeed = 0.0f;
    g_targetTurnSpeed = 0.0f;
    g_speedPwm = 0.0f;
    g_diffPwm = 0.0f;
    g_leftPwm = 0;
    g_rightPwm = 0;
    g_carEnable = 0;
    Motor_StopAll();
    App_Control_ResetPID();
}

void App_Control_UpdateEncoderSpeed(void)
{
    int16_t leftDelta;
    int16_t rightDelta;

    leftDelta = Encoder_GetLeftDelta();
    rightDelta = Encoder_GetRightDelta();

    g_leftEncoderTotal += leftDelta;
    g_rightEncoderTotal += rightDelta;
    g_forwardEncoderTotal = (g_leftEncoderTotal + g_rightEncoderTotal) / 2;
    g_turnEncoderTotal = (g_rightEncoderTotal - g_leftEncoderTotal) / 2;

    g_leftSpeed = (float)leftDelta;
    g_rightSpeed = (float)rightDelta;
    g_forwardSpeed = (g_leftSpeed + g_rightSpeed) * 0.5f;
    g_turnSpeed = (g_rightSpeed - g_leftSpeed) * 0.5f;
}

void App_Control_ApplyMotorOutput(void)
{
    float pwmLimit;
    int32_t leftPwmTemp;
    int32_t rightPwmTemp;

    if (!g_carEnable || g_pwmLimit <= 0.5f)
    {
        g_forwardSpeedError = g_targetForwardSpeed - g_forwardSpeed;
        g_speedPwm = 0.0f;
        g_diffPwm = 0.0f;
        g_leftPwm = 0;
        g_rightPwm = 0;
        Motor_StopAll();
        App_Control_ResetPID();
        return;
    }

    pwmLimit = App_Control_LimitFloat(g_pwmLimit, APP_PWM_LIMIT_MIN, (float)PWM_MAX_DUTY);
    PID_SetOutputLimit(&ForwardPID, pwmLimit);
    PID_SetOutputLimit(&TurnPID, pwmLimit * 0.85f);

    g_forwardSpeedError = g_targetForwardSpeed - g_forwardSpeed;
    g_speedPwm = PID_Calc(&ForwardPID, g_targetForwardSpeed, g_forwardSpeed);
    g_diffPwm = PID_Calc(&TurnPID, g_targetTurnSpeed, g_turnSpeed);

    leftPwmTemp = (int32_t)(g_speedPwm - g_diffPwm);
    rightPwmTemp = (int32_t)(g_speedPwm + g_diffPwm);

    g_leftPwm = App_Control_LimitI16(leftPwmTemp, (int16_t)(-pwmLimit), (int16_t)pwmLimit);
    g_rightPwm = App_Control_LimitI16(rightPwmTemp, (int16_t)(-pwmLimit), (int16_t)pwmLimit);

    Motor_SetPWM(g_leftPwm, g_rightPwm);
}
