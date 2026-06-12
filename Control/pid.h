#ifndef __PID_H
#define __PID_H

typedef struct
{
    float Kp;
    float Ki;
    float Kd;
    float Integral;
    float LastError;
    float OutputLimit;
    float IntegralLimit;
} PID_TypeDef;

/* Speed and turn loops use the same positional PID core.
 * OutputLimit clamps motor PWM command, IntegralLimit prevents integral windup.
 */
void PID_Init(PID_TypeDef *pid, float kp, float ki, float kd, float outputLimit, float integralLimit);
void PID_SetTunings(PID_TypeDef *pid, float kp, float ki, float kd);
void PID_SetOutputLimit(PID_TypeDef *pid, float outputLimit);
void PID_Reset(PID_TypeDef *pid);
float PID_Calc(PID_TypeDef *pid, float target, float measure);

#endif
