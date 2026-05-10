/* TB6612 电机驱动接口。上层只传入左右轮有符号 PWM，方向引脚和占空比由 Motor.c 统一处理。 */
#ifndef __MOTOR_H
#define __MOTOR_H

#include <stdint.h>

void Motor_Init(void);
void Motor_SetLeftPWM(int16_t PWM);
void Motor_SetRightPWM(int16_t PWM);
void Motor_SetPWM(int16_t LeftPWM, int16_t RightPWM);
void Motor_StopAll(void);

#endif
