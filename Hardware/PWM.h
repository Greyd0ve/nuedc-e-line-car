/* TIM2 PWM 输出接口。训练板映射：PWMA=PA0/TIM2_CH1，PWMB=PA1/TIM2_CH2。 */
#ifndef __PWM_H
#define __PWM_H

#include <stdint.h>

#define PWM_MAX_DUTY    1000U

void PWM_Init(void);
void PWM_SetCompareA(uint16_t Compare);
void PWM_SetCompareB(uint16_t Compare);

#endif
