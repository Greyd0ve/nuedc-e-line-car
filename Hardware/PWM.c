/*
 * 文件：Hardware/PWM.c
 * 作用：生成 TB6612 的两路 PWM。
 *
 * TIM2 周期为 PWM_MAX_DUTY，PA0 输出 PWMA，PA1 输出 PWMB。
 * Motor.c 负责方向控制，这里只负责占空比。
 *
 * 本版说明：已按小车训练板修正 PWM 引脚为 PA0/PA1。
 */
#include "stm32f10x.h"
#include "PWM.h"

/*
 * Training board pin map:
 *   PWMA -> PA0 -> TIM2_CH1
 *   PWMB -> PA1 -> TIM2_CH2
 */
void PWM_Init(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    TIM_InternalClockConfig(TIM2);

    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period = PWM_MAX_DUTY - 1;
    TIM_TimeBaseInitStructure.TIM_Prescaler = 4 - 1;
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);

    TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;

    TIM_OC1Init(TIM2, &TIM_OCInitStructure);
    TIM_OC2Init(TIM2, &TIM_OCInitStructure);

    TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM2, ENABLE);

    TIM_Cmd(TIM2, ENABLE);
}

/* 设置左电机 PWM 占空比。 */
void PWM_SetCompareA(uint16_t Compare)
{
    if (Compare > PWM_MAX_DUTY)
    {
        Compare = PWM_MAX_DUTY;
    }
    TIM_SetCompare1(TIM2, Compare);
}

/* 设置右电机 PWM 占空比。 */
void PWM_SetCompareB(uint16_t Compare)
{
    if (Compare > PWM_MAX_DUTY)
    {
        Compare = PWM_MAX_DUTY;
    }
    TIM_SetCompare2(TIM2, Compare);
}
