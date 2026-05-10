/* 项目适配说明：板载 RP1~RP4 电位器读取模块。本版本最大速度由网页 [slider,RP,x] 控制，电位器接口保留给后续扩展。 */
/*
 * 文件：Hardware/RP.c
 * 作用：读取 RP1~RP4 电位器 ADC 值。
 *
 * 本工程主要使用 RP4 作为最高速度和 PWM 限幅旋钮。main.c 中会把 RP4 的 0~4095 ADC
 * 映射为速度缩放和 PWM 上限。
 *
 * 注释版说明：仅增加注释，未改变 ADC 采样逻辑。
 */
#include "stm32f10x.h"

/*
 * Training board potentiometer pin map:
 *   RP1 -> PA3 -> ADC2_IN3
 *   RP2 -> PA4 -> ADC2_IN4
 *   RP3 -> PA5 -> ADC2_IN5
 *   RP4 -> PA6 -> ADC2_IN6
 */
/* 初始化 ADC 和 RP1~RP4 对应的模拟输入通道。 */
void RP_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    RCC_ADCCLKConfig(RCC_PCLK2_Div6);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    ADC_InitTypeDef ADC_InitStructure;
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_NbrOfChannel = 1;
    ADC_Init(ADC2, &ADC_InitStructure);

    ADC_Cmd(ADC2, ENABLE);

    ADC_ResetCalibration(ADC2);
    while (ADC_GetResetCalibrationStatus(ADC2) == SET);
    ADC_StartCalibration(ADC2);
    while (ADC_GetCalibrationStatus(ADC2) == SET);
}

/* 读取第 n 个电位器的 ADC 值，范围 0~4095。 */
uint16_t RP_GetValue(uint8_t n)
{
    uint8_t channel = ADC_Channel_3;

    if (n == 1)
    {
        channel = ADC_Channel_3;
    }
    else if (n == 2)
    {
        channel = ADC_Channel_4;
    }
    else if (n == 3)
    {
        channel = ADC_Channel_5;
    }
    else if (n == 4)
    {
        channel = ADC_Channel_6;
    }

    ADC_RegularChannelConfig(ADC2, channel, 1, ADC_SampleTime_55Cycles5);
    ADC_SoftwareStartConvCmd(ADC2, ENABLE);
    while (ADC_GetFlagStatus(ADC2, ADC_FLAG_EOC) == RESET);
    return ADC_GetConversionValue(ADC2);
}
