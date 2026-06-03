#include "stm32f10x.h"
#include "BeepLed.h"

/*
 * H题声光提示扩展：
 *   PB1 -> BEEP，有源蜂鸣器，低电平触发
 *   PB5 -> LED_EXT，外接提示LED，高电平点亮
 *
 * 逻辑：
 *   PB1 = 0 蜂鸣器响
 *   PB1 = 1 蜂鸣器不响
 *   PB5 = 1 LED亮
 *   PB5 = 0 LED灭
 */

#define BEEP_PORT       GPIOB
#define BEEP_PIN        GPIO_Pin_1

#define LED_EXT_PORT    GPIOB
#define LED_EXT_PIN     GPIO_Pin_5

void BeepLed_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = BEEP_PIN | LED_EXT_PIN;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    BeepLed_AllOff();
}

/* 蜂鸣器：低电平触发 */
void Beep_On(void)
{
    GPIO_ResetBits(BEEP_PORT, BEEP_PIN);
}

void Beep_Off(void)
{
    GPIO_SetBits(BEEP_PORT, BEEP_PIN);
}

void Beep_Turn(void)
{
    if (GPIO_ReadOutputDataBit(BEEP_PORT, BEEP_PIN))
    {
        Beep_On();
    }
    else
    {
        Beep_Off();
    }
}

/* 外接LED：高电平点亮 */
void LedExt_On(void)
{
    GPIO_SetBits(LED_EXT_PORT, LED_EXT_PIN);
}

void LedExt_Off(void)
{
    GPIO_ResetBits(LED_EXT_PORT, LED_EXT_PIN);
}

void LedExt_Turn(void)
{
    if (GPIO_ReadOutputDataBit(LED_EXT_PORT, LED_EXT_PIN))
    {
        LedExt_Off();
    }
    else
    {
        LedExt_On();
    }
}

void BeepLed_AllOn(void)
{
    Beep_On();
    LedExt_On();
}

void BeepLed_AllOff(void)
{
    Beep_Off();
    LedExt_Off();
}


void BeepLed_AllTurn(void)
{
    Beep_Turn();
    LedExt_Turn();
}
