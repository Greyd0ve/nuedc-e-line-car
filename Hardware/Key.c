/* 项目适配说明：K1~K4 用于启动 H 题不同训练任务。急停锁定时 main.c 会屏蔽按键启动。 */
/*
 * 文件：Hardware/Key.c
 * 作用：K1~K4 按键扫描和消抖。
 *
 * 逻辑说明：
 *   按键为低电平有效。Key_Tick() 在 TIM1 1ms 中断中周期调用，完成简单状态机消抖；
 *   Key_GetNum() 在主循环中读取一次性按键事件。
 *
 * 注释版说明：仅增加注释，未改变按键逻辑。
 */
#include "stm32f10x.h"
#include "Delay.h"

uint8_t Key_Num;

/*
 * Training board key map:
 *   K1 -> PB11
 *   K2 -> PB10
 *   K3 -> PA11
 *   K4 -> PA12
 * Keys are active-low.
 */
void Key_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

/* 获取一次按键事件。读取后 Key_Num 自动清零，所以同一次按下只会返回一次。 */
uint8_t Key_GetNum(void)
{
    uint8_t Temp;
    if (Key_Num)
    {
        Temp = Key_Num;
        Key_Num = 0;
        return Temp;
    }
    return 0;
}

uint8_t Key_GetState(void)
{
    if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11) == 0)
    {
        return 1;
    }
    if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_10) == 0)
    {
        return 2;
    }
    if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_11) == 0)
    {
        return 3;
    }
    if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_12) == 0)
    {
        return 4;
    }
    return 0;
}

/* 1ms 调用一次的按键消抖状态机。 */
void Key_Tick(void)
{
    static uint8_t Count;
    static uint8_t CurrState, PrevState;

    Count++;
    if (Count >= 20)
    {
        Count = 0;
        PrevState = CurrState;
        CurrState = Key_GetState();

        if (CurrState == 0 && PrevState != 0)
        {
            Key_Num = PrevState;
        }
    }
}
