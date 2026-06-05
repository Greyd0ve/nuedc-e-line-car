/*
 * 文件：Hardware/Key.c
 * 作用：K1~K4 按键扫描和消抖。
 *
 * 真实 PCB 按键连接：
 *   K1 -> PB10
 *   K2 -> PB11
 *   K3 -> PA11
 *   K4 -> PA12
 *
 * 逻辑说明：
 *   1. 四个按键均为低电平有效。
 *   2. Key_Tick() 在 TIM1 1ms 中断中周期调用，完成简单消抖。
 *   3. Key_GetNum() 在 main.c 主循环中读取一次性按键事件。
 *   4. 读取后 Key_Num 自动清零，所以同一次按下只返回一次。
 *
 * 返回值：
 *   K1 按下释放后 -> Key_GetNum() 返回 1
 *   K2 按下释放后 -> Key_GetNum() 返回 2
 *   K3 按下释放后 -> Key_GetNum() 返回 3
 *   K4 按下释放后 -> Key_GetNum() 返回 4
 */
#include "stm32f10x.h"
#include "Delay.h"

uint8_t Key_Num;

void Key_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    /* K1=PB10，K2=PB11。 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* K3=PA11，K4=PA12。 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

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
    /* 注意：这里按真实 PCB 连接修改了 PB10/PB11 对应关系。
     * 旧代码是 PB11->K1、PB10->K2；现在修正为 PB10->K1、PB11->K2。
     */
    if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_10) == 0)
    {
        return 1;
    }
    if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11) == 0)
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

void Key_Tick(void)
{
    static uint8_t Count;
    static uint8_t CurrState;
    static uint8_t PrevState;

    Count++;
    if (Count >= 20)
    {
        Count = 0;
        PrevState = CurrState;
        CurrState = Key_GetState();

        /* 按键释放瞬间上报一次事件。 */
        if (CurrState == 0 && PrevState != 0)
        {
            Key_Num = PrevState;
        }
    }
}
