/*
 * 文件：Hardware/Encoder.c
 * 作用：左右编码器读取。
 *
 * 逻辑说明：
 *   训练板编码器引脚不是完整的硬件编码器定时器组合，因此这里使用 EXTI 双边沿中断
 *   对 A/B 相状态进行四倍频解码。每 10 ms 主控制中断读取一次增量并清零。
 *
 * 注释版说明：仅增加注释，未改变解码表、引脚和中断逻辑。
 */
#include "stm32f10x.h"
#include "Encoder.h"

/*
 * Training board encoder pin map:
 *   Left motor : E1A -> PA7, E1B -> PB1
 *   Right motor: E2A -> PB5, E2B -> PB6
 * These pins are not a complete TIM encoder pair on this board, so a 4-edge
 * EXTI quadrature decoder is used instead of TIM3/TIM4 encoder mode.
 */
/* 若车轮正转时 OLED 显示速度为负，只改这里的方向符号。 */
#define LEFT_ENCODER_SIGN   (-1)
#define RIGHT_ENCODER_SIGN  (+1)

static volatile int16_t LeftDelta = 0;
static volatile int16_t RightDelta = 0;
static volatile uint8_t LeftLast = 0;
static volatile uint8_t RightLast = 0;

static uint8_t Encoder_ReadLeftState(void)
{
    uint8_t a = (uint8_t)GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_7);
    uint8_t b = (uint8_t)GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1);
    return (uint8_t)((a << 1) | b);
}

static uint8_t Encoder_ReadRightState(void)
{
    uint8_t a = (uint8_t)GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_5);
    uint8_t b = (uint8_t)GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_6);
    return (uint8_t)((a << 1) | b);
}

/* 四相编码器状态转移表：非法跳变记为 0，正常相邻跳变记为 +/-1。 */
static int8_t Encoder_Decode(uint8_t last, uint8_t now)
{
    static const int8_t table[16] =
    {
         0, +1, -1,  0,
        -1,  0,  0, +1,
        +1,  0,  0, -1,
         0, -1, +1,  0
    };
    return table[((last & 0x03) << 2) | (now & 0x03)];
}

static void Encoder_UpdateLeft(void)
{
    uint8_t now = Encoder_ReadLeftState();
    LeftDelta += (int16_t)(Encoder_Decode(LeftLast, now) * LEFT_ENCODER_SIGN);
    LeftLast = now;
}

static void Encoder_UpdateRight(void)
{
    uint8_t now = Encoder_ReadRightState();
    RightDelta += (int16_t)(Encoder_Decode(RightLast, now) * RIGHT_ENCODER_SIGN);
    RightLast = now;
}

void Encoder_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_5 | GPIO_Pin_6;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource7);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource1);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource5);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource6);

    EXTI_InitTypeDef EXTI_InitStructure;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;

    EXTI_InitStructure.EXTI_Line = EXTI_Line1;
    EXTI_Init(&EXTI_InitStructure);
    EXTI_InitStructure.EXTI_Line = EXTI_Line5;
    EXTI_Init(&EXTI_InitStructure);
    EXTI_InitStructure.EXTI_Line = EXTI_Line6;
    EXTI_Init(&EXTI_InitStructure);
    EXTI_InitStructure.EXTI_Line = EXTI_Line7;
    EXTI_Init(&EXTI_InitStructure);

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
    NVIC_Init(&NVIC_InitStructure);

    LeftLast = Encoder_ReadLeftState();
    RightLast = Encoder_ReadRightState();
    Encoder_ClearAll();
}

/* 读取左轮过去一个控制周期内的增量，并立即清零。 */
int16_t Encoder_GetLeftDelta(void)
{
    int16_t temp;
    __disable_irq();
    temp = LeftDelta;
    LeftDelta = 0;
    __enable_irq();
    return temp;
}

int16_t Encoder_GetRightDelta(void)
{
    int16_t temp;
    __disable_irq();
    temp = RightDelta;
    RightDelta = 0;
    __enable_irq();
    return temp;
}

void Encoder_ClearAll(void)
{
    __disable_irq();
    LeftDelta = 0;
    RightDelta = 0;
    __enable_irq();
}

void EXTI1_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line1) != RESET)
    {
        Encoder_UpdateLeft();
        EXTI_ClearITPendingBit(EXTI_Line1);
    }
}

/* EXTI5/6/7 共用中断入口，分别处理右轮 B5/B6 和左轮 PA7。 */
void EXTI9_5_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line7) != RESET)
    {
        Encoder_UpdateLeft();
        EXTI_ClearITPendingBit(EXTI_Line7);
    }

    if (EXTI_GetITStatus(EXTI_Line5) != RESET)
    {
        Encoder_UpdateRight();
        EXTI_ClearITPendingBit(EXTI_Line5);
    }

    if (EXTI_GetITStatus(EXTI_Line6) != RESET)
    {
        Encoder_UpdateRight();
        EXTI_ClearITPendingBit(EXTI_Line6);
    }
}
