/* 项目适配说明：八路灰度循迹模块通过 AD0/AD1/AD2 选通，OUT 读取当前通道，用于黑色弧线循迹。 */
/*
 * 文件：Hardware/Grayscale.c
 * 作用：八路灰度循迹传感器读取。
 *
 * 逻辑说明：
 *   传感器只有一个 OUT 输出，通过 AD0/AD1/AD2 三根地址线选择 X1~X8 通道。
 *   Grayscale_ReadAll() 会依次选择 8 个通道并读取 OUT，供 main.c 计算循迹误差。
 *
 * 注释版说明：仅增加注释，未改变读取顺序和延时。
 */
#include "stm32f10x.h"
#include "Grayscale.h"

/*
 * Training board 8-channel grayscale module pin map:
 *   AD0 -> PA8
 *   AD1 -> PA15
 *   AD2 -> PB3
 *   OUT -> PB0
 * PB3/PA15 are JTAG pins by default, so JTAG is disabled and SWD remains enabled.
 */
#define GS_AD0_PORT GPIOA
#define GS_AD0_PIN  GPIO_Pin_8
#define GS_AD1_PORT GPIOA
#define GS_AD1_PIN  GPIO_Pin_15
#define GS_AD2_PORT GPIOB
#define GS_AD2_PIN  GPIO_Pin_3
#define GS_OUT_PORT GPIOB
#define GS_OUT_PIN  GPIO_Pin_0

/* 通过 AD0/AD1/AD2 选择当前要读取的灰度通道，channel 范围 0~7。 */
static void Grayscale_Select(uint8_t channel)
{
    GPIO_WriteBit(GS_AD0_PORT, GS_AD0_PIN, (BitAction)((channel >> 0) & 0x01));
    GPIO_WriteBit(GS_AD1_PORT, GS_AD1_PIN, (BitAction)((channel >> 1) & 0x01));
    GPIO_WriteBit(GS_AD2_PORT, GS_AD2_PIN, (BitAction)((channel >> 2) & 0x01));
}

void Grayscale_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_InitStructure.GPIO_Pin = GS_AD0_PIN | GS_AD1_PIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GS_AD2_PIN;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Pin = GS_OUT_PIN;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    Grayscale_Select(0);
}

uint8_t Grayscale_RawOUT(void)
{
    return (uint8_t)GPIO_ReadInputDataBit(GS_OUT_PORT, GS_OUT_PIN);
}

/* 读取单个通道：先切换地址，短暂等待模拟开关稳定，再读 OUT。 */
uint8_t Grayscale_ReadOne(uint8_t channel)
{
    if (channel >= GRAYSCALE_CHANNELS)
    {
        channel = 0;
    }

    Grayscale_Select(channel);
    for (volatile uint16_t d = 0; d < 800; d++) { __NOP(); }
    return Grayscale_RawOUT();
}

/* 连续读取 X1~X8，结果存入 sensor[0]~sensor[7]。 */
void Grayscale_ReadAll(uint8_t sensor[GRAYSCALE_CHANNELS])
{
    uint8_t i;
    for (i = 0; i < GRAYSCALE_CHANNELS; i++)
    {
        sensor[i] = Grayscale_ReadOne(i);
    }
}
