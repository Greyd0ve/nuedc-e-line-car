#include "app_line.h"
#include "stm32f10x.h"
#include <stdint.h>

#ifndef GRAYSCALE_CHANNELS
#define GRAYSCALE_CHANNELS 8U
#endif

#define APP_LINE_CONTROL_PERIOD_MS 10U

#define LINE_AD0_PORT      GPIOA
#define LINE_AD0_PIN       GPIO_Pin_8
#define LINE_AD1_PORT      GPIOB
#define LINE_AD1_PIN       GPIO_Pin_3
#define LINE_AD2_PORT      GPIOB
#define LINE_AD2_PIN       GPIO_Pin_4
#define LINE_OUT_PORT      GPIOB
#define LINE_OUT_PIN       GPIO_Pin_0

extern volatile float g_lineBlackLevelF;
extern volatile float g_lineReverseOrderF;
extern volatile float g_lineTurnSign;
extern volatile float g_lineKp;
extern volatile float g_lineKd;
extern volatile float g_lineTurnLimit;
extern volatile float g_lineFilterAlpha;

extern volatile int16_t g_lineError;
extern volatile uint8_t g_lineValid;
extern volatile uint8_t g_lineMask;
extern volatile uint8_t g_lineRawMask;
extern volatile int8_t g_lastLineDir;
extern volatile uint16_t g_lineLostMs;

static volatile float g_lineErrorFiltered = 0.0f;
static volatile float g_lineLastCtrlError = 0.0f;

static float App_Line_LimitFloat(float value, float minVal, float maxVal)
{
    if (value < minVal) return minVal;
    if (value > maxVal) return maxVal;
    return value;
}

void App_Line_GPIOForceInit(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = LINE_AD0_PIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = LINE_AD1_PIN | LINE_AD2_PIN;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Pin = LINE_OUT_PIN;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_ResetBits(LINE_AD0_PORT, LINE_AD0_PIN);
    GPIO_ResetBits(LINE_AD1_PORT, LINE_AD1_PIN);
    GPIO_ResetBits(LINE_AD2_PORT, LINE_AD2_PIN);
}

void App_Line_ResetState(void)
{
    g_lineLostMs = 0;
    g_lineErrorFiltered = 0.0f;
    g_lineLastCtrlError = 0.0f;
}

static void App_Line_SelectChannelDirect(uint8_t channel)
{
    volatile uint16_t d;

    if (channel & 0x01U) GPIO_SetBits(LINE_AD0_PORT, LINE_AD0_PIN);
    else GPIO_ResetBits(LINE_AD0_PORT, LINE_AD0_PIN);

    if (channel & 0x02U) GPIO_SetBits(LINE_AD1_PORT, LINE_AD1_PIN);
    else GPIO_ResetBits(LINE_AD1_PORT, LINE_AD1_PIN);

    if (channel & 0x04U) GPIO_SetBits(LINE_AD2_PORT, LINE_AD2_PIN);
    else GPIO_ResetBits(LINE_AD2_PORT, LINE_AD2_PIN);

    for (d = 0; d < 2000; d++) __NOP();
}

static uint8_t App_Line_ReadOneDirect(uint8_t channel)
{
    uint8_t a;
    uint8_t b;
    uint8_t c;
    volatile uint16_t d;

    App_Line_SelectChannelDirect(channel);
    a = (uint8_t)GPIO_ReadInputDataBit(LINE_OUT_PORT, LINE_OUT_PIN);
    for (d = 0; d < 300; d++) __NOP();
    b = (uint8_t)GPIO_ReadInputDataBit(LINE_OUT_PORT, LINE_OUT_PIN);
    for (d = 0; d < 300; d++) __NOP();
    c = (uint8_t)GPIO_ReadInputDataBit(LINE_OUT_PORT, LINE_OUT_PIN);

    return (uint8_t)(((uint16_t)a + (uint16_t)b + (uint16_t)c) >= 2U);
}

static void App_Line_ReadAllDirect(uint8_t raw[GRAYSCALE_CHANNELS])
{
    uint8_t i;
    for (i = 0; i < GRAYSCALE_CHANNELS; i++) raw[i] = App_Line_ReadOneDirect(i);
}

void App_Line_Update(void)
{
    uint8_t raw[GRAYSCALE_CHANNELS];
    static const int16_t weight[GRAYSCALE_CHANNELS] = {-350, -250, -150, -50, 50, 150, 250, 350};
    int32_t sum = 0;
    int16_t count = 0;
    uint8_t mask = 0;
    uint8_t i;
    uint8_t blackLevel;
    uint8_t reverseOrder;

    blackLevel = (g_lineBlackLevelF <= 0.5f) ? 0U : 1U;
    reverseOrder = (g_lineReverseOrderF <= 0.5f) ? 0U : 1U;

    App_Line_ReadAllDirect(raw);

    g_lineRawMask = 0;
    for (i = 0; i < GRAYSCALE_CHANNELS; i++)
    {
        if (raw[i] == blackLevel) g_lineRawMask |= (uint8_t)(1U << i);
    }

    for (i = 0; i < GRAYSCALE_CHANNELS; i++)
    {
        uint8_t physicalIndex;
        uint8_t isBlack;

        physicalIndex = reverseOrder ? (uint8_t)(GRAYSCALE_CHANNELS - 1U - i) : i;
        isBlack = (raw[physicalIndex] == blackLevel) ? 1U : 0U;

        if (isBlack)
        {
            mask |= (uint8_t)(1U << i);
            sum += weight[i];
            count++;
        }
    }

    g_lineMask = mask;

    if (count > 0)
    {
        float rawError;
        float alpha;

        g_lineValid = 1;
        rawError = (float)(sum / count);
        alpha = App_Line_LimitFloat(g_lineFilterAlpha, 0.0f, 1.0f);
        g_lineErrorFiltered = g_lineErrorFiltered * (1.0f - alpha) + rawError * alpha;
        g_lineError = (int16_t)g_lineErrorFiltered;
        g_lastLineDir = (g_lineError >= 0) ? 1 : -1;
        g_lineLostMs = 0;
    }
    else
    {
        g_lineValid = 0;
        if (g_lineLostMs < 60000U) g_lineLostMs += APP_LINE_CONTROL_PERIOD_MS;
    }
}

float App_Line_CalcTurnCmd(void)
{
    float error;
    float dError;
    float turn;

    error = (float)g_lineError;
    dError = error - g_lineLastCtrlError;
    g_lineLastCtrlError = error;

    turn = (-g_lineTurnSign) * (error * g_lineKp + dError * g_lineKd);

    return App_Line_LimitFloat(turn, -g_lineTurnLimit, g_lineTurnLimit);
}
