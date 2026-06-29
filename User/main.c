#include "stm32f10x.h"
#include "OLED.h"
#include "Timer.h"
#include "Key.h"
#include "Motor.h"
#include "Encoder.h"
#include "Serial.h"
#include "Grayscale.h"
#include "BeepLed.h"
#include "../App/app_control.h"
#include "../App/app_line.h"
#include "../App/app_e_car.h"
#include <stdint.h>

#define CONTROL_PERIOD_MS          10U
#define OLED_REFRESH_PERIOD_MS     200U

volatile uint8_t g_flag_10ms = 0U;
volatile uint8_t g_oledRefreshFlag = 0U;

static volatile uint16_t g_oledRefreshMs = 0U;

int main(void)
{
    OLED_Init();
    Key_Init();
    Grayscale_Init();
    Motor_Init();
    Encoder_Init();
    App_Line_GPIOForceInit();
    BeepLed_Init();
    Serial_Init();
    App_Control_Init();
    ECar_Init();
    Timer_Init();

    OLED_Clear();
    ECar_ShowStatus();

    while (1)
    {
        if (g_flag_10ms)
        {
            g_flag_10ms = 0U;
            ECar_Control10ms();
        }

        ECar_KeyProcess();

        if (g_oledRefreshFlag)
        {
            g_oledRefreshFlag = 0U;
            ECar_ShowStatus();
        }
    }
}

void TIM1_UP_IRQHandler(void)
{
    static uint8_t controlDiv = 0U;

    if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
    {
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);

        Key_Tick();
        ECar_PromptTick1ms();

        controlDiv++;
        if (controlDiv >= CONTROL_PERIOD_MS)
        {
            controlDiv = 0U;
            g_flag_10ms = 1U;
        }

        if (g_oledRefreshMs < OLED_REFRESH_PERIOD_MS)
        {
            g_oledRefreshMs++;
        }
        else
        {
            g_oledRefreshMs = 0U;
            g_oledRefreshFlag = 1U;
        }
    }
}
