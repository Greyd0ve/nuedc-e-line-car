/*
 * File: User/main.c
 * Version: Bluetooth + 8-channel grayscale tracing + direct grayscale GPIO read
 *          + presetFast + PB1/PB5 prompt output version.
 *
 * Changes in this version:
 *   1. PC13 board LED is no longer used.
 *   2. PB1 is used as BEEP output. The buzzer is active-low.
 *   3. PB5 is used as LED_EXT output. The external LED is active-high.
 *   4. Existing Prompt_Start()/Prompt_Tick1ms() now drive PB1 and PB5 only.
 *
 * Existing behavior kept:
 *   1. Power-on default mode is Bluetooth remote mode.
 *   2. [key,tracing,down] enters line tracing mode.
 *   3. [key,Bluetooth,down] returns to Bluetooth remote mode.
 *   4. [key,emergency,down] stops PWM immediately and enters safety lock.
 *   5. [key,unlock,down] releases the safety lock.
 *   6. [slider,RP,0~100] controls maximum PWM percent.
 *   7. SW1/K1 sets RP to 0%, clears PWM, but does not enter safety lock.
 *   8. SW2/K2 switches between Bluetooth mode and tracing mode; locked state blocks it.
 *   9. [key,presetFast,down] loads fast stable preset v1.
 *   10. SW3/K3 loads fast stable preset v1.
 */

#include "stm32f10x.h"
#include "OLED.h"
#include "Timer.h"
#include "Key.h"
#include "Motor.h"
#include "Encoder.h"
#include "Serial.h"
#include "Grayscale.h"
#include "PWM.h"
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

/* ================================================================
 * 1. Basic timing and safety parameters
 * ================================================================ */

#define CONTROL_PERIOD_MS              10U
#define BT_TIMEOUT_MS                  600U
#define PLOT_REPORT_PERIOD_MS          100U
#define OLED_REFRESH_PERIOD_MS         100U

#define PWM_LIMIT_MIN                  0.0f
#define PWM_LIMIT_MAX                  ((float)PWM_MAX_DUTY)

/* ================================================================
 * 2. Line tracing default parameters
 * ================================================================ */

volatile float g_lineBlackLevelF = 1.0f;
volatile float g_lineReverseOrderF = 0.0f;
volatile float g_lineTurnSign = 1.0f;

volatile float g_traceBaseSpeed = 24.0f;
volatile float g_traceSearchSpeed = 8.0f;

volatile float g_lineKp = 0.135f;
volatile float g_lineKd = 0.055f;
volatile float g_lineTurnLimit = 90.0f;
volatile float g_lineLostTurn = 72.0f;
volatile float g_lineFilterAlpha = 0.65f;
volatile float g_lineSlowGain = 0.55f;
volatile float g_lineEdgeTurnExtra = 22.0f;
volatile float g_lineEdgeSpeedRatio = 0.55f;
volatile float g_lineMinTurn = 18.0f;

volatile float g_forwardSlewStep = 3.0f;
volatile float g_turnSlewStep = 14.0f;

/* ================================================================
 * 3. Speed loop and turn loop PID parameters
 * ================================================================ */

typedef struct
{
    float Kp;
    float Ki;
    float Kd;
    float Integral;
    float LastError;
    float OutputLimit;
    float IntegralLimit;
} PID_TypeDef;

static PID_TypeDef ForwardPID;
static PID_TypeDef TurnPID;

volatile float g_forwardKp = 14.0f;
volatile float g_forwardKi = 0.55f;
volatile float g_forwardKd = 1.8f;

volatile float g_turnKp = 10.0f;
volatile float g_turnKi = 0.04f;
volatile float g_turnKd = 1.0f;

volatile float g_maxForwardCmd = 70.0f;
volatile float g_maxTurnCmd = 75.0f;

/* ================================================================
 * 4. Runtime states
 * ================================================================ */

typedef enum
{
    WORK_BT = 0,
    WORK_TRACING = 1
} WorkMode_t;

volatile WorkMode_t g_workMode = WORK_BT;

volatile float g_targetForwardSpeed = 0.0f;
volatile float g_targetTurnSpeed = 0.0f;

volatile uint8_t g_carEnable = 0;
volatile uint32_t g_lastCmdTickMs = 1000;
volatile uint8_t g_safetyLocked = 0;

volatile float g_btSpeedLimitPercent = 30.0f;
volatile float g_speedScale = 0.30f;
volatile float g_pwmLimit = 300.0f;

volatile float g_leftSpeed = 0.0f;
volatile float g_rightSpeed = 0.0f;
volatile float g_forwardSpeed = 0.0f;
volatile float g_turnSpeed = 0.0f;

volatile float g_speedPwm = 0.0f;
volatile float g_diffPwm = 0.0f;

volatile int16_t g_leftPwm = 0;
volatile int16_t g_rightPwm = 0;
volatile float g_forwardSpeedError = 0.0f;

/* Kept for compatibility with old BT_proto.c. */
volatile uint8_t g_sendPlot = 0;
volatile uint16_t g_sendDisplay = 0;

volatile uint16_t g_oledRefreshMs = 0;
volatile uint16_t g_plotReportMs = 0;

volatile int16_t g_lineError = 0;
volatile uint8_t g_lineValid = 0;
volatile uint8_t g_lineMask = 0;
volatile uint8_t g_lineRawMask = 0;
volatile int8_t g_lastLineDir = 1;
volatile uint16_t g_lineLostMs = 0;

static volatile float g_lineErrorFiltered = 0.0f;
static volatile float g_lineLastCtrlError = 0.0f;
static volatile uint16_t g_promptMs = 0;

/* ================================================================
 * 5. Utility functions
 * ================================================================ */

static float absf_local(float x)
{
    return (x >= 0.0f) ? x : -x;
}

static float limit_float(float value, float minVal, float maxVal)
{
    if (value < minVal)
    {
        return minVal;
    }
    if (value > maxVal)
    {
        return maxVal;
    }
    return value;
}

static int16_t limit_i16(int32_t value, int16_t minVal, int16_t maxVal)
{
    if (value < minVal)
    {
        return minVal;
    }
    if (value > maxVal)
    {
        return maxVal;
    }
    return (int16_t)value;
}

static float slew_float(float current, float target, float maxStep)
{
    if (maxStep <= 0.0f)
    {
        return target;
    }
    if (target > current + maxStep)
    {
        return current + maxStep;
    }
    if (target < current - maxStep)
    {
        return current - maxStep;
    }
    return target;
}

static char ascii_lower_char(char c)
{
    if (c >= 'A' && c <= 'Z')
    {
        return (char)(c + ('a' - 'A'));
    }
    return c;
}

static int str_equal_ignore_case(const char *a, const char *b)
{
    while (*a != '\0' && *b != '\0')
    {
        if (ascii_lower_char(*a) != ascii_lower_char(*b))
        {
            return 0;
        }
        a++;
        b++;
    }
    return (*a == '\0' && *b == '\0');
}

static int str_is_down(const char *s)
{
    return str_equal_ignore_case(s, "down") || str_equal_ignore_case(s, "d");
}

static int str_is_name(const char *s, const char *a, const char *b, const char *c)
{
    if (str_equal_ignore_case(s, a))
    {
        return 1;
    }
    if (b && str_equal_ignore_case(s, b))
    {
        return 1;
    }
    if (c && str_equal_ignore_case(s, c))
    {
        return 1;
    }
    return 0;
}

/* ================================================================
 * 6. PB1 buzzer and PB5 external LED prompt output
 * ================================================================ */

#define PROMPT_BEEP_PORT       GPIOB
#define PROMPT_BEEP_PIN        GPIO_Pin_1
#define PROMPT_LED_PORT        GPIOB
#define PROMPT_LED_PIN         GPIO_Pin_5

static void PromptIO_BeepOn(void)
{
    /* Active-low buzzer: PB1 = 0 means ON. */
    GPIO_ResetBits(PROMPT_BEEP_PORT, PROMPT_BEEP_PIN);
}

static void PromptIO_BeepOff(void)
{
    GPIO_SetBits(PROMPT_BEEP_PORT, PROMPT_BEEP_PIN);
}

static void PromptIO_BeepTurn(void)
{
    if (GPIO_ReadOutputDataBit(PROMPT_BEEP_PORT, PROMPT_BEEP_PIN))
    {
        PromptIO_BeepOn();
    }
    else
    {
        PromptIO_BeepOff();
    }
}

static void PromptIO_LedOn(void)
{
    /* Active-high LED: PB5 = 1 means ON. */
    GPIO_SetBits(PROMPT_LED_PORT, PROMPT_LED_PIN);
}

static void PromptIO_LedOff(void)
{
    GPIO_ResetBits(PROMPT_LED_PORT, PROMPT_LED_PIN);
}

static void PromptIO_LedTurn(void)
{
    if (GPIO_ReadOutputDataBit(PROMPT_LED_PORT, PROMPT_LED_PIN))
    {
        PromptIO_LedOff();
    }
    else
    {
        PromptIO_LedOn();
    }
}

static void PromptIO_AllOn(void)
{
    PromptIO_BeepOn();
    PromptIO_LedOn();
}

static void PromptIO_AllOff(void)
{
    PromptIO_BeepOff();
    PromptIO_LedOff();
}

static void PromptIO_AllTurn(void)
{
    PromptIO_BeepTurn();
    PromptIO_LedTurn();
}

static void PromptIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = PROMPT_BEEP_PIN | PROMPT_LED_PIN;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    PromptIO_AllOff();
}

/* ================================================================
 * 7. PID and stop control
 * ================================================================ */

static void PID_Reset(PID_TypeDef *pid)
{
    pid->Integral = 0.0f;
    pid->LastError = 0.0f;
}

static float PID_Calc(PID_TypeDef *pid, float target, float measure)
{
    float error;
    float derivative;
    float integralCandidate;
    float output;

    error = target - measure;
    derivative = error - pid->LastError;
    integralCandidate = pid->Integral + error;
    integralCandidate = limit_float(integralCandidate, -pid->IntegralLimit, pid->IntegralLimit);

    output = pid->Kp * error + pid->Ki * integralCandidate + pid->Kd * derivative;

    if (output > pid->OutputLimit)
    {
        output = pid->OutputLimit;
        if (error < 0.0f)
        {
            pid->Integral = integralCandidate;
        }
    }
    else if (output < -pid->OutputLimit)
    {
        output = -pid->OutputLimit;
        if (error > 0.0f)
        {
            pid->Integral = integralCandidate;
        }
    }
    else
    {
        pid->Integral = integralCandidate;
    }

    pid->LastError = error;
    return output;
}

static void Control_Init(void)
{
    ForwardPID.Kp = g_forwardKp;
    ForwardPID.Ki = g_forwardKi;
    ForwardPID.Kd = g_forwardKd;
    ForwardPID.Integral = 0.0f;
    ForwardPID.LastError = 0.0f;
    ForwardPID.OutputLimit = (float)PWM_MAX_DUTY;
    ForwardPID.IntegralLimit = 260.0f;

    TurnPID.Kp = g_turnKp;
    TurnPID.Ki = g_turnKi;
    TurnPID.Kd = g_turnKd;
    TurnPID.Integral = 0.0f;
    TurnPID.LastError = 0.0f;
    TurnPID.OutputLimit = (float)PWM_MAX_DUTY * 0.85f;
    TurnPID.IntegralLimit = 220.0f;
}

static void Control_UpdatePIDParam(void)
{
    ForwardPID.Kp = g_forwardKp;
    ForwardPID.Ki = g_forwardKi;
    ForwardPID.Kd = g_forwardKd;

    TurnPID.Kp = g_turnKp;
    TurnPID.Ki = g_turnKi;
    TurnPID.Kd = g_turnKd;
}

static void Control_ForcePWMZero(void)
{
    g_targetForwardSpeed = 0.0f;
    g_targetTurnSpeed = 0.0f;
    g_speedPwm = 0.0f;
    g_diffPwm = 0.0f;
    g_leftPwm = 0;
    g_rightPwm = 0;
    g_carEnable = 0;
    Motor_StopAll();
    PID_Reset(&ForwardPID);
    PID_Reset(&TurnPID);
}

static void Prompt_Start(uint16_t ms)
{
    g_promptMs = ms;
    PromptIO_AllOn();
}

static void Prompt_Tick1ms(void)
{
    if (g_promptMs > 0)
    {
        g_promptMs--;
        if ((g_promptMs % 80U) == 0U)
        {
            PromptIO_AllTurn();
        }
        if (g_promptMs == 0U)
        {
            PromptIO_AllOff();
        }
    }
}

/* ================================================================
 * 8. Mode switching and safety lock
 * ================================================================ */

void App_EmergencyStop(void)
{
    g_safetyLocked = 1;
    g_workMode = WORK_BT;
    Control_ForcePWMZero();
    Prompt_Start(500);
}

void App_UnlockControl(void)
{
    g_safetyLocked = 0;
    g_workMode = WORK_BT;
    g_lastCmdTickMs = 0;
    Control_ForcePWMZero();
    Prompt_Start(180);
}

void App_StartBluetoothMode(void)
{
    if (g_safetyLocked)
    {
        return;
    }

    g_workMode = WORK_BT;
    g_lastCmdTickMs = 0;
    Control_ForcePWMZero();
    Prompt_Start(160);
}

void App_StartTracingMode(void)
{
    if (g_safetyLocked)
    {
        return;
    }

    g_workMode = WORK_TRACING;
    g_lineLostMs = 0;
    g_lineErrorFiltered = 0.0f;
    g_lineLastCtrlError = 0.0f;
    g_targetForwardSpeed = 0.0f;
    g_targetTurnSpeed = 0.0f;
    g_carEnable = 1;
    g_lastCmdTickMs = 0;
    Encoder_ClearAll();
    PID_Reset(&ForwardPID);
    PID_Reset(&TurnPID);
    Prompt_Start(160);
}

/* ================================================================
 * 9. Bluetooth/web packet parser
 * ================================================================ */

static void ApplySpeedLimitPercent(float percent)
{
    float ratio;

    percent = limit_float(percent, 0.0f, 100.0f);
    ratio = percent / 100.0f;

    g_btSpeedLimitPercent = percent;
    g_speedScale = ratio;
    g_pwmLimit = PWM_LIMIT_MAX * ratio;
}

static void ApplyFastPreset(void)
{
    ApplySpeedLimitPercent(55.0f);

    g_traceBaseSpeed = 60.0f;
    g_lineKp = 0.350f;
    g_lineKd = 0.600f;
    g_lineTurnLimit = 180.0f;
    g_lineMinTurn = 34.0f;
    g_lineFilterAlpha = 0.58f;
    g_lineSlowGain = 0.88f;
    g_lineEdgeTurnExtra = 82.0f;
    g_lineEdgeSpeedRatio = 0.24f;
    g_forwardSlewStep = 14.0f;
    g_turnSlewStep = 60.0f;
    g_lineLostTurn = 130.0f;

    g_lineTurnSign = 1.0f;
    g_lineBlackLevelF = 1.0f;
    g_lineReverseOrderF = 0.0f;

    g_lineLostMs = 0;
    g_lineErrorFiltered = 0.0f;
    g_lineLastCtrlError = 0.0f;
    PID_Reset(&ForwardPID);
    PID_Reset(&TurnPID);
    Prompt_Start(220);
}

static void Main_ApplySliderPacket(const char *name, float value)
{
    if (str_is_name(name, "RP", "rp", "speedLimit"))
    {
        ApplySpeedLimitPercent(value);
        return;
    }

    if (str_is_name(name, "speedKp", "forwardKp", "fKp"))
    {
        g_forwardKp = limit_float(value, 0.0f, 80.0f);
        return;
    }
    if (str_is_name(name, "speedKi", "forwardKi", "fKi"))
    {
        g_forwardKi = limit_float(value, 0.0f, 20.0f);
        return;
    }
    if (str_is_name(name, "speedKd", "forwardKd", "fKd"))
    {
        g_forwardKd = limit_float(value, 0.0f, 30.0f);
        return;
    }

    if (str_is_name(name, "turnKp", "diffKp", "tKp"))
    {
        g_turnKp = limit_float(value, 0.0f, 80.0f);
        return;
    }
    if (str_is_name(name, "turnKi", "diffKi", "tKi"))
    {
        g_turnKi = limit_float(value, 0.0f, 20.0f);
        return;
    }
    if (str_is_name(name, "turnKd", "diffKd", "tKd"))
    {
        g_turnKd = limit_float(value, 0.0f, 30.0f);
        return;
    }

    if (str_is_name(name, "maxForward", "maxSpeed", "btSpeed"))
    {
        g_maxForwardCmd = limit_float(value, 0.0f, 200.0f);
        return;
    }
    if (str_is_name(name, "maxTurn", "btTurn", "remoteTurn"))
    {
        g_maxTurnCmd = limit_float(value, 0.0f, 200.0f);
        return;
    }

    if (str_is_name(name, "traceKp", "lineKp", "lineP"))
    {
        g_lineKp = limit_float(value, 0.0f, 1.0f);
        return;
    }
    if (str_is_name(name, "traceKd", "lineKd", "lineD"))
    {
        g_lineKd = limit_float(value, 0.0f, 1.0f);
        return;
    }
    if (str_is_name(name, "traceSpeed", "lineSpeed", "baseSpeed"))
    {
        g_traceBaseSpeed = limit_float(value, 0.0f, 120.0f);
        return;
    }
    if (str_is_name(name, "searchSpeed", "lostSpeed", "findSpeed"))
    {
        g_traceSearchSpeed = limit_float(value, 0.0f, 80.0f);
        return;
    }
    if (str_is_name(name, "turnLimit", "lineTurnLimit", "traceTurnLimit"))
    {
        g_lineTurnLimit = limit_float(value, 0.0f, 180.0f);
        return;
    }
    if (str_is_name(name, "lostTurn", "lineLostTurn", "findTurn"))
    {
        g_lineLostTurn = limit_float(value, 0.0f, 180.0f);
        return;
    }
    if (str_is_name(name, "filter", "lineFilter", "alpha"))
    {
        if (value > 1.0f)
        {
            value = value / 100.0f;
        }
        g_lineFilterAlpha = limit_float(value, 0.0f, 1.0f);
        return;
    }
    if (str_is_name(name, "slowGain", "lineSlow", "curveSlow"))
    {
        if (value > 1.0f)
        {
            value = value / 100.0f;
        }
        g_lineSlowGain = limit_float(value, 0.0f, 0.95f);
        return;
    }
    if (str_is_name(name, "edgeBoost", "edgeTurn", "edgeExtra"))
    {
        g_lineEdgeTurnExtra = limit_float(value, 0.0f, 100.0f);
        return;
    }
    if (str_is_name(name, "edgeSlow", "edgeSpeed", "edgeRatio"))
    {
        if (value > 1.0f)
        {
            value = value / 100.0f;
        }
        g_lineEdgeSpeedRatio = limit_float(value, 0.05f, 1.0f);
        return;
    }
    if (str_is_name(name, "minTurn", "lineMinTurn", "traceMinTurn"))
    {
        g_lineMinTurn = limit_float(value, 0.0f, 80.0f);
        return;
    }
    if (str_is_name(name, "forwardSlew", "speedSlew", "fSlew"))
    {
        g_forwardSlewStep = limit_float(value, 0.0f, 30.0f);
        return;
    }
    if (str_is_name(name, "turnSlew", "diffSlew", "tSlew"))
    {
        g_turnSlewStep = limit_float(value, 0.0f, 60.0f);
        return;
    }
    if (str_is_name(name, "lineSign", "traceSign", "turnSign"))
    {
        g_lineTurnSign = (value < 0.0f) ? -1.0f : 1.0f;
        return;
    }
    if (str_is_name(name, "blackLevel", "lineLevel", "black"))
    {
        g_lineBlackLevelF = (value <= 0.0f) ? 0.0f : 1.0f;
        return;
    }
    if (str_is_name(name, "lineReverse", "reverseLine", "sensorReverse"))
    {
        g_lineReverseOrderF = (value <= 0.0f) ? 0.0f : 1.0f;
        return;
    }
}

static void Main_ApplyJoystickPacket(char **tok, int n)
{
    int turnRaw;
    int forwardRaw;
    float maxForward;
    float maxTurn;

    if (n < 3)
    {
        return;
    }
    if (g_safetyLocked || g_workMode != WORK_BT)
    {
        return;
    }

    turnRaw = atoi(tok[1]);
    forwardRaw = atoi(tok[2]);

    maxForward = g_maxForwardCmd * g_speedScale;
    maxTurn = g_maxTurnCmd * g_speedScale;

    g_targetForwardSpeed = limit_float((float)forwardRaw * maxForward / 100.0f, -maxForward, maxForward);
    g_targetTurnSpeed = limit_float((float)(-turnRaw) * maxTurn / 100.0f, -maxTurn, maxTurn);
    g_carEnable = 1;
}

static void Main_ApplyPacket(char *payload)
{
    char *tok[12];
    int n;
    char *p;

    n = 0;
    p = payload;
    tok[n++] = p;
    while (*p != '\0' && n < 12)
    {
        if (*p == ',')
        {
            *p = '\0';
            tok[n++] = p + 1;
        }
        p++;
    }

    if (n <= 0 || tok[0][0] == '\0')
    {
        return;
    }

    if (str_is_name(tok[0], "key", "k", 0))
    {
        if (n >= 3 && str_is_down(tok[2]))
        {
            if (str_is_name(tok[1], "emergency", "emg", "stop"))
            {
                App_EmergencyStop();
                return;
            }
            if (str_is_name(tok[1], "unlock", "release", "resume"))
            {
                App_UnlockControl();
                return;
            }
            if (str_is_name(tok[1], "presetFast", "fast", "fastPreset"))
            {
                ApplyFastPreset();
                return;
            }
            if (str_is_name(tok[1], "tracing", "trace", "line"))
            {
                App_StartTracingMode();
                return;
            }
            if (str_is_name(tok[1], "Bluetooth", "BT", "remote"))
            {
                App_StartBluetoothMode();
                return;
            }
        }
        return;
    }

    if (str_is_name(tok[0], "slider", "s", 0))
    {
        if (n >= 3)
        {
            Main_ApplySliderPacket(tok[1], (float)atof(tok[2]));
        }
        return;
    }

    if (str_is_name(tok[0], "joystick", "j", 0))
    {
        Main_ApplyJoystickPacket(tok, n);
        return;
    }

    if (str_is_name(tok[0], "cmd", "car", "vel"))
    {
        float maxForward;
        float maxTurn;

        if (n >= 3 && !g_safetyLocked && g_workMode == WORK_BT)
        {
            maxForward = g_maxForwardCmd * g_speedScale;
            maxTurn = g_maxTurnCmd * g_speedScale;
            g_targetForwardSpeed = limit_float((float)atof(tok[1]), -maxForward, maxForward);
            g_targetTurnSpeed = limit_float((float)atof(tok[2]), -maxTurn, maxTurn);
            g_carEnable = 1;
        }
        return;
    }
}

static void Main_BTProcess(void)
{
    static uint8_t receiving = 0;
    static uint8_t index = 0;
    static char packet[128];
    uint8_t byte;

    while (Serial_ReadByte(&byte))
    {
        char c;
        c = (char)byte;

        if (c == '[')
        {
            receiving = 1;
            index = 0;
            continue;
        }

        if (!receiving)
        {
            continue;
        }

        if (c == ']')
        {
            packet[index] = '\0';
            receiving = 0;
            Main_ApplyPacket(packet);
            g_lastCmdTickMs = 0;
            continue;
        }

        if (c == '\r' || c == '\n')
        {
            continue;
        }

        if (index < sizeof(packet) - 1U)
        {
            packet[index++] = c;
        }
        else
        {
            receiving = 0;
            index = 0;
        }
    }
}

/* ================================================================
 * 10. 8-channel grayscale tracing with direct GPIO read
 * ================================================================ */

#define LINE_AD0_PORT      GPIOA
#define LINE_AD0_PIN       GPIO_Pin_8

#define LINE_AD1_PORT      GPIOB
#define LINE_AD1_PIN       GPIO_Pin_3

#define LINE_AD2_PORT      GPIOB
#define LINE_AD2_PIN       GPIO_Pin_4

#define LINE_OUT_PORT      GPIOB
#define LINE_OUT_PIN       GPIO_Pin_0

static void Line_GPIOForceInit(void)
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

static void Line_SelectChannelDirect(uint8_t channel)
{
    volatile uint16_t d;

    if (channel & 0x01U)
    {
        GPIO_SetBits(LINE_AD0_PORT, LINE_AD0_PIN);
    }
    else
    {
        GPIO_ResetBits(LINE_AD0_PORT, LINE_AD0_PIN);
    }

    if (channel & 0x02U)
    {
        GPIO_SetBits(LINE_AD1_PORT, LINE_AD1_PIN);
    }
    else
    {
        GPIO_ResetBits(LINE_AD1_PORT, LINE_AD1_PIN);
    }

    if (channel & 0x04U)
    {
        GPIO_SetBits(LINE_AD2_PORT, LINE_AD2_PIN);
    }
    else
    {
        GPIO_ResetBits(LINE_AD2_PORT, LINE_AD2_PIN);
    }

    for (d = 0; d < 2000; d++)
    {
        __NOP();
    }
}

static uint8_t Line_ReadOneDirect(uint8_t channel)
{
    uint8_t a;
    uint8_t b;
    uint8_t c;
    volatile uint16_t d;

    Line_SelectChannelDirect(channel);

    a = (uint8_t)GPIO_ReadInputDataBit(LINE_OUT_PORT, LINE_OUT_PIN);

    for (d = 0; d < 300; d++)
    {
        __NOP();
    }

    b = (uint8_t)GPIO_ReadInputDataBit(LINE_OUT_PORT, LINE_OUT_PIN);

    for (d = 0; d < 300; d++)
    {
        __NOP();
    }

    c = (uint8_t)GPIO_ReadInputDataBit(LINE_OUT_PORT, LINE_OUT_PIN);

    return (uint8_t)(((uint16_t)a + (uint16_t)b + (uint16_t)c) >= 2U);
}

static void Line_ReadAllDirect(uint8_t raw[GRAYSCALE_CHANNELS])
{
    uint8_t i;

    for (i = 0; i < GRAYSCALE_CHANNELS; i++)
    {
        raw[i] = Line_ReadOneDirect(i);
    }
}

static void Line_Update(void)
{
    uint8_t raw[GRAYSCALE_CHANNELS];
    static const int16_t weight[GRAYSCALE_CHANNELS] = {-350, -250, -150, -50, 50, 150, 250, 350};
    int32_t sum;
    int16_t count;
    uint8_t mask;
    uint8_t i;
    uint8_t blackLevel;
    uint8_t reverseOrder;

    sum = 0;
    count = 0;
    mask = 0;
    blackLevel = (g_lineBlackLevelF <= 0.5f) ? 0U : 1U;
    reverseOrder = (g_lineReverseOrderF <= 0.5f) ? 0U : 1U;

    Line_ReadAllDirect(raw);

    g_lineRawMask = 0;
    for (i = 0; i < GRAYSCALE_CHANNELS; i++)
    {
        if (raw[i] == blackLevel)
        {
            g_lineRawMask |= (uint8_t)(1U << i);
        }
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
        alpha = limit_float(g_lineFilterAlpha, 0.0f, 1.0f);

        g_lineErrorFiltered = g_lineErrorFiltered * (1.0f - alpha) + rawError * alpha;
        g_lineError = (int16_t)g_lineErrorFiltered;
        g_lastLineDir = (g_lineError >= 0) ? 1 : -1;
        g_lineLostMs = 0;
    }
    else
    {
        g_lineValid = 0;
        if (g_lineLostMs < 60000U)
        {
            g_lineLostMs += CONTROL_PERIOD_MS;
        }
    }
}

static float Line_CalcTurnCmd(void)
{
    float error;
    float dError;
    float desiredSign;
    float turn;

    error = (float)g_lineError;
    dError = error - g_lineLastCtrlError;
    g_lineLastCtrlError = error;

    desiredSign = (((-g_lineTurnSign) * error) >= 0.0f) ? 1.0f : -1.0f;
    turn = (-g_lineTurnSign) * (error * g_lineKp + dError * g_lineKd);

    if (absf_local(error) > 70.0f && absf_local(turn) < g_lineMinTurn)
    {
        turn = desiredSign * g_lineMinTurn;
    }

    if ((g_lineMask & 0xC3U) != 0U)
    {
        turn += desiredSign * g_lineEdgeTurnExtra;
    }

    return limit_float(turn, -g_lineTurnLimit, g_lineTurnLimit);
}

static void Tracing_Control10ms(void)
{
    float forward;
    float turn;

    Line_Update();

    if (g_speedScale <= 0.01f || g_pwmLimit <= 0.5f)
    {
        g_targetForwardSpeed = 0.0f;
        g_targetTurnSpeed = 0.0f;
        g_carEnable = 0;
        return;
    }

    if (g_lineValid)
    {
        float e;
        float slowRatio;
        float minForward;

        e = absf_local((float)g_lineError) / 350.0f;
        if (e > 1.0f)
        {
            e = 1.0f;
        }

        slowRatio = 1.0f - limit_float(g_lineSlowGain, 0.0f, 0.95f) * e;

        if ((g_lineMask & 0xC3U) != 0U)
        {
            slowRatio *= limit_float(g_lineEdgeSpeedRatio, 0.05f, 1.0f);
        }

        forward = g_traceBaseSpeed * g_speedScale * slowRatio;
        minForward = g_traceSearchSpeed * g_speedScale;
        if (forward < minForward)
        {
            forward = minForward;
        }

        turn = Line_CalcTurnCmd() * g_speedScale;
    }
    else
    {
        forward = g_traceSearchSpeed * g_speedScale;
        turn = (-g_lineTurnSign) * (float)g_lastLineDir * g_lineLostTurn * g_speedScale;
    }

    g_targetForwardSpeed = slew_float(g_targetForwardSpeed, forward, g_forwardSlewStep);
    g_targetTurnSpeed = slew_float(g_targetTurnSpeed, turn, g_turnSlewStep);
    g_carEnable = 1;
}

/* ================================================================
 * 11. Speed loop, motor output, display and keys
 * ================================================================ */

static void Control_Run10ms(void)
{
    int16_t leftDelta;
    int16_t rightDelta;
    float pwmLimit;
    int32_t leftPwmTemp;
    int32_t rightPwmTemp;

    leftDelta = Encoder_GetLeftDelta();
    rightDelta = Encoder_GetRightDelta();

    g_leftSpeed = (float)leftDelta;
    g_rightSpeed = (float)rightDelta;
    g_forwardSpeed = (g_leftSpeed + g_rightSpeed) * 0.5f;
    g_turnSpeed = (g_rightSpeed - g_leftSpeed) * 0.5f;

    if (g_safetyLocked)
    {
        Control_ForcePWMZero();
        return;
    }

    Control_UpdatePIDParam();

    if (g_workMode == WORK_TRACING)
    {
        Tracing_Control10ms();
    }
    else
    {
        if (g_lastCmdTickMs > BT_TIMEOUT_MS)
        {
            g_targetForwardSpeed = 0.0f;
            g_targetTurnSpeed = 0.0f;
            g_carEnable = 0;
        }
    }

    if (!g_carEnable || g_pwmLimit <= 0.5f)
    {
        g_forwardSpeedError = g_targetForwardSpeed - g_forwardSpeed;
        g_speedPwm = 0.0f;
        g_diffPwm = 0.0f;
        g_leftPwm = 0;
        g_rightPwm = 0;
        Motor_StopAll();
        PID_Reset(&ForwardPID);
        PID_Reset(&TurnPID);
        return;
    }

    pwmLimit = limit_float(g_pwmLimit, PWM_LIMIT_MIN, PWM_LIMIT_MAX);
    ForwardPID.OutputLimit = pwmLimit;
    TurnPID.OutputLimit = pwmLimit * 0.85f;

    g_forwardSpeedError = g_targetForwardSpeed - g_forwardSpeed;
    g_speedPwm = PID_Calc(&ForwardPID, g_targetForwardSpeed, g_forwardSpeed);
    g_diffPwm = PID_Calc(&TurnPID, g_targetTurnSpeed, g_turnSpeed);

    leftPwmTemp = (int32_t)(g_speedPwm - g_diffPwm);
    rightPwmTemp = (int32_t)(g_speedPwm + g_diffPwm);

    g_leftPwm = limit_i16(leftPwmTemp, (int16_t)(-pwmLimit), (int16_t)pwmLimit);
    g_rightPwm = limit_i16(rightPwmTemp, (int16_t)(-pwmLimit), (int16_t)pwmLimit);

    Motor_SetPWM(g_leftPwm, g_rightPwm);
}

static char *ModeString(void)
{
    if (g_safetyLocked)
    {
        return "LOCK";
    }
    if (g_workMode == WORK_TRACING)
    {
        return "TRACE";
    }
    return "BT";
}

static void Serial_SendPlotStatus(void)
{
    int modeCode;

    modeCode = g_safetyLocked ? 9 : (int)g_workMode;

    Serial_Printf("[p,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d]\r\n",
                  modeCode,
                  (int)g_lineError,
                  (int)g_lineMask,
                  (int)g_targetForwardSpeed,
                  (int)g_forwardSpeed,
                  (int)g_targetTurnSpeed,
                  (int)g_turnSpeed,
                  (int)g_leftPwm,
                  (int)g_rightPwm,
                  (int)g_lineValid);
}

static void OLED_ShowStatus(void)
{
    OLED_Printf(0, 0, OLED_8X16, "M:%s LK:%d RP:%03d", ModeString(), (int)g_safetyLocked, (int)g_btSpeedLimitPercent);
    OLED_Printf(0, 16, OLED_8X16, "T:%+04d V:%+04d", (int)g_targetForwardSpeed, (int)g_forwardSpeed);
    OLED_Printf(0, 32, OLED_8X16, "L:%+04d R:%+04d", (int)g_leftPwm, (int)g_rightPwm);
    OLED_Printf(0, 48, OLED_8X16, "R:%02X M:%02X E:%+03d", (int)g_lineRawMask, (int)g_lineMask, (int)(g_lineError / 10));
    OLED_Update();
}

static void Main_KeyProcess(void)
{
    uint8_t key;

    key = Key_GetNum();
    if (key == 0U)
    {
        return;
    }

    if (key == 1U)
    {
        ApplySpeedLimitPercent(0.0f);
        Control_ForcePWMZero();
        Prompt_Start(160);
        return;
    }

    if (key == 2U)
    {
        if (g_safetyLocked)
        {
            Prompt_Start(80);
            return;
        }

        if (g_workMode == WORK_BT)
        {
            App_StartTracingMode();
        }
        else
        {
            App_StartBluetoothMode();
        }
        return;
    }

    if (key == 3U)
    {
        ApplyFastPreset();
        return;
    }
}

/* ================================================================
 * 12. Main function and 1 ms timer interrupt
 * ================================================================ */

int main(void)
{
    OLED_Init();
    Key_Init();
    Grayscale_Init();
    Motor_Init();
    Encoder_Init();

    Line_GPIOForceInit();
    PromptIO_Init();

    Serial_Init();
    Timer_Init();
    Control_Init();

    ApplySpeedLimitPercent(g_btSpeedLimitPercent);
    PromptIO_AllOff();

    OLED_Printf(0, 0, OLED_8X16, "BT/Trace Car");
    OLED_Printf(0, 16, OLED_8X16, "Default: BT");
    OLED_Printf(0, 32, OLED_8X16, "RP:%d%%", (int)g_btSpeedLimitPercent);
    OLED_Printf(0, 48, OLED_8X16, "Ready");
    OLED_Update();

    while (1)
    {
        Main_BTProcess();
        Main_KeyProcess();

        if (g_plotReportMs >= PLOT_REPORT_PERIOD_MS)
        {
            g_plotReportMs = 0;
            Serial_SendPlotStatus();
        }

        if (g_oledRefreshMs >= OLED_REFRESH_PERIOD_MS)
        {
            g_oledRefreshMs = 0;
            OLED_ShowStatus();
        }
    }
}

void TIM1_UP_IRQHandler(void)
{
    static uint8_t controlDiv = 0;

    if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
    {
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);

        Key_Tick();
        Prompt_Tick1ms();

        if (g_lastCmdTickMs < 60000U)
        {
            g_lastCmdTickMs++;
        }

        if (g_oledRefreshMs < 60000U)
        {
            g_oledRefreshMs++;
        }
        if (g_plotReportMs < 60000U)
        {
            g_plotReportMs++;
        }

        controlDiv++;
        if (controlDiv >= CONTROL_PERIOD_MS)
        {
            controlDiv = 0;
            Control_Run10ms();
        }
    }
}
