#include "app_e_car.h"
#include "app_control.h"
#include "app_line.h"
#include "BeepLed.h"
#include "Encoder.h"
#include "Key.h"
#include "Motor.h"
#include "OLED.h"
#include "PWM.h"
#include "stm32f10x.h"
#include <stdint.h>

#define E_CAR_CONTROL_PERIOD_MS      10U
#define E_CAR_TARGET_LAP_MIN         1U
#define E_CAR_TARGET_LAP_MAX         5U
#define E_CAR_TURN_SIGN              1.0f

#define E_CAR_FAULT_NONE             0U
#define E_CAR_FAULT_LINE_LOST        1U
#define E_CAR_FAULT_RECOVER_TIMEOUT  2U
#define E_CAR_FAULT_CORNER_TIMEOUT   3U

ECarParam_t g_eCarParam =
{
    30.0f,
    18.0f,
    12.0f,
    35.0f,

    0.25f,
    0.50f,
    80.0f,

    300U,
    150U,
    250U,
    1200U,

    4500,
    28120,

    5U
};

volatile float g_forwardKp = 14.0f;
volatile float g_forwardKi = 0.55f;
volatile float g_forwardKd = 1.8f;
volatile float g_turnKp = 10.0f;
volatile float g_turnKi = 0.04f;
volatile float g_turnKd = 1.0f;

volatile float g_pwmLimit = (float)PWM_MAX_DUTY * 0.65f;
volatile float g_targetForwardSpeed = 0.0f;
volatile float g_targetTurnSpeed = 0.0f;
volatile uint8_t g_carEnable = 0U;

volatile float g_leftSpeed = 0.0f;
volatile float g_rightSpeed = 0.0f;
volatile float g_forwardSpeed = 0.0f;
volatile float g_turnSpeed = 0.0f;
volatile float g_speedPwm = 0.0f;
volatile float g_diffPwm = 0.0f;
volatile float g_forwardSpeedError = 0.0f;
volatile int16_t g_leftPwm = 0;
volatile int16_t g_rightPwm = 0;

volatile int32_t g_leftEncoderTotal = 0;
volatile int32_t g_rightEncoderTotal = 0;
volatile int32_t g_forwardEncoderTotal = 0;
volatile int32_t g_turnEncoderTotal = 0;

volatile float g_lineBlackLevelF = 1.0f;
volatile float g_lineReverseOrderF = 0.0f;
volatile float g_lineTurnSign = 1.0f;
volatile float g_lineKp = 0.25f;
volatile float g_lineKd = 0.50f;
volatile float g_lineTurnLimit = 80.0f;
volatile float g_lineFilterAlpha = 0.58f;
volatile int16_t g_lineError = 0;
volatile uint8_t g_lineValid = 0U;
volatile uint8_t g_lineMask = 0U;
volatile uint8_t g_lineRawMask = 0U;
volatile int8_t g_lastLineDir = 1;
volatile uint16_t g_lineLostMs = 0U;

static volatile ECarState_t s_state = E_CAR_IDLE;
static volatile uint8_t s_targetLap = 1U;
static volatile uint8_t s_lapCount = 0U;
static volatile uint8_t s_cornerCount = 0U;
static volatile uint32_t s_runningTimeMs = 0U;
static volatile uint32_t s_stateMs = 0U;
static volatile uint32_t s_lostMs = 0U;
static volatile uint32_t s_recoverStableMsCount = 0U;
static volatile int32_t s_lastCornerForwardPulse = 0;
static volatile int32_t s_startForwardPulse = 0;
static volatile int32_t s_lapStartForwardPulse = 0;
static volatile int32_t s_currentLapPulse = 0;
static volatile float s_lapProgress = 0.0f;
static volatile uint8_t s_faultCode = E_CAR_FAULT_NONE;
static volatile uint16_t s_promptMs = 0U;

static float s_lastLineError = 0.0f;

static float ECar_LimitFloat(float value, float minVal, float maxVal)
{
    if (value < minVal) return minVal;
    if (value > maxVal) return maxVal;
    return value;
}

static uint8_t ECar_IsMotionState(ECarState_t state)
{
    return (uint8_t)(state == E_CAR_LINE_RUN ||
                     state == E_CAR_CORNER_ENTER ||
                     state == E_CAR_CORNER_TURN ||
                     state == E_CAR_LINE_RECOVER);
}

static uint8_t ECar_CountBlack(uint8_t mask)
{
    uint8_t count = 0U;
    uint8_t i;

    for (i = 0U; i < 8U; i++)
    {
        if ((mask & (uint8_t)(1U << i)) != 0U)
        {
            count++;
        }
    }

    return count;
}

static int32_t ECar_GetForwardPulse(void)
{
    return g_forwardEncoderTotal;
}

static void ECar_SetState(ECarState_t state)
{
    s_state = state;
    s_stateMs = 0U;
}

static void ECar_PromptStart(uint16_t ms)
{
    s_promptMs = ms;
    if (ms > 0U)
    {
        BeepLed_AllOn();
    }
    else
    {
        BeepLed_AllOff();
    }
}

void ECar_PromptTick1ms(void)
{
    if (s_promptMs > 0U)
    {
        s_promptMs--;
        if ((s_promptMs % 80U) == 0U)
        {
            BeepLed_AllTurn();
        }
        if (s_promptMs == 0U)
        {
            BeepLed_AllOff();
        }
    }
}

static void ECar_SafeStop(void)
{
    g_targetForwardSpeed = 0.0f;
    g_targetTurnSpeed = 0.0f;
    g_carEnable = 0U;
    Motor_StopAll();
    App_Control_ResetPID();
    App_Control_ForcePWMZero();
}

static void ECar_ClearEncoderTotals(void)
{
    __disable_irq();
    g_leftEncoderTotal = 0;
    g_rightEncoderTotal = 0;
    g_forwardEncoderTotal = 0;
    g_turnEncoderTotal = 0;
    Encoder_ClearAll();
    __enable_irq();
}

static void ECar_SyncLineParams(void)
{
    g_lineKp = g_eCarParam.line_kp;
    g_lineKd = g_eCarParam.line_kd;
    g_lineTurnLimit = g_eCarParam.turn_limit;
}

static void ECar_ResetLineState(void)
{
    App_Line_ResetState();
    g_lineError = 0;
    g_lineValid = 0U;
    g_lineMask = 0U;
    g_lineRawMask = 0U;
    g_lastLineDir = 1;
    g_lineLostMs = 0U;
    s_lastLineError = 0.0f;
}

static void ECar_ResetRunData(void)
{
    ECar_ClearEncoderTotals();
    ECar_ResetLineState();

    s_lapCount = 0U;
    s_cornerCount = 0U;
    s_runningTimeMs = 0U;
    s_stateMs = 0U;
    s_lostMs = 0U;
    s_recoverStableMsCount = 0U;
    s_lastCornerForwardPulse = 0;
    s_startForwardPulse = 0;
    s_lapStartForwardPulse = 0;
    s_currentLapPulse = 0;
    s_lapProgress = 0.0f;
    s_faultCode = E_CAR_FAULT_NONE;
}

static void ECar_UpdateLapProgress(void)
{
    int32_t pulse;
    int32_t lapPulse;
    int32_t defaultPulse;
    float progress;

    pulse = ECar_GetForwardPulse();
    lapPulse = pulse - s_lapStartForwardPulse;
    if (lapPulse < 0)
    {
        lapPulse = 0;
    }

    s_currentLapPulse = lapPulse;
    defaultPulse = g_eCarParam.lap_pulse_default;
    if (defaultPulse <= 0)
    {
        defaultPulse = 1;
    }

    progress = (float)lapPulse / (float)defaultPulse;
    s_lapProgress = ECar_LimitFloat(progress, 0.0f, 0.999f);
}

static float ECar_CalcLineTurnCmd(void)
{
    float error;
    float dError;
    float turn;

    error = (float)g_lineError;
    dError = error - s_lastLineError;
    s_lastLineError = error;

    turn = (-g_lineTurnSign) * (error * g_eCarParam.line_kp + dError * g_eCarParam.line_kd);
    return ECar_LimitFloat(turn, -g_eCarParam.turn_limit, g_eCarParam.turn_limit);
}

static float ECar_CalcSearchTurnCmd(void)
{
    float sign;

    sign = (s_lastLineError >= 0.0f) ? 1.0f : -1.0f;
    return (-g_lineTurnSign) * sign * g_eCarParam.turn_limit * 0.45f;
}

static void ECar_SetSpeedCmd(float forward, float turn)
{
    g_targetForwardSpeed = forward;
    g_targetTurnSpeed = turn;
    g_carEnable = 1U;
}

static uint8_t ECar_IsCornerDetected(void)
{
    int32_t pulse;
    int32_t interval;
    uint8_t blackCount;

    if (!g_lineValid)
    {
        return 0U;
    }

    blackCount = ECar_CountBlack(g_lineMask);
    if (blackCount < g_eCarParam.corner_black_count_th)
    {
        return 0U;
    }

    pulse = ECar_GetForwardPulse();
    interval = pulse - s_lastCornerForwardPulse;
    if (interval < g_eCarParam.min_corner_interval_pulse)
    {
        return 0U;
    }

    return 1U;
}

static uint8_t ECar_IsStableLineAfterCorner(void)
{
    uint8_t blackCount;

    if (!g_lineValid)
    {
        return 0U;
    }

    blackCount = ECar_CountBlack(g_lineMask);
    if (blackCount == 0U)
    {
        return 0U;
    }
    if (blackCount >= g_eCarParam.corner_black_count_th)
    {
        return 0U;
    }

    return 1U;
}

static void ECar_EnterFault(uint8_t faultCode)
{
    s_faultCode = faultCode;
    ECar_SafeStop();
    ECar_SetState(E_CAR_FAULT);
    ECar_PromptStart(900U);
}

static void ECar_EnterFinish(void)
{
    s_lapProgress = 0.999f;
    ECar_SafeStop();
    ECar_SetState(E_CAR_FINISH);
    ECar_PromptStart(600U);
}

static void ECar_EnterRecover(void)
{
    s_lostMs = 0U;
    s_recoverStableMsCount = 0U;
    ECar_SetState(E_CAR_LINE_RECOVER);
}

static void ECar_HandleLineRun(void)
{
    float turnCmd;

    if (g_lineValid)
    {
        s_lostMs = 0U;
        turnCmd = ECar_CalcLineTurnCmd();
    }
    else
    {
        if (s_lostMs < 60000U)
        {
            s_lostMs += E_CAR_CONTROL_PERIOD_MS;
        }

        if (s_lostMs >= g_eCarParam.lost_timeout_ms)
        {
            ECar_EnterFault(E_CAR_FAULT_LINE_LOST);
            return;
        }

        turnCmd = ECar_CalcSearchTurnCmd();
    }

    ECar_SetSpeedCmd(g_eCarParam.base_speed, turnCmd);

    if (ECar_IsCornerDetected())
    {
        ECar_SetState(E_CAR_CORNER_ENTER);
    }
}

static void ECar_HandleCornerEnter(void)
{
    int32_t pulse;

    pulse = ECar_GetForwardPulse();
    s_lastCornerForwardPulse = pulse;

    if (s_cornerCount < 250U)
    {
        s_cornerCount++;
    }

    if ((s_cornerCount % 4U) == 0U)
    {
        if (s_lapCount < 250U)
        {
            s_lapCount++;
        }

        if (s_lapCount >= s_targetLap)
        {
            ECar_EnterFinish();
            return;
        }

        s_lapStartForwardPulse = pulse;
        s_currentLapPulse = 0;
        s_lapProgress = 0.0f;
    }

    s_lostMs = 0U;
    s_recoverStableMsCount = 0U;
    ECar_SetState(E_CAR_CORNER_TURN);
}

static void ECar_HandleCornerTurn(void)
{
    ECar_SetSpeedCmd(g_eCarParam.corner_forward_speed,
                     g_eCarParam.corner_turn_speed * E_CAR_TURN_SIGN);

    if (s_stateMs >= g_eCarParam.corner_min_turn_ms)
    {
        if (ECar_IsStableLineAfterCorner())
        {
            ECar_EnterRecover();
            return;
        }
    }

    if (s_stateMs >= g_eCarParam.corner_max_turn_ms)
    {
        if (g_lineValid)
        {
            ECar_EnterRecover();
        }
        else
        {
            ECar_EnterFault(E_CAR_FAULT_CORNER_TIMEOUT);
        }
    }
}

static void ECar_HandleRecover(void)
{
    float turnCmd;
    uint32_t recoverTimeoutMs;

    if (g_lineValid)
    {
        s_lostMs = 0U;
        turnCmd = ECar_CalcLineTurnCmd();
    }
    else
    {
        if (s_lostMs < 60000U)
        {
            s_lostMs += E_CAR_CONTROL_PERIOD_MS;
        }

        if (s_lostMs >= g_eCarParam.lost_timeout_ms)
        {
            ECar_EnterFault(E_CAR_FAULT_RECOVER_TIMEOUT);
            return;
        }

        turnCmd = ECar_CalcSearchTurnCmd();
    }

    ECar_SetSpeedCmd(g_eCarParam.recover_speed, turnCmd);

    if (ECar_IsStableLineAfterCorner())
    {
        if (s_recoverStableMsCount < 60000U)
        {
            s_recoverStableMsCount += E_CAR_CONTROL_PERIOD_MS;
        }

        if (s_recoverStableMsCount >= g_eCarParam.recover_stable_ms)
        {
            ECar_SetState(E_CAR_LINE_RUN);
            return;
        }
    }
    else
    {
        s_recoverStableMsCount = 0U;
    }

    recoverTimeoutMs = (uint32_t)g_eCarParam.lost_timeout_ms +
                       (uint32_t)g_eCarParam.recover_stable_ms + 500U;
    if (s_stateMs >= recoverTimeoutMs)
    {
        ECar_EnterFault(E_CAR_FAULT_RECOVER_TIMEOUT);
    }
}

void ECar_Init(void)
{
    ECar_SyncLineParams();
    s_targetLap = 1U;
    ECar_ResetRunData();
    ECar_SafeStop();
    ECar_SetState(E_CAR_IDLE);
}

void ECar_Reset(void)
{
    ECar_SyncLineParams();
    ECar_SafeStop();
    ECar_ResetRunData();
    ECar_SetState(E_CAR_READY);
}

void ECar_Start(void)
{
    if (s_targetLap < E_CAR_TARGET_LAP_MIN)
    {
        s_targetLap = E_CAR_TARGET_LAP_MIN;
    }
    if (s_targetLap > E_CAR_TARGET_LAP_MAX)
    {
        s_targetLap = E_CAR_TARGET_LAP_MAX;
    }

    ECar_SyncLineParams();
    ECar_SafeStop();
    ECar_ResetRunData();

    s_startForwardPulse = ECar_GetForwardPulse();
    s_lapStartForwardPulse = s_startForwardPulse;
    s_lastCornerForwardPulse = s_startForwardPulse;
    ECar_SetState(E_CAR_LINE_RUN);
    g_carEnable = 1U;
    ECar_PromptStart(120U);
}

void ECar_Stop(void)
{
    ECar_SafeStop();
    ECar_ResetRunData();
    ECar_SetState(E_CAR_READY);
    ECar_PromptStart(220U);
}

void ECar_Control10ms(void)
{
    ECar_SyncLineParams();
    App_Control_UpdateEncoderSpeed();
    App_Line_Update();

    if (ECar_IsMotionState(s_state))
    {
        if (s_runningTimeMs <= 0xFFFFFFFFU - E_CAR_CONTROL_PERIOD_MS)
        {
            s_runningTimeMs += E_CAR_CONTROL_PERIOD_MS;
        }
    }

    if (s_stateMs <= 0xFFFFFFFFU - E_CAR_CONTROL_PERIOD_MS)
    {
        s_stateMs += E_CAR_CONTROL_PERIOD_MS;
    }

    ECar_UpdateLapProgress();
    App_Control_UpdatePIDParam();

    switch (s_state)
    {
        case E_CAR_IDLE:
        case E_CAR_READY:
        case E_CAR_FINISH:
        case E_CAR_FAULT:
            ECar_SafeStop();
            break;

        case E_CAR_LINE_RUN:
            ECar_HandleLineRun();
            break;

        case E_CAR_CORNER_ENTER:
            ECar_HandleCornerEnter();
            break;

        case E_CAR_CORNER_TURN:
            ECar_HandleCornerTurn();
            break;

        case E_CAR_LINE_RECOVER:
            ECar_HandleRecover();
            break;

        default:
            ECar_EnterFault(E_CAR_FAULT_LINE_LOST);
            break;
    }

    App_Control_ApplyMotorOutput();
}

void ECar_KeyProcess(void)
{
    uint8_t key;

    key = Key_GetNum();
    if (key == 0U)
    {
        return;
    }

    if (key == 1U)
    {
        if (!ECar_IsMotionState(s_state) && s_state != E_CAR_FAULT)
        {
            s_targetLap++;
            if (s_targetLap > E_CAR_TARGET_LAP_MAX)
            {
                s_targetLap = E_CAR_TARGET_LAP_MIN;
            }
            ECar_SafeStop();
            ECar_SetState(E_CAR_READY);
            ECar_PromptStart((uint16_t)(80U + 30U * s_targetLap));
        }
        return;
    }

    if (key == 2U)
    {
        if (!ECar_IsMotionState(s_state) && s_state != E_CAR_FAULT)
        {
            ECar_Start();
        }
        return;
    }

    if (key == 3U)
    {
        ECar_Stop();
        return;
    }

    if (key == 4U)
    {
        if (s_state == E_CAR_FAULT)
        {
            ECar_Reset();
            ECar_PromptStart(180U);
        }
        else
        {
            ECar_SafeStop();
            ECar_ResetRunData();
            ECar_SetState(E_CAR_IDLE);
            ECar_PromptStart(260U);
        }
    }
}

void ECar_ShowStatus(void)
{
    uint8_t progressPercent;
    int32_t lineErr;

    progressPercent = (uint8_t)(s_lapProgress * 100.0f);
    if (progressPercent > 99U)
    {
        progressPercent = 99U;
    }

    lineErr = (int32_t)g_lineError;
    if (lineErr > 999)
    {
        lineErr = 999;
    }
    if (lineErr < -999)
    {
        lineErr = -999;
    }

    OLED_Clear();
    OLED_ShowString(0, 0, "E CAR", OLED_8X16);

    OLED_ShowString(0, 16, "N:", OLED_8X16);
    OLED_ShowNum(16, 16, s_targetLap, 1, OLED_8X16);
    OLED_ShowString(40, 16, "L:", OLED_8X16);
    OLED_ShowNum(56, 16, s_lapCount, 1, OLED_8X16);

    OLED_ShowString(0, 32, "C:", OLED_8X16);
    OLED_ShowNum(16, 32, s_cornerCount, 2, OLED_8X16);
    OLED_ShowString(48, 32, "S:", OLED_8X16);
    OLED_ShowNum(64, 32, (uint8_t)s_state, 1, OLED_8X16);
    OLED_ShowString(88, 32, "F:", OLED_8X16);
    OLED_ShowNum(104, 32, s_faultCode, 1, OLED_8X16);

    OLED_ShowString(0, 48, "P:", OLED_8X16);
    OLED_ShowNum(16, 48, progressPercent, 2, OLED_8X16);
    OLED_ShowString(32, 48, "% E:", OLED_8X16);
    OLED_ShowSignedNum(64, 48, lineErr, 3, OLED_8X16);

    OLED_Update();
}

float ECar_GetLapProgress(void)
{
    return s_lapProgress;
}

uint8_t ECar_GetLapCount(void)
{
    return s_lapCount;
}

uint8_t ECar_GetTargetLap(void)
{
    return s_targetLap;
}

ECarState_t ECar_GetState(void)
{
    return s_state;
}

uint8_t ECar_SetTargetLap(uint8_t lap)
{
    if (lap < E_CAR_TARGET_LAP_MIN || lap > E_CAR_TARGET_LAP_MAX)
    {
        return 0U;
    }

    if (ECar_IsMotionState(s_state))
    {
        return 0U;
    }

    s_targetLap = lap;
    if (s_state != E_CAR_FAULT)
    {
        ECar_SafeStop();
        ECar_SetState(E_CAR_READY);
    }

    return 1U;
}

uint8_t ECar_GetCornerCount(void)
{
    return s_cornerCount;
}

uint8_t ECar_GetFaultCode(void)
{
    return s_faultCode;
}

uint32_t ECar_GetRunningTimeMs(void)
{
    return s_runningTimeMs;
}
