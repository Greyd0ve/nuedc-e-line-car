/*
 * 文件：User/main.c
 * 版本：Bluetooth + 8路灰度循迹精简调参版
 *
 * 本文件只负责应用层逻辑，不修改 Hardware 文件夹中的驱动代码。
 *
 * 功能总览：
 *   1. 上电默认进入蓝牙遥控模式。
 *   2. 网页发送 [key,tracing,down] 后进入八路灰度黑线循迹模式。
 *   3. 网页发送 [key,Bluetooth,down] 后回到蓝牙遥控模式。
 *   4. 网页急停 [key,emergency,down] 后立即关闭 PWM，并进入安全锁定。
 *   5. 只有收到 [key,unlock,down] 后才解除锁定，避免误触后小车继续运动。
 *   6. 网页滑杆 [slider,RP,0~100] 用作最大 PWM 百分比限幅。
 *   7. 支持网页滑杆在线调 PID、循迹、速度、滤波、斜坡等重要参数。
 *   8. 周期回传 [p,...] 绘图包，便于在网页绘图区观察控制状态。
 *
 * 注意：
 *   - 本文件不依赖 BT_proto.c 的解析函数，主循环中直接从 Serial_ReadByte() 取串口数据并解析。
 *   - 这样做是为了满足“只修改 main.c，不修改 Hardware 文件夹”的要求。
 *   - Hardware/BT_proto.c 可以继续保留在工程里，但本 main.c 不调用 BT_Process()。
 */

#include "stm32f10x.h"
#include "OLED.h"
#include "LED.h"
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
 * 1. 基础控制周期和安全参数
 * ================================================================ */

/* 速度 PID、循迹控制统一每 10 ms 运行一次。 */
#define CONTROL_PERIOD_MS              10U

/* 蓝牙遥控超时时间。超过该时间未收到新运动包，蓝牙模式自动停车。 */
#define BT_TIMEOUT_MS                  600U

/* 网页 [p,...] 绘图包回传周期。100 ms 一次既比较实时，也不会占满串口带宽。 */
#define PLOT_REPORT_PERIOD_MS          100U

/* OLED 本地状态刷新周期。OLED 是软件 I2C，全屏刷新太频繁会影响主循环。 */
#define OLED_REFRESH_PERIOD_MS         100U

/* 蓝牙 RP 滑杆对应最大 PWM 限幅。PWM_MAX_DUTY 在 Hardware/PWM.h 中定义，通常为 1000。 */
#define PWM_LIMIT_MIN                  0.0f
#define PWM_LIMIT_MAX                  ((float)PWM_MAX_DUTY)

/* ================================================================
 * 2. 八路灰度循迹默认参数
 *
 * 这些参数都可以通过网页 [slider,参数名,数值] 在线修改。
 * 具体参数名见本文档下方 Main_ApplySliderPacket() 和配套 MD 文档。
 * ================================================================ */

/* 大部分数字量灰度模块检测黑线输出 1；如果你的模块黑线输出 0，可网页发送 [slider,blackLevel,0]。 */
volatile float g_lineBlackLevelF = 1.0f;

/* 若灰度板左右顺序和小车实际安装方向相反，可改为 1。也可以用 [slider,lineReverse,1] 在线设置。 */
volatile float g_lineReverseOrderF = 0.0f;

/* 循迹转向符号。若黑线在右侧时小车反而向左修，发送 [slider,lineSign,-1]。 */
volatile float g_lineTurnSign = 1.0f;

/* 纯循迹基础速度，单位是“编码器脉冲/10ms”的目标速度。实际还会乘 g_speedScale。 */
volatile float g_traceBaseSpeed = 24.0f;

/* 丢线找线速度。 */
volatile float g_traceSearchSpeed = 8.0f;

/* 循迹 P/D 参数。P 决定基本转向，D 用于进弯时提前给方向。 */
volatile float g_lineKp = 0.135f;
volatile float g_lineKd = 0.055f;

/* 循迹转向限幅，防止差速目标过大。弯道转不过时可适当增大。 */
volatile float g_lineTurnLimit = 90.0f;

/* 丢线后按最后一次方向找线的转向力度。 */
volatile float g_lineLostTurn = 72.0f;

/* 灰度误差一阶低通滤波系数，0~1。越大反应越快，越小越平滑。 */
volatile float g_lineFilterAlpha = 0.65f;

/* 偏差越大自动降速。0.55 表示大偏差时最多降低约 55% 基础速度。 */
volatile float g_lineSlowGain = 0.55f;

/* 最外侧探头看到黑线时，认为进入急弯或偏差较大，额外加转向并降速。 */
volatile float g_lineEdgeTurnExtra = 22.0f;
volatile float g_lineEdgeSpeedRatio = 0.55f;

/* 中等偏差以上的最小转向，避免转向量太小导致小车直冲弯道。 */
volatile float g_lineMinTurn = 18.0f;

/* 目标速度斜坡限制。每 10ms 目标最多变化多少，用于减少电机突变。 */
volatile float g_forwardSlewStep = 3.0f;
volatile float g_turnSlewStep = 14.0f;

/* ================================================================
 * 3. 速度环/差速环 PID 参数
 *
 * ForwardPID 控制平均速度；TurnPID 控制左右轮速度差。
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

/* 速度环 PID，网页参数名：speedKp / speedKi / speedKd。 */
volatile float g_forwardKp = 14.0f;
volatile float g_forwardKi = 0.55f;
volatile float g_forwardKd = 1.8f;

/* 差速环 PID，网页参数名：turnKp 或 diffKp，turnKi/diffKi，turnKd/diffKd。 */
volatile float g_turnKp = 10.0f;
volatile float g_turnKi = 0.04f;
volatile float g_turnKd = 1.0f;

/* 蓝牙遥控最大目标速度/转向目标。实际会乘 g_speedScale。 */
volatile float g_maxForwardCmd = 70.0f;
volatile float g_maxTurnCmd = 75.0f;

/* ================================================================
 * 4. 运行状态变量
 * ================================================================ */

typedef enum
{
    WORK_BT = 0,       /* 蓝牙遥控模式 */
    WORK_TRACING = 1   /* 八路灰度循迹模式 */
} WorkMode_t;

volatile WorkMode_t g_workMode = WORK_BT;

/* 上层控制输出的目标速度。单位是编码器脉冲/10ms。 */
volatile float g_targetForwardSpeed = 0.0f;
volatile float g_targetTurnSpeed = 0.0f;

/* 控制使能：1 允许 PID 输出电机，0 停车。 */
volatile uint8_t g_carEnable = 0;

/* 距离上一次收到蓝牙有效数据包的时间，单位 ms。 */
volatile uint32_t g_lastCmdTickMs = 1000;

/* 急停安全锁：1 表示锁定，此时任何运动命令都无效。 */
volatile uint8_t g_safetyLocked = 0;

/* 蓝牙 RP 滑杆值和换算出的限速。 */
volatile float g_btSpeedLimitPercent = 30.0f;
volatile float g_speedScale = 0.30f;
volatile float g_pwmLimit = 300.0f;

/* 编码器测得的实时速度。 */
volatile float g_leftSpeed = 0.0f;
volatile float g_rightSpeed = 0.0f;
volatile float g_forwardSpeed = 0.0f;
volatile float g_turnSpeed = 0.0f;

/* PID 输出。g_speedPwm 为前进速度环输出，g_diffPwm 为差速环输出。 */
volatile float g_speedPwm = 0.0f;
volatile float g_diffPwm = 0.0f;

/* 左右轮最终 PWM。 */
volatile int16_t g_leftPwm = 0;
volatile int16_t g_rightPwm = 0;

/* 用于网页绘图的前进速度误差。 */
volatile float g_forwardSpeedError = 0.0f;

/* 这些变量保留给旧版 BT_proto.c 链接使用。本 main.c 不调用 BT_Process()，但工程中若仍编译 BT_proto.c，需要这些符号存在。 */
volatile uint8_t g_sendPlot = 0;
volatile uint16_t g_sendDisplay = 0;

/* OLED 和 plot 的节拍标志。 */
volatile uint16_t g_oledRefreshMs = 0;
volatile uint16_t g_plotReportMs = 0;

/* 灰度循迹状态。 */
volatile int16_t g_lineError = 0;
volatile uint8_t g_lineValid = 0;
volatile uint8_t g_lineMask = 0;
volatile int8_t g_lastLineDir = 1;
volatile uint16_t g_lineLostMs = 0;

static volatile float g_lineErrorFiltered = 0.0f;
static volatile float g_lineLastCtrlError = 0.0f;
static volatile uint16_t g_promptMs = 0;

/* ================================================================
 * 5. 小工具函数
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
 * 6. PID 和停车控制
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

    /* 简单抗积分饱和：输出顶到限幅时，只允许积分向脱离饱和方向变化。 */
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
    LED_ON();
}

static void Prompt_Tick1ms(void)
{
    if (g_promptMs > 0)
    {
        g_promptMs--;
        if ((g_promptMs % 80U) == 0U)
        {
            LED_Turn();
        }
        if (g_promptMs == 0U)
        {
            LED_OFF();
        }
    }
}

/* ================================================================
 * 7. 模式切换和急停接口
 *
 * 这些函数名保留，是为了兼容工程中已有的 BT_proto.c。
 * 本 main.c 内置解析器也会调用这些函数。
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
 * 8. 蓝牙/网页协议解析
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

static void Main_ApplySliderPacket(const char *name, float value)
{
    /* 最高 PWM 限幅：0~100，对应 0%~100%。 */
    if (str_is_name(name, "RP", "rp", "speedLimit"))
    {
        ApplySpeedLimitPercent(value);
        return;
    }

    /* 速度环 PID。示例：[slider,speedKp,10] -> Kp=10。 */
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

    /* 差速/转向环 PID。diffKp 和 turnKp 等价。 */
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

    /* 蓝牙手动模式最大目标速度/转向速度。 */
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

    /* 循迹参数。 */
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
        /* 支持 0~1，也支持 0~100 两种输入。 */
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

    /*
     * 用户指定协议：
     *   [joystick,左右转向控制数据,前后速度控制数据,忽略,忽略]
     *
     * 因此：tok[1] 用作左右转向，tok[2] 用作前后速度，tok[3]/tok[4] 不处理。
     */
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

    /* key 包：模式切换、急停、解锁。 */
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

    /* slider 包：RP 限速 + 在线调参。 */
    if (str_is_name(tok[0], "slider", "s", 0))
    {
        if (n >= 3)
        {
            Main_ApplySliderPacket(tok[1], (float)atof(tok[2]));
        }
        return;
    }

    /* joystick 包：蓝牙遥控模式使用。 */
    if (str_is_name(tok[0], "joystick", "j", 0))
    {
        Main_ApplyJoystickPacket(tok, n);
        return;
    }

    /* 兼容串口助手直接命令：[cmd,forward,turn]。 */
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
 * 9. 八路灰度循迹算法
 * ================================================================ */

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

    Grayscale_ReadAll(raw);

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

    /* 0xC3 覆盖最左两路和最右两路。外侧触发时加大转向，帮助过弯。 */
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
 * 10. 网页绘图回传与 OLED 本地显示
 * ================================================================ */

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
    /*
     * 调参网页绘图协议：短格式 [p,数值1,数值2,...]
     * 依次回传：
     *   1. 当前设定速度目标：g_targetForwardSpeed
     *   2. 当前前进速度 PID 输出 PWM：g_speedPwm
     *   3. 前进速度误差：g_forwardSpeedError = 目标速度 - 实际速度
     *   4. 左轮最终 PWM：g_leftPwm
     *   5. 右轮最终 PWM：g_rightPwm
     */
    Serial_Printf("[p,%d,%d,%d,%d,%d]\r\n",
                  (int)g_targetForwardSpeed,
                  (int)g_speedPwm,
                  (int)g_forwardSpeedError,
                  (int)g_leftPwm,
                  (int)g_rightPwm);
}

static void OLED_ShowStatus(void)
{
    OLED_Printf(0, 0, OLED_8X16, "M:%s LK:%d RP:%03d", ModeString(), (int)g_safetyLocked, (int)g_btSpeedLimitPercent);
    OLED_Printf(0, 16, OLED_8X16, "T:%+04d V:%+04d", (int)g_targetForwardSpeed, (int)g_forwardSpeed);
    OLED_Printf(0, 32, OLED_8X16, "L:%+04d R:%+04d", (int)g_leftPwm, (int)g_rightPwm);
    OLED_Printf(0, 48, OLED_8X16, "E:%+04d M:%02X", (int)g_lineError, (int)g_lineMask);
    OLED_Update();
}

/* ================================================================
 * 11. 主函数
 * ================================================================ */

int main(void)
{
    OLED_Init();
    LED_Init();
    Key_Init();
    Grayscale_Init();
    Motor_Init();
    Encoder_Init();
    Serial_Init();
    Timer_Init();
    Control_Init();

    ApplySpeedLimitPercent(g_btSpeedLimitPercent);
    LED_OFF();

    OLED_Printf(0, 0, OLED_8X16, "BT/Trace Car");
    OLED_Printf(0, 16, OLED_8X16, "Default: BT");
    OLED_Printf(0, 32, OLED_8X16, "RP:%d%%", (int)g_btSpeedLimitPercent);
    OLED_Printf(0, 48, OLED_8X16, "Ready");
    OLED_Update();

    while (1)
    {
        /* 只使用 main.c 内置解析器，不调用 BT_Process()，因此 Hardware 文件夹无需改动。 */
        Main_BTProcess();

        if (g_oledRefreshMs >= OLED_REFRESH_PERIOD_MS)
        {
            g_oledRefreshMs = 0;
            OLED_ShowStatus();
        }

        if (g_plotReportMs >= PLOT_REPORT_PERIOD_MS)
        {
            g_plotReportMs = 0;
            Serial_SendPlotStatus();
        }
    }
}

/* ================================================================
 * 12. TIM1 1ms 中断：控制核心
 * ================================================================ */

void TIM1_UP_IRQHandler(void)
{
    static uint8_t controlDiv = 0;

    if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
    {
        int16_t leftDelta;
        int16_t rightDelta;

        Key_Tick();
        Prompt_Tick1ms();

        if (g_lastCmdTickMs < 0xFFFFFFFFUL)
        {
            g_lastCmdTickMs++;
        }
        if (g_oledRefreshMs < 1000U)
        {
            g_oledRefreshMs++;
        }
        if (g_plotReportMs < 1000U)
        {
            g_plotReportMs++;
        }

        controlDiv++;
        if (controlDiv >= CONTROL_PERIOD_MS)
        {
            float forwardTarget;
            float turnTarget;
            float leftCommand;
            float rightCommand;

            controlDiv = 0;
            Control_UpdatePIDParam();

            leftDelta = Encoder_GetLeftDelta();
            rightDelta = Encoder_GetRightDelta();

            g_leftSpeed = (float)leftDelta;
            g_rightSpeed = (float)rightDelta;
            g_forwardSpeed = (g_leftSpeed + g_rightSpeed) * 0.5f;
            g_turnSpeed = (g_rightSpeed - g_leftSpeed) * 0.5f;

            if (g_safetyLocked)
            {
                Control_ForcePWMZero();
            }
            else
            {
                if (g_workMode == WORK_TRACING)
                {
                    Tracing_Control10ms();
                }
                else
                {
                    /* 蓝牙遥控模式超时保护。 */
                    if (g_lastCmdTickMs > BT_TIMEOUT_MS)
                    {
                        g_targetForwardSpeed = 0.0f;
                        g_targetTurnSpeed = 0.0f;
                        g_carEnable = 0;
                    }
                }

                if (!g_carEnable && absf_local(g_targetForwardSpeed) < 0.01f && absf_local(g_targetTurnSpeed) < 0.01f)
                {
                    Control_ForcePWMZero();
                }
                else
                {
                    forwardTarget = g_carEnable ? g_targetForwardSpeed : 0.0f;
                    turnTarget = g_carEnable ? g_targetTurnSpeed : 0.0f;

                    g_forwardSpeedError = forwardTarget - g_forwardSpeed;
                    g_speedPwm = PID_Calc(&ForwardPID, forwardTarget, g_forwardSpeed);
                    g_diffPwm = PID_Calc(&TurnPID, turnTarget, g_turnSpeed);

                    leftCommand = g_speedPwm - g_diffPwm;
                    rightCommand = g_speedPwm + g_diffPwm;

                    g_leftPwm = limit_i16((int32_t)leftCommand, -(int16_t)g_pwmLimit, (int16_t)g_pwmLimit);
                    g_rightPwm = limit_i16((int32_t)rightCommand, -(int16_t)g_pwmLimit, (int16_t)g_pwmLimit);

                    Motor_SetPWM(g_leftPwm, g_rightPwm);
                }
            }
        }

        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
    }
}
