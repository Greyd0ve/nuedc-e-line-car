/*
 * 文件：Hardware/BT_proto.c
 * 作用：解析蓝牙串口小程序发送的数据包。
 *
 * 支持的数据包来自蓝牙串口协议：
 *   [joystick,lx,ly,rx,ry] 或短格式 [j,lx,ly,rx,ry]
 *   [slider,id,value]      或短格式 [s,id,value]
 *   [slider,RP,0~100]     使用名为 RP 的滑杆控制最高 PWM 输出百分比
 *   [key,id,down/up]       或短格式 [k,id,d/u]
 *   [key,tracing,down]     进入八路灰度循迹模式
 *   [key,Bluetooth,down]   进入蓝牙遥控模式
 *   [key,emergency,down]   立即急停并进入安全锁定状态
 *   [key,unlock,down]      解除安全锁，回到蓝牙遥控空闲状态
 *
 * 本版修改重点：
 *   原先最高速度由板载 RP4 电位器控制；现在改为由蓝牙滑杆 RP 控制。
 *   例如：[slider,RP,0] 表示 PWM 最大输出限制为 0%；
 *         [slider,RP,100] 表示 PWM 最大输出限制为 100%。
 */
#include "BT_proto.h"
#include "Serial.h"
#include "PWM.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

extern volatile float g_targetForwardSpeed;
extern volatile float g_targetTurnSpeed;
extern volatile uint8_t g_carEnable;
extern volatile uint32_t g_lastCmdTickMs;

extern volatile float g_forwardKp;
extern volatile float g_forwardKi;
extern volatile float g_forwardKd;
extern volatile float g_turnKp;
extern volatile float g_turnKi;
extern volatile float g_turnKd;
extern volatile float g_maxForwardCmd;
extern volatile float g_maxTurnCmd;
extern volatile float g_pwmLimit;
extern volatile float g_speedScale;
extern volatile float g_btSpeedLimitPercent;
extern volatile uint8_t g_safetyLocked;

extern void App_StartTracingMode(void);
extern void App_StartBluetoothMode(void);
extern void App_EmergencyStop(void);
extern void App_UnlockControl(void);

static float absf_local(float x)
{
    return (x >= 0.0f) ? x : -x;
}

static int str_equal(const char *a, const char *b)
{
    return strcmp(a, b) == 0;
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

static int str_is_tracing_name(const char *s)
{
    return str_equal_ignore_case(s, "tracing") || str_equal_ignore_case(s, "trace") || str_equal_ignore_case(s, "line");
}

static int str_is_bluetooth_name(const char *s)
{
    return str_equal_ignore_case(s, "Bluetooth") || str_equal_ignore_case(s, "BT") || str_equal_ignore_case(s, "remote");
}

static int str_is_emergency_name(const char *s)
{
    return str_equal_ignore_case(s, "emergency") || str_equal_ignore_case(s, "emg") || str_equal_ignore_case(s, "stop");
}

static int str_is_unlock_name(const char *s)
{
    return str_equal_ignore_case(s, "unlock") || str_equal_ignore_case(s, "release") || str_equal_ignore_case(s, "resume");
}

static int str_is_rp_name(const char *s)
{
    return str_equal(s, "RP") || str_equal(s, "rp") || str_equal(s, "Rp") || str_equal(s, "rP");
}

static float clip_float(float x, float minVal, float maxVal)
{
    if (x < minVal)
    {
        return minVal;
    }
    if (x > maxVal)
    {
        return maxVal;
    }
    return x;
}

/*
 * 通过蓝牙串口虚拟滑杆 RP 设置最高速度/PWM 限幅。
 * 协议示例：
 *   [slider,RP,0]   -> PWM 0% 输出，g_pwmLimit = 0
 *   [slider,RP,100] -> PWM 100% 输出，g_pwmLimit = PWM_MAX_DUTY
 *
 * 同时同步 g_speedScale，使 H 题自动路径速度也随 RP 百分比缩放。
 */
static void BT_ApplySpeedLimitPercent(int value)
{
    float percent = clip_float((float)value, 0.0f, 100.0f);
    float ratio = percent / 100.0f;

    g_btSpeedLimitPercent = percent;
    g_pwmLimit = (float)PWM_MAX_DUTY * ratio;
    g_speedScale = ratio;

    g_maxForwardCmd = 80.0f * g_speedScale;
    g_maxTurnCmd = 85.0f * g_speedScale;
}

static float slider_to_command(int value, float maxAbs)
{
    if (abs(value) <= 100)
    {
        return value * maxAbs / 100.0f;
    }
    return (float)value;
}

static void apply_packet(char *payload)
{
    char *tok[10] = {0};
    int n = 0;
    char *p = payload;

    tok[n++] = p;
    while (*p != '\0' && n < 10)
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

    /*
     * 1. 安全控制最高优先级。
     *
     * 网页急停按钮会发送：
     *   [key,emergency,down]
     *   [joystick,0,0,0,0]
     *
     * 所以必须先识别 emergency，让主程序立即关 PWM 并进入安全锁。
     * 解锁命令 [key,unlock,down] 也必须在安全锁状态下仍然可用。
     */
    if (str_equal(tok[0], "k") || str_equal(tok[0], "key"))
    {
        if (n >= 3)
        {
            int isDown = str_equal(tok[2], "d") || str_equal(tok[2], "down");

            if (isDown && str_is_emergency_name(tok[1]))
            {
                App_EmergencyStop();
                return;
            }

            if (isDown && str_is_unlock_name(tok[1]))
            {
                App_UnlockControl();
                return;
            }
        }
    }

    /*
     * 2. 摇杆控制。
     *
     * 协议：
     *   [joystick,左X,左Y,右X,右Y]
     *
     * 本车目前使用左摇杆 Y 作为前进/后退，右摇杆 X 作为转向。
     * 也就是说：tok[2] -> forward，tok[3] -> turn。
     */
    if (str_equal(tok[0], "j") || str_equal(tok[0], "joystick"))
    {
        if (n >= 4)
        {
            int forward;
            int turn;

            if (g_safetyLocked)
            {
                return;
            }

            forward = atoi(tok[2]);
            turn = atoi(tok[3]);

            g_targetForwardSpeed = clip_float(forward * g_maxForwardCmd / 100.0f,
                                              -g_maxForwardCmd,
                                              g_maxForwardCmd);

            g_targetTurnSpeed = clip_float(-turn * g_maxTurnCmd / 100.0f,
                                           -g_maxTurnCmd,
                                           g_maxTurnCmd);

            g_carEnable = 1;
            return;
        }
    }

    /*
     * 3. 滑杆控制。
     *
     * 最重要的滑杆是 RP：
     *   [slider,RP,0]   -> PWM 最大输出 0%
     *   [slider,RP,100] -> PWM 最大输出 100%
     *
     * 急停锁定时仍允许 RP 更新，因为这不直接产生运动输出；
     * 其他滑杆在急停锁定时全部忽略。
     */
    if (str_equal(tok[0], "s") || str_equal(tok[0], "slider"))
    {
        if (n >= 3)
        {
            int value = atoi(tok[2]);
            int id;

            if (str_is_rp_name(tok[1]))
            {
                BT_ApplySpeedLimitPercent(value);
                return;
            }

            if (g_safetyLocked)
            {
                return;
            }

            id = atoi(tok[1]);

            if (id == 1)
            {
                g_targetForwardSpeed = clip_float(slider_to_command(value, g_maxForwardCmd),
                                                  -g_maxForwardCmd,
                                                  g_maxForwardCmd);
                g_carEnable = 1;
            }
            else if (id == 2)
            {
                g_targetTurnSpeed = clip_float(slider_to_command(value, g_maxTurnCmd),
                                               -g_maxTurnCmd,
                                               g_maxTurnCmd);
                g_carEnable = 1;
            }
            else if (id == 3)
            {
                g_forwardKp = clip_float(absf_local((float)value) / 10.0f, 0.0f, 50.0f);
            }
            else if (id == 4)
            {
                g_forwardKi = clip_float(absf_local((float)value) / 100.0f, 0.0f, 20.0f);
            }
            else if (id == 5)
            {
                g_forwardKd = clip_float(absf_local((float)value) / 10.0f, 0.0f, 20.0f);
            }
            else if (id == 6)
            {
                g_turnKp = clip_float(absf_local((float)value) / 10.0f, 0.0f, 50.0f);
            }
            else if (id == 7)
            {
                g_turnKi = clip_float(absf_local((float)value) / 100.0f, 0.0f, 20.0f);
            }
            else if (id == 8)
            {
                g_turnKd = clip_float(absf_local((float)value) / 10.0f, 0.0f, 20.0f);
            }
            else if (id == 9)
            {
                g_maxForwardCmd = clip_float(absf_local((float)value), 10.0f, 300.0f);
            }
            else if (id == 10)
            {
                g_maxTurnCmd = clip_float(absf_local((float)value), 10.0f, 300.0f);
            }
            return;
        }
    }

    /*
     * 4. 直接速度命令。
     *
     * 这类命令便于串口助手或脚本调试：
     *   [car,forward,turn]
     *   [vel,forward,turn]
     *   [cmd,forward,turn]
     */
    if (str_equal(tok[0], "car") || str_equal(tok[0], "vel") || str_equal(tok[0], "cmd"))
    {
        if (g_safetyLocked)
        {
            return;
        }

        if (n >= 3)
        {
            g_targetForwardSpeed = clip_float((float)atof(tok[1]), -g_maxForwardCmd, g_maxForwardCmd);
            g_targetTurnSpeed = clip_float((float)atof(tok[2]), -g_maxTurnCmd, g_maxTurnCmd);
            g_carEnable = 1;
            return;
        }
    }

    /*
     * 5. 普通按键命令。
     *
     * 名称按键：
     *   [key,tracing,down]   -> 八路灰度循迹模式
     *   [key,Bluetooth,down] -> 蓝牙遥控模式
     *
     * 数字按键兼容旧代码：
     *   key 1 停止输出，key 2 使能，key 3 清速度，key 4 停止并失能。
     */
    if (str_equal(tok[0], "k") || str_equal(tok[0], "key"))
    {
        if (n >= 3)
        {
            int id;
            int isDown = str_equal(tok[2], "d") || str_equal(tok[2], "down");

            if (g_safetyLocked)
            {
                return;
            }

            if (isDown && str_is_tracing_name(tok[1]))
            {
                App_StartTracingMode();
                return;
            }

            if (isDown && str_is_bluetooth_name(tok[1]))
            {
                App_StartBluetoothMode();
                return;
            }

            id = atoi(tok[1]);

            if (id == 1 && isDown)
            {
                g_carEnable = 0;
                g_targetForwardSpeed = 0.0f;
                g_targetTurnSpeed = 0.0f;
            }
            else if (id == 2 && isDown)
            {
                g_carEnable = 1;
            }
            else if (id == 3 && isDown)
            {
                g_targetForwardSpeed = 0.0f;
                g_targetTurnSpeed = 0.0f;
            }
            else if (id == 4 && isDown)
            {
                g_targetForwardSpeed = 0.0f;
                g_targetTurnSpeed = 0.0f;
                g_carEnable = 0;
            }
            return;
        }
    }
}

/* 从串口环形缓冲区取字节，提取 [ ... ] 包后交给 BT_ParsePacket。 */
void BT_Process(void)
{
    static uint8_t receiving = 0;
    static uint8_t index = 0;
    static char packet[96];
    uint8_t byte;
    uint8_t gotPacket = 0;

    while (Serial_ReadByte(&byte))
    {
        char c = (char)byte;

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
            gotPacket = 1;
            break;
        }

        if (c == '\r' || c == '\n')
        {
            continue;
        }

        if (index < sizeof(packet) - 1)
        {
            packet[index++] = c;
        }
        else
        {
            receiving = 0;
            index = 0;
        }
    }

    if (gotPacket)
    {
        apply_packet(packet);
        g_lastCmdTickMs = 0;
    }
}
