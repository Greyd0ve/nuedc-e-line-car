#include "app_e_serial.h"
#include "app_e_car.h"
#include "Serial.h"
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define E_SERIAL_PACKET_BUF_SIZE 96U
#define E_SERIAL_FIELD_MAX       8U
#define E_SERIAL_FIELD_LEN       16U
#define E_SERIAL_PLOT_PERIOD_MS  100U
#define E_SERIAL_JOYSTICK_ACK_MS 500U

extern volatile float g_targetForwardSpeed;
extern volatile float g_forwardSpeed;
extern volatile int16_t g_lineError;
extern volatile int16_t g_leftPwm;

static uint8_t s_serialReady = 0U;
static uint8_t s_receiving = 0U;
static uint8_t s_packetLen = 0U;
static char s_packet[E_SERIAL_PACKET_BUF_SIZE];
static uint16_t s_plotMs = 0U;
static uint32_t s_serialMs = 0U;
static uint32_t s_lastJoystickAckMs = 0U;
static uint8_t s_joystickAcked = 0U;

static char ESerial_ToLower(char c)
{
    if (c >= 'A' && c <= 'Z')
    {
        return (char)(c + ('a' - 'A'));
    }
    return c;
}

static uint8_t ESerial_StrEqualIgnoreCase(const char *a, const char *b)
{
    while (*a != '\0' && *b != '\0')
    {
        if (ESerial_ToLower(*a) != ESerial_ToLower(*b))
        {
            return 0U;
        }
        a++;
        b++;
    }
    return (uint8_t)(*a == '\0' && *b == '\0');
}

static char *ESerial_Trim(char *text)
{
    char *end;

    while (*text == ' ' || *text == '\t')
    {
        text++;
    }

    end = text + strlen(text);
    while (end > text && (end[-1] == ' ' || end[-1] == '\t'))
    {
        end--;
    }
    *end = '\0';

    return text;
}

static uint8_t ESerial_ParseFloat(const char *text, float *value)
{
    char *endPtr;
    double parsed;

    if (text == 0 || value == 0)
    {
        return 0U;
    }

    parsed = strtod(text, &endPtr);
    if (endPtr == text)
    {
        return 0U;
    }

    while (*endPtr == ' ' || *endPtr == '\t')
    {
        endPtr++;
    }
    if (*endPtr != '\0')
    {
        return 0U;
    }

    *value = (float)parsed;
    return 1U;
}

static int32_t ESerial_RoundToInt(float value)
{
    if (value >= 0.0f)
    {
        return (int32_t)(value + 0.5f);
    }
    return (int32_t)(value - 0.5f);
}

static void ESerial_SendFixedValue(float value, uint8_t decimals)
{
    int32_t scale = 1;
    int32_t scaled;
    int32_t integerPart;
    int32_t fracPart;
    uint8_t i;

    for (i = 0U; i < decimals; i++)
    {
        scale *= 10;
    }

    if (value >= 0.0f)
    {
        scaled = (int32_t)(value * (float)scale + 0.5f);
    }
    else
    {
        scaled = (int32_t)(value * (float)scale - 0.5f);
    }

    if (decimals == 0U)
    {
        Serial_Printf("%ld", (long)scaled);
        return;
    }

    integerPart = scaled / scale;
    fracPart = scaled % scale;
    if (fracPart < 0)
    {
        fracPart = -fracPart;
    }

    Serial_Printf("%ld.", (long)integerPart);
    if (decimals == 1U)
    {
        Serial_Printf("%01ld", (long)fracPart);
    }
    else if (decimals == 2U)
    {
        Serial_Printf("%02ld", (long)fracPart);
    }
    else
    {
        Serial_Printf("%03ld", (long)fracPart);
    }
}

static void ESerial_SendSetOkFloat(const char *name, float value, uint8_t decimals)
{
    Serial_Printf("[status,ok,set,%s,", name);
    ESerial_SendFixedValue(value, decimals);
    Serial_SendString("]\r\n");
}

static void ESerial_SendSetOkInt(const char *name, int32_t value)
{
    Serial_Printf("[status,ok,set,%s,%ld]\r\n", name, (long)value);
}

static void ESerial_SendRangeError(const char *name)
{
    Serial_Printf("[status,err,range,%s]\r\n", name);
}

static void ESerial_SendUnknownError(const char *name)
{
    Serial_Printf("[status,err,unknown,%s]\r\n", name);
}

static void ESerial_SendBadPacket(void)
{
    Serial_SendString("[status,err,bad-packet]\r\n");
}

static uint8_t ESerial_ValueInRange(float value, float minVal, float maxVal)
{
    return (uint8_t)(value >= minVal && value <= maxVal);
}

static uint8_t ESerial_SetLapFromFloat(const char *replyName, float value)
{
    int32_t lap;

    lap = ESerial_RoundToInt(value);
    if (lap < 1 || lap > 5)
    {
        ESerial_SendRangeError(replyName);
        return 0U;
    }

    if (!ECar_SetTargetLap((uint8_t)lap))
    {
        Serial_SendString("[status,err,busy]\r\n");
        return 0U;
    }

    ESerial_SendSetOkInt(replyName, lap);
    return 1U;
}

static uint8_t ESerial_SetNamedParam(const char *name, float value)
{
    if (ESerial_StrEqualIgnoreCase(name, "target") ||
        ESerial_StrEqualIgnoreCase(name, "base") ||
        ESerial_StrEqualIgnoreCase(name, "base_speed"))
    {
        if (!ESerial_ValueInRange(value, 0.0f, 60.0f))
        {
            ESerial_SendRangeError(name);
            return 0U;
        }
        g_eCarParam.base_speed = value;
        ESerial_SendSetOkFloat(name, g_eCarParam.base_speed, 0U);
        return 1U;
    }

    if (ESerial_StrEqualIgnoreCase(name, "Kp") ||
        ESerial_StrEqualIgnoreCase(name, "line_kp"))
    {
        if (!ESerial_ValueInRange(value, 0.0f, 3.0f))
        {
            ESerial_SendRangeError(name);
            return 0U;
        }
        g_eCarParam.line_kp = value;
        ESerial_SendSetOkFloat(name, g_eCarParam.line_kp, 2U);
        return 1U;
    }

    if (ESerial_StrEqualIgnoreCase(name, "Kd") ||
        ESerial_StrEqualIgnoreCase(name, "line_kd"))
    {
        if (!ESerial_ValueInRange(value, 0.0f, 8.0f))
        {
            ESerial_SendRangeError(name);
            return 0U;
        }
        g_eCarParam.line_kd = value;
        ESerial_SendSetOkFloat(name, g_eCarParam.line_kd, 2U);
        return 1U;
    }

    if (ESerial_StrEqualIgnoreCase(name, "Ki") ||
        ESerial_StrEqualIgnoreCase(name, "n") ||
        ESerial_StrEqualIgnoreCase(name, "lap"))
    {
        return ESerial_SetLapFromFloat(name, value);
    }

    if (ESerial_StrEqualIgnoreCase(name, "recover_speed"))
    {
        if (!ESerial_ValueInRange(value, 0.0f, 40.0f))
        {
            ESerial_SendRangeError(name);
            return 0U;
        }
        g_eCarParam.recover_speed = value;
        ESerial_SendSetOkFloat(name, g_eCarParam.recover_speed, 0U);
        return 1U;
    }

    if (ESerial_StrEqualIgnoreCase(name, "corner_forward"))
    {
        if (!ESerial_ValueInRange(value, 0.0f, 30.0f))
        {
            ESerial_SendRangeError(name);
            return 0U;
        }
        g_eCarParam.corner_forward_speed = value;
        ESerial_SendSetOkFloat(name, g_eCarParam.corner_forward_speed, 0U);
        return 1U;
    }

    if (ESerial_StrEqualIgnoreCase(name, "corner_turn"))
    {
        if (!ESerial_ValueInRange(value, -100.0f, 100.0f))
        {
            ESerial_SendRangeError(name);
            return 0U;
        }
        g_eCarParam.corner_turn_speed = value;
        ESerial_SendSetOkFloat(name, g_eCarParam.corner_turn_speed, 0U);
        return 1U;
    }

    if (ESerial_StrEqualIgnoreCase(name, "turnLimit") ||
        ESerial_StrEqualIgnoreCase(name, "turn_limit"))
    {
        if (!ESerial_ValueInRange(value, 0.0f, 150.0f))
        {
            ESerial_SendRangeError(name);
            return 0U;
        }
        g_eCarParam.turn_limit = value;
        ESerial_SendSetOkFloat(name, g_eCarParam.turn_limit, 0U);
        return 1U;
    }

    if (ESerial_StrEqualIgnoreCase(name, "lost_timeout_ms"))
    {
        if (!ESerial_ValueInRange(value, 100.0f, 2000.0f))
        {
            ESerial_SendRangeError(name);
            return 0U;
        }
        g_eCarParam.lost_timeout_ms = (uint16_t)ESerial_RoundToInt(value);
        ESerial_SendSetOkInt(name, (int32_t)g_eCarParam.lost_timeout_ms);
        return 1U;
    }

    if (ESerial_StrEqualIgnoreCase(name, "lap_pulse_default"))
    {
        if (!ESerial_ValueInRange(value, 10000.0f, 60000.0f))
        {
            ESerial_SendRangeError(name);
            return 0U;
        }
        g_eCarParam.lap_pulse_default = ESerial_RoundToInt(value);
        ESerial_SendSetOkInt(name, g_eCarParam.lap_pulse_default);
        return 1U;
    }

    if (ESerial_StrEqualIgnoreCase(name, "corner_black"))
    {
        int32_t count;

        count = ESerial_RoundToInt(value);
        if (count < 1 || count > 8)
        {
            ESerial_SendRangeError(name);
            return 0U;
        }
        g_eCarParam.corner_black_count_th = (uint8_t)count;
        ESerial_SendSetOkInt(name, count);
        return 1U;
    }

    ESerial_SendUnknownError(name);
    return 0U;
}

static uint8_t ESerial_SplitFields(char *packet, char *fields[], uint8_t maxFields)
{
    uint8_t count = 0U;
    char *p;

    if (packet == 0 || fields == 0 || maxFields == 0U)
    {
        return 0U;
    }

    fields[count++] = ESerial_Trim(packet);
    p = packet;
    while (*p != '\0')
    {
        if (*p == ',')
        {
            *p = '\0';
            if (count >= maxFields)
            {
                return 0U;
            }
            fields[count++] = ESerial_Trim(p + 1);
        }
        p++;
    }

    return count;
}

static void ESerial_SendParamSnapshot(void)
{
    Serial_Printf("[ecar,val,n,%u,base,", ECar_GetTargetLap());
    ESerial_SendFixedValue(g_eCarParam.base_speed, 0U);
    Serial_SendString(",Kp,");
    ESerial_SendFixedValue(g_eCarParam.line_kp, 2U);
    Serial_SendString(",Kd,");
    ESerial_SendFixedValue(g_eCarParam.line_kd, 2U);
    Serial_SendString(",turnLimit,");
    ESerial_SendFixedValue(g_eCarParam.turn_limit, 0U);
    Serial_SendString("]\r\n");
}

static void ESerial_SendStateSnapshot(void)
{
    Serial_Printf("[ecar,state,s,%u,n,%u,lap,%u,corner,%u,err,%d,pwm,%d,prog,",
                  (unsigned int)ECar_GetState(),
                  (unsigned int)ECar_GetTargetLap(),
                  (unsigned int)ECar_GetLapCount(),
                  (unsigned int)ECar_GetCornerCount(),
                  (int)g_lineError,
                  (int)g_leftPwm);
    ESerial_SendFixedValue(ECar_GetLapProgress() * 100.0f, 0U);
    Serial_Printf(",fault,%u,time,%lu]\r\n",
                  (unsigned int)ECar_GetFaultCode(),
                  (unsigned long)ECar_GetRunningTimeMs());
}

static void ESerial_HandleSlider(char *fields[], uint8_t fieldCount)
{
    float value;

    if (fieldCount < 3U)
    {
        ESerial_SendBadPacket();
        return;
    }

    if (!ESerial_ParseFloat(fields[2], &value))
    {
        ESerial_SendRangeError(fields[1]);
        return;
    }

    (void)ESerial_SetNamedParam(fields[1], value);
}

static void ESerial_HandleKey(char *fields[], uint8_t fieldCount)
{
    if (fieldCount >= 3U &&
        ESerial_StrEqualIgnoreCase(fields[1], "emergency") &&
        ESerial_StrEqualIgnoreCase(fields[2], "down"))
    {
        ECar_Stop();
        Serial_SendString("[status,ok,emergency]\r\n");
        return;
    }

    ESerial_SendUnknownError((fieldCount >= 2U) ? fields[1] : "key");
}

static void ESerial_HandleJoystick(void)
{
    if (!s_joystickAcked ||
        (uint32_t)(s_serialMs - s_lastJoystickAckMs) >= E_SERIAL_JOYSTICK_ACK_MS)
    {
        Serial_SendString("[status,ignored,joystick]\r\n");
        s_lastJoystickAckMs = s_serialMs;
        s_joystickAcked = 1U;
    }
}

static void ESerial_HandleECar(char *fields[], uint8_t fieldCount)
{
    float value;

    if (fieldCount < 2U)
    {
        ESerial_SendBadPacket();
        return;
    }

    if (ESerial_StrEqualIgnoreCase(fields[1], "start"))
    {
        ECar_Start();
        Serial_SendString("[status,ok,start]\r\n");
        return;
    }

    if (ESerial_StrEqualIgnoreCase(fields[1], "stop"))
    {
        ECar_Stop();
        Serial_SendString("[status,ok,stop]\r\n");
        return;
    }

    if (ESerial_StrEqualIgnoreCase(fields[1], "get"))
    {
        ESerial_SendParamSnapshot();
        return;
    }

    if (ESerial_StrEqualIgnoreCase(fields[1], "status"))
    {
        ESerial_SendStateSnapshot();
        return;
    }

    if (ESerial_StrEqualIgnoreCase(fields[1], "set"))
    {
        if (fieldCount < 4U)
        {
            ESerial_SendBadPacket();
            return;
        }
        if (!ESerial_ParseFloat(fields[3], &value))
        {
            ESerial_SendRangeError(fields[2]);
            return;
        }
        (void)ESerial_SetNamedParam(fields[2], value);
        return;
    }

    ESerial_SendUnknownError(fields[1]);
}

static void ESerial_HandlePacket(char *packet)
{
    char *fields[E_SERIAL_FIELD_MAX];
    uint8_t fieldCount;

    fieldCount = ESerial_SplitFields(packet, fields, E_SERIAL_FIELD_MAX);
    if (fieldCount == 0U || fields[0][0] == '\0')
    {
        ESerial_SendBadPacket();
        return;
    }

    if (ESerial_StrEqualIgnoreCase(fields[0], "slider"))
    {
        ESerial_HandleSlider(fields, fieldCount);
        return;
    }

    if (ESerial_StrEqualIgnoreCase(fields[0], "key"))
    {
        ESerial_HandleKey(fields, fieldCount);
        return;
    }

    if (ESerial_StrEqualIgnoreCase(fields[0], "joystick"))
    {
        ESerial_HandleJoystick();
        return;
    }

    if (ESerial_StrEqualIgnoreCase(fields[0], "start"))
    {
        ECar_Start();
        Serial_SendString("[status,ok,start]\r\n");
        return;
    }

    if (ESerial_StrEqualIgnoreCase(fields[0], "stop"))
    {
        ECar_Stop();
        Serial_SendString("[status,ok,stop]\r\n");
        return;
    }

    if (ESerial_StrEqualIgnoreCase(fields[0], "get"))
    {
        ESerial_SendParamSnapshot();
        return;
    }

    if (ESerial_StrEqualIgnoreCase(fields[0], "status"))
    {
        ESerial_SendStateSnapshot();
        return;
    }

    if (ESerial_StrEqualIgnoreCase(fields[0], "ecar"))
    {
        ESerial_HandleECar(fields, fieldCount);
        return;
    }

    ESerial_SendUnknownError(fields[0]);
}

void ECar_Serial_Init(void)
{
    s_serialReady = 1U;
    s_receiving = 0U;
    s_packetLen = 0U;
    s_plotMs = 0U;
    s_serialMs = 0U;
    s_lastJoystickAckMs = 0U;
    s_joystickAcked = 0U;
    Serial_SendString("[status,ok,serial,9600]\r\n");
}

void ECar_SerialProcess(void)
{
    uint8_t byte;

    if (!s_serialReady)
    {
        return;
    }

    while (Serial_ReadByte(&byte))
    {
        char ch = (char)byte;

        if (ch == '[')
        {
            s_receiving = 1U;
            s_packetLen = 0U;
            continue;
        }

        if (!s_receiving)
        {
            continue;
        }

        if (ch == ']')
        {
            s_packet[s_packetLen] = '\0';
            s_receiving = 0U;
            ESerial_HandlePacket(s_packet);
            continue;
        }

        if (ch == '\r' || ch == '\n')
        {
            continue;
        }

        if (s_packetLen < (uint8_t)(E_SERIAL_PACKET_BUF_SIZE - 1U))
        {
            s_packet[s_packetLen++] = ch;
        }
        else
        {
            s_receiving = 0U;
            s_packetLen = 0U;
            ESerial_SendBadPacket();
        }
    }
}

void ECar_SerialPlot10ms(void)
{
    if (!s_serialReady)
    {
        return;
    }

    s_serialMs += 10U;
    s_plotMs += 10U;
    if (s_plotMs < E_SERIAL_PLOT_PERIOD_MS)
    {
        return;
    }

    s_plotMs = 0U;
    Serial_Printf("[plot,%d,%d,%d,%d]\r\n",
                  (int)g_targetForwardSpeed,
                  (int)g_forwardSpeed,
                  (int)g_lineError,
                  (int)g_leftPwm);
}
