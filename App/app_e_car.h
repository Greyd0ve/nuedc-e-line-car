#ifndef __APP_E_CAR_H
#define __APP_E_CAR_H

#include <stdint.h>

typedef enum
{
    E_CAR_IDLE = 0,
    E_CAR_READY,
    E_CAR_LINE_RUN,
    E_CAR_CORNER_ENTER,
    E_CAR_CORNER_TURN,
    E_CAR_LINE_RECOVER,
    E_CAR_FINISH,
    E_CAR_FAULT
} ECarState_t;

typedef struct
{
    float base_speed;
    float recover_speed;
    float corner_forward_speed;
    float corner_turn_speed;

    float line_kp;
    float line_kd;
    float turn_limit;

    uint16_t lost_timeout_ms;
    uint16_t recover_stable_ms;
    uint16_t corner_min_turn_ms;
    uint16_t corner_max_turn_ms;

    int32_t min_corner_interval_pulse;
    int32_t lap_pulse_default;

    uint8_t corner_black_count_th;
} ECarParam_t;

extern ECarParam_t g_eCarParam;

void ECar_Init(void);
void ECar_Reset(void);
void ECar_Start(void);
void ECar_Stop(void);
void ECar_Control10ms(void);
void ECar_KeyProcess(void);
void ECar_ShowStatus(void);
float ECar_GetLapProgress(void);
uint8_t ECar_GetLapCount(void);
uint8_t ECar_GetTargetLap(void);
ECarState_t ECar_GetState(void);
void ECar_PromptTick1ms(void);

#endif
