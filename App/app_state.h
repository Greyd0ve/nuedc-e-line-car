#ifndef __APP_STATE_H
#define __APP_STATE_H

typedef enum
{
    WORK_STANDBY = 0,
    WORK_BT = 1
} WorkMode_t;

typedef enum
{
    LOCAL_STANDBY = 0,
    LOCAL_MPU_DEBUG = 1
} LocalMode_t;

void App_EmergencyStop(void);
void App_UnlockControl(void);

#endif
