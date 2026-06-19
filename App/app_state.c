#include "app_state.h"
#include <stdint.h>

extern volatile WorkMode_t g_workMode;
extern volatile LocalMode_t g_localMode;
extern volatile uint8_t g_safetyLocked;
extern volatile uint32_t g_lastCmdTickMs;

extern void App_StateTaskReset(void);
extern void App_StateForcePWMZero(void);
extern void App_StatePromptStart(uint16_t ms);

void App_EmergencyStop(void)
{
    g_safetyLocked = 1;
    g_workMode = WORK_STANDBY;
    g_localMode = LOCAL_STANDBY;
    App_StateTaskReset();
    App_StateForcePWMZero();
    App_StatePromptStart(500);
}

void App_UnlockControl(void)
{
    g_safetyLocked = 0;
    g_workMode = WORK_STANDBY;
    g_localMode = LOCAL_STANDBY;
    App_StateTaskReset();
    g_lastCmdTickMs = 0;
    App_StateForcePWMZero();
    App_StatePromptStart(180);
}
