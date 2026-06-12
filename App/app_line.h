#ifndef __APP_LINE_H
#define __APP_LINE_H

void App_Line_GPIOForceInit(void);
void App_Line_ResetState(void);
void App_Line_Update(void);
float App_Line_CalcTurnCmd(void);
void App_Line_TracingControl10ms(void);

#endif
