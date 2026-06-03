#ifndef __BEEPLED_H
#define __BEEPLED_H

#include <stdint.h>

void BeepLed_Init(void);

void Beep_On(void);
void Beep_Off(void);
void Beep_Turn(void);

void LedExt_On(void);
void LedExt_Off(void);
void LedExt_Turn(void);

void BeepLed_AllOn(void);
void BeepLed_AllOff(void);
void BeepLed_AllTurn(void);

#endif
