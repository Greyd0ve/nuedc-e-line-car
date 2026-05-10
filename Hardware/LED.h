/* 板载 PC13 LED 接口。当前用于启动、过点、急停等提示；如接蜂鸣器，可参考此模块扩展。 */
#ifndef __LED_H
#define __LED_H

void LED_Init(void);
void LED_ON(void);
void LED_OFF(void);
void LED_Turn(void);

#endif
