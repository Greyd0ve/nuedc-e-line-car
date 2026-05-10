/* 板载 K1~K4 按键接口。Key_Tick() 在 1ms 定时中断里消抖，Key_GetNum() 在主循环读取事件。 */
#ifndef __KEY_H
#define __KEY_H

void Key_Init(void);
uint8_t Key_GetNum(void);
void Key_Tick(void);

#endif
