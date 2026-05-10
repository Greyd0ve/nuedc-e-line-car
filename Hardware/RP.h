/* 板载电位器 ADC 读取接口。本版最高速度已改用蓝牙 [slider,RP,x]，RP 模块保留备用。 */
#ifndef __RP_H
#define __RP_H

void RP_Init(void);
uint16_t RP_GetValue(uint8_t n);

#endif
