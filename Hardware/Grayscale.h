/* 八路灰度循迹模块接口。AD0/AD1/AD2 选择通道，OUT 读取当前通道黑白状态。 */
#ifndef __GRAYSCALE_H
#define __GRAYSCALE_H

#include <stdint.h>

#define GRAYSCALE_CHANNELS 8

void Grayscale_Init(void);
void Grayscale_ReadAll(uint8_t sensor[GRAYSCALE_CHANNELS]);
uint8_t Grayscale_ReadOne(uint8_t channel);
uint8_t Grayscale_RawOUT(void);

#endif
