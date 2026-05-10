/* 编码器模块接口。用于获取左右轮在一个控制周期内的增量脉冲，主控用它做速度闭环和里程估计。 */
#ifndef __ENCODER_H
#define __ENCODER_H

#include <stdint.h>

void Encoder_Init(void);
int16_t Encoder_GetLeftDelta(void);
int16_t Encoder_GetRightDelta(void);
void Encoder_ClearAll(void);

#endif
