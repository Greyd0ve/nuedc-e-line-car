/* USART1 蓝牙/USB-TTL 串口接口。TX=PA9，RX=PA10，包含发送函数和接收环形缓冲区读取函数。 */
#ifndef __SERIAL_H
#define __SERIAL_H

#include <stdio.h>
#include <stdint.h>

void Serial_Init(void);
uint8_t Serial_ReadByte(uint8_t *byte);
uint32_t Serial_GetRxOverflowCount(void);
void Serial_SendByte(uint8_t Byte);
void Serial_SendArray(uint8_t *Array, uint16_t Length);
void Serial_SendString(char *String);
void Serial_SendNumber(uint32_t Number, uint8_t Length);
void Serial_Printf(char *format, ...);

uint8_t Serial_GetRxFlag(void);
uint8_t Serial_GetRxData(void);

#endif
