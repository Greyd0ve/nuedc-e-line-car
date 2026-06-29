#include "stm32f10x.h"
#include "Serial.h"
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>

#define SERIAL_BAUD_RATE    9600U
#define SERIAL_RX_BUF_SIZE  256U

static volatile uint8_t Serial_RxBuf[SERIAL_RX_BUF_SIZE];
static volatile uint16_t Serial_RxW = 0U;
static volatile uint16_t Serial_RxR = 0U;
static volatile uint32_t Serial_RxOverflowCount = 0U;

uint8_t Serial_RxData = 0U;
uint8_t Serial_RxFlag = 0U;

void Serial_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate = SERIAL_BAUD_RATE;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_Init(USART1, &USART_InitStructure);

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_Init(&NVIC_InitStructure);

    USART_Cmd(USART1, ENABLE);
}

void Serial_SendByte(uint8_t byte)
{
    USART_SendData(USART1, byte);
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
    {
    }
}

void Serial_SendArray(const uint8_t *array, uint16_t length)
{
    uint16_t i;

    if (array == 0)
    {
        return;
    }

    for (i = 0U; i < length; i++)
    {
        Serial_SendByte(array[i]);
    }
}

void Serial_SendString(const char *string)
{
    if (string == 0)
    {
        return;
    }

    while (*string != '\0')
    {
        Serial_SendByte((uint8_t)(*string));
        string++;
    }
}

static uint32_t Serial_Pow(uint32_t x, uint32_t y)
{
    uint32_t result = 1U;

    while (y > 0U)
    {
        result *= x;
        y--;
    }

    return result;
}

void Serial_SendNumber(uint32_t number, uint8_t length)
{
    uint8_t i;

    for (i = 0U; i < length; i++)
    {
        Serial_SendByte((uint8_t)(number / Serial_Pow(10U, (uint32_t)(length - i - 1U)) % 10U + '0'));
    }
}

int fputc(int ch, FILE *f)
{
    (void)f;
    Serial_SendByte((uint8_t)ch);
    return ch;
}

void Serial_Printf(const char *format, ...)
{
    char string[100];
    va_list arg;

    if (format == 0)
    {
        return;
    }

    va_start(arg, format);
    vsnprintf(string, sizeof(string), format, arg);
    va_end(arg);

    string[sizeof(string) - 1U] = '\0';
    Serial_SendString(string);
}

uint8_t Serial_GetRxFlag(void)
{
    if (Serial_RxFlag != 0U)
    {
        Serial_RxFlag = 0U;
        return 1U;
    }

    return 0U;
}

uint8_t Serial_GetRxData(void)
{
    return Serial_RxData;
}

uint8_t Serial_ReadByte(uint8_t *byte)
{
    if (byte == 0)
    {
        return 0U;
    }

    if (Serial_RxR == Serial_RxW)
    {
        return 0U;
    }

    *byte = Serial_RxBuf[Serial_RxR];
    Serial_RxR = (uint16_t)((Serial_RxR + 1U) % SERIAL_RX_BUF_SIZE);
    return 1U;
}

uint32_t Serial_GetRxOverflowCount(void)
{
    return Serial_RxOverflowCount;
}

void USART1_IRQHandler(void)
{
    if (USART_GetITStatus(USART1, USART_IT_RXNE) == SET)
    {
        uint8_t byte;
        uint16_t next;

        byte = (uint8_t)USART_ReceiveData(USART1);
        Serial_RxData = byte;
        Serial_RxFlag = 1U;

        next = (uint16_t)((Serial_RxW + 1U) % SERIAL_RX_BUF_SIZE);
        if (next != Serial_RxR)
        {
            Serial_RxBuf[Serial_RxW] = byte;
            Serial_RxW = next;
        }
        else
        {
            Serial_RxOverflowCount++;
        }
    }
}
