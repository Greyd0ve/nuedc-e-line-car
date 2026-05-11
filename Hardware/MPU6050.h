#ifndef __MPU6050_H
#define __MPU6050_H

#include <stdint.h>

/*
 * MPU6050 driver for STM32F103C8T6 standard peripheral library.
 * I2C bus is shared with OLED:
 *   SCL -> PB8
 *   SDA -> PB9
 *
 * Default MPU6050 7-bit address is 0x68 when AD0 is tied to GND.
 * If your module AD0 is tied to VCC, change MPU6050_ADDR_7BIT to 0x69.
 */
#define MPU6050_ADDR_7BIT          0x68U

#define MPU6050_OK                 0U
#define MPU6050_ERR_NACK           1U
#define MPU6050_ERR_ID             2U
#define MPU6050_ERR_PARAM          3U

typedef struct
{
    int16_t AccX;
    int16_t AccY;
    int16_t AccZ;
    int16_t Temp;
    int16_t GyroX;
    int16_t GyroY;
    int16_t GyroZ;
} MPU6050_RawData_t;

typedef struct
{
    float AccX_g;
    float AccY_g;
    float AccZ_g;
    float Temp_C;
    float GyroX_dps;
    float GyroY_dps;
    float GyroZ_dps;
} MPU6050_Data_t;

void MPU6050_InitBus(void);
uint8_t MPU6050_Init(void);
uint8_t MPU6050_GetID(void);
uint8_t MPU6050_ReadReg(uint8_t regAddr);
uint8_t MPU6050_WriteReg(uint8_t regAddr, uint8_t data);
uint8_t MPU6050_ReadLen(uint8_t regAddr, uint8_t *buf, uint8_t len);
uint8_t MPU6050_ReadRaw(MPU6050_RawData_t *raw);
uint8_t MPU6050_ReadData(MPU6050_Data_t *data);

#endif
