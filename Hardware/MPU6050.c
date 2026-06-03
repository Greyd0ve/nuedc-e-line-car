#include "stm32f10x.h"
#include "MPU6050.h"

/* ================================================================
 * MPU6050 software I2C driver
 * Board wiring:
 *   SCL -> PB8
 *   SDA -> PB9
 * This bus can be shared with the current OLED module, because both
 * devices use different I2C addresses and PB8/PB9 are configured as
 * open-drain outputs.
 * ================================================================ */

#define MPU6050_SCL_PORT           GPIOB
#define MPU6050_SCL_PIN            GPIO_Pin_8
#define MPU6050_SDA_PORT           GPIOB
#define MPU6050_SDA_PIN            GPIO_Pin_9

#define MPU6050_ADDR_WRITE         ((uint8_t)(MPU6050_ADDR_7BIT << 1))
#define MPU6050_ADDR_READ          ((uint8_t)((MPU6050_ADDR_7BIT << 1) | 0x01U))

/* MPU6050 register map */
#define MPU6050_REG_SMPLRT_DIV     0x19U
#define MPU6050_REG_CONFIG         0x1AU
#define MPU6050_REG_GYRO_CONFIG    0x1BU
#define MPU6050_REG_ACCEL_CONFIG   0x1CU
#define MPU6050_REG_ACCEL_XOUT_H   0x3BU
#define MPU6050_REG_PWR_MGMT_1     0x6BU
#define MPU6050_REG_PWR_MGMT_2     0x6CU
#define MPU6050_REG_WHO_AM_I       0x75U

static void MPU6050_DelayUs(uint16_t us)
{
    uint16_t i;
    while (us--)
    {
        /* Approximate delay for 72 MHz STM32F103. I2C is intentionally slow
         * and tolerant here, suitable for shared OLED + MPU6050 software I2C. */
        for (i = 0; i < 10; i++)
        {
            __NOP();
        }
    }
}

static void MPU6050_DelayMs(uint16_t ms)
{
    while (ms--)
    {
        MPU6050_DelayUs(1000);
    }
}

static void MPU6050_SCL(uint8_t level)
{
    GPIO_WriteBit(MPU6050_SCL_PORT, MPU6050_SCL_PIN, (BitAction)(level ? Bit_SET : Bit_RESET));
}

static void MPU6050_SDA(uint8_t level)
{
    GPIO_WriteBit(MPU6050_SDA_PORT, MPU6050_SDA_PIN, (BitAction)(level ? Bit_SET : Bit_RESET));
}

static uint8_t MPU6050_ReadSDA(void)
{
    return (uint8_t)GPIO_ReadInputDataBit(MPU6050_SDA_PORT, MPU6050_SDA_PIN);
}

void MPU6050_InitBus(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = MPU6050_SCL_PIN | MPU6050_SDA_PIN;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* Release I2C bus. External pull-up or module pull-up should keep the lines high. */
    MPU6050_SCL(1);
    MPU6050_SDA(1);
    MPU6050_DelayUs(10);
}

static void MPU6050_I2C_Start(void)
{
    MPU6050_SDA(1);
    MPU6050_SCL(1);
    MPU6050_DelayUs(4);
    MPU6050_SDA(0);
    MPU6050_DelayUs(4);
    MPU6050_SCL(0);
    MPU6050_DelayUs(4);
}

static void MPU6050_I2C_Stop(void)
{
    MPU6050_SDA(0);
    MPU6050_DelayUs(4);
    MPU6050_SCL(1);
    MPU6050_DelayUs(4);
    MPU6050_SDA(1);
    MPU6050_DelayUs(4);
}

static uint8_t MPU6050_I2C_WaitAck(void)
{
    uint16_t timeout;

    MPU6050_SDA(1);      /* release SDA */
    MPU6050_DelayUs(2);
    MPU6050_SCL(1);
    MPU6050_DelayUs(2);

    timeout = 0;
    while (MPU6050_ReadSDA())
    {
        timeout++;
        if (timeout > 250U)
        {
            MPU6050_SCL(0);
            MPU6050_I2C_Stop();
            return MPU6050_ERR_NACK;
        }
    }

    MPU6050_SCL(0);
    MPU6050_DelayUs(2);
    return MPU6050_OK;
}

static void MPU6050_I2C_Ack(void)
{
    MPU6050_SCL(0);
    MPU6050_SDA(0);
    MPU6050_DelayUs(2);
    MPU6050_SCL(1);
    MPU6050_DelayUs(4);
    MPU6050_SCL(0);
    MPU6050_SDA(1);
    MPU6050_DelayUs(2);
}

static void MPU6050_I2C_NAck(void)
{
    MPU6050_SCL(0);
    MPU6050_SDA(1);
    MPU6050_DelayUs(2);
    MPU6050_SCL(1);
    MPU6050_DelayUs(4);
    MPU6050_SCL(0);
    MPU6050_DelayUs(2);
}

static uint8_t MPU6050_I2C_SendByte(uint8_t byte)
{
    uint8_t i;

    for (i = 0; i < 8; i++)
    {
        MPU6050_SCL(0);
        MPU6050_SDA((uint8_t)((byte & 0x80U) != 0U));
        byte <<= 1;
        MPU6050_DelayUs(2);
        MPU6050_SCL(1);
        MPU6050_DelayUs(4);
    }

    MPU6050_SCL(0);
    return MPU6050_I2C_WaitAck();
}

static uint8_t MPU6050_I2C_ReadByte(uint8_t ack)
{
    uint8_t i;
    uint8_t byte;

    byte = 0;
    MPU6050_SDA(1);      /* release SDA */

    for (i = 0; i < 8; i++)
    {
        MPU6050_SCL(0);
        MPU6050_DelayUs(2);
        MPU6050_SCL(1);
        byte <<= 1;
        if (MPU6050_ReadSDA())
        {
            byte |= 0x01U;
        }
        MPU6050_DelayUs(4);
    }

    MPU6050_SCL(0);
    if (ack)
    {
        MPU6050_I2C_Ack();
    }
    else
    {
        MPU6050_I2C_NAck();
    }

    return byte;
}

uint8_t MPU6050_WriteReg(uint8_t regAddr, uint8_t data)
{
    uint8_t err;

    MPU6050_I2C_Start();

    err = MPU6050_I2C_SendByte(MPU6050_ADDR_WRITE);
    if (err != MPU6050_OK) { return err; }

    err = MPU6050_I2C_SendByte(regAddr);
    if (err != MPU6050_OK) { MPU6050_I2C_Stop(); return err; }

    err = MPU6050_I2C_SendByte(data);
    MPU6050_I2C_Stop();

    return err;
}

uint8_t MPU6050_ReadReg(uint8_t regAddr)
{
    uint8_t data;

    data = 0xFFU;

    MPU6050_I2C_Start();
    if (MPU6050_I2C_SendByte(MPU6050_ADDR_WRITE) != MPU6050_OK) { return data; }
    if (MPU6050_I2C_SendByte(regAddr) != MPU6050_OK) { MPU6050_I2C_Stop(); return data; }

    MPU6050_I2C_Start();
    if (MPU6050_I2C_SendByte(MPU6050_ADDR_READ) != MPU6050_OK) { return data; }
    data = MPU6050_I2C_ReadByte(0);
    MPU6050_I2C_Stop();

    return data;
}

uint8_t MPU6050_ReadLen(uint8_t regAddr, uint8_t *buf, uint8_t len)
{
    uint8_t i;
    uint8_t err;

    if (buf == 0 || len == 0)
    {
        return MPU6050_ERR_PARAM;
    }

    MPU6050_I2C_Start();

    err = MPU6050_I2C_SendByte(MPU6050_ADDR_WRITE);
    if (err != MPU6050_OK) { return err; }

    err = MPU6050_I2C_SendByte(regAddr);
    if (err != MPU6050_OK) { MPU6050_I2C_Stop(); return err; }

    MPU6050_I2C_Start();

    err = MPU6050_I2C_SendByte(MPU6050_ADDR_READ);
    if (err != MPU6050_OK) { return err; }

    for (i = 0; i < len; i++)
    {
        buf[i] = MPU6050_I2C_ReadByte((uint8_t)(i != (uint8_t)(len - 1U)));
    }

    MPU6050_I2C_Stop();
    return MPU6050_OK;
}

uint8_t MPU6050_GetID(void)
{
    return MPU6050_ReadReg(MPU6050_REG_WHO_AM_I);
}

uint8_t MPU6050_Init(void)
{
    uint8_t id;
    uint8_t err;

    MPU6050_InitBus();
    MPU6050_DelayMs(100);

    id = MPU6050_GetID();

/*
 * 常见 MPU6050 的 WHO_AM_I 为 0x68。
 * 部分兼容模块或 MPU6500 类芯片会返回 0x70。
 * 当前模块实测 WHO_AM_I = 0x70，因此这里同时接受 0x68 和 0x70。
 */
		if (id != 0x68U && id != 0x70U)
		{
				return MPU6050_ERR_ID;
		}

    /* Wake up, choose X gyro PLL clock. */
    err = MPU6050_WriteReg(MPU6050_REG_PWR_MGMT_1, 0x01U);
    if (err != MPU6050_OK) { return err; }

    /* Enable all axes. */
    err = MPU6050_WriteReg(MPU6050_REG_PWR_MGMT_2, 0x00U);
    if (err != MPU6050_OK) { return err; }

    /* Gyro output rate 1 kHz, sample rate = 1 kHz / (1 + 7) = 125 Hz. */
    err = MPU6050_WriteReg(MPU6050_REG_SMPLRT_DIV, 0x07U);
    if (err != MPU6050_OK) { return err; }

    /* DLPF_CFG = 3: accelerometer about 44 Hz, gyro about 42 Hz. */
    err = MPU6050_WriteReg(MPU6050_REG_CONFIG, 0x03U);
    if (err != MPU6050_OK) { return err; }

    /* Gyro full scale: +/-250 deg/s, sensitivity 131 LSB/(deg/s). */
    err = MPU6050_WriteReg(MPU6050_REG_GYRO_CONFIG, 0x00U);
    if (err != MPU6050_OK) { return err; }

    /* Accel full scale: +/-2g, sensitivity 16384 LSB/g. */
    err = MPU6050_WriteReg(MPU6050_REG_ACCEL_CONFIG, 0x00U);
    if (err != MPU6050_OK) { return err; }

    MPU6050_DelayMs(20);
    return MPU6050_OK;
}

uint8_t MPU6050_ReadRaw(MPU6050_RawData_t *raw)
{
    uint8_t buf[14];
    uint8_t err;

    if (raw == 0)
    {
        return MPU6050_ERR_PARAM;
    }

    err = MPU6050_ReadLen(MPU6050_REG_ACCEL_XOUT_H, buf, 14);
    if (err != MPU6050_OK)
    {
        return err;
    }

    raw->AccX  = (int16_t)((uint16_t)buf[0]  << 8 | buf[1]);
    raw->AccY  = (int16_t)((uint16_t)buf[2]  << 8 | buf[3]);
    raw->AccZ  = (int16_t)((uint16_t)buf[4]  << 8 | buf[5]);
    raw->Temp  = (int16_t)((uint16_t)buf[6]  << 8 | buf[7]);
    raw->GyroX = (int16_t)((uint16_t)buf[8]  << 8 | buf[9]);
    raw->GyroY = (int16_t)((uint16_t)buf[10] << 8 | buf[11]);
    raw->GyroZ = (int16_t)((uint16_t)buf[12] << 8 | buf[13]);

    return MPU6050_OK;
}

uint8_t MPU6050_ReadData(MPU6050_Data_t *data)
{
    MPU6050_RawData_t raw;
    uint8_t err;

    if (data == 0)
    {
        return MPU6050_ERR_PARAM;
    }

    err = MPU6050_ReadRaw(&raw);
    if (err != MPU6050_OK)
    {
        return err;
    }

    data->AccX_g = (float)raw.AccX / 16384.0f;
    data->AccY_g = (float)raw.AccY / 16384.0f;
    data->AccZ_g = (float)raw.AccZ / 16384.0f;
    data->Temp_C = (float)raw.Temp / 340.0f + 36.53f;
    data->GyroX_dps = (float)raw.GyroX / 131.0f;
    data->GyroY_dps = (float)raw.GyroY / 131.0f;
    data->GyroZ_dps = (float)raw.GyroZ / 131.0f;

    return MPU6050_OK;
}
