#include "sht21.h"
#include "i2c_lowlevel.h"

// L?nh c?a SHT21
#define CMD_TEMP_HOLD   0xE3
#define CMD_HUMI_HOLD   0xE5

//---------------------------------------------------
// Hàm ki?m CRC
//---------------------------------------------------
uint8_t SHT21_CheckCRC(uint8_t *data)
{
    uint8_t crc = 0x00;
    uint8_t poly = 0x31;

    for (int i = 0; i < 2; i++)
    {
        crc ^= data[i];
        for (int b = 0; b < 8; b++)
        {
            if (crc & 0x80) crc = (crc << 1) ^ poly;
            else            crc <<= 1;
        }
    }
    return (crc == data[2]);
}

//---------------------------------------------------
// Ð?c nhi?t d?
//---------------------------------------------------
float SHT21_ReadTemperature(void)
{
    uint8_t buf[3];
    uint16_t raw;

    I2C1_WriteReg(SHT21_ADDR, 0xF3); // TEMP NO HOLD MASTER

    for (volatile int i=0; i<800000; i++);   // delay ~80-100ms (tu? clock b?n)

    I2C1_ReadBytes(SHT21_ADDR, buf, 3);

    raw = ((uint16_t)buf[0] << 8) | buf[1];
    raw &= 0xFFFC;

    return -46.85f + 175.72f * ((float)raw / 65536.0f);
}


//---------------------------------------------------
// Ð?c d? ?m
//---------------------------------------------------
float SHT21_ReadHumidity(void)
{
    uint8_t buf[3];
    uint16_t raw;

    I2C1_WriteReg(SHT21_ADDR, 0xF5); // HUMIDITY NO HOLD MASTER

    for (volatile int i=0; i<800000; i++);   // delay measurement

    I2C1_ReadBytes(SHT21_ADDR, buf, 3);

    raw = ((uint16_t)buf[0] << 8) | buf[1];
    raw &= 0xFFFC;

    return -6.0f + 125.0f * ((float)raw / 65536.0f);
}

