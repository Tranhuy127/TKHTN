#ifndef I2C_LOWLEVEL_H
#define I2C_LOWLEVEL_H

#include <stdint.h>

// Functions now use I2C2 for Sensor
void I2C2_Sensor_Init(void);
void I2C2_WriteReg(uint8_t dev7bit, uint8_t reg);
void I2C2_ReadBytes(uint8_t dev7bit, uint8_t *buf, uint16_t len);

#endif