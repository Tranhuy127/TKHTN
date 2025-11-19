#ifndef I2C_LOWLEVEL_H
#define I2C_LOWLEVEL_H

#include <stdint.h>

void I2C1_Init(void);
void I2C1_WriteReg(uint8_t dev7bit, uint8_t reg);
void I2C1_ReadBytes(uint8_t dev7bit, uint8_t *buf, uint16_t len);

#endif
