#ifndef SHT21_H
#define SHT21_H

#include <stdint.h>

#define SHT21_ADDR  0x40

float SHT21_ReadTemperature(void);
float SHT21_ReadHumidity(void);
uint8_t SHT21_CheckCRC(uint8_t *data);

#endif
