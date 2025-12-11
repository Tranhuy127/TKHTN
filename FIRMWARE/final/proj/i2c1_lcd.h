#ifndef __I2C1_LCD_H
#define __I2C1_LCD_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f10x.h"

// ==========================================================
// CONFIGURATION
// ==========================================================
#define LCD_ADDR        0x27

// ==========================================================
// FUNCTION PROTOTYPES
// ==========================================================

void delay_ms(uint32_t ms);
void delay_us(uint32_t us);

/* I2C1 Functions for LCD */
void I2C1_Init(void);
void I2C1_WriteByte(uint8_t dev_addr, uint8_t data);

/* LCD High Level Functions */
void LCD_Init(void);
void LCD_Clear(void);
void LCD_SetCursor(uint8_t row, uint8_t col);
void LCD_SendCmd(uint8_t cmd);
void LCD_SendData(uint8_t data);
void LCD_Print(const char *str);
void LCD_PrintTempHumi(float t, float h);
void SystemClock_Config_72MHz(void);
#ifdef __cplusplus
}
#endif

#endif /* __I2C1_LCD_H */