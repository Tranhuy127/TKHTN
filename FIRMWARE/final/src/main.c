#include "stm32f10x.h"
#include <stdio.h>
#include "i2c_lowlevel.h" // I2C2 for Sensor
#include "sht21.h"
#include "pwm.h"
#include "i2c1_lcd.h"     // I2C1 for LCD (Renamed header)

// ==========================================================
// SYSTEM CLOCK CONFIGURATION (72MHz)
// ==========================================================

int main(void) {
    
    Gen_PWM();

    // 1. Init I2C2 for Sensor (PB10, PB11)
    I2C2_Sensor_Init(); 
    
    // 2. Init LCD (Internally inits I2C1 on PB6, PB7)
    LCD_Init();

    volatile float g_t = 0.0f, g_h = 0.0f;
    volatile uint16_t pwm_duty = 1000;

    while (1) {
        g_t = SHT21_ReadTemperature();
        g_h = SHT21_ReadHumidity();

        LCD_PrintTempHumi(g_t, g_h);

        // Simple hysteresis logic for PWM
        if ((g_t >= 30.0f) && (g_t < 33.0f)) {
            pwm_duty = 1100; 
        } else if (g_t >= 33.0f){
            pwm_duty = 1200; 
        } else {
            pwm_duty = 1000;
        }
        
         // Safety check example
       if (g_t >= 80.0f) {
           LCD_Clear();
           LCD_Print("Error: Temp High");
        }

        TIM3->CCR1 = pwm_duty;
        
        // delay_ms(500); // Update rate
    }
}