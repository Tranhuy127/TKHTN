#include "stm32f10x.h"
#include "i2c1_lcd.h" // Nh? d?i tên header tuong ?ng
#include <stdio.h>

// ==========================================================
// CONFIGURATION
// ==========================================================
// Luu ý: N?u code cu ch?y 0x27 thì gi? 0x27. 
// N?u n?p vào không lên thì d?i thành 0x3F (do hình Logic Analyzer lúc nãy báo l?i NAK ? 0x27)

// Mapping chân PCF8574 (Chu?n thông d?ng)
#define LCD_RS          (1 << 0)
#define LCD_RW          (1 << 1)
#define LCD_EN          (1 << 2)
#define LCD_BL          (1 << 3)

static uint8_t lcd_backlight_val = LCD_BL;

// ==========================================================
// DELAY FUNCTIONS (Gi? nguyên t? code cu c?a b?n)
// ==========================================================
static void delay_us(uint32_t us) {
    us *= 12; 
    while (us--) __NOP();
}

static void delay_ms(uint32_t ms) {
    while (ms--) delay_us(1000);
}

// ==========================================================
// I2C1 DRIVER (PB6, PB7) -> Ðã chuy?n t? I2C2 sang
// ==========================================================
#define I2C1_BASE 0x40005400UL
#define I2C1_CR1   (*(volatile uint16_t *)(I2C1_BASE + 0x00))
#define I2C1_CR2   (*(volatile uint16_t *)(I2C1_BASE + 0x04))
#define I2C1_OAR1  (*(volatile uint16_t *)(I2C1_BASE + 0x08))
#define I2C1_OAR2  (*(volatile uint16_t *)(I2C1_BASE + 0x0C))
#define I2C1_DR    (*(volatile uint16_t *)(I2C1_BASE + 0x10))
#define I2C1_SR1   (*(volatile uint16_t *)(I2C1_BASE + 0x14))
#define I2C1_SR2   (*(volatile uint16_t *)(I2C1_BASE + 0x18))
#define I2C1_CCR   (*(volatile uint16_t *)(I2C1_BASE + 0x1C))
#define I2C1_TRISE (*(volatile uint16_t *)(I2C1_BASE + 0x20))

// ------------------------------------------------------
// I2C1 INIT
// ------------------------------------------------------
void I2C1_Init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN | RCC_APB2ENR_AFIOEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    // PB6=SCL, PB7=SDA, AF Open-drain 50MHz
    GPIOB->CRL &= ~((0xF << (6*4)) | (0xF << (7*4)));
    GPIOB->CRL |=  ((0xF << (6*4)) | (0xF << (7*4)));
    AFIO->MAPR &= ~(1<<1);

    I2C1_CR1 &= ~(1<<0); // PE=0
    I2C1_CR1 |=  (1<<15); // reset
    I2C1_CR1 &= ~(1<<15);

    I2C1_CR2 = 36;  // PCLK1 = 36MHz
    I2C1_CCR = 180; // Standard mode 100kHz
    I2C1_TRISE = 37;

    I2C1_CR1 |= (1<<0); // PE=1
}

void I2C1_WriteByte(uint8_t dev_addr, uint8_t data) {

    while (I2C1->SR2 & I2C_SR2_BUSY);
    I2C1->CR1 |= I2C_CR1_START;
    while (!(I2C1->SR1 & I2C_SR1_SB));

    I2C1->DR = (dev_addr << 1);
    while (!(I2C1->SR1 & I2C_SR1_ADDR));
    (void)I2C1->SR1; (void)I2C1->SR2;

    while (!(I2C1->SR1 & I2C_SR1_TXE));
    I2C1->DR = data;

    while (!(I2C1->SR1 & I2C_SR1_BTF));
    I2C1->CR1 |= I2C_CR1_STOP;
}

// ==========================================================
// LCD DRIVER (Ð?i hàm g?i xu?ng I2C1)
// ==========================================================
static void LCD_SendNibble(uint8_t nibble, uint8_t rs) {
    uint8_t data = 0;

    data = (nibble & 0xF0); 

    if (rs) data |= LCD_RS;
    data |= lcd_backlight_val; 

    // 1. EN = 1
    data |= LCD_EN;
    I2C1_WriteByte(LCD_ADDR, data); // G?i hàm I2C1
    delay_us(5); 

    // 2. EN = 0
    data &= ~LCD_EN;
    I2C1_WriteByte(LCD_ADDR, data); // G?i hàm I2C1
    delay_us(50); 
}

static void LCD_SendByte(uint8_t byte, uint8_t rs) {
    LCD_SendNibble(byte & 0xF0, rs);        
    LCD_SendNibble((byte << 4) & 0xF0, rs); 
}

void LCD_SendCmd(uint8_t cmd) {
    LCD_SendByte(cmd, 0);
}

void LCD_SendData(uint8_t data) {
    LCD_SendByte(data, 1);
}

void LCD_Clear(void) {
    LCD_SendCmd(0x01);
    delay_ms(2);
}

void LCD_SetCursor(uint8_t row, uint8_t col) {
    uint8_t addr = (row == 0) ? 0x00 : 0x40;
    addr += col;
    LCD_SendCmd(0x80 | addr);
}

void LCD_Init(void) {
    // Kh?i t?o I2C1 tru?c khi dùng LCD
    I2C1_Init(); 

    delay_ms(50);
    
    LCD_SendNibble(0x30, 0); delay_ms(10);
    LCD_SendNibble(0x30, 0); delay_ms(1);
    LCD_SendNibble(0x30, 0); delay_ms(1);
    
    LCD_SendNibble(0x20, 0); delay_ms(1);

    LCD_SendCmd(0x28); 
    LCD_SendCmd(0x08); 
    LCD_Clear();
    LCD_SendCmd(0x06); 
    LCD_SendCmd(0x0C); 
}

void LCD_Print(const char *s) {
    while (*s) LCD_SendData((uint8_t)(*s++));
}

void LCD_PrintTempHumi(float t, float h) {
    char line[17];
    
    LCD_SetCursor(0, 0);
    snprintf(line, sizeof(line), "T: %5.1f C     ", t);
    LCD_Print(line);

    LCD_SetCursor(1, 0);
    snprintf(line, sizeof(line), "H: %5.1f %%     ", h);
    LCD_Print(line);
}

// ==========================================================
// SYSTEM CLOCK 
// ==========================================================
void SystemClock_Config_72MHz(void) {
    RCC->CR |= RCC_CR_HSEON;
    while(!(RCC->CR & RCC_CR_HSERDY));
    
    FLASH->ACR |= FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY_2;
    
    RCC->CFGR |= RCC_CFGR_PLLSRC | RCC_CFGR_PLLMULL9 | RCC_CFGR_PPRE1_DIV2;
    
    RCC->CR |= RCC_CR_PLLON;
    while(!(RCC->CR & RCC_CR_PLLRDY));
    
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL;
    while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
}
