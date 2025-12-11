#include "i2c_lowlevel.h"
#include "stm32f10x.h"

// Define I2C2 Base Address (0x40005800 for STM32F103)
#define I2C2_BASE  0x40005800UL

// Map registers to I2C2 Base
#define I2C2_CR1   (*(volatile uint16_t *)(I2C2_BASE + 0x00))
#define I2C2_CR2   (*(volatile uint16_t *)(I2C2_BASE + 0x04))
#define I2C2_OAR1  (*(volatile uint16_t *)(I2C2_BASE + 0x08))
#define I2C2_OAR2  (*(volatile uint16_t *)(I2C2_BASE + 0x0C))
#define I2C2_DR    (*(volatile uint16_t *)(I2C2_BASE + 0x10))
#define I2C2_SR1   (*(volatile uint16_t *)(I2C2_BASE + 0x14))
#define I2C2_SR2   (*(volatile uint16_t *)(I2C2_BASE + 0x18))
#define I2C2_CCR   (*(volatile uint16_t *)(I2C2_BASE + 0x1C))
#define I2C2_TRISE (*(volatile uint16_t *)(I2C2_BASE + 0x20))

// ------------------------------------------------------
// I2C2 INIT (PB10 = SCL, PB11 = SDA)
// ------------------------------------------------------
void I2C2_Sensor_Init(void)
{
    // Enable Clock for GPIOB and I2C2
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN | RCC_APB2ENR_AFIOEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;

    // PB10=SCL, PB11=SDA, AF Open-drain 50MHz
    // PB10 and PB11 are in CRH (Control Register High)
    GPIOB->CRH &= ~( (0xF << 8) | (0xF << 12) ); // Clear bits 8-11 (PB10) and 12-15 (PB11)
    GPIOB->CRH |=  ( (0xF << 8) | (0xF << 12) ); // Set 0xF (AF Open-Drain)

    // Reset I2C2
    I2C2_CR1 |=  (1<<15); // SWRST
    I2C2_CR1 &= ~(1<<15);

    // Timing Configuration
    I2C2_CR2 = 36;  // PCLK1 = 36MHz
    I2C2_CCR = 180; // Standard mode 100kHz
    I2C2_TRISE = 37;

    I2C2_CR1 |= (1<<0); // PE=1 (Enable I2C2)
}

// ------------------------------------------------------
// Write Register (Using I2C2)
// ------------------------------------------------------
void I2C2_WriteReg(uint8_t dev7bit, uint8_t reg)
{
    while (I2C2_SR2 & (1<<1)); // Wait BUSY

    I2C2_CR1 |= (1<<8);         // START
    while (!(I2C2_SR1 & 1));    // Wait SB
    (void)I2C2_SR1; (void)I2C2_SR2;

    I2C2_DR = (dev7bit << 1) | 0; // Send Address + Write
    
    while (!(I2C2_SR1 & (1<<1))); // Wait ADDR
    (void)I2C2_SR1; (void)I2C2_SR2;

    while (!(I2C2_SR1 & (1<<7))); // Wait TXE
    I2C2_DR = reg;
    while (!(I2C2_SR1 & (1<<7))); // Wait TXE

    // Wait BTF before Stop (recommended)
    while (!(I2C2_SR1 & (1<<2))); 

    I2C2_CR1 |= (1<<9);         // STOP
}

// ------------------------------------------------------
// Read N bytes (Using I2C2)
// ------------------------------------------------------
void I2C2_ReadBytes(uint8_t dev7bit, uint8_t *buf, uint16_t len)
{
    volatile uint16_t tmp;
    uint8_t addr_read = (dev7bit<<1) | 1;

    // CASE 1: Read 1 byte
    if (len == 1)
    {
        while (I2C2_SR2 & (1<<1)); // Wait BUSY

        I2C2_CR1 &= ~(1<<10);       // ACK = 0 

        // START
        I2C2_CR1 |= (1<<8);
        while (!(I2C2_SR1 & 1));
        (void)I2C2_SR1;

        // Send Addr + Read
        I2C2_DR = addr_read;
        while (!(I2C2_SR1 & (1<<1)));
        tmp = I2C2_SR1; tmp = I2C2_SR2;   // Clear ADDR

        I2C2_CR1 |= (1<<9);               // STOP

        while (!(I2C2_SR1 & (1<<6)));     // RXNE
        buf[0] = I2C2_DR;
        return;
    }
    // CASE 2: Read 2 bytes
    else if (len == 2)
    {
        while (I2C2_SR2 & (1<<1));

        I2C2_CR1 &= ~(1<<10);     // ACK = 0
        I2C2_CR1 |=  (1<<11);     // POS = 1

        // START
        I2C2_CR1 |= (1<<8);
        while (!(I2C2_SR1 & 1));
        (void)I2C2_SR1;

        I2C2_DR = addr_read;
        while (!(I2C2_SR1 & (1<<1)));
        tmp = I2C2_SR1; tmp = I2C2_SR2;

        while (!(I2C2_SR1 & (1<<2))); // BTF = 1

        I2C2_CR1 |= (1<<9);        // STOP

        buf[0] = I2C2_DR;          // Byte 1
        buf[1] = I2C2_DR;          // Byte 2

        I2C2_CR1 &= ~(1<<11);      // POS = 0
        return;
    }
    // CASE 3: Read > 2 bytes
    else
    {
        while (I2C2_SR2 & (1<<1));

        I2C2_CR1 |= (1<<10);       // ACK = 1

        // START
        I2C2_CR1 |= (1<<8);
        while (!(I2C2_SR1 & 1));
        (void)I2C2_SR1;

        I2C2_DR = addr_read;
        while (!(I2C2_SR1 & (1<<1)));
        tmp = I2C2_SR1; tmp = I2C2_SR2;

        while (len > 3)
        {
            while (!(I2C2_SR1 & (1<<6)));   // RXNE
            *buf++ = I2C2_DR;
            len--;
        }

        while (!(I2C2_SR1 & (1<<2)));       // BTF

        I2C2_CR1 &= ~(1<<10);               // ACK = 0
        *buf++ = I2C2_DR;                   // N-2

        I2C2_CR1 |= (1<<9);                 // STOP

        while (!(I2C2_SR1 & (1<<6)));
        *buf++ = I2C2_DR;                   // N-1

        while (!(I2C2_SR1 & (1<<6)));
        *buf++ = I2C2_DR;                   // N

        return;
    }
}