#include "i2c_lowlevel.h"
#include "stm32f10x.h"

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
// I2C INIT
// ------------------------------------------------------
void I2C1_Init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN | RCC_APB2ENR_AFIOEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    // PB6=SCL, PB7=SDA, AF Open-drain 50MHz
    GPIOB->CRL &= ~((0xF << (6*4)) | (0xF << (7*4)));
    GPIOB->CRL |=  ((0xB << (6*4)) | (0xB << (7*4)));
    AFIO->MAPR &= ~(1<<1);

    I2C1_CR1 &= ~(1<<0); // PE=0
    I2C1_CR1 |=  (1<<15); // reset
    I2C1_CR1 &= ~(1<<15);

    I2C1_CR2 = 36;  // PCLK1 = 36MHz
    I2C1_CCR = 180; // Standard mode 100kHz
    I2C1_TRISE = 37;

    I2C1_CR1 |= (1<<0); // PE=1
}

// ------------------------------------------------------
// Ghi thanh ghi
// ------------------------------------------------------
void I2C1_WriteReg(uint8_t dev7bit, uint8_t reg)
{
    while (I2C1_SR2 & (1<<1)); // BUSY

    I2C1_CR1 |= (1<<8);         // START
    while (!(I2C1_SR1 & 1));
    (void)I2C1_SR1;

    I2C1_DR = (dev7bit<<1) | 0; // write
    while (!(I2C1_SR1 & (1<<1)));
    (void)I2C1_SR1; (void)I2C1_SR2;

    while (!(I2C1_SR1 & (1<<7)));
    I2C1_DR = reg;
    while (!(I2C1_SR1 & (1<<7)));

    I2C1_CR1 |= (1<<9);         // STOP
}

// ------------------------------------------------------
// Read N bytes (the CHU?N cho STM32F1)
// ------------------------------------------------------
void I2C1_ReadBytes(uint8_t dev7bit, uint8_t *buf, uint16_t len)
{
    volatile uint16_t tmp;
    uint8_t addr_read = (dev7bit<<1) | 1;

    // CASE 1: doc 1 byte
    if (len == 1)
    {
        // doi BUSY = 0
        while (I2C1_SR2 & (1<<1));

        I2C1_CR1 &= ~(1<<10);       // ACK = 0 truoc khi doc 1 byte

        // START
        I2C1_CR1 |= (1<<8);
        while (!(I2C1_SR1 & 1));
        (void)I2C1_SR1;

        // gui dia chi + R
        I2C1_DR = addr_read;
        while (!(I2C1_SR1 & (1<<1)));
        tmp = I2C1_SR1; tmp = I2C1_SR2;   // clear ADDR

        I2C1_CR1 |= (1<<9);               // STOP

        while (!(I2C1_SR1 & (1<<6)));     // RXNE
        buf[0] = I2C1_DR;
        return;
    }

    // CASE 2: doc 2 byte
    else if (len == 2)
    {
        while (I2C1_SR2 & (1<<1));

        I2C1_CR1 &= ~(1<<10);     // ACK = 0
        I2C1_CR1 |=  (1<<11);     // POS = 1

        // START
        I2C1_CR1 |= (1<<8);
        while (!(I2C1_SR1 & 1));
        (void)I2C1_SR1;

        // gui dia chi + R
        I2C1_DR = addr_read;
        while (!(I2C1_SR1 & (1<<1)));
        tmp = I2C1_SR1; tmp = I2C1_SR2;

        while (!(I2C1_SR1 & (1<<2))); // BTF = 1 (2 byte san)

        I2C1_CR1 |= (1<<9);        // STOP

        buf[0] = I2C1_DR;          // N-1
        buf[1] = I2C1_DR;          // N

        I2C1_CR1 &= ~(1<<11);      // POS = 0
        return;
    }

    // CASE 3: doc N > 2 byte
    else
    {
        while (I2C1_SR2 & (1<<1));

        I2C1_CR1 |= (1<<10);       // ACK = 1

        // START
        I2C1_CR1 |= (1<<8);
        while (!(I2C1_SR1 & 1));
        (void)I2C1_SR1;

        // gui dia chi + R
        I2C1_DR = addr_read;
        while (!(I2C1_SR1 & (1<<1)));
        tmp = I2C1_SR1; tmp = I2C1_SR2;

        // doc cac byte giua
        while (len > 3)
        {
            while (!(I2C1_SR1 & (1<<6)));   // RXNE
            *buf++ = I2C1_DR;
            len--;
        }

        // con 3 byte cuoi
        while (!(I2C1_SR1 & (1<<2)));       // BTF

        I2C1_CR1 &= ~(1<<10);               // ACK = 0 cho byte cuoi
        *buf++ = I2C1_DR;                   // N-2

        I2C1_CR1 |= (1<<9);                 // STOP

        while (!(I2C1_SR1 & (1<<6)));
        *buf++ = I2C1_DR;                   // N-1

        while (!(I2C1_SR1 & (1<<6)));
        *buf++ = I2C1_DR;                   // N

        return;
    }
}
