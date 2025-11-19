#include "i2c.h"

/*
1. If only 1 BYTE needs to be Read
	a) Write the slave Address, and wait for the ADDR bit (bit 1 in SR1) to be set
	b) the Acknowledge disable is made during EV6 (before ADDR flag is cleared) and the STOP condition generation is made after EV6
	c) Wait for the RXNE (Receive Buffer not Empty) bit to set
	d) Read the data from the DR

2. If Multiple BYTES needs to be read
  a) Write the slave Address, and wait for the ADDR bit (bit 1 in SR1) to be set
	b) Clear the ADDR bit by reading the SR1 and SR2 Registers
	c) Wait for the RXNE (Receive buffer not empty) bit to set
	d) Read the data from the DR 
	e) Generate the Acknowlegment by settint the ACK (bit 10 in SR1)
	f) To generate the nonacknowledge pulse after the last received data byte, the ACK bit must be cleared just after reading the 
		 second last data byte (after second last RxNE event)
	g) In order to generate the Stop/Restart condition, software must set the STOP/START bit 
	   after reading the second last data byte (after the second last RxNE event)
*/

#define I2C1_BASEADRESS   0x40005400UL
#define I2C1_CR1   (*(volatile uint16_t *)(I2C1_BASEADRESS + 0x00))
#define I2C1_CR2   (*(volatile uint16_t *)(I2C1_BASEADRESS + 0x04))
#define I2C1_OAR1  (*(volatile uint16_t *)(I2C1_BASEADRESS + 0x08))
#define I2C1_OAR2  (*(volatile uint16_t *)(I2C1_BASEADRESS + 0x0C))
#define I2C1_DR    (*(volatile uint16_t *)(I2C1_BASEADRESS + 0x10))
#define I2C1_SR1   (*(volatile uint16_t *)(I2C1_BASEADRESS + 0x14))
#define I2C1_SR2   (*(volatile uint16_t *)(I2C1_BASEADRESS + 0x18))
#define I2C1_CCR   (*(volatile uint16_t *)(I2C1_BASEADRESS + 0x1C))
#define I2C1_TRISE (*(volatile uint16_t *)(I2C1_BASEADRESS + 0x20))


I2C_InitTypeDef I2C_InitStructure;

void gpio_config(void);
void rcc_config(void);
void I2C_Setup(void);
int I2C_Read(uint8_t dev7bit, uint8_t reg);

int I2C_Read(uint8_t dev7bit, uint8_t reg){
	uint8_t val;
	
	while (I2C1_SR2 & (1<<1));  // wait BUSY in SR2 = 1
	
	/* START */
	I2C1_CR1 |= (0b1 << 10);
	I2C1_CR1 |= (0b1 << 8);
	while (!(I2C1_SR1 & 1)); // wait SB in SR1 = 1 <=> start bit generated
	
	/**** STEP 1-a ****/	
	/* Gui dia chi + W */
	I2C1_DR = (uint16_t)(dev7bit << 1);
	while (!(I2C1->SR1 & (1<<1))); // wait for ADDR bit to set
  
	/**** STEP 1-b ****/	
	I2C1_CR1 &= ~(1<<10); // clear the ACK bit
	uint8_t temp = I2C1->SR1 | I2C1->SR2;  // read SR1 and SR2 to clear the ADDR bit.... EV6 condition
	
	/* Gui dia chi thanh ghi */
	while (!(I2C1_SR1 & (0b1 << 7))) { /* wait I2C_SR1_TXE = 1, data reg empty */ }
  I2C1_DR = reg; // dia chi master muon doc
  while (!(I2C1_SR1 & (0b1 << 7))) { /* wait data sent <=> data reg empty */ }
	while (I2C1_SR1 & (0b1 << 10)) { /* wait AF = 0 <=> no ack failure */ }
	
	/* Re-START */
  I2C1_CR1 |= (0b1 << 8);
  while (!(I2C1_SR1 & 1)); // wait SB in SR1 = 1 <=> start bit generated
	 
	/* Gui dia chi + R */
  I2C1_DR = (uint16_t)((dev7bit << 1) | 1);
	while (!(I2C1_SR1 & (1 << 1)));  // wait ADDR = 1
	temp = I2C1->SR1 | I2C1->SR2;  // read SR1 and SR2 to clear the ADDR bit.... EV6 condition

  /* Doc 1 byte: tat ACK, set STOP, roi doc DR khi RXNE=1 */
  I2C1_CR1 &= ~(0b1 << 10);        /* NACK cho byte cuoi */
  temp = I2C1->SR1 | I2C1->SR2;  // read SR1 and SR2 to clear the ADDR bit.... EV6 condition

  I2C1_CR1 |= (0b1 << 9);        /* STOP ngay cho truong hop 1 byte */

  while (!(I2C1_SR1 & (0b1 << 6))) { /* wait I2C_SR1_RXNE = 1 <=> data not empty, have to read */ }
  val = (uint8_t)I2C1_DR;

  /* Bat lai ACK neu sau do doc nhieu byte khac */
  I2C1_CR1 |= (0b1 << 10);
  return val;
}

void I2C_Setup(void){
	rcc_config();
	gpio_config();
	
	I2C1_CR1 &= ~(1); // PE = 0 disable i2c before setup i2c
	
	I2C1_CR1 |= (0b1 << 15); // reset i2c
	I2C1_CR1 &= ~(0b1 << 15); // set normal i2c operate after reset
	
	I2C1_CR1 &= ~(0b1 << 1); // SMBUS = 0 mode I2C. SMBUS = 1 mode SMbus
	
	I2C1_CR2 &= ~0x3F; // FREQ[5:0] = 100100 <=> 36Mhz
	I2C1_CR2 |= 36; // FREQ = 36MHz, 36d = 100100b
	
	I2C1_CCR |= (0b1 << 15); // FAST MODE, DUTY CYCLE bit 14 = 0 (default) => t_low/t_high = 2
	
	
	I2C1_CCR |= 30;
	
	I2C1_TRISE = 12; // 36 + 1, TRISE = PCLK1(MHz) * 300 ns + 1, TRISE = PCLK1(MHz) * 1000 ns + 1, remind in UM10204 - NXP original reference
	
	I2C1_CR1 |= (1);// PE = 1 enable i2c i2c
}

void rcc_config(void){
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE); // APB1 max 36Mhz
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE); // APB1 max 72Mhz	
}

void gpio_config(void){
	
	uint32_t* BASEADRESS_GPIOB = (uint32_t*) 0x40010C00U;
	uint32_t* GPIOB_CRL = (uint32_t*) (BASEADRESS_GPIOB + 0x00U);
	// PB6,7 = AF mode OpenDrain ; speed 50MHz
	*GPIOB_CRL &=  ~((0b1111U << (6*4)) | (0b1111U << (7*4))); 
	*GPIOB_CRL |=  ((0b1111U << (6*4)) | (0b1111U << (7*4))); 
	uint32_t* BASEADRESS_AFIO = (uint32_t*) 0x40010000U;
	uint32_t* AFIO_MAPR = (uint32_t*)((uintptr_t)BASEADRESS_AFIO + 0x04U);
	*AFIO_MAPR &= ~(0b1 << 1); //no remap SCL=PB6. SDA=PB7
}
