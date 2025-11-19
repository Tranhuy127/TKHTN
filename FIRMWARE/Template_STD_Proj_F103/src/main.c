#include "stm32f10x.h"
#include "i2c.h"
#include "i2c_lowlevel.h"
#include "sht21.h"

void I2C_Setup(void);
void I2C_Read(uint8_t dev7bit, uint8_t reg);

int main(void){
		SystemInit(); 
    I2C1_Init();            
		volatile float g_t, g_h;
    while (1)
    {
      g_t = SHT21_ReadTemperature();
			g_h = SHT21_ReadHumidity();
        //debug viewer
    }
	/*
		I2C_Setup();
	
    uint8_t device_addr = 0b1000000;   
    uint8_t reg_addr = 0xE3;
		
	while(1){
    I2C_Read(device_addr, reg_addr);
	}
	*/
}

#if 0
#include "stm32f10x.h"
#include <stdint.h>

// ===== Pins: PB6=SCL, PB7=SDA =====
#define PB6 (6u)
#define PB7 (7u)
#define BIT(n) (1u<<(n))

void SDA_high(void){ GPIOB->BSRR = BIT(PB7); }
void SDA_low (void){ GPIOB->BRR  = BIT(PB7); }
void SCL_high(void){ GPIOB->BSRR = BIT(PB6); }
void SCL_low (void){ GPIOB->BRR  = BIT(PB6); }
int  SDA_read(void){ return (GPIOB->IDR>>PB7) & 1u; }
int  SCL_read(void){ return (GPIOB->IDR>>PB6) & 1u; }

//
void SDA_in (void)
{
	GPIOB->CRL &= ~(0xFu<<(PB7*4)); //clear CNF7, MODE7
	GPIOB->CRL |=  (0x4u<<(PB7*4)); //CNF7=01 (floating input), MODE7=00 (input mode)
}
void SDA_out(void)
{
	GPIOB->CRL &= ~(0xFu<<(PB7*4)); //clear CNF7, MODE7
	GPIOB->CRL |=  (0x5u<<(PB7*4)); //CNF7=01 (open drain) MODE7=01 (output maxsp10Mhz)
}
void SCL_od (void)
{
	GPIOB->CRL &= ~(0xFu<<(PB6*4)); //clear CNF6, MODE6
	GPIOB->CRL |=  (0x5u<<(PB6*4)); // opendrain+output maxsp10Mhz
}

void i2c_delay(void){ for(volatile int i=0;i<1500;i++){} }

// ===== I2C bit-bang =====
void I2C_GPIO_Init(void){
  RCC->APB2ENR |= 1<<3;  //I/O port B clock enable: Set and cleared by software.
  SCL_od();
  SDA_out();
  SDA_high();
  SCL_high();
  i2c_delay();
}
void I2C_start(void){
  SDA_out();
  SDA_high();
  i2c_delay();
  SCL_high();
  while(!SCL_read()){} //recheck SCL
  i2c_delay();
  SDA_low();
  i2c_delay();
  SCL_low();
  i2c_delay();
}

void I2C_stop(void){
  SDA_out();
  SDA_low();
  i2c_delay();
  SCL_high();
  while(!SCL_read()){}
  i2c_delay();
  SDA_high();
  i2c_delay();
}
int I2C_wait_ack(void){
  SDA_in();
  SCL_high();
  for(int t=0; t<5000 && !SCL_read();++t){}
  i2c_delay();
  int nack = SDA_read();
  SCL_low();
  i2c_delay();
  SDA_out();
  SDA_high();
  return nack;
}
void i2c_tx(uint8_t v){
  SDA_out();
  for(int i=0;i<8;i++)
  {
    (v&0x80)? SDA_high(): SDA_low();
    i2c_delay();
    SCL_high();
    while(!SCL_read()){}
    i2c_delay();
    SCL_low();
    i2c_delay();
    v<<=1;
  }
  SDA_high(); // release for ACK
}

uint8_t i2c_rx(int ack)
{
  uint8_t d=0;
  SDA_in();
  for(int i=0;i<8;i++)
  {
    d<<=1;
    SCL_high();
    while(!SCL_read()){}
    i2c_delay();
    if(SDA_read()) d|=1;
    SCL_low();
    i2c_delay();
  }
  SDA_out();
  ack? SDA_low(): SDA_high();
  i2c_delay();
  SCL_high();
  while(!SCL_read()){}
  i2c_delay();
  SCL_low();
  i2c_delay();
  SDA_high();
  return d;
}

// ===== BME280 (addr 0x77) =====
#define BME_WR  ((0x77u<<1) | 0u)
#define BME_RD  ((0x77u<<1) | 1u)

int bme_write8(uint8_t reg, uint8_t val) 	//CÓ TH? T?I UU TIMING
{
  I2C_start();
  i2c_tx(BME_WR);
  if(I2C_wait_ack()){ I2C_stop(); return 1; }
  i2c_tx(reg);
  if(I2C_wait_ack()){ I2C_stop(); return 2; }
  i2c_tx(val);
  if(I2C_wait_ack()){ I2C_stop(); return 3; }
  I2C_stop();
  return 0;
}
int bme_read8(uint8_t reg, uint8_t *out){  //CÓ TH? T?I UU TIMING
  if(!out) return -1;
  I2C_start();
  i2c_tx(BME_WR);
  if(I2C_wait_ack()){ I2C_stop(); return 1; }
  i2c_tx(reg);
  if(I2C_wait_ack()){ I2C_stop(); return 2; }
  I2C_start();
  i2c_tx(BME_RD);
  if(I2C_wait_ack()){ I2C_stop(); return 3; }
  *out = i2c_rx(0);  //out <- read ; NACK
  I2C_stop();
  return 0;
}

// helpers
void delay_ms_soft(unsigned ms){
  for(unsigned i=0;i<ms;i++)
  { for(unsigned k=0;k<8000;k++) __NOP(); }
}
uint16_t read_u16_le(uint8_t addr) //có th? t?i UU! -> có th? viét l?i d?c 2 byte luôn :v
{
  uint8_t lo=0,hi=0;
  bme_read8(addr,&lo);
  bme_read8(addr+1,&hi);
  return (uint16_t)(((uint16_t)hi<<8)|lo);
}
int16_t read_s16_le(uint8_t addr){
  return (int16_t)read_u16_le(addr);
}

// config: osrs_T/P=x1, humidity off, normal mode
int bme_config(void){
  if (bme_write8(0xE0, 0xB6)) return -1; delay_ms_soft(3); // reset
  if (bme_write8(0xF2, 0x00)) return -2; // osrs_H=0 (off)
  if (bme_write8(0xF4, 0x27)) return -3; // osrs_T=x1, osrs_P=x1, mode=normal
  if (bme_write8(0xF5, 0x00)) return -4; // filter off, tstandby 0.5ms
  return 0;
}
uint32_t bme_read_raw20(uint8_t base) //cái nì cung z, ch?c vi?t thêm ch? nh?n 3 byte :((
{
  uint8_t msb=0,lsb=0,xlsb=0;
  bme_read8(base+0,&msb);
  bme_read8(base+1,&lsb);
  bme_read8(base+2,&xlsb);
  return ((uint32_t)msb<<12) | ((uint32_t)lsb<<4) | ((uint32_t)xlsb>>4);
}
//d?c b?ng thông s? bù
void bme_read_calib_T(uint16_t* T1,int16_t* T2,int16_t* T3)
{
  *T1 = read_u16_le(0x88);
  *T2 = read_s16_le(0x8A);
  *T3 = read_s16_le(0x8C);
}
void bme_read_calib_P(uint16_t* P1,int16_t* P2,int16_t* P3,int16_t* P4,int16_t* P5,int16_t* P6,int16_t* P7,int16_t* P8,int16_t* P9){
  *P1=read_u16_le(0x8E); *P2=read_s16_le(0x90); *P3=read_s16_le(0x92);
  *P4=read_s16_le(0x94); *P5=read_s16_le(0x96); *P6=read_s16_le(0x98);
  *P7=read_s16_le(0x9A); *P8=read_s16_le(0x9C); *P9=read_s16_le(0x9E);
}

// convert : datasheet
int32_t bme_temp_centi_from_raw(uint32_t adc_T, uint16_t T1, int16_t T2, int16_t T3, int32_t* t_fine_out){
  int32_t var1 = ((((int32_t)adc_T>>3) - ((int32_t)T1<<1)) * (int32_t)T2) >> 11;
  int32_t var2 = (((((int32_t)adc_T>>4) - (int32_t)T1) * (((int32_t)adc_T>>4) - (int32_t)T1)) >> 12) * (int32_t)T3 >> 14;
  int32_t t_fine = var1 + var2;
  if(t_fine_out) *t_fine_out = t_fine;
  return (t_fine * 5 + 128) >> 8; //oC
}

uint32_t bme_press_pa_from_raw(uint32_t adc_P, int32_t t_fine,
  uint16_t P1,int16_t P2,int16_t P3,int16_t P4,int16_t P5,int16_t P6,int16_t P7,int16_t P8,int16_t P9)
{
  int32_t var1 = (t_fine>>1) - 64000;
  int32_t var2 = (((var1>>2)*(var1>>2))>>11)*P6;
  var2 += ((var1*P5)<<1);
  var2 = (var2>>2) + ((int32_t)P4<<16);
  var1 = (((int32_t)P3 * (((var1>>2)*(var1>>2))>>13))>>3) + (((int32_t)P2*var1)>>1);
  var1 = ((32768 + var1) * (int32_t)P1) >> 15;
  if(var1==0) return 0;
  uint32_t p = (((uint32_t)((int32_t)1048576 - (int32_t)adc_P) - (var2>>12)) * 3125u);
  if(p < 0x80000000u) p = (p << 1) / (uint32_t)var1; else p = (p / (uint32_t)var1) << 1;
  var1 = ((int32_t)P9 * (int32_t)(((p>>3)*(p>>3))>>13)) >> 12;
  var2 = ((int32_t)(p>>2) * (int32_t)P8) >> 13;
  p = (uint32_t)((int32_t)p + ((var1 + var2 + P7) >> 4));
  return p; // Pascal
}
//main
int main(void){
  I2C_GPIO_Init();
  delay_ms_soft(5);
  if(bme_config()!=0) for(;;){ __NOP(); } //ch? config xong
//d?c thông s? bù
  uint16_t T1; int16_t T2,T3;
  bme_read_calib_T(&T1,&T2,&T3);
  uint16_t P1; int16_t P2,P3,P4,P5,P6,P7,P8,P9;
  bme_read_calib_P(&P1,&P2,&P3,&P4,&P5,&P6,&P7,&P8,&P9);
//d?c raw -> convert (oC and Pascal)
  uint32_t t_raw = bme_read_raw20(0xFA);
  int32_t t_fine=0, T_centi = bme_temp_centi_from_raw(t_raw, T1,T2,T3, &t_fine);

  uint32_t p_raw = bme_read_raw20(0xF7);
  uint32_t P_pa  = bme_press_pa_from_raw(p_raw, t_fine, P1,P2,P3,P4,P5,P6,P7,P8,P9);


  while(1){};
}
#endif
