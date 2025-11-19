#include "UART.h"

void UART1_Init(void);
void UART1_Send_Byte(char data);
void UART1_Send_Multi_Byte(char* arr, int size);

#define GPIOA_BASE_ADDRESS   0x40010800U   // GPIOA on APB2
#define USART1_BASE_ADDRESS  0x40013800U   // USART1 on APB2
#define RCC_BASE_ADDRESS     0x40021000U   // RCC

void Send_Data(void){
	
	  // Initialize UART1 (PA9 = TX, PA10 = RX)
    UART1_Init();

    char msg[] = "134\r\n";

		while (1)
    {
        UART1_Send_Multi_Byte(msg, sizeof(msg) - 1);

        for (volatile int i = 0; i < 1000000; i++);
    }
}

void UART1_Init(void) {
    /************ GPIO configuration ************/
    // Enable clock for GPIOA and USART1
    uint32_t* RCC_APB2ENR = (uint32_t*)(RCC_BASE_ADDRESS + 0x18);
    *RCC_APB2ENR |= (1 << 2);   // Bit2: GPIOAEN = 1 (enable GPIOA clock)
    *RCC_APB2ENR |= (1 << 14);  // Bit14: USART1EN = 1 (enable USART1 clock)

    uint32_t* GPIOA_CRH = (uint32_t*)(GPIOA_BASE_ADDRESS + 0x04);

    // Clear bits for PA9 (TX) and PA10 (RX)
    *GPIOA_CRH &= ~((0xF << 4) | (0xF << 8));

    // PA9 = Alternate function push-pull, output 50MHz  --> 1011 (0xB)
    *GPIOA_CRH |= (0xB << 4);

    // PA10 = Input floating --> 0100 (0x4)
    *GPIOA_CRH |= (0x4 << 8);

    /************ USART configuration ************/
    uint32_t* USART1_BRR  = (uint32_t*)(USART1_BASE_ADDRESS + 0x08);
    uint32_t* USART1_CR1  = (uint32_t*)(USART1_BASE_ADDRESS + 0x0C);

    // Baudrate = 115200 @ 72MHz (PCLK2)
    // USARTDIV = 72MHz / (16 * 115200) = 39.0625
    // Mantissa = 39 (0x27), Fraction = 1 --> BRR = (39 << 4) | 1 = 0x271
    *USART1_BRR = 0x04E2;

    // Word length = 8 bits, no parity
    *USART1_CR1 &= ~((1 << 12) | (1 << 10));

    // Enable USART, transmitter, receiver
    *USART1_CR1 |= ((1 << 13) | (1 << 3) | (1 << 2));
}

/************ Send single byte ************/
void UART1_Send_Byte(char data) {
    uint32_t* USART1_SR = (uint32_t*)(USART1_BASE_ADDRESS + 0x00);
    uint32_t* USART1_DR = (uint32_t*)(USART1_BASE_ADDRESS + 0x04);

    // Wait until TXE = 1 (Transmit data register empty)
    while(((*USART1_SR) & (1 << 7)) == 0);

    // Write data to Data Register
    *USART1_DR = data;

    // Wait until TC = 1 (Transmission complete)
    while(((*USART1_SR) & (1 << 6)) == 0);
}

/************ Send multiple bytes ************/
void UART1_Send_Multi_Byte(char* arr, int size) {
    for(int i = 0; i < size; i++) {
        UART1_Send_Byte(arr[i]);
    }
}

// hoac //
/*
void UART_SendString(const char* str)
{
    while(*str) UART2_SendChar(*str++);
}
*/
