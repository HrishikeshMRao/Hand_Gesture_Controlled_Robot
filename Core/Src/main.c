#include "main.h"
#include "string.h"
#include "stdlib.h"
#include "stm32f103xb.h"
#include <stdio.h>

#define SystemClock 24000000
#define baudrate 115200

#define I2CFreq 100000
#define IMU_address 0b01010000
#define memAddress 0x08
#define buffer 8
#define length 6

void init_i2c2(void);

uint16_t i2c_buff[buffer];
uint32_t n=0;
char str1[256];
char str2[64];
char *data1 = "1st loop going\n";
char *data2 = "2nd loop going\n";
char *data3 = "3rd loop going\n";

void USART1_SendChar(char c) {
	// Wait until the Transmit Data Register (TDR) is empty
	while (!(USART1->SR & USART_SR_TXE));

	// Send the character
	USART1->DR = c;
}

void USART1_SendString(char *str) {
	while (*str != '\0') {
		USART1_SendChar(*str);
		str++;
	}
}
void print(uint16_t(*data1)[buffer],uint32_t data2, uint8_t len)
{
	sprintf(str1, "%d", (uint16_t)*data1[0]);
	for(int i=0;i<len-1;i++)
	{
		sprintf(str2, "%d", (uint16_t)*data1[i+1]);
		strcat(str1," , ");
		strcat(str1,str2);
	}
	sprintf(str2, "%lu", (uint32_t)data2);
	strcat(str1," , ");
	strcat(str1,str2);
	strcat(str1, "\n");
	USART1_SendString(str1);
}
void USART1_Init(void) {

	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;

	GPIOA->CRH &= ~(GPIO_CRH_MODE10 | GPIO_CRH_MODE9 | GPIO_CRH_CNF10 | GPIO_CRH_CNF9); //reset the pins first

	GPIOA->CRH |= GPIO_CRH_MODE9 | GPIO_CRH_CNF9_1;

	// Configure PA10 (RX) as floating input
	GPIOA->CRH |= GPIO_CRH_CNF10_0;

	// Configure USART1
	USART1->CR1 &= ~(USART_CR1_UE);  // Disable USART1 during configuration

	uint32_t baud = (uint32_t)(SystemClock / baudrate);//calculate the baud rate

	// Set baud rate to 115200 (assuming PCLK2 = 72MHz)
	USART1->BRR = baud;  // Integer part of the division
	USART1->CR1 |= USART_CR1_TE;  // Enable transmitter
	USART1->CR1 |= USART_CR1_UE;  // Enable USART1
}

void SystemClock_Config(void) {

	// Configure PLL
	RCC->CFGR |= RCC_CFGR_PLLSRC;  // Use HSE as PLL source

	//RCC->CR |= RCC_CFGR_PLLXTPRE;
	// Enable HSE (High-Speed External) oscillator (8MHz on Blue Pill)
	RCC->CR |= RCC_CR_HSEON;
	while (!(RCC->CR & RCC_CR_HSERDY));

	RCC->CFGR |= RCC_CFGR_PLLMULL3; // PLL multiplication factor = 9 (8MHz * 9 = 72MHz)

	// Enable PLL
	RCC->CR |= RCC_CR_PLLON;
	while (!(RCC->CR & RCC_CR_PLLRDY));

	// Set PLL as the system clock source
	RCC->CFGR |= RCC_CFGR_SW_1;
    while ((RCC->CFGR & RCC_CFGR_SWS_PLL) != RCC_CFGR_SWS_PLL);

}

void init_i2c2(void) {

	RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
	RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
	I2C2->CR2 |= 24;
	I2C2->CCR |= 120;
	I2C2->TRISE |= 25;

	I2C2->CR1 |= I2C_CR1_ACK;
	GPIOB->CRH |= GPIO_CRH_CNF10 | GPIO_CRH_MODE10 | GPIO_CRH_CNF11 | GPIO_CRH_MODE11; //Alt func output open drain
	//GPIOB->ODR |= (1<<10)|(1<<11);
	I2C2->CR1 |= I2C_CR1_PE; //enable peripheral done last
}

void i2c_write(uint8_t device_address,uint8_t memory)
{
	uint32_t k=0;
	//I2C2->CR1 |= I2C_CR1_ACK;
	I2C2->CR1 |= I2C_CR1_START;
	while (!(I2C2->SR1 & I2C_SR1_SB));

	I2C2->DR = device_address;
	while (!(I2C2->SR1 & I2C_SR1_ADDR))
	{
		/*if(I2C2->SR1 & I2C_SR1_AF)
		{
			I2C2->SR1 &= ~I2C_SR1_AF;
			I2C2->CR1 |= I2C_CR1_STOP;
			k++;
			break;
		}*/
	}
	/*if(k==0)
	{*/
		I2C2->SR2;
		I2C2->DR = memory;
		while(!(I2C2->SR1 & I2C_SR1_TXE));
		//I2C2->CR1 |= I2C_CR1_STOP;
		//n++;
	//}

}
void DMA_set(uint16_t len)
{
		RCC->AHBENR |= RCC_AHBENR_DMA1EN;
		I2C2->CR2 |= I2C_CR2_DMAEN;
		DMA1_Channel5->CMAR = (uint32_t) i2c_buff;  //0b00100000000000000000000001111000
		DMA1_Channel5->CPAR = (uint32_t) &I2C2->DR; //0b01000000000000000101100000010000
		DMA1_Channel5->CCR |= DMA_CCR_TCIE | DMA_CCR_MSIZE_1 | DMA_CCR_MINC | DMA_CCR_CIRC;
		DMA1_Channel5->CNDTR = len;
		DMA1_Channel5->CCR |= DMA_CCR_EN;

}
void i2c_read(uint8_t device_address,uint16_t len) {

	//uint32_t k=0;

	I2C2->CR1 |= I2C_CR1_START;
	while (!(I2C2->SR1 & I2C_SR1_SB));

	I2C2->DR = device_address+1;
	while (!(I2C2->SR1 & I2C_SR1_ADDR))
	{
			/*if(I2C2->SR1 & I2C_SR1_AF)
			{
					I2C2->SR1 &= ~I2C_SR1_AF;
					k++;
					break;
			}*/
	}
	/*if(k==0)
	{*/
		I2C2->SR2;

		//DMA1_Channel5->CCR |=  DMA_CCR_EN;
		for(int i=0;i<len;i++)
		{
			while (!(DMA1->ISR & DMA_ISR_TCIF5));
			DMA1->IFCR |= DMA_IFCR_CTCIF5;
			/*if(i==len-2)
			I2C2->CR1 &= ~I2C_CR1_ACK;*/
		}
	//}
	I2C2->CR1 |= I2C_CR1_STOP;
	//DMA1_Channel5->CCR &= ~DMA_CCR_EN;
	//n=0;
}

int main(void) {

	SystemClock_Config();
	USART1_Init();
	init_i2c2();
	DMA_set(length);
	for (volatile int i = 0; i < 2000000; i++); // give some delay to let the imu setup its Status registers.
	while (1) {

		//print(&i2c_buff,buffer);
		for (volatile int i = 0; i < 1000000; i++);
		//if(n==0)
		i2c_write(IMU_address,memAddress);
		//else
		i2c_read(IMU_address,length);
		print(&i2c_buff,I2C2->DR,buffer);
	    // Add a delay (you might want to use a proper delay function)
	}
}


