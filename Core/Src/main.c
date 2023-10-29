#include "main.h"
#include "string.h"
#include "stdlib.h"
#include "stm32f103xb.h"
#include "stm32f1xx.h"
#include <stdio.h>


#define SystemClock 24000000
#define baudrate 115200

#define I2CFreq 100000
#define IMU_address 0b01010000

#define memwrite 0x3d
#define writeData 0b00001000

#define memAddress  0x28
#define buffer 6
#define length 6

int16_t LinACC_X, LinACC_Y, LinACC_Z;

int8_t i2c_buff[buffer];
uint32_t *address[buffer];

char str1[256];
char str2[64];

void init_i2c2(void);
void SystemClock_Config(void);
void motor_init(void);
void USART1_Init(void);
void init_i2c2(void);
void DMA_set(void);

//I2c event and error interrupts setup
void copy_address_set(uint32_t* address[])
{

	for(volatile int i=0;i<buffer;i++)
	address[i]= & i2c_buff[i];

}

void USART1_SendChar(char c)
{

	while (!(USART1->SR & USART_SR_TXE)); // Wait until the Transmit Data Register (TDR) is empty
	USART1->DR = c;	// Send the character

}

void USART1_SendString(char *str)
{

	while (*str != '\0')
	{
		USART1_SendChar(*str);
		str++;
	}

}

void print(uint32_t *data1[])
{

	LinACC_X = (((int16_t)*data1[1]) << 8) | ((int16_t)*data1[0]);
	LinACC_X /= 100;

	LinACC_Y = (((int16_t)*data1[3]) << 8) | ((int16_t)*data1[2]);
	LinACC_Y /= 100;

	LinACC_Z = (((int16_t)*data1[5]) << 8) | ((int16_t)*data1[4]);
	LinACC_Z /= 100;

	sprintf(str1, "%d", LinACC_X);
	strcat(str1, " , ");

	sprintf(str2, "%d", LinACC_Y);
	strcat(str1,str2);
	strcat(str1, " , ");

	sprintf(str2, "%d", LinACC_Z);
	strcat(str1,str2);
	strcat(str1, "\n");

	USART1_SendString(str1);

}

void USART1_Init(void)
{

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

void SystemClock_Config(void)
{

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

void init_i2c2(void)
{

	RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
	RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
	I2C2->CR2 |= 24;
	I2C2->CCR |= 120;
	I2C2->TRISE |= 25;

	GPIOB->CRH |= GPIO_CRH_CNF10 | GPIO_CRH_MODE10 | GPIO_CRH_CNF11 | GPIO_CRH_MODE11; //Alt func output open drain
	I2C2->CR1 |= I2C_CR1_PE; //enable peripheral done last
	I2C2->CR1 |= I2C_CR1_ACK;

}

void DMA_set(void)
{

	RCC->AHBENR |= RCC_AHBENR_DMA1EN;
	I2C2->CR2 |= I2C_CR2_DMAEN;
	DMA1_Channel5->CMAR = (uint32_t) &i2c_buff[0];  //0b00100000000000000000000001111000
	DMA1_Channel5->CPAR = (uint32_t) &I2C2->DR; //0b01000000000000000101100000010000
	DMA1_Channel5->CNDTR = length;
	DMA1_Channel5->CCR |= DMA_CCR_TCIE | DMA_CCR_PL  | DMA_CCR_CIRC | DMA_CCR_MINC;

}

void i2c_memWrite(uint8_t device_address,uint8_t memory,uint8_t input)
{

	I2C2->CR1 |= I2C_CR1_START;
	while (!(I2C2->SR1 & I2C_SR1_SB));

	I2C2->DR = device_address;

	while (!(I2C2->SR1 & I2C_SR1_ADDR));
	while(!(I2C2->SR2 & I2C_SR2_TRA));

	I2C2->DR = memory;

	while(!(I2C2->SR1 & I2C_SR1_TXE));
	I2C2->SR2;

	I2C2->DR = input;
	while(!(I2C2->SR1 & I2C_SR1_TXE));
	I2C2->SR2;

	I2C2->CR1 |= I2C_CR1_STOP;
	while(I2C2->CR1 & I2C_CR1_STOP);

}

void i2c_write(uint8_t device_address,uint8_t memory)
{

	I2C2->CR1 |= I2C_CR1_START;
	while (!(I2C2->SR1 & I2C_SR1_SB));

	I2C2->DR = device_address;

	while (!(I2C2->SR1 & I2C_SR1_ADDR));
	while(!(I2C2->SR2 & I2C_SR2_TRA));

	I2C2->DR = memory;

	while(!(I2C2->SR1 & I2C_SR1_TXE));

}

void i2c_read(uint8_t device_address)
{

	DMA1_Channel5->CCR |= DMA_CCR_EN;
	I2C2->CR2 |= I2C_CR2_LAST;

	I2C2->CR1 |= I2C_CR1_START;
	while (!(I2C2->SR1 & I2C_SR1_SB));

	I2C2->DR = device_address+1;

	while (!(I2C2->SR1 & I2C_SR1_ADDR));
	while((I2C2->SR2 & I2C_SR2_TRA));

	while(!(DMA1->ISR & DMA_ISR_TCIF5));
	DMA1_Channel5->CCR &= ~DMA_CCR_EN;
	DMA1->IFCR |= DMA_IFCR_CGIF5;

	I2C2->CR1 |= I2C_CR1_STOP;
	while(I2C2->CR1 & I2C_CR1_STOP);

	I2C2->CR2 &= ~I2C_CR2_LAST;

}

void TIM2_config (void)
{

	TIM2->PSC = 2;
	TIM2->ARR = 1000;
	TIM2->CCR2 =0;

	TIM2->EGR |= TIM_EGR_UG;
	TIM2->CR1 |= TIM_CR1_CEN;
	TIM2->CCMR1 &= ~TIM_CCMR1_OC2M;
	TIM2->CCMR1 |= TIM_CCMR1_OC2PE | TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1;
	TIM2->CCER |= TIM_CCER_CC2E;

}

void motor_init(void)
{

	  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	  RCC->APB2ENR |= RCC_APB2ENR_AFIOEN | RCC_APB2ENR_IOPAEN;

	  GPIOA->CRL |= GPIO_CRL_MODE1_1 | GPIO_CRL_MODE1_0 | GPIO_CRL_CNF1_1;
	  GPIOA->CRL &= ~GPIO_CRL_CNF1_0;

}

void motor_code(void)
{

	 TIM2->CCR2 = (LinACC_X*500/39)+500;

}

int main(void) {


	for (volatile int i = 0; i < 2000000; i++);
	copy_address_set(address);
	SystemClock_Config();
	TIM2_config();
	motor_init();
	USART1_Init();
	init_i2c2();
	DMA_set();
	i2c_memWrite(IMU_address,memwrite,writeData);


	while (1) {

		i2c_write(IMU_address,memAddress);
		i2c_read(IMU_address);
		print(address);
		motor_code();
		for (volatile int i = 0; i < 100000; i++); // Add a delay (you might want to use a proper delay function)
	}
}
