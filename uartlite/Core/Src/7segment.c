/*
 * 7segment.c
 *
 *  Created on: Feb 21, 2023
 *      Author: fkepl
 */

#include "7segment.h"

#define SEVSEG_NEN_PIN 11

uint16_t msg[8] = { 0x0000,0x0100,0x0200,0x0300,0x0400,0x0500,0x0600,0x0700 };

/*
 * The 7 segment interface for the two 7 segment digits will work as follows:
 * 		There is a 16 bit shift register where the most significant 8 bits are
 * 		the tens place, and the least significant 8 bits are the ones place
 *
 * 		The SPI2 interface will write 16 bits to the shift register, MSB first.
 * 		SSE is active low.
 */

void print_7segment_number(int count) {
	uint16_t toWrite = 0;
	GPIOB->BRR = 1 << SEVSEG_NEN_PIN;
	GPIOB->BSRR = 1 << 12 ; // nss high
	if (count > 100) {
		count = 99;
	}

	toWrite |= font[(count / 10) + '0'] << 8;
	toWrite |= font[(count % 10 ) + '0'];
//	while ( ~ (SPI2->SR & SPI_SR_BSY));
	SPI2->DR = ~toWrite;
	while (SPI2->SR & SPI_SR_BSY);
	nano_wait(15000);
	GPIOB->BRR = 1 << 12; // nss low
	GPIOB->BSRR = 1 << SEVSEG_NEN_PIN;

}

void init_7segment(void){
    setup_7segmentDMA();
    enable_7segmentDMA();
    init_7segmentSPI2();
}

void setup_7segmentDMA(void) {
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;

    DMA1_Channel5->CCR &= ~DMA_CCR_EN;          // make sure it is off

    DMA1_Channel5->CPAR = (uint32_t) &(SPI2->DR);

    DMA1_Channel5->CMAR = (uint32_t) msg;

    DMA1_Channel5->CNDTR = 8;
    //DMA1_Channel5->CNDTR = 2;

    DMA1_Channel5->CCR |= DMA_CCR_DIR;

    DMA1_Channel5->CCR |= DMA_CCR_MINC;

    DMA1_Channel5->CCR &= ~DMA_CCR_MSIZE;
    DMA1_Channel5->CCR |= DMA_CCR_MSIZE_0;

    DMA1_Channel5->CCR &= ~DMA_CCR_PSIZE;
    DMA1_Channel5->CCR |= DMA_CCR_PSIZE_0;

    DMA1_Channel5->CCR |= DMA_CCR_CIRC;
}

void enable_7segmentDMA(void) {
    DMA1_Channel5->CCR |= DMA_CCR_EN;
}

void init_7segmentSPI2(void) {
    SPI2->CR1 &= ~SPI_CR1_SPE;

    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;

    GPIOB->MODER &= ~0xc3f00000;
    GPIOB->MODER |= 0x82600000;

    GPIOB->ODR &= ~0x00000800;

    GPIOB->AFR[1] &= ~0xf00f0f00;
    GPIOB->AFR[1] |= 0x00000500;

    SPI2->CR1 |= SPI_CR1_BR;

    SPI2->CR1 |= SPI_CR1_MSTR;

    SPI2->CR2 = SPI_CR2_SSOE | SPI_CR2_NSSP | SPI_CR2_DS;

    SPI2->CR2 |= SPI_CR2_TXDMAEN;

    SPI2->CR1 |= SPI_CR1_SPE;
}

void init_7segmentSPI2_shift(void) {
	SPI2->CR1 &= ~SPI_CR1_SPE;
	RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;

	GPIOB->MODER &= ~(GPIO_MODER_MODER10 | GPIO_MODER_MODER11 | GPIO_MODER_MODER12 | GPIO_MODER_MODER15);
	GPIOB->MODER |= GPIO_MODER_MODER10_1 | GPIO_MODER_MODER11_0 | GPIO_MODER_MODER12_0 | GPIO_MODER_MODER15_1;

	GPIOB->BRR |= 1 << 11; // enable the drivers (active low)

	GPIOB->AFR[1] &= ~(GPIO_AFRH_AFSEL10  | GPIO_AFRH_AFSEL15); // | GPIO_AFRH_AFSEL12 // slave select
	GPIOB->AFR[1] |= 5 << GPIO_AFRH_AFSEL10_Pos; // alternate function for SPI2 SCK

	SPI2->CR1 |= SPI_CR1_BR | SPI_CR1_MSTR;
	SPI2->CR2 |=  SPI_CR2_DS | SPI_CR2_NSSP | SPI_CR2_SSOE; /*  */ // 16 bit datasize
	SPI2->CR1 |= SPI_CR1_CPOL ; // | SPI_CR1_SSM; // invert clock polarity and set slave select to be software based
	SPI2->CR1 |= SPI_CR1_SPE;

}

const char font[] = {
    0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
    0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
    0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
    0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
    0x00, // 32: space
    0x86, // 33: exclamation
    0x22, // 34: double quote
    0x76, // 35: octothorpe
    0x00, // dollar
    0x00, // percent
    0x00, // ampersand
    0x20, // 39: single quote
    0x39, // 40: open paren
    0x0f, // 41: close paren
    0x49, // 42: asterisk
    0x00, // plus
    0x10, // 44: comma
    0x40, // 45: minus
    0x80, // 46: period
    0x00, // slash
    // digits
    0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07, 0x7f, 0x67,
    // seven unknown
    0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
    // Uppercase
    0x77, 0x7c, 0x39, 0x5e, 0x79, 0x71, 0x6f, 0x76, 0x30, 0x1e, 0x00, 0x38, 0x00,
    0x37, 0x3f, 0x73, 0x7b, 0x31, 0x6d, 0x78, 0x3e, 0x00, 0x00, 0x00, 0x6e, 0x00,
    0x39, // 91: open square bracket
    0x00, // backslash
    0x0f, // 93: close square bracket
    0x00, // circumflex
    0x08, // 95: underscore
    0x20, // 96: backquote
    // Lowercase
    0x5f, 0x7c, 0x58, 0x5e, 0x79, 0x71, 0x6f, 0x74, 0x10, 0x0e, 0x00, 0x30, 0x00,
    0x54, 0x5c, 0x73, 0x7b, 0x50, 0x6d, 0x78, 0x1c, 0x00, 0x00, 0x00, 0x6e, 0x00
};
