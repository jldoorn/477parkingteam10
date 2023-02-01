/*
 * uartstream.c
 *
 *  Created on: Feb 1, 2023
 *      Author: jdoorn
 */

#include "uartstream.h"
#include <string.h>



void set_rx_dma(void * buff, int size, USART_TypeDef * usartx, DMA_Channel_TypeDef * dmax) {
	dmax->CCR &= ~DMA_CCR_EN;

	dmax->CCR = DMA_CCR_MINC;
	dmax->CMAR = buff;
	dmax->CPAR = &usartx->RDR;
	dmax->CNDTR = size;

	dmax->CCR |= DMA_CCR_EN;
}

void set_tx_dma(void * buff, int size, USART_TypeDef * usartx, DMA_Channel_TypeDef * dmax) {
	dmax->CCR &= ~DMA_CCR_EN;

	dmax->CCR = DMA_CCR_MINC;
	dmax->CMAR = buff;
	dmax->CPAR = &usartx->TDR;
	dmax->CNDTR = size;

	dmax->CCR |= DMA_CCR_EN;
}

int rx_line_blocking(USART_TypeDef * usartx, char * buff, int max_rx) {
	int count = 0;
	char c = 0;
	while (c != '\n' && count < max_rx) {
		while (!LL_USART_IsActiveFlag_RXNE(usartx));
		c = usartx->RDR;
		*buff++ = c;
		count++;
	}

	return count;
}


void putcharusart(char c, USART_TypeDef * usartx) {
	while (!LL_USART_IsActiveFlag_TXE(usartx)) {}
	usartx->TDR = c;
}

void writestring(char * str, USART_TypeDef * usartx) {
	while (*str) {
		putcharusart(*str++, usartx);
	}
}

char getcharusart(USART_TypeDef * usartx) {
	while (!LL_USART_IsActiveFlag_RXNE(usartx)){}
	return usartx->RDR;
}

