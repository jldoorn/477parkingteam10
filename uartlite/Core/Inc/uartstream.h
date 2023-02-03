/*
 * uartstream.h
 *
 *  Created on: Feb 1, 2023
 *      Author: jdoorn
 */



#ifndef INC_UARTSTREAM_H_
#define INC_UARTSTREAM_H_
#define UART_BUFFER_SIZE 1024
#include <stdio.h>
#include "main.h"
//char * wifi_uart_buffer[UART_BUFFER_SIZE];

FILE * get_uart_file(USART_TypeDef * usartx);
void set_rx_dma(void * buff, int size, USART_TypeDef * usartx, DMA_Channel_TypeDef * dmax);
void set_tx_dma(void * buff, int size, USART_TypeDef * usartx, DMA_Channel_TypeDef * dmax);
int rx_line_blocking(USART_TypeDef * usartx, char * buff, int max_rx);

void putcharusart(char c, USART_TypeDef * usartx) ;

void writestring(char * str, USART_TypeDef * usartx) ;
void usart_write_n(char * str, int n, USART_TypeDef * usartx);

char getcharusart(USART_TypeDef * usartx);

#endif /* INC_UARTSTREAM_H_ */
