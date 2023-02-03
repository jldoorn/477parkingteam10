/*
 * fifo.h
 *
 *  Created on: Feb 1, 2023
 *      Author: jdoorn
 */

#ifndef INC_FIFO_H_
#define INC_FIFO_H_
#define FIFO_SIZE 1024
//#include "main.h"
#include "stm32f0xx.h"
typedef struct fifo_t {
	char buffer[FIFO_SIZE];
	unsigned short head;
	unsigned short tail;
} fifo_t;

extern fifo_t usart7_rx_fifo;
extern fifo_t usart5_rx_fifo;

void fifo_put(fifo_t * fifo, char c);
int fifo_pop(fifo_t * fifo, char * c);
int fifo_full(fifo_t * fifo);
int fifo_empty(fifo_t * fifo);
int fifo_read_until(fifo_t * fifo, char * buffer, char term, int size);
int fifo_read_n(fifo_t * fifo, char * buffer, int size);
char fifo_peak(fifo_t * fifo);
void fifo_block_empty(fifo_t * fifo);
void flush_fifo_to_usart(fifo_t *fifo, USART_TypeDef *usart);


#endif /* INC_FIFO_H_ */
