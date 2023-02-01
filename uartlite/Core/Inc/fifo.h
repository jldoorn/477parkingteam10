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

#endif /* INC_FIFO_H_ */
