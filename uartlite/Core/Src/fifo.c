/*
 * fifo.c
 *
 *  Created on: Feb 1, 2023
 *      Author: jdoorn
 */

#include "fifo.h"


void fifo_put(fifo_t * fifo, char c) {
	if (!fifo_full(fifo)) {
		fifo->buffer[fifo->head] = c;
		fifo->head++;
		fifo->head %= FIFO_SIZE;
	}
}
int fifo_pop(fifo_t * fifo, char * pop) {
	char tmp;
	if (!fifo_empty(fifo)) {
		tmp = fifo->buffer[fifo->tail];
		fifo->tail++;
		fifo->tail %= FIFO_SIZE;
		*pop = tmp;
		return 0;
	}
	return -1;
}
int fifo_full(fifo_t * fifo) {
	return ((fifo->head + 1) % FIFO_SIZE) == fifo->tail ? 1 : 0;

}
int fifo_empty(fifo_t * fifo) {
	return fifo->head == fifo->tail ? 1 : 0;
}

char fifo_peak(fifo_t * fifo) {
	return fifo->buffer[fifo->tail];
}

int fifo_read_until(fifo_t * fifo, char * buffer, char term, int size) {
	int count = 0;
	char tmp = -1;

	while (count < size && tmp != term) {
		while (fifo_empty(fifo)) {
			asm volatile ("wfi"); // wait for an interrupt
		}
		if (fifo_pop(fifo, &tmp)) {
			return -1;
		}
		buffer[count] = tmp;
		count++;
	}

	return count;
}

int fifo_read_n(fifo_t * fifo, char * buffer, int size) {
	int count = 0;
	for (count = 0; count < size; count ++ ) {
			while (fifo_empty(fifo)){
				asm volatile ("wfi"); // wait for an interrupt
			}
			if (fifo_pop(fifo, buffer++)) {
				return -1;
			}
		}
	return count;

}

void fifo_block_empty(fifo_t * fifo) {
	while (fifo_empty(fifo)) {
				asm volatile ("wfi"); // wait for an interrupt
			}
}
