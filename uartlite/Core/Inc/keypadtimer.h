/*
 * keypadtimer.h
 *
 *  Created on: Feb 16, 2023
 *      Author: fkepl
 */

#ifndef INC_KEYPADTIMER_H_
#define INC_KEYPADTIMER_H_
#include "main.h"

extern volatile int rows;
extern volatile uint8_t col;

int read_rows();
void update_history(int c, int rows);
void push_queue(int n);
void drive_column(int c);
uint8_t pop_queue();
char get_keypress();

#endif /* INC_KEYPADTIMER_H_ */
