/*
 * keypad.h
 *
 *  Created on: Feb 16, 2023
 *      Author: fkepl
 */

#ifndef INC_KEYPAD_H_
#define INC_KEYPAD_H_
#include "main.h"

void keypad(void);
void displayCount_oled(char disp[3]);
void displayCount_7segment(char disp[3]);

int get_num_spots();

#endif /* INC_KEYPAD_H_ */
