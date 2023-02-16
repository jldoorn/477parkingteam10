/*
 * keypad.c
 *
 *  Created on: Feb 16, 2023
 *      Author: fkepl
 */

#include "keypad.h"
#include "keypadtimer.h"
#include "spidisp.h"


void keypad(){
	LL_TIM_EnableCounter(TIM6);
	spi_init_oled();
	spi_display2("test");
	for(;;) {
		char key = get_keypress();
		if (key == 'A')
			spi_display2("10");
		else if (key == 'B')
    	   	spi_display2("20");
	}
}
