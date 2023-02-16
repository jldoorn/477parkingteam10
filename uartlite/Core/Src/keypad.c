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
	int count = 0;
	LL_TIM_EnableCounter(TIM6);
	spi_init_oled();
	spi_display1("Enter # of Spots");
	char key;
	for(;;) {
		key = get_keypress();
		if (key == '0')
			spi_display2("0");
		else if (key == '1')
    	   	spi_display2("1");
		else if (key == '2')
			spi_display2("2");
		else if (key == '3')
			spi_display2("3");
		else if (key == '4')
			spi_display2("4");
		else if (key == '5')
			spi_display2("5");
		else if (key == '6')
			spi_display2("6");
		else if (key == '7')
			spi_display2("7");
		else if (key == '8')
			spi_display2("8");
		else if (key == '9')
			spi_display2("9");
	}
}
