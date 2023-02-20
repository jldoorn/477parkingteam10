/*
 * keypad.c
 *
 *  Created on: Feb 16, 2023
 *      Author: fkepl
 */

#include "keypad.h"
#include "keypadtimer.h"
#include "spidisp.h"
#include "string.h"
#include "stdio.h"


void keypad(){
	int count = 0;
	int sti = 0;
	int ips = 0;
	LL_TIM_EnableCounter(TIM6);
	spi_init_oled();
	spi_display1("Enter # of Spots");
	char key;
	char* temp;
	char disp[3] = "";
	while(count < 2) {
		key = get_keypress();
		if(count < 2){
			if (key == '0'){
				//spi_display2("0");
				temp = "0";
				strncat(disp, temp, 1);
				count++;
			}
			else if (key == '1'){
				//spi_display2("1");
				temp = "1";
				strncat(disp, temp, 1);
				count++;
			}
			else if (key == '2'){
				//spi_display2("2");
				temp = "2";
				strncat(disp, temp, 1);
				count++;
			}
			else if (key == '3'){
				//spi_display2("3");
				temp = "3";
				strncat(disp, temp, 1);
				count++;
			}
			else if (key == '4'){
				//spi_display2("4");
				temp = "4";
				strncat(disp, temp, 1);
				count++;
			}
			else if (key == '5'){
				//spi_display2("5");
				temp = "5";
				strncat(disp, temp, 1);
				count++;
			}
			else if (key == '6'){
				//spi_display2("6");
				temp = "6";
				strncat(disp, temp, 1);
				count++;
			}
			else if (key == '7'){
				//spi_display2("7");
				temp = "7";
				strncat(disp, temp, 1);
				count++;
			}
			else if (key == '8'){
				//spi_display2("8");
				temp = "8";
				strncat(disp, temp, 1);
				count++;
			}
			else if (key == '9'){
				//spi_display2("9");
				temp = "9";
				strncat(disp, temp, 1);
				count++;
			}
			else if (key == '*'){
				count++;
			}
			spi_display2(disp);
		}
	}
	sscanf(disp, "%d", &ips);
	sscanf(disp, "%d", &sti);
	spi_display1("                    ");
	spi_display1("# of Open Spots");
	for(;;){
		key = get_keypress();
		if (key == 'A'){
			if(sti < ips){
				sti++;
				sprintf(disp, "%d", sti);
				spi_display2(disp);
			}
			else{
				spi_display1("                    ");
				spi_display1("Error");
				spi_display2("                    ");
				spi_display2("Exceeded Limit");
				break;
			}
		}
		else if(key == 'B'){
			if(sti > 0){
				sti--;
				sprintf(disp, "%d", sti);
				spi_display2(disp);
			}
			else{
				spi_display1("                    ");
				spi_display1("Error");
				spi_display2("                    ");
				spi_display2("Exceeded Limit");
				break;
			}
		}
	}

}
