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
#include "7segment.h"


void keypad(){

	// Declare/Initialize Variables
	int count = 0;								// Variable to keep the input to 2 digits
	int sti = 0;								// Variable that holds the current number of spots
	int ips = 0;								// Variable that holds the initial number of spots
	char key;									// Variable where key presses are stored initially
	char* temp;									// Variable where the key being pressed is temporarily stored
	char disp[3] = "";							// Variable to hold the initial parking spot count before converted to integer

	// Set Up Environment
	LL_TIM_EnableCounter(TIM6);					// Initialize timer 6 counter
	spi_init_oled();							// Initialize OLED
	spi_display1("Enter # of Spots");			// Prompt User Input

	// Gather Total # of Parking Spaces Before Anyone is Parked (going to rewrite as switch statement if I have time)
	while(count < 2) {							// Ensure only 2 digits are entered
		key = get_keypress();					// Get Key Press
		if (key == '0'){
			temp = "0";
			strncat(disp, temp, 1);				// Concatenate temp to array of chars
			count++;
		}
		else if (key == '1'){
			temp = "1";
			strncat(disp, temp, 1);
			count++;
		}
		else if (key == '2'){
			temp = "2";
			strncat(disp, temp, 1);
			count++;
		}
		else if (key == '3'){
			temp = "3";
			strncat(disp, temp, 1);
			count++;
		}
		else if (key == '4'){
			temp = "4";
			strncat(disp, temp, 1);
			count++;
		}
		else if (key == '5'){
			temp = "5";
			strncat(disp, temp, 1);
			count++;
		}
		else if (key == '6'){
			temp = "6";
			strncat(disp, temp, 1);
			count++;
		}
		else if (key == '7'){
			temp = "7";
			strncat(disp, temp, 1);
			count++;
		}
		else if (key == '8'){
			temp = "8";
			strncat(disp, temp, 1);
			count++;
		}
		else if (key == '9'){
			temp = "9";
			strncat(disp, temp, 1);
			count++;
		}
		else if (key == '*'){
			count++;
		}

		//displayCount_oled(disp);				// Uncomment to use oled
		displayCount_7segment(disp);			// Comment to use oled
	}

	// Process User Input
	sscanf(disp, "%d", &ips);					// Convert Array of chars to an int for the initial count of spots
	sscanf(disp, "%d", &sti);					// Convert Array of chars to an int for the running count of spots

	spi_display1("                    ");		// Clear top line of OLED
	spi_display2("                    ");		// Clear top line of OLED -> Comment to use oled
	//spi_display1("# of Open Spots");			// Display current count of spots -> Uncomment to use oled

	// Add functionality to add and decrement count number for testing purposes
	for(;;){
		key = get_keypress();
		if (key == 'A'){
			sti++;
			sprintf(disp, "%d", sti);			//Convert back to string

			//displayCount_oled(disp);			// Uncomment to use oled
			displayCount_7segment(disp);		// Comment to use oled
		}
		else if(key == 'B'){
			if(sti > 0){
				sti--;
				sprintf(disp, "%d", sti);

				//displayCount_oled(disp);		// Uncomment to use oled
				displayCount_7segment(disp);	// Comment to use oled
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
void displayCount_oled(char disp[3]){
	spi_display2(disp);
}

void displayCount_7segment(char disp[3]){
	// Display # of Parking Spaces on 7 segment
	// *Not tested* builds but gives a warning that might need debugging
	msg[0] |= font[disp[0]];
	msg[1] |= font[disp[1]];
}

int get_num_spots() {
	// Declare/Initialize Variables
		int count = 0;								// Variable to keep the input to 2 digits
		int ips = 0;								// Variable that holds the initial number of spots
		char key;									// Variable where key presses are stored initially
		char* temp;									// Variable where the key being pressed is temporarily stored
		char disp[3] = "";							// Variable to hold the initial parking spot count before converted to integer

		// Set Up Environment
		LL_TIM_EnableCounter(TIM6);					// Initialize timer 6 counter
		spi_init_oled();							// Initialize OLED
		spi_display1("Enter # of Spots");			// Prompt User Input

		// Gather Total # of Parking Spaces Before Anyone is Parked (going to rewrite as switch statement if I have time)
		while(count < 2) {							// Ensure only 2 digits are entered
			key = get_keypress();					// Get Key Press
			if (key == '0'){
				temp = "0";
				strncat(disp, temp, 1);				// Concatenate temp to array of chars
				count++;
			}
			else if (key == '1'){
				temp = "1";
				strncat(disp, temp, 1);
				count++;
			}
			else if (key == '2'){
				temp = "2";
				strncat(disp, temp, 1);
				count++;
			}
			else if (key == '3'){
				temp = "3";
				strncat(disp, temp, 1);
				count++;
			}
			else if (key == '4'){
				temp = "4";
				strncat(disp, temp, 1);
				count++;
			}
			else if (key == '5'){
				temp = "5";
				strncat(disp, temp, 1);
				count++;
			}
			else if (key == '6'){
				temp = "6";
				strncat(disp, temp, 1);
				count++;
			}
			else if (key == '7'){
				temp = "7";
				strncat(disp, temp, 1);
				count++;
			}
			else if (key == '8'){
				temp = "8";
				strncat(disp, temp, 1);
				count++;
			}
			else if (key == '9'){
				temp = "9";
				strncat(disp, temp, 1);
				count++;
			}
			else if (key == '*'){
				count++;
			}

			displayCount_oled(disp);				// Uncomment to use oled
//			displayCount_7segment(disp);			// Comment to use oled
		}

		sscanf(disp, "%d", &ips);					// Convert Array of chars to an int for the initial count of spots

		return ips;

}
