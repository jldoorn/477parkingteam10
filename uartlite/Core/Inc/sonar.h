/*
 * sonar.h
 *
 *  Created on: Feb 15, 2023
 *      Author: jdoorn
 */

#ifndef INC_SONAR_H_
#define INC_SONAR_H_
#include "main.h"

#define SONAR_DEBOUNCE_WIDTH 5

#define SONAR_THRESH_TRIG 10
#define SONAR_THRESH_CLEAR 50
#define SONAR_TRIGGERED 1

typedef struct {

	uint16_t buffer[SONAR_DEBOUNCE_WIDTH];
	uint8_t buffer_idx;
	uint8_t triggered;
	uint8_t trigger_read;
} sonar_debouncer_t;

int sonar(void);
uint8_t read_trigger_val();


#endif /* INC_SONAR_H_ */
