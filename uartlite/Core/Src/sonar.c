/*
 * sonar.c
 *
 *  Created on: Feb 15, 2023
 *      Author: jdoorn
 */
#include "sonar.h"
#include "asmutils.h"
#include "ustimer.h"

sonar_debouncer_t debounce_state = {0}; // need to set buffer to max val on init

void us_enable(void) {
	LL_GPIO_SetOutputPin( PROX_TRIG_GPIO_Port, PROX_TRIG_Pin);
	return;
}

void us_disable(void) {
	LL_GPIO_ResetOutputPin( PROX_TRIG_GPIO_Port, PROX_TRIG_Pin);
	return;
}

void sonar_init() {
	int i;
	for (i = 0; i < SONAR_DEBOUNCE_WIDTH; i++) {
		debounce_state.buffer[i] = UINT16_MAX;
	}
	debounce_state.buffer_idx = 0;
	debounce_state.triggered=0;
	debounce_state.trigger_read = 0;
}

uint8_t read_trigger_val() {
	if (debounce_state.triggered && !debounce_state.trigger_read) {
		debounce_state.trigger_read = SONAR_TRIGGERED;
		return SONAR_TRIGGERED;
	}
	return !SONAR_TRIGGERED;
}

void debounce() {
	int i;

	// check all under threshold
	if (debounce_state.trigger_read) {
		for (i = 0; i < SONAR_DEBOUNCE_WIDTH; i++) {
			if (debounce_state.buffer[i] >= SONAR_THRESH_CLEAR)
				continue;
			else
				break;
		}
		if (i == SONAR_DEBOUNCE_WIDTH) {
			debounce_state.triggered = 0;
			debounce_state.trigger_read = 0;
			return;
		}
	}
	for (i = 0; i < SONAR_DEBOUNCE_WIDTH; i++) {
		if (debounce_state.buffer[i] <= SONAR_THRESH_TRIG)
			continue;
		else
			break;
	}
	if (i == SONAR_DEBOUNCE_WIDTH) {
		debounce_state.triggered = 1;
		return;
	}
}

int sonar(void) {

	int us_count;

	us_enable();
	nano_wait(10000);
	us_disable();

//	nano_wait(10000);
//	us_enable();

	uscounter_clear();
	while( !(LL_GPIO_IsInputPinSet(PROX_MEAS_GPIO_Port, PROX_MEAS_Pin)));
	uscounter_start();
	while( (LL_GPIO_IsInputPinSet(PROX_MEAS_GPIO_Port, PROX_MEAS_Pin)));
	uscounter_stop();

	us_count = uscounter_get_count();
	debounce_state.buffer[debounce_state.buffer_idx++] = us_count;
	debounce_state.buffer_idx %= SONAR_DEBOUNCE_WIDTH;

	debounce();

	return us_count;
}
