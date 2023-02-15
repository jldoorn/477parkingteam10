/*
 * sonar.c
 *
 *  Created on: Feb 15, 2023
 *      Author: jdoorn
 */
#include "sonar.h"
#include "asmutils.h"
#include "ustimer.h"

void us_enable(void) {
	LL_GPIO_SetOutputPin( PROX_TRIG_GPIO_Port, PROX_TRIG_Pin);
	return;
}

void us_disable(void) {
	LL_GPIO_ResetOutputPin( PROX_TRIG_GPIO_Port, PROX_TRIG_Pin);
	return;
}

int sonar(void) {

	us_enable();
	nano_wait(10000);
	us_disable();

	nano_wait(10000);
	us_enable();


	while( !(LL_GPIO_IsInputPinSet(PROX_MEAS_GPIO_Port, PROX_MEAS_Pin)));

	uscounter_clear();
	uscounter_start();
	while( (LL_GPIO_IsInputPinSet(PROX_MEAS_GPIO_Port, PROX_MEAS_Pin)));
	uscounter_stop();
	return uscounter_get_count();
}
