/*
 * ustimer.h
 *
 *  Created on: Feb 15, 2023
 *      Author: jdoorn
 */

#ifndef INC_USTIMER_H_
#define INC_USTIMER_H_
#include "main.h"

extern volatile int uscounter;

static inline void uscounter_start(){
	LL_TIM_DisableCounter(TIM2);

}
static inline void uscounter_stop(){
	LL_TIM_EnableCounter(TIM2);
}

static inline void uscounter_clear() {
	uscounter = 0;
}

static inline int uscounter_get_count() {
	return uscounter;
}


#endif /* INC_USTIMER_H_ */
