/*
 * spidisp.h
 *
 *  Created on: Feb 15, 2023
 *      Author: jdoorn
 */

#ifndef INC_SPIDISP_H_
#define INC_SPIDISP_H_

#include "main.h"

void spi_init_oled();
void spi_display1(const char *string);
void spi_display2(const char *string);

#endif /* INC_SPIDISP_H_ */
