/*
 * 7segment.h
 *
 *  Created on: Feb 21, 2023
 *      Author: fkepl
 */

#ifndef INC_7SEGMENT_H_
#define INC_7SEGMENT_H_

#include "main.h"

extern uint16_t msg[];
extern const char font[];

void init_7segment(void);
void setup_7segmentDMA(void);
void enable_7segmentDMA(void);
void init_7segmentSPI2(void);
void init_7segmentSPI2_shift(void);
void print_7segment_number(int count);
void write_7segment_bits(uint16_t val);

#endif /* INC_7SEGMENT_H_ */
