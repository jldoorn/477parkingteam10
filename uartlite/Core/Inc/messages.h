/*
 * messages.h
 *
 *  Created on: Feb 8, 2023
 *      Author: jdoorn
 */

#ifndef INC_MESSAGES_H_
#define INC_MESSAGES_H_
#include <stdint.h>
#include "stm32f091xc.h"

#define API_PING 			1
#define API_ACK				2
#define API_CAR_DETECT		3
#define API_DISTANCE_SEND	4

#define API_DIRECTION_IN	0
#define API_DIRECTION_OUT	1


typedef struct {
	uint8_t module_id;
	uint8_t command;
	union {
		uint8_t direction;
		uint8_t distance;
	} body;

} message_t;

void debug_message(message_t * msg, char * messagebuff);
message_t * get_message();

#endif /* INC_MESSAGES_H_ */
