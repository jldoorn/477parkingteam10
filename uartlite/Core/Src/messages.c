/*
 * messages.c
 *
 *  Created on: Feb 8, 2023
 *      Author: jdoorn
 */


#include "messages.h"
#include "uartstream.h"
#include <stdio.h>

message_t master_message = {0};

/**
 * Turns a message object into a string, then prints it out to a debug uart stream
 * @param msg
 * @param usartx
 */
void debug_message(message_t * msg, char * messagebuff) {
	switch (msg->command) {
		case API_ACK:
			sprintf(messagebuff, "Got an ack from station %d\r\n", msg->module_id);
			break;
		case API_PING:
			sprintf(messagebuff, "Got a ping from station %d\r\n", msg->module_id);
			break;
		case API_CAR_DETECT:
			sprintf(messagebuff, "Detected a car from station %d direction %s\r\n",
					msg->module_id, msg->body.direction == API_DIRECTION_OUT ? "OUT" : "IN");
			break;
		case API_DISTANCE_SEND:
			sprintf(messagebuff, "Got a distance measurement from station %d: %dcm\r\n", msg->module_id, msg->body.distance);
			break;
		default:
			sprintf(messagebuff, "Got an unknown api command\r\n");
	}

	return;
}

message_t * get_message() {
	return &master_message;
}

