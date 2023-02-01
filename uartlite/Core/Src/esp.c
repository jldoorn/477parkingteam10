/*
 * esp.c
 *
 *  Created on: Feb 1, 2023
 *      Author: jdoorn
 */

#include "esp.h"
#include "fifo.h"
#include "main.h"
#include "uartstream.h"
#include <string.h>

esp_response_t resp;
char esp_buffer[1024] = {0};

//void putcharusart(char c, USART_TypeDef * usartx) {
//	while (!LL_USART_IsActiveFlag_TXE(usartx)) {}
//	usartx->TDR = c;
//}

void esp_disable_echo() {
	char * cur = esp_buffer;
	sprintf(esp_buffer, "ATE0\r\n");
	while (*cur) {
		putcharusart(*cur++, USART7);
	}
}

int esp_check_status(fifo_t * fifo) {
	char * cur;
		do {
		fifo_read_until(fifo, esp_buffer, '\n', 1024);
		} while (strnstr(esp_buffer, "ATE0", 1024) || (strnstr(esp_buffer, "\r\n", 1024) == esp_buffer));

	if (strnstr(esp_buffer, "OK", 1024)) {
		return 1;
	}
	return 0;
}

void esp_set_station() {
	sprintf(esp_buffer, "AT+CWMODE=2\r\n");
	writestring(esp_buffer, USART7);
}

void esp_broadcast_net(char * ssid, char * pass) {
	sprintf(esp_buffer, "AT+CWSAP=\"%s\",\"%s\",5,3\r\n", ssid, pass);
	writestring(esp_buffer, USART7);
}


//esp_response_t * _get_status_response(fifo_t * fifo) {
////	esp_response_enum resp_enum;
////	esp_status_enum status_enum;
////	for (;;) {
////		fifo_read_until(fifo, esp_buffer, '\n', 1024);
////		if strnstr(esp_buffer, "ready") {
////			resp_enum = ESP_
////		}
////	}
//}

//esp_response_t * get_response(fifo_t * fifo) {
//
//	while (fifo_empty(fifo));
//
//	switch (fifo_peak(fifo)) {
//		case '+':
//			return get_extended_response(fifo);
//			break;
//		default:
//			return _get_status_response(fifo);
//			break;
//	}
//
//}

