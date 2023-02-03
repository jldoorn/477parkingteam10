/*
 * esp.c
 *
 *  Created on: Feb 1, 2023
 *      Author: jdoorn
 *      https://espressif-docs.readthedocs-hosted.com/projects/esp-at/en/release-v2.2.0.0_esp32/AT_Command_Set/Basic_AT_Commands.html
 */

#include "esp.h"
#include "fifo.h"
#include "uartstream.h"
#include <string.h>

esp_response_t resp;
char esp_buffer[1024] = { 0 };

//void putcharusart(char c, USART_TypeDef * usartx) {
//	while (!LL_USART_IsActiveFlag_TXE(usartx)) {}
//	usartx->TDR = c;
//}

void esp_disable_echo(USART_TypeDef *usartx) {
	char *cur = esp_buffer;
	sprintf(esp_buffer, "ATE0\r\n");
	while (*cur) {
		putcharusart(*cur++, usartx);
	}
}

int esp_check_status(fifo_t *fifo) {
	do {
		fifo_read_until(fifo, esp_buffer, '\n', 1024);
	} while (strnstr(esp_buffer, "ATE0", 1024)
			|| (strnstr(esp_buffer, "\r\n", 1024) == esp_buffer));

	if (strnstr(esp_buffer, "OK", 1024)) {
		return 1;
	}
	return 0;
}

void esp_set_station(USART_TypeDef *usartx) {
	sprintf(esp_buffer, "AT+CWMODE=2\r\n");
	writestring(esp_buffer, usartx);
}

void esp_broadcast_net(char *ssid, char *pass, USART_TypeDef *usartx) {
	sprintf(esp_buffer, "AT+CWSAP=\"%s\",\"%s\",5,3\r\n", ssid, pass);
	writestring(esp_buffer, usartx);
}

void esp_multiplex_en(USART_TypeDef *usartx) {
	sprintf(esp_buffer, "AT+CIPMUX=1\r\n");
	writestring(esp_buffer, usartx);
}

void esp_listen_incoming_udp(int port, USART_TypeDef *usartx) {
	sprintf(esp_buffer, "AT+CIPSTART=0,\"UDP\",\"0.0.0.0\",%d,%d,2\r\n", port,
			port);
	writestring(esp_buffer, usartx);
}

int setup_esp(fifo_t *fifo, USART_TypeDef *usartx, USART_TypeDef *debug) {
	esp_disable_echo(usartx);
	if (esp_check_status(fifo) != 1) {
		writestring("Failed echo disable\n", debug);
		return -1;
	}
	esp_set_station(usartx);
	if (esp_check_status(fifo) != 1) {
		writestring("Failed set station\n", debug);
		return -1;
	}
	esp_broadcast_net("myap", "12345678", usartx);
	if (esp_check_status(fifo) != 1) {
		writestring("Failed broadcast net\n", debug);
		return -1;
	}
	esp_multiplex_en(usartx);
	if (esp_check_status(fifo) != 1) {
		writestring("failed multiplex en\n", debug);
		return -1;
	}
	esp_listen_incoming_udp(8080, usartx);
	if (esp_check_status(fifo) != 1) {
		writestring("failed udp listen\n", debug);
		return -1;
	}
	writestring("esp setup success\n", debug);
	return 0;
}

void esp_debug_response(fifo_t *fifo, USART_TypeDef * debug) {
	int buffer_offset;
	int ipd_data_count;
	int link_id;
	fifo_block_empty(fifo);
	switch (fifo_peak(fifo)) {
		case '+': // incoming update
			buffer_offset = fifo_read_until(fifo, esp_buffer, ':', 1024);
			if (strnstr(esp_buffer, "+IPD", 1024)) {
				writestring("Incoming packet data\n>", debug);
				sscanf(esp_buffer, "+IPD:,%d,%d:", &link_id, &ipd_data_count);
				while (ipd_data_count > 0){
					fifo_read_n(fifo, esp_buffer, ipd_data_count % 1024);
					usart_write_n(esp_buffer, ipd_data_count % 1024, debug);
					ipd_data_count -= (ipd_data_count % 1024);
				}
				writestring("<End of packet data\n", debug);
			} else {
				writestring("ESP Status Update: ", debug);
				buffer_offset += fifo_read_until(fifo, esp_buffer + buffer_offset, '\n', 1024 - buffer_offset - 1);
				usart_write_n(esp_buffer, buffer_offset, debug);
			}
			break;
		case '\r':
			fifo_read_until(fifo, esp_buffer, '\n', 1024); // odd newline
			break;
		default:
			writestring("Got unexpected update: ", debug);
			buffer_offset = fifo_read_until(fifo, esp_buffer, '\n', 1024);
			usart_write_n(esp_buffer, buffer_offset, debug);
			break;
	}
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

