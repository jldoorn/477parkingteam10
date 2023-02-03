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

//esp_response_t resp;
char esp_buffer[1024] = { 0 };
esp_data_incoming_t esp_incoming = {0};

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


// Gets modem to point of
void esp_send_init(int count, USART_TypeDef * usartx) {
	sprintf(esp_buffer, "AT+CIPSEND=0,%d\r\n", count);
	writestring(esp_buffer, usartx);
}

int setup_esp(fifo_t *fifo, USART_TypeDef *usartx, USART_TypeDef *debug) {
	flush_fifo_to_usart(fifo, usartx);
	esp_disable_echo(usartx);
	if (esp_check_status(fifo) != 1) {
		writestring("Failed echo disable\n", debug);
		return -1;
	}
	flush_fifo_to_usart(fifo, usartx);
	esp_set_station(usartx);
	if (esp_check_status(fifo) != 1) {
		writestring("Failed set station\n", debug);
		return -1;
	}
	flush_fifo_to_usart(fifo, usartx);
	esp_broadcast_net("myap", "12345678", usartx);
	if (esp_check_status(fifo) != 1) {
		writestring("Failed broadcast net\n", debug);
		return -1;
	}
	flush_fifo_to_usart(fifo, usartx);
	esp_multiplex_en(usartx);
	if (esp_check_status(fifo) != 1) {
		writestring("failed multiplex en\n", debug);
		return -1;
	}
	flush_fifo_to_usart(fifo, usartx);
	esp_listen_incoming_udp(8080, usartx);
	if (esp_check_status(fifo) != 1) {
		writestring("failed udp listen\n", debug);
		return -1;
	}
	flush_fifo_to_usart(fifo, usartx);
	writestring("esp setup success\n", debug);
	return 0;
}

esp_response_enum esp_debug_response(fifo_t *fifo, USART_TypeDef * debug) {
	int buffer_offset;
	int ipd_data_count;
	int link_id;
	fifo_block_empty(fifo);
	char debug_buffer[100];
	switch (fifo_peak(fifo)) {
		case '+': // incoming update
			buffer_offset = fifo_read_until(fifo, debug_buffer, ':', 100);
			if (strnstr(debug_buffer, "+IPD", 100)) {
				sscanf(debug_buffer, "+IPD,%d,%d:", &link_id, &ipd_data_count);
				sprintf(esp_buffer, "Packet in on link %d with length %d\r\n", link_id, ipd_data_count);
				writestring(esp_buffer, debug);
				writestring("Incoming packet data>", debug);
				esp_incoming.count = ipd_data_count;
				while (ipd_data_count > 0){
					fifo_read_n(fifo, esp_incoming.buffer, ipd_data_count % 1024);
					usart_write_n(esp_incoming.buffer, ipd_data_count % 1024, debug);
					ipd_data_count -= (ipd_data_count % 1024);
				}
				writestring("<End of packet data\r\n", debug);
				return ESP_DATA;
			} else {
				writestring("ESP Status Update: ", debug);
				usart_write_n(debug_buffer, buffer_offset, debug);
				buffer_offset = fifo_read_until(fifo, esp_buffer, '\n', 1024);
				usart_write_n(esp_buffer, buffer_offset, debug);

			}
			break;
		case '\r':
			fifo_read_until(fifo, esp_buffer, '\n', 1024); // odd newline
			break;
		default:
			writestring("Got status: ", debug);
			buffer_offset = fifo_read_until(fifo, esp_buffer, '\n', 1024);
			usart_write_n(esp_buffer, buffer_offset, debug);
			break;
	}

	if (strnstr(esp_buffer, "SEND OK", 1024) == esp_buffer) {
		return ESP_SEND_OK;
	}

	if (strnstr(esp_buffer, "OK", 1024) == esp_buffer) {
			return ESP_OK;
		}

	return ESP_STATUS;
}

void esp_send_data(char * buf, int n, USART_TypeDef * usartx, fifo_t * fifo, USART_TypeDef * debug) {
	char tmp = 0;
	int cnt = 0;
	char tbuff[50];
	esp_send_init(n, usartx);
	while (tmp != '>') {
			while (!fifo_empty(fifo)) {
				fifo_pop(fifo, &tmp);
				putcharusart(tmp, debug);
			}
		}
	usart_write_n(buf, n, usartx);
//	cnt = fifo_read_until(fifo, esp_buffer, '\n', 1024);
//	while (cnt < strlen("Recv bytes")) {
//		cnt = fifo_read_until(fifo, esp_buffer, '\n', 1024);
//	}
//	sprintf(tbuff, "Recv %d bytes", n);
//	if (!strnstr(esp_buffer, tbuff, cnt)) {
//		writestring("Send failed\r\n", debug);
//		return;
//	}
//	while (cnt < strlen("SEND OK")) {
//			cnt = fifo_read_until(fifo, esp_buffer, '\n', 1024);
//		}
//	if (!strnstr(esp_buffer, "SEND OK", cnt)) {
//			writestring("Send failed\r\n", debug);
//			return;
//		}
//		writestring("send succeed\r\n", debug);
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

