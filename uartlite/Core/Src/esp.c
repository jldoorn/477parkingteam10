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
esp_data_incoming_t esp_incoming = { 0 };

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

void esp_set_mode(USART_TypeDef *usartx, int mode) {
	sprintf(esp_buffer, "AT+CWMODE=%d\r\n", mode);
	writestring(esp_buffer, usartx);
}

void esp_broadcast_net(char *ssid, char *pass, USART_TypeDef *usartx) {
	sprintf(esp_buffer, "AT+CWSAP=\"%s\",\"%s\",5,3\r\n", ssid, pass);
	writestring(esp_buffer, usartx);
}

void esp_multiplex_set(USART_TypeDef *usartx, int mux_mode) {
	sprintf(esp_buffer, "AT+CIPMUX=%d\r\n", mux_mode);
	writestring(esp_buffer, usartx);
}

void esp_listen_incoming_udp(int port, USART_TypeDef *usartx) {
	sprintf(esp_buffer, "AT+CIPSTART=0,\"UDP\",\"0.0.0.0\",%d,%d,2\r\n", port,
			port);
	writestring(esp_buffer, usartx);
}

// Gets modem to point of
void esp_send_mux_init(int count, USART_TypeDef *usartx) {
	sprintf(esp_buffer, "AT+CIPSEND=0,%d\r\n", count);
	writestring(esp_buffer, usartx);
}

void esp_send_simple_init(int count, USART_TypeDef *usartx) {
	sprintf(esp_buffer, "AT+CIPSEND=%d\r\n", count);
	writestring(esp_buffer, usartx);
}
void esp_reset(USART_TypeDef *usartx) {
	writestring("AT+RST\r\n", usartx);
}

void esp_gateway_ip_set(char *gateway_ip, USART_TypeDef *usartx) {
	sprintf(esp_buffer, "AT+CIPAP=\"%s\"\r\n", gateway_ip);
	writestring(esp_buffer, usartx);
}

/**
 * Configures the ESP in station mode and opens port to receive UDP packets
 *
 * @param fifo a pointer to the fifo that is filled from the interrupt handling the RX from the ESP
 * @param usartx a pointer to the USART that the ESP is attached to
 * @param debug	a pointer to the USART that the debug messages should be sent to
 * @return zero if setup successful
 */
int setup_esp(esp_handle_t *esp, char *ssid, char *pwd, char *gateway_ip) {
	flush_fifo_to_usart(esp->fifo, esp->debug);
	writestring("resetting esp\r\n", esp->debug);
	esp_reset(esp->usartx);
	while (esp_debug_response(esp) != ESP_READY)
		;
	writestring("disabling echo\r\n", esp->debug);
	esp_disable_echo(esp->usartx);
	while (esp_debug_response(esp) != ESP_OK)
		;
	writestring("setting ap mode\r\n", esp->debug);
	esp_set_mode(esp->usartx, ESP_MODE_AP);
	while (esp_debug_response(esp) != ESP_OK)
		;

	esp_gateway_ip_set(gateway_ip, esp->usartx);
	while (esp_debug_response(esp) != ESP_OK)
		;

	esp_broadcast_net(ssid, pwd, esp->usartx);
	while (esp_debug_response(esp) != ESP_OK)
		;

	esp_multiplex_set(esp->usartx, ESP_MUX_MULTI);
	while (esp_debug_response(esp) != ESP_OK)
		;

	esp_listen_incoming_udp(8080, esp->usartx);
	while (esp_debug_response(esp) != ESP_OK)
		;

	writestring("esp setup success\r\n", esp->debug);
	return 0;
}

void esp_join_ap(USART_TypeDef *usartx, char *ssid, char *pw) {
	sprintf(esp_buffer, "AT+CWJAP=\"%s\",\"%s\"\r\n", ssid, pw);
	writestring(esp_buffer, usartx);
}

void esp_setup_join(char *ssid, char *pw, esp_handle_t *esp) {
	flush_fifo_to_usart(esp->fifo, esp->debug);
	esp_reset(esp->usartx);
	while (esp_debug_response(esp) != ESP_READY)
		;
	esp_disable_echo(esp->usartx);
	while (esp_debug_response(esp) != ESP_OK)
		;

	esp_set_mode(esp->usartx, ESP_MODE_STATION);
	while (esp_debug_response(esp) != ESP_OK)
		;

	esp_join_ap(esp->usartx, ssid, pw);
	while (esp_debug_response(esp) != ESP_OK)
		;
}

void esp_udp_single_conn(USART_TypeDef *usartx, char *ip_conn, int port) {
	sprintf(esp_buffer, "AT+CIPSTART=\"UDP\",\"%s\",%d\r\n", ip_conn, port);
	writestring(esp_buffer, usartx);
}

void esp_init_udp_station(char *ip_conn, int port, esp_handle_t *esp) {
	esp_multiplex_set(esp->usartx, ESP_MUX_SINGLE);
	while (esp_debug_response(esp) != ESP_OK)
		;

	esp_udp_single_conn(esp->usartx, ip_conn, port);
	while (esp_debug_response(esp) != ESP_OK)
		;
}

/**
 * Blocks on the ESP module until new content received, then decodes the type of
 * message. If the message is incoming data, this method will update the global
 * esp_incoming object
 *
 * @param fifo 	a pointer to the fifo that is filled from the interrupt handling
 * 				the RX from the ESP
 * @param debug	a pointer to the USART that the debug messages should be sent to
 * @return The status if any of the ESP message
 */
esp_response_enum esp_debug_response(esp_handle_t *esp) {
	int buffer_offset;
	int ipd_data_count;
	int link_id;
	fifo_block_empty(esp->fifo);
	char debug_buffer[100];
	switch (fifo_peak(esp->fifo)) {
	case '+': // incoming update
		buffer_offset = fifo_read_until(esp->fifo, debug_buffer, ':', 100);
		usart_write_n(debug_buffer, buffer_offset % 1024, esp->debug);
		if (strnstr(debug_buffer, "+IPD", 100)) {

#ifdef ESP_STA
			sscanf(debug_buffer, "+IPD,%d:",&ipd_data_count);
						sprintf(esp_buffer, "Packet in on station with length %d\r\n",
								ipd_data_count);
#endif

#ifdef ESP_AP
			sscanf(debug_buffer, "+IPD,%d,%d:", &link_id, &ipd_data_count);
			sprintf(esp_buffer, "Packet in on AP with length %d\r\n",
					ipd_data_count);
#endif

			writestring(esp_buffer, esp->debug);
			writestring("Incoming packet data>", esp->debug);
			esp_incoming.count = ipd_data_count;
			while (ipd_data_count > 0) {
				fifo_read_n(esp->fifo, esp_incoming.buffer,
						ipd_data_count % 1024);
				usart_write_n(esp_incoming.buffer, ipd_data_count % 1024,
						esp->debug);
				ipd_data_count -= (ipd_data_count % 1024);
			}
			writestring("<End of packet data\r\n", esp->debug);
			return ESP_DATA;
		} else {
			writestring("ESP Status Update: ", esp->debug);
			usart_write_n(debug_buffer, buffer_offset, esp->debug);
			buffer_offset = fifo_read_until(esp->fifo, esp_buffer, '\n', 1024);
			usart_write_n(esp_buffer, buffer_offset, esp->debug);

		}
		break;
	case '\r':
		fifo_read_until(esp->fifo, esp_buffer, '\n', 1024); // odd newline
		break;
	default:
		writestring("Got status: ", esp->debug);
		buffer_offset = fifo_read_until(esp->fifo, esp_buffer, '\n', 1024);
		usart_write_n(esp_buffer, buffer_offset, esp->debug);
		break;
	}

	if (strnstr(esp_buffer, "SEND OK", 1024) == esp_buffer) {
		return ESP_SEND_OK;
	}

	if (strnstr(esp_buffer, "ready", 1024) == esp_buffer) {
		return ESP_READY;
	}

	if (strnstr(esp_buffer, "OK", 1024) == esp_buffer) {
		return ESP_OK;
	}

	return ESP_STATUS;
}

void esp_send_data(char *buf, int n, esp_handle_t *esp, int multi) {
	char tmp = 0;
	if (multi) {
		esp_send_mux_init(n, esp->usartx);
	} else {
		esp_send_simple_init(n, esp->usartx);
	}
	while (tmp != '>') {
		while (!fifo_empty(esp->fifo)) {
			fifo_pop(esp->fifo, &tmp);
			putcharusart(tmp, esp->debug);
		}
	}
	usart_write_n(buf, n, esp->usartx);

}


