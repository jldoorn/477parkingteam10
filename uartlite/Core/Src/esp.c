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

void esp_listen_incoming_udp(int port, USART_TypeDef *usartx, int link_id) {
	sprintf(esp_buffer, "AT+CIPSTART=%d,\"UDP\",\"0.0.0.0\",%d,%d,2\r\n",link_id, port,
			port);
	writestring(esp_buffer, usartx);
}

// Gets modem to point of
void esp_send_mux_init(int count, USART_TypeDef *usartx, int link_id) {
	sprintf(esp_buffer, "AT+CIPSEND=%d,%d\r\n", link_id, count);
	writestring(esp_buffer, usartx);
}

void esp_send_simple_init(int count, USART_TypeDef *usartx) {
	sprintf(esp_buffer, "AT+CIPSEND=%d\r\n", count);
	writestring(esp_buffer, usartx);
}
void esp_reset(USART_TypeDef *usartx) {
	LL_GPIO_ResetOutputPin(WIFI_EN_GPIO_Port, WIFI_EN_Pin);
		LL_GPIO_ResetOutputPin(WIFI_RST_GPIO_Port, WIFI_RST_Pin);
//		LL_GPIO_ResetOutputPin(DEBUG_7_GPIO_Port, DEBUG_7_Pin);
		LL_GPIO_SetOutputPin(WIFI_EN_GPIO_Port, WIFI_EN_Pin);
		LL_GPIO_SetOutputPin(WIFI_RST_GPIO_Port, WIFI_RST_Pin);
	writestring("AT+RST\r\n", usartx);
}

void esp_gateway_ip_set(char *gateway_ip, USART_TypeDef *usartx) {
	sprintf(esp_buffer, "AT+CIPAP=\"%s\"\r\n", gateway_ip);
	writestring(esp_buffer, usartx);
}

/**
 * Configures the ESP in Access point mode and opens port to receive UDP packets
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

	esp_listen_incoming_udp(8080, esp->usartx, 0);
	while (esp_debug_response(esp) != ESP_OK)
		;

	writestring("esp setup success\r\n", esp->debug);
	return 0;
}


// Join an existing access point
void esp_join_ap(USART_TypeDef *usartx, char *ssid, char *pw) {
	sprintf(esp_buffer, "AT+CWJAP=\"%s\",\"%s\"\r\n", ssid, pw);
	writestring(esp_buffer, usartx);
}

// set ip address (static) for a wifi station
// IP a string, must be in the format "%d.%d.%d.%d"
void esp_sta_set_ip(USART_TypeDef *usartx, char * ip_addr) {
	sprintf(esp_buffer, "AT+CIPSTA=\"%s\"\r\n", ip_addr);
	writestring(esp_buffer, usartx);
}

// Example of how to initialize station, connect to a network, and set a static ip
void esp_setup_join(char *ssid, char *pw, esp_handle_t *esp, char * static_ip) {
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
	esp_sta_set_ip(esp->usartx, static_ip);
	while (esp_debug_response(esp) != ESP_OK)
			;
}

void esp_ap_set_ip(USART_TypeDef *usartx, char * ip_addr) {
	sprintf(esp_buffer, "AT+CIPAP=\"%s\",\"%s\",\"255.255.255.0\"\r\n", ip_addr, ip_addr);
	writestring(esp_buffer, usartx);
}




void esp_udp_single_conn(USART_TypeDef *usartx, char *ip_conn, int port) {
	sprintf(esp_buffer, "AT+CIPSTART=\"UDP\",\"%s\",%d\r\n", ip_conn, port);
	writestring(esp_buffer, usartx);
}

void esp_udp_multi_conn(USART_TypeDef *usartx, char * ip_conn, int port, int conn_id) {
	sprintf(esp_buffer, "AT+CIPSTART=%d,\"UDP\",\"%s\",%d\r\n",conn_id, ip_conn, port);
	writestring(esp_buffer, usartx);
}

// Sets an ESP to multiplex udp connections, then connects to an UDP socket at ipconn, port
// conn_id will be used for sending data. There can be four simultaneous connections, id 0, 1, 2, 3
void esp_init_udp_station(char *ip_conn, int port, esp_handle_t *esp, int conn_id) {
	if (conn_id > 3 || conn_id < 0) {
		writestring("Error,  conn_id must be in range [0,3]\r\n", esp->debug);
	}
	esp_multiplex_set(esp->usartx, ESP_MUX_MULTI);
	while (esp_debug_response(esp) != ESP_OK)
		;

	esp_udp_multi_conn(esp->usartx, ip_conn, port, conn_id);
	while (esp_debug_response(esp) != ESP_OK)
		;
}

void esp_new_udp_station(char *ip_conn, int port, esp_handle_t *esp, int conn_id) {
	esp_udp_multi_conn(esp->usartx, ip_conn, port, conn_id);
	while (esp_debug_response(esp) != ESP_OK)
		;
}

void esp_new_udp_listen(int port, esp_handle_t *esp, int conn_id) {
	esp_listen_incoming_udp(port, esp->usartx, conn_id);
	while (esp_debug_response(esp) != ESP_OK)
			;
}

// may need to do UDP mux for station...


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

//#ifdef ESP_STA
//			sscanf(debug_buffer, "+IPD,%d:",&ipd_data_count);
//						sprintf(esp_buffer, "Packet in on station with length %d\r\n",
//								ipd_data_count);
//#endif

//#ifdef ESP_AP
			sscanf(debug_buffer, "+IPD,%d,%d:", &link_id, &ipd_data_count);
			sprintf(esp_buffer, "Packet in on AP with length %d\r\n",
					ipd_data_count);
//#endif

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
			esp_incoming.link_id = link_id;
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
		buffer_offset = fifo_read_until(esp->fifo, esp_buffer, '\n', 1024);
		if (strnstr(esp_buffer, "WIFI DISCONNECT", buffer_offset)) {
			return ESP_DISCONNECT;
		}
		writestring("Got status: ", esp->debug);


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

void esp_send_data(char *buf, int n, esp_handle_t *esp, int link_id) {
	char tmp = 0;

		esp_send_mux_init(n, esp->usartx, link_id);

	while (tmp != '>') {
		while (!fifo_empty(esp->fifo)) {
			fifo_pop(esp->fifo, &tmp);
			putcharusart(tmp, esp->debug);
		}
	}
	usart_write_n(buf, n, esp->usartx);

}


