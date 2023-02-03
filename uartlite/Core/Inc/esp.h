/*
 * esp.h
 *
 *  Created on: Feb 1, 2023
 *      Author: jdoorn
 */
#include "fifo.h"
#include "stm32f091xc.h"

#ifndef INC_ESP_H_
#define INC_ESP_H_
#define UDP_BUF_SIZE 1024


typedef enum {ESP_STATUS, ESP_DATA, ESP_RESULTS, ESP_READY, ESP_PARSE_FAIL, ESP_OK, ESP_ERROR, ESP_SEND_OK, ESP_SEND_FAIL, ESP_SET_OK} esp_response_enum;
//typedef enum {ESP_OK, ESP_ERROR, ESP_SEND_OK, ESP_SEND_FAIL, ESP_SET_OK} esp_status_enum;

typedef struct {
	int count;
	char buffer[UDP_BUF_SIZE];
} esp_data_incoming_t;

//typedef struct {
//	esp_response_enum status;
//	union {
//		esp_status_enum esp_status_code;
//		esp_data_response_t esp_data_header;
//	} body;
//} esp_response_t;
int setup_esp(fifo_t *fifo, USART_TypeDef *usartx, USART_TypeDef *debug);
esp_response_enum esp_debug_response(fifo_t *fifo, USART_TypeDef * debug);
void esp_send_data(char * buf, int n, USART_TypeDef * usartx, fifo_t * fifo, USART_TypeDef * debug) ;

extern esp_data_incoming_t esp_incoming;
//void esp_disable_echo();
//int esp_check_status(fifo_t * fifo);
//
//void esp_set_station();
//
//void esp_broadcast_net(char * ssid, char * pass);

#endif /* INC_ESP_H_ */
