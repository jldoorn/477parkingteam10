/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ustimer.h"
#include "fifo.h"
#include "esp.h"
#include "string.h"
#include "uartstream.h"
#include "messages.h"
#include "asmutils.h"
#include "sonar.h"
#include "spidisp.h"
#include "keypadtimer.h"
#include "keypad.h"
#include "7segment.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
fifo_t usart5_rx_fifo = { 0 };
fifo_t usart7_rx_fifo = { 0 };
volatile int uscounter = 0;
volatile uint8_t trigger_measurement_event = 0;
volatile uint16_t timeout_tenth_second_counter = 0;
volatile int rows = 0;
volatile uint8_t col = 0;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
char charbuff[1024] = { 0 };

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART5_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
static void MX_USART7_UART_Init(void);
static void MX_TIM14_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int __io_putchar(int c) {
	while (!LL_USART_IsActiveFlag_TXE(USART5)) {
	}
	USART5->TDR = c;

	return c;
}

void sonar_demo() {
	sonar_init();
	while (1) {
		if (read_trigger_val() == SONAR_TRIGGERED) {
			spi_display2("Sonar Trig");
			nano_wait(500000000);
			spi_display2("          ");
		}
	}
}

void sonar_display_distance() {
	uint16_t distance;
	sonar_init();
	while (1) {
		if (trigger_measurement_event) {
			distance = sonar();
//			spi_display1("               ");
			sprintf(charbuff, "dist: %d\r\n", distance);
			writestring(charbuff, USART5);
			trigger_measurement_event = 0;
			nano_wait(200000000);
		}
	}
}

void esp_init_routine(esp_handle_t *esp_handle, int sta_id, int sister_mode) {
	sprintf(charbuff, "192.168.0.%d", sta_id + 2 + sister_mode);
	esp_setup_join("myap", "12345678", esp_handle, charbuff);
	writestring("Connect success!!\r\n", USART5);

	esp_init_udp_station("192.168.0.1", 8080, esp_handle, PTS_AM_UDP_LINK);
	if (sister_mode == 0) {
		esp_new_udp_listen(5000, esp_handle, PTS_SISTER_UDP_LINK);
	} else {
		sprintf(charbuff, "192.168.0.%d", sta_id + 2);
		esp_new_udp_station(charbuff, 5000, esp_handle, PTS_SISTER_UDP_LINK);
	}
}

void pipe_wifi_to_debug() {
	char c;
	LL_GPIO_ResetOutputPin(WIFI_EN_GPIO_Port, WIFI_EN_Pin);
	LL_GPIO_ResetOutputPin(WIFI_RST_GPIO_Port, WIFI_RST_Pin);
	LL_GPIO_ResetOutputPin(DEBUG_7_GPIO_Port, DEBUG_7_Pin);
	LL_GPIO_SetOutputPin(WIFI_EN_GPIO_Port, WIFI_EN_Pin);
	LL_GPIO_SetOutputPin(WIFI_RST_GPIO_Port, WIFI_RST_Pin);
	writestring("Testing 123\r\n", USART5);
	while (1) {
		if (USART5->ISR & USART_ISR_RXNE) {
			c = USART5->RDR;
			putcharusart(c, USART7);
//			putcharusart(c, USART5);
		}
		if (USART7->ISR & USART_ISR_RXNE) {
			putcharusart(USART7->RDR, USART5);
		}
	}
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_SYSCFG);
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

	/* SysTick_IRQn interrupt configuration */
	NVIC_SetPriority(SysTick_IRQn, 3);

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_USART5_UART_Init();
	MX_SPI1_Init();
	MX_TIM2_Init();
	MX_TIM6_Init();
	MX_TIM7_Init();
	MX_USART7_UART_Init();
	MX_TIM14_Init();
	/* USER CODE BEGIN 2 */

	uint8_t station_id;
	uint8_t direction_switch_pos;

	esp_handle_t esp_handle;
	esp_handle.debug = USART5;
	esp_handle.fifo = &usart7_rx_fifo;
	esp_handle.usartx = USART7;
	message_t *espmsg;

	direction_switch_pos =
			LL_GPIO_IsInputPinSet(USER_IN_SW1_GPIO_Port, USER_IN_SW1_Pin) ?
					SWITCH_DIR_IN : SWITCH_DIR_OUT;
	station_id = (
			LL_GPIO_IsInputPinSet(USER_IN_SW2_GPIO_Port, USER_IN_SW2_Pin) ?
					0x4 : 0x0)
			| (LL_GPIO_IsInputPinSet(USER_IN_SW3_GPIO_Port, USER_IN_SW3_Pin) ?
					0x2 : 0x0)
			| (LL_GPIO_IsInputPinSet(USER_IN_SW4_GPIO_Port, USER_IN_SW4_Pin) ?
					0x1 : 0x0);
	sprintf(charbuff, "Station ID: %d, Dir: %d\r\n", station_id,
			direction_switch_pos);
	writestring(charbuff, USART5);
	station_id = 2;
	int str_length = 0;
//  	 NVIC_SetPriority(USART3_8_IRQn, 0);
//  	 NVIC_SetPriority(TIM7_IRQn, 1);
	LL_GPIO_ResetOutputPin(WIFI_EN_GPIO_Port, WIFI_EN_Pin);
	LL_GPIO_SetOutputPin(WIFI_EN_GPIO_Port, WIFI_EN_Pin);

//	LL_USART_EnableIT_RXNE(USART5);
	LL_USART_EnableIT_RXNE(USART7);
//
//	setup_esp(&esp_handle, "myap", "12345678", "192.168.0.1");
//	LL_USART_DisableIT_RXNE(USART7);
//	pipe_wifi_to_debug();

	LL_TIM_EnableIT_UPDATE(TIM7);
	LL_TIM_EnableIT_UPDATE(TIM14);

	int num_spots;
	int sonar_read;

	esp_init_routine(&esp_handle, station_id, 0);

	//spi_display1("                    ");
	//spi_display1("Wifi Connected");
	espmsg = get_message();
	espmsg->module_id = 2;
	espmsg->command = API_PING;
	esp_send_data((char*) espmsg, sizeof(espmsg), &esp_handle, PTS_AM_UDP_LINK);
	writestring("Main: sent ping\r\n", USART5);

	sonar_init();


	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {

		if (LL_TIM_IsEnabledCounter(TIM14)) {
			if (timeout_tenth_second_counter > MAX_DELAY_SISTER_TRIGGER) {
				LL_TIM_DisableCounter(TIM14); // boo hoo, we timed out.
				writestring("Timed out on detection event\r\n", USART5);
			}
		}

		if (!fifo_empty(esp_handle.fifo)) {
			switch (esp_debug_response(&esp_handle)) {
			case ESP_DATA:
				espmsg = (message_t*) esp_incoming.buffer;


				switch (espmsg->command) {
				case API_CAR_DETECT:
					switch (espmsg->body.direction) {
					case API_DIRECTION_IN:
						num_spots--;
						num_spots = num_spots < 0 ? 0 : num_spots;
						break;
					case API_DIRECTION_OUT:
						num_spots++;
						break;
					default:
						break;
					}
					break;
				case API_PING:
					writestring("Got ping from station\r\n", USART5);
					break;
				case API_ACK:
					// stop and reset the timer

					LL_TIM_DisableCounter(TIM14);

					writestring("Got ack from station\r\n", USART5);
					break;
				case API_SISTER_TRIGGER:
					writestring("Got sister trigger from station\r\n", USART5);

					if (LL_TIM_IsEnabledCounter(TIM14)) {
						// is timer running? if so, check trigger status
						if (peek_trigger_val()) {
							// stop timer, reset timer, ack the trigger, transmit flow event
							LL_TIM_DisableCounter(TIM14);

							// ack the trigger
							espmsg->module_id = station_id;
							espmsg->command = API_ACK;
							esp_send_data((char*) espmsg, sizeof(espmsg),
									&esp_handle,
									PTS_SISTER_UDP_LINK);
							writestring("Main: acked the sister trigger\r\n",
							USART5);
							while (esp_debug_response(&esp_handle)
									!= ESP_SEND_OK)
								;

							// transmit flow event
							espmsg->module_id = station_id;
							espmsg->command = API_CAR_DETECT;
							espmsg->body.direction =
									direction_switch_pos == SWITCH_DIR_IN ?
											API_DIRECTION_IN :
											API_DIRECTION_OUT;
							esp_send_data((char*) espmsg, sizeof(espmsg),
									&esp_handle,
									PTS_AM_UDP_LINK);
							writestring("Main: sent flow event\r\n", USART5);
							while (esp_debug_response(&esp_handle)
									!= ESP_SEND_OK)
								;
							writestring("Send success!!\r\n", USART5);
						}
					}

					break;
				default:
					break;
				}

				break;
			case ESP_DISCONNECT:

				esp_init_routine(&esp_handle, 0, 0);

				break;
			default:
				break;
			}

		}

		if (trigger_measurement_event) {

			sonar_read = sonar();
			sprintf(charbuff, "Sonar triggered, dist: %d\r\n", sonar_read);
			writestring(charbuff, USART5);
			trigger_measurement_event = 0;
		}

		if (read_trigger_val() == SONAR_TRIGGERED) {
			timeout_tenth_second_counter = 0; // zero out the timeout
			LL_TIM_EnableCounter(TIM14); // enable counter

			espmsg->module_id = station_id;
			espmsg->command = API_SISTER_TRIGGER;
			espmsg->body.placement = API_PLACEMENT_REAR;

			esp_send_data((char*) espmsg, sizeof(espmsg), &esp_handle,
					PTS_SISTER_UDP_LINK);
			writestring("Main: sent detection to sister\r\n", USART5);
			while (esp_debug_response(&esp_handle) != ESP_SEND_OK)
				;
			writestring("Send success!!\r\n", USART5);

		}

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

//	  esp_debug_response(&usart7_rx_fifo, USART5);
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);
	while (LL_FLASH_GetLatency() != LL_FLASH_LATENCY_1) {
	}
	LL_RCC_HSI48_Enable();

	/* Wait till HSI48 is ready */
	while (LL_RCC_HSI48_IsReady() != 1) {

	}
	LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
	LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
	LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI48);

	/* Wait till System clock is ready */
	while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI48) {

	}
	LL_Init1msTick(48000000);
	LL_SetSystemCoreClock(48000000);
}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void) {

	/* USER CODE BEGIN SPI1_Init 0 */

	/* USER CODE END SPI1_Init 0 */

	LL_SPI_InitTypeDef SPI_InitStruct = { 0 };

	LL_GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* Peripheral clock enable */
	LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_SPI1);

	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
	/**SPI1 GPIO Configuration
	 PA5   ------> SPI1_SCK
	 PA6   ------> SPI1_MISO
	 PA7   ------> SPI1_MOSI
	 PA15   ------> SPI1_NSS
	 */
	GPIO_InitStruct.Pin = LL_GPIO_PIN_5;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_0;
	LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = LL_GPIO_PIN_6;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_0;
	LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = LL_GPIO_PIN_7;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_0;
	LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = LL_GPIO_PIN_15;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_0;
	LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* USER CODE BEGIN SPI1_Init 1 */
//
	/* USER CODE END SPI1_Init 1 */
	/* SPI1 parameter configuration*/
	SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
	SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
	SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_10BIT;
	SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
	SPI_InitStruct.ClockPhase = LL_SPI_PHASE_1EDGE;
	SPI_InitStruct.NSS = LL_SPI_NSS_HARD_OUTPUT;
	SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV256;
	SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
	SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
	SPI_InitStruct.CRCPoly = 7;
	LL_SPI_Init(SPI1, &SPI_InitStruct);
	LL_SPI_SetStandard(SPI1, LL_SPI_PROTOCOL_MOTOROLA);
	LL_SPI_EnableNSSPulseMgt(SPI1);
	/* USER CODE BEGIN SPI1_Init 2 */
//	SPI1->CR1 &= ~SPI_CR1_SPE;
//	    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
//	    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
//
//	    // set pins PA15,PA5,PA6,PA7 for NSS,SCK,MISO,MOSI
//	    GPIOA->MODER &= ~0xc000fc00;
//	    GPIOA->MODER |= 0x8000a800;
//
//	    GPIOA->AFR[0] &= ~0xfff00000;
//	    GPIOA->AFR[1] &= ~0xf0000000;
//	    SPI1->CR1 |= SPI_CR1_BR;
//
//	    SPI1->CR1 |= SPI_CR1_MSTR;
//
//	    SPI1->CR2 = SPI_CR2_SSOE | SPI_CR2_NSSP | SPI_CR2_DS_0 | SPI_CR2_DS_3;
	SPI1->CR1 |= SPI_CR1_SPE;

	/* USER CODE END SPI1_Init 2 */

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	LL_TIM_InitTypeDef TIM_InitStruct = { 0 };

	/* Peripheral clock enable */
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);

	/* TIM2 interrupt Init */
	NVIC_SetPriority(TIM2_IRQn, 0);
	NVIC_EnableIRQ(TIM2_IRQn);

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	TIM_InitStruct.Prescaler = 47;
	TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
	TIM_InitStruct.Autoreload = 4294967294;
	TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
	LL_TIM_Init(TIM2, &TIM_InitStruct);
	LL_TIM_DisableARRPreload(TIM2);
	LL_TIM_SetClockSource(TIM2, LL_TIM_CLOCKSOURCE_INTERNAL);
	LL_TIM_SetTriggerOutput(TIM2, LL_TIM_TRGO_RESET);
	LL_TIM_DisableMasterSlaveMode(TIM2);
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */

}

/**
 * @brief TIM6 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM6_Init(void) {

	/* USER CODE BEGIN TIM6_Init 0 */

	/* USER CODE END TIM6_Init 0 */

	LL_TIM_InitTypeDef TIM_InitStruct = { 0 };

	/* Peripheral clock enable */
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM6);

	/* TIM6 interrupt Init */
	NVIC_SetPriority(TIM6_DAC_IRQn, 0);
	NVIC_EnableIRQ(TIM6_DAC_IRQn);

	/* USER CODE BEGIN TIM6_Init 1 */

	/* USER CODE END TIM6_Init 1 */
	TIM_InitStruct.Prescaler = 47;
	TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
	TIM_InitStruct.Autoreload = 999;
	LL_TIM_Init(TIM6, &TIM_InitStruct);
	LL_TIM_DisableARRPreload(TIM6);
	LL_TIM_SetTriggerOutput(TIM6, LL_TIM_TRGO_RESET);
	LL_TIM_DisableMasterSlaveMode(TIM6);
	/* USER CODE BEGIN TIM6_Init 2 */

	/* USER CODE END TIM6_Init 2 */

}

/**
 * @brief TIM7 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM7_Init(void) {

	/* USER CODE BEGIN TIM7_Init 0 */

	/* USER CODE END TIM7_Init 0 */

	LL_TIM_InitTypeDef TIM_InitStruct = { 0 };

	/* Peripheral clock enable */
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM7);

	/* TIM7 interrupt Init */
	NVIC_SetPriority(TIM7_IRQn, 0);
	NVIC_EnableIRQ(TIM7_IRQn);

	/* USER CODE BEGIN TIM7_Init 1 */

	/* USER CODE END TIM7_Init 1 */
	TIM_InitStruct.Prescaler = 479;
	TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
	TIM_InitStruct.Autoreload = 9999;
	LL_TIM_Init(TIM7, &TIM_InitStruct);
	LL_TIM_DisableARRPreload(TIM7);
	LL_TIM_SetTriggerOutput(TIM7, LL_TIM_TRGO_RESET);
	LL_TIM_DisableMasterSlaveMode(TIM7);
	/* USER CODE BEGIN TIM7_Init 2 */

	/* USER CODE END TIM7_Init 2 */

}

/**
 * @brief TIM14 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM14_Init(void) {

	/* USER CODE BEGIN TIM14_Init 0 */

	/* USER CODE END TIM14_Init 0 */

	LL_TIM_InitTypeDef TIM_InitStruct = { 0 };

	/* Peripheral clock enable */
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM14);

	/* TIM14 interrupt Init */
	NVIC_SetPriority(TIM14_IRQn, 0);
	NVIC_EnableIRQ(TIM14_IRQn);

	/* USER CODE BEGIN TIM14_Init 1 */

	/* USER CODE END TIM14_Init 1 */
	TIM_InitStruct.Prescaler = 4799;
	TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
	TIM_InitStruct.Autoreload = 999;
	TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
	LL_TIM_Init(TIM14, &TIM_InitStruct);
	LL_TIM_DisableARRPreload(TIM14);
	/* USER CODE BEGIN TIM14_Init 2 */

	/* USER CODE END TIM14_Init 2 */

}

/**
 * @brief USART5 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART5_UART_Init(void) {

	/* USER CODE BEGIN USART5_Init 0 */

	/* USER CODE END USART5_Init 0 */

	LL_USART_InitTypeDef USART_InitStruct = { 0 };

	LL_GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* Peripheral clock enable */
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART5);

	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOD);
	/**USART5 GPIO Configuration
	 PC12   ------> USART5_TX
	 PD2   ------> USART5_RX
	 */
	GPIO_InitStruct.Pin = LL_GPIO_PIN_12;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
	LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = LL_GPIO_PIN_2;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
	LL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/* USART5 interrupt Init */
	NVIC_SetPriority(USART3_8_IRQn, 0);
	NVIC_EnableIRQ(USART3_8_IRQn);

	/* USER CODE BEGIN USART5_Init 1 */

	/* USER CODE END USART5_Init 1 */
	USART_InitStruct.BaudRate = 115200;
	USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
	USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
	USART_InitStruct.Parity = LL_USART_PARITY_NONE;
	USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
	USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
	USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
	LL_USART_Init(USART5, &USART_InitStruct);
	LL_USART_DisableIT_CTS(USART5);
	LL_USART_ConfigAsyncMode(USART5);
	LL_USART_Enable(USART5);
	/* USER CODE BEGIN USART5_Init 2 */

	/* USER CODE END USART5_Init 2 */

}

/**
 * @brief USART7 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART7_UART_Init(void) {

	/* USER CODE BEGIN USART7_Init 0 */

	/* USER CODE END USART7_Init 0 */

	LL_USART_InitTypeDef USART_InitStruct = { 0 };

	LL_GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* Peripheral clock enable */
	LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_USART7);

	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
	/**USART7 GPIO Configuration
	 PC0   ------> USART7_TX
	 PC1   ------> USART7_RX
	 */
	GPIO_InitStruct.Pin = LL_GPIO_PIN_0;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
	LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = LL_GPIO_PIN_1;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
	LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/* USART7 interrupt Init */
	NVIC_SetPriority(USART3_8_IRQn, 0);
	NVIC_EnableIRQ(USART3_8_IRQn);

	/* USER CODE BEGIN USART7_Init 1 */

	/* USER CODE END USART7_Init 1 */
	USART_InitStruct.BaudRate = 115200;
	USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
	USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
	USART_InitStruct.Parity = LL_USART_PARITY_NONE;
	USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
	USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
	USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
	LL_USART_Init(USART7, &USART_InitStruct);
	LL_USART_DisableIT_CTS(USART7);
	LL_USART_ConfigAsyncMode(USART7);
	LL_USART_Enable(USART7);
	/* USER CODE BEGIN USART7_Init 2 */

	/* USER CODE END USART7_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	LL_GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOD);

	/**/
	LL_GPIO_ResetOutputPin(WIFI_EN_GPIO_Port, WIFI_EN_Pin);

	/**/
	LL_GPIO_ResetOutputPin(DEBUG_7_GPIO_Port, DEBUG_7_Pin);

	/**/
	LL_GPIO_ResetOutputPin(DEBUG_8_GPIO_Port, DEBUG_8_Pin);

  /**/
  LL_GPIO_ResetOutputPin(LED2_GPIO_Port, LED2_Pin);

	/**/
	LL_GPIO_ResetOutputPin(KEYPADO1_GPIO_Port, KEYPADO1_Pin);

	/**/
	LL_GPIO_ResetOutputPin(PROX_TRIG_GPIO_Port, PROX_TRIG_Pin);

	/**/
	LL_GPIO_ResetOutputPin(WIFI_RST_GPIO_Port, WIFI_RST_Pin);

	/**/
	GPIO_InitStruct.Pin = WIFI_EN_Pin;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(WIFI_EN_GPIO_Port, &GPIO_InitStruct);

	/**/
	GPIO_InitStruct.Pin = KEYPADI1_Pin;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
	LL_GPIO_Init(KEYPADI1_GPIO_Port, &GPIO_InitStruct);

	/**/
	GPIO_InitStruct.Pin = KEYPADI2_Pin;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
	LL_GPIO_Init(KEYPADI2_GPIO_Port, &GPIO_InitStruct);

	/**/
	GPIO_InitStruct.Pin = KEYPADI3_Pin;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
	LL_GPIO_Init(KEYPADI3_GPIO_Port, &GPIO_InitStruct);

	/**/
	GPIO_InitStruct.Pin = DEBUG_7_Pin;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(DEBUG_7_GPIO_Port, &GPIO_InitStruct);

	/**/
	GPIO_InitStruct.Pin = DEBUG_8_Pin;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(DEBUG_8_GPIO_Port, &GPIO_InitStruct);

	/**/
	GPIO_InitStruct.Pin = OUTFLOW_BTN_Pin;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(OUTFLOW_BTN_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LED2_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED2_GPIO_Port, &GPIO_InitStruct);

	/**/
	GPIO_InitStruct.Pin = KEYPADO1_Pin;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(KEYPADO1_GPIO_Port, &GPIO_InitStruct);

	/**/
	GPIO_InitStruct.Pin = USER_IN_SW4_Pin;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(USER_IN_SW4_GPIO_Port, &GPIO_InitStruct);

	/**/
	GPIO_InitStruct.Pin = USER_IN_SW3_Pin;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(USER_IN_SW3_GPIO_Port, &GPIO_InitStruct);

	/**/
	GPIO_InitStruct.Pin = USER_IN_SW2_Pin;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(USER_IN_SW2_GPIO_Port, &GPIO_InitStruct);

	/**/
	GPIO_InitStruct.Pin = USER_IN_SW1_Pin;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(USER_IN_SW1_GPIO_Port, &GPIO_InitStruct);

	/**/
	GPIO_InitStruct.Pin = PROX_TRIG_Pin;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(PROX_TRIG_GPIO_Port, &GPIO_InitStruct);

	/**/
	GPIO_InitStruct.Pin = PROX_MEAS_Pin;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(PROX_MEAS_GPIO_Port, &GPIO_InitStruct);

	/**/
	GPIO_InitStruct.Pin = WIFI_RST_Pin;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(WIFI_RST_GPIO_Port, &GPIO_InitStruct);

	/**/
	GPIO_InitStruct.Pin = STA_ID_0_Pin;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
	LL_GPIO_Init(STA_ID_0_GPIO_Port, &GPIO_InitStruct);

	/**/
	GPIO_InitStruct.Pin = STA_ID_1_Pin;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
	LL_GPIO_Init(STA_ID_1_GPIO_Port, &GPIO_InitStruct);

	/**/
	GPIO_InitStruct.Pin = STA_ID_2_Pin;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
	LL_GPIO_Init(STA_ID_2_GPIO_Port, &GPIO_InitStruct);

	/**/
	GPIO_InitStruct.Pin = STA_DIR_Pin;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
	LL_GPIO_Init(STA_DIR_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
