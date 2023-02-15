#include "stm32f0xx.h"

// Be sure to change this to your login...
const char login[] = "hlovell";

void nano_wait(unsigned int n) {
    asm(    "        mov r0,%0\n"
            "repeat: sub r0,#83\n"
            "        bgt repeat\n" : : "r"(n) : "r0", "cc");
}


//===========================================================================
// 2.1 FROM LAB 6
// ..........................................................................
// Configuring GPIO
//===========================================================================
void enable_ports(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;

    GPIOB->MODER &= ~0xCC00;
    GPIOB->MODER |= 0x4000;

    GPIOC->MODER &= ~0xffff;
    GPIOC->MODER |= 0x5500;

    GPIOC->PUPDR &= ~0xff;
    GPIOC->PUPDR |= 0xaa;
}

//===========================================================================
// Debouncing a Keypad
//===========================================================================
const char keymap[] = "DCBA#9630852*741";
uint8_t hist[16];
uint8_t col;
char queue[2];
int qin;
int qout;

void drive_column(int c) {
    GPIOC->BSRR = 0xf00000 | (1 << (c + 4));
}

int read_rows() {
    return GPIOC->IDR & 0xf;
}

void push_queue(int n) {
    n = (n & 0xff) | 0x80;
    queue[qin] = n;
    qin ^= 1;
}

uint8_t pop_queue() {
    uint8_t tmp = queue[qout] & 0x7f;
    queue[qout] = 0;
    qout ^= 1;
    return tmp;
}

void update_history(int c, int rows) {
    for(int i = 0; i < 4; i++) {
        hist[4*c+i] = (hist[4*c+i]<<1) + ((rows>>i)&1);
        if (hist[4*c+i] == 1)
          push_queue(4*c+i);
    }
}

char get_keypress() {
    for(;;) {
        asm volatile ("wfi" : :);   // wait for an interrupt
        if (queue[qout] == 0)
            continue;
        return keymap[pop_queue()];
    }
}
// temp here
uint16_t msg[8] = { 0x0000,0x0100,0x0200,0x0300,0x0400,0x0500,0x0600,0x0700 };
void append_display(char val) {
    uint16_t tmp[8] = { 0x0000,0x0100,0x0200,0x0300,0x0400,0x0500,0x0600,0x0700 };
    for(int i = 0; i<8; i++) {
        msg[i] = msg[i+1] & 0xff;
        msg[i] |= tmp[i];
    }
    msg[7] = val;
    msg[7] |= tmp[7];
}


//===========================================================================
// This Lab 7
// ..........................................................................
// 2.2 Bit Bang SPI LED Array
//===========================================================================
int msg_index = 0;
//uint16_t msg[8] = { 0x0000,0x0100,0x0200,0x0300,0x0400,0x0500,0x0600,0x0700 };
extern const char font[];

void setup_bb(void) {
    GPIOB->MODER &= ~0xcf000000;
    GPIOB->MODER |= 0x45000000;

    GPIOB->ODR &= ~0x2000;
    GPIOB->ODR |= 0x1000;
}

void small_delay(void) {
//    nano_wait(5000000);
}

void bb_write_bit(int val) {
    if (val == 0) {
        GPIOB->ODR &= ~0x8000;
    }
    else {
        GPIOB->ODR |= 0x8000;
    }
    small_delay();
    GPIOB->ODR |= 0x2000;
    small_delay();
    GPIOB->ODR &= ~0x2000;
}

void bb_write_halfword(int halfword) {
    GPIOB->ODR &= ~0x1000;

    for (int j = 15; j >= 0; j--) { bb_write_bit( (halfword>>j) & 1 ); }

    GPIOB->ODR |= 0x1000;
}

void init_tim7(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;

    TIM7->PSC = 48-1;

    TIM7->ARR = 1000-1;

    TIM7->DIER |= TIM_DIER_UIE;
    TIM7->CR1 |= TIM_CR1_CEN;

    NVIC->ISER[0] = 1<<TIM7_IRQn;
}

void TIM7_IRQHandler(void) {
    TIM7->SR &= ~TIM_SR_UIF;

    bb_write_halfword(msg[msg_index]);
    msg_index++;
    if (msg_index == 8) {
        msg_index = 0;
    }
}


//===========================================================================
// 2.3 SPI DMA LED Array
//===========================================================================
void init_spi2(void) {
    SPI2->CR1 &= ~SPI_CR1_SPE;

    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;

    // set pins PB12,PB13,PB15 for SPI2 NSS, SCK, and MOSI
    GPIOB->MODER &= ~0xcf000000;
    GPIOB->MODER |= 0x8a000000;

    GPIOB->AFR[1] &= ~0xf0ff0000;

    SPI2->CR1 |= SPI_CR1_BR;

    SPI2->CR1 |= SPI_CR1_MSTR;

    SPI2->CR2 = SPI_CR2_SSOE | SPI_CR2_NSSP | SPI_CR2_DS;

    SPI2->CR2 |= SPI_CR2_TXDMAEN;

    SPI2->CR1 |= SPI_CR1_SPE;
}

void setup_dma(void) {
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;

    DMA1_Channel5->CCR &= ~DMA_CCR_EN;          // make sure it is off

    DMA1_Channel5->CPAR = &(SPI2->DR);

    DMA1_Channel5->CMAR = (uint32_t) msg;

    DMA1_Channel5->CNDTR = 8;

    DMA1_Channel5->CCR |= DMA_CCR_DIR;

    DMA1_Channel5->CCR |= DMA_CCR_MINC;

    DMA1_Channel5->CCR &= ~DMA_CCR_MSIZE;
    DMA1_Channel5->CCR |= DMA_CCR_MSIZE_0;

    DMA1_Channel5->CCR &= ~DMA_CCR_PSIZE;
    DMA1_Channel5->CCR |= DMA_CCR_PSIZE_0;

    DMA1_Channel5->CCR |= DMA_CCR_CIRC;
}

void enable_dma(void) {
    DMA1_Channel5->CCR |= DMA_CCR_EN;
}


//===========================================================================
// 2.4 SPI OLED Display
//===========================================================================
void setup_spi1() {
    SPI1->CR1 &= ~SPI_CR1_SPE;

    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

    // set pins PA15,PA5,PA6,PA7 for NSS,SCK,MISO,MOSI
    GPIOA->MODER &= ~0xc000fc00;
    GPIOA->MODER |= 0x8000a800;

    GPIOA->AFR[0] &= ~0xfff00000;
    GPIOA->AFR[1] &= ~0xf0000000;

    SPI1->CR1 |= SPI_CR1_BR;

    SPI1->CR1 |= SPI_CR1_MSTR;

    SPI1->CR2 = SPI_CR2_SSOE | SPI_CR2_NSSP | SPI_CR2_DS_0 | SPI_CR2_DS_3;

    SPI1->CR1 |= SPI_CR1_SPE;
}

void spi_cmd(unsigned int data) {
    while ((SPI1->SR & SPI_SR_TXE) != SPI_SR_TXE);

    SPI1->DR = data;
}

void spi_data(unsigned int data) {
    while ((SPI1->SR & SPI_SR_TXE) != SPI_SR_TXE);

    data |= 0x200;

    SPI1->DR = data;
}

void spi_init_oled() {
    nano_wait(1000000);

    spi_cmd(0x38);

    spi_cmd(0x08);

    spi_cmd(0x01);

    nano_wait(2000000);

    spi_cmd(0x06);

    spi_cmd(0x02);

    spi_cmd(0x0c);
}

void spi_display1(const char *string) {
    spi_cmd(0x02);

    int k = 0;
    while (string[k] != '\0') {
        spi_data(string[k]);
        k++;
    }
}

void spi_display2(const char *string) {
    spi_cmd(0xc0);

    int k = 0;
        while (string[k] != '\0') {
            spi_data(string[k]);
            k++;
        }
}


//===========================================================================
// Main and supporting functions
//===========================================================================

#include <time.h>

int counter = 0;

void us_enable(void) {
	GPIOB->ODR |= 0x80;
	return;
}

void us_disable(void) {
	GPIOB->ODR &= ~0x80;
	return;
}

void init_tim2(void) {
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	TIM2->PSC = 48-1;
	TIM2->ARR = 10-1;
	TIM2->DIER |= TIM_DIER_UIE;
	TIM2->CR1 |= TIM_CR1_CEN;
	NVIC->ISER[0] |= 1 << TIM2_IRQn;

}

void TIM2_IRQHandler(void) {
    TIM2->SR &= ~TIM_SR_UIF;

    counter += 1;
}

int sonar(void) {

	us_enable();
	nano_wait(10000);
	us_disable();

	nano_wait(10000);
	us_enable();


	while( ((GPIOB->IDR>>5) & 1) == 0);

	counter = 0;

	while( ((GPIOB->IDR >> 5) & 1) == 1 );


	return counter;

}

int main(void)
{


    // GPIO enable
    enable_ports();
    us_enable();
    // setup keypad
    //init_tim6();

    // 2.4 SPI OLED
    init_tim2();
     setup_spi1();
     spi_init_oled();
     spi_display1("Distance:");

     // Testing
    int distance = 0;
    char distChar[32];
    int distanceIn = 0;

    for (;;) {
    	distance = sonar();
    	distanceIn = distance / 14.8;
    	sprintf(distChar, "%d", distanceIn);
    	spi_display2("       ");
    	spi_display2(distChar);
    	nano_wait(100000000);
    }

}

const char font[] = {
    0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
    0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
    0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
    0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
    0x00, // 32: space
    0x86, // 33: exclamation
    0x22, // 34: double quote
    0x76, // 35: octothorpe
    0x00, // dollar
    0x00, // percent
    0x00, // ampersand
    0x20, // 39: single quote
    0x39, // 40: open paren
    0x0f, // 41: close paren
    0x49, // 42: asterisk
    0x00, // plus
    0x10, // 44: comma
    0x40, // 45: minus
    0x80, // 46: period
    0x00, // slash
    // digits
    0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07, 0x7f, 0x67,
    // seven unknown
    0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
    // Uppercase
    0x77, 0x7c, 0x39, 0x5e, 0x79, 0x71, 0x6f, 0x76, 0x30, 0x1e, 0x00, 0x38, 0x00,
    0x37, 0x3f, 0x73, 0x7b, 0x31, 0x6d, 0x78, 0x3e, 0x00, 0x00, 0x00, 0x6e, 0x00,
    0x39, // 91: open square bracket
    0x00, // backslash
    0x0f, // 93: close square bracket
    0x00, // circumflex
    0x08, // 95: underscore
    0x20, // 96: backquote
    // Lowercase
    0x5f, 0x7c, 0x58, 0x5e, 0x79, 0x71, 0x6f, 0x74, 0x10, 0x0e, 0x00, 0x30, 0x00,
    0x54, 0x5c, 0x73, 0x7b, 0x50, 0x6d, 0x78, 0x1c, 0x00, 0x00, 0x00, 0x6e, 0x00
};
