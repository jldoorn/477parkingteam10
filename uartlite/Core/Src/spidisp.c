/*
 * spidisp.c
 *
 *  Created on: Feb 15, 2023
 *      Author: jdoorn
 */


#include "spidisp.h"
#include "asmutils.h"


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
