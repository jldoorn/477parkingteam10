/*
 * keypadtimer.c
 *
 *  Created on: Feb 16, 2023
 *      Author: fkepl
 */

#include "keypadtimer.h"
const char keymap[] = "DCBA#9630852*741";
uint8_t hist[16];
int n;
queue[2];
int qin;
int qout;

int read_rows() {
    return (GPIOC->IDR & (0xf << 2))>>2;
}

void update_history(int c, int rows) {
    for(int i = 0; i < 4; i++) {
            hist[4*c+i] = (hist[4*c+i]<<1) + ((rows>>i)&1);
            if (hist[4*c+i] == 1)
              push_queue(4*c+i);
        }
}

void push_queue(int n) {
    n = (n & 0xff) | 0x80;
    queue[qin] = n;
    qin ^= 1;
}

void drive_column(int c) {
    GPIOC->BSRR = (0xf00000 | (1 << (c + 4))) << 2;
}

uint8_t pop_queue() {
    uint8_t tmp = queue[qout] & 0x7f;
    queue[qout] = 0;
    qout ^= 1;
    return tmp;
}

char get_keypress() {
    for(;;) {
        asm volatile ("wfi" : :);   // wait for an interrupt
        if (queue[qout] == 0)
            continue;
        return keymap[pop_queue()];
    }
}
