/*
 * asmutils.c
 *
 *  Created on: Feb 15, 2023
 *      Author: jdoorn
 */
#include "asmutils.h"

void nano_wait(unsigned int n) {
    asm(    "        mov r0,%0\n"
            "repeat: sub r0,#83\n"
            "        bgt repeat\n" : : "r"(n) : "r0", "cc");
}
