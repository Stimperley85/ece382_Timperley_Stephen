/*
PowerMain.c
Runs on any Cortex M

Documentation statement:
Power functions - iterative and recursive
Dr. Stan Baek
June 03 2021

Simplified BSD License (FreeBSD License)
Copyright (c) 2021, Stan Baek, All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are
those of the authors and should not be interpreted as representing official
policies, either expressed or implied, of the FreeBSD Project.
*/

#include <stdint.h>
#include "msp.h"

extern uint32_t PowerASM_1(uint32_t, uint32_t);
extern uint32_t PowerASM_2(uint32_t, uint32_t);


uint32_t power1(uint32_t base, uint32_t exp) {

    uint32_t ret = 1; // if exp == 0, it will return 1
    while (exp > 0) {
        ret = ret * base;
        exp--;
    }
    return ret;
}


// Recursive
uint32_t power2(uint32_t base, uint32_t exp) {

    if (exp == 0) {
        return 1;
    } else {
        return base * power2(base, exp-1);
    }
}


void Program1(void) {
    uint32_t x = power1(2, 3);
    return;
}


void Program2(void) {
    uint32_t x = power2(2, 3);
    return;
}


void Program3(void) {
    uint32_t x = PowerASM_1(0, 1);
    return;
}


void Program4(void) {
    uint32_t x = PowerASM_2(2, 3);
    return;
}

void main(void) {

    //Program1();
    //Program2();
    Program3();
    Program4();

    while(1);
}
