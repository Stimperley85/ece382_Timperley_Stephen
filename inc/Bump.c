// Bump.c
// Runs on MSP432
// Provide low-level functions that interface bump switches the robot.
// Daniel Valvano and Jonathan Valvano
// July 11, 2019

/* This example accompanies the book
   "Embedded Systems: Introduction to Robotics,
   Jonathan W. Valvano, ISBN: 9781074544300, copyright (c) 2019
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/

Simplified BSD License (FreeBSD License)
Copyright (c) 2019, Jonathan Valvano, All rights reserved.

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

// Negative logic bump sensors
// P4.7 Bump6, left side of robot
// P4.6 Bump5
// P4.5 Bump4
// P4.3 Bump3
// P4.2 Bump2
// P4.0 Bump1, right side of robot

#include <stdint.h>
#include "msp.h"
#include "../inc/Clock.h"


// Initialize Bump sensors
// Make six Port 4 pins inputs
// Activate interface pullup
// pins 7,6,5,3,2,0
void Bump_Init(void){
    // write this as part of Homework 8
    // 1) configure P4.7-P4.5, P4.3, P4.2, and P4.0 as GPIO
    //1110 1101 = 0xED
    //0001 0010 = 0x12
    P4->SEL0 &= ~0xED; // change select0 to 0
    P4->SEL1 &= ~0xED; // change select1 to 0

    // 2) make P4.7-P4.5, P4.3, P4.2, and P4.0 in
    P4->DIR &= ~0xED; //change DIR to 0
    // 3) enable pull resistors on P4.7-P4.5, P4.3, P4.2, and P4.0
    P4->REN |= 0xED; //change REN to 1
    //    P4.7-P4.5, P4.3, P4.2, and P4.0 are pull-up
    P4->OUT |= 0xED; //change OUT to 1
}


// Read current state of 6 switches
// Returns a 6-bit positive logic result (0 to 63)
// bit 5 Bump6
// bit 4 Bump5
// bit 3 Bump4
// bit 2 Bump3
// bit 1 Bump2
// bit 0 Bump1
uint8_t Bump_Read(void) {
    // write this as part of Lab 8
    // 1)read the sensors (which are active low) and convert to active high
    uint8_t S1 = (~(P4->IN)) & 0x01; //read data and take the first inverted bit
    uint8_t S2 = (~(P4->IN)) & 0x0C; //read data and take the next inverted bits
    uint8_t S3 = (~(P4->IN)) & 0xE0; //read data and take the next inverted bits
    uint8_t S4 = 0x00; //initialize 6 (really 8) bit return value
    // 2. Select, shift, combine, and output
    S2 = S2 >> 1; //shift 1 right
    S3 = S3 >> 2; //shift 1 right
    S4 = (S1 | S2 | S3); //or all bits together to get final value
    return S4; // return the 6 bit bump data
}

