// Lab05_SoftwareDesignmain.c
// Runs on MSP432
// things to the LCD.
// Stan Baek
// May 29, 2022

/*
Simplified BSD License (FreeBSD License)
Copyright (c) 2022, Stanley Baek, All rights reserved.

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

// Red SparkFun Nokia 5110 (LCD-10168)
// -----------------------------------
// Signal        (Nokia 5110) LaunchPad pin
// 3.3V          (VCC, pin 1) power
// Ground        (GND, pin 2) ground
// UCA3STE       (SCE, pin 3) connected to P9.4
// Reset         (RST, pin 4) connected to P9.3
// Data/Command  (D/C, pin 5) connected to P9.6
// UCA3SIMO      (DN,  pin 6) connected to P9.7
// UCA3CLK       (SCLK, pin 7) connected to P9.5
// back light    (LED, pin 8) not connected, consists of 4 3.3 V white LEDs which draw ~80mA total

// Blue Adafruit 338 Nokia 5110
// ---------------
// Signal        (Nokia 5110) LaunchPad pin
// Ground        (Gnd, pin 1) ground
// 3.3V          (Vcc, pin 2) power
// UCA3CLK       (Clk, pin 3) connected to P9.5
// UCA3SIMO      (Din, pin 4) connected to P9.7
// Data/Command  (D/C, pin 5) connected to P9.6
// UCA3STE       (CS,  pin 6) connected to P9.4
// Reset         (RST, pin 7) connected to P9.3
// back light    (LED, pin 8) not connected, consists of 4 white LEDs which draw ~80mA total

#include <stdbool.h>
#include <stdint.h>
#include "msp.h"
#include "../inc/Clock.h"
#include "../inc/LaunchPad.h"
#include "../inc/Nokia5110.h"


// =============== Program 5.1 =====================================

// Run this program to find the best contrast for your Nokia 5110 LCD.
// It will display the USAF symbol.
void Program5_1(void) {

    Clock_Init48MHz();  // set system clock to 48 MHz
    Nokia5110_Init();

    // Contrast value 0xB1 looks good on red SparkFun.
    // Adjust this from 0xA0 (lighter) to 0xCF (darker) if necessary.
    uint8_t const contrast = 0xB1;
    Nokia5110_SetContrast(contrast);

    Nokia5110_Clear();

    while(1) {

        for(int count = 0; count < 36; count++) {
            // three step process: clear, build, display
            Nokia5110_ClearBuffer();
            Nokia5110_PrintBMP(count, 47, af_logo, 0);   // af is declared in Nokia5110.c
            Nokia5110_DisplayBuffer();
            Clock_Delay1ms(60);                // delay ~60 ms
        }

        for(int count = 35; count > 0; count--) {
            // three step process: clear, build, display
            Nokia5110_ClearBuffer();
            Nokia5110_PrintBMP(count, 47, af_logo, 0);
            Nokia5110_DisplayBuffer();
            Clock_Delay1ms(60);                // delay ~60 ms
        }
    }
}


// =============== Program 5.2 =====================================

// Print characters to the Nokia 5110 48x84 LCD.
// Use this program to test
// Nokia5110_OutChar, Nokia5110_OutString, Nokia5110_OutU8Hex, Nokia5110_OutBin8
void Program5_2(void) {

    Clock_Init48MHz();  // set system clock to 48 MHz
    LaunchPad_Init();   // for LED
    Nokia5110_Init();
	
    // Contrast value 0xB1 looks good on red SparkFun.
    // Adjust this from 0xA0 (lighter) to 0xCF (darker) if necessary.
    uint8_t const contrast = 0xB1;
    Nokia5110_SetContrast(contrast);

    LEDOUT = 1;

    Clock_Delay1ms(100);    // delay 100 ms
    Nokia5110_Clear();
    Nokia5110_SetCursor2(1, 1);         // 1st row, 1st col

    // The string will automatically wrap, so padding spaces may
    // be needed to make the output look optimal.
    Nokia5110_OutString("************* LCD Test *************");

    Nokia5110_SetCursor2(4, 1);  // 4th row, 1st col
    Nokia5110_OutChar(127);     // print AF sign

    while(1) {

        for(int i = 0; i <= 32; i++) {
            Nokia5110_SetCursor2(4, 4);     // 4th row 4th column
            Nokia5110_OutChar((i%26)+'A');  // print capital letters
            Nokia5110_SetCursor2(4, 7);     // 4th row 7th column
            Nokia5110_OutChar((i%26)+'a');  // print lowercase letters

            Nokia5110_SetCursor2(4, 9);     // 4th row 9th column
            Nokia5110_OutU8Hex(i<<3);       // 8-bit hexadecimal
            Nokia5110_SetCursor2(5, 4);     // 5th row 4th column
            Nokia5110_Out8Bin(i<<3);        // 8-bit binary

            Clock_Delay1ms(1000);               // delay 1000 ms
            LEDOUT = ~LEDOUT;
        }
    }
}


// =============== Program 5.3 =====================================

// Print characters to the Nokia 5110 48x84 LCD.
// Use this program to test
// Nokia5110_OutU16Dec, Nokia5110_OutS16Dec, Nokia5110_OutU32Dec, Nokia5110_OutS32Dec

#define SIZE 13

void Program5_3(void){

    Clock_Init48MHz();  // set system clock to 48 MHz
    LaunchPad_Init();   // for LED
    Nokia5110_Init();

    // Contrast value 0xB1 looks good on red SparkFun.
    // Adjust this from 0xA0 (lighter) to 0xCF (darker) if necessary.
    uint8_t const contrast = 0xB1;
    Nokia5110_SetContrast(contrast);

    LEDOUT = 1;

    Clock_Delay1ms(100);            // delay 100 ms
    Nokia5110_Clear();
    Nokia5110_SetCursor2(1, 1);     // 1st row, 1st col
    Nokia5110_OutString("* LCD Test *");
    Nokia5110_SetCursor2(2, 1);     // 1st row, 1st col
    Nokia5110_OutString("  uint   int");

    uint8_t num8[SIZE] = {0, 0x02, 0x04, 0x10, 0x20, 0x40, 0x70, 0x7F, 0xFF, 0xF8, 0xFC, 0xF0, 0x80};
    uint16_t num16[SIZE] = {0, 0x4, 0x10, 0x100, 0x400, 0x1000, 0x4000, 0x7FFF, 0xFFFF, 0xFFF0, 0xFF00, 0xF000, 0x8000};
    uint32_t num32[SIZE] = {0, 0x4, 0x10, 0x1000, 0x100000, 0x1000000, 0x10000000, 0x7FFFFFFF, 0xFFFFFFFF, 0xFFFFFFF0, 0xFFFF0000, 0xF0000000, 0x80000000};

    while(1){
        for(int i = 0; i < SIZE; i++) {
            Nokia5110_SetCursor2(3, 4);
            Nokia5110_OutUDec(num8[i], 3);              // unsigned 8 bit
            Nokia5110_SetCursor2(3, 9);
            Nokia5110_OutSDec((int8_t)num8[i], 4);      // signed 8 bit

            Nokia5110_SetCursor2(4, 2);
            Nokia5110_OutUDec(num16[i], 5);             // unsigned 16 bit
            Nokia5110_SetCursor2(4, 7);
            Nokia5110_OutSDec((int16_t)num16[i], 6);    // signed 16 bit

            Nokia5110_SetCursor2(5, 2);
            Nokia5110_OutUDec(num32[i], 11);            // unsigned 32 bit
            Nokia5110_SetCursor2(6, 1);
            Nokia5110_OutSDec((int32_t)num32[i], 12);   // signed 32 bit

            Clock_Delay1ms(2000);                       // delay 2 s
            LEDOUT = ~LEDOUT;
        }
    }
}


void main(void){
    //Program5_1();
    Program5_2();
    //Program5_3();

}

