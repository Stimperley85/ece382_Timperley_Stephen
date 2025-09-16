// Lab11_LCDmain.c
// Runs on MSP432
// Test the functions in Nokia5110.c by printing various
// things to the LCD.
// Daniel Valvano
// January 18, 2019
//
// Updated by Stan Baek
// June 11, 2022

// Font table, initialization, and other functions based
// off of Nokia_5110_Example from Spark Fun:
// 7-17-2011
// Spark Fun Electronics 2011
// Nathan Seidle
// http://dlnmh9ip6v2uc.cloudfront.net/datasheets/LCD/Monochrome/Nokia_5110_Example.pde

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

// Blue Nokia 5110
// ---------------
// Signal        (Nokia 5110) LaunchPad pin
// Reset         (RST, pin 1) connected to P9.3
// UCA3STE       (CE,  pin 2) connected to P9.4
// Data/Command  (DC,  pin 3) connected to P9.6
// UCA3SIMO      (Din, pin 4) connected to P9.7
// UCA3CLK       (Clk, pin 5) connected to P9.5
// 3.3V          (Vcc, pin 6) power
// back light    (BL,  pin 7) not connected, consists of 4 3.3 V white LEDs which draw ~80mA total
// Ground        (Gnd, pin 8) ground

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

#include "msp.h"
#include "../inc/Clock.h"
#include "../inc/LaunchPad.h"
#include "../inc/Nokia5110.h"
#include "../inc/SPIA3.h"
#include "../inc/UART0.h"


// =============== Program 11.1 =====================================
// To test the serial communication via USB
// Transmit strings to your PC.
void Program11_1(void) {

    Clock_Init48MHz();  // set system clock to 48 MHz
    LaunchPad_Init();
	
    uint32_t const baudrate = 115200;   // UART via USB
    UART0_Init(baudrate);  // Initialize UART0

    LEDOUT = 1;

    // Open Terminal in CCS

    UART0_OutString("--- ECE 382 --------\n\r");
    UART0_OutString("1234567890\n\r");
    UART0_OutString("abcdefghijklmnopqrstuvwxyz\n\r");
    UART0_OutString("ABCDEFGHIJKLMNOPQRSTUVWXYZ\n\r");
    UART0_OutString("!@#$%^&*()_+-=\n\r");
    UART0_OutString(",./<>?;'\"[]\\{}|\n\r");

    uint32_t data=0;

    while(1){
        UART0_OutString("\n\rdata = ");
        UART0_OutUDec(data);
        data = data+27;
        LEDOUT = ~LEDOUT;
        Clock_Delay1ms(1000); // delay 1 sec.
    }
}


// char Name[] = {'B', 'a', 'e', 'k', 0}; // Add NULL at the end.
// it is the same as the following.
char Name[] = "Baek";

// =============== Program 11.2 =====================================
// Transmit the first three letters of your last name
// (or first name if your last name has less than 3 letters)
// via UART0 (RX=P1.2 and TX=P1.3)
void Program11_2(void){

    Clock_Init48MHz();      // set system clock to 48 MHz
    LaunchPad_Init();

    const uint32_t baudrate = 19200;
    UART0_Init(baudrate);  // Initialize UART0

    LEDOUT = 1;

    while(1){
        UART0_OutString(Name);
        LEDOUT = ~LEDOUT;
        Clock_Delay1ms(200);  // delay 200 msec at 48 MHz
    }
}


// =============== Program 11.3 =====================================

// Transmit the first three letters of your last name
// (or first name if your last name has less than 3 letters)
// via SPI A3: P9.4 (STE), P9.5 (CLK), P9.7 (MOSI)
void Program11_3(void){

    Clock_Init48MHz();                   // set system clock to 48 MHz
    LaunchPad_Init();
    SPIA3_Init();

    LEDOUT = 1;

    while(1) {
        SPIA3_OutString(Name);
        LEDOUT = ~LEDOUT;           // Toggle LED
        Clock_Delay1ms(200);        // delay 200 ms
    }
}


void main(void){
    Program11_1();
    //Program11_2();
    //Program11_3();
}
