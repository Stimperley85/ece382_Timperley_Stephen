// Lab05_LED_Switchesmain.c
// Runs on MSP432
// Edited by Capt Beyer to include assembly implementation
// 9 Sep 2019
// Daniel and Jonathan Valvano
// February 28, 2017
//
// Updated by Stan Baek
// June 11, 2022
 
/* This example accompanies the books
   "Embedded Systems: Introduction to the MSP432 Microcontroller",
       ISBN: 978-1512185676, Jonathan Valvano, copyright (c) 2017
   "Embedded Systems: Real-Time Interfacing to the MSP432 Microcontroller",
       ISBN: 978-1514676585, Jonathan Valvano, copyright (c) 2017
   "Embedded Systems: Real-Time Operating Systems for ARM Cortex-M Microcontrollers",
       ISBN: 978-1466468863, , Jonathan Valvano, copyright (c) 2017
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/

Simplified BSD License (FreeBSD License)
Copyright (c) 2017, Jonathan Valvano, All rights reserved.

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
// P4.7 Bump6, Window2, left side of robot
// P4.6 Bump5, Window2
// P4.5 Bump4
// P4.3 Bump3
// P4.2 Bump2
// P4.0 Bump1, Activate, right side of robot


#include <stdint.h>
#include <stdbool.h>
#include "msp.h"
#include "../inc/Clock.h"
#include "../inc/Bump.h"
#include "../inc/Nokia5110.h"


// The following functions are defined in LEDs.asm
extern void LED_Oscillate(void);
extern void LED_Init(void);
extern void LED_Toggle(void);
extern void LED_On(void);
extern void LED_Off(void);


// =============== Program 8.1 =====================================

// read bump switch values on P4.7-P4.5, P4.3, P4.2, and P4.0
// Demo code working with the bump switches
void Program8_1(void){
  
    Clock_Init48MHz();  // makes bus clock 48 MHz
    Bump_Init();
    Nokia5110_Init();

    // Contrast value 0xB1 looks good on red SparkFun.
    // Adjust this from 0xA0 (lighter) to 0xCF (darker) if necessary.
    uint8_t  contrast = 0xB1;
    Nokia5110_SetContrast(contrast);
    Nokia5110_Clear();

    uint8_t bump = 0x00;

    // The string will automatically wrap, so padding spaces may
    // be needed to make the output look optimal.
    Nokia5110_OutString("Lab8: Bump");
    Nokia5110_SetCursor2(3, 1); Nokia5110_OutString("Bump: ");

    while(1) {
        bump = Bump_Read(); // read bump switches
        Nokia5110_SetCursor2(3, 7); Nokia5110_OutU8Hex(bump);
    }
}

// =============== Program 8.2 =====================================

// Toggle LED on P1.0
// Implement LED_Toggle and LED_Oscillate() within the LEDs.asm file, and demo
void Program8_2(void){
    Clock_Init48MHz();  // makes bus clock 48 MHz
    LED_Init();         // activate output for LED
    LED_Oscillate();
}


// =============== Program 8.3 =====================================
// Alarm operation:
// - Initialize ports
// - Wait 100 ms
// - Read the alarm activation switch and window switches
// - If the alarm is armed and windows are not secure, toggle the LED
// - Otherwise, keep the LED off
// P1.0 is used to control the LED (positive logic alarm indicator).
//
// Truth Table:
// - Activate       |  Window Status                  |  LED (Alarm Status)
// - Off            |  Any state                      |  LED off
// - On             |  Windows not fully closed       |  LED flashes at 5 Hz
// - On             |  Both windows closed            |  LED off
void Program8_3(void) {

    Clock_Init48MHz();  // Initialize system clock to 48 MHz
    Bump_Init();        // Initialize bump switches

    LED_Init();         // Initialize LED on P1.0
    LED_Off();          // Ensure the LED starts off

    bool isArmed;       // True if the alarm is armed, false otherwise
    uint8_t windows;    // Stores the state of the window switches
    uint8_t bump = 0x00;// Variable to hold bump switch readings

    while(1) {

        //===== write this as part of Lab 8 ======
        // Warning: Do not use hard-coded numbers. Use BUMP1 - BUMP6 defined in bump.h

        // wait for 100 ms
        Clock_Delay1ms(100);

        // Read the state of the bump switches
        bump = Bump_Read();       // update this line

        // Read the alarm activation switch (right-most bump switch, BUMP1)
        // If activated (armed), isArmed is set to true, otherwise false
        isArmed = (BUMP1 == (bump & BUMP1));    // update this line. Do not use hard-coded numbers.

        // Read the window switches (BUMP5 and BUMP6)
        // windows is set based on the state of the two left bump switches
        windows = (((bump & BUMP5)==BUMP5) && ((bump & BUMP6)==BUMP6));    // update this line. Do not use hard-coded numbers.

        // If the alarm is armed and the windows (BUMP5 and BUMP6) are not secure, toggle the LED
        // Otherwise, turn the LED off
        // Do not use hard-coded numbers.
        if ((isArmed == 1) && (windows == 0)){
            LED_Toggle();
        } else {
            LED_Off();
        }

    }
}


void main(void){
    Program8_1();
    // Program8_2();
    // Program8_3();
}
