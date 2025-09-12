// Lab10_Debugmain.c
// Runs on MSP432
// Student version to Debug lab
// Daniel and Jonathan Valvano
// September 4, 2017
// Interrupt interface for QTRX reflectance sensor array
// Pololu part number 3672.
// Debugging dump, and Flash black box recorder
// Updated by Stan Baek
// May 20, 2022

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

#include <stdbool.h>
#include "msp.h"
#include "../inc/CortexM.h"
#include "../inc/Clock.h"
#include "../inc/LaunchPad.h"
#include "../inc/TimerA2.h"
#include "../inc/Nokia5110.h"


// =============== Program 10.1 =====================================
// Flash RGB LEDs in the main function using the foreground thread.

void Program10_1(void){

    Clock_Init48MHz();  // makes bus clock 48 MHz
    LaunchPad_Init();   // buttons and LEDs

    const uint32_t blink_ms = 250; // 250 ms

    // Wait until one of the switches is pressed
    // RED and BLUE will blink at 2 Hz.
    while(LaunchPad_SW() == SW_OFF) {
        LaunchPad_RGB(RED);         // Red LED on
        Clock_Delay1ms(blink_ms);
        LaunchPad_RGB(BLUE);        // Blue LED on
        Clock_Delay1ms(blink_ms);
    }

    const uint32_t period_us = 10000;
    const uint32_t high_us = 5000;  // magenta
    const uint32_t low_us = period_us - high_us;

    // RED and BLUE will blink very fast, resulting in the LED appearing magenta.
    // Magenta: R=1, G=0, B=1

    while(1) {      // foreground thread
        LaunchPad_RGB(RED);         // Red LED on
        Clock_Delay1us(high_us);
        LaunchPad_RGB(BLUE);        // Blue LED on
        Clock_Delay1us(low_us);
    }
}


// =============== Program 10.2 =====================================
// Flash RGB LEDs using a Timer in the background

// This function is called every 1ms by TimerA2.
void Flash(void) {

    static uint16_t Time_1ms = 0;

    // Use Time_1ms to keep the LED light RED for 5 ms and BLUE for 5 ms.
    if (Time_1ms <= 5) {
        LaunchPad_RGB(RED);         // Red LED on
    } else {
        LaunchPad_RGB(BLUE);        // Blue LED on
    }

    // increment Time_1ms every time this function is executed.
    Time_1ms++;
    // if it increments to 10, roll over to 0.
    if (Time_1ms == 10){
        Time_1ms = 0;
    }
}


// Code to test background and foreground threads.
// Background: Pulse LEDs on and off.
// Foreground: Display a counter on LCD
void Program10_2(void){

    Clock_Init48MHz();
    LaunchPad_Init();
    Nokia5110_Init();
    DisableInterrupts();    // Do not start timer.

    // Contrast value 0xB1 looks good on red SparkFun
    // Adjust this from 0xA0 (lighter) to 0xCF (darker) if necessary.
    uint8_t const contrast = 0xB1;
    Nokia5110_SetContrast(contrast);

    Nokia5110_Clear();
    Nokia5110_SetCursor2(3,1); Nokia5110_OutString("SW to cont.");

    // TimerA2
    // Initialize TimerA2 to run every 1 ms
    uint16_t const period_4us = 250;        // T = 250 * 4us = 1ms
    TimerA2_Init(&Flash, period_4us);       // f = 1 kHz

    uint32_t const flash_ms = 250; // 250 ms

    // Wait until SW2 is pressed
    // RED and BLUE will blink
    while(LaunchPad_SW() != SWL_ON) {
        LaunchPad_RGB(RED);         // Red LED on
        LaunchPad_RSLK(LED_FL | LED_FR | LED_BL | LED_BR);
        Clock_Delay1ms(flash_ms);
        LaunchPad_RGB(BLUE);        // Blue LED on
        LaunchPad_RSLK(LED_OFF);
        Clock_Delay1ms(flash_ms);
    }

    Nokia5110_Clear();
    Nokia5110_OutString("Lab 10 P10_2");
    Nokia5110_SetCursor2(3,1); Nokia5110_OutString("Count:");

    uint32_t counter = 0;
    EnableInterrupts();

    // foreground
    while(1) {

        Clock_Delay1ms(200);
        Nokia5110_SetCursor2(3,7); Nokia5110_OutUDec(counter++, 5);
    }
}


// =============== Program 10.3 =====================================
// Investigate causal dependency and fix it using atomic operation

extern void IncrementInit(void);
extern int Increment(void);

void TimerTask(void) {  // called every 1ms

    static uint32_t Time_1ms = 0;

    if (Time_1ms == 1000) {
        return;
    }

    Increment();
    Time_1ms++;

}

void Program10_3(void){

    Clock_Init48MHz();
    Nokia5110_Init();
    LaunchPad_Init();
    IncrementInit();
    DisableInterrupts();    // Do not start timer.

    // Contrast value 0xB1 looks good on red SparkFun
    // Adjust this from 0xA0 (lighter) to 0xCF (darker) if necessary.
    uint8_t const contrast = 0xB1;
    Nokia5110_SetContrast(contrast);

    Nokia5110_Clear();
    Nokia5110_OutString("Lab 10 CMT");
    Nokia5110_SetCursor2(2,1); Nokia5110_OutString("Count=");

    // TimerA2
    // Initialize TimerA2 to run every 1 ms
    uint16_t const period_4us = 250;              // T = 250 * 4us = 1ms
    TimerA2_Init(&TimerTask, period_4us);

    LaunchPad_RGB(MAGENTA);
    uint16_t count;

    EnableInterrupts();     // uncomment for background thread

    for (int i = 0; i < 1000; i++) {
        //DisableInterrupts();
        count = Increment();        // uncomment for foreground thread
        //EnableInterrupts();
        Clock_Delay1us(1000);
    }

    Clock_Delay1ms(1);
    count = Increment();
    Nokia5110_SetCursor2(2,7); Nokia5110_OutUDec(count, 5);

    LaunchPad_RGB(CYAN);

    while(1) {
        // go to low power mode while waiting for the next interrupt.
        WaitForInterrupt();
    }
}


void main(void) {

    // Program10_1();
    // Program10_2();
    Program10_3();

}
