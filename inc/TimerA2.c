// TimerA2.c
// Runs on MSP432
// Use Timer A2 in periodic mode to request interrupts at a particular
// period.
// WARNING: CC3100 uses Timer A2
// Daniel Valvano
// July 5, 2017
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

#include <stdint.h>
#include "msp.h"

#define TIMER_A2_IRQ    	12
#define TIMER_A2_PRIORITY	3

void (*TimerA2Task)(void);   // user function

// ***************** TimerA2_Init ****************
// Activate Timer A2 interrupts to run user task periodically
// Inputs:  task is a pointer to a user function
//          period_4us in units (48/SMCLK), 16 bits
// Outputs: none
// With SMCLK 12 MHz, period has units 4us
void TimerA2_Init(void(*task)(void), uint16_t period_4us){

    TimerA2Task = task;             // user function

    // halt timer
    // bits5-4 = 00,       stop mode
    TIMER_A2->CTL &= ~0x0030;       // halt Timer A2

    // SMCLK, divide by 4, stop mode, clear, no interrupt
    // bits15-10,        reserved
    // bits9-8,          clock source to SMCLK
    // bits7-6,          input clock divider /8
    // bits5-4,          stop mode
    // bit3,             reserved
    // bit2,             set this bit to clear
    // bit1,             no interrupt on timer
    TIMER_A2->CTL = 0x02C0;

    // bits15-14,        no capture mode
    // bits13-12,        capture/compare input select
    // bit11,            synchronize capture source
    // bit10,            synchronized capture/compare input
    // bit9,             reserved
    // bit8,             compare mode
    // bits7-5,          output mode
    // bit4,             enable capture/compare interrupt on CCIFG
    // bit3,             read capture/compare input from here
    // bit2,             output this value in output mode 0
    // bit1,             capture overflow status
    // bit0,             clear capture/compare interrupt pending
    TIMER_A2->CCTL[0] = 0x0010;


	// compare match value
    TIMER_A2->CCR[0] = (period_4us - 1);   

	// configure for input clock divider (TAIDEX) /6
    TIMER_A2->EX0 = 0x0005;    

    // interrupts enabled in the main program after all devices initialized
    // priority 3, only upper 3 bits are used.
	// Use TIMER_A2_IRQ and TIMER_A2_PRIORITY
    NVIC->IP[TIMER_A2_IRQ] = TIMER_A2_PRIORITY << 5; 

    // enable interrupt 12 in NVIC
    NVIC->ISER[0] = 1 << TIMER_A2_IRQ;   // 0x00001000 or 1 << 12

    // reset and start Timer A2 in up mode
    // MC = Up mode
    // TACLR = reset
    // bits5-4=01,       up mode
    // bit2=1,           TACLR clear
    TIMER_A2->CTL |= 0x0014;
}


// ------------TimerA2_Stop------------
// Deactivate the interrupt running a user task periodically.
// Input: none
// Output: none
void TimerA2_Stop(void) {
    // solution
    // halt Timer A1 - MC  = stop mode
    TIMER_A2->CTL &= ~0x0030;       // halt Timer A2

    NVIC->ICER[0] = 0x00001000;     // disable interrupt 12 in NVIC
}


void TA2_0_IRQHandler(void) {

    // solution
    // acknowledge capture/compare interrupt 0 / CCIFG = no interrupt pending
    TIMER_A2->CCTL[0] &= ~0x0001;

    // execute user task
    (*TimerA2Task)();

}
