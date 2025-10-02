// PWM.c
// Runs on MSP432
// PWM on P2.4 using TimerA0 TA0.CCR1
// PWM on P2.5 using TimerA0 TA0.CCR2
// PWM on P2.6 using TimerA0 TA0.CCR3
// PWM on P2.7 using TimerA0 TA0.CCR4
// MCLK = SMCLK = 3MHz DCO; ACLK = 32.768kHz
// TACCR0 generates a square wave of freq ACLK/1024 =32Hz
// Derived from msp432p401_portmap_01.c in MSPware
// Jonathan Valvano
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

#include "msp.h"


//***************************PWM_Init34*******************************
// Initializes PWM outputs on P2.6 and P2.7
// Inputs: period (in 1.333µs units)
// Configures TimerA0 with SMCLK=12MHz, divide by 8, up-down mode
// P2.6: PWM signal based on TA0CCR3, P2.7: PWM signal based on TA0CCR4
// TimerA0 period: T_TA0 = T_c_TA0 * CCR0 = CCR0 * 2/3 us.
// PWM period: T_PWM = T_TA0 * 2 = CCR0 * 4/3 us.
// Duty cycle for P2.6: duty_right/period, for P2.7: duty_left/period
void PWM_Init34(uint16_t period){

    // write this as part of Lab 13
    // Initialize P2.7 and P2.6 - EN PWML/PWMR

    
    P2->SEL0 |= 0xC0;
    P2->SEL1 &= ~0xC0;
    P2->DIR |= 0xC0;
    TIMER_A0->CCTL[0] = 0x0000;		// CCI0 toggle
    TIMER_A0->CCR[0] =	period;		// T_PWM is CCR0*4/3 us, if CCR0 = 15000, T_PWM = 20 ms.
    TIMER_A0->EX0 = 0b000;		// divide by 1
    TIMER_A0->CCTL[3] |= 0x0040;		// CCR3 toggle/reset
    TIMER_A0->CCR[3] =	0;		// CCR3 duty cycle is duty1/period
    TIMER_A0->CCTL[4] |= 0x0040;			// CCR4 toggle/reset
    TIMER_A0->CCR[4] =	0;		// CCR4 duty cycle is duty2/period
    TIMER_A0->CTL =	0x02F0;			// SMCLK=12MHz, divide by 8, up-down mode


}


//***************************PWM_DutyRight*******************************
// change duty cycle of PWM output on P2.6
// Inputs:  duty_permil, duty cycle is in per 1,000 (0.1%)
// Calculates duty cycle as (duty_permil * 30) / 2
// Updates CCR3 with new duty cycle
// Outputs: none
void PWM_DutyRight(uint16_t duty_permil){

    // convert duty_permil (0 - 999) to duty in TA0 unit (0 - 14999)
    // duty = duty_permil * 15000/1000 (or 30/2)
    uint16_t duty = (duty_permil * 30) >> 1;

    // write this as part of Lab 13

    // if new duty cycle is greater than period, return immediately
    if (duty > TIMER_A0->CCR[0]) {
        return;
    }
    // assign new duty cycle to CCR[3]
    TIMER_A0->CCR[3] = duty;


}


//***************************PWM_DutyLeft*******************************
// Updates duty cycle of PWM on P2.7
// Inputs: duty_permil (duty cycle in 0.1% steps)
// Outputs: none
// Calculates duty cycle as (duty_permil * 30) / 2
// Updates CCR4 with new duty cycle
void PWM_DutyLeft(uint16_t duty_permil){

    // convert duty_permil (0 - 999) to duty in TA0 unit (0 - 14999)
    // duty = duty_permil * 15000/1000 (or 30/2)
    uint16_t duty = (duty_permil * 30) >> 1;

    // write this as part of Lab 13

    // if new duty cycle is greater than period, return immediately
    if (duty > TIMER_A0->CCR[0]) {
        return;
    }
    // assign new duty cycle to CCR[4]
    TIMER_A0->CCR[4] = duty;

}
