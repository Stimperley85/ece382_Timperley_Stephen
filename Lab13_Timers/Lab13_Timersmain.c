// Lab13_Timersmain.c
// Runs on MSP432
// Student starter code for Timers lab
// Daniel and Jonathan Valvano
// July 11, 2019
// PWM output to motor
// Second Periodic interrupt
//
// Updated by Stan Baek
// June 10, 2021

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
// P4.7 Bump5, left side of robot
// P4.6 Bump4
// P4.5 Bump3
// P4.3 Bump2
// P4.2 Bump1
// P4.0 Bump0, right side of robot

// Left motor direction connected to P5.4 (J3.29)
// Left motor PWM connected to P2.7/TA0CCP4 (J4.40)
// Left motor enable connected to P3.7 (J4.31)
// Right motor direction connected to P5.5 (J3.30)
// Right motor PWM connected to P2.6/TA0CCP3 (J4.39)
// Right motor enable connected to P3.6 (J2.11)

#include <stdint.h>
#include <stdbool.h>
#include "msp.h"

#include "../inc/Clock.h"
#include "../inc/CortexM.h"
#include "../inc/LaunchPad.h"
#include "../inc/Motor.h"
#include "../inc/TimerA1.h"
#include "../inc/TimerA2.h"
#include "../inc/Nokia5110.h"
#include "../inc/bump.h"


// =============== Program 13.1 =====================================
// Test TimerA1 periodic interrupt
// Use TimerA1 to blink the red LED at 5 Hz in the background
// while the blue LED blinks at 2.5 Hz in the foreground.
// and the LCD updates the elapsed time at 5 Hz.

static uint16_t Time_100ms = 0;
static bool IsNewDataDisplayed = false;    // semaphore

// This function will be called by ISR
void Task(void) {
    Time_100ms++;
    LEDOUT ^= 0x01;                 // toggle LED
    //REDLED ^= 0x01;
    IsNewDataDisplayed = false;
}

// Clear LCD
void LCDClear1(void) {
    Nokia5110_Clear();              // Erase entire display
    Nokia5110_OutString("Lab13:Timers");
    Nokia5110_SetCursor2(3,1); Nokia5110_OutString("Time:");
    Nokia5110_SetCursor2(4,8); Nokia5110_OutString("00 ms");
}

// displays Time_100ms
void LCDOut1(void) {
    Nokia5110_SetCursor2(4,3); Nokia5110_OutUDec(Time_100ms, 5);
    IsNewDataDisplayed = true;
}


// Red LED blinks at 10 Hz by periodic interrupt.
// Blue LED blinks at 5 Hz in the foreground thread
// LCD displays time elapsed at 4 Hz in the foreground thread.
void Program13_1(void) {


    Clock_Init48MHz();
    LaunchPad_Init();  // built-in switches and LEDs
    Bump_Init();
    Nokia5110_Init();

    // Contrast value 0xB1 looks good on red SparkFun
    // Adjust this from 0xA0 (lighter) to 0xCF (darker) if necessary.
    uint8_t const contrast = 0xB1;
    Nokia5110_SetContrast(contrast);

    LCDClear1();

    IsNewDataDisplayed = true;
    const uint16_t period_2us = 50000;      // T = 100,000 us -> f = 10 Hz
    TimerA1_Init(&Task, period_2us);        // periodic interrupt

    EnableInterrupts();

    while(1) {              // foreground thread

        BLUELED ^= 0x01;    // toggle Blue LED

        if (!IsNewDataDisplayed) {
            LCDOut1();           // display time elapsed on LCD
        }

        Clock_Delay1ms(200);    // delay 200 ms, 5 Hz

        if (Bump_Read()) {
            TimerA1_Stop();
        }

        if (LaunchPad_SW()) {   // Any switch is pressed
            NVIC->ISER[0] = 0x00000400;
            TIMER_A1->CTL |= 0x0014;
        }
    }
}


// =============== Program 13.2 =====================================
// Use motor functions defined in Motor.c to move the robot.
// Test PWM.c and Motors.c
// Use motor functions in Motor.c to move the robot.
// Do not modify this function.
void Program13_2(void){
    Clock_Init48MHz();
    LaunchPad_Init(); // built-in switches and LEDs
    Motor_Init();     // your function

    uint16_t leftDuty_permil = 0;   //0 %
    uint16_t rightDuty_permil = 0;  //0 %
    uint32_t const delay_ms = 2000; // 2 sec

    while(1){
		
        leftDuty_permil = 400;      // 40%
        rightDuty_permil = 400;     // 40%
        Motor_Forward(leftDuty_permil, rightDuty_permil);
        Clock_Delay1ms(delay_ms);   // run for a while and stop

        leftDuty_permil = 250;      // 25%
        rightDuty_permil = 250;     // 25%
        Motor_Backward(leftDuty_permil, rightDuty_permil);
        Clock_Delay1ms(delay_ms);   // run for a while and stop

        leftDuty_permil = 200;      // 20%
        rightDuty_permil = 200;     // 20%
        Motor_TurnLeft(leftDuty_permil, rightDuty_permil);
        Clock_Delay1ms(delay_ms);   // run for a while and stop

        leftDuty_permil = 200;      // 20%
        rightDuty_permil = 200;     // 20%
        Motor_TurnRight(leftDuty_permil, rightDuty_permil);
        Clock_Delay1ms(delay_ms);   // run for a while and stop

    }
}



// =============== Program 13.3 =====================================
// Use PWM to move the robot like Program13_2,
// but use TimerA1 to avoid blocking.
														   
static uint16_t Timer_10ms = 0;

// Use the periodic interrupt to track elapsed time.
// Transition to the next motor function occurs every 2 seconds.
// This function is called by the TimerA1 ISR, which runs at 100 Hz (every 10 ms).
void MotorController(void){


    uint16_t leftDuty_permil = 0; //0 %
    uint16_t rightDuty_permil = 0; //0 %

	// Write this for Lab 13
    if (Timer_10ms < 200){
        leftDuty_permil = 400;      // 40%
        rightDuty_permil = 400;     // 40%
        Motor_Forward(leftDuty_permil, rightDuty_permil);
        Timer_10ms++;
    } else if ((Timer_10ms >= 200) && (Timer_10ms < 400)){
        Motor_Coast();
        Timer_10ms++;

    } else if ((Timer_10ms >= 400) && (Timer_10ms < 600)){
        leftDuty_permil = 250;      // 25%
        rightDuty_permil = 250;     // 25%
        Motor_Backward(leftDuty_permil, rightDuty_permil);
        Timer_10ms++;
    } else if ((Timer_10ms >= 600) && (Timer_10ms < 800)){
        leftDuty_permil = 200;      // 20%
        rightDuty_permil = 200;     // 20%
        Motor_TurnLeft(leftDuty_permil, rightDuty_permil);
        Timer_10ms++;
    } else if ((Timer_10ms >= 800) && (Timer_10ms < 1000)){
        leftDuty_permil = 200;      // 20%
        rightDuty_permil = 200;     // 20%
        Motor_TurnRight(leftDuty_permil, rightDuty_permil);
        Timer_10ms++;
    } else if (Timer_10ms >= 1000){
        Timer_10ms = 0;
    }
}


// Write this program that uses PWM to move the robot
// like Program13_2, but uses TimerA1 to periodically
// track the time elapsed
void Program13_3(void) {

	Clock_Init48MHz();
    Nokia5110_Init();
    LaunchPad_Init();
	Bump_Init();
	Motor_Init();


    DisableInterrupts();

    // Contrast value 0xB1 looks good on red SparkFun
    // Adjust this from 0xA0 (lighter) to 0xCF (darker) if necessary.
    uint8_t const contrast = 0xB1;
    Nokia5110_SetContrast(contrast);

    Nokia5110_Clear();
    Nokia5110_OutString("Lab 13");
    Nokia5110_SetCursor2(3,1); Nokia5110_OutString("Motor:");


	// Write this for Lab 13

    // Initialize Timer A1 to run MotorController at 100 Hz
	const uint16_t period_2us = 5000     ; // T = 5,000 * 2us = 10ms --> 100 Hz
	TimerA1_Init(&MotorController, period_2us);  // 100 Hz


    // Enable Interrupts
    EnableInterrupts();

    while(1) {

        if (0 == Timer_10ms) {
            Nokia5110_SetCursor2(3,7); Nokia5110_OutString("FRWD ");
        } else if (200 == Timer_10ms) {
            Nokia5110_SetCursor2(3,7); Nokia5110_OutString("COAST");
        } else if (400 == Timer_10ms) {
            Nokia5110_SetCursor2(3,7); Nokia5110_OutString("BKWD ");
        } else if (600 == Timer_10ms) {
            Nokia5110_SetCursor2(3,7); Nokia5110_OutString("LEFT ");
        } else if (800 == Timer_10ms) {
            Nokia5110_SetCursor2(3,7); Nokia5110_OutString("RIGHT");
        }

    }

}
int main(void){
	//Program13_1();
	//Program13_2();
	Program13_3();
}
