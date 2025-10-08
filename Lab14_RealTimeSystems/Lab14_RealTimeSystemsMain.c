// Lab14_EdgeInterruptsmain.c
// Runs on MSP432, interrupt version
// Main test program for interrupt driven bump switches the robot.
// Daniel Valvano and Jonathan Valvano
// July 11, 2019
//
// Updated by Stan Baek
// June 11, 2022
// Comments revised by Stan Baek
// October 13, 2024

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
#include "../inc/Clock.h"
#include "../inc/CortexM.h"
#include "../inc/LaunchPad.h"
#include "../inc/Motor.h"
#include "../inc/BumpInt.h"
#include "../inc/TimerA1.h"
#include "../inc/Nokia5110.h"


// ===================== Program 14.1 ==============================
// Test program for detecting button presses using interrupts.
// SW1 (P1.1) and SW2 (P1.4) are connected to interrupts.
// Each press increments the respective counter for SW1 or SW2.

uint16_t NumSW1Pressed = 0;
uint16_t NumSW2Pressed = 0;

// ===================== LCDClear1 ==============================
// Clears the Nokia5110 LCD and sets the contrast. 
// Displays the header "Program14_1" along with labels for the switch counts (SW1, SW2).
void LCDClear1(void){

    // Contrast value 0xB1 looks good on red SparkFun
    // Adjust this from 0xA0 (lighter) to 0xCF (darker) if necessary.
    uint8_t const contrast = 0xB1;
    Nokia5110_SetContrast(contrast);

    Nokia5110_Clear();      // erase entire display
    Nokia5110_OutString("Program14_1");
    Nokia5110_SetCursor2(3,1); Nokia5110_OutString("SWR:");
    Nokia5110_SetCursor2(4,1); Nokia5110_OutString("SWL:");
}

// ===================== LCDOut1 ==============================
// Updates the display to show the number of times SW1 and SW2 have been pressed.
void LCDOut1(void){
    Nokia5110_SetCursor2(3,5); Nokia5110_OutUDec(NumSW1Pressed, 5);
    Nokia5110_SetCursor2(4,5); Nokia5110_OutUDec(NumSW2Pressed, 5);
}

// ===================== PORT1_IRQHandler ==============================
// ISR for handling interrupts on Port 1 (P1.1 and P1.4).
// If SW1 (P1.1) is pressed, increment the SW1 press counter.
// If SW2 (P1.4) is pressed, increment the SW2 press counter.
void PORT1_IRQHandler(void) {   // P1 interrupt

    if (P1->IFG & 0x02) {       // interrupt from P1.1
        P1->IFG &= ~0x02;       // clear interrupt pending
        NumSW1Pressed++;
    }
    if (P1->IFG & 0x10) {       // interrupt from P1.4
        P1->IFG &= ~0x10;       // clear interrupt pending
        NumSW2Pressed++;
    }
}

// test of interrupt-driven switch interface
void Program14_1(void) {

    DisableInterrupts();
    Clock_Init48MHz(); // 48 MHz clock;

    Nokia5110_Init();

    NumSW1Pressed = 0;
    NumSW2Pressed = 0;
    LCDClear1();

    LaunchPad_Init();
    // P1.1 & P1.4 are already initialized as input in LanchPad_Init();
    // P1.1: Right Switch
    // P1.4: Left Switch
    P1->IES |= 0x12;            // falling edge event
    P1->IFG &= ~0x12;           // clear flag: no interrupt pending
    P1->IE |= 0x12;             // enable interrupt on P1.1 & P1.4
    NVIC->IP[35] = 0x80;        // priority 4: 0b10000000
    NVIC->ISER[1] = 0x00000008; // enable interrupt on P1

    EnableInterrupts();

    while(1){
        LEDOUT ^= 0x01;
        // Enters low power sleep mode waiting for interrupt
        // Processor sleeps until next hardware interrupt
        // Returns after ISR has been run
        WaitForInterrupt();
        LCDOut1();
    }
}


// ===================== Program14_2 ==============================
// Test program for detecting bump sensor collisions using interrupts.
// Bump sensors (P4.x) are edge-triggered, and the bump data is displayed on the LCD.


uint8_t BumpData;
uint16_t NumCollisions;

// ===================== LCDClear2 ==============================
// Clears the Nokia5110 LCD and sets the contrast. Displays the
// header "Program14_2" along with labels for bump data and collision count.
void LCDClear2(void) {

    // Contrast value 0xB1 looks good on red SparkFun
    // Adjust this from 0xA0 (lighter) to 0xCF (darker) if necessary.
    uint8_t const contrast = 0xB1;
    Nokia5110_SetContrast(contrast);

    Nokia5110_Clear();      // erase entire display
    Nokia5110_OutString("Program14_2");
    Nokia5110_SetCursor2(3,1); Nokia5110_OutString("Bump:");
    Nokia5110_SetCursor2(4,1); Nokia5110_OutString("nCol:");
}

// ===================== LCDOut2 ==============================
// Updates the display to show the current bump sensor data (in hex) and the
// number of collisions (in decimal, fixed length of 5).
void LCDOut2(void) {
    // Write this as part of Lab 14

    // At row 3 and col 6
    // Display the bump data in hex
    // Write your code here.
    uint8_t x = BumpInt_Read();
    Nokia5110_SetCursor2(3,6);
    Nokia5110_OutU8Hex(x);

    // At row 4 col 6
    // Display the number of collisions in decimal with a fixed length of 5.
    // Write your code here.
//    uint32_t num = 0;
//    for (int i=0;i<8;i++){
//
//        if (x & 0x01){
//            num++;
//        }
//        x = x >> 1;
//    }
//    Nokia5110_SetCursor2(4,6);
//    Nokia5110_OutChar(num+'0');
      Nokia5110_SetCursor2(4,6);
      Nokia5110_OutUDec(NumCollisions,5);
}

// ===================== Collision2 ==============================
// Callback function for handling bump sensor collisions.
// Increments the collision counter and updates the bump data based on which sensor was triggered.
void Collision2(uint8_t bumpSensor) {
    LEDOUT ^= 0x01;
    LEDOUT ^= 0x01;
	NumCollisions++;
	LEDOUT ^= 0x01;
	BumpData = bumpSensor;
}


// test of interrupt-driven bump interface
void Program14_2(void) { 

    DisableInterrupts();
    Clock_Init48MHz(); // 48 MHz clock; 12 MHz Timer A clock
    Nokia5110_Init();
    LCDClear2();
	NumCollisions = 0;
	BumpInt_Init(&Collision2);

	EnableInterrupts();

	while(1){
		BLUELED ^= 0x01;
        // Enters low power sleep mode waiting for interrupt
        // Processor sleeps until next hardware interrupt
        // Returns after ISR has been run
		WaitForInterrupt();
		LCDOut2();
	}
}


// ===================== Program14_3 ==============================
// Test program for controlling a robot with edge-triggered interrupts (bump sensors) 
// and TimerA1 for timed control of motor commands.

// Structure defining a command for controlling the robot�셲 motors.
// Contains motor duty cycles (left and right), the motor function to be called, 
// and the duration for which the command should be executed.
typedef struct command {
    uint16_t dutyLeft_permil;   // 0 to 999
    uint16_t dutyRight_permil;    // 0 to 999
    void (*MotorFunction)(uint16_t, uint16_t);
    uint32_t duration_ms;           // time to run in ms
} command_t;


// Write this as part of Lab 14
// Code four control commands
// 1. Move backward slowly for 2 second (20% duty cycle)
// 2. Turn right slowly for 1.5 seconds (30% duty cycle)
// 3. Move forward quickly for 1 second (40% duty cycle)
// 4. Turn left slowly for 1.5 seconds  (30% duty cycle)
#define NUM_STEPS   4    // update this line
const command_t Control[NUM_STEPS]={
	{200, 200, &Motor_Backward, 2000},
	{300, 300, &Motor_TurnRight, 1500},	// replace this line
	{400, 400, &Motor_Forward, 1000}, // replace this line
	{300, 300, &Motor_TurnLeft, 1500}  // replace this line
};



uint32_t ElapsedTime_ms;
uint32_t CurrentStep;   // 0, 1, 2..., NUM-1


// ===================== Controller3 ==============================
// ISR called at 1000 Hz by TimerA1 to control the robot's movement.
// This function increments the elapsed time and moves to the next command when the 
// duration for the current command is complete.
void Controller3(void) {
    // Write this as part of Lab 14
    Control[CurrentStep].MotorFunction(Control[CurrentStep].dutyLeft_permil, Control[CurrentStep].dutyRight_permil);
    // Controller should increment the timer (ElapsedTime_ms)
    ElapsedTime_ms++;
    if (ElapsedTime_ms == Control[CurrentStep].duration_ms){
        CurrentStep = (CurrentStep+1)%4;
        ElapsedTime_ms = 0;
    }

}


// ===================== Collision3 ==============================
// Callback function for handling collisions. 
// When a collision is detected, the robot stops and restarts the control sequence, 
// beginning with the backward movement command.
void Collision3(uint8_t bumpSensor) {
    // Write this as part of Lab 14
    // Note: After collision, the robot must move backward.
    CurrentStep = 0;
    ElapsedTime_ms = 0;

}

// ===================== Program14_3 ==============================
// Integrated robot control program using bump sensors for collision detection 
// and TimerA1 for executing motor control commands at a fixed rate (1000 Hz).
// Initializes the motors, bump sensors, and TimerA1, and handles collisions and command execution.
void Program14_3(void) {
    DisableInterrupts();
    Clock_Init48MHz();   // 48 MHz clock; 12 MHz Timer A clock
    LaunchPad_Init();
    Motor_Init();
	// write this as part of Lab 14, Integrated Robotic System
	// Initialize Bump with the Collision() function you wrote
    BumpInt_Init(&Collision3);
	// Initialize Timer A1 with the Controller() function you wrote at 1000 Hz
    uint16_t const period_2us = 500;	// Update this line for T = 1ms

	
	// Initialize Step to the first command
    CurrentStep = 0;
	// Run the first command
    //Control[0].MotorFunction;
    // Reset Elapsed Time
    ElapsedTime_ms = 0;

    TimerA1_Init(&Controller3, period_2us);



    // ================================================
    EnableInterrupts();
    while(1){
        // Enters low power sleep mode waiting for interrupt
        // Processor sleeps until next hardware interrupt
        WaitForInterrupt();
    }
}

int main(void){
    //Program14_1();
    //Program14_2();
    Program14_3();
}
