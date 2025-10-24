// Lab15_ADCmain.c
// Runs on MSP432, RSLK1.1
// Test the operation of the GP2Y0A21YK0F infrared distance
// sensors by repeatedly taking measurements.  Either
// perform statistical analysis of the data, stream data
// directly from all three channels, or stream calibrated
// measurements from all three channels.  In this case, the
// results are sent through the UART to a computer running
// TExaSdisplay or another terminal program.
// Daniel Valvano
// July 11, 2019
// Updated by Stan Baek, July23, 2022

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

// 5V is connected to all three Pololu #136 GP2Y0A21YK0F Vcc's (+5V)
// ground connected to all three Pololu #136 GP2Y0A21YK0F grounds
// MSP432 P6.1/A14 (analog input A14 to MSP432) connected to center GP2Y0A21YK0F Vout
// MSP432 P9.1/A16 (analog input A16 to MSP432) connected to left GP2Y0A21YK0F Vout
// MSP432 P9.0/A17 (analog input A17 to MSP432) connected to right GP2Y0A21YK0F Vout

#include <stdint.h>
#include <stdbool.h>
#include "msp.h"					
#include "../inc/Clock.h"
#include "../inc/CortexM.h"
#include "../inc/IRDistance.h"
#include "../inc/TimerA1.h"
#include "../inc/LaunchPad.h"
#include "../inc/ADC14.h"
#include "../inc/LPF.h"
#include "../inc/Nokia5110.h"
#include "../inc/Classifier.h"

// Global variables for Lab15
uint32_t FilteredLeft, FilteredCenter, FilteredRight;
uint16_t DistLeft_mm, DistCenter_mm, DistRight_mm;

bool IsSamplingDone;    // semaphore


// =============== Program 15.1 =====================================
// Test ADC14.c and IRDistance.c

void LCDClear1(void){

    // Contrast value 0xB1 looks good on red SparkFun
    // Adjust this from 0xA0 (lighter) to 0xCF (darker) if necessary.
    uint8_t const contrast = 0xB8;
    Nokia5110_SetContrast(contrast);

    Nokia5110_Clear(); // erase entire display
    Nokia5110_SetCursor2(1,1); Nokia5110_OutString("Lab 15: ADC");
    Nokia5110_SetCursor2(2,1); Nokia5110_OutString("   LPF  dist");
    Nokia5110_SetCursor2(3,1); Nokia5110_OutString("L=");
    Nokia5110_SetCursor2(4,1); Nokia5110_OutString("C=");
    Nokia5110_SetCursor2(5,1); Nokia5110_OutString("R=");
}

// Display ADC data and calculated distances
// The LCD will display something similar to
// Lab 15: ADC
//    LPF  dist
// L= 8847  199
// C= 5846  273
// R= 2274  800
void LCDOut1(void){

    // Write this as part of Lab 15
    uint16_t raw17, raw14, raw16;
    ADC_In17_14_16(&raw17, &raw14, &raw16);         // sample
    // At row 3 display the filtered left ADC data and the left distance
    Nokia5110_SetCursor2(3,4);
    Nokia5110_OutUDec(FilteredLeft,4);
    Nokia5110_SetCursor2(3,10);
    Nokia5110_OutUDec(DistLeft_mm,3);

    // At row 4 display the filtered left ADC data and the left distance
    Nokia5110_SetCursor2(4,4);
    Nokia5110_OutUDec(FilteredCenter,4);
    Nokia5110_SetCursor2(4,10);
    Nokia5110_OutUDec(DistCenter_mm,3);


    // At row 5 display the filtered right ADC data and the right distance
    Nokia5110_SetCursor2(5,4);
    Nokia5110_OutUDec(FilteredRight,4);
    Nokia5110_SetCursor2(5,10);
    Nokia5110_OutUDec(DistRight_mm,3);
}



// ADC sampling
// Runs at 2000 Hz by TimerA1 periodic interrupt.
void AdcSampling1(void){

    uint16_t raw17, raw14, raw16;
    ADC_In17_14_16(&raw17, &raw14, &raw16);         // sample

    FilteredLeft = LPF_Calc3(raw16);                // left is channel 16, P9.1
    DistLeft_mm = LeftConvert(FilteredLeft);        // distance in mm
    FilteredCenter = LPF_Calc2(raw14);              // center is channel 14, P4.1
    DistCenter_mm = CenterConvert(FilteredCenter);  // distance in mm
    FilteredRight = LPF_Calc(raw17);                // right is channel 17 P9.0
    DistRight_mm = RightConvert(FilteredRight);     // distance in mm

    IsSamplingDone = true;     // semaphore
}


// 1. Test ADC0_InitSWTriggerCh17_14_16()
// 2. Test ADC_In17_14_16()
// 3. Find calibration coefficients for convert
// 4. Test IRDistance.c
int Program15_1(void) { // example program 15.1

    Clock_Init48MHz();
    DisableInterrupts();
    LaunchPad_Init();
    Nokia5110_Init();
    LCDClear1();

    // Initialize ADC
    uint16_t raw17, raw14, raw16;
    ADC0_InitSWTriggerCh17_14_16();   // initialize channels 17,14,16
    ADC_In17_14_16(&raw17,&raw14,&raw16);  // sample

    // Initialized Digital Low Pass Filters
    uint32_t const filterLength = 256; // replace with your choice
    LPF_Init(raw17, filterLength);     // channel 17
    LPF_Init2(raw14, filterLength);    // channel 14
    LPF_Init3(raw16, filterLength);    // channel 16

    // Initialize TimerA1 for periodic interrupt.
    const uint16_t period_2us = 250;  // T = 500us --> f = 2000Hz
    TimerA1_Init(&AdcSampling1, period_2us);

    IsSamplingDone = false;

    EnableInterrupts();

    while(1) {
        // wait until 1000 samples are captured.
        for(int n = 0; n < 1000; n++) {
            while(!IsSamplingDone); // wait until 1 data is sampled.
            IsSamplingDone = false;
        }
        // show every 1000th point
        LCDClear1();
        LCDOut1();
    }
}


// =============== Program 15.2 =====================================
// Test classify using corner cases

scenario_t Solution(int32_t Left, int32_t Center, int32_t Right);
int32_t const CornerCases[18] = {49,50,51,149,150,151,211,212,213,353,354,355,599,600,601,799,800,801};

void Program15_2(void){

    scenario_t result, truth;
    int32_t left, right, center; // sensor readings
    uint32_t errors = 0;
    for(int i = 0; i < 18; i++){
        left = CornerCases[i];
        for(int j = 0; j < 18; j++){
            center = CornerCases[j];
            for(int k=0; k < 18; k++){
                right = CornerCases[k];
                result = Classify(left, center, right); // your solution
                truth = Solution(left, center, right);  // correct answer
                if(result != truth){
                    errors++;
                }
            }
        }
    }

    while(1);  // stay here forever
}


// =============== Program 15.3 =====================================
// Test classify using all cases

// This exhaustive test will take over 16 hours to complete
void Program15_3(void) {

    scenario_t result, truth;
    uint32_t errors = 0;
    for(int32_t left = 0; left < 1000; left++){
        for(int32_t center = 0; center < 1000;  center++){
            for(int32_t right = 0; right < 1000; right++){
                result = Classify(left, center, right); // your solution
                truth = Solution(left, center, right);  // correct answer
                if(result != truth){
                    errors++;
                }
            }
        }
    }

    while(1);   // stay here forever
}


// =============== Program 15.4 =====================================
// To test Classify()

scenario_t Classification;

// string representation for LCD display
char StrScenario[][11] = {  "Error     ",
                            "L2Close   ",
                            "R2Close   ",
                            "RL2Close  ",
                            "C2Close   ",
                            "LC2Close  ",
                            "RC2Close  ",
                            "RLC2Close ",
                            "Straight  ",
                            "LeftTurn  ",
                            "RightTurn ",
                            "TeeJoint  ",
                            "LeftJoint ",
                            "RightJoint",
                            "CrossRoad ",
                            "Blocked   "};

void LCDClear4(void){

    // Contrast value 0xB1 looks good on red SparkFun
    // Adjust this from 0xA0 (lighter) to 0xCF (darker) if necessary.
    uint8_t const contrast = 0xB8;
    Nokia5110_SetContrast(contrast);

    Nokia5110_Clear(); // erase entire display
    Nokia5110_SetCursor2(1,1); Nokia5110_OutString("Lab 15: ADC");
    Nokia5110_SetCursor2(2,1); Nokia5110_OutString("   LPF  dist");
    Nokia5110_SetCursor2(3,1); Nokia5110_OutString("L=");
    Nokia5110_SetCursor2(4,1); Nokia5110_OutString("C=");
    Nokia5110_SetCursor2(5,1); Nokia5110_OutString("R=");
}

// Display ADC data and calculated distances and the classification
// The LCD will display something similar to
// Lab 15: ADC
//    LPF  dist
// L= 8847  199
// C= 5846  273
// R= 2274  800
// RightJoint
void LCDOut4(void){

    // Write this as part of Lab 15
    // Write this as part of Lab 15
    //uint16_t raw17, raw14, raw16;
    //ADC_In17_14_16(&raw17, &raw14, &raw16);         // sample
    // At row 3 display the filtered left ADC data and the left distance
    Nokia5110_SetCursor2(3,4);
    Nokia5110_OutUDec(FilteredLeft,4);
    Nokia5110_SetCursor2(3,9);
    Nokia5110_OutUDec(DistLeft_mm,3);

    // At row 4 display the filtered left ADC data and the left distance
    Nokia5110_SetCursor2(4,4);
    Nokia5110_OutUDec(FilteredCenter,4);
    Nokia5110_SetCursor2(4,9);
    Nokia5110_OutUDec(DistCenter_mm,3);


    // At row 5 display the filtered right ADC data and the right distance
    Nokia5110_SetCursor2(5,4);
    Nokia5110_OutUDec(FilteredRight,4);
    Nokia5110_SetCursor2(5,9);
    Nokia5110_OutUDec(DistRight_mm,3);

    // At row 6 display the classification.
    Nokia5110_SetCursor2(6,1);
    Nokia5110_OutString(StrScenario[Classification]);

}

// ADC sampling
// Runs at 2000 Hz by TimerA1 periodic interrupt.
void AdcSampling4(void){

    uint16_t raw17, raw14, raw16;
    ADC_In17_14_16(&raw17, &raw14, &raw16);     // sample

    FilteredLeft = LPF_Calc3(raw16);            // left is channel 16, P9.1
    DistLeft_mm = LeftConvert(FilteredLeft);       // distance in mm
    FilteredCenter = LPF_Calc2(raw14);          // center is channel 14, P4.1
    DistCenter_mm = CenterConvert(FilteredCenter); // distance in mm
    FilteredRight = LPF_Calc(raw17);            // right is channel 17 P9.0
    DistRight_mm = RightConvert(FilteredRight);    // distance in mm

    Classification = Classify(LeftConvert(FilteredLeft), CenterConvert(FilteredCenter), RightConvert(FilteredRight));

    IsSamplingDone = true;     // semaphore
}

// 1. Test ADC0_InitSWTriggerCh17_14_16()
// 2. Test ADC_In17_14_16()
// 3. Find calibration coefficients for convert
// 4. Test IRDistance.c
int Program15_4(void) { // example program 15.1

    Clock_Init48MHz();
    DisableInterrupts();
    LaunchPad_Init();
    Nokia5110_Init();
    LCDClear4();

    // Initialize ADC
    uint16_t raw17, raw14, raw16;
    ADC0_InitSWTriggerCh17_14_16();   // initialize channels 17,14,16
    ADC_In17_14_16(&raw17,&raw14,&raw16);  // sample

    // Initialized Digital Low Pass Filters
    uint32_t const filterLength = 256; // replace with your choice
    LPF_Init(raw17, filterLength);     // channel 17
    LPF_Init2(raw14, filterLength);    // channel 14
    LPF_Init3(raw16, filterLength);    // channel 16

    // Initialize TimerA1 for periodic interrupt.
    uint16_t const period_2us = 250;  // T = 500us --> f = 2000Hz
    TimerA1_Init(&AdcSampling4, period_2us);

    IsSamplingDone = false;

    EnableInterrupts();

    while(1) {
        // wait until 1000 samples are captured.
        for(int n = 0; n < 1000; n++) {
            while(!IsSamplingDone); // wait until 1 data is sampled.
            IsSamplingDone = false;
        }
        // show every 1000th point
        LCDOut4();
    }
}


int main(void){
	Program15_1();
	//Program15_2();
	//Program15_4();
}
