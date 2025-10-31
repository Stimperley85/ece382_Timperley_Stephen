// Program17_1.c
// This program is designed to run on the MSP432 microcontroller.
// It implements a proportional-integral (PI) controller to maintain a target speed for a motor.
// The system reads speed values from tachometers, adjusts motor speed via PWM, and displays information on an LCD.
//
// Author: Stan Baek
// Department of Electrical & Computer Engineering
// United States Air Force Academy
// Date: June 08, 2022
// Comments revised by Stan Baek
// October 25, 2024

/*
Simplified BSD License (FreeBSD License)
Copyright (c) 2022, Stan Baek, All rights reserved.

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


// Required libraries for microcontroller functions and peripherals
#include <stdbool.h>
#include "msp.h"                    // MSP432 microcontroller library
#include "../inc/Clock.h"           // System clock management
#include "../inc/CortexM.h"         // Cortex M specific functions
#include "../inc/PWM.h"             // PWM signal control
#include "../inc/LaunchPad.h"       // MSP432 LaunchPad functionality
#include "../inc/UART0.h"           // UART for data transmission
#include "../inc/Motor.h"           // Motor control
#include "../inc/Bump.h"            // Bump sensors
#include "../inc/TimerA2.h"         // Timer A2
#include "../inc/Nokia5110.h"       // Nokia LCD
#include "../inc/LPF.h"             // Low-pass filter
#include "../inc/Tachometer.h"      // Tachometer for motor feedback


// This macro limits a value within a specified minimum and maximum range.
// MINMAX ensures that the value of X stays within the bounds [Min, Max].
// If X is less than Min, it’s set to Min. If X is greater than Max, it’s set to Max.
// This helps prevent overflows or invalid values that could interfere with system stability.
// Usage example:
//      x = MINMAX(0, 100, x);  // This limits x to be between 0 and 100.
// Parameters:
// - Min: The minimum allowable value for X.
// - Max: The maximum allowable value for X.
// - X: The variable being constrained.
#define MINMAX(Min, Max, X) ((X) < (Min) ? (Min) : ((X) > (Max) ? (Max) : (X)))

// Constants defining PWM limits
#define PWMMIN 0        // Minimum PWM value
#define PWMMAX 999      // Maximum PWM value

// This divider scales down the controller output by dividing it inside the Controller.
// Specifically, (Kp * error) / GAIN_DIVIDER is used to compute the controller output.
// This allows us to select real (non-integer) values for Kp, such as 1.01, 2.43, and 3.75,
// which increases flexibility in tuning Kp.
// Without this scaling, we would be restricted to integer values like 1, 2, and 3.
// Note: Using a floating-point type for Kp would slow down computations significantly—over 1000 times.
// By using integers with a divider, we maintain computational efficiency while achieving precise tuning.
#define GAIN_DIVIDER    1000

// Proportional-Integral (PI) Controller Gains
// The values of Kp and Ki should be determined experimentally during tuning.
// Important: This Ki value differs from the Ki seen in lecture notes because it’s normalized:
// Ki(normalized) = Ki * DeltaT / 2, where DeltaT is the time step.
// A tuning tip: Start with Kp set to 0 and adjust Ki to help reduce any steady-state error.

// Select gain values based on experimental tuning results.
// static uint32_t Kp = 1;  // This corresponds to an actual Kp of 0.001 due to division by GainDivider.
// static uint32_t Kp = 91; // Here, Kp is effectively 0.091 (91/1000).

static int32_t Kp = 800;  // Proportional gain, scaled by GAIN_DIVIDER
static int32_t Ki = 1;  // Integral gain, scaled by GAIN_DIVIDER

// Target speed for the motors, in revolutions per minute (rpm)
static uint16_t DesiredSpeed_rpm = 80;    // 80 rpm

// =============== IMPORTANT NOTE =====================================
// Use the 'static' storage class specifier to restrict the scope to this file.
// Without a storage class specifier, objects have external linkage ('extern').
// ====================================================================

// Flags and counters for system state tracking
static bool IsControllerEnabled = false; // Do not run the controller if this semaphore is false.

// The number of times the controller has executed.
// Display some information on the LCD when this value reaches a certain threshold.
static uint8_t NumControllerExecuted = 0;   // Counts controller executions for display updates

// Buffers for storing recent speed and PWM values for performance analysis
#define TACHBUFF_SIZE 10  // Tachometer data buffer size for speed averaging
#define BUFFER_SIZE 1000  // Storage size for analysis data over a 20-second period
static uint16_t LeftSpeedBuffer[BUFFER_SIZE];
static uint16_t RightSpeedBuffer[BUFFER_SIZE];
static uint16_t LeftDutyBuffer[BUFFER_SIZE];
static uint16_t RightDutyBuffer[BUFFER_SIZE];
static uint16_t BufferIndex = 0;  // Current position in buffer arrays

// Speed-related and error-tracking variables for the PI controller
static uint16_t LeftSpeed_rpm = 0;
static uint16_t RightSpeed_rpm = 0;
static int32_t ErrorL = 0;  // Current left motor error (target - measured speed)
static int32_t ErrorR = 0;  // Current right motor error (target - measured speed)
static int32_t AccumSpeedErrorL = 0;  // Accumulated error for integral control, left motor
static int32_t AccumSpeedErrorR = 0;  // Accumulated error for integral control, right motor
static int16_t LeftDuty_permil = 0;  // PWM duty cycle, left motor
static int16_t RightDuty_permil = 0;  // PWM duty cycle, right motor

// Function to initialize and clear the LCD display with setup data
// Displays target speeds and initializes the user interface
static void LCDClear(void){

    // Contrast value 0xB1 looks good on red SparkFun.
    // Adjust this from 0xA0 (lighter) to 0xCF (darker) if necessary.
    uint8_t const contrast = 0xB1;
    Nokia5110_SetContrast(contrast);

    Nokia5110_Clear();  // erase entire display
    Nokia5110_SetCursor2(1, 1);
    Nokia5110_OutString("RPM  L   R  ");
    Nokia5110_SetCursor2(2, 1); Nokia5110_OutString("Ref");
    Nokia5110_SetCursor2(2, 5); Nokia5110_OutUDec(DesiredSpeed_rpm, 4);
    Nokia5110_SetCursor2(2, 9); Nokia5110_OutUDec(DesiredSpeed_rpm, 4);
    Nokia5110_SetCursor2(3, 1); Nokia5110_OutString("S");   // Speeds
    Nokia5110_SetCursor2(4, 1); Nokia5110_OutString("E");   // Errors
    Nokia5110_SetCursor2(5, 1); Nokia5110_OutString("A");   // Accumulated errors
    Nokia5110_SetCursor2(6, 1); Nokia5110_OutString("D");   // Duty cycles
}

// Updates LCD display with current motor speeds, control errors, and PWM duties
static void LCDOut(void){
    Nokia5110_SetCursor2(3, 2); Nokia5110_OutUDec(LeftSpeed_rpm, 5);
    Nokia5110_SetCursor2(3, 8); Nokia5110_OutUDec(RightSpeed_rpm, 5);
    Nokia5110_SetCursor2(4, 2); Nokia5110_OutSDec(ErrorL, 5);
    Nokia5110_SetCursor2(4, 8); Nokia5110_OutSDec(ErrorR, 5);
    Nokia5110_SetCursor2(5, 2); Nokia5110_OutUDec(AccumSpeedErrorL, 5);
    Nokia5110_SetCursor2(5, 8); Nokia5110_OutUDec(AccumSpeedErrorR, 5);
    Nokia5110_SetCursor2(6, 2); Nokia5110_OutUDec(LeftDuty_permil, 5);
    Nokia5110_SetCursor2(6, 8); Nokia5110_OutUDec(RightDuty_permil, 5);
}


// ------------ average ------------
// Calculates and returns the average value of an array of unsigned integers
// Used to smooth tachometer data for speed calculation
static uint16_t average(uint16_t *data, int data_length) {
    uint32_t sum = 0;
    for (int i = 0; i < data_length; i++) {
        sum += data[i];
    }
    return sum/data_length;
}


// If a bump switch is pressed and hold for more than five iterations,
// the delay will decrease, and the numbers will increment rapidly.
#define BUMP_DELAY       200       // Initial delay for bump button press in ms
#define BUMP_HOLD_DELAY  15        // Reduced delay when bump button is held

// UpdateParameters adjusts controller parameters (DesiredSpeed_rpm, Kp, and Ki) based on bump switch input.
// When the bump switches are pressed and held, values increment or decrement quickly after five iterations.
static void UpdateParameters(void) {


    uint32_t delay_ms = BUMP_DELAY;   // Delay between bump checks
    uint32_t bumpCount = 0;           // Counter for bump hold duration to enable faster increment

    // Step 1: Wait for bump or launchpad button release.
    while(LaunchPad_SW() || Bump_Read()) {
        Clock_Delay1ms(delay_ms); LaunchPad_RGB(RGB_OFF); // Red LED off
        Clock_Delay1ms(delay_ms); LaunchPad_RGB(RED);     // Red LED on
    }

    // Display initial update menu on Nokia LCD screen.
    Nokia5110_Clear();
    Nokia5110_SetCursor2(1,1); Nokia5110_OutString("Update Param");
    Nokia5110_SetCursor2(2,1); Nokia5110_OutString("RefS");   // Reference Speed
    Nokia5110_SetCursor2(2,10); Nokia5110_OutString("RPM");   // in RPM
    Nokia5110_SetCursor2(3,1); Nokia5110_OutString("B6: +, B5: -");  // Button mappings for speed
    Nokia5110_SetCursor2(6,1);  Nokia5110_OutString("SW to Cont.");  // Instruct to press SW to continue

    // Step 2: Loop until LaunchPad button SW is pressed.
    while(!LaunchPad_SW()) {
        // Update LCD with the current DesiredSpeed_rpm.
        Nokia5110_SetCursor2(2,5); Nokia5110_OutUDec(DesiredSpeed_rpm, 4);

        uint8_t bump = Bump_Read();

        if (bump == 0x20) {         // Bump 6 is pressed, increase DesiredSpeed_rpm
            DesiredSpeed_rpm++;
            bumpCount++;
        } else if (bump == 0x10) {  // Bump 5 is pressed, decrease DesiredSpeed_rpm
            DesiredSpeed_rpm = (DesiredSpeed_rpm == 0)? 0 : DesiredSpeed_rpm-1;
            bumpCount++;
        } else {
            bumpCount = 0;          // Reset bump counter if no button is pressed
        }

        // Reduce delay after five continuous bumps for rapid increment.
        delay_ms = (bumpCount > 5) ? BUMP_HOLD_DELAY : BUMP_DELAY;

        // Flash blue LED to indicate parameter update.
        BLUELED ^= 1;
        Clock_Delay1ms(delay_ms);
    }

    // Step 3: Wait for SW or Bump button release to avoid accidental inputs.
    while(LaunchPad_SW() || Bump_Read()) {
        Clock_Delay1ms(200); LaunchPad_RGB(RGB_OFF); // Red LED off
        Clock_Delay1ms(200); LaunchPad_RGB(RED);     // Red LED on
    }

    // Step 4: Set up LCD to update Kp and Ki values.
    Nokia5110_Clear();
    Nokia5110_SetCursor2(1,1); Nokia5110_OutString("Update Kp&Ki");
    Nokia5110_SetCursor2(2,1); Nokia5110_OutString("Kp:");
    Nokia5110_SetCursor2(3,1); Nokia5110_OutString("B6: +, B5: -");  // Mapping for Kp update
    Nokia5110_SetCursor2(4,1); Nokia5110_OutString("Ki:");
    Nokia5110_SetCursor2(5,1); Nokia5110_OutString("B2: +, B1: -");  // Mapping for Ki update
    Nokia5110_SetCursor2(6,1);  Nokia5110_OutString("SW to Cont.");  // Instruct to press SW to continue

    // Step 5: Loop until SW is pressed to finalize Kp and Ki update.
    while(!LaunchPad_SW()) {
        // Update Kp and Ki values on the LCD screen.
        Nokia5110_SetCursor2(2,4); Nokia5110_OutUDec(Kp, 5);
        Nokia5110_SetCursor2(4,4); Nokia5110_OutUDec(Ki, 5);
        uint8_t bump = Bump_Read();

        // Adjust Kp and Ki based on bump switch input.
        if (bump == BUMP6) {
            Kp++;
            bumpCount++;
        } else if (bump == BUMP5) {
            Kp = (Kp == 0) ? 0 : Kp-1;
            bumpCount++;
        } else if (bump == BUMP2) {
            Ki++;
            bumpCount++;
        } else if (bump == BUMP1) {
            Ki = (Ki == 0) ? 0 : Ki-1;
            bumpCount++;
        } else {
            bumpCount = 0;          // Reset if no bump is pressed
        }

        delay_ms = (bumpCount > 5) ? 25 : 200;  // Decrease delay for fast increment after 5 bumps

        // Toggle blue LED during parameter adjustment.
        BLUELED ^= 1;
        Clock_Delay1ms(delay_ms);
    }

    // Step 6: Flash yellow LED to signal completion of parameter update.
    for(int k = 0; k < 5; k++){
        LaunchPad_RGB(YELLOW);
        Clock_Delay1ms(100);
        LaunchPad_RGB(RGB_OFF);
        Clock_Delay1ms(100);
    }

    // Reset accumulated error variables to start with updated parameters.
    AccumSpeedErrorL = 0;
    AccumSpeedErrorR = 0;

}

// send analysis data to PC.
static void TxBuffer(void) {

    // wait for release
    while(LaunchPad_SW() || Bump_Read()) {
        Clock_Delay1ms(200); LaunchPad_RGB(RGB_OFF); // off
        Clock_Delay1ms(200); LaunchPad_RGB(RED); // red
    }

    uint8_t isTxEnabled = 0;

    while(1) {

        Nokia5110_Clear();
        Nokia5110_OutString("Tx Buffer?");
        Nokia5110_SetCursor2(2,1);  Nokia5110_OutString("Bump: Y/N");
        Nokia5110_SetCursor2(4,1);  Nokia5110_OutString("SW to Cont.");

        while(!LaunchPad_SW()) {    // until SW is pressed
            // update the screen
            if (isTxEnabled) {      // Display Y/N on LCD
                Nokia5110_SetCursor2(1,12);
                Nokia5110_OutString("Y");
            } else {
                Nokia5110_SetCursor2(1,12);
                Nokia5110_OutString("N");
            }

            if (Bump_Read()) {
                isTxEnabled ^= 1;   // Toggle Y/N
            }

            // flash the blue LED while desired speeds are updated.
            BLUELED ^= 1;
            Clock_Delay1ms(200);
        }

        // SW is pressed
        if(isTxEnabled) {   // Tx Buffer? is Y. Start transmitting data via USB
            UART0_OutString("\n\r***Receiving buffer data***\n\r");
            for (int i = 0; i < BUFFER_SIZE; i++) {
                UART0_OutUDec(i); UART0_OutChar(',');
                UART0_OutSDec(LeftSpeedBuffer[i]); UART0_OutChar(',');
                UART0_OutSDec(RightSpeedBuffer[i]); UART0_OutChar(',');
                UART0_OutSDec(LeftDutyBuffer[i]); UART0_OutChar(',');
                UART0_OutSDec(RightDutyBuffer[i]); UART0_OutString("\n\r");
            }
            Nokia5110_SetCursor2(4,1);
            Nokia5110_OutString("TX is Done   ");
            Nokia5110_SetCursor2(5,1);
            Nokia5110_OutString("SW to Cont.");

            while(!LaunchPad_SW());     // until SW is pressed
            Clock_Delay1ms(500);
        } else {    // Tx Buffer? is N. wrap up the function and exit.
            break;
        }
    }

    for(int k = 0; k < 5; k++){
        LaunchPad_RGB(YELLOW);
        Clock_Delay1ms(100);
        LaunchPad_RGB(RGB_OFF);
        Clock_Delay1ms(100);
    }
}


//**********************************************************
// Proportional-Integral (PI) controller for motor speed control
// Utilizes tachometer data to adjust motor speed.
static void Controller(void){


    static uint8_t idxTachoData = 0;             // Index for TachoPeriod arrays.
    static uint16_t LeftTachoPeriod[TACHBUFF_SIZE]; // Array to store left tachometer periods.
    static uint16_t RightTachoPeriod[TACHBUFF_SIZE]; // Array to store right tachometer periods.

    // If controller is disabled, exit the function.
    if(!IsControllerEnabled) {
        return;
    }

    // Check if SW button is pressed; if so, disable the controller.
    if (LaunchPad_SW()) {
        IsControllerEnabled = false;
        AccumSpeedErrorL = 0;   // Reset accumulated errors
        AccumSpeedErrorR = 0;
        return;
    }

    // If program reaches here, controller is enabled.
    // Proceed with speed control.

    // ====================================================================
    // Complete the rest for lab17
    // ====================================================================

    // Obtain tachometer data and store in arrays.
    Tachometer_GetSpeeds(&LeftTachoPeriod[idxTachoData], &RightTachoPeriod[idxTachoData]);
    // Update index in circular buffer.
    idxTachoData = (idxTachoData + 1) % TACHBUFF_SIZE;
    // Step 1: Calculate average tachometer period over last ten values.
    uint16_t leftPeriod = average(LeftTachoPeriod, TACHBUFF_SIZE);;
    uint16_t rightPeriod = average(RightTachoPeriod, TACHBUFF_SIZE);;

    // Step 2: Calculate actual motor speed (RPM) using tachometer data.
    LeftSpeed_rpm = PULSE2RPM / leftPeriod;
    RightSpeed_rpm = PULSE2RPM / rightPeriod;

    // Step 3: Calculate speed errors by comparing to desired speed.
    ErrorL =  DesiredSpeed_rpm - LeftSpeed_rpm;
    ErrorR =  DesiredSpeed_rpm - RightSpeed_rpm;

    // Step 4: Accumulate errors for integral control.
    AccumSpeedErrorL = AccumSpeedErrorL + ErrorL;
    AccumSpeedErrorR = AccumSpeedErrorR + ErrorR;

    // Step 5: Calculate control outputs (duty cycles) using PI control formula.
    LeftDuty_permil = (Kp*ErrorL)+(Ki*AccumSpeedErrorL);
    RightDuty_permil = (Kp*ErrorR)+(Ki*AccumSpeedErrorR);

    // Step 6: Ensure duty cycles are within predefined bounds, MINMAX.
    if (LeftDuty_permil > PWMMAX){
        LeftDuty_permil = PWMMAX;
    }
    if (LeftDuty_permil < PWMMIN){
        LeftDuty_permil = PWMMIN;
    }
    if (RightDuty_permil > PWMMAX){
        RightDuty_permil = PWMMAX;
    }
    if (RightDuty_permil < PWMMIN){
        RightDuty_permil = PWMMIN;
    }

	
    // ====================================================================
    // Do not modify anything below this line
    // ====================================================================

    // Step 7: Update motor speed using calculated duty cycles.
    Motor_Forward(LeftDuty_permil, RightDuty_permil);

    // Log data for performance analysis if buffer is not full.
    if (BufferIndex < BUFFER_SIZE) {
        LeftSpeedBuffer[BufferIndex] = LeftSpeed_rpm;
        RightSpeedBuffer[BufferIndex] = RightSpeed_rpm;
        LeftDutyBuffer[BufferIndex] = LeftDuty_permil;
        RightDutyBuffer[BufferIndex] = RightDuty_permil;
        BufferIndex++;
    }

    // Increment counter to keep track of controller executions.
    NumControllerExecuted++;

}


// Main program to initialize peripherals, control loop, and data transmission.
// Runs a controller at 50 Hz to adjust motor speeds and periodically updates an LCD display.
void Program17_1(void) {

    DisableInterrupts();             // Disable interrupts during initialization
    Clock_Init48MHz();               // Set the system clock to 48 MHz
    LaunchPad_Init();                // Initialize LaunchPad buttons and LEDs
    Bump_Init();                     // Initialize bump sensors
    Motor_Init();                    // Initialize motor controls
    Nokia5110_Init();                // Initialize the Nokia LCD
    LCDClear();                      // Clear the LCD screen

    Tachometer_Init(); // Initialize tachometers to measure wheel distances with set priority

    uint32_t const baudrate = 115200; // Set UART baud rate for PC communication
    UART0_Init(baudrate);             // Initialize UART0 communication with set baud rate

    // Use TimerA2 to run the controller at 50 Hz (every 20ms)
    uint16_t const period_4us = 5000;       // Timer period to achieve 20ms (5000 x 4us)
    TimerA2_Init(&Controller, period_4us);  // Initialize TimerA2 for controller

    // Variables to control display update rate
    uint16_t const LcdUpdateRate = 5; // LCD updates every 5 controller cycles (every 100ms)

    BufferIndex = 0;                  // Initialize data buffer index
    IsControllerEnabled = false;      // Initially disable the controller
    Tachometer_ResetSteps();          // Reset tachometer step counters

    EnableInterrupts();               // Enable interrupts to start program operation

    // Main program loop
    while(1) {

        // Enter low-power mode, waiting for interrupts
        WaitForInterrupt();

        // Update the LCD display every 10 Hz (5 controller runs)
        // Note: Avoid adding LCDOut inside the ISR since Nokia5110 is a slow device.
        if (NumControllerExecuted == LcdUpdateRate) {
            LCDOut();                  // Call function to output data on the LCD
            NumControllerExecuted = 0; // Reset count after LCD update
        }

        // Skip the following code if the controller is enabled, returning to low-power mode
        if (IsControllerEnabled) {
            continue;
        }

        // *************************************************************
        // If the program reaches this point, the controller is disabled.
        // *************************************************************

        LaunchPad_RGB(RGB_OFF); // Turn off RGB LED on LaunchPad
        Motor_Coast();          // Set motors to coast mode (stop gradually)
        Clock_Delay1ms(300);    // Delay to stabilize

        // Update control parameters based on user input or other settings
        UpdateParameters();

        // Transmit the buffer data to the PC for analysis
        TxBuffer();

        LCDClear();               // Clear the LCD screen
        BufferIndex = 0;          // Reset buffer index for new data collection
        AccumSpeedErrorL = 0;     // Reset accumulated speed error (left wheel)
        AccumSpeedErrorR = 0;     // Reset accumulated speed error (right wheel)

        // Enable the controller for active speed control
        IsControllerEnabled = true;

    }
}

