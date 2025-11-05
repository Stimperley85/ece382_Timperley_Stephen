// Program17_3.c
// Implements a proportional (P) controller for a wall-following robot on the MSP432 microcontroller.
// The system reads wall distances using IR sensors, adjusts motor speeds via PWM,
// and displays relevant information on an LCD screen.
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
#include "../inc/ADC14.h"           // Analog-to-digital converter
#include "../inc/TimerA1.h"         // Timer A1
#include "../inc/TimerA2.h"         // Timer A2
#include "../inc/IRDistance.h"      // Infrared distance measurement
#include "../inc/Nokia5110.h"       // Nokia LCD
#include "../inc/LPF.h"             // Low-pass filter


/************** Program17_3 ******************************************
 * Proportional controller for maintaining a set distance from walls.
*/

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

// PWM parameters for controlling motor speeds.
#define PWM_AVERAGE 350                 // Average PWM for balancing
#define SWING 300
#define PWMIN (PWM_AVERAGE - SWING)     // Minimum PWM threshold
#define PWMAX (PWM_AVERAGE + SWING)     // Maximum PWM threshold

// This divider scales down the controller output by dividing it inside the Controller.
// Specifically, (Kp * error) / GAIN_DIVIDER is used to compute the controller output.
// This allows us to select real (non-integer) values for Kp, such as 1.01, 2.43, and 3.75,
// which increases flexibility in tuning Kp.
// Without this scaling, we would be restricted to integer values like 1, 2, and 3.
// Note: Using a floating-point type for Kp would slow down computations significantly—over 1000 times.
// By using integers with a divider, we maintain computational efficiency while achieving precise tuning.
#define GAIN_DIVIDER 100

// Controller gain: experimentally determined to achieve system stability.
// static int16_t Kp = 250; // This Kp value is actually 2.5  because it will be divided by GAIN_DIVIDER.
// static int16_t Kp = 0;               // Stable Kp value of 100 (1.0 due to GAIN_DIVIDER)

// solution
static int16_t Kp = 150;


// =============== IMPORTANT NOTE =====================================
// Use the 'static' storage class specifier to restrict the scope to this file.
// Without a storage class specifier, objects have external linkage ('extern').
// ====================================================================

// Boolean flag to enable or disable the controller function.
static bool IsControllerEnabled = false;

// Flag to enable or disable motor control; useful for checking sensor readings.
static bool IsActuatorEnabled = false;

// LCD update rate: used to track number of controller executions for display.
static uint8_t NumControllerExecuted = 0;

// Data buffers for storing performance data, such as error and duty cycle values.
#define BUFFER_SIZE 1000              // Buffer for 20 seconds of data (at 50Hz controller rate)
static int32_t ErrorBuffer[BUFFER_SIZE];          // Stores controller error values
static uint16_t LeftDutyBuffer[BUFFER_SIZE];      // Stores left motor duty cycles
static uint16_t RightDutyBuffer[BUFFER_SIZE];     // Stores right motor duty cycles
static uint16_t BufferIndex = 0;                  // Index for data buffering

// IR distance variables to store readings from wall sensors (in mm).
int32_t Left, Center, Right;          // Distances to the left, center, and right walls
int32_t Error = 0;                    // Error signal for wall following


// Clears and initializes the LCD display with default text and formatting
static void LCDClear(void) {

    // Set the contrast level; 0xB1 works well on the red SparkFun display.
    // Adjust this from 0xA0 (lighter) to 0xCF (darker) if needed.
    uint8_t const contrast = 0xB1;
    Nokia5110_SetContrast(contrast);  // Apply the contrast setting
    Nokia5110_Clear();                // Clear the entire display screen

    // Display initial text for each measurement label on the LCD.
    Nokia5110_OutString("17.3:Kp");  // Display program ID and Kp label at the top
    Nokia5110_SetCursor2(2,1); Nokia5110_OutString("IR distance");

    // Set labels and units for left, center, right IR sensor values, and error.
    Nokia5110_SetCursor2(3,1); Nokia5110_OutString("L= ");
    Nokia5110_OutUDec(0, 5); Nokia5110_OutString(" mm");    // Initialize left distance
    Nokia5110_SetCursor2(4,1); Nokia5110_OutString("C= ");
    Nokia5110_OutUDec(0, 5); Nokia5110_OutString(" mm");    // Initialize center distance
    Nokia5110_SetCursor2(5,1); Nokia5110_OutString("R= ");
    Nokia5110_OutUDec(0, 5); Nokia5110_OutString(" mm");    // Initialize right distance
    Nokia5110_SetCursor2(6,1); Nokia5110_OutString("E= ");
    Nokia5110_OutUDec(0, 5); Nokia5110_OutString(" mm");    // Initialize error

}

// Updates the LCD display with real-time data values for Kp, left, center, right distances, and error.
static void LCDOut(void) {
    // Update the Kp gain value at a predefined position.
    Nokia5110_SetCursor2(1,8); Nokia5110_OutUDec(Kp, 5);

    // Display sensor distances and error in their respective positions.
    Nokia5110_SetCursor2(3,4); Nokia5110_OutSDec(Left, 6);    // Left distance
    Nokia5110_SetCursor2(4,4); Nokia5110_OutSDec(Center, 6);  // Center distance
    Nokia5110_SetCursor2(5,4); Nokia5110_OutSDec(Right, 6);   // Right distance
    Nokia5110_SetCursor2(6,4); Nokia5110_OutSDec(Error, 6);   // Error value
}


// If a bump switch is pressed and hold for more than five iterations,
// the delay will decrease, and the numbers will increment rapidly.
#define BUMP_DELAY       200       // Initial delay for bump button press in ms
#define BUMP_HOLD_DELAY  15        // Reduced delay when bump button is held

// UpdateParameters adjusts controller parameter, Kp, based on bump switch input.
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
    Nokia5110_SetCursor2(1,1); Nokia5110_OutString("Update Kp");
    Nokia5110_SetCursor2(2,1); Nokia5110_OutString("Kp:");          // Proportional controller gain
    Nokia5110_SetCursor2(3,1); Nokia5110_OutString("B6: +, B5: -"); // Button mappings for speed
    Nokia5110_SetCursor2(6,1);  Nokia5110_OutString("SW 2 Enter");  // Instruct to press SW to continue

    // Step 2: Loop until LaunchPad button SW is pressed.
    while(!LaunchPad_SW()) {
        // Update LCD with the current Kp.
        Nokia5110_SetCursor2(2,4); Nokia5110_OutUDec(Kp, 5);

        uint8_t bump = Bump_Read();

        if (bump == 0x20) {     // Bump 6 is pressed, increase Kp
            Kp++;
            bumpCount++;
        } else if (bump == 0x10) { // Bump 5 is pressed, decrease Kp
            Kp = (Kp == 0)? 0: Kp-1;
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

    // Step 3: Flash yellow LED to signal completion of parameter update.
    for(int k = 0; k < 5; k++){
        LaunchPad_RGB(YELLOW);
        Clock_Delay1ms(100);
        LaunchPad_RGB(RGB_OFF);
        Clock_Delay1ms(100);
    }
}

// Enables or disables the robot's actuator (motor) control based on user input from LaunchPad and bump sensors.
// The function interacts with the display, buttons, and LEDs to guide the user through the actuator enabling process.
static void EnableActuator(void) {

    // Wait until all buttons on the LaunchPad and bump sensors are released
    while(LaunchPad_SW() || Bump_Read()) {
        Clock_Delay1ms(200); LaunchPad_RGB(RGB_OFF);    // Turn off RGB LED
        Clock_Delay1ms(200); LaunchPad_RGB(RED);        // Flash red LED
    }

    // Prepare the display for actuator control interface
    Nokia5110_Clear();
    Nokia5110_SetCursor2(1,1); Nokia5110_OutString("Motor:");
    Nokia5110_SetCursor2(2,1); Nokia5110_OutString("Bump: On/Off");
    Nokia5110_SetCursor2(4,1);  Nokia5110_OutString("SW to Enter");

    // Loop until LaunchPad's SW button is pressed
    while(!LaunchPad_SW()) {

        // Update the screen based on the current state of the actuator
        if (IsActuatorEnabled) {
            Nokia5110_SetCursor2(1,8);
            Nokia5110_OutString("ON "); // Display actuator ON
        } else {
            Nokia5110_SetCursor2(1,8);
            Nokia5110_OutString("OFF"); // Display actuator OFF
        }

        // If a bump sensor is pressed, toggle actuator state (on/off)
        if (Bump_Read()) {
            IsActuatorEnabled ^= 1; // Toggle the state of actuator
        }

        BLUELED ^= 1;           // Toggle the blue LED to show it's active
        Clock_Delay1ms(200);    // Delay to slow LED flashing
    }

    // Finalize the setup with a yellow flash to indicate exit from setup mode
    for(int k = 0; k < 5; k++){
        LaunchPad_RGB(YELLOW);  // Flash yellow LED
        Clock_Delay1ms(100);
        LaunchPad_RGB(RGB_OFF);
        Clock_Delay1ms(100);
    }
}

// Function to send analysis data to PC via UART communication.
static void TxBuffer(void) {

    // Step 1: Wait for button release.
    // Ensures the function starts only after any active button press is released.
    while(LaunchPad_SW() || Bump_Read()) {
        Clock_Delay1ms(200); LaunchPad_RGB(RGB_OFF); // Turn off LED
        Clock_Delay1ms(200); LaunchPad_RGB(RED);     // Turn on red LED
    }

    uint8_t isTxEnabled = 0; // Flag to determine if data transmission is enabled

    while(1) {
        // Display prompt on LCD screen
        Nokia5110_Clear();
        Nokia5110_OutString("Tx Buffer?");
        Nokia5110_SetCursor2(2,1);  Nokia5110_OutString("Bump: Y/N");
        Nokia5110_SetCursor2(4,1);  Nokia5110_OutString("SW to Cont.");

        // Step 2: Wait for SW button press.
        while(!LaunchPad_SW()) {
            // Update Y/N indicator based on `isTxEnabled` status.
            Nokia5110_SetCursor2(1,12);
            Nokia5110_OutString(isTxEnabled ? "Y" : "N");

            // Toggle `isTxEnabled` if a bump switch is pressed.
            if (Bump_Read()) {
                isTxEnabled ^= 1; // Toggle between Y/N
            }

            // Flash the blue LED during screen update.
            BLUELED ^= 1;
            Clock_Delay1ms(200);
        }

        // Step 3: Transmit buffer data if `isTxEnabled` is true.
        if(isTxEnabled) {
            UART0_OutString("\n\r***Receiving buffer data***\n\r");

            // Transmit each buffer element over UART
            for (int i = 0; i < BUFFER_SIZE; i++) {
                UART0_OutUDec(i); UART0_OutChar(',');
                UART0_OutSDec(ErrorBuffer[i]); UART0_OutChar(',');
                UART0_OutSDec(LeftDutyBuffer[i]); UART0_OutChar(',');
                UART0_OutSDec(RightDutyBuffer[i]); UART0_OutString("\n\r");
            }

            // Display transmission completion message.
            Nokia5110_SetCursor2(4,1);
            Nokia5110_OutString("TX is Done   ");
            Nokia5110_SetCursor2(5,1);
            Nokia5110_OutString("SW to Cont.");

            // Wait for SW button press to exit transmission mode.
            while(!LaunchPad_SW());
            Clock_Delay1ms(500);
        } else {
            // If `isTxEnabled` is false, exit the function.
            break;
        }
    }

    // Flash the yellow LED to indicate the end of the function.
    for(int k = 0; k < 5; k++){
        LaunchPad_RGB(YELLOW);
        Clock_Delay1ms(100);
        LaunchPad_RGB(RGB_OFF);
        Clock_Delay1ms(100);
    }
}

// Periodic ADC sampling function for IR sensors.
// This function should be triggered periodically by TimerA ISR.
static void IRsampling(void){

    uint16_t raw17, raw14, raw16;               // Variables to store raw ADC values for each sensor
    ADC_In17_14_16(&raw17, &raw14, &raw16);     // Read ADC values from channels 17, 14, and 16

    uint32_t nr = LPF_Calc(raw17);              // Apply low-pass filter (LPF) to smooth right sensor data
    uint32_t nc = LPF_Calc2(raw14);             // Apply LPF to smooth center sensor data
    uint32_t nl = LPF_Calc3(raw16);             // Apply LPF to smooth left sensor data

    Left = LeftConvert(nl);                     // Convert smoothed left data to distance (or other scaled units)
    Center = CenterConvert(nc);                 // Convert smoothed center data to distance
    Right = RightConvert(nr);                   // Convert smoothed right data to distance
}


// Proportional controller function to keep the robot centered between two walls using IR sensors.
// Runs at 100 Hz (configured by TimerA ISR).
static void Controller(void){

    // If the controller is disabled, exit the function without performing any control actions
    if (!IsControllerEnabled) {
        return;
    }

    // Check if either button SW1 or SW2 is pressed on the LaunchPad.
    // If pressed, disable the controller to allow for user intervention
    if (LaunchPad_SW()) {
        IsControllerEnabled = false;
        return;
    }

    // Controller is enabled, so proceed with executing the control logic

    // ====================================================================
    // Lab 17 implementation begins here
    // ====================================================================

    // Implement a proportional controller to maintain distance from both walls


    // Calculate error as the difference between Left and Right wall distances
    Error = Left - Right;
    int16_t adjustment = (Kp*Error)/GAIN_DIVIDER;

    // Calculate the left and right motor duty cycles based on proportional control
    int16_t leftDuty_permil = PWM_AVERAGE - adjustment;   // Adjust left motor speed based on error
    int16_t rightDuty_permil = PWM_AVERAGE + adjustment;  //Adjust right motor speed based on error

    // Ensure the calculated PWM duty cycles are within the motor's operational range
    if (leftDuty_permil > PWMAX){
        leftDuty_permil = PWMAX;
    }
    if (leftDuty_permil < PWMIN){
        leftDuty_permil = PWMIN;
    }
    if (rightDuty_permil > PWMAX){
        rightDuty_permil = PWMAX;
    }
    if (rightDuty_permil < PWMIN){
        rightDuty_permil = PWMIN;
    }


    // ====================================================================
    // Do not modify anything below this line
    // ====================================================================

    // Update motor speed values based on calculated duty cycles if actuator control is enabled
    if (IsActuatorEnabled) {
        Motor_Forward(leftDuty_permil, rightDuty_permil); // Set motor speeds to maintain center position

        // If there is remaining space in the buffer, store control data for analysis
        if (BufferIndex < BUFFER_SIZE) {
            ErrorBuffer[BufferIndex] = Error;                 // Store current error value
            LeftDutyBuffer[BufferIndex] = leftDuty_permil;    // Store left motor duty cycle
            RightDutyBuffer[BufferIndex] = rightDuty_permil;  // Store right motor duty cycle
            BufferIndex++;                                    // Increment buffer index
        }
    }

    // Track the number of times the controller has executed for scheduling and logging purposes
    NumControllerExecuted++;

}


// Main program to initialize peripherals, start control loop, and handle data transmission.
// Runs a controller at 50 Hz to read wall distances using IR sensors, adjust motor speeds via PWM,
// and periodically update an LCD display.
void Program17_3(void){

    DisableInterrupts();             // Disable interrupts during initialization
    Clock_Init48MHz();               // Set the system clock to 48 MHz
    LaunchPad_Init();                // Initialize LaunchPad buttons and LEDs
    Bump_Init();                     // Initialize bump sensors
    Motor_Init();                    // Initialize motor controls
    Nokia5110_Init();                // Initialize the Nokia LCD
    LCDClear();                      // Clear the LCD screen

    uint32_t const baudrate = 115200; // Set UART baud rate for PC communication
    UART0_Init(baudrate);             // Initialize UART0 communication with set baud rate

    // Use TimerA2 to run the controller at 50 Hz (every 20ms)
    uint16_t const period_4us = 5000;       // Timer period to achieve 20ms (5000 x 4us)
    TimerA2_Init(&Controller, period_4us);  // Initialize TimerA2 for controller

    // Use TimerA1 to sample the IR sensors at 2000 Hz
    uint16_t const period_2us = 250;        // Timer period to achieve 0.5ms (250 x 2us)
    TimerA1_Init(&IRsampling, period_2us);  // Initialize TimerA1 for controller

    // Initialize ADC channels for sensors on pins 17, 14, and 16
    ADC0_InitSWTriggerCh17_14_16();
    uint16_t raw17, raw14, raw16;
    ADC_In17_14_16(&raw17, &raw14, &raw16); // Initial ADC sampling for calibration

    LPF_Init(raw17, 64);     // Initialize low-pass filter for right sensor (P9.0/channel 17)
    LPF_Init2(raw14, 64);    // Initialize LPF for center sensor (P4.1/channel 12)
    LPF_Init3(raw16, 64);    // Initialize LPF for left sensor (P9.1/channel 16)

    // Set rate for updating the LCD display: updates every 5 controller cycles (10 Hz)
    uint16_t const LcdUpdateRate = 5;

    IsControllerEnabled = false;      // Start with controller disabled
    IsActuatorEnabled = false;        // Actuator disabled at startup
    NumControllerExecuted = 0;        // Reset execution count

    EnableInterrupts();               // Enable global interrupts to start periodic tasks

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

        // Enable/Disable actuators (motors)
        EnableActuator();

        // Transmit the buffer data to the PC for analysis
        TxBuffer();

        LCDClear();               // Clear the LCD screen
        BufferIndex = 0;

        // Enable the controller for active speed control
        IsControllerEnabled = true;

    }
}
