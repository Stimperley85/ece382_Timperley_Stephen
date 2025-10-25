// Lab16_Tachmain.c
// Runs on MSP432
// Test the operation of the tachometer by implementing
// a simple DC motor speed controller.
// Daniel Valvano
// July 11, 2019
//
// Updated by Stan Baek
// June 11, 2021
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

#include <stdbool.h>
#include "msp.h"
#include "../inc/Clock.h"
#include "../inc/CortexM.h"
#include "../inc/LaunchPad.h"
#include "../inc/Motor.h"
#include "../inc/Nokia5110.h"
#include "../inc/Tachometer.h"
#include "../inc/TimerA2.h"
#include "../inc/TA3InputCapture.h"
#include "../inc/Bump.h"
#include "../inc/UART0.h"


// Macro to constrain a value within a given range [MIN, MAX]
// If X < MIN, X is set to MIN.
// If X > MAX, X is set to MAX.
// Example:
// x = MINMAX(0, 100, x);  // x will be between 0 and 100.
#define MINMAX(Min, Max, X) ((X) < (Min) ? (Min) : ( (X) > (Max)? (Max) : (X) ) )

// variables to average tachometer readings
#define TACHBUFF_SIZE 10
uint16_t LeftTachoPeriod[TACHBUFF_SIZE];
uint16_t RightTachoPeriod[TACHBUFF_SIZE];

bool IsCollectionDone = false;

// =============== Program 16.1 =====================================
// 1. Test functions in TA3InputCapture.c and Tachometer.c.
// 2. Measure and display timer periods, motor speeds, and duty cycles.

uint16_t LeftPeriod;        // Timer period for the left motor
uint16_t RightPeriod;       // Timer period for the right motor
int16_t  LeftDistance_mm;   // Distance traveled by the left motor (in mm)
int16_t  RightDistance_mm;  // Distance traveled by the right motor (in mm)
uint16_t LeftSpeed_rpm;     // Speed of the left motor (in RPM)
uint16_t RightSpeed_rpm;    // Speed of the right motor (in RPM)

#define BUFFER_SIZE 1000    // Size of data buffers for storing measurements

uint16_t SpeedBufferL[BUFFER_SIZE];     // Buffer to store left motor speed data
uint16_t SpeedBufferR[BUFFER_SIZE];     // Buffer to store right motor speed data
uint16_t DistanceBufferL[BUFFER_SIZE];  // Buffer to store left motor distance data
uint16_t DistanceBufferR[BUFFER_SIZE];  // Buffer to store right motor distance data
uint16_t DutyBuffer[BUFFER_SIZE];       // Buffer to store duty cycle values

// Function to clear and initialize the LCD display with specific settings
void LCDClear1(void){


    // Set contrast to 0xB1 (adjustable between 0xA0 and 0xCF if necessary)
    uint8_t const contrast = 0xB1;
    Nokia5110_SetContrast(contrast);    // Apply the contrast setting

    // Clear the entire display to prepare for new output
    Nokia5110_Clear();

    // Display the title and labels for the measurements (Left "L" and Right "R")
    Nokia5110_SetCursor2(1, 1); Nokia5110_OutString("Lab16_1");  // Title: Lab16_1
    Nokia5110_SetCursor2(2, 1); Nokia5110_OutString("      L  R  ");  // Labels: L and R for left and right motor
    Nokia5110_SetCursor2(3, 1); Nokia5110_OutString("P");   // Label for periods (P)
    Nokia5110_SetCursor2(4, 1); Nokia5110_OutString("S");   // Label for speeds (S)
    Nokia5110_SetCursor2(5, 1); Nokia5110_OutString("D");   // Label for distances (D)
}

// Display Tachometer data
void LCDSpeed1(void){

    // Write this as part of Lab 16
    LCDClear1();
    Nokia5110_SetCursor2(3, 4); Nokia5110_OutUDec(LeftPeriod,4);   // Label for periods (P)
    Nokia5110_SetCursor2(3, 9); Nokia5110_OutUDec(RightPeriod,4);   // Label for periods (P)
    Nokia5110_SetCursor2(4, 4); Nokia5110_OutUDec(LeftSpeed_rpm,4);   // Label for speeds (S)
    Nokia5110_SetCursor2(4, 9); Nokia5110_OutUDec(RightSpeed_rpm,4);   // Label for speeds (S)
    Nokia5110_SetCursor2(5, 4); Nokia5110_OutUDec(LeftDistance_mm,4);   // Label for distances (D)
    Nokia5110_SetCursor2(5, 9); Nokia5110_OutUDec(RightDistance_mm,4);   // Label for distances (D)

}

// Transmit data stored in buffers to the PC via UART.
// This function asks the user whether to transmit data and responds accordingly.
// The decision is made based on the state of the left switch (SWL) to toggle Yes/No and
// the bump sensor to confirm the action. If transmission is selected, buffer data is
// sent to the PC via UART. The function exits if No is selected or after data transmission is complete.
static void TxBuffer(void) {

    // Wait until both the LaunchPad switch and bump sensors are released
    // Ensures that data transmission does not start until both input controls are inactive.
    while(LaunchPad_SW() || Bump_Read()) {
        // Flash the red LED to indicate that the program is waiting for input
        Clock_Delay1ms(200);
        LaunchPad_RGB(RGB_OFF); // LED off
        Clock_Delay1ms(200);
        LaunchPad_RGB(RED); // LED on (red)
    }

    // Variable to track whether transmission is enabled
    bool isTxEnabled = false;

    while(1) {

        // Clear the LCD and display transmission query message
        Nokia5110_Clear();
        Nokia5110_OutString("Tx Buffer?");
        Nokia5110_SetCursor2(2,1);
        Nokia5110_OutString("SWL: Y/N"); // Prompt user to press left switch for Yes/No
        Nokia5110_SetCursor2(4,1);
        Nokia5110_OutString("Bump 2 Enter"); // Instruct user to press bump sensor to confirm action

        // Loop until the bump sensor is pressed to confirm the action
        while(!Bump_Read()) {
            // Display "Y" (Yes) or "N" (No) on the LCD depending on the current state of isTxEnabled
            if (isTxEnabled) {
                Nokia5110_SetCursor2(1,12);
                Nokia5110_OutString("Y");
            } else {
                Nokia5110_SetCursor2(1,12);
                Nokia5110_OutString("N");
            }

            // If the left switch is pressed, toggle the state of isTxEnabled
            // Each press changes the selection between Yes (Y) and No (N)
            if (LaunchPad_SW() == SWL_ON) {
                isTxEnabled ^= 1;  // Toggle Yes/No selection
            }

            // Flash the blue LED to indicate that the system is ready and awaiting user input
            BLUELED ^= 1;
            Clock_Delay1ms(200); // Small delay to create a flashing effect
        }

        // If the bump sensor is pressed and transmission is enabled (Y)
        if(isTxEnabled) {
            // Send buffer data to the PC via UART
            UART0_OutString("\n\r***Receiving buffer data***\n\r");

            // Transmit data stored in Speed, Distance, and Duty buffers
            for (int i = 0; i < BUFFER_SIZE; i++) {
                UART0_OutUDec(i); UART0_OutChar(','); // Send index
                UART0_OutSDec(SpeedBufferL[i]); UART0_OutChar(','); // Send left wheel speed
                UART0_OutSDec(SpeedBufferR[i]); UART0_OutChar(','); // Send right wheel speed
                UART0_OutSDec(DistanceBufferL[i]); UART0_OutChar(','); // Send left wheel distance
                UART0_OutSDec(DistanceBufferR[i]); UART0_OutChar(','); // Send right wheel distance
                UART0_OutSDec(DutyBuffer[i]); UART0_OutString("\n\r"); // Send motor duty cycle
            }

            // Notify the user that transmission is complete
            Nokia5110_SetCursor2(4,1);
            Nokia5110_OutString("TX is Done   ");
            Nokia5110_SetCursor2(5,1);
            Nokia5110_OutString("Bump 2 Cont");

            // Wait for the bump sensor to be pressed again to confirm and exit transmission mode
            while(!Bump_Read());
            Clock_Delay1ms(500);

        } else {
            // If transmission is not enabled (No), exit the function
            break;
        }
    }

    // After transmission or exit, flash the yellow LED 5 times to indicate the function is complete
    for(int k = 0; k < 5; k++){
        LaunchPad_RGB(YELLOW);  // Flash yellow LED
        Clock_Delay1ms(100);    // Delay to create flashing effect
        LaunchPad_RGB(RGB_OFF); // Turn off LED
        Clock_Delay1ms(100);
    }
}

// ------------average------------
// Calculate the average value from an array of 16-bit unsigned integers
// Input: data is an array of 16-bit unsigned numbers
//        data_length is the number of elements in data
// Output: the average value of the data
// Note: overflow is not considered
uint16_t average(uint16_t *data, int data_length) {

    uint32_t sum = 0;
    for (int i = 0; i < data_length; i++) {
        sum += data[i];
    }
    return sum/data_length;
}


// Collect tachometer data every 10 ms (called by TimerA1 ISR)
// This function collects speed and distance data from the robot's wheels,
// adjusts the motor duty cycle at predefined intervals,
// stores the data in buffers for later analysis, and updates the LCD display.
void Collect1(void) {

    // These static variables retain their values between function calls.
    static uint16_t time_10ms = 0;      // Keeps track of elapsed time in 10 ms increments.
    static uint16_t duty_permil = 0;    // Duty cycle in permille (0 to 1000), starts at 0.
    static uint8_t idxTachoData = 0;    // Index for the tachometer data buffers (to cycle through readings).

    // ========== Step 1: Collect Tachometer Data ==========
    // Get the speed data for both left and right wheels.
    Tachometer_GetSpeeds(&LeftTachoPeriod[idxTachoData], &RightTachoPeriod[idxTachoData]);

    // Increment the index, wrapping it around when it reaches the buffer size limit (TACHBUFF_SIZE).
    idxTachoData = (idxTachoData + 1) % TACHBUFF_SIZE;

    // Get the distance data for both left and right wheels.
    Tachometer_GetDistances(&LeftDistance_mm, &RightDistance_mm);

    // ========== Step 2: Calculate Average Speed ==========
    // Average the last TACHBUFF_SIZE period values to smooth out the speed measurements.
    LeftPeriod = average(LeftTachoPeriod, TACHBUFF_SIZE);
    RightPeriod = average(RightTachoPeriod, TACHBUFF_SIZE);

    // Convert tachometer period into speed in RPM (rotations per minute).
    // PULSE2RPM is a constant defined elsewhere that converts the period into RPM.
    LeftSpeed_rpm = PULSE2RPM / LeftPeriod;
    RightSpeed_rpm = PULSE2RPM / RightPeriod;

    // ========== Step 3: Adjust Motor Duty Cycle ==========
    // Update motor duty cycle and move motors forward at different power levels based on elapsed time.
    if(time_10ms == 0) {                // At the start (0 ms), set duty cycle to 20%.
        duty_permil = 200;              // 20% duty cycle (200/1000)
        Motor_Forward(duty_permil, duty_permil);
    } else if(time_10ms == 200) {       // At 2 seconds (200 * 10 ms), set duty cycle to 40%.
        duty_permil = 400;              // 40% duty cycle
        Motor_Forward(duty_permil, duty_permil);
    } else if(time_10ms == 400) {       // At 4 seconds (400 * 10 ms), set duty cycle to 60%.
        duty_permil = 600;              // 60% duty cycle
        Motor_Forward(duty_permil, duty_permil);
    } else if(time_10ms == 600) {       // At 6 seconds (600 * 10 ms), set duty cycle to 80%.
        duty_permil = 800;              // 80% duty cycle
        Motor_Forward(duty_permil, duty_permil);
    } else if(time_10ms == 800) {       // At 8 seconds (800 * 10 ms), reset duty cycle to 20%.
        duty_permil = 200;              // 20% duty cycle
        Motor_Forward(duty_permil, duty_permil);
    }

    // ========== Step 4: Record Data ==========
    // If we are still within the 10-second data collection window, store the data in buffers.
    if(time_10ms < 1000){  // 1000 * 10 ms = 10 seconds

        // Store the current speed, distance, and duty cycle in the respective buffers.
        SpeedBufferL[time_10ms] = LeftSpeed_rpm;
        SpeedBufferR[time_10ms] = RightSpeed_rpm;
        DistanceBufferL[time_10ms] = LeftDistance_mm;
        DistanceBufferR[time_10ms] = RightDistance_mm;
        DutyBuffer[time_10ms] = duty_permil;

        // Increment the time counter (10 ms per increment).
        time_10ms++;

        // Update the LCD display with the current speed and distance data.
        LCDSpeed1();
    }

    // ========== Step 5: Heartbeat Indicator ==========
    // Every 20th iteration (every 200 ms), toggle the blue LED to indicate the system is running.
    if (time_10ms % 20 == 0) {
        BLUELED ^= 1; // Toggle the blue LED on/off.
    }

    // ========== Step 6: Stop Condition ==========
    // If we have collected data for 10 seconds or if a bump sensor is triggered, stop the motors and the timer.
    if((time_10ms == 1000) || Bump_Read()){  // Either the time hits 10 seconds or a bump is detected.
        duty_permil = 0;                     // Set duty cycle to 0 to stop the motors.
        Motor_Coast();                       // Coast the motors (stop them gently).
        IsCollectionDone = true;             // Set the collection flag to indicate data collection is complete.
        TimerA2_Stop();                      // Stop TimerA1 to stop the data collection process.
    }
}


int Program16_1(void){

    // ========== Initialization Phase ==========
    DisableInterrupts();    // Disable all interrupts during initialization to prevent unexpected behavior.
    Clock_Init48MHz();      // Set the system clock to 48 MHz for fast operation.
    LaunchPad_Init();       // Initialize the LaunchPad hardware, including switches and LEDs.
    Bump_Init();            // Initialize bump sensors for collision detection.
    Motor_Init();           // Initialize the motor driver to control the robot's motors.
    Nokia5110_Init();       // Initialize the Nokia 5110 LCD display.
    LCDClear1();            // Clear and set up the LCD display for Program 16.1.
    Tachometer_Init();      // Initialize the tachometer to measure wheel speeds and distances.

    // Initialize UART for communication over USB (baud rate = 115200 bps).
    const uint32_t baudrate = 115200;
    UART0_Init(baudrate);  // Initialize UART0 for communication with a PC.

    // Set up TimerA2 to trigger the Collect1() function every 20 ms (50 Hz frequency).
    // period_4us = 5000 corresponds to 20 ms (5000 * 4 us = 20 ms).
    const uint16_t period_4us = 5000;
    TimerA2_Init(&Collect1, period_4us); // Initialize TimerA2 with Collect1() as the ISR (Interrupt Service Routine).

    // Reset the data collection flag to false (no data collected yet).
    IsCollectionDone = false;

    // Enable interrupts to allow the system to respond to hardware events.
    EnableInterrupts();

    // ========== Main Loop ==========
    while(1) {
        // Enter low-power mode, waiting for interrupts to trigger further actions.
        // The system will "sleep" here, conserving energy, and wake up when an interrupt occurs.
        WaitForInterrupt();

        // Check if the data collection process is completed (set in the ISR).
        if (IsCollectionDone) {
            // If data collection is complete, transmit the collected data via UART to the PC.
            TxBuffer();  // Transmit the speed, distance, and duty cycle data stored in buffers.
        }
    }
}


// =============== Program 16.2 =====================================
// Develop a simple speed controller using the tachometer.
// This program uses feedback from the tachometer to control motor speed
// and adjust the PWM duty cycle dynamically, keeping the motor speed close to a desired RPM.

// Constants for desired speeds and PWM duty cycle bounds
#define DESIREDMAX 120  // Maximum desired RPM
#define DESIREDMIN 30   // Minimum desired RPM
#define MAXDUTY 999     // Maximum duty cycle (99.9%)
#define MINDUTY 10      // Minimum duty cycle (1.0%)

// Variables to control motor speed
static uint16_t DesiredL_rpm = 80;  // Desired left motor RPM, initially set to 80
static uint16_t DesiredR_rpm = 80;  // Desired right motor RPM, initially set to 80
static uint16_t ActualL_rpm = 0;    // Measured left motor RPM
static uint16_t ActualR_rpm = 0;    // Measured right motor RPM
static uint16_t LeftDuty_permil = 0;  // Left motor duty cycle in permille (1/1000)
static uint16_t RightDuty_permil = 0; // Right motor duty cycle in permille (1/1000)

// Semaphore to control whether the speed controller is enabled
bool IsControllerEnabled = false;

// ========== LCD Functions for Display ==========
void LCDClear2(void){

    // Set LCD contrast and clear the display, preparing it for output
    uint8_t const contrast = 0xB1;
    Nokia5110_SetContrast(contrast);

    Nokia5110_Clear();
    Nokia5110_SetCursor2(1, 1);
    Nokia5110_OutString("Desired(RPM)L     R     ");
    Nokia5110_SetCursor2(3, 1);
    Nokia5110_OutString("Actual (RPM)L     R     ");
    Nokia5110_SetCursor2(5, 1);
    Nokia5110_OutString("PWM (permil)L     R     ");
}


void LCDDesired2(void){
    // Display the desired motor speeds for both left and right motors
    Nokia5110_SetCursor2(2, 2);         // Desired left RPM
    Nokia5110_OutUDec(DesiredL_rpm, 5);
    Nokia5110_SetCursor2(2, 8);         // Desired right RPM
    Nokia5110_OutUDec(DesiredR_rpm, 5);
}

void LCDActual2(void){
    // Display the actual (measured) motor speeds and PWM duty cycles

    // Write your code here for the actual left RPM
    Nokia5110_SetCursor2(4, 2);

    // Write your code here for the actual right RPM
    Nokia5110_SetCursor2(4, 8);

    // Write your code here for the left duty cycle (permille)
    Nokia5110_SetCursor2(6, 1);

    // Write your code here for the right duty cycle (permille)
    Nokia5110_SetCursor2(6, 7);

}


// ========== Update the Desired Speeds ==========
static void UpdateParameters(void) {

    // Wait until any button press is released
    while(LaunchPad_SW()) {
        Clock_Delay1ms(200); LaunchPad_RGB(RGB_OFF);  // Flash off
        Clock_Delay1ms(200); LaunchPad_RGB(RED);      // Flash red LED
    }

    // Stop the motors temporarily while updating parameters
    Motor_Coast();
    Clock_Delay1ms(300);  // Delay for a smooth stop

    // Prompt the user to adjust the desired speeds using switches
    // The loop continues until a bump sensor is pressed
    while(!Bump_Read()) {

        // Display current desired speeds on the LCD
         LCDDesired2();

         // Right switch pressed: Increase desired right speed by 10 RPM, with wrap-around
         if((LaunchPad_SW() & SWR_ON)) {
            // write your code here

        }

         // Left switch pressed: Increase desired left speed by 10 RPM, with wrap-around
         if((LaunchPad_SW() & SWL_ON)){
             // write your code here

        }

         // Flash the blue LED while updating speeds to provide feedback
         BLUELED ^= 1;
         Clock_Delay1ms(200);
     }

     // After updating the desired speeds, flash the yellow LED for 2 seconds to indicate confirmation
     for(int k = 0; k < 10; k++){
         LaunchPad_RGB(YELLOW);
         Clock_Delay1ms(100);
         LaunchPad_RGB(RGB_OFF);
         Clock_Delay1ms(100);
     }
}


// This function is called by TA1 ISR every 20 ms.
void Controller2(void) {

    // Index for the tachometer data buffers
    static uint8_t idxTachoData = 0;

    // If the controller is disabled, exit the function
    if(!IsControllerEnabled) { return; }

    // If a switch is pressed, disable the controller and exit the function
    if (LaunchPad_SW()) {
        IsControllerEnabled = false;
        return;
    }

    // Collect speed data from the tachometers
    // Write your code here


    // Move to the next index in the tachometer buffer, wrapping around when necessary
    // Write your code here
    idxTachoData = 0;

    // Calculate the actual motor speeds (in RPM) using an "average filter" to reduce noise
    // omega (rpm) = PULSE2RPM / N_pulses, where N_pulses is the tachometer period.
    // Since N_pulses can be noisy, apply a simple low-pass filter (LPF) by averaging the last n readings.
    // The LeftTachPeriod and RightTachPeriod buffers store the most recent n measurements (TACHBUFF_SIZE).
    // Use the "average" of these values to reduce noise in the speed calculation.

    // Write your code here
    ActualL_rpm = 0;
    ActualR_rpm = 0;



    // Simple controller logic to adjust the left motor speed:
    // - Decrease the duty cycle if the actual speed is higher than the desired speed + 3 RPM
    // - Increase the duty cycle if the actual speed is lower than the desired speed - 3 RPM
    // Write your code here


    // Repeat the same control logic for the right motor




    // Ensure that the duty cycle stays within the defined bounds [MINDUTY, MAXDUTY]
    // Use the MINMAX macro defined at the beginning of this source file.
    // Write your code here
    LeftDuty_permil = 0;
    RightDuty_permil = 0;


    // Drive the motors forward with the updated duty cycle values
    // Write your code here

    // Update the LCD display to show the actual motor speeds and duty cycles
    // Write your code here


}


// ========== Main Program: Speed Controller ==========
void Program16_2(void){

    // ========== Initialization Phase ==========
    DisableInterrupts();           // Disable all interrupts during initialization
    Clock_Init48MHz();              // Set the system clock to 48 MHz
    LaunchPad_Init();               // Initialize the LaunchPad hardware (switches, LEDs)
    Bump_Init();                    // Initialize the bump sensors
    Nokia5110_Init();               // Initialize the Nokia 5110 LCD
    LCDClear2();                    // Clear and set up the LCD for Program 16.2
    LCDDesired2();                  // Display the initial desired speeds on the LCD
    Motor_Init();                   // Initialize the motor driver
    Tachometer_Init();              // Initialize the tachometers for speed measurements

    // Set up TimerA2 to call the Controller2() function every 20 ms (50 Hz)
    uint16_t const period_4us = 5000;       // 5000 * 4us = 20ms
    TimerA2_Init(&Controller2, period_4us); // Initialize TimerA2

    IsControllerEnabled = false;   // Start with the controller disabled

    EnableInterrupts(); // Enable interrupts to allow hardware events to be handled

    // ========== Main Loop ==========
    while(1) {

        // Enter low-power mode, waiting for the next interrupt
        WaitForInterrupt();

        // If the controller is already enabled, skip the rest and go back to low power mode
        if (IsControllerEnabled) {
            continue;
        }

        // If the controller is disabled, allow the user to update the desired speeds
        UpdateParameters();

        // Enable the controller after updating the speeds
        IsControllerEnabled = true;
    }
}


// =============== Program 16.3 =====================================
// Develop a finite state machine (FSM) to control the robot's movement.
// The robot moves forward, backward, and makes left or right turns based on bump sensor feedback.

typedef enum {
    Stop = 0,      // Stop state
    Forward,       // Move forward
    Backward,      // Move backward
    LeftTurn,      // Turn left
    RightTurn      // Turn right
} state_t;

static state_t CurrentState = Forward;  // Initial state: Forward
static state_t NextState = Forward;     // Next state variable


// Structure to define robot movement commands
typedef struct command {
    uint16_t left_permil;     // PWM duty cycle for the left wheel
    uint16_t right_permil;    // PWM duty cycle for the right wheel
    void (*MotorFunction)(uint16_t, uint16_t);  // Motor function to call (e.g., Motor_Forward)
    int32_t dist_mm;          // Wheel displacement in mm
} command_t;

// Control parameters for various states (distances and duty cycles)
#define FRWD_DIST       700   // Replace this line for forward distance
#define BKWD_DIST       90   // Replace this line for backward distance
#define TR90_DIST       111   // Replace this line for 90 degree turn
#define TR60_DIST       74   // Replace this line for 60 degree turn
#define TR30_DIST       37   // Replace this line for 30 degree turn

#define NUM_STATES      5     // Number of robot states
#define LEN_STR_STATE   5     // String length for state names
static char strState[NUM_STATES][LEN_STR_STATE] = {"STOP", "FRWD", "BKWD", "LFTR", "RGTR"};  // State names

// Control commands for each state
command_t ControlCommands[NUM_STATES] = {
    {0,   0,   &Motor_Stop,        0},          // Stop indefinitely
    {200, 200, &Motor_Forward,     FRWD_DIST},  // Move forward until bump sensor triggered
    {150, 150, &Motor_Backward,    BKWD_DIST},  // Move backward for 90mm
    {150, 150, &Motor_TurnLeft,    0},          // Turn left
    {150, 150, &Motor_TurnRight,   0}           // Turn right
};

// Clear the LCD and display initial state
static void LCDClear3(void) {
    Nokia5110_Clear();                  // Clear the entire display
    Nokia5110_OutString("Lab13:Timers"); // Display the lab title
    Nokia5110_SetCursor2(3,1); Nokia5110_OutString("ST:");  // Show current state
    Nokia5110_SetCursor2(5, 1); Nokia5110_OutString("D");   // Show distance traveled
}

// Update the LCD with the current state and motor data
static void LCDOut3(void) {
    // Write your code here
    Nokia5110_SetCursor2(3,6); Nokia5110_OutString(strState[CurrentState]);  // Show current state
    Nokia5110_SetCursor2(5, 4); Nokia5110_OutUDec(ControlCommands[CurrentState].dist_mm,5);   // Show distance traveled

}

// Counter for how many times the controller function has been called
static uint8_t NumControllerExecuted = 0;  // Updated every 20ms

// Main control logic for the robot, executed by the TimerA2 ISR every 20ms
static void Controller3(void) {

    static uint16_t timer_20ms = 0;   // Timer to track elapsed time
    static uint8_t bumpRead = 0x00;   // Stores the bump sensor reading

    NumControllerExecuted++;  // Increment counter every time the controller runs

    // Get the PWM duty cycles for the current state
    uint16_t left_permil = ControlCommands[CurrentState].left_permil;
    uint16_t right_permil = ControlCommands[CurrentState].right_permil;

    // FSM Output: Execute the motor command for the current state
    // Write your code here
    ControlCommands[CurrentState].MotorFunction(left_permil, right_permil);

    // State transition logic based on bump sensors and distance
    bumpRead = Bump_Read();  // Read bump sensor status
    Tachometer_GetDistances(&LeftDistance_mm, &RightDistance_mm);  // Get current wheel distances

    switch (CurrentState) {

        case Stop:  // Remain in Stop state indefinitely
                break;

        case Forward:  // Moving forward



            // Write your code here
            if (RightDistance_mm >= ControlCommands[CurrentState].dist_mm){
                NextState = Stop;
            }
            if ((bumpRead & 0b00001000) || (bumpRead & 0b00000100)){
                NextState = Backward;
            }
            if (bumpRead & 0b00000001){
                NextState = LeftTurn;
                ControlCommands[NextState].dist_mm = TR30_DIST;
            }
            if (bumpRead & 0b00000010){
                NextState = LeftTurn;
                ControlCommands[NextState].dist_mm = TR60_DIST;
            }
            if (bumpRead & 0b00100000){
                NextState = RightTurn;
                ControlCommands[NextState].dist_mm = TR30_DIST;
            }
            if (bumpRead & 0b00010000){
                NextState = RightTurn;
                ControlCommands[NextState].dist_mm = TR60_DIST;
            }
            break;

        case Backward:  // Moving backward

            // Write your code here
            if (RightDistance_mm <= -ControlCommands[CurrentState].dist_mm) {  // Finished backing up
                NextState = LeftTurn;  // Move left again
            }
            break;

        case LeftTurn:  // Turning left

            // Write your code here
            if (RightDistance_mm >= ControlCommands[CurrentState].dist_mm) {  // Finished left turn
                NextState = Forward;  // Move forward again
            }

            break;

        case RightTurn:  // Turning right
            if (LeftDistance_mm >= ControlCommands[CurrentState].dist_mm) {  // Finished right turn
                NextState = Forward;  // Move forward again
            }
            break;

        default:
            break;
    }

    // Update the timer or reset if transitioning to a new state
    if (CurrentState == NextState) {
        timer_20ms++;  // Stay in current state, increment timer
    } else {
        timer_20ms = 0;  // New state, reset timer and distance measurements
        Tachometer_ResetSteps();
    }

    // Set the current state to the next state for the next iteration
    CurrentState = NextState;
}

// ========== Main Program: Finite State Machine Control ==========
void Program16_3(void) {

    // ========== Initialization Phase ==========
    DisableInterrupts();    // Disable interrupts during initialization
    Clock_Init48MHz();      // Set the system clock to 48 MHz
    LaunchPad_Init();       // Initialize the LaunchPad hardware (buttons, LEDs)
    Bump_Init();            // Initialize bump sensors
    Motor_Init();           // Initialize motor driver
    Nokia5110_Init();       // Initialize Nokia 5110 LCD display
    Tachometer_Init();      // Initialize tachometers for wheel distance measurement

    // Set LCD contrast
    uint8_t const contrast = 0xA8;
    Nokia5110_SetContrast(contrast);  // Adjust LCD contrast

    // Set TimerA2 to call the Controller3() function every 20 ms (50 Hz)
    const uint16_t period_4us = 5000;       // 20 ms period
    TimerA2_Init(&Controller3, period_4us); // Initialize TimerA2

    Tachometer_ResetSteps();  // Reset tachometer distance measurements

    // Set the LCD update rate to 10 Hz (update every 5 controller cycles)
    const uint16_t LcdUpdateRate = 5;    // 50 Hz / 5 = 10 Hz
    LCDClear3();  // Clear the LCD and display initial state

    EnableInterrupts();  // Enable interrupts

    // ========== Main Loop ==========
    while(1) {

        // Enter low-power mode while waiting for the next interrupt
        WaitForInterrupt();

        // Update the LCD every 10 Hz
        if (NumControllerExecuted == LcdUpdateRate) {
            LCDOut3();  // Output current state and motor data to the LCD
            NumControllerExecuted = 0;  // Reset the execution count
        }
    }
}


void main(void){
    //Program16_1();
    //Program16_2();
    Program16_3();
}
