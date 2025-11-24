#include <stdint.h>
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
static bool IsControllerEnabled = false; // Do not run the controller if this semaphore is false.


// =============== Program 16.1 =====================================
// 1. Test functions in TA3InputCapture.c and Tachometer.c.
// 2. Measure and display timer periods, motor speeds, and duty cycles.

uint16_t LeftPeriod;        // Timer period for the left motor
uint16_t RightPeriod;       // Timer period for the right motor
int16_t  LeftDistance_mm;   // Distance traveled by the left motor (in mm)
int16_t  RightDistance_mm;  // Distance traveled by the right motor (in mm)
uint16_t LeftSpeed_rpm;     // Speed of the left motor (in RPM)
uint16_t RightSpeed_rpm;    // Speed of the right motor (in RPM)

static uint16_t timer_20ms = 0;   // Timer to track elapsed time
static uint8_t bumpRead = 0x00;   // Stores the bump sensor reading

#define BUFFER_SIZE 1000    // Size of data buffers for storing measurements

uint16_t SpeedBufferL[BUFFER_SIZE];     // Buffer to store left motor speed data
uint16_t SpeedBufferR[BUFFER_SIZE];     // Buffer to store right motor speed data
int16_t XBuffer[BUFFER_SIZE];  // Buffer to store left motor distance data
int16_t YBuffer[BUFFER_SIZE];  // Buffer to store right motor distance data

static uint16_t BufferIndex = 0;  // Current position in buffer arrays


static char heading = 'N';

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
                UART0_OutSDec(XBuffer[i]); UART0_OutChar(','); // Send left wheel distance
                UART0_OutSDec(YBuffer[i]); UART0_OutString("\n\r"); // Send right wheel distance
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
//#define FRWD_DIST       700   // Replace this line for forward distance
#define BKWD_DIST       70   // Replace this line for backward distance
#define TR90_DIST      18//111   // Replace this line for 90 degree turn
#define TR90_DIST_RIGHT 12
//#define TR60_DIST       74   // Replace this line for 60 degree turn
#define TR30_DIST       1//37   // Replace this line for 30 degree turn

#define NUM_STATES      5     // Number of robot states
#define LEN_STR_STATE   5     // String length for state names

#define STOP_MARGIN     60  // How close to (0,0) should the robot stop
#define STOP_MARGIN_X   200

static char strState[NUM_STATES][LEN_STR_STATE] = {"STOP", "FRWD", "BKWD", "LFTR", "RGTR"};  // State names

static int16_t x_cord = 360;
static int16_t y_cord = 0;


// Control commands for each state
command_t ControlCommands[NUM_STATES] = {
    {0,   0,   &Motor_Stop,        0},          // Stop indefinitely
    {397, 400, &Motor_Forward,     0},  // Move forward until bump sensor triggered
    {205, 200, &Motor_Backward,    BKWD_DIST},  // Move backward for 90mm
    {150, 150, &Motor_TurnLeft,    0},          // Turn left
    {150, 150, &Motor_TurnRight,   0}           // Turn right
};

// Clear the LCD and display initial state
static void LCDClear3(void) {
    Nokia5110_Clear();                  // Clear the entire display
    Nokia5110_OutString("Lab13:Timers"); // Display the lab title
    Nokia5110_SetCursor2(3,1); Nokia5110_OutString("ST:");  // Show current state
    Nokia5110_SetCursor2(4, 1); Nokia5110_OutString("X");   // Show distance traveled
    Nokia5110_SetCursor2(5, 1); Nokia5110_OutString("Y");   // Show distance traveled
    Nokia5110_SetCursor2(6, 1); Nokia5110_OutString("T");   // Show distance traveled
    Nokia5110_SetCursor2(6, 10); Nokia5110_OutString("H");   // Show distance traveled
}

// Update the LCD with the current state and motor data
static void LCDOut3(void) {
    // Write your code here
    Nokia5110_SetCursor2(3,6); Nokia5110_OutString(strState[CurrentState]);  // Show current state
    Nokia5110_SetCursor2(4, 5); Nokia5110_OutSDec(x_cord,5);   // Show distance traveled
    Nokia5110_SetCursor2(5, 5); Nokia5110_OutSDec(y_cord,5);   // Show distance traveled
    Nokia5110_SetCursor2(6, 2); Nokia5110_OutUDec(timer_20ms/50,5);
    Nokia5110_SetCursor2(6, 12); Nokia5110_OutChar(heading);   // Show distance traveled
}

// Counter for how many times the controller function has been called
static uint8_t NumControllerExecuted = 0;  // Updated every 20ms

// Main control logic for the robot, executed by the TimerA2 ISR every 20ms
static void Controller3(void) {



    if(!IsControllerEnabled) { return; }

    // If a switch is pressed, disable the controller and exit the function
    if (LaunchPad_SW()) {
        IsControllerEnabled = false;
        return;
    }

    NumControllerExecuted++;  // Increment counter every time the controller runs

    static uint8_t color = RED;

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

            if (timer_20ms%25 == 0){
                if (color == RED){
                    color = BLUE;
                } else {
                    color = RED;
                }
            }


            LaunchPad_RGB(color);

            break;

        case Forward:  // Moving forward

            // Write your code here


            if (bumpRead & 0b00001100){
                NextState = Backward;
            }

            //Stop at 0
            if (((x_cord < STOP_MARGIN_X) && (x_cord > -STOP_MARGIN_X)) && ((y_cord < STOP_MARGIN) && (y_cord > -STOP_MARGIN))){
                NextState = Stop;
            }


            break;

        case Backward:  // Moving backward

            //Update coordinates
            if (RightDistance_mm <= -ControlCommands[CurrentState].dist_mm){
                if ((heading == 'N') && (x_cord > 0)){
                    NextState = LeftTurn;
                    ControlCommands[NextState].dist_mm = TR90_DIST;
                } else if ((heading == 'S') && (x_cord < 0)){
                    NextState = LeftTurn;
                    ControlCommands[NextState].dist_mm = TR90_DIST;
                } else if ((heading == 'E') && (y_cord < 0)){
                    NextState = LeftTurn;
                    ControlCommands[NextState].dist_mm = TR90_DIST;
                } else if ((heading == 'W') && (y_cord > 0)){
                    NextState = LeftTurn;
                    ControlCommands[NextState].dist_mm = TR90_DIST;
                } else if ((heading == 'N') && (x_cord < 0)){
                    NextState = RightTurn;
                    ControlCommands[NextState].dist_mm = TR90_DIST_RIGHT;
                } else if ((heading == 'S') && (x_cord > 0)){
                    NextState = RightTurn;
                    ControlCommands[NextState].dist_mm = TR90_DIST_RIGHT;
                } else if ((heading == 'E') && (y_cord > 0)){
                    NextState = RightTurn;
                    ControlCommands[NextState].dist_mm = TR90_DIST_RIGHT;
                } else if ((heading == 'W') && (y_cord < 0)){
                    NextState = RightTurn;
                    ControlCommands[NextState].dist_mm = TR90_DIST_RIGHT;
                }
            }

            //Stop at 0
            if (((x_cord < STOP_MARGIN_X) && (x_cord > -STOP_MARGIN_X)) && ((y_cord < STOP_MARGIN) && (y_cord > (-STOP_MARGIN)))){
                NextState = Stop;
            }


            break;

        case LeftTurn:  // Turning left

            // Write your code here
            // Set New Heading


            // Set next state
            if (RightDistance_mm >= ControlCommands[CurrentState].dist_mm) {  // Finished left turn
                NextState = Forward;  // Move forward again
                if (heading == 'N'){
                    heading = 'W';
                } else if (heading == 'W'){
                    heading = 'S';
                } else if (heading == 'S'){
                    heading = 'E';
                } else if (heading == 'E'){
                    heading = 'N';
                }
                Tachometer_ResetSteps();
            }

            //Stop at 0
            if (((x_cord < STOP_MARGIN_X) && (x_cord > -STOP_MARGIN_X)) && ((y_cord < STOP_MARGIN) && (y_cord > -STOP_MARGIN))){
                NextState = Stop;
            }

            break;

        case RightTurn:  // Turning right



            if (LeftDistance_mm >= ControlCommands[CurrentState].dist_mm) {  // Finished left turn
                NextState = Forward;  // Move forward again
                if (heading == 'N'){
                    heading = 'E';
                } else if (heading == 'E'){
                    heading = 'S';
                } else if (heading == 'S'){
                    heading = 'W';
                } else if (heading == 'W'){
                    heading = 'N';
                }
                Tachometer_ResetSteps();
            }

            //Stop at 0
            if (((x_cord < STOP_MARGIN_X) && (x_cord > -STOP_MARGIN_X)) && ((y_cord < STOP_MARGIN) && (y_cord > -STOP_MARGIN))){
                NextState = Stop;
            }
            break;

        default:
            break;
    }

    // Update the timer or reset if transitioning to a new state
    if ((CurrentState != Backward) && (CurrentState != RightTurn) && (CurrentState != LeftTurn)) {

        if (heading == 'N'){
            y_cord = y_cord + ((RightDistance_mm + LeftDistance_mm)/2);
        } else if (heading == 'S'){
            y_cord = y_cord - ((RightDistance_mm + LeftDistance_mm)/2);
        } else if (heading == 'E'){
            x_cord = x_cord + ((RightDistance_mm + LeftDistance_mm)/2);
        } else if (heading == 'W'){
            x_cord = x_cord - ((RightDistance_mm + LeftDistance_mm)/2);
        }


    }

    if ((CurrentState == Backward) && (NextState != Backward)){
        if (heading == 'N'){
            y_cord = y_cord + ((RightDistance_mm + LeftDistance_mm)/2);
        } else if (heading == 'S'){
            y_cord = y_cord - ((RightDistance_mm + LeftDistance_mm)/2);
        } else if (heading == 'E'){
            x_cord = x_cord + ((RightDistance_mm + LeftDistance_mm)/2);
        } else if (heading == 'W'){
            x_cord = x_cord - ((RightDistance_mm + LeftDistance_mm)/2);
        }
    }

    if ((CurrentState != Backward)&&(CurrentState != RightTurn) && (CurrentState != LeftTurn)){
        Tachometer_ResetSteps();
    }


    timer_20ms++;
    //    timer_20ms++;  // Stay in current state, increment timer
    //} else {
    //    timer_20ms = 0;  // New state, reset timer and distance measurements
    //    Tachometer_ResetSteps();

    // Set the current state to the next state for the next iteration
    CurrentState = NextState;

    //if(timer_20ms < 1000){  // 1000 * 10 ms = 10 seconds

        // Store the current speed, distance, and duty cycle in the respective buffers.
    if (timer_20ms%4 == 0) {
        SpeedBufferL[timer_20ms/4] = LeftSpeed_rpm;
        SpeedBufferR[timer_20ms/4] = RightSpeed_rpm;
        XBuffer[timer_20ms/4] = x_cord;
        YBuffer[timer_20ms/4] = y_cord;
    }
    //}
}

// ========== Main Program: Finite State Machine Control ==========
void Level1(void) {
    //static uint16_t BufferIndex = 0;
    // ========== Initialization Phase ==========
    DisableInterrupts();    // Disable interrupts during initialization
    Clock_Init48MHz();      // Set the system clock to 48 MHz
    LaunchPad_Init();       // Initialize the LaunchPad hardware (buttons, LEDs)
    Bump_Init();            // Initialize bump sensors
    Motor_Init();           // Initialize motor driver
    Nokia5110_Init();       // Initialize Nokia 5110 LCD display
    Tachometer_Init();      // Initialize tachometers for wheel distance measurement

    uint32_t const baudrate = 115200; // Set UART baud rate for PC communication
    UART0_Init(baudrate);             // Initialize UART0 communication with set baud rate

    // Set LCD contrast
    uint8_t const contrast = 0xA8;
    Nokia5110_SetContrast(contrast);  // Adjust LCD contrast

    // Set TimerA2 to call the Controller3() function every 20 ms (50 Hz)
    const uint16_t period_4us = 5000;       // 20 ms period
    TimerA2_Init(&Controller3, period_4us); // Initialize TimerA2

    //Tachometer_ResetSteps();  // Reset tachometer distance measurements

    // Set the LCD update rate to 10 Hz (update every 5 controller cycles)
    const uint16_t LcdUpdateRate = 5;    // 50 Hz / 5 = 10 Hz
    LCDClear3();  // Clear the LCD and display initial state

    BufferIndex = 0;
    IsControllerEnabled = false;      // Initially disable the controller
    Tachometer_ResetSteps();          // Reset tachometer step counters

    EnableInterrupts();               // Enable interrupts to start program operation


    // ========== Main Loop ==========
    while(1) {


        // Enter low-power mode, waiting for interrupts
        WaitForInterrupt();

        // Update the LCD display every 10 Hz (5 controller runs)
        // Note: Avoid adding LCDOut inside the ISR since Nokia5110 is a slow device.
        if (NumControllerExecuted == LcdUpdateRate) {
            LCDOut3();                  // Call function to output data on the LCD
            NumControllerExecuted = 0; // Reset count after LCD update
        }

        // Skip the following code if the controller is enabled, returning to low-power mode
        if (IsControllerEnabled) {
            continue;
        }

        // *************************************************************
        // If the program reaches this point, the controller is disabled.
        // *************************************************************

        //LaunchPad_RGB(RGB_OFF); // Turn off RGB LED on LaunchPad
        Motor_Coast();          // Set motors to coast mode (stop gradually)
        Clock_Delay1ms(300);    // Delay to stabilize

        // Update control parameters based on user input or other settings
        //UpdateParameters();

        // Transmit the buffer data to the PC for analysis
        TxBuffer();

        LCDClear3();               // Clear the LCD screen
        BufferIndex = 0;          // Reset buffer index for new data collection

        // Enable the controller for active speed control
        IsControllerEnabled = true;
    }
}


//void main(void){
//    //Program16_1();
//    //Program16_2();
//    Program16_3();


