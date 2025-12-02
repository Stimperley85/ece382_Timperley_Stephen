
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
#include "../inc/Classifier.h"      // Classifier


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
#define PWM_AVERAGE 800      // Average PWM for balancing
#define SWING 300   //Define max swing in PWM
#define PWMIN (PWM_AVERAGE - SWING)     // Minimum PWM threshold
#define PWMAX 1000 //(PWM_AVERAGE + SWING)     // Maximum PWM threshold


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
static int16_t Kp = 45;//60;//45;  //Initial Kp for the robot


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
//static int32_t ErrorBuffer[BUFFER_SIZE];          // Stores controller error values
static uint16_t TimerGO[BUFFER_SIZE];
static uint16_t LeftDutyBuffer[BUFFER_SIZE];      // Stores left motor duty cycles
static uint16_t RightDutyBuffer[BUFFER_SIZE];     // Stores right motor duty cycles
static uint16_t BufferIndex = 0;                  // Index for data buffering

// IR distance variables to store readings from wall sensors (in mm).
int32_t Left, Center, Right;          // Distances to the left, center, and right walls
int32_t Error = 0;                    // Error signal for wall following

typedef enum {
    Stop = 0,      // Stop state
    ForwardRight,       // Move forward
    LeftTurns,
    RightTurns,
    ForwardLeft

} state_t;

static scenario_t turn = Straight;

static state_t CurrentState = ForwardRight;


static uint16_t timer_10msGO = 0;

static uint8_t color = RED;

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

char StrState2[][8] = {     "Stop",      // Stop state
                            "FR  ",       // Move forward
                            "LT  ",
                            "RT  ",
                            "FL  "};







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
    Nokia5110_SetCursor2(3,1); Nokia5110_OutString("T=");
    //Nokia5110_OutUDec(0, 5); Nokia5110_OutString(" mm");    // Initialize left distance
    Nokia5110_SetCursor2(4,1); Nokia5110_OutString("C= ");
    //Nokia5110_OutUDec(0, 5); Nokia5110_OutString(" mm");    // Initialize center distance
    //Nokia5110_SetCursor2(5,1); Nokia5110_OutString("R= ");
    //Nokia5110_OutUDec(0, 5); Nokia5110_OutString(" mm");    // Initialize right distance
    //Nokia5110_SetCursor2(6,1); Nokia5110_OutString("E= ");
    //Nokia5110_OutUDec(0, 5); Nokia5110_OutString(" mm");    // Initialize error

}



// Updates the LCD display with real-time data values for Kp, left, center, right distances, and error.
static void LCDOut(void) {
    // Update the Kp gain value at a predefined position.
    Nokia5110_SetCursor2(1,8); Nokia5110_OutUDec(Kp, 5);

    // Display sensor distances and error in their respective positions.
    Nokia5110_SetCursor2(3,4); Nokia5110_OutSDec(timer_10msGO, 6);    // Left distance
    Nokia5110_SetCursor2(4,4); Nokia5110_OutSDec(Right, 6);  // Center distance
    Nokia5110_SetCursor2(5,1); Nokia5110_OutString(StrState2[CurrentState]);   // Right distance
    Nokia5110_SetCursor2(6,1); Nokia5110_OutString(StrScenario[turn]);   // Error value
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
                UART0_OutSDec(TimerGO[i]); UART0_OutChar(',');
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







static uint16_t timerCheck = 0;

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

    static state_t NextState = ForwardRight;
    static int16_t leftDuty_permil = 0;
    static int16_t rightDuty_permil = 0;
    turn = Classify(Left, Center, Right);
    static int16_t timer_10msSTOP = 0;
    static int16_t adjustment = 0;
    const int16_t turnTimer1 = 200;//200;
    const int16_t turnTimer2 = 750;
    const int16_t rightTurnTimer = 80;
    const int16_t rightCenterPosition = 200;
    const int16_t leftCenterPosition = 200;
    const float insideTurnOffset = 0.46873;///0.46873; // 0.69 * (161/237)
    const float outsideTurnOffset = 0.76;//0.76;
    //int16_t adjustment = 0;
    //int16_t timeExtra = 0;

    switch (CurrentState) {
        case ForwardRight:
            // Controller is enabled, so proceed with executing the control logic

            // ====================================================================
            // Lab 17 implementation begins here
            // ====================================================================

            // Implement a proportional controller to maintain distance from both walls


            // Calculate error as the difference between Left and Right wall distances
            if ((timer_10msGO < turnTimer2) && (timer_10msGO > turnTimer1)){
                if ((turn == LeftTurn)||(turn == TeeJoint)){
                    NextState = LeftTurns;
                }
            }
            if (timer_10msGO >= turnTimer2){
                if ((turn == LeftTurn)||(turn == TeeJoint)||(turn == RightTurn)){
                    NextState = RightTurns;
                    timerCheck = timer_10msGO;
                }
            }

            Error = (rightCenterPosition - Right);
            adjustment = (Kp*Error)/GAIN_DIVIDER;

            // Calculate the left and right motor duty cycles based on proportional control
            leftDuty_permil = PWM_AVERAGE - adjustment;   // Adjust left motor speed based on error
            rightDuty_permil = PWM_AVERAGE + adjustment;  //Adjust right motor speed based on error

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

            timer_10msGO++;

            break;

        case LeftTurns:

            if (turn == Straight){
                NextState = ForwardRight;
            }
            leftDuty_permil = insideTurnOffset*(PWM_AVERAGE);// 161/237
            rightDuty_permil = outsideTurnOffset*PWM_AVERAGE;
            timer_10msGO++;
            break;

        case RightTurns:

            if ((turn == Straight)||((timer_10msGO-timerCheck)>rightTurnTimer)){
                NextState = ForwardLeft;
            }
            rightDuty_permil = insideTurnOffset*(PWM_AVERAGE);//163
            leftDuty_permil = outsideTurnOffset*PWM_AVERAGE;
            timer_10msGO++;
            break;


        case ForwardLeft:

            if (turn == Blocked){
                NextState = Stop;
            }

            Error = -(leftCenterPosition - Left);
            adjustment = (Kp*Error)/GAIN_DIVIDER;

            // Calculate the left and right motor duty cycles based on proportional control
            leftDuty_permil = PWM_AVERAGE - adjustment;   // Adjust left motor speed based on error
            rightDuty_permil = PWM_AVERAGE + adjustment;  //Adjust right motor speed based on error

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
            timer_10msGO++;
            break;

        case Stop:

            if (turn != Blocked){
                NextState = ForwardLeft;
            }
            leftDuty_permil = 0;
            rightDuty_permil = 0;


            if (timer_10msSTOP%50 == 0){
                if (color == RED){
                    color = BLUE;
                } else {
                    color = RED;
                }
            }

            LaunchPad_RGB(color);

            timer_10msSTOP++;

            break;

    }
    CurrentState = NextState;





    // ====================================================================
    // Do not modify anything below this line
    // ====================================================================

    // Update motor speed values based on calculated duty cycles if actuator control is enabled
    if (IsActuatorEnabled) {
        if (CurrentState != Stop){
            Motor_Forward(leftDuty_permil, rightDuty_permil); // Set motor speeds to maintain center position
        } else {
            Motor_Stop(leftDuty_permil, rightDuty_permil);
        }

        // If there is remaining space in the buffer, store control data for analysis
        if (BufferIndex < BUFFER_SIZE) {
            TimerGO[BufferIndex] = timer_10msGO;                 // Store current error value
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
void Level2(void){

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
    uint16_t const period_4us = 2500;//400//5000;       // Timer period to achieve 20ms (5000 x 4us)
    TimerA2_Init(&Controller, period_4us);  // Initialize TimerA2 for controller

    // Use TimerA1 to sample the IR sensors at 2000 Hz
    uint16_t const period_2us = 250;//250        // Timer period to achieve 0.5ms (250 x 2us)
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
    IsActuatorEnabled = true;        // Actuator disabled at startup
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
        //Clock_Delay1ms(300);    // Delay to stabilize

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
