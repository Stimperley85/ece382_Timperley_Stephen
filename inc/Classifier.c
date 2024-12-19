/*
 * Classifier.c
 * Runs on MSP432
 *
 *
 * Conversion function for a GP2Y0A21YK0F IR sensor and classification function
 *  to take 3 distance readings and determine what the current state of the robot is.
 *
 *  Created on: Jul 24, 2020
 *  Author: Captain Steven Beyer
 *
 * This example accompanies the book
 * "Embedded Systems: Introduction to Robotics,
 * Jonathan W. Valvano, ISBN: 9781074544300, copyright (c) 2019
 * For more information about my classes, my research, and my books, see
 * http://users.ece.utexas.edu/~valvano/
 *
 * Simplified BSD License (FreeBSD License
 *
 * Copyright (c) 2019, Jonathan Valvano, All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 * USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are
 * those of the authors and should not be interpreted as representing official
 * policies, either expressed or implied, of the FreeBSD Project.
 */

#include <stdint.h>
#include "../inc/Classifier.h"


// Complete the following lines
//#define SIDEMAX     // largest side distance to wall in mm
//#define SIDEMIN     // smallest side distance to wall in mm
//#define CENTEROPEN  // distance to wall between open/blocked
//#define CENTERMIN   // min distance to wall in the front
//#define IRMIN       // min possible reading of IR sensor
//#define IRMAX       // max possible reading of IR sensor


/* Classify
* Utilize three distance values from the left, center, and right
* distance sensors to determine and classify the situation into one
* of many scenarios (enumerated by scenario)
*
* Input
*   int32_t Left: distance measured by left sensor
*   int32_t Center: distance measured by center sensor
*   int32_t Right: distance measured by right sensor
*
* Output
*   scenario_t: value representing the scenario the robot
*       currently detects (e.g. RightTurn, LeftTurn, etc.)
*/
scenario_t Classify(int32_t left_mm, int32_t center_mm, int32_t right_mm) {

    scenario_t result = ClassificationError;

    // Add your your code here


    return result;
}

