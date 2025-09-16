// SPIA3.c
// Runs on MSP432
// Use eUSCI_A3 to send data via SPI A3
// Stan Baek
// June 08 2022


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

#include <stdio.h>
#include <stdint.h>
#include "msp.h"
//#include "SPIA3.h"

//#define P9DIR                   (*((volatile uint8_t *)0x40004C84))   /* Port 9 Direction */
//#define P9SEL0                  (*((volatile uint8_t *)0x40004C8A))   /* Port 9 Select 0 */
//#define P9SEL1                  (*((volatile uint8_t *)0x40004C8C))   /* Port 9 Select 1 */
//#define UCA3CTLW0               (*((volatile uint16_t *)0x40001C00))  /* eUSCI_Ax Control Word Register 0 */
//#define UCA3BRW                 (*((volatile uint16_t *)0x40001C06))  /* eUSCI_Ax Baud Rate Control Word Register */
//#define UCA3MCTLW               (*((volatile uint16_t *)0x40001C08))  /* eUSCI_Ax Modulation Control Word Register */
//#define UCA3STATW               (*((volatile uint16_t *)0x40001C0A))  /* eUSCI_Ax Status Register */
//#define UCA3RXBUF               (*((volatile uint16_t *)0x40001C0C))  /* eUSCI_Ax Receive Buffer Register */
//#define UCA3TXBUF               (*((volatile uint16_t *)0x40001C0E))  /* eUSCI_Ax Transmit Buffer Register */
//#define UCA3IE                  (*((volatile uint16_t *)0x40001C1A))  /* eUSCI_Ax Interrupt Enable Register */
//#define UCA3IFG                 (*((volatile uint16_t *)0x40001C1C))  /* eUSCI_Ax Interrupt Flag Register */


//********SPIA3_Init*****************
// Initialize SPI A3.
// One feature of the MSP432 is that its SSIs can get their baud clock from
// either the auxiliary clock (ACLK = REFOCLK/1 = 32,768 Hz
// see ClockSystem.c) or from the low-speed subsystem
// master clock (SMCLK <= 12 MHz see ClockSystem.c).  The
// SSI can further divide this clock signal by using the
// 16-bit Bit Rate Control prescaler Register, UCAxBRW.
// Inputs: none
// Outputs: none
// Assumes: low-speed subsystem master clock 12 MHz
void SPIA3_Init(void) {

    // write this as part of Lab 11


    // hold the eUSCI module in reset mode
    EUSCI_A3->CTLW0 |= 0x01;

    // configure UCA3CTLW0 for:
    // bit15      UCCKPH = 1;   data shifts in on first edge, out on following edge
    // bit14      UCCKPL = 0;   clock is low when inactive
    // bit13      UCMSB = 1;    MSB first
    // bit12      UC7BIT = 0;   8-bit data
    // bit11      UCMST = 1;    master mode
    // bits10-9   UCMODEx = 2;  UCSTE active low
    // bit8       UCSYNC = 1;   synchronous mode
    // bits7-6    UCSSELx = 2;  eUSCI clock SMCLK
    // bits5-2                  reserved
    // bit1       UCSTEM = 1;   UCSTE pin enables slave
    // bit0       UCSWRST = 1;  reset enabled
    EUSCI_A3->CTLW0 &= 0x0000;
    EUSCI_A3->CTLW0 |= 0b101011011111;

    // set the baud rate for the eUSCI which gets its clock from SMCLK
    // Clock_Init48MHz() from ClockSystem.c sets SMCLK = HFXTCLK/4 = 12 MHz
    // if the SMCLK is set to 12 MHz, divide by 3 for 4 MHz baud clock
    EUSCI_A3->BRW = 3;

    // modulation is not used in SPI mode, so clear UCA3MCTLW
    EUSCI_A3->MCTLW &= 0;
    
    // configure P9.7, P9.5, and P9.4 as primary module function
    P9->SEL0 |= 0xB0;
    P9->SEL1 &= ~0xB0;

    // enable eUSCI module
    EUSCI_A3->CTLW0 &= 0;

    // disable interrupts
    EUSCI_A3->IE &= 0b00;

}

//********SPIA3_Wait4Tx*****************
// Wait for transmitter to be empty (let previous frame finish)
// Inputs: None
// Outputs: None
void SPIA3_Wait4Tx(void) {

    // Wait for transmitter to be empty (UCTXIFG)

}

//********SPIA3_Wait4TxRxReady*****************
// Wait until SPI not busy
// Inputs: None
// Outputs: None
void SPIA3_Wait4TxRxReady(void) {

    // Wait until STATW.UCBUSY (eUSCI_A busy) is 0.
    // This bit indicates if a transmit or receive operation is in progress.
    // 0b = eUSCI_A inactive
    // 1b = eUSCI_A transmitting or receiving
    while((EUSCI_A3->STATW & 0x0001));
}

//********SPIA3_WriteTxBuffer****************
// Write data to TXBUF
// Inputs: data  8-bit data to transmit
// Outputs: none
// Assumes: UCA3 and Port 9 have already been initialized and enabled
// The eUSCI module has no hardware FIFOs.
void SPIA3_WriteTxBuffer(char data) {

    // Send data using TXBUF

}


//********SPIA3_OutChar*****************
// Transmit a character via the SPI channel.
// Inputs: data  8-bit data to transmit
// Outputs: none
// Assumes: UCA3 and Port 9 have already been initialized and enabled
// The eUSCI module has no hardware FIFOs.
void SPIA3_OutChar(char data) {
    // 1) Wait for transmitter to be empty (let previous frame finish)
    // 2) Write data to TXBUF, starts SPI

}


//********SPIA3_OutString*****************
// Output String (NULL termination)
// Input: pointer to a NULL-terminated string to be transferred
// Output: none
void SPIA3_OutString(const char* ptr){

    // you write this as part of Lab 11

}
