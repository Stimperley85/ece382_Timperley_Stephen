// UART0.c
// Runs on MSP432
// Busy-wait device driver for the UART UCA0.
// Daniel Valvano
// July 11, 2019
// Modified by EE345L students Charlie Gough && Matt Hawk
// Modified by EE345M students Agustinus Darmawan && Mingjie Qiu

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

// UCA0RXD (VCP receive) is connected to P1.2
// UCA0TXD (VCP transmit) is connected to P1.3

#include <stdint.h>
#include <stdio.h>
#include "UART0.h"
#include "msp.h"

// Standard ASCII characters
// Carriage return (CR)
#define CR   0x0D
// Line feed (LF)
#define LF   0x0A
// Backspace (BS)
#define BS   0x08
// Escape (ESC)
#define ESC  0x1B
// Space (SP)
#define SP   0x20
// Delete (DEL)
#define DEL  0x7F

// NOTE: UART0 is connected to the USB port

//------------UART0_Init------------
// Initializes UART0 (with a 12 MHz SMCLK clock),
// using an 8-bit data length, no parity, and one stop bit.
// Input: baudrate in baud per second
// Output: none
void UART0_Init(uint32_t baudrate){

    // you write this as part of Lab 11

    // hold the USCI module in reset mode
    EUSCI_A0->CTLW0 |= 0x01;

    // Configure UART settings:

    // bit15=0,      no parity bits
    // bit14=x,      not used (parity disabled)
    // bit13=0,      LSB first
    // bit12=0,      8-bit data length
    // bit11=0,      1 stop bit
    // bits10-8=000, asynchronous UART mode
    // bits7-6=11,   clock source set to SMCLK
    // bit5=0,       ignore erroneous characters (flag not set)
    // bit4=0,       ignore break characters (flag not set)
    // bit3=0,       not in dormant mode
    // bit2=0,       data mode (not address mode)
    // bit1=0,       do not transmit break
    // bit0=1,       hold logic in reset state for configuration
    EUSCI_A0->CTLW0 &= 0x0000; //0b0000000000000000;
    EUSCI_A0->CTLW0 |= 0b0000000011000001;

    // Set the baud rate
    // N = clock/baud rate, for 115,200 baud: N = 12,000,000 / 115,200 = 104.1667
    // UCBR = int(N) = 104
    // Note: 'baudrate' is a function argument passed into this initialization function. 
    EUSCI_A0->BRW = 104;

    // Clear modulation control
    EUSCI_A0->MCTLW = 0;
    
    // Configure P1.3 and P1.2 as primary UART function pins
    P1->SEL0 |= 0x0C;
    P1->SEL1 &= ~0x0C;

    // enable the USCI module
    EUSCI_A0->CTLW0 &= ~0x01;

    // Disable UART interrupts (transmit ready, receive full, etc.)
    EUSCI_A0->IE = 0x0;

}

//------------UART0_InChar------------
// Waits for a new character to be received on UART0
// Input: none
// Output: ASCII code of the received character
char UART0_InChar(void) {

    while((EUSCI_A0->IFG & 0x01) == 0);     // Wait for the receive flag to be set
    return((char)(EUSCI_A0->RXBUF));        // Return received character

}

//------------UART0_OutChar------------
// Sends an 8-bit character via UART0
// Input: 8-bit ASCII character to be transmitted
// Output: none
void UART0_OutChar(char data){
    // you write this as part of Lab 11
    while((EUSCI_A0->IFG & 0x02) == 0);
    EUSCI_A0->TXBUF = data;
}


//------------UART0_OutString------------
// Transmits a null-terminated string via UART0
// Input: pointer to the null-terminated string
// Output: none
void UART0_OutString(const char* ptr){
    // you write this as part of Lab 11
    // You must use UART0_OutChar
    while (*ptr != 0x00){
        UART0_OutChar(*ptr);
        ptr++;
    }
}

//===============================================================


//------------UART0_InUDec------------
// InUDec accepts ASCII input in unsigned decimal format
//     and converts to a 32-bit unsigned number
//     valid range is 0 to 4294967295 (2^32-1)
// Input: none
// Output: 32-bit unsigned number
// If you enter a number above 4294967295, it will return an incorrect value
// Backspace will remove last digit typed
uint32_t UART0_InUDec(void) {
    uint32_t number=0, length=0;
    char character = UART0_InChar();
    while(character != CR) { // accepts until <enter> is typed
    // The next line checks that the input is a digit, 0-9.
    // If the character is not 0-9, it is ignored and not echoed
        if((character >= '0') && (character <= '9')) {
            number = 10*number+(character-'0');   // this line overflows if above 4294967295
            length++;
            UART0_OutChar(character);
        }
        // If the input is a backspace, then the return number is
        // changed and a backspace is output to the screen
        else if((character==BS) && length){
            number /= 10;
            length--;
            UART0_OutChar(character);
        }
        character = UART0_InChar();
    }

    return number;
}

//-----------------------UART0_OutUDec-----------------------
// Output a 32-bit number in unsigned decimal format
// Input: 32-bit number to be transferred
// Output: none
// Variable format 1-10 digits with no space before or after
void UART0_OutUDec(uint32_t n){

    if(n >= 10){
        UART0_OutUDec(n/10);
        n = n%10;
    }

    UART0_OutChar(n+'0'); /* n is between 0 and 9 */
}

//-----------------------UART0_OutSDec-----------------------
// Output a 32-bit number in signed decimal format
// Input: 32-bit number to be transferred
// Output: none
// Variable format 1-10 digits with no space before or after
void UART0_OutSDec(int32_t n){

    if(n < 0){
      UART0_OutChar('-');
      UART0_OutUDec(-n);
    }else{
      UART0_OutUDec(n);
    }
}


uint32_t Messageindexb;
char Messageb[8];

void static fillmessageb(uint32_t n){
// This function uses recursion to convert decimal number
//   of unspecified length as an ASCII string
  if(n >= 10){
    fillmessageb(n/10);
    n = n%10;
  }
  Messageb[Messageindexb] = (n+'0'); /* n is between 0 and 9 */
  if(Messageindexb<7)Messageindexb++;
}


void static fillmessage4b(uint32_t n){
  if(n>=1000){  // 1000 to 9999
    Messageindexb = 0;
  } else if(n>=100){  // 100 to 999
    Messageb[0] = ' ';
    Messageindexb = 1;
  }else if(n>=10){ //
    Messageb[0] = ' '; /* n is between 10 and 99 */
    Messageb[1] = ' ';
    Messageindexb = 2;
  }else{
    Messageb[0] = ' '; /* n is between 0 and 9 */
    Messageb[1] = ' ';
    Messageb[2] = ' ';
    Messageindexb = 3;
  }
  fillmessageb(n);
}


void static fillmessage5b(uint32_t n){
    if(n > 99999) n = 99999;
    if(n >= 10000) {  // 10000 to 99999
        Messageindexb = 0;
    } else if(n >= 1000) {  // 1000 to 9999
        Messageb[0] = ' ';
        Messageindexb = 1;
    } else if(n >= 100) {  // 100 to 999
        Messageb[0] = ' ';
        Messageb[1] = ' ';
        Messageindexb = 2;
    } else if(n >= 10) { //
        Messageb[0] = ' '; /* n is between 10 and 99 */
        Messageb[1] = ' ';
        Messageb[2] = ' ';
        Messageindexb = 3;
    } else {
        Messageb[0] = ' '; /* n is between 0 and 9 */
        Messageb[1] = ' ';
        Messageb[2] = ' ';
        Messageb[3] = ' ';
        Messageindexb = 4;
    }

    fillmessageb(n);
}


//-----------------------UART0_OutUDec4-----------------------
// Output a 32-bit number in unsigned decimal format
// Input: 32-bit number to be transferred
// Output: none
// Fixed format 4 digits with no space before or after
void UART0_OutUDec4(uint32_t n){
    if(n>9999) {
        UART0_OutString("****");
    } else {
        Messageindexb = 0;
        fillmessage4b(n);
        Messageb[Messageindexb] = 0; // terminate
        UART0_OutString(Messageb);
    }
}


//-----------------------UART0_OutUDec5-----------------------
// Output a 32-bit number in unsigned decimal format
// Input: 32-bit number to be transferred
// Output: none
// Fixed format 5 digits with no space before or after
void UART0_OutUDec5(uint32_t n){
  if(n>99999){
    UART0_OutString("*****");
  }else{
    Messageindexb = 0;
    fillmessage5b(n);
    Messageb[Messageindexb] = 0; // terminate
    UART0_OutString(Messageb);
  }
}


//-----------------------UART0_OutUFix1-----------------------
// Output a 32-bit number in unsigned decimal format
// Input: 32-bit number to be transferred
// Output: none
// fixed format <digit>.<digit> with no space before or after
void UART0_OutUFix1(uint32_t n){
    UART0_OutUDec(n/10);
    UART0_OutChar('.');
    UART0_OutUDec(n%10);
}


//-----------------------UART0_OutUFix2-----------------------
// Output a 32-bit number in unsigned decimal format
// Input: 32-bit number to be transferred
// Output: none
// fixed format <digit>.<digit><digit> with no space before or after
void UART0_OutUFix2(uint32_t n){
    UART0_OutUDec(n/100);
    UART0_OutChar('.');
    n = n%100;
    UART0_OutUDec(n/10);
    UART0_OutUDec(n%10);
}


//---------------------UART0_InUHex----------------------------------------
// Accepts ASCII input in unsigned hexadecimal (base 16) format
// Input: none
// Output: 32-bit unsigned number
// No '$' or '0x' need be entered, just the 1 to 8 hex digits
// It will convert lower case a-f to upper-case A-F
//     and converts to a 16 bit unsigned number
//     value range is 0 to FFFFFFFF
// If you enter a number above FFFFFFFF, it will return an incorrect value
// Backspace will remove last digit typed
uint32_t UART0_InUHex(void){

    uint32_t number=0, digit, length=0;
    char character = UART0_InChar();

    while(character != CR) {
        digit = 0x10; // assume bad
        if((character>='0') && (character<='9')){
            digit = character-'0';
        }
        else if((character>='A') && (character<='F')){
            digit = (character-'A')+0xA;
        }
        else if((character>='a') && (character<='f')){
            digit = (character-'a')+0xA;
        }

        // If the character is not 0-9 or A-F, it is ignored and not echoed
        if(digit <= 0xF){
            number = number*0x10+digit;
            length++;
            UART0_OutChar(character);
        }
        // Backspace outputted and return value changed if a backspace is inputted
        else if((character==BS) && length){
            number /= 0x10;
            length--;
            UART0_OutChar(character);
        }

        character = UART0_InChar();
    }

    return number;
}


//--------------------------UART0_OutUHex----------------------------
// Output a 32-bit number in unsigned hexadecimal format
// Input: 32-bit number to be transferred
// Output: none
// Variable format 1 to 8 digits with no space before or after
void UART0_OutUHex(uint32_t number){
// This function uses recursion to convert the number of
//   unspecified length as an ASCII string
  if(number >= 0x10){
    UART0_OutUHex(number/0x10);
    UART0_OutUHex(number%0x10);
  }
  else{
    if(number < 0xA){
      UART0_OutChar(number+'0');
     }
    else{
      UART0_OutChar((number-0x0A)+'A');
    }
  }
}


//--------------------------UART0_OutUHex2----------------------------
// Output a 32-bit number in unsigned hexadecimal format
// Input: 32-bit number to be transferred
// Output: none
// Fixed format 2 digits with no space before or after
void outnibble(uint32_t n){
    if(n < 0xA){
   UART0_OutChar(n+'0');
  }
  else{
    UART0_OutChar((n-0x0A)+'A');
  }
}
void UART0_OutUHex2(uint32_t number){
  outnibble(number/0x10); // ms digit
  outnibble(number%0x10); // ls digit
}


//------------UART0_InString------------
// Accepts ASCII characters from the serial port
//    and adds them to a string until <enter> is typed
//    or until max length of the string is reached.
// It echoes each character as it is input.
// If a backspace is input, the string is modified
//    and the backspace is echoed
// terminates the string with a null character
// uses busy-waiting synchronization on RDRF
// Input: pointer to empty buffer, size of buffer
// Output: Null terminated string
// -- Modified by Agustinus Darmawan + Mingjie Qiu --
void UART0_InString(char *bufPt, uint16_t max) {
int length=0;
char character;
  character = UART0_InChar();
  while(character != CR){
    if(character == BS){
      if(length){
        bufPt--;
        length--;
        UART0_OutChar(BS);
      }
    }
    else if(length < max){
      *bufPt = character;
      bufPt++;
      length++;
      UART0_OutChar(character);
    }
    character = UART0_InChar();
  }
  *bufPt = 0;
}


// Get input from UART, echo
int fgetc (FILE *f){
  char ch = UART0_InChar();  // receive from keyboard
  UART0_OutChar(ch);            // echo
  return ch;
}


// Function called when file error occurs.
int ferror(FILE *f){
  /* Your implementation of ferror */
  return EOF;
}


// Abstraction of general output device
// Volume 2 section 3.4.5

//-----CCS implementation
#include "file.h"
int uart_open(const char *path, unsigned flags, int llv_fd){

    const uint32_t baudrate = 115200;
    UART0_Init(baudrate);  // Initialize UART0
    return 0;
}
int uart_close( int dev_fd){
    return 0;
}
int uart_read(int dev_fd, char *buf, unsigned count){
	
	char ch;
    ch = UART0_InChar();    // receive from keyboard
    ch = *buf;         // return by reference
    UART0_OutChar(ch);  // echo
    return 1;
}
int uart_write(int dev_fd, const char *buf, unsigned count){ 

	unsigned int num=count;
    while(num){
        if(*buf == 10){
           UART0_OutChar(13);
        }
        UART0_OutChar(*buf);
        buf++;
        num--;
    }
    return count;
}


off_t uart_lseek(int dev_fd, off_t ioffset, int origin){
    return 0;
}

int uart_unlink(const char * path){
    return 0;
}


int uart_rename(const char *old_name, const char *new_name){
    return 0;
}


//------------UART0_Initprintf------------
// Initialize the UART for 115,200 baud rate (assuming 48 MHz bus clock),
// 8 bit word length, no parity bits, one stop bit
// Input: none
// Output: none
void UART0_Initprintf(void){

    int ret_val; FILE *fptr;

    const uint32_t baudrate = 115200;
    UART0_Init(baudrate);  // Initialize UART0

    ret_val = add_device("uart", _SSA, uart_open, uart_close, uart_read, uart_write, uart_lseek, uart_unlink, uart_rename);
    if(ret_val) return; // error
    fptr = fopen("uart","w");
    if(fptr == 0) return; // error
    freopen("uart:", "w", stdout); // redirect stdout to uart
    setvbuf(stdout, NULL, _IONBF, 0); // turn off buffering for stdout
}
/*
// Keil uVision Code
// Print a character to UART.
//------------UART0_Initprintf------------
// Initialize the UART0_Initprintf for 115,200 baud rate (assuming 3 MHz bus clock),
// 8 bit word length, no parity bits, one stop bit
// Input: none
// Output: none
void UART0_Initprintf(void){
  UART0_Init();
}
int fputc(int ch, FILE *f){
  if((ch == 10) || (ch == 13) || (ch == 27)){
    UART0_OutChar(13);
    UART0_OutChar(10);
    return 1;
  }
  UART0_OutChar(ch);
  return 1;
}
*/
