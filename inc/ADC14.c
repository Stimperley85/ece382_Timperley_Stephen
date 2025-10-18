// ADC14.c
// Runs on MSP432, TI-RSLK MAX 1.1
// ADC input, software trigger, 14-bit conversion,
// student version
// Jonathan Valvano
// July 11, 2019

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
#include "../inc/ADC14.h"

// P9.0 = A17
// P6.1 = A14
// P9.1 = A16
// Lab 15 assignment RSLK 1.1, use software trigger, 3.3V reference, RSLK1.1
void ADC0_InitSWTriggerCh17_14_16(void){
    // you can use any of the MEM[n], MCTL[n] except n=6 (6 is used by TExaS)

    // write this for Lab 15


    // 1. ADC14ENC = 0 to allow programming
    ADC14->CTL0 &= ~0x00000002;

    // 2. wait for BUSY to be zero
    while(ADC14->CTL0 & 0x00010000){};

    // 3. ADC14CTL0: single, SMCLK, on, disabled, /1, 32 SHT
    // 31-30 ADC14PDIV  predivider,             Predivide by 1
    // 29-27 ADC14SHSx  SHM source              ADC14SC bit
    // 26    ADC14SHP   SHM pulse-mode          SAMPCON the sampling timer
    // 25    ADC14ISSH  invert sample-and-hold  not inverted
    // 24-22 ADC14DIVx  clock divider           divide by 1
    // 21-19 ADC14SSELx clock source select     SMCLK
    // 18-17 ADC14CONSEQx mode select           Sequence-of-channels
    // 16    ADC14BUSY  ADC14 busy              (read only)
    // 15-12 ADC14SHT1x sample-and-hold time    32 clocks
    // 11-8  ADC14SHT0x sample-and-hold time    32 clocks
    // 7     ADC14MSC   multiple sample         continue conversions automatically after first SHI signal trigger
    // 6-5   reserved                           (reserved)
    // 4     ADC14ON    ADC14 on                powered up
    // 3-2   reserved                           (reserved)
    // 1     ADC14ENC   enable conversion       ADC14 disabled
    // 0     ADC14SC    ADC14 start             No start (yet)
    ADC14->CTL0 = 0x04220390;

    // 4. ADC14CTL1: 14-bit, ref on, regular power, start with MEM2
    // 20-16 STARTADDx  start addr              ADC14MEM2
    // 15-6  reserved                           (reserved)
    // 5-4   ADC14RES   ADC14 resolution        14 bit, 16 clocks
    // 3     ADC14DF    data read-back format   Binary unsigned
    // 2     REFBURST   reference buffer burst  reference on continuously
    // 1-0   ADC14PWRMD ADC power modes         Regular power mode
    ADC14->CTL1 = 0x00020030;

    // 5.a channel 17, 0 to 3.3V, not end of sequence
    // 15   ADC14WINCTH Window comp threshold   not used
    // 14   ADC14WINC   Comparator enable       Comparator disabled
    // 13   ADC14DIF    Differential mode       Single-ended mode enabled
    // 12   reserved                            (reserved)
    // 11-8 ADC14VRSEL  V(R+) and V(R-)         V(R+) = AVCC, V(R-) = AVSS
    // 7    ADC14EOS    End of sequence         Not end of sequence
    // 6-5  reserved                            (reserved)
    // 4-0  ADC14INCHx  Input channel           A17
    ADC14->MCTL[2] = 0x00000011;

    // 5.b channel 14, 0 to 3.3V, not end of sequence
    // 15   ADC14WINCTH Window comp threshold   not used
    // 14   ADC14WINC   Comparator enable       Comparator disabled
    // 13   ADC14DIF    Differential mode       Single-ended mode enabled
    // 12   reserved                            (reserved)
    // 11-8 ADC14VRSEL  V(R+) and V(R-)         V(R+) = VCC, V(R-) = AVSS
    // 7    ADC14EOS    End of sequence         Not end of sequence
    // 6-5  reserved                            (reserved)
    // 4-0  ADC14INCHx  Input channel           A14
    ADC14->MCTL[3] = 0x0000000E;

    // 5.c channel 16, 0 to 3.3V, end of sequence
    // 15   ADC14WINCTH Window comp threshold   not used
    // 14   ADC14WINC   Comparator enable       Comparator disabled
    // 13   ADC14DIF    Differential mode       Single-ended mode enabled
    // 12   reserved                            (reserved)
    // 11-8 ADC14VRSEL  V(R+) and V(R-)         V(R+) = VCC, V(R-) = AVSS
    // 7    ADC14EOS    End of sequence         End of sequence
    // 6-5  reserved                            (reserved)
    // 4-0  ADC14INCHx  Input channel           A16
    ADC14->MCTL[4] = 0x00000090;

    // 6. no interrupts
    ADC14->IER0 = 0;
    ADC14->IER1 = 0;

    // 7.a analog mode on P6.1/A14
    P6->SEL1 |= 0x02;
    P6->SEL0 |= 0x02;

    // 7.b analog mode on P9.0/A17 and P9.1/A16
    P9->SEL1 |= 0x03;
    P9->SEL0 |= 0x03;

    // 8. enable ADC14
    ADC14->CTL0 |= 0x00000002;


}

// ADC14IFGR0 bit 4 is set when conversion done
//                  cleared on read ADC14MEM4
// ADC14MEM2 14-bit conversion in bits 13-0 (31-16 undefined, 15-14 zero)
// ADC14MEM3 14-bit conversion in bits 13-0 (31-16 undefined, 15-14 zero)
// ADC14MEM4 14-bit conversion in bits 13-0 (31-16 undefined, 15-14 zero)
// Lab 15 assignment RSLK 1.1, use software trigger, 3.3V reference
void ADC_In17_14_16(uint16_t *ch17, uint16_t *ch14, uint16_t *ch16){

    // you write this as part of Lab 15
	
    // 1. wait for BUSY to be zero
    while(ADC14->CTL0 & 0x00010000){};

    // 2. start single conversion
	ADC14->CTL0 |= 0x00000001;

	// 3. wait for ADC14IFG4
	while((ADC14->IFGR0 & 0x00000010) != 0x00000010){};
    // 4.a P9.0/A17 result
     *ch17 = ADC14->MEM[2];
    // 4b) P6.1/A14 result
     *ch14 = ADC14->MEM[3];
    // 4c) P9.1/A16 result
     *ch16 = ADC14->MEM[4];

}
