; LEDs.asm
; Runs on MSP432
; Capt Steven Beyer
; September 9, 2019


; Code to activate LED on P6.7. This code accompanies the Lab05_LED_Switchesmain.c

        .thumb

        .text
; Addresses for Port 1 registers
; Find the address values on Valvano, page 155.
; The dots between addresses are for easy reading.
; Do not add dots in your code.
        .align 4
P1SEL0  .word  0x40004C0A
P1SEL1  .word  0x40004C0C
P1DIR   .word  0x40004C04
P1OUT   .word  0x40004C02
P1REN   .word  0x40004C06

; Use this delay for Homework 8.
; You need to adjust this value in Lab8.
DELAY   .word 1200000

        ; global functions external function can access these functions.
        .global LED_Init
        .global LED_Off
        .global LED_On
        .global LED_Toggle
        .global LED_Oscillate

; function to initialize P1.0 (LED)
LED_Init:       .asmfunc
        LDR     R1, P1SEL0		   ; Load the address of P1SEL0
        LDRB    R0, [R1]
        BIC     R0, R0, #0x01      ; Clear P1SEL0.0
        STRB    R0, [R1]
        LDR     R1, P1SEL1         ; Load the address of P1SEL1
        LDRB    R0, [R1]
        BIC     R0, R0, #0x01      ; Clear P1SEL1.0
        STRB    R0, [R1]           ; store byte
        LDR     R1, P1DIR
        LDRB    R0, [R1]           ; load byte
        ORR     R0, R0, #0x01      ; bit set
        STRB    R0, [R1]
        BX      LR
        .endasmfunc


; function to turn off LED (P1.0)
LED_Off:        .asmfunc
        LDR     R1, P1OUT
        LDRB    R0, [R1]        ; 8-bit read
        BIC     R0, R0, #0x01   ; turn off
        STRB    R0, [R1]
        BX      LR
        .endasmfunc


; function to turn on P1.0
LED_On:         .asmfunc
        LDR     R1, P1OUT
        LDRB    R0, [R1]       ; 8-bit read
        ORR     R0, R0, #0x01   ; turn on
        STRB    R0, [R1]
        BX      LR
        .endasmfunc


; function to toggle P1.0
; Turn off LED if it is on.
; Turn on LED if it is off.
; To toggle a bit, you don't need to know whether it is currently on or off.
; Do not use LED_Off or LED_On
LED_Toggle: 	.asmfunc
; add your code here
        LDR     R1, P1OUT
        LDRB    R0, [R1]
        EOR     R0, R0, #0x01
        STRB    R0, [R1]
        BX      LR

        .endasmfunc


; function to continuously toggle P1.0 every half second
; use a loop as a timer
LED_Oscillate:  .asmfunc
; add your code here

Begin   BL      LED_Toggle  ; togles
        LDR     R0, DELAY   ; loads the delay constant in R0
Loop1   CMP     R0, #0      ; compare if equals 0
        BEQ     Begin       ; branch to begin, exit loop
        SUB     R0, R0, #1  ; subtract 1 from delay constant in R0
        B       Loop1       ; stay in loop

        .endasmfunc

        .end
