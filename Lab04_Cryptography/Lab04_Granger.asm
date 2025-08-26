; Lab03_Granger.asm
; Runs on any Cortex M
; Documentation statement:
; Cryptography function to encrypt and decrypt a message using a 1 byte key.
; Dr. Stan Baek
; June 03 2021
;
;Simplified BSD License (FreeBSD License)
;Copyright (c) 2020, Steven Beyer, All rights reserved.
;
;Redistribution and use in source and binary forms, with or without modification,
;are permitted provided that the following conditions are met:
;
;1. Redistributions of source code must retain the above copyright notice,
;   this list of conditions and the following disclaimer.
;2. Redistributions in binary form must reproduce the above copyright notice,
;   this list of conditions and the following disclaimer in the documentation
;   and/or other materials provided with the distribution.
;
;THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
;AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
;IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
;ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
;LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
;DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
;LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
;AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
;OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
;USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
;
;The views and conclusions contained in the software and documentation are
;those of the authors and should not be interpreted as representing official
;policies, either expressed or implied, of the FreeBSD Project.

        .thumb

        .data
        .align 2
        ; variables in RAM
enc_msg .space 20   ; reserve 20 bytes of memory to store the encrypted message

; Do not modify the following two lines
extra1 	.word 0x92C7DDEB, 0xD7C4D3DA, 0xD092D392, 0xDB92D5C7, 0xDDCB92DC, 0xD192C0C7, 0x92D7D6DD, 0xCB92D4DB, 0xD192C7DD, 0xC092DCD3
extra2 	.word 0x92D6D3D7, 0xDB92D7DF, 0xDAC692DC, 0xD7D692D7, 0xC2CBC0D1, 0x92D6D7C6, 0xC1C1D7DF, 0x93D7D5D3, 0xB2F47291

dec_msg .space 256   ; reserve 256 bytes of memory to store the decrypted message

        .text
        .align 4
        ;variables in ROM
key     .byte   0xB2        ; key to encrypt/decrypt spell
spell   .string "Wingardium Leviosa!#"

        ; 32-bit pointers to each of the variables in memory
EMAddr  .word   enc_msg
DMAddr  .word   dec_msg
SPAddr  .word   spell

        .global Encrypt ; external function
        .global Decrypt ; external function
        .global main    ; main function

;main:   .asmfunc

        ; load registers for the arguments of the Encrypt function.
        LDR     R0, SPAddr
        LDRB    R1, key
        LDR     R2, EMAddr

        ; call the Encrypt function to encrypt the spell.
        BL      Encrypt

        ; load registers for the arguments of the Decrypt function.
        LDR     R0, EMAddr
        LDRB    R1, key
        LDR     R2, DMAddr

        ; call the Decrypt function.
        BL      Decrypt

Stall   B       Stall   ; Stall here and observe the registers and memory

        .endasmfunc

        .end
