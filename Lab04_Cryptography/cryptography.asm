; cryptography.asm
; Runs on any Cortex M
; Basic XOR encryption and decryption functions for cryptography.
; Capt Steven Beyer
; July 23 2020
; Updated by Stan Baek
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
       .text
       .align 2
       .global Encrypt
       .global Decrypt
       .global XOR_bytes

EOM    .byte  '#'


;------------ Encrypt ------------
; This subroutine takes the address of a plain text message and a key,
;   then encrypts the message by performing an XOR operation.
;   The encrypted result is stored at the specified memory location.
;   The encryption stops when EOM is encountered.
;
; Inputs:
;   R0 - Address of the plain text message
;   R1 - Key for XOR encryption
;   R2 - Address to store the encrypted message
;
; Return:
;   None
Encrypt:   .asmfunc
; ============ Add your code and comments below ===================

    ; Use EOM defined at line 44 for '#'
    PUSH {LR, R4-R8}        ; store registers in stack
    MOV R4, R0              ; copy address at R0 to R4
    MOV R5, R1              ; copy key at R1 to R5
    MOV R6, R2              ; copy address at R2 to R6
    LDRB R7, EOM            ; copy EOM character to R7

Loop
    LDRB R0, [R4], #1       ; load value of address at R4 to R0, then increment R4
    MOV R1, R5              ; copy key at R5 to R1
    MOV R8, R0              ; copy R0 to R8 to check later
    ; You must use the XOR_bytes function for exclusive or!
    BL XOR_bytes            ; XOR_Bytes
    STRB R0, [R6], #1       ; Store 8 bit value of R0 to address at R6 then increment
    CMP R8, R7              ; compare R0 to the null character (EOM)
    BEQ Done                ; if null character finish
    B Loop                  ; loop if you get this far


Done
    POP {LR, R4-R8}         ; restore saved registers from stack
    BX LR                   ; return to LR

; =============== End of your code ================================
    .endasmfunc


;------------ Decrypt ------------
; This subroutine takes the address of an encrypted message and a key,
;   then decrypts the message by performing an XOR operation.
;   The decrypted result is stored at the specified memory location.
;   The decryption stops when the EOM is encountered.
;
; Inputs:
;   R0 - Address of the encrypted message
;   R1 - Key for XOR decryption
;   R2 - Address to store the decrypted message
;
; Return:
;   None
Decrypt:    .asmfunc
; ============ Add your code and comments below ===================

    ; Use EOM defined at line 44 for '#'
    PUSH {LR, R4-R8}        ; store registers in stack
    MOV R4, R0              ; copy address at R0 to R4
    MOV R5, R1              ; copy key at R1 to R5
    MOV R6, R2              ; copy address at R2 to R6
    LDRB R7, EOM            ; copy EOM character to R7

Loop2
    LDRB R0, [R4], #1       ; load value of address at R4 to R0, then increment R4
    MOV R1, R5              ; copy key at R5 to R1
    ; You must use the XOR_bytes function for exclusive or!
    BL XOR_bytes            ; XOR_Bytes
    STRB R0, [R6], #1       ; Store 8 bit value of R0 to address at R6 then increment
    CMP R0, R7              ; compare R0 to the null character (EOM)
    BEQ Done2               ; if null character finish
    B Loop2                 ; loop if you get this far


Done2
    POP {LR, R4-R8}         ; restore saved registers from stack
    BX LR                   ; return to LR


; =============== End of your code ================================
    .endasmfunc
    .end
