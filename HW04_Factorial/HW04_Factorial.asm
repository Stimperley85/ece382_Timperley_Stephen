; HW04_Factorial.asm
; Runs on any Cortex M processor
; A simple project for memory access.
; Stan Baek
; May 12, 2021

;Simplified BSD License (FreeBSD License)
;Copyright (c) 2021, Stanley Baek, All rights reserved.
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

; CAREFULLY READ EVERY COMMENT TO ENSURE WHAT EACH LINE IS DOING!
; we align 32 bit variables to 32-bits
; we align op codes to 16 bits
        .thumb

        .data                   ; following lines are placed in RAM
        .align 2                ; forces machine code to be halfword-aligned
Res1    .space 20               ; reserves 20 bytes in RAM to store the results.
Res2    .space 20               ; reserves 20 bytes in RAM to store the results.

        .text                   ; following lines are placed in Flash ROM
        .align 4                ; forces machine code to be word-aligned

        ; variable
Nums    .byte 1, 2, 4, 7, 0
        ; Nums: Non-negative integers to find their factorials
        ; Look at the variable type of numbers in main.c.  It is uint8_t.

        ; pointers to variables
Res1Addr .word  Res1            ; pointer to Res
Res2Addr .word  Res2            ; pointer to Res
NumsAddr .word  Nums            ; pointer to Nums
Size     .word  5

        .global  main            ; main is a global function

main:   .asmfunc                ; begin assembly function

        LDR     R4, NumsAddr    ; R4 <= address of Nums, R4 is a pointer
        LDR     R5, Res1Addr    ; R5 <= address of Res1, R5 is a pointer
        LDR     R6, Size        ; R6 <= Size of arrays

; Use the iterative function for each number
Loop1   CMP     R6, #0          ; R6 == 0 ?
        BEQ     Next            ; if R6 == 0, done. Go to Next for recursive call
        LDRB    R0, [R4], #1    ; load R0 with the number R4 is pointing to, and increment R4 by 1
        BL      fact_iter       ; call fact_iter
        STR     R0, [R5], #4    ; store the output into RAM where R5 is pointing, and increment R5 by 4
        SUB     R6, #1          ; R6--
        B       Loop1           ; go to Loop1 for the next number B

Next    LDR     R4, NumsAddr    ; R4 <= address of Nums, R4 is a pointer
        LDR     R5, Res2Addr    ; R5 <= address of Res2, R5 is a pointer
        LDR     R6, Size        ; R6 <= Size of arrays

; Use the recursive function for each number
Loop2   CMP     R6, #0          ; R6 == 0 ?
        BEQ     Stall           ; if R6 == 0, done. Go to Stall and stay there forever.
        LDRB    R0, [R4], #1    ; load R0 with the number R4 is pointing to, and increment R4 by 1
        BL      fact_rec        ; call fact_rec
        STR     R0, [R5], #4    ; store the output into RAM where R5 is pointing, and increment R5 by 4
        SUB     R6, #1          ; R6--
        BL       Loop2           ; go to Loop2 for the next number B

Stall   B        Stall          ; Stay here forever and observe the results in Memory Browser

       .endasmfunc

; Interative function to computer n!
fact_iter:  .asmfunc            ; Begin assembly function

; ============ Add your code and comments below ===================
        MOV     R1, R0                ; Move R0 to another register so that we can use R0 for return value.
        MOV     R0, #1                ; Initalize return value; ret = 1
Loop3   CMP     R1, #0                ; n==0?
        BEQ     Exit1                ; If equal, done; else, execute the following lines.
        MUL     R0, R0, R1                ; ret = ret * n
        SUB     R1, #1                ; n--
        B       Loop3           ; while(n != 0) BL
Exit1   BX LR                ; Return
; ============= End of your code ================================

        .endasmfunc


; Recursive function to computer n!
fact_rec:   .asmfunc            ; Begin assembly function

        CMP     R0, #0          ; Compare R0 with 0
        BGT     Recur           ; If R0 > 0, Go to Recur and call this function recursively.
        MOV     R0, #1          ; If R0 == 0, R0 <= 1 (return 1)
        BX      LR              ; Return

; ============ Add your code and comments below ===================
Recur   PUSH    {LR, R1}                ; Preserve registers
        MOV     R1, R0                ; Save n for later use
        SUB     R0, #1                ; n-1
        BL       fact_rec                ; fact_rec(n-1) for (n-1)!
        MUL     R0, R1, R0                ; n * fact_rec(n-1)
        POP     {LR, R1}                ; Restore registers and return
        BX      LR
; ============= End of your code ================================

        .endasmfunc

       .end
