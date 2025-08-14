; sqrt.asm
; Runs on any Cortex M
; Stan Baek
; 11 May 2023
;
;Simplified BSD License (FreeBSD License)
;Copyright (c) 2022, Stan Baek All rights reserved.
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

        .global  main

main:	.asmfunc
        MOV     R0, #63001      ; R0 = s = 630001 = 251^2

    ; Do NOT modify anything between double dashed lines.
	; ==================================================

        MOV     R1, #10         ; R1 = 10
        UDIV    R2, R0, R1      ; initial guess, R2 = s/10
        ADD     R2, #1          ; R2 = t = s/10 + 1
        MOV     R3, #16         ; R3 = i = 16
        MOV     R1, #2          ; R1 = 2

	; =================================================
	; Do NOT modify anything above this line

    ; Write this as part of Lab 2

loop   MUL R4, R2, R2                         ; R4 = t*t
       ADD R4, R4, R0                         ; R4 = t*t + s
       UDIV R4, R4, R2                         ; R4 = (t*t + s)/t
       UDIV R2, R4, R1                         ; R2 = t = ((t*t + s)/t)/2
       SUB R3, #1                 		; i--
       CMP R3, #0                 		; i != 0?
       BNE loop                 		; If so, go to loop

    ; end of your code

	; Do NOT modify anything below this line
	; ==================================================

stall   B       stall			; stay here forever.
		.endasmfunc

       .end
